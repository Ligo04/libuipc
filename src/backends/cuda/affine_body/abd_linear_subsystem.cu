/*
 * NVIDIA contributions: blockwise ABD Hessian emission and fused
 * preconditioner integration.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <affine_body/abd_linear_subsystem.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>
#include <utils/matrix_assembler.h>
#include <utils/matrix_unpacker.h>
#include <uipc/builtin/attribute_name.h>
#include <affine_body/inter_affine_body_constitution_manager.h>
#include <affine_body/abd_linear_subsystem_reporter.h>
#include <affine_body/affine_body_kinetic.h>
#include <affine_body/affine_body_constitution.h>
#include <utils/report_extent_check.h>
#include <cstdint>
#include <limits>

namespace uipc::backend::cuda
{
UIPC_GENERIC void zero_out_lower(Matrix12x12& H)
{
    // clear lower triangle (3x3 block based)
    for(IndexT jj = 0; jj < 4; ++jj)
    {
        for(IndexT ii = jj + 1; ii < 4; ++ii)
        {
            H.block<3, 3>(ii * 3, jj * 3).setZero();
        }
    }
}

namespace
{
    using namespace cuda_tool;  // for eigen:: / view aliases used inside kernel bodies

    template <typename T>
    void loose_resize(cuda_tool::DeviceBuffer<T>& buffer, SizeT size, Float reserve_ratio)
    {
        if(buffer.capacity() < size)
            buffer.reserve_discard(static_cast<SizeT>(size * reserve_ratio));
        buffer.resize_discard(size);
    }

    inline UIPC_GENERIC std::uint64_t pack_body_pair(IndexT left, IndexT right, int col_bits)
    {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(left)) << col_bits)
               | static_cast<std::uint32_t>(right);
    }

    inline UIPC_GENERIC IndexT body_pair_left(std::uint64_t key, int col_bits)
    {
        return static_cast<IndexT>(key >> col_bits);
    }

    inline UIPC_GENERIC IndexT body_pair_right(std::uint64_t key, int col_bits)
    {
        const auto mask = (std::uint64_t{1} << col_bits) - std::uint64_t{1};
        return static_cast<IndexT>(key & mask);
    }

    __global__ void abd_dytopo_build_body_pair_keys_kernel(
        cuda_tool::CTripletMatrixView<Float, 3, 3> hessians,
        cuda_tool::CBufferView<IndexT>             v2b,
        cuda_tool::BufferView<std::uint64_t>       keys,
        cuda_tool::BufferView<IndexT>              receiver_indices,
        IndexT                                     vertex_offset,
        int                                        body_pair_col_bits,
        int                                        n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;

        const auto& [g_i, g_j, H3x3] = hessians(I);
        const auto i                 = g_i - vertex_offset;
        const auto j                 = g_j - vertex_offset;
        const auto body_i            = v2b(i);
        const auto body_j            = v2b(j);
        const auto left              = min(body_i, body_j);
        const auto right             = max(body_i, body_j);

        UIPC_KERNEL_ASSERT(left >= 0 && right >= left,
                           "ABD DyTopo body pair must be ordered and nonnegative");
        keys(I)             = pack_body_pair(left, right, body_pair_col_bits);
        receiver_indices(I) = I;
        (void)H3x3;
    }

    __global__ void abd_dytopo_body_pair_block_counts_kernel(
        cuda_tool::CBufferView<std::uint64_t> unique_keys,
        cuda_tool::BufferView<int>            block_counts,
        int                                   body_pair_col_bits,
        int                                   n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const auto key = unique_keys(I);
        // An off-diagonal 12x12 body pair contributes all sixteen 3x3
        // blocks. A diagonal body pair contributes the ten upper blocks;
        // the lower six were discarded by ge2sym in the legacy path.
        block_counts(I) = body_pair_left(key, body_pair_col_bits)
                                  == body_pair_right(key, body_pair_col_bits) ?
                              10 :
                              16;
    }

    __global__ void abd_dytopo_body_pair_total_block_count_kernel(
        cuda_tool::CBufferView<int> block_counts,
        cuda_tool::CBufferView<int> block_offsets,
        cuda_tool::Dense<int>       total_count,
        int                         n)
    {
        if(blockIdx.x == 0 && threadIdx.x == 0)
            *total_count = n == 0 ? 0 : block_offsets(n - 1) + block_counts(n - 1);
    }

    inline UIPC_GENERIC int same_body_upper_block_slot(int block_i, int block_j)
    {
        // Row starts are 0, 4, 7, 9 for the upper triangle of a 4x4
        // block matrix.
        return block_i * 3 - block_i * (block_i - 1) / 2 + block_j;
    }

    __global__ void abd_linear_subsystem_assemble_kinetic_shape_k1_kernel(
        cuda_tool::CBufferView<IndexT>    is_fixed,
        cuda_tool::CBufferView<IndexT>    is_external_kinetic,
        cuda_tool::CBufferView<Vector12>  shape_gradient,
        cuda_tool::CBufferView<Vector12>  kinetic_gradient,
        cuda_tool::DenseVectorView<Float> gradients,
        int                               n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Vector12 src;

        if(is_fixed(i))
        {
            src.setZero();  // if fixed, set to zero
        }
        else
        {
            src = shape_gradient(i);

            // if not external kinetic, add kinetic gradient
            if(!is_external_kinetic(i)) [[likely]]
            {
                src += kinetic_gradient(i);
            }
        }

        gradients.segment<12>(i * 12) = src;

        // cout << "EKG(" << i << "): " << src.transpose().eval() << "\n";
    }

    __global__ void abd_linear_subsystem_assemble_kinetic_shape_k2_kernel(
        cuda_tool::TripletMatrixView<Float, 3> dst,
        cuda_tool::CBufferView<IndexT>         is_fixed,
        cuda_tool::CBufferView<IndexT>         is_external_kinetic,
        cuda_tool::CBufferView<Matrix12x12>    shape_hessian,
        cuda_tool::CBufferView<Matrix12x12>    kinetic_hessian,
        cuda_tool::BufferView<Matrix12x12>     diag_hessian,
        int                                    n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        TripletMatrixUnpacker MA{dst};
        Matrix12x12           H12x12;

        if(is_fixed(I))
        {
            // Fill kinetic hessian to identity to avoid singularity
            H12x12.setIdentity();
        }
        else
        {
            // if not fixed, fill shape hessian
            H12x12 = shape_hessian(I);

            // if not external kinetic, add kinetic gradient
            if(!is_external_kinetic(I)) [[likely]]
            {
                H12x12 += kinetic_hessian(I);
            }
        }

        // record diagonal hessian for diag-inv preconditioner
        diag_hessian(I) = H12x12;

        // set the lower triangle blocks to zero for robustness
        zero_out_lower(H12x12);

        MA.block<4, 4>(I * 4 * 4)  // triplet range of [I*4*4, (I+1)*4*4)
            .write(I * 4,          // begin row
                   I * 4,          // begin col
                   H12x12);
    }

    __global__ void abd_linear_subsystem_assemble_reporters_k1_kernel(
        cuda_tool::DenseVectorView<Float>        dst,
        cuda_tool::CDoubletVectorView<Float, 12> src,
        cuda_tool::CBufferView<IndexT>           is_fixed,
        int                                      n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [body_i, G12] = src(I);

        if(is_fixed(body_i))
        {
            // Do nothing
        }
        else
        {
            dst.segment<12>(body_i * 12).atomic_add(G12);
        }
    }

    __global__ void abd_linear_subsystem_assemble_reporters_k2_kernel(
        cuda_tool::TripletMatrixView<Float, 3>       dst,
        cuda_tool::CTripletMatrixView<Float, 12, 12> src,
        cuda_tool::BufferView<Matrix12x12>           diag_hessian,
        cuda_tool::CBufferView<IndexT>               is_fixed,
        int                                          n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        TripletMatrixUnpacker MU{dst};
        Matrix12x12           H12x12;
        auto&& [body_i, body_j, Value] = src(I);
        H12x12                         = Value;

        bool has_fixed = (is_fixed(body_i) || is_fixed(body_j));

        // Fill diagonal hessian for diag-inv preconditioner
        if(body_i == body_j && !has_fixed)
        {
            eigen::atomic_add(diag_hessian(body_i), H12x12);
        }

        if(has_fixed)
        {
            // Zero out hessian for fixed bodies
            H12x12.setZero();
        }
        else
        {
            if(body_i == body_j)
            {
                // Since body_i == body_j, we only fill the upper triangle part
                zero_out_lower(H12x12);
            }
            else if(body_i > body_j)
            {
                // If all the reporters only report upper triangle part, this branch should not be hit
                H12x12.setZero();
            }
        }

        MU.block<4, 4>(I * 4 * 4)  // triplet range of [I*4*4, (I+1)*4*4)
            .write(body_i * 4,     // begin row
                   body_j * 4,     // begin col
                   H12x12);
    }

    __global__ void abd_linear_subsystem_assemble_dytopo_effect_k1_kernel(
        cuda_tool::CDoubletVectorView<Float, 3> dytopo_effect_gradient,
        cuda_tool::DenseVectorView<Float>       gradients,
        cuda_tool::CBufferView<IndexT>          v2b,
        cuda_tool::CBufferView<ABDJacobi>       Js,
        cuda_tool::CBufferView<IndexT>          is_fixed,
        IndexT                                  vertex_offset,
        int                                     n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const auto& [g_i, G3] = dytopo_effect_gradient(I);

        auto  i      = g_i - vertex_offset;
        auto  body_i = v2b(i);
        auto& J_i    = Js(i);

        if(is_fixed(body_i))
        {
            // Do nothing
        }
        else
        {
            Vector12 G12 = J_i.T() * G3;
            gradients.segment<12>(body_i * 12).atomic_add(G12);

            // cout << "DG(" << I << "): " << G12.transpose().eval() << "\n";
        }
    }

    __global__ void abd_linear_subsystem_assemble_dytopo_effect_k2_kernel(
        cuda_tool::CTripletMatrixView<Float, 3, 3> dytopo_effect_hessian,
        cuda_tool::TripletMatrixView<Float, 3>     dst,
        cuda_tool::CBufferView<IndexT>             v2b,
        cuda_tool::CBufferView<ABDJacobi>          Js,
        cuda_tool::CBufferView<IndexT>             is_fixed,
        cuda_tool::BufferView<Matrix12x12>         diag_hessian,
        IndexT                                     vertex_offset,
        int                                        n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const auto& [g_i, g_j, H3x3] = dytopo_effect_hessian(I);

        auto i = g_i - vertex_offset;
        auto j = g_j - vertex_offset;

        auto body_i = v2b(i);
        auto body_j = v2b(j);

        auto& J_i = Js(i);
        auto& J_j = Js(j);

        // We know half contact hessian i <= j
        // but we don't know body_i and body_j order
        // so test and swap if necessary
        IndexT L = body_i;
        IndexT R = body_j;
        if(body_i > body_j)
        {
            L = body_j;
            R = body_i;
        }

        const bool has_fixed = is_fixed(body_i) || is_fixed(body_j);
        for(IndexT block_i = 0; block_i < 4; ++block_i)
        {
            for(IndexT block_j = 0; block_j < 4; ++block_j)
            {
                Matrix3x3 block = Matrix3x3::Zero();
                if(!has_fixed)
                {
                    if(body_i < body_j)
                    {
                        block = ABDJacobi::JT_H_J_block(J_i.T(), H3x3, J_j, block_i, block_j);
                    }
                    else if(body_i > body_j)
                    {
                        block = ABDJacobi::JT_H_J_block(
                            J_j.T(), H3x3.transpose(), J_i, block_i, block_j);
                    }
                    else if(i != j)
                    {
                        block = ABDJacobi::JT_H_J_block(J_i.T(), H3x3, J_j, block_i, block_j)
                                + ABDJacobi::JT_H_J_block(
                                    J_j.T(), H3x3.transpose(), J_i, block_i, block_j);
                    }
                    else
                    {
                        block = ABDJacobi::JT_H_J_block(J_i.T(), H3x3, J_j, block_i, block_j);
                    }
                }

                if(body_i == body_j && !has_fixed)
                {
                    for(IndexT row = 0; row < 3; ++row)
                        for(IndexT col = 0; col < 3; ++col)
                            atomicAdd(&diag_hessian(body_i)(3 * block_i + row, 3 * block_j + col),
                                      block(row, col));
                }

                // A same-body contribution is symmetric after both vertex
                // orders are accumulated. Keep the raw triplet contract: the
                // lower 3x3 blocks are explicit zero entries.
                if(body_i == body_j && block_i > block_j)
                    block.setZero();

                dst(I * 16 + block_i * 4 + block_j).write(L * 4 + block_i, R * 4 + block_j, block);
            }
        }
    }

    // One CUDA block owns one ordered body pair. Its sixteen warps each
    // reduce one 3x3 block of the corresponding 12x12 body-pair matrix.
    // Same-body lower blocks are retained long enough to update the complete
    // diagonal Hessian, but only the ten upper blocks enter the global
    // triplet stream, matching the legacy ge2sym contract.
    __global__ void abd_linear_subsystem_aggregate_dytopo_body_pairs_kernel(
        cuda_tool::CTripletMatrixView<Float, 3, 3> dytopo_effect_hessian,
        cuda_tool::TripletMatrixView<Float, 3>     dst,
        cuda_tool::CBufferView<IndexT>             sorted_receiver_indices,
        cuda_tool::CBufferView<std::uint64_t>      unique_body_pair_keys,
        cuda_tool::CBufferView<int>                body_pair_counts,
        cuda_tool::CBufferView<int>                body_pair_offsets,
        cuda_tool::CBufferView<int>                body_pair_block_offsets,
        cuda_tool::CBufferView<IndexT>             v2b,
        cuda_tool::CBufferView<ABDJacobi>          Js,
        cuda_tool::CBufferView<IndexT>             is_fixed,
        cuda_tool::BufferView<Matrix12x12>         diag_hessian,
        IndexT                                     vertex_offset,
        int                                        body_pair_col_bits,
        int                                        pair_count)
    {
        constexpr int warp_size = 32;

        const int pair_index  = blockIdx.x;
        const int block_index = threadIdx.x / warp_size;
        const int lane_index  = threadIdx.x & (warp_size - 1);
        if(pair_index >= pair_count || block_index >= 16)
            return;

        const auto pair_key  = unique_body_pair_keys(pair_index);
        const auto left      = body_pair_left(pair_key, body_pair_col_bits);
        const auto right     = body_pair_right(pair_key, body_pair_col_bits);
        const auto block_i   = block_index / 4;
        const auto block_j   = block_index % 4;
        const bool same_body = left == right;
        const bool has_fixed = is_fixed(left) || is_fixed(right);

        Matrix3x3 sum = Matrix3x3::Zero();
        if(!has_fixed)
        {
            const int pair_offset = body_pair_offsets(pair_index);
            const int pair_size   = body_pair_counts(pair_index);
            for(int k = lane_index; k < pair_size; k += warp_size)
            {
                const auto receiver_index = sorted_receiver_indices(pair_offset + k);
                const auto& [g_i, g_j, H3x3] = dytopo_effect_hessian(receiver_index);

                const auto i      = g_i - vertex_offset;
                const auto j      = g_j - vertex_offset;
                const auto body_i = v2b(i);
                const auto body_j = v2b(j);
                Matrix3x3  block;

                if(body_i < body_j)
                {
                    block = ABDJacobi::JT_H_J_block(Js(i).T(), H3x3, Js(j), block_i, block_j);
                }
                else if(body_i > body_j)
                {
                    block = ABDJacobi::JT_H_J_block(
                        Js(j).T(), H3x3.transpose(), Js(i), block_i, block_j);
                }
                else if(i != j)
                {
                    block = ABDJacobi::JT_H_J_block(Js(i).T(), H3x3, Js(j), block_i, block_j)
                            + ABDJacobi::JT_H_J_block(
                                Js(j).T(), H3x3.transpose(), Js(i), block_i, block_j);
                }
                else
                {
                    block = ABDJacobi::JT_H_J_block(Js(i).T(), H3x3, Js(j), block_i, block_j);
                }
                sum += block;
            }

#pragma unroll
            for(int delta = warp_size / 2; delta > 0; delta /= 2)
            {
#pragma unroll
                for(int row = 0; row < 3; ++row)
                {
#pragma unroll
                    for(int col = 0; col < 3; ++col)
                    {
                        sum(row, col) +=
                            __shfl_down_sync(0xFFFFFFFFu, sum(row, col), delta);
                    }
                }
            }
        }

        if(lane_index != 0)
            return;

        if(same_body)
        {
            if(!has_fixed)
            {
#pragma unroll
                for(int row = 0; row < 3; ++row)
                {
#pragma unroll
                    for(int col = 0; col < 3; ++col)
                    {
                        diag_hessian(left)(3 * block_i + row, 3 * block_j + col) +=
                            sum(row, col);
                    }
                }
            }

            if(block_i > block_j)
                return;

            const int output_index = body_pair_block_offsets(pair_index)
                                     + same_body_upper_block_slot(block_i, block_j);
            dst(output_index).write(left * 4 + block_i, right * 4 + block_j, sum);
        }
        else
        {
            const int output_index = body_pair_block_offsets(pair_index) + block_index;
            dst(output_index).write(left * 4 + block_i, right * 4 + block_j, sum);
        }
    }

    __global__ void abd_linear_subsystem_retrieve_solution_kernel(
        cuda_tool::BufferView<Vector12> dq, cuda_tool::CDenseVectorView<Float> x, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        // retrieve solution for each body
        dq(i) = -x.segment<12>(i * 12).as_eigen();
    }

    __global__ void abd_linear_subsystem_diag_norm_kernel(
        cuda_tool::CBufferView<Matrix12x12> diag_hess,
        cuda_tool::BufferView<Float>        diag_blocks_norm,
        cuda_tool::CBufferView<IndexT>      is_fixed,
        int                                 n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        for(int i = 0; i < 12; i++)
            diag_blocks_norm(idx * 12 + i) =
                is_fixed(idx) ? 0 : abs(diag_hess(idx)(i, i));
    }

    __global__ void abd_linear_subsystem_mass_norm_kernel(
        cuda_tool::CBufferView<ABDJacobiDyadicMass> mass,
        cuda_tool::BufferView<Float>                block_norm,
        cuda_tool::CBufferView<IndexT>              is_fixed,
        int                                         n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        block_norm(idx) = is_fixed(idx) ? 0 : mass(idx).mass();
    }
}  // namespace
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDLinearSubsystem);

// ref: https://github.com/spiriMirror/libuipc/issues/271
constexpr U64 ABDLinearSubsystemUID = 0ull;

void ABDLinearSubsystem::do_build(DiagLinearSubsystem::BuildInfo& info)
{
    m_impl.affine_body_dynamics        = require<AffineBodyDynamics>();
    m_impl.affine_body_vertex_reporter = require<AffineBodyVertexReporter>();
    m_impl.dt_attr = world().scene().config().find<Float>("dt");
    UIPC_ASSERT(m_impl.dt_attr, "Scene config must have a 'dt' attribute.");

    m_impl.dytopo_effect_receiver = find<ABDDyTopoEffectReceiver>();
}

void ABDLinearSubsystem::Impl::init()
{
    auto reporter_view = reporters.view();
    for(auto&& [i, r] : enumerate(reporter_view))
        r->m_index = i;
    for(auto& r : reporter_view)
        r->init();

    reporter_gradient_offsets_counts.resize(reporter_view.size());
    reporter_hessian_offsets_counts.resize(reporter_view.size());

    SizeT body_count = abd().body_count();
    body_id_to_shape_hessian.resize(body_count);
    body_id_to_shape_gradient.resize(body_count);
    body_id_to_kinetic_hessian.resize(body_count);
    body_id_to_kinetic_gradient.resize(body_count);
    diag_hessian.resize(body_count);
}

void ABDLinearSubsystem::Impl::report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    info.extent(abd().body_count() * 12);
}

void ABDLinearSubsystem::Impl::receive_init_dof_info(WorldVisitor& w,
                                                     GlobalLinearSystem::InitDofInfo& info)
{
    auto& geo_infos = abd().geo_infos;
    auto  geo_slots = w.scene().geometries();

    IndexT offset = info.dof_offset();

    // fill the dof_offset and dof_count for each geometry
    affine_body_dynamics->for_each(
        geo_slots,
        [&](const AffineBodyDynamics::ForEachInfo& foreach_info, geometry::SimplicialComplex& sc)
        {
            auto I          = foreach_info.global_index();
            auto dof_offset = sc.meta().find<IndexT>(builtin::dof_offset);
            UIPC_ASSERT(dof_offset, "dof_offset not found on ABD mesh why can it happen?");
            auto dof_count = sc.meta().find<IndexT>(builtin::dof_count);
            UIPC_ASSERT(dof_count, "dof_count not found on ABD mesh why can it happen?");

            IndexT this_dof_count = 12 * sc.instances().size();
            view(*dof_offset)[0]  = offset;
            view(*dof_count)[0]   = this_dof_count;

            offset += this_dof_count;
        });

    UIPC_ASSERT(offset == info.dof_offset() + info.dof_count(), "dof size mismatch");
}

SizeT ABDLinearSubsystem::Impl::_prepare_dytopo_body_pair_aggregation()
{
    using namespace cuda_tool;

    const SizeT receiver_count =
        dytopo_effect_receiver ? dytopo_effect_receiver->hessians().triplet_count() : 0;
    dytopo_prepared_receiver_count = receiver_count;
    dytopo_unique_body_pair_count  = 0;
    dytopo_aggregated_block_count  = 0;

    if(receiver_count == 0)
        return 0;

    UIPC_ASSERT(receiver_count <= static_cast<SizeT>(std::numeric_limits<int>::max()),
                "ABD DyTopo receiver count {} exceeds the CUDA kernel index range",
                receiver_count);
    UIPC_ASSERT(abd().body_count()
                    <= static_cast<SizeT>(std::numeric_limits<IndexT>::max()),
                "ABD body count {} exceeds the IndexT range",
                abd().body_count());

    std::uint32_t max_body_id =
        abd().body_count() > 0 ? static_cast<std::uint32_t>(abd().body_count() - 1) : 0;
    int col_bits = 1;
    while(max_body_id >>= 1)
        ++col_bits;
    dytopo_body_pair_col_bits = col_bits;

    loose_resize(dytopo_body_pair_keys_input, receiver_count, reserve_ratio);
    loose_resize(dytopo_body_pair_keys, receiver_count, reserve_ratio);
    loose_resize(dytopo_receiver_indices_input, receiver_count, reserve_ratio);
    loose_resize(dytopo_receiver_indices, receiver_count, reserve_ratio);

    auto k = abd_dytopo_build_body_pair_keys_kernel;
    int  n = static_cast<int>(receiver_count);
    k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
        dytopo_effect_receiver->hessians(),
        abd().vertex_id_to_body_id.cview(),
        dytopo_body_pair_keys_input.view(),
        dytopo_receiver_indices_input.view(),
        affine_body_vertex_reporter->vertex_offset(),
        dytopo_body_pair_col_bits,
        n);

    DeviceRadixSort().SortPairs(dytopo_body_pair_keys_input.data(),
                                dytopo_body_pair_keys.data(),
                                dytopo_receiver_indices_input.data(),
                                dytopo_receiver_indices.data(),
                                static_cast<int>(receiver_count),
                                0,
                                2 * dytopo_body_pair_col_bits);

    loose_resize(dytopo_unique_body_pair_keys, receiver_count, reserve_ratio);
    loose_resize(dytopo_body_pair_counts, receiver_count, reserve_ratio);
    DeviceRunLengthEncode().Encode(dytopo_body_pair_keys.data(),
                                   dytopo_unique_body_pair_keys.data(),
                                   dytopo_body_pair_counts.data(),
                                   dytopo_unique_body_pair_count_dev.data(),
                                   static_cast<int>(receiver_count));

    const int pair_count = dytopo_unique_body_pair_count_dev;
    UIPC_ASSERT(pair_count >= 0, "ABD DyTopo body-pair run count must be nonnegative");
    dytopo_unique_body_pair_count = static_cast<SizeT>(pair_count);
    dytopo_unique_body_pair_keys.resize_discard(pair_count);
    dytopo_body_pair_counts.resize_discard(pair_count);
    loose_resize(dytopo_body_pair_offsets, pair_count, reserve_ratio);
    loose_resize(dytopo_body_pair_block_counts, pair_count, reserve_ratio);
    loose_resize(dytopo_body_pair_block_offsets, pair_count, reserve_ratio);

    DeviceScan().ExclusiveSum(dytopo_body_pair_counts.data(),
                              dytopo_body_pair_offsets.data(),
                              pair_count);

    auto k2 = abd_dytopo_body_pair_block_counts_kernel;
    if(pair_count > 0)
        k2<<<cuda_tool::best_grid_dim(pair_count, k2), cuda_tool::best_block_dim(k2), 0, nullptr>>>(
            dytopo_unique_body_pair_keys.cview(),
            dytopo_body_pair_block_counts.view(),
            dytopo_body_pair_col_bits,
            pair_count);

    DeviceScan().ExclusiveSum(dytopo_body_pair_block_counts.data(),
                              dytopo_body_pair_block_offsets.data(),
                              pair_count);
    abd_dytopo_body_pair_total_block_count_kernel<<<1, 1>>>(
        dytopo_body_pair_block_counts.cview(),
        dytopo_body_pair_block_offsets.cview(),
        dytopo_aggregated_block_count_dev.viewer(),
        pair_count);

    const int block_count = dytopo_aggregated_block_count_dev;
    UIPC_ASSERT(block_count >= 0, "ABD DyTopo aggregated block count must be nonnegative");
    dytopo_aggregated_block_count = static_cast<SizeT>(block_count);

    return dytopo_aggregated_block_count;
}

void ABDLinearSubsystem::Impl::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    // 1. Gradient Count
    constexpr SizeT G12_to_dof = 12;
    SizeT           body_count = abd().body_count();
    auto            dof_count  = body_count * G12_to_dof;

    auto has_complement =
        has_flags(info.component_flags(), GlobalLinearSystem::ComponentFlags::Complement);

    SizeT H12x12_count = 0;

    if(has_complement)
    {
        // 1) Body hessian: kinetic + shape
        if(!info.gradient_only())
            H12x12_count += abd().body_count();

        // 2) Reporters
        auto reporter_view = reporters.view();
        auto grad_counts   = reporter_gradient_offsets_counts.counts();
        auto hess_counts   = reporter_hessian_offsets_counts.counts();

        for(auto&& R : reporter_view)
        {
            ReportExtentInfo extent_info;
            extent_info.m_gradient_only = info.gradient_only();
            R->report_extent(extent_info);

            grad_counts[R->m_index] = extent_info.m_gradient_count;
            hess_counts[R->m_index] = extent_info.m_hessian_count;
        }

        reporter_gradient_offsets_counts.scan();
        reporter_hessian_offsets_counts.scan();

        if(!info.gradient_only())
            H12x12_count += reporter_hessian_offsets_counts.total_count();
    }


    SizeT dytopo_H3x3_count = 0;
    if(dytopo_effect_receiver && !info.gradient_only())
        dytopo_H3x3_count = _prepare_dytopo_body_pair_aggregation();


    auto H3x3_count = H12x12_count * (4 * 4) + dytopo_H3x3_count;

    if(info.gradient_only())
    {
        UIPC_ASSERT(H3x3_count == 0,
                    "Hessian block count should be zero (got {}) when gradient_only is true",
                    H3x3_count);
    }

    info.extent(H3x3_count, dof_count);
}

void ABDLinearSubsystem::Impl::assemble(GlobalLinearSystem::DiagInfo& info)
{
    using namespace cuda_tool;

    // 0) Prepare buffers for reporters
    {
        auto N = abd().body_count();

        reporter_gradients.reshape(N);
        reporter_gradients.resize_doublets_discard(
            reporter_gradient_offsets_counts.total_count());

        reporter_hessians.reshape(N, N);
        reporter_hessians.resize_triplets_discard(
            reporter_hessian_offsets_counts.total_count());
    }

    bool has_complement =
        has_flags(info.component_flags(), GlobalLinearSystem::ComponentFlags::Complement);

    IndexT hess_offset = 0;

    // 1) Static Topo Effect: Kinetic + Shape + Other Reporters
    if(has_complement)
    {
        _assemble_kinetic_shape(hess_offset, info);
        _assemble_reporters(hess_offset, info);
    }
    else  // contact only
    {
        info.gradients().buffer_view().fill(0);
    }

    // 2) Dynamic Topology Effect
    _assemble_dytopo_effect(hess_offset, info);

    UIPC_ASSERT(hess_offset == info.hessians().triplet_count(),
                "Hessian size mismatch: expected {}, got {}",
                info.hessians().triplet_count(),
                hess_offset);
}

void ABDLinearSubsystem::Impl::_assemble_kinetic_shape(IndexT& hess_offset,
                                                       GlobalLinearSystem::DiagInfo& info)
{
    using namespace cuda_tool;

    Float dt = dt_attr->view()[0];

    // Collect Kinetic
    ABDLinearSubsystem::ComputeGradientHessianInfo this_info{
        info.gradient_only(), body_id_to_kinetic_gradient, body_id_to_kinetic_hessian, dt};
    abd().kinetic->compute_gradient_hessian(this_info);

    // Collect Shape
    for(auto&& [i, cst] : enumerate(abd().constitutions.view()))
    {
        ABDLinearSubsystem::ComputeGradientHessianInfo this_info{
            info.gradient_only(),
            abd().subview(body_id_to_shape_gradient, cst->m_index),
            abd().subview(body_id_to_shape_hessian, cst->m_index),
            dt};

        cst->compute_gradient_hessian(this_info);
    }

    {
        auto k = abd_linear_subsystem_assemble_kinetic_shape_k1_kernel;
        int  n = (int)abd().body_count();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                abd().body_id_to_is_fixed.cview(),
                abd().body_id_to_external_kinetic.cview(),
                body_id_to_shape_gradient.cview(),
                body_id_to_kinetic_gradient.cview(),
                info.gradients(),
                n);
    }

    if(info.gradient_only())
        return;

    auto body_count = body_id_to_shape_hessian.size();
    auto H3x3_count = body_count * (4 * 4);
    auto body_H3x3  = info.hessians().subview(hess_offset, H3x3_count);

    {
        auto k = abd_linear_subsystem_assemble_kinetic_shape_k2_kernel;
        int  n = (int)body_count;
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                body_H3x3,
                abd().body_id_to_is_fixed.cview(),
                abd().body_id_to_external_kinetic.cview(),
                body_id_to_shape_hessian.cview(),
                body_id_to_kinetic_hessian.cview(),
                diag_hessian.view(),
                n);
    }

    hess_offset += H3x3_count;
}

void ABDLinearSubsystem::Impl::_assemble_reporters(IndexT& offset,
                                                   GlobalLinearSystem::DiagInfo& info)
{
    using namespace cuda_tool;

    // Fill TripletMatrix and DoubletVector
    for(auto& R : reporters.view())
    {
        AssembleInfo assemble_info{this, R->m_index, info.gradient_only()};
        R->assemble(assemble_info);
    }

    if(reporter_gradients.doublet_count())
    {
        auto k = abd_linear_subsystem_assemble_reporters_k1_kernel;
        int  n = (int)reporter_gradients.doublet_count();
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.gradients(),
            reporter_gradients.cview(),
            abd().body_id_to_is_fixed.cview(),
            n);
    }

    if(!info.gradient_only() && reporter_hessians.triplet_count())
    {
        // get rest
        auto H3x3s = info.hessians().subview(offset);

        auto k = abd_linear_subsystem_assemble_reporters_k2_kernel;
        int  n = (int)reporter_hessians.triplet_count();
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            H3x3s,
            reporter_hessians.cview(),
            diag_hessian.view(),
            abd().body_id_to_is_fixed.cview(),
            n);

        offset += reporter_hessians.triplet_count() * (4 * 4);
    }
}

void ABDLinearSubsystem::Impl::_assemble_dytopo_effect(IndexT& offset,
                                                       GlobalLinearSystem::DiagInfo& info)
{
    using namespace cuda_tool;

    auto  vertex_offset = affine_body_vertex_reporter->vertex_offset();
    SizeT dytopo_effect_gradient_count = 0;
    if(dytopo_effect_receiver)
    {
        dytopo_effect_gradient_count =
            dytopo_effect_receiver->gradients().doublet_count();
    }

    if(dytopo_effect_gradient_count)
    {
        auto k = abd_linear_subsystem_assemble_dytopo_effect_k1_kernel;
        int  n = (int)dytopo_effect_gradient_count;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            dytopo_effect_receiver->gradients(),
            info.gradients(),
            abd().vertex_id_to_body_id.cview(),
            abd().vertex_id_to_J.cview(),
            abd().body_id_to_is_fixed.cview(),
            vertex_offset,
            n);
    }

    if(info.gradient_only())
        return;

    SizeT dytopo_effect_hessian_count = 0;
    if(dytopo_effect_receiver)
        dytopo_effect_hessian_count = dytopo_effect_receiver->hessians().triplet_count();

    if(dytopo_effect_hessian_count)
    {
        UIPC_ASSERT(dytopo_prepared_receiver_count == dytopo_effect_hessian_count,
                    "ABD DyTopo body-pair preparation is stale: prepared {}, assembling {}",
                    dytopo_prepared_receiver_count,
                    dytopo_effect_hessian_count);
    }

    auto H3x3_count         = dytopo_aggregated_block_count;
    auto dytopo_effect_H3x3 = info.hessians().subview(offset, H3x3_count);

    if(dytopo_effect_hessian_count)
    {
        // Half Contact Hessian
        // ref: https://github.com/spiriMirror/libuipc/issues/272
        constexpr int block_dim = 16 * 32;
        const int pair_count = static_cast<int>(dytopo_unique_body_pair_count);
        if(pair_count > 0)
            abd_linear_subsystem_aggregate_dytopo_body_pairs_kernel<<<pair_count, block_dim, 0, nullptr>>>(
                dytopo_effect_receiver->hessians(),
                dytopo_effect_H3x3,
                dytopo_receiver_indices.cview(),
                dytopo_unique_body_pair_keys.cview(),
                dytopo_body_pair_counts.cview(),
                dytopo_body_pair_offsets.cview(),
                dytopo_body_pair_block_offsets.cview(),
                abd().vertex_id_to_body_id.cview(),
                abd().vertex_id_to_J.cview(),
                abd().body_id_to_is_fixed.cview(),
                diag_hessian.view(),
                vertex_offset,
                dytopo_body_pair_col_bits,
                pair_count);
    }

    offset += H3x3_count;
}

void ABDLinearSubsystem::Impl::accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    info.satisfied(true);
}

void ABDLinearSubsystem::Impl::retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    using namespace cuda_tool;

    auto dq = abd().body_id_to_dq.view();
    {
        auto k = abd_linear_subsystem_retrieve_solution_kernel;
        int  n = (int)abd().body_count();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                dq, info.solution(), n);
    }
}

Float ABDLinearSubsystem::Impl::diag_norm()
{
    auto diag_hess = diag_hessian.view();
    block_norm.resize(diag_hess.size() * 12);
    {
        auto k = abd_linear_subsystem_diag_norm_kernel;
        int  n = (int)diag_hess.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                diag_hess.cview(), block_norm.view(), abd().body_id_to_is_fixed.cview(), n);
    }

    cuda_tool::DeviceReduce().Max(
        block_norm.data(), reduced_norm.data(), block_norm.size());

    return reduced_norm;
}

Float ABDLinearSubsystem::Impl::mass_norm()
{
    auto mass = abd().body_id_to_abd_mass.view();
    block_norm.resize(mass.size());
    {
        auto k = abd_linear_subsystem_mass_norm_kernel;
        int  n = (int)mass.size();
        if(n > 0)
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                mass.cview(), block_norm.view(), abd().body_id_to_is_fixed.cview(), n);
    }

    cuda_tool::DeviceReduce().Max(
        block_norm.data(), reduced_norm.data(), block_norm.size());

    return reduced_norm;
}
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
void ABDLinearSubsystem::do_init(InitInfo& info)
{
    m_impl.init();
}

void ABDLinearSubsystem::do_report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    m_impl.report_extent(info);
}

void ABDLinearSubsystem::do_assemble(GlobalLinearSystem::DiagInfo& info)
{
    m_impl.assemble(info);
}

void ABDLinearSubsystem::do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    m_impl.accuracy_check(info);
}

void ABDLinearSubsystem::do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    m_impl.retrieve_solution(info);
}

Float ABDLinearSubsystem::do_diag_norm(GlobalLinearSystem::DiagNormInfo& info)
{
    return m_impl.diag_norm();
}

Float ABDLinearSubsystem::do_mass_norm(GlobalLinearSystem::DiagNormInfo& info)
{
    return m_impl.mass_norm();
}

U64 ABDLinearSubsystem::get_uid() const noexcept
{
    return ABDLinearSubsystemUID;
}

void ABDLinearSubsystem::add_reporter(ABDLinearSubsystemReporter* reporter)
{
    UIPC_ASSERT(reporter, "reporter cannot be null");
    check_state(SimEngineState::BuildSystems, "add_reporter");
    m_impl.reporters.register_sim_system(*reporter);
}

void ABDLinearSubsystem::do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    m_impl.report_init_extent(info);
}

void ABDLinearSubsystem::do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info)
{
    m_impl.receive_init_dof_info(world(), info);
}

ABDLinearSubsystem::AssembleInfo::AssembleInfo(Impl* impl, IndexT index, bool gradient_only) noexcept
    : m_impl(impl)
    , m_index(index)
    , m_gradient_only(gradient_only)
{
}

cuda_tool::DoubletVectorView<Float, 12> ABDLinearSubsystem::AssembleInfo::gradients() const
{
    auto [offset, count] = m_impl->reporter_gradient_offsets_counts[m_index];
    return m_impl->reporter_gradients.view().subview(offset, count);
}

cuda_tool::TripletMatrixView<Float, 12, 12> ABDLinearSubsystem::AssembleInfo::hessians() const
{
    auto [offset, count] = m_impl->reporter_hessian_offsets_counts[m_index];
    return m_impl->reporter_hessians.view().subview(offset, count);
}

bool ABDLinearSubsystem::AssembleInfo::gradient_only() const noexcept
{
    return m_gradient_only;
}

void ABDLinearSubsystem::ReportExtentInfo::gradient_count(SizeT size)
{
    m_gradient_count = size;
}

void ABDLinearSubsystem::ReportExtentInfo::hessian_count(SizeT size)
{
    m_hessian_count = size;
}

void ABDLinearSubsystem::ReportExtentInfo::check(std::string_view name) const
{
    check_report_extent(m_gradient_only_checked, m_gradient_only, m_hessian_count, name);
}

AffineBodyDynamics::Impl& ABDLinearSubsystem::Impl::abd() const noexcept
{
    return affine_body_dynamics->m_impl;
}
}  // namespace uipc::backend::cuda
