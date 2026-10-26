/*
 * NVIDIA contributions: ABD diagonal update/apply/dot fusion and its
 * local-preconditioner integration.
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

#include <linear_system/local_preconditioner.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/abd_linear_subsystem.h>
#include <linear_system/global_linear_system.h>
#include <linear_system/fused_pcg_kernels.h>
#include <cuda_tool/cuda_tool.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void abd_diag_preconditioner_do_assemble_kernel(
        cuda_tool::CBufferView<Matrix12x12> diag_hessian,
        cuda_tool::BufferView<Matrix12x12>  diag_inv,
        int                                 n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        diag_inv(i) = cuda_tool::eigen::inverse(diag_hessian(i));
    }

    __global__ void abd_diag_preconditioner_do_apply_kernel(
        cuda_tool::CDenseVectorView<Float> r,
        cuda_tool::DenseVectorView<Float>  z,
        cuda_tool::CDense<IndexT>          converged,
        cuda_tool::BufferView<Matrix12x12> diag_inv,
        int                                n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(*converged != 0)
            return;
        z.segment<12>(i * 12).as_eigen() = diag_inv(i) * r.segment<12>(i * 12).as_eigen();
    }

    __global__ void abd_diag_preconditioner_fused_pcg_kernel(
        cuda_tool::CBufferView<Matrix12x12> diag_inv,
        cuda_tool::DenseVectorView<Float>   x,
        cuda_tool::CDenseVectorView<Float>  p,
        cuda_tool::DenseVectorView<Float>   r,
        cuda_tool::CDenseVectorView<Float>  Ap,
        cuda_tool::DenseVectorView<Float>   z,
        cuda_tool::CDense<Float>            rz,
        cuda_tool::CDense<Float>            pAp,
        cuda_tool::Dense<Float>             rz_new,
        cuda_tool::CDense<IndexT>           converged,
        int                                 body_count)
    {
        constexpr int    WarpSize      = 32;
        constexpr int    WarpsPerBlock = 32 / WarpSize;
        __shared__ Float body_dots[WarpsPerBlock];

        const int  lane          = threadIdx.x & (WarpSize - 1);
        const int  warp_in_block = threadIdx.x / WarpSize;
        const int  body          = blockIdx.x * WarpsPerBlock + warp_in_block;
        const bool running       = *converged == 0;
        const bool valid         = running && body < body_count && lane < 12;

        Float r_value = 0;
        if(valid)
        {
            const int   i     = body * 12 + lane;
            const Float alpha = *rz / *pAp;
            x(i) += alpha * p(i);
            r_value = r(i) - alpha * Ap(i);
            r(i)    = r_value;
        }

        Float z_value = 0;
#pragma unroll
        for(int col = 0; col < 12; ++col)
        {
            const Float r_col = __shfl_sync(0xffffffffu, r_value, col);
            if(valid)
                z_value = fma(diag_inv(body)(lane, col), r_col, z_value);
        }
        if(valid)
            z(body * 12 + lane) = z_value;

        Float dot = valid ? r_value * z_value : Float{0};
#pragma unroll
        for(int offset = 16; offset > 0; offset >>= 1)
            dot += __shfl_down_sync(0xffffffffu, dot, offset);
        if(lane == 0)
            body_dots[warp_in_block] = body < body_count && running ? dot : Float{0};
        __syncthreads();

        if(threadIdx.x == 0)
        {
            Float block_dot = 0;
#pragma unroll
            for(int i = 0; i < WarpsPerBlock; ++i)
                block_dot += body_dots[i];
            if(block_dot != Float{0})
                atomicAdd(rz_new.data(), block_dot);
        }
    }
}  // namespace

void launch_fused_pcg_abd_update_apply_dot(cuda_tool::CBufferView<Matrix12x12> diag_inv,
                                           cuda_tool::DenseVectorView<Float>  x,
                                           cuda_tool::CDenseVectorView<Float> p,
                                           cuda_tool::DenseVectorView<Float>  r,
                                           cuda_tool::CDenseVectorView<Float> Ap,
                                           cuda_tool::DenseVectorView<Float> z,
                                           cuda_tool::CVarView<Float>        rz,
                                           cuda_tool::CVarView<Float> pAp,
                                           cuda_tool::VarView<Float>  rz_new,
                                           cuda_tool::CVarView<IndexT> converged,
                                           cudaStream_t stream)
{
    UIPC_ASSERT(x.size() == diag_inv.size() * 12,
                "ABD fused PCG segment has {} scalars for {} bodies",
                x.size(),
                diag_inv.size());

    constexpr int WarpsPerBlock = 1;
    const int     body_count    = (int)diag_inv.size();
    const int     grid_size = (body_count + WarpsPerBlock - 1) / WarpsPerBlock;
    if(grid_size > 0)
    {
        abd_diag_preconditioner_fused_pcg_kernel<<<grid_size, 32, 0, stream>>>(
            diag_inv,
            x,
            p,
            r,
            Ap,
            z,
            rz.cviewer(),
            pAp.cviewer(),
            rz_new.viewer(),
            converged.cviewer(),
            body_count);
    }
}

class ABDDiagPreconditioner final : public LocalPreconditioner
{
  public:
    using LocalPreconditioner::LocalPreconditioner;

    ABDLinearSubsystem* abd_linear_subsystem = nullptr;

    cuda_tool::DeviceBuffer<Matrix12x12> diag_inv;

    virtual void do_build(BuildInfo& info) override
    {
        auto& global_linear_system = require<GlobalLinearSystem>();
        abd_linear_subsystem       = &require<ABDLinearSubsystem>();

        info.connect(abd_linear_subsystem);
    }

    virtual void do_init(InitInfo& info) override {}

    virtual void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) override
    {
        auto diag_hessian = abd_linear_subsystem->diag_hessian();
        diag_inv.resize(diag_hessian.size());

        int n = (int)diag_inv.size();
        if(n > 0)
        {
            auto k = abd_diag_preconditioner_do_assemble_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                diag_hessian, diag_inv.view(), n);
        }
    }

    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override
    {
        auto converged = info.converged();

        int n = (int)diag_inv.size();
        if(n > 0)
        {
            auto k = abd_diag_preconditioner_do_apply_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, info.stream()>>>(
                info.r(), info.z(), converged.cviewer(), diag_inv.view(), n);
        }
    }

    virtual bool do_fused_pcg_update_apply_dot(GlobalLinearSystem::FusedPcgUpdateApplyDotInfo& info) override
    {
        launch_fused_pcg_abd_update_apply_dot(diag_inv.cview(),
                                              info.x(),
                                              info.p(),
                                              info.r(),
                                              info.Ap(),
                                              info.z(),
                                              info.rz(),
                                              info.pAp(),
                                              info.rz_new(),
                                              info.converged(),
                                              info.stream());
        return true;
    }
};

REGISTER_SIM_SYSTEM(ABDDiagPreconditioner);
}  // namespace uipc::backend::cuda
