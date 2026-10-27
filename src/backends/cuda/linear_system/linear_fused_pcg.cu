/*
 * NVIDIA contributions: fused iteration kernels, convergence/direction-state
 * publication, ping-pong workspaces and CUDA Graph lifecycle integration.
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

#include <linear_system/linear_fused_pcg.h>
#include <sim_engine.h>
#include <linear_system/global_linear_system.h>
#include <linear_system/fused_pcg_kernels.h>
#include <cuda_tool/linear_reduction.h>
#include <uipc/common/timer.h>
#include <cub/warp/warp_reduce.cuh>
#include <cuda_tool/cub.h>
#include <algorithm>
#include <limits>
#include <optional>
namespace uipc::backend::cuda
{
namespace
{
    constexpr int FusedPcgDirectionBlockSize = 256;
    constexpr IndexT FusedPcgPublishClaimed = std::numeric_limits<IndexT>::min();
    constexpr IndexT FusedPcgRunning        = 0;
    constexpr IndexT FusedPcgConverged      = 1;
    constexpr IndexT FusedPcgExhausted      = 2;

    static_assert(IndexT{0} > FusedPcgPublishClaimed);

    constexpr IndexT select_graph_mode(IndexT requested, bool conditional_while_supported)
    {
        if(requested <= 0)
            return 0;
        if(requested == 1)
            return 1;
        return conditional_while_supported ? 2 : 1;
    }

    static_assert(select_graph_mode(0, false) == 0);
    static_assert(select_graph_mode(1, false) == 1);
    static_assert(select_graph_mode(2, false) == 1);
    static_assert(select_graph_mode(2, true) == 2);

    __global__ void fused_dot_kernel(cuda_tool::CDenseVectorView<Float> x,
                                     cuda_tool::CDenseVectorView<Float> y,
                                     cuda_tool::Dense<Float> d_result,
                                     int                     n)
    {
        constexpr int block_dim = 256;
        constexpr int warp_size = 32;
        constexpr int num_warps = block_dim / warp_size;

        using WarpReduce = cub::WarpReduce<Float, warp_size>;
        __shared__ typename WarpReduce::TempStorage temp_storage[num_warps];

        int   i   = blockIdx.x * blockDim.x + threadIdx.x;
        Float val = (i < n) ? x(i) * y(i) : Float(0);

        int   warp_id  = threadIdx.x / warp_size;
        int   lane_id  = threadIdx.x & (warp_size - 1);
        Float warp_sum = WarpReduce(temp_storage[warp_id]).Sum(val);

        // two-level reduction: one atomic per block instead of one per warp —
        // ~4k same-address atomic doubles serialize badly on a single counter
        __shared__ Float s_partials[num_warps];
        if(lane_id == 0)
            s_partials[warp_id] = warp_sum;
        __syncthreads();
        if(threadIdx.x < warp_size)
        {
            Float partial =
                (threadIdx.x < num_warps) ? s_partials[threadIdx.x] : Float(0);
            __syncwarp();
            partial = WarpReduce(temp_storage[0]).Sum(partial);
            if(threadIdx.x == 0)
                cuda_tool::atomic_add(d_result.data(), partial);
        }
    }

    __global__ void fused_update_xr_kernel(cuda_tool::CDense<Float> d_rz,
                                           cuda_tool::CDense<Float> d_pAp,
                                           cuda_tool::CDense<IndexT> d_converged,
                                           cuda_tool::DenseVectorView<Float>  x,
                                           cuda_tool::CDenseVectorView<Float> p,
                                           cuda_tool::DenseVectorView<Float>  r,
                                           cuda_tool::CDenseVectorView<Float> Ap,
                                           int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(*d_converged != 0)
            return;
        Float alpha = *d_rz / *d_pAp;
        x(i) += alpha * p(i);
        r(i) -= alpha * Ap(i);
    }

    __global__ void fused_update_p_kernel(cuda_tool::CDense<Float>  d_rz_new,
                                          cuda_tool::CDense<Float>  d_rz,
                                          cuda_tool::CDense<IndexT> d_converged,
                                          cuda_tool::DenseVectorView<Float>  p,
                                          cuda_tool::CDenseVectorView<Float> z,
                                          int                                n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(*d_converged != 0)
            return;
        Float beta = *d_rz_new / *d_rz;
        p(i)       = z(i) + beta * p(i);
    }

    __global__ void fused_swap_rz_kernel(cuda_tool::CDense<Float>  d_rz_new,
                                         cuda_tool::Dense<Float>   d_rz,
                                         cuda_tool::CDense<IndexT> d_converged)
    {
        if(*d_converged != 0)
            return;
        *d_rz = *d_rz_new;
    }

    __global__ void fused_update_converged_kernel(cuda_tool::CDense<Float> d_rz_new,
                                                  cuda_tool::Dense<IndexT> d_converged,
                                                  cuda_tool::CDense<Float> d_rz_tol,
                                                  int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Float rz_new = *d_rz_new;
        Float rz_tol = *d_rz_tol;
        *d_converged = abs(rz_new) <= rz_tol ? 1 : 0;
    }

    __global__ void fused_pcg_publish_update_direction_kernel(
        cuda_tool::DenseVectorView<Float>  p,
        cuda_tool::CDenseVectorView<Float> z,
        cuda_tool::Dense<Float>            rz,
        cuda_tool::CDense<Float>           rz_accum,
        cuda_tool::Dense<Float>            published_rz_new,
        cuda_tool::Dense<Float>            next_rz_accum,
        cuda_tool::Dense<Float>            beta,
        cuda_tool::Dense<IndexT>           converged,
        cuda_tool::CDense<Float>           rz_tol,
        cuda_tool::Dense<IndexT>           iteration_count,
        IndexT                             iteration_budget,
        int                                n)
    {
        __shared__ Float  block_beta;
        __shared__ IndexT block_updates_p;

        if(threadIdx.x == 0)
        {
            block_updates_p          = 0;
            IndexT observed =
                atomicCAS(converged.data(), FusedPcgRunning, FusedPcgPublishClaimed);

            if(observed == FusedPcgRunning)
            {
                const Float old_value = *rz;
                const Float new_value = *rz_accum;
                const bool  done      = ::fabs(new_value) <= *rz_tol;
                *published_rz_new     = new_value;

                // Only the scalar publisher counts this completed iteration.
                // A positive exhausted state guards the second captured slot
                // without reporting a budget exit as numerical convergence.
                bool exhausted = false;
                if(iteration_count.data())
                {
                    UIPC_KERNEL_ASSERT(::isfinite(new_value),
                                       "FusedPCG Iter: r^T*z = %g is not finite",
                                       new_value);
                    exhausted = ++(*iteration_count) >= iteration_budget;
                }

                if(done || exhausted)
                {
                    const IndexT stopped = done ? FusedPcgConverged : FusedPcgExhausted;
                    __threadfence();
                    atomicExch(converged.data(), stopped);
                    observed = stopped;
                }
                else
                {
                    const Float next_beta = new_value / old_value;
                    *beta                 = next_beta;
                    *rz                   = new_value;
                    *next_rz_accum        = Float{0};

                    // Publish the scalar state before exposing the negative
                    // per-block countdown. Running is restored only after
                    // every block has consumed beta and updated its p slice.
                    __threadfence();
                    const IndexT countdown = -static_cast<IndexT>(gridDim.x);
                    atomicExch(converged.data(), countdown);
                    observed = countdown;
                }
            }
            else
            {
                while(observed == FusedPcgPublishClaimed)
                    observed = atomicAdd(converged.data(), IndexT{0});
            }

            if(observed < 0 && observed != FusedPcgPublishClaimed)
            {
                // Acquire the publisher's beta and next-iteration scalars.
                __threadfence();
                block_beta      = *beta;
                block_updates_p = 1;
            }
        }
        __syncthreads();

        const int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(block_updates_p && i < n)
            p(i) = z(i) + block_beta * p(i);

        __syncthreads();
        if(threadIdx.x == 0 && block_updates_p)
            atomicAdd(converged.data(), IndexT{1});
    }

#if CUDA_TOOL_GRAPH_WHILE
    // --- full-GPU while-loop mode kernels (conditional node, CUDA >= 12.4) ---

    // per-launch reset, first node of the setup chain
    __global__ void pcg_while_reset_kernel(cuda_tool::Dense<IndexT> d_converged,
                                           cuda_tool::Dense<IndexT> d_iter)
    {
        *d_converged = 0;
        *d_iter      = 0;
    }

    // last node of the setup chain: rz_tol = tol_rate * |rz0| on device,
    // and the zero-system early exit (skip the loop body entirely)
    __global__ void pcg_while_setup_kernel(cuda_tool::CDense<Float> d_rz,
                                           cuda_tool::Dense<Float>  d_rz_tol,
                                           cuda_tool::Dense<IndexT> d_converged,
                                           Float                    tol_rate,
                                           IndexT iteration_budget,
                                           cudaGraphConditionalHandle handle)
    {
        Float rz0 = *d_rz;
        UIPC_KERNEL_ASSERT(::isfinite(rz0), "FusedPCG Init: r^T*z = %g is not finite", rz0);
        *d_rz_tol = ::abs(rz0) * tol_rate;
        *d_converged = ::abs(rz0) == Float{0.0} ? FusedPcgConverged :
                       iteration_budget <= 0    ? FusedPcgExhausted :
                                                  FusedPcgRunning;
        cudaGraphSetConditional(handle, *d_converged == FusedPcgRunning ? 1u : 0u);
    }

    // Each fused direction publisher already records its iteration and guards
    // subsequent slots on convergence or budget exhaustion. Re-enter only after
    // both ping-pong slots complete and leave the solver running.
    __global__ void pcg_while_control_kernel(cudaGraphConditionalHandle handle,
                                             cuda_tool::CDense<IndexT> d_converged)
    {
        cudaGraphSetConditional(handle, *d_converged == FusedPcgRunning ? 1u : 0u);
    }
#endif
}  // namespace

REGISTER_SIM_SYSTEM(LinearFusedPCG);

void LinearFusedPCG::do_build(BuildInfo& info)
{
    auto& config = world().scene().config();

    auto        solver_attr = config.find<std::string>("linear_system/solver");
    std::string solver_name =
        solver_attr ? solver_attr->view()[0] : std::string{"fused_pcg"};
    if(solver_name != "fused_pcg")
    {
        throw SimSystemException("LinearFusedPCG unused");
    }

    auto& global_linear_system = require<GlobalLinearSystem>();

    max_iter_ratio = 2;

    auto tol_rate_attr = config.find<Float>("linear_system/tol_rate");
    global_tol_rate    = tol_rate_attr->view()[0];

    auto check_attr = config.find<IndexT>("linear_system/check_interval");
    if(check_attr)
        check_interval = check_attr->view()[0];

    auto graph_attr = config.find<IndexT>("linear_system/use_cuda_graph");
    if(graph_attr)
        m_use_cuda_graph = graph_attr->view()[0];

    // v1 scope: graph replay is only enabled for the default IPC pipeline.
    // The al-ipc pipeline hits a host-side fail-fast (0xC0000409) during
    // stream capture *in the C++ test binary only* (the equivalent python
    // al-ipc scenes capture and replay fine) — root cause not yet found, so
    // keep al-ipc on the plain launch path until it is. See doc 09.
    auto constitution_attr = config.find<std::string>("contact/constitution");
    if(constitution_attr && constitution_attr->view()[0] != "ipc" && m_use_cuda_graph != 0)
    {
        m_use_cuda_graph = 0;
        logger::info("LinearFusedPCG: contact/constitution != ipc — CUDA graph replay disabled");
    }

    // graph mode: 2 = full-GPU while-loop (CUDA >= 12.4 toolkit+driver),
    // 1 = host-checked block replay, 0 = plain launches.
    // Mode 2 captures a pair of fused iterations with device-side stopping;
    // it removes host checks inside the solve but launch_sync still waits.
    // Keep mode 1 as the default until matched scene measurements establish
    // whether the conditional graph is faster for the intended workload.
    bool conditional_while_supported = false;
#if CUDA_TOOL_GRAPH_WHILE
    conditional_while_supported = cuda_tool::GraphWhile::runtime_supported();
#endif
    m_graph_mode = select_graph_mode(m_use_cuda_graph, conditional_while_supported);

    auto dump_attr = config.find<IndexT>("extras/debug/dump_linear_pcg");
    if(dump_attr && dump_attr->view()[0] != 0)
        logger::warn(
            "LinearFusedPCG: extras/debug/dump_linear_pcg is enabled but "
            "fused_pcg does not support PCG vector dumps. "
            "Set linear_system/solver to \"linear_pcg\" to use this feature.");

    logger::info("LinearFusedPCG: max_iter_ratio = {}, tol_rate = {}, check_interval = {}, graph_mode = {}",
                 max_iter_ratio,
                 global_tol_rate,
                 check_interval,
                 m_graph_mode);
}

void LinearFusedPCG::do_solve(GlobalLinearSystem::SolvingInfo& info)
{
    auto x = info.x();
    auto b = info.b();

    x.buffer_view().fill(0);

    auto N = x.size();
    if(r.capacity() < N)
    {
        auto M = reserve_ratio * N;
        r.reserve(M);
        z.reserve(M);
        p.reserve(M);
        Ap.reserve(M);
    }

    r.resize(N);
    z.resize(N);
    p.resize(N);
    Ap.resize(N);

    if(m_graph_mode != 0)
    {
        for(auto& graph_Ap : m_graph_Ap)
        {
            if(graph_Ap.capacity() < N)
                graph_Ap.reserve(reserve_ratio * N);
            graph_Ap.resize(N);
        }
    }

    auto iter = fused_pcg(x, b, max_iter_ratio * b.size());

    info.iter_count(iter);
}

void LinearFusedPCG::check_init_rz_nan_inf(Float rz)
{
    if(!std::isfinite(rz)) [[unlikely]]
    {
        auto norm_r = ctx().norm(r.cview());
        auto norm_z = ctx().norm(z.cview());
        bool r_bad  = !std::isfinite(norm_r);
        auto hint = r_bad ? "gradient assembling produced NaN values, likely due to error in formula implementation" :
                            "preconditioner failed, likely due to inverse matrix calculation failure";
        UIPC_ASSERT(false,
                    "Frame {}, Newton {}, FusedPCG Init: r^T*z = {}, norm(r) = {}, norm(z) = {}. "
                    "Hint: {}.",
                    engine().frame(),
                    engine().newton_iter(),
                    rz,
                    norm_r,
                    norm_z,
                    hint);
    }
}

void LinearFusedPCG::check_iter_rz_nan_inf(Float rz, SizeT k)
{
    if(!std::isfinite(rz)) [[unlikely]]
    {
        auto norm_r = ctx().norm(r.cview());
        auto norm_z = ctx().norm(z.cview());
        bool r_ok   = std::isfinite(norm_r);
        bool z_bad  = !std::isfinite(norm_z);
        auto hint   = (r_ok && z_bad) ?
                          "preconditioner failed, likely due to inverse matrix calculation failure" :
                          "PCG iteration diverged";
        UIPC_ASSERT(false,
                    "Frame {}, Newton {}, FusedPCG Iter {}: r^T*z = {}, norm(r) = {}, norm(z) = {}. "
                    "Hint: {}.",
                    engine().frame(),
                    engine().newton_iter(),
                    k,
                    rz,
                    norm_r,
                    norm_z,
                    hint);
    }
}

// d_result = x^T * y  (device-only CUB warp reduction)
void fused_dot(cuda_tool::CDenseVectorView<Float> x,
               cuda_tool::CDenseVectorView<Float> y,
               cuda_tool::VarView<Float>          d_result,
               cudaStream_t                       stream = nullptr)
{
    cudaMemsetAsync(d_result.data(), 0, sizeof(Float), stream);

    constexpr int block_dim   = 256;
    int           n           = x.size();
    int           block_count = (n + block_dim - 1) / block_dim;

    if(block_count > 0)
    {
        fused_dot_kernel<<<block_count, block_dim, 0, stream>>>(
            x.cviewer(), y.cviewer(), d_result.viewer(), n);
    }
}

// Graph-only accumulation variant. The ping-pong residual-dot destination is
// cleared by the preceding direction kernel (or by the once-per-replay prefix).
void fused_dot_accumulate(cuda_tool::CDenseVectorView<Float> x,
                          cuda_tool::CDenseVectorView<Float> y,
                          cuda_tool::VarView<Float>          d_result,
                          cudaStream_t                       stream)
{
    constexpr int block_dim   = 256;
    int           n           = x.size();
    int           block_count = (n + block_dim - 1) / block_dim;

    if(block_count > 0)
    {
        fused_dot_kernel<<<block_count, block_dim, 0, stream>>>(
            x.cviewer(), y.cviewer(), d_result.viewer(), n);
    }
}

void launch_fused_pcg_publish_update_direction(cuda_tool::DenseVectorView<Float> p,
                                               cuda_tool::CDenseVectorView<Float> z,
                                               cuda_tool::VarView<Float> rz,
                                               cuda_tool::CVarView<Float> rz_accum,
                                               cuda_tool::VarView<Float> published_rz_new,
                                               cuda_tool::VarView<Float> next_rz_accum,
                                               cuda_tool::VarView<Float> beta,
                                               cuda_tool::VarView<IndexT> converged,
                                               cuda_tool::CVarView<Float> rz_tol,
                                               cudaStream_t stream,
                                               cuda_tool::VarView<IndexT> iteration_count,
                                               IndexT iteration_budget)
{
    const int n = (int)p.size();
    const int grid_size =
        std::max(1, (n + FusedPcgDirectionBlockSize - 1) / FusedPcgDirectionBlockSize);
    fused_pcg_publish_update_direction_kernel<<<grid_size, FusedPcgDirectionBlockSize, 0, stream>>>(
        p,
        z,
        rz.viewer(),
        rz_accum.cviewer(),
        published_rz_new.viewer(),
        next_rz_accum.viewer(),
        beta.viewer(),
        converged.viewer(),
        rz_tol.cviewer(),
        iteration_count.viewer(),
        iteration_budget,
        n);
}

// Same as linear_pcg update_xr: alpha = rz/pAp, x += alpha*p, r -= alpha*Ap. Alpha computed on device from d_rz, d_pAp.
void fused_update_xr(cuda_tool::CVarView<Float>         d_rz,
                     cuda_tool::CVarView<Float>         d_pAp,
                     cuda_tool::CVarView<IndexT>        d_converged,
                     cuda_tool::DenseVectorView<Float>  x,
                     cuda_tool::CDenseVectorView<Float> p,
                     cuda_tool::DenseVectorView<Float>  r,
                     cuda_tool::CDenseVectorView<Float> Ap,
                     cudaStream_t                       stream = nullptr)
{
    int n = r.size();
    if(n > 0)
    {
        fused_update_xr_kernel<<<cuda_tool::best_grid_dim(n, fused_update_xr_kernel), cuda_tool::best_block_dim(fused_update_xr_kernel), 0, stream>>>(
            d_rz.cviewer(),
            d_pAp.cviewer(),
            d_converged.cviewer(),
            x.viewer(),
            p.cviewer(),
            r.viewer(),
            Ap.cviewer(),
            n);
    }
}

// Same as linear_pcg update_p: beta = rz_new/rz, p = z + beta*p.
// Convergence is guarded by d_converged.
void fused_update_p(cuda_tool::CVarView<Float>         d_rz_new,
                    cuda_tool::CVarView<Float>         d_rz,
                    cuda_tool::CVarView<IndexT>        d_converged,
                    cuda_tool::DenseVectorView<Float>  p,
                    cuda_tool::CDenseVectorView<Float> z,
                    cudaStream_t                       stream = nullptr)
{
    int n = p.size();
    if(n > 0)
    {
        fused_update_p_kernel<<<cuda_tool::best_grid_dim(n, fused_update_p_kernel), cuda_tool::best_block_dim(fused_update_p_kernel), 0, stream>>>(
            d_rz_new.cviewer(), d_rz.cviewer(), d_converged.cviewer(), p.viewer(), z.cviewer(), n);
    }
}

// d_rz = d_rz_new when not converged (single-thread write).
void fused_swap_rz(cuda_tool::CVarView<Float>  d_rz_new,
                   cuda_tool::VarView<Float>   d_rz,
                   cuda_tool::CVarView<IndexT> d_converged,
                   cudaStream_t                stream = nullptr)
{
    // single-thread kernel (muda Launch() parity: 1 block x 1 thread)
    fused_swap_rz_kernel<<<1, 1, 0, stream>>>(
        d_rz_new.cviewer(), d_rz.viewer(), d_converged.cviewer());
}

// d_converged = |rz_new| <= rz_tol (single-thread write); rz_tol lives on
// device so a captured CUDA graph survives tolerance changes between solves.
void fused_update_converged(cuda_tool::CVarView<Float> d_rz_new,
                            cuda_tool::VarView<IndexT> d_converged,
                            cuda_tool::CVarView<Float> d_rz_tol,
                            cudaStream_t               stream = nullptr)
{
    int n = 1;
    fused_update_converged_kernel<<<cuda_tool::best_grid_dim(n, fused_update_converged_kernel), cuda_tool::best_block_dim(fused_update_converged_kernel), 0, stream>>>(
        d_rz_new.cviewer(), d_converged.viewer(), d_rz_tol.cviewer(), n);
}

// One reference PCG iteration on `stream`, used by the uncaptured fallback.
// The Graph path below implements the same recurrence with its specialized
// pipelined and fused kernels.
// `timed` adds the per-iteration "SpMV"/"Apply Preconditioner" Timers —
// plain path only; during graph capture no Timer objects may be created
// (empirically corrupts state in the single-process test suite binary).
void LinearFusedPCG::run_iteration(cuda_tool::DenseVectorView<Float> x, cudaStream_t stream, bool timed)
{
    // Ap = A * p,  pAp = p^T * Ap
    {
        std::optional<Timer> timer;
        if(timed)
            timer.emplace("SpMV");
        spmv_dot(p.cview(), Ap.view(), d_pAp.view(), stream);
    }

    // alpha = rz / pAp,  x += alpha * p,  r -= alpha * Ap
    fused_update_xr(
        d_rz.view(), d_pAp.view(), d_converged.view(), x, p.cview(), r.view(), Ap.cview(), stream);

    // z = P^{-1} * r
    {
        std::optional<Timer> timer;
        if(timed)
            timer.emplace("Apply Preconditioner");
        apply_preconditioner(z, r, d_converged.view(), stream);
    }

    // rz_new = r^T * z, keep convergence flag on device for preconditioner skip.
    fused_dot(r.cview(), z.cview(), d_rz_new.view(), stream);
    fused_update_converged(d_rz_new.view(), d_converged.view(), d_rz_tol.view(), stream);

    // p = z + beta * p (skip when abs(rz_new) <= rz_tol), then rz = rz_new.
    fused_update_p(d_rz_new.view(), d_rz.view(), d_converged.view(), p.view(), z.cview(), stream);
    fused_swap_rz(d_rz_new.view(), d_rz.view(), d_converged.view(), stream);
}

void LinearFusedPCG::run_graph_iteration(cuda_tool::DenseVectorView<Float> x,
                                         SizeT                             slot,
                                         cudaStream_t stream,
                                         cuda_tool::VarView<IndexT> iteration_count,
                                         IndexT iteration_budget)
{
    const SizeT next_slot = slot ^ SizeT { 1 };

    // Ap = A * p and pAp = p^T * Ap. The same pipelined kernel clears the
    // opposite slot for its next use and skips the full sparse traversal once
    // an earlier iteration in this replay block converges.
    spmv_dot_pipelined(p.cview(),
                       m_graph_Ap[slot].view(),
                       m_graph_pAp[slot].view(),
                       m_graph_Ap[next_slot].view(),
                       m_graph_pAp[next_slot].view(),
                       d_converged.view(),
                       stream);

    const bool fused = fused_pcg_update_apply_dot(x,
                                                  p.cview(),
                                                  r.view(),
                                                  m_graph_Ap[slot].cview(),
                                                  z.view(),
                                                  d_rz.view(),
                                                  m_graph_pAp[slot].view(),
                                                  m_graph_rz_accum[slot].view(),
                                                  d_converged.view(),
                                                  stream);
    if(!fused)
    {
        fused_update_xr(d_rz.view(),
                        m_graph_pAp[slot].view(),
                        d_converged.view(),
                        x,
                        p.cview(),
                        r.view(),
                        m_graph_Ap[slot].cview(),
                        stream);
        apply_preconditioner(z, r, d_converged.view(), stream);
        fused_dot_accumulate(r.cview(), z.cview(), m_graph_rz_accum[slot].view(), stream);
    }
    launch_fused_pcg_publish_update_direction(p.view(),
                                              z.cview(),
                                              d_rz.view(),
                                              m_graph_rz_accum[slot].view(),
                                              d_rz_new.view(),
                                              m_graph_rz_accum[next_slot].view(),
                                              m_graph_beta.view(),
                                              d_converged.view(),
                                              d_rz_tol.view(),
                                              stream,
                                              iteration_count,
                                              iteration_budget);
}

// ---------------------------------------------------------------------------
// CUDA graph block replay
// ---------------------------------------------------------------------------

void LinearFusedPCG::destroy_graph()
{
    m_graph.reset_graph();
    m_graph_n                 = 0;
    m_graph_layout_generation = 0;
}

#if CUDA_TOOL_GRAPH_WHILE
void LinearFusedPCG::destroy_while()
{
    m_while.reset_graph();
    m_while_n                 = 0;
    m_while_layout_generation = 0;
}

bool LinearFusedPCG::while_key_matches(cuda_tool::DenseVectorView<Float>  x,
                                       cuda_tool::CDenseVectorView<Float> b,
                                       SizeT max_iter) const
{
    if(!m_while.ready())
        return false;
    auto                        A    = matrix_data_ptrs();
    std::array<const void*, 20> ptrs = {x.data(),
                                        b.data(),
                                        r.buffer_view().data(),
                                        z.buffer_view().data(),
                                        p.buffer_view().data(),
                                        A[0],
                                        A[1],
                                        A[2],
                                        d_rz.data(),
                                        d_rz_new.data(),
                                        d_rz_tol.data(),
                                        d_converged.data(),
                                        d_iter.data(),
                                        m_graph_Ap[0].buffer_view().data(),
                                        m_graph_Ap[1].buffer_view().data(),
                                        m_graph_pAp[0].data(),
                                        m_graph_pAp[1].data(),
                                        m_graph_rz_accum[0].data(),
                                        m_graph_rz_accum[1].data(),
                                        m_graph_beta.data()};
    return m_while_n == x.size() && m_while_max_iter == max_iter
           && m_while_layout_generation == linear_system_layout_generation()
           && m_while_ptrs == ptrs;
}

void LinearFusedPCG::rebuild_while(cuda_tool::DenseVectorView<Float>  x,
                                   cuda_tool::CDenseVectorView<Float> b,
                                   SizeT                              max_iter)
{
    destroy_while();
    const IndexT iteration_budget = max_iter > 0 ? static_cast<IndexT>(max_iter - 1) : 0;

    auto result = m_while.capture(
        // setup chain: reset -> r=b -> precond -> p=z -> rz = r^T z -> rz_tol
        [&](cudaStream_t stream, cudaGraphConditionalHandle handle)
        {
            pcg_while_reset_kernel<<<1, 1, 0, stream>>>(d_converged.viewer(),
                                                        d_iter.viewer());
            cuda_tool::BufferLaunch(stream).copy(r.buffer_view(), b.buffer_view());
            apply_preconditioner(z, r, d_converged.view(), stream);
            cuda_tool::BufferLaunch(stream).copy(p.buffer_view(), z.buffer_view());
            fused_dot(r.cview(), z.cview(), d_rz.view(), stream);
            // Each solve starts at slot 0. The two-slot body preserves parity
            // across WHILE replays, so no clear nodes are needed inside it.
            cuda_tool::BufferLaunch(stream).fill<Float>(m_graph_Ap[0].buffer_view(), 0);
            CUDA_TOOL_CHECK(cudaMemsetAsync(m_graph_pAp[0].data(), 0, sizeof(Float), stream));
            CUDA_TOOL_CHECK(
                cudaMemsetAsync(m_graph_rz_accum[0].data(), 0, sizeof(Float), stream));
            pcg_while_setup_kernel<<<1, 1, 0, stream>>>(d_rz.cviewer(),
                                                        d_rz_tol.viewer(),
                                                        d_converged.viewer(),
                                                        global_tol_rate,
                                                        iteration_budget,
                                                        handle);
        },
        // Both slots use the optimized iteration. A stop in slot 0 also masks
        // slot 1, including its iteration counter, before WHILE re-evaluation.
        [&](cudaStream_t stream, cudaGraphConditionalHandle handle)
        {
            run_graph_iteration(x, 0, stream, d_iter.view(), iteration_budget);
            run_graph_iteration(x, 1, stream, d_iter.view(), iteration_budget);
            pcg_while_control_kernel<<<1, 1, 0, stream>>>(handle, d_converged.cviewer());
        });

    if(result != cuda_tool::GraphWhile::Result::Ok)
    {
        logger::warn(
            "LinearFusedPCG: while-loop graph capture failed (code {}: {}); "
            "falling back to block replay / plain launches",
            (int)result,
            m_while.failure_detail());
        return;
    }

    auto A                    = matrix_data_ptrs();
    m_while_ptrs              = {x.data(),
                                 b.data(),
                                 r.buffer_view().data(),
                                 z.buffer_view().data(),
                                 p.buffer_view().data(),
                                 A[0],
                                 A[1],
                                 A[2],
                                 d_rz.data(),
                                 d_rz_new.data(),
                                 d_rz_tol.data(),
                                 d_converged.data(),
                                 d_iter.data(),
                                 m_graph_Ap[0].buffer_view().data(),
                                 m_graph_Ap[1].buffer_view().data(),
                                 m_graph_pAp[0].data(),
                                 m_graph_pAp[1].data(),
                                 m_graph_rz_accum[0].data(),
                                 m_graph_rz_accum[1].data(),
                                 m_graph_beta.data()};
    m_while_n                 = x.size();
    m_while_max_iter          = max_iter;
    m_while_layout_generation = linear_system_layout_generation();
    logger::info("LinearFusedPCG: captured fused full-GPU while-loop graph (n = {}, slots = 2)",
                 x.size());
}
#endif

bool LinearFusedPCG::graph_key_matches(cuda_tool::DenseVectorView<Float>  x,
                                       cuda_tool::CDenseVectorView<Float> b,
                                       SizeT interval,
                                       SizeT max_iter) const
{
    if(!m_graph.ready())
        return false;
    auto                        A    = matrix_data_ptrs();
    std::array<const void*, 19> ptrs = {x.data(),
                                        b.data(),
                                        r.buffer_view().data(),
                                        z.buffer_view().data(),
                                        p.buffer_view().data(),
                                        Ap.buffer_view().data(),
                                        m_graph_Ap[0].buffer_view().data(),
                                        m_graph_Ap[1].buffer_view().data(),
                                        A[0],
                                        A[1],
                                        A[2],
                                        d_rz.data(),
                                        d_rz_new.data(),
                                        d_pAp.data(),
                                        m_graph_pAp[0].data(),
                                        m_graph_pAp[1].data(),
                                        m_graph_rz_accum[0].data(),
                                        m_graph_rz_accum[1].data(),
                                        m_graph_beta.data()};
    return m_graph_n == x.size() && m_graph_interval == interval
           && m_graph_max_iter == max_iter
           && m_graph_layout_generation == linear_system_layout_generation()
           && m_graph_ptrs == ptrs;
}

void LinearFusedPCG::rebuild_graph(cuda_tool::DenseVectorView<Float>  x,
                                   cuda_tool::CDenseVectorView<Float> b,
                                   SizeT                              interval,
                                   SizeT                              max_iter)
{
    destroy_graph();

    // recorded, not executed; the block is launched for real right after
    auto result = m_graph.capture(
        [&](cudaStream_t capture_stream)
        {
            // Every replay starts at slot 0. This prefix keeps odd intervals
            // correct; for an even configured interval it is a cheap
            // once-per-block redundant clear, not a per-iteration node.
            cuda_tool::BufferLaunch(capture_stream).fill<Float>(m_graph_Ap[0].buffer_view(), 0);
            CUDA_TOOL_CHECK(cudaMemsetAsync(m_graph_pAp[0].data(), 0, sizeof(Float), capture_stream));
            CUDA_TOOL_CHECK(cudaMemsetAsync(
                m_graph_rz_accum[0].data(), 0, sizeof(Float), capture_stream));
            for(SizeT i = 0; i < interval; ++i)
                run_graph_iteration(x, i & SizeT{1}, capture_stream);
        });

    if(result != cuda_tool::GraphCapture::Result::Ok)
    {
        // a callee launched outside the capture stream (e.g. the MAS
        // preconditioner engine) or the runtime rejected the capture
        logger::warn(
            "LinearFusedPCG: CUDA graph capture failed (code {}); "
            "graph replay disabled for this instance",
            (int)result);
        return;
    }

    logger::info("LinearFusedPCG: captured CUDA graph (interval = {}, n = {})",
                 interval,
                 x.size());

    auto A                    = matrix_data_ptrs();
    m_graph_ptrs              = {x.data(),
                                 b.data(),
                                 r.buffer_view().data(),
                                 z.buffer_view().data(),
                                 p.buffer_view().data(),
                                 Ap.buffer_view().data(),
                                 m_graph_Ap[0].buffer_view().data(),
                                 m_graph_Ap[1].buffer_view().data(),
                                 A[0],
                                 A[1],
                                 A[2],
                                 d_rz.data(),
                                 d_rz_new.data(),
                                 d_pAp.data(),
                                 m_graph_pAp[0].data(),
                                 m_graph_pAp[1].data(),
                                 m_graph_rz_accum[0].data(),
                                 m_graph_rz_accum[1].data(),
                                 m_graph_beta.data()};
    m_graph_n                 = x.size();
    m_graph_interval          = interval;
    m_graph_max_iter          = max_iter;
    m_graph_layout_generation = linear_system_layout_generation();
}

SizeT LinearFusedPCG::fused_pcg(cuda_tool::DenseVectorView<Float>  x,
                                cuda_tool::CDenseVectorView<Float> b,
                                SizeT                              max_iter)
{
    Timer pcg_timer{"FusedPCG"};

#if CUDA_TOOL_GRAPH_WHILE
    if(m_graph_mode == 2)
    {
        if(!while_key_matches(x, b, max_iter))
            rebuild_while(x, b, max_iter);

        if(m_while.ready())
        {
            // one launch for the whole solve; zero D2H/H2D inside the loop
            CUDA_TOOL_CHECK(m_while.launch_sync());
            IndexT iters = d_iter;
            IndexT converged = d_converged;
            return converged == FusedPcgConverged ? (SizeT)iters : max_iter;
        }
        // capture failed: fall through to block replay / plain launches
    }
#endif

    d_converged = 0;

    // r = b - A*x, but x0 = 0 so r = b
    r.buffer_view().copy_from(b.buffer_view());

    // z = P^{-1} * r
    {
        Timer timer{"Apply Preconditioner"};
        apply_preconditioner(z, r, d_converged.view());
    }

    // p = z
    p = z;

    // rz = r^T * z
    fused_dot(r.cview(), z.cview(), d_rz.view());
    Float rz_host = d_rz;
    check_init_rz_nan_inf(rz_host);
    Float abs_rz0 = std::abs(rz_host);

    if(abs_rz0 == Float{0.0})
        return 0;

    Float rz_tol = global_tol_rate * abs_rz0;
    // synchronous upload: an async copy on the default stream would race with
    // the graph launch stream (blocking streams do not wait for
    // legacy-stream work), letting the converged kernel read a stale/uninit
    // tolerance. (Symptom was dx=0 -> flat line-search energy.)
    CUDA_TOOL_CHECK(cudaMemcpy(d_rz_tol.data(), &rz_tol, sizeof(Float), cudaMemcpyHostToDevice));
    SizeT effective_check_interval = check_interval > 0 ? check_interval : SizeT{1};

    SizeT total_iters = max_iter > 0 ? max_iter - 1 : 0;
    SizeT iter_done   = 0;
    bool  converged   = false;

    while(true)
    {
        SizeT block = std::min(effective_check_interval, total_iters - iter_done);

        bool graph_block = m_graph_mode != 0 && !m_graph.disabled()
                           && block == effective_check_interval;
        if(graph_block)
        {
            if(!graph_key_matches(x, b, effective_check_interval, max_iter))
                rebuild_graph(x, b, effective_check_interval, max_iter);

            if(m_graph.ready())
            {
                m_graph.launch_sync();  // replay, then host-check below
            }
            else  // capture failed: plain path
            {
                for(SizeT i = 0; i < block; ++i)
                    run_iteration(x, nullptr, true);
            }
        }
        else  // tail block / graph disabled
        {
            for(SizeT i = 0; i < block; ++i)
                run_iteration(x, nullptr, true);
        }
        iter_done += block;

        // host convergence check, same cadence as the plain loop
        Float rz_new_host = d_rz_new;
        check_iter_rz_nan_inf(rz_new_host, iter_done);
        if((std::abs(rz_new_host) / abs_rz0) <= global_tol_rate)
        {
            converged = true;
            break;
        }
        if(iter_done >= total_iters)
            break;
    }

    return converged ? iter_done : max_iter;
}
}  // namespace uipc::backend::cuda
