/*
 * NVIDIA contributions: fused iteration, ping-pong buffer and CUDA Graph
 * cache state/interfaces.
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

#pragma once
#include <linear_system/iterative_solver.h>
#include <cuda_tool/cuda_tool.h>
#include <cuda_tool/graph.h>
#include <array>

namespace uipc::backend::cuda
{
// Fused PCG: keeps dot-product scalars (rz, pAp, rz_new) on device
// to eliminate per-iteration host synchronizations.  The update kernels read
// alpha = rz/pAp and beta = rz_new/rz directly from device memory.
// SpMV and dot(p,Ap) are fused into a single kernel pass.
// Convergence is checked every `check_interval` iterations via a single D2H copy.
//
// CUDA graph replay records a block of `check_interval` iterations once per
// (buffer-set, N) and replays it as one graph launch. The graph path implements
// the same PCG recurrence with pipelined SpMV clears, fused local-preconditioner
// update/apply/dot work, and fused direction publication; the uncaptured path
// keeps the reference kernel sequence. If capture fails (e.g. a preconditioner
// launches outside the capture stream), the solver permanently falls back to
// the plain loop for that instance.
class LinearFusedPCG : public IterativeSolver
{
  public:
    using IterativeSolver::IterativeSolver;

  protected:
    virtual void do_build(BuildInfo& info) override;
    virtual void do_solve(GlobalLinearSystem::SolvingInfo& info) override;

  private:
    using DeviceDenseVector = cuda_tool::DeviceDenseVector<Float>;

    SizeT fused_pcg(cuda_tool::DenseVectorView<Float>  x,
                    cuda_tool::CDenseVectorView<Float> b,
                    SizeT                              max_iter);
    void  check_init_rz_nan_inf(Float rz);
    void  check_iter_rz_nan_inf(Float rz, SizeT k);

    // One iteration of the PCG loop body on `stream` (the unit of graph
    // capture and of the uncaptured fallback path).
    void run_iteration(cuda_tool::DenseVectorView<Float> x, cudaStream_t stream, bool timed);
    void run_graph_iteration(cuda_tool::DenseVectorView<Float> x, SizeT slot, cudaStream_t stream);

    // Capture `interval` iterations into m_graph (no execution during
    // capture); on any failure disable graph replay for this instance.
    void rebuild_graph(cuda_tool::DenseVectorView<Float>  x,
                       cuda_tool::CDenseVectorView<Float> b,
                       SizeT                              interval,
                       SizeT                              max_iter);
    bool graph_key_matches(cuda_tool::DenseVectorView<Float>  x,
                           cuda_tool::CDenseVectorView<Float> b,
                           SizeT                              interval,
                           SizeT                              max_iter) const;
    void destroy_graph();

    DeviceDenseVector r;
    DeviceDenseVector z;
    DeviceDenseVector p;
    DeviceDenseVector Ap;

    // Block-replay only. The active SpMV clears the opposite slot in-kernel,
    // and the direction kernel clears the opposite residual-dot slot. This
    // removes all per-iteration clear nodes without changing the legacy plain
    // or conditional-graph paths.
    std::array<DeviceDenseVector, 2>           m_graph_Ap;
    std::array<cuda_tool::DeviceVar<Float>, 2> m_graph_pAp;
    std::array<cuda_tool::DeviceVar<Float>, 2> m_graph_rz_accum;
    cuda_tool::DeviceVar<Float>                m_graph_beta;

    cuda_tool::DeviceVar<Float>  d_rz;
    cuda_tool::DeviceVar<Float>  d_pAp;
    cuda_tool::DeviceVar<Float>  d_rz_new;
    cuda_tool::DeviceVar<IndexT> d_converged;
    // rz_tol on device so a captured graph survives rz_tol changes
    cuda_tool::DeviceVar<Float> d_rz_tol;

    Float max_iter_ratio  = 2.0;
    Float global_tol_rate = 1e-4;
    Float reserve_ratio   = 1.5;
    SizeT check_interval  = 5;

    // --- CUDA graph state ---
    IndexT m_use_cuda_graph = 1;  // config: linear_system/use_cuda_graph
    // 0 = plain loop, 1 = block replay, 2 = full-GPU while-loop graph
    IndexT                  m_graph_mode = 0;
    cuda_tool::GraphCapture m_graph;
    // validity key: every device pointer baked into the captured kernels
    std::array<const void*, 19> m_graph_ptrs{};
    SizeT                       m_graph_n                 = 0;
    SizeT                       m_graph_interval          = 0;
    SizeT                       m_graph_max_iter          = 0;
    SizeT                       m_graph_layout_generation = 0;

    // --- full-GPU while-loop graph (CUDA >= 12.4) ---
    cuda_tool::GraphWhile        m_while;
    cuda_tool::DeviceVar<IndexT> d_iter;
    std::array<const void*, 12>  m_while_ptrs{};
    SizeT                        m_while_n                 = 0;
    SizeT                        m_while_max_iter          = 0;
    SizeT                        m_while_layout_generation = 0;
    bool while_key_matches(cuda_tool::DenseVectorView<Float>  x,
                           cuda_tool::CDenseVectorView<Float> b,
                           SizeT                              max_iter) const;
    void rebuild_while(cuda_tool::DenseVectorView<Float>  x,
                       cuda_tool::CDenseVectorView<Float> b,
                       SizeT                              max_iter);
    void destroy_while();
};
}  // namespace uipc::backend::cuda
