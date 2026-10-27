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

#include <type_define.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
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
                                               cuda_tool::VarView<IndexT> iteration_count = {},
                                               IndexT iteration_budget = 0);

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
                                           cudaStream_t stream);

void launch_fused_pcg_fem_update_apply_dot(cuda_tool::CBufferView<Matrix3x3> diag_inv,
                                           cuda_tool::DenseVectorView<Float>  x,
                                           cuda_tool::CDenseVectorView<Float> p,
                                           cuda_tool::DenseVectorView<Float>  r,
                                           cuda_tool::CDenseVectorView<Float> Ap,
                                           cuda_tool::DenseVectorView<Float> z,
                                           cuda_tool::CVarView<Float>        rz,
                                           cuda_tool::CVarView<Float> pAp,
                                           cuda_tool::VarView<Float>  rz_new,
                                           cuda_tool::CVarView<IndexT> converged,
                                           cudaStream_t stream);
}  // namespace uipc::backend::cuda
