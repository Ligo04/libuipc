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

#include <app/app.h>
#include <linear_system/spmv.h>

#include <array>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

TEST_CASE("fused PCG pipelined SpMV preserves result and clears the next slot",
          "[build_solve_focused][fused_pcg][spmv_pipeline]")
{
    constexpr int BlockRows = 2;
    constexpr int Dofs      = BlockRows * 3;

    const std::array<int, 3> rows = {0, 0, 1};
    const std::array<int, 3> cols = {0, 1, 1};
    std::array<Matrix3x3, 3> values;
    values[0] = Float{2} * Matrix3x3::Identity();
    values[1] = Matrix3x3::Identity();
    values[2] = Float{3} * Matrix3x3::Identity();

    cuda_tool::DeviceBCOOMatrix<Float, 3> A;
    A.resize(BlockRows, BlockRows, rows.size());
    A.row_indices().copy_from(rows.data());
    A.col_indices().copy_from(cols.data());
    A.values().copy_from(values.data());

    const std::array<Float, Dofs>       x_host = {1, 2, 3, 4, 5, 6};
    cuda_tool::DeviceDenseVector<Float> x;
    cuda_tool::DeviceDenseVector<Float> y;
    cuda_tool::DeviceDenseVector<Float> next_y;
    x.resize(Dofs);
    y.resize(Dofs);
    next_y.resize(Dofs);
    x.buffer_view().copy_from(x_host.data());
    y.buffer_view().fill(Float{0});
    next_y.buffer_view().fill(Float{7});

    cuda_tool::DeviceVar<Float>  dot;
    cuda_tool::DeviceVar<Float>  next_dot;
    cuda_tool::DeviceVar<IndexT> converged;
    cuda_tool::DeviceVar<IndexT> triplet_count;
    dot           = Float{0};
    next_dot      = Float{11};
    converged     = IndexT{0};
    triplet_count = static_cast<IndexT>(rows.size());

    Spmv().rbk_sym_spmv_dot_pipelined(Float{1},
                                      A.cview(),
                                      x.cview(),
                                      y.view(),
                                      dot.view(),
                                      next_y.view(),
                                      next_dot.view(),
                                      converged.view(),
                                      triplet_count.cviewer(),
                                      A.triplet_capacity(),
                                      nullptr);

    const std::array<Float, Dofs> expected_y = {6, 9, 12, 13, 17, 21};
    std::array<Float, Dofs>       actual_y{};
    std::array<Float, Dofs>       actual_next_y{};
    y.buffer_view().copy_to(actual_y.data());
    next_y.buffer_view().copy_to(actual_next_y.data());
    for(int i = 0; i < Dofs; ++i)
    {
        REQUIRE(actual_y[i] == Catch::Approx(expected_y[i]).margin(1e-12));
        REQUIRE(actual_next_y[i] == Float{0});
    }
    REQUIRE(static_cast<Float>(dot) == Catch::Approx(323.0).margin(1e-12));
    REQUIRE(static_cast<Float>(next_dot) == Float{0});

    // Once converged, neither the output nor the next-slot clear is allowed
    // to run. This is the skip used by later inactive iterations in a replay.
    y.buffer_view().fill(Float{13});
    next_y.buffer_view().fill(Float{17});
    dot       = Float{19};
    next_dot  = Float{23};
    converged = IndexT{1};

    Spmv().rbk_sym_spmv_dot_pipelined(Float{1},
                                      A.cview(),
                                      x.cview(),
                                      y.view(),
                                      dot.view(),
                                      next_y.view(),
                                      next_dot.view(),
                                      converged.view(),
                                      triplet_count.cviewer(),
                                      A.triplet_capacity(),
                                      nullptr);

    y.buffer_view().copy_to(actual_y.data());
    next_y.buffer_view().copy_to(actual_next_y.data());
    for(int i = 0; i < Dofs; ++i)
    {
        REQUIRE(actual_y[i] == Float{13});
        REQUIRE(actual_next_y[i] == Float{17});
    }
    REQUIRE(static_cast<Float>(dot) == Float{19});
    REQUIRE(static_cast<Float>(next_dot) == Float{23});
}

TEST_CASE("fused PCG pipelined SpMV accumulates across multiple CTA32 blocks",
          "[build_solve_focused][fused_pcg][spmv_pipeline]")
{
    constexpr int TripletCount = 65;
    constexpr int Dofs         = 3;

    std::array<int, TripletCount>       rows{};
    std::array<int, TripletCount>       cols{};
    std::array<Matrix3x3, TripletCount> values;
    for(auto& value : values)
        value = Matrix3x3::Identity();

    cuda_tool::DeviceBCOOMatrix<Float, 3> A;
    A.resize(1, 1, TripletCount);
    A.row_indices().copy_from(rows.data());
    A.col_indices().copy_from(cols.data());
    A.values().copy_from(values.data());

    const std::array<Float, Dofs>       x_host = {1, 2, 3};
    cuda_tool::DeviceDenseVector<Float> x;
    cuda_tool::DeviceDenseVector<Float> y;
    cuda_tool::DeviceDenseVector<Float> next_y;
    x.resize(Dofs);
    y.resize(Dofs);
    next_y.resize(Dofs);
    x.buffer_view().copy_from(x_host.data());
    y.buffer_view().fill(Float{0});
    next_y.buffer_view().fill(Float{7});

    cuda_tool::DeviceVar<Float>  dot;
    cuda_tool::DeviceVar<Float>  next_dot;
    cuda_tool::DeviceVar<IndexT> converged;
    cuda_tool::DeviceVar<IndexT> triplet_count;
    dot           = Float{0};
    next_dot      = Float{11};
    converged     = IndexT{0};
    triplet_count = TripletCount;

    Spmv().rbk_sym_spmv_dot_pipelined(Float{1},
                                      A.cview(),
                                      x.cview(),
                                      y.view(),
                                      dot.view(),
                                      next_y.view(),
                                      next_dot.view(),
                                      converged.view(),
                                      triplet_count.cviewer(),
                                      A.triplet_capacity(),
                                      nullptr);

    const std::array<Float, Dofs> expected_y = {65, 130, 195};
    std::array<Float, Dofs>       actual_y{};
    std::array<Float, Dofs>       actual_next_y{};
    y.buffer_view().copy_to(actual_y.data());
    next_y.buffer_view().copy_to(actual_next_y.data());
    for(int i = 0; i < Dofs; ++i)
    {
        REQUIRE(actual_y[i] == Catch::Approx(expected_y[i]).margin(1e-12));
        REQUIRE(actual_next_y[i] == Float{0});
    }
    REQUIRE(static_cast<Float>(dot) == Catch::Approx(910.0).margin(1e-12));
    REQUIRE(static_cast<Float>(next_dot) == Float{0});
}

TEST_CASE("fused PCG pipelined SpMV honors active count and clears a zero-count ping-pong slot",
          "[build_solve_focused][fused_pcg][spmv_pipeline]")
{
    constexpr int BlockRows     = 2;
    constexpr int Dofs          = BlockRows * 3;
    constexpr int ActiveCount   = 3;
    constexpr int ReservedCount = 67;

    std::array<int, ReservedCount>       rows{};
    std::array<int, ReservedCount>       cols{};
    std::array<Matrix3x3, ReservedCount> values;
    for(auto& value : values)
        value = Float{100} * Matrix3x3::Identity();

    rows[0]   = 0;
    cols[0]   = 0;
    values[0] = Float{2} * Matrix3x3::Identity();
    rows[1]   = 0;
    cols[1]   = 1;
    values[1] = Matrix3x3::Identity();
    rows[2]   = 1;
    cols[2]   = 1;
    values[2] = Float{3} * Matrix3x3::Identity();

    cuda_tool::DeviceBCOOMatrix<Float, 3> A;
    A.resize(BlockRows, BlockRows, ReservedCount);
    A.row_indices().copy_from(rows.data());
    A.col_indices().copy_from(cols.data());
    A.values().copy_from(values.data());

    const std::array<Float, Dofs>       x_host = {1, 2, 3, 4, 5, 6};
    cuda_tool::DeviceDenseVector<Float> x;
    cuda_tool::DeviceDenseVector<Float> y;
    cuda_tool::DeviceDenseVector<Float> next_y;
    x.resize(Dofs);
    y.resize(Dofs);
    next_y.resize(Dofs);
    x.buffer_view().copy_from(x_host.data());
    y.buffer_view().fill(Float{0});
    next_y.buffer_view().fill(Float{7});

    cuda_tool::DeviceVar<Float>  dot;
    cuda_tool::DeviceVar<Float>  next_dot;
    cuda_tool::DeviceVar<IndexT> converged;
    cuda_tool::DeviceVar<IndexT> triplet_count;
    dot           = Float{0};
    next_dot      = Float{11};
    converged     = IndexT{0};
    triplet_count = ActiveCount;

    Spmv().rbk_sym_spmv_dot_pipelined(Float{1},
                                      A.cview(),
                                      x.cview(),
                                      y.view(),
                                      dot.view(),
                                      next_y.view(),
                                      next_dot.view(),
                                      converged.view(),
                                      triplet_count.cviewer(),
                                      A.triplet_capacity(),
                                      nullptr);

    const std::array<Float, Dofs> expected_y = {6, 9, 12, 13, 17, 21};
    std::array<Float, Dofs>       actual_y{};
    std::array<Float, Dofs>       actual_next_y{};
    y.buffer_view().copy_to(actual_y.data());
    next_y.buffer_view().copy_to(actual_next_y.data());
    for(int i = 0; i < Dofs; ++i)
    {
        REQUIRE(actual_y[i] == Catch::Approx(expected_y[i]).margin(1e-12));
        REQUIRE(actual_next_y[i] == Float{0});
    }
    REQUIRE(static_cast<Float>(dot) == Catch::Approx(323.0).margin(1e-12));
    REQUIRE(static_cast<Float>(next_dot) == Float{0});

    // Mirror the Graph ping-pong: the just-cleared slot becomes the current
    // output, while the previous nonzero y/pAp slot must be cleared for reuse.
    triplet_count = IndexT{0};
    Spmv().rbk_sym_spmv_dot_pipelined(Float{1},
                                      A.cview(),
                                      x.cview(),
                                      next_y.view(),
                                      next_dot.view(),
                                      y.view(),
                                      dot.view(),
                                      converged.view(),
                                      triplet_count.cviewer(),
                                      A.triplet_capacity(),
                                      nullptr);

    y.buffer_view().copy_to(actual_y.data());
    next_y.buffer_view().copy_to(actual_next_y.data());
    for(int i = 0; i < Dofs; ++i)
    {
        REQUIRE(actual_y[i] == Float{0});
        REQUIRE(actual_next_y[i] == Float{0});
    }
    REQUIRE(static_cast<Float>(dot) == Float{0});
    REQUIRE(static_cast<Float>(next_dot) == Float{0});
}
