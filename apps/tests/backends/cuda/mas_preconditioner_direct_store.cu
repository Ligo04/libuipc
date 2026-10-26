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
#include <finite_element/mas_preconditioner_engine.h>

#include <array>
#include <numeric>
#include <vector>

namespace cuda_tool = uipc::backend::cuda_tool;
using namespace uipc;
using namespace uipc::backend::cuda;

namespace
{
constexpr int VertexCount = MASPreconditionerEngine::BANKSIZE;
constexpr int ScalarCount = VertexCount * 3;

std::vector<Float> expected_diagonal_apply(const std::vector<Float>& r)
{
    std::vector<Float> expected(ScalarCount);
    for(int vertex = 0; vertex < VertexCount; ++vertex)
    {
        for(int component = 0; component < 3; ++component)
        {
            const Float diagonal = Float{2 + component} + Float{0.125} * vertex;
            expected[3 * vertex + component] = r[3 * vertex + component] / diagonal;
        }
    }
    return expected;
}

void require_near(const std::vector<Float>& actual,
                  const std::vector<Float>& expected,
                  Float                     tolerance = 3e-5)
{
    REQUIRE(actual.size() == expected.size());
    for(SizeT i = 0; i < actual.size(); ++i)
        REQUIRE(actual[i] == Catch::Approx(expected[i]).margin(tolerance));
}
}  // namespace

TEST_CASE("MAS local solve direct stores replace cleared atomic accumulation",
          "[build_solve_focused][mas][direct_store]")
{
    std::vector<unsigned int> neighbor_starts(VertexCount, 0);
    std::vector<unsigned int> neighbor_counts(VertexCount, 0);
    std::vector<unsigned int> neighbor_list;
    std::vector<int>          part_to_real(VertexCount);
    std::vector<int>          real_to_part(VertexCount);
    std::iota(part_to_real.begin(), part_to_real.end(), 0);
    std::iota(real_to_part.begin(), real_to_part.end(), 0);

    MASPreconditionerEngine engine;
    engine.init_neighbor(VertexCount,
                         0,
                         VertexCount,
                         span<const unsigned int>{neighbor_list},
                         span<const unsigned int>{neighbor_starts},
                         span<const unsigned int>{neighbor_counts},
                         span<const int>{part_to_real},
                         span<const int>{real_to_part});
    engine.init_matrix();
    engine.set_active_level_num(1);

    std::array<Matrix3x3, VertexCount> values;
    std::array<int, VertexCount>       rows;
    std::array<int, VertexCount>       cols;
    for(int vertex = 0; vertex < VertexCount; ++vertex)
    {
        values[vertex].setZero();
        for(int component = 0; component < 3; ++component)
            values[vertex](component, component) =
                Float{2 + component} + Float{0.125} * vertex;
        rows[vertex] = vertex;
        cols[vertex] = vertex;
    }

    cuda_tool::DeviceBuffer<Matrix3x3> d_values(VertexCount);
    cuda_tool::DeviceBuffer<int>       d_rows(VertexCount);
    cuda_tool::DeviceBuffer<int>       d_cols(VertexCount);
    d_values.view().copy_from(values.data());
    d_rows.view().copy_from(rows.data());
    d_cols.view().copy_from(cols.data());
    engine.set_preconditioner(d_values.cview(), d_rows.cview(), d_cols.cview(), 0);

    cuda_tool::DeviceDenseVector<Float> d_r;
    cuda_tool::DeviceDenseVector<Float> d_z;
    cuda_tool::DeviceVar<IndexT>        d_converged{IndexT{0}};
    d_r.resize(ScalarCount);
    d_z.resize(ScalarCount);

    std::vector<Float> actual(ScalarCount);
    for(int pass = 0; pass < 2; ++pass)
    {
        std::vector<Float> residual(ScalarCount);
        for(int i = 0; i < ScalarCount; ++i)
            residual[i] = Float{0.25 * (pass + 1)} + Float{0.03125} * (i + 1);

        d_r.buffer_view().copy_from(residual.data());
        d_z.buffer_view().fill(Float{-100 - pass});
        engine.apply(d_r.cview(), d_z.view(), d_converged.view(), nullptr);
        d_z.copy_to(actual);
        require_near(actual, expected_diagonal_apply(residual));
    }

    SECTION("converged apply leaves z untouched")
    {
        d_z.buffer_view().fill(Float{17});
        d_converged = IndexT{1};
        engine.apply(d_r.cview(), d_z.view(), d_converged.view(), nullptr);
        d_z.copy_to(actual);
        require_near(actual, std::vector<Float>(ScalarCount, Float{17}), Float{0});
    }
}

TEST_CASE("MAS fused PCG update apply dot matches the reference arithmetic",
          "[build_solve_focused][mas][fused_pcg]")
{
    std::vector<unsigned int> neighbor_starts(VertexCount, 0);
    std::vector<unsigned int> neighbor_counts(VertexCount, 0);
    std::vector<unsigned int> neighbor_list;
    std::vector<int>          part_to_real(VertexCount);
    std::vector<int>          real_to_part(VertexCount);
    std::iota(part_to_real.begin(), part_to_real.end(), 0);
    std::iota(real_to_part.begin(), real_to_part.end(), 0);

    MASPreconditionerEngine engine;
    engine.init_neighbor(VertexCount,
                         0,
                         VertexCount,
                         span<const unsigned int>{neighbor_list},
                         span<const unsigned int>{neighbor_starts},
                         span<const unsigned int>{neighbor_counts},
                         span<const int>{part_to_real},
                         span<const int>{real_to_part});
    engine.init_matrix();
    engine.set_active_level_num(1);

    std::array<Matrix3x3, VertexCount> values;
    std::array<int, VertexCount>       rows;
    std::array<int, VertexCount>       cols;
    for(int vertex = 0; vertex < VertexCount; ++vertex)
    {
        values[vertex].setZero();
        for(int component = 0; component < 3; ++component)
            values[vertex](component, component) =
                Float{2 + component} + Float{0.125} * vertex;
        rows[vertex] = vertex;
        cols[vertex] = vertex;
    }

    cuda_tool::DeviceBuffer<Matrix3x3> d_values(VertexCount);
    cuda_tool::DeviceBuffer<int>       d_rows(VertexCount);
    cuda_tool::DeviceBuffer<int>       d_cols(VertexCount);
    d_values.view().copy_from(values.data());
    d_rows.view().copy_from(rows.data());
    d_cols.view().copy_from(cols.data());
    engine.set_preconditioner(d_values.cview(), d_rows.cview(), d_cols.cview(), 0);

    cuda_tool::DeviceDenseVector<Float> d_x;
    cuda_tool::DeviceDenseVector<Float> d_p;
    cuda_tool::DeviceDenseVector<Float> d_r;
    cuda_tool::DeviceDenseVector<Float> d_Ap;
    cuda_tool::DeviceDenseVector<Float> d_z;
    d_x.resize(ScalarCount);
    d_p.resize(ScalarCount);
    d_r.resize(ScalarCount);
    d_Ap.resize(ScalarCount);
    d_z.resize(ScalarCount);

    constexpr Float              Rz           = 3.0;
    constexpr Float              PAp          = 2.0;
    constexpr Float              Alpha        = Rz / PAp;
    constexpr Float              InitialRzNew = 0.75;
    cuda_tool::DeviceVar<Float>  d_rz{Rz};
    cuda_tool::DeviceVar<Float>  d_pAp{PAp};
    cuda_tool::DeviceVar<Float>  d_rz_new{InitialRzNew};
    cuda_tool::DeviceVar<IndexT> d_converged{IndexT{0}};

    std::vector<Float> actual_x(ScalarCount);
    std::vector<Float> actual_r(ScalarCount);
    std::vector<Float> actual_z(ScalarCount);
    for(int pass = 0; pass < 2; ++pass)
    {
        std::vector<Float> x(ScalarCount);
        std::vector<Float> p(ScalarCount);
        std::vector<Float> r(ScalarCount);
        std::vector<Float> Ap(ScalarCount);
        std::vector<Float> expected_x(ScalarCount);
        std::vector<Float> expected_r(ScalarCount);
        for(int i = 0; i < ScalarCount; ++i)
        {
            x[i]          = Float{0.01 * (i + 1) + 0.1 * pass};
            p[i]          = Float{0.02 * (i % 7 + 1)};
            r[i]          = Float{0.4 + 0.003 * i + 0.2 * pass};
            Ap[i]         = Float{0.005 * (i % 5 + 1)};
            expected_x[i] = x[i] + Alpha * p[i];
            expected_r[i] = r[i] - Alpha * Ap[i];
        }
        const auto expected_z      = expected_diagonal_apply(expected_r);
        Float      expected_rz_new = InitialRzNew;
        for(int i = 0; i < ScalarCount; ++i)
            expected_rz_new += expected_r[i] * expected_z[i];

        d_x.buffer_view().copy_from(x.data());
        d_p.buffer_view().copy_from(p.data());
        d_r.buffer_view().copy_from(r.data());
        d_Ap.buffer_view().copy_from(Ap.data());
        d_z.buffer_view().fill(Float{-100 - pass});
        d_rz_new = InitialRzNew;

        engine.fused_pcg_update_apply_dot(d_x.view(),
                                          d_p.cview(),
                                          d_r.view(),
                                          d_Ap.cview(),
                                          d_z.view(),
                                          d_rz.view(),
                                          d_pAp.view(),
                                          d_rz_new.view(),
                                          d_converged.view(),
                                          nullptr);

        d_x.buffer_view().copy_to(actual_x.data());
        d_r.buffer_view().copy_to(actual_r.data());
        d_z.buffer_view().copy_to(actual_z.data());
        require_near(actual_x, expected_x, 1e-12);
        require_near(actual_r, expected_r, 1e-12);
        require_near(actual_z, expected_z);
        REQUIRE(static_cast<Float>(d_rz_new) == Catch::Approx(expected_rz_new).margin(1e-4));
    }

    SECTION("converged status is a full no-op")
    {
        d_x.buffer_view().fill(Float{7});
        d_r.buffer_view().fill(Float{11});
        d_z.buffer_view().fill(Float{13});
        d_rz_new    = Float{17};
        d_converged = IndexT{1};

        engine.fused_pcg_update_apply_dot(d_x.view(),
                                          d_p.cview(),
                                          d_r.view(),
                                          d_Ap.cview(),
                                          d_z.view(),
                                          d_rz.view(),
                                          d_pAp.view(),
                                          d_rz_new.view(),
                                          d_converged.view(),
                                          nullptr);

        d_x.buffer_view().copy_to(actual_x.data());
        d_r.buffer_view().copy_to(actual_r.data());
        d_z.buffer_view().copy_to(actual_z.data());
        require_near(actual_x, std::vector<Float>(ScalarCount, Float{7}), Float{0});
        require_near(actual_r, std::vector<Float>(ScalarCount, Float{11}), Float{0});
        require_near(actual_z, std::vector<Float>(ScalarCount, Float{13}), Float{0});
        REQUIRE(static_cast<Float>(d_rz_new) == Float{17});
    }
}
