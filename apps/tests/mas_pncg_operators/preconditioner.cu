#include <finite_element/mas_preconditioner_engine.h>
#include <app/app.h>
#include <Eigen/Cholesky>
#include <array>
#include <vector>

namespace
{
using namespace uipc;
using namespace uipc::backend::cuda;
namespace ct = uipc::backend::cuda_tool;

struct TwoNodeMAS
{
    MASPreconditionerEngine           engine;
    ct::DeviceBuffer<Eigen::Matrix3d> values;
    ct::DeviceBuffer<int>             rows;
    ct::DeviceBuffer<int>             cols;

    TwoNodeMAS()
    {
        std::array<unsigned int, 2> neighbors{1, 0};
        std::array<unsigned int, 2> starts{0, 1};
        std::array<unsigned int, 2> counts{1, 1};
        std::array<int, 16>         part;
        part.fill(-1);
        part[0] = 0;
        part[1] = 1;
        std::array<int, 2> real{0, 1};
        engine.init_neighbor(2, 2, 16, neighbors, starts, counts, part, real);
        engine.init_matrix();
        rows.resize_discard(3);
        cols.resize_discard(3);
        values.resize_discard(3);
        std::array<int, 3> row_ids{0, 0, 1};
        std::array<int, 3> col_ids{0, 1, 1};
        rows.view().copy_from(row_ids.data());
        cols.view().copy_from(col_ids.data());
    }

    void assemble(double a, double b, double coupling)
    {
        std::array<Eigen::Matrix3d, 3> blocks{a * Eigen::Matrix3d::Identity(),
                                              coupling * Eigen::Matrix3d::Identity(),
                                              b * Eigen::Matrix3d::Identity()};
        values.view().copy_from(blocks.data());
        engine.set_preconditioner(values.cview(), rows.cview(), cols.cview(), 0);
    }
};

Eigen::Matrix<double, 48, 48> dense(const MASPreconditionerEngine::ClusterMatrixSym& packed)
{
    Eigen::Matrix<double, 48, 48> result;
    for(int row = 0; row < 16; ++row)
        for(int col = row; col < 16; ++col)
        {
            const auto& block = packed.M[16 * row - row * (row + 1) / 2 + col];
            result.block<3, 3>(row * 3, col * 3) = block;
            if(row != col)
                result.block<3, 3>(col * 3, row * 3) = block.transpose();
        }
    return result;
}

TEST_CASE("MAS nonlinear base retains double precision and padded identity", "[cuda][mas_pncg]")
{
    TwoNodeMAS fixture;
    double     a = 2.000000000123;
    double     b = 3.000000000456;
    fixture.assemble(a, b, 0.25);
    REQUIRE(fixture.engine.capture_nonlinear_base(11));
    auto base = fixture.engine.nonlinear_base(11);
    REQUIRE(base.valid());
    REQUIRE(base.domain_count == 1);
    REQUIRE(base.real_vertex_count == 2);
    REQUIRE(base.partition_slot_count == 16);
    REQUIRE_FALSE(fixture.engine.nonlinear_base(12).valid());

    MASPreconditionerEngine::ClusterMatrixSym packed_A, packed_B;
    base.base_matrices.copy_to(&packed_A);
    base.base_inverses.copy_to(&packed_B);
    auto A = dense(packed_A);
    auto B = dense(packed_B);
    Eigen::Matrix<double, 48, 48> expected = Eigen::Matrix<double, 48, 48>::Identity();
    expected.block<3, 3>(0, 0) = a * Eigen::Matrix3d::Identity();
    expected.block<3, 3>(3, 3) = b * Eigen::Matrix3d::Identity();
    expected.block<3, 3>(0, 3) = expected.block<3, 3>(3, 0) =
        0.25 * Eigen::Matrix3d::Identity();
    REQUIRE(A.isApprox(expected, 1e-14));
    auto identity = Eigen::Matrix<double, 48, 48>::Identity().eval();
    Eigen::Matrix<double, 48, 48> reference = expected.llt().solve(identity);
    REQUIRE(B.isApprox(reference, 1e-12));
    REQUIRE((A * B).isApprox(identity, 1e-12));

    fixture.assemble(7.0, 8.0, 0.5);
    base.base_matrices.copy_to(&packed_A);
    base.base_inverses.copy_to(&packed_B);
    REQUIRE(dense(packed_A).isApprox(expected, 1e-14));
    REQUIRE(dense(packed_B).isApprox(reference, 1e-12));
}

TEST_CASE("MAS frozen coarse application excludes fine solve and live numeric changes", "[cuda][mas_pncg]")
{
    TwoNodeMAS fixture;
    fixture.assemble(2.0, 3.0, 0.25);
    REQUIRE(fixture.engine.capture_nonlinear_base(21));
    ct::DeviceDenseVector<Float> r, z;
    r.resize(6);
    z.resize(6);
    std::array<Float, 6> rhs{1, 2, 3, 4, 5, 6};
    r.view().buffer_view().copy_from(rhs.data());
    fixture.engine.apply_frozen_coarse(21, r.cview(), z.view());
    std::array<Float, 6> before;
    z.cview().buffer_view().copy_to(before.data());
    for(int i = 0; i < 3; ++i)
    {
        double expected = (rhs[i] + rhs[i + 3]) / 5.5;
        REQUIRE(std::abs(before[i] - expected) < 1e-6);
        REQUIRE(std::abs(before[i + 3] - expected) < 1e-6);
    }
    fixture.assemble(7.0, 8.0, 0.5);
    z.view().buffer_view().fill(Float{99});
    fixture.engine.apply_frozen_coarse(21, r.cview(), z.view());
    std::array<Float, 6> after;
    z.cview().buffer_view().copy_to(after.data());
    for(int i = 0; i < 6; ++i)
        REQUIRE(std::abs(after[i] - before[i]) < 1e-12);
}

TEST_CASE("MAS nonlinear capture rejects zero curvature on real slots", "[cuda][mas_pncg]")
{
    TwoNodeMAS fixture;
    fixture.assemble(2.0, 3.0, 0.25);
    REQUIRE(fixture.engine.capture_nonlinear_base(31));
    fixture.assemble(0.0, 3.0, 0.0);
    REQUIRE_FALSE(fixture.engine.capture_nonlinear_base(32));
    REQUIRE_FALSE(fixture.engine.nonlinear_base(31).valid());
    REQUIRE_FALSE(fixture.engine.nonlinear_base(32).valid());
}

TEST_CASE("MAS hierarchy preserves analytic corrections across split and padded domains",
          "[cuda][mas_pncg]")
{
    for(auto [node_count, coarse_levels, split] : {std::array{33, 2, 0},
                                                   std::array{257, 3, 0},
                                                   std::array{4, 1, 1},
                                                   std::array{17, 2, 1},
                                                   std::array{257, 3, 1}})
    {
        CAPTURE(node_count, split);
        int slot_count = ((node_count + 15) / 16) * 16;
        std::vector<unsigned int> neighbors, starts(node_count), counts(node_count);
        std::vector<int> part(slot_count, -1), real(node_count), diagonal(node_count);
        for(int i = 0; i < node_count; ++i)
        {
            starts[i] = static_cast<unsigned int>(neighbors.size());
            if(split)
            {
                int partner = i ^ 1;
                if(partner < node_count)
                    neighbors.push_back(partner);
            }
            else
            {
                if(i > 0)
                    neighbors.push_back(i - 1);
                if(i + 1 < node_count)
                    neighbors.push_back(i + 1);
            }
            counts[i] = static_cast<unsigned int>(neighbors.size()) - starts[i];
            real[i]   = split ? i : node_count - 1 - i;
            part[real[i]] = i;
            diagonal[i]   = i;
        }
        MASPreconditionerEngine engine;
        engine.init_neighbor(
            node_count, static_cast<int>(neighbors.size()), slot_count, neighbors, starts, counts, part, real);
        engine.init_matrix();
        ct::DeviceBuffer<int> rows(node_count), cols(node_count);
        rows.view().copy_from(diagonal.data());
        cols.view().copy_from(diagonal.data());
        constexpr double a = 2.000000000123;
        std::vector<Eigen::Matrix3d> blocks(node_count, a * Eigen::Matrix3d::Identity());
        ct::DeviceBuffer<Eigen::Matrix3d> values(node_count);
        values.view().copy_from(blocks.data());
        engine.set_preconditioner(values.cview(), rows.cview(), cols.cview(), 0);
        REQUIRE(engine.capture_nonlinear_base(41));
        auto base = engine.nonlinear_base(41);
        REQUIRE(base.domain_count == slot_count / 16);
        std::vector<MASPreconditionerEngine::ClusterMatrixSym> matrices(base.domain_count),
            inverses(base.domain_count);
        base.base_matrices.copy_to(matrices.data());
        base.base_inverses.copy_to(inverses.data());
        const auto identity = Eigen::Matrix<double, 48, 48>::Identity().eval();
        for(int domain = 0; domain < base.domain_count; ++domain)
        {
            Eigen::Matrix<double, 48, 48> expected = identity;
            for(int slot = 0; slot < 16; ++slot)
                if(part[16 * domain + slot] >= 0)
                    expected.block<3, 3>(3 * slot, 3 * slot) =
                        a * Eigen::Matrix3d::Identity();
            REQUIRE(dense(matrices[domain]).isApprox(expected, 1e-14));
            REQUIRE(dense(inverses[domain]).isApprox(expected.llt().solve(identity), 1e-12));
        }

        // Restriction sums H and r over the same aggregates. Connected
        // constants and disconnected pair means have analytic coarse solves.
        std::vector<Float> rhs(3 * node_count), result(3 * node_count),
            coarse(3 * node_count);
        for(int i = 0; i < 3 * node_count; ++i)
            rhs[i] = 1 + i % (split ? 11 : 3);
        for(int i = 0; i < node_count; ++i)
            for(int component = 0; component < 3; ++component)
            {
                int   k         = 3 * i + component;
                int   partner   = i ^ 1;
                Float aggregate = split && partner < node_count ?
                                      (rhs[k] + rhs[3 * partner + component]) / (2 * a) :
                                      rhs[k] / a;
                coarse[k]       = coarse_levels * aggregate;
            }
        ct::DeviceDenseVector<Float> r, z;
        r.resize(rhs.size());
        z.resize(rhs.size());
        r.view().buffer_view().copy_from(rhs.data());
        engine.apply_frozen_coarse(41, r.cview(), z.view());
        z.cview().buffer_view().copy_to(result.data());
        Float max_error = 0;
        for(int i = 0; i < 3 * node_count; ++i)
            max_error = std::max(max_error, std::abs(result[i] - coarse[i]));
        REQUIRE(max_error < 1e-5);
        ct::DeviceVar<IndexT> converged(0);
        engine.apply(r.cview(), z.view(), converged.cview());
        z.cview().buffer_view().copy_to(result.data());
        max_error = 0;
        for(int i = 0; i < 3 * node_count; ++i)
            max_error =
                std::max(max_error, std::abs(result[i] - (rhs[i] / a + coarse[i])));
        REQUIRE(max_error < 1e-5);
    }
}
}  // namespace
