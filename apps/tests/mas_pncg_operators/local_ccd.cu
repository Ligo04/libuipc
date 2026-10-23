#include <nonlinear_system/mas_pncg_local_ccd.h>
#include <app/app.h>
#include <array>
#include <limits>
#include <vector>

namespace pncg_local_ccd_test
{
using namespace uipc;
using namespace uipc::backend::cuda;
namespace ct = uipc::backend::cuda_tool;

template <typename T>
ct::DeviceBuffer<T> upload(const std::vector<T>& values)
{
    ct::DeviceBuffer<T> device(values.size());
    device.view().copy_from(values.data());
    return device;
}

__global__ void unequal_motion_guard(ct::BufferView<Float> results)
{
    Vector3 x[4] = {{0.2, 0.2, 1}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    Vector3 dx[4];
    for(auto& v : dx)
        v = Vector3{0, 0, -2};
    results(0) = pncg::point_triangle_safe_step(x, dx, 0).fraction;
    // A third constraint slows only the triangle's domain.
    for(int i = 1; i < 4; ++i)
        dx[i] *= 0.1;
    Float      toi = 1;
    const bool hit = distance::point_triangle_ccd(
        x[0], x[1], x[2], x[3], dx[0], dx[1], dx[2], dx[3], 0.1, 0.0, 1000, toi);
    results(1) = hit;
    results(2) = toi;
    results(3) = (x[0] + toi * dx[0] - x[1] - toi * dx[1]).z();
}
}  // namespace pncg_local_ccd_test

TEST_CASE("MAS-PNCG local CCD scales domains and singletons without changing raw history",
          "[mas_pncg][cccd][device]")
{
    using namespace pncg_local_ccd_test;
    auto mapping    = upload<IndexT>({0, 1, 16, -1, -1, 17});
    auto fixed      = upload<IndexT>({0, 0, 0, 0, 0, 1});
    auto fractions  = upload<Float>({0.2, 0.6, 1, 1, 1, 0.4, 0.8, 1});
    auto per_vertex = upload<Float>({1, 1, 1, 1, 1, 1});
    auto counts     = upload<IndexT>({0, 0, 0});
    MASPreconditionerEngine::FrozenLevel0View base;
    base.real_to_part      = mapping.view();
    base.domain_count      = 2;
    base.real_vertex_count = 6;
    ct::DeviceDenseVector<Float> raw, trial;
    raw.resize_discard(18);
    trial.resize_discard(18);
    std::array<Float, 18> values;
    for(int i = 0; i < 18; ++i)
        values[i] = i < 15 ? i + 1 : 0;
    raw.buffer_view().copy_from(values.data());
    pncg::local_ccd_scale_kernel<<<1, 32>>>(raw.cview(),
                                            trial.view(),
                                            base,
                                            fixed.view(),
                                            fractions.view(),
                                            per_vertex.view(),
                                            counts.view());
    std::array<Float, 18> original, actual;
    raw.copy_to(original.data());
    trial.copy_to(actual.data());
    const Float expected[] = {0.2, 0.2, 0.6, 0.4, 0.8, 1};
    for(int i = 0; i < 18; ++i)
    {
        REQUIRE(original[i] == values[i]);
        REQUIRE(actual[i] == Catch::Approx(values[i] * expected[i / 3]));
    }
    std::array<IndexT, 3> stats;
    counts.view().copy_to(stats.data());
    REQUIRE(stats[1] == 5);
    REQUIRE(stats[2] == 0);
}

TEST_CASE("MAS-PNCG actual-direction CCD catches collision introduced by unequal domain steps",
          "[mas_pncg][cccd][device]")
{
    using namespace pncg_local_ccd_test;
    ct::DeviceBuffer<Float> results(4);
    unequal_motion_guard<<<1, 1>>>(results.view());
    std::array<Float, 4> values;
    results.view().copy_to(values.data());
    REQUIRE(values[0] == 1);
    REQUIRE(values[1] == 1);
    REQUIRE(values[2] > 0);
    REQUIRE(values[2] < 1 / 1.8);
    REQUIRE(values[3] > 0);
}

TEST_CASE("MAS-PNCG local pair kernels propagate thick-contact bounds to incident domains",
          "[mas_pncg][cccd][device]")
{
    using namespace pncg_local_ccd_test;
    bool point_triangle = true;
    SECTION("point-triangle") {}
    SECTION("edge-edge")
    {
        point_triangle = false;
    }
    std::vector<Vector3> x, dx(4, Vector3::Zero());
    if(point_triangle)
    {
        x     = {{0.2, 0.2, 1}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
        dx[0] = Vector3{0, 0, -2};
    }
    else
    {
        x     = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 1}, {0, 1, 1}};
        dx[2] = dx[3] = Vector3{0, 0, -2};
    }
    auto positions     = upload(x);
    auto displacements = upload(dx);
    auto thickness     = upload<Float>({0.1, 0.1, 0.1, 0.1});
    auto pairs         = upload<Vector2i>({{0, point_triangle ? 0 : 1}});
    auto vertices      = upload<IndexT>({0});
    auto triangles     = upload<Vector3i>({{1, 2, 3}});
    auto edges         = upload<Vector2i>({{0, 1}, {2, 3}});
    auto mapping       = upload<IndexT>({0, 1, 16, 17});
    auto fixed         = upload<IndexT>({0, 0, 0, 0});
    auto fractions     = upload<Float>({1, 1, 1, 1, 1, 1});
    auto counts        = upload<IndexT>({0, 0, 0});
    MASPreconditionerEngine::FrozenLevel0View base;
    base.real_to_part      = mapping.view();
    base.domain_count      = 2;
    base.real_vertex_count = 4;
    auto launch            = [&](auto kernel)
    {
        kernel<<<1, 32>>>(pairs.view(),
                          vertices.view(),
                          edges.view(),
                          triangles.view(),
                          positions.view(),
                          displacements.view(),
                          thickness.view(),
                          base,
                          fixed.view(),
                          fractions.view(),
                          counts.view(),
                          0,
                          0.1);
    };
    if(point_triangle)
        launch(pncg::local_ccd_pairs_kernel<true>);
    else
        launch(pncg::local_ccd_pairs_kernel<false>);
    Float  result[6];
    IndexT stats[3];
    fractions.view().copy_to(result);
    counts.view().copy_to(stats);
    REQUIRE(result[0] > 0);
    REQUIRE(result[0] < 0.4);
    REQUIRE(result[1] == result[0]);
    REQUIRE(result[2] == 1);
    REQUIRE(stats[0] == 1);
    REQUIRE(stats[2] == 0);
}

TEST_CASE("MAS-PNCG local CCD selects a descending actual direction without changing raw history",
          "[mas_pncg][cccd][descent_guard][device]")
{
    using namespace pncg_local_ccd_test;
    Matrix2x2 B, H;
    B << 5, -3, -3, 2;
    H << 2, 3, 3, 5;
    const Vector2 gradient{1, 1};
    const Vector2 raw_direction = -B * gradient;
    REQUIRE(B(0, 0) > 0);
    REQUIRE(B(0, 0) * B(1, 1) - B(0, 1) * B(1, 0) > 0);
    REQUIRE((B * H).isApprox(Matrix2x2::Identity()));
    REQUIRE(raw_direction(0) == -2);
    REQUIRE(raw_direction(1) == 1);

    Float first = 0.1, second = 1;
    bool  expect_fallback = true;
    bool  strictly_uphill = true;
    SECTION("mixed domain fractions turn a descent direction uphill") {}
    SECTION("mixed descending fractions retain the local proposal")
    {
        first           = 0.8;
        second          = 0.1;
        expect_fallback = false;
        strictly_uphill = false;
    }
    SECTION("uniform scaling retains the local proposal")
    {
        first = second  = 0.2;
        expect_fallback = false;
        strictly_uphill = false;
    }
    SECTION("zero slope also restores the descending raw direction")
    {
        first           = 0.5;
        strictly_uphill = false;
    }

    auto mapping    = upload<IndexT>({0, 16});
    auto fixed      = upload<IndexT>({0, 0});
    auto fractions  = upload<Float>({first, second});
    auto per_vertex = upload<Float>({1, 1});
    auto counts     = upload<IndexT>({0, 0, 0});
    MASPreconditionerEngine::FrozenLevel0View base;
    base.real_to_part      = mapping.view();
    base.domain_count      = 2;
    base.real_vertex_count = 2;
    const std::array<Float, 6> initial{raw_direction(0), 0, 0, raw_direction(1), 0, 0};
    ct::DeviceDenseVector<Float> raw, proposal;
    raw.resize_discard(initial.size());
    proposal.resize_discard(initial.size());
    raw.buffer_view().copy_from(initial.data());
    pncg::local_ccd_scale_kernel<<<1, 32>>>(raw.cview(),
                                            proposal.view(),
                                            base,
                                            fixed.view(),
                                            fractions.view(),
                                            per_vertex.view(),
                                            counts.view());

    std::array<Float, 6> scaled;
    proposal.copy_to(scaled.data());
    const Vector2 local_direction{scaled[0], scaled[3]};
    REQUIRE(local_direction(0) == Catch::Approx(first * raw_direction(0)));
    REQUIRE(local_direction(1) == Catch::Approx(second * raw_direction(1)));
    const Float raw_slope      = gradient.dot(raw_direction);
    const Float proposal_slope = gradient.dot(local_direction);
    const auto  selected =
        pncg::select_local_ccd_direction(raw.cview(), raw_slope, proposal.cview(), proposal_slope);
    REQUIRE(selected.valid);
    REQUIRE(selected.fallback == expect_fallback);
    REQUIRE(selected.slope < 0);

    std::array<Float, 6> actual, history;
    selected.direction.buffer_view().copy_to(actual.data());
    raw.copy_to(history.data());
    REQUIRE(history == initial);
    REQUIRE(actual == (expect_fallback ? initial : scaled));
    REQUIRE(selected.slope == Catch::Approx(gradient.dot(Vector2{actual[0], actual[3]})));

    if(strictly_uphill)
    {
        REQUIRE(raw_slope == -1);
        REQUIRE(proposal_slope == Catch::Approx(0.8));
        REQUIRE(local_direction.dot(H * local_direction) > 0);
        // For this SPD quadratic both terms are positive for every t > 0,
        // so shortening the unchanged local proposal cannot decrease energy.
        for(Float step : {1e-12, 1e-6, 0.1, 1.0})
        {
            const Vector2 local_delta = step * local_direction;
            const Vector2 raw_delta   = step * raw_direction;
            REQUIRE(gradient.dot(local_delta) + 0.5 * local_delta.dot(H * local_delta) > 0);
            REQUIRE(gradient.dot(raw_delta) + 0.5 * raw_delta.dot(H * raw_delta) < 0);
        }
    }
}

TEST_CASE("MAS-PNCG local CCD descent selection rejects invalid slopes", "[mas_pncg][cccd][descent_guard]")
{
    using namespace pncg_local_ccd_test;
    const Float nan = std::numeric_limits<Float>::quiet_NaN();
    const Float inf = std::numeric_limits<Float>::infinity();
    for(Float invalid : {nan, inf, -inf})
    {
        REQUIRE_FALSE(pncg::select_local_ccd_direction({}, invalid, {}, -0.1).valid);
        REQUIRE_FALSE(pncg::select_local_ccd_direction({}, -1, {}, invalid).valid);
    }
    REQUIRE_FALSE(pncg::select_local_ccd_direction({}, 0, {}, -0.1).valid);
    REQUIRE_FALSE(pncg::select_local_ccd_direction({}, 1, {}, -0.1).valid);
}
