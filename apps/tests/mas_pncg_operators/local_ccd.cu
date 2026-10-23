#include <nonlinear_system/mas_pncg_local_ccd.h>
#include <app/app.h>
#include <array>
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
