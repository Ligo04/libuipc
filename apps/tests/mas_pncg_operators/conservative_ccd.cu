#include <nonlinear_system/mas_pncg_ccd_math.h>
#include <app/catch2.h>
#include <array>
#include <limits>
#include <vector>

namespace pncg_cccd_test
{
using namespace uipc;
using namespace uipc::backend::cuda;
namespace ct = uipc::backend::cuda_tool;

struct Pair
{
    Vector3 x[4], dx[4];
    Float   thickness = 0;
    bool    edge_edge = false;
};

Pair point_crossing()
{
    Pair pair;
    pair.x[0] = Vector3{0.25, 0.25, 1};
    pair.x[1] = Vector3{0, 0, 0};
    pair.x[2] = Vector3{1, 0, 0};
    pair.x[3] = Vector3{0, 1, 0};
    for(auto& v : pair.dx)
        v.setZero();
    pair.dx[0] = Vector3{0, 0, -2};
    return pair;
}

Pair edge_crossing()
{
    Pair pair;
    pair.edge_edge = true;
    pair.x[0]      = Vector3{-1, 0, 0};
    pair.x[1]      = Vector3{1, 0, 0};
    pair.x[2]      = Vector3{0, -1, 1};
    pair.x[3]      = Vector3{0, 1, 1};
    pair.dx[0] = pair.dx[1] = Vector3::Zero();
    pair.dx[2] = pair.dx[3] = Vector3{0, 0, -2};
    return pair;
}

Pair point_tangency()
{
    Pair pair  = point_crossing();
    pair.x[0]  = Vector3{0.25, 1, 0.25};
    pair.dx[0] = Vector3{0, -1, 0};
    pair.dx[3] = Vector3{0, 0, 1};
    // At t = 0.5 the point has triangle barycentrics (0.25, 0.25, 0.5).
    // The signed height numerator is (t - 0.5)^2: no endpoint sign change.
    return pair;
}

UIPC_GENERIC pncg::CCDStep evaluate(const Pair& pair)
{
    return pair.edge_edge ?
               pncg::edge_edge_safe_step(pair.x, pair.dx, pair.thickness) :
               pncg::point_triangle_safe_step(pair.x, pair.dx, pair.thickness);
}

void require_fraction(pncg::CCDStep step, Float upper)
{
    REQUIRE(std::isfinite(step.fraction));
    REQUIRE(step.fraction >= 0);
    REQUIRE(step.fraction <= upper);
}

// Independently evaluate the signed tetrahedron volume at a trajectory time.
long double volume(const Pair& pair, long double time)
{
    Eigen::Vector<long double, 3> p[4];
    for(int i = 0; i < 4; ++i)
        p[i] = pair.x[i].cast<long double>() + time * pair.dx[i].cast<long double>();
    return (p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0]));
}

#ifdef __CUDACC__
__global__ void evaluate_pairs(ct::CBufferView<Pair>         inputs,
                               ct::BufferView<pncg::CCDStep> outputs)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i < inputs.size())
        outputs(i) = evaluate(inputs(i));
}
#endif
}  // namespace pncg_cccd_test

TEST_CASE("MAS-PNCG conservative cubic boundary includes lower degree and tangent roots",
          "[mas_pncg][cccd][host]")
{
    using namespace pncg_cccd_test;
    struct Fixture
    {
        pncg::CubicPolynomial polynomial;
        Float                 first_root;
    };
    const Fixture fixtures[] = {
        {{0.5, -1, 0, 0}, 0.5},      // Linear crossing.
        {{0.1875, -1, 1, 0}, 0.25},  // Two quadratic roots, same endpoint signs.
        {{0.25, -1, 1, 0}, 0.5},     // Quadratic tangency.
        {{0.25, -0.75, 0, 1}, 0.5},     // (t - 0.5)^2 (t + 1), b == 0.
        {{-0.08, 0.66, -1.5, 1}, 0.2},  // Three roots: 0.2, 0.5, 0.8.
        {{1, -1, 0, 0}, 1},             // Endpoint root must not return 1.
    };
    for(const auto& fixture : fixtures)
    {
        const auto step = pncg::cubic_safe_step(fixture.polynomial, 0);
        require_fraction(step, fixture.first_root);
        REQUIRE(step.fraction < fixture.first_root);
        REQUIRE(step.fraction > fixture.first_root - 2e-5);
        REQUIRE_FALSE(step.needs_fallback);
    }
}

TEST_CASE("MAS-PNCG conservative cubic uses a certified prefix and flags uncertain cases",
          "[mas_pncg][cccd][host]")
{
    using namespace pncg_cccd_test;
    for(const auto& polynomial :
        {pncg::CubicPolynomial{}, pncg::CubicPolynomial{-0.125, 0.75, -1.5, 1}})
    {
        const auto step = pncg::cubic_safe_step(polynomial, 0.1);
        REQUIRE(step.needs_fallback);
        REQUIRE(step.fraction == 0.1);
    }
    REQUIRE(pncg::cubic_safe_step({1, 0, 0, 0}, 0).fraction == 1);
    REQUIRE(pncg::cubic_safe_step({0, 1, 0, 0}, 0).needs_fallback);
    REQUIRE(pncg::cubic_safe_step({1, 0, 0, 0}, -0.1).needs_fallback);
    REQUIRE(pncg::cubic_safe_step({1, 0, 0, 0}, 1.1).fraction == 0);
    REQUIRE(pncg::cubic_safe_step({std::numeric_limits<Float>::infinity(), 0, 0, 0}, 0)
                .needs_fallback);
    // The first coplanarity root can precede a separately certified geometric
    // prefix (e.g. crossing the extension of a triangle outside its interior).
    const auto later = pncg::cubic_safe_step({0.1875, -1, 1, 0}, 0.3);
    require_fraction(later, 0.75);
    REQUIRE(later.fraction == 0.5);
    REQUIRE_FALSE(later.needs_fallback);
    for(Float scale : {1e-200, -1e-200, 1e200, -1e200})
    {
        const auto scaled = pncg::cubic_safe_step({0.25 * scale, -scale, scale, 0}, 0);
        REQUIRE(scaled.fraction < 0.5);
        REQUIRE(scaled.fraction > 0.49998);
    }
}

TEST_CASE("MAS-PNCG conservative pair steps precede analytic crossings and thickness contact",
          "[mas_pncg][cccd][host]")
{
    using namespace pncg_cccd_test;
    for(auto pair : {point_crossing(), edge_crossing()})
    {
        auto step = evaluate(pair);
        require_fraction(step, 0.5);
        REQUIRE(step.fraction > 0.44);
        REQUIRE_FALSE(step.needs_fallback);
        pair.thickness = 0.2;
        step           = evaluate(pair);
        require_fraction(step, 0.4);
        REQUIRE(step.fraction > 0.35);
        REQUIRE(step.needs_fallback);
        pair.thickness = 1;
        REQUIRE(evaluate(pair).fraction == 0);
        pair.thickness = -0.1;
        REQUIRE(evaluate(pair).fraction == 0);
    }
}

TEST_CASE("MAS-PNCG conservative point-triangle tangency stops before interior contact",
          "[mas_pncg][cccd][host]")
{
    using namespace pncg_cccd_test;
    const Pair pair = point_tangency();
    REQUIRE(volume(pair, 0) * volume(pair, 1) > 0);
    REQUIRE(volume(pair, 0.5) == 0);
    const auto step = evaluate(pair);
    require_fraction(step, 0.5);
    REQUIRE(step.fraction > 0.44);
    REQUIRE_FALSE(step.needs_fallback);
}

TEST_CASE("MAS-PNCG cubic boundaries stay before independently constructed earliest roots",
          "[mas_pncg][cccd][host]")
{
    using namespace pncg_cccd_test;
    for(int i = 1; i <= 64; ++i)
    {
        const Float first = i / 128.0;
        for(Float second : {first, 0.75, 1.5, -0.5})
        {
            const Float                 third = 2;
            const pncg::CubicPolynomial f{-first * second * third,
                                          first * second + first * third + second * third,
                                          -(first + second + third),
                                          1};
            const auto                  step = pncg::cubic_safe_step(f, 0);
            require_fraction(step, first);
            REQUIRE(step.fraction < first);
            REQUIRE(step.fraction > first - 2e-5);
        }
    }
}

TEST_CASE("MAS-PNCG relative-motion bounds dominate interpolated primitive motion",
          "[mas_pncg][cccd][host]")
{
    using namespace pncg_cccd_test;
    const Vector3 v[4] = {
        {0.4, -0.2, 0.9}, {-0.1, 0.3, -0.5}, {1.1, -0.4, 0.3}, {-0.7, 0.2, -0.6}};
    const Float pt_bound = pncg::point_triangle_relative_bound(v[0], v[1], v[2], v[3]);
    const Float ee_bound = pncg::edge_edge_relative_bound(v[0], v[1], v[2], v[3]);
    for(int i = 0; i <= 16; ++i)
        for(int j = 0; j <= 16; ++j)
        {
            const Float   s = i / 16.0, t = j / 16.0;
            const Vector3 pt =
                v[0] - ((1 - s) * v[1] + s * (1 - t) * v[2] + s * t * v[3]);
            const Vector3 ee = (1 - s) * v[0] + s * v[1] - (1 - t) * v[2] - t * v[3];
            REQUIRE(pt.norm() <= pt_bound + 1e-14);
            REQUIRE(ee.norm() <= ee_bound + 1e-14);
        }
}

TEST_CASE("MAS-PNCG conservative coplanar motion retains a distance-certified prefix",
          "[mas_pncg][cccd][host]")
{
    using namespace pncg_cccd_test;
    auto point  = point_crossing();
    point.x[0]  = Vector3{2, 0, 0};
    point.dx[0] = Vector3{-2, 0, 0};
    auto edges  = edge_crossing();
    edges.x[2]  = Vector3{-1, 1, 0};
    edges.x[3]  = Vector3{1, 1, 0};
    edges.dx[2] = edges.dx[3] = Vector3{0, -2, 0};
    for(const auto& pair : {point, edges})
    {
        const auto step = evaluate(pair);
        REQUIRE(step.needs_fallback);
        require_fraction(step, 0.5);
        REQUIRE(step.fraction > 0.44);
    }
    point.x[2] = point.x[1];
    REQUIRE(evaluate(point).fraction == 0);
    REQUIRE(evaluate(point).needs_fallback);
}

TEST_CASE("MAS-PNCG conservative bounds are invariant to common velocity and rigid frames",
          "[mas_pncg][cccd][host]")
{
    using namespace pncg_cccd_test;
    const Matrix3x3 rotation =
        Eigen::AngleAxisd(0.7, Vector3{1, 2, 3}.normalized()).toRotationMatrix();
    for(const auto& original : {point_crossing(), edge_crossing()})
    {
        Pair transformed = original;
        for(int i = 0; i < 4; ++i)
        {
            transformed.x[i]  = rotation * original.x[i] + Vector3{16, 32, -64};
            transformed.dx[i] = rotation * original.dx[i] + Vector3{4, -2, 8};
        }
        const auto a = evaluate(original), b = evaluate(transformed);
        REQUIRE(a.needs_fallback == b.needs_fallback);
        REQUIRE(std::abs(a.fraction - b.fraction) < 1e-12);
        const Float original_bound =
            original.edge_edge ?
                pncg::edge_edge_relative_bound(
                    original.dx[0], original.dx[1], original.dx[2], original.dx[3]) :
                pncg::point_triangle_relative_bound(
                    original.dx[0], original.dx[1], original.dx[2], original.dx[3]);
        const Float transformed_bound =
            original.edge_edge ?
                pncg::edge_edge_relative_bound(transformed.dx[0],
                                               transformed.dx[1],
                                               transformed.dx[2],
                                               transformed.dx[3]) :
                pncg::point_triangle_relative_bound(transformed.dx[0],
                                                    transformed.dx[1],
                                                    transformed.dx[2],
                                                    transformed.dx[3]);
        REQUIRE(std::abs(original_bound - transformed_bound) < 1e-12);
        for(auto& dx : transformed.dx)
            dx = Vector3{4, -2, 8};
        const auto rigid = evaluate(transformed);
        REQUIRE(rigid.fraction == 1);
        REQUIRE_FALSE(rigid.needs_fallback);
    }
}

TEST_CASE("MAS-PNCG coplanarity coefficients match independently evaluated moving volumes",
          "[mas_pncg][cccd][host]")
{
    using namespace pncg_cccd_test;
    Pair pair    = point_crossing();
    pair.dx[1]   = Vector3{0.3, -0.2, 0.5};
    pair.dx[2]   = Vector3{-0.1, 0.4, 0.2};
    pair.dx[3]   = Vector3{0.7, -0.1, -0.3};
    const auto f = pncg::coplanarity_polynomial(pair.x, pair.dx);
    REQUIRE(f.a != 0);
    for(Float t : {0., 0.125, 0.375, 0.5, 0.75, 1.})
    {
        const long double expected = volume(pair, t);
        const Float       actual   = ((f.a * t + f.b) * t + f.c) * t + f.d;
        REQUIRE(std::abs(actual - expected) < 1e-14);
    }
}

#ifdef __CUDACC__
TEST_CASE("MAS-PNCG conservative CCD device pair evaluation matches host fixtures",
          "[cuda][mas_pncg][cccd]")
{
    using namespace pncg_cccd_test;
    std::vector<Pair> inputs{point_crossing(), edge_crossing(), point_tangency()};
    inputs.push_back(inputs.front());
    inputs.back().thickness = 0.2;
    inputs.push_back(inputs[1]);
    inputs.back().x[2]  = Vector3{-1, 1, 0};
    inputs.back().x[3]  = Vector3{1, 1, 0};
    inputs.back().dx[2] = inputs.back().dx[3] = Vector3{0, -2, 0};
    inputs.push_back(inputs.front());
    inputs.back().dx[0][0] = std::numeric_limits<Float>::quiet_NaN();
    ct::DeviceBuffer<Pair> device_inputs(inputs.size());
    device_inputs.view().copy_from(inputs.data());
    ct::DeviceBuffer<pncg::CCDStep> device_outputs(inputs.size());
    evaluate_pairs<<<1, 32>>>(device_inputs.cview(), device_outputs.view());
    std::vector<pncg::CCDStep> outputs;
    device_outputs.copy_to(outputs);
    for(size_t i = 0; i < inputs.size(); ++i)
    {
        const auto expected = evaluate(inputs[i]);
        REQUIRE(outputs[i].needs_fallback == expected.needs_fallback);
        REQUIRE(std::abs(outputs[i].fraction - expected.fraction) < 1e-12);
    }
}
#endif
