#include <nonlinear_system/mas_pncg_fused.h>
#include <app/app.h>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace pncg_fused_test
{
using namespace uipc;
using namespace uipc::backend::cuda;
namespace ct     = uipc::backend::cuda_tool;
using HostVector = Eigen::Matrix<Float, Eigen::Dynamic, 1>;
using HostMatrix = Eigen::Matrix<Float, Eigen::Dynamic, Eigen::Dynamic>;
constexpr int vertex_offset = 17;

template <typename T>
ct::DeviceBuffer<T> upload(const std::vector<T>& host)
{
    ct::DeviceBuffer<T> result(host.size());
    if(!host.empty())
        result.view().copy_from(host.data());
    return result;
}

ct::DeviceDenseVector<Float> upload(const HostVector& host)
{
    ct::DeviceDenseVector<Float> result;
    result.resize_discard(host.size());
    if(host.size())
        result.buffer_view().copy_from(host.data());
    return result;
}

HostVector download(const ct::DeviceDenseVector<Float>& device)
{
    HostVector result(device.size());
    if(result.size())
        device.copy_to(result.data());
    return result;
}

void require_close(Float actual, Float expected)
{
    REQUIRE(std::isfinite(actual));
    REQUIRE(std::abs(actual - expected) <= 2e-11 * std::max(Float{1}, std::abs(expected)));
}

void require_vector(const HostVector& actual, const HostVector& expected)
{
    REQUIRE(actual.size() == expected.size());
    REQUIRE(actual.allFinite());
    REQUIRE((actual - expected).norm() <= 3e-11 * std::max(Float{1}, expected.norm()));
}

struct ResidualInput
{
    HostVector          g;
    std::vector<Float>  mass, length, eta0;
    std::vector<IndexT> fixed;

    explicit ResidualInput(int vertices)
        : g(3 * vertices)
        , mass(vertices)
        , length(vertices)
        , eta0(vertices)
        , fixed(vertices)
    {
        for(int i = 0; i < vertices; ++i)
        {
            fixed[i]  = i % 11 == 0;
            mass[i]   = fixed[i] ? 0 : 0.5 + (i % 7) * 0.25;
            length[i] = 0.1 + (i % 5) * 0.03;
            eta0[i]   = 0.15 + (i % 9) * 0.01;
            g.segment<3>(3 * i) = Vector3(0.1 * (i % 13), -0.2, 0.07 * (i % 17));
            if(i % 19 == 0)
                g.segment<3>(3 * i).setZero();
        }
    }
};

pncg::ResidualSums residual_reference(ResidualInput& input, int vertices, Float atol, Float rtol, bool initialize)
{
    pncg::ResidualSums result;
    for(int i = 0; i < vertices; ++i)
    {
        const Float correction =
            input.fixed[i] ? 0 : input.g.segment<3>(3 * i).norm() / input.mass[i];
        const Float eta = correction / input.length[i];
        if(initialize)
            input.eta0[i] = eta;
        const Float ratio = eta / std::max(atol, rtol * input.eta0[i]);
        if(!input.fixed[i])
            result.sum += input.g.segment<3>(3 * i).squaredNorm() / input.mass[i];
        result.ratio      = std::max(result.ratio, ratio);
        result.correction = std::max(result.correction, correction);
    }
    return result;
}

void require_residual(const pncg::ResidualSums& actual, const pncg::ResidualSums& expected)
{
    REQUIRE(actual.invalid == 0);
    require_close(actual.sum, expected.sum);
    require_close(actual.ratio, expected.ratio);
    require_close(actual.correction, expected.correction);
}

struct DirectionInput
{
    HostMatrix                      base;
    HostVector                      g, z, old_z, p;
    std::vector<IPCNormalSurrogate> contacts;

    DirectionInput(int vertices, int contact_count, bool coupled = true)
        : base(HostMatrix::Zero(3 * vertices, 3 * vertices))
        , g(3 * vertices)
        , z(3 * vertices)
        , old_z(3 * vertices)
        , p(3 * vertices)
    {
        for(int i = 0; i < g.size(); ++i)
        {
            base(i, i) = 2 + 0.03 * (i % 23);
            if(coupled && i > 0)
                base(i, i - 1) = base(i - 1, i) = 0.125;
            g(i) = 0.5 + 0.12 * (i % 11 - 5);
            z(i) = g(i) / (1 + 0.2 * (i % 7));
            p(i) = 0.07 + 0.11 * ((i * 7) % 13 - 6);
        }
        old_z = 0.1 * z;
        for(int i = 0; i < contact_count; ++i)
        {
            IPCNormalSurrogate contact;
            contact.valid = 1;
            contact.count = std::min(vertices, 1 + i % 4);
            contact.kind  = contact.count == 1 ? 4 : 0;
            contact.stiffness = 99;  // Update columns already include their curvature scale.
            for(int j = 0; j < contact.count; ++j)
            {
                contact.vertices[j] = vertex_offset + (j + i) % vertices;
                contact.q[j]        = (0.08 + 0.02 * (i % 5))
                               * Vector3(1 + j, (j % 2 ? -1 : 1) * 0.5, 0.25 - 0.1 * j);
            }
            contacts.push_back(contact);
        }
    }
};

HostMatrix full_curvature(const DirectionInput& input, int dofs, int contact_count, int offset = vertex_offset)
{
    HostMatrix result = input.base.topLeftCorner(dofs, dofs);
    for(int i = 0; i < contact_count; ++i)
    {
        const auto& contact = input.contacts[i];
        if(contact.valid != 1)
            continue;
        HostVector column = HostVector::Zero(dofs);
        for(int j = 0; j < contact.count; ++j)
            column.segment<3>(3 * (contact.vertices[j] - offset)) += contact.q[j];
        result.noalias() += column * column.transpose();
    }
    return result;
}

struct DirectionReference
{
    Float      mu, nu, slope;
    HostVector direction;
};

DirectionReference direction_reference(const DirectionInput& input, int dofs, int contact_count, bool history)
{
    const HostMatrix h = full_curvature(input, dofs, contact_count);
    const HostVector g = input.g.head(dofs), z = input.z.head(dofs),
                     p = input.p.head(dofs);
    Float mu = g.dot(z) / z.dot(h * z), nu = 0;
    if(history)
    {
        HostMatrix span(dofs, 2);
        span.col(0)                         = -z;
        span.col(1)                         = p;
        const Eigen::Matrix2d       reduced = span.transpose() * h * span;
        Eigen::LLT<Eigen::Matrix2d> factor(reduced);
        REQUIRE(factor.info() == Eigen::Success);
        const Eigen::Vector2d coefficients = factor.solve(-span.transpose() * g);
        mu = coefficients(0);
        nu = coefficients(1);
    }
    HostVector direction = -mu * z + nu * p;
    return {mu, nu, g.dot(direction), std::move(direction)};
}

struct DeviceDirection
{
    ct::DeviceDenseVector<Float>         g, z, old_z, p, hz, hp, direction;
    ct::DeviceBuffer<IPCNormalSurrogate> contacts;

    explicit DeviceDirection(const DirectionInput& input)
        : g(upload(input.g))
        , z(upload(input.z))
        , old_z(upload(input.old_z))
        , p(upload(input.p))
        , hz(upload(HostVector(input.base * input.z)))
        , hp(upload(HostVector(input.base * input.p)))
        , contacts(upload(input.contacts))
    {
        direction.resize_discard(input.g.size());
    }

    pncg::DirectionResult run(pncg::FusedOperations& operations,
                              bool                   history,
                              bool                   graph,
                              Float                  threshold = 0.3)
    {
        return operations.direction(g.cview(),
                                    z.cview(),
                                    old_z.cview(),
                                    p.cview(),
                                    hz.cview(),
                                    hp.cview(),
                                    contacts.cview(),
                                    vertex_offset,
                                    history,
                                    threshold,
                                    direction.view(),
                                    graph);
    }
};

void require_direction(const pncg::DirectionResult& actual,
                       const HostVector&            actual_direction,
                       const DirectionReference&    expected,
                       bool                         history)
{
    REQUIRE(actual.valid == 1);
    REQUIRE(actual.used_history == IndexT(history));
    require_close(actual.mu, expected.mu);
    require_close(actual.nu, expected.nu);
    require_close(actual.slope, expected.slope);
    REQUIRE(actual.slope < 0);
    require_vector(actual_direction, expected.direction);
}

static __global__ void scale_input(ct::DenseVectorView<Float> values, Float factor)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i < values.size())
        values(i) *= factor;
}

TEST_CASE("MAS-PNCG fused residual matches fixed-mass CPU residuals", "[cuda][mas_pncg][fused]")
{
    for(int vertices : {0, 1, 85, 86, 257, 259})
    {
        CAPTURE(vertices);
        ResidualInput input(vertices);
        auto          g    = upload(input.g);
        auto          mass = upload(input.mass), length = upload(input.length),
             eta0                   = upload(input.eta0);
        auto                  fixed = upload(input.fixed);
        pncg::FusedOperations operations;
        for(bool initialize : {true, false})
        {
            CAPTURE(initialize);
            const auto expected =
                residual_reference(input, vertices, 1e-6, 1e-4, initialize);
            const auto actual = operations.residual(
                g.cview(), mass.cview(), fixed.cview(), length.cview(), eta0.view(), 1e-6, 1e-4, initialize, false);
            require_residual(actual, expected);
            std::vector<Float> actual_eta0;
            eta0.copy_to(actual_eta0);
            REQUIRE(actual_eta0.size() == input.eta0.size());
            for(int i = 0; i < vertices; ++i)
                require_close(actual_eta0[i], input.eta0[i]);
            input.g *= 0.375;
            if(input.g.size())
                g.buffer_view().copy_from(input.g.data());
        }
        REQUIRE(operations.graph_captures() == 0);
        REQUIRE(operations.graph_replays() == 0);
        REQUIRE(operations.graph_fallbacks() == 0);
    }
}

TEST_CASE("MAS-PNCG fused residual masks fixed gradients and reports invalid movable values",
          "[cuda][mas_pncg][fused]")
{
    ResidualInput input(3);
    input.g.segment<3>(0).setConstant(std::numeric_limits<Float>::quiet_NaN());
    SECTION("fixed NaNs do not contaminate the free residual") {}
    SECTION("a stationary free gradient has zero residual")
    {
        input.g.setZero();
    }
    SECTION("nonfinite movable gradient is invalid")
    {
        input.g(3) = std::numeric_limits<Float>::infinity();
    }
    SECTION("zero movable mass is invalid")
    {
        input.mass[1] = 0;
    }
    auto g    = upload(input.g);
    auto mass = upload(input.mass), length = upload(input.length),
         eta0                   = upload(input.eta0);
    auto                  fixed = upload(input.fixed);
    pncg::FusedOperations operations;
    const auto            actual = operations.residual(
        g.cview(), mass.cview(), fixed.cview(), length.cview(), eta0.view(), 1e-6, 1e-4, true, false);
    if(!std::isfinite(input.g(3)) || input.mass[1] == 0)
        REQUIRE(actual.invalid != 0);
    else
        require_residual(actual, residual_reference(input, 3, 1e-6, 1e-4, true));
}

TEST_CASE("MAS-PNCG fused directions minimize an independent contact-augmented quadratic",
          "[cuda][mas_pncg][fused]")
{
    for(auto shape : {std::pair{4, 3}, std::pair{87, 7}, std::pair{3, 263}})
    {
        CAPTURE(shape.first, shape.second);
        DirectionInput input(shape.first, shape.second);
        // An inactive update must not dereference its absent vertex IDs.
        input.contacts.push_back(IPCNormalSurrogate{});
        DeviceDirection       device(input);
        pncg::FusedOperations operations;
        for(bool history : {false, true})
        {
            CAPTURE(history);
            const auto expected =
                direction_reference(input, input.g.size(), input.contacts.size(), history);
            const auto actual = device.run(operations, history, false);
            REQUIRE(actual.update_columns == shape.second);
            const auto direction = download(device.direction);
            require_direction(actual, direction, expected, history);
            const HostMatrix h =
                full_curvature(input, input.g.size(), input.contacts.size());
            require_close(input.g.dot(direction), actual.slope);
            REQUIRE(std::abs(input.z.dot(input.g + h * direction)) < 1e-9);
            if(history)
                REQUIRE(std::abs(input.p.dot(input.g + h * direction)) < 1e-9);
        }
    }
}

TEST_CASE("MAS-PNCG fused direction rejects unusable curvature and restart history",
          "[cuda][mas_pncg][fused]")
{
    DirectionInput input(2, 0);
    input.base.setIdentity();
    input.g.setZero();
    input.g(0) = 1;
    input.g(1) = 0.5;
    input.z.setZero();
    input.z(0) = 1;
    input.p.setZero();
    input.p(1) = 1;
    input.old_z.setZero();
    bool valid = true, history = true;
    SECTION("collinear history keeps the one-dimensional direct solve")
    {
        input.p = 2 * input.z;
        history = false;
    }
    SECTION("indefinite reduced matrix keeps only positive one-dimensional curvature")
    {
        input.base(0, 0) = input.base(1, 1) = 2;
        input.base(0, 1) = input.base(1, 0) = 3;
        history                             = false;
    }
    SECTION("negative history curvature keeps the one-dimensional direct solve")
    {
        input.base(1, 1) = -1;
        history          = false;
    }
    SECTION("finite history whose squared norm overflows keeps the one-dimensional direct solve")
    {
        input.g(1) = 0;
        input.p(1) = 1e200;
        REQUIRE(input.p.allFinite());
        REQUIRE(std::isinf(input.p.squaredNorm()));
        history = false;
    }
    SECTION("nonpositive first curvature is rejected")
    {
        input.base(0, 0) = -1;
        valid            = false;
    }
    SECTION("nonpositive gradient preconditioner product is rejected")
    {
        input.z *= -1;
        valid = false;
    }
    SECTION("restart threshold rejects stale history")
    {
        input.old_z = 0.5 * input.z;
        valid       = false;
    }
    SECTION("nonfinite gradient is rejected")
    {
        input.g(0) = std::numeric_limits<Float>::quiet_NaN();
        valid      = false;
    }
    SECTION("nonfinite base curvature is rejected")
    {
        input.base(0, 0) = std::numeric_limits<Float>::infinity();
        valid            = false;
    }
    SECTION("nonfinite preconditioned gradient is rejected")
    {
        input.z(0) = std::numeric_limits<Float>::quiet_NaN();
        valid      = false;
    }
    SECTION("nonfinite contact curvature is rejected")
    {
        IPCNormalSurrogate contact;
        contact.valid       = 1;
        contact.count       = 1;
        contact.vertices[0] = vertex_offset;
        contact.q[0].setConstant(std::numeric_limits<Float>::quiet_NaN());
        input.contacts.push_back(contact);
        valid = false;
    }
    DeviceDirection       device(input);
    pncg::FusedOperations operations;
    const auto            actual = device.run(operations, true, false);
    if(valid)
    {
        const auto expected = direction_reference(input, 6, input.contacts.size(), history);
        require_direction(actual, download(device.direction), expected, history);
    }
    else
    {
        REQUIRE(actual.valid == 0);
        REQUIRE(actual.used_history == 0);
        REQUIRE(actual.mu == 0);
        REQUIRE(actual.nu == 0);
        REQUIRE(download(device.direction).isZero());
    }
}

TEST_CASE("MAS-PNCG fused acceptance copies the accepted history with a fractional step",
          "[cuda][mas_pncg][fused]")
{
    for(int vertices : {0, 1, 87, 259})
        for(Float fraction : {0.0, 0.375, 1.0})
        {
            CAPTURE(vertices, fraction);
            DirectionInput input(vertices, 0);
            auto           direction = upload(input.g), z = upload(input.z);
            auto           p = upload(HostVector::Constant(3 * vertices, -99));
            auto old_z       = upload(HostVector::Constant(3 * vertices, 77));
            pncg::FusedOperations operations;
            operations.accept(direction.cview(), z.cview(), p.view(), old_z.view(), fraction);
            require_vector(download(p), HostVector(fraction * input.g));
            require_vector(download(old_z), input.z);
            REQUIRE(cudaGetLastError() == cudaSuccess);
        }
}

TEST_CASE("MAS-PNCG fused residual graphs replay new producer data and recapture changed views",
          "[cuda][mas_pncg][fused][graph]")
{
    constexpr int vertices = 259;
    ResidualInput input(vertices);
    auto          g    = upload(input.g);
    auto          mass = upload(input.mass), length = upload(input.length),
         eta0                   = upload(input.eta0);
    auto                  fixed = upload(input.fixed);
    pncg::FusedOperations operations;
    auto run = [&](ct::CDenseVectorView<Float> gradient, int count, Float atol, bool graph)
    {
        const auto actual = operations.residual(gradient.subview(0, 3 * count),
                                                mass.cview().subview(0, count),
                                                fixed.cview().subview(0, count),
                                                length.cview().subview(0, count),
                                                eta0.view().subview(0, count),
                                                atol,
                                                1e-4,
                                                false,
                                                graph);
        require_residual(actual, residual_reference(input, count, atol, 1e-4, false));
    };
    run(g.cview(), vertices, 1e-6, false);
    REQUIRE(operations.graph_captures() == 0);
    run(g.cview(), vertices, 1e-6, true);
    REQUIRE(operations.graph_captures() == 1);
    REQUIRE(operations.graph_replays() == 1);
    for(Float factor : {0.5, 1.7, -0.8})
    {
        scale_input<<<(3 * vertices + 127) / 128, 128>>>(g.view(), factor);
        input.g *= factor;
        run(g.cview(), vertices, 1e-6, true);
        REQUIRE(operations.graph_captures() == 1);
        run(g.cview(), vertices, 1e-6, false);
    }
    REQUIRE(operations.graph_replays() == 4);

    auto replacement = upload(input.g);
    REQUIRE(replacement.cview().data() != g.cview().data());
    run(replacement.cview(), vertices, 1e-6, true);
    REQUIRE(operations.graph_captures() == 2);
    run(replacement.cview(), vertices - 1, 1e-6, true);
    REQUIRE(operations.graph_captures() == 3);
    run(replacement.cview(), vertices - 1, 2e-6, true);
    REQUIRE(operations.graph_captures() == 4);
    REQUIRE(operations.graph_replays() == 7);
    const auto initialized =
        operations.residual(replacement.cview().subview(0, 3 * (vertices - 1)),
                            mass.cview().subview(0, vertices - 1),
                            fixed.cview().subview(0, vertices - 1),
                            length.cview().subview(0, vertices - 1),
                            eta0.view().subview(0, vertices - 1),
                            2e-6,
                            1e-4,
                            true,
                            true);
    require_residual(initialized, residual_reference(input, vertices - 1, 2e-6, 1e-4, true));
    REQUIRE(operations.graph_captures() == 5);
    run(replacement.cview(), vertices - 1, 2e-6, true);
    REQUIRE(operations.graph_captures() == 5);
    REQUIRE(operations.graph_replays() == 9);
    REQUIRE(operations.graph_fallbacks() == 0);
}

TEST_CASE("MAS-PNCG fused direction graphs replay new gradients and invalidate sizes and addresses",
          "[cuda][mas_pncg][fused][graph]")
{
    constexpr int  vertices = 87;
    DirectionInput input(vertices, 5, false);
    // Keep contacts inside the retained prefix for the same-pointer size test.
    for(auto& contact : input.contacts)
        for(int j = 0; j < contact.count; ++j)
            contact.vertices[j] = vertex_offset + j;
    DeviceDirection       device(input);
    pncg::FusedOperations operations;
    auto                  run = [&](ct::DenseVectorView<Float> output,
                   int                        dofs,
                   int                        contact_count,
                   bool                       history,
                   Float                      threshold,
                   bool                       graph)
    {
        const auto actual =
            operations.direction(device.g.cview().subview(0, dofs),
                                 device.z.cview().subview(0, dofs),
                                 device.old_z.cview().subview(0, dofs),
                                 device.p.cview().subview(0, dofs),
                                 device.hz.cview().subview(0, dofs),
                                 device.hp.cview().subview(0, dofs),
                                 device.contacts.cview().subview(0, contact_count),
                                 vertex_offset,
                                 history,
                                 threshold,
                                 output.subview(0, dofs),
                                 graph);
        REQUIRE(actual.update_columns == contact_count);
        HostVector actual_direction(dofs);
        output.subview(0, dofs).buffer_view().copy_to(actual_direction.data());
        require_direction(actual,
                          actual_direction,
                          direction_reference(input, dofs, contact_count, history),
                          history);
    };
    run(device.direction.view(), 3 * vertices, 5, true, 0.3, false);
    REQUIRE(operations.graph_captures() == 0);
    run(device.direction.view(), 3 * vertices, 5, true, 0.3, true);
    REQUIRE(operations.graph_captures() == 1);
    for(Float factor : {0.7, 1.3, -0.5})
    {
        // Keep g^T z positive while changing both products on the producer stream.
        scale_input<<<(3 * vertices + 127) / 128, 128>>>(device.g.view(), factor);
        scale_input<<<(3 * vertices + 127) / 128, 128>>>(device.z.view(), factor);
        scale_input<<<(3 * vertices + 127) / 128, 128>>>(device.old_z.view(), factor);
        scale_input<<<(3 * vertices + 127) / 128, 128>>>(device.hz.view(), factor);
        input.g *= factor;
        input.z *= factor;
        input.old_z *= factor;
        run(device.direction.view(), 3 * vertices, 5, true, 0.3, true);
        REQUIRE(operations.graph_captures() == 1);
        run(device.direction.view(), 3 * vertices, 5, true, 0.3, false);
    }
    REQUIRE(operations.graph_replays() == 4);
    ct::DeviceDenseVector<Float> replacement;
    replacement.resize_discard(3 * vertices);
    REQUIRE(replacement.cview().data() != device.direction.cview().data());
    run(replacement.view(), 3 * vertices, 5, true, 0.3, true);
    REQUIRE(operations.graph_captures() == 2);
    run(replacement.view(), 3 * (vertices - 1), 5, true, 0.3, true);
    REQUIRE(operations.graph_captures() == 3);
    run(replacement.view(), 3 * (vertices - 1), 4, true, 0.3, true);
    REQUIRE(operations.graph_captures() == 3);
    run(replacement.view(), 3 * (vertices - 1), 4, true, 0.4, true);
    REQUIRE(operations.graph_captures() == 4);
    run(replacement.view(), 3 * (vertices - 1), 4, false, 0.4, true);
    REQUIRE(operations.graph_captures() == 5);
    run(replacement.view(), 3 * (vertices - 1), 4, true, 0.4, true);
    REQUIRE(operations.graph_captures() == 5);
    REQUIRE(operations.graph_replays() == 10);
    REQUIRE(operations.graph_fallbacks() == 0);
}

TEST_CASE("MAS-PNCG fused empty direction is a stationary invalid direction", "[cuda][mas_pncg][fused]")
{
    DirectionInput        input(0, 0);
    DeviceDirection       device(input);
    pncg::FusedOperations operations;
    for(bool graph : {false, true})
    {
        const auto actual = device.run(operations, false, graph);
        REQUIRE(actual.valid == 0);
        REQUIRE(actual.used_history == 0);
        REQUIRE(actual.mu == 0);
        REQUIRE(actual.nu == 0);
        REQUIRE(actual.slope == 0);
        REQUIRE(actual.update_columns == 0);
    }
    REQUIRE(operations.graph_captures() == 1);
    REQUIRE(operations.graph_replays() == 1);
    REQUIRE(operations.graph_fallbacks() == 0);
}
}  // namespace pncg_fused_test
