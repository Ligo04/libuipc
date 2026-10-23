#include <contact_system/ipc_ee_curvature.h>
#include <nonlinear_system/mas_pncg_fused.h>
#include <app/app.h>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

namespace pncg_mollified_ee_test
{
using namespace uipc;
using namespace uipc::backend::cuda;
namespace ct         = uipc::backend::cuda_tool;
constexpr int offset = 17;
using Dense48        = Eigen::Matrix<Float, 48, 48>;
using Vector48       = Eigen::Matrix<Float, 48, 1>;

template <typename T>
ct::DeviceBuffer<T> upload(const std::vector<T>& values)
{
    ct::DeviceBuffer<T> device(values.size());
    if(!values.empty())
        device.view().copy_from(values.data());
    return device;
}

ct::DeviceDenseVector<Float> upload_vector(const Vector12& values)
{
    ct::DeviceDenseVector<Float> device;
    device.resize_discard(12);
    device.buffer_view().copy_from(values.data());
    return device;
}

Matrix12x12 positive_part(const Matrix12x12& matrix)
{
    Eigen::SelfAdjointEigenSolver<Matrix12x12> solver((matrix + matrix.transpose()) * 0.5);
    REQUIRE(solver.info() == Eigen::Success);
    return solver.eigenvectors() * solver.eigenvalues().cwiseMax(0).asDiagonal()
           * solver.eigenvectors().transpose();
}

void require_matrix(const Matrix12x12& actual, const Matrix12x12& expected, Float tolerance = 2e-9)
{
    REQUIRE(actual.allFinite());
    REQUIRE((actual - expected).norm()
            <= tolerance * std::max(Float{1}, expected.norm()));
}

void require_scalar(Float actual, Float expected, Float tolerance = 2e-9)
{
    REQUIRE(std::isfinite(actual));
    REQUIRE(std::abs(actual - expected)
            <= tolerance * std::max(Float{1}, std::abs(expected)));
}

struct EEInput
{
    Vector3  positions[4], rest[4];
    Vector4i vertices = Vector4i{offset, offset + 1, offset + 2, offset + 3};
    Float    kt2 = 1, d_hat = 0.5, thickness = 0;

    EEInput()
    {
        positions[0] = Vector3{-0.5, 0, 0};
        positions[1] = Vector3{0.5, 0, 0};
        positions[2] = Vector3{-0.5, 0.2, 0.01};
        positions[3] = Vector3{0.5, 0.2, -0.01};
        for(int i = 0; i < 4; ++i)
            rest[i] = positions[i];
    }
};

struct EEOutput
{
    Float               energy = 0;
    Vector12            gradient;
    Matrix12x12         raw_hessian;
    IPCNormalSurrogate  contact;
    IPCContactCurvature curvature;
    IndexT              success = 0;
};

__global__ void evaluate_ee(ct::CBufferView<EEInput> inputs, ct::BufferView<EEOutput> outputs)
{
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if(i >= inputs.size())
        return;
    const auto& input = inputs(i);
    EEOutput    output;
    output.gradient.setZero();
    output.raw_hessian.setZero();
    output.contact.vertices        = input.vertices;
    output.contact.curvature_index = i;
    const auto* x                  = input.positions;
    const auto* r                  = input.rest;
    output.success = ipc_surrogate::export_edge_edge_curvature(output.contact,
                                                               output.curvature,
                                                               input.kt2,
                                                               input.d_hat,
                                                               input.thickness,
                                                               r[0],
                                                               r[1],
                                                               r[2],
                                                               r[3],
                                                               x[0],
                                                               x[1],
                                                               x[2],
                                                               x[3]);
    if(output.success)
    {
        const Vector4i flag = distance::edge_edge_distance_flag(x[0], x[1], x[2], x[3]);
        output.energy = sym::codim_ipc_simplex_contact::mollified_EE_barrier_energy(
            flag, input.kt2, input.d_hat, input.thickness, r[0], r[1], r[2], r[3], x[0], x[1], x[2], x[3]);
        sym::codim_ipc_simplex_contact::mollified_EE_barrier_gradient_hessian(
            output.gradient,
            output.raw_hessian,
            flag,
            input.kt2,
            input.d_hat,
            input.thickness,
            r[0],
            r[1],
            r[2],
            r[3],
            x[0],
            x[1],
            x[2],
            x[3]);
    }
    outputs(i) = output;
}

std::vector<EEOutput> evaluate(const std::vector<EEInput>& inputs)
{
    auto                       device_inputs = upload(inputs);
    ct::DeviceBuffer<EEOutput> device_outputs(inputs.size());
    if(!inputs.empty())
        evaluate_ee<<<(inputs.size() + 31) / 32, 32>>>(device_inputs.cview(),
                                                       device_outputs.view());
    std::vector<EEOutput> outputs;
    device_outputs.copy_to(outputs);
    return outputs;
}

Matrix12x12 global_matrix(const Matrix12x12& local, const Vector4i& vertices)
{
    Matrix12x12 result = Matrix12x12::Zero();
    for(int a = 0; a < 4; ++a)
        for(int b = 0; b < 4; ++b)
            result.block<3, 3>(3 * (vertices[a] - offset), 3 * (vertices[b] - offset)) +=
                local.block<3, 3>(3 * a, 3 * b);
    return result;
}

struct ContactBatch
{
    std::vector<IPCNormalSurrogate>  contacts;
    std::vector<IPCContactCurvature> curvatures;
};

void append_contact(ContactBatch& batch, IPCNormalSurrogate contact, const Matrix12x12& curvature)
{
    contact.curvature_index = batch.curvatures.size();
    IPCContactCurvature packed;
    packed.assign(curvature);
    batch.contacts.push_back(contact);
    batch.curvatures.push_back(packed);
}

ContactBatch full_contact(const Matrix12x12& curvature)
{
    IPCNormalSurrogate contact;
    contact.kind      = 1;
    contact.count     = 4;
    contact.valid     = 1;
    contact.mollified = 1;
    contact.vertices  = Vector4i{offset, offset + 1, offset + 2, offset + 3};
    contact.feature   = Vector4i::Ones();
    ContactBatch result;
    append_contact(result, contact, curvature);
    return result;
}

struct SortedContacts
{
    ct::DeviceBuffer<unsigned long long> hashes;
    ct::DeviceBuffer<IndexT>             order;
};

SortedContacts sort_contacts(ct::CBufferView<IPCNormalSurrogate> contacts)
{
    SortedContacts sorted;
    sorted.hashes.resize_discard(contacts.size());
    sorted.order.resize_discard(contacts.size());
    if(contacts.size() == 0)
        return sorted;
    pncg::contact_hash_kernel<<<(contacts.size() + 63) / 64, 64>>>(
        contacts, sorted.hashes.view(), sorted.order.view());
    std::vector<unsigned long long> hashes;
    sorted.hashes.copy_to(hashes);
    std::vector<IndexT> order(contacts.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(),
                     order.end(),
                     [&](IndexT a, IndexT b) { return hashes[a] < hashes[b]; });
    std::vector<unsigned long long> ordered;
    for(IndexT index : order)
        ordered.push_back(hashes[index]);
    sorted.hashes.view().copy_from(ordered.data());
    sorted.order.view().copy_from(order.data());
    return sorted;
}

Matrix12x12 column_operator(const std::vector<IPCNormalSurrogate>& columns)
{
    Matrix12x12 matrix = Matrix12x12::Zero();
    for(const auto& column : columns)
    {
        if(column.valid != 1)
            continue;
        Vector12 u = Vector12::Zero();
        for(int j = 0; j < column.count; ++j)
        {
            REQUIRE(column.vertices[j] >= offset);
            REQUIRE(column.vertices[j] < offset + 4);
            REQUIRE(column.q[j].allFinite());
            u.segment<3>(3 * (column.vertices[j] - offset)) += column.q[j];
        }
        matrix.noalias() += u * u.transpose();
    }
    return matrix;
}

struct UpdateResult
{
    std::vector<IPCNormalSurrogate> columns;
    IndexT                          error = 0, active = 0;
};

UpdateResult update(const ContactBatch&        current,
                    const ContactBatch&        baseline,
                    const std::vector<IndexT>& fixed = {0, 0, 0, 0})
{
    auto contacts = upload(current.contacts), old_contacts = upload(baseline.contacts);
    auto curvatures     = upload(current.curvatures),
         old_curvatures = upload(baseline.curvatures);
    auto fixed_vertices = upload(fixed);
    auto sorted         = sort_contacts(contacts.cview()),
         old_sorted     = sort_contacts(old_contacts.cview());
    ct::DeviceBuffer<IPCNormalSurrogate> columns(current.contacts.size()
                                                 + 11 * current.curvatures.size());
    columns.fill(IPCNormalSurrogate{});
    ct::DeviceVar<IndexT> error(0), active(0);
    if(!current.contacts.empty())
        pncg::contact_updates_kernel<<<(current.contacts.size() + 63) / 64, 64>>>(
            contacts.cview(),
            sorted.hashes.cview(),
            sorted.order.cview(),
            old_contacts.cview(),
            old_sorted.hashes.cview(),
            old_sorted.order.cview(),
            fixed_vertices.cview(),
            offset,
            std::cos(25.0 * std::acos(-1.0) / 180.0),
            columns.view(),
            error.view(),
            active.view(),
            curvatures.cview(),
            old_curvatures.cview());
    UpdateResult result;
    columns.copy_to(result.columns);
    result.error  = static_cast<IndexT>(error);
    result.active = static_cast<IndexT>(active);
    if(result.error == 0)
        REQUIRE(result.active
                == std::count_if(result.columns.begin(),
                                 result.columns.end(),
                                 [](const auto& c) { return c.valid == 1; }));
    return result;
}

TEST_CASE("MAS-PNCG mollified EE raw derivatives agree with independent finite differences",
          "[cuda][mas_pncg][mollified_ee]")
{
    for(Float thickness : {0.0, 0.01})
    {
        EEInput center;
        center.thickness          = thickness;
        constexpr Float      step = 1e-6;
        std::vector<EEInput> inputs{center};
        for(int i = 0; i < 12; ++i)
        {
            EEInput plus = center, minus = center;
            plus.positions[i / 3][i % 3] += step;
            minus.positions[i / 3][i % 3] -= step;
            inputs.push_back(plus);
            inputs.push_back(minus);
        }
        const auto  outputs = evaluate(inputs);
        Vector12    numeric_gradient;
        Matrix12x12 numeric_hessian;
        for(int i = 0; i < 12; ++i)
        {
            REQUIRE(outputs[1 + 2 * i].success == 1);
            REQUIRE(outputs[2 + 2 * i].success == 1);
            numeric_gradient(i) =
                (outputs[1 + 2 * i].energy - outputs[2 + 2 * i].energy) / (2 * step);
            numeric_hessian.col(i) =
                (outputs[1 + 2 * i].gradient - outputs[2 + 2 * i].gradient) / (2 * step);
        }
        const auto& value = outputs.front();
        REQUIRE(value.success == 1);
        REQUIRE(value.contact.valid == 1);
        REQUIRE(value.contact.mollified == 1);
        REQUIRE((value.gradient - numeric_gradient).norm()
                <= 2e-6 * std::max(Float{1}, value.gradient.norm()));
        require_matrix(value.raw_hessian, numeric_hessian, 2e-5);
        require_matrix(value.raw_hessian, value.raw_hessian.transpose(), 1e-12);
        for(int axis = 0; axis < 3; ++axis)
        {
            Vector12 translation = Vector12::Zero();
            for(int vertex = 0; vertex < 4; ++vertex)
                translation(3 * vertex + axis) = 1;
            REQUIRE((value.raw_hessian * translation).norm()
                    <= 1e-10 * std::max(Float{1}, value.raw_hessian.norm()));
        }
        require_matrix(value.curvature.matrix(), positive_part(value.raw_hessian));
    }
}

TEST_CASE("MAS-PNCG EE exporter handles parallel limits permutations and invalid inputs",
          "[cuda][mas_pncg][mollified_ee]")
{
    EEInput              original;
    std::vector<EEInput> inputs{original};
    for(auto permutation :
        {std::array{1, 0, 3, 2}, std::array{2, 3, 0, 1}, std::array{3, 2, 1, 0}})
    {
        auto changed = original;
        for(int i = 0; i < 4; ++i)
        {
            changed.positions[i] = original.positions[permutation[i]];
            changed.rest[i]      = original.rest[permutation[i]];
            changed.vertices[i]  = original.vertices[permutation[i]];
        }
        inputs.push_back(changed);
    }
    auto parallel             = original;
    parallel.positions[2].z() = parallel.positions[3].z() = 0;
    inputs.push_back(parallel);
    auto disabled = original;
    disabled.kt2  = 0;
    inputs.push_back(disabled);
    auto unmollified             = original;
    unmollified.positions[2].z() = 0.03;
    unmollified.positions[3].z() = -0.03;
    inputs.push_back(unmollified);
    auto nan_position             = original;
    nan_position.positions[0].x() = std::numeric_limits<Float>::quiet_NaN();
    inputs.push_back(nan_position);
    auto infinite_rest        = original;
    infinite_rest.rest[0].x() = std::numeric_limits<Float>::infinity();
    inputs.push_back(infinite_rest);
    auto overlap         = original;
    overlap.positions[2] = original.positions[0];
    overlap.positions[3] = original.positions[1];
    inputs.push_back(overlap);
    auto invalid_distance  = original;
    invalid_distance.d_hat = 0;
    inputs.push_back(invalid_distance);
    auto invalid_stiffness = original;
    invalid_stiffness.kt2  = -1;
    inputs.push_back(invalid_stiffness);
    auto invalid_thickness      = original;
    invalid_thickness.thickness = -0.01;
    inputs.push_back(invalid_thickness);
    const auto outputs = evaluate(inputs);
    for(int i = 0; i < 4; ++i)
    {
        REQUIRE(outputs[i].success == 1);
        REQUIRE((outputs[i].contact.vertices - original.vertices).isZero());
        require_scalar(outputs[i].energy, outputs[0].energy);
        require_matrix(outputs[i].curvature.matrix(), outputs[0].curvature.matrix());
        require_matrix(outputs[i].curvature.matrix(),
                       positive_part(global_matrix(outputs[i].raw_hessian,
                                                   inputs[i].vertices)));
    }
    REQUIRE(outputs[4].success == 1);
    REQUIRE(outputs[4].contact.mollified == 1);
    REQUIRE(outputs[4].energy == 0);
    REQUIRE(outputs[4].gradient.isZero(1e-12));
    REQUIRE(outputs[4].curvature.matrix().norm() > 0);
    require_matrix(outputs[4].curvature.matrix(), positive_part(outputs[4].raw_hessian));
    REQUIRE(outputs[5].success == 1);
    REQUIRE(outputs[5].curvature.matrix().isZero(1e-12));
    REQUIRE(outputs[6].success == 1);
    REQUIRE(outputs[6].contact.mollified == 0);
    for(int i = 7; i < outputs.size(); ++i)
    {
        REQUIRE(outputs[i].success == 0);
        REQUIRE(outputs[i].contact.valid == 0);
    }
    REQUIRE(evaluate({}).empty());
}

TEST_CASE("MAS-PNCG full EE increments are the epoch-relative positive matrix difference",
          "[cuda][mas_pncg][mollified_ee]")
{
    Matrix12x12 base = Matrix12x12::Identity();
    base.diagonal().array() += Eigen::Array<Float, 12, 1>::LinSpaced(12, 0.0, 2.0);
    const auto baseline = full_contact(base);
    for(Float scale : {1.0, 2.0, 0.5, 0.0, 3.0, 1.0})
    {
        CAPTURE(scale);
        const auto result = update(full_contact(scale * base), baseline);
        REQUIRE(result.error == 0);
        REQUIRE(result.columns.size() == 12);
        require_matrix(column_operator(result.columns),
                       std::max(Float{0}, scale - 1) * base);
        if(scale <= 1)
            REQUIRE(result.active == 0);
        else
            REQUIRE(result.active == 12);
    }
    const auto absent = update({}, baseline);
    REQUIRE(absent.error == 0);
    REQUIRE(absent.active == 0);
    REQUIRE(absent.columns.empty());

    Matrix12x12 rotation = Matrix12x12::Identity();
    rotation(0, 0) = rotation(7, 7) = std::cos(0.6);
    rotation(0, 7)                  = -std::sin(0.6);
    rotation(7, 0)                  = std::sin(0.6);
    Matrix12x12 rotated             = rotation * base * rotation.transpose();
    rotated(0, 0) += 0.8;
    rotated(4, 4) -= 0.4;
    const Matrix12x12                          difference = rotated - base;
    Eigen::SelfAdjointEigenSolver<Matrix12x12> eigenvalues(difference);
    REQUIRE(eigenvalues.eigenvalues().minCoeff() < 0);
    REQUIRE(eigenvalues.eigenvalues().maxCoeff() > 0);
    const auto changed = update(full_contact(rotated), baseline);
    REQUIRE(changed.error == 0);
    require_matrix(column_operator(changed.columns), positive_part(difference));
    Eigen::SelfAdjointEigenSolver<Matrix12x12> domination(
        base + column_operator(changed.columns) - rotated);
    REQUIRE(domination.eigenvalues().minCoeff()
            >= -2e-9 * std::max(Float{1}, rotated.norm()));
    Matrix12x12 mask = Matrix12x12::Identity();
    mask.block<3, 3>(0, 0).setZero();
    const auto masked = update(full_contact(rotated), baseline, {1, 0, 0, 0});
    REQUIRE(masked.error == 0);
    require_matrix(column_operator(masked.columns), mask * positive_part(difference) * mask);

    // Rotating columns within a repeated eigenspace changes the factorization,
    // but cannot create a physical contact update.
    Matrix12x12 factor = Matrix12x12::Identity();
    factor.block<3, 3>(0, 0) *= 2;
    rotation.setIdentity();
    rotation(0, 0) = rotation(1, 1)    = std::cos(0.37);
    rotation(0, 1)                     = -std::sin(0.37);
    rotation(1, 0)                     = std::sin(0.37);
    const Matrix12x12 alternate_factor = factor * rotation;
    const Matrix12x12 same = alternate_factor * alternate_factor.transpose();
    const auto        basis_changed =
        update(full_contact(same), full_contact(factor * factor.transpose()));
    REQUIRE(basis_changed.error == 0);
    REQUIRE(column_operator(basis_changed.columns).norm() <= 2e-9 * same.norm());
}

TEST_CASE("MAS-PNCG full EE matching preserves feature transitions multiplicity and fixed masking",
          "[cuda][mas_pncg][mollified_ee]")
{
    const Matrix12x12 identity    = Matrix12x12::Identity();
    const auto        baseline    = full_contact(identity);
    auto              current     = full_contact(2 * identity);
    current.contacts[0].feature   = Vector4i{1, 0, 1, 0};
    current.contacts[0].mollified = 0;
    auto result                   = update(current, baseline);
    REQUIRE(result.error == 0);
    require_matrix(column_operator(result.columns), identity);

    const std::vector<IndexT> fixed{0, 1, 0, 1};
    result           = update(current, baseline, fixed);
    Matrix12x12 mask = Matrix12x12::Identity();
    mask.block<3, 3>(3, 3).setZero();
    mask.block<3, 3>(9, 9).setZero();
    REQUIRE(result.error == 0);
    require_matrix(column_operator(result.columns), mask);
    REQUIRE(result.active == 6);
    const auto stationary = update(current, baseline, {1, 1, 1, 1});
    REQUIRE(stationary.error == 0);
    REQUIRE(stationary.active == 0);

    auto other_edges = current;
    other_edges.contacts[0].vertices =
        Vector4i{offset, offset + 2, offset + 1, offset + 3};
    result = update(other_edges, baseline);
    REQUIRE(result.error == 0);
    require_matrix(column_operator(result.columns), 2 * identity);

    for(int i = 0; i < 2; ++i)
        append_contact(current, current.contacts[0], 2 * identity);
    IPCNormalSurrogate plane;
    plane.kind        = 4;
    plane.count       = 1;
    plane.valid       = 1;
    plane.plane       = 5;
    plane.vertices[0] = offset;
    plane.q[0]        = Vector3::UnitX();
    plane.stiffness   = 2;
    current.contacts.insert(current.contacts.begin() + 1, plane);
    Matrix12x12 expected = 5 * identity;
    expected(0, 0) += 2;
    result = update(current, baseline);
    REQUIRE(result.error == 0);
    REQUIRE(result.columns.size() == 37);
    REQUIRE(result.active == 37);
    require_matrix(column_operator(result.columns), expected);
    std::reverse(current.contacts.begin(), current.contacts.end());
    const auto reversed = update(current, baseline);
    REQUIRE(reversed.error == 0);
    require_matrix(column_operator(reversed.columns), expected);
}

TEST_CASE("MAS-PNCG actual mollified EE transitions use PSD baseline curvature",
          "[cuda][mas_pncg][mollified_ee]")
{
    EEInput              center;
    std::vector<EEInput> inputs{center};
    auto                 closer = center;
    closer.positions[2].y() = closer.positions[3].y() = 0.18;
    inputs.push_back(closer);
    auto farther             = center;
    farther.positions[2].y() = farther.positions[3].y() = 0.22;
    inputs.push_back(farther);
    auto unmollified             = center;
    unmollified.positions[2].z() = 0.03;
    unmollified.positions[3].z() = -0.03;
    inputs.push_back(unmollified);
    auto vertex_edge = center;
    vertex_edge.positions[2].x() += 0.7;
    vertex_edge.positions[3].x() += 0.7;
    inputs.push_back(vertex_edge);
    const auto   outputs = evaluate(inputs);
    ContactBatch baseline;
    append_contact(baseline, outputs[0].contact, outputs[0].curvature.matrix());
    REQUIRE(outputs[0].contact.mollified == 1);
    REQUIRE(outputs[3].contact.mollified == 0);
    REQUIRE((outputs[0].contact.feature - outputs[4].contact.feature).norm() > 0);
    for(int i = 0; i < outputs.size(); ++i)
    {
        CAPTURE(i);
        REQUIRE(outputs[i].success == 1);
        ContactBatch current;
        append_contact(current, outputs[i].contact, outputs[i].curvature.matrix());
        const auto result = update(current, baseline);
        REQUIRE(result.error == 0);
        require_matrix(column_operator(result.columns),
                       positive_part(outputs[i].curvature.matrix()
                                     - outputs[0].curvature.matrix()));
        if(i == 0)
            REQUIRE(result.active == 0);
    }
    auto invalid                   = baseline;
    invalid.curvatures[0].upper[0] = std::numeric_limits<Float>::quiet_NaN();
    REQUIRE(update(invalid, {}).error != 0);
}

MASPreconditionerEngine::ClusterMatrixSym pack(const Dense48& matrix)
{
    MASPreconditionerEngine::ClusterMatrixSym result;
    int                                       entry = 0;
    for(int row = 0; row < 16; ++row)
        for(int col = row; col < 16; ++col)
            result.M[entry++] = matrix.block<3, 3>(3 * row, 3 * col);
    return result;
}

TEST_CASE("MAS-PNCG actual EE columns retain the selected local direct solve",
          "[cuda][mas_pncg][mollified_ee]")
{
    const auto exported = evaluate({EEInput{}}).front();
    REQUIRE(exported.success == 1);
    ContactBatch batch;
    append_contact(batch, exported.contact, exported.curvature.matrix());
    const auto updates = update(batch, {});
    REQUIRE(updates.error == 0);
    std::vector<IndexT> selected(8, -1);
    int                 selected_count = 0;
    for(int i = 0; i < updates.columns.size() && selected_count < 8; ++i)
        if(updates.columns[i].valid == 1)
            selected[selected_count++] = i;
    REQUIRE(selected_count > 0);

    std::vector<int> part_to_real(16, -1);
    part_to_real[0] = 2;
    part_to_real[1] = 0;
    part_to_real[2] = 3;
    part_to_real[3] = 1;
    const std::vector<int> real_to_part{1, 3, 0, 2};
    Dense48                base = Dense48::Identity();
    for(int i = 0; i < 12; ++i)
        base(i, i) = 2 + 0.1 * i;
    base(0, 4) = base(4, 0) = 0.1;
    Vector12 gradient, initial = Vector12::Constant(0.125);
    for(int i = 0; i < 12; ++i)
        gradient(i) = 1 + 0.2 * (i % 5);
    Eigen::Matrix<Float, 48, 8> u   = Eigen::Matrix<Float, 48, 8>::Zero();
    Vector48                    rhs = Vector48::Zero();
    for(int part = 0; part < 4; ++part)
    {
        const int real           = part_to_real[part];
        rhs.segment<3>(3 * part) = gradient.segment<3>(3 * real);
        for(int column = 0; column < selected_count; ++column)
            for(int j = 0; j < updates.columns[selected[column]].count; ++j)
                if(updates.columns[selected[column]].vertices[j] - offset == real)
                    u.block<3, 1>(3 * part, column) +=
                        updates.columns[selected[column]].q[j];
    }
    const Vector48 reference = (base + u * u.transpose()).ldlt().solve(rhs);
    auto           matrices  = upload(std::vector{pack(base)});
    auto inverses = upload(std::vector{pack(base.ldlt().solve(Dense48::Identity()))});
    auto part = upload(part_to_real), real = upload(real_to_part);
    auto columns   = upload(updates.columns);
    auto selection = upload(selected);
    auto g = upload_vector(gradient), z = upload_vector(initial);
    ct::DeviceVar<IndexT>                     error(0), fallbacks(0);
    MASPreconditionerEngine::FrozenLevel0View frozen{
        matrices.cview(), inverses.cview(), part.cview(), real.cview(), 1, 4, 16, 71};
    pncg::local_solve_kernel<<<1, 64>>>(frozen,
                                        columns.cview(),
                                        selection.cview(),
                                        g.cview(),
                                        z.view(),
                                        offset,
                                        8,
                                        error.view(),
                                        fallbacks.view());
    Vector12 actual;
    z.copy_to(actual.data());
    REQUIRE(static_cast<IndexT>(error) == 0);
    for(int vertex = 0; vertex < 4; ++vertex)
        for(int component = 0; component < 3; ++component)
            require_scalar(actual(3 * vertex + component),
                           initial(3 * vertex + component)
                               + reference(3 * real_to_part[vertex] + component),
                           2e-8);
}

TEST_CASE("MAS-PNCG all actual EE update columns enter plain fused and graph directions",
          "[cuda][mas_pncg][mollified_ee][graph]")
{
    const auto exported = evaluate({EEInput{}}).front();
    REQUIRE(exported.success == 1);
    ContactBatch batch;
    for(int i = 0; i < 8; ++i)
        append_contact(batch, exported.contact, exported.curvature.matrix());
    const auto largest = update(batch, {});
    REQUIRE(largest.error == 0);
    REQUIRE(largest.active >= 12);
    REQUIRE(largest.columns.size() == 96);
    ct::DeviceBuffer<IPCNormalSurrogate> columns(96);
    columns.fill(IPCNormalSurrogate{});
    ct::DeviceBuffer<Vector3> projections(96);
    Matrix12x12               base = 2 * Matrix12x12::Identity();
    base.diagonal().array() += Eigen::Array<Float, 12, 1>::LinSpaced(12, 0.0, 1.0);
    Vector12 g, z, p;
    for(int i = 0; i < 12; ++i)
    {
        g(i) = 0.3 + 0.11 * (i % 7);
        z(i) = g(i) / base(i, i);
        p(i) = 0.07 * ((i * 5) % 11 - 5);
    }
    auto dg = upload_vector(g), dz = upload_vector(z), dp = upload_vector(p);
    auto old_z = upload_vector(Vector12::Zero());
    auto hz = upload_vector(base * z), hp = upload_vector(base * p);
    ct::DeviceDenseVector<Float> direction;
    direction.resize_discard(12);
    pncg::FusedOperations operations;
    for(int count : {0, 1, 8, 2, 0})
    {
        CAPTURE(count);
        ContactBatch current;
        for(int i = 0; i < count; ++i)
            append_contact(current, exported.contact, exported.curvature.matrix());
        const auto updates = update(current, {});
        REQUIRE(updates.error == 0);
        const auto n = updates.columns.size();
        if(n)
            columns.view().subview(0, n).copy_from(updates.columns.data());
        const Matrix12x12    correction = column_operator(updates.columns);
        Matrix<Float, 12, 2> span;
        span.col(0) = -z;
        span.col(1) = p;
        const Eigen::Matrix2d reduced = span.transpose() * (base + correction) * span;
        const Eigen::Vector2d coefficients = reduced.ldlt().solve(-span.transpose() * g);
        const Vector12 expected_direction = span * coefficients;
        REQUIRE(g.dot(expected_direction) < 0);

        if(n)
            pncg::projection_kernel<<<(n + 63) / 64, 64>>>(
                columns.cview().subview(0, n),
                dz.cview(),
                dp.cview(),
                offset,
                projections.view().subview(0, n));
        std::vector<Vector3> projected(n);
        projections.cview().subview(0, n).copy_to(projected.data());
        Vector3 sums = Vector3::Zero();
        for(const auto& value : projected)
            sums += value;
        require_scalar(sums(0), z.dot(correction * z));
        require_scalar(sums(1), z.dot(correction * p));
        require_scalar(sums(2), p.dot(correction * p));
        const auto plain = solve_pncg_subspace(z.dot(base * z) + sums(0),
                                               z.dot(base * p) + sums(1),
                                               p.dot(base * p) + sums(2),
                                               g.dot(z),
                                               g.dot(p),
                                               true);
        REQUIRE(plain.valid);
        REQUIRE(plain.used_history);
        require_scalar(plain.mu, coefficients(0));
        require_scalar(plain.nu, coefficients(1));
        for(bool graph : {false, true})
        {
            const auto actual = operations.direction(dg.cview(),
                                                     dz.cview(),
                                                     old_z.cview(),
                                                     dp.cview(),
                                                     hz.cview(),
                                                     hp.cview(),
                                                     columns.cview().subview(0, n),
                                                     offset,
                                                     true,
                                                     0.3,
                                                     direction.view(),
                                                     graph);
            REQUIRE(actual.valid == 1);
            REQUIRE(actual.used_history == 1);
            REQUIRE(actual.update_columns == updates.active);
            require_scalar(actual.mu, coefficients(0));
            require_scalar(actual.nu, coefficients(1));
            require_scalar(actual.slope, g.dot(expected_direction));
            Vector12 actual_direction;
            direction.copy_to(actual_direction.data());
            REQUIRE((actual_direction - expected_direction).norm()
                    <= 2e-8 * std::max(Float{1}, expected_direction.norm()));
        }
        REQUIRE(operations.graph_captures() == 1);
        REQUIRE(operations.graph_fallbacks() == 0);
    }
    REQUIRE(operations.graph_replays() == 5);
}
}  // namespace pncg_mollified_ee_test
