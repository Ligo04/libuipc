#include <nonlinear_system/mas_pncg_math.h>
#include <nonlinear_system/mas_pncg_kernels.h>
#include <app/app.h>
#include <Eigen/Cholesky>
#include <algorithm>
#include <numeric>
#include <vector>

namespace
{
using namespace uipc;
using namespace uipc::backend::cuda;
namespace ct  = uipc::backend::cuda_tool;
using Dense48 = Eigen::Matrix<double, 48, 48>;

template <typename T>
ct::DeviceBuffer<T> upload(const std::vector<T>& values)
{
    ct::DeviceBuffer<T> result(values.size());
    if(!values.empty())
        result.view().copy_from(values.data());
    return result;
}

ct::DeviceDenseVector<Float> upload_vector(const std::vector<Float>& values)
{
    ct::DeviceDenseVector<Float> result;
    result.resize_discard(values.size());
    result.buffer_view().copy_from(values.data());
    return result;
}

IPCNormalSurrogate pair_contact(Float stiffness)
{
    IPCNormalSurrogate result;
    result.kind      = 3;
    result.count     = 2;
    result.vertices  = Vector4i{17, 18, -1, -1};
    result.feature   = Vector4i{1, 1, -1, -1};
    result.q[0]      = Vector3::UnitX();
    result.q[1]      = -Vector3::UnitX();
    result.stiffness = stiffness;
    result.valid     = 1;
    return result;
}

Eigen::MatrixXd contact_operator(const std::vector<IPCNormalSurrogate>& columns,
                                 int vertices = 2)
{
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(3 * vertices, 3 * vertices);
    for(const auto& column : columns)
    {
        if(column.valid != 1)
            continue;
        Eigen::VectorXd u = Eigen::VectorXd::Zero(3 * vertices);
        for(int j = 0; j < column.count; ++j)
            u.segment<3>(3 * (column.vertices[j] - 17)) += column.q[j];
        result.noalias() += u * u.transpose();
    }
    return result;
}

struct ContactUpdates
{
    std::vector<IPCNormalSurrogate> columns;
    std::vector<unsigned long long> hashes;
    IndexT                          error = 0;
};

ContactUpdates update_contacts(const std::vector<IPCNormalSurrogate>& current,
                               const std::vector<IPCNormalSurrogate>& baseline,
                               const std::vector<IndexT>& fixed = {0, 0})
{
    auto                                 contacts       = upload(current);
    auto                                 old_contacts   = upload(baseline);
    auto                                 fixed_vertices = upload(fixed);
    ct::DeviceBuffer<unsigned long long> keys(current.size()),
        base_keys(baseline.size());
    ct::DeviceBuffer<IndexT> order(current.size()), base_order(baseline.size());
    ct::DeviceBuffer<IPCNormalSurrogate> updates(current.size());
    ContactUpdates                       result;
    auto sort_hashes = [](auto& records, auto& hashes, auto& indices)
    {
        std::vector<unsigned long long> unsorted(records.size());
        if(records.size() == 0)
            return unsorted;
        pncg::contact_hash_kernel<<<1, 64>>>(
            records.cview(), hashes.view(), indices.view());
        hashes.copy_to(unsorted.data());
        std::vector<IndexT> permutation(unsorted.size());
        std::iota(permutation.begin(), permutation.end(), 0);
        std::stable_sort(permutation.begin(),
                         permutation.end(),
                         [&](IndexT a, IndexT b)
                         { return unsorted[a] < unsorted[b]; });
        std::vector<unsigned long long> sorted;
        for(IndexT i : permutation)
            sorted.push_back(unsorted[i]);
        hashes.view().copy_from(sorted.data());
        indices.view().copy_from(permutation.data());
        return unsorted;
    };
    result.hashes = sort_hashes(contacts, keys, order);
    sort_hashes(old_contacts, base_keys, base_order);
    ct::DeviceVar<IndexT> error(0);
    ct::DeviceVar<IndexT> column_count(0);
    if(!current.empty())
        pncg::contact_updates_kernel<<<1, 64>>>(contacts.cview(),
                                                keys.cview(),
                                                order.cview(),
                                                old_contacts.cview(),
                                                base_keys.cview(),
                                                base_order.cview(),
                                                fixed_vertices.cview(),
                                                17,
                                                std::cos(25.0 * std::acos(-1.0) / 180.0),
                                                updates.view(),
                                                error.view(),
                                                column_count.view());
    REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
    updates.copy_to(result.columns);
    result.error = static_cast<IndexT>(error);
    return result;
}

TEST_CASE("MAS-PNCG subspace minimizes an independent dense quadratic", "[cuda][mas_pncg]")
{
    Eigen::Matrix4d R;
    R << 2, 1, 0, 0, 0, 1, -1, 0, 0, 0, 3, 1, 1, 0, 0, 2;
    Eigen::Matrix4d H = R.transpose() * R + Eigen::Matrix4d::Identity();
    Eigen::Vector4d g{1, -2, 3, 0.5}, z{1, -1, 1, 0.5}, p{-1, 0.5, 0.5, 1};
    Eigen::Matrix<double, 4, 2> span;
    span << -z, p;
    Eigen::Matrix2d reduced   = span.transpose() * H * span;
    Eigen::Vector2d reference = reduced.ldlt().solve(-span.transpose() * g);
    auto            result    = solve_pncg_subspace(
        z.dot(H * z), z.dot(H * p), p.dot(H * p), g.dot(z), g.dot(p), true);
    REQUIRE(result.valid);
    REQUIRE(result.used_history);
    REQUIRE(Eigen::Vector2d(result.mu, result.nu).isApprox(reference, 1e-12));
    Eigen::Vector4d direction = -result.mu * z + result.nu * p;
    REQUIRE((span.transpose() * (g + H * direction)).norm() < 1e-11);
    REQUIRE(g.dot(direction) < 0);

    SECTION("collinear history reduces to the steepest preconditioned direction")
    {
        p             = 2 * z;
        auto fallback = solve_pncg_subspace(
            z.dot(H * z), z.dot(H * p), p.dot(H * p), g.dot(z), g.dot(p), true);
        REQUIRE(fallback.valid);
        REQUIRE_FALSE(fallback.used_history);
        REQUIRE(fallback.nu == 0);
        REQUIRE(fallback.mu == Catch::Approx(g.dot(z) / z.dot(H * z)).epsilon(1e-12));
    }
    SECTION("positive diagonal entries do not make an indefinite span admissible")
    {
        auto fallback = solve_pncg_subspace(2, 3, 2, 1, 0.5, true);
        REQUIRE(fallback.valid);
        REQUIRE_FALSE(fallback.used_history);
        REQUIRE(fallback.mu == Catch::Approx(0.5));
        REQUIRE(fallback.nu == 0);
        REQUIRE_FALSE(solve_pncg_subspace(-1, 0, 1, 1, 0, true).valid);
    }
}

TEST_CASE("MAS-PNCG contact updates represent the restart-relative operator", "[cuda][mas_pncg]")
{
    auto            contact  = pair_contact(4);
    Eigen::VectorXd q        = Eigen::VectorXd::Zero(6);
    q[0]                     = 1;
    q[3]                     = -1;
    Eigen::MatrixXd rank_one = q * q.transpose();

    SECTION("base contacts stiffen, soften, disappear and reactivate without cumulative increments")
    {
        for(Float stiffness : {10.0, 7.0, 0.0, 2.0, 14.0})
        {
            CAPTURE(stiffness);
            std::vector<IPCNormalSurrogate> current;
            if(stiffness != 0)
                current.push_back(pair_contact(stiffness));
            auto result = update_contacts(current, {contact});
            REQUIRE(result.error == 0);
            Eigen::MatrixXd expected = (4.0 + std::max(stiffness - 4.0, 0.0)) * rank_one;
            REQUIRE((4.0 * rank_one + contact_operator(result.columns)).isApprox(expected, 1e-12));
        }
    }
    SECTION("new contacts are recomputed against the empty base on every iteration")
    {
        for(Float stiffness : {2.0, 9.0, 1.0, 0.0, 3.0})
        {
            CAPTURE(stiffness);
            std::vector<IPCNormalSurrogate> current;
            if(stiffness != 0)
                current.push_back(pair_contact(stiffness));
            auto result = update_contacts(current, {});
            REQUIRE(result.error == 0);
            REQUIRE((contact_operator(result.columns) - stiffness * rank_one).norm() < 1e-12);
        }
    }
    SECTION("duplicate occurrences and input permutations preserve total curvature")
    {
        auto plane     = contact;
        plane.kind     = 4;
        plane.count    = 1;
        plane.vertices = Vector4i{18, -1, -1, -1};
        plane.feature  = Vector4i{1, -1, -1, -1};
        plane.plane    = 3;
        plane.q[0]     = Vector3::UnitZ();
        plane.q[1].setZero();
        plane.stiffness = 5;
        auto ten        = pair_contact(10);
        auto a = update_contacts({ten, plane, ten, ten}, {contact, contact});
        auto b = update_contacts({plane, ten, ten, ten}, {contact, contact});
        Eigen::MatrixXd expected = 22.0 * rank_one;
        expected(5, 5) += 5.0;
        REQUIRE(a.error == 0);
        REQUIRE(b.error == 0);
        REQUIRE(contact_operator(a.columns).isApprox(expected, 1e-12));
        REQUIRE(contact_operator(b.columns).isApprox(expected, 1e-12));
        REQUIRE(a.hashes[0] == a.hashes[2]);
        REQUIRE(a.hashes[0] == b.hashes[1]);
        REQUIRE(a.hashes[1] == b.hashes[0]);
        REQUIRE(a.hashes[0] != a.hashes[1]);
    }
    SECTION("sign reversal preserves the column while rotation adds full current curvature")
    {
        auto changed = pair_contact(10);
        changed.q[0] *= -1;
        changed.q[1] *= -1;
        auto reversed = update_contacts({changed}, {contact});
        REQUIRE(contact_operator(reversed.columns).isApprox(6.0 * rank_one, 1e-12));
        changed.q[0]            = Vector3::UnitY();
        changed.q[1]            = -Vector3::UnitY();
        auto            rotated = update_contacts({changed}, {contact});
        Eigen::VectorXd r       = Eigen::VectorXd::Zero(6);
        r[1]                    = 1;
        r[4]                    = -1;
        REQUIRE(contact_operator(rotated.columns).isApprox(10.0 * r * r.transpose(), 1e-12));
    }
    SECTION("changed barycentric weights require a fresh column despite unchanged normal and norm")
    {
        auto base         = contact;
        base.kind         = 0;
        base.count        = 4;
        base.vertices     = Vector4i{17, 18, 19, 20};
        base.feature      = Vector4i::Ones();
        base.q[0]         = Vector3::UnitZ();
        base.q[1]         = -0.2 * Vector3::UnitZ();
        base.q[2]         = -0.3 * Vector3::UnitZ();
        base.q[3]         = -0.5 * Vector3::UnitZ();
        auto changed      = base;
        changed.stiffness = 10;
        changed.q[1]      = base.q[2];
        changed.q[2]      = base.q[1];
        Eigen::VectorXd old_q(12), new_q(12);
        for(int j = 0; j < 4; ++j)
        {
            old_q.segment<3>(3 * j) = base.q[j];
            new_q.segment<3>(3 * j) = changed.q[j];
        }
        REQUIRE(old_q.squaredNorm() == Catch::Approx(new_q.squaredNorm()).epsilon(1e-14));
        REQUIRE(changed.q[0].isApprox(base.q[0], 1e-14));
        REQUIRE(old_q.dot(new_q) / old_q.norm() / new_q.norm()
                > std::cos(25.0 * std::acos(-1.0) / 180.0));
        auto fresh = update_contacts({changed}, {base}, {0, 0, 0, 0});
        REQUIRE(fresh.error == 0);
        REQUIRE(contact_operator(fresh.columns, 4).isApprox(10.0 * new_q * new_q.transpose(), 1e-12));

        auto reversed      = base;
        reversed.stiffness = 10;
        for(auto& block : reversed.q)
            block *= -1;
        auto delta = update_contacts({reversed}, {base}, {0, 0, 0, 0});
        REQUIRE(delta.error == 0);
        REQUIRE(contact_operator(delta.columns, 4).isApprox(6.0 * old_q * old_q.transpose(), 1e-12));
    }
    SECTION("fixed vertices contribute no update direction and unsupported contacts request restart")
    {
        auto result = update_contacts({pair_contact(10)}, {contact}, {0, 1});
        Eigen::MatrixXd expected = Eigen::MatrixXd::Zero(6, 6);
        expected(0, 0)           = 6;
        REQUIRE(result.error == 0);
        REQUIRE(contact_operator(result.columns).isApprox(expected, 1e-12));
        auto unsupported  = contact;
        unsupported.valid = 2;
        REQUIRE(update_contacts({unsupported}, {contact}).error == 2);
    }
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

TEST_CASE("MAS-PNCG local solve matches the complete dense updated operator", "[cuda][mas_pncg]")
{
    bool enormous   = false;
    bool cancelling = false;
    SECTION("coupled blocks, permuted partition slots and padded identity") {}
    SECTION("axis-aligned enormous updates recover from Woodbury cancellation")
    {
        enormous = true;
    }
    SECTION("implicit residual retains identity next to cancelling large entries")
    {
        cancelling = true;
    }
    Dense48            A = Dense48::Identity();
    std::vector<Float> rhs{1, -2, 3, 4, -5, 6};
    std::vector<int>   map(16, -1);
    map[0] = 1;
    map[1] = 0;
    std::vector<IPCNormalSurrogate> updates;
    if(cancelling)
    {
        rhs          = {1, 1, 0, 0, 0, 0};
        auto column  = pair_contact(1);
        column.count = 1;
        column.q[0]  = Vector3{1e8, -1e8, 0};
        column.q[1].setZero();
        updates.push_back(column);
    }
    else if(enormous)
    {
        A.topLeftCorner<6, 6>().diagonal() << 2, 3, 4, 5, 6, 7;
        for(int axis = 0; axis < 3; ++axis)
        {
            auto column  = pair_contact(1);
            column.count = 1;
            column.q[0]  = 1e12 * Vector3::Unit(axis);
            column.q[1].setZero();
            updates.push_back(column);
        }
    }
    else
    {
        Eigen::Matrix<double, 6, 6> R;
        for(int i = 0; i < 6; ++i)
            for(int j = 0; j < 6; ++j)
                R(i, j) = (i == j ? 1.0 : 0.0) + 0.03 * (i + 1) * (j + 2);
        A.topLeftCorner<6, 6>() =
            R.transpose() * R + Eigen::Matrix<double, 6, 6>::Identity();
        for(int axis = 0; axis < 3; ++axis)
        {
            auto column = pair_contact(1);
            column.q[0] = (axis + 1.0) * Vector3::Unit(axis);
            column.q[1] = -0.4 * column.q[0];
            updates.push_back(column);
        }
    }
    auto matrices = upload(std::vector{pack(A)});
    auto inverses = upload(std::vector{pack(A.ldlt().solve(Dense48::Identity()))});
    auto part_to_real = upload(map);
    auto real_to_part = upload(std::vector<int>{1, 0});
    auto columns      = upload(updates);
    auto selected     = upload(cancelling ? std::vector<IndexT>{0, -1, -1, -1} :
                                        std::vector<IndexT>{2, 0, 1, -1});
    auto gradient     = upload_vector(rhs);
    std::vector<Float>    initial(6, enormous || cancelling ? 0.0 : 0.125);
    auto                  z = upload_vector(initial);
    ct::DeviceVar<IndexT> error(0), fallbacks(0);
    MASPreconditionerEngine::FrozenLevel0View base{
        matrices.cview(), inverses.cview(), part_to_real.cview(), real_to_part.cview(), 1, 2, 16, 7};
    pncg::local_solve_kernel<<<1, 64>>>(base,
                                        columns.cview(),
                                        selected.cview(),
                                        gradient.cview(),
                                        z.view(),
                                        17,
                                        4,
                                        error.view(),
                                        fallbacks.view());
    REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
    REQUIRE(static_cast<IndexT>(error) == 0);
    if(enormous)
        REQUIRE(static_cast<IndexT>(fallbacks) == 1);
    if(cancelling)
        REQUIRE(static_cast<IndexT>(fallbacks) == 0);

    Eigen::Matrix<double, 48, Eigen::Dynamic> U(48, updates.size());
    U.setZero();
    Eigen::Matrix<double, 48, 1> g = Eigen::Matrix<double, 48, 1>::Zero();
    for(int vertex = 0; vertex < 2; ++vertex)
    {
        g.segment<3>(3 * vertex) =
            Eigen::Map<const Vector3>(rhs.data() + 3 * map[vertex]);
        for(int column = 0; column < updates.size(); ++column)
            for(int j = 0; j < updates[column].count; ++j)
                if(updates[column].vertices[j] - 17 == map[vertex])
                    U.block<3, 1>(3 * vertex, column) += updates[column].q[j];
    }
    Eigen::Matrix<double, 48, 1> reference;
    if(cancelling)
    {
        REQUIRE((U.transpose() * g).isZero(0));
        reference = g;
    }
    else
    {
        Dense48 updated = A + U * U.transpose();
        reference       = updated.ldlt().solve(g);
    }
    std::vector<Float> actual;
    z.copy_to(actual);
    for(int vertex = 0; vertex < 2; ++vertex)
        for(int component = 0; component < 3; ++component)
        {
            int index = 3 * map[vertex] + component;
            double expected = initial[index] + reference[3 * vertex + component];
            CAPTURE(enormous, cancelling, index, actual[index], expected);
            REQUIRE(std::abs(actual[index] - expected) < 1e-12 * std::abs(expected) + 1e-38);
        }
}

TEST_CASE("MAS-PNCG local residuals ignore fixed and unrelated vertices", "[cuda][mas_pncg]")
{
    auto evaluate = [](bool append)
    {
        std::vector<Float>  initial{6, 8, 0, 900, 900, 900};
        std::vector<Float>  current{0.6, 0.8, 0, 900, 900, 900};
        std::vector<Float>  mass{2, 7}, length{3, 4};
        std::vector<IndexT> fixed{0, 1};
        if(append)
        {
            initial.insert(initial.end(), {1e12, 0, 0});
            current.insert(current.end(), {1e12, 0, 0});
            mass.push_back(1e9);
            length.push_back(1e9);
            fixed.push_back(0);
        }
        auto                    g      = upload_vector(initial);
        auto                    masses = upload(mass), lengths = upload(length);
        auto                    fixed_vertices = upload(fixed);
        ct::DeviceBuffer<Float> eta0(mass.size()), terms(mass.size()),
            ratios(mass.size()), corrections(mass.size());
        auto residual = [&](bool initialize)
        {
            pncg::residual_kernel<<<1, 64>>>(g.cview(),
                                             masses.cview(),
                                             fixed_vertices.cview(),
                                             lengths.cview(),
                                             eta0.view(),
                                             terms.view(),
                                             ratios.view(),
                                             corrections.view(),
                                             1e-4,
                                             1e-2,
                                             initialize);
        };
        residual(true);
        g.buffer_view().copy_from(current.data());
        residual(false);
        REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
        std::vector<Float> result;
        ratios.copy_to(result);
        REQUIRE(result[0] == Catch::Approx(10.0).epsilon(1e-12));
        REQUIRE(result[1] == 0.0);
        std::vector<Float> norms;
        terms.copy_to(norms);
        REQUIRE(norms[0] == Catch::Approx(0.5).epsilon(1e-12));
        REQUIRE(norms[1] == 0.0);
        pncg::mask_kernel<<<1, 64>>>(g.view(), fixed_vertices.cview());
        REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
        std::vector<Float> masked;
        g.copy_to(masked);
        for(int i = 0; i < 3; ++i)
        {
            REQUIRE(masked[i] == current[i]);
            REQUIRE(masked[3 + i] == 0.0);
        }
        return result;
    };
    auto small    = evaluate(false);
    auto extended = evaluate(true);
    REQUIRE(small[0] == extended[0]);
    REQUIRE(small[1] == extended[1]);
}
}  // namespace

namespace
{
TEST_CASE("MAS-PNCG zero-gradient detection preserves nonzero components below norm resolution",
          "[cuda][mas_pncg][paper]")
{
    ct::DeviceDenseVector<Float> gradient;
    gradient.resize_discard(6);
    std::vector<Float> values{0, 0, 0, 1e-170, 0, 0};
    gradient.buffer_view().copy_from(values.data());
    std::vector<IndexT>   flags{0, 0};
    auto                  fixed = upload(flags);
    ct::DeviceVar<IndexT> nonzero(0);
    pncg::nonzero_gradient_kernel<<<1, 32>>>(
        gradient.cview(), fixed.cview(), nonzero.view());
    REQUIRE(static_cast<IndexT>(nonzero) == 1);
    flags[1] = 1;
    fixed.view().copy_from(flags.data());
    nonzero = 0;
    pncg::nonzero_gradient_kernel<<<1, 32>>>(
        gradient.cview(), fixed.cview(), nonzero.view());
    REQUIRE(static_cast<IndexT>(nonzero) == 0);
}
}  // namespace
