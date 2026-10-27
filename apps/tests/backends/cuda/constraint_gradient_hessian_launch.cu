#include <app/app.h>
#include <utils/constraint_gradient_hessian_launch.h>
#include <affine_body/constitutions/affine_body_revolute_joint_function.h>
#include <affine_body/constitutions/affine_body_prismatic_joint_function.h>
#include <affine_body/constraints/external_articulation_constraint_function.h>
#include <affine_body/constitutions/joint_limit_penalty.h>
#include <affine_body/utils.h>
#include <Eigen/Geometry>
#include <array>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <numbers>
#include <vector>
using namespace uipc;
using namespace uipc::backend::cuda;
using namespace uipc::backend::cuda_tool;
namespace
{
constexpr Float Sentinel = 1234567.25, Ratio = 0.375;
constexpr int   Guard = -777;
enum class Kind
{
    Revolute,
    DrivingRevolute,
    RevoluteLimit,
    Prismatic,
    DrivingPrismatic,
    PrismaticLimit,
    Transform,
    External,
    Position
};
constexpr std::array Names{"revolute",
                           "driving_revolute",
                           "revolute_limit",
                           "prismatic",
                           "driving_prismatic",
                           "prismatic_limit",
                           "soft_transform",
                           "external_articulation",
                           "soft_position"};
namespace RJ  = sym::affine_body_revolute_joint;
namespace DRJ = sym::affine_body_driving_revolute_joint;
namespace RL  = sym::affine_body_revolute_joint_limit;
namespace PJ  = sym::affine_body_prismatic_joint;
namespace DPJ = sym::affine_body_driving_prismatic_joint;
namespace ERJ = sym::external_revolute_joint_constraint;
namespace EPJ = sym::external_prismatic_joint_constraint;
template <typename T>
void upload(DeviceBuffer<T>& dst, const std::vector<T>& src)
{
    dst.resize(src.size());
    if(!src.empty())
        dst.view().copy_from(src.data());
}
template <typename T>
std::vector<T> download(const DeviceBuffer<T>& src)
{
    std::vector<T> result(src.size());
    if(!result.empty())
        src.copy_to(result);
    return result;
}
void synchronize_launch()
{
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());
}
template <typename A, typename B>
void near(const A& actual, const B& expected)
{
    REQUIRE(actual.allFinite());
    REQUIRE(expected.allFinite());
    REQUIRE((actual - expected).cwiseAbs().maxCoeff()
            <= 2e-10 + 2e-9 * expected.cwiseAbs().maxCoeff());
}

template <int N>
struct SparseOutput
{
    int                               vertices, ng, nh;
    DeviceBuffer<int>                 gi, hr, hc;
    DeviceBuffer<Vector<Float, N>>    g;
    DeviceBuffer<Matrix<Float, N, N>> h;
    SparseOutput(int nv, int gradients, int hessians)
        : vertices(nv)
        , ng(gradients)
        , nh(hessians)
    {
        reset();
    }
    void reset()
    {
        upload(gi, std::vector<int>(ng + 2, Guard));
        upload(hr, std::vector<int>(nh + 2, Guard));
        upload(hc, std::vector<int>(nh + 2, Guard));
        upload(g, std::vector<Vector<Float, N>>(ng + 2, Vector<Float, N>::Constant(Sentinel)));
        upload(h, std::vector<Matrix<Float, N, N>>(nh + 2, Matrix<Float, N, N>::Constant(Sentinel)));
    }
    auto gradients()
    {
        return DoubletVectorView<Float, N>{vertices, ng + 2, gi.data(), g.data()}
            .subview(1, ng);
    }
    auto hessians()
    {
        return TripletMatrixView<Float, N>{
            vertices, vertices, nh + 2, hr.data(), hc.data(), h.data()}
            .subview(1, nh);
    }
    void check(bool untouched_h, bool upper_triangle = true) const
    {
        auto indices = download(gi);
        auto values  = download(g);
        for(int i : {0, ng + 1})
        {
            REQUIRE(indices[i] == Guard);
            REQUIRE(values[i].isConstant(Sentinel));
        }
        for(int i = 1; i <= ng; ++i)
        {
            REQUIRE(indices[i] >= 0);
            REQUIRE(indices[i] < vertices);
            REQUIRE(values[i].allFinite());
        }
        auto rows   = download(hr);
        auto cols   = download(hc);
        auto blocks = download(h);
        for(int i = 0; i < nh + 2; ++i)
            if(untouched_h || i == 0 || i == nh + 1)
            {
                REQUIRE(rows[i] == Guard);
                REQUIRE(cols[i] == Guard);
                REQUIRE(blocks[i].isConstant(Sentinel));
            }
            else
            {
                REQUIRE(rows[i] >= 0);
                REQUIRE(cols[i] >= 0);
                if(upper_triangle)
                    REQUIRE(cols[i] >= rows[i]);
                REQUIRE(cols[i] < vertices);
                REQUIRE(blocks[i].allFinite());
            }
    }
};

struct Fixture
{
    Kind                  kind;
    int                   n, nv, nh;
    std::vector<Vector2i> pairs, refs, joint_pairs;
    std::vector<IndexT>   indices, constrained, passive, fixed;
    std::vector<U64>      uid;
    std::vector<Float>    strength, init_angle, aim_angle, current_angle,
        init_distance, aim_distance, current_distance, lower, upper,
        theta_gradient, coupling_mass, mass;
    std::vector<Vector2>             strength2;
    std::vector<Vector6>             left_basis, right_basis, cs, ts, ns, bs;
    std::vector<Vector12>            q, previous, aim12, rest12, ref_previous;
    std::vector<Vector3>             x, previous_x, aim3;
    std::vector<ABDJacobiDyadicMass> body_mass;
    DeviceBuffer<Vector2i>           dpairs, drefs, djoint_pairs;
    DeviceBuffer<IndexT>             dindices, dconstrained, dpassive, dfixed;
    DeviceBuffer<U64>                duid;
    DeviceBuffer<Float> dstrength, dinit_angle, daim_angle, dcurrent_angle,
        dinit_distance, daim_distance, dcurrent_distance, dlower, dupper,
        dtheta_gradient, dcoupling_mass, dmass;
    DeviceBuffer<Vector2>  dstrength2;
    DeviceBuffer<Vector6>  dleft_basis, dright_basis, dcs, dts, dns, dbs;
    DeviceBuffer<Vector12> dq, dprevious, daim12, drest12, dref_previous;
    DeviceBuffer<Vector3>  dx, dprevious_x, daim3;
    DeviceBuffer<ABDJacobiDyadicMass> dbody_mass;
    SparseOutput<3>                   out3;
    SparseOutput<12>                  out12;
    Fixture(Kind k, int count, int hcount = -1)
        : kind(k)
        , n(count)
        , nv(2 * count + 5)
        , nh(hcount < 0 ? count : hcount)
        , pairs(n)
        , refs(n)
        , joint_pairs(nh)
        , indices(n)
        , constrained(n)
        , passive(n)
        , fixed(nv)
        , uid(n)
        , strength(n)
        , init_angle(n)
        , aim_angle(n)
        , current_angle(n)
        , init_distance(n)
        , aim_distance(n)
        , current_distance(n)
        , lower(n)
        , upper(n)
        , theta_gradient(n)
        , coupling_mass(nh)
        , mass(nv)
        , strength2(n)
        , left_basis(n)
        , right_basis(n)
        , cs(n)
        , ts(n)
        , ns(n)
        , bs(n)
        , q(nv)
        , previous(nv)
        , aim12(n)
        , rest12(n)
        , ref_previous(nv)
        , x(nv)
        , previous_x(nv)
        , aim3(n)
        , body_mass(nv)
        , out3(nv, k == Kind::Position ? n : 0, k == Kind::Position ? n : 0)
        , out12(nv,
                k == Kind::Position  ? 0 :
                k == Kind::Transform ? n :
                                       2 * n,
                k == Kind::Position  ? 0 :
                k == Kind::Transform ? n :
                                       3 * nh)
    {
        for(int v = 0; v < nv; ++v)
        {
            const Float angle = 0.025 * (v % 5);
            Matrix3x3   a =
                Eigen::AngleAxis<Float>(angle, Vector3::UnitX()).toRotationMatrix();
            a = a * Vector3{1.02, 0.99, 1.01}.asDiagonal();
            Matrix3x3 ap =
                Eigen::AngleAxis<Float>(angle - 0.01, Vector3::UnitX()).toRotationMatrix();
            ap                    = ap * Vector3{1.02, 0.99, 1.01}.asDiagonal();
            q[v].head<3>()        = Vector3{0.02 * (v % 7), 0.01, -0.02};
            q[v].tail<9>()        = A_to_q(a);
            previous[v]           = q[v];
            previous[v].tail<9>() = A_to_q(ap);
            previous[v].head<3>() -= Vector3{0.003, 0.001, -0.002};
            ref_previous[v] = previous[v];
            ref_previous[v][1] -= 0.002;
            x[v]          = q[v].head<3>();
            previous_x[v] = previous[v].head<3>();
            fixed[v]      = v % 3 == 0;
            mass[v]       = 1.0 + 0.05 * (v % 7);
            body_mass[v]  = ABDJacobiDyadicMass::from_dyadic_mass(
                mass[v], Vector3{0.01, -0.02, 0.03}, Matrix3x3::Identity() * 0.2);
        }
        for(int i = 0; i < n; ++i)
        {
            pairs[i]            = {4 + 2 * i, 3 + 2 * i};
            indices[i]          = 3 + 2 * i;
            constrained[i]      = i % 4 != 0;
            passive[i]          = i % 3 == 1;
            strength[i]         = 1.5 + 0.03 * (i % 5);
            strength2[i]        = {strength[i], 0.7 + 0.02 * (i % 7)};
            init_angle[i]       = 0.03;
            Float winding       = (i % 7 == 6) ? 4 * std::numbers::pi : 0;
            current_angle[i]    = init_angle[i] + winding + 0.04;
            aim_angle[i]        = init_angle[i] + winding + 0.08;
            init_distance[i]    = 0.02;
            current_distance[i] = 0.035;
            aim_distance[i]     = 0.09;
            left_basis[i] << 0, 1, 0, 0, 0, 1;
            right_basis[i] = left_basis[i];
            cs[i] << 0.01, -0.02, 0.03, -0.01, 0.02, -0.03;
            ts[i] << 1, 0, 0, 1, 0, 0;
            ns[i] << 0, 1, 0, 0, 1, 0;
            bs[i] << 0, 0, 1, 0, 0, 1;
            rest12[i] << 0.01, 0.02, -0.01, 1.01, 0.02, -0.01, -0.01, 0.01,
                0.02, 0.99, 0.01, 0.02;
            aim12[i] = q[indices[i]] + Vector12::Constant(0.01);
            aim3[i]  = x[indices[i]] + Vector3{0.02, -0.01, 0.015};
            refs[i]  = {(i % 2) ? pairs[i][0] : -1, (i % 3) ? pairs[i][1] : -1};
            uid[i]   = (i % 2) ? 456 : 123;
            theta_gradient[i] = 0.4 + 0.02 * (i % 5);
            if(kind == Kind::External && uid[i] == 456)
            {
                left_basis[i].head<3>()  = cs[i].head<3>();
                right_basis[i].head<3>() = cs[i].tail<3>();
                left_basis[i].tail<3>()  = Vector3::UnitX();
                right_basis[i].tail<3>() = Vector3::UnitX();
            }
            Float value = 0;
            if(kind == Kind::RevoluteLimit)
            {
                Float delta;
                RL::DeltaTheta<Float>(delta,
                                      left_basis[i],
                                      q[pairs[i][0]],
                                      previous[pairs[i][0]],
                                      right_basis[i],
                                      q[pairs[i][1]],
                                      previous[pairs[i][1]]);
                value = current_angle[i] - init_angle[i] + delta;
            }
            else
            {
                Vector9 f;
                DPJ::F01_q<Float>(f,
                                  Vector3(cs[i].head<3>()),
                                  Vector3(ts[i].head<3>()),
                                  q[pairs[i][0]],
                                  Vector3(cs[i].tail<3>()),
                                  Vector3(ts[i].tail<3>()),
                                  q[pairs[i][1]]);
                DPJ::Distance<Float>(value, f);
            }
            Float init = kind == Kind::RevoluteLimit ? init_angle[i] : init_distance[i];
            Float offset = i % 3 == 0 ? 0.3 : i % 3 == 1 ? 0.0 : -0.3;
            lower[i]     = init + value + offset - 0.1;
            upper[i]     = init + value + offset + 0.1;
        }
        for(int i = 0; i < nh; ++i)
        {
            REQUIRE(n > 0);
            joint_pairs[i]   = {i % n, i % 3 == 0 ? i % n : (i + 1) % n};
            coupling_mass[i] = 0.8 + 0.02 * (i % 5);
        }
        upload(dpairs, pairs);
        upload(drefs, refs);
        upload(djoint_pairs, joint_pairs);
        upload(dindices, indices);
        upload(dconstrained, constrained);
        upload(dpassive, passive);
        upload(dfixed, fixed);
        upload(duid, uid);
        upload(dstrength, strength);
        upload(dinit_angle, init_angle);
        upload(daim_angle, aim_angle);
        upload(dcurrent_angle, current_angle);
        upload(dinit_distance, init_distance);
        upload(daim_distance, aim_distance);
        upload(dcurrent_distance, current_distance);
        upload(dlower, lower);
        upload(dupper, upper);
        upload(dtheta_gradient, theta_gradient);
        upload(dcoupling_mass, coupling_mass);
        upload(dmass, mass);
        upload(dstrength2, strength2);
        upload(dleft_basis, left_basis);
        upload(dright_basis, right_basis);
        upload(dcs, cs);
        upload(dts, ts);
        upload(dns, ns);
        upload(dbs, bs);
        upload(dq, q);
        upload(dprevious, previous);
        upload(daim12, aim12);
        upload(drest12, rest12);
        upload(dref_previous, ref_previous);
        upload(dx, x);
        upload(dprevious_x, previous_x);
        upload(daim3, aim3);
        upload(dbody_mass, body_mass);
    }
    Matrix12x12 constraint_mass(int i) const
    {
        int         v      = indices[i];
        const auto& m      = body_mass[v];
        Vector3     mc     = m.mass_times_x_bar();
        Matrix3x3   second = mc * mc.transpose() / m.mass();
        Matrix12x12 cm =
            ABDJacobiDyadicMass::from_dyadic_mass(m.mass(), mc, second).to_mat();
        return strength2[i][1] * m.to_mat() + (strength2[i][0] - strength2[i][1]) * cm;
    }
    Float energy(int i, const Vector12& a, const Vector12& b) const
    {
        const auto  pair = pairs[i];
        const Float kappa =
            strength[i] * (body_mass[pair[0]].mass() + body_mass[pair[1]].mass());
        Float e = 0;
        if(kind == Kind::Position)
        {
            int v = indices[i];
            if(fixed[v])
                return 0;
            Vector3 aim = (1 - Ratio) * previous_x[v] + Ratio * aim3[i];
            return 0.5 * mass[v] * strength[i] * (a.head<3>() - aim).squaredNorm();
        }
        if(kind == Kind::Transform)
        {
            int v = indices[i];
            if(fixed[v])
                return 0;
            Vector12 d = a - ((1 - Ratio) * previous[v] + Ratio * aim12[i]);
            return 0.5 * d.dot(constraint_mass(i) * d);
        }
        if(kind == Kind::Revolute)
        {
            const auto& r = rest12[i];
            Vector6     f;
            RJ::Faxis<Float>(f,
                             Vector3(r.segment<3>(0)),
                             Vector3(r.segment<3>(3)),
                             a,
                             Vector3(r.segment<3>(6)),
                             Vector3(r.segment<3>(9)),
                             b);
            RJ::Eaxis<Float>(e, kappa, f);
        }
        else if(kind == Kind::DrivingRevolute)
        {
            if(!constrained[i])
                return 0;
            Vector12 f;
            DRJ::F01_q<Float>(f,
                              Vector3(left_basis[i].head<3>()),
                              Vector3(left_basis[i].tail<3>()),
                              a,
                              Vector3(right_basis[i].head<3>()),
                              Vector3(right_basis[i].tail<3>()),
                              b);
            Float theta;
            DRJ::theta<Float>(theta, f);
            Float reference = current_angle[i] - init_angle[i];
            Float unwrapped =
                reference + std::remainder(theta - reference, 2 * std::numbers::pi);
            Float aim    = passive[i] ? current_angle[i] : aim_angle[i];
            Float target = aim - init_angle[i] - (unwrapped - theta);
            DRJ::E<Float>(e, kappa, f, target);
        }
        else if(kind == Kind::RevoluteLimit)
        {
            Float delta;
            RL::DeltaTheta<Float>(
                delta, left_basis[i], a, previous[pair[0]], right_basis[i], b, previous[pair[1]]);
            Float value = current_angle[i] - init_angle[i] + delta;
            e           = joint_limit::eval_penalty_energy<Float>(
                value, lower[i] - init_angle[i], upper[i] - init_angle[i], strength[i]);
        }
        else if(kind == Kind::Prismatic)
        {
            Vector9 f;
            PJ::F01<Float>(f,
                           Vector3(cs[i].head<3>()),
                           Vector3(ts[i].head<3>()),
                           a,
                           Vector3(cs[i].tail<3>()),
                           Vector3(ts[i].tail<3>()),
                           b);
            PJ::E01<Float>(e, kappa, f);
            Float extra;
            PJ::E23<Float>(extra,
                           kappa,
                           Vector3(ns[i].head<3>()),
                           Vector3(bs[i].head<3>()),
                           a,
                           Vector3(ns[i].tail<3>()),
                           Vector3(bs[i].tail<3>()),
                           b);
            e += extra;
        }
        else if(kind == Kind::DrivingPrismatic)
        {
            if(!constrained[i])
                return 0;
            Vector9 f;
            DPJ::F01_q<Float>(f,
                              Vector3(cs[i].head<3>()),
                              Vector3(ts[i].head<3>()),
                              a,
                              Vector3(cs[i].tail<3>()),
                              Vector3(ts[i].tail<3>()),
                              b);
            Float target = (passive[i] ? current_distance[i] : aim_distance[i])
                           - init_distance[i];
            DPJ::E01<Float>(e, kappa, f, target);
        }
        else if(kind == Kind::PrismaticLimit)
        {
            Vector9 f;
            Float   value;
            DPJ::F01_q<Float>(f,
                              Vector3(cs[i].head<3>()),
                              Vector3(ts[i].head<3>()),
                              a,
                              Vector3(cs[i].tail<3>()),
                              Vector3(ts[i].tail<3>()),
                              b);
            DPJ::Distance<Float>(value, f);
            e = joint_limit::eval_penalty_energy<Float>(value,
                                                        lower[i] - init_distance[i],
                                                        upper[i] - init_distance[i],
                                                        strength[i]);
        }
        else if(kind == Kind::External)
        {
            const Vector12 pa =
                refs[i][0] >= 0 ? ref_previous[refs[i][0]] : previous[pair[0]];
            const Vector12 pb =
                refs[i][1] >= 0 ? ref_previous[refs[i][1]] : previous[pair[1]];
            Float delta;
            if(uid[i] == 123)
                ERJ::DeltaTheta<Float>(delta, left_basis[i], a, pa, right_basis[i], b, pb);
            else
                EPJ::DeltaTheta<Float>(delta, left_basis[i], a, pa, right_basis[i], b, pb);
            e = theta_gradient[i] * delta;
        }
        return e;
    }
    void launch(bool gradient_only, bool empty_h = false)
    {
        auto g3  = out3.gradients();
        auto g12 = out12.gradients();
        auto h3  = empty_h ? TripletMatrixView<Float, 3>{} : out3.hessians();
        auto h12 = empty_h ? TripletMatrixView<Float, 12>{} : out12.hessians();
        switch(kind)
        {
            case Kind::Revolute:
                launch_revolute_joint_gradient_hessian(
                    {dpairs, drest12, dstrength, dbody_mass, dq, g12, h12, gradient_only});
                break;
            case Kind::DrivingRevolute:
                launch_driving_revolute_joint_gradient_hessian({dpairs,
                                                                dleft_basis,
                                                                dright_basis,
                                                                dconstrained,
                                                                dstrength,
                                                                dpassive,
                                                                dinit_angle,
                                                                daim_angle,
                                                                dcurrent_angle,
                                                                dq,
                                                                dbody_mass,
                                                                g12,
                                                                h12,
                                                                gradient_only});
                break;
            case Kind::RevoluteLimit:
                launch_revolute_joint_limit_gradient_hessian({dpairs,
                                                              dleft_basis,
                                                              dright_basis,
                                                              dinit_angle,
                                                              dcurrent_angle,
                                                              dlower,
                                                              dupper,
                                                              dstrength,
                                                              dq,
                                                              dprevious,
                                                              g12,
                                                              h12,
                                                              gradient_only});
                break;
            case Kind::Prismatic:
                launch_prismatic_joint_gradient_hessian(
                    {dpairs, dcs, dts, dns, dbs, dstrength, dbody_mass, dq, g12, h12, gradient_only});
                break;
            case Kind::DrivingPrismatic:
                launch_driving_prismatic_joint_gradient_hessian({dpairs,
                                                                 dconstrained,
                                                                 dpassive,
                                                                 dstrength,
                                                                 dcs,
                                                                 dts,
                                                                 dinit_distance,
                                                                 dcurrent_distance,
                                                                 daim_distance,
                                                                 dq,
                                                                 dbody_mass,
                                                                 g12,
                                                                 h12,
                                                                 gradient_only});
                break;
            case Kind::PrismaticLimit:
                launch_prismatic_joint_limit_gradient_hessian(
                    {dpairs, dcs, dts, dinit_distance, dlower, dupper, dstrength, dq, g12, h12, gradient_only});
                break;
            case Kind::Transform:
                launch_soft_transform_constraint_gradient_hessian({Ratio,
                                                                   dindices.view(),
                                                                   dq,
                                                                   dprevious,
                                                                   daim12.view(),
                                                                   dstrength2.view(),
                                                                   dbody_mass,
                                                                   g12,
                                                                   dfixed,
                                                                   h12,
                                                                   gradient_only});
                break;
            case Kind::External:
                launch_external_articulation_constraint_gradient_hessian(
                    {g12,
                     dtheta_gradient,
                     dpairs,
                     dleft_basis,
                     dright_basis,
                     dq,
                     dprevious,
                     drefs,
                     dref_previous.view(),
                     duid,
                     123,
                     456,
                     h12,
                     dcoupling_mass,
                     djoint_pairs,
                     3,
                     gradient_only});
                break;
            case Kind::Position:
                launch_soft_position_constraint_gradient_hessian({Ratio,
                                                                  dindices.view(),
                                                                  dx,
                                                                  dprevious_x,
                                                                  daim3.view(),
                                                                  dstrength.view(),
                                                                  dmass,
                                                                  g3,
                                                                  dfixed,
                                                                  h3,
                                                                  gradient_only});
                break;
        }
    }
    void check(bool gradient_only) const
    {
        out3.check(gradient_only);
        out12.check(gradient_only, kind != Kind::External);
        if(gradient_only)
            return;
        if(kind == Kind::Position)
        {
            auto h = download(out3.h);
            for(int i = 0; i < n; ++i)
            {
                int       v        = indices[i];
                Matrix3x3 expected = Matrix3x3::Zero();
                if(!fixed[v])
                    expected = mass[v] * strength[i] * Matrix3x3::Identity();
                near(h[i + 1], expected);
            }
        }
        if(kind == Kind::Transform)
        {
            auto h = download(out12.h);
            for(int i = 0; i < n; ++i)
            {
                Matrix12x12 expected = Matrix12x12::Zero();
                if(!fixed[indices[i]])
                    expected = constraint_mass(i);
                near(h[i + 1], expected);
            }
        }
        if(kind == Kind::DrivingRevolute || kind == Kind::DrivingPrismatic)
        {
            auto g = download(out12.g);
            auto h = download(out12.h);
            for(int i = 0; i < n; ++i)
                if(!constrained[i])
                {
                    for(int j = 0; j < 2; ++j)
                        REQUIRE(g[1 + 2 * i + j].isZero());
                    for(int j = 0; j < 3; ++j)
                        REQUIRE(h[1 + 3 * i + j].isZero());
                }
        }
    }
    void check_finite_difference() const
    {
        auto            g3   = download(out3.g);
        auto            g12  = download(out12.g);
        constexpr Float step = 1e-6;
        for(int i = 0; i < std::min(n, 7); ++i)
        {
            Vector12 a =
                q[kind == Kind::Transform || kind == Kind::Position ? indices[i] : pairs[i][0]];
            Vector12 b          = q[pairs[i][1]];
            int      dimensions = kind == Kind::Position  ? 3 :
                                  kind == Kind::Transform ? 12 :
                                                            24;
            for(int d = 0; d < dimensions; ++d)
            {
                Vector12 ap = a, am = a, bp = b, bm = b;
                if(d < 12)
                {
                    ap[d] += step;
                    am[d] -= step;
                }
                else
                {
                    bp[d - 12] += step;
                    bm[d - 12] -= step;
                }
                Float expected = (energy(i, ap, bp) - energy(i, am, bm)) / (2 * step);
                Float actual = kind == Kind::Position  ? g3[i + 1][d] :
                               kind == Kind::Transform ? g12[i + 1][d] :
                                                         g12[1 + 2 * i + d / 12][d % 12];
                CAPTURE(i, d, actual, expected);
                REQUIRE(std::isfinite(actual));
                REQUIRE(std::isfinite(expected));
                REQUIRE(std::abs(actual - expected) <= 2e-6 + 2e-5 * std::abs(expected));
            }
        }
    }
};
template <typename T>
void append_values(Json& values, const std::vector<T>& data)
{
    for(const auto& block : data)
        for(int i = 0; i < block.size(); ++i)
            values.push_back(block.data()[i]);
}
void check_contract(Kind kind, int n, int nh = -1)
{
    CAPTURE(Names[static_cast<int>(kind)], n, nh);
    Fixture f(kind, n, nh);
    f.launch(false);
    synchronize_launch();
    f.check(false);
    f.check_finite_difference();
    auto g3   = download(f.out3.g);
    auto g12  = download(f.out12.g);
    auto gi3  = download(f.out3.gi);
    auto gi12 = download(f.out12.gi);
    if(const char* path = std::getenv("UIPC_CONSTRAINT_DERIVATIVE_SNAPSHOT"); path && n == 33)
    {
        Json values = Json::array();
        append_values(values, g3);
        append_values(values, g12);
        append_values(values, download(f.out3.h));
        append_values(values, download(f.out12.h));
        std::ofstream(path, std::ios::app) << Json{
            {"kind", Names[static_cast<int>(kind)]},
            {"n", n},
            {"nh", f.nh},
            {"values", values}}.dump() << '\n';
    }
    for(bool empty_h : {false, true})
    {
        f.out3.reset();
        f.out12.reset();
        f.launch(true, empty_h);
        synchronize_launch();
        f.check(true);
        auto a = download(f.out3.g);
        auto b = download(f.out12.g);
        REQUIRE(download(f.out3.gi) == gi3);
        REQUIRE(download(f.out12.gi) == gi12);
        for(int i = 0; i < f.out3.ng; ++i)
            near(a[i + 1], g3[i + 1]);
        for(int i = 0; i < f.out12.ng; ++i)
            near(b[i + 1], g12[i + 1]);
    }
}
}  // namespace
TEST_CASE("joint and constraint production derivative launches preserve gradient-only contracts",
          "[cuda][constraint_gradient_launch]")
{
    for(int k = 0; k < 9; ++k)
        for(int n : {0, 1, 15, 16, 17, 31, 33, 257})
            check_contract(static_cast<Kind>(k), n);
    for(int n : {1, 17, 33})
    {
        check_contract(Kind::External, n, 0);
        check_contract(Kind::External, n, 2 * n + 1);
    }
}
TEST_CASE("joint and constraint derivative launch timings",
          "[.benchmark][cuda][constraint_gradient_benchmark]")
{
    const char* count = std::getenv("UIPC_CONSTRAINT_BENCH_ELEMENTS");
    const int   n     = count ? std::atoi(count) : 4096;
    REQUIRE(n > 0);
    for(int k = 0; k < 9; ++k)
    {
        Fixture f(static_cast<Kind>(k), n, k == 7 ? 2 * n + 1 : n);
        for(bool gradient_only : {true, false})
        {
            for(int i = 0; i < 8; ++i)
                f.launch(gradient_only);
            synchronize_launch();
            cudaEvent_t start, stop;
            CUDA_TOOL_CHECK(cudaEventCreate(&start));
            CUDA_TOOL_CHECK(cudaEventCreate(&stop));
            CUDA_TOOL_CHECK(cudaEventRecord(start));
            constexpr int Repeats = 40;
            for(int i = 0; i < Repeats; ++i)
                f.launch(gradient_only);
            CUDA_TOOL_CHECK(cudaEventRecord(stop));
            CUDA_TOOL_CHECK(cudaEventSynchronize(stop));
            float ms = 0;
            CUDA_TOOL_CHECK(cudaEventElapsedTime(&ms, start, stop));
            CUDA_TOOL_CHECK(cudaEventDestroy(start));
            CUDA_TOOL_CHECK(cudaEventDestroy(stop));
            std::cout << "CONSTRAINT_BENCH "
                      << Json{{"kind", Names[k]},
                              {"gradient_only", gradient_only},
                              {"elements", n},
                              {"hessian_elements", f.nh},
                              {"repeats", Repeats},
                              {"us_per_launch_batch", ms * 1000 / Repeats}}
                             .dump()
                      << '\n';
        }
    }
}
