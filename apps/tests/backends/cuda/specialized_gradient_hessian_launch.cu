#include <app/app.h>
#include <utils/specialized_gradient_hessian_launch.h>
#include <finite_element/constitutions/stable_neo_hookean_3d_function.h>
#include <finite_element/constitutions/strain_limiting_baraff_witkin_shell_2d.h>
#include <finite_element/fem_utils.h>
#include <Eigen/SVD>
#include <affine_body/constitutions/affine_body_spherical_joint_function.h>
#include <array>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

using namespace uipc;
using namespace uipc::backend::cuda;
using namespace uipc::backend::cuda_tool;

namespace
{
constexpr Float Sentinel = 1234567.25, Dt = 0.125;
constexpr int   Guard = -777;
enum class Kind
{
    SNH,
    BW,
    ARAP,
    Sphere,
    Normal,
    Friction,
    FEM1,
    FEM2,
    ABD1,
    ABD2
};
constexpr std::array Names{"stable_neo_hookean",
                           "baraff_witkin",
                           "abd_arap",
                           "spherical_joint",
                           "half_plane_normal",
                           "half_plane_friction",
                           "fem_bdf1",
                           "fem_bdf2",
                           "abd_bdf1",
                           "abd_bdf2"};

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
    void check(bool untouched_h) const
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
                REQUIRE(cols[i] >= rows[i]);
                REQUIRE(cols[i] < vertices);
                REQUIRE(blocks[i].allFinite());
            }
    }
};

struct Fixture
{
    Kind                  kind;
    int                   n, nv;
    std::vector<Vector4i> tet;
    std::vector<Vector3i> tri;
    std::vector<Vector2i> pairs;
    std::vector<Vector3>  x, previous;
    std::vector<Vector12> q, qt;
    std::vector<Float> mu, lambda, stretch, rate, mass, kappa, volume, thickness, dhat;
    std::vector<IndexT>              fixed;
    std::vector<ABDJacobiDyadicMass> body_mass;
    std::vector<Vector6>             anchors;
    DeviceBuffer<Vector4i>           dtet;
    DeviceBuffer<Vector3i>           dtri;
    DeviceBuffer<Vector2i>           dpairs;
    DeviceBuffer<Vector3>            dx, dprevious, plane, normal;
    DeviceBuffer<Vector12>           dq, dqt, dg;
    DeviceBuffer<Matrix12x12>        dense_h;
    DeviceBuffer<Matrix3x3>          dm3;
    DeviceBuffer<Matrix2x2>          dm2;
    DeviceBuffer<Float> dmu, dlambda, dstretch, drate, dmass, dkappa, dvolume,
        dthickness, ddhat;
    DeviceBuffer<IndexT>              dfixed, contacts;
    DeviceBuffer<ABDJacobiDyadicMass> dbody_mass;
    DeviceBuffer<Vector6>             danchors;
    DeviceBuffer2D<ContactCoeff>      table{Extent2D{1, 1}};
    SparseOutput<3>                   out3;
    SparseOutput<12>                  out12;

    static int stencil(Kind k)
    {
        return k == Kind::SNH ? 4 : k == Kind::BW ? 3 : 1;
    }
    bool dense() const
    {
        return kind == Kind::ARAP || kind == Kind::ABD1 || kind == Kind::ABD2;
    }
    bool sparse12() const { return kind == Kind::Sphere; }
    Fixture(Kind k, int count)
        : kind(k)
        , n(count)
        , nv(7 + 4 * count)
        , tet(n)
        , tri(n)
        , pairs(n)
        , x(nv, Vector3::Zero())
        , previous(x)
        , q(nv, Vector12::Zero())
        , qt(q)
        , mu(n, 1.1)
        , lambda(n, 1.7)
        , stretch(n, 2.2)
        , rate(n, 0.9)
        , mass(nv)
        , kappa(nv)
        , volume(nv)
        , thickness(nv, 0.001)
        , dhat(nv, 0.1)
        , fixed(nv)
        , body_mass(nv)
        , anchors(n)
        , out3(nv,
               (k == Kind::Sphere || k == Kind::ARAP || k == Kind::ABD1 || k == Kind::ABD2) ?
                   0 :
                   n * stencil(k),
               (k == Kind::Sphere || k == Kind::ARAP || k == Kind::ABD1 || k == Kind::ABD2) ?
                   0 :
                   n * stencil(k) * (stencil(k) + 1) / 2)
        , out12(nv, k == Kind::Sphere ? 2 * n : 0, k == Kind::Sphere ? 3 * n : 0)
    {
        for(int v = 0; v < nv; ++v)
        {
            mass[v]      = 1.0 + 0.01 * (v % 7);
            kappa[v]     = 1.2 + 0.02 * (v % 5);
            volume[v]    = 0.7 + 0.01 * (v % 3);
            fixed[v]     = (v % 3) == 0;
            body_mass[v] = ABDJacobiDyadicMass::from_dyadic_mass(
                mass[v], Vector3::Zero(), Matrix3x3::Identity() * 0.2);
            q[v] << 0.03 * (v % 7), 0.01, -0.02, 1.1, 0.03, 0.0, -0.02, 0.9,
                0.01, 0.0, 0.02, 1.05;
            qt[v]       = q[v] - Vector12::Constant(0.005);
            x[v]        = {0.1 * (v % 7), 0.03 + 0.001 * (v % 5), 0.02};
            previous[v] = x[v] - Vector3{0.0002 * (1 + v % 3), 0.0, -0.0001};
        }
        for(int i = 0; i < n; ++i)
        {
            int b  = 7 + 4 * i;
            tet[i] = {b + 2, b, b + 3, b + 1};
            tri[i] = {b + 2, b, b + 3};
            pairs[i] = kind == Kind::Sphere ? Vector2i{b + 1, b} : Vector2i{b, 0};
            anchors[i] << 0.1, -0.05, 0.02, -0.03, 0.04, 0.01;
            if(kind == Kind::SNH || kind == Kind::BW)
            {
                Vector3 p{0.03 * (i % 5), 0.02 * (i % 3), 0.0};
                x[tet[i][0]] = p;
                x[tet[i][1]] = p + Vector3{1.08, 0.03, 0.01};
                x[tet[i][2]] = p + Vector3{0.01, 0.94, 0.04};
                x[tet[i][3]] = p + Vector3{0.02, -0.01, 1.03};
            }
        }
        upload(dtet, tet);
        upload(dtri, tri);
        upload(dpairs, pairs);
        upload(dx, x);
        upload(dprevious, previous);
        upload(dq, q);
        upload(dqt, qt);
        upload(dmu, mu);
        upload(dlambda, lambda);
        upload(dstretch, stretch);
        upload(drate, rate);
        upload(dmass, mass);
        upload(dkappa, kappa);
        upload(dvolume, volume);
        upload(dthickness, thickness);
        upload(ddhat, dhat);
        upload(dfixed, fixed);
        upload(dbody_mass, body_mass);
        upload(danchors, anchors);
        upload(dm3, std::vector<Matrix3x3>(n, Matrix3x3::Identity()));
        upload(dm2, std::vector<Matrix2x2>(n, Matrix2x2::Identity()));
        upload(contacts, std::vector<IndexT>(nv + 1, 0));
        upload(plane, std::vector<Vector3>{Vector3::Zero()});
        upload(normal, std::vector<Vector3>{Vector3{0, 1, 0}});
        ContactCoeff coeff{2.0, 0.4};
        table.view().copy_from(&coeff);
        reset();
    }
    void reset()
    {
        out3.reset();
        out12.reset();
        upload(dg, std::vector<Vector12>(n + 2, Vector12::Constant(Sentinel)));
        upload(dense_h, std::vector<Matrix12x12>(n + 2, Matrix12x12::Constant(Sentinel)));
    }
    void launch(bool gradient_only, bool empty_h = false)
    {
        auto g3  = out3.gradients();
        auto h3  = empty_h ? TripletMatrixView<Float, 3>{} : out3.hessians();
        auto g12 = out12.gradients();
        auto h12 = empty_h ? TripletMatrixView<Float, 12>{} : out12.hessians();
        auto gd  = dg.view(1, n);
        auto hd  = empty_h ? BufferView<Matrix12x12>{} : dense_h.view(1, n);
        switch(kind)
        {
            case Kind::SNH:
                launch_stable_neo_hookean_3d_gradient_hessian(
                    {dmu, dlambda, dtet, dx, dm3, g3, h3, dvolume, Dt, gradient_only});
                break;
            case Kind::BW:
                launch_baraff_witkin_shell_gradient_hessian(
                    {dmu, dlambda, dstretch, drate, dtri, dx, dthickness, g3, h3, dvolume, Dt, dm2, 6, gradient_only});
                break;
            case Kind::ARAP:
                launch_abd_arap_gradient_hessian(
                    {dq.view(0, n), dvolume.view(0, n), gd, hd, dkappa.view(0, n), Dt, gradient_only});
                break;
            case Kind::Sphere:
                launch_affine_body_spherical_joint_gradient_hessian(
                    {dpairs, danchors, dkappa, dbody_mass, dq, g12, h12, 3, gradient_only});
                break;
            case Kind::Normal:
                launch_half_plane_normal_gradient_hessian(
                    {g3, h3, dpairs, plane, normal, table.cview().viewer(), contacts, dx, dthickness, ddhat, nv, Dt, gradient_only});
                break;
            case Kind::Friction:
                launch_half_plane_friction_gradient_hessian(
                    {g3, h3, dpairs, plane, normal, table.cview().viewer(), contacts, dx, dprevious, dthickness, 0.01, ddhat, nv, Dt, gradient_only});
                break;
            case Kind::FEM1:
                launch_fem_bdf1_gradient_hessian({dfixed.view(0, n),
                                                  dx.view(0, n),
                                                  dprevious.view(0, n),
                                                  dmass.view(0, n),
                                                  g3,
                                                  h3,
                                                  gradient_only});
                break;
            case Kind::FEM2:
                launch_fem_bdf2_gradient_hessian({dfixed.view(0, n),
                                                  dx.view(0, n),
                                                  dprevious.view(0, n),
                                                  dmass.view(0, n),
                                                  g3,
                                                  h3,
                                                  2.25,
                                                  gradient_only});
                break;
            case Kind::ABD1:
                launch_abd_bdf1_gradient_hessian({dfixed.view(0, n),
                                                  dq.view(0, n),
                                                  dq.view(0, n),
                                                  dqt.view(0, n),
                                                  dbody_mass.view(0, n),
                                                  hd,
                                                  gd,
                                                  gradient_only});
                break;
            case Kind::ABD2:
                launch_abd_bdf2_gradient_hessian({dfixed.view(0, n),
                                                  dq.view(0, n),
                                                  dqt.view(0, n),
                                                  dbody_mass.view(0, n),
                                                  hd,
                                                  gd,
                                                  2.25,
                                                  gradient_only});
                break;
        }
    }
    std::vector<Vector3> expected3() const
    {
        std::vector<Vector3> result(out3.ng, Vector3::Zero());
        for(int i = 0; i < n; ++i)
            if(kind == Kind::SNH)
            {
                auto      ids = tet[i];
                Matrix3x3 dm  = Matrix3x3::Identity();
                auto f = fem::F(x[ids[0]], x[ids[1]], x[ids[2]], x[ids[3]], dm);
                Matrix3x3 p;
                snk1::dEdF(p, mu[i], lambda[i], f);
                auto shape = fem::tetrahedron_shape_gradients(dm);
                for(int a = 0; a < 4; ++a)
                    result[4 * i + a] = fem::project_F_gradient(
                        shape.col(a), Vector9(flatten(p) * volume[i] * Dt * Dt));
            }
            else if(kind == Kind::BW)
            {
                namespace BW  = sym::strainlimiting_baraff_witkin_shell_2d;
                auto      ids = tri[i];
                Matrix2x2 ib  = Matrix2x2::Identity();
                auto      f   = BW::Ds3x2(x[ids[0]], x[ids[1]], x[ids[2]]);
                Matrix<Float, 3, 2> p;
                BW::dEdF(p, f, Vector2{1, 0}, Vector2{0, 1}, lambda[i], stretch[i], mu[i], rate[i]);
                Vector9 g = BW::dFdX(ib).transpose() * BW::flatten(p) * volume[i] * Dt * Dt;
                for(int a = 0; a < 3; ++a)
                    result[3 * i + a] = g.segment<3>(3 * a);
            }
            else if(kind == Kind::FEM1 || kind == Kind::FEM2)
                if(!fixed[i])
                    result[i] = (kind == Kind::FEM2 ? 2.25 : 1.0) * mass[i]
                                * (x[i] - previous[i]);
        return result;
    }
    std::vector<Vector12> expected12() const
    {
        std::vector<Vector12> result(sparse12() ? 2 * n : n, Vector12::Zero());
        for(int i = 0; i < n; ++i)
            if(kind == Kind::ARAP)
            {
                Matrix3x3 f;
                for(int r = 0; r < 3; ++r)
                    f.row(r) = q[i].segment<3>(3 + 3 * r);
                Eigen::JacobiSVD<Matrix3x3> svd(f, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Matrix3x3 p =
                    2 * kappa[i] * (f - svd.matrixU() * svd.matrixV().transpose());
                for(int r = 0; r < 3; ++r)
                    for(int c = 0; c < 3; ++c)
                        result[i][3 + 3 * r + c] = p(r, c) * volume[i] * Dt * Dt;
            }
            else if(kind == Kind::Sphere)
            {
                namespace SJ          = sym::affine_body_spherical_joint;
                auto              ids = pairs[i];
                Vector3           ft, g;
                Vector<Float, 24> g24;
                Vector3 a = anchors[i].head<3>(), b = anchors[i].tail<3>();
                SJ::Ft<Float>(ft, a, q[ids[0]], b, q[ids[1]]);
                SJ::dEdFt<Float>(
                    g, kappa[i] * (body_mass[ids[0]].mass() + body_mass[ids[1]].mass()), ft);
                SJ::JtT_Gt<Float>(g24, g, a, b);
                result[2 * i]     = g24.head<12>();
                result[2 * i + 1] = g24.tail<12>();
            }
            else if(!fixed[i])
                result[i] = (kind == Kind::ABD2 ? 2.25 : 1.0)
                            * (body_mass[i] * (q[i] - qt[i]));
        return result;
    }
    void check(bool gradient_only) const
    {
        out3.check(gradient_only);
        out12.check(gradient_only);
        auto g = download(dg);
        auto h = download(dense_h);
        for(int i = 0; i < n + 2; ++i)
        {
            if(!dense() || i == 0 || i == n + 1)
                REQUIRE(g[i].isConstant(Sentinel));
            else
                REQUIRE(g[i].allFinite());
            if(!dense() || gradient_only || i == 0 || i == n + 1)
                REQUIRE(h[i].isConstant(Sentinel));
            else
            {
                REQUIRE(h[i].allFinite());
                near(h[i], h[i].transpose());
            }
        }
    }
};

template <typename T>
void append_values(Json& values, const std::vector<T>& blocks)
{
    for(const auto& block : blocks)
        for(int i = 0; i < block.size(); ++i)
            values.push_back(block.data()[i]);
}
void check_contract(Kind kind, int n)
{
    CAPTURE(Names[static_cast<int>(kind)], n);
    Fixture f(kind, n);
    f.launch(false);
    synchronize_launch();
    f.check(false);
    const auto g3  = download(f.out3.g);
    const auto g12 = download(f.out12.g);
    const auto gd  = download(f.dg);
    const auto gi3 = download(f.out3.gi), gi12 = download(f.out12.gi);
    auto       expected3  = f.expected3();
    auto       expected12 = f.expected12();
    if(kind != Kind::Normal && kind != Kind::Friction)
        for(int i = 0; i < f.out3.ng; ++i)
            near(g3[i + 1], expected3[i]);
    if(f.sparse12())
        for(int i = 0; i < 2 * n; ++i)
            near(g12[i + 1], expected12[i]);
    if(f.dense())
        for(int i = 0; i < n; ++i)
            near(gd[i + 1], expected12[i]);
    if(const auto path = std::getenv("UIPC_GRADIENT_ONLY_SNAPSHOT"); path && n == 33)
    {
        Json values = Json::array();
        append_values(values, g3);
        append_values(values, g12);
        append_values(values, gd);
        append_values(values, download(f.out3.h));
        append_values(values, download(f.out12.h));
        append_values(values, download(f.dense_h));
        std::ofstream(path, std::ios::app)
            << Json{{"kind", Names[static_cast<int>(kind)]}, {"values", values}}.dump()
            << '\n';
    }
    for(bool empty_h : {false, true})
    {
        f.reset();
        f.launch(true, empty_h);
        synchronize_launch();
        f.check(true);
        auto a = download(f.out3.g);
        auto b = download(f.out12.g);
        auto c = download(f.dg);
        REQUIRE(download(f.out3.gi) == gi3);
        REQUIRE(download(f.out12.gi) == gi12);
        for(int i = 0; i < f.out3.ng; ++i)
            near(a[i + 1], g3[i + 1]);
        for(int i = 0; i < f.out12.ng; ++i)
            near(b[i + 1], g12[i + 1]);
        if(f.dense())
            for(int i = 0; i < n; ++i)
                near(c[i + 1], gd[i + 1]);
    }
}
}  // namespace

TEST_CASE("remaining gradient-only production launches preserve derivative contracts",
          "[cuda][specialized_gradient_launch]")
{
    for(int k = 0; k < 10; ++k)
        for(int n : {0, 1, 31, 33, 257})
            check_contract(static_cast<Kind>(k), n);
}

TEST_CASE("remaining gradient-only production launch timings",
          "[.benchmark][cuda][specialized_gradient_benchmark]")
{
    const auto count = std::getenv("UIPC_GRADIENT_BENCH_ELEMENTS");
    const int  n     = count ? std::atoi(count) : 4096;
    REQUIRE(n > 0);
    for(int k = 0; k < 10; ++k)
    {
        Fixture f(static_cast<Kind>(k), n);
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
            std::cout << "GRADIENT_BENCH "
                      << Json{{"kind", Names[k]},
                              {"gradient_only", gradient_only},
                              {"elements", n},
                              {"repeats", Repeats},
                              {"us_per_launch", ms * 1000 / Repeats}}
                             .dump()
                      << '\n';
        }
    }
}
