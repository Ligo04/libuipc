#include <app/app.h>
#include <utils/remaining_material_launch.h>
#include <utils/codim_thickness.h>
#include <finite_element/fem_utils.h>
#include <finite_element/constitutions/arap_function.h>
#include <finite_element/constitutions/hookean_spring_1d_function.h>
#include <finite_element/constitutions/kirchhoff_rod_bending_function.h>
#include <finite_element/constitutions/strain_plastic_discrete_shell_bending_function.h>
#include <inter_primitive_effect_system/constitutions/soft_vertex_stitch_function.h>
#include <inter_primitive_effect_system/constitutions/soft_vertex_edge_stitch_function.h>
#include <inter_primitive_effect_system/constitutions/soft_vertex_triangle_stitch_function.h>
#include <affine_body/constitutions/affine_body_fixed_joint_function.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <chrono>
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
using Vector24     = Vector<Float, 24>;
using Matrix24x24  = Matrix<Float, 24, 24>;
constexpr Float Dt = 0.125, Sentinel = 1234567.25;
constexpr int   GuardIndex = -777;
enum class Kind
{
    Fixed,
    ARAP,
    Spring,
    Rod,
    Strain,
    Point,
    Edge,
    Triangle
};
constexpr std::array Names{
    "fixed_joint", "arap", "spring", "rod", "strain_bending", "point_stitch", "edge_stitch", "triangle_stitch"};

template <typename T>
void upload(DeviceBuffer<T>& dst, const std::vector<T>& src)
{
    dst.resize(src.size());
    if(!src.empty())
        dst.view().copy_from(src.data());
}
template <typename T>
auto download(const DeviceBuffer<T>& src)
{
    std::vector<T> dst(src.size());
    if(!dst.empty())
        src.copy_to(dst);
    return dst;
}
void sync()
{
    CUDA_TOOL_CHECK(cudaGetLastError());
    CUDA_TOOL_CHECK(cudaDeviceSynchronize());
}
template <typename A, typename B>
void near(const A& a, const B& b)
{
    REQUIRE(a.allFinite());
    REQUIRE(b.allFinite());
    Float scale = std::max(a.cwiseAbs().maxCoeff(), b.cwiseAbs().maxCoeff());
    REQUIRE((a - b).cwiseAbs().maxCoeff() <= 2e-11 + 2e-10 * scale);
}
template <int N>
auto project(const Matrix<Float, N, N>& raw)
{
    Eigen::SelfAdjointEigenSolver<Matrix<Float, N, N>> solver(raw);
    REQUIRE(solver.info() == Eigen::Success);
    return Matrix<Float, N, N>(solver.eigenvectors()
                               * solver.eigenvalues().cwiseMax(0).asDiagonal()
                               * solver.eigenvectors().transpose());
}

// Independent primitives have permuted, globally offset vertex IDs. The
// thickness data varies by vertex to catch primitive/vertex indexing mistakes.
struct Fixture
{
    Kind                  kind;
    int                   count, vertices;
    int                   block_size = 0;
    std::vector<Vector2i> t2;
    std::vector<Vector3i> t3;
    std::vector<Vector4i> t4;
    std::vector<Vector3>  x, rest_x;
    std::vector<Float> k, mu, lambda, length, thick, volume, theta, height, area;
    std::vector<Matrix3x3>           inverse3;
    std::vector<Matrix2x2>           inverse2;
    std::vector<Vector12>            q;
    std::vector<ABDJacobiDyadicMass> mass;
    std::vector<Vector6>             cbar, tbar, nbar, bbar;
    DeviceBuffer<Vector2i>           d2;
    DeviceBuffer<Vector3i>           d3;
    DeviceBuffer<Vector4i>           d4;
    DeviceBuffer<Vector3>            dx, drest;
    DeviceBuffer<Float> dk, dmu, dlambda, dlength, dthick, dvolume, dtheta, dheight, darea;
    DeviceBuffer<Matrix3x3>           dm3;
    DeviceBuffer<Matrix2x2>           dm2;
    DeviceBuffer<Vector12>            dq;
    DeviceBuffer<ABDJacobiDyadicMass> dmass;
    DeviceBuffer<Vector6>             dcbar, dtbar, dnbar, dbbar;

    Fixture(Kind kind_, int n)
        : kind(kind_)
        , count(n)
        , vertices(7 + 4 * n)
        , t2(n)
        , t3(n)
        , t4(n)
        , x(vertices, Vector3::Zero())
        , rest_x(x)
        , k(n, 2.0)
        , mu(n, 1.1)
        , lambda(n, 1.7)
        , length(n, 1.2)
        , thick(vertices)
        , volume(n, 0.7)
        , theta(n, 0.15)
        , height(n, 0.8)
        , area(n, 0.5)
        , inverse3(n, Matrix3x3::Identity())
        , inverse2(n, Matrix2x2::Identity())
        , q(vertices, Vector12::Zero())
        , mass(vertices)
        , cbar(n)
        , tbar(n)
        , nbar(n)
        , bbar(n)
    {
        for(int v = 0; v < vertices; ++v)
        {
            thick[v] = 0.07 + 0.01 * (v % 7);
            q[v].tail<9>() =
                Eigen::Map<const Vector9>(Matrix3x3::Identity().eval().data());
            mass[v] = ABDJacobiDyadicMass(1.0 + 0.01 * (v % 5), Vector3::Zero());
        }
        for(int i = 0; i < n; ++i)
        {
            int b = 7 + 4 * i;
            t2[i] = {b + 1, b};
            t3[i] = {b + 2, b, b + 1};
            t4[i] = {b + 2, b, b + 3, b + 1};
            k[i] += 0.01 * (i % 11);
            volume[i] += 0.001 * (i % 7);
            if(kind == Kind::Point)
                length[i] = i % 3 == 0 ? 0.0 : (i % 3 == 1 ? 0.5 : 1.5);
            if(kind == Kind::Spring || kind == Kind::Point || kind == Kind::Fixed)
            {
                x[t2[i][0]] = {0.1, 0.2, 0.0};
                x[t2[i][1]] = {1.0 + 0.03 * (i % 5), 0.3, 0.2};
                if(kind == Kind::Spring)
                    thick[t2[i][1]] = thick[t2[i][0]];
            }
            else if(kind == Kind::Rod)
            {
                for(int a = 0; a < 3; ++a)
                    rest_x[t3[i][a]] = {Float(a), 0, 0};
                x[t3[i][0]] = {0, 0.1, 0};
                x[t3[i][1]] = {1, 0.2, 0.1};
                x[t3[i][2]] = {2, 0.3 + 0.01 * (i % 5), 0.4};
            }
            else if(kind == Kind::Edge)
            {
                x[t3[i][0]] = {0, 0, 0};
                x[t3[i][1]] = {1.1, 0.1, 0.0};
                x[t3[i][2]] = {0.2, 0.9, 0.1};
            }
            else if(kind == Kind::Strain)
            {
                std::array<Vector3, 4> hinge = {Vector3{0, 1, 0.2},
                                                Vector3{-1, 0, 0},
                                                Vector3{1, 0, 0},
                                                Vector3{0, -1, -0.3}};
                if(i % 7 == 6)
                    hinge = {Vector3{0, 0, 0},
                             Vector3{1, 0, 0},
                             Vector3{2, 0, 0},
                             Vector3{3, 0, 0}};
                for(int a = 0; a < 4; ++a)
                    x[t4[i][a]] = hinge[a];
            }
            else
            {
                x[t4[i][0]] = {0, 0, 0};
                x[t4[i][1]] = {1.1, 0.1, 0};
                x[t4[i][2]] = {0.2, 0.9, 0.1};
                x[t4[i][3]] = {0.1, 0.2, 1.2};
            }
            cbar[i] << 0.1, 0.2, 0, -0.1, 0, 0.2;
            tbar[i] << 1, 0, 0, 1, 0, 0;
            nbar[i] << 0, 1, 0, 0, 1, 0;
            bbar[i] << 0, 0, 1, 0, 0, 1;
            q[t2[i][0]].head<3>() = Vector3{0.2, 0.1, 0.0};
            q[t2[i][1]].head<3>() = Vector3{0.1, -0.1, 0.3};
            q[t2[i][1]][4]        = 0.15;
            q[t2[i][1]][6]        = -0.1;
        }
        upload(d2, t2);
        upload(d3, t3);
        upload(d4, t4);
        upload(dx, x);
        upload(drest, rest_x);
        upload(dk, k);
        upload(dmu, mu);
        upload(dlambda, lambda);
        upload(dlength, length);
        upload(dthick, thick);
        upload(dvolume, volume);
        upload(dtheta, theta);
        upload(dheight, height);
        upload(darea, area);
        upload(dm3, inverse3);
        upload(dm2, inverse2);
        upload(dq, q);
        upload(dmass, mass);
        upload(dcbar, cbar);
        upload(dtbar, tbar);
        upload(dnbar, nbar);
        upload(dbbar, bbar);
    }

    template <int N>
    const auto& topology() const
    {
        if constexpr(N == 2)
            return t2;
        else if constexpr(N == 3)
            return t3;
        else
            return t4;
    }
    template <int N, int D>
    auto oracle(int i) const
    {
        using GType = Vector<Float, N * D>;
        using HType = Matrix<Float, N * D, N * D>;
        GType g;
        HType h;
        if constexpr(D == 12)
        {
            namespace FJ = sym::affine_body_fixed_joint;
            auto ids     = t2[i];
            auto qi = q[ids[0]], qj = q[ids[1]];
            Float stiffness = k[i] * (mass[ids[0]].mass() + mass[ids[1]].mass());
            Vector9     fr, gr;
            Vector3     ft, gt;
            Vector24    rotg, transg;
            Matrix9x9   rh;
            Matrix3x3   th;
            Matrix24x24 rotha, tranha;
            FJ::Fr<Float>(fr,
                          tbar[i].head<3>(),
                          nbar[i].head<3>(),
                          bbar[i].head<3>(),
                          qi,
                          tbar[i].tail<3>(),
                          nbar[i].tail<3>(),
                          bbar[i].tail<3>(),
                          qj);
            FJ::Ft<Float>(ft, cbar[i].head<3>(), qi, cbar[i].tail<3>(), qj);
            FJ::dEdFr(gr, stiffness, fr);
            FJ::dEdFt(gt, stiffness, ft);
            FJ::JrT_Gr<Float>(rotg,
                              gr,
                              tbar[i].head<3>(),
                              nbar[i].head<3>(),
                              bbar[i].head<3>(),
                              qi,
                              tbar[i].tail<3>(),
                              nbar[i].tail<3>(),
                              bbar[i].tail<3>(),
                              qj);
            FJ::JtT_Gt<Float>(transg, gt, cbar[i].head<3>(), cbar[i].tail<3>());
            FJ::ddEdFr(rh, stiffness, fr);
            FJ::ddEdFt(th, stiffness, ft);
            rh = project<9>(rh);
            th = project<3>(th);
            FJ::JrT_Hr_Jr<Float>(rotha,
                                 rh,
                                 tbar[i].head<3>(),
                                 nbar[i].head<3>(),
                                 bbar[i].head<3>(),
                                 qi,
                                 tbar[i].tail<3>(),
                                 nbar[i].tail<3>(),
                                 bbar[i].tail<3>(),
                                 qj);
            FJ::JtT_Ht_Jt<Float>(tranha, th, cbar[i].head<3>(), cbar[i].tail<3>());
            g = rotg + transg;
            h = rotha + tranha;
        }
        else
        {
            Vector<Float, N * 3> X;
            for(int a = 0; a < N; ++a)
                X.template segment<3>(3 * a) = x[topology<N>()[i][a]];
            if constexpr(N == 2)
            {
                if(kind == Kind::Spring)
                {
                    namespace S = sym::hookean_spring_1d;
                    S::dEdX(g, k[i], X, length[i]);
                    S::ddEddX(h, k[i], X, length[i]);
                    Float r = edge_thickness(thick[t2[i][0]], thick[t2[i][1]]);
                    Float scale = length[i] * r * r * std::numbers::pi * Dt * Dt;
                    g *= scale;
                    h *= scale;
                }
                else if(length[i] == 0)
                {
                    Vector3 delta = k[i] * Dt * Dt
                                    * (X.template head<3>() - X.template tail<3>());
                    g.template head<3>() = delta;
                    g.template tail<3>() = -delta;
                    Matrix3x3 block = k[i] * Dt * Dt * Matrix3x3::Identity();
                    h.template block<3, 3>(0, 0) = block;
                    h.template block<3, 3>(0, 3) = -block;
                    h.template block<3, 3>(3, 0) = -block;
                    h.template block<3, 3>(3, 3) = block;
                }
                else
                {
                    sym::soft_vertex_stitch::dEdX(g, k[i] * Dt * Dt, X, length[i]);
                    sym::soft_vertex_stitch::ddEddX(h, k[i] * Dt * Dt, X, length[i]);
                }
                h = project<N * 3>(h);
            }
            if constexpr(N == 3)
            {
                if(kind == Kind::Rod)
                {
                    Float L = (rest_x[t3[i][1]] - rest_x[t3[i][0]]).norm()
                              + (rest_x[t3[i][2]] - rest_x[t3[i][1]]).norm();
                    sym::kirchhoff_rod_bending::dEdX(
                        g, k[i], X, L, thick[t3[i][1]], std::numbers::pi);
                    sym::kirchhoff_rod_bending::ddEddX(
                        h, k[i], X, L, thick[t3[i][1]], std::numbers::pi);
                    g *= Dt * Dt;
                    h *= Dt * Dt;
                    h = project<9>(h);
                }
                else
                {
                    sym::soft_vertex_edge_stitch::dEdX(g, lambda[i], mu[i], X, inverse2[i]);
                    sym::soft_vertex_edge_stitch::ddEddX(h, lambda[i], mu[i], X, inverse2[i]);
                    Float scale = area[i] * 2 * thick[i] * Dt * Dt;
                    g *= scale;
                    h = project<9>(h) * scale;
                }
            }
            if constexpr(N == 4)
            {
                if(kind == Kind::Strain)
                {
                    namespace S = sym::strain_plastic_discrete_shell_bending;
                    S::dEdx(g,
                            X.template segment<3>(0),
                            X.template segment<3>(3),
                            X.template segment<3>(6),
                            X.template segment<3>(9),
                            length[i],
                            height[i],
                            theta[i],
                            k[i]);
                    S::ddEddx(h,
                              X.template segment<3>(0),
                              X.template segment<3>(3),
                              X.template segment<3>(6),
                              X.template segment<3>(9),
                              length[i],
                              height[i],
                              theta[i],
                              k[i]);
                    g *= volume[i] * Dt * Dt;
                    h *= volume[i] * Dt * Dt;
                    h = project<12>(h);
                }
                else
                {
                    Matrix3x3 F =
                        fem::F(x[t4[i][0]], x[t4[i][1]], x[t4[i][2]], x[t4[i][3]], inverse3[i]);
                    Vector9    grad;
                    Matrix9x9  raw;
                    Matrix9x12 J = fem::dFdx(inverse3[i]);
                    if(kind == Kind::ARAP)
                    {
                        sym::arap_3d::dEdF(grad, k[i] * Dt * Dt, volume[i], F);
                        sym::arap_3d::ddEddF(raw, k[i] * Dt * Dt, volume[i], F);
                    }
                    else
                    {
                        Vector9 vec = Eigen::Map<const Vector9>(F.data());
                        sym::soft_vertex_triangle_stitch::dEdVecF(grad, mu[i], lambda[i], vec);
                        sym::soft_vertex_triangle_stitch::ddEddVecF(raw, mu[i], lambda[i], vec);
                        grad *= volume[i] * Dt * Dt;
                        raw *= volume[i] * Dt * Dt;
                    }
                    g = J.transpose() * grad;
                    h = J.transpose() * project<9>(raw) * J;
                }
            }
        }
        return std::pair<GType, HType>{g, h};
    }

    template <typename Info, typename Launch>
    void dispatch(Launch fn, Info info)
    {
        info.block_size = block_size;
        fn(info);
    }
    template <int D>
    void launch(bool gradient, DoubletVectorView<Float, D> g, TripletMatrixView<Float, D> h)
    {
        if constexpr(D == 12)
            dispatch<AffineBodyFixedJointGradientHessianLaunchInfo>(
                launch_affine_body_fixed_joint_gradient_hessian,
                {d2.cview(),
                 dcbar.cview(),
                 dtbar.cview(),
                 dnbar.cview(),
                 dbbar.cview(),
                 dk.cview(),
                 dmass.cview(),
                 dq.cview(),
                 g,
                 h,
                 gradient,
                 3});
        else
            switch(kind)
            {
                case Kind::ARAP:
                    dispatch<ARAP3DGradientHessianLaunchInfo>(launch_arap_3d_gradient_hessian,
                                                              {dk.cview(),
                                                               d4.cview(),
                                                               dx.cview(),
                                                               dm3.cview(),
                                                               g,
                                                               h,
                                                               dvolume.cview(),
                                                               Dt,
                                                               gradient});
                    break;
                case Kind::Spring:
                    dispatch<HookeanSpring1DGradientHessianLaunchInfo>(
                        launch_hookean_spring_1d_gradient_hessian,
                        {g,
                         h,
                         dk.cview(),
                         dlength.cview(),
                         dthick.cview(),
                         d2.cview(),
                         dx.cview(),
                         Dt,
                         std::numbers::pi,
                         gradient});
                    break;
                case Kind::Rod:
                    dispatch<KirchhoffRodBendingGradientHessianLaunchInfo>(
                        launch_kirchhoff_rod_bending_gradient_hessian,
                        {d3.view(), dk.view(), dthick.cview(), dx.cview(), drest.cview(), g, h, Dt, std::numbers::pi, gradient});
                    break;
                case Kind::Strain:
                    dispatch<StrainPlasticBendingGradientHessianLaunchInfo>(
                        launch_strain_plastic_bending_gradient_hessian,
                        {d4.view(),
                         dk.view(),
                         dtheta.view(),
                         dheight.view(),
                         dvolume.view(),
                         dlength.view(),
                         dx.cview(),
                         g,
                         h,
                         Dt,
                         gradient});
                    break;
                case Kind::Point:
                    dispatch<SoftVertexStitchGradientHessianLaunchInfo>(
                        launch_soft_vertex_stitch_gradient_hessian,
                        {d2.cview(), dx.cview(), dk.cview(), dlength.cview(), g, h, Dt, gradient});
                    break;
                case Kind::Edge:
                    dispatch<SoftVertexEdgeStitchGradientHessianLaunchInfo>(
                        launch_soft_vertex_edge_stitch_gradient_hessian,
                        {d3.cview(),
                         dx.cview(),
                         dmu.cview(),
                         dlambda.cview(),
                         dm2.cview(),
                         darea.cview(),
                         dthick.cview(),
                         g,
                         h,
                         Dt,
                         gradient});
                    break;
                case Kind::Triangle:
                    dispatch<SoftVertexTriangleStitchGradientHessianLaunchInfo>(
                        launch_soft_vertex_triangle_stitch_gradient_hessian,
                        {d4.cview(),
                         dx.cview(),
                         dmu.cview(),
                         dlambda.cview(),
                         dm3.cview(),
                         dvolume.cview(),
                         g,
                         h,
                         Dt,
                         gradient});
                    break;
                default:
                    FAIL("Unsupported material fixture");
            }
    }
};

template <int N, int D>
struct Output
{
    int                               n, vertices;
    DeviceBuffer<int>                 gi, hr, hc;
    DeviceBuffer<Vector<Float, D>>    g;
    DeviceBuffer<Matrix<Float, D, D>> h;
    Output(int count, int nv)
        : n(count)
        , vertices(nv)
    {
        reset();
    }
    void reset()
    {
        upload(gi, std::vector<int>(n * N + 2, GuardIndex));
        upload(hr, std::vector<int>(n * N * (N + 1) / 2 + 2, GuardIndex));
        upload(hc, std::vector<int>(n * N * (N + 1) / 2 + 2, GuardIndex));
        upload(g, std::vector<Vector<Float, D>>(n * N + 2, Vector<Float, D>::Constant(Sentinel)));
        upload(h,
               std::vector<Matrix<Float, D, D>>(n * N * (N + 1) / 2 + 2,
                                                Matrix<Float, D, D>::Constant(Sentinel)));
    }
    auto gv()
    {
        return DoubletVectorView<Float, D>{vertices, n * N + 2, gi.data(), g.data()}
            .subview(1, n * N);
    }
    auto hv()
    {
        return TripletMatrixView<Float, D>{
            vertices, vertices, n * N * (N + 1) / 2 + 2, hr.data(), hc.data(), h.data()}
            .subview(1, n * N * (N + 1) / 2);
    }
    void guards(bool untouched)
    {
        auto ids    = download(gi);
        auto values = download(g);
        for(int s : {0, n * N + 1})
        {
            REQUIRE(ids[s] == GuardIndex);
            REQUIRE(values[s].isConstant(Sentinel));
        }
        auto rows   = download(hr);
        auto cols   = download(hc);
        auto blocks = download(h);
        for(int s = 0; s < int(blocks.size()); ++s)
            if(untouched || s == 0 || s + 1 == int(blocks.size()))
            {
                REQUIRE(rows[s] == GuardIndex);
                REQUIRE(cols[s] == GuardIndex);
                REQUIRE(blocks[s].isConstant(Sentinel));
            }
    }
};

template <int N, int D>
void validate(Fixture& f)
{
    Output<N, D> out(f.count, f.vertices);
    f.launch<D>(false, out.gv(), out.hv());
    sync();
    out.guards(false);
    auto gi   = download(out.gi);
    auto g    = download(out.g);
    auto rows = download(out.hr);
    auto cols = download(out.hc);
    auto h    = download(out.h);
    for(int i = 0; i < f.count; ++i)
    {
        auto [expected_g, expected_h] = f.oracle<N, D>(i);
        auto ids                      = f.topology<N>()[i];
        int  slot                     = 1 + i * N * (N + 1) / 2;
        for(int a = 0; a < N; ++a)
        {
            REQUIRE(gi[1 + i * N + a] == ids[a]);
            near(g[1 + i * N + a], expected_g.template segment<D>(a * D));
            for(int b = a; b < N; ++b, ++slot)
            {
                REQUIRE(rows[slot] == std::min(ids[a], ids[b]));
                REQUIRE(cols[slot] == std::max(ids[a], ids[b]));
                Matrix<Float, D, D> block =
                    expected_h.template block<D, D>(a * D, b * D);
                if(ids[a] > ids[b])
                    block.transposeInPlace();
                near(h[slot], block);
            }
        }
    }
    auto history = download(f.dtheta);
    for(bool null_h : {true, false})
    {
        out.reset();
        f.launch<D>(true, out.gv(), null_h ? TripletMatrixView<Float, D>{} : out.hv());
        sync();
        out.guards(true);
        auto gradient_indices = download(out.gi);
        auto gradient         = download(out.g);
        for(int s = 1; s <= f.count * N; ++s)
        {
            REQUIRE(gradient_indices[s] == gi[s]);
            near(gradient[s], g[s]);
        }
        REQUIRE(download(f.dtheta) == history);
    }
}

template <int N, int D>
Json measure(Fixture& f, bool gradient)
{
    Output<N, D> out(f.count, f.vertices);
    auto         launch = [&]
    {
        f.launch<D>(gradient, out.gv(), gradient ? TripletMatrixView<Float, D>{} : out.hv());
    };
    const auto warm_end =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    do
    {
        for(int j = 0; j < 32; ++j)
            launch();
        sync();
    } while(std::chrono::steady_clock::now() < warm_end);
    cudaEvent_t start, stop;
    CUDA_TOOL_CHECK(cudaEventCreate(&start));
    CUDA_TOOL_CHECK(cudaEventCreate(&stop));
    std::vector<double> samples;
    constexpr int       Launches = 20;
    for(int round = 0; round < 7; ++round)
    {
        CUDA_TOOL_CHECK(cudaEventRecord(start));
        for(int j = 0; j < Launches; ++j)
            launch();
        CUDA_TOOL_CHECK(cudaEventRecord(stop));
        CUDA_TOOL_CHECK(cudaEventSynchronize(stop));
        float ms = 0;
        CUDA_TOOL_CHECK(cudaEventElapsedTime(&ms, start, stop));
        samples.push_back(ms / Launches);
    }
    CUDA_TOOL_CHECK(cudaEventDestroy(start));
    CUDA_TOOL_CHECK(cudaEventDestroy(stop));
    sync();
    out.guards(gradient);
    auto values = download(out.g);
    for(int i = 0; i < std::min(f.count, 513); ++i)
    {
        const int sample = f.count <= 513 ? i : int((int64_t(i) * (f.count - 1)) / 512);
        auto expected = f.oracle<N, D>(sample).first;
        for(int a = 0; a < N; ++a)
            near(values[1 + sample * N + a], expected.template segment<D>(a * D));
    }
    if(!gradient)
    {
        auto hs = download(out.h);
        for(int s = 1; s < int(hs.size()) - 1; ++s)
        {
            REQUIRE(hs[s].allFinite());
            REQUIRE((hs[s].array() != Sentinel).all());
        }
    }
    auto sorted = samples;
    std::sort(sorted.begin(), sorted.end());
    return Json{{"material", Names[int(f.kind)]},
                {"gradient_only", gradient},
                {"block_size", f.block_size},
                {"primitive_count", f.count},
                {"launches_per_sample", Launches},
                {"samples_ms", samples},
                {"median_ms", sorted[sorted.size() / 2]},
                {"min_ms", sorted.front()},
                {"max_ms", sorted.back()}};
}

void check(Kind kind, int count)
{
    Fixture f(kind, count);
    if(kind == Kind::Fixed)
        validate<2, 12>(f);
    else if(kind == Kind::Spring || kind == Kind::Point)
        validate<2, 3>(f);
    else if(kind == Kind::Rod || kind == Kind::Edge)
        validate<3, 3>(f);
    else
        validate<4, 3>(f);
}
Json bench(Kind kind)
{
    Fixture f(kind, 65537);
    Json    rows = Json::array();
    for(bool gradient : {true, false})
    {
        if(const char* mode = std::getenv("UIPC_REMAINING_TUNE_MODE"))
            if(gradient != (std::string(mode) == "gradient"))
                continue;
        if(const char* size = std::getenv("UIPC_REMAINING_TUNE_CTA"))
            f.block_size = std::atoi(size);
        if(const char* blocks = std::getenv("UIPC_REMAINING_BLOCKS"))
        {
            auto settings = Json::parse(std::ifstream(blocks));
            f.block_size =
                settings.at(Names[int(kind)]).at(gradient ? "gradient" : "full").get<int>();
        }
        if(kind == Kind::Fixed)
            rows.push_back(measure<2, 12>(f, gradient));
        else if(kind == Kind::Spring || kind == Kind::Point)
            rows.push_back(measure<2, 3>(f, gradient));
        else if(kind == Kind::Rod || kind == Kind::Edge)
            rows.push_back(measure<3, 3>(f, gradient));
        else
            rows.push_back(measure<4, 3>(f, gradient));
    }
    return rows;
}
}  // namespace

TEST_CASE("remaining production material launches preserve G H and gradient-only contracts",
          "[cuda][remaining_material_launch]")
{
    for(int kind = 0; kind < int(Names.size()); ++kind)
    {
        INFO(Names[kind]);
        for(int n : {0, 1, 15, 16, 17, 31, 32, 33, 257, 1025})
        {
            INFO(n);
            check(Kind(kind), n);
        }
    }
}
TEST_CASE("remaining production material CUDA-event benchmark",
          "[.benchmark][cuda][remaining_material_benchmark]")
{
    Json result = Json::array();
    for(int k = 0; k < int(Names.size()); ++k)
        for(const auto& row : bench(Kind(k)))
            result.push_back(row);
    const char* output = std::getenv("UIPC_REMAINING_BENCH_OUTPUT");
    if(output)
        std::ofstream(output) << result.dump(2) << '\n';
    std::cout << "REMAINING_MATERIAL_BENCH " << result.dump() << '\n';
}
