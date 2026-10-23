#pragma once
#include <contact_system/ipc_contact_curvature.h>
#include <contact_system/contact_models/codim_ipc_simplex_normal_contact_function.h>
#include <utils/distance/distance_flagged.h>
#include <utils/make_spd.h>

namespace uipc::backend::cuda::ipc_surrogate
{
inline __device__ bool export_edge_edge_curvature(IPCNormalSurrogate&  s,
                                                  IPCContactCurvature& packed,
                                                  Float                kt2,
                                                  Float                d_hat,
                                                  Float          thickness,
                                                  const Vector3& rest_A0,
                                                  const Vector3& rest_A1,
                                                  const Vector3& rest_B0,
                                                  const Vector3& rest_B1,
                                                  const Vector3& A0,
                                                  const Vector3& A1,
                                                  const Vector3& B0,
                                                  const Vector3& B1)
{
    packed      = IPCContactCurvature{};
    s.kind      = 1;
    s.count     = 4;
    s.valid     = 0;
    s.mollified = 0;
    for(int j = 0; j < 3; ++j)
    {
        if(!isfinite(rest_A0[j]) || !isfinite(rest_A1[j])
           || !isfinite(rest_B0[j]) || !isfinite(rest_B1[j]) || !isfinite(A0[j])
           || !isfinite(A1[j]) || !isfinite(B0[j]) || !isfinite(B1[j]))
            return false;
    }
    if(!isfinite(kt2) || !isfinite(d_hat) || !isfinite(thickness) || kt2 < 0.0
       || d_hat <= 0.0 || thickness < 0.0)
        return false;

    Vector4i flag = distance::edge_edge_distance_flag(A0, A1, B0, B1);
    s.feature     = flag;
    Float D;
    distance::edge_edge_distance2(flag, A0, A1, B0, B1, D);
    if(!isfinite(D) || D <= thickness * thickness)
        return false;

    Float eps_x;
    distance::edge_edge_mollifier_threshold(
        rest_A0, rest_A1, rest_B0, rest_B1, static_cast<Float>(1e-3), eps_x);
    Float cross_norm2;
    distance::edge_edge_cross_norm2(A0, A1, B0, B1, cross_norm2);
    if(!isfinite(eps_x) || !isfinite(cross_norm2) || eps_x < 0.0 || cross_norm2 < 0.0)
        return false;
    s.mollified = cross_norm2 < eps_x;

    Vector12    G;
    Matrix12x12 H;
    sym::codim_ipc_simplex_contact::mollified_EE_barrier_gradient_hessian(
        G, H, flag, kt2, d_hat, thickness, rest_A0, rest_A1, rest_B0, rest_B1, A0, A1, B0, B1);
    for(int i = 0; i < 12; ++i)
    {
        if(!isfinite(G[i]))
            return false;
        for(int j = 0; j < 12; ++j)
            if(!isfinite(H(i, j)))
                return false;
    }
    make_spd(H);
    for(int i = 0; i < 12; ++i)
        for(int j = 0; j < 12; ++j)
            if(!isfinite(H(i, j)))
                return false;

    canonicalize(s, &H);
    packed.assign(H);
    s.valid = 1;
    return true;
}
}  // namespace uipc::backend::cuda::ipc_surrogate
