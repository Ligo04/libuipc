#pragma once
#include <uipc/backend/macro.h>
#include <cuda_tool/cuda_tool.h>
#include <cuda_tool/linear_system/views.h>
#include <affine_body/abd_jacobi_matrix.h>
#include <contact_system/contact_coeff.h>

namespace uipc::backend::cuda
{
// Production dispatch shared by the constitutions and launch-contract tests.
// Empty input is a no-op. Gradient-only dispatch never accesses Hessians.
// Both variants use the full kernel's occupancy hint to retain its grid shape;
// a larger gradient-only block can reduce parallelism for small inputs.
struct StableNeoHookean3DGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Float>          mus{};
    cuda_tool::CBufferView<Float>          lambdas{};
    cuda_tool::CBufferView<Vector4i>       indices{};
    cuda_tool::CBufferView<Vector3>        xs{};
    cuda_tool::CBufferView<Matrix3x3>      Dm_invs{};
    cuda_tool::DoubletVectorView<Float, 3> G3s{};
    cuda_tool::TripletMatrixView<Float, 3> H3x3s{};
    cuda_tool::CBufferView<Float>          volumes{};
    Float                                  dt{};
    bool                                   gradient_only = false;
};
UIPC_BACKEND_API void launch_stable_neo_hookean_3d_gradient_hessian(
    const StableNeoHookean3DGradientHessianLaunchInfo& info);

struct BaraffWitkinShellGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Float>          mus{};
    cuda_tool::CBufferView<Float>          lambdas{};
    cuda_tool::CBufferView<Float>          stretch_bs{};
    cuda_tool::CBufferView<Float>          strainRates{};
    cuda_tool::CBufferView<Vector3i>       indices{};
    cuda_tool::CBufferView<Vector3>        xs{};
    cuda_tool::CBufferView<Float>          thicknesses{};
    cuda_tool::DoubletVectorView<Float, 3> G3s{};
    cuda_tool::TripletMatrixView<Float, 3> H3x3s{};
    cuda_tool::CBufferView<Float>          rest_areas{};
    Float                                  dt{};
    cuda_tool::CBufferView<Matrix2x2>      IBs{};
    SizeT                                  half_hessian_size{};
    bool                                   gradient_only = false;
};
UIPC_BACKEND_API void launch_baraff_witkin_shell_gradient_hessian(
    const BaraffWitkinShellGradientHessianLaunchInfo& info);

struct ABDARAPGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector12>   qs{};
    cuda_tool::CBufferView<Float>      volumes{};
    cuda_tool::BufferView<Vector12>    gradients{};
    cuda_tool::BufferView<Matrix12x12> hessians{};
    cuda_tool::CBufferView<Float>      kappas{};
    Float                              dt{};
    bool                               gradient_only = false;
};
UIPC_BACKEND_API void launch_abd_arap_gradient_hessian(const ABDARAPGradientHessianLaunchInfo& info);

struct AffineBodySphericalJointGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector2i>            body_ids{};
    cuda_tool::CBufferView<Vector6>             rest_cs{};
    cuda_tool::CBufferView<Float>               strength_ratio{};
    cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses{};
    cuda_tool::CBufferView<Vector12>            qs{};
    cuda_tool::DoubletVectorView<Float, 12>     G12s{};
    cuda_tool::TripletMatrixView<Float, 12>     H12x12s{};
    SizeT                                       HalfHessianSize{};
    bool                                        gradient_only = false;
};
UIPC_BACKEND_API void launch_affine_body_spherical_joint_gradient_hessian(
    const AffineBodySphericalJointGradientHessianLaunchInfo& info);

struct HalfPlaneNormalGradientHessianLaunchInfo
{
    cuda_tool::DoubletVectorView<Float, 3> Grad{};
    cuda_tool::TripletMatrixView<Float, 3> Hess{};
    cuda_tool::CBufferView<Vector2i>       PHs{};
    cuda_tool::CBufferView<Vector3>        plane_positions{};
    cuda_tool::CBufferView<Vector3>        plane_normals{};
    cuda_tool::CDense2D<ContactCoeff>      table{};
    cuda_tool::CBufferView<IndexT>         contact_ids{};
    cuda_tool::CBufferView<Vector3>        Ps{};
    cuda_tool::CBufferView<Float>          thicknesses{};
    cuda_tool::CBufferView<Float>          d_hats{};
    IndexT                                 half_plane_vertex_offset{};
    Float                                  dt{};
    bool                                   gradient_only = false;
};
UIPC_BACKEND_API void launch_half_plane_normal_gradient_hessian(
    const HalfPlaneNormalGradientHessianLaunchInfo& info);

struct HalfPlaneFrictionGradientHessianLaunchInfo
{
    cuda_tool::DoubletVectorView<Float, 3> Grad{};
    cuda_tool::TripletMatrixView<Float, 3> Hess{};
    cuda_tool::CBufferView<Vector2i>       PHs{};
    cuda_tool::CBufferView<Vector3>        plane_positions{};
    cuda_tool::CBufferView<Vector3>        plane_normals{};
    cuda_tool::CDense2D<ContactCoeff>      table{};
    cuda_tool::CBufferView<IndexT>         contact_ids{};
    cuda_tool::CBufferView<Vector3>        Ps{};
    cuda_tool::CBufferView<Vector3>        prev_Ps{};
    cuda_tool::CBufferView<Float>          thicknesses{};
    Float                                  eps_v{};
    cuda_tool::CBufferView<Float>          d_hats{};
    IndexT                                 half_plane_vertex_offset{};
    Float                                  dt{};
    bool                                   gradient_only = false;
};
UIPC_BACKEND_API void launch_half_plane_friction_gradient_hessian(
    const HalfPlaneFrictionGradientHessianLaunchInfo& info);

struct FEMBDF1GradientHessianLaunchInfo
{
    cuda_tool::CBufferView<IndexT>            is_fixed{};
    cuda_tool::CBufferView<Vector3>           xs{};
    cuda_tool::CBufferView<Vector3>           x_tildes{};
    cuda_tool::CBufferView<Float>             masses{};
    cuda_tool::DoubletVectorView<Float, 3>    G3s{};
    cuda_tool::TripletMatrixView<Float, 3, 3> H3x3s{};
    bool                                      gradient_only = false;
};
UIPC_BACKEND_API void launch_fem_bdf1_gradient_hessian(const FEMBDF1GradientHessianLaunchInfo& info);

struct FEMBDF2GradientHessianLaunchInfo
{
    cuda_tool::CBufferView<IndexT>            is_fixed{};
    cuda_tool::CBufferView<Vector3>           xs{};
    cuda_tool::CBufferView<Vector3>           x_tildes{};
    cuda_tool::CBufferView<Float>             masses{};
    cuda_tool::DoubletVectorView<Float, 3>    G3s{};
    cuda_tool::TripletMatrixView<Float, 3, 3> H3x3s{};
    Float                                     inv_beta{};
    bool                                      gradient_only = false;
};
UIPC_BACKEND_API void launch_fem_bdf2_gradient_hessian(const FEMBDF2GradientHessianLaunchInfo& info);

struct ABDBDF1GradientHessianLaunchInfo
{
    cuda_tool::CBufferView<IndexT>              is_fixed{};
    cuda_tool::CBufferView<Vector12>            qs{};
    cuda_tool::CBufferView<Vector12>            q_prevs{};
    cuda_tool::CBufferView<Vector12>            q_tildes{};
    cuda_tool::CBufferView<ABDJacobiDyadicMass> masses{};
    cuda_tool::BufferView<Matrix12x12>          hessians{};
    cuda_tool::BufferView<Vector12>             gradients{};
    bool                                        gradient_only = false;
};
UIPC_BACKEND_API void launch_abd_bdf1_gradient_hessian(const ABDBDF1GradientHessianLaunchInfo& info);

struct ABDBDF2GradientHessianLaunchInfo
{
    cuda_tool::CBufferView<IndexT>              is_fixed{};
    cuda_tool::CBufferView<Vector12>            qs{};
    cuda_tool::CBufferView<Vector12>            q_tildes{};
    cuda_tool::CBufferView<ABDJacobiDyadicMass> masses{};
    cuda_tool::BufferView<Matrix12x12>          hessians{};
    cuda_tool::BufferView<Vector12>             gradients{};
    Float                                       inv_beta{};
    bool                                        gradient_only = false;
};
UIPC_BACKEND_API void launch_abd_bdf2_gradient_hessian(const ABDBDF2GradientHessianLaunchInfo& info);

}  // namespace uipc::backend::cuda
