#pragma once
#include <uipc/backend/macro.h>
#include <cuda_tool/buffer.h>
#include <cuda_tool/linear_system/views.h>
#include <affine_body/abd_jacobi_matrix.h>
#include <type_define.h>

namespace uipc::backend::cuda
{
// Production launches shared with CUDA correctness and performance checks.
// Empty input is a no-op; gradient-only launches never access the Hessian view.
// Shared-workspace full launches accept at most 32 threads (16 for strain
// bending); gradient launches follow the device kernel thread limit.
struct AffineBodyFixedJointGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector2i>            body_ids;
    cuda_tool::CBufferView<Vector6>             rest_cs;
    cuda_tool::CBufferView<Vector6>             rest_ts;
    cuda_tool::CBufferView<Vector6>             rest_ns;
    cuda_tool::CBufferView<Vector6>             rest_bs;
    cuda_tool::CBufferView<Float>               strength_ratio;
    cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses;
    cuda_tool::CBufferView<Vector12>            qs;
    cuda_tool::DoubletVectorView<Float, 12>     G12s;
    cuda_tool::TripletMatrixView<Float, 12>     H12x12s;
    bool                                        gradient_only   = false;
    SizeT                                       HalfHessianSize = 0;
    // Zero selects the production default; positive values are used for CTA tuning.
    int block_size = 0;
};
UIPC_BACKEND_API void launch_affine_body_fixed_joint_gradient_hessian(
    const AffineBodyFixedJointGradientHessianLaunchInfo& info);

struct ARAP3DGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Float>          kappas;
    cuda_tool::CBufferView<Vector4i>       indices;
    cuda_tool::CBufferView<Vector3>        xs;
    cuda_tool::CBufferView<Matrix3x3>      Dm_invs;
    cuda_tool::DoubletVectorView<Float, 3> G3s;
    cuda_tool::TripletMatrixView<Float, 3> H3x3s;
    cuda_tool::CBufferView<Float>          volumes;
    Float                                  dt            = 0;
    bool                                   gradient_only = false;
    // Zero selects the production default; positive values are used for CTA tuning.
    int block_size = 0;
};
UIPC_BACKEND_API void launch_arap_3d_gradient_hessian(const ARAP3DGradientHessianLaunchInfo& info);

struct HookeanSpring1DGradientHessianLaunchInfo
{
    cuda_tool::DoubletVectorView<Float, 3> G3s;
    cuda_tool::TripletMatrixView<Float, 3> H3x3s;
    cuda_tool::CBufferView<Float>          kappas;
    cuda_tool::CBufferView<Float>          rest_lengths;
    cuda_tool::CBufferView<Float>          thicknesses;
    cuda_tool::CBufferView<Vector2i>       indices;
    cuda_tool::CBufferView<Vector3>        xs;
    Float                                  dt            = 0;
    double                                 Pi            = 0;
    bool                                   gradient_only = false;
    // Zero selects the production default; positive values are used for CTA tuning.
    int block_size = 0;
};
UIPC_BACKEND_API void launch_hookean_spring_1d_gradient_hessian(
    const HookeanSpring1DGradientHessianLaunchInfo& info);

struct KirchhoffRodBendingGradientHessianLaunchInfo
{
    cuda_tool::BufferView<Vector3i>        hinges;
    cuda_tool::BufferView<Float>           bending_stiffnesses;
    cuda_tool::CBufferView<Float>          thicknesses;
    cuda_tool::CBufferView<Vector3>        xs;
    cuda_tool::CBufferView<Vector3>        x_bars;
    cuda_tool::DoubletVectorView<Float, 3> G3s;
    cuda_tool::TripletMatrixView<Float, 3> H3x3s;
    Float                                  dt            = 0;
    Float                                  Pi            = 0;
    bool                                   gradient_only = false;
    // Zero selects the production default; positive values are used for CTA tuning.
    int block_size = 0;
};
UIPC_BACKEND_API void launch_kirchhoff_rod_bending_gradient_hessian(
    const KirchhoffRodBendingGradientHessianLaunchInfo& info);

struct StrainPlasticBendingGradientHessianLaunchInfo
{
    cuda_tool::BufferView<Vector4i>        stencils;
    cuda_tool::BufferView<Float>           bending_stiffnesses;
    cuda_tool::BufferView<Float>           theta_bars;
    cuda_tool::BufferView<Float>           h_bars;
    cuda_tool::BufferView<Float>           V_bars;
    cuda_tool::BufferView<Float>           L0s;
    cuda_tool::CBufferView<Vector3>        xs;
    cuda_tool::DoubletVectorView<Float, 3> G3s;
    cuda_tool::TripletMatrixView<Float, 3> H3x3s;
    Float                                  dt            = 0;
    bool                                   gradient_only = false;
    // Zero selects the production default; positive values are used for CTA tuning.
    int block_size = 0;
};
UIPC_BACKEND_API void launch_strain_plastic_bending_gradient_hessian(
    const StrainPlasticBendingGradientHessianLaunchInfo& info);

struct SoftVertexStitchGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector2i>          topos;
    cuda_tool::CBufferView<Vector3>           xs;
    cuda_tool::CBufferView<Float>             kappas;
    cuda_tool::CBufferView<Float>             rest_lengths;
    cuda_tool::DoubletVectorView<Float, 3>    G3s;
    cuda_tool::TripletMatrixView<Float, 3, 3> H3x3s;
    Float                                     dt            = 0;
    bool                                      gradient_only = false;
    // Zero selects the production default; positive values are used for CTA tuning.
    int block_size = 0;
};
UIPC_BACKEND_API void launch_soft_vertex_stitch_gradient_hessian(
    const SoftVertexStitchGradientHessianLaunchInfo& info);

struct SoftVertexEdgeStitchGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector3i>          topos;
    cuda_tool::CBufferView<Vector3>           xs;
    cuda_tool::CBufferView<Float>             mus;
    cuda_tool::CBufferView<Float>             lambdas;
    cuda_tool::CBufferView<Matrix2x2>         IBs;
    cuda_tool::CBufferView<Float>             rest_areas;
    cuda_tool::CBufferView<Float>             thick;
    cuda_tool::DoubletVectorView<Float, 3>    G3s;
    cuda_tool::TripletMatrixView<Float, 3, 3> H3x3s;
    Float                                     dt            = 0;
    bool                                      gradient_only = false;
    // Zero selects the production default; positive values are used for CTA tuning.
    int block_size = 0;
};
UIPC_BACKEND_API void launch_soft_vertex_edge_stitch_gradient_hessian(
    const SoftVertexEdgeStitchGradientHessianLaunchInfo& info);

struct SoftVertexTriangleStitchGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector4i>          topos;
    cuda_tool::CBufferView<Vector3>           xs;
    cuda_tool::CBufferView<Float>             mus;
    cuda_tool::CBufferView<Float>             lambdas;
    cuda_tool::CBufferView<Matrix3x3>         Dm_invs;
    cuda_tool::CBufferView<Float>             rest_vols;
    cuda_tool::DoubletVectorView<Float, 3>    G3s;
    cuda_tool::TripletMatrixView<Float, 3, 3> H3x3s;
    Float                                     dt            = 0;
    bool                                      gradient_only = false;
    // Zero selects the production default; positive values are used for CTA tuning.
    int block_size = 0;
};
UIPC_BACKEND_API void launch_soft_vertex_triangle_stitch_gradient_hessian(
    const SoftVertexTriangleStitchGradientHessianLaunchInfo& info);

}  // namespace uipc::backend::cuda
