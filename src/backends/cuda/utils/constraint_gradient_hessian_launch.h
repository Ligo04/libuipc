#pragma once
#include <uipc/backend/macro.h>
#include <cuda_tool/cuda_tool.h>
#include <cuda_tool/linear_system/views.h>
#include <affine_body/abd_jacobi_matrix.h>

namespace uipc::backend::cuda
{
// Shared production/test dispatch. Empty inputs are no-ops; gradient-only
// dispatch accepts empty Hessian storage. External articulation retains
// independent joint and joint-pair counts.

struct RevoluteJointGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector2i>            body_ids{};
    cuda_tool::CBufferView<Vector12>            rest_positions{};
    cuda_tool::CBufferView<Float>               strength_ratio{};
    cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses{};
    cuda_tool::CBufferView<Vector12>            qs{};
    cuda_tool::DoubletVectorView<Float, 12>     G12s{};
    cuda_tool::TripletMatrixView<Float, 12>     H12x12s{};
    bool                                        gradient_only = false;
};
UIPC_BACKEND_API void launch_revolute_joint_gradient_hessian(const RevoluteJointGradientHessianLaunchInfo& info);

struct DrivingRevoluteJointGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector2i>            body_ids{};
    cuda_tool::CBufferView<Vector6>             l_basis{};
    cuda_tool::CBufferView<Vector6>             r_basis{};
    cuda_tool::CBufferView<IndexT>              is_constrained{};
    cuda_tool::CBufferView<Float>               strength_ratios{};
    cuda_tool::CBufferView<IndexT>              is_passive{};
    cuda_tool::CBufferView<Float>               init_angles{};
    cuda_tool::CBufferView<Float>               aim_angles{};
    cuda_tool::CBufferView<Float>               current_angles{};
    cuda_tool::CBufferView<Vector12>            qs{};
    cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses{};
    cuda_tool::DoubletVectorView<Float, 12>     G12s{};
    cuda_tool::TripletMatrixView<Float, 12>     H12x12s{};
    bool                                        gradient_only = false;
};
UIPC_BACKEND_API void launch_driving_revolute_joint_gradient_hessian(
    const DrivingRevoluteJointGradientHessianLaunchInfo& info);

struct RevoluteJointLimitGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector2i>        body_ids{};
    cuda_tool::CBufferView<Vector6>         l_basis{};
    cuda_tool::CBufferView<Vector6>         r_basis{};
    cuda_tool::CBufferView<Float>           init_angles{};
    cuda_tool::CBufferView<Float>           current_angles{};
    cuda_tool::CBufferView<Float>           lowers{};
    cuda_tool::CBufferView<Float>           uppers{};
    cuda_tool::CBufferView<Float>           strengths{};
    cuda_tool::CBufferView<Vector12>        qs{};
    cuda_tool::CBufferView<Vector12>        q_prevs{};
    cuda_tool::DoubletVectorView<Float, 12> G12s{};
    cuda_tool::TripletMatrixView<Float, 12> H12x12s{};
    bool                                    gradient_only = false;
};
UIPC_BACKEND_API void launch_revolute_joint_limit_gradient_hessian(
    const RevoluteJointLimitGradientHessianLaunchInfo& info);

struct PrismaticJointGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector2i>            body_ids{};
    cuda_tool::CBufferView<Vector6>             rest_cs{};
    cuda_tool::CBufferView<Vector6>             rest_ts{};
    cuda_tool::CBufferView<Vector6>             rest_ns{};
    cuda_tool::CBufferView<Vector6>             rest_bs{};
    cuda_tool::CBufferView<Float>               strength_ratio{};
    cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses{};
    cuda_tool::CBufferView<Vector12>            qs{};
    cuda_tool::DoubletVectorView<Float, 12>     G12s{};
    cuda_tool::TripletMatrixView<Float, 12>     H12x12s{};
    bool                                        gradient_only = false;
};
UIPC_BACKEND_API void launch_prismatic_joint_gradient_hessian(
    const PrismaticJointGradientHessianLaunchInfo& info);

struct DrivingPrismaticJointGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector2i>            body_ids{};
    cuda_tool::CBufferView<IndexT>              is_constrained{};
    cuda_tool::CBufferView<IndexT>              is_passive{};
    cuda_tool::CBufferView<Float>               strength_ratios{};
    cuda_tool::CBufferView<Vector6>             rest_positions{};
    cuda_tool::CBufferView<Vector6>             rest_tangents{};
    cuda_tool::CBufferView<Float>               init_distances{};
    cuda_tool::CBufferView<Float>               curr_distances{};
    cuda_tool::CBufferView<Float>               aim_distances{};
    cuda_tool::CBufferView<Vector12>            qs{};
    cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses{};
    cuda_tool::DoubletVectorView<Float, 12>     G12s{};
    cuda_tool::TripletMatrixView<Float, 12>     H12x12s{};
    bool                                        gradient_only = false;
};
UIPC_BACKEND_API void launch_driving_prismatic_joint_gradient_hessian(
    const DrivingPrismaticJointGradientHessianLaunchInfo& info);

struct PrismaticJointLimitGradientHessianLaunchInfo
{
    cuda_tool::CBufferView<Vector2i>        body_ids{};
    cuda_tool::CBufferView<Vector6>         rest_cs{};
    cuda_tool::CBufferView<Vector6>         rest_ts{};
    cuda_tool::CBufferView<Float>           init_distances{};
    cuda_tool::CBufferView<Float>           lowers{};
    cuda_tool::CBufferView<Float>           uppers{};
    cuda_tool::CBufferView<Float>           strengths{};
    cuda_tool::CBufferView<Vector12>        qs{};
    cuda_tool::DoubletVectorView<Float, 12> G12s{};
    cuda_tool::TripletMatrixView<Float, 12> H12x12s{};
    bool                                    gradient_only = false;
};
UIPC_BACKEND_API void launch_prismatic_joint_limit_gradient_hessian(
    const PrismaticJointLimitGradientHessianLaunchInfo& info);

struct SoftTransformConstraintGradientHessianLaunchInfo
{
    Float                                       substep_ratio{};
    cuda_tool::BufferView<IndexT>               indices{};
    cuda_tool::CBufferView<Vector12>            qs{};
    cuda_tool::CBufferView<Vector12>            q_prevs{};
    cuda_tool::BufferView<Vector12>             aim_transforms{};
    cuda_tool::BufferView<Vector2>              strength_ratios{};
    cuda_tool::CBufferView<ABDJacobiDyadicMass> body_masses{};
    cuda_tool::DoubletVectorView<Float, 12>     gradients{};
    cuda_tool::CBufferView<IndexT>              is_fixed{};
    cuda_tool::TripletMatrixView<Float, 12>     hessians{};
    bool                                        gradient_only = false;
};
UIPC_BACKEND_API void launch_soft_transform_constraint_gradient_hessian(
    const SoftTransformConstraintGradientHessianLaunchInfo& info);

struct ExternalArticulationConstraintGradientHessianLaunchInfo
{
    cuda_tool::DoubletVectorView<Float, 12> gradients{};
    cuda_tool::CBufferView<Float>           joint_id_to_G_theta{};
    cuda_tool::CBufferView<Vector2i>        joint_id_to_body_ids{};
    cuda_tool::CBufferView<Vector6>         joint_id_to_L_basis{};
    cuda_tool::CBufferView<Vector6>         joint_id_to_R_basis{};
    cuda_tool::CBufferView<Vector12>        qs{};
    cuda_tool::CBufferView<Vector12>        q_prevs{};
    cuda_tool::CBufferView<Vector2i>        joint_id_to_ref_q_prev_ids{};
    cuda_tool::BufferView<Vector12>         ref_q_prevs{};
    cuda_tool::CBufferView<U64>             joint_id_to_uid{};
    U64                                     RevoluteUID{};
    U64                                     PrismaticUID{};
    cuda_tool::TripletMatrixView<Float, 12> hessians{};
    cuda_tool::CBufferView<Float>           joint_joint_id_to_mass{};
    cuda_tool::CBufferView<Vector2i>        joint_joint_id_to_joint_ij{};
    SizeT                                   HalfHessianSize{};
    bool                                    gradient_only = false;
};
UIPC_BACKEND_API void launch_external_articulation_constraint_gradient_hessian(
    const ExternalArticulationConstraintGradientHessianLaunchInfo& info);

struct SoftPositionConstraintGradientHessianLaunchInfo
{
    Float                                  substep_ratio{};
    cuda_tool::BufferView<IndexT>          indices{};
    cuda_tool::CBufferView<Vector3>        xs{};
    cuda_tool::CBufferView<Vector3>        x_prevs{};
    cuda_tool::BufferView<Vector3>         aim_positions{};
    cuda_tool::BufferView<Float>           strength_ratio{};
    cuda_tool::CBufferView<Float>          masses{};
    cuda_tool::DoubletVectorView<Float, 3> gradients{};
    cuda_tool::CBufferView<IndexT>         is_fixed{};
    cuda_tool::TripletMatrixView<Float, 3> hessians{};
    bool                                   gradient_only = false;
};
UIPC_BACKEND_API void launch_soft_position_constraint_gradient_hessian(
    const SoftPositionConstraintGradientHessianLaunchInfo& info);

}  // namespace uipc::backend::cuda
