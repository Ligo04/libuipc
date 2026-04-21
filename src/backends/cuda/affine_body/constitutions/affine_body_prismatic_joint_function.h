#pragma once
#include <type_define.h>

namespace uipc::backend::cuda
{
namespace sym::affine_body_prismatic_joint
{
#include "sym/affine_body_prismatic_joint.inl"
}
namespace sym::affine_body_driving_prismatic_joint
{
#include "sym/affine_body_driving_prismatic_joint.inl"
}

MUDA_DEVICE MUDA_INLINE void compute_relative_distance(Float& out_distance,
                                                       const Vector6&  C_bar,
                                                       const Vector6&  t_bar,
                                                       const Vector12& q_i,
                                                       const Vector12& q_j)
{
    Vector9 F01_q;
    sym::affine_body_driving_prismatic_joint::F01_q<Float>(F01_q,
                                                           C_bar.segment<3>(0),
                                                           t_bar.segment<3>(0),
                                                           q_i,
                                                           C_bar.segment<3>(3),
                                                           t_bar.segment<3>(3),
                                                           q_j);
    sym::affine_body_driving_prismatic_joint::Distance<Float>(out_distance, F01_q);
}
}  // namespace uipc::backend::cuda