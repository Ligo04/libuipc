#pragma once
#include <type_define.h>

namespace uipc::backend::cuda
{
struct IPCNormalSurrogate
{
    // 0: PT, 1: EE, 2: PE, 3: PP, 4: PH; vertex roles remain distinct.
    IndexT   kind     = -1;
    IndexT   count    = 0;
    Vector4i vertices = Vector4i::Constant(-1);
    IndexT   plane    = -1;
    Vector4i feature  = Vector4i::Constant(-1);
    Vector3 q[4] = {Vector3::Zero(), Vector3::Zero(), Vector3::Zero(), Vector3::Zero()};
    Float stiffness = 0.0;
    // EE records reference a complete PSD Hessian in a separate compact buffer.
    IndexT curvature_index = -1;
    IndexT mollified       = 0;
    // 0: invalid geometry/curvature, 1: supported, 2: unsupported export.
    IndexT valid = 0;
};
}  // namespace uipc::backend::cuda
