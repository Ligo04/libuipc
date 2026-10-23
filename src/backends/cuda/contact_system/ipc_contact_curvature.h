#pragma once
#include <contact_system/ipc_normal_surrogate.h>

namespace uipc::backend::cuda
{
struct IPCContactCurvature
{
    Float upper[78]{};

    UIPC_GENERIC void assign(const Matrix12x12& matrix)
    {
        int k = 0;
        for(int i = 0; i < 12; ++i)
            for(int j = i; j < 12; ++j)
                upper[k++] = matrix(i, j);
    }

    UIPC_GENERIC Matrix12x12 matrix() const
    {
        Matrix12x12 result;
        int         k = 0;
        for(int i = 0; i < 12; ++i)
            for(int j = i; j < 12; ++j)
                result(i, j) = result(j, i) = upper[k++];
        return result;
    }
};

namespace ipc_surrogate
{
    UIPC_GENERIC inline void swap_slots(IPCNormalSurrogate& s, int a, int b, Matrix12x12* curvature = nullptr)
    {
        IndexT vertex  = s.vertices[a];
        s.vertices[a]  = s.vertices[b];
        s.vertices[b]  = vertex;
        IndexT feature = s.feature[a];
        s.feature[a]   = s.feature[b];
        s.feature[b]   = feature;
        Vector3 q      = s.q[a];
        s.q[a]         = s.q[b];
        s.q[b]         = q;
        if(curvature)
        {
            for(int c = 0; c < 3; ++c)
                for(int j = 0; j < 12; ++j)
                {
                    Float value                = (*curvature)(3 * a + c, j);
                    (*curvature)(3 * a + c, j) = (*curvature)(3 * b + c, j);
                    (*curvature)(3 * b + c, j) = value;
                }
            for(int c = 0; c < 3; ++c)
                for(int i = 0; i < 12; ++i)
                {
                    Float value                = (*curvature)(i, 3 * a + c);
                    (*curvature)(i, 3 * a + c) = (*curvature)(i, 3 * b + c);
                    (*curvature)(i, 3 * b + c) = value;
                }
        }
    }

    UIPC_GENERIC inline void canonicalize(IPCNormalSurrogate& s, Matrix12x12* curvature = nullptr)
    {
        if(s.kind == 0)
        {
            for(int a = 1; a < 3; ++a)
                for(int b = a + 1; b < 4; ++b)
                    if(s.vertices[a] > s.vertices[b])
                        swap_slots(s, a, b, curvature);
        }
        else if(s.kind == 1)
        {
            if(s.vertices[0] > s.vertices[1])
                swap_slots(s, 0, 1, curvature);
            if(s.vertices[2] > s.vertices[3])
                swap_slots(s, 2, 3, curvature);
            if(s.vertices[0] > s.vertices[2]
               || (s.vertices[0] == s.vertices[2] && s.vertices[1] > s.vertices[3]))
            {
                swap_slots(s, 0, 2, curvature);
                swap_slots(s, 1, 3, curvature);
            }
        }
        else if(s.kind == 2)
        {
            if(s.vertices[1] > s.vertices[2])
                swap_slots(s, 1, 2, curvature);
        }
        else if(s.kind == 3 && s.vertices[0] > s.vertices[1])
            swap_slots(s, 0, 1, curvature);
    }
}  // namespace ipc_surrogate
}  // namespace uipc::backend::cuda
