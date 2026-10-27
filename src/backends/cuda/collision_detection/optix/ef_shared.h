#pragma once
#include <cstdint>

namespace uipc::backend::cuda::optix_ef
{
struct Proxy
{
    float lower[3];
    float upper[3];
    int   bid;
    int   cid;
};

struct Transform
{
    double origin[3];
    double inverse_extent[3];
};

struct LaunchParams
{
    unsigned long long traversable;
    const Proxy*       queries;
    const Proxy*       leaves;
    const Transform*   transform;
    const int*         body_self_collision;
    const int*         contact_masks;
    int                contact_mask_pitch;
    int                query_start;
    int                query_count;
    int*               pair_count;
    int*               pairs;
    unsigned long long pair_capacity;
};

// Coordinates are normalized to [0, 1]. This covers normalization and
// float ray-origin/interval rounding, including zero-width query boxes.
inline constexpr double rounding_margin = 16.0 * 0x1p-23;
}  // namespace uipc::backend::cuda::optix_ef
