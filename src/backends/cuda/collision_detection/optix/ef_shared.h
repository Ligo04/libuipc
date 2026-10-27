#pragma once
#include <cstddef>
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

struct Group
{
    Proxy bounds;
    int   begin;
    int   count;
};

struct CullingCounters
{
    unsigned long long group_tests;
    unsigned long long body_rejections;
    unsigned long long contact_rejections;
    unsigned long long skipped_leaves;
    unsigned long long leaf_tests;
    unsigned long long emitted_pairs;
};

struct LaunchParams
{
    unsigned long long traversable;
    const Proxy*       queries;
    const Proxy*       leaves;
    const Group*       groups;
    const int*         group_leaf_indices;
    const Transform*   transform;
    CullingCounters*   culling;
    const int*         body_self_collision;
    const int*         contact_masks;
    int                contact_mask_pitch;
    int                query_start;
    int                query_count;
    // OptiX copies every byte, including the alignment space before pointers.
    int                reserved;
    int*               pair_count;
    int*               pairs;
    unsigned long long pair_capacity;
};
static_assert(offsetof(LaunchParams, reserved) == 84);
static_assert(offsetof(LaunchParams, pair_count) == 88);
static_assert(sizeof(LaunchParams) == 112);

// Coordinates are normalized to [0, 1]. This covers normalization and
// float ray-origin/interval rounding, including zero-width query boxes.
inline constexpr double rounding_margin = 16.0 * 0x1p-23;
}  // namespace uipc::backend::cuda::optix_ef
