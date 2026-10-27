#include "ef_shared.h"
#include <optix.h>
#include <optix_device.h>

using namespace uipc::backend::cuda::optix_ef;

extern "C" {
__constant__ LaunchParams uipc_optix_ef_params;
}

extern "C" __global__ void __raygen__ef()
{
    const int local_query = static_cast<int>(optixGetLaunchIndex().x);
    if(local_query >= uipc_optix_ef_params.query_count)
        return;
    const Proxy query =
        uipc_optix_ef_params.queries[uipc_optix_ef_params.query_start + local_query];
    const Transform transform = *uipc_optix_ef_params.transform;
    const double    center_y  = (0.5 * double(query.lower[1])
                             + 0.5 * double(query.upper[1]) - transform.origin[1])
                            * transform.inverse_extent[1];
    const double center_z = (0.5 * double(query.lower[2])
                             + 0.5 * double(query.upper[2]) - transform.origin[2])
                            * transform.inverse_extent[2];
    const double begin =
        (double(query.lower[0]) - transform.origin[0]) * transform.inverse_extent[0] + 2.0;
    const double end =
        (double(query.upper[0]) - transform.origin[0]) * transform.inverse_extent[0] + 2.0;
    optixTrace(uipc_optix_ef_params.traversable,
               make_float3(-2.0f, float(center_y), float(center_z)),
               make_float3(1.0f, 0.0f, 0.0f),
               __double2float_rd(begin - rounding_margin),
               __double2float_ru(end + rounding_margin),
               0.0f,
               OptixVisibilityMask(255),
               OPTIX_RAY_FLAG_DISABLE_CLOSESTHIT,
               0,
               1,
               0);
}

extern "C" __global__ void __miss__ef() {}

namespace
{
__forceinline__ __device__ bool intersects(const Proxy& query, const Proxy& leaf)
{
    for(int axis = 0; axis < 3; ++axis)
        if(query.lower[axis] > leaf.upper[axis] || leaf.lower[axis] > query.upper[axis])
            return false;
    return true;
}

// 0 accepts, 1 rejects a homogeneous body, 2 rejects a contact-table entry.
__forceinline__ __device__ int tag_rejection(const Proxy& query, const Proxy& leaf)
{
    if(query.bid >= 0 && query.bid == leaf.bid
       && !uipc_optix_ef_params.body_self_collision[query.bid])
        return 1;
    if(query.cid >= 0 && leaf.cid >= 0)
    {
        const char* row = reinterpret_cast<const char*>(uipc_optix_ef_params.contact_masks)
                          + query.cid * uipc_optix_ef_params.contact_mask_pitch;
        if(!reinterpret_cast<const int*>(row)[leaf.cid])
            return 2;
    }
    return 0;
}

__forceinline__ __device__ void emit_pair(int leaf)
{
    const int index = atomicAdd(uipc_optix_ef_params.pair_count, 1);
    if(index >= 0
       && static_cast<unsigned long long>(index) < uipc_optix_ef_params.pair_capacity)
    {
        uipc_optix_ef_params.pairs[2ull * index] =
            static_cast<int>(optixGetLaunchIndex().x);
        uipc_optix_ef_params.pairs[2ull * index + 1] = leaf;
    }
    if(uipc_optix_ef_params.culling)
        atomicAdd(&uipc_optix_ef_params.culling->emitted_pairs, 1ull);
}
}  // namespace

extern "C" __global__ void __intersection__ef()
{
    const int    local_query = static_cast<int>(optixGetLaunchIndex().x);
    const Proxy& query =
        uipc_optix_ef_params.queries[uipc_optix_ef_params.query_start + local_query];
    const int primitive = static_cast<int>(optixGetPrimitiveIndex());
    auto*     counters  = uipc_optix_ef_params.culling;
    if(uipc_optix_ef_params.groups)
    {
        const Group& group = uipc_optix_ef_params.groups[primitive];
        if(counters)
            atomicAdd(&counters->group_tests, 1ull);
        if(!intersects(query, group.bounds))
            return;
        const int rejected = tag_rejection(query, group.bounds);
        if(rejected)
        {
            if(counters)
            {
                atomicAdd(rejected == 1 ? &counters->body_rejections : &counters->contact_rejections,
                          1ull);
                atomicAdd(&counters->skipped_leaves,
                          static_cast<unsigned long long>(group.count));
            }
            return;
        }
    }
    else
    {
        const Proxy& leaf = uipc_optix_ef_params.leaves[primitive];
        if(counters)
            atomicAdd(&counters->leaf_tests, 1ull);
        if(!intersects(query, leaf) || tag_rejection(query, leaf))
            return;
    }
    // Report exactly one hit per group. Any-hit emits children once even if
    // OptiX internally replicates this custom primitive during construction.
    optixReportIntersection(optixGetRayTmin(), 0);
}

extern "C" __global__ void __anyhit__ef()
{
    const int primitive = static_cast<int>(optixGetPrimitiveIndex());
    if(uipc_optix_ef_params.groups)
    {
        const Group& group       = uipc_optix_ef_params.groups[primitive];
        const int    local_query = static_cast<int>(optixGetLaunchIndex().x);
        const Proxy& query =
            uipc_optix_ef_params.queries[uipc_optix_ef_params.query_start + local_query];
        for(int offset = 0; offset < group.count; ++offset)
        {
            const int leaf_index =
                uipc_optix_ef_params.group_leaf_indices[group.begin + offset];
            const Proxy& leaf = uipc_optix_ef_params.leaves[leaf_index];
            if(uipc_optix_ef_params.culling)
                atomicAdd(&uipc_optix_ef_params.culling->leaf_tests, 1ull);
            if(intersects(query, leaf) && !tag_rejection(query, leaf))
                emit_pair(leaf_index);
        }
    }
    else
    {
        emit_pair(primitive);
    }
    optixIgnoreIntersection();
}
