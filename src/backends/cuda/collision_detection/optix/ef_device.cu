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

extern "C" __global__ void __intersection__ef()
{
    const int    local_query = static_cast<int>(optixGetLaunchIndex().x);
    const Proxy& query =
        uipc_optix_ef_params.queries[uipc_optix_ef_params.query_start + local_query];
    const Proxy& leaf = uipc_optix_ef_params.leaves[optixGetPrimitiveIndex()];
    for(int axis = 0; axis < 3; ++axis)
        if(query.lower[axis] > leaf.upper[axis] || leaf.lower[axis] > query.upper[axis])
            return;
    if(query.bid >= 0 && query.bid == leaf.bid
       && !uipc_optix_ef_params.body_self_collision[query.bid])
        return;
    if(query.cid >= 0 && leaf.cid >= 0)
    {
        const char* row = reinterpret_cast<const char*>(uipc_optix_ef_params.contact_masks)
                          + query.cid * uipc_optix_ef_params.contact_mask_pitch;
        if(!reinterpret_cast<const int*>(row)[leaf.cid])
            return;
    }
    // Any-hit owns emission: REQUIRE_SINGLE_ANYHIT_CALL prevents duplicate EF
    // pairs when the acceleration structure internally replicates primitives.
    optixReportIntersection(optixGetRayTmin(), 0);
}

extern "C" __global__ void __anyhit__ef()
{
    const int index = atomicAdd(uipc_optix_ef_params.pair_count, 1);
    if(index >= 0
       && static_cast<unsigned long long>(index) < uipc_optix_ef_params.pair_capacity)
    {
        uipc_optix_ef_params.pairs[2ull * index] =
            static_cast<int>(optixGetLaunchIndex().x);
        uipc_optix_ef_params.pairs[2ull * index + 1] =
            static_cast<int>(optixGetPrimitiveIndex());
    }
    optixIgnoreIntersection();
}
