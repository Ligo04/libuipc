#include <collision_detection/optix_ef_bvh.h>
#include <stdexcept>

#if UIPC_WITH_OPTIX
#include <collision_detection/optix/ef_shared.h>
#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stubs.h>
#include <optix_stack_size.h>
#include <optix_ef_device_ptx.h>
#include <cuda_tool/cub.h>
#include <cub/block/block_reduce.cuh>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace uipc::backend::cuda
{
namespace
{
    using namespace cuda_tool;
    using namespace optix_ef;

    void check_optix(OptixResult result, const char* operation, const char* log = nullptr)
    {
        if(result != OPTIX_SUCCESS)
            throw std::runtime_error(std::string(operation) + ": " + optixGetErrorName(result)
                                     + " (" + optixGetErrorString(result) + ")"
                                     + (log ? std::string("\n") + log : ""));
    }

    struct BoundsStats
    {
        double lower[3];
        double upper[3];
        double half_extent[2];
    };

    struct MergeBoundsStats
    {
        __host__ __device__ BoundsStats operator()(const BoundsStats& a,
                                                   const BoundsStats& b) const
        {
            BoundsStats result;
            for(int axis = 0; axis < 3; ++axis)
            {
                result.lower[axis] =
                    a.lower[axis] < b.lower[axis] ? a.lower[axis] : b.lower[axis];
                result.upper[axis] =
                    a.upper[axis] > b.upper[axis] ? a.upper[axis] : b.upper[axis];
            }
            for(int axis = 0; axis < 2; ++axis)
                result.half_extent[axis] = a.half_extent[axis] > b.half_extent[axis] ?
                                               a.half_extent[axis] :
                                               b.half_extent[axis];
            return result;
        }
    };

    __host__ __device__ BoundsStats empty_bounds_stats()
    {
        constexpr double largest = std::numeric_limits<double>::max();
        return {{largest, largest, largest}, {-largest, -largest, -largest}, {0.0, 0.0}};
    }

    constexpr int bounds_block_size = 256;

    __global__ void OptixEFBVH_prepare_proxies_kernel(CBufferView<AABB> query_boxes,
                                                      CBufferView<IndexT> query_bids,
                                                      CBufferView<IndexT> query_cids,
                                                      CBufferView<AABB> leaf_boxes,
                                                      CBufferView<IndexT> leaf_bids,
                                                      CBufferView<IndexT> leaf_cids,
                                                      Proxy*       queries,
                                                      Proxy*       leaves,
                                                      BoundsStats* partials)
    {
        using BlockReduce = cub::BlockReduce<BoundsStats, bounds_block_size>;
        __shared__ typename BlockReduce::TempStorage storage;
        const SizeT index       = SizeT(blockIdx.x) * blockDim.x + threadIdx.x;
        const int   query_count = static_cast<int>(query_boxes.size());
        const int total_count = query_count + static_cast<int>(leaf_boxes.size());
        BoundsStats stats = empty_bounds_stats();
        if(index < total_count)
        {
            const bool query       = index < query_count;
            const int  local_index = query ? index : index - query_count;
            const auto box = query ? query_boxes(local_index) : leaf_boxes(local_index);
            Proxy proxy;
            for(int axis = 0; axis < 3; ++axis)
            {
                const double lower = box.min()[axis];
                const double upper = box.max()[axis];
                UIPC_KERNEL_ASSERT(isfinite(lower) && isfinite(upper) && lower <= upper,
                                   "OptiX EF proxy %d axis %d has invalid bounds [%g, %g]",
                                   static_cast<int>(index),
                                   axis,
                                   lower,
                                   upper);
                proxy.lower[axis] = box.min()[axis];
                proxy.upper[axis] = box.max()[axis];
                stats.lower[axis] = lower;
                stats.upper[axis] = upper;
                if(query && axis > 0)
                    stats.half_extent[axis - 1] = 0.5 * (upper - lower);
            }
            proxy.bid = query ? query_bids(local_index) : leaf_bids(local_index);
            proxy.cid = query ? query_cids(local_index) : leaf_cids(local_index);
            if(query)
                queries[local_index] = proxy;
            else
                leaves[local_index] = proxy;
        }
        const BoundsStats aggregate =
            BlockReduce(storage).Reduce(stats, MergeBoundsStats{});
        if(threadIdx.x == 0)
            partials[blockIdx.x] = aggregate;
    }

    __global__ void OptixEFBVH_prepare_morton_keys_kernel(const Proxy* leaves,
                                                          int leaf_count,
                                                          const BoundsStats* reduced,
                                                          U32* keys,
                                                          int* indices)
    {
        const SizeT index = SizeT(blockIdx.x) * blockDim.x + threadIdx.x;
        if(index >= leaf_count)
            return;
        const Proxy       leaf  = leaves[index];
        const BoundsStats stats = *reduced;
        float             center[3];
        for(int axis = 0; axis < 3; ++axis)
        {
            const double extent = stats.upper[axis] - stats.lower[axis];
            const double normalized =
                extent > 0.0 ? (0.5 * double(leaf.lower[axis])
                                + 0.5 * double(leaf.upper[axis]) - stats.lower[axis])
                                   / extent :
                               0.0;
            center[axis] = static_cast<float>(normalized);
        }
        keys[index] = info_stackless_detail::morton3D(center[0], center[1], center[2]);
        indices[index] = static_cast<int>(index);
    }

    __global__ void OptixEFBVH_prepare_tag_keys_kernel(const Proxy* leaves,
                                                       const int*   indices,
                                                       int          leaf_count,
                                                       U64*         keys)
    {
        const SizeT index = SizeT(blockIdx.x) * blockDim.x + threadIdx.x;
        if(index >= leaf_count)
            return;
        const Proxy leaf = leaves[indices[index]];
        keys[index] =
            (U64{static_cast<U32>(leaf.bid)} << 32) | static_cast<U32>(leaf.cid);
    }

    __global__ void OptixEFBVH_mark_tag_runs_kernel(const U64* keys, int leaf_count, int* starts)
    {
        const SizeT index = SizeT(blockIdx.x) * blockDim.x + threadIdx.x;
        if(index < leaf_count)
            starts[index] = index == 0 || keys[index] != keys[index - 1] ?
                                static_cast<int>(index) :
                                0;
    }

    struct MaxRunStart
    {
        __host__ __device__ int operator()(int a, int b) const
        {
            return a > b ? a : b;
        }
    };

    __global__ void OptixEFBVH_mark_group_heads_kernel(const int* run_starts,
                                                       int        leaf_count,
                                                       int        group_size,
                                                       int*       heads)
    {
        const SizeT index = SizeT(blockIdx.x) * blockDim.x + threadIdx.x;
        if(index < leaf_count)
            heads[index] = (static_cast<int>(index) - run_starts[index]) % group_size == 0;
    }

    __global__ void OptixEFBVH_write_group_starts_kernel(
        const int* heads, const int* offsets, int leaf_count, int* starts, int* group_count)
    {
        const SizeT index = SizeT(blockIdx.x) * blockDim.x + threadIdx.x;
        if(index >= leaf_count)
            return;
        if(heads[index])
            starts[offsets[index]] = static_cast<int>(index);
        if(index + 1 == leaf_count)
            *group_count = offsets[index] + heads[index];
    }

    __global__ void OptixEFBVH_aggregate_groups_kernel(const Proxy* leaves,
                                                       const int*   indices,
                                                       const int*   starts,
                                                       int          leaf_count,
                                                       int          group_count,
                                                       Group*       groups)
    {
        const SizeT index = SizeT(blockIdx.x) * blockDim.x + threadIdx.x;
        if(index >= group_count)
            return;
        const int begin = starts[index];
        const int end = index + 1 < group_count ? starts[index + 1] : leaf_count;
        UIPC_KERNEL_ASSERT(begin >= 0 && begin < end && end <= leaf_count
                               && end - begin <= OptixEFBVH::max_group_size,
                           "OptiX EF group %d has invalid range [%d, %d)",
                           static_cast<int>(index),
                           begin,
                           end);
        Proxy bounds = leaves[indices[begin]];
        for(int offset = begin + 1; offset < end; ++offset)
        {
            const Proxy leaf = leaves[indices[offset]];
            for(int axis = 0; axis < 3; ++axis)
            {
                bounds.lower[axis] = fminf(bounds.lower[axis], leaf.lower[axis]);
                bounds.upper[axis] = fmaxf(bounds.upper[axis], leaf.upper[axis]);
            }
            if(bounds.bid != leaf.bid)
                bounds.bid = -1;
            if(bounds.cid != leaf.cid)
                bounds.cid = -1;
        }
        groups[index] = {bounds, begin, end - begin};
    }

    __global__ void OptixEFBVH_prepare_gas_bounds_kernel(const Proxy* leaves,
                                                         const Group* groups,
                                                         int primitive_count,
                                                         const BoundsStats* reduced,
                                                         optix_ef::Transform* transform_output,
                                                         OptixAabb* bounds)
    {
        const SizeT index = SizeT(blockIdx.x) * blockDim.x + threadIdx.x;
        if(index >= primitive_count)
            return;
        const BoundsStats   stats = *reduced;
        optix_ef::Transform transform;
        for(int axis = 0; axis < 3; ++axis)
        {
            const double extent    = stats.upper[axis] - stats.lower[axis];
            transform.origin[axis] = stats.lower[axis];
            transform.inverse_extent[axis] = extent > 0.0 ? 1.0 / extent : 1.0;
        }
        if(index == 0)
            *transform_output = transform;
        const Proxy leaf = groups ? groups[index].bounds : leaves[index];
        float       lower[3], upper[3];
        for(int axis = 0; axis < 3; ++axis)
        {
            const double padding = axis > 0 ? stats.half_extent[axis - 1] : 0.0;
            const double normalized_lower =
                (double(leaf.lower[axis]) - transform.origin[axis] - padding)
                * transform.inverse_extent[axis];
            const double normalized_upper =
                (double(leaf.upper[axis]) - transform.origin[axis] + padding)
                * transform.inverse_extent[axis];
            lower[axis] = __double2float_rd(normalized_lower - rounding_margin);
            upper[axis] = __double2float_ru(normalized_upper + rounding_margin);
        }
        bounds[index] = {lower[0], lower[1], lower[2], upper[0], upper[1], upper[2]};
    }

    __global__ void OptixEFBVH_prepare_launch_kernel(LaunchParams params,
                                                     bvh_simplex_detail::NodePredicate predicate,
                                                     LaunchParams* destination)
    {
        params.body_self_collision = predicate.body_self_collision.data();
        // Obtain the first logical element on the device so Dense2D subview
        // offsets remain honored as well as its physical row pitch.
        params.contact_masks = predicate.cmts.total_size() ? &predicate.cmts(0, 0) : nullptr;
        params.contact_mask_pitch = predicate.cmts.pitch_bytes();
        params.reserved           = 0;
        *params.pair_count        = 0;
        *destination              = params;
    }

    struct alignas(OPTIX_SBT_RECORD_ALIGNMENT) SbtRecord
    {
        char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    };
}  // namespace

class OptixEFBVH::Impl
{
  public:
    ~Impl()
    {
        if(pipeline)
            optixPipelineDestroy(pipeline);
        for(auto group : groups)
            if(group)
                optixProgramGroupDestroy(group);
        if(module)
            optixModuleDestroy(module);
        if(context)
            optixDeviceContextDestroy(context);
    }

    void initialize_pipeline()
    {
        if(pipeline)
            return;
        CUDA_TOOL_CHECK(cudaFree(nullptr));
        check_optix(optixInit(), "optixInit");
        OptixDeviceContextOptions context_options{};
        check_optix(optixDeviceContextCreate(nullptr, &context_options, &context),
                    "optixDeviceContextCreate");

        OptixModuleCompileOptions module_options{};
        module_options.optLevel   = OPTIX_COMPILE_OPTIMIZATION_LEVEL_3;
        module_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;
        OptixPipelineCompileOptions pipeline_options{};
        pipeline_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
        pipeline_options.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_CUSTOM;
        pipeline_options.pipelineLaunchParamsVariableName = "uipc_optix_ef_params";
#if OPTIX_VERSION >= 90100
        pipeline_options.pipelineLaunchParamsSizeInBytes = sizeof(LaunchParams);
#endif
        std::array<char, 8192> log{};
        size_t                 log_size = log.size();
        auto                   result   = optixModuleCreate(context,
                                        &module_options,
                                        &pipeline_options,
                                        uipc_optix_ef_device_ptx,
                                        uipc_optix_ef_device_ptx_size,
                                        log.data(),
                                        &log_size,
                                        &module);
        check_optix(result, "optixModuleCreate(EF)", log.data());

        std::array<OptixProgramGroupDesc, 3> descriptions{};
        descriptions[0].kind          = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
        descriptions[0].raygen.module = module;
        descriptions[0].raygen.entryFunctionName = "__raygen__ef";
        descriptions[1].kind                   = OPTIX_PROGRAM_GROUP_KIND_MISS;
        descriptions[1].miss.module            = module;
        descriptions[1].miss.entryFunctionName = "__miss__ef";
        descriptions[2].kind              = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
        descriptions[2].hitgroup.moduleIS = module;
        descriptions[2].hitgroup.entryFunctionNameIS = "__intersection__ef";
        descriptions[2].hitgroup.moduleAH            = module;
        descriptions[2].hitgroup.entryFunctionNameAH = "__anyhit__ef";
        OptixProgramGroupOptions group_options{};
        log.fill(0);
        log_size = log.size();
        result   = optixProgramGroupCreate(context,
                                         descriptions.data(),
                                         static_cast<unsigned int>(descriptions.size()),
                                         &group_options,
                                         log.data(),
                                         &log_size,
                                         groups.data());
        check_optix(result, "optixProgramGroupCreate(EF)", log.data());

        OptixPipelineLinkOptions link_options{};
        link_options.maxTraceDepth = 1;
        log.fill(0);
        log_size = log.size();
        result   = optixPipelineCreate(context,
                                     &pipeline_options,
                                     &link_options,
                                     groups.data(),
                                     static_cast<unsigned int>(groups.size()),
                                     log.data(),
                                     &log_size,
                                     &pipeline);
        check_optix(result, "optixPipelineCreate(EF)", log.data());
        OptixStackSizes stack_sizes{};
        for(auto group : groups)
            check_optix(optixUtilAccumulateStackSizes(group, &stack_sizes, pipeline),
                        "optixUtilAccumulateStackSizes(EF)");
        unsigned int traversal = 0, state = 0, continuation = 0;
        check_optix(optixUtilComputeStackSizes(&stack_sizes, 1, 0, 0, &traversal, &state, &continuation),
                    "optixUtilComputeStackSizes(EF)");
        check_optix(optixPipelineSetStackSize(pipeline, traversal, state, continuation, 1),
                    "optixPipelineSetStackSize(EF)");

        std::array<SbtRecord, 3> records{};
        for(size_t index = 0; index < records.size(); ++index)
            check_optix(optixSbtRecordPackHeader(groups[index], &records[index]),
                        "optixSbtRecordPackHeader(EF)");
        sbt_records.copy_from(records.data(), records.size());
        const auto first_record = reinterpret_cast<CUdeviceptr>(sbt_records.data());
        sbt.raygenRecord                = first_record;
        sbt.missRecordBase              = first_record + sizeof(SbtRecord);
        sbt.missRecordCount             = 1;
        sbt.missRecordStrideInBytes     = sizeof(SbtRecord);
        sbt.hitgroupRecordBase          = first_record + 2 * sizeof(SbtRecord);
        sbt.hitgroupRecordCount         = 1;
        sbt.hitgroupRecordStrideInBytes = sizeof(SbtRecord);
        transform.resize_discard(1);
        reduced.resize_discard(1);
        launch_params.resize_discard(1);
    }

    void enable_culling_statistics(bool enabled)
    {
        if(enabled && !culling_enabled)
        {
            culling.resize_discard(1);
            CUDA_TOOL_CHECK(cudaMemsetAsync(
                culling.data(), 0, sizeof(CullingCounters), cuda_tool::default_stream()));
        }
        culling_enabled = enabled;
    }

    CullingStatistics culling_statistics() const
    {
        if(!culling_enabled)
            return {};
        CullingCounters counts{};
        culling.copy_to(&counts);
        CUDA_TOOL_CHECK(cudaStreamSynchronize(cuda_tool::default_stream()));
        return {counts.group_tests,
                counts.body_rejections,
                counts.contact_rejections,
                counts.skipped_leaves,
                counts.leaf_tests,
                counts.emitted_pairs};
    }

    SizeT group_count() const noexcept
    {
        return static_cast<SizeT>(built_primitive_count);
    }

    void regroup(int leaf_count, int group_size)
    {
        const auto stream = cuda_tool::default_stream();
        morton_keys.resize_discard(leaf_count);
        sorted_morton_keys.resize_discard(leaf_count);
        identity_indices.resize_discard(leaf_count);
        morton_indices.resize_discard(leaf_count);
        tag_keys.resize_discard(leaf_count);
        sorted_tag_keys.resize_discard(leaf_count);
        group_leaf_indices.resize_discard(leaf_count);
        run_markers.resize_discard(leaf_count);
        run_starts.resize_discard(leaf_count);
        group_heads.resize_discard(leaf_count);
        // CUB WarpSpeed uses a masked 16-byte tail store. Give initcheck a
        // fully allocated granule without initializing scan outputs.
        const SizeT scan_capacity = (SizeT(leaf_count) + 3) / 4 * 4;
        group_offsets.reserve_discard_amortized(scan_capacity);
        group_offsets.resize_discard(leaf_count);
        group_starts.resize_discard(leaf_count);
        device_group_count.resize_discard(1);

        int block_size = cuda_tool::best_block_dim(OptixEFBVH_prepare_morton_keys_kernel);
        OptixEFBVH_prepare_morton_keys_kernel<<<(SizeT(leaf_count) + block_size - 1) / block_size, block_size, 0, stream>>>(
            leaves.data(),
            leaf_count,
            reduced.data(),
            morton_keys.data(),
            identity_indices.data());
        // CUB radix sorting is stable, so the second sort retains Morton order
        // inside each exact (body, contact) run.
        DeviceRadixSort(stream).SortPairs(morton_keys.data(),
                                          sorted_morton_keys.data(),
                                          identity_indices.data(),
                                          morton_indices.data(),
                                          leaf_count,
                                          0,
                                          30);
        block_size = cuda_tool::best_block_dim(OptixEFBVH_prepare_tag_keys_kernel);
        OptixEFBVH_prepare_tag_keys_kernel<<<(SizeT(leaf_count) + block_size - 1) / block_size, block_size, 0, stream>>>(
            leaves.data(), morton_indices.data(), leaf_count, tag_keys.data());
        DeviceRadixSort(stream).SortPairs(tag_keys.data(),
                                          sorted_tag_keys.data(),
                                          morton_indices.data(),
                                          group_leaf_indices.data(),
                                          leaf_count);
        block_size = cuda_tool::best_block_dim(OptixEFBVH_mark_tag_runs_kernel);
        OptixEFBVH_mark_tag_runs_kernel<<<(SizeT(leaf_count) + block_size - 1) / block_size, block_size, 0, stream>>>(
            sorted_tag_keys.data(), leaf_count, run_markers.data());
        DeviceScan(stream).InclusiveScan(
            run_markers.data(), run_starts.data(), MaxRunStart{}, leaf_count);
        block_size = cuda_tool::best_block_dim(OptixEFBVH_mark_group_heads_kernel);
        OptixEFBVH_mark_group_heads_kernel<<<(SizeT(leaf_count) + block_size - 1) / block_size, block_size, 0, stream>>>(
            run_starts.data(), leaf_count, group_size, group_heads.data());
        DeviceScan(stream).ExclusiveSum(group_heads.data(), group_offsets.data(), leaf_count);
        block_size = cuda_tool::best_block_dim(OptixEFBVH_write_group_starts_kernel);
        OptixEFBVH_write_group_starts_kernel<<<(SizeT(leaf_count) + block_size - 1) / block_size, block_size, 0, stream>>>(
            group_heads.data(),
            group_offsets.data(),
            leaf_count,
            group_starts.data(),
            device_group_count.data());
        CUDA_TOOL_CHECK(cudaGetLastError());
        device_group_count.copy_to(&primitive_count);
        CUDA_TOOL_CHECK(cudaStreamSynchronize(stream));
        if(primitive_count <= 0 || primitive_count > leaf_count)
            throw std::runtime_error("OptiX EF grouping produced an invalid primitive count");
        leaf_groups.resize_discard(primitive_count);
        membership_leaf_count = leaf_count;
        membership_group_size = group_size;
    }

    void update(CBufferView<AABB>   query_boxes,
                CBufferView<IndexT> query_bids,
                CBufferView<IndexT> query_cids,
                CBufferView<AABB>   leaf_boxes,
                CBufferView<IndexT> leaf_bids,
                CBufferView<IndexT> leaf_cids,
                bool                reuse_topology,
                SizeT               group_size)
    {
        if(query_boxes.size() == 0 || leaf_boxes.size() == 0
           || query_boxes.size() != query_bids.size()
           || query_boxes.size() != query_cids.size()
           || leaf_boxes.size() != leaf_bids.size()
           || leaf_boxes.size() != leaf_cids.size())
            throw std::invalid_argument("OptiX EF proxy arrays have invalid dimensions");
        const SizeT total_count = query_boxes.size() + leaf_boxes.size();
        if(total_count > static_cast<SizeT>(std::numeric_limits<int>::max()))
            throw std::overflow_error("OptiX EF proxy count exceeds 32-bit indexing");
        initialize_pipeline();
        const auto stream = cuda_tool::default_stream();
        if(culling_enabled)
            CUDA_TOOL_CHECK(cudaMemsetAsync(culling.data(), 0, sizeof(CullingCounters), stream));
        const int block_count =
            static_cast<int>((total_count + bounds_block_size - 1) / bounds_block_size);
        queries.resize_discard(query_boxes.size());
        leaves.resize_discard(leaf_boxes.size());
        partials.resize_discard(block_count);
        OptixEFBVH_prepare_proxies_kernel<<<block_count, bounds_block_size, 0, stream>>>(
            query_boxes,
            query_bids,
            query_cids,
            leaf_boxes,
            leaf_bids,
            leaf_cids,
            queries.data(),
            leaves.data(),
            partials.data());
        CUDA_TOOL_CHECK(cudaGetLastError());
        DeviceReduce(stream).Reduce(partials.data(),
                                    reduced.data(),
                                    block_count,
                                    MergeBoundsStats{},
                                    empty_bounds_stats());
        const int  leaf_count = static_cast<int>(leaf_boxes.size());
        const bool grouped    = group_size > 1;
        const bool rebuild_membership =
            grouped
            && (!reuse_topology || membership_leaf_count != leaf_count
                || membership_group_size != group_size || active_group_size != group_size);
        if(rebuild_membership)
            regroup(leaf_count, static_cast<int>(group_size));
        else if(!grouped)
            primitive_count = leaf_count;
        if(grouped)
        {
            const int block_size =
                cuda_tool::best_block_dim(OptixEFBVH_aggregate_groups_kernel);
            OptixEFBVH_aggregate_groups_kernel<<<(SizeT(primitive_count) + block_size - 1) / block_size, block_size, 0, stream>>>(
                leaves.data(),
                group_leaf_indices.data(),
                group_starts.data(),
                leaf_count,
                primitive_count,
                leaf_groups.data());
        }
        gas_bounds.resize_discard(primitive_count);
        const int block_size =
            cuda_tool::best_block_dim(OptixEFBVH_prepare_gas_bounds_kernel);
        OptixEFBVH_prepare_gas_bounds_kernel<<<(SizeT(primitive_count) + block_size - 1) / block_size, block_size, 0, stream>>>(
            leaves.data(),
            grouped ? leaf_groups.data() : nullptr,
            primitive_count,
            reduced.data(),
            transform.data(),
            gas_bounds.data());
        CUDA_TOOL_CHECK(cudaGetLastError());

        CUdeviceptr aabb_address = reinterpret_cast<CUdeviceptr>(gas_bounds.data());
        unsigned int geometry_flags = OPTIX_GEOMETRY_FLAG_REQUIRE_SINGLE_ANYHIT_CALL;
        OptixBuildInput input{};
        input.type = OPTIX_BUILD_INPUT_TYPE_CUSTOM_PRIMITIVES;
        input.customPrimitiveArray.aabbBuffers = &aabb_address;
        input.customPrimitiveArray.numPrimitives =
            static_cast<unsigned int>(primitive_count);
        input.customPrimitiveArray.strideInBytes = sizeof(OptixAabb);
        input.customPrimitiveArray.flags         = &geometry_flags;
        input.customPrimitiveArray.numSbtRecords = 1;
        OptixAccelBuildOptions build_options{};
        build_options.buildFlags =
            OPTIX_BUILD_FLAG_ALLOW_UPDATE | OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
        const bool refit = reuse_topology && !rebuild_membership && traversable != 0
                           && built_primitive_count == primitive_count
                           && active_group_size == group_size;
        build_options.operation =
            refit ? OPTIX_BUILD_OPERATION_UPDATE : OPTIX_BUILD_OPERATION_BUILD;
        if(built_primitive_count != primitive_count || !traversable)
        {
            OptixAccelBufferSizes sizes{};
            check_optix(optixAccelComputeMemoryUsage(context, &build_options, &input, 1, &sizes),
                        "optixAccelComputeMemoryUsage(EF)");
            gas.resize_discard(sizes.outputSizeInBytes);
            scratch.resize_discard(std::max(sizes.tempSizeInBytes, sizes.tempUpdateSizeInBytes));
        }
        check_optix(optixAccelBuild(context,
                                    stream,
                                    &build_options,
                                    &input,
                                    1,
                                    reinterpret_cast<CUdeviceptr>(scratch.data()),
                                    scratch.size(),
                                    reinterpret_cast<CUdeviceptr>(gas.data()),
                                    gas.size(),
                                    &traversable,
                                    nullptr,
                                    0),
                    refit ? "optixAccelBuild(EF UPDATE)" : "optixAccelBuild(EF BUILD)");
        built_primitive_count = primitive_count;
        active_group_size     = group_size;
        if(refit)
            ++stats.refits;
        else
            ++stats.builds;
    }

    void launch_query(SizeT                                    start,
                      SizeT                                    count,
                      const bvh_simplex_detail::NodePredicate& predicate,
                      InfoStacklessBVH::QueryBuffer&           output)
    {
        constexpr auto pair_limit = std::numeric_limits<int>::max() - 1;
        if(count > OptixEFBVH::max_query_count)
            throw std::overflow_error("OptiX EF query exceeds the 2^30 launch limit");
        if(!traversable || start > queries.size() || count > queries.size() - start)
            throw std::invalid_argument("OptiX EF query range requires an updated GAS");
        if(count > static_cast<SizeT>(pair_limit) / leaves.size()
           || output.m_pairs.size() > static_cast<SizeT>(pair_limit))
            throw std::overflow_error("OptiX EF query exceeds 32-bit pair indexing");
        static_assert(sizeof(Vector2i) == 2 * sizeof(int));
        const auto   stream = cuda_tool::default_stream();
        LaunchParams params{};
        params.traversable = traversable;
        params.queries     = queries.data();
        params.leaves      = leaves.data();
        params.groups = active_group_size > 1 ? leaf_groups.data() : nullptr;
        params.group_leaf_indices =
            active_group_size > 1 ? group_leaf_indices.data() : nullptr;
        params.culling       = culling_enabled ? culling.data() : nullptr;
        params.transform     = transform.data();
        params.query_start   = static_cast<int>(start);
        params.query_count   = static_cast<int>(count);
        params.pair_count    = output.m_cpNum.data();
        params.pairs         = reinterpret_cast<int*>(output.m_pairs.data());
        params.pair_capacity = output.m_pairs.size();
        OptixEFBVH_prepare_launch_kernel<<<1, 1, 0, stream>>>(
            params, predicate, launch_params.data());
        CUDA_TOOL_CHECK(cudaGetLastError());
        if(count == 0)
            return;
        check_optix(optixLaunch(pipeline,
                                stream,
                                reinterpret_cast<CUdeviceptr>(launch_params.data()),
                                sizeof(LaunchParams),
                                &sbt,
                                static_cast<unsigned int>(count),
                                1,
                                1),
                    "optixLaunch(EF)");
        ++stats.launches;
    }

    Statistics stats{};

  private:
    OptixDeviceContext                context = nullptr;
    OptixModule                       module  = nullptr;
    std::array<OptixProgramGroup, 3>  groups{};
    OptixPipeline                     pipeline = nullptr;
    OptixShaderBindingTable           sbt{};
    OptixTraversableHandle            traversable           = 0;
    int                               built_primitive_count = 0;
    int                               primitive_count       = 0;
    int                               membership_leaf_count = 0;
    SizeT                             membership_group_size = 0;
    SizeT                             active_group_size     = 0;
    bool                              culling_enabled       = false;
    DeviceBuffer<Proxy>               queries, leaves;
    DeviceBuffer<BoundsStats>         partials, reduced;
    DeviceBuffer<optix_ef::Transform> transform;
    DeviceBuffer<OptixAabb>           gas_bounds;
    DeviceBuffer<std::byte>           gas, scratch;
    DeviceBuffer<SbtRecord>           sbt_records;
    DeviceBuffer<LaunchParams>        launch_params;
    DeviceBuffer<Group>               leaf_groups;
    DeviceBuffer<CullingCounters>     culling;
    DeviceBuffer<U32>                 morton_keys, sorted_morton_keys;
    DeviceBuffer<U64>                 tag_keys, sorted_tag_keys;
    DeviceBuffer<int> identity_indices, morton_indices, group_leaf_indices;
    DeviceBuffer<int> run_markers, run_starts, group_heads, group_offsets;
    DeviceBuffer<int> group_starts, device_group_count;
};
}  // namespace uipc::backend::cuda
#else
namespace uipc::backend::cuda
{
class OptixEFBVH::Impl
{
};
}  // namespace uipc::backend::cuda
#endif

namespace uipc::backend::cuda
{
OptixEFBVH::OptixEFBVH()                                 = default;
OptixEFBVH::~OptixEFBVH()                                = default;
OptixEFBVH::OptixEFBVH(OptixEFBVH&&) noexcept            = default;
OptixEFBVH& OptixEFBVH::operator=(OptixEFBVH&&) noexcept = default;

bool OptixEFBVH::compiled() noexcept
{
#if UIPC_WITH_OPTIX
    return true;
#else
    return false;
#endif
}

void OptixEFBVH::set_group_size(SizeT size)
{
    if(size == 0 || size > max_group_size)
        throw std::invalid_argument("OptiX EF group size must be in [1, 64]");
    m_group_size = size;
}

SizeT OptixEFBVH::group_size() const noexcept
{
    return m_group_size;
}

SizeT OptixEFBVH::group_count() const noexcept
{
#if UIPC_WITH_OPTIX
    return m_impl ? m_impl->group_count() : 0;
#else
    return 0;
#endif
}

void OptixEFBVH::enable_culling_statistics(bool enabled)
{
    m_culling_statistics_enabled = enabled;
#if UIPC_WITH_OPTIX
    if(m_impl)
        m_impl->enable_culling_statistics(enabled);
#endif
}

OptixEFBVH::CullingStatistics OptixEFBVH::culling_statistics() const
{
#if UIPC_WITH_OPTIX
    return m_culling_statistics_enabled && m_impl ? m_impl->culling_statistics() :
                                                    CullingStatistics{};
#else
    return {};
#endif
}

void OptixEFBVH::update(cuda_tool::CBufferView<AABB>   query_boxes,
                        cuda_tool::CBufferView<IndexT> query_bids,
                        cuda_tool::CBufferView<IndexT> query_cids,
                        cuda_tool::CBufferView<AABB>   leaf_boxes,
                        cuda_tool::CBufferView<IndexT> leaf_bids,
                        cuda_tool::CBufferView<IndexT> leaf_cids,
                        bool                           reuse_topology)
{
#if UIPC_WITH_OPTIX
    if(!m_impl)
        m_impl = std::make_unique<Impl>();
    m_impl->enable_culling_statistics(m_culling_statistics_enabled);
    m_impl->update(query_boxes, query_bids, query_cids, leaf_boxes, leaf_bids, leaf_cids, reuse_topology, m_group_size);
#else
    throw std::runtime_error("The CUDA backend was built without OptiX support");
#endif
}

void OptixEFBVH::launch_query(SizeT start,
                              SizeT count,
                              const bvh_simplex_detail::NodePredicate& predicate,
                              InfoStacklessBVH::QueryBuffer& output)
{
#if UIPC_WITH_OPTIX
    if(!m_impl)
        throw std::logic_error("OptiX EF query requires update before launch");
    m_impl->launch_query(start, count, predicate, output);
#else
    throw std::runtime_error("The CUDA backend was built without OptiX support");
#endif
}

OptixEFBVH::Statistics OptixEFBVH::statistics() const noexcept
{
#if UIPC_WITH_OPTIX
    return m_impl ? m_impl->stats : Statistics{};
#else
    return {};
#endif
}
}  // namespace uipc::backend::cuda
