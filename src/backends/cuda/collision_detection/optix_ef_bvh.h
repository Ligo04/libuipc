#pragma once
#include <collision_detection/filters/simplex_candidate_predicates.cuh>
#include <memory>

namespace uipc::backend::cuda
{
class OptixEFBVH
{
  public:
    struct Statistics
    {
        U64 builds   = 0;
        U64 refits   = 0;
        U64 launches = 0;
    };

    struct CullingStatistics
    {
        U64 group_tests        = 0;
        U64 body_rejections    = 0;
        U64 contact_rejections = 0;
        U64 skipped_leaves     = 0;
        U64 leaf_tests         = 0;
        U64 emitted_pairs      = 0;
    };

    OptixEFBVH();
    ~OptixEFBVH();
    OptixEFBVH(OptixEFBVH&&) noexcept;
    OptixEFBVH& operator=(OptixEFBVH&&) noexcept;

    static bool compiled() noexcept;
    // OptiX 9.1 Programming Guide, Limits: width * height * depth <= 2^30.
    static constexpr SizeT max_query_count    = SizeT{1} << 30;
    static constexpr SizeT default_group_size = 16;
    static constexpr SizeT max_group_size     = 64;

    // Membership changes take effect at the next update; size one is flat.
    void  set_group_size(SizeT size);
    SizeT group_size() const noexcept;
    SizeT group_count() const noexcept;
    void  enable_culling_statistics(bool enabled);
    // A blocking device read only when enabled. Counters reset at update and
    // count invocation/retry work. Skipped leaves count metadata rejections.
    CullingStatistics culling_statistics() const;

    void update(cuda_tool::CBufferView<AABB>   query_boxes,
                cuda_tool::CBufferView<IndexT> query_bids,
                cuda_tool::CBufferView<IndexT> query_cids,
                cuda_tool::CBufferView<AABB>   leaf_boxes,
                cuda_tool::CBufferView<IndexT> leaf_bids,
                cuda_tool::CBufferView<IndexT> leaf_cids,
                bool                           reuse_topology);

    void launch_query(SizeT                                    start,
                      SizeT                                    count,
                      const bvh_simplex_detail::NodePredicate& predicate,
                      InfoStacklessBVH::QueryBuffer&           output);

    Statistics statistics() const noexcept;

  private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
    SizeT                 m_group_size                 = default_group_size;
    bool                  m_culling_statistics_enabled = false;
};
}  // namespace uipc::backend::cuda
