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

    OptixEFBVH();
    ~OptixEFBVH();
    OptixEFBVH(OptixEFBVH&&) noexcept;
    OptixEFBVH& operator=(OptixEFBVH&&) noexcept;

    static bool compiled() noexcept;
    // OptiX 9.1 Programming Guide, Limits: width * height * depth <= 2^30.
    static constexpr SizeT max_query_count = SizeT{1} << 30;

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
};
}  // namespace uipc::backend::cuda
