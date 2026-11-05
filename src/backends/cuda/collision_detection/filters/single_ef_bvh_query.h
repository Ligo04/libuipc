#pragma once
#include <collision_detection/info_stackless_bvh.h>
#include <collision_detection/filters/simplex_candidate_predicates.cuh>
#include <functional>
#include <limits>
#include <memory>

namespace uipc::backend::cuda
{
class SingleEFBVHQuery
{
  public:
    SingleEFBVHQuery();
    ~SingleEFBVHQuery();

    struct PrimitiveBounds
    {
        cuda_tool::CBufferView<AABB>   aabbs;
        cuda_tool::CBufferView<IndexT> bids;
        cuda_tool::CBufferView<IndexT> cids;
    };
    struct Statistics
    {
        SizeT refreshes = 0, reuses = 0, topology_builds = 0;
        SizeT primary_queries = 0, reverse_queries = 0;
        SizeT stored_pts = 0, stored_ees = 0;
        bool  pt_cover = false, ee_cover = false;
        Float cover_skin = 1.0, other_skin = 0.01;
    };

    // Unsupported topology returns false without publishing candidate results.
    // new_frame follows the pipeline's topology-reuse boundary. Standalone
    // callers can leave it false to treat consecutive detects as one frame.
    bool detect(InfoStacklessBVH&                                 triangle_bvh,
                const bvh_simplex_detail::PointTrianglePredicate& pt_pred,
                const bvh_simplex_detail::EdgeEdgePredicate&      ee_pred,
                const bvh_simplex_detail::NodePredicate&          node_pred,
                PrimitiveBounds                                   points,
                PrimitiveBounds                                   edges,
                PrimitiveBounds                                   triangles,
                InfoStacklessBVH::QueryBuffer&                    pts,
                InfoStacklessBVH::QueryBuffer&                    ees,
                bool                         new_frame       = false,
                const std::function<void()>& before_readback = {});

    bool  force_fresh      = false;
    Float cache_skin_scale = 1.0;
    SizeT max_chunk_edges  = std::numeric_limits<IndexT>::max();

    SizeT      refresh_count() const noexcept;
    SizeT      reuse_count() const noexcept;
    SizeT      isolated_point_count() const noexcept;
    SizeT      standalone_edge_count() const noexcept;
    Statistics statistics() const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
}  // namespace uipc::backend::cuda
