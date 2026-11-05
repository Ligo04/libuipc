#pragma once
#include <collision_detection/info_stackless_bvh.h>
#include <collision_detection/filters/simplex_candidate_predicates.cuh>
#include <functional>
#include <limits>
#include <memory>

namespace uipc::backend::cuda
{
struct TrajectoryPrefix;
class SimplexBVHQuery
{
  public:
    enum class Method
    {
        InfoStackless,
        SingleEF
    };
    SimplexBVHQuery();
    ~SimplexBVHQuery();

    struct PrimitiveBounds
    {
        cuda_tool::CBufferView<AABB>   aabbs;
        cuda_tool::CBufferView<IndexT> bids;
        cuda_tool::CBufferView<IndexT> cids;
    };
    struct CodimQueries
    {
        PrimitiveBounds                         points;
        bvh_simplex_detail::PointPointPredicate pp;
        bvh_simplex_detail::PointEdgePredicate  pe;
        InfoStacklessBVH::QueryBuffer*          pps = nullptr;
        InfoStacklessBVH::QueryBuffer*          pes = nullptr;
    };
    struct Statistics
    {
        SizeT refreshes = 0, reuses = 0, topology_builds = 0;
        SizeT primary_queries = 0, reverse_queries = 0;
        SizeT stored_pts = 0, stored_ees = 0, stored_pps = 0, stored_pes = 0;
        SizeT ef_pairs = 0, refresh_waits = 0, ef_retries = 0, typed_refills = 0;
        SizeT cross_frame_reuses = 0, fused_prefixes = 0;
        bool  pt_cover = false, ee_cover = false;
        // Effective path, including production's unsupported-EF direct fallback.
        bool  single_ef_active = false;
        Float cover_skin = 1.0, other_skin = 0.01;
    };

    // Standalone EF queries return false on unsupported ownership topology.
    // A bound codimensional context enables the complete primitive-tree fallback.
    // Both methods cache PP/PE when this context is supplied.
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
                const std::function<void()>& before_readback = {},
                TrajectoryPrefix*            prefix          = nullptr,
                const CodimQueries*          codim           = nullptr);

    Method method           = Method::SingleEF;
    bool   force_fresh      = false;
    Float  cache_skin_scale = 1.0;
    SizeT  max_chunk_edges  = std::numeric_limits<IndexT>::max();

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
