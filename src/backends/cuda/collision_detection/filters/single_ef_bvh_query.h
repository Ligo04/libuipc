#pragma once
#include <collision_detection/info_stackless_bvh.h>
#include <collision_detection/filters/simplex_candidate_predicates.cuh>
#include <limits>

namespace uipc::backend::cuda
{
class SingleEFBVHQuery
{
  public:
    SingleEFBVHQuery() = default;

    struct PrimitiveBounds
    {
        cuda_tool::CBufferView<AABB>   aabbs;
        cuda_tool::CBufferView<IndexT> bids;
        cuda_tool::CBufferView<IndexT> cids;
    };

    struct QuerySegment
    {
        Vector3 from, to;
        Float   radius;
    };

    // Unsupported topology returns false without publishing candidate results.
    bool detect(InfoStacklessBVH&                                 triangle_bvh,
                const bvh_simplex_detail::PointTrianglePredicate& pt_pred,
                const bvh_simplex_detail::EdgeEdgePredicate&      ee_pred,
                const bvh_simplex_detail::NodePredicate&          node_pred,
                PrimitiveBounds                                   points,
                PrimitiveBounds                                   edges,
                PrimitiveBounds                                   triangles,
                InfoStacklessBVH::QueryBuffer&                    pts,
                InfoStacklessBVH::QueryBuffer&                    ees,
                bool reuse_bvh_topology = false);

    SizeT max_chunk_edges = std::numeric_limits<IndexT>::max();

    SizeT isolated_point_count() const noexcept
    {
        return m_eligibility == Eligibility::Supported ?
                   m_owned_vertices.size() - m_edge_count :
                   0;
    }
    SizeT standalone_edge_count() const noexcept
    {
        return m_eligibility == Eligibility::Supported ?
                   m_owned_edges.size() - m_triangle_count :
                   0;
    }

  private:
    enum class Eligibility
    {
        Unchecked,
        Supported,
        Unsupported
    };
    Eligibility m_eligibility  = Eligibility::Unchecked;
    const void* m_vertices     = nullptr;
    const void* m_edges        = nullptr;
    const void* m_triangles    = nullptr;
    SizeT       m_vertex_count = 0, m_edge_count = 0, m_triangle_count = 0;
    SizeT       m_position_count = 0;

    bool prepare_topology(const bvh_simplex_detail::PointTrianglePredicate& pt_pred,
                          const bvh_simplex_detail::EdgeEdgePredicate& ee_pred);

    cuda_tool::DeviceBuffer<Vector2i>     m_edge_vertices;
    cuda_tool::DeviceBuffer<Vector3i>     m_triangle_vertices;
    cuda_tool::DeviceBuffer<Vector2i>     m_owned_vertices;
    cuda_tool::DeviceBuffer<Vector3i>     m_owned_edges;
    cuda_tool::DeviceBuffer<AABB>         m_query_boxes, m_leaf_boxes;
    cuda_tool::DeviceBuffer<QuerySegment> m_query_segments;
    cuda_tool::DeviceBuffer<IndexT> m_query_bids, m_query_cids, m_leaf_bids, m_leaf_cids;
    InfoStacklessBVH::QueryBuffer    m_ef_pairs;
    cuda_tool::DeviceBuffer<uint8_t> m_masks;
    cuda_tool::DeviceBuffer<U64> m_pt_counts, m_ee_counts, m_pt_offsets, m_ee_offsets;
    // PT/EE totals and maximum EF count, read together after all chunks finish.
    cuda_tool::DeviceBuffer<U64> m_query_counts, m_chunk_base;
};
}  // namespace uipc::backend::cuda
