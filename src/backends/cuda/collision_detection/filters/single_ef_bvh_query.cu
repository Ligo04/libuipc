#include <collision_detection/filters/single_ef_bvh_query.h>
#include <collision_detection/segment_aabb.h>
#include <cuda_tool/cub.h>
#include <algorithm>
#include <array>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace uipc::backend::cuda
{
namespace
{
    using namespace cuda_tool;
    using namespace bvh_simplex_detail;

    U64 edge_key(IndexT a, IndexT b)
    {
        if(a > b)
            std::swap(a, b);
        return (U64{static_cast<U32>(a)} << 32) | static_cast<U32>(b);
    }

    template <typename T>
    void upload(DeviceBuffer<T>& buffer, const std::vector<T>& data)
    {
        buffer.resize_discard(data.size());
        buffer.view().copy_from(data.data());
    }

    void grow_candidates(DeviceBuffer<Vector2i>& pairs, SizeT required, Float reserve_ratio)
    {
        if(required <= pairs.size())
            return;
        constexpr auto limit = std::numeric_limits<IndexT>::max() - 1;
        const auto     grown = static_cast<SizeT>(
            std::min(static_cast<Float>(limit),
                     std::max(static_cast<Float>(required), required * reserve_ratio)));
        pairs.resize_preserve(grown);
    }

    __global__ void make_query_proxies(SingleEFBVHQuery::PrimitiveBounds points,
                                       SingleEFBVHQuery::PrimitiveBounds edges,
                                       EdgeEdgePredicate     ee_pred,
                                       CBufferView<Vector2i> edge_vertices,
                                       CBufferView<Vector2i> owned_vertices,
                                       BufferView<AABB>      boxes,
                                       BufferView<IndexT>    bids,
                                       BufferView<IndexT>    cids,
                                       BufferView<SingleEFBVHQuery::QuerySegment> segments)
    {
        const auto i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= boxes.size())
            return;
        auto box = edges.aabbs(i);
        for(int slot = 0; slot < 2; ++slot)
        {
            const auto v = owned_vertices(i)[slot];
            if(v >= 0)
                box.extend(points.aabbs(v));
        }
        const auto ends = edge_vertices(i);
        const auto bid0 = points.bids(ends[0]), bid1 = points.bids(ends[1]);
        const auto cid0 = points.cids(ends[0]), cid1 = points.cids(ends[1]);
        boxes(i) = box;
        bids(i)  = bid0 == bid1 ? bid0 : -1;
        cids(i)  = cid0 == cid1 ? cid0 : -1;
        if(ee_pred.alpha == 0.0)
        {
            const auto e = ee_pred.Es(i);
            segments(i)  = {ee_pred.Ps(e[0]),
                            ee_pred.Ps(e[1]),
                            edge_dcd_expansion(ee_pred.d_hats(e[0]), ee_pred.d_hats(e[1]))
                                + edge_thickness(ee_pred.thicknesses(e[0]),
                                                ee_pred.thicknesses(e[1]))};
        }
    }

    __global__ void make_leaf_proxies(SingleEFBVHQuery::PrimitiveBounds points,
                                      SingleEFBVHQuery::PrimitiveBounds edges,
                                      SingleEFBVHQuery::PrimitiveBounds triangles,
                                      CBufferView<Vector3i> triangle_vertices,
                                      CBufferView<Vector3i> owned_edges,
                                      BufferView<AABB>      boxes,
                                      BufferView<IndexT>    bids,
                                      BufferView<IndexT>    cids)
    {
        const auto i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= boxes.size())
            return;
        auto       box = triangles.aabbs(i);
        const auto vs  = triangle_vertices(i);
        auto       bid = points.bids(vs[0]);
        auto       cid = points.cids(vs[0]);
        for(int slot = 0; slot < 3; ++slot)
        {
            const auto e = owned_edges(i)[slot];
            if(e >= 0)
                box.extend(edges.aabbs(e));
            if(bid != points.bids(vs[slot]))
                bid = -1;
            if(cid != points.cids(vs[slot]))
                cid = -1;
        }
        boxes(i) = box;
        bids(i)  = bid;
        cids(i)  = cid;
    }

    struct AcceptEdgeFace
    {
        __device__ bool operator()(const InfoStacklessBVH::LeafPredInfo&) const
        {
            return true;
        }
    };

    struct PreparedEdgeFaceNodePredicate
    {
        NodePredicate    tags;
        SegmentAABBQuery segment;
        AABB             owned_points[2];
        uint8_t          owned_mask = 0;

        __device__ bool operator()(InfoStacklessBVH::NodePredInfo info, const AABB& bound) const
        {
            if(!tags(info))
                return false;
            // Owned endpoint boxes are a cheap PT acceptance route. Their
            // expansion need not equal the edge's activation expansion.
            for(int slot = 0; slot < 2; ++slot)
                if((owned_mask & (1 << slot)) && owned_points[slot].intersects(bound))
                    return true;
            // Cover fl(fl(x) +/- fl(radius)), including cancellation.
            const Float scale = std::max(bound.min().cwiseAbs().maxCoeff(),
                                         bound.max().cwiseAbs().maxCoeff());
            const Float roundoff = 8.0 * std::numeric_limits<float>::epsilon() * scale
                                   + 8.0 * std::numeric_limits<float>::denorm_min();
            return segment.intersects(bound, roundoff);
        }
    };

    struct EdgeFaceNodePredicate
    {
        NodePredicate                               tags;
        CBufferView<SingleEFBVHQuery::QuerySegment> segments;
        CBufferView<Vector2i>                       owned_vertices;
        CBufferView<AABB>                           point_boxes;
        IndexT                                      edge_offset;

        __device__ auto prepare_query(IndexT query_id) const
        {
            PreparedEdgeFaceNodePredicate prepared;
            prepared.tags = tags;
            if(query_id < 0)
                return prepared;
            const auto edge          = query_id + edge_offset;
            const auto query_segment = segments(edge);
            prepared.segment         = SegmentAABBQuery(
                query_segment.from, query_segment.to, query_segment.radius);
            const auto vs = owned_vertices(edge);
            for(int slot = 0; slot < 2; ++slot)
                if(vs[slot] >= 0)
                {
                    prepared.owned_points[slot] = point_boxes(vs[slot]);
                    prepared.owned_mask |= 1 << slot;
                }
            return prepared;
        }
    };

    __global__ void count_candidates(CBufferView<Vector2i> ef,
                                     IndexT                edge_offset,
                                     CBufferView<Vector2i> owned_vertices,
                                     CBufferView<Vector3i> owned_edges,
                                     SingleEFBVHQuery::PrimitiveBounds points,
                                     SingleEFBVHQuery::PrimitiveBounds edges,
                                     SingleEFBVHQuery::PrimitiveBounds triangles,
                                     PointTrianglePredicate pt_pred,
                                     EdgeEdgePredicate      ee_pred,
                                     NodePredicate          node_pred,
                                     BufferView<uint8_t>    masks,
                                     BufferView<U64>        pt_counts,
                                     BufferView<U64>        ee_counts)
    {
        const auto i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i == 0)
        {
            pt_counts(ef.size()) = 0;
            ee_counts(ef.size()) = 0;
        }
        if(i >= ef.size())
            return;
        const auto pair = ef(i);
        const auto e = pair[0] + edge_offset, f = pair[1];
        uint8_t    mask = 0;
        U64        np = 0, ne = 0;
        for(int slot = 0; slot < 2; ++slot)
        {
            const auto v = owned_vertices(e)[slot];
            if(v >= 0 && points.aabbs(v).intersects(triangles.aabbs(f))
               && node_pred({v,
                             points.bids(v),
                             points.cids(v),
                             triangles.bids(f),
                             triangles.cids(f)})
               && pt_pred({v,
                           f,
                           points.bids(v),
                           points.cids(v),
                           triangles.bids(f),
                           triangles.cids(f)}))
            {
                mask |= uint8_t{1} << slot;
                ++np;
            }
        }
        for(int slot = 0; slot < 3; ++slot)
        {
            const auto b = owned_edges(f)[slot];
            if(b >= 0 && b < e && edges.aabbs(b).intersects(edges.aabbs(e))
               && node_pred(
                   {b, edges.bids(b), edges.cids(b), edges.bids(e), edges.cids(e)})
               && ee_pred(
                   {b, e, edges.bids(b), edges.cids(b), edges.bids(e), edges.cids(e)}))
            {
                mask |= uint8_t{1} << (slot + 2);
                ++ne;
            }
        }
        masks(i)     = mask;
        pt_counts(i) = np;
        ee_counts(i) = ne;
    }

    __global__ void fill_candidates(CBufferView<Vector2i> ef,
                                    IndexT                edge_offset,
                                    CBufferView<Vector2i> owned_vertices,
                                    CBufferView<Vector3i> owned_edges,
                                    CBufferView<uint8_t>  masks,
                                    CBufferView<U64>      pt_offsets,
                                    CBufferView<U64>      ee_offsets,
                                    BufferView<Vector2i>  pts,
                                    BufferView<Vector2i>  ees)
    {
        const auto i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= ef.size())
            return;
        const auto pair = ef(i);
        const auto e = pair[0] + edge_offset, f = pair[1];
        auto       p = pt_offsets(i), q = ee_offsets(i);
        const auto mask = masks(i);
        for(int slot = 0; slot < 2; ++slot)
            if(mask & (uint8_t{1} << slot))
                pts(p++) = Vector2i{owned_vertices(e)[slot], f};
        for(int slot = 0; slot < 3; ++slot)
            if(mask & (uint8_t{1} << (slot + 2)))
                ees(q++) = Vector2i{owned_edges(f)[slot], e};
    }
}  // namespace

bool SingleEFBVHQuery::prepare_topology(const bvh_simplex_detail::PointTrianglePredicate& pt,
                                        const bvh_simplex_detail::EdgeEdgePredicate& ee)
{
    if(m_vertices == pt.Vs.data() && m_edges == ee.Es.data()
       && m_triangles == pt.Fs.data() && m_vertex_count == pt.Vs.size()
       && m_edge_count == ee.Es.size() && m_triangle_count == pt.Fs.size()
       && m_position_count == pt.Ps.size() && m_eligibility != Eligibility::Unchecked)
        return m_eligibility == Eligibility::Supported;

    m_vertices       = pt.Vs.data();
    m_edges          = ee.Es.data();
    m_triangles      = pt.Fs.data();
    m_vertex_count   = pt.Vs.size();
    m_edge_count     = ee.Es.size();
    m_triangle_count = pt.Fs.size();
    m_position_count = pt.Ps.size();
    m_eligibility    = Eligibility::Unsupported;
    const auto limit = static_cast<SizeT>(std::numeric_limits<IndexT>::max() - 1);
    if(m_vertex_count == 0 || m_edge_count == 0 || m_triangle_count == 0
       || m_vertex_count > limit || m_edge_count > limit || m_triangle_count > limit)
        return false;

    std::vector<IndexT>   vertices(m_vertex_count);
    std::vector<Vector2i> edges(m_edge_count);
    std::vector<Vector3i> faces(m_triangle_count);
    pt.Vs.copy_to(vertices.data());
    ee.Es.copy_to(edges.data());
    pt.Fs.copy_to(faces.data());
    std::unordered_map<IndexT, IndexT> surface_indices;
    for(SizeT v = 0; v < vertices.size(); ++v)
        if(vertices[v] < 0 || static_cast<SizeT>(vertices[v]) >= m_position_count
           || !surface_indices.emplace(vertices[v], static_cast<IndexT>(v)).second)
            return false;

    std::unordered_map<U64, IndexT> edge_indices;
    std::vector<Vector2i>           edge_vertices(edges.size());
    std::vector<Vector2i> owned_vertices(edges.size(), Vector2i::Constant(-1));
    std::vector<IndexT>   representatives(vertices.size(), -1);
    for(SizeT e = 0; e < edges.size(); ++e)
    {
        const auto a = edges[e][0], b = edges[e][1];
        if(a == b || !surface_indices.contains(a) || !surface_indices.contains(b)
           || !edge_indices.emplace(edge_key(a, b), static_cast<IndexT>(e)).second)
            return false;
        for(int slot = 0; slot < 2; ++slot)
        {
            const auto v           = surface_indices.at(edges[e][slot]);
            edge_vertices[e][slot] = v;
            if(representatives[v] < 0)
            {
                representatives[v]      = static_cast<IndexT>(e);
                owned_vertices[e][slot] = v;
            }
        }
    }

    std::vector<Vector3i> triangle_vertices(faces.size());
    std::vector<Vector3i> owned_edges(faces.size(), Vector3i::Constant(-1));
    std::vector<IndexT>   owner_faces(edges.size(), -1);
    for(SizeT f = 0; f < faces.size(); ++f)
    {
        const auto face = faces[f];
        if(face[0] == face[1] || face[1] == face[2] || face[0] == face[2])
            return false;
        for(int slot = 0; slot < 3; ++slot)
        {
            if(!surface_indices.contains(face[slot]))
                return false;
            triangle_vertices[f][slot] = surface_indices.at(face[slot]);
            const auto it =
                edge_indices.find(edge_key(face[slot], face[(slot + 1) % 3]));
            if(it == edge_indices.end())
                return false;
            const auto e = it->second;
            if(owner_faces[e] < 0)
            {
                owner_faces[e]       = static_cast<IndexT>(f);
                owned_edges[f][slot] = e;
            }
        }
    }
    if(std::find(representatives.begin(), representatives.end(), -1)
           != representatives.end()
       || std::find(owner_faces.begin(), owner_faces.end(), -1) != owner_faces.end())
        return false;

    upload(m_edge_vertices, edge_vertices);
    upload(m_triangle_vertices, triangle_vertices);
    upload(m_owned_vertices, owned_vertices);
    upload(m_owned_edges, owned_edges);
    m_eligibility = Eligibility::Supported;
    return true;
}

bool SingleEFBVHQuery::detect(InfoStacklessBVH& triangle_bvh,
                              const bvh_simplex_detail::PointTrianglePredicate& pt_pred,
                              const bvh_simplex_detail::EdgeEdgePredicate& ee_pred,
                              const bvh_simplex_detail::NodePredicate& node_pred,
                              PrimitiveBounds                points,
                              PrimitiveBounds                edges,
                              PrimitiveBounds                triangles,
                              InfoStacklessBVH::QueryBuffer& pts,
                              InfoStacklessBVH::QueryBuffer& ees)
{
    UIPC_ASSERT(pt_pred.alpha == ee_pred.alpha && pt_pred.alpha >= 0.0,
                "Single EF PT and EE queries must share a nonnegative alpha");
    if(!prepare_topology(pt_pred, ee_pred))
        return false;
    UIPC_ASSERT(points.aabbs.size() == m_vertex_count && edges.aabbs.size() == m_edge_count
                    && triangles.aabbs.size() == m_triangle_count,
                "Single EF primitive bounds do not match topology");
    m_query_boxes.resize_discard(m_edge_count);
    m_query_bids.resize_discard(m_edge_count);
    m_query_cids.resize_discard(m_edge_count);
    if(pt_pred.alpha == 0.0)
        m_query_segments.resize_discard(m_edge_count);
    m_leaf_boxes.resize_discard(m_triangle_count);
    m_leaf_bids.resize_discard(m_triangle_count);
    m_leaf_cids.resize_discard(m_triangle_count);
    make_query_proxies<<<(m_edge_count + 255) / 256, 256>>>(
        points, edges, ee_pred, m_edge_vertices, m_owned_vertices, m_query_boxes, m_query_bids, m_query_cids, m_query_segments);
    make_leaf_proxies<<<(m_triangle_count + 255) / 256, 256>>>(
        points, edges, triangles, m_triangle_vertices, m_owned_edges, m_leaf_boxes, m_leaf_bids, m_leaf_cids);
    triangle_bvh.build(m_leaf_boxes, m_leaf_bids, m_leaf_cids);

    SizeT       used_pts = 0, used_ees = 0;
    const SizeT pair_limit = std::numeric_limits<IndexT>::max() - 1;
    const auto  chunk_size =
        std::min({m_edge_count, std::max(SizeT{1}, max_chunk_edges), pair_limit / m_triangle_count});
    for(SizeT start = 0; start < m_edge_count; start += chunk_size)
    {
        const auto count  = std::min(chunk_size, m_edge_count - start);
        auto       launch = [&](bool rebuild_query)
        {
            auto run = [&](auto predicate)
            {
                triangle_bvh.launch_query(m_query_boxes.view(start, count),
                                          m_query_bids.view(start, count),
                                          m_query_cids.view(start, count),
                                          {},
                                          predicate,
                                          AcceptEdgeFace{},
                                          m_ef_pairs,
                                          rebuild_query);
            };
            // Swept edges are bilinear patches. Use only their full swept AABB,
            // with the original lightweight tag predicate and no static slab.
            if(pt_pred.alpha != 0.0)
                run(node_pred);
            else
                run(EdgeFaceNodePredicate{node_pred,
                                          m_query_segments,
                                          m_owned_vertices,
                                          points.aabbs,
                                          static_cast<IndexT>(start)});
        };
        launch(true);
        const int ef_count = m_ef_pairs.m_cpNum;
        UIPC_ASSERT(ef_count >= 0, "Single EF query counter overflow");
        const auto n = static_cast<SizeT>(ef_count);
        if(n > m_ef_pairs.m_pairs.size())
        {
            // Exact logical size keeps the traversal's signed capacity representable.
            m_ef_pairs.reserve(n);
            launch(false);
        }
        m_ef_pairs.m_size = n;
        if(n == 0)
            continue;
        m_masks.resize_discard(n);
        m_pt_counts.resize_discard(n + 1);
        m_ee_counts.resize_discard(n + 1);
        m_pt_offsets.resize_discard(n + 1);
        m_ee_offsets.resize_discard(n + 1);
        count_candidates<<<(n + 255) / 256, 256>>>(m_ef_pairs.view(),
                                                   static_cast<IndexT>(start),
                                                   m_owned_vertices,
                                                   m_owned_edges,
                                                   points,
                                                   edges,
                                                   triangles,
                                                   pt_pred,
                                                   ee_pred,
                                                   node_pred,
                                                   m_masks,
                                                   m_pt_counts,
                                                   m_ee_counts);
        cuda_tool::DeviceScan().ExclusiveSum(
            m_pt_counts.data(), m_pt_offsets.data(), static_cast<int>(n + 1));
        cuda_tool::DeviceScan().ExclusiveSum(
            m_ee_counts.data(), m_ee_offsets.data(), static_cast<int>(n + 1));
        U64 pt_count = 0, ee_count = 0;
        m_pt_offsets.view(n, 1).copy_to(&pt_count);
        m_ee_offsets.view(n, 1).copy_to(&ee_count);
        const U64 total_pts = used_pts + pt_count, total_ees = used_ees + ee_count;
        if(total_pts > pair_limit || total_ees > pair_limit || total_pts + total_ees > pair_limit)
            throw std::overflow_error("Single EF candidate count exceeds 32-bit downstream indexing");
        grow_candidates(pts.m_pairs, total_pts, triangle_bvh.config().reserve_ratio);
        grow_candidates(ees.m_pairs, total_ees, triangle_bvh.config().reserve_ratio);
        fill_candidates<<<(n + 255) / 256, 256>>>(m_ef_pairs.view(),
                                                  static_cast<IndexT>(start),
                                                  m_owned_vertices,
                                                  m_owned_edges,
                                                  m_masks,
                                                  m_pt_offsets,
                                                  m_ee_offsets,
                                                  pts.m_pairs.view(used_pts, pt_count),
                                                  ees.m_pairs.view(used_ees, ee_count));
        used_pts = total_pts;
        used_ees = total_ees;
    }
    pts.m_size = used_pts;
    ees.m_size = used_ees;
    pts.m_cpNum.fill(static_cast<IndexT>(used_pts));
    ees.m_cpNum.fill(static_cast<IndexT>(used_ees));
    return true;
}
}  // namespace uipc::backend::cuda
