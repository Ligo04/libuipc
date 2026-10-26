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
        // An overflow replays the entire query, so no partial output is retained.
        pairs.resize_discard(grown);
    }

    __global__ void make_query_proxies(SingleEFBVHQuery::PrimitiveBounds points,
                                       SingleEFBVHQuery::PrimitiveBounds edges,
                                       EdgeEdgePredicate     ee_pred,
                                       CBufferView<IndexT>    vertices,
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
        if(i >= edges.aabbs.size())
        {
            // A point without an incident edge owns one query proxy. It only
            // emits PT pairs, and never masquerades as a geometric edge.
            const auto v = owned_vertices(i)[0];
            boxes(i)     = points.aabbs(v);
            bids(i)      = points.bids(v);
            cids(i)      = points.cids(v);
            if(ee_pred.alpha == 0.0)
            {
                const auto p = ee_pred.Ps(vertices(v));
                segments(i)  = {p, p, 0.0};
            }
            return;
        }
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
        if(i >= triangles.aabbs.size())
        {
            // A wire edge has no incident face. Store it in a standalone
            // leaf so the ordinary b < e rule also covers wire-wire EE pairs.
            const auto e = owned_edges(i)[0];
            boxes(i)     = edges.aabbs(e);
            bids(i)      = edges.bids(e);
            cids(i)      = edges.cids(e);
            return;
        }
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
                                     CVarView<int>         ef_count,
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
        masks(i)     = 0;
        pt_counts(i) = 0;
        ee_counts(i) = 0;
        // Scan the reusable capacity, zero-padding inactive entries on device.
        // An overflowing EF query must be replayed before any result is used.
        const int n = *ef_count;
        if(n < 0 || static_cast<SizeT>(n) > ef.size() || i >= n)
            return;
        const auto pair = ef(i);
        const auto e = pair[0] + edge_offset, f = pair[1];
        uint8_t    mask = 0;
        U64        np = 0, ne = 0;
        for(int slot = 0; slot < 2; ++slot)
        {
            const auto v = owned_vertices(e)[slot];
            if(v >= 0 && f < triangles.aabbs.size()
               && points.aabbs(v).intersects(triangles.aabbs(f))
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
            if(e < edges.aabbs.size() && b >= 0 && b < e
               && edges.aabbs(b).intersects(edges.aabbs(e))
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

    __global__ void accumulate_chunk_counts(CVarView<int>    ef_count,
                                            CBufferView<U64> pt_offsets,
                                            CBufferView<U64> ee_offsets,
                                            BufferView<U64>  totals,
                                            BufferView<U64>  chunk_base)
    {
        chunk_base(0)      = totals(0);
        chunk_base(1)      = totals(1);
        const int n        = *ef_count;
        const U64 required = n < 0 ? std::numeric_limits<U64>::max() : U64(n);
        totals(2)          = std::max(totals(2), required);
        if(n >= 0 && static_cast<SizeT>(n) < pt_offsets.size())
        {
            totals(0) += pt_offsets(pt_offsets.size() - 1);
            totals(1) += ee_offsets(ee_offsets.size() - 1);
        }
    }

    __global__ void publish_query_counts(CBufferView<U64> totals,
                                         SizeT            ef_capacity,
                                         SizeT            pt_capacity,
                                         SizeT            ee_capacity,
                                         VarView<int>     pt_count,
                                         VarView<int>     ee_count)
    {
        constexpr U64 limit = std::numeric_limits<IndexT>::max() - 1;
        const bool valid = totals(2) <= ef_capacity && totals(0) <= pt_capacity
                           && totals(1) <= ee_capacity && totals(0) + totals(1) <= limit;
        *pt_count = valid ? static_cast<int>(totals(0)) : 0;
        *ee_count = valid ? static_cast<int>(totals(1)) : 0;
    }

    __global__ void fill_candidates(CBufferView<Vector2i> ef,
                                    CVarView<int>         ef_count,
                                    IndexT                edge_offset,
                                    CBufferView<Vector2i> owned_vertices,
                                    CBufferView<Vector3i> owned_edges,
                                    CBufferView<uint8_t>  masks,
                                    CBufferView<U64>      pt_offsets,
                                    CBufferView<U64>      ee_offsets,
                                    CBufferView<U64>      chunk_base,
                                    BufferView<Vector2i>  pts,
                                    BufferView<Vector2i>  ees)
    {
        const auto i = blockIdx.x * blockDim.x + threadIdx.x;
        const int  n = *ef_count;
        if(n < 0 || static_cast<SizeT>(n) > ef.size() || i >= n)
            return;
        const auto pair = ef(i);
        const auto e = pair[0] + edge_offset, f = pair[1];
        auto p = chunk_base(0) + pt_offsets(i), q = chunk_base(1) + ee_offsets(i);
        const auto mask = masks(i);
        for(int slot = 0; slot < 2; ++slot)
            if((mask & (uint8_t{1} << slot)) && p < pts.size())
                pts(p++) = Vector2i{owned_vertices(e)[slot], f};
        for(int slot = 0; slot < 3; ++slot)
            if((mask & (uint8_t{1} << (slot + 2))) && q < ees.size())
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
    for(SizeT v = 0; v < representatives.size(); ++v)
        if(representatives[v] < 0)
            owned_vertices.emplace_back(static_cast<IndexT>(v), -1);
    for(SizeT e = 0; e < owner_faces.size(); ++e)
        if(owner_faces[e] < 0)
            owned_edges.emplace_back(static_cast<IndexT>(e), -1, -1);
    if(owned_vertices.size() > limit || owned_edges.size() > limit)
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
    const auto query_count = m_owned_vertices.size();
    const auto leaf_count  = m_owned_edges.size();
    m_query_boxes.resize_discard(query_count);
    m_query_bids.resize_discard(query_count);
    m_query_cids.resize_discard(query_count);
    if(pt_pred.alpha == 0.0)
        m_query_segments.resize_discard(query_count);
    m_leaf_boxes.resize_discard(leaf_count);
    m_leaf_bids.resize_discard(leaf_count);
    m_leaf_cids.resize_discard(leaf_count);
    make_query_proxies<<<(query_count + 255) / 256, 256>>>(
        points, edges, ee_pred, pt_pred.Vs, m_edge_vertices, m_owned_vertices, m_query_boxes, m_query_bids, m_query_cids, m_query_segments);
    make_leaf_proxies<<<(leaf_count + 255) / 256, 256>>>(
        points, edges, triangles, m_triangle_vertices, m_owned_edges, m_leaf_boxes, m_leaf_bids, m_leaf_cids);
    triangle_bvh.build(m_leaf_boxes, m_leaf_bids, m_leaf_cids);

    const SizeT pair_limit = std::numeric_limits<IndexT>::max() - 1;
    const auto  chunk_size =
        std::min({query_count, std::max(SizeT{1}, max_chunk_edges), pair_limit / leaf_count});
    m_query_counts.resize_discard(3);
    m_chunk_base.resize_discard(2);
    for(;;)
    {
        const auto capacity =
            std::min(m_ef_pairs.m_pairs.size(), chunk_size * leaf_count);
        m_masks.resize_discard(capacity);
        m_pt_counts.resize_discard(capacity + 1);
        m_ee_counts.resize_discard(capacity + 1);
        m_pt_offsets.resize_discard(capacity + 1);
        m_ee_offsets.resize_discard(capacity + 1);
        m_query_counts.fill(0);
        for(SizeT start = 0; start < query_count; start += chunk_size)
        {
            const auto count = std::min(chunk_size, query_count - start);
            auto       run   = [&](auto predicate)
            {
                triangle_bvh.launch_query(m_query_boxes.view(start, count),
                                          m_query_bids.view(start, count),
                                          m_query_cids.view(start, count),
                                          {},
                                          predicate,
                                          AcceptEdgeFace{},
                                          m_ef_pairs);
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
            count_candidates<<<(capacity + 255) / 256, 256>>>(
                m_ef_pairs.m_pairs.view(0, capacity),
                m_ef_pairs.m_cpNum.cview(),
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
            cuda_tool::DeviceScan().ExclusiveSum(m_pt_counts.data(),
                                                 m_pt_offsets.data(),
                                                 static_cast<int>(capacity + 1));
            cuda_tool::DeviceScan().ExclusiveSum(m_ee_counts.data(),
                                                 m_ee_offsets.data(),
                                                 static_cast<int>(capacity + 1));
            accumulate_chunk_counts<<<1, 1>>>(m_ef_pairs.m_cpNum.cview(),
                                              m_pt_offsets.view(),
                                              m_ee_offsets.view(),
                                              m_query_counts.view(),
                                              m_chunk_base.view());
            fill_candidates<<<(capacity + 255) / 256, 256>>>(
                m_ef_pairs.m_pairs.view(0, capacity),
                m_ef_pairs.m_cpNum.cview(),
                static_cast<IndexT>(start),
                m_owned_vertices,
                m_owned_edges,
                m_masks,
                m_pt_offsets,
                m_ee_offsets,
                m_chunk_base,
                pts.m_pairs.view(),
                ees.m_pairs.view());
        }
        publish_query_counts<<<1, 1>>>(m_query_counts.view(),
                                       capacity,
                                       pts.m_pairs.size(),
                                       ees.m_pairs.size(),
                                       pts.m_cpNum.view(),
                                       ees.m_cpNum.view());
        // The normal path has one readback after every chunk has completed.
        // Overflow is exceptional: grow reusable storage and replay all chunks.
        std::array<U64, 3> counts{};
        m_query_counts.copy_to(counts.data());
        if(counts[2] > pair_limit)
            throw std::overflow_error("Single EF query counter exceeds 32-bit indexing");
        if(counts[2] > capacity)
        {
            grow_candidates(m_ef_pairs.m_pairs, counts[2], triangle_bvh.config().reserve_ratio);
            continue;
        }
        if(counts[0] > pair_limit || counts[1] > pair_limit || counts[0] + counts[1] > pair_limit)
            throw std::overflow_error("Single EF candidate count exceeds 32-bit downstream indexing");
        if(counts[0] > pts.m_pairs.size() || counts[1] > ees.m_pairs.size())
        {
            grow_candidates(pts.m_pairs, counts[0], triangle_bvh.config().reserve_ratio);
            grow_candidates(ees.m_pairs, counts[1], triangle_bvh.config().reserve_ratio);
            continue;
        }
        pts.m_size = counts[0];
        ees.m_size = counts[1];
        return true;
    }
}
}  // namespace uipc::backend::cuda
