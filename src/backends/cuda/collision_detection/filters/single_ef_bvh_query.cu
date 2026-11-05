#include <collision_detection/filters/single_ef_bvh_query.h>
#include <collision_detection/filters/bvh_contact_cover.h>
#include <collision_detection/filters/bvh_pair_judge.h>
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
    using Bounds                   = SingleEFBVHQuery::PrimitiveBounds;
    constexpr SizeT PairLimit      = std::numeric_limits<IndexT>::max() - 1;
    constexpr int   Threads        = 256;
    constexpr int   TopologyPeriod = 8;

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
    void reserve_pairs(DeviceBuffer<Vector2i>& pairs, SizeT n, Float ratio, bool preserve)
    {
        if(n <= pairs.size())
            return;
        if(n > PairLimit)
            throw std::overflow_error("Single EF pair count exceeds 32-bit indexing");
        const SizeT capacity = static_cast<SizeT>(
            std::min(Float(PairLimit), std::max(Float(n), Float(n) * ratio)));
        if(preserve)
            pairs.resize(capacity);
        else
            pairs.resize_discard(capacity);
    }

    // Every position-free input is snapshotted, including independent predicate
    // tables and primitive metadata. In-place edits invalidate the typed cache.
    struct Metadata
    {
        enum Array
        {
            PTContacts,
            EEContacts,
            PTSubscenes,
            EESubscenes,
            PTSelf,
            EESelf,
            NodeSelf,
            PointBodies,
            PointContacts,
            EdgeBodies,
            EdgeContacts,
            FaceBodies,
            FaceContacts,
            Arrays
        };
        enum Matrix
        {
            PTMask,
            EEMask,
            NodeMask,
            PTSubmask,
            EESubmask,
            Matrices
        };
        CBufferView<IndexT> arrays[Arrays];
        CDense2D<IndexT>    matrices[Matrices];
        SizeT               offsets[int(Arrays) + int(Matrices)];
        SizeT               total = 0, work = 0;

        Metadata(const PointTrianglePredicate& pt,
                 const EdgeEdgePredicate&      ee,
                 const NodePredicate&          node,
                 Bounds                        p,
                 Bounds                        e,
                 Bounds                        f)
            : arrays{pt.contact_element_ids,
                     ee.contact_element_ids,
                     pt.subscene_element_ids,
                     ee.subscene_element_ids,
                     pt.body_self_collision,
                     ee.body_self_collision,
                     node.body_self_collision,
                     p.bids,
                     p.cids,
                     e.bids,
                     e.cids,
                     f.bids,
                     f.cids}
            , matrices{pt.contact_mask_tabular,
                       ee.contact_mask_tabular,
                       node.cmts,
                       pt.subscene_mask_tabular,
                       ee.subscene_mask_tabular}
        {
            for(int i = 0; i < int(Arrays) + int(Matrices); ++i)
            {
                const SizeT n = i < Arrays ? arrays[i].size() :
                                             matrices[i - Arrays].total_size();
                offsets[i]    = total;
                total += n;
                work = std::max(work, n);
            }
        }
        bool same_shape(const Metadata& previous) const
        {
            for(int i = 0; i < Arrays; ++i)
                if(arrays[i].size() != previous.arrays[i].size())
                    return false;
            for(int i = 0; i < Matrices; ++i)
                if(matrices[i].dim().x != previous.matrices[i].dim().x
                   || matrices[i].dim().y != previous.matrices[i].dim().y)
                    return false;
            return true;
        }
    };
    struct Validation
    {
        CBufferView<IndexT>   vertices, previous_vertices;
        CBufferView<Vector2i> edges, previous_edges;
        CBufferView<Vector3i> faces, previous_faces;
        Metadata              metadata;
        CBufferView<IndexT>   previous_metadata;
        CBufferView<AABB>     current[3], grown[3];
        CBufferView<IndexT>   groups[3];
        CBufferView<Float>    shrink[3];
        bool                  topology, metadata_check, containment;
    };
    __device__ void invalidate(IndexT* control, int reason)
    {
        atomicExch(control, 1);
        atomicExch(control + reason, 1);
    }
    __global__ void validate_cache(Validation input, IndexT* control, SizeT n)
    {
        const SizeT i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(input.topology)
        {
            if((i < input.vertices.size() && input.vertices(i) != input.previous_vertices(i))
               || (i < input.edges.size() && input.edges(i) != input.previous_edges(i))
               || (i < input.faces.size() && input.faces(i) != input.previous_faces(i)))
                invalidate(control, 1);
        }
        if(input.metadata_check)
        {
            for(int a = 0; a < Metadata::Arrays; ++a)
                if(i < input.metadata.arrays[a].size()
                   && input.metadata.arrays[a](i)
                          != input.previous_metadata(input.metadata.offsets[a] + i))
                    invalidate(control, 2);
            for(int a = 0; a < Metadata::Matrices; ++a)
                if(i < input.metadata.matrices[a].total_size()
                   && input.metadata.matrices[a].flatten(static_cast<int>(i))
                          != input.previous_metadata(
                              input.metadata.offsets[Metadata::Arrays + a] + i))
                    invalidate(control, 2);
        }
        if(input.containment)
            for(int family = 0; family < 3; ++family)
                if(i < input.current[family].size())
                {
                    const auto now = input.current[family](i),
                               fat = input.grown[family](i);
                    const bool contained =
                        (fat.min().array() <= now.min().array()).all()
                        && (now.max().array() <= fat.max().array()).all();
                    const int group = input.groups[family](i);
                    if(!contained)
                    {
                        atomicExch(control, 1);
                        if(group == 0)
                            atomicExch(control + 5, 1);
                    }
                    // The next-lower skin determines next frame's level,
                    // even when the full cache still contains this detect.
                    if(group != 0)
                    {
                        const Float shrink = input.shrink[family](i);
                        bool        lower  = true;
                        for(int axis = 0; axis < 3; ++axis)
                            lower &= Float(fat.min()[axis]) + shrink
                                         <= Float(now.min()[axis])
                                     && Float(now.max()[axis])
                                            <= Float(fat.max()[axis]) - shrink;
                        if(!lower)
                            atomicExch(control + (group == 1 ? 3 : 4), 1);
                    }
                }
    }
    __global__ void save_metadata(Metadata metadata, BufferView<IndexT> snapshot)
    {
        const SizeT i = blockIdx.x * blockDim.x + threadIdx.x;
        for(int a = 0; a < Metadata::Arrays; ++a)
            if(i < metadata.arrays[a].size())
                snapshot(metadata.offsets[a] + i) = metadata.arrays[a](i);
        for(int a = 0; a < Metadata::Matrices; ++a)
            if(i < metadata.matrices[a].total_size())
                snapshot(metadata.offsets[Metadata::Arrays + a] + i) =
                    metadata.matrices[a].flatten(static_cast<int>(i));
    }

    __global__ void grow_primitives(CBufferView<AABB>     current,
                                    CBufferView<IndexT>   groups,
                                    CBufferView<IndexT>   vertices,
                                    CBufferView<Vector2i> edges,
                                    CBufferView<Vector3i> faces,
                                    CBufferView<Float>    d_hats,
                                    int                   family,
                                    Float                 uniform,
                                    Float                 cover,
                                    Float                 other,
                                    BufferView<AABB>      grown,
                                    BufferView<Float>     shrink)
    {
        const SizeT i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= current.size())
            return;
        const IndexT vertex = family == 0 ? vertices(i) :
                              family == 1 ? edges(i)[0] :
                                            faces(i)[0];
        const int    group  = groups(i);
        const Float  skin   = std::max(Float{0},
                                    d_hats(vertex)
                                        * (group == 1 ? cover :
                                              group == 2 ? other :
                                                           uniform));
        auto         box    = current(i);
        for(int axis = 0; axis < 3; ++axis)
        {
            box.min()[axis] =
                nextafterf(static_cast<float>(Float(box.min()[axis]) - skin), -INFINITY);
            box.max()[axis] =
                nextafterf(static_cast<float>(Float(box.max()[axis]) + skin), INFINITY);
        }
        grown(i)  = box;
        shrink(i) = std::max(Float{0},
                             d_hats(vertex)
                                 * (group == 1 ? cover - uniform :
                                    group == 2 ? other - 0.01 * uniform :
                                                 0.0));
    }
    __global__ void make_query_proxies(Bounds                points,
                                       Bounds                edges,
                                       CBufferView<AABB>     grown_points,
                                       CBufferView<AABB>     grown_edges,
                                       CBufferView<Vector2i> owned_vertices,
                                       CBufferView<IndexT>   point_queries,
                                       CBufferView<IndexT>   edge_queries,
                                       BufferView<AABB>      boxes,
                                       BufferView<IndexT>    cids,
                                       BufferView<IndexT>    primary,
                                       BufferView<IndexT>    reverse,
                                       bool                  pt_cover)
    {
        const SizeT i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i == 0)
        {
            primary(boxes.size()) = 0;
            reverse(boxes.size()) = 0;
        }
        if(i >= boxes.size())
            return;
        const auto owned = owned_vertices(i);
        AABB       box;
        IndexT     cid   = -1;
        bool       first = true, a = false, b = false;
        if(i < grown_edges.size())
        {
            box   = grown_edges(i);
            cid   = edges.cids(i);
            first = false;
            a     = edge_queries(i) != 0;
        }
        for(int slot = 0; slot < 2; ++slot)
            if(owned[slot] >= 0)
            {
                const IndexT v = owned[slot];
                if(first)
                {
                    box   = grown_points(v);
                    cid   = points.cids(v);
                    first = false;
                }
                else
                {
                    box.extend(grown_points(v));
                    if(cid != points.cids(v))
                        cid = -1;
                }
                a |= point_queries(v) != 0;
                b |= pt_cover && !point_queries(v);
            }
        boxes(i)   = box;
        cids(i)    = cid;
        primary(i) = a;
        reverse(i) = b;
    }
    __global__ void make_leaf_proxies(Bounds                edges,
                                      Bounds                faces,
                                      CBufferView<AABB>     grown_edges,
                                      CBufferView<AABB>     grown_faces,
                                      CBufferView<Vector3i> owned_edges,
                                      CBufferView<IndexT>   face_queries,
                                      BufferView<AABB>      boxes,
                                      BufferView<IndexT>    cids,
                                      BufferView<IndexT>    cover_tags)
    {
        const SizeT i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= boxes.size())
            return;
        const auto owned = owned_edges(i);
        AABB       box;
        IndexT     cid   = -1;
        bool       first = true;
        if(i < grown_faces.size())
        {
            box   = grown_faces(i);
            cid   = faces.cids(i);
            first = false;
        }
        for(int slot = 0; slot < 3; ++slot)
            if(owned[slot] >= 0)
            {
                const IndexT e = owned[slot];
                if(first)
                {
                    box   = grown_edges(e);
                    cid   = edges.cids(e);
                    first = false;
                }
                else
                {
                    box.extend(grown_edges(e));
                    if(cid != edges.cids(e))
                        cid = -1;
                }
            }
        boxes(i) = box;
        cids(i)  = cid;
        // BID is an aggregate cover tag in this tree. It is never used as a body id.
        cover_tags(i) = i < grown_faces.size() && face_queries(i) ? 0 : 1;
    }
    __global__ void gather_queries(CBufferView<AABB>   boxes,
                                   CBufferView<IndexT> cids,
                                   CBufferView<IndexT> flags,
                                   CBufferView<IndexT> offsets,
                                   BufferView<IndexT>  ids,
                                   BufferView<AABB>    out_boxes,
                                   BufferView<IndexT>  out_cids)
    {
        const SizeT i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i < boxes.size() && flags(i))
        {
            const IndexT dst = offsets(i);
            ids(dst)         = i;
            out_boxes(dst)   = boxes(i);
            out_cids(dst)    = cids(i);
        }
    }
    __global__ void publish_sizes(CBufferView<IndexT> a, CBufferView<IndexT> b, IndexT* counts)
    {
        counts[0] = a(a.size() - 1);
        counts[1] = b(b.size() - 1);
    }
    struct ProxyNode
    {
        NodePredicate   node;
        bool            reverse = false, symmetric = false;
        __device__ bool operator()(InfoStacklessBVH::NodePredInfo info) const
        {
            if(reverse && info.node_bid == 1)
                return false;
            // Only homogeneous primitive contact metadata can cull a mixed EF
            // subtree. EE may reach its pair in reverse order, requiring symmetry.
            info.query_bid = info.node_bid = -1;
            if(!symmetric || info.query_cid < 0 || info.node_cid < 0
               || info.query_cid >= node.cmts.dim().x
               || info.node_cid >= node.cmts.dim().y)
                return true;
            return node(info);
        }
    };
    struct ProxyLeaf
    {
        CBufferView<IndexT> face_queries;
        bool                reverse;
        __device__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
        {
            return !reverse || (info.j < face_queries.size() && face_queries(info.j));
        }
    };
    __global__ void publish_ef_count(CVarView<int> count, IndexT* out)
    {
        *out = *count;
    }
    __global__ void count_candidates(CBufferView<Vector2i>  ef,
                                     CBufferView<IndexT>    query_ids,
                                     CBufferView<Vector2i>  owned_vertices,
                                     CBufferView<Vector3i>  owned_edges,
                                     Bounds                 points,
                                     Bounds                 edges,
                                     Bounds                 faces,
                                     CBufferView<AABB>      fat_points,
                                     CBufferView<AABB>      fat_edges,
                                     CBufferView<AABB>      fat_faces,
                                     CBufferView<IndexT>    point_queries,
                                     CBufferView<IndexT>    edge_queries,
                                     PointTrianglePredicate pt,
                                     EdgeEdgePredicate      ee,
                                     NodePredicate          node,
                                     bool                   reverse,
                                     BufferView<uint8_t>    masks,
                                     BufferView<U64>        pt_counts,
                                     BufferView<U64>        ee_counts)
    {
        const SizeT i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i == 0)
        {
            pt_counts(ef.size()) = 0;
            ee_counts(ef.size()) = 0;
        }
        if(i >= ef.size())
            return;
        const auto   pair = ef(i);
        const IndexT e = query_ids(pair[0]), f = pair[1];
        uint8_t      mask = 0;
        U64          np = 0, ne = 0;
        for(int slot = 0; slot < 2; ++slot)
        {
            const IndexT v = owned_vertices(e)[slot];
            if(v >= 0 && f < faces.aabbs.size()
               && (reverse ? !point_queries(v) : point_queries(v))
               && fat_points(v).intersects(fat_faces(f))
               && node({v, points.bids(v), points.cids(v), faces.bids(f), faces.cids(f)})
               && pt({v, f, points.bids(v), points.cids(v), faces.bids(f), faces.cids(f)}))
            {
                mask |= uint8_t{1} << slot;
                ++np;
            }
        }
        if(!reverse && e < edges.aabbs.size() && edge_queries(e))
            for(int slot = 0; slot < 3; ++slot)
            {
                const IndexT b = owned_edges(f)[slot];
                if(b < 0 || b == e || (edge_queries(b) && b < e))
                    continue;
                const IndexT lo = min(b, e), hi = max(b, e);
                if(fat_edges(lo).intersects(fat_edges(hi))
                   && node({lo,
                            edges.bids(lo),
                            edges.cids(lo),
                            edges.bids(hi),
                            edges.cids(hi)})
                   && ee({lo,
                          hi,
                          edges.bids(lo),
                          edges.cids(lo),
                          edges.bids(hi),
                          edges.cids(hi)}))
                {
                    mask |= uint8_t{1} << (slot + 2);
                    ++ne;
                }
            }
        masks(i)     = mask;
        pt_counts(i) = np;
        ee_counts(i) = ne;
    }
    __global__ void accumulate_counts(CBufferView<U64> pts,
                                      CBufferView<U64> ees,
                                      U64*             totals,
                                      BufferView<U64>  base)
    {
        base(0) = totals[0];
        base(1) = totals[1];
        totals[0] += pts(pts.size() - 1);
        totals[1] += ees(ees.size() - 1);
    }
    __global__ void fill_candidates(CBufferView<Vector2i> ef,
                                    CBufferView<IndexT>   query_ids,
                                    CBufferView<Vector2i> owned_vertices,
                                    CBufferView<Vector3i> owned_edges,
                                    CBufferView<uint8_t>  masks,
                                    CBufferView<U64>      po,
                                    CBufferView<U64>      eo,
                                    CBufferView<U64>      base,
                                    BufferView<Vector2i>  pts,
                                    BufferView<Vector2i>  ees)
    {
        const SizeT i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= ef.size())
            return;
        const auto   pair = ef(i);
        const IndexT e = query_ids(pair[0]), f = pair[1];
        auto         p = base(0) + po(i), q = base(1) + eo(i);
        for(int slot = 0; slot < 2; ++slot)
            if(masks(i) & (uint8_t{1} << slot))
                pts(p++) = Vector2i{owned_vertices(e)[slot], f};
        for(int slot = 0; slot < 3; ++slot)
            if(masks(i) & (uint8_t{1} << (slot + 2)))
            {
                const IndexT b = owned_edges(f)[slot];
                ees(q++)       = Vector2i{min(b, e), max(b, e)};
            }
    }
}  // namespace

struct SingleEFBVHQuery::Impl
{
    enum class Eligibility
    {
        Unchecked,
        Supported,
        Unsupported
    };
    Eligibility m_eligibility = Eligibility::Unchecked;
    SizeT m_vertex_count = 0, m_edge_count = 0, m_triangle_count = 0, m_position_count = 0;
    DeviceBuffer<Vector2i>    m_owned_vertices;
    DeviceBuffer<Vector3i>    m_owned_edges;
    DeviceBuffer<IndexT>      m_previous_vertices;
    DeviceBuffer<Vector2i>    m_previous_edges;
    DeviceBuffer<Vector3i>    m_previous_triangles;
    std::vector<IndexT>       m_h_vertices;
    std::vector<Vector2i>     m_h_edges;
    std::vector<Vector3i>     m_h_faces;
    DeviceBuffer<IndexT>      m_metadata;
    std::unique_ptr<Metadata> m_previous_metadata;
    DeviceBuffer<IndexT>      m_point_queries, m_edge_queries, m_face_queries;
    DeviceBuffer<IndexT>      m_groups[3];
    DeviceBuffer<AABB>        m_grown[3], m_query_boxes, m_leaf_boxes;
    DeviceBuffer<Float>       m_shrink[3];
    DeviceBuffer<IndexT>      m_query_cids, m_leaf_cids, m_leaf_tags;
    bool m_node_symmetric = false, m_reverse_needed = false;
    bool m_cache_valid = false, m_tree_ready = false, m_rebuild_topology = true;
    bool m_skip_pt = false, m_skip_ee = false;
    bool m_frame_cover_left = false, m_frame_other_left = false;
    bool m_cover_thick = false, m_other_uniform = false;
    SizeT             m_frames = 0, m_cached_chunk_size = 0;
    InfoStacklessBVH* m_tree = nullptr;
    Statistics        m_stats;
    struct Chain
    {
        SideStream           side;
        DeviceBuffer<IndexT> ids, bids, cids, flags, offsets;
        DeviceBuffer<AABB>   boxes;
        std::vector<std::unique_ptr<InfoStacklessBVH::QueryBuffer>> chunks;
        SizeT                                                       count = 0;
    } m_chains[2];
    DeviceBuffer<Vector2i> m_raw_pts, m_raw_ees;
    DeviceBuffer<uint8_t>  m_masks;
    DeviceBuffer<U64> m_pt_counts, m_ee_counts, m_pt_offsets, m_ee_offsets, m_chunk_base;
    DeviceBuffer<IndexT> m_judge_flags, m_tile_counts, m_tile_offsets,
        m_dummy_counts, m_ee_queue_counts;
    DeviceBuffer<int4>     m_ee_queue;
    ReadbackBuffer<IndexT> m_control, m_counts, m_query_sizes, m_ef_counts;
    ReadbackBuffer<U64>    m_raw_counts;
    cudaEvent_t            m_ready    = nullptr;
    int                    m_sm_count = 0;

    ~Impl()
    {
        if(m_ready)
            cudaEventDestroy(m_ready);
    }
    void init()
    {
        if(m_ready)
            return;
        CUDA_TOOL_CHECK(cudaEventCreateWithFlags(&m_ready, cudaEventDisableTiming));
        int device = 0;
        CUDA_TOOL_CHECK(cudaGetDevice(&device));
        CUDA_TOOL_CHECK(cudaDeviceGetAttribute(&m_sm_count, cudaDevAttrMultiProcessorCount, device));
        m_control.resize(6);
        m_counts.resize(8);
        m_query_sizes.resize(2);
        m_ef_counts.resize(2);
        m_raw_counts.resize(2);
        m_dummy_counts.resize(5);
        m_ee_queue_counts.resize(3);
        m_chunk_base.resize(2);
    }
    void wait()
    {
        CUDA_TOOL_CHECK(cudaEventRecord(m_ready, default_stream()));
        CUDA_TOOL_CHECK(cudaEventSynchronize(m_ready));
    }
    bool prepare_topology(const PointTrianglePredicate&, const EdgeEdgePredicate&, bool unchanged);
    void update_cover(const Metadata&);
    void prepare_proxies(const PointTrianglePredicate&, const EdgeEdgePredicate&, Bounds, Bounds, Bounds, Float);
    void refresh(InfoStacklessBVH&,
                 const PointTrianglePredicate&,
                 const EdgeEdgePredicate&,
                 const NodePredicate&,
                 Bounds,
                 Bounds,
                 Bounds,
                 SizeT,
                 bool);
    void judge(InfoStacklessBVH&,
               const PointTrianglePredicate&,
               const EdgeEdgePredicate&,
               Bounds,
               Bounds,
               Bounds,
               InfoStacklessBVH::QueryBuffer&,
               InfoStacklessBVH::QueryBuffer&,
               bool                         gated,
               const std::function<void()>& hook);
};
bool SingleEFBVHQuery::Impl::prepare_topology(const bvh_simplex_detail::PointTrianglePredicate& pt,
                                              const bvh_simplex_detail::EdgeEdgePredicate& ee,
                                              bool unchanged)
{
    if(unchanged && m_eligibility == Eligibility::Supported)
        return true;
    m_cache_valid = false;

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
            const auto v = surface_indices.at(edges[e][slot]);
            if(representatives[v] < 0)
            {
                representatives[v]      = static_cast<IndexT>(e);
                owned_vertices[e][slot] = v;
            }
        }
    }

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

    upload(m_owned_vertices, owned_vertices);
    upload(m_owned_edges, owned_edges);
    m_previous_vertices.resize_discard(pt.Vs.size());
    m_previous_edges.resize_discard(ee.Es.size());
    m_previous_triangles.resize_discard(pt.Fs.size());
    m_previous_vertices.view().copy_from(pt.Vs);
    m_previous_edges.view().copy_from(ee.Es);
    m_previous_triangles.view().copy_from(pt.Fs);
    m_h_vertices  = std::move(vertices);
    m_h_edges     = std::move(edges);
    m_h_faces     = std::move(faces);
    m_eligibility = Eligibility::Supported;
    return true;
}

void SingleEFBVHQuery::Impl::update_cover(const Metadata& metadata)
{
    m_metadata.resize_discard(metadata.total);
    if(metadata.work)
        save_metadata<<<(metadata.work + Threads - 1) / Threads, Threads>>>(
            metadata, m_metadata.view());
    std::vector<IndexT> values(metadata.total);
    m_metadata.copy_to(values.data());
    m_previous_metadata = std::make_unique<Metadata>(metadata);
    auto readable       = [&](int matrix)
    {
        const auto dim = metadata.matrices[matrix].dim();
        if(dim.x <= 0 || dim.x != dim.y)
            return false;
        const auto* mask = values.data() + metadata.offsets[Metadata::Arrays + matrix];
        for(int a = 0; a < dim.x; ++a)
            for(int b = 0; b < a; ++b)
                if((mask[a * dim.x + b] != 0) != (mask[b * dim.x + a] != 0))
                    return false;
        return true;
    };
    auto choose = [&](int matrix, int array, bool edges)
    {
        const SizeT          n = metadata.matrices[matrix].dim().x;
        uipc::vector<IndexT> mask;
        if(n)
            mask.assign(values.begin() + metadata.offsets[Metadata::Arrays + matrix],
                        values.begin() + metadata.offsets[Metadata::Arrays + matrix] + n * n);
        uipc::vector<IndexT> weights(n + 1, 0);
        const auto*          ids = values.data() + metadata.offsets[array];
        auto                 add = [&](IndexT vertex)
        {
            const IndexT cid = ids[vertex];
            auto& weight = weights[cid >= 0 && static_cast<SizeT>(cid) < n ? cid : n];
            if(weight < std::numeric_limits<IndexT>::max())
                ++weight;
        };
        if(edges)
            for(const auto& e : m_h_edges)
            {
                add(e[0]);
                add(e[1]);
            }
        else
            for(const auto v : m_h_vertices)
                add(v);
        return contact_vertex_cover(mask, n, weights.data());
    };
    const bool readable_pt = readable(Metadata::PTMask),
               readable_ee = readable(Metadata::EEMask);
    const auto point_cover =
        readable_pt ? choose(Metadata::PTMask, Metadata::PTContacts, false) :
                      uipc::vector<IndexT>{};
    const auto edge_cover = readable_ee ?
                                choose(Metadata::EEMask, Metadata::EEContacts, true) :
                                uipc::vector<IndexT>{};
    auto       member     = [](const auto& cover, IndexT cid)
    {
        return cid < 0 || static_cast<SizeT>(cid) >= cover.size() || cover[cid] != 0;
    };
    const auto* pt_ids = values.data() + metadata.offsets[Metadata::PTContacts];
    const auto* ee_ids = values.data() + metadata.offsets[Metadata::EEContacts];
    std::vector<IndexT> points(m_vertex_count, 1), edges(m_edge_count, 1),
        faces(m_triangle_count, 1);
    SizeT np = 0, ne = 0;
    for(SizeT i = 0; i < points.size(); ++i)
    {
        points[i] = member(point_cover, pt_ids[m_h_vertices[i]]);
        np += points[i];
    }
    for(SizeT i = 0; i < edges.size(); ++i)
    {
        const auto e = m_h_edges[i];
        edges[i] = member(edge_cover, ee_ids[e[0]]) && member(edge_cover, ee_ids[e[1]]);
        ne += edges[i];
    }
    m_stats.pt_cover = !m_skip_pt && readable_pt && np * 5 < points.size() * 3;
    m_stats.ee_cover = !m_skip_ee && readable_ee && ne * 5 < edges.size() * 3;
    if(!m_stats.pt_cover)
        std::fill(points.begin(), points.end(), 1);
    if(!m_stats.ee_cover)
        std::fill(edges.begin(), edges.end(), 1);
    for(SizeT i = 0; i < faces.size(); ++i)
    {
        const auto f = m_h_faces[i];
        faces[i]     = !m_stats.pt_cover
                   || (member(point_cover, pt_ids[f[0]])
                       && member(point_cover, pt_ids[f[1]])
                       && member(point_cover, pt_ids[f[2]]));
    }
    // If two primitives both contain an endpoint outside the vertex cover,
    // their cross-endpoint mask is disabled. Requiring all endpoints inside
    // therefore also covers mixed-CID edges and faces without a fallback.
    upload(m_point_queries, points);
    upload(m_edge_queries, edges);
    upload(m_face_queries, faces);
    m_reverse_needed =
        m_stats.pt_cover
        && std::any_of(faces.begin(), faces.end(), [](auto f) { return f != 0; });
    auto groups = [&](const std::vector<IndexT>& selected, bool cover, int family)
    {
        std::vector<IndexT> group(selected.size(), 0);
        if(cover)
            for(SizeT i = 0; i < group.size(); ++i)
                group[i] = selected[i] ? 1 : 2;
        upload(m_groups[family], group);
    };
    groups(points, m_stats.pt_cover, 0);
    groups(edges, m_stats.ee_cover, 1);
    groups(faces, m_stats.pt_cover, 2);
    m_node_symmetric = readable(Metadata::NodeMask);
}

void SingleEFBVHQuery::Impl::prepare_proxies(const PointTrianglePredicate& pt,
                                             const EdgeEdgePredicate&      ee,
                                             Bounds points,
                                             Bounds edges,
                                             Bounds faces,
                                             Float  skin_scale)
{
    m_stats.cover_skin     = m_cover_thick ? 2.0 : 1.0;
    m_stats.other_skin     = m_other_uniform ? 1.0 : 0.01;
    const Bounds inputs[3] = {points, edges, faces};
    for(int family = 0; family < 3; ++family)
    {
        const SizeT n = inputs[family].aabbs.size();
        m_grown[family].resize_discard(n);
        m_shrink[family].resize_discard(n);
        if(n)
            grow_primitives<<<(n + Threads - 1) / Threads, Threads>>>(
                inputs[family].aabbs,
                m_groups[family].cview(),
                pt.Vs,
                ee.Es,
                pt.Fs,
                pt.d_hats,
                family,
                skin_scale,
                skin_scale * m_stats.cover_skin,
                skin_scale * m_stats.other_skin,
                m_grown[family].view(),
                m_shrink[family].view());
    }
    const SizeT queries = m_owned_vertices.size(), leaves = m_owned_edges.size();
    m_query_boxes.resize_discard(queries);
    m_query_cids.resize_discard(queries);
    m_leaf_boxes.resize_discard(leaves);
    m_leaf_cids.resize_discard(leaves);
    m_leaf_tags.resize_discard(leaves);
    for(auto& chain : m_chains)
    {
        chain.ids.resize_discard(queries);
        chain.boxes.resize_discard(queries);
        chain.bids.resize(queries, IndexT{-1});
        chain.cids.resize_discard(queries);
        chain.flags.resize_discard(queries + 1);
        chain.offsets.resize_discard(queries + 1);
    }
    make_query_proxies<<<(queries + Threads - 1) / Threads, Threads>>>(
        points,
        edges,
        m_grown[0].cview(),
        m_grown[1].cview(),
        m_owned_vertices.cview(),
        m_point_queries.cview(),
        m_edge_queries.cview(),
        m_query_boxes.view(),
        m_query_cids.view(),
        m_chains[0].flags.view(),
        m_chains[1].flags.view(),
        m_reverse_needed);
    make_leaf_proxies<<<(leaves + Threads - 1) / Threads, Threads>>>(
        edges,
        faces,
        m_grown[1].cview(),
        m_grown[2].cview(),
        m_owned_edges.cview(),
        m_face_queries.cview(),
        m_leaf_boxes.view(),
        m_leaf_cids.view(),
        m_leaf_tags.view());
    for(auto& chain : m_chains)
    {
        DeviceScan().ExclusiveSum(chain.flags.data(),
                                  chain.offsets.data(),
                                  static_cast<int>(queries + 1));
        gather_queries<<<(queries + Threads - 1) / Threads, Threads>>>(
            m_query_boxes.cview(),
            m_query_cids.cview(),
            chain.flags.cview(),
            chain.offsets.cview(),
            chain.ids.view(),
            chain.boxes.view(),
            chain.cids.view());
    }
    publish_sizes<<<1, 1>>>(m_chains[0].offsets.cview(),
                            m_chains[1].offsets.cview(),
                            m_query_sizes.device_data());
}

void SingleEFBVHQuery::Impl::refresh(InfoStacklessBVH&             tree,
                                     const PointTrianglePredicate& pt_input,
                                     const EdgeEdgePredicate&      ee_input,
                                     const NodePredicate&          node,
                                     Bounds                        points,
                                     Bounds                        edges,
                                     Bounds                        faces,
                                     SizeT                         chunk_size,
                                     bool                          reuse_order)
{
    if(m_rebuild_topology || m_tree != &tree || !m_tree_ready
       || !tree.refit(m_leaf_boxes.cview(), m_leaf_tags.cview(), m_leaf_cids.cview()))
    {
        tree.build(m_leaf_boxes.cview(), m_leaf_tags.cview(), m_leaf_cids.cview());
        ++m_stats.topology_builds;
    }
    m_tree             = &tree;
    m_tree_ready       = true;
    m_rebuild_topology = false;
    m_query_sizes.enqueue_copy();
    wait();
    SizeT rounds = 0;
    for(int family = 0; family < 2; ++family)
    {
        auto& chain       = m_chains[family];
        chain.count       = m_query_sizes[family];
        const SizeT count = (chain.count + chunk_size - 1) / chunk_size;
        while(chain.chunks.size() < count)
            chain.chunks.push_back(std::make_unique<InfoStacklessBVH::QueryBuffer>());
        chain.chunks.resize(count);
        rounds = std::max(rounds, count);
    }
    m_stats.primary_queries = m_chains[0].count;
    m_stats.reverse_queries = m_chains[1].count;
    store_value(m_raw_counts.device_data(), U64{0});
    store_value(m_raw_counts.device_data() + 1, U64{0});
    std::array<U64, 2> totals{};
    auto               pt = pt_input;
    auto               ee = ee_input;
    pt.geometry           = false;
    ee.geometry           = false;
    for(SizeT round = 0; round < rounds; ++round)
    {
        bool pending[2] = {round < m_chains[0].chunks.size(),
                           round < m_chains[1].chunks.size()};
        bool first[2]   = {true, true};
        do
        {
            bool forked[2] = {false, false};
            try
            {
                for(int family = 0; family < 2; ++family)
                    if(pending[family])
                    {
                        auto&       chain  = m_chains[family];
                        auto&       ef     = *chain.chunks[round];
                        const SizeT offset = round * chunk_size,
                                    count = std::min(chunk_size, chain.count - offset);
                        const auto stream = chain.side.fork();
                        forked[family]    = true;
                        tree.launch_query(chain.boxes.cview(offset, count),
                                          chain.bids.cview(offset, count),
                                          chain.cids.cview(offset, count),
                                          {},
                                          ProxyNode{node, family == 1, m_node_symmetric},
                                          ProxyLeaf{m_face_queries.cview(), family == 1},
                                          ef,
                                          first[family],
                                          reuse_order,
                                          stream);
                        publish_ef_count<<<1, 1, 0, stream>>>(
                            ef.m_cpNum.cview(), m_ef_counts.device_data() + family);
                    }
            }
            catch(...)
            {
                for(int family = 0; family < 2; ++family)
                    if(forked[family])
                        m_chains[family].side.join();
                CUDA_TOOL_CHECK(cudaStreamSynchronize(default_stream()));
                throw;
            }
            for(int family = 0; family < 2; ++family)
                if(forked[family])
                    m_chains[family].side.join();
            m_ef_counts.enqueue_copy();
            wait();
            for(int family = 0; family < 2; ++family)
                if(pending[family])
                {
                    const int count = m_ef_counts[family];
                    if(count < 0)
                        throw std::overflow_error("Single EF proxy counter exceeds 32-bit indexing");
                    pending[family] =
                        tree.prepare_query_result(*m_chains[family].chunks[round], count);
                    first[family] = false;
                }
        } while(pending[0] || pending[1]);
        for(int family = 0; family < 2; ++family)
        {
            auto& chain = m_chains[family];
            if(round >= chain.chunks.size())
                continue;
            const auto& cached = *chain.chunks[round];
            const SizeT n      = cached.size();
            if(!n)
                continue;
            const auto ids =
                chain.ids.cview(round * chunk_size,
                                std::min(chunk_size, chain.count - round * chunk_size));
            m_masks.resize_discard(n);
            m_pt_counts.resize_discard(n + 1);
            m_ee_counts.resize_discard(n + 1);
            m_pt_offsets.resize_discard(n + 1);
            m_ee_offsets.resize_discard(n + 1);
            count_candidates<<<(n + Threads - 1) / Threads, Threads>>>(
                cached.view(),
                ids,
                m_owned_vertices.cview(),
                m_owned_edges.cview(),
                points,
                edges,
                faces,
                m_grown[0].cview(),
                m_grown[1].cview(),
                m_grown[2].cview(),
                m_point_queries.cview(),
                m_edge_queries.cview(),
                pt,
                ee,
                node,
                family == 1,
                m_masks.view(),
                m_pt_counts.view(),
                m_ee_counts.view());
            DeviceScan().ExclusiveSum(m_pt_counts.data(),
                                      m_pt_offsets.data(),
                                      static_cast<int>(n + 1));
            DeviceScan().ExclusiveSum(m_ee_counts.data(),
                                      m_ee_offsets.data(),
                                      static_cast<int>(n + 1));
            accumulate_counts<<<1, 1>>>(m_pt_offsets.cview(),
                                        m_ee_offsets.cview(),
                                        m_raw_counts.device_data(),
                                        m_chunk_base.view());
            m_raw_counts.enqueue_copy();
            wait();
            totals = {m_raw_counts[0], m_raw_counts[1]};
            if(totals[0] > PairLimit || totals[1] > PairLimit
               || totals[0] + totals[1] > PairLimit)
                throw std::overflow_error("Single EF cached candidates exceed 32-bit downstream indexing");
            reserve_pairs(m_raw_pts, totals[0], tree.config().reserve_ratio, true);
            reserve_pairs(m_raw_ees, totals[1], tree.config().reserve_ratio, true);
            fill_candidates<<<(n + Threads - 1) / Threads, Threads>>>(
                cached.view(),
                ids,
                m_owned_vertices.cview(),
                m_owned_edges.cview(),
                m_masks.cview(),
                m_pt_offsets.cview(),
                m_ee_offsets.cview(),
                m_chunk_base.cview(),
                m_raw_pts.view(),
                m_raw_ees.view());
        }
    }
    m_stats.stored_pts  = totals[0];
    m_stats.stored_ees  = totals[1];
    m_cached_chunk_size = chunk_size;
    m_cache_valid       = true;
    ++m_stats.refreshes;
}

void SingleEFBVHQuery::Impl::judge(InfoStacklessBVH&              tree,
                                   const PointTrianglePredicate&  pt_input,
                                   const EdgeEdgePredicate&       ee_input,
                                   Bounds                         points,
                                   Bounds                         edges,
                                   Bounds                         faces,
                                   InfoStacklessBVH::QueryBuffer& pts,
                                   InfoStacklessBVH::QueryBuffer& ees,
                                   bool                           gated,
                                   const std::function<void()>&   hook)
{
    using namespace bvh_pair_judge;
    const SizeT np = m_stats.stored_pts, ne = m_stats.stored_ees;
    reserve_pairs(pts.m_pairs, np, tree.config().reserve_ratio, false);
    reserve_pairs(ees.m_pairs, ne, tree.config().reserve_ratio, false);
    const int et = (ne + JudgeTile - 1) / JudgeTile, pt = (np + JudgeTile - 1) / JudgeTile;
    const int tiles = et + pt;
    m_judge_flags.resize_discard(ne + np);
    m_tile_counts.resize_discard(tiles);
    m_tile_offsets.resize_discard(tiles);
    EEQueue queue{nullptr, nullptr, m_ee_queue_counts.data(), static_cast<int>(ne)};
    if(ne && ee_input.alpha != 0.0 && ee_input.f32_positions.size())
    {
        m_ee_queue.resize_discard(2 * ne);
        queue.second = m_ee_queue.data();
        queue.pairs  = m_ee_queue.data() + ne;
    }
    auto ee_pred               = ee_input;
    auto pt_pred               = pt_input;
    ee_pred.skip_position_free = true;
    pt_pred.skip_position_free = true;
    const IndexT* invalid      = gated ? m_control.device_data() : nullptr;
    // Both chains follow prefix + validation, and join before the shared scan.
    const auto ee_stream = m_chains[0].side.fork();
    const auto pt_stream = m_chains[1].side.fork();
    bvh_pair_judge::judge_ee(m_raw_ees.cview(0, ne),
                             edges.aabbs,
                             edges.aabbs,
                             EEPredicate{{}, {}, ee_pred},
                             m_judge_flags.view(0, ne),
                             m_tile_counts.view(0, et),
                             queue,
                             m_sm_count,
                             ee_stream,
                             invalid);
    bvh_pair_judge::judge_stored(m_raw_pts.cview(0, np),
                                 points.aabbs,
                                 faces.aabbs,
                                 pt_pred,
                                 m_judge_flags.view(ne, np),
                                 m_tile_counts.view(et, pt),
                                 static_cast<int>(np),
                                 pt_stream,
                                 invalid);
    m_chains[0].side.join();
    m_chains[1].side.join();
    PairLists lists{};
    for(int i = 0; i < JudgedLists; ++i)
        lists.list[i].count = m_dummy_counts.data() + i;
    lists.list[2]            = {m_raw_ees.data(),
                                m_judge_flags.data(),
                                ees.m_pairs.data(),
                                ees.m_cpNum.data(),
                                static_cast<int>(ne),
                                0};
    lists.list[3]            = {m_raw_pts.data(),
                                m_judge_flags.data() + ne,
                                pts.m_pairs.data(),
                                pts.m_cpNum.data(),
                                static_cast<int>(np),
                                et};
    lists.list[4].first_tile = tiles;
    compact(lists,
            m_tile_counts.cview(),
            m_tile_offsets.view(),
            CBufferView<IndexT>{m_control.device_data(), 3},
            BufferView<IndexT>{m_counts.device_data(), 8},
            m_ee_queue_counts.data(),
            tiles,
            m_sm_count);
    m_control.enqueue_copy();
    m_counts.enqueue_copy();
    CUDA_TOOL_CHECK(cudaEventRecord(m_ready, default_stream()));
    if(hook)
        hook();
    CUDA_TOOL_CHECK(cudaEventSynchronize(m_ready));
    if(!gated || !m_control[0])
    {
        pts.m_size = m_counts[3];
        ees.m_size = m_counts[2];
    }
}

SingleEFBVHQuery::SingleEFBVHQuery()
    : m_impl(std::make_unique<Impl>())
{
}
SingleEFBVHQuery::~SingleEFBVHQuery() = default;
SizeT SingleEFBVHQuery::refresh_count() const noexcept
{
    return m_impl->m_stats.refreshes;
}
SizeT SingleEFBVHQuery::reuse_count() const noexcept
{
    return m_impl->m_stats.reuses;
}
SizeT SingleEFBVHQuery::isolated_point_count() const noexcept
{
    return m_impl->m_eligibility == Impl::Eligibility::Supported ?
               m_impl->m_owned_vertices.size() - m_impl->m_edge_count :
               0;
}
SizeT SingleEFBVHQuery::standalone_edge_count() const noexcept
{
    return m_impl->m_eligibility == Impl::Eligibility::Supported ?
               m_impl->m_owned_edges.size() - m_impl->m_triangle_count :
               0;
}
SingleEFBVHQuery::Statistics SingleEFBVHQuery::statistics() const noexcept
{
    return m_impl->m_stats;
}

bool SingleEFBVHQuery::detect(InfoStacklessBVH&              tree,
                              const PointTrianglePredicate&  pt,
                              const EdgeEdgePredicate&       ee,
                              const NodePredicate&           node,
                              PrimitiveBounds                points,
                              PrimitiveBounds                edges,
                              PrimitiveBounds                faces,
                              InfoStacklessBVH::QueryBuffer& pts,
                              InfoStacklessBVH::QueryBuffer& ees,
                              bool                           new_frame,
                              const std::function<void()>&   hook)
{
    UIPC_ASSERT(pt.alpha == ee.alpha && pt.alpha >= 0.0,
                "Single EF PT and EE queries must share a nonnegative alpha");
    UIPC_ASSERT(points.aabbs.size() == pt.Vs.size()
                    && edges.aabbs.size() == ee.Es.size()
                    && faces.aabbs.size() == pt.Fs.size(),
                "Single EF primitive bounds do not match topology");
    auto& s = *m_impl;
    s.init();
    if(new_frame)
    {
        ++s.m_frames;
        if(s.m_frames % TopologyPeriod == 0)
            s.m_rebuild_topology = true;
        s.m_cover_thick      = s.m_frame_cover_left;
        s.m_other_uniform    = s.m_frame_other_left;
        s.m_frame_cover_left = s.m_frame_other_left = false;
    }
    const bool topology_shape =
        s.m_eligibility == Impl::Eligibility::Supported
        && s.m_vertex_count == pt.Vs.size() && s.m_edge_count == ee.Es.size()
        && s.m_triangle_count == pt.Fs.size() && s.m_position_count == pt.Ps.size();
    const Metadata metadata(pt, ee, node, points, edges, faces);
    const bool     metadata_shape =
        s.m_previous_metadata && metadata.same_shape(*s.m_previous_metadata);
    const bool policy_change = s.m_skip_pt != pt.skip_position_free
                               || s.m_skip_ee != ee.skip_position_free;
    const SizeT prospective_leaves = topology_shape ? s.m_owned_edges.size() : 0;
    const SizeT chunk_size    = prospective_leaves ?
                                    std::min({s.m_owned_vertices.size(),
                                              std::max(SizeT{1}, max_chunk_edges),
                                              PairLimit / prospective_leaves}) :
                                    0;
    const bool  known_refresh = force_fresh || new_frame || !s.m_cache_valid
                               || !topology_shape || !metadata_shape || policy_change
                               || s.m_cached_chunk_size != chunk_size
                               || s.m_tree != &tree;
    const IndexT initial[6] = {known_refresh ? 1 : 0,
                               topology_shape ? 0 : 1,
                               metadata_shape && !policy_change ? 0 : 1,
                               0,
                               0,
                               0};
    for(int i = 0; i < 6; ++i)
        store_value(s.m_control.device_data() + i, initial[i]);
    Validation validation{
        pt.Vs,
        s.m_previous_vertices.cview(),
        ee.Es,
        s.m_previous_edges.cview(),
        pt.Fs,
        s.m_previous_triangles.cview(),
        metadata,
        s.m_metadata.cview(),
        {points.aabbs, edges.aabbs, faces.aabbs},
        {s.m_grown[0].cview(), s.m_grown[1].cview(), s.m_grown[2].cview()},
        {s.m_groups[0].cview(), s.m_groups[1].cview(), s.m_groups[2].cview()},
        {s.m_shrink[0].cview(), s.m_shrink[1].cview(), s.m_shrink[2].cview()},
        topology_shape,
        metadata_shape,
        !known_refresh};
    const SizeT work =
        std::max({metadata.work, pt.Vs.size(), ee.Es.size(), pt.Fs.size()});
    if(work)
        validate_cache<<<(work + Threads - 1) / Threads, Threads>>>(
            validation, s.m_control.device_data(), work);
    if(!known_refresh)
    {
        s.judge(tree, pt, ee, points, edges, faces, pts, ees, true, hook);
        if(!s.m_control[0])
        {
            s.m_frame_cover_left |= s.m_control[3] != 0;
            s.m_frame_other_left |= s.m_control[4] != 0;
            ++s.m_stats.reuses;
            return true;
        }
    }
    else
    {
        s.m_control.enqueue_copy();
        s.wait();
    }
    s.m_cache_valid             = false;
    const bool topology_changed = s.m_control[1] != 0,
               metadata_changed = s.m_control[2] != 0;
    s.m_frame_cover_left |= s.m_control[3] != 0;
    s.m_frame_other_left |= s.m_control[4] != 0;
    s.m_cover_thick |= s.m_control[3] != 0;
    s.m_other_uniform |= s.m_control[4] != 0;
    if(!s.prepare_topology(pt, ee, !topology_changed))
    {
        // A speculative, gated judge can clear the device counts. Preserve
        // the last published result when the caller must use its fallback.
        pts.m_cpNum.copy_from(static_cast<int>(pts.m_size));
        ees.m_cpNum.copy_from(static_cast<int>(ees.m_size));
        return false;
    }

    if(topology_changed || metadata_changed)
    {
        s.m_skip_pt = pt.skip_position_free;
        s.m_skip_ee = ee.skip_position_free;
        s.update_cover(metadata);
    }
    if(topology_changed)
        s.m_rebuild_topology = true;
    s.prepare_proxies(pt, ee, points, edges, faces, std::max(Float{0}, cache_skin_scale));
    const SizeT actual_chunk = std::min({s.m_owned_vertices.size(),
                                         std::max(SizeT{1}, max_chunk_edges),
                                         PairLimit / s.m_owned_edges.size()});
    s.refresh(tree, pt, ee, node, points, edges, faces, actual_chunk, !topology_changed && !metadata_changed);
    for(int i = 0; i < 3; ++i)
        store_value(s.m_control.device_data() + i, IndexT{0});
    s.judge(tree, pt, ee, points, edges, faces, pts, ees, false, hook);
    return true;
}
}  // namespace uipc::backend::cuda
