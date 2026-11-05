#include <collision_detection/filters/simplex_bvh_query.h>
#include <collision_detection/filters/bvh_contact_cover.h>
#include <collision_detection/filters/bvh_pair_judge.h>
#include <collision_detection/filters/trajectory_prefix.h>
#include <cuda_tool/cub.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
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
    using Bounds                   = SimplexBVHQuery::PrimitiveBounds;
    using Codim                    = SimplexBVHQuery::CodimQueries;
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
            CodimBodies,
            CodimContacts,
            PPContacts,
            PEContacts,
            PPSubscenes,
            PESubscenes,
            PPSelf,
            PESelf,
            PPDimensions,
            PPVertices,
            PECodimVertices,
            PEEdges,
            Arrays
        };
        enum Matrix
        {
            PTMask,
            EEMask,
            NodeMask,
            PTSubmask,
            EESubmask,
            PPMask,
            PEMask,
            PPSubmask,
            PESubmask,
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
                 Bounds                        f,
                 const Codim*                  codim)
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
                     f.cids,
                     codim ? codim->points.bids : CBufferView<IndexT>{},
                     codim ? codim->points.cids : CBufferView<IndexT>{},
                     codim ? codim->pp.contact_element_ids : CBufferView<IndexT>{},
                     codim ? codim->pe.contact_element_ids : CBufferView<IndexT>{},
                     codim ? codim->pp.subscene_element_ids : CBufferView<IndexT>{},
                     codim ? codim->pe.subscene_element_ids : CBufferView<IndexT>{},
                     codim ? codim->pp.body_self_collision : CBufferView<IndexT>{},
                     codim ? codim->pe.body_self_collision : CBufferView<IndexT>{},
                     codim ? codim->pp.dimensions : CBufferView<IndexT>{},
                     codim ? codim->pp.Vs : CBufferView<IndexT>{},
                     codim ? codim->pe.codimVs : CBufferView<IndexT>{},
                     codim ? CBufferView<IndexT>{reinterpret_cast<const IndexT*>(
                                                     codim->pe.Es.data()),
                                                 2 * codim->pe.Es.size()} :
                             CBufferView<IndexT>{}}
            , matrices{pt.contact_mask_tabular,
                       ee.contact_mask_tabular,
                       node.cmts,
                       pt.subscene_mask_tabular,
                       ee.subscene_mask_tabular,
                       codim ? codim->pp.contact_mask_tabular : CDense2D<IndexT>{},
                       codim ? codim->pe.contact_mask_tabular : CDense2D<IndexT>{},
                       codim ? codim->pp.subscene_mask_tabular : CDense2D<IndexT>{},
                       codim ? codim->pe.subscene_mask_tabular : CDense2D<IndexT>{}}
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
        CBufferView<IndexT>   codim_vertices, previous_codim_vertices;
        Metadata              metadata;
        CBufferView<IndexT>   previous_metadata;
        CBufferView<AABB>     current[4], grown[4];
        CBufferView<IndexT>   groups[4];
        CBufferView<Float>    shrink[4];
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
               || (i < input.faces.size() && input.faces(i) != input.previous_faces(i))
               || (i < input.codim_vertices.size()
                   && input.codim_vertices(i) != input.previous_codim_vertices(i)))
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
            for(int family = 0; family < 4; ++family)
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

    struct FatBounds
    {
        Bounds                current[4];
        CBufferView<IndexT>   groups[4];
        CBufferView<IndexT>   vertices, codim_vertices;
        CBufferView<Vector2i> edges;
        CBufferView<Vector3i> faces;
        CBufferView<Float>    d_hats;
        Float                 uniform, cover, other;

        __device__ IndexT vertex(int family, SizeT i) const
        {
            return family == 0 ? vertices(i) :
                   family == 1 ? edges(i)[0] :
                   family == 2 ? faces(i)[0] :
                                 codim_vertices(i);
        }
        __device__ AABB box(int family, SizeT i) const
        {
            const int   group = groups[family](i);
            const Float skin  = std::max(Float{0},
                                        d_hats(vertex(family, i))
                                            * (group == 1 ? cover :
                                                group == 2 ? other :
                                                             uniform));
            AABB        box   = current[family].aabbs(i);
            for(int axis = 0; axis < 3; ++axis)
            {
                box.min()[axis] =
                    nextafterf(static_cast<float>(Float(box.min()[axis]) - skin), -INFINITY);
                box.max()[axis] =
                    nextafterf(static_cast<float>(Float(box.max()[axis]) + skin), INFINITY);
            }
            return box;
        }
        __device__ Float shrink(int family, SizeT i) const
        {
            const int group = groups[family](i);
            return std::max(Float{0},
                            d_hats(vertex(family, i))
                                * (group == 1 ? cover - uniform :
                                   group == 2 ? other - 0.01 * uniform :
                                                0.0));
        }
    };
    struct ProxyBounds
    {
        FatBounds             fat;
        CBufferView<Vector2i> owned_vertices;
        CBufferView<Vector3i> owned_edges;
        CBufferView<IndexT>   point_queries, edge_queries, face_queries;
        CBufferView<IndexT>   ids[2];
        BufferView<AABB>      grown[4], query_boxes[2], leaf_boxes;
        BufferView<Float>     shrink[4];
        BufferView<IndexT>    query_cids[2], leaf_cids, leaf_tags;
    };
    __device__ void extend_proxy(AABB& box, IndexT& cid, bool& first, const AABB& next, IndexT next_cid)
    {
        if(first)
        {
            box   = next;
            cid   = next_cid;
            first = false;
        }
        else
        {
            box.extend(next);
            if(cid != next_cid)
                cid = -1;
        }
    }
    // Every range reads immutable current bounds. Proxy ranges recompute cheap
    // growth instead of reading grown output written by another block here.
    __global__ void prepare_cache_bounds(ProxyBounds p)
    {
        int         range = 0, block = blockIdx.x;
        const SizeT sizes[7] = {p.grown[0].size(),
                                p.grown[1].size(),
                                p.grown[2].size(),
                                p.grown[3].size(),
                                p.ids[0].size(),
                                p.ids[1].size(),
                                p.leaf_boxes.size()};
        for(; range < 6; ++range)
        {
            const int blocks = (sizes[range] + blockDim.x - 1) / blockDim.x;
            if(block < blocks)
                break;
            block -= blocks;
        }
        const SizeT i = SizeT(block) * blockDim.x + threadIdx.x;
        if(i >= sizes[range])
            return;
        if(range < 4)
        {
            p.grown[range](i)  = p.fat.box(range, i);
            p.shrink[range](i) = p.fat.shrink(range, i);
            return;
        }
        AABB   box;
        IndexT cid   = -1;
        bool   first = true;
        if(range < 6)
        {
            const int    chain = range - 4;
            const IndexT e     = p.ids[chain](i);
            if(chain == 0 && e < p.fat.current[1].aabbs.size() && p.edge_queries(e))
                extend_proxy(box, cid, first, p.fat.box(1, e), p.fat.current[1].cids(e));
            for(int slot = 0; slot < 2; ++slot)
            {
                const IndexT v = p.owned_vertices(e)[slot];
                if(v >= 0 && (chain == 0 ? p.point_queries(v) != 0 : p.point_queries(v) == 0))
                    extend_proxy(
                        box, cid, first, p.fat.box(0, v), p.fat.current[0].cids(v));
            }
            p.query_boxes[chain](i) = box;
            p.query_cids[chain](i)  = cid;
        }
        else
        {
            if(i < p.fat.current[2].aabbs.size())
                extend_proxy(box, cid, first, p.fat.box(2, i), p.fat.current[2].cids(i));
            for(int slot = 0; slot < 3; ++slot)
            {
                const IndexT e = p.owned_edges(i)[slot];
                if(e >= 0)
                    extend_proxy(
                        box, cid, first, p.fat.box(1, e), p.fat.current[1].cids(e));
            }
            p.leaf_boxes(i) = box;
            p.leaf_cids(i)  = cid;
            // These are cover tags, never body IDs.
            p.leaf_tags(i) = i < p.face_queries.size() && p.face_queries(i) ? 0 : 1;
        }
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
    __global__ void gather_direct_bounds(CBufferView<IndexT> ids,
                                         CBufferView<AABB>   fat,
                                         Bounds              metadata,
                                         BufferView<AABB>    boxes,
                                         BufferView<IndexT>  bids,
                                         BufferView<IndexT>  cids)
    {
        const SizeT i = SizeT(blockIdx.x) * blockDim.x + threadIdx.x;
        if(i >= ids.size())
            return;
        const IndexT j = ids(i);
        boxes(i)       = fat(j);
        bids(i)        = metadata.bids(j);
        cids(i)        = metadata.cids(j);
    }
    __global__ void normalize_direct_pairs(CBufferView<Vector2i> in,
                                           CBufferView<IndexT>   query_ids,
                                           CBufferView<IndexT>   leaf_ids,
                                           BufferView<Vector2i>  out,
                                           bool                  ordered)
    {
        const SizeT i = SizeT(blockIdx.x) * blockDim.x + threadIdx.x;
        if(i >= in.size())
            return;
        const auto   p = in(i);
        const IndexT a = query_ids.size() ? query_ids(p[0]) : p[0];
        const IndexT b = leaf_ids.size() ? leaf_ids(p[1]) : p[1];
        out(i) = ordered ? Vector2i{min(a, b), max(a, b)} : Vector2i{a, b};
    }
    // Validation and geometry gates use device memory. Only the final six
    // flags are published to the mapped readback after the producers finish.
    __global__ void publish_control(CBufferView<IndexT> control, IndexT* out)
    {
        const int i = threadIdx.x;
        if(i < control.size())
            out[i] = control(i);
    }
    struct CandidateCounts
    {
        U64 pt = 0, ee = 0;
    };
    struct AddCandidateCounts
    {
        __host__ __device__ CandidateCounts operator()(CandidateCounts a, CandidateCounts b) const
        {
            return {a.pt + b.pt, a.ee + b.ee};
        }
    };
    __global__ void count_candidates(CBufferView<Vector2i>       ef,
                                     CVarView<int>               live,
                                     CBufferView<IndexT>         query_ids,
                                     CBufferView<Vector2i>       owned_vertices,
                                     CBufferView<Vector3i>       owned_edges,
                                     Bounds                      points,
                                     Bounds                      edges,
                                     Bounds                      faces,
                                     CBufferView<AABB>           fat_points,
                                     CBufferView<AABB>           fat_edges,
                                     CBufferView<AABB>           fat_faces,
                                     CBufferView<IndexT>         point_queries,
                                     CBufferView<IndexT>         edge_queries,
                                     PointTrianglePredicate      pt,
                                     EdgeEdgePredicate           ee,
                                     NodePredicate               node,
                                     bool                        reverse,
                                     BufferView<uint8_t>         masks,
                                     BufferView<CandidateCounts> counts)
    {
        const SizeT i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i == 0)
        {
            counts(ef.size()) = {};
        }
        if(i >= ef.size())
            return;
        // Never expand a truncated traversal, even if its partial output fits.
        const int n = *live;
        if(n < 0 || SizeT(n) > ef.size() || i >= SizeT(n))
        {
            masks(i)  = 0;
            counts(i) = {};
            return;
        }
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
        masks(i)  = mask;
        counts(i) = {np, ne};
    }
    // Chunks of a chain execute on one stream, so reservation needs no atomics.
    __global__ void reserve_chunk(CBufferView<CandidateCounts> offsets,
                                  VarView<CandidateCounts>     totals,
                                  BufferView<CandidateCounts>  bases,
                                  SizeT                        chunk)
    {
        bases(chunk) = *totals;
        *totals = AddCandidateCounts{}(*totals, offsets(offsets.size() - 1));
    }
    __global__ void publish_refresh_counts(CVarView<CandidateCounts> a,
                                           CVarView<CandidateCounts> b,
                                           U64*                      out)
    {
        out[0] = a->pt;
        out[1] = a->ee;
        out[2] = b->pt;
        out[3] = b->ee;
    }
    __global__ void fill_candidates(CBufferView<Vector2i>        ef,
                                    CVarView<int>                live,
                                    CBufferView<IndexT>          query_ids,
                                    CBufferView<Vector2i>        owned_vertices,
                                    CBufferView<Vector3i>        owned_edges,
                                    CBufferView<uint8_t>         masks,
                                    CBufferView<CandidateCounts> offsets,
                                    CBufferView<CandidateCounts> bases,
                                    SizeT                        chunk,
                                    BufferView<Vector2i>         pts,
                                    BufferView<Vector2i>         ees)
    {
        const SizeT i = blockIdx.x * blockDim.x + threadIdx.x;
        const int   n = *live;
        if(n < 0 || SizeT(n) > ef.size() || i >= SizeT(n))
            return;
        const auto   pair = ef(i);
        const IndexT e = query_ids(pair[0]), f = pair[1];
        const auto   base = bases(chunk), total = offsets(offsets.size() - 1);
        auto         p = base.pt + offsets(i).pt, q = base.ee + offsets(i).ee;
        const bool   pt_fits = base.pt + total.pt <= pts.size();
        const bool   ee_fits = base.ee + total.ee <= ees.size();
        for(int slot = 0; slot < 2; ++slot)
            if(pt_fits && (masks(i) & (uint8_t{1} << slot)))
                pts(p++) = Vector2i{owned_vertices(e)[slot], f};
        for(int slot = 0; slot < 3; ++slot)
            if(ee_fits && (masks(i) & (uint8_t{1} << (slot + 2))))
            {
                const IndexT b = owned_edges(f)[slot];
                ees(q++)       = Vector2i{min(b, e), max(b, e)};
            }
    }
}  // namespace

struct SimplexBVHQuery::Impl
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
    DeviceBuffer<IndexT>      m_previous_codim;
    SizeT                     m_codim_count = 0;
    Method                    m_method      = Method::SingleEF;
    bool                      m_use_ef      = true;
    std::vector<IndexT>       m_h_vertices;
    std::vector<Vector2i>     m_h_edges;
    std::vector<Vector3i>     m_h_faces;
    std::vector<Vector2i>     m_h_owned_vertices;
    DeviceBuffer<IndexT>      m_metadata;
    std::unique_ptr<Metadata> m_previous_metadata;
    DeviceBuffer<IndexT>      m_point_queries, m_edge_queries, m_face_queries;
    DeviceBuffer<IndexT>      m_groups[4];
    DeviceBuffer<AABB>        m_grown[4], m_leaf_boxes;
    DeviceBuffer<Float>       m_shrink[4];
    DeviceBuffer<IndexT>      m_leaf_cids, m_leaf_tags;
    bool m_node_symmetric = false, m_reverse_needed = false;
    bool m_cache_valid = false, m_tree_ready = false, m_rebuild_topology = true;
    bool m_skip_pt = false, m_skip_ee = false;
    bool m_frame_cover_left = false, m_frame_other_left = false;
    bool m_cover_thick = false, m_other_uniform = false;
    SizeT             m_frames = 0, m_cached_chunk_size = 0;
    Float             m_cached_skin_scale = -1;
    InfoStacklessBVH* m_tree              = nullptr;
    Statistics        m_stats;
    struct Chain
    {
        SideStream           side;
        DeviceBuffer<IndexT> ids, bids, cids;
        DeviceBuffer<AABB>   boxes;
        std::vector<std::unique_ptr<InfoStacklessBVH::QueryBuffer>> chunks;
        SizeT                                                       count = 0;
        DeviceBuffer<uint8_t>                                       masks;
        DeviceBuffer<CandidateCounts> counts, offsets, bases;
        DeviceVar<CandidateCounts>    totals;
        DeviceBuffer<Vector2i>        pts, ees;
    } m_chains[2];
    DeviceBuffer<Vector2i>        m_raw_pts, m_raw_ees, m_raw_pps, m_raw_pes;
    InfoStacklessBVH              m_edge_tree, m_cover_tree, m_codim_tree;
    InfoStacklessBVH::QueryBuffer m_direct[5];
    struct Gather
    {
        DeviceBuffer<IndexT> ids, bids, cids;
        DeviceBuffer<AABB>   boxes;
    } m_info_queries[3], m_info_faces;
    SideStream           m_aux_side;
    DeviceBuffer<IndexT> m_judge_flags, m_tile_counts, m_tile_offsets,
        m_dummy_counts, m_ee_queue_counts;
    DeviceBuffer<int4>     m_ee_queue;
    DeviceBuffer<IndexT>   m_control_device;
    ReadbackBuffer<IndexT> m_control, m_counts, m_ef_counts, m_direct_counts;
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
        m_control_device.resize(6);
        m_control.resize(6);
        m_counts.resize(8);
        m_direct_counts.resize(5);
        m_ef_counts.resize(2);
        m_raw_counts.resize(4);
        m_dummy_counts.resize(5);
        m_ee_queue_counts.resize(3);
    }
    void wait()
    {
        CUDA_TOOL_CHECK(cudaEventRecord(m_ready, default_stream()));
        CUDA_TOOL_CHECK(cudaEventSynchronize(m_ready));
    }
    bool prepare_topology(const PointTrianglePredicate&, const EdgeEdgePredicate&, bool unchanged);
    bool prepare_direct_topology(const PointTrianglePredicate&, const EdgeEdgePredicate&);
    void update_cover(const Metadata&);
    void prepare_proxies(const PointTrianglePredicate&,
                         const EdgeEdgePredicate&,
                         Bounds,
                         Bounds,
                         Bounds,
                         Float,
                         const Codim*);
    void refresh_direct(InfoStacklessBVH&,
                        const PointTrianglePredicate&,
                        const EdgeEdgePredicate&,
                        const NodePredicate&,
                        Bounds,
                        Bounds,
                        Bounds,
                        const Codim*,
                        bool auxiliary_only);
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
               const std::function<void()>& hook,
               const Codim*                 codim);
};
bool SimplexBVHQuery::Impl::prepare_topology(const bvh_simplex_detail::PointTrianglePredicate& pt,
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

    m_h_owned_vertices = owned_vertices;
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

bool SimplexBVHQuery::Impl::prepare_direct_topology(const PointTrianglePredicate& pt,
                                                    const EdgeEdgePredicate& ee)
{
    m_vertex_count   = pt.Vs.size();
    m_edge_count     = ee.Es.size();
    m_triangle_count = pt.Fs.size();
    m_position_count = pt.Ps.size();
    if(std::max({m_vertex_count, m_edge_count, m_triangle_count}) > PairLimit)
        return false;
    m_h_vertices.resize(m_vertex_count);
    m_h_edges.resize(m_edge_count);
    m_h_faces.resize(m_triangle_count);
    pt.Vs.copy_to(m_h_vertices.data());
    ee.Es.copy_to(m_h_edges.data());
    pt.Fs.copy_to(m_h_faces.data());
    auto valid = [&](IndexT v) { return v >= 0 && SizeT(v) < m_position_count; };
    for(auto v : m_h_vertices)
        if(!valid(v))
            return false;
    for(const auto& e : m_h_edges)
        for(auto v : e)
            if(!valid(v))
                return false;
    for(const auto& f : m_h_faces)
        for(auto v : f)
            if(!valid(v))
                return false;
    m_previous_vertices.resize_discard(pt.Vs.size());
    m_previous_edges.resize_discard(ee.Es.size());
    m_previous_triangles.resize_discard(pt.Fs.size());
    m_previous_vertices.view().copy_from(pt.Vs);
    m_previous_edges.view().copy_from(ee.Es);
    m_previous_triangles.view().copy_from(pt.Fs);
    m_h_owned_vertices.clear();
    m_owned_vertices.resize_discard(0);
    m_owned_edges.resize_discard(0);
    m_use_ef      = false;
    m_eligibility = Eligibility::Supported;
    return true;
}

void SimplexBVHQuery::Impl::update_cover(const Metadata& metadata)
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
    m_groups[3].resize_discard(m_codim_count);
    BufferLaunch().fill(m_groups[3].view(), IndexT{0});
    std::vector<IndexT> direct_ids[4];
    for(SizeT i = 0; i < edges.size(); ++i)
        if(edges[i])
            direct_ids[0].push_back(static_cast<IndexT>(i));
    for(SizeT i = 0; i < points.size(); ++i)
        direct_ids[points[i] ? 1 : 2].push_back(static_cast<IndexT>(i));
    for(SizeT i = 0; i < faces.size(); ++i)
        if(faces[i])
            direct_ids[3].push_back(static_cast<IndexT>(i));
    for(int family = 0; family < 3; ++family)
        upload(m_info_queries[family].ids, direct_ids[family]);
    upload(m_info_faces.ids, direct_ids[3]);
    m_node_symmetric = readable(Metadata::NodeMask);
    std::vector<IndexT> ids[2];
    for(SizeT i = 0; i < m_h_owned_vertices.size(); ++i)
    {
        bool primary = i < edges.size() && edges[i] != 0, reverse = false;
        for(const IndexT v : m_h_owned_vertices[i])
            if(v >= 0)
            {
                primary |= points[v] != 0;
                reverse |= m_reverse_needed && points[v] == 0;
            }
        if(primary)
            ids[0].push_back(static_cast<IndexT>(i));
        if(reverse)
            ids[1].push_back(static_cast<IndexT>(i));
    }
    for(int family = 0; family < 2; ++family)
    {
        auto& chain = m_chains[family];
        upload(chain.ids, ids[family]);
        chain.count = ids[family].size();
        chain.bids.resize(chain.count, IndexT{-1});
        chain.boxes.resize_discard(chain.count);
        chain.cids.resize_discard(chain.count);
    }
    m_stats.primary_queries =
        m_use_ef ? m_chains[0].count :
                   m_info_queries[0].ids.size() + m_info_queries[1].ids.size();
    m_stats.reverse_queries = m_use_ef         ? m_chains[1].count :
                              m_reverse_needed ? m_info_queries[2].ids.size() :
                                                 0;
}

void SimplexBVHQuery::Impl::prepare_proxies(const PointTrianglePredicate& pt,
                                            const EdgeEdgePredicate&      ee,
                                            Bounds       points,
                                            Bounds       edges,
                                            Bounds       faces,
                                            Float        skin_scale,
                                            const Codim* codim)
{
    m_stats.cover_skin = m_cover_thick ? 2.0 : 1.0;
    m_stats.other_skin = m_other_uniform ? 1.0 : 0.01;
    const Bounds cp    = codim ? codim->points : Bounds{};
    ProxyBounds  p{};
    p.fat                  = {{points, edges, faces, cp},
                              {m_groups[0].cview(),
                               m_groups[1].cview(),
                               m_groups[2].cview(),
                               m_groups[3].cview()},
                              pt.Vs,
             codim ? codim->pp.codimVs : CBufferView<IndexT>{},
                              ee.Es,
                              pt.Fs,
                              pt.d_hats,
                              skin_scale,
                              skin_scale * m_stats.cover_skin,
                              skin_scale * m_stats.other_skin};
    p.owned_vertices       = m_owned_vertices.cview();
    p.owned_edges          = m_owned_edges.cview();
    p.point_queries        = m_point_queries.cview();
    p.edge_queries         = m_edge_queries.cview();
    p.face_queries         = m_face_queries.cview();
    int          blocks    = 0;
    const Bounds inputs[4] = {points, edges, faces, cp};
    for(int family = 0; family < 4; ++family)
    {
        const SizeT n = inputs[family].aabbs.size();
        m_grown[family].resize_discard(n);
        m_shrink[family].resize_discard(n);
        p.grown[family]  = m_grown[family].view();
        p.shrink[family] = m_shrink[family].view();
        blocks += (n + Threads - 1) / Threads;
    }
    if(m_use_ef)
    {
        for(int family = 0; family < 2; ++family)
        {
            auto& chain           = m_chains[family];
            p.ids[family]         = chain.ids.cview();
            p.query_boxes[family] = chain.boxes.view();
            p.query_cids[family]  = chain.cids.view();
            blocks += (chain.count + Threads - 1) / Threads;
        }
        const SizeT leaves = m_owned_edges.size();
        m_leaf_boxes.resize_discard(leaves);
        m_leaf_cids.resize_discard(leaves);
        m_leaf_tags.resize_discard(leaves);
        p.leaf_boxes = m_leaf_boxes.view();
        p.leaf_cids  = m_leaf_cids.view();
        p.leaf_tags  = m_leaf_tags.view();
        blocks += (leaves + Threads - 1) / Threads;
    }
    if(blocks)
        prepare_cache_bounds<<<blocks, Threads>>>(p);
}

void SimplexBVHQuery::Impl::refresh(InfoStacklessBVH&             tree,
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
    m_tree                     = &tree;
    m_tree_ready               = true;
    SizeT             first[2] = {}, all_chunks = 0;
    std::vector<bool> pending[2];
    for(int family = 0; family < 2; ++family)
    {
        auto&       chain = m_chains[family];
        const SizeT count = (chain.count + chunk_size - 1) / chunk_size;
        while(chain.chunks.size() < count)
        {
            auto chunk = std::make_unique<InfoStacklessBVH::QueryBuffer>();
            const SizeT queries =
                std::min(chunk_size, chain.count - chain.chunks.size() * chunk_size);
            chunk->m_pairs.resize_discard(
                std::min(chunk->m_pairs.size(), queries * m_owned_edges.size()));
            chain.chunks.push_back(std::move(chunk));
        }
        chain.chunks.resize(count);
        chain.bases.resize_discard(count);
        first[family] = all_chunks;
        all_chunks += count;
        pending[family].assign(count, true);
        store_value(chain.totals.data(), CandidateCounts{});
        reserve_pairs(chain.pts, 1024, tree.config().reserve_ratio, false);
        reserve_pairs(chain.ees, 1024, tree.config().reserve_ratio, false);
    }
    m_ef_counts.resize(all_chunks);
    auto pt      = pt_input;
    auto ee      = ee_input;
    pt.geometry  = false;
    ee.geometry  = false;
    auto scratch = [&](Chain& chain)
    {
        SizeT capacity = 0;
        for(const auto& chunk : chain.chunks)
            capacity = std::max(capacity, chunk->m_pairs.size());
        chain.masks.resize_discard(capacity);
        chain.counts.resize_discard(capacity + 1);
        chain.offsets.resize_discard(capacity + 1);
    };
    auto expand = [&](int family, SizeT chunk, cudaStream_t stream, bool reserve)
    {
        auto&       chain = m_chains[family];
        auto&       ef    = *chain.chunks[chunk];
        const SizeT n     = ef.m_pairs.size();
        const auto  ids =
            chain.ids.cview(chunk * chunk_size,
                            std::min(chunk_size, chain.count - chunk * chunk_size));
        count_candidates<<<(n + Threads - 1) / Threads, Threads, 0, stream>>>(
            ef.m_pairs.cview(),
            ef.m_cpNum.cview(),
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
            chain.masks.view(0, n),
            chain.counts.view(0, n + 1));
        DeviceScan(stream).ExclusiveScan(chain.counts.data(),
                                         chain.offsets.data(),
                                         AddCandidateCounts{},
                                         CandidateCounts{},
                                         static_cast<int>(n + 1));
        if(reserve)
            reserve_chunk<<<1, 1, 0, stream>>>(chain.offsets.cview(0, n + 1),
                                               chain.totals.view(),
                                               chain.bases.view(),
                                               chunk);
        fill_candidates<<<(n + Threads - 1) / Threads, Threads, 0, stream>>>(
            ef.m_pairs.cview(),
            ef.m_cpNum.cview(),
            ids,
            m_owned_vertices.cview(),
            m_owned_edges.cview(),
            chain.masks.cview(0, n),
            chain.offsets.cview(0, n + 1),
            chain.bases.cview(),
            chunk,
            chain.pts.view(),
            chain.ees.view());
    };
    bool initial = true, retry;
    do
    {
        for(auto& chain : m_chains)
            scratch(chain);
        bool forked[2] = {};
        try
        {
            // Mark both forks before either join so their full pipelines overlap.
            for(int family = 0; family < 2; ++family)
                if(std::any_of(pending[family].begin(),
                               pending[family].end(),
                               [](bool p) { return p; }))
                    m_chains[family].side.mark();
            for(int family = 0; family < 2; ++family)
            {
                auto& chain = m_chains[family];
                for(SizeT chunk = 0; chunk < chain.chunks.size(); ++chunk)
                {
                    if(!pending[family][chunk])
                        continue;
                    const auto stream  = chain.side.fork_marked();
                    forked[family]     = true;
                    auto&       ef     = *chain.chunks[chunk];
                    const SizeT offset = chunk * chunk_size,
                                count = std::min(chunk_size, chain.count - offset);
                    tree.launch_query(chain.boxes.cview(offset, count),
                                      chain.bids.cview(offset, count),
                                      chain.cids.cview(offset, count),
                                      {},
                                      ProxyNode{node, family == 1, m_node_symmetric},
                                      ProxyLeaf{m_face_queries.cview(), family == 1},
                                      ef,
                                      initial,
                                      reuse_order,
                                      stream);
                    publish_ef_count<<<1, 1, 0, stream>>>(ef.m_cpNum.cview(),
                                                          m_ef_counts.device_data()
                                                              + first[family] + chunk);
                    expand(family, chunk, stream, true);
                }
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
        publish_refresh_counts<<<1, 1>>>(m_chains[0].totals.cview(),
                                         m_chains[1].totals.cview(),
                                         m_raw_counts.device_data());
        m_ef_counts.enqueue_copy();
        m_raw_counts.enqueue_copy();
        wait();
        ++m_stats.refresh_waits;
        retry = false;
        for(int family = 0; family < 2; ++family)
        {
            auto& chain = m_chains[family];
            for(SizeT chunk = 0; chunk < chain.chunks.size(); ++chunk)
            {
                auto&     ef    = *chain.chunks[chunk];
                const int count = m_ef_counts[first[family] + chunk];
                if(count < 0)
                    throw std::overflow_error("Single EF proxy counter exceeds 32-bit indexing");
                pending[family][chunk] = SizeT(count) > ef.m_pairs.size();
                ef.m_size              = count;
                if(pending[family][chunk])
                {
                    const SizeT queries =
                        std::min(chunk_size, chain.count - chunk * chunk_size);
                    const SizeT capacity = std::min(
                        queries * m_owned_edges.size(),
                        static_cast<SizeT>(std::min(
                            Float(PairLimit),
                            std::max(Float(count), Float(count) * tree.config().reserve_ratio))));
                    ef.m_pairs.resize_discard(capacity);
                    retry = true;
                    ++m_stats.ef_retries;
                }
            }
        }
        initial = false;
    } while(retry);

    CandidateCounts totals[2] = {{m_raw_counts[0], m_raw_counts[1]},
                                 {m_raw_counts[2], m_raw_counts[3]}};
    const U64 np = totals[0].pt + totals[1].pt, ne = totals[0].ee + totals[1].ee;
    if(np > PairLimit || ne > PairLimit || np + ne > PairLimit)
        throw std::overflow_error("Single EF cached candidates exceed 32-bit downstream indexing");
    for(int family = 0; family < 2; ++family)
    {
        auto&      chain  = m_chains[family];
        const bool refill = totals[family].pt > chain.pts.size()
                            || totals[family].ee > chain.ees.size();
        if(!refill)
            continue;
        // Discard growth requires refilling every chunk in this chain, including
        // chunks that fitted before growth. Saved bases must not be reserved twice.
        reserve_pairs(chain.pts, totals[family].pt, tree.config().reserve_ratio, false);
        reserve_pairs(chain.ees, totals[family].ee, tree.config().reserve_ratio, false);
        const auto stream = chain.side.fork();
        try
        {
            for(SizeT chunk = 0; chunk < chain.chunks.size(); ++chunk)
                expand(family, chunk, stream, false);
        }
        catch(...)
        {
            chain.side.join();
            CUDA_TOOL_CHECK(cudaStreamSynchronize(default_stream()));
            throw;
        }
        chain.side.join();
        ++m_stats.typed_refills;
    }
    reserve_pairs(m_raw_pts, np, tree.config().reserve_ratio, false);
    reserve_pairs(m_raw_ees, ne, tree.config().reserve_ratio, false);
    SizeT po = 0, eo = 0;
    m_stats.ef_pairs = 0;
    for(int family = 0; family < 2; ++family)
    {
        const auto& chain = m_chains[family];
        if(totals[family].pt)
            m_raw_pts.view(po, totals[family].pt)
                .copy_from(chain.pts.cview(0, totals[family].pt));
        if(totals[family].ee)
            m_raw_ees.view(eo, totals[family].ee)
                .copy_from(chain.ees.cview(0, totals[family].ee));
        po += totals[family].pt;
        eo += totals[family].ee;
        for(const auto& chunk : chain.chunks)
            m_stats.ef_pairs += chunk->size();
    }
    m_stats.stored_pts  = np;
    m_stats.stored_ees  = ne;
    m_cached_chunk_size = chunk_size;
}

void SimplexBVHQuery::Impl::refresh_direct(InfoStacklessBVH&             tree,
                                           const PointTrianglePredicate& pt,
                                           const EdgeEdgePredicate&      ee,
                                           const NodePredicate&          node,
                                           Bounds                        points,
                                           Bounds                        edges,
                                           Bounds                        faces,
                                           const Codim*                  codim,
                                           bool auxiliary_only)
{
    const Bounds cp = codim ? codim->points : Bounds{};
    if(!auxiliary_only)
    {
        const int    families[4] = {1, 0, 0, 2};
        const Bounds metadata[4] = {edges, points, points, faces};
        for(int i = 0; i < 4; ++i)
        {
            auto&       g = i == 3 ? m_info_faces : m_info_queries[i];
            const SizeT n = g.ids.size();
            g.boxes.resize_discard(n);
            g.bids.resize_discard(n);
            g.cids.resize_discard(n);
            if(n)
                gather_direct_bounds<<<(n + Threads - 1) / Threads, Threads>>>(
                    g.ids.cview(),
                    m_grown[families[i]].cview(),
                    metadata[i],
                    g.boxes.view(),
                    g.bids.view(),
                    g.cids.view());
        }
    }
    const bool active[5] = {
        codim && cp.aabbs.size() && points.aabbs.size(),
        codim && cp.aabbs.size() && edges.aabbs.size(),
        !auxiliary_only && edges.aabbs.size(),
        !auxiliary_only && faces.aabbs.size() && m_info_queries[1].ids.size(),
        !auxiliary_only && m_stats.pt_cover && m_info_queries[2].ids.size()
            && m_info_faces.ids.size()};
    auto pp_pred = codim ? codim->pp : PointPointPredicate{};
    auto pe_pred = codim ? codim->pe : PointEdgePredicate{};
    auto pt_pred = pt;
    auto ee_pred = ee;
    pp_pred.geometry = pe_pred.geometry = pt_pred.geometry = ee_pred.geometry = false;
    auto launch = [&](int i, bool initial, cudaStream_t stream)
    {
        auto&      q           = m_direct[i];
        const bool reuse_order = !m_rebuild_topology;
        if(i == 0)
            m_codim_tree.launch_query(
                m_grown[0].cview(), points.bids, points.cids, {}, node, pp_pred, q, initial, reuse_order, stream);
        else if(i == 1)
            m_edge_tree.launch_query(
                m_grown[3].cview(), cp.bids, cp.cids, {}, node, pe_pred, q, initial, reuse_order, stream);
        else if(i == 2)
        {
            if(!m_stats.ee_cover)
                m_edge_tree.launch_detect({}, node, ee_pred, q, stream);
            else
            {
                auto& g = m_info_queries[0];
                m_edge_tree.launch_query(g.boxes.cview(),
                                         g.bids.cview(),
                                         g.cids.cview(),
                                         {},
                                         node,
                                         bvh_pair_judge::EEPredicate{
                                             g.ids.cview(), m_edge_queries.cview(), ee_pred},
                                         q,
                                         initial,
                                         reuse_order,
                                         stream);
            }
        }
        else
        {
            auto& g   = m_info_queries[i == 3 ? 1 : 2];
            auto& bvh = i == 3 ? tree : m_cover_tree;
            bvh.launch_query(
                g.boxes.cview(),
                g.bids.cview(),
                g.cids.cview(),
                {},
                node,
                InfoStacklessBVHSimplexTrajectoryFilter_detect_cover_AllP_AllT_pred{
                    g.ids.cview(),
                    i == 3 ? CBufferView<IndexT>{} : m_info_faces.ids.cview(),
                    pt_pred},
                q,
                initial,
                reuse_order,
                stream);
        }
    };
    auto update_tree = [&](InfoStacklessBVH&   bvh,
                           CBufferView<AABB>   boxes,
                           CBufferView<IndexT> bids,
                           CBufferView<IndexT> cids,
                           cudaStream_t        stream)
    {
        if(m_rebuild_topology || !bvh.refit(boxes, bids, cids, stream))
        {
            bvh.build(boxes, bids, cids, stream);
            ++m_stats.topology_builds;
        }
    };
    bool pending[5];
    for(int i = 0; i < 5; ++i)
    {
        pending[i] = active[i];
        if(!active[i])
        {
            m_direct[i].m_cpNum.copy_from(0);
            m_direct[i].m_size = 0;
        }
    }
    bool initial = true, retry = false;
    do
    {
        m_chains[0].side.mark();
        m_chains[1].side.mark();
        m_aux_side.mark();
        const cudaStream_t streams[3] = {m_chains[0].side.fork_marked(),
                                         m_chains[1].side.fork_marked(),
                                         m_aux_side.fork_marked()};
        try
        {
            if(initial)
            {
                if(active[0])
                    update_tree(m_codim_tree, m_grown[3].cview(), cp.bids, cp.cids, streams[0]);
                if(active[1] || active[2])
                    update_tree(
                        m_edge_tree, m_grown[1].cview(), edges.bids, edges.cids, streams[0]);
                if(active[3])
                    update_tree(tree, m_grown[2].cview(), faces.bids, faces.cids, streams[1]);
                if(active[4])
                    update_tree(m_cover_tree,
                                m_info_faces.boxes.cview(),
                                m_info_faces.bids.cview(),
                                m_info_faces.cids.cview(),
                                streams[2]);
            }
            for(int i = 0; i < 5; ++i)
                if(pending[i])
                    launch(i, initial, streams[i < 3 ? 0 : i - 2]);
        }
        catch(...)
        {
            m_chains[0].side.join();
            m_chains[1].side.join();
            m_aux_side.join();
            CUDA_TOOL_CHECK(cudaStreamSynchronize(default_stream()));
            throw;
        }
        m_chains[0].side.join();
        m_chains[1].side.join();
        m_aux_side.join();
        for(int i = 0; i < 5; ++i)
            publish_ef_count<<<1, 1>>>(m_direct[i].m_cpNum.cview(),
                                       m_direct_counts.device_data() + i);
        m_direct_counts.enqueue_copy();
        wait();
        ++m_stats.refresh_waits;
        retry = false;
        for(int i = 0; i < 5; ++i)
        {
            const int n = m_direct_counts[i];
            if(n < 0)
                throw std::overflow_error("Simplex BVH direct pair count exceeds indexing");
            pending[i] = active[i] && SizeT(n) > m_direct[i].m_pairs.size();
            if(pending[i])
            {
                reserve_pairs(m_direct[i].m_pairs, n, tree.config().reserve_ratio, false);
                retry = true;
            }
            m_direct[i].m_size = n;
        }
        initial = false;
    } while(retry);

    // Retain the existing optional traversal audit at the common refresh owner.
    if(const char* path = std::getenv("UIPC_BVH_QUERY_CHECK"); path && *path)
    {
        FILE*       log      = std::fopen(path, "a");
        const char* names[5] = {"PP", "PE", "EE", "PT", "PcoverT"};
        const auto  less     = [](const Vector2i& a, const Vector2i& b)
        { return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]); };
        bool same = true;
        for(int i = 0; i < 5; ++i)
        {
            if(!active[i])
                continue;
            auto&                 bvh = i == 0 ? m_codim_tree :
                                        i < 3  ? m_edge_tree :
                                        i == 3 ? tree :
                                                 m_cover_tree;
            auto&                 q   = m_direct[i];
            const int             n   = q.m_size;
            std::vector<Vector2i> original(n);
            q.view().copy_to(original.data());
            const bool cooperative = bvh.config().warp_cooperative_query;
            bvh.config().warp_cooperative_query = false;
            try
            {
                launch(i, false, default_stream());
            }
            catch(...)
            {
                bvh.config().warp_cooperative_query = cooperative;
                if(log)
                    std::fclose(log);
                throw;
            }
            const int nr                        = q.m_cpNum;
            bvh.config().warp_cooperative_query = cooperative;
            if(nr < 0 || SizeT(nr) > q.m_pairs.size())
                throw std::runtime_error("BVH query reference traversal exceeds successful output capacity");
            std::vector<Vector2i> reference(nr), sorted = original;
            q.m_pairs.cview(0, nr).copy_to(reference.data());
            q.m_pairs.view(0, n).copy_from(original.data());
            q.m_cpNum.copy_from(n);
            std::sort(sorted.begin(), sorted.end(), less);
            std::sort(reference.begin(), reference.end(), less);
            SizeT a = 0, b = 0, only_a = 0, only_b = 0;
            while(a < sorted.size() && b < reference.size())
            {
                if(less(sorted[a], reference[b]))
                {
                    ++only_a;
                    ++a;
                }
                else if(less(reference[b], sorted[a]))
                {
                    ++only_b;
                    ++b;
                }
                else
                {
                    ++a;
                    ++b;
                }
            }
            only_a += sorted.size() - a;
            only_b += reference.size() - b;
            same &= only_a == 0 && only_b == 0;
            if(log)
                std::fprintf(log,
                             "{\"detect\":%zu,\"alpha\":%.9g,\"stored\":1,\"same\":%d,\"%s\":[%d,%d,%zu,%zu]}\n",
                             size_t(m_stats.refreshes),
                             double(pt.alpha),
                             int(!only_a && !only_b),
                             names[i],
                             n,
                             nr,
                             size_t(only_a),
                             size_t(only_b));
        }
        if(log)
            std::fclose(log);
        if(!same)
            throw std::runtime_error("BVH warp-cooperative query check mismatch");
    }

    const SizeT np = SizeT(m_direct[3].m_size) + SizeT(m_direct[4].m_size);
    const SizeT ne = m_direct[2].m_size, npp = m_direct[0].m_size,
                npe = m_direct[1].m_size;
    if(np + ne + npp + npe > PairLimit)
        throw std::overflow_error("Simplex BVH cached candidates exceed downstream indexing");
    reserve_pairs(m_raw_pps, npp, tree.config().reserve_ratio, false);
    reserve_pairs(m_raw_pes, npe, tree.config().reserve_ratio, false);
    if(npp)
        m_raw_pps.view(0, npp).copy_from(m_direct[0].view());
    if(npe)
        m_raw_pes.view(0, npe).copy_from(m_direct[1].view());
    m_stats.stored_pps = npp;
    m_stats.stored_pes = npe;
    if(!auxiliary_only)
    {
        reserve_pairs(m_raw_ees, ne, tree.config().reserve_ratio, false);
        reserve_pairs(m_raw_pts, np, tree.config().reserve_ratio, false);
        for(int i = 2; i < 5; ++i)
        {
            const int n = m_direct[i].m_size;
            if(!n)
                continue;
            const auto query_ids =
                i == 2 ? (m_stats.ee_cover ? m_info_queries[0].ids.cview() :
                                             CBufferView<IndexT>{}) :
                         m_info_queries[i == 3 ? 1 : 2].ids.cview();
            const auto leaf_ids =
                i == 4 ? m_info_faces.ids.cview() : CBufferView<IndexT>{};
            auto out = i == 2 ? m_raw_ees.view(0, n) :
                                m_raw_pts.view(i == 4 ? m_direct[3].m_size : 0, n);
            normalize_direct_pairs<<<(n + Threads - 1) / Threads, Threads>>>(
                m_direct[i].view(), query_ids, leaf_ids, out, i == 2);
        }
        m_stats.stored_pts = np;
        m_stats.stored_ees = ne;
        m_stats.ef_pairs   = 0;
    }
}

void SimplexBVHQuery::Impl::judge(InfoStacklessBVH&              tree,
                                  const PointTrianglePredicate&  pt_input,
                                  const EdgeEdgePredicate&       ee_input,
                                  Bounds                         points,
                                  Bounds                         edges,
                                  Bounds                         faces,
                                  InfoStacklessBVH::QueryBuffer& pts,
                                  InfoStacklessBVH::QueryBuffer& ees,
                                  bool                           gated,
                                  const std::function<void()>&   hook,
                                  const Codim*                   codim)
{
    using namespace bvh_pair_judge;
    const SizeT sizes[4] = {
        m_stats.stored_pps, m_stats.stored_pes, m_stats.stored_ees, m_stats.stored_pts};
    const SizeT total = sizes[0] + sizes[1] + sizes[2] + sizes[3];
    if(total > PairLimit)
        throw std::overflow_error("Simplex BVH judge exceeds downstream indexing");
    InfoStacklessBVH::QueryBuffer* outputs[4] = {
        codim ? codim->pps : nullptr, codim ? codim->pes : nullptr, &ees, &pts};
    DeviceBuffer<Vector2i>* raw[4] = {&m_raw_pps, &m_raw_pes, &m_raw_ees, &m_raw_pts};
    PairLists lists{};
    SizeT     offsets[4]{};
    int       tiles = 0;
    for(int i = 0; i < 4; ++i)
    {
        if(outputs[i])
            reserve_pairs(outputs[i]->m_pairs, sizes[i], tree.config().reserve_ratio, false);
        offsets[i]    = i ? offsets[i - 1] + sizes[i - 1] : 0;
        lists.list[i] = {raw[i]->data(),
                         nullptr,
                         outputs[i] ? outputs[i]->m_pairs.data() : nullptr,
                         outputs[i] ? outputs[i]->m_cpNum.data() :
                                      m_dummy_counts.data() + i,
                         static_cast<int>(sizes[i]),
                         tiles};
        tiles += (sizes[i] + JudgeTile - 1) / JudgeTile;
    }
    lists.list[4].count      = m_dummy_counts.data() + 4;
    lists.list[4].first_tile = tiles;
    m_judge_flags.resize_discard(total);
    m_tile_counts.resize_discard(tiles);
    m_tile_offsets.resize_discard(tiles);
    for(int i = 0; i < 4; ++i)
        lists.list[i].flags = m_judge_flags.data() + offsets[i];
    EEQueue queue{nullptr, nullptr, m_ee_queue_counts.data(), static_cast<int>(sizes[2])};
    if(sizes[2] && ee_input.alpha != 0.0 && ee_input.f32_positions.size())
    {
        m_ee_queue.resize_discard(2 * sizes[2]);
        queue.second = m_ee_queue.data();
        queue.pairs  = m_ee_queue.data() + sizes[2];
    }
    auto ee_pred               = ee_input;
    auto pt_pred               = pt_input;
    ee_pred.skip_position_free = pt_pred.skip_position_free = true;
    const IndexT* invalid = gated ? m_control_device.data() : nullptr;
    m_chains[0].side.mark();
    m_chains[1].side.mark();
    m_aux_side.mark();
    const cudaStream_t streams[3] = {m_chains[0].side.fork_marked(),
                                     m_chains[1].side.fork_marked(),
                                     m_aux_side.fork_marked()};
    try
    {
        judge_ee(m_raw_ees.cview(0, sizes[2]),
                 edges.aabbs,
                 edges.aabbs,
                 EEPredicate{{}, {}, ee_pred},
                 m_judge_flags.view(offsets[2], sizes[2]),
                 m_tile_counts.view(lists.list[2].first_tile,
                                    (sizes[2] + JudgeTile - 1) / JudgeTile),
                 queue,
                 m_sm_count,
                 streams[0],
                 invalid);
        judge_stored(m_raw_pts.cview(0, sizes[3]),
                     points.aabbs,
                     faces.aabbs,
                     pt_pred,
                     m_judge_flags.view(offsets[3], sizes[3]),
                     m_tile_counts.view(lists.list[3].first_tile,
                                        (sizes[3] + JudgeTile - 1) / JudgeTile),
                     static_cast<int>(sizes[3]),
                     streams[1],
                     invalid);
        if(codim)
        {
            judge_stored(m_raw_pps.cview(0, sizes[0]),
                         points.aabbs,
                         codim->points.aabbs,
                         codim->pp,
                         m_judge_flags.view(offsets[0], sizes[0]),
                         m_tile_counts.view(lists.list[0].first_tile,
                                            (sizes[0] + JudgeTile - 1) / JudgeTile),
                         static_cast<int>(sizes[0]),
                         streams[2],
                         invalid);
            judge_stored(m_raw_pes.cview(0, sizes[1]),
                         codim->points.aabbs,
                         edges.aabbs,
                         codim->pe,
                         m_judge_flags.view(offsets[1], sizes[1]),
                         m_tile_counts.view(lists.list[1].first_tile,
                                            (sizes[1] + JudgeTile - 1) / JudgeTile),
                         static_cast<int>(sizes[1]),
                         streams[2],
                         invalid);
        }
    }
    catch(...)
    {
        m_chains[0].side.join();
        m_chains[1].side.join();
        m_aux_side.join();
        CUDA_TOOL_CHECK(cudaStreamSynchronize(default_stream()));
        throw;
    }
    m_chains[0].side.join();
    m_chains[1].side.join();
    m_aux_side.join();
    compact(lists,
            m_tile_counts.cview(),
            m_tile_offsets.view(),
            m_control_device.cview(0, 3),
            BufferView<IndexT>{m_counts.device_data(), 8},
            m_ee_queue_counts.data(),
            tiles,
            m_sm_count);
    publish_control<<<1, 32>>>(m_control_device.cview(), m_control.device_data());
    m_control.enqueue_copy();
    m_counts.enqueue_copy();
    CUDA_TOOL_CHECK(cudaEventRecord(m_ready, default_stream()));
    if(hook)
        hook();
    CUDA_TOOL_CHECK(cudaEventSynchronize(m_ready));
    if(!gated || !m_control[0])
        for(int i = 0; i < 4; ++i)
            if(outputs[i])
                outputs[i]->m_size = m_counts[i];
}

SimplexBVHQuery::SimplexBVHQuery()
    : m_impl(std::make_unique<Impl>())
{
}
SimplexBVHQuery::~SimplexBVHQuery() = default;
SizeT SimplexBVHQuery::refresh_count() const noexcept
{
    return m_impl->m_stats.refreshes;
}
SizeT SimplexBVHQuery::reuse_count() const noexcept
{
    return m_impl->m_stats.reuses;
}
SizeT SimplexBVHQuery::isolated_point_count() const noexcept
{
    return m_impl->m_eligibility == Impl::Eligibility::Supported && m_impl->m_use_ef ?
               m_impl->m_owned_vertices.size() - m_impl->m_edge_count :
               0;
}
SizeT SimplexBVHQuery::standalone_edge_count() const noexcept
{
    return m_impl->m_eligibility == Impl::Eligibility::Supported && m_impl->m_use_ef ?
               m_impl->m_owned_edges.size() - m_impl->m_triangle_count :
               0;
}
SimplexBVHQuery::Statistics SimplexBVHQuery::statistics() const noexcept
{
    return m_impl->m_stats;
}

bool SimplexBVHQuery::detect(InfoStacklessBVH&              tree,
                             const PointTrianglePredicate&  pt,
                             const EdgeEdgePredicate&       ee,
                             const NodePredicate&           node,
                             PrimitiveBounds                points,
                             PrimitiveBounds                edges,
                             PrimitiveBounds                faces,
                             InfoStacklessBVH::QueryBuffer& pts,
                             InfoStacklessBVH::QueryBuffer& ees,
                             bool                           new_frame,
                             const std::function<void()>&   hook,
                             TrajectoryPrefix*              prefix,
                             const Codim*                   codim)
{
    UIPC_ASSERT(pt.alpha == ee.alpha && pt.alpha >= 0.0,
                "Simplex PT and EE queries must share a nonnegative alpha");
    UIPC_ASSERT(points.aabbs.size() == pt.Vs.size()
                    && edges.aabbs.size() == ee.Es.size()
                    && faces.aabbs.size() == pt.Fs.size(),
                "Simplex primitive bounds do not match topology");
    UIPC_ASSERT(!codim
                    || (codim->pps && codim->pes
                        && codim->points.aabbs.size() == codim->pp.codimVs.size()
                        && codim->pp.alpha == pt.alpha && codim->pe.alpha == pt.alpha),
                "Codimensional queries must bind matching bounds, motion and outputs");
    auto& s = *m_impl;
    s.init();
    const Bounds cp        = codim ? codim->points : Bounds{};
    const auto cp_vertices = codim ? codim->pp.codimVs : CBufferView<IndexT>{};
    if(new_frame)
    {
        ++s.m_frames;
        if(s.m_frames % TopologyPeriod == 0)
            s.m_rebuild_topology = true;
        s.m_cover_thick      = s.m_frame_cover_left;
        s.m_other_uniform    = s.m_frame_other_left;
        s.m_frame_cover_left = s.m_frame_other_left = false;
    }
    const bool method_changed = s.m_method != method;
    const bool topology_shape =
        !method_changed && s.m_eligibility == Impl::Eligibility::Supported
        && s.m_vertex_count == pt.Vs.size() && s.m_edge_count == ee.Es.size()
        && s.m_triangle_count == pt.Fs.size() && s.m_position_count == pt.Ps.size()
        && s.m_codim_count == cp_vertices.size();
    const Metadata metadata(pt, ee, node, points, edges, faces, codim);
    const bool     metadata_shape =
        s.m_previous_metadata && metadata.same_shape(*s.m_previous_metadata);
    const bool policy_change = s.m_skip_pt != pt.skip_position_free
                               || s.m_skip_ee != ee.skip_position_free;
    const SizeT prospective_leaves =
        topology_shape && s.m_use_ef ? s.m_owned_edges.size() : 0;
    const SizeT chunk_size = prospective_leaves ?
                                 std::min({s.m_owned_vertices.size(),
                                           std::max(SizeT{1}, max_chunk_edges),
                                           PairLimit / prospective_leaves}) :
                                 0;
    const Float skin_scale = std::max(Float{0}, cache_skin_scale);
    const bool  known_refresh =
        force_fresh || !s.m_cache_valid || !topology_shape || !metadata_shape
        || policy_change || s.m_cached_skin_scale != skin_scale
        || s.m_cached_chunk_size != chunk_size || s.m_tree != &tree;
    const IndexT initial[6] = {known_refresh ? 1 : 0,
                               topology_shape ? 0 : 1,
                               metadata_shape && !policy_change && !method_changed ? 0 : 1,
                               0,
                               0,
                               0};
    Validation validation{pt.Vs,
                          s.m_previous_vertices.cview(),
                          ee.Es,
                          s.m_previous_edges.cview(),
                          pt.Fs,
                          s.m_previous_triangles.cview(),
                          cp_vertices,
                          s.m_previous_codim.cview(),
                          metadata,
                          s.m_metadata.cview(),
                          {points.aabbs, edges.aabbs, faces.aabbs, cp.aabbs},
                          {s.m_grown[0].cview(),
                           s.m_grown[1].cview(),
                           s.m_grown[2].cview(),
                           s.m_grown[3].cview()},
                          {s.m_groups[0].cview(),
                           s.m_groups[1].cview(),
                           s.m_groups[2].cview(),
                           s.m_groups[3].cview()},
                          {s.m_shrink[0].cview(),
                           s.m_shrink[1].cview(),
                           s.m_shrink[2].cview(),
                           s.m_shrink[3].cview()},
                          topology_shape,
                          metadata_shape,
                          !known_refresh};
    if(prefix)
    {
        for(auto& family : prefix->family)
        {
            family.previous_vertices  = nullptr;
            family.previous_edges     = nullptr;
            family.previous_triangles = nullptr;
            family.previous_bids      = nullptr;
            family.previous_cids      = nullptr;
            family.cached_grown       = nullptr;
            family.cached_groups      = nullptr;
            family.cached_shrink      = nullptr;
            family.grown              = nullptr;
            family.is_query           = nullptr;
        }
        prefix->escaped                 = nullptr;
        prefix->accepted_step_endpoints = true;
        auto& check                     = prefix->validation;
        check                           = {};
        check.invalid                   = s.m_control_device.data();
        check.topology_changed          = s.m_control_device.data() + 1;
        check.metadata_changed          = s.m_control_device.data() + 2;
        check.group_escaped[0]          = s.m_control_device.data() + 5;
        check.group_escaped[1]          = s.m_control_device.data() + 3;
        check.group_escaped[2]          = s.m_control_device.data() + 4;
        check.initial_invalid           = initial[0];
        check.initial_topology          = initial[1];
        check.initial_metadata          = initial[2];
        static_assert(int(Metadata::Arrays) + int(Metadata::Matrices) - 8
                      <= PrefixValidation::MaxRanges);
        if(metadata_shape)
        {
            for(int a = 0; a < Metadata::Arrays; ++a)
            {
                // These eight ranges are produced and checked by their own primitive threads.
                if(a >= Metadata::PointBodies && a <= Metadata::CodimContacts)
                    continue;
                auto& range    = check.ranges[check.count++];
                range.values   = metadata.arrays[a];
                range.previous = s.m_metadata.data() + metadata.offsets[a];
                range.n        = metadata.arrays[a].size();
                check.work     = std::max(check.work, range.n);
            }
            for(int a = 0; a < Metadata::Matrices; ++a)
            {
                auto& range  = check.ranges[check.count++];
                range.matrix = metadata.matrices[a];
                range.previous =
                    s.m_metadata.data() + metadata.offsets[Metadata::Arrays + a];
                range.n         = metadata.matrices[a].total_size();
                range.is_matrix = true;
                check.work      = std::max(check.work, range.n);
            }
        }
        for(int family = 0; family < 4; ++family)
        {
            auto& f = prefix->family[(family + 1) % 4];
            if(metadata_shape)
            {
                f.previous_bids = s.m_metadata.data()
                                  + metadata.offsets[Metadata::PointBodies + 2 * family];
                f.previous_cids = s.m_metadata.data()
                                  + metadata.offsets[Metadata::PointContacts + 2 * family];
            }
            if(!known_refresh)
            {
                f.cached_grown  = s.m_grown[family].data();
                f.cached_groups = s.m_groups[family].data();
                f.cached_shrink = s.m_shrink[family].data();
            }
        }
        if(topology_shape)
        {
            prefix->family[0].previous_vertices = s.m_previous_codim.data();
            prefix->family[1].previous_vertices = s.m_previous_vertices.data();
            prefix->family[2].previous_edges    = s.m_previous_edges.data();
            prefix->family[3].previous_triangles = s.m_previous_triangles.data();
        }
        launch_trajectory_prefix(*prefix, default_stream());
        ++s.m_stats.fused_prefixes;
    }
    else
    {
        s.m_control_device.view().copy_from(initial);
        const SizeT work = std::max({metadata.work,
                                     pt.Vs.size(),
                                     ee.Es.size(),
                                     pt.Fs.size(),
                                     cp_vertices.size()});
        if(work)
            validate_cache<<<(work + Threads - 1) / Threads, Threads>>>(
                validation, s.m_control_device.data(), work);
    }
    if(!known_refresh)
    {
        s.judge(tree, pt, ee, points, edges, faces, pts, ees, true, hook, codim);
        if(!s.m_control[0])
        {
            s.m_frame_cover_left |= s.m_control[3] != 0;
            s.m_frame_other_left |= s.m_control[4] != 0;
            ++s.m_stats.reuses;
            if(new_frame)
                ++s.m_stats.cross_frame_reuses;
            return true;
        }
    }
    else
    {
        publish_control<<<1, 32>>>(s.m_control_device.cview(), s.m_control.device_data());
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
    s.m_method     = method;
    bool supported = topology_shape && !topology_changed;
    if(!supported)
    {
        s.m_use_ef = method == Method::SingleEF;
        supported  = s.m_use_ef && s.prepare_topology(pt, ee, false);
        if(!supported && (method == Method::InfoStackless || codim))
            supported = s.prepare_direct_topology(pt, ee);
    }
    if(!supported)
    {
        pts.m_cpNum.copy_from(static_cast<int>(pts.m_size));
        ees.m_cpNum.copy_from(static_cast<int>(ees.m_size));
        if(codim)
        {
            codim->pps->m_cpNum.copy_from(static_cast<int>(codim->pps->m_size));
            codim->pes->m_cpNum.copy_from(static_cast<int>(codim->pes->m_size));
        }
        return false;
    }
    s.m_codim_count = cp_vertices.size();
    if(topology_changed)
    {
        s.m_previous_codim.resize_discard(cp_vertices.size());
        s.m_previous_codim.view().copy_from(cp_vertices);
        s.m_rebuild_topology = true;
    }
    if(topology_changed || metadata_changed)
    {
        s.m_skip_pt = pt.skip_position_free;
        s.m_skip_ee = ee.skip_position_free;
        s.update_cover(metadata);
    }
    s.prepare_proxies(pt, ee, points, edges, faces, skin_scale, codim);
    if(s.m_use_ef)
    {
        const SizeT actual_chunk = std::min({s.m_owned_vertices.size(),
                                             std::max(SizeT{1}, max_chunk_edges),
                                             PairLimit / s.m_owned_edges.size()});
        s.refresh(tree, pt, ee, node, points, edges, faces, actual_chunk, !topology_changed && !metadata_changed);
        if(codim && cp.aabbs.size())
            s.refresh_direct(tree, pt, ee, node, points, edges, faces, codim, true);
        else
            s.m_stats.stored_pps = s.m_stats.stored_pes = 0;
    }
    else
    {
        s.refresh_direct(tree, pt, ee, node, points, edges, faces, codim, false);
        s.m_cached_chunk_size = 0;
    }
    s.m_tree              = &tree;
    s.m_rebuild_topology  = false;
    s.m_tree_ready        = true;
    s.m_cached_skin_scale = skin_scale;
    s.m_cache_valid       = true;
    ++s.m_stats.refreshes;
    CUDA_TOOL_CHECK(cudaMemsetAsync(
        s.m_control_device.data(), 0, 3 * sizeof(IndexT), default_stream()));
    s.judge(tree, pt, ee, points, edges, faces, pts, ees, false, hook, codim);
    return true;
}
}  // namespace uipc::backend::cuda
