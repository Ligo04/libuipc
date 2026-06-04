#include <linear_system/agipc_coarse_linear_system.h>
#include <uipc/common/timer.h>
#include <muda/cub/device/device_scan.h>
#include <muda/cub/device/device_reduce.h>
#include <muda/ext/eigen/atomic.h>
#include <muda/launch/launch.h>
#include <cub/warp/warp_reduce.cuh>
#include <algorithm>
#include <cmath>

namespace uipc::backend::cuda
{
namespace
{
    constexpr int kAffineBlockCount = 4;
    constexpr int kBlockDof         = 3;

    MUDA_GENERIC int ceil_div_int(int a, int b)
    {
        return (a + b - 1) / b;
    }


    MUDA_INLINE MUDA_DEVICE int find_directed_edge_entry(int        row,
                                                         int        col,
                                                         const int* row_offsets,
                                                         const int* col_indices)
    {
        int begin = row_offsets[row];
        int end   = row_offsets[row + 1];
        while(begin < end)
        {
            int mid       = begin + (end - begin) / 2;
            int mid_value = col_indices[mid];
            if(mid_value < col)
                begin = mid + 1;
            else
                end = mid;
        }
        if(begin < row_offsets[row + 1] && col_indices[begin] == col)
            return begin;
        return -1;
    }

    MUDA_INLINE MUDA_DEVICE void protect_directed_edge(int        row,
                                                       int        col,
                                                       const int* row_offsets,
                                                       const int* col_indices,
                                                       Float*     edge_tags,
                                                       int* protected_flags)
    {
        int entry = find_directed_edge_entry(row, col, row_offsets, col_indices);
        if(entry >= 0)
        {
            edge_tags[entry]       = Float{0};
            protected_flags[entry] = 1;
        }
    }

    MUDA_INLINE MUDA_DEVICE void protect_undirected_edge(int        a,
                                                         int        b,
                                                         const int* row_offsets,
                                                         const int* col_indices,
                                                         Float*     edge_tags,
                                                         int* protected_flags)
    {
        protect_directed_edge(a, b, row_offsets, col_indices, edge_tags, protected_flags);
        protect_directed_edge(b, a, row_offsets, col_indices, edge_tags, protected_flags);
    }

    MUDA_INLINE MUDA_DEVICE Matrix3x3 tet_deformation_gradient(muda::CDense1D<Vector3> xs,
                                                               muda::CDense1D<Vector4i> tets,
                                                               muda::CDense1D<Matrix3x3> dm_invs,
                                                               int tet_id)
    {
        const Vector4i tet = tets(tet_id);
        const Vector3& x0  = xs(tet(0));
        const Vector3& x1  = xs(tet(1));
        const Vector3& x2  = xs(tet(2));
        const Vector3& x3  = xs(tet(3));

        Matrix3x3 Ds;
        Ds.col(0) = x1 - x0;
        Ds.col(1) = x2 - x0;
        Ds.col(2) = x3 - x0;
        return Ds * dm_invs(tet_id);
    }

    MUDA_INLINE MUDA_DEVICE Matrix3x3 green_strain_from_tet(muda::CDense1D<Vector3> xs,
                                                            muda::CDense1D<Vector4i> tets,
                                                            muda::CDense1D<Matrix3x3> dm_invs,
                                                            int tet_id)
    {
        Matrix3x3 F = tet_deformation_gradient(xs, tets, dm_invs, tet_id);
        return Float{0.5} * (F.transpose() * F - Matrix3x3::Identity());
    }

    MUDA_INLINE MUDA_DEVICE Float matrix_frobenius_norm(const Matrix3x3& A)
    {
        return sqrt(A.squaredNorm());
    }

    MUDA_DEVICE int fine_block_begin_for_node(int node_id, int abd_node_count)
    {
        return node_id < abd_node_count ?
                   node_id * kAffineBlockCount :
                   abd_node_count * kAffineBlockCount + (node_id - abd_node_count);
    }

    MUDA_DEVICE int fine_block_count_for_node(int node_id, int abd_node_count)
    {
        return node_id < abd_node_count ? kAffineBlockCount : 1;
    }

    MUDA_DEVICE int node_id_from_fine_block(int fine_block, int abd_node_count, int node_count)
    {
        int abd_block_count = abd_node_count * kAffineBlockCount;
        int node_id         = fine_block < abd_block_count ?
                                  fine_block / kAffineBlockCount :
                                  abd_node_count + (fine_block - abd_block_count);
        return (node_id >= 0 && node_id < node_count) ? node_id : -1;
    }

    MUDA_DEVICE int global_vertex_from_node(muda::CDense1D<IndexT> node_to_global_vertex, int node_id)
    {
        if(node_to_global_vertex.total_size() == 0)
            return node_id;
        return node_to_global_vertex(node_id);
    }

    MUDA_DEVICE int global_vertex_to_agipc_node(int global_vertex,
                                                muda::CDense1D<IndexT> global_vertex_body_ids,
                                                int abd_node_count,
                                                int fem_node_count,
                                                int abd_global_body_offset,
                                                int fem_global_body_offset,
                                                int fem_global_body_count,
                                                int fem_global_vertex_offset)
    {
        if(global_vertex < 0)
            return -1;

        // FEM global vertices have a contiguous reporter-owned vertex range.  Check
        // it before body-id based ABD collapsing: GlobalVertexManager::body_ids() is
        // also valid for FEM vertices, and in mixed ABD/FEM scenes FEM body ids can
        // numerically overlap the ABD body range if offsets are ignored.
        int fem_node = global_vertex - fem_global_vertex_offset;
        if(fem_node >= 0 && fem_node < fem_node_count)
        {
            if(global_vertex_body_ids.total_size() > global_vertex && fem_global_body_count > 0)
            {
                int body_id = static_cast<int>(global_vertex_body_ids(global_vertex));
                int fem_body = body_id - fem_global_body_offset;
                if(fem_body < 0 || fem_body >= fem_global_body_count)
                    return -1;
            }
            return abd_node_count + fem_node;
        }

        // ABD surface reporters emit global vertex ids, while the AGIPC node graph
        // collapses all vertices of the same affine body to that body's 12-DoF node.
        if(global_vertex_body_ids.total_size() > global_vertex)
        {
            int body_id = static_cast<int>(global_vertex_body_ids(global_vertex));
            int abd_node = body_id - abd_global_body_offset;
            if(abd_node >= 0 && abd_node < abd_node_count)
                return abd_node;
        }

        return -1;
    }

    MUDA_DEVICE Vector3 rest_position_for_agipc_node(muda::CDense1D<Vector3> rest_positions,
                                                     muda::CDense1D<IndexT> node_to_global_vertex,
                                                     int abd_node_count,
                                                     int node_id)
    {
        // FEM nodes evaluate affine embedding at their rest position.  ABD nodes
        // already store native q=[p,A] 12-DoF blocks, so they never need a
        // representative vertex for interpolation weights.
        if(node_id < abd_node_count || rest_positions.total_size() == 0)
            return Vector3::Zero();

        int global_vertex = global_vertex_from_node(node_to_global_vertex, node_id);
        if(global_vertex < 0 || global_vertex >= rest_positions.total_size())
            return Vector3::Zero();

        return rest_positions(global_vertex);
    }

    MUDA_DEVICE Float affine_weight(muda::CDense1D<Vector3> rest_positions,
                                    muda::CDense1D<IndexT> node_to_global_vertex,
                                    int            abd_node_count,
                                    int            node_id,
                                    int            affine_part,
                                    const Vector3& centroid)
    {
        // AGIPC affine embedding uses the basis [1, X-X̄, Y-Ȳ, Z-Z̄] evaluated at
        // rest coordinates relative to the aggregate's rest centroid. Centering keeps
        // the 12-DoF block well-conditioned regardless of world position. ABD fine
        // blocks use their native 12-DoF q parts; FEM nodes in a 12-DoF aggregate use
        // this scalar basis.
        if(affine_part == 0)
            return Float{1};

        Vector3 rest = rest_position_for_agipc_node(
            rest_positions, node_to_global_vertex, abd_node_count, node_id);
        Vector3 local = rest - centroid;
        if(affine_part == 1)
            return local.x();
        if(affine_part == 2)
            return local.y();
        return local.z();
    }

    // Rest centroid of the coarse node that `node_id` belongs to. Used to center
    // the affine embedding basis. Returns zero when no centroid data is available
    // (e.g. ABD-only nodes), which reduces to the uncentered basis.
    MUDA_DEVICE Vector3 coarse_centroid_for_node(int node_id,
                                                 muda::CDense1D<int> node_to_coarse_node,
                                                 muda::CDense1D<Vector3> coarse_rest_centroid)
    {
        if(node_to_coarse_node.total_size() == 0 || coarse_rest_centroid.total_size() == 0)
            return Vector3::Zero();
        if(node_id < 0 || node_id >= node_to_coarse_node.total_size())
            return Vector3::Zero();
        int c = node_to_coarse_node(node_id);
        if(c < 0 || c >= coarse_rest_centroid.total_size())
            return Vector3::Zero();
        return coarse_rest_centroid(c);
    }

    MUDA_DEVICE void count_adjacency_edges_kernel(int t,
                                                  int tet_count,
                                                  muda::CDense1D<Vector4i> tets,
                                                  muda::CDense1D<Vector2i> surface_edges,
                                                  muda::CDense1D<Vector3i> surface_triangles,
                                                  muda::CDense1D<IndexT> global_vertex_body_ids,
                                                  int  node_count,
                                                  int  abd_node_count,
                                                  int  fem_node_count,
                                                  int  abd_global_body_offset,
                                                  int  fem_global_body_offset,
                                                  int  fem_global_body_count,
                                                  int  fem_global_vertex_offset,
                                                  int* edge_counts)
    {
        int surf_edge_count = surface_edges.total_size();
        int surf_tri_count  = surface_triangles.total_size();
        int total_count     = tet_count + surf_edge_count + surf_tri_count;
        if(t >= total_count)
            return;

        auto add_directed = [&](int row_node, int col_node)
        {
            if(row_node < 0 || col_node < 0 || row_node == col_node)
                return;
            if(row_node >= node_count || col_node >= node_count)
                return;
            muda::atomic_add(&edge_counts[row_node], 1);
            muda::atomic_add(&edge_counts[col_node], 1);
        };

        // FEM elastic adjacency comes from tetrahedron edges (the elastic stencil),
        // which is INVARIANT mesh topology — mirroring the reference's mesh-derived
        // CSR. (Previously this used the per-iteration fine_A Hessian, which also
        // carries contact coupling and forced a full rebuild every Newton step.)
        if(t < tet_count)
        {
            Vector4i tet  = tets(t);
            int      n[4] = {abd_node_count + tet[0],
                             abd_node_count + tet[1],
                             abd_node_count + tet[2],
                             abd_node_count + tet[3]};
            add_directed(n[0], n[1]);
            add_directed(n[0], n[2]);
            add_directed(n[0], n[3]);
            add_directed(n[1], n[2]);
            add_directed(n[1], n[3]);
            add_directed(n[2], n[3]);
            return;
        }

        auto vertex_to_node = [&](int global_vertex) -> int
        {
            return global_vertex_to_agipc_node(global_vertex,
                                               global_vertex_body_ids,
                                               abd_node_count,
                                               fem_node_count,
                                               abd_global_body_offset,
                                               fem_global_body_offset,
                                               fem_global_body_count,
                                               fem_global_vertex_offset);
        };

        int surface_index = t - tet_count;
        if(surface_index < surf_edge_count)
        {
            Vector2i e = surface_edges(surface_index);
            add_directed(vertex_to_node(e[0]), vertex_to_node(e[1]));
            return;
        }

        Vector3i tri = surface_triangles(surface_index - surf_edge_count);
        int      a   = vertex_to_node(tri[0]);
        int      b   = vertex_to_node(tri[1]);
        int      c   = vertex_to_node(tri[2]);
        add_directed(a, b);
        add_directed(a, c);
        add_directed(b, c);
    }

    MUDA_DEVICE void protect_contact_edge_tags_kernel(int triplet_id,
                                                      muda::CBCOOMatrixViewer<Float, 3, 3> contact_hessian,
                                                      muda::CDense1D<IndexT> global_vertex_body_ids,
                                                      int node_count,
                                                      int abd_node_count,
                                                      int fem_node_count,
                                                      int abd_global_body_offset,
                                                      int fem_global_body_offset,
                                                      int fem_global_body_count,
                                                      int fem_global_vertex_offset,
                                                      const int* row_offsets,
                                                      const int* col_indices,
                                                      Float*     edge_tags,
                                                      int* protected_flags)
    {
        auto triplet  = contact_hessian(triplet_id);
        int  row_node = global_vertex_to_agipc_node(triplet.row_index,
                                                   global_vertex_body_ids,
                                                   abd_node_count,
                                                   fem_node_count,
                                                   abd_global_body_offset,
                                                   fem_global_body_offset,
                                                   fem_global_body_count,
                                                   fem_global_vertex_offset);
        int  col_node = global_vertex_to_agipc_node(triplet.col_index,
                                                   global_vertex_body_ids,
                                                   abd_node_count,
                                                   fem_node_count,
                                                   abd_global_body_offset,
                                                   fem_global_body_offset,
                                                   fem_global_body_count,
                                                   fem_global_vertex_offset);
        if(row_node < 0 || col_node < 0 || row_node == col_node)
            return;

        // DyTopo contact Hessian rows/cols are global surface vertices.  Convert
        // exactly those endpoints to AGIPC nodes and protect only that algebraic
        // edge; non-contact neighbours stay collapsible.
        protect_undirected_edge(row_node, col_node, row_offsets, col_indices, edge_tags, protected_flags);
    }


    MUDA_DEVICE void fill_adjacency_edges_kernel(int t,
                                                 int tet_count,
                                                 muda::CDense1D<Vector4i> tets,
                                                 muda::CDense1D<Vector2i> surface_edges,
                                                 muda::CDense1D<Vector3i> surface_triangles,
                                                 muda::CDense1D<IndexT> global_vertex_body_ids,
                                                 int node_count,
                                                 int abd_node_count,
                                                 int fem_node_count,
                                                 int abd_global_body_offset,
                                                 int fem_global_body_offset,
                                                 int fem_global_body_count,
                                                 int fem_global_vertex_offset,
                                                 int force_protected,
                                                 const int* row_offsets,
                                                 int*       fill_offsets,
                                                 int*       col_indices,
                                                 Float*     edge_tags,
                                                 int*       protected_flags)
    {
        int surf_edge_count = surface_edges.total_size();
        int surf_tri_count  = surface_triangles.total_size();
        int total_count     = tet_count + surf_edge_count + surf_tri_count;
        if(t >= total_count)
            return;

        auto vertex_to_node = [&](int global_vertex) -> int
        {
            return global_vertex_to_agipc_node(global_vertex,
                                               global_vertex_body_ids,
                                               abd_node_count,
                                               fem_node_count,
                                               abd_global_body_offset,
                                               fem_global_body_offset,
                                               fem_global_body_count,
                                               fem_global_vertex_offset);
        };

        auto write_pair = [&](int row_node, int col_node)
        {
            if(row_node < 0 || col_node < 0 || row_node == col_node)
                return;
            if(row_node >= node_count || col_node >= node_count)
                return;

            // Start from the paper's first-Newton/history protection only.  Contact and
            // strain protection are applied later as edge-exact passes over contact
            // Hessian entries and tet boundary edges, so ABD/FEM graph edges can still
            // participate in global connected-component aggregation when safe.
            bool structural_protected = force_protected != 0;

            int row_slot       = muda::atomic_add(&fill_offsets[row_node], 1);
            int row_e          = row_offsets[row_node] + row_slot;
            col_indices[row_e] = col_node;
            edge_tags[row_e]   = structural_protected ? Float{0} : Float{1};
            protected_flags[row_e] = structural_protected ? 1 : 0;

            int col_slot       = muda::atomic_add(&fill_offsets[col_node], 1);
            int col_e          = row_offsets[col_node] + col_slot;
            col_indices[col_e] = row_node;
            edge_tags[col_e]   = structural_protected ? Float{0} : Float{1};
            protected_flags[col_e] = structural_protected ? 1 : 0;
        };

        // FEM tet edges (invariant elastic stencil) — see count_adjacency_edges_kernel.
        if(t < tet_count)
        {
            Vector4i tet  = tets(t);
            int      n[4] = {abd_node_count + tet[0],
                             abd_node_count + tet[1],
                             abd_node_count + tet[2],
                             abd_node_count + tet[3]};
            write_pair(n[0], n[1]);
            write_pair(n[0], n[2]);
            write_pair(n[0], n[3]);
            write_pair(n[1], n[2]);
            write_pair(n[1], n[3]);
            write_pair(n[2], n[3]);
            return;
        }

        int surface_index = t - tet_count;
        if(surface_index < surf_edge_count)
        {
            Vector2i e = surface_edges(surface_index);
            write_pair(vertex_to_node(e[0]), vertex_to_node(e[1]));
            return;
        }

        Vector3i tri = surface_triangles(surface_index - surf_edge_count);
        int      a   = vertex_to_node(tri[0]);
        int      b   = vertex_to_node(tri[1]);
        int      c   = vertex_to_node(tri[2]);
        write_pair(a, b);
        write_pair(a, c);
        write_pair(b, c);
    }

    // Reset all edge tags to the per-iteration base state on a cached CSR
    // structure: collapsible (tag=1, unprotected) normally, or fully protected
    // (tag=0) on the first Newton step / history reset. Contact and Green-strain
    // protection passes then run on top, exactly as in the full-rebuild path.
    MUDA_DEVICE void reset_edge_tags_kernel(int e, int force_protected, Float* edge_tags, int* protected_flags)
    {
        edge_tags[e]       = force_protected ? Float{0} : Float{1};
        protected_flags[e] = force_protected ? 1 : 0;
    }

    MUDA_DEVICE void sort_and_unique_adjacency_rows_kernel(int row,
                                                           const int* row_offsets,
                                                           int*   col_indices,
                                                           Float* edge_tags,
                                                           int* protected_flags,
                                                           int* unique_edge_counts)
    {
        int begin = row_offsets[row];
        int end   = row_offsets[row + 1];

        // 每一行度数通常很小；用行内插入排序保持 CSR col_indices 有序，匹配 AGIPC 的二分查找保护边语义。
        for(int i = begin + 1; i < end; ++i)
        {
            int   c  = col_indices[i];
            Float tg = edge_tags[i];
            int   pf = protected_flags[i];
            int   j  = i - 1;
            while(j >= begin && col_indices[j] > c)
            {
                col_indices[j + 1]     = col_indices[j];
                edge_tags[j + 1]       = edge_tags[j];
                protected_flags[j + 1] = protected_flags[j];
                --j;
            }
            col_indices[j + 1]     = c;
            edge_tags[j + 1]       = tg;
            protected_flags[j + 1] = pf;
        }

        int unique_count = 0;
        int last_col     = -1;
        int out          = begin;
        for(int i = begin; i < end; ++i)
        {
            int c = col_indices[i];
            if(c == last_col)
            {
                int prev        = out - 1;
                edge_tags[prev] = min(edge_tags[prev], edge_tags[i]);
                protected_flags[prev] = max(protected_flags[prev], protected_flags[i]);
                continue;
            }

            col_indices[out]     = c;
            edge_tags[out]       = edge_tags[i];
            protected_flags[out] = protected_flags[i];
            last_col             = c;
            ++out;
            ++unique_count;
        }

        for(int i = out; i < end; ++i)
        {
            col_indices[i]     = -1;
            edge_tags[i]       = Float{0};
            protected_flags[i] = 1;
        }
        unique_edge_counts[row] = unique_count;
    }

    MUDA_DEVICE void compute_tet_green_strain_tags_kernel(int tet_id,
                                                          muda::CDense1D<Vector4i> tets,
                                                          muda::CDense1D<Matrix3x3> dm_invs,
                                                          muda::CDense1D<Vector3> xs,
                                                          muda::CDense1D<Vector3> rest_xs,
                                                          int abd_node_count,
                                                          int fem_node_count,
                                                          int force_protected,
                                                          int has_history,
                                                          Float green_strain_tau,
                                                          const Matrix3x3* previous_green,
                                                          Matrix3x3* current_green,
                                                          const int* row_offsets,
                                                          const int* col_indices,
                                                          Float* edge_tags,
                                                          int* protected_flags)
    {
        if(tet_id >= dm_invs.total_size())
            return;

        Matrix3x3 current = green_strain_from_tet(xs, tets, dm_invs, tet_id);
        current_green[tet_id] = current;

        Vector4i tet      = tets(tet_id);
        int      nodes[4] = {abd_node_count + tet(0),
                             abd_node_count + tet(1),
                             abd_node_count + tet(2),
                             abd_node_count + tet(3)};
        for(int i = 0; i < 4; ++i)
        {
            if(nodes[i] < abd_node_count || nodes[i] >= abd_node_count + fem_node_count)
                return;
        }

        if(force_protected != 0)
        {
            protect_undirected_edge(nodes[0], nodes[1], row_offsets, col_indices, edge_tags, protected_flags);
            protect_undirected_edge(nodes[0], nodes[2], row_offsets, col_indices, edge_tags, protected_flags);
            protect_undirected_edge(nodes[0], nodes[3], row_offsets, col_indices, edge_tags, protected_flags);
            protect_undirected_edge(nodes[1], nodes[2], row_offsets, col_indices, edge_tags, protected_flags);
            protect_undirected_edge(nodes[1], nodes[3], row_offsets, col_indices, edge_tags, protected_flags);
            protect_undirected_edge(nodes[2], nodes[3], row_offsets, col_indices, edge_tags, protected_flags);
            return;
        }

        if(has_history == 0)
            return;

        Matrix3x3 previous = previous_green[tet_id];
        Matrix3x3 dG       = current - previous;

        // Paper / reference criterion (method.md L7-8, edge_tag.cu:105-119): the
        // Green-strain increment is measured by the Frobenius norm of the FULL
        // tensor change ||dG||_F, and protection is all-or-nothing per tet. The
        // default tau=5e-5 is calibrated for this norm. (A per-edge directional
        // projection |dir.dG.dir| <= ||dG||_F would systematically under-protect
        // and discard shear/transverse components.)
        Float delta_norm = matrix_frobenius_norm(dG);
        (void)rest_xs;  // material edge directions not needed for the Frobenius metric
        if(delta_norm <= green_strain_tau)
            return;

        protect_undirected_edge(nodes[0], nodes[1], row_offsets, col_indices, edge_tags, protected_flags);
        protect_undirected_edge(nodes[0], nodes[2], row_offsets, col_indices, edge_tags, protected_flags);
        protect_undirected_edge(nodes[0], nodes[3], row_offsets, col_indices, edge_tags, protected_flags);
        protect_undirected_edge(nodes[1], nodes[2], row_offsets, col_indices, edge_tags, protected_flags);
        protect_undirected_edge(nodes[1], nodes[3], row_offsets, col_indices, edge_tags, protected_flags);
        protect_undirected_edge(nodes[2], nodes[3], row_offsets, col_indices, edge_tags, protected_flags);
    }

    // Read-only find. The union kernel only ever sets parent[high]=low with
    // low<high, so every non-root satisfies parent[x]<x and a valid chain is
    // strictly decreasing. We must NOT do in-place path compression here:
    // concurrent threads writing a stale (larger) root can break the
    // strictly-decreasing invariant and create a cycle, making this loop spin
    // forever on the GPU. Path shortening is handled separately and atomically
    // by compress_union_parent_kernel between union passes. The `next < root`
    // guard is a defensive stop against any transient non-monotone read.
    MUDA_DEVICE int find_union_root(const int* parent, int node)
    {
        int root = node;
        while(true)
        {
            int next = parent[root];
            if(next == root || next < 0)
                break;
            if(next >= root)  // invariant violated by a racing write: stop safely
                break;
            root = next;
        }
        return root;
    }

    MUDA_DEVICE void init_union_parent_kernel(int node_id, int* parent, int* size)
    {
        parent[node_id] = node_id;
        size[node_id]   = 1;
    }

    MUDA_DEVICE void union_collapsible_edges_kernel(int node_id,
                                                    int node_count,
                                                    int max_aggregate_size,
                                                    const int*   row_offsets,
                                                    const int*   col_indices,
                                                    const Float* edge_tags,
                                                    int*         parent,
                                                    int*         size)
    {
        if(node_id >= node_count)
            return;

        for(int e = row_offsets[node_id]; e < row_offsets[node_id + 1]; ++e)
        {
            int neighbor = col_indices[e];
            if(neighbor < 0 || neighbor >= node_count || neighbor == node_id)
                continue;
            if(edge_tags && edge_tags[e] == Float{0})
                continue;

            int a = node_id;
            int b = neighbor;
            while(true)
            {
                int root_a = find_union_root(parent, a);
                int root_b = find_union_root(parent, b);
                if(root_a == root_b)
                    break;

                // Bounded aggregation: the paper/reference aggregate within
                // bounded warp groups, so a supernode never spans an unbounded
                // region. Skip the merge if the combined (approximate) size would
                // exceed the cap. The size read is racy under concurrency, but a
                // soft cap is enough to prevent mesh-spanning aggregates that the
                // 3/12-DoF embedding cannot represent.
                if(max_aggregate_size > 0 && size[root_a] + size[root_b] > max_aggregate_size)
                    break;

                int high      = max(root_a, root_b);
                int low       = min(root_a, root_b);
                int high_size = size[high];  // snapshot before the merge
                int old       = muda::atomic_cas(&parent[high], high, low);
                if(old == high)
                {
                    // We won the merge. Fold high's size into low's CURRENT root
                    // (low may itself have been merged under a smaller root since
                    // the find above), so the size stays attached to a real root
                    // and the cap check on later passes reads an accurate total.
                    int r = find_union_root(parent, low);
                    muda::atomic_add(&size[r], high_size);
                    break;
                }
                if(old == low)
                    break;
                a = old;
                b = low;
            }
        }
    }

    MUDA_DEVICE void compress_union_parent_kernel(int node_id, int* parent)
    {
        parent[node_id] = find_union_root(parent, node_id);
    }

    MUDA_DEVICE void mark_component_roots_kernel(int node_id, const int* parent, int* component_flags)
    {
        component_flags[node_id] = parent[node_id] == node_id ? 1 : 0;
    }

    MUDA_DEVICE void build_component_map_kernel(int        node_id,
                                                const int* parent,
                                                const int* component_offsets,
                                                int*       node_to_coarse_node)
    {
        int root                     = parent[node_id];
        node_to_coarse_node[node_id] = component_offsets[root];
    }

    MUDA_DEVICE void count_nodes_per_coarse_node_kernel(int node_id,
                                                        int abd_node_count,
                                                        const int* node_to_coarse_node,
                                                        int* coarse_node_counts,
                                                        int* coarse_node_contains_abd)
    {
        int coarse_node = node_to_coarse_node[node_id];
        muda::atomic_add(&coarse_node_counts[coarse_node], 1);
        if(node_id < abd_node_count)
            muda::atomic_exch(&coarse_node_contains_abd[coarse_node], 1);
    }

    // Accumulate each FEM node's rest position into its coarse node's centroid sum,
    // and count FEM members per coarse node. ABD nodes are skipped (they use native
    // 12-DoF q blocks, not the scalar affine basis).
    MUDA_DEVICE void accumulate_coarse_centroid_kernel(int node_id,
                                                       int abd_node_count,
                                                       muda::CDense1D<Vector3> rest_positions,
                                                       muda::CDense1D<IndexT> node_to_global_vertex,
                                                       const int* node_to_coarse_node,
                                                       Vector3* coarse_rest_sum,
                                                       int* coarse_fem_member_count)
    {
        if(node_id < abd_node_count)
            return;
        Vector3 rest = rest_position_for_agipc_node(
            rest_positions, node_to_global_vertex, abd_node_count, node_id);
        int coarse_node = node_to_coarse_node[node_id];
        muda::atomic_add(&coarse_rest_sum[coarse_node].x(), rest.x());
        muda::atomic_add(&coarse_rest_sum[coarse_node].y(), rest.y());
        muda::atomic_add(&coarse_rest_sum[coarse_node].z(), rest.z());
        muda::atomic_add(&coarse_fem_member_count[coarse_node], 1);
    }

    MUDA_DEVICE void finalize_coarse_centroid_kernel(int coarse_node,
                                                     const int* coarse_fem_member_count,
                                                     Vector3* coarse_rest_centroid)
    {
        int n = coarse_fem_member_count[coarse_node];
        if(n > 0)
            coarse_rest_centroid[coarse_node] /= static_cast<Float>(n);
        else
            coarse_rest_centroid[coarse_node] = Vector3::Zero();
    }

    MUDA_DEVICE void decide_coarse_dof_kernel(int        coarse_node,
                                              const int* coarse_node_counts,
                                              const int* coarse_node_contains_abd,
                                              int* coarse_dof,
                                              int* coarse_block_counts,
                                              int  affine_threshold,
                                              int* affine_node_count)
    {
        // `coarse_node` is a compact aggregate id, not the original node id.  Decide
        // ABD/native affine blocks from aggregate membership instead of id ordering.
        bool contains_abd = coarse_node_contains_abd[coarse_node] != 0;
        bool use_affine = contains_abd || coarse_node_counts[coarse_node] > affine_threshold;

        coarse_dof[coarse_node]          = use_affine ? 12 : 3;
        coarse_block_counts[coarse_node] = use_affine ? 4 : 1;
        if(use_affine)
            muda::atomic_add(affine_node_count, 1);
    }

    MUDA_DEVICE void build_node_to_coarse_block_kernel(int node_id,
                                                       const int* node_to_coarse_node,
                                                       const int* coarse_block_offsets,
                                                       const int* coarse_block_counts,
                                                       int* node_to_coarse_block_begin,
                                                       int* node_to_coarse_block_count)
    {
        int coarse_node                     = node_to_coarse_node[node_id];
        node_to_coarse_block_begin[node_id] = coarse_block_offsets[coarse_node];
        node_to_coarse_block_count[node_id] = coarse_block_counts[coarse_node];
    }

    // ABD fine blocks map identity 1:1 to their native 12-DoF coarse blocks
    // (q=[p,A0,A1,A2] -> 4 coarse blocks, one per fine part). Only FEM fine
    // blocks may fan out to a 4-block affine aggregate, so ABD never produces the
    // 4x4 (15/16 zero) triplet expansion. Mirrors reference coarse_triplet.cu.
    MUDA_DEVICE void block_mapping_for_fine_block(int fine_block,
                                                  int abd_node_count,
                                                  int node_count,
                                                  const int* node_to_coarse_block_begin,
                                                  const int* node_to_coarse_block_count,
                                                  int& coarse_block_begin,
                                                  int& coarse_block_count,
                                                  int& node_id)
    {
        int abd_block_count = abd_node_count * kAffineBlockCount;
        if(fine_block < abd_block_count)
        {
            node_id = fine_block / kAffineBlockCount;
            coarse_block_begin =
                node_to_coarse_block_begin[node_id] + fine_block % kAffineBlockCount;
            coarse_block_count = 1;
            return;
        }

        node_id = abd_node_count + (fine_block - abd_block_count);
        if(node_id < 0 || node_id >= node_count)
        {
            coarse_block_begin = fine_block;
            coarse_block_count = 1;
            node_id            = -1;
            return;
        }

        coarse_block_begin = node_to_coarse_block_begin[node_id];
        coarse_block_count = node_to_coarse_block_count[node_id];
    }

    MUDA_DEVICE bool same_coarse_block_range(int begin_a, int count_a, int begin_b, int count_b)
    {
        return begin_a == begin_b && count_a == count_b;
    }

    MUDA_DEVICE void count_coarse_triplet_blocks_kernel(int triplet_id,
                                                        muda::CBCOOMatrixViewer<Float, 3, 3> fine_A,
                                                        const int* node_to_coarse_block_begin,
                                                        const int* node_to_coarse_block_count,
                                                        int  abd_node_count,
                                                        int  node_count,
                                                        int* block_counts)
    {
        auto triplet = fine_A(triplet_id);
        int row_node = node_id_from_fine_block(triplet.row_index, abd_node_count, node_count);
        int col_node = node_id_from_fine_block(triplet.col_index, abd_node_count, node_count);
        if(row_node < 0 || col_node < 0)
        {
            block_counts[triplet_id] = 0;
            return;
        }

        int row_begin, row_count, rnode;
        int col_begin, col_count, cnode;
        block_mapping_for_fine_block(triplet.row_index,
                                     abd_node_count,
                                     node_count,
                                     node_to_coarse_block_begin,
                                     node_to_coarse_block_count,
                                     row_begin,
                                     row_count,
                                     rnode);
        block_mapping_for_fine_block(triplet.col_index,
                                     abd_node_count,
                                     node_count,
                                     node_to_coarse_block_begin,
                                     node_to_coarse_block_count,
                                     col_begin,
                                     col_count,
                                     cnode);

        // When an off-diagonal fine block (row<col) collapses into the SAME coarse
        // block range, its contribution and its unstored transpose both land in
        // that range; emit only the upper triangle (cp>=rp) with B+B^T folded in.
        int count = row_count * col_count;
        if(triplet.row_index != triplet.col_index
           && same_coarse_block_range(row_begin, row_count, col_begin, col_count))
            count = row_count * (row_count + 1) / 2;
        block_counts[triplet_id] = count;
    }

    MUDA_DEVICE void build_coarse_triplets_kernel(
        int                                  triplet_id,
        muda::CBCOOMatrixViewer<Float, 3, 3> fine_A,
        const int*                           block_offsets,
        const int*                           node_to_coarse_block_begin,
        const int*                           node_to_coarse_block_count,
        int                                  abd_node_count,
        int                                  node_count,
        muda::CDense1D<Vector3>              rest_positions,
        muda::CDense1D<IndexT>               node_to_global_vertex,
        muda::CDense1D<int>                  node_to_coarse_node,
        muda::CDense1D<Vector3>              coarse_rest_centroid,
        muda::TripletMatrixViewer<Float, 3>  coarse_A)
    {
        auto triplet = fine_A(triplet_id);

        // Match count_coarse_triplet_blocks_kernel: invalid endpoints contribute
        // zero coarse blocks, so emit nothing here (block_offsets reserved 0).
        int row_node_check =
            node_id_from_fine_block(triplet.row_index, abd_node_count, node_count);
        int col_node_check =
            node_id_from_fine_block(triplet.col_index, abd_node_count, node_count);
        if(row_node_check < 0 || col_node_check < 0)
            return;

        int row_begin, row_count, row_node;
        int col_begin, col_count, col_node;
        block_mapping_for_fine_block(triplet.row_index,
                                     abd_node_count,
                                     node_count,
                                     node_to_coarse_block_begin,
                                     node_to_coarse_block_count,
                                     row_begin,
                                     row_count,
                                     row_node);
        block_mapping_for_fine_block(triplet.col_index,
                                     abd_node_count,
                                     node_count,
                                     node_to_coarse_block_begin,
                                     node_to_coarse_block_count,
                                     col_begin,
                                     col_count,
                                     col_node);

        Matrix3x3 B        = triplet.value;
        int       out_base = block_offsets[triplet_id];
        int       out_id   = out_base;

        auto rweight = [&](int node, int count, int part) -> Float
        {
            if(count != kAffineBlockCount)
                return Float{1};
            Vector3 c = coarse_centroid_for_node(node, node_to_coarse_node, coarse_rest_centroid);
            return affine_weight(
                rest_positions, node_to_global_vertex, abd_node_count, node, part, c);
        };
        auto write_canonical = [&](int oid, int oi, int oj, const Matrix3x3& v)
        {
            if(oi <= oj)
                coarse_A(oid).write(oi, oj, v);
            else
                coarse_A(oid).write(oj, oi, v.transpose());
        };

        // fine_A is upper-triangular (ge2sym). When an off-diagonal fine block
        // (row<col) collapses into the SAME coarse block range, both it and its
        // unstored transpose B^T land in that range. Emit the upper triangle
        // (cp>=rp) with the fully symmetrized Galerkin contribution
        // B*(w_rp*w_cq) + B^T*(w_cp*w_rq); otherwise the coarse operator would be
        // asymmetric with halved coupling. Mirrors reference coarse_triplet.cu.
        if(triplet.row_index != triplet.col_index
           && same_coarse_block_range(row_begin, row_count, col_begin, col_count))
        {
            Matrix3x3 BT = B.transpose();
            for(int rp = 0; rp < row_count; ++rp)
            {
                Float row_w_p = rweight(row_node, row_count, rp);
                Float col_w_p = rweight(col_node, col_count, rp);
                for(int cp = rp; cp < col_count; ++cp)
                {
                    Float     row_w_q = rweight(row_node, row_count, cp);
                    Float     col_w_q = rweight(col_node, col_count, cp);
                    int       oi      = row_begin + rp;
                    int       oj      = col_begin + cp;
                    Matrix3x3 v =
                        (B * (row_w_p * col_w_q) + BT * (col_w_p * row_w_q)).eval();
                    // oi<=oj always holds here (cp>=rp, same range), so write directly.
                    coarse_A(out_id).write(oi, oj, v);
                    ++out_id;
                }
            }
            return;
        }

        for(int rp = 0; rp < row_count; ++rp)
        {
            Float row_weight = rweight(row_node, row_count, rp);
            for(int cp = 0; cp < col_count; ++cp)
            {
                Float     col_weight = rweight(col_node, col_count, cp);
                int       oi         = row_begin + rp;
                int       oj         = col_begin + cp;
                Matrix3x3 v          = (B * (row_weight * col_weight)).eval();
                write_canonical(out_id++, oi, oj, v);
            }
        }
    }

    MUDA_DEVICE void restrict_rhs_kernel(int node_id,
                                         int abd_node_count,
                                         muda::CDenseVectorViewer<Float> fine_b,
                                         muda::CDense1D<Vector3> rest_positions,
                                         muda::CDense1D<IndexT> node_to_global_vertex,
                                         const Vector3& centroid,
                                         muda::DenseVectorViewer<Float> coarse_b,
                                         const int* node_to_coarse_block_begin,
                                         const int* node_to_coarse_block_count)
    {
        int fine_begin   = fine_block_begin_for_node(node_id, abd_node_count);
        int fine_count   = fine_block_count_for_node(node_id, abd_node_count);
        int coarse_begin = node_to_coarse_block_begin[node_id];
        int coarse_count = node_to_coarse_block_count[node_id];

        // Restriction is U^T b.  Multiple ABD bodies or FEM vertices may now share
        // one affine coarse node, so every contribution is accumulated atomically.
        for(int b = 0; b < fine_count; ++b)
        {
            Vector3 fine = fine_b.segment((fine_begin + b) * 3, 3).as_eigen();
            for(int p = 0; p < coarse_count; ++p)
            {
                Float w = Float{0};
                if(node_id < abd_node_count)
                {
                    // ABD q=[p,A0,A1,A2] already uses four native 3-vector parts.
                    w = p == b ? Float{1} : Float{0};
                }
                else
                {
                    w = coarse_count == kAffineBlockCount ?
                            affine_weight(rest_positions,
                                          node_to_global_vertex,
                                          abd_node_count,
                                          node_id,
                                          p,
                                          centroid) :
                            Float{1};
                }
                if(w != Float{0})
                    coarse_b.segment((coarse_begin + p) * 3, 3)
                        .atomic_add((w * fine).eval());
            }
        }
    }

    MUDA_DEVICE void prolongate_kernel(int node_id,
                                       int abd_node_count,
                                       muda::DenseVectorViewer<Float>  fine_x,
                                       muda::CDenseVectorViewer<Float> coarse_x,
                                       muda::CDense1D<Vector3> rest_positions,
                                       muda::CDense1D<IndexT> node_to_global_vertex,
                                       const Vector3& centroid,
                                       const int* node_to_coarse_block_begin,
                                       const int* node_to_coarse_block_count)
    {
        int fine_begin   = fine_block_begin_for_node(node_id, abd_node_count);
        int fine_count   = fine_block_count_for_node(node_id, abd_node_count);
        int coarse_begin = node_to_coarse_block_begin[node_id];
        int coarse_count = node_to_coarse_block_count[node_id];

        for(int b = 0; b < fine_count; ++b)
        {
            Vector3 value = Vector3::Zero();
            for(int p = 0; p < coarse_count; ++p)
            {
                Float w = Float{0};
                if(node_id < abd_node_count)
                {
                    w = p == b ? Float{1} : Float{0};
                }
                else
                {
                    w = coarse_count == kAffineBlockCount ?
                            affine_weight(rest_positions,
                                          node_to_global_vertex,
                                          abd_node_count,
                                          node_id,
                                          p,
                                          centroid) :
                            Float{1};
                }
                if(w != Float{0})
                    value += w * coarse_x.segment((coarse_begin + p) * 3, 3).as_eigen();
            }
            fine_x.segment((fine_begin + b) * 3, 3).as_eigen() = value;
        }
    }

    MUDA_DEVICE void sum_int_kernel(int i, const int* values, int* result)
    {
        muda::atomic_add(result, values[i]);
    }

    MUDA_DEVICE void compact_adjacency_rows_kernel(int        row,
                                                   const int* row_offsets,
                                                   const int* unique_row_offsets,
                                                   const int*   unique_counts,
                                                   const int*   col_indices,
                                                   const Float* edge_tags,
                                                   const int*   protected_flags,
                                                   int*   unique_col_indices,
                                                   Float* unique_edge_tags,
                                                   int* unique_protected_flags)
    {
        int src_begin = row_offsets[row];
        int dst_begin = unique_row_offsets[row];
        int count     = unique_counts[row];
        for(int i = 0; i < count; ++i)
        {
            int src                     = src_begin + i;
            int dst                     = dst_begin + i;
            unique_col_indices[dst]     = col_indices[src];
            unique_edge_tags[dst]       = edge_tags[src];
            unique_protected_flags[dst] = protected_flags[src];
        }
    }
}  // namespace

void AGIPCCoarseLinearSystem::build(const BuildInfo& info, const Config& config)
{
    Timer timer{"AGIPC Coarse Linear System Build"};
    UIPC_ASSERT(config.block_size > 0,
                "AGIPC coarse block_size must be positive, got {}.",
                config.block_size);

    m_built                 = false;
    m_rest_positions        = info.rest_positions;
    m_node_to_global_vertex = info.node_to_global_vertex;
    if(info.abd_node_count + info.fem_node_count == 0 || info.fine_A.triplet_count() == 0)
        return;

    m_abd_node_count = info.abd_node_count;
    m_node_count     = info.abd_node_count + info.fem_node_count;
    UIPC_ASSERT(info.node_to_global_vertex.size() == m_node_count,
                "AGIPC node/global-vertex map size mismatch: map={}, nodes={}",
                info.node_to_global_vertex.size(),
                m_node_count);

    build_adjacency(info, config);
    build_mapping(config);
    build_dofs(config);
    build_triplets(info, config);
    build_rhs(info, config);

    m_coarse_x.resize(m_coarse_block_count * kBlockDof);
    m_coarse_x.fill(0);
    m_built = m_coarse_block_count > 0 && m_coarse_A.triplet_count() > 0;

    logger::info("AGIPC coarse system: nodes={}, abd_nodes={}, edges={}, protected_edges={}, coarse_nodes={}, affine_nodes={}, coarse_blocks={}, temp_triplets={}, unique_triplets={}",
                 m_node_count,
                 m_abd_node_count,
                 m_edge_count,
                 m_protected_edge_count,
                 m_coarse_node_count,
                 m_affine_node_count,
                 m_coarse_block_count,
                 m_temporary_triplet_count,
                 m_coarse_A.triplet_count());
}

void AGIPCCoarseLinearSystem::build_adjacency(const BuildInfo& info, const Config& config)
{
    auto node_count     = static_cast<int>(m_node_count);
    auto abd_node_count = static_cast<int>(info.abd_node_count);
    auto block_size     = static_cast<int>(config.block_size);
    auto tet_count      = static_cast<int>(info.fem_tets.size());

    // Per-iteration first-step protection (paper: first Newton step / history
    // reset is fully protected). Needed by both the full-build and cached paths.
    bool has_tet_green_data =
        info.fem_tets.size() > 0 && info.fem_Dm_invs.size() == info.fem_tets.size()
        && info.fem_current_positions.size() > 0
        && info.fem_rest_positions.size() == info.fem_current_positions.size();
    bool has_compatible_tet_history = m_has_previous_tet_green_strains
                                      && m_previous_tet_count == info.fem_tets.size();
    bool force_protected = info.newton_iter == 0
                           || (has_tet_green_data && !has_compatible_tet_history);

    // Adjacency topology cache: the graph is built from invariant mesh topology,
    // so its CSR structure is reusable while the signature sizes are unchanged.
    bool cache_valid = m_adjacency_cached && m_cached_node_count == m_node_count
                       && m_cached_tet_count == static_cast<SizeT>(tet_count)
                       && m_cached_surf_edge_count == info.surface_edges.size()
                       && m_cached_surf_tri_count == info.surface_triangles.size()
                       && m_edge_count > 0;

    if(cache_valid)
    {
        // Reuse cached CSR structure; only reset tags then re-apply protection.
        muda::ParallelFor(block_size)
            .file_line(__FILE__, __LINE__)
            .apply(static_cast<int>(m_edge_count),
                   [force_protected = force_protected ? 1 : 0,
                    edge_tags       = m_edge_tags.data(),
                    protected_flags = m_protected_flags.data()] MUDA_DEVICE(int e) mutable {
                       reset_edge_tags_kernel(e, force_protected, edge_tags, protected_flags);
                   });
    }
    else
    {
        build_adjacency_structure(info, config, force_protected);
        m_adjacency_cached       = true;
        m_cached_node_count      = m_node_count;
        m_cached_tet_count       = static_cast<SizeT>(tet_count);
        m_cached_surf_edge_count = info.surface_edges.size();
        m_cached_surf_tri_count  = info.surface_triangles.size();
    }

    apply_edge_protection(info, config, force_protected);
}

void AGIPCCoarseLinearSystem::build_adjacency_structure(const BuildInfo& info,
                                                        const Config&    config,
                                                        bool force_protected_flag)
{
    auto node_count     = static_cast<int>(m_node_count);
    auto abd_node_count = static_cast<int>(info.abd_node_count);
    auto block_size     = static_cast<int>(config.block_size);
    auto tet_count      = static_cast<int>(info.fem_tets.size());

    loose_resize(m_edge_counts, m_node_count + 1, config.reserve_ratio);
    loose_resize(m_row_offsets, m_node_count + 1, config.reserve_ratio);
    loose_resize(m_fill_offsets, m_node_count, config.reserve_ratio);
    m_edge_counts.fill(0);

    // Aggregation graph is built from INVARIANT mesh topology (FEM tet edges +
    // surface edges/triangles), not from the per-iteration fine_A Hessian. This
    // matches the reference's mesh-derived CSR and avoids carrying contact
    // coupling into the aggregation graph (contact is handled by the separate
    // edge-protection pass over the contact Hessian).
    int topology_candidate_count = static_cast<int>(
        tet_count + info.surface_edges.size() + info.surface_triangles.size());
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(topology_candidate_count,
               [tet_count,
                tets = info.fem_tets.cviewer().name("fem_tets"),
                surface_edges = info.surface_edges.cviewer().name("surface_edges"),
                surface_triangles = info.surface_triangles.cviewer().name("surface_triangles"),
                global_vertex_body_ids = info.global_vertex_body_ids.cviewer().name("global_vertex_body_ids"),
                node_count,
                abd_node_count,
                fem_node_count = static_cast<int>(info.fem_node_count),
                abd_global_body_offset = static_cast<int>(info.abd_global_body_offset),
                fem_global_body_offset = static_cast<int>(info.fem_global_body_offset),
                fem_global_body_count = static_cast<int>(info.fem_global_body_count),
                fem_global_vertex_offset = static_cast<int>(info.fem_global_vertex_offset),
                edge_counts = m_edge_counts.data()] MUDA_DEVICE(int t) mutable
               {
                   count_adjacency_edges_kernel(t,
                                                tet_count,
                                                tets,
                                                surface_edges,
                                                surface_triangles,
                                                global_vertex_body_ids,
                                                node_count,
                                                abd_node_count,
                                                fem_node_count,
                                                abd_global_body_offset,
                                                fem_global_body_offset,
                                                fem_global_body_count,
                                                fem_global_vertex_offset,
                                                edge_counts);
               });
    m_edge_counts.view(m_node_count, 1).fill(0);
    muda::DeviceScan().ExclusiveSum(m_edge_counts.data(), m_row_offsets.data(), node_count + 1);

    m_total_edge_count_var = 0;
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(node_count,
               [values = m_edge_counts.data(),
                result = m_total_edge_count_var.data()] MUDA_DEVICE(int i) mutable
               { sum_int_kernel(i, values, result); });
    int total_edges = m_total_edge_count_var;
    m_edge_count    = static_cast<SizeT>(std::max(total_edges, 0));

    loose_resize(m_col_indices, m_edge_count, config.reserve_ratio);
    loose_resize(m_edge_tags, m_edge_count, config.reserve_ratio);
    loose_resize(m_protected_flags, m_edge_count, config.reserve_ratio);
    loose_resize(m_unique_edge_counts, m_node_count + 1, config.reserve_ratio);
    loose_resize(m_unique_row_offsets, m_node_count + 1, config.reserve_ratio);
    m_fill_offsets.fill(0);

    if(m_edge_count > 0)
    {
        bool force_protected = force_protected_flag;
        muda::ParallelFor(block_size)
            .file_line(__FILE__, __LINE__)
            .apply(topology_candidate_count,
                   [tet_count,
                    tets = info.fem_tets.cviewer().name("fem_tets"),
                    surface_edges = info.surface_edges.cviewer().name("surface_edges"),
                    surface_triangles = info.surface_triangles.cviewer().name("surface_triangles"),
                    global_vertex_body_ids =
                        info.global_vertex_body_ids.cviewer().name("global_vertex_body_ids"),
                    node_count,
                    abd_node_count,
                    fem_node_count = static_cast<int>(info.fem_node_count),
                    abd_global_body_offset = static_cast<int>(info.abd_global_body_offset),
                    fem_global_body_offset = static_cast<int>(info.fem_global_body_offset),
                    fem_global_body_count = static_cast<int>(info.fem_global_body_count),
                    fem_global_vertex_offset = static_cast<int>(info.fem_global_vertex_offset),
                    force_protected = force_protected ? 1 : 0,
                    row_offsets     = m_row_offsets.data(),
                    fill_offsets    = m_fill_offsets.data(),
                    col_indices     = m_col_indices.data(),
                    edge_tags       = m_edge_tags.data(),
                    protected_flags = m_protected_flags.data()] MUDA_DEVICE(int t) mutable
                   {
                       fill_adjacency_edges_kernel(t,
                                                   tet_count,
                                                   tets,
                                                   surface_edges,
                                                   surface_triangles,
                                                   global_vertex_body_ids,
                                                   node_count,
                                                   abd_node_count,
                                                   fem_node_count,
                                                   abd_global_body_offset,
                                                   fem_global_body_offset,
                                                   fem_global_body_count,
                                                   fem_global_vertex_offset,
                                                   force_protected,
                                                   row_offsets,
                                                   fill_offsets,
                                                   col_indices,
                                                   edge_tags,
                                                   protected_flags);
                   });

        muda::ParallelFor(block_size)
            .file_line(__FILE__, __LINE__)
            .apply(node_count,
                   [row_offsets     = m_row_offsets.data(),
                    col_indices     = m_col_indices.data(),
                    edge_tags       = m_edge_tags.data(),
                    protected_flags = m_protected_flags.data(),
                    unique_edge_counts = m_unique_edge_counts.data()] MUDA_DEVICE(int row) mutable
                   {
                       sort_and_unique_adjacency_rows_kernel(
                           row, row_offsets, col_indices, edge_tags, protected_flags, unique_edge_counts);
                   });

        m_unique_edge_counts.view(m_node_count, 1).fill(0);
        muda::DeviceScan().ExclusiveSum(m_unique_edge_counts.data(),
                                        m_unique_row_offsets.data(),
                                        node_count + 1);
        m_total_edge_count_var = 0;
        muda::ParallelFor(block_size)
            .file_line(__FILE__, __LINE__)
            .apply(node_count,
                   [values = m_unique_edge_counts.data(),
                    result = m_total_edge_count_var.data()] MUDA_DEVICE(int i) mutable
                   { sum_int_kernel(i, values, result); });
        int unique_edges = m_total_edge_count_var;
        SizeT compact_edge_count = static_cast<SizeT>(std::max(unique_edges, 0));
        loose_resize(m_unique_col_indices, compact_edge_count, config.reserve_ratio);
        loose_resize(m_unique_edge_tags, compact_edge_count, config.reserve_ratio);
        loose_resize(m_unique_protected_flags, compact_edge_count, config.reserve_ratio);
        muda::ParallelFor(block_size)
            .file_line(__FILE__, __LINE__)
            .apply(node_count,
                   [row_offsets        = m_row_offsets.data(),
                    unique_row_offsets = m_unique_row_offsets.data(),
                    unique_counts      = m_unique_edge_counts.data(),
                    col_indices        = m_col_indices.data(),
                    edge_tags          = m_edge_tags.data(),
                    protected_flags    = m_protected_flags.data(),
                    unique_col_indices = m_unique_col_indices.data(),
                    unique_edge_tags   = m_unique_edge_tags.data(),
                    unique_protected_flags =
                        m_unique_protected_flags.data()] MUDA_DEVICE(int row) mutable
                   {
                       compact_adjacency_rows_kernel(row,
                                                     row_offsets,
                                                     unique_row_offsets,
                                                     unique_counts,
                                                     col_indices,
                                                     edge_tags,
                                                     protected_flags,
                                                     unique_col_indices,
                                                     unique_edge_tags,
                                                     unique_protected_flags);
                   });
        m_edge_count      = compact_edge_count;
        m_row_offsets     = m_unique_row_offsets.view();
        m_col_indices     = m_unique_col_indices.view();
        m_edge_tags       = m_unique_edge_tags.view();
        m_protected_flags = m_unique_protected_flags.view();
    }
}

void AGIPCCoarseLinearSystem::apply_edge_protection(const BuildInfo& info,
                                                    const Config&    config,
                                                    bool force_protected_flag)
{
    auto node_count     = static_cast<int>(m_node_count);
    auto abd_node_count = static_cast<int>(info.abd_node_count);
    auto block_size     = static_cast<int>(config.block_size);
    bool has_tet_green_data =
        info.fem_tets.size() > 0 && info.fem_Dm_invs.size() == info.fem_tets.size()
        && info.fem_current_positions.size() > 0
        && info.fem_rest_positions.size() == info.fem_current_positions.size();
    bool has_compatible_tet_history = m_has_previous_tet_green_strains
                                      && m_previous_tet_count == info.fem_tets.size();

    if(m_edge_count > 0)
    {
        if(info.contact_hessian.triplet_count() > 0)
        {
            muda::ParallelFor(block_size)
                .file_line(__FILE__, __LINE__)
                .apply(info.contact_hessian.triplet_count(),
                       [contact_hessian = info.contact_hessian.cviewer().name("contact_hessian"),
                        global_vertex_body_ids =
                            info.global_vertex_body_ids.cviewer().name("global_vertex_body_ids"),
                        node_count,
                        abd_node_count,
                        fem_node_count = static_cast<int>(info.fem_node_count),
                        abd_global_body_offset = static_cast<int>(info.abd_global_body_offset),
                        fem_global_body_offset = static_cast<int>(info.fem_global_body_offset),
                        fem_global_body_count = static_cast<int>(info.fem_global_body_count),
                        fem_global_vertex_offset = static_cast<int>(info.fem_global_vertex_offset),
                        row_offsets = m_row_offsets.data(),
                        col_indices = m_col_indices.data(),
                        edge_tags   = m_edge_tags.data(),
                        protected_flags = m_protected_flags.data()] MUDA_DEVICE(int triplet_id) mutable
                       {
                           protect_contact_edge_tags_kernel(triplet_id,
                                                            contact_hessian,
                                                            global_vertex_body_ids,
                                                            node_count,
                                                            abd_node_count,
                                                            fem_node_count,
                                                            abd_global_body_offset,
                                                            fem_global_body_offset,
                                                            fem_global_body_count,
                                                            fem_global_vertex_offset,
                                                            row_offsets,
                                                            col_indices,
                                                            edge_tags,
                                                            protected_flags);
                       });
        }

        if(has_tet_green_data)
        {
            loose_resize(m_current_tet_green_strains,
                         info.fem_tets.size(),
                         config.reserve_ratio);
            loose_resize(m_previous_tet_green_strains,
                         info.fem_tets.size(),
                         config.reserve_ratio);
            muda::ParallelFor(block_size)
                .file_line(__FILE__, __LINE__)
                .apply(info.fem_tets.size(),
                       [tets = info.fem_tets.cviewer().name("fem_tets"),
                        dm_invs = info.fem_Dm_invs.cviewer().name("fem_Dm_invs"),
                        xs = info.fem_current_positions.cviewer().name("fem_current_positions"),
                        rest_xs = info.fem_rest_positions.cviewer().name("fem_rest_positions"),
                        abd_node_count,
                        fem_node_count  = static_cast<int>(info.fem_node_count),
                        force_protected = force_protected_flag ? 1 : 0,
                        has_history     = has_compatible_tet_history ? 1 : 0,
                        green_strain_tau = config.green_strain_tau,
                        previous_green   = m_previous_tet_green_strains.data(),
                        current_green    = m_current_tet_green_strains.data(),
                        row_offsets      = m_row_offsets.data(),
                        col_indices      = m_col_indices.data(),
                        edge_tags        = m_edge_tags.data(),
                        protected_flags = m_protected_flags.data()] MUDA_DEVICE(int tet_id) mutable
                       {
                           compute_tet_green_strain_tags_kernel(tet_id,
                                                                tets,
                                                                dm_invs,
                                                                xs,
                                                                rest_xs,
                                                                abd_node_count,
                                                                fem_node_count,
                                                                force_protected,
                                                                has_history,
                                                                green_strain_tau,
                                                                previous_green,
                                                                current_green,
                                                                row_offsets,
                                                                col_indices,
                                                                edge_tags,
                                                                protected_flags);
                       });
            m_previous_tet_green_strains.view().copy_from(
                m_current_tet_green_strains.view());
            m_has_previous_tet_green_strains = true;
            m_previous_tet_count             = info.fem_tets.size();
        }
        else
        {
            m_has_previous_tet_green_strains = false;
            m_previous_tet_count             = 0;
        }
    }

    m_protected_edge_count_var = 0;
    if(m_edge_count > 0)
    {
        muda::ParallelFor(block_size)
            .file_line(__FILE__, __LINE__)
            .apply(static_cast<int>(m_edge_count),
                   [values = m_protected_flags.data(),
                    result = m_protected_edge_count_var.data()] MUDA_DEVICE(int i) mutable
                   { sum_int_kernel(i, values, result); });
    }
    int protected_edges    = m_protected_edge_count_var;
    m_protected_edge_count = static_cast<SizeT>(std::max(protected_edges, 0));
}

void AGIPCCoarseLinearSystem::build_mapping(const Config& config)
{
    auto node_count = static_cast<int>(m_node_count);
    auto block_size = static_cast<int>(config.block_size);

    loose_resize(m_union_parent, m_node_count, config.reserve_ratio);
    loose_resize(m_union_size, m_node_count, config.reserve_ratio);
    loose_resize(m_component_flags, m_node_count, config.reserve_ratio);
    loose_resize(m_component_offsets, m_node_count, config.reserve_ratio);
    loose_resize(m_node_to_coarse_node, m_node_count, config.reserve_ratio);

    if(node_count == 0)
    {
        m_coarse_node_count = 0;
        return;
    }

    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(node_count,
               [parent = m_union_parent.data(),
                size   = m_union_size.data()] MUDA_DEVICE(int node_id) mutable
               { init_union_parent_kernel(node_id, parent, size); });

    int   max_aggregate_size = static_cast<int>(config.max_aggregate_size);
    SizeT pass_count = std::max<SizeT>(SizeT{1}, config.component_passes);
    for(SizeT pass = 0; pass < pass_count; ++pass)
    {
        muda::ParallelFor(block_size)
            .file_line(__FILE__, __LINE__)
            .apply(node_count,
                   [node_count,
                    max_aggregate_size,
                    row_offsets = m_row_offsets.data(),
                    col_indices = m_col_indices.data(),
                    edge_tags   = m_edge_tags.data(),
                    parent      = m_union_parent.data(),
                    size = m_union_size.data()] MUDA_DEVICE(int node_id) mutable
                   {
                       union_collapsible_edges_kernel(node_id,
                                                      node_count,
                                                      max_aggregate_size,
                                                      row_offsets,
                                                      col_indices,
                                                      edge_tags,
                                                      parent,
                                                      size);
                   });

        muda::ParallelFor(block_size)
            .file_line(__FILE__, __LINE__)
            .apply(node_count,
                   [parent = m_union_parent.data()] MUDA_DEVICE(int node_id) mutable
                   { compress_union_parent_kernel(node_id, parent); });
    }

    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(node_count,
               [parent = m_union_parent.data(),
                component_flags = m_component_flags.data()] MUDA_DEVICE(int node_id) mutable
               { mark_component_roots_kernel(node_id, parent, component_flags); });

    muda::DeviceScan().ExclusiveSum(
        m_component_flags.data(), m_component_offsets.data(), node_count);

    m_total_coarse_node_count_var = 0;
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(node_count,
               [values = m_component_flags.data(),
                result = m_total_coarse_node_count_var.data()] MUDA_DEVICE(int i) mutable
               { sum_int_kernel(i, values, result); });
    int coarse_nodes    = m_total_coarse_node_count_var;
    m_coarse_node_count = static_cast<SizeT>(std::max(coarse_nodes, 0));

    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(node_count,
               [parent            = m_union_parent.data(),
                component_offsets = m_component_offsets.data(),
                node_to_coarse_node = m_node_to_coarse_node.data()] MUDA_DEVICE(int node_id) mutable
               {
                   build_component_map_kernel(node_id, parent, component_offsets, node_to_coarse_node);
               });
}

void AGIPCCoarseLinearSystem::build_dofs(const Config& config)
{
    auto node_count        = static_cast<int>(m_node_count);
    auto coarse_node_count = static_cast<int>(m_coarse_node_count);
    auto block_size        = static_cast<int>(config.block_size);
    auto abd_node_count    = static_cast<int>(m_abd_node_count);

    loose_resize(m_coarse_node_counts, m_coarse_node_count, config.reserve_ratio);
    loose_resize(m_coarse_node_contains_abd, m_coarse_node_count, config.reserve_ratio);
    loose_resize(m_coarse_fem_member_count, m_coarse_node_count, config.reserve_ratio);
    loose_resize(m_coarse_rest_centroid, m_coarse_node_count, config.reserve_ratio);
    loose_resize(m_coarse_dof, m_coarse_node_count, config.reserve_ratio);
    loose_resize(m_coarse_block_counts, m_coarse_node_count, config.reserve_ratio);
    loose_resize(m_coarse_block_offsets, m_coarse_node_count, config.reserve_ratio);
    loose_resize(m_node_to_coarse_block_begin, m_node_count, config.reserve_ratio);
    loose_resize(m_node_to_coarse_block_count, m_node_count, config.reserve_ratio);

    m_coarse_node_counts.fill(0);
    m_coarse_node_contains_abd.fill(0);
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(node_count,
               [abd_node_count,
                node_to_coarse_node = m_node_to_coarse_node.data(),
                coarse_node_counts  = m_coarse_node_counts.data(),
                coarse_node_contains_abd =
                    m_coarse_node_contains_abd.data()] MUDA_DEVICE(int node_id) mutable
               {
                   count_nodes_per_coarse_node_kernel(node_id,
                                                      abd_node_count,
                                                      node_to_coarse_node,
                                                      coarse_node_counts,
                                                      coarse_node_contains_abd);
               });

    // Per-coarse-node rest centroid for the centered affine basis.
    m_coarse_fem_member_count.fill(0);
    m_coarse_rest_centroid.fill(Vector3::Zero());
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(node_count,
               [abd_node_count,
                rest_positions = m_rest_positions.cviewer().name("rest_positions"),
                node_to_global_vertex = m_node_to_global_vertex.cviewer().name("node_to_global_vertex"),
                node_to_coarse_node = m_node_to_coarse_node.data(),
                coarse_rest_sum     = m_coarse_rest_centroid.data(),
                coarse_fem_member_count =
                    m_coarse_fem_member_count.data()] MUDA_DEVICE(int node_id) mutable
               {
                   accumulate_coarse_centroid_kernel(node_id,
                                                     abd_node_count,
                                                     rest_positions,
                                                     node_to_global_vertex,
                                                     node_to_coarse_node,
                                                     coarse_rest_sum,
                                                     coarse_fem_member_count);
               });
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(coarse_node_count,
               [coarse_fem_member_count = m_coarse_fem_member_count.data(),
                coarse_rest_centroid = m_coarse_rest_centroid.data()] MUDA_DEVICE(int coarse_node) mutable
               {
                   finalize_coarse_centroid_kernel(coarse_node,
                                                   coarse_fem_member_count,
                                                   coarse_rest_centroid);
               });

    m_affine_node_count_var = 0;
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(coarse_node_count,
               [coarse_node_counts       = m_coarse_node_counts.data(),
                coarse_node_contains_abd = m_coarse_node_contains_abd.data(),
                coarse_dof               = m_coarse_dof.data(),
                coarse_block_counts      = m_coarse_block_counts.data(),
                affine_threshold = static_cast<int>(config.affine_threshold),
                affine_node_count = m_affine_node_count_var.data()] MUDA_DEVICE(int coarse_node) mutable
               {
                   decide_coarse_dof_kernel(coarse_node,
                                            coarse_node_counts,
                                            coarse_node_contains_abd,
                                            coarse_dof,
                                            coarse_block_counts,
                                            affine_threshold,
                                            affine_node_count);
               });

    muda::DeviceScan().ExclusiveSum(m_coarse_block_counts.data(),
                                    m_coarse_block_offsets.data(),
                                    coarse_node_count);
    m_total_coarse_block_count_var = 0;
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(coarse_node_count,
               [values = m_coarse_block_counts.data(),
                result = m_total_coarse_block_count_var.data()] MUDA_DEVICE(int i) mutable
               { sum_int_kernel(i, values, result); });
    int coarse_blocks    = m_total_coarse_block_count_var;
    int affine_nodes     = m_affine_node_count_var;
    m_coarse_block_count = static_cast<SizeT>(std::max(coarse_blocks, 0));
    m_affine_node_count  = static_cast<SizeT>(std::max(affine_nodes, 0));

    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(node_count,
               [node_to_coarse_node  = m_node_to_coarse_node.data(),
                coarse_block_offsets = m_coarse_block_offsets.data(),
                coarse_block_counts  = m_coarse_block_counts.data(),
                node_to_coarse_block_begin = m_node_to_coarse_block_begin.data(),
                node_to_coarse_block_count =
                    m_node_to_coarse_block_count.data()] MUDA_DEVICE(int node_id) mutable
               {
                   build_node_to_coarse_block_kernel(node_id,
                                                     node_to_coarse_node,
                                                     coarse_block_offsets,
                                                     coarse_block_counts,
                                                     node_to_coarse_block_begin,
                                                     node_to_coarse_block_count);
               });
}

void AGIPCCoarseLinearSystem::build_triplets(const BuildInfo& info, const Config& config)
{
    auto fine_A             = info.fine_A;
    auto fine_triplet_count = fine_A.triplet_count();
    auto block_size         = static_cast<int>(config.block_size);
    auto abd_node_count     = static_cast<int>(m_abd_node_count);
    auto node_count         = static_cast<int>(m_node_count);

    loose_resize(m_triplet_block_counts, fine_triplet_count, config.reserve_ratio);
    loose_resize(m_triplet_block_offsets, fine_triplet_count, config.reserve_ratio);

    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(fine_triplet_count,
               [fine_A = fine_A.cviewer().name("fine_A"),
                node_to_coarse_block_begin = m_node_to_coarse_block_begin.data(),
                node_to_coarse_block_count = m_node_to_coarse_block_count.data(),
                abd_node_count,
                node_count,
                block_counts = m_triplet_block_counts.data()] MUDA_DEVICE(int triplet_id) mutable
               {
                   count_coarse_triplet_blocks_kernel(triplet_id,
                                                      fine_A,
                                                      node_to_coarse_block_begin,
                                                      node_to_coarse_block_count,
                                                      abd_node_count,
                                                      node_count,
                                                      block_counts);
               });

    muda::DeviceScan().ExclusiveSum(m_triplet_block_counts.data(),
                                    m_triplet_block_offsets.data(),
                                    fine_triplet_count);
    m_total_temporary_triplet_count_var = 0;
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(fine_triplet_count,
               [values = m_triplet_block_counts.data(),
                result = m_total_temporary_triplet_count_var.data()] MUDA_DEVICE(int i) mutable
               { sum_int_kernel(i, values, result); });
    int temporary_triplets = m_total_temporary_triplet_count_var;
    m_temporary_triplet_count = static_cast<SizeT>(std::max(temporary_triplets, 0));

    m_coarse_triplet_A.reshape(static_cast<int>(m_coarse_block_count),
                               static_cast<int>(m_coarse_block_count));
    if(m_coarse_triplet_A.triplet_capacity() < m_temporary_triplet_count)
        m_coarse_triplet_A.reserve_triplets(
            static_cast<size_t>(config.reserve_ratio * m_temporary_triplet_count));
    m_coarse_triplet_A.resize_triplets(m_temporary_triplet_count);

    if(m_temporary_triplet_count > 0)
    {
        muda::ParallelFor(block_size)
            .file_line(__FILE__, __LINE__)
            .apply(fine_triplet_count,
                   [fine_A        = fine_A.cviewer().name("fine_A"),
                    block_offsets = m_triplet_block_offsets.data(),
                    node_to_coarse_block_begin = m_node_to_coarse_block_begin.data(),
                    node_to_coarse_block_count = m_node_to_coarse_block_count.data(),
                    abd_node_count,
                    node_count,
                    rest_positions = info.rest_positions.cviewer().name("rest_positions"),
                    node_to_global_vertex = info.node_to_global_vertex.cviewer().name("node_to_global_vertex"),
                    node_to_coarse_node = m_node_to_coarse_node.cviewer().name("node_to_coarse_node"),
                    coarse_rest_centroid = m_coarse_rest_centroid.cviewer().name("coarse_rest_centroid"),
                    coarse_A = m_coarse_triplet_A.viewer().name("coarse_A")] MUDA_DEVICE(int triplet_id) mutable
                   {
                       build_coarse_triplets_kernel(triplet_id,
                                                    fine_A,
                                                    block_offsets,
                                                    node_to_coarse_block_begin,
                                                    node_to_coarse_block_count,
                                                    abd_node_count,
                                                    node_count,
                                                    rest_positions,
                                                    node_to_global_vertex,
                                                    node_to_coarse_node,
                                                    coarse_rest_centroid,
                                                    coarse_A);
                   });
        m_converter.convert(m_coarse_triplet_A, m_coarse_A);
    }
    else
    {
        m_coarse_A.reshape(static_cast<int>(m_coarse_block_count),
                           static_cast<int>(m_coarse_block_count));
        m_coarse_A.resize_triplets(0);
    }
}

void AGIPCCoarseLinearSystem::build_rhs(const BuildInfo& info, const Config& config)
{
    auto coarse_dof_count = m_coarse_block_count * kBlockDof;
    m_coarse_b.resize(coarse_dof_count);
    m_coarse_b.fill(0);

    if(coarse_dof_count == 0)
        return;

    auto block_size = static_cast<int>(config.block_size);
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(static_cast<int>(m_node_count),
               [abd_node_count = static_cast<int>(m_abd_node_count),
                fine_b         = info.fine_b.cviewer().name("fine_b"),
                rest_positions = info.rest_positions.cviewer().name("rest_positions"),
                node_to_global_vertex = info.node_to_global_vertex.cviewer().name("node_to_global_vertex"),
                node_to_coarse_node = m_node_to_coarse_node.cviewer().name("node_to_coarse_node"),
                coarse_rest_centroid = m_coarse_rest_centroid.cviewer().name("coarse_rest_centroid"),
                coarse_b = m_coarse_b.viewer().name("coarse_b"),
                node_to_coarse_block_begin = m_node_to_coarse_block_begin.data(),
                node_to_coarse_block_count =
                    m_node_to_coarse_block_count.data()] MUDA_DEVICE(int node_id) mutable
               {
                   Vector3 centroid =
                       coarse_centroid_for_node(node_id, node_to_coarse_node, coarse_rest_centroid);
                   restrict_rhs_kernel(node_id,
                                       abd_node_count,
                                       fine_b,
                                       rest_positions,
                                       node_to_global_vertex,
                                       centroid,
                                       coarse_b,
                                       node_to_coarse_block_begin,
                                       node_to_coarse_block_count);
               });
}

void AGIPCCoarseLinearSystem::prolongate_to(muda::DenseVectorView<Float> fine_x) const
{
    if(!m_built || m_node_count == 0)
        return;

    constexpr int block_size = 256;
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(static_cast<int>(m_node_count),
               [abd_node_count = static_cast<int>(m_abd_node_count),
                fine_x         = fine_x.viewer().name("fine_x"),
                coarse_x       = m_coarse_x.viewer().name("coarse_x"),
                rest_positions = m_rest_positions.cviewer().name("rest_positions"),
                node_to_global_vertex = m_node_to_global_vertex.cviewer().name("node_to_global_vertex"),
                node_to_coarse_node = m_node_to_coarse_node.cviewer().name("node_to_coarse_node"),
                coarse_rest_centroid = m_coarse_rest_centroid.cviewer().name("coarse_rest_centroid"),
                node_to_coarse_block_begin = m_node_to_coarse_block_begin.data(),
                node_to_coarse_block_count =
                    m_node_to_coarse_block_count.data()] MUDA_DEVICE(int node_id) mutable
               {
                   Vector3 centroid =
                       coarse_centroid_for_node(node_id, node_to_coarse_node, coarse_rest_centroid);
                   prolongate_kernel(node_id,
                                     abd_node_count,
                                     fine_x,
                                     coarse_x,
                                     rest_positions,
                                     node_to_global_vertex,
                                     centroid,
                                     node_to_coarse_block_begin,
                                     node_to_coarse_block_count);
               });
}
}  // namespace uipc::backend::cuda
