#pragma once

#include <algorithm/matrix_converter.h>
#include <linear_system/spmv.h>
#include <muda/buffer/device_buffer.h>
#include <muda/ext/linear_system/device_bcoo_matrix.h>
#include <muda/ext/linear_system/device_dense_vector.h>
#include <muda/ext/linear_system/device_triplet_matrix.h>
#include <global_geometry/global_simplicial_surface_manager.h>

namespace uipc::backend::cuda
{
// AGIPC algebraic coarse system for mixed ABD/FEM 3x3 block matrices.
// It mirrors the paper pipeline: CSR adjacency + edge tags -> global
// connected-component aggregation -> 3/12-DoF supernodes -> Galerkin coarse
// Hessian/RHS -> prolongation.
class AGIPCCoarseLinearSystem
{
  public:
    struct Config
    {
        bool  enable           = false;
        SizeT affine_threshold = 32;
        // 0 => adaptive fine-refinement budget min(300,max(5,(coarse_iter+2)/3)),
        // matching the reference; a positive value forces a fixed budget.
        SizeT fine_refine_iters = 0;
        SizeT block_size        = 256;
        Float green_strain_tau  = 5e-5;
        SizeT component_passes  = 8;
        // Maximum number of fine nodes a single supernode may aggregate. The
        // paper/reference aggregate within bounded warp groups (group-local hash
        // mapping), so a supernode never spans an unbounded mesh region. Bounding
        // the size keeps the 3/12-DoF embedding able to represent the aggregate's
        // deformation; an unbounded global aggregate produces a poor coarse search
        // direction and blows up Newton iterations on stiff-contact frames.
        SizeT max_aggregate_size = 32;
        SizeT coarse_max_iters   = 150;
        // Relative residual tolerance for the (inexact) coarse PCG solve. Looser
        // than the fine-solve tol_rate because the coarse solve is only a Newton
        // correction refined afterwards. Matches reference CoarseSolveConfig.
        Float coarse_pcg_relative_tol              = 1e-3;
        SizeT coarse_check_interval                = 30;
        Float displacement_tolerance               = 0.0;
        bool  use_fine_preconditioner_after_coarse = true;
        // Use the MAS multilevel preconditioner for the coarse PCG instead of the
        // 3x3 block-Jacobi. Stronger -> fewer coarse iterations on stiff systems.
        bool  use_mas_preconditioner = false;
        Float reserve_ratio          = 1.5;
    };

    struct BuildInfo
    {
        muda::CBCOOMatrixView<Float, 3> fine_A;
        muda::CDenseVectorView<Float>   fine_b;
        muda::CBufferView<Vector3>      current_positions;
        muda::CBufferView<Vector3>      rest_positions;
        muda::CBufferView<IndexT>       node_to_global_vertex;
        muda::CBCOOMatrixView<Float, 3> contact_hessian;
        muda::CBufferView<Vector4i>     fem_tets;
        muda::CBufferView<Matrix3x3>    fem_Dm_invs;
        muda::CBufferView<Vector3>      fem_current_positions;
        muda::CBufferView<Vector3>      fem_rest_positions;
        muda::CBufferView<Vector2i>     surface_edges;
        muda::CBufferView<Vector3i>     surface_triangles;
        muda::CBufferView<IndexT>       global_vertex_body_ids;
        IndexT                          abd_global_body_offset   = 0;
        IndexT                          fem_global_body_offset   = 0;
        IndexT                          fem_global_body_count    = 0;
        IndexT                          fem_global_vertex_offset = 0;
        SizeT                           abd_node_count           = 0;
        SizeT                           fem_node_count           = 0;
        SizeT                           frame                    = 0;
        SizeT                           newton_iter              = 0;
    };

    void build(const BuildInfo& info, const Config& config);
    void prolongate_to(muda::DenseVectorView<Float> fine_x) const;

    muda::DeviceBCOOMatrix<Float, 3>& hessian() noexcept { return m_coarse_A; }
    const muda::DeviceBCOOMatrix<Float, 3>& hessian() const noexcept
    {
        return m_coarse_A;
    }

    muda::DeviceDenseVector<Float>&       rhs() noexcept { return m_coarse_b; }
    const muda::DeviceDenseVector<Float>& rhs() const noexcept
    {
        return m_coarse_b;
    }

    muda::DeviceDenseVector<Float>& solution() noexcept { return m_coarse_x; }
    const muda::DeviceDenseVector<Float>& solution() const noexcept
    {
        return m_coarse_x;
    }

    SizeT node_count() const noexcept { return m_node_count; }
    SizeT coarse_node_count() const noexcept { return m_coarse_node_count; }
    SizeT coarse_block_count() const noexcept { return m_coarse_block_count; }
    SizeT temporary_triplet_count() const noexcept
    {
        return m_temporary_triplet_count;
    }
    SizeT edge_count() const noexcept { return m_edge_count; }
    SizeT protected_edge_count() const noexcept
    {
        return m_protected_edge_count;
    }
    SizeT affine_node_count() const noexcept { return m_affine_node_count; }
    SizeT abd_node_count() const noexcept { return m_abd_node_count; }
    bool  built() const noexcept { return m_built; }

    // MUDA extended device lambdas require the enclosing host member to be public.
    // These are build-stage entry points owned by build(); external code should not call them directly.
    void build_adjacency(const BuildInfo& info, const Config& config);
    void build_adjacency_structure(const BuildInfo& info, const Config& config, bool force_protected_flag);
    void apply_edge_protection(const BuildInfo& info, const Config& config, bool force_protected_flag);
    void build_mapping(const Config& config);
    void build_dofs(const Config& config);
    void build_triplets(const BuildInfo& info, const Config& config);
    void build_rhs(const BuildInfo& info, const Config& config);

  private:
    template <typename T>
    void loose_resize(muda::DeviceBuffer<T>& buffer, SizeT size, Float reserve_ratio)
    {
        if(buffer.capacity() < size)
            buffer.reserve(static_cast<size_t>(static_cast<Float>(size) * reserve_ratio));
        buffer.resize(static_cast<size_t>(size));
    }

    muda::DeviceBuffer<int>       m_edge_counts;
    muda::DeviceBuffer<int>       m_row_offsets;
    muda::DeviceBuffer<int>       m_fill_offsets;
    muda::DeviceBuffer<int>       m_col_indices;
    muda::DeviceBuffer<Float>     m_edge_tags;
    muda::DeviceBuffer<Matrix3x3> m_current_tet_green_strains;
    muda::DeviceBuffer<Matrix3x3> m_previous_tet_green_strains;
    muda::DeviceBuffer<int>       m_protected_flags;
    muda::DeviceBuffer<int>       m_unique_edge_counts;
    muda::DeviceBuffer<int>       m_unique_row_offsets;
    muda::DeviceBuffer<int>       m_unique_col_indices;
    muda::DeviceBuffer<Float>     m_unique_edge_tags;
    muda::DeviceBuffer<int>       m_unique_protected_flags;

    muda::DeviceBuffer<int> m_union_parent;
    muda::DeviceBuffer<int> m_union_size;  // subtree size at each root for bounded aggregation
    muda::DeviceBuffer<int> m_component_flags;
    muda::DeviceBuffer<int> m_component_offsets;
    muda::DeviceBuffer<int> m_node_to_coarse_node;

    muda::DeviceBuffer<int> m_coarse_node_counts;
    muda::DeviceBuffer<int> m_coarse_node_contains_abd;
    // Per-coarse-node rest-position centroid (FEM members only). The affine
    // embedding basis is centered at this centroid ([1, X-X̄, Y-Ȳ, Z-Z̄]) so the
    // 12-DoF block stays well-conditioned regardless of the aggregate's world
    // position; an uncentered [1,X,Y,Z] basis is ill-conditioned for aggregates
    // far from the origin and makes the coarse PCG converge slowly.
    muda::DeviceBuffer<int>     m_coarse_fem_member_count;
    muda::DeviceBuffer<Vector3> m_coarse_rest_centroid;
    muda::DeviceBuffer<int>     m_coarse_dof;
    muda::DeviceBuffer<int>     m_coarse_block_counts;
    muda::DeviceBuffer<int>     m_coarse_block_offsets;
    muda::DeviceBuffer<int>     m_node_to_coarse_block_begin;
    muda::DeviceBuffer<int>     m_node_to_coarse_block_count;

    muda::DeviceBuffer<int>             m_triplet_block_counts;
    muda::DeviceBuffer<int>             m_triplet_block_offsets;
    muda::DeviceTripletMatrix<Float, 3> m_coarse_triplet_A;
    muda::DeviceBCOOMatrix<Float, 3>    m_coarse_A;
    MatrixConverter<Float, 3>           m_converter;

    muda::DeviceDenseVector<Float> m_coarse_b;
    muda::DeviceDenseVector<Float> m_coarse_x;
    muda::CBufferView<Vector3>     m_rest_positions;
    muda::CBufferView<IndexT>      m_node_to_global_vertex;

    muda::DeviceVar<int> m_total_edge_count_var;
    muda::DeviceVar<int> m_total_coarse_node_count_var;
    muda::DeviceVar<int> m_total_coarse_block_count_var;
    muda::DeviceVar<int> m_total_temporary_triplet_count_var;
    muda::DeviceVar<int> m_protected_edge_count_var;
    muda::DeviceVar<int> m_affine_node_count_var;

    SizeT m_node_count                     = 0;
    SizeT m_abd_node_count                 = 0;
    SizeT m_edge_count                     = 0;
    SizeT m_coarse_node_count              = 0;
    SizeT m_coarse_block_count             = 0;
    SizeT m_temporary_triplet_count        = 0;
    SizeT m_protected_edge_count           = 0;
    SizeT m_affine_node_count              = 0;
    SizeT m_previous_tet_count             = 0;
    bool  m_has_previous_tet_green_strains = false;
    bool  m_built                          = false;

    // Adjacency-topology cache. The aggregation graph is built from invariant mesh
    // topology (FEM tet edges + surface), so its CSR structure (row offsets, col
    // indices) is constant while the mesh connectivity is unchanged. Cache it and
    // rebuild only when these signature sizes change; per-iteration we just reset
    // edge tags and re-apply contact/strain protection on the cached structure.
    bool  m_adjacency_cached       = false;
    SizeT m_cached_node_count      = 0;
    SizeT m_cached_tet_count       = 0;
    SizeT m_cached_surf_edge_count = 0;
    SizeT m_cached_surf_tri_count  = 0;
};
}  // namespace uipc::backend::cuda
