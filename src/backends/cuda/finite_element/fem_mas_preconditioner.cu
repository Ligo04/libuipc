#include <finite_element/fem_mas_preconditioner.h>
#include <Eigen/Dense>
#include <linear_system/local_preconditioner.h>
#include <finite_element/finite_element_method.h>
#include <linear_system/global_linear_system.h>
#include <finite_element/fem_linear_subsystem.h>
#include <global_geometry/global_vertex_manager.h>
#include <finite_element/mas_preconditioner_engine.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/geometry/utils/mesh_partition.h>
#include <uipc/common/log.h>
#include <backends/common/backend_path_tool.h>
#include <sim_engine.h>
#include <set>
#include <memory>

namespace uipc::backend::cuda
{
namespace
{
    namespace eigen = cuda_tool::eigen;

    __global__ void assemble_diag_inv_for_unpartitioned_kernel(
        cuda_tool::CBCOOMatrixView<Float, 3> triplet,
        cuda_tool::BufferView<Matrix3x3>     diag,
        cuda_tool::CBufferView<int>          unpart,
        int                                  fem_offset,
        int                                  fem_count,
        int                                  n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        auto&& [g_i, g_j, H3x3] = triplet(I);

        int i = g_i - fem_offset;
        int j = g_j - fem_offset;

        if(i < 0 || i >= fem_count || j < 0 || j >= fem_count)
            return;

        if(i == j && unpart(i) == 1)
            diag(i) = eigen::inverse(H3x3);
    }

    __global__ void apply_diag_inv_for_unpartitioned_kernel(
        cuda_tool::CDenseVectorView<Float> r_view,
        cuda_tool::DenseVectorView<Float>  z_view,
        cuda_tool::CBufferView<Matrix3x3>  diag,
        cuda_tool::CBufferView<int>        unpart,
        cuda_tool::CDense<IndexT>          converged,
        int                                n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(*converged != 0)
            return;
        if(unpart(i) == 1)
        {
            z_view.segment<3>(i * 3).as_eigen() =
                diag(i) * r_view.segment<3>(i * 3).as_eigen();
        }
    }

    __global__ void FEMMASPreconditioner_validate_nonlinear_diag_kernel(
        cuda_tool::CBufferView<Matrix3x3> inverses,
        cuda_tool::CBufferView<int>       unpartitioned,
        cuda_tool::BufferView<IndexT>     status,
        int                               n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n || unpartitioned(i) != 1)
            return;
        const auto& A     = inverses(i);
        double      scale = 0.0;
        for(int row = 0; row < 3; ++row)
            for(int col = 0; col < 3; ++col)
            {
                if(!isfinite(A(row, col)))
                {
                    atomicExch(status.data(), IndexT{1});
                    return;
                }
                scale = fmax(scale, fabs(A(row, col)));
            }
        if(!(scale > 0.0))
        {
            atomicExch(status.data(), IndexT{1});
            return;
        }
        double L[3][3] = {};
        for(int row = 0; row < 3; ++row)
            for(int col = 0; col <= row; ++col)
            {
                if(fabs(A(row, col) - A(col, row)) > 1e-10 * scale)
                {
                    atomicExch(status.data(), IndexT{1});
                    return;
                }
                double value = 0.5 * (A(row, col) + A(col, row));
                for(int k = 0; k < col; ++k)
                    value -= L[row][k] * L[col][k];
                if(row == col)
                {
                    if(!isfinite(value) || !(value > 1e-14 * scale))
                    {
                        atomicExch(status.data(), IndexT{1});
                        return;
                    }
                    L[row][col] = sqrt(value);
                }
                else
                    L[row][col] = value / L[col][col];
            }
    }

    __global__ void FEMMASPreconditioner_apply_nonlinear_diag_kernel(
        cuda_tool::CDenseVectorView<Float> r,
        cuda_tool::DenseVectorView<Float>  z,
        cuda_tool::CBufferView<Matrix3x3>  inverses,
        cuda_tool::CBufferView<int>        unpartitioned,
        int                                n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i < n && unpartitioned(i) == 1)
            z.segment<3>(i * 3).as_eigen() = inverses(i) * r.segment<3>(i * 3).as_eigen();
    }
}  // namespace

// Must match REGISTER_CONSTITUTION_UIDS EmptyUID in src/constitution/empty.cpp
constexpr ::uipc::U64 kEmptyConstitutionUID = 0ull;
void assemble_diag_inv_for_unpartitioned(cuda_tool::DeviceBuffer<Matrix3x3>& diag_inv,
                                         const cuda_tool::DeviceBuffer<int>& unpart_flags,
                                         cuda_tool::CBCOOMatrixView<Float, 3> A,
                                         int   fem_block_offset,
                                         int   fem_block_count,
                                         SizeT num_verts)
{
    diag_inv.resize(num_verts);
    diag_inv.fill(Matrix3x3::Identity());

    int n = A.triplet_count();
    if(n > 0)
    {
        auto k = assemble_diag_inv_for_unpartitioned_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            A, diag_inv.view(), unpart_flags.cview(), fem_block_offset, fem_block_count, n);
    }
}

void apply_diag_inv_for_unpartitioned(const cuda_tool::DeviceBuffer<Matrix3x3>& diag_inv,
                                      const cuda_tool::DeviceBuffer<int>& unpart_flags,
                                      cuda_tool::DenseVectorView<Float>  z,
                                      cuda_tool::CDenseVectorView<Float> r,
                                      cuda_tool::CVarView<IndexT> converged,
                                      SizeT                       num_verts,
                                      cudaStream_t                stream)
{
    int n = (int)num_verts;
    if(n > 0)
    {
        auto k = apply_diag_inv_for_unpartitioned_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, stream>>>(
            r, z, diag_inv.cview(), unpart_flags.cview(), converged.cviewer(), n);
    }
}

/**
 * @brief FEM MAS (MultiLevel Additive Schwarz) Preconditioner
 *
 * A multi-level domain-decomposition preconditioner that replaces the
 * simpler diagonal (Jacobi) preconditioner for much better convergence
 * on stiff problems. Based on the StiffGIPC paper.
 *
 * Activation:
 *   Set scene config `linear_system/fem_preconditioner = "mas"`, or select
 *   `nonlinear_system/solver = "mas_pncg"`. MAS is
 *   all-or-nothing: every non-Empty FEM SimplicialComplex is partitioned
 *   internally (fixed cluster size = BANKSIZE 16) in do_init; a
 *   pre-existing `mesh_part` vertex attribute (custom C++ partitioning)
 *   is respected as-is. Partial manual tagging alone does NOT activate
 *   MAS (measured net-negative — see agent_docs/09 coverage rule).
 */
void FEMMASPreconditioner::do_build(BuildInfo& info)
{
    finite_element_method       = &require<FiniteElementMethod>();
    global_linear_system        = &require<GlobalLinearSystem>();
    fem_linear_subsystem        = &require<FEMLinearSubsystem>();
    auto& global_vertex_manager = require<GlobalVertexManager>();

    auto config        = world().scene().config();
    auto nonlinear     = config.find<std::string>("nonlinear_system/solver");
    bool nonlinear_mas = nonlinear && nonlinear->view()[0] == "mas_pncg";
    auto precond = config.find<std::string>("linear_system/fem_preconditioner");
    if(!nonlinear_mas && (!precond || precond->view()[0] != "mas"))
    {
        throw SimSystemException("FEMMASPreconditioner: linear_system/fem_preconditioner != \"mas\", disabled.");
    }

    info.connect(fem_linear_subsystem);
}

void FEMMASPreconditioner::do_init(InitInfo& info)
{
    m_nonlinear_ready   = false;
    m_has_partition     = false;
    m_has_unpartitioned = false;
    auto& fem           = finite_element_method->m_impl;

    SizeT vert_num = fem.xs.size();
    if(vert_num == 0)
        return;

    // ---- 1. Build vertex adjacency from element connectivity ----

    std::vector<std::set<unsigned int>> vert_neighbors(vert_num);

    auto add_edge = [&](IndexT a, IndexT b)
    {
        if(a != b && a >= 0 && b >= 0 && a < static_cast<IndexT>(vert_num)
           && b < static_cast<IndexT>(vert_num))
        {
            vert_neighbors[a].insert(static_cast<unsigned int>(b));
            vert_neighbors[b].insert(static_cast<unsigned int>(a));
        }
    };

    for(auto& tet : fem.h_tets)
        for(int i = 0; i < 4; i++)
            for(int j = i + 1; j < 4; j++)
                add_edge(tet[i], tet[j]);

    for(auto& tri : fem.h_codim_2ds)
        for(int i = 0; i < 3; i++)
            for(int j = i + 1; j < 3; j++)
                add_edge(tri[i], tri[j]);

    for(auto& edge : fem.h_codim_1ds)
        add_edge(edge[0], edge[1]);

    // ---- 2. Build CSR neighbor arrays ----

    std::vector<unsigned int> h_neighbor_list;
    std::vector<unsigned int> h_neighbor_start(vert_num, 0);
    std::vector<unsigned int> h_neighbor_num(vert_num, 0);

    for(SizeT i = 0; i < vert_num; i++)
    {
        h_neighbor_start[i] = static_cast<unsigned int>(h_neighbor_list.size());
        h_neighbor_num[i] = static_cast<unsigned int>(vert_neighbors[i].size());
        for(auto n : vert_neighbors[i])
            h_neighbor_list.push_back(n);
    }

    // ---- 3. Partition every FEM geometry and build partition mappings ----
    //
    // MAS is all-or-nothing: every non-Empty FEM SimplicialComplex is
    // covered. A pre-existing mesh_part attribute (custom C++
    // partitioning) is respected as-is; everything else is partitioned
    // internally on a private clone with the fixed cluster size
    // (BANKSIZE), so the user's scene geometry is never mutated.

    std::vector<IndexT> part_ids(vert_num, -1);
    bool                has_parts = false;

    // Partition IDs are mesh-local (each mesh starts at 0).
    // We add a running offset so that IDs are globally unique
    // across all geometries in the FEM system.
    IndexT partition_offset = 0;

    auto geo_slots = world().scene().geometries();
    for(auto& geo_info : fem.geo_infos)
    {
        auto& geo_slot = geo_slots[geo_info.geo_slot_index];
        auto& geo      = geo_slot->geometry();
        auto* sc       = geo.as<geometry::SimplicialComplex>();
        if(!sc || sc->dim() < 1 || geo_info.vertex_count == 0)
            continue;

        // Empty constitution (no material): skip, stays on the
        // diagonal fallback
        auto cuid = geo.meta().find<U64>(builtin::constitution_uid);
        if(!cuid || cuid->view()[0] == kEmptyConstitutionUID)
            continue;

        std::vector<IndexT> local_parts;
        if(auto mesh_part = sc->vertices().find<IndexT>("mesh_part"))
        {
            auto pv = mesh_part->view();
            local_parts.assign(pv.begin(), pv.end());
        }
        else
        {
            auto tmp_geo = std::static_pointer_cast<geometry::Geometry>(geo.clone());
            auto* tmp_sc = tmp_geo->as<geometry::SimplicialComplex>();
            geometry::mesh_partition(*tmp_sc, BANKSIZE);
            auto pv = tmp_sc->vertices().find<IndexT>("mesh_part")->view();
            local_parts.assign(pv.begin(), pv.end());
        }

        has_parts        = true;
        IndexT local_max = 0;
        for(SizeT v = 0; v < geo_info.vertex_count; v++)
        {
            IndexT local_pid                     = local_parts[v];
            part_ids[geo_info.vertex_offset + v] = local_pid + partition_offset;
            local_max = std::max(local_max, local_pid);
        }
        partition_offset += local_max + 1;  // next mesh starts after this mesh's max
    }

    if(!has_parts)
    {
        m_has_partition = false;
        return;
    }

    // Tag Empty (non-cloth) FEM vertices; they must not carry mesh_part when using MAS.
    std::vector<uint8_t> h_non_cloth_fem(vert_num, 0);
    for(auto& geo_info : fem.geo_infos)
    {
        auto& geo_slot = geo_slots[geo_info.geo_slot_index];
        auto& geo      = geo_slot->geometry();
        auto  cuid     = geo.meta().find<U64>(builtin::constitution_uid);
        if(!cuid || cuid->view()[0] != kEmptyConstitutionUID)
            continue;
        for(SizeT v = 0; v < geo_info.vertex_count; ++v)
            h_non_cloth_fem[geo_info.vertex_offset + v] = 1;
    }
    for(SizeT v = 0; v < vert_num; ++v)
    {
        UIPC_ASSERT(!(h_non_cloth_fem[v] != 0 && part_ids[v] >= 0),
                    "MAS: Empty FEM vertex {} has mesh_part / partition id {}. "
                    "Strip mesh_part on Empty geometries.",
                    v,
                    part_ids[v]);
    }

    // Check if any vertices are unpartitioned (part_ids[v] == -1)
    {
        std::vector<int> h_unpart_flags(vert_num, 0);
        bool             any_unpartitioned = false;
        for(SizeT i = 0; i < vert_num; i++)
        {
            if(part_ids[i] < 0)
            {
                h_unpart_flags[i] = 1;
                any_unpartitioned = true;
            }
        }
        m_has_unpartitioned = any_unpartitioned;
        unpartitioned_flags.resize(vert_num);
        unpartitioned_flags.view().copy_from(h_unpart_flags.data());
    }

    // ---- 4. Build partition-ordered index mappings ----

    IndexT max_part_id = 0;
    for(auto pid : part_ids)
        if(pid > max_part_id)
            max_part_id = pid;

    std::vector<std::vector<int>> part_blocks(max_part_id + 1);
    for(SizeT i = 0; i < vert_num; i++)
        if(part_ids[i] >= 0)
            part_blocks[part_ids[i]].push_back(static_cast<int>(i));

    // Validate: no partition block should exceed BANKSIZE.
    // If this fires, partition IDs from different meshes are colliding.
    for(SizeT b = 0; b < part_blocks.size(); b++)
    {
        UIPC_ASSERT(static_cast<int>(part_blocks[b].size()) <= BANKSIZE,
                    "MAS: partition {} has {} vertices (max {}). "
                    "Partition IDs from different meshes may be colliding — "
                    "need global offset.",
                    b,
                    part_blocks[b].size(),
                    BANKSIZE);
    }

    // Each partition block is padded to BANKSIZE alignment
    int part_map_size = 0;
    for(auto& block : part_blocks)
    {
        int padded = (static_cast<int>(block.size()) + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
        part_map_size += padded;
    }

    std::vector<int> h_part_to_real(part_map_size, -1);
    std::vector<int> h_real_to_part(vert_num, -1);

    int offset = 0;
    for(auto& block : part_blocks)
    {
        for(SizeT i = 0; i < block.size(); i++)
        {
            int real_idx                    = block[i];
            h_part_to_real[offset + (int)i] = real_idx;
            h_real_to_part[real_idx]        = offset + static_cast<int>(i);
        }
        int padded = (static_cast<int>(block.size()) + BANKSIZE - 1) / BANKSIZE * BANKSIZE;
        offset += padded;
    }

    // Validate mappings to avoid out-of-range indices in MAS kernels.
    for(SizeT i = 0; i < vert_num; ++i)
    {
        auto pid = part_ids[i];
        if(pid < 0)
        {
            UIPC_ASSERT(h_real_to_part[i] == -1,
                        "MAS: unpartitioned vertex {} must map to -1, got {}.",
                        i,
                        h_real_to_part[i]);
        }
        else
        {
            UIPC_ASSERT(h_real_to_part[i] >= 0 && h_real_to_part[i] < part_map_size,
                        "MAS: real_to_part[{}]={} out of range [0, {}).",
                        i,
                        h_real_to_part[i],
                        part_map_size);
        }
    }

    for(int i = 0; i < part_map_size; ++i)
    {
        int rid = h_part_to_real[i];
        if(rid >= 0)
        {
            UIPC_ASSERT(rid < static_cast<int>(vert_num),
                        "MAS: part_to_real[{}]={} out of range [0, {}).",
                        i,
                        rid,
                        vert_num);
        }
    }

    // ---- 5. Initialize the engine ----

    engine.init_neighbor(static_cast<int>(vert_num),
                         static_cast<int>(h_neighbor_list.size()),
                         part_map_size,
                         h_neighbor_list,
                         h_neighbor_start,
                         h_neighbor_num,
                         h_part_to_real,
                         h_real_to_part);

    engine.init_matrix();

    m_has_partition = true;
}

void FEMMASPreconditioner::do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info)
{
    if(!m_has_partition || !engine.is_initialized())
        return;

    auto A          = info.A();
    int  dof_offset = static_cast<int>(info.dof_offset());

    // MAS assembly for partitioned vertices
    engine.set_preconditioner(A.values(), A.row_indices(), A.col_indices(), dof_offset / 3);

    auto dump_mas = world().scene().config().find<IndexT>("extras/debug/dump_mas_matrices");
    if(dump_mas && dump_mas->view()[0] != 0)
    {
        auto& cuda_engine = static_cast<SimEngine&>(uipc::backend::SimSystem::engine());
        backend::BackendPathTool path_tool{std::string{workspace()}};
        auto out_dir = path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "debug");
        this->engine.dump_cluster_matrices_debug(
            out_dir.string(), cuda_engine.frame(), cuda_engine.newton_iter());
    }

    // Diagonal fallback assembly for unpartitioned vertices
    if(m_has_unpartitioned)
    {
        SizeT num_verts        = finite_element_method->xs().size();
        int   fem_block_offset = dof_offset / 3;
        int   fem_block_count  = static_cast<int>(info.dof_count()) / 3;

        assemble_diag_inv_for_unpartitioned(
            diag_inv, unpartitioned_flags, A, fem_block_offset, fem_block_count, num_verts);
    }
}

void FEMMASPreconditioner::do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info)
{
    if(!m_has_partition || !engine.is_initialized())
    {
        // switch enabled but nothing partitionable (e.g. all FEM
        // geometries are Empty constitution): identity fallback so
        // this subsystem's z is never left stale
        cuda_tool::BufferLaunch(info.stream())
            .copy(info.z().buffer_view(), info.r().buffer_view());
        return;
    }

    auto converged = info.converged();

    // MAS for partitioned vertices
    engine.apply(info.r(), info.z(), converged, info.stream());

    // Diagonal fallback for unpartitioned vertices
    if(m_has_unpartitioned)
    {
        apply_diag_inv_for_unpartitioned(diag_inv,
                                         unpartitioned_flags,
                                         info.z(),
                                         info.r(),
                                         converged,
                                         diag_inv.size(),
                                         info.stream());
    }
}

bool FEMMASPreconditioner::capture_nonlinear_base(SizeT epoch)
{
    m_nonlinear_ready = false;
    m_nonlinear_epoch = MASPreconditionerEngine::invalid_epoch;
    if(epoch == MASPreconditionerEngine::invalid_epoch)
        return false;
    m_nonlinear_vertex_count = static_cast<int>(finite_element_method->xs().size());
    if(m_has_partition && !engine.capture_nonlinear_base(epoch))
        return false;
    if(m_has_unpartitioned)
    {
        if(diag_inv.size() != m_nonlinear_vertex_count)
            return false;
        nonlinear_diag_inv.resize_discard(diag_inv.size());
        nonlinear_diag_inv.view().copy_from(diag_inv.cview());
        nonlinear_status.resize_discard(1);
        nonlinear_status.fill(IndexT{0});
        int n = m_nonlinear_vertex_count;
        if(n > 0)
        {
            auto validate = FEMMASPreconditioner_validate_nonlinear_diag_kernel;
            validate<<<cuda_tool::best_grid_dim(n, validate), cuda_tool::best_block_dim(validate)>>>(
                nonlinear_diag_inv.cview(),
                unpartitioned_flags.cview(),
                nonlinear_status.view(),
                n);
        }
        CUDA_TOOL_CHECK(cudaGetLastError());
        IndexT failed = 1;
        nonlinear_status.cview().copy_to(&failed);
        CUDA_TOOL_CHECK(cudaStreamSynchronize(nullptr));
        if(failed != 0)
            return false;
    }
    else
        CUDA_TOOL_CHECK(cudaStreamSynchronize(nullptr));
    m_nonlinear_epoch = epoch;
    m_nonlinear_ready = true;
    return true;
}

MASPreconditionerEngine::FrozenLevel0View FEMMASPreconditioner::nonlinear_base(SizeT expected_epoch) const
{
    if(!m_nonlinear_ready || expected_epoch != m_nonlinear_epoch)
        return {};
    if(m_has_partition)
        return engine.nonlinear_base(expected_epoch);
    MASPreconditionerEngine::FrozenLevel0View result;
    result.epoch             = m_nonlinear_epoch;
    result.real_vertex_count = m_nonlinear_vertex_count;
    return result;
}

void FEMMASPreconditioner::apply_frozen_coarse(SizeT epoch,
                                               cuda_tool::CDenseVectorView<Float> r,
                                               cuda_tool::DenseVectorView<Float> z)
{
    UIPC_ASSERT(m_nonlinear_ready && epoch == m_nonlinear_epoch,
                "FEM MAS nonlinear base epoch {} is not available",
                epoch);
    UIPC_ASSERT(r.size() == m_nonlinear_vertex_count * 3 && z.size() == r.size(),
                "FEM MAS frozen application requires a FEM-local vector slice");
    UIPC_ASSERT(r.size() == 0 || r.buffer_view().data() != z.buffer_view().data(),
                "FEM MAS frozen application requires distinct input and output buffers");
    if(!m_has_partition)
    {
        cuda_tool::BufferLaunch().copy(z.buffer_view(), r.buffer_view());
        return;
    }
    engine.apply_frozen_coarse(epoch, r, z);
    if(m_has_unpartitioned && m_nonlinear_vertex_count > 0)
    {
        auto apply = FEMMASPreconditioner_apply_nonlinear_diag_kernel;
        int  n     = m_nonlinear_vertex_count;
        apply<<<cuda_tool::best_grid_dim(n, apply), cuda_tool::best_block_dim(apply)>>>(
            r, z, nonlinear_diag_inv.cview(), unpartitioned_flags.cview(), n);
        CUDA_TOOL_CHECK(cudaGetLastError());
    }
}

REGISTER_SIM_SYSTEM(FEMMASPreconditioner);
}  // namespace uipc::backend::cuda
