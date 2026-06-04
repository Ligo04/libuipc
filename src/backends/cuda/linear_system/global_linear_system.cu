#include <linear_system/global_linear_system.h>
#include <linear_system/diag_linear_subsystem.h>
#include <linear_system/off_diag_linear_subsystem.h>
#include <uipc/common/range.h>
#include <linear_system/iterative_solver.h>
#include <linear_system/linear_fused_pcg.h>
#include <linear_system/global_preconditioner.h>
#include <linear_system/local_preconditioner.h>
#include <affine_body/abd_linear_subsystem.h>
#include <finite_element/fem_linear_subsystem.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <affine_body/affine_body_body_reporter.h>
#include <fstream>
#include <set>
#include <vector>
#include <string_view>
#include <sim_engine.h>
#include <backends/common/backend_path_tool.h>
#include <Eigen/Sparse>
#include <utils/matrix_market.h>
#include <muda/cub/device/device_reduce.h>
#include <muda/ext/eigen/inverse.h>
#include <muda/launch/launch.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalLinearSystem);

namespace
{
    MUDA_DEVICE void build_agipc_node_to_global_vertex_kernel(int node,
                                                              int abd_node_count,
                                                              int fem_node_count,
                                                              IndexT abd_vertex_offset,
                                                              IndexT abd_vertex_count,
                                                              IndexT fem_vertex_offset,
                                                              IndexT fem_vertex_count,
                                                              IndexT* node_to_global_vertex)
    {
        int node_count = abd_node_count + fem_node_count;
        if(node >= node_count)
            return;

        // ABD coarse nodes represent affine bodies, not surface vertices. Their native
        // 12-DoF q blocks are aggregated by the coarse mapper; use a representative
        // global vertex only for contact/body-id lookup, or -1 when none exists.
        if(node < abd_node_count)
        {
            node_to_global_vertex[node] =
                node < abd_vertex_count ? abd_vertex_offset + node : IndexT{-1};
            return;
        }

        int fem_node = node - abd_node_count;
        node_to_global_vertex[node] =
            fem_node < fem_vertex_count ? fem_vertex_offset + fem_node : IndexT{-1};
    }

    MUDA_DEVICE void clear_agipc_coarse_diag_kernel(int block, Matrix3x3* diag_inv)
    {
        diag_inv[block] = Matrix3x3::Zero();
    }

    MUDA_DEVICE void assemble_agipc_coarse_diag_kernel(
        int triplet_id, muda::CBCOOMatrixViewer<Float, 3, 3> coarse_A, Matrix3x3* diag_inv)
    {
        auto triplet = coarse_A(triplet_id);
        if(triplet.row_index != triplet.col_index)
            return;

        // Block-Jacobi: invert each 3x3 diagonal block of the Galerkin system.
        // A centered affine basis can produce rank-deficient diagonal blocks for
        // small/degenerate aggregates (e.g. a coordinate column that is identically
        // zero), whose naive inverse is NaN/Inf and breaks the coarse PCG. Guard
        // with a finiteness check and fall back to identity (unpreconditioned) for
        // that block so the solve stays robust.
        Matrix3x3 A   = triplet.value;
        Matrix3x3 inv = muda::eigen::inverse(A);
        bool      ok  = true;
        for(int r = 0; r < 3; ++r)
            for(int c = 0; c < 3; ++c)
                ok = ok && isfinite(inv(r, c));
        diag_inv[triplet.row_index] = ok ? inv : Matrix3x3::Identity();
    }

    MUDA_DEVICE void apply_agipc_coarse_diag_kernel(int block,
                                                    muda::CDenseVectorViewer<Float> r,
                                                    muda::DenseVectorViewer<Float> z,
                                                    const IndexT*    converged,
                                                    const Matrix3x3* diag_inv)
    {
        if(converged && *converged != 0)
            return;

        z.segment<3>(block * 3).as_eigen() =
            diag_inv[block] * r.segment<3>(block * 3).as_eigen();
    }

    // Structural checksum of a BCOO matrix's (row,col) index pattern. Used to
    // detect whether the coarse graph topology changed between Newton iterations
    // so the MAS hierarchy can be reused. Order-independent (commutative combine)
    // so triplet reordering by the converter does not spuriously invalidate.
    MUDA_DEVICE void agipc_index_checksum_kernel(int triplet_id,
                                                 muda::CBCOOMatrixViewer<Float, 3, 3> coarse_A,
                                                 unsigned long long* checksum)
    {
        auto               t = coarse_A(triplet_id);
        unsigned long long key =
            (static_cast<unsigned long long>(static_cast<unsigned int>(t.row_index)) * 73856093ULL)
            ^ (static_cast<unsigned long long>(static_cast<unsigned int>(t.col_index))
               * 19349663ULL);
        muda::atomic_add(checksum, key);  // commutative => order-independent
    }
}  // namespace


SizeT GlobalLinearSystem::dof_count() const
{
    UIPC_ASSERT(m_impl.initialized,
                "GlobalLinearSystem::dof_count() is called before GlobalLinearSystem::init().");
    return m_impl.diag_dof_offsets_counts.total_count();
}

void GlobalLinearSystem::compute_gradient(ComputeGradientInfo& info)
{
    m_impl.compute_gradient(info);
}

void GlobalLinearSystem::do_build()
{
    auto& config = world().scene().config();
    auto dump_linear_system_attr = config.find<IndexT>("extras/debug/dump_linear_system");

    m_impl.need_debug_dump =
        dump_linear_system_attr ? dump_linear_system_attr->view()[0] : false;

    auto coarse_enable_attr = config.find<IndexT>("linear_system/coarse/enable");
    m_impl.agipc_coarse_config.enable =
        coarse_enable_attr && coarse_enable_attr->view()[0] != 0;

    if(auto affine_attr = config.find<IndexT>("linear_system/coarse/affine_threshold"))
        m_impl.agipc_coarse_config.affine_threshold =
            std::max<SizeT>(1, static_cast<SizeT>(affine_attr->view()[0]));

    if(auto refine_attr = config.find<IndexT>("linear_system/coarse/fine_refine_iters"))
        m_impl.agipc_coarse_config.fine_refine_iters =
            std::max<SizeT>(0, static_cast<SizeT>(refine_attr->view()[0]));

    if(auto block_attr = config.find<IndexT>("linear_system/coarse/block_size"))
        m_impl.agipc_coarse_config.block_size =
            std::max<SizeT>(32, static_cast<SizeT>(block_attr->view()[0]));

    if(auto component_pass_attr = config.find<IndexT>("linear_system/coarse/component_passes"))
        m_impl.agipc_coarse_config.component_passes =
            std::max<SizeT>(1, static_cast<SizeT>(component_pass_attr->view()[0]));

    if(auto max_agg_attr = config.find<IndexT>("linear_system/coarse/max_aggregate_size"))
        m_impl.agipc_coarse_config.max_aggregate_size =
            std::max<SizeT>(1, static_cast<SizeT>(max_agg_attr->view()[0]));

    if(auto coarse_tol_attr = config.find<Float>("linear_system/coarse/pcg_relative_tol"))
        m_impl.agipc_coarse_config.coarse_pcg_relative_tol = coarse_tol_attr->view()[0];

    if(auto tau_attr = config.find<Float>("linear_system/coarse/green_strain_tau"))
        m_impl.agipc_coarse_config.green_strain_tau = tau_attr->view()[0];

    if(auto max_iter_attr = config.find<IndexT>("linear_system/coarse/max_iterations"))
        m_impl.agipc_coarse_config.coarse_max_iters =
            std::max<SizeT>(1, static_cast<SizeT>(max_iter_attr->view()[0]));

    if(auto interval_attr = config.find<IndexT>("linear_system/coarse/check_interval"))
        m_impl.agipc_coarse_config.coarse_check_interval =
            std::max<SizeT>(1, static_cast<SizeT>(interval_attr->view()[0]));

    if(auto disp_tol_attr = config.find<Float>("linear_system/coarse/displacement_tolerance"))
        m_impl.agipc_coarse_config.displacement_tolerance = disp_tol_attr->view()[0];

    if(auto fine_precond_attr =
           config.find<IndexT>("linear_system/coarse/use_fine_preconditioner_after_coarse"))
        m_impl.agipc_coarse_config.use_fine_preconditioner_after_coarse =
            fine_precond_attr->view()[0] != 0;

    if(auto mas_attr = config.find<IndexT>("linear_system/coarse/use_mas_preconditioner"))
        m_impl.agipc_coarse_config.use_mas_preconditioner = mas_attr->view()[0] != 0;

    m_impl.global_vertex_manager = &require<GlobalVertexManager>();
    m_impl.global_simplicial_surface_manager = find<GlobalSimplicialSurfaceManager>();
    m_impl.global_dytopo_effect_manager   = find<GlobalDyTopoEffectManager>();
    m_impl.affine_body_vertex_reporter    = find<AffineBodyVertexReporter>();
    m_impl.affine_body_body_reporter      = find<AffineBodyBodyReporter>();
    m_impl.finite_element_method          = find<FiniteElementMethod>();
    m_impl.finite_element_body_reporter   = find<FiniteElementBodyReporter>();
    m_impl.finite_element_vertex_reporter = find<FiniteElementVertexReporter>();

    logger::info("GlobalLinearSystem AGIPC coarse config: enable={}, affine_threshold={}, fine_refine_iters={}, block_size={}, component_passes={}, green_strain_tau={}, coarse_max_iters={}, coarse_check_interval={}, displacement_tolerance={}, use_fine_preconditioner_after_coarse={}",
                 m_impl.agipc_coarse_config.enable,
                 m_impl.agipc_coarse_config.affine_threshold,
                 m_impl.agipc_coarse_config.fine_refine_iters,
                 m_impl.agipc_coarse_config.block_size,
                 m_impl.agipc_coarse_config.component_passes,
                 m_impl.agipc_coarse_config.green_strain_tau,
                 m_impl.agipc_coarse_config.coarse_max_iters,
                 m_impl.agipc_coarse_config.coarse_check_interval,
                 m_impl.agipc_coarse_config.displacement_tolerance,
                 m_impl.agipc_coarse_config.use_fine_preconditioner_after_coarse);
}

void GlobalLinearSystem::_dump_A_b()
{
    auto path_tool = BackendPathTool(workspace());
    auto output_folder = path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "debug");
    auto output_path_A = fmt::format("{}A.{}.{}.mtx",
                                     output_folder.string(),
                                     engine().frame(),
                                     engine().newton_iter());
    export_matrix_market(output_path_A, m_impl.bcoo_A.cview());
    logger::info("Dumped global linear system matrix A to {}", output_path_A);

    auto output_path_b = fmt::format("{}b.{}.{}.mtx",
                                     output_folder.string(),
                                     engine().frame(),
                                     engine().newton_iter());
    export_vector_market(output_path_b, m_impl.b.cview());
    logger::info("Dumped global linear system vector b to {}", output_path_b);
}

void GlobalLinearSystem::_dump_x()
{
    auto path_tool = BackendPathTool(workspace());
    auto output_folder = path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "debug");
    export_vector_market(fmt::format("{}x.{}.{}.mtx",
                                     output_folder.string(),
                                     engine().frame(),
                                     engine().newton_iter()),
                         m_impl.x.cview());
}


void GlobalLinearSystem::solve()
{
    m_impl.build_linear_system();

    if(m_impl.empty_system) [[unlikely]]
        return;

    logger::info("GlobalLinearSystem has {} DoFs, Unique Triplet Count: {}",
                 m_impl.b.size(),
                 m_impl.bcoo_A.triplet_count());

    if(m_impl.need_debug_dump) [[unlikely]]
        _dump_A_b();

    m_impl.current_frame       = engine().frame();
    m_impl.current_newton_iter = engine().newton_iter();
    m_impl.solve_linear_system();

    if(m_impl.need_debug_dump) [[unlikely]]
        _dump_x();

    m_impl.distribute_solution();
}

Float GlobalLinearSystem::diag_norm()
{
    m_impl.build_linear_system();
    return m_impl.diag_norm();
}

Float GlobalLinearSystem::mass_norm()
{
    m_impl.build_linear_system();
    return m_impl.mass_norm();
}

void GlobalLinearSystem::Impl::init()
{
    // 1) Init all diag subsystems and off-diag subsystems

    auto diag_subsystem_view     = diag_subsystems.view();
    auto off_diag_subsystem_view = off_diag_subsystems.view();

    {
        // Sort the diag subsystems by their UIDs to ensure the order is consistent
        // ref: https://github.com/spiriMirror/libuipc/issues/271
        std::ranges::sort(diag_subsystem_view,
                          [](const DiagLinearSubsystem* a, const DiagLinearSubsystem* b)
                          { return a->uid() < b->uid(); });
        std::ranges::sort(off_diag_subsystem_view,
                          [](const OffDiagLinearSubsystem* a,
                             const OffDiagLinearSubsystem* b) -> bool
                          { return a->uid() < b->uid(); });
    }


    auto total_count = diag_subsystem_view.size() + off_diag_subsystem_view.size();
    subsystem_infos.resize(total_count);

    // Diag System Always Go First
    auto diag_span = span{subsystem_infos}.subspan(0, diag_subsystem_view.size());
    // Off Diag System Always After Diag System
    auto off_diag_span = span{subsystem_infos}.subspan(diag_subsystem_view.size(),
                                                       off_diag_subsystem_view.size());
    {
        auto offset = 0;
        for(auto i : range(diag_span.size()))
        {
            auto& dst_diag                  = diag_span[i];
            dst_diag.is_diag                = true;
            dst_diag.local_index            = i;
            auto index                      = offset + i;
            dst_diag.index                  = index;
            diag_subsystem_view[i]->m_index = index;
        }

        offset += diag_subsystem_view.size();
        for(auto i : range(off_diag_span.size()))
        {
            auto& dst_off_diag       = off_diag_span[i];
            dst_off_diag.is_diag     = false;
            dst_off_diag.local_index = i;
            dst_off_diag.index       = offset + i;
        }

        for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
            diag_subsystem->init();

        for(auto&& [i, off_diag_subsystem] : enumerate(off_diag_subsystem_view))
            off_diag_subsystem->init();
    }


    // 2) DoF Offsets/Counts
    {
        diag_dof_offsets_counts.resize(diag_subsystem_view.size());
        auto diag_dof_counts = diag_dof_offsets_counts.counts();
        for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
        {
            InitDofExtentInfo info;
            diag_subsystem->report_init_extent(info);
            diag_dof_counts[i] = info.m_dof_count;
        }
        diag_dof_offsets_counts.scan();
        auto diag_dof_offsets = diag_dof_offsets_counts.offsets();
        for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
        {
            InitDofInfo info;
            info.m_dof_offset = diag_dof_offsets[i];
            info.m_dof_count  = diag_dof_counts[i];
            diag_subsystem->receive_init_dof_info(info);
        }
    }
    accuracy_statisfied_flags.resize(diag_subsystem_view.size());

    // 3) Triplet Offsets/Counts
    subsystem_triplet_offsets_counts.resize(total_count);
    off_diag_lr_triplet_counts.resize(off_diag_subsystem_view.size());

    // 4) Preconditioner
    // find out diag systems that don't have preconditioner
    auto local_preconditioner_view = local_preconditioners.view();

    for(auto precond : local_preconditioner_view)
    {
        auto index = precond->m_subsystem->m_index;
        diag_span[index].has_local_preconditioner = true;
    }

    no_precond_diag_subsystem_indices.reserve(diag_span.size());
    for(auto&& [i, diag_info] : enumerate(diag_span))
    {
        if(!diag_info.has_local_preconditioner)
        {
            no_precond_diag_subsystem_indices.push_back(i);
        }
    }

    for(auto precond : local_preconditioner_view)
    {
        precond->init();
    }

    initialized = true;
}

void GlobalLinearSystem::Impl::build_linear_system()
{
    Timer timer{"Build Linear System"};
    empty_system = !_update_subsystem_extent();

    if(empty_system) [[unlikely]]
    {
        logger::warn("The global linear system is empty, skip *assembling, *solving and *solution distributing phase.");
        return;
    }

    _assemble_linear_system();

    converter.ge2sym(triplet_A);
    converter.convert(triplet_A, bcoo_A);

    _assemble_preconditioner();

    logger::info("GlobalLinearSystem has {} DoFs, Unique Triplet Count: {}",
                 b.size(),
                 bcoo_A.triplet_count());
}

bool GlobalLinearSystem::Impl::_update_subsystem_extent()
{
    bool dof_count_changed     = false;
    bool triplet_count_changed = false;

    auto diag_subsystem_view       = diag_subsystems.view();
    auto off_diag_subsystem_view   = off_diag_subsystems.view();
    auto diag_dof_counts           = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets          = diag_dof_offsets_counts.offsets();
    auto subsystem_triplet_counts  = subsystem_triplet_offsets_counts.counts();
    auto subsystem_triplet_offsets = subsystem_triplet_offsets_counts.offsets();

    for(const auto& subsystem_info : subsystem_infos)
    {
        if(subsystem_info.is_diag)
        {
            auto           dof_i          = subsystem_info.local_index;
            auto           triplet_i      = subsystem_info.index;
            auto&          diag_subsystem = diag_subsystem_view[dof_i];
            DiagExtentInfo info;
            diag_subsystem->report_extent(info);

            dof_count_changed |= diag_dof_counts[dof_i] != info.m_dof_count;
            diag_dof_counts[dof_i] = info.m_dof_count;


            triplet_count_changed |= subsystem_triplet_counts[triplet_i] != info.m_block_count;
            subsystem_triplet_counts[triplet_i] = info.m_block_count;
        }
        else
        {
            auto triplet_i = subsystem_info.index;
            auto& off_diag_subsystem = off_diag_subsystem_view[subsystem_info.local_index];
            OffDiagExtentInfo info;
            off_diag_subsystem->report_extent(info);

            auto total_block_count = info.m_lr_block_count + info.m_rl_block_count;

            triplet_count_changed |= subsystem_triplet_counts[triplet_i] != total_block_count;
            subsystem_triplet_counts[triplet_i] = total_block_count;
            off_diag_lr_triplet_counts[subsystem_info.local_index] =
                SizeT2{info.m_lr_block_count, info.m_rl_block_count};
        }
    }

    SizeT total_dof     = 0;
    SizeT total_triplet = 0;

    if(dof_count_changed)
    {
        diag_dof_offsets_counts.scan();
    }
    total_dof = diag_dof_offsets_counts.total_count();
    if(x.capacity() < total_dof)
    {
        auto reserve_count = total_dof * reserve_ratio;
        x.reserve(reserve_count);
        b.reserve(reserve_count);
    }
    auto blocked_dof = total_dof / DoFBlockSize;
    triplet_A.reshape(blocked_dof, blocked_dof);
    x.resize(total_dof);
    b.resize(total_dof);

    if(triplet_count_changed) [[likely]]
    {
        subsystem_triplet_offsets_counts.scan();
    }
    total_triplet = subsystem_triplet_offsets_counts.total_count();

    if(triplet_A.triplet_capacity() < total_triplet)
    {
        auto reserve_count = total_triplet * reserve_ratio;
        triplet_A.reserve_triplets(reserve_count);
        bcoo_A.reserve_triplets(reserve_count);
    }
    triplet_A.resize_triplets(total_triplet);

    if(total_dof == 0 || total_triplet == 0) [[unlikely]]
    {
        return false;
    }

    return true;
}

void GlobalLinearSystem::Impl::_assemble_linear_system()
{
    auto HA = triplet_A.view();

    // Clear and invalidate previous values
    triplet_A.values().fill(Matrix3x3::Zero());
    triplet_A.row_indices().fill(-1);
    triplet_A.col_indices().fill(-1);

    auto B = b.view();
    B.buffer_view().fill(0.0);

    auto diag_subsystem_view     = diag_subsystems.view();
    auto off_diag_subsystem_view = off_diag_subsystems.view();

    auto diag_dof_counts  = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets = diag_dof_offsets_counts.offsets();

    auto subsystem_triplet_counts  = subsystem_triplet_offsets_counts.counts();
    auto subsystem_triplet_offsets = subsystem_triplet_offsets_counts.offsets();

    for(const auto& subsystem_info : subsystem_infos)
    {
        if(subsystem_info.is_diag)
        {
            auto  dof_i          = subsystem_info.local_index;
            auto  triplet_i      = subsystem_info.index;
            auto& diag_subsystem = diag_subsystem_view[dof_i];

            int  dof_offset         = diag_dof_offsets[dof_i];
            int  dof_count          = diag_dof_counts[dof_i];
            int  blocked_dof_offset = dof_offset / DoFBlockSize;
            int  blocked_dof_count  = dof_count / DoFBlockSize;
            int2 ij_offset          = {blocked_dof_offset, blocked_dof_offset};
            int2 ij_count           = {blocked_dof_count, blocked_dof_count};

            DiagInfo info{this};

            info.m_index     = triplet_i;
            info.m_gradients = B.subview(dof_offset, dof_count);
            info.m_hessians  = HA.subview(subsystem_triplet_offsets[triplet_i],
                                         subsystem_triplet_counts[triplet_i])
                                  .submatrix(ij_offset, ij_count);

            diag_subsystem->assemble(info);
        }
        else
        {
            auto triplet_i   = subsystem_info.index;
            auto local_index = subsystem_info.local_index;
            auto& off_diag_subsystem = off_diag_subsystem_view[subsystem_info.local_index];
            auto& l_diag_index = off_diag_subsystem->m_l->m_index;
            auto& r_diag_index = off_diag_subsystem->m_r->m_index;


            int l_blocked_dof_offset = diag_dof_offsets[l_diag_index] / DoFBlockSize;
            int l_blocked_dof_count = diag_dof_counts[l_diag_index] / DoFBlockSize;

            int r_blocked_dof_offset = diag_dof_offsets[r_diag_index] / DoFBlockSize;
            int r_blocked_dof_count = diag_dof_counts[r_diag_index] / DoFBlockSize;

            auto lr_triplet_offset = subsystem_triplet_offsets[triplet_i];
            auto lr_triplet_count  = off_diag_lr_triplet_counts[local_index].x;
            auto rl_triplet_offset = lr_triplet_offset + lr_triplet_count;
            auto rl_triplet_count  = off_diag_lr_triplet_counts[local_index].y;

            OffDiagInfo info{this};
            info.m_index = triplet_i;

            info.m_lr_hessian =
                HA.subview(lr_triplet_offset, lr_triplet_count)
                    .submatrix(int2{l_blocked_dof_offset, r_blocked_dof_offset},
                               int2{l_blocked_dof_count, r_blocked_dof_count});

            info.m_rl_hessian =
                HA.subview(rl_triplet_offset, rl_triplet_count)
                    .submatrix(int2{r_blocked_dof_offset, l_blocked_dof_offset},
                               int2{r_blocked_dof_count, l_blocked_dof_count});

            // logger::info("rl_offset: {}, lr_offset: {}", rl_triplet_offset, lr_triplet_offset);

            off_diag_subsystem->assemble(info);
        }
    }
}

void GlobalLinearSystem::Impl::_assemble_preconditioner()
{
    if(global_preconditioner)
    {
        GlobalPreconditionerAssemblyInfo info{this};
        global_preconditioner->assemble(info);
    }

    for(auto&& preconditioner : local_preconditioners.view())
    {
        LocalPreconditionerAssemblyInfo info{this, preconditioner->m_subsystem->m_index};
        preconditioner->assemble(info);
    }
}


void GlobalLinearSystem::Impl::rebuild_agipc_node_to_global_vertex_map()
{
    agipc_abd_node_count = 0;
    agipc_fem_node_count = 0;

    for(auto&& diag_subsystem : diag_subsystems.view())
    {
        auto index = diag_subsystem->m_index;
        if(dynamic_cast<ABDLinearSubsystem*>(diag_subsystem) != nullptr)
            agipc_abd_node_count = diag_dof_offsets_counts.counts()[index] / 12;
        else if(dynamic_cast<FEMLinearSubsystem*>(diag_subsystem) != nullptr)
            agipc_fem_node_count = diag_dof_offsets_counts.counts()[index] / 3;
    }

    SizeT node_count = agipc_abd_node_count + agipc_fem_node_count;
    agipc_node_to_global_vertex.resize(node_count);
    if(node_count == 0)
        return;

    IndexT abd_vertex_offset =
        affine_body_vertex_reporter ? affine_body_vertex_reporter->vertex_offset() : 0;
    IndexT abd_vertex_count =
        affine_body_vertex_reporter ? affine_body_vertex_reporter->vertex_count() : 0;
    IndexT fem_vertex_offset = finite_element_vertex_reporter ?
                                   finite_element_vertex_reporter->vertex_offset() :
                                   0;
    IndexT fem_vertex_count  = finite_element_vertex_reporter ?
                                   finite_element_vertex_reporter->vertex_count() :
                                   0;

    constexpr int block_size = 256;
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(static_cast<int>(node_count),
               [abd_node_count = static_cast<int>(agipc_abd_node_count),
                fem_node_count = static_cast<int>(agipc_fem_node_count),
                abd_vertex_offset,
                abd_vertex_count,
                fem_vertex_offset,
                fem_vertex_count,
                node_to_global_vertex =
                    agipc_node_to_global_vertex.data()] MUDA_DEVICE(int node) mutable
               {
                   build_agipc_node_to_global_vertex_kernel(node,
                                                            abd_node_count,
                                                            fem_node_count,
                                                            abd_vertex_offset,
                                                            abd_vertex_count,
                                                            fem_vertex_offset,
                                                            fem_vertex_count,
                                                            node_to_global_vertex);
               });
}

void GlobalLinearSystem::Impl::solve_linear_system()
{
    Timer timer{"Solve Linear System"};
    if(!iterative_solver)
        return;

    auto solve_with_views = [&](muda::DenseVectorView<Float>  dst_x,
                                muda::CDenseVectorView<Float> rhs_b,
                                bool                          use_initial_guess,
                                SizeT                         max_iter_override,
                                bool accuracy_check_enabled)
    {
        SolvingInfo info{this};
        info.m_b                      = rhs_b;
        info.m_x                      = dst_x;
        info.m_use_initial_guess      = use_initial_guess;
        info.m_max_iter_override      = max_iter_override;
        info.m_accuracy_check_enabled = accuracy_check_enabled;
        iterative_solver->solve(info);
        return info.m_iter_count;
    };

    if(agipc_coarse_config.enable)
    {
        rebuild_agipc_node_to_global_vertex_map();

        AGIPCCoarseLinearSystem::BuildInfo build_info;
        build_info.fine_A            = bcoo_A.cview();
        build_info.fine_b            = b.cview();
        build_info.current_positions = global_vertex_manager->positions();
        build_info.rest_positions    = global_vertex_manager->rest_positions();
        build_info.node_to_global_vertex = agipc_node_to_global_vertex.view();
        if(global_dytopo_effect_manager)
            build_info.contact_hessian = global_dytopo_effect_manager->hessians();
        if(finite_element_method)
        {
            build_info.fem_tets    = finite_element_method->tets();
            build_info.fem_Dm_invs = finite_element_method->Dm3x3_invs();
            build_info.fem_current_positions = finite_element_method->xs();
            build_info.fem_rest_positions    = finite_element_method->x_bars();
        }
        if(global_simplicial_surface_manager)
        {
            build_info.surface_edges = global_simplicial_surface_manager->surf_edges();
            build_info.surface_triangles =
                global_simplicial_surface_manager->surf_triangles();
        }
        if(global_vertex_manager)
            build_info.global_vertex_body_ids = global_vertex_manager->body_ids();
        build_info.abd_global_body_offset =
            affine_body_body_reporter ? affine_body_body_reporter->body_offset() : 0;
        build_info.fem_global_body_offset =
            finite_element_body_reporter ? finite_element_body_reporter->body_offset() : 0;
        build_info.fem_global_body_count =
            finite_element_body_reporter ? finite_element_body_reporter->body_count() : 0;
        build_info.fem_global_vertex_offset =
            finite_element_vertex_reporter ?
                finite_element_vertex_reporter->vertex_offset() :
                0;
        build_info.abd_node_count = agipc_abd_node_count;
        build_info.fem_node_count = agipc_fem_node_count;
        build_info.frame          = current_frame;
        build_info.newton_iter    = current_newton_iter;
        agipc_coarse_system.build(build_info, agipc_coarse_config);

        // Degenerate short-circuit: if no edge actually collapsed, the coarse
        // system has exactly as many blocks as the fine system (ABD blocks map
        // 1:1, every FEM node is its own 3-DoF coarse node). Running the full
        // coarse pipeline + fine refinement then costs strictly more than a
        // single fine solve with zero DoF reduction (e.g. rigid/ABD-only scenes
        // where all coupling edges are contact-protected). Fall back to baseline.
        SizeT agipc_fine_block_count = agipc_abd_node_count * 4 + agipc_fem_node_count;
        bool agipc_no_reduction =
            agipc_coarse_system.built()
            && agipc_coarse_system.coarse_block_count() >= agipc_fine_block_count;
        if(agipc_no_reduction)
        {
            logger::info(
                "AGIPC coarse build produced no DoF reduction (coarse_blocks={} >= "
                "fine_blocks={}); using the baseline fine solve for this step.",
                agipc_coarse_system.coarse_block_count(),
                agipc_fine_block_count);
        }

        if(agipc_coarse_system.built() && !agipc_no_reduction)
        {
            auto* fused_solver = dynamic_cast<LinearFusedPCG*>(iterative_solver.view());
            if(fused_solver == nullptr)
            {
                // The chunked coarse PCG state machine lives on LinearFusedPCG.
                // Any other solver (e.g. linear_pcg) cannot drive it, so degrade
                // gracefully to the ordinary fine solve instead of aborting.
                logger::warn(
                    "AGIPC coarse solve is enabled but linear_system/solver is not "
                    "fused_pcg; falling back to the ordinary fine solve for this step.");
            }
            else
            {
                agipc_solving_coarse = true;
                assemble_agipc_coarse_preconditioner();
                fused_solver->begin_coarse_solve(agipc_coarse_system.solution().view(),
                                                 agipc_coarse_system.rhs().cview(),
                                                 agipc_coarse_config.coarse_pcg_relative_tol);

                SizeT       coarse_iter      = 0;
                Float       coarse_probe_inf = 0.0;
                const SizeT coarse_max_iters =
                    std::max<SizeT>(SizeT{1}, agipc_coarse_config.coarse_max_iters);
                const SizeT coarse_check_interval =
                    std::max<SizeT>(SizeT{1}, agipc_coarse_config.coarse_check_interval);
                std::string_view coarse_stop_reason = "max_iterations";
                while(coarse_iter < coarse_max_iters)
                {
                    SizeT remaining = coarse_max_iters - coarse_iter;
                    SizeT chunk    = std::min(remaining, coarse_check_interval);
                    SizeT advanced = fused_solver->coarse_solve_iterations(
                        agipc_coarse_system.solution().view(), chunk);
                    coarse_iter += advanced;

                    if(fused_solver->coarse_solve_converged())
                    {
                        coarse_stop_reason = "residual";
                        break;
                    }

                    if(advanced == 0)
                    {
                        coarse_stop_reason = "numerical_failure";
                        break;
                    }

                    if(coarse_iter % coarse_check_interval == 0 || coarse_iter >= coarse_max_iters)
                    {
                        agipc_coarse_system.prolongate_to(x.view());
                        if(fused_solver->check_displacement_converged(
                               x.cview(), agipc_coarse_config.displacement_tolerance, &coarse_probe_inf))
                        {
                            coarse_stop_reason = "prolongated_displacement";
                            break;
                        }
                    }
                }
                agipc_solving_coarse = false;

                x.fill(0);
                agipc_coarse_system.prolongate_to(x.view());

                SizeT refine_iter   = 0;
                agipc_refining_fine = true;
                // Fine refinement budget: if fine_refine_iters>0 use it as a fixed
                // cap; otherwise scale adaptively with how hard the coarse solve was
                // (a poorly-converged coarse direction needs more correction), as in
                // the reference: min(300, max(5, (coarse_iter+2)/3)).
                SizeT refine_budget =
                    agipc_coarse_config.fine_refine_iters > 0 ?
                        agipc_coarse_config.fine_refine_iters :
                        std::min<SizeT>(300, std::max<SizeT>(5, (coarse_iter + 2) / 3));
                if(refine_budget > 0)
                {
                    refine_iter =
                        solve_with_views(x.view(), b.cview(), true, refine_budget, true);
                }
                agipc_refining_fine = false;

                logger::info("AGIPC Newton linear solve: coarse_iter={}, stop_reason={}, probe_inf={}, fine_refine_iter={}, coarse_blocks={}, coarse_triplets={}",
                             coarse_iter,
                             coarse_stop_reason,
                             coarse_probe_inf,
                             refine_iter,
                             agipc_coarse_system.coarse_block_count(),
                             agipc_coarse_system.hessian().triplet_count());
                return;
            }  // else (fused_solver != nullptr)
        }

        logger::warn("AGIPC coarse solve requested but coarse system was empty or unsupported; falling back to original fused PCG.");
    }

    SizeT fine_iter = solve_with_views(x.view(), b.cview(), false, 0, true);
    logger::info("Iterative linear solver iteration count: {}", fine_iter);
}

void GlobalLinearSystem::Impl::distribute_solution()
{
    auto diag_subsystem_view = diag_subsystems.view();
    auto diag_dof_counts     = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets    = diag_dof_offsets_counts.offsets();

    // distribute the solution to all diag subsystems
    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystems.view()))
    {
        SolutionInfo info{this};
        info.m_solution = x.view().subview(diag_dof_offsets[i], diag_dof_counts[i]);
        diag_subsystem->retrieve_solution(info);
    }
}

void GlobalLinearSystem::Impl::apply_preconditioner(muda::DenseVectorView<Float> z,
                                                    muda::CDenseVectorView<Float> r,
                                                    muda::CVarView<IndexT> converged)
{
    if(agipc_solving_coarse)
    {
        apply_agipc_coarse_preconditioner(z, r, converged);
        return;
    }

    if(agipc_refining_fine && !agipc_coarse_config.use_fine_preconditioner_after_coarse)
    {
        // Upstream AGIPC exposes this switch for ablation: after a coarse solve,
        // fine refinement can intentionally run with identity preconditioning.
        z.buffer_view().copy_from(r.buffer_view());
        return;
    }

    apply_fine_preconditioner(z, r, converged);
}

void GlobalLinearSystem::Impl::assemble_agipc_coarse_preconditioner()
{
    constexpr int block_size = 256;
    int block_count = static_cast<int>(agipc_coarse_system.coarse_block_count());
    agipc_coarse_diag_inv.resize(block_count);
    if(block_count == 0)
        return;

    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(block_count,
               [diag_inv = agipc_coarse_diag_inv.data()] MUDA_DEVICE(int block) mutable
               { clear_agipc_coarse_diag_kernel(block, diag_inv); });

    int triplet_count = agipc_coarse_system.hessian().triplet_count();
    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(triplet_count,
               [coarse_A = agipc_coarse_system.hessian().cviewer().name("agipc_coarse_A"),
                diag_inv = agipc_coarse_diag_inv.data()] MUDA_DEVICE(int triplet_id) mutable {
                   assemble_agipc_coarse_diag_kernel(triplet_id, coarse_A, diag_inv);
               });

    if(agipc_coarse_config.use_mas_preconditioner)
        assemble_agipc_coarse_mas_preconditioner();
}

void GlobalLinearSystem::Impl::assemble_agipc_coarse_mas_preconditioner()
{
    agipc_coarse_mas_ready = false;
    int block_count = static_cast<int>(agipc_coarse_system.coarse_block_count());
    auto& coarse_A = agipc_coarse_system.hessian();
    int   tcount   = coarse_A.triplet_count();
    if(block_count < 1 || tcount < 1)
        return;

    // Structural checksum to decide whether the cached MAS hierarchy is reusable.
    agipc_mas_checksum_var = 0ull;
    muda::ParallelFor(256)
        .file_line(__FILE__, __LINE__)
        .apply(tcount,
               [coarse_A = coarse_A.cviewer().name("coarse_A"),
                checksum = agipc_mas_checksum_var.data()] MUDA_DEVICE(int t) mutable
               { agipc_index_checksum_kernel(t, coarse_A, checksum); });
    unsigned long long checksum = agipc_mas_checksum_var;

    bool structure_unchanged =
        agipc_coarse_mas.is_initialized()
        && agipc_mas_cached_blocks == static_cast<SizeT>(block_count)
        && agipc_mas_cached_triplets == static_cast<SizeT>(tcount)
        && agipc_mas_cached_checksum == checksum;

    if(!structure_unchanged)
    {
        // Rebuild the graph hierarchy: host neighbor CSR + sequential BANKSIZE
        // partition -> init_neighbor + init_matrix (the expensive multilevel build).
        std::vector<int> h_rows(tcount), h_cols(tcount);
        coarse_A.row_indices().copy_to(h_rows.data());
        coarse_A.col_indices().copy_to(h_cols.data());

        std::vector<std::set<unsigned int>> nbr(block_count);
        for(int t = 0; t < tcount; ++t)
        {
            int a = h_rows[t], b = h_cols[t];
            if(a < 0 || b < 0 || a >= block_count || b >= block_count || a == b)
                continue;
            nbr[a].insert(static_cast<unsigned int>(b));
            nbr[b].insert(static_cast<unsigned int>(a));
        }

        std::vector<unsigned int> h_neighbor_list;
        std::vector<unsigned int> h_neighbor_start(block_count, 0);
        std::vector<unsigned int> h_neighbor_num(block_count, 0);
        for(int i = 0; i < block_count; ++i)
        {
            h_neighbor_start[i] = static_cast<unsigned int>(h_neighbor_list.size());
            h_neighbor_num[i] = static_cast<unsigned int>(nbr[i].size());
            for(auto n : nbr[i])
                h_neighbor_list.push_back(n);
        }

        constexpr int BANK          = MASPreconditionerEngine::BANKSIZE;
        int           part_map_size = ((block_count + BANK - 1) / BANK) * BANK;
        std::vector<int> h_part_to_real(part_map_size, -1);
        std::vector<int> h_real_to_part(block_count, -1);
        for(int i = 0; i < block_count; ++i)
        {
            h_part_to_real[i] = i;
            h_real_to_part[i] = i;
        }

        agipc_coarse_mas.init_neighbor(block_count,
                                       static_cast<int>(h_neighbor_list.size()),
                                       part_map_size,
                                       h_neighbor_list,
                                       h_neighbor_start,
                                       h_neighbor_num,
                                       h_part_to_real,
                                       h_real_to_part);
        agipc_coarse_mas.init_matrix();
        if(!agipc_coarse_mas.is_initialized())
            return;

        agipc_coarse_mas_indices.resize(tcount);
        muda::ParallelFor(256)
            .file_line(__FILE__, __LINE__)
            .apply(tcount,
                   [idx = agipc_coarse_mas_indices.viewer().name("idx")] MUDA_DEVICE(int i) mutable
                   { idx(i) = static_cast<uint32_t>(i); });

        agipc_mas_cached_blocks   = static_cast<SizeT>(block_count);
        agipc_mas_cached_triplets = static_cast<SizeT>(tcount);
        agipc_mas_cached_checksum = checksum;
    }

    // Always re-feed matrix values (they change every Newton iteration even when
    // the structure is identical). This assembles + inverts the cluster matrices.
    agipc_coarse_mas.set_preconditioner(coarse_A.values(),
                                        coarse_A.row_indices(),
                                        coarse_A.col_indices(),
                                        agipc_coarse_mas_indices.view(),
                                        0,
                                        0);
    agipc_coarse_mas_ready = true;
}

void GlobalLinearSystem::Impl::apply_agipc_coarse_preconditioner(
    muda::DenseVectorView<Float> z, muda::CDenseVectorView<Float> r, muda::CVarView<IndexT> converged)
{
    if(agipc_coarse_config.use_mas_preconditioner && agipc_coarse_mas_ready)
    {
        agipc_coarse_mas.apply(r, z, converged);
        return;
    }

    constexpr int block_size  = 256;
    int           block_count = static_cast<int>(agipc_coarse_diag_inv.size());
    UIPC_ASSERT(r.size() == static_cast<SizeT>(block_count * 3),
                "AGIPC coarse preconditioner size mismatch: r={}, diag_blocks={}.",
                r.size(),
                block_count);

    muda::ParallelFor(block_size)
        .file_line(__FILE__, __LINE__)
        .apply(block_count,
               [r         = r.cviewer().name("r"),
                z         = z.viewer().name("z"),
                converged = converged.data(),
                diag_inv = agipc_coarse_diag_inv.data()] MUDA_DEVICE(int block) mutable {
                   apply_agipc_coarse_diag_kernel(block, r, z, converged, diag_inv);
               });
}

void GlobalLinearSystem::Impl::apply_fine_preconditioner(muda::DenseVectorView<Float> z,
                                                         muda::CDenseVectorView<Float> r,
                                                         muda::CVarView<IndexT> converged)
{
    (void)converged;
    auto diag_dof_counts  = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets = diag_dof_offsets_counts.offsets();

    if(global_preconditioner)
    {
        ApplyPreconditionerInfo info{this};
        info.m_z         = z;
        info.m_r         = r;
        info.m_converged = converged;
        global_preconditioner->apply(info);
    }

    for(auto& preconditioner : local_preconditioners.view())
    {
        ApplyPreconditionerInfo info{this};
        auto                    index  = preconditioner->m_subsystem->m_index;
        auto                    offset = diag_dof_offsets[index];
        auto                    count  = diag_dof_counts[index];
        info.m_z                       = z.subview(offset, count);
        info.m_r                       = r.subview(offset, count);
        info.m_converged               = converged;
        preconditioner->apply(info);
    }

    if(!global_preconditioner)
    {
        // For diag subsystems without local preconditioner, just copy r to z
        for(auto i : no_precond_diag_subsystem_indices)
        {
            auto offset = diag_dof_offsets[i];
            auto count  = diag_dof_counts[i];
            auto z_sub  = z.subview(offset, count);
            auto r_sub  = r.subview(offset, count);
            z_sub.buffer_view().copy_from(r_sub.buffer_view());
        }
    }
}

void GlobalLinearSystem::Impl::spmv(Float                         a,
                                    muda::CDenseVectorView<Float> x,
                                    Float                         b,
                                    muda::DenseVectorView<Float>  y)
{
    if(agipc_solving_coarse)
    {
        spmver.rbk_sym_spmv(a, agipc_coarse_system.hessian().cview(), x, b, y);
        return;
    }

    spmver.rbk_sym_spmv(a, bcoo_A.cview(), x, b, y);

    // Just some debug options
    //  * spmver.sym_spmv(a, bcoo_A.cview(), x, b, y);      // Slightly slower
    //  * spmver.cpu_sym_spmv(a, bcoo_A.cview(), x, b, y);  // Much slower
}

void GlobalLinearSystem::Impl::spmv_dot(muda::CDenseVectorView<Float> x,
                                        muda::DenseVectorView<Float>  y,
                                        muda::VarView<Float>          d_dot)
{
    if(agipc_solving_coarse)
    {
        spmver.rbk_sym_spmv_dot(1.0, agipc_coarse_system.hessian().cview(), x, 0.0, y, d_dot);
        return;
    }

    spmver.rbk_sym_spmv_dot(1.0, bcoo_A.cview(), x, 0.0, y, d_dot);
}

bool GlobalLinearSystem::Impl::accuracy_statisfied(muda::DenseVectorView<Float> r)
{
    if(agipc_solving_coarse)
        return false;

    auto diag_dof_counts  = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets = diag_dof_offsets_counts.offsets();

    for(auto&& [i, diag_subsystems] : enumerate(diag_subsystems.view()))
    {
        AccuracyInfo info{this};
        info.m_r = r.subview(diag_dof_offsets[i], diag_dof_counts[i]);
        diag_subsystems->accuracy_check(info);

        accuracy_statisfied_flags[i] = info.m_statisfied ? 1 : 0;
    }

    return std::ranges::all_of(accuracy_statisfied_flags,
                               [](bool flag) { return flag; });
}

void GlobalLinearSystem::Impl::compute_gradient(ComputeGradientInfo& info)
{
    auto diag_subsystem_view = diag_subsystems.view();

    // report extent first (gradient only mode)
    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
    {
        DiagExtentInfo diag_info;
        diag_info.m_gradient_only   = true;
        diag_info.m_component_flags = info.m_flags;
        diag_subsystem->report_extent(diag_info);
    }

    // assemble gradient only
    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
    {
        DiagInfo diag_info{this};
        diag_info.m_index           = diag_subsystem->m_index;
        diag_info.m_gradients       = info.m_gradients;
        diag_info.m_hessians        = TripletMatrixView{};
        diag_info.m_gradient_only   = true;
        diag_info.m_component_flags = info.m_flags;
        diag_subsystem->assemble(diag_info);
    }
}

Float GlobalLinearSystem::Impl::diag_norm()
{
    Float norm = 0;

    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystems.view()))
    {
        DiagNormInfo info(this, diag_subsystem->m_index);
        norm = max(norm, diag_subsystem->diag_norm(info));
    }

    return norm;
}

Float GlobalLinearSystem::Impl::mass_norm()
{
    Float norm = 0;

    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystems.view()))
    {
        DiagNormInfo info(this, diag_subsystem->m_index);
        norm = max(norm, diag_subsystem->mass_norm(info));
    }

    return norm;
}

void GlobalLinearSystem::DiagExtentInfo::extent(SizeT hessian_block_count, SizeT dof_count) noexcept
{
    m_block_count = hessian_block_count;
    UIPC_ASSERT(dof_count % DoFBlockSize == 0,
                "dof_count must be multiple of {}, yours {}.",
                DoFBlockSize,
                dof_count);
    m_dof_count = dof_count;
}

void GlobalLinearSystem::OffDiagExtentInfo::extent(SizeT lr_hessian_block_count,
                                                   SizeT rl_hassian_block_count) noexcept
{
    m_lr_block_count = lr_hessian_block_count;
    m_rl_block_count = rl_hassian_block_count;
}
auto GlobalLinearSystem::AssemblyInfo::A() const -> CBCOOMatrixView
{
    return m_impl->bcoo_A.cview();
}

SizeT GlobalLinearSystem::LocalPreconditionerAssemblyInfo::dof_offset() const
{
    auto diag_dof_offsets = m_impl->diag_dof_offsets_counts.offsets();
    return diag_dof_offsets[m_index];
}

SizeT GlobalLinearSystem::LocalPreconditionerAssemblyInfo::dof_count() const
{
    auto diag_dof_counts = m_impl->diag_dof_offsets_counts.counts();
    return diag_dof_counts[m_index];
}
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
void GlobalLinearSystem::add_subsystem(DiagLinearSubsystem* subsystem)
{
    check_state(SimEngineState::BuildSystems, "add_subsystem()");
    UIPC_ASSERT(subsystem != nullptr, "The subsystem should not be nullptr.");
    m_impl.diag_subsystems.register_sim_system(*subsystem);
}

void GlobalLinearSystem::add_subsystem(OffDiagLinearSubsystem* subsystem)
{
    check_state(SimEngineState::BuildSystems, "add_subsystem()");
    m_impl.off_diag_subsystems.register_sim_system(*subsystem);
}

void GlobalLinearSystem::add_solver(IterativeSolver* solver)
{
    check_state(SimEngineState::BuildSystems, "add_solver()");
    UIPC_ASSERT(solver != nullptr, "The solver should not be nullptr.");
    m_impl.iterative_solver.register_sim_system(*solver);
}

void GlobalLinearSystem::add_preconditioner(LocalPreconditioner* preconditioner)
{
    check_state(SimEngineState::BuildSystems, "add_preconditioner()");
    UIPC_ASSERT(preconditioner != nullptr, "The preconditioner should not be nullptr.");
    m_impl.local_preconditioners.register_sim_system(*preconditioner);
}

void GlobalLinearSystem::add_preconditioner(GlobalPreconditioner* preconditioner)
{
    check_state(SimEngineState::BuildSystems, "add_preconditioner()");
    UIPC_ASSERT(preconditioner != nullptr, "The preconditioner should not be nullptr.");
    m_impl.global_preconditioner.register_sim_system(*preconditioner);
}

void GlobalLinearSystem::init()
{
    m_impl.init();
}

void GlobalLinearSystem::ComputeGradientInfo::flags(ComponentFlags flags) noexcept
{
    m_flags = flags;
}

void GlobalLinearSystem::ComputeGradientInfo::buffer_view(muda::DenseVectorView<Float> grad) noexcept
{
    m_gradients = grad;
}
}  // namespace uipc::backend::cuda
