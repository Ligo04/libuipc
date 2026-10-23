#include <nonlinear_system/mas_pncg_solver.h>
#include <nonlinear_system/mas_pncg_math.h>
#include <nonlinear_system/mas_pncg_kernels.h>
#include <nonlinear_system/mas_pncg_fused.h>
#include <nonlinear_system/mas_pncg_local_ccd.h>
#include <linear_system/global_linear_system.h>
#include <finite_element/fem_linear_subsystem.h>
#include <finite_element/fem_mas_preconditioner.h>
#include <finite_element/finite_element_vertex_reporter.h>
#include <finite_element/finite_element_extra_constitution.h>
#include <affine_body/affine_body_dynamics.h>
#include <contact_system/simplex_normal_contact.h>
#include <contact_system/vertex_half_plane_normal_contact.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <implicit_geometry/half_plane.h>
#include <implicit_geometry/half_plane_vertex_reporter.h>
#include <sim_engine.h>
#include <cmath>
#include <limits>
#include <algorithm>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(MASPNCGSolver);
MASPNCGSolver::~MASPNCGSolver() = default;

namespace
{
    template <typename T>
    void resize_output(cuda_tool::DeviceBuffer<T>& buffer, SizeT n)
    {
        if(buffer.capacity() < n)
            buffer.reserve_discard(n + n / 2);
        buffer.resize_discard(n);
    }

    void increment(Json& stats, const char* key, SizeT count = 1)
    {
        stats[key] = stats.at(key).get<SizeT>() + count;
    }
}  // namespace

void MASPNCGSolver::do_build()
{
    auto& config = world().scene().config();
    if(config.find<std::string>("nonlinear_system/solver")->view()[0] != "mas_pncg")
        throw SimSystemException("MASPNCGSolver: nonlinear_system/solver != mas_pncg, disabled.");
    if(find<AffineBodyDynamics>())
        throw SimEngineException("MAS-PNCG currently supports FEM coordinates; ABD requires its separate affine-coordinate extension.");
    m_system    = &require<GlobalLinearSystem>();
    m_fem       = &require<FiniteElementMethod>();
    m_subsystem = &require<FEMLinearSubsystem>();
    m_mas       = &require<FEMMASPreconditioner>();
    m_vertices  = &require<FiniteElementVertexReporter>();
    m_simplex   = find<SimplexNormalContact>(QueryOptions{.exact = false});
    m_planes = find<VertexHalfPlaneNormalContact>(QueryOptions{.exact = false});
    auto index = [&](const char* name)
    { return config.find<IndexT>(name)->view()[0]; };
    auto value = [&](const char* name)
    { return config.find<Float>(name)->view()[0]; };
    m_max_iter     = index("nonlinear_system/mas_pncg/max_iter");
    m_top_k        = index("nonlinear_system/mas_pncg/top_k");
    m_max_age      = index("nonlinear_system/mas_pncg/max_base_age");
    m_rtol         = value("nonlinear_system/mas_pncg/gradient_rtol");
    m_atol         = value("nonlinear_system/mas_pncg/gradient_atol");
    m_velocity_tol = value("nonlinear_system/mas_pncg/velocity_tol");
    m_iteration_policy =
        config.find<std::string>("nonlinear_system/mas_pncg/iteration_policy")->view()[0];
    m_ccd_mode =
        config.find<std::string>("nonlinear_system/mas_pncg/ccd_mode")->view()[0];
    if(uses_local_ccd())
    {
        if(m_iteration_policy != "paper")
            throw SimEngineException("Local conservative CCD requires MAS-PNCG paper policy.");
        m_global_vertices = &require<GlobalVertexManager>();
        m_surface         = &require<GlobalSimplicialSurfaceManager>();
        m_contact_manager = find<GlobalContactManager>();
        m_trajectory = find<SimplexTrajectoryFilter>(QueryOptions{.exact = false});
        m_half_planes    = find<HalfPlane>();
        m_plane_vertices = find<HalfPlaneVertexReporter>();
    }
    m_preconditioned_tol = value("nonlinear_system/mas_pncg/preconditioned_gradient_tol");
    m_restart_threshold = value("nonlinear_system/mas_pncg/restart_threshold");
    m_cos_rotation = std::cos(value("nonlinear_system/mas_pncg/rotation_deg")
                              * std::acos(-1.0) / 180.0);
    m_update_mode =
        config.find<std::string>("nonlinear_system/mas_pncg/update_mode")->view()[0];
    m_use_fused = index("nonlinear_system/mas_pncg/use_fused_kernels") != 0;
    m_use_graph = m_use_fused && index("nonlinear_system/mas_pncg/use_cuda_graph") != 0;
    if(m_use_fused)
        m_fused = std::make_unique<pncg::FusedOperations>();
    on_init_scene([this] { validate_capabilities(); });
    logger::info("Nonlinear solver: MAS-PNCG ({}, {} policy, FEM MAS, {} CCD); Newton/PCG settings are inactive.",
                 m_update_mode,
                 m_iteration_policy,
                 m_ccd_mode);
}

void MASPNCGSolver::begin_frame()
{
    if(m_failed)
        throw SimEngineException("MAS-PNCG cannot advance an incomplete frame; create a fresh Engine/World from a valid state.");
    m_stats     = {{"nonlinear_solver", "mas_pncg"},
                   {"update_mode", m_update_mode},
                   {"iteration_policy", m_iteration_policy},
                   {"convergence_criterion", "not_converged"},
                   {"independent_residual_converged", false},
                   {"preconditioned_gradient_norm", 0.0},
                   {"preconditioned_gradient_tolerance", m_preconditioned_tol},
                   {"convergence_checked_on_rebuild", false},
                   {"orthogonality_restarts", 0},
                   {"numerical_retries", 0},
                   {"restart_ratio", 0.0},
                   {"energy_increase_steps", 0},
                   {"nonlinear_iterations", 0},
                   {"accepted_steps", 0},
                   {"base_rebuilds", 0},
                   {"base_age", 0},
                   {"restarts", 0},
                   {"one_d_steps", 0},
                   {"two_d_steps", 0},
                   {"truncated_columns", 0},
                   {"direct_local_fallbacks", 0},
                   {"contact_rebuild_fallbacks", 0},
                   {"normal_contacts", 0},
                   {"ee_curvature_contacts", 0},
                   {"mollified_contacts", 0},
                   {"peak_mollified_contacts", 0},
                   {"contact_update_columns", 0},
                   {"rejected_trials", 0},
                   {"nonfinite_trials", 0},
                   {"ccd_mode", m_ccd_mode},
                   {"local_ccd_steps", 0},
                   {"local_ccd_candidate_pairs", 0},
                   {"local_ccd_pair_fallbacks", 0},
                   {"local_ccd_clamped_vertices", 0},
                   {"local_ccd_non_descent_steps", 0},
                   {"local_ccd_min_fraction", 1.0},
                   {"local_ccd_last_max_fraction", 1.0},
                   {"local_ccd_last_actual_slope", 0.0},
                   {"local_ccd_global_guard_reductions", 0},
                   {"local_ccd_min_global_guard_fraction", 1.0},
                   {"gradient_initial", 0.0},
                   {"gradient_final", 0.0},
                   {"max_local_residual_ratio", 0.0}};
    m_need_base = true;
    if(m_fused)
    {
        m_frame_captures  = m_fused->graph_captures();
        m_frame_replays   = m_fused->graph_replays();
        m_frame_fallbacks = m_fused->graph_fallbacks();
    }
    m_have_history         = false;
    m_reference_ready      = false;
    m_retry_used           = false;
    m_age                  = 0;
    m_started_with_rebuild = m_pending_restart = m_stop_after_accept =
        m_zero_gradient                        = false;
    m_preconditioned_norm                      = 0;
    m_dt            = world().scene().config().find<Float>("dt")->view()[0];
    m_vertex_offset = m_vertices->vertex_offset();
    m_dof_offset    = m_subsystem->dof_offset();
    if(m_dof_offset != 0 || m_subsystem->dof_count() != m_system->dof_count())
        throw SimEngineException("MAS-PNCG requires an exclusively FEM generalized layout.");

    auto&               fem = m_fem->m_impl;
    std::vector<Float>  lengths(fem.xs.size(), 1.0), masses(fem.xs.size());
    std::vector<IndexT> fixed(fem.xs.size()), dynamic(fem.xs.size());
    m_fem->masses().copy_to(masses.data());
    m_fem->is_fixed().copy_to(fixed.data());
    m_fem->is_dynamic().copy_to(dynamic.data());
    cuda_tool::wait_device();
    Vector3 low  = Vector3::Constant(std::numeric_limits<Float>::max());
    Vector3 high = -low;
    for(auto& geometry : fem.geo_infos)
    {
        Vector3 a = Vector3::Constant(std::numeric_limits<Float>::max());
        Vector3 b = -a;
        for(SizeT j = 0; j < geometry.vertex_count; ++j)
        {
            auto v = geometry.vertex_offset + j;
            a      = a.cwiseMin(fem.h_rest_positions[v]);
            b      = b.cwiseMax(fem.h_rest_positions[v]);
            if(!fixed[v])
            {
                low  = low.cwiseMin(fem.h_rest_positions[v]);
                high = high.cwiseMax(fem.h_rest_positions[v]);
            }
        }
        Float length = (b - a).norm();
        if(!std::isfinite(length) || length <= 0)
            length = 1;
        for(SizeT j = 0; j < geometry.vertex_count; ++j)
            lengths[geometry.vertex_offset + j] = length;
    }
    Float total_mass = 0;
    for(SizeT i = 0; i < masses.size(); ++i)
        if(!fixed[i])
        {
            if(!dynamic[i] || !std::isfinite(masses[i]) || masses[i] <= 0)
                throw SimEngineException("MAS-PNCG requires positive-mass dynamic free FEM vertices; quasi-static vertices need a separate metric.");
            total_mass += masses[i];
        }
    Float extent = total_mass > 0 ? (high - low).norm() : 1;
    if(!std::isfinite(extent) || extent <= 0)
        extent = 1;
    m_scale = total_mass > 0 ? std::sqrt(total_mass) * extent : 1;
    resize_output(m_lengths, lengths.size());
    m_lengths.view().copy_from(lengths.data());
    resize_output(m_eta0, lengths.size());
    resize_output(m_residual_terms, lengths.size());
    resize_output(m_ratios, lengths.size());
    resize_output(m_corrections, lengths.size());
    auto n = m_system->dof_count();
    for(auto vector : {&m_z, &m_old_z, &m_p, &m_direction, &m_hz, &m_hp})
        vector->resize_discard(n);
    m_p.fill(0);
    m_old_z.fill(0);
    if(uses_local_ccd())
    {
        m_trial_direction.resize_discard(n);
        resize_output(m_vertex_fractions, fem.xs.size());
        m_ccd_counts.resize_discard(3);
    }
}

bool MASPNCGSolver::uses_local_ccd() const noexcept
{
    return m_ccd_mode == "local_conservative";
}

void MASPNCGSolver::limit_local_ccd()
{
    Timer      timer{"MAS-PNCG Local Conservative CCD"};
    const auto base = m_mas->nonlinear_base(m_epoch);
    UIPC_ASSERT(base.valid(), "Local CCD requires a current MAS partition snapshot.");
    resize_output(m_domain_fractions, base.domain_count + base.real_vertex_count);
    m_domain_fractions.fill(1);
    m_ccd_counts.fill(0);
    const auto positions     = m_global_vertices->positions();
    const auto displacements = m_global_vertices->displacements();
    const auto thicknesses   = m_global_vertices->thicknesses();
    const auto vertices      = m_surface->surf_vertices();
    const auto edges         = m_surface->surf_edges();
    const auto triangles     = m_surface->surf_triangles();
    if(m_trajectory)
    {
        constexpr Float eta = 0.1;  // Native standard IPC simplex safety margin.
        auto launch_pairs = [&](auto kernel, auto pairs)
        {
            int n = pairs.size();
            if(n)
                kernel<<<cuda_tool::best_grid_dim(n, kernel), cuda_tool::best_block_dim(kernel)>>>(
                    pairs,
                    vertices,
                    edges,
                    triangles,
                    positions,
                    displacements,
                    thicknesses,
                    base,
                    m_fem->is_fixed(),
                    m_domain_fractions.view(),
                    m_ccd_counts.view(),
                    m_vertex_offset,
                    eta);
            increment(m_stats, "local_ccd_candidate_pairs", n);
        };
        launch_pairs(pncg::local_ccd_pairs_kernel<true>, m_trajectory->candidate_PTs());
        launch_pairs(pncg::local_ccd_pairs_kernel<false>, m_trajectory->candidate_EEs());
    }
    if(m_half_planes && m_plane_vertices && m_contact_manager && vertices.size())
    {
        auto kernel = pncg::local_ccd_planes_kernel;
        kernel<<<cuda_tool::best_grid_dim(vertices.size(), kernel), cuda_tool::best_block_dim(kernel)>>>(
            vertices,
            positions,
            displacements,
            thicknesses,
            m_global_vertices->contact_element_ids(),
            m_global_vertices->subscene_element_ids(),
            m_contact_manager->contact_mask_tabular().viewer(),
            m_contact_manager->subscene_mask_tabular().viewer(),
            m_half_planes->positions(),
            m_half_planes->normals(),
            base,
            m_fem->is_fixed(),
            m_domain_fractions.view(),
            m_ccd_counts.view(),
            m_vertex_offset,
            m_plane_vertices->vertex_offset());
    }
    if(base.real_vertex_count)
    {
        auto kernel = pncg::local_ccd_scale_kernel;
        kernel<<<cuda_tool::best_grid_dim(base.real_vertex_count, kernel), cuda_tool::best_block_dim(kernel)>>>(
            m_direction.cview(),
            m_trial_direction.view(),
            base,
            m_fem->is_fixed(),
            m_domain_fractions.cview(),
            m_vertex_fractions.view(),
            m_ccd_counts.view());
        cuda_tool::DeviceReduce().Min(m_vertex_fractions.data(), m_sum.data(), base.real_vertex_count);
        cuda_tool::DeviceReduce().Max(m_vertex_fractions.data(), m_max.data(), base.real_vertex_count);
    }
    else
    {
        m_sum = 1;
        m_max = 1;
    }
    IndexT counts[3] = {};
    m_ccd_counts.cview().copy_to(counts);
    const Float min_fraction = m_sum;
    const Float max_fraction = m_max;
    increment(m_stats, "local_ccd_steps");
    increment(m_stats, "local_ccd_pair_fallbacks", counts[0]);
    increment(m_stats, "local_ccd_clamped_vertices", counts[1]);
    m_stats["local_ccd_min_fraction"] =
        std::min(m_stats["local_ccd_min_fraction"].get<Float>(), min_fraction);
    m_stats["local_ccd_last_max_fraction"] = max_fraction;
    if(counts[2] || !std::isfinite(min_fraction) || min_fraction <= 0)
        throw SimEngineException("MAS-PNCG local CCD has no positive finite candidate step.");
    // Preserve raw m_direction for Algorithm 1's history. All actual motion
    // consumers receive the separate trial through the normal FEM scatter.
    m_slope = m_system->ctx().dot(m_system->nonlinear_gradient(),
                                  m_trial_direction.cview());
    if(!std::isfinite(m_slope))
        throw SimEngineException("MAS-PNCG local CCD produced a nonfinite directional derivative.");
    m_stats["local_ccd_last_actual_slope"] = m_slope;
    if(m_slope >= 0)
        increment(m_stats, "local_ccd_non_descent_steps");
    m_system->scatter_nonlinear_direction(m_trial_direction.cview());
}

void MASPNCGSolver::record_local_ccd_guard(Float fraction)
{
    if(!uses_local_ccd())
        return;
    if(fraction < 1)
        increment(m_stats, "local_ccd_global_guard_reductions");
    m_stats["local_ccd_min_global_guard_fraction"] =
        std::min(m_stats["local_ccd_min_global_guard_fraction"].get<Float>(), fraction);
}

void MASPNCGSolver::validate_capabilities()
{
    auto& fem = m_fem->m_impl;
    for(auto& geometry : fem.geo_infos)
    {
        auto uid = geometry.dim_uid.uid;
        if(uid != 9 && uid != 10 && uid != 11 && uid != 819)
            throw SimEngineException(fmt::format("MAS-PNCG: unsupported FEM constitution UID {} (supported: ARAP, StableNeoHookean, NeoHookeanShell, StrainLimitingBaraffWitkinShell).",
                                                 uid));
    }
    for(auto extra : fem.extra_constitutions.view())
        if(extra->uid() != 17)
            throw SimEngineException(fmt::format("MAS-PNCG: unsupported FEM extra constitution UID {}; only stateless DiscreteShellBending is supported.",
                                                 extra->uid()));
    for(SizeT i = 0; i < fem.h_masses.size(); ++i)
        if(!fem.h_vertex_is_fixed[i]
           && (!fem.h_vertex_is_dynamic[i] || !std::isfinite(fem.h_masses[i])
               || fem.h_masses[i] <= 0))
            throw SimEngineException("MAS-PNCG requires positive-mass dynamic free FEM vertices; quasi-static vertices need a separate metric.");
}

void MASPNCGSolver::objective_changed()
{
    m_need_base       = true;
    m_have_history    = false;
    m_reference_ready = false;
    m_pending_restart = m_stop_after_accept = false;
    if(m_iteration_policy == "paper")
        m_old_z.fill(0);
}

bool MASPNCGSolver::needs_base() const
{
    return m_need_base || m_update_mode == "full_rebuild"
           || (m_max_age > 0 && m_age >= m_max_age);
}

void MASPNCGSolver::export_contacts()
{
    SizeT simplex_count = m_simplex ? m_simplex->normal_surrogate_count() : 0;
    SizeT plane_count   = m_planes ? m_planes->normal_surrogate_count() : 0;
    SizeT ee_count      = m_simplex ? m_simplex->EEs().size() : 0;
    SizeT max_columns   = std::numeric_limits<IndexT>::max() / 4;
    if(ee_count > max_columns / 12 || simplex_count + plane_count > max_columns - 11 * ee_count)
        throw SimEngineException("MAS-PNCG contact count exceeds supported CUDA index range.");
    resize_output(m_current, simplex_count + plane_count);
    resize_output(m_curvatures, ee_count);
    m_stats["normal_contacts"]       = m_current.size();
    m_stats["ee_curvature_contacts"] = ee_count;
    if(simplex_count)
        m_simplex->export_normal_surrogates(m_current.view().subview(0, simplex_count),
                                            m_curvatures.view());
    if(plane_count)
        m_planes->export_normal_surrogates(m_current.view().subview(simplex_count, plane_count));
    m_error           = 0;
    m_mollified_count = 0;
    int n             = m_current.size();
    if(n)
    {
        auto k = pncg::validate_contacts_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k)>>>(
            m_current.cview(),
            m_error.view(),
            m_curvatures.cview(),
            m_mollified_count.view());
    }
    if(static_cast<IndexT>(m_error))
        throw SimEngineException("MAS-PNCG contact export rejected invalid geometry or nonfinite curvature.");
    const IndexT mollified        = m_mollified_count;
    m_stats["mollified_contacts"] = mollified;
    m_stats["peak_mollified_contacts"] =
        std::max(m_stats["peak_mollified_contacts"].get<IndexT>(), mollified);
    sort_contacts();
}

void MASPNCGSolver::sort_contacts()
{
    int n = m_current.size();
    resize_output(m_hash_in, n);
    resize_output(m_hash, n);
    resize_output(m_order_in, n);
    resize_output(m_order, n);
    if(!n)
        return;
    auto k = pncg::contact_hash_kernel;
    k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k)>>>(
        m_current.cview(), m_hash_in.view(), m_order_in.view());
    cuda_tool::DeviceRadixSort().SortPairs(
        m_hash_in.data(), m_hash.data(), m_order_in.data(), m_order.data(), n);
}

void MASPNCGSolver::capture_contacts()
{
    resize_output(m_baseline, m_current.size());
    resize_output(m_base_curvatures, m_curvatures.size());
    resize_output(m_base_hash, m_hash.size());
    resize_output(m_base_order, m_order.size());
    m_baseline.view().copy_from(m_current.cview());
    m_base_curvatures.view().copy_from(m_curvatures.cview());
    m_base_hash.view().copy_from(m_hash.cview());
    m_base_order.view().copy_from(m_order.cview());
    m_updates.resize_discard(0);
}

bool MASPNCGSolver::build_updates()
{
    resize_output(m_updates, m_current.size() + 11 * m_curvatures.size());
    m_error        = 0;
    m_update_count = 0;
    int n          = m_current.size();
    if(n)
    {
        auto k = pncg::contact_updates_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k)>>>(
            m_current.cview(),
            m_hash.cview(),
            m_order.cview(),
            m_baseline.cview(),
            m_base_hash.cview(),
            m_base_order.cview(),
            m_fem->is_fixed(),
            m_vertex_offset,
            m_cos_rotation,
            m_updates.view(),
            m_error.view(),
            m_update_count.view(),
            m_curvatures.cview(),
            m_base_curvatures.cview());
    }
    IndexT error = m_error;
    if(error >= 3)
        throw SimEngineException("MAS-PNCG contact surrogate has invalid geometry, curvature or FEM indices.");
    return error == 0;
}

bool MASPNCGSolver::evaluate(bool rebuild)
{
    m_started_with_rebuild = rebuild;
    if(rebuild)
    {
        m_system->rebuild_nonlinear_base();
        auto matrix = m_system->nonlinear_matrix();
        int  blocks = m_system->dof_count() / 3;
        m_base.resize_discard(blocks, blocks, matrix.triplet_count());
        m_base.row_indices().copy_from(matrix.row_indices());
        m_base.col_indices().copy_from(matrix.col_indices());
        m_base.values().copy_from(matrix.values());
        if(!m_mas->capture_nonlinear_base(++m_epoch))
        {
            const auto& diagnostic = m_mas->nonlinear_capture_diagnostic();
            m_stats["baseline_capture_failure"] = {{"stage", diagnostic.stage_name()},
                                                   {"reason", diagnostic.reason_name()},
                                                   {"epoch", diagnostic.epoch},
                                                   {"level", diagnostic.level},
                                                   {"domain", diagnostic.domain},
                                                   {"index", diagnostic.index},
                                                   {"row", diagnostic.row},
                                                   {"column", diagnostic.column},
                                                   {"value", diagnostic.value},
                                                   {"threshold", diagnostic.threshold}};
            throw SimEngineException(fmt::format("MAS-PNCG could not capture an SPD double MAS baseline: stage={}, reason={}, level={}, domain={}, index={}, row={}, column={}, value={}, threshold={}.",
                                                 diagnostic.stage_name(),
                                                 diagnostic.reason_name(),
                                                 diagnostic.level,
                                                 diagnostic.domain,
                                                 diagnostic.index,
                                                 diagnostic.row,
                                                 diagnostic.column,
                                                 diagnostic.value,
                                                 diagnostic.threshold));
        }
        export_contacts();
        capture_contacts();
        m_have_history = false;
        m_p.fill(0);
        m_need_base = false;
        m_age       = 0;
        increment(m_stats, "base_rebuilds");
    }
    else
    {
        m_system->evaluate_nonlinear_gradient();
        if(m_update_mode == "incremental")
        {
            export_contacts();
            if(!build_updates())
            {
                increment(m_stats, "contact_rebuild_fallbacks");
                return false;
            }
        }
        else
            m_updates.resize_discard(0);
    }
    update_residual();
    return true;
}

void MASPNCGSolver::update_residual()
{
    m_zero_gradient = false;
    int n           = m_fem->xs().size();
    if(!n)
    {
        m_residual = m_local_ratio = m_kinetic_correction = 0;
        m_zero_gradient                                   = true;
        return;
    }
    if(m_use_fused)
    {
        const auto sums      = m_fused->residual(m_system->nonlinear_gradient(),
                                            m_fem->masses(),
                                            m_fem->is_fixed(),
                                            m_lengths.cview(),
                                            m_eta0.view(),
                                            m_atol,
                                            m_rtol,
                                            !m_reference_ready,
                                            m_use_graph);
        m_residual           = std::sqrt(sums.sum);
        m_local_ratio        = sums.ratio;
        m_kinetic_correction = sums.correction;
        if(sums.invalid)
            throw SimEngineException("MAS-PNCG encountered a nonfinite independent residual.");
    }
    else
    {
        auto k = pncg::residual_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k)>>>(
            m_system->nonlinear_gradient(),
            m_fem->masses(),
            m_fem->is_fixed(),
            m_lengths.cview(),
            m_eta0.view(),
            m_residual_terms.view(),
            m_ratios.view(),
            m_corrections.view(),
            m_atol,
            m_rtol,
            !m_reference_ready);
        cuda_tool::DeviceReduce().Sum(m_residual_terms.data(), m_sum.data(), n);
        m_residual = std::sqrt(static_cast<Float>(m_sum));
        cuda_tool::DeviceReduce().Max(m_ratios.data(), m_max.data(), n);
        m_local_ratio = m_max;
        cuda_tool::DeviceReduce().Max(m_corrections.data(), m_max.data(), n);
        m_kinetic_correction = m_max;
    }
    if(!std::isfinite(m_residual) || !std::isfinite(m_local_ratio)
       || !std::isfinite(m_kinetic_correction))
        throw SimEngineException("MAS-PNCG encountered a nonfinite independent residual.");
    if(!m_reference_ready)
    {
        m_initial_residual          = m_residual;
        m_reference_ready           = true;
        m_stats["gradient_initial"] = m_residual;
    }
    if(m_iteration_policy == "paper" && m_kinetic_correction == 0)
    {
        m_error    = 0;
        auto check = pncg::nonzero_gradient_kernel;
        int  count = m_system->nonlinear_gradient().size();
        check<<<cuda_tool::best_grid_dim(count, check), cuda_tool::best_block_dim(check)>>>(
            m_system->nonlinear_gradient(), m_fem->is_fixed(), m_error.view());
        m_zero_gradient = static_cast<IndexT>(m_error) == 0;
    }
    m_stats["gradient_final"]           = m_residual;
    m_stats["max_local_residual_ratio"] = m_local_ratio;
}

bool MASPNCGSolver::stationary(bool targets_reached)
{
    bool residual_converged =
        targets_reached && m_local_ratio <= 1
        && m_residual <= std::max(m_atol * m_scale, m_rtol * m_initial_residual)
        && m_kinetic_correction <= m_velocity_tol * m_dt;
    m_stats["independent_residual_converged"] = residual_converged;
    m_stats["convergence_criterion"]          = "not_converged";
    if(m_iteration_policy == "strict")
    {
        if(residual_converged)
            m_stats["convergence_criterion"] = "independent_residual";
        return residual_converged;
    }
    if(!targets_reached)
        return false;
    if(m_zero_gradient)
    {
        m_stats["convergence_criterion"] = "zero_gradient";
        return true;
    }
    if(m_stop_after_accept)
    {
        m_stats["convergence_criterion"] = "preconditioned_gradient";
        return true;
    }
    return false;
}

bool MASPNCGSolver::accepts_energy(Float initial, Float trial, Float fraction) const
{
    if(!std::isfinite(initial) || !std::isfinite(trial)
       || !std::isfinite(fraction) || fraction <= 0)
        return false;
    return m_iteration_policy == "paper" ? fraction <= 1 :
                                           trial <= initial + 1e-4 * fraction * m_slope;
}

void MASPNCGSolver::record_energy(Float initial, Float trial)
{
    m_stats["last_energy_before"] = initial;
    m_stats["last_energy_after"]  = trial;
    if(trial > initial)
        increment(m_stats, "energy_increase_steps");
}

bool MASPNCGSolver::apply_preconditioner()
{
    auto base = m_mas->nonlinear_base(m_epoch);
    if(!base.valid())
        return false;
    auto g = m_system->nonlinear_gradient();
    m_mas->apply_frozen_coarse(m_epoch, g, m_z.view());
    resize_output(m_selected, base.domain_count * m_top_k);
    m_selected.fill(-1);
    m_error            = 0;
    m_truncated        = 0;
    m_direct_fallbacks = 0;
    int n              = m_updates.size();
    if(n)
    {
        resize_output(m_domain_in, 4 * n);
        resize_output(m_domains, 4 * n);
        auto k = pncg::domain_keys_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k)>>>(
            m_updates.cview(), base.real_to_part, m_vertex_offset, m_domain_in.view());
        cuda_tool::DeviceRadixSort().SortKeys(m_domain_in.data(), m_domains.data(), 4 * n);
        if(base.domain_count)
        {
            auto select = pncg::select_domains_kernel;
            select<<<cuda_tool::best_grid_dim(base.domain_count, select), cuda_tool::best_block_dim(select)>>>(
                m_domains.cview(),
                m_updates.cview(),
                base.real_to_part,
                m_vertex_offset,
                base.domain_count,
                m_top_k,
                m_selected.view(),
                m_truncated.view());
        }
    }
    if(base.domain_count)
        pncg::local_solve_kernel<<<base.domain_count, 64>>>(base,
                                                            m_updates.cview(),
                                                            m_selected.cview(),
                                                            g,
                                                            m_z.view(),
                                                            m_vertex_offset,
                                                            m_top_k,
                                                            m_error.view(),
                                                            m_direct_fallbacks.view());
    int count = m_z.size();
    if(count)
    {
        auto k = pncg::mask_kernel;
        k<<<cuda_tool::best_grid_dim(count, k), cuda_tool::best_block_dim(k)>>>(
            m_z.view(), m_fem->is_fixed());
    }
    IndexT error = m_error;
    increment(m_stats, "truncated_columns", static_cast<IndexT>(m_truncated));
    increment(m_stats, "direct_local_fallbacks", static_cast<IndexT>(m_direct_fallbacks));
    return error == 0;
}

MASPNCGSolver::DirectionStatus MASPNCGSolver::compute_direction()
{
    increment(m_stats, "nonlinear_iterations");
    m_pending_restart = false;
    if(!apply_preconditioner())
        return DirectionStatus::Restart;
    auto g = m_system->nonlinear_gradient();
    if(m_use_fused)
    {
        m_spmv.rbk_sym_spmv(1, m_base.cview(), m_z.cview(), 0, m_hz.view());
        if(m_have_history)
            m_spmv.rbk_sym_spmv(1, m_base.cview(), m_p.cview(), 0, m_hp.view());
        const auto result = m_fused->direction(g,
                                               m_z.cview(),
                                               m_old_z.cview(),
                                               m_p.cview(),
                                               m_hz.cview(),
                                               m_hp.cview(),
                                               m_updates.cview(),
                                               m_vertex_offset,
                                               m_have_history,
                                               m_restart_threshold,
                                               m_direction.view(),
                                               m_use_graph,
                                               m_iteration_policy == "paper");
        if(!result.valid)
            return DirectionStatus::Restart;
        m_slope = result.slope;
        if(m_iteration_policy == "paper")
        {
            if(!std::isfinite(result.preconditioned_norm)
               || !std::isfinite(result.restart_ratio))
                return DirectionStatus::Restart;
            m_preconditioned_norm = result.preconditioned_norm;
            m_pending_restart     = result.restart_next;
            m_stats["preconditioned_gradient_norm"] = m_preconditioned_norm;
            m_stats["restart_ratio"]                = result.restart_ratio;
        }
        m_system->scatter_nonlinear_direction(m_direction.cview());
        increment(m_stats, result.used_history ? "two_d_steps" : "one_d_steps");
        increment(m_stats, "contact_update_columns", result.update_columns);
        return DirectionStatus::Ready;
    }
    Float gz = m_system->ctx().dot(g, m_z.cview());
    if(!std::isfinite(gz) || gz <= 0)
        return DirectionStatus::Restart;
    if(m_iteration_policy == "paper")
    {
        m_preconditioned_norm = std::sqrt(m_system->ctx().dot(m_z, m_z));
        Float ratio = std::abs(m_system->ctx().dot(g, m_old_z.cview())) / gz;
        if(!std::isfinite(m_preconditioned_norm) || !std::isfinite(ratio))
            return DirectionStatus::Restart;
        m_pending_restart                       = ratio > m_restart_threshold;
        m_stats["preconditioned_gradient_norm"] = m_preconditioned_norm;
        m_stats["restart_ratio"]                = ratio;
    }
    else if(m_have_history)
    {
        Float ratio = std::abs(m_system->ctx().dot(g, m_old_z.cview())) / gz;
        if(!std::isfinite(ratio) || ratio > m_restart_threshold)
            return DirectionStatus::Restart;
    }
    m_spmv.rbk_sym_spmv(1, m_base.cview(), m_z.cview(), 0, m_hz.view());
    Float a = m_system->ctx().dot(m_z, m_hz), b = 0, c = 0, gp = 0;
    if(m_have_history)
    {
        m_spmv.rbk_sym_spmv(1, m_base.cview(), m_p.cview(), 0, m_hp.view());
        b  = m_system->ctx().dot(m_z, m_hp);
        c  = m_system->ctx().dot(m_p, m_hp);
        gp = m_system->ctx().dot(g, m_p.cview());
    }
    int n = m_updates.size();
    if(n)
    {
        resize_output(m_projections, n);
        auto k = pncg::projection_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k)>>>(
            m_updates.cview(),
            m_z.cview(),
            m_p.cview(),
            m_vertex_offset,
            m_projections.view());
        cuda_tool::DeviceReduce().Reduce(m_projections.data(),
                                         m_projection_sum.data(),
                                         n,
                                         pncg::ProjectionSum{},
                                         Vector3::Zero().eval());
        Vector3 sums = m_projection_sum;
        a += sums[0];
        b += sums[1];
        c += sums[2];
        increment(m_stats, "contact_update_columns", static_cast<IndexT>(m_update_count));
    }
    auto solution = solve_pncg_subspace(a, b, c, gz, gp, m_have_history);
    if(!solution.valid)
        return DirectionStatus::Restart;
    n = m_z.size();
    if(n)
    {
        auto k = pncg::direction_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k)>>>(
            m_z.cview(), m_p.cview(), m_direction.view(), solution.mu, solution.nu);
    }
    m_slope = m_system->ctx().dot(g, m_direction.cview());
    if(!std::isfinite(m_slope) || m_slope >= 0)
        return DirectionStatus::Restart;
    m_system->scatter_nonlinear_direction(m_direction.cview());
    increment(m_stats, solution.used_history ? "two_d_steps" : "one_d_steps");
    return DirectionStatus::Ready;
}

bool MASPNCGSolver::retry()
{
    if(m_retry_used)
        return false;
    m_retry_used   = true;
    m_need_base    = true;
    m_have_history = false;
    increment(m_stats, "restarts");
    increment(m_stats, "numerical_retries");
    return true;
}

void MASPNCGSolver::accept(Float fraction)
{
    const bool  paper            = m_iteration_policy == "paper";
    const Float history_fraction = paper ? 1.0 : fraction;
    int         n                = m_direction.size();
    if(m_use_fused)
        m_fused->accept(m_direction.cview(), m_z.cview(), m_p.view(), m_old_z.view(), history_fraction);
    else
    {
        if(n)
        {
            auto k = pncg::scale_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k)>>>(
                m_direction.cview(), m_p.view(), history_fraction);
        }
        m_old_z = m_z;
    }
    m_have_history = true;
    m_retry_used   = false;
    ++m_age;
    m_stats["base_age"] = m_age;
    increment(m_stats, "accepted_steps");
    if(paper)
    {
        // Algorithm 1 tests the pre-step norm only on a fresh-rebuild iteration.
        m_stop_after_accept = m_started_with_rebuild
                              && m_preconditioned_norm <= m_preconditioned_tol;
        m_stats["convergence_checked_on_rebuild"] = m_started_with_rebuild;
        if(m_pending_restart && !m_stop_after_accept)
        {
            m_need_base = true;
            increment(m_stats, "orthogonality_restarts");
            increment(m_stats, "restarts");
        }
    }
}

void MASPNCGSolver::fail()
{
    m_failed    = true;
    m_need_base = true;
}

void MASPNCGSolver::rejected_trial(bool nonfinite)
{
    increment(m_stats, "rejected_trials");
    if(nonfinite)
        increment(m_stats, "nonfinite_trials");
}

void MASPNCGSolver::recover()
{
    m_failed          = false;
    m_need_base       = true;
    m_have_history    = false;
    m_reference_ready = false;
    m_pending_restart = m_stop_after_accept = m_zero_gradient = false;
}

Json MASPNCGSolver::frame_stats() const
{
    Json stats                  = m_stats.is_object() ? m_stats :
                                                        Json{{"nonlinear_solver", "mas_pncg"},
                                                             {"nonlinear_iterations", 0}};
    stats["fused_kernels"]      = m_use_fused;
    stats["cuda_graph_enabled"] = m_use_graph;
    stats["cuda_graph_captures"] =
        m_fused ? m_fused->graph_captures() - m_frame_captures : 0;
    stats["cuda_graph_replays"] = m_fused ? m_fused->graph_replays() - m_frame_replays : 0;
    stats["cuda_graph_fallbacks"] =
        m_fused ? m_fused->graph_fallbacks() - m_frame_fallbacks : 0;
    return stats;
}
}  // namespace uipc::backend::cuda
