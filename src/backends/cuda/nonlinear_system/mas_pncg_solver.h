#pragma once
#include <sim_system.h>
#include <cuda_tool/cuda_tool.h>
#include <contact_system/ipc_contact_curvature.h>
#include <finite_element/mas_preconditioner_engine.h>
#include <linear_system/spmv.h>
#include <memory>

namespace uipc::backend::cuda
{
class GlobalLinearSystem;
class FiniteElementMethod;
class FEMLinearSubsystem;
class FEMMASPreconditioner;
class FiniteElementVertexReporter;
class SimplexNormalContact;
class VertexHalfPlaneNormalContact;
namespace pncg
{
    class FusedOperations;
}

class MASPNCGSolver final : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    ~MASPNCGSolver() override;
    enum class DirectionStatus
    {
        Ready,
        Restart
    };
    void            begin_frame();
    void            objective_changed();
    bool            needs_base() const;
    bool            evaluate(bool rebuild);
    bool            stationary(bool targets_reached);
    DirectionStatus compute_direction();
    bool            retry();
    void            accept(Float fraction);
    void            rejected_trial(bool nonfinite);
    void            fail();
    void            recover();
    IndexT          max_iter() const noexcept { return m_max_iter; }
    Float           slope() const noexcept { return m_slope; }
    Json            frame_stats() const;

  private:
    void do_build() override;
    void validate_capabilities();
    void export_contacts();
    void sort_contacts();
    void capture_contacts();
    bool build_updates();
    bool apply_preconditioner();
    void update_residual();

    GlobalLinearSystem*           m_system    = nullptr;
    FiniteElementMethod*          m_fem       = nullptr;
    FEMLinearSubsystem*           m_subsystem = nullptr;
    FEMMASPreconditioner*         m_mas       = nullptr;
    FiniteElementVertexReporter*  m_vertices  = nullptr;
    SimplexNormalContact*         m_simplex   = nullptr;
    VertexHalfPlaneNormalContact* m_planes    = nullptr;

    IndexT                                 m_max_iter          = 1024;
    IndexT                                 m_top_k             = 8;
    IndexT                                 m_max_age           = 32;
    Float                                  m_cos_rotation      = 0;
    Float                                  m_restart_threshold = 0.3;
    Float                                  m_rtol              = 1e-5;
    Float                                  m_atol              = 1e-8;
    Float                                  m_velocity_tol      = 0.05;
    std::string                            m_update_mode;
    bool                                   m_use_fused = true;
    bool                                   m_use_graph = true;
    std::unique_ptr<pncg::FusedOperations> m_fused;
    SizeT  m_frame_captures = 0, m_frame_replays = 0, m_frame_fallbacks = 0;
    bool   m_need_base          = true;
    bool   m_have_history       = false;
    bool   m_reference_ready    = false;
    bool   m_retry_used         = false;
    bool   m_failed             = false;
    SizeT  m_epoch              = 0;
    IndexT m_age                = 0;
    Float  m_slope              = 0;
    Float  m_initial_residual   = 0;
    Float  m_residual           = 0;
    Float  m_local_ratio        = 0;
    Float  m_kinetic_correction = 0;
    Float  m_scale              = 1;
    Float  m_dt                 = 0.01;
    IndexT m_vertex_offset      = 0;
    IndexT m_dof_offset         = 0;
    Json   m_stats;

    cuda_tool::DeviceBCOOMatrix<Float, 3> m_base;
    cuda_tool::DeviceDenseVector<Float>   m_z, m_old_z, m_p, m_direction;
    cuda_tool::DeviceDenseVector<Float>   m_hz, m_hp;
    Spmv                                  m_spmv;
    cuda_tool::DeviceBuffer<Float>        m_lengths, m_eta0, m_residual_terms;
    cuda_tool::DeviceBuffer<Float>        m_ratios, m_corrections;
    cuda_tool::DeviceVar<Float>           m_sum, m_max;
    cuda_tool::DeviceBuffer<IPCNormalSurrogate> m_current, m_baseline, m_updates;
    cuda_tool::DeviceBuffer<IPCContactCurvature> m_curvatures, m_base_curvatures;
    cuda_tool::DeviceBuffer<unsigned long long> m_hash_in, m_hash, m_base_hash;
    cuda_tool::DeviceBuffer<IndexT> m_order_in, m_order, m_base_order;
    cuda_tool::DeviceBuffer<unsigned long long> m_domain_in, m_domains;
    cuda_tool::DeviceBuffer<IndexT>             m_selected;
    cuda_tool::DeviceBuffer<Vector3>            m_projections;
    cuda_tool::DeviceVar<Vector3>               m_projection_sum;
    cuda_tool::DeviceVar<IndexT> m_error, m_truncated, m_direct_fallbacks, m_update_count;
    cuda_tool::DeviceVar<IndexT> m_mollified_count;
};
}  // namespace uipc::backend::cuda
