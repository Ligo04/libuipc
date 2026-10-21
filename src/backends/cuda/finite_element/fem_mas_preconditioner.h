#pragma once
#include <linear_system/local_preconditioner.h>
#include <finite_element/mas_preconditioner_engine.h>

namespace uipc::backend::cuda
{
class FiniteElementMethod;
class FEMLinearSubsystem;

class FEMMASPreconditioner : public LocalPreconditioner
{
  public:
    using LocalPreconditioner::LocalPreconditioner;
    static constexpr int BANKSIZE = MASPreconditionerEngine::BANKSIZE;

    bool capture_nonlinear_base(SizeT epoch);
    MASPreconditionerEngine::FrozenLevel0View nonlinear_base(SizeT expected_epoch) const;
    // Operates on distinct FEM-local input/output slices on the default stream.
    // Excludes the Level-0 solve; unpartitioned entries use their captured fallback.
    void apply_frozen_coarse(SizeT                              epoch,
                             cuda_tool::CDenseVectorView<Float> r,
                             cuda_tool::DenseVectorView<Float>  z);

  private:
    FiniteElementMethod*               finite_element_method = nullptr;
    GlobalLinearSystem*                global_linear_system  = nullptr;
    FEMLinearSubsystem*                fem_linear_subsystem  = nullptr;
    MASPreconditionerEngine            engine;
    bool                               m_has_partition     = false;
    bool                               m_has_unpartitioned = false;
    cuda_tool::DeviceBuffer<Matrix3x3> diag_inv;
    cuda_tool::DeviceBuffer<int>       unpartitioned_flags;

    bool  m_nonlinear_ready        = false;
    SizeT m_nonlinear_epoch        = MASPreconditionerEngine::invalid_epoch;
    int   m_nonlinear_vertex_count = 0;
    cuda_tool::DeviceBuffer<Matrix3x3> nonlinear_diag_inv;
    cuda_tool::DeviceBuffer<IndexT>    nonlinear_status;

    void do_build(BuildInfo& info) override;
    void do_init(InitInfo& info) override;
    void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) override;
    void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override;
};
}  // namespace uipc::backend::cuda
