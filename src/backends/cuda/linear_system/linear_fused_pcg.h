#pragma once
#include <linear_system/iterative_solver.h>
#include <muda/ext/linear_system/device_bcoo_matrix.h>
#include <algorithm/matrix_converter.h>
#include <muda/buffer/device_var.h>

namespace uipc::backend::cuda
{
// Fused PCG: keeps dot-product scalars (rz, pAp, rz_new) on device
// to eliminate per-iteration host synchronizations.  The update kernels read
// alpha = rz/pAp and beta = rz_new/rz directly from device memory.
// SpMV and dot(p,Ap) are fused into a single kernel pass.
// Convergence is checked every `check_interval` iterations via a single D2H copy.
class LinearFusedPCG : public IterativeSolver
{
  public:
    using IterativeSolver::IterativeSolver;

    void begin_coarse_solve(muda::DenseVectorView<Float>  x,
                            muda::CDenseVectorView<Float> b,
                            Float                         relative_tol);
    SizeT coarse_solve_iterations(muda::DenseVectorView<Float> x, SizeT iteration_count);
    bool coarse_solve_converged() const noexcept { return coarse_converged; }
    bool check_displacement_converged(muda::CDenseVectorView<Float> fine_x,
                                      Float                         tolerance,
                                      Float* out_inf_norm = nullptr);

  protected:
    virtual void do_build(BuildInfo& info) override;
    virtual void do_solve(GlobalLinearSystem::SolvingInfo& info) override;

  private:
    using DeviceDenseVector   = muda::DeviceDenseVector<Float>;
    using DeviceTripletMatrix = muda::DeviceTripletMatrix<Float, 3>;
    using DeviceBCOOMatrix    = muda::DeviceBCOOMatrix<Float, 3>;

    SizeT fused_pcg(muda::DenseVectorView<Float>  x,
                    muda::CDenseVectorView<Float> b,
                    SizeT                         max_iter,
                    bool                          use_initial_guess,
                    bool                          accuracy_check_enabled);
    void  check_init_rz_nan_inf(Float rz);
    void  check_iter_rz_nan_inf(Float rz, SizeT k);

    DeviceDenseVector r;
    DeviceDenseVector z;
    DeviceDenseVector p;
    DeviceDenseVector Ap;

    muda::DeviceVar<Float>         d_rz;
    muda::DeviceVar<Float>         d_pAp;
    muda::DeviceVar<Float>         d_rz_new;
    muda::DeviceVar<IndexT>        d_converged;
    muda::DeviceVar<Float>         d_max_abs_value;
    muda::DeviceDenseVector<Float> abs_buffer;
    bool                           coarse_started   = false;
    bool                           coarse_converged = false;
    Float                          coarse_rz_tol    = 0;

    Float max_iter_ratio  = 2.0;
    Float global_tol_rate = 1e-4;
    Float reserve_ratio   = 1.5;
    SizeT check_interval  = 5;
};
}  // namespace uipc::backend::cuda
