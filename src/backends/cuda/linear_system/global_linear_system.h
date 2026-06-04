#pragma once
#include <sim_system.h>
#include <functional>
#include <uipc/common/list.h>
#include <uipc/common/vector.h>
#include <muda/ext/linear_system.h>
#include <algorithm/matrix_converter.h>
#include <linear_system/spmv.h>
#include <linear_system/agipc_coarse_linear_system.h>
#include <utils/offset_count_collection.h>
#include <energy_component_flags.h>
#include <global_geometry/global_vertex_manager.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <affine_body/affine_body_vertex_reporter.h>
#include <affine_body/affine_body_body_reporter.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/mas_preconditioner_engine.h>
#include <finite_element/finite_element_body_reporter.h>
#include <finite_element/finite_element_vertex_reporter.h>
#include <global_geometry/global_simplicial_surface_manager.h>
namespace uipc::backend::cuda
{
// Define a simple POD to avoid constructing CUDA's built-in vector type with pmr allocators in host code
struct SizeT2
{
    SizeT x;
    SizeT y;
};

class DiagLinearSubsystem;
class OffDiagLinearSubsystem;
class IterativeSolver;
class LocalPreconditioner;
class GlobalPreconditioner;

class GlobalLinearSystem : public SimSystem
{
    static constexpr SizeT DoFBlockSize = 3;

  public:
    using SimSystem::SimSystem;
    using TripletMatrixView = muda::TripletMatrixView<Float, 3>;
    using CBCOOMatrixView   = muda::CBCOOMatrixView<Float, 3>;
    using DenseVectorView   = muda::DenseVectorView<Float>;
    using CDenseVectorView  = muda::CDenseVectorView<Float>;
    using ComponentFlags    = EnergyComponentFlags;

    class Impl;

    class InitDofExtentInfo
    {
      public:
        void extent(SizeT dof_count) noexcept { m_dof_count = dof_count; }

      private:
        friend class Impl;
        SizeT m_dof_count = 0;
    };

    class InitDofInfo
    {
      public:
        IndexT dof_offset() const { return m_dof_offset; }
        IndexT dof_count() const { return m_dof_count; }

      private:
        friend class Impl;
        IndexT m_dof_offset = 0;
        IndexT m_dof_count  = 0;
    };

    class DiagExtentInfo
    {
      public:
        bool           gradient_only() const { return m_gradient_only; }
        ComponentFlags component_flags() const { return m_component_flags; }
        void extent(SizeT hessian_block_count, SizeT dof_count) noexcept;

      private:
        friend class Impl;
        ComponentFlags m_component_flags = ComponentFlags::All;
        SizeT          m_dof_count       = 0;
        SizeT          m_block_count     = 0;
        bool           m_gradient_only   = false;
    };

    class ComputeGradientInfo
    {
      public:
        // - Contact: only consider contact part
        // - Complement: only consider non-contact part
        void flags(ComponentFlags component) noexcept;
        void buffer_view(muda::DenseVectorView<Float> grad) noexcept;

      private:
        friend class Impl;
        muda::DenseVectorView<Float> m_gradients;
        ComponentFlags               m_flags = ComponentFlags::All;
    };

    class DiagInfo
    {
      public:
        DiagInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        TripletMatrixView hessians() { return m_hessians; }
        DenseVectorView   gradients() { return m_gradients; }
        bool              gradient_only() const { return m_gradient_only; }
        ComponentFlags    component_flags() const { return m_component_flags; }

      private:
        friend class Impl;
        SizeT             m_index = ~0ull;
        TripletMatrixView m_hessians;
        DenseVectorView   m_gradients;
        bool              m_gradient_only   = false;
        ComponentFlags    m_component_flags = ComponentFlags::All;
        Impl*             m_impl            = nullptr;
    };

    class OffDiagExtentInfo
    {
      public:
        void extent(SizeT lr_hessian_block_count, SizeT rl_hassian_block_count) noexcept;

      private:
        friend class Impl;
        SizeT m_lr_block_count = 0;
        SizeT m_rl_block_count = 0;
    };

    class OffDiagInfo
    {
      public:
        OffDiagInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        TripletMatrixView lr_hessian() const { return m_lr_hessian; }
        TripletMatrixView rl_hessian() const { return m_rl_hessian; }

      private:
        friend class Impl;
        SizeT             m_index = ~0ull;
        TripletMatrixView m_lr_hessian;
        TripletMatrixView m_rl_hessian;
        Impl*             m_impl = nullptr;
    };

    class AssemblyInfo
    {
      public:
        AssemblyInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        CBCOOMatrixView A() const;

      protected:
        friend class Impl;
        Impl* m_impl = nullptr;
    };

    class GlobalPreconditionerAssemblyInfo : public AssemblyInfo
    {
      public:
        using AssemblyInfo::AssemblyInfo;
    };

    class LocalPreconditionerAssemblyInfo : public AssemblyInfo
    {
      public:
        LocalPreconditionerAssemblyInfo(Impl* impl, SizeT index) noexcept
            : AssemblyInfo(impl)
            , m_index(index)
        {
        }

        SizeT dof_offset() const;
        SizeT dof_count() const;

      private:
        SizeT m_index;
    };

    using DiagNormInfo = LocalPreconditionerAssemblyInfo;

    class ApplyPreconditionerInfo
    {
      public:
        ApplyPreconditionerInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        DenseVectorView        z() { return m_z; }
        CDenseVectorView       r() { return m_r; }
        muda::CVarView<IndexT> converged() { return m_converged; }

      private:
        friend class Impl;
        DenseVectorView        m_z;
        CDenseVectorView       m_r;
        muda::CVarView<IndexT> m_converged;
        Impl*                  m_impl = nullptr;
    };

    class AccuracyInfo
    {
      public:
        AccuracyInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        CDenseVectorView r() const { return m_r; }

        void satisfied(bool satisfied) { m_statisfied = satisfied; }

      private:
        friend class Impl;
        CDenseVectorView m_r;
        Impl*            m_impl       = nullptr;
        bool             m_statisfied = true;
    };

    class SolvingInfo
    {
      public:
        SolvingInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        DenseVectorView  x() { return m_x; }
        CDenseVectorView b() { return m_b; }
        void  iter_count(SizeT iter_count) { m_iter_count = iter_count; }
        bool  use_initial_guess() const { return m_use_initial_guess; }
        SizeT max_iter_override() const { return m_max_iter_override; }
        bool accuracy_check_enabled() const { return m_accuracy_check_enabled; }

      private:
        friend class Impl;
        DenseVectorView  m_x;
        CDenseVectorView m_b;
        SizeT            m_iter_count             = 0;
        bool             m_use_initial_guess      = false;
        SizeT            m_max_iter_override      = 0;
        bool             m_accuracy_check_enabled = true;
        Impl*            m_impl                   = nullptr;
    };

    class SolutionInfo
    {
      public:
        SolutionInfo(Impl* impl)
            : m_impl(impl)
        {
        }

        CDenseVectorView solution() { return m_solution; }

      private:
        friend class Impl;
        CDenseVectorView m_solution;
        Impl*            m_impl = nullptr;
    };

  private:
    class LinearSubsytemInfo
    {
      public:
        bool  is_diag                  = false;
        bool  has_local_preconditioner = false;
        SizeT local_index              = ~0ull;
        SizeT index                    = ~0ull;
    };

  public:
    class Impl
    {
      public:
        void init();

        void build_linear_system();
        bool _update_subsystem_extent();
        void _assemble_linear_system();
        void _assemble_preconditioner();
        void solve_linear_system();
        void distribute_solution();

        Float reserve_ratio = 1.1;

        std::vector<LinearSubsytemInfo> subsystem_infos;

        OffsetCountCollection<IndexT> diag_dof_offsets_counts;
        OffsetCountCollection<IndexT> subsystem_triplet_offsets_counts;

        std::vector<SizeT2> off_diag_lr_triplet_counts;


        std::vector<int> accuracy_statisfied_flags;
        std::vector<int> no_precond_diag_subsystem_indices;

        // Containers
        SimSystemSlotCollection<DiagLinearSubsystem>    diag_subsystems;
        SimSystemSlotCollection<OffDiagLinearSubsystem> off_diag_subsystems;
        SimSystemSlotCollection<LocalPreconditioner>    local_preconditioners;

        SimSystemSlot<IterativeSolver>      iterative_solver;
        SimSystemSlot<GlobalPreconditioner> global_preconditioner;

        // Linear System
        muda::LinearSystemContext           ctx;
        muda::DeviceDenseVector<Float>      x;
        muda::DeviceDenseVector<Float>      b;
        muda::DeviceTripletMatrix<Float, 3> triplet_A;
        muda::DeviceBCOOMatrix<Float, 3>    bcoo_A;
        muda::DeviceDenseMatrix<Float>      debug_A;  // dense A for debug

        Spmv                      spmver;
        MatrixConverter<Float, 3> converter;

        // AGIPC coarse solve state lives at the global linear-system layer so the
        // same iterative solver can solve either fine or Galerkin coarse systems.
        AGIPCCoarseLinearSystem         agipc_coarse_system;
        AGIPCCoarseLinearSystem::Config agipc_coarse_config;
        muda::DeviceBuffer<Matrix3x3>   agipc_coarse_diag_inv;
        // Optional MAS (multi-level additive Schwarz) preconditioner for the coarse
        // PCG. Far stronger than the 3x3 block-Jacobi above; enabled via
        // linear_system/coarse/use_mas_preconditioner. Rebuilt per coarse solve
        // because the coarse aggregation topology changes every Newton iteration.
        MASPreconditionerEngine             agipc_coarse_mas;
        bool                                agipc_coarse_mas_ready = false;
        muda::DeviceBuffer<Eigen::Matrix3d> agipc_coarse_mas_values;
        muda::DeviceBuffer<int>             agipc_coarse_mas_rows;
        muda::DeviceBuffer<int>             agipc_coarse_mas_cols;
        muda::DeviceBuffer<uint32_t>        agipc_coarse_mas_indices;
        // Cache the MAS graph hierarchy across Newton iterations: the expensive
        // init_neighbor/init_matrix only depend on the coarse graph *structure*,
        // which is unchanged while the aggregation is stable (the common case in a
        // settled simulation). A (block_count, triplet_count, index-checksum) key
        // detects structural change; on a hit we skip the rebuild and only re-feed
        // the matrix values via set_preconditioner.
        SizeT                               agipc_mas_cached_blocks   = 0;
        SizeT                               agipc_mas_cached_triplets = 0;
        unsigned long long                  agipc_mas_cached_checksum = 0;
        muda::DeviceVar<unsigned long long> agipc_mas_checksum_var;
        bool                                agipc_solving_coarse = false;
        bool                                agipc_refining_fine  = false;
        SimSystemSlot<GlobalVertexManager>  global_vertex_manager;
        SimSystemSlot<GlobalSimplicialSurfaceManager> global_simplicial_surface_manager;
        SimSystemSlot<GlobalDyTopoEffectManager> global_dytopo_effect_manager;
        SimSystemSlot<AffineBodyVertexReporter>  affine_body_vertex_reporter;
        SimSystemSlot<AffineBodyBodyReporter>    affine_body_body_reporter;
        SimSystemSlot<FiniteElementMethod>       finite_element_method;
        SimSystemSlot<FiniteElementBodyReporter> finite_element_body_reporter;
        SimSystemSlot<FiniteElementVertexReporter> finite_element_vertex_reporter;
        muda::DeviceBuffer<IndexT> agipc_node_to_global_vertex;
        SizeT                      agipc_abd_node_count = 0;
        SizeT                      agipc_fem_node_count = 0;

        void rebuild_agipc_node_to_global_vertex_map();

        bool  initialized         = false;
        bool  empty_system        = true;
        SizeT current_frame       = 0;
        SizeT current_newton_iter = 0;

        void apply_preconditioner(muda::DenseVectorView<Float>  z,
                                  muda::CDenseVectorView<Float> r,
                                  muda::CVarView<IndexT>        converged);
        void assemble_agipc_coarse_preconditioner();
        void assemble_agipc_coarse_mas_preconditioner();
        void apply_agipc_coarse_preconditioner(muda::DenseVectorView<Float>  z,
                                               muda::CDenseVectorView<Float> r,
                                               muda::CVarView<IndexT> converged);

        void spmv(Float a, muda::CDenseVectorView<Float> x, Float b, muda::DenseVectorView<Float> y);
        void spmv_dot(muda::CDenseVectorView<Float> x,
                      muda::DenseVectorView<Float>  y,
                      muda::VarView<Float>          d_dot);
        void apply_fine_preconditioner(muda::DenseVectorView<Float>  z,
                                       muda::CDenseVectorView<Float> r,
                                       muda::CVarView<IndexT>        converged);

        bool accuracy_statisfied(muda::DenseVectorView<Float> r);
        void compute_gradient(ComputeGradientInfo& info);

        Float diag_norm();
        Float mass_norm();

        bool        need_debug_dump = false;
        std::string debug_dump_path;
    };

    SizeT dof_count() const;
    void  compute_gradient(ComputeGradientInfo& info);

    muda::LinearSystemContext& ctx() noexcept { return m_impl.ctx; }

  protected:
    void do_build() override;

  private:
    friend class SimEngine;
    friend class IterativeSolver;
    friend class DiagLinearSubsystem;
    friend class OffDiagLinearSubsystem;
    friend class LocalPreconditioner;
    friend class GlobalPreconditioner;
    friend class GlobalDiffSimManager;
    friend class CurrentFrameDiffDofReporter;

    void add_subsystem(DiagLinearSubsystem* subsystem);
    void add_subsystem(OffDiagLinearSubsystem* subsystem);
    void add_solver(IterativeSolver* solver);
    void add_preconditioner(LocalPreconditioner* preconditioner);
    void add_preconditioner(GlobalPreconditioner* preconditioner);

    // only be called by SimEngine::do_init();
    void init();

    // only be called by SimEngine::do_advance()
    void solve();

    // only be called by SimEngine::do_advance()
    Float diag_norm();
    Float mass_norm();

    Impl m_impl;

    // local debug dump functions
    void _dump_A_b();
    void _dump_x();
};
}  // namespace uipc::backend::cuda
