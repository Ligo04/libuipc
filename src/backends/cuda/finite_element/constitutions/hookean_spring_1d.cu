#include <utils/remaining_material_launch.h>
#include <finite_element/codim_1d_constitution.h>
#include <finite_element/codim_1d_constitution_diff_parm_reporter.h>
#include <finite_element/constitutions/hookean_spring_1d_function.h>
#include <kernel_cout.h>
#include <cuda_tool/cuda_tool.h>
#include <Eigen/Dense>
#include <utils/codim_thickness.h>
#include <utils/matrix_assembler.h>
#include <numbers>
#include <utils/fixed_bank_soa_evd.h>

namespace uipc::backend::cuda
{
namespace
{
    namespace NS = sym::hookean_spring_1d;

    constexpr SizeT StencilSize     = 2;
    constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    __global__ void HookeanSpring1D_do_compute_energy_kernel(
        cuda_tool::CBufferView<Float>    kappas,
        cuda_tool::CBufferView<Float>    rest_lengths,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::BufferView<Float>     energies,
        cuda_tool::CBufferView<Vector2i> indices,
        cuda_tool::CBufferView<Vector3>  xs,
        Float                            dt,
        double                           Pi,
        int                              n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        Vector6  X;
        Vector2i idx = indices(I);
        for(int i = 0; i < 2; ++i)
            X.segment<3>(3 * i) = xs(idx(i));

        Float L0    = rest_lengths(I);
        Float r     = edge_thickness(thicknesses(idx(0)), thicknesses(idx(1)));
        Float kappa = kappas(I);

        Float Vdt2 = L0 * r * r * Pi * dt * dt;

        Float E;
        NS::E(E, kappa, X, L0);
        energies(I) = E * Vdt2;
    }

    template <bool GradientOnly>
    __global__ void HookeanSpring1D_do_compute_gradient_hessian_kernel(
        cuda_tool::DoubletVectorView<Float, 3> G3s,
        cuda_tool::TripletMatrixView<Float, 3> H3x3s,
        cuda_tool::CBufferView<Float>          kappas,
        cuda_tool::CBufferView<Float>          rest_lengths,
        cuda_tool::CBufferView<Float>          thicknesses,
        cuda_tool::CBufferView<Vector2i>       indices,
        cuda_tool::CBufferView<Vector3>        xs,
        Float                                  dt,
        double                                 Pi,
        int                                    n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        Vector6  X;
        Vector2i idx = indices(I);
        for(int i = 0; i < 2; ++i)
            X.segment<3>(3 * i) = xs(idx(i));

        Float L0    = rest_lengths(I);
        Float r     = edge_thickness(thicknesses(idx(0)), thicknesses(idx(1)));
        Float kappa = kappas(I);

        Float Vdt2 = L0 * r * r * Pi * dt * dt;

        Vector6 G;
        NS::dEdX(G, kappa, X, L0);
        G *= Vdt2;
        DoubletVectorAssembler VA{G3s};
        VA.segment<StencilSize>(I * StencilSize).write(idx, G);

        if constexpr(!GradientOnly)
        {
            Matrix6x6 H;
            NS::ddEddX(H, kappa, X, L0);
            H *= Vdt2;
            constexpr int    SharedLanePitch = 32;
            __shared__ Float shared_h[6 * 6 * SharedLanePitch];
            FixedBankSoAMap<6, SharedLanePitch> H_workspace(shared_h + threadIdx.x);
            H_workspace = H;
            Vector<Float, 6> eigen_values;
            selfadjoint_evd_fixed_bank_shared<6>(H_workspace, eigen_values);
            TripletMatrixAssembler MA{H3x3s};
            MA.half_block<StencilSize>(I * HalfHessianSize)
                .write_psd_from_eigendecomposition(idx, H_workspace, eigen_values);
        }
    }
}  // namespace

void launch_hookean_spring_1d_gradient_hessian(const HookeanSpring1DGradientHessianLaunchInfo& info)
{
    int n = static_cast<int>(info.indices.size());
    if(n <= 0)
        return;
    if(info.gradient_only)
    {
        auto      k = HookeanSpring1D_do_compute_gradient_hessian_kernel<true>;
        const int block_size = info.block_size > 0 ? info.block_size : 32;
        k<<<(n + block_size - 1) / block_size, block_size, 0, nullptr>>>(info.G3s,
                                                                         info.H3x3s,
                                                                         info.kappas,
                                                                         info.rest_lengths,
                                                                         info.thicknesses,
                                                                         info.indices,
                                                                         info.xs,
                                                                         info.dt,
                                                                         info.Pi,
                                                                         n);
    }
    else
    {
        auto      k = HookeanSpring1D_do_compute_gradient_hessian_kernel<false>;
        const int BlockSize = info.block_size > 0 ? info.block_size : 32;
        UIPC_ASSERT(BlockSize <= 32, "Full CTA exceeds the fixed-bank workspace lane pitch.");
        k<<<(n + BlockSize - 1) / BlockSize, BlockSize, 0, nullptr>>>(info.G3s,
                                                                      info.H3x3s,
                                                                      info.kappas,
                                                                      info.rest_lengths,
                                                                      info.thicknesses,
                                                                      info.indices,
                                                                      info.xs,
                                                                      info.dt,
                                                                      info.Pi,
                                                                      n);
    }
}


class HookeanSpring1D final : public Codim1DConstitution
{
  public:
    // Constitution UID by libuipc specification
    static constexpr U64   ConstitutionUID = 12ull;
    static constexpr SizeT StencilSize     = 2;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    using Codim1DConstitution::Codim1DConstitution;

    vector<Float>                  h_kappas;
    cuda_tool::DeviceBuffer<Float> kappas;

    virtual U64 get_uid() const noexcept override { return ConstitutionUID; }

    virtual void do_build(BuildInfo& info) override {}

    virtual void do_report_extent(ReportExtentInfo& info) override
    {
        info.energy_count(kappas.size());
        info.gradient_count(kappas.size() * StencilSize);

        if(info.gradient_only())
            return;

        info.hessian_count(kappas.size() * HalfHessianSize);
    }

    virtual void do_init(FiniteElementMethod::FilteredInfo& info) override
    {
        using ForEachInfo = FiniteElementMethod::ForEachInfo;

        auto geo_slots = world().scene().geometries();

        auto N = info.primitive_count();

        h_kappas.resize(N);

        info.for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc) -> auto
            {
                auto kappa = sc.edges().find<Float>("kappa");
                return kappa->view();
            },
            [&](const ForEachInfo& I, Float kappa)
            {
                auto vI = I.global_index();
                // retrieve material parameters
                h_kappas[vI] = kappa;
            });

        kappas.resize(N);
        kappas.view().copy_from(h_kappas.data());
    }

    virtual void do_compute_energy(ComputeEnergyInfo& info) override
    {
        auto k = HookeanSpring1D_do_compute_energy_kernel;
        int  n = (int)info.indices().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                kappas.cview(),
                info.rest_lengths(),
                info.thicknesses(),
                info.energies(),
                info.indices(),
                info.xs(),
                info.dt(),
                std::numbers::pi,
                n);
        }
    }

    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        launch_hookean_spring_1d_gradient_hessian({info.gradients(),
                                                   info.hessians(),
                                                   kappas.cview(),
                                                   info.rest_lengths(),
                                                   info.thicknesses(),
                                                   info.indices(),
                                                   info.xs(),
                                                   info.dt(),
                                                   std::numbers::pi,
                                                   info.gradient_only()});
    }
};

REGISTER_SIM_SYSTEM(HookeanSpring1D);
}  // namespace uipc::backend::cuda
