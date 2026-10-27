#include <utils/specialized_gradient_hessian_launch.h>
#include <finite_element/finite_element_kinetic.h>
#include <time_integrator/bdf2_flag.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void FiniteElementBDF2Kinetic_do_compute_energy_kernel(
        cuda_tool::CBufferView<IndexT>  is_fixed,
        cuda_tool::CBufferView<Vector3> xs,
        cuda_tool::CBufferView<Vector3> x_tildes,
        cuda_tool::CBufferView<Float>   masses,
        cuda_tool::BufferView<Float>    Ks,
        Float                           inv_beta,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& K = Ks(i);

        if(is_fixed(i))
        {
            K = 0.0;
        }
        else
        {
            const auto& x       = xs(i);
            const auto& x_tilde = x_tildes(i);
            const auto& M       = masses(i);
            const auto  dx      = x - x_tilde;

            K = inv_beta / 2 * dx.dot(M * dx);
        }
    }

    template <bool GradientOnly>
    __global__ void FiniteElementBDF2Kinetic_do_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<IndexT>            is_fixed,
        cuda_tool::CBufferView<Vector3>           xs,
        cuda_tool::CBufferView<Vector3>           x_tildes,
        cuda_tool::CBufferView<Float>             masses,
        cuda_tool::DoubletVectorView<Float, 3>    G3s,
        cuda_tool::TripletMatrixView<Float, 3, 3> H3x3s,
        Float                                     inv_beta,
        int                                       n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& m       = masses(i);
        auto& x       = xs(i);
        auto& x_tilde = x_tildes(i);

        Vector3 G;

        if(is_fixed(i))
        {
            G = Vector3::Zero();
        }
        else
        {
            G = inv_beta * m * (x - x_tilde);
        }

        G3s(i).write(i, G);

        if constexpr(!GradientOnly)
        {
            const Matrix3x3 H = (inv_beta * masses(i)) * Matrix3x3::Identity();
            H3x3s(i).write(i, i, H);
        }
    }
}  // namespace

void launch_fem_bdf2_gradient_hessian(const FEMBDF2GradientHessianLaunchInfo& info)
{
    const int n = static_cast<int>(info.xs.size());
    if(n <= 0)
        return;
    const int block_size = cuda_tool::best_block_dim(
        FiniteElementBDF2Kinetic_do_compute_gradient_hessian_kernel<false>);
    auto launch = [&]<bool GradientOnly>()
    {
        auto k = FiniteElementBDF2Kinetic_do_compute_gradient_hessian_kernel<GradientOnly>;
        k<<<(n + block_size - 1) / block_size, block_size, 0, nullptr>>>(
            info.is_fixed, info.xs, info.x_tildes, info.masses, info.G3s, info.H3x3s, info.inv_beta, n);
    };
    if(info.gradient_only)
        launch.template operator()<true>();
    else
        launch.template operator()<false>();
}

class FiniteElementBDF2Kinetic final : public FiniteElementKinetic
{
  public:
    static constexpr Float beta     = 4.0 / 9.0;  // BDF2 beta coefficient
    static constexpr Float inv_beta = Float{1} / beta;

    using FiniteElementKinetic::FiniteElementKinetic;

    void do_build(BuildInfo& info) override
    {
        // need BDF2 flag for BDF2 time integration
        require<BDF2Flag>();
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        auto k = FiniteElementBDF2Kinetic_do_compute_energy_kernel;
        int  n = (int)info.xs().size();
        if(n > 0)
        {
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                info.is_fixed().cview(),
                info.xs().cview(),
                info.x_tildes().cview(),
                info.masses().cview(),
                info.energies(),
                inv_beta,
                n);
        }
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        launch_fem_bdf2_gradient_hessian({info.is_fixed().cview(),
                                          info.xs().cview(),
                                          info.x_tildes(),
                                          info.masses().cview(),
                                          info.gradients(),
                                          info.hessians(),
                                          inv_beta,
                                          info.gradient_only()});
    }
};

REGISTER_SIM_SYSTEM(FiniteElementBDF2Kinetic);
}  // namespace uipc::backend::cuda
