#include <utils/remaining_material_launch.h>
#include <inter_primitive_effect_system/inter_primitive_constitution.h>
#include <inter_primitive_effect_system/constitutions/soft_vertex_stitch_function.h>
#include <uipc/builtin/attribute_name.h>
#include <utils/matrix_assembler.h>
#include <utils/fixed_bank_soa_evd.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void SoftVertexStitch_do_compute_energy_kernel(
        cuda_tool::CBufferView<Vector2i> topos,
        cuda_tool::CBufferView<Vector3>  xs,
        cuda_tool::CBufferView<Float>    kappas,
        cuda_tool::CBufferView<Float>    rest_lengths,
        cuda_tool::BufferView<Float>     Es,
        Float                            dt,
        int                              n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        const Vector2i& PP  = topos(I);
        Float           Kt2 = kappas(I) * dt * dt;
        Float           L0  = rest_lengths(I);
        Vector3         dx  = xs(PP[0]) - xs(PP[1]);
        if(L0 == 0.0)
        {
            Es(I) = 0.5 * Kt2 * dx.squaredNorm();
        }
        else
        {
            Float dist = dx.norm();
            Float diff = dist - L0;
            Es(I)      = 0.5 * Kt2 * diff * diff;
        }
    }

    template <bool GradientOnly, SizeT StencilSize, SizeT HalfHessianSize>
    __global__ void SoftVertexStitch_do_compute_gradient_hessian_kernel(
        cuda_tool::CBufferView<Vector2i>          topos,
        cuda_tool::CBufferView<Vector3>           xs,
        cuda_tool::CBufferView<Float>             kappas,
        cuda_tool::CBufferView<Float>             rest_lengths,
        cuda_tool::DoubletVectorView<Float, 3>    G3s,
        cuda_tool::TripletMatrixView<Float, 3, 3> H3x3s,
        Float                                     dt,
        int                                       n)
    {
        int I = blockIdx.x * blockDim.x + threadIdx.x;
        if(I >= n)
            return;
        namespace SVS = sym::soft_vertex_stitch;

        Vector6         X;
        const Vector2i& PP = topos(I);
        for(int i = 0; i < 2; ++i)
            X.segment<3>(3 * i) = xs(PP[i]);

        Float   Kt2 = kappas(I) * dt * dt;
        Float   L0  = rest_lengths(I);
        Vector3 dx  = X.head<3>() - X.tail<3>();

        Vector6 G;
        if(L0 == 0.0)
        {
            G.head<3>() = Kt2 * dx;
            G.tail<3>() = -Kt2 * dx;
        }
        else
            SVS::dEdX(G, Kt2, X, L0);

        DoubletVectorAssembler VA{G3s};
        VA.segment<StencilSize>(I * StencilSize).write(PP, G);

        if constexpr(!GradientOnly)
        {
            Matrix6x6 H;
            if(L0 == 0.0)
            {
                Matrix3x3 blk       = Kt2 * Matrix3x3::Identity();
                H.block<3, 3>(0, 0) = blk;
                H.block<3, 3>(0, 3) = -blk;
                H.block<3, 3>(3, 0) = -blk;
                H.block<3, 3>(3, 3) = blk;
            }
            else
                SVS::ddEddX(H, Kt2, X, L0);
            constexpr int    SharedLanePitch = 32;
            __shared__ Float shared_h[6 * 6 * SharedLanePitch];
            FixedBankSoAMap<6, SharedLanePitch> H_workspace(shared_h + threadIdx.x);
            H_workspace = H;
            Vector<Float, 6> eigen_values;
            selfadjoint_evd_fixed_bank_shared<6>(H_workspace, eigen_values);
            TripletMatrixAssembler MA{H3x3s};
            MA.half_block<StencilSize>(I * HalfHessianSize)
                .write_psd_from_eigendecomposition(PP, H_workspace, eigen_values);
        }
    }
}  // namespace

void launch_soft_vertex_stitch_gradient_hessian(const SoftVertexStitchGradientHessianLaunchInfo& info)
{
    int n = static_cast<int>(info.topos.size());
    if(n <= 0)
        return;
    if(info.gradient_only)
    {
        auto k = SoftVertexStitch_do_compute_gradient_hessian_kernel<true, 2, 3>;
        const int block_size = info.block_size > 0 ? info.block_size : 32;
        k<<<(n + block_size - 1) / block_size, block_size, 0, nullptr>>>(
            info.topos, info.xs, info.kappas, info.rest_lengths, info.G3s, info.H3x3s, info.dt, n);
    }
    else
    {
        auto k = SoftVertexStitch_do_compute_gradient_hessian_kernel<false, 2, 3>;
        const int BlockSize = info.block_size > 0 ? info.block_size : 32;
        UIPC_ASSERT(BlockSize <= 32, "Full CTA exceeds the fixed-bank workspace lane pitch.");
        k<<<(n + BlockSize - 1) / BlockSize, BlockSize, 0, nullptr>>>(
            info.topos, info.xs, info.kappas, info.rest_lengths, info.G3s, info.H3x3s, info.dt, n);
    }
}


class SoftVertexStitch : public InterPrimitiveConstitution
{
  public:
    static constexpr U64   ConstitutionUID = 22;
    static constexpr SizeT StencilSize     = 2;
    static constexpr SizeT HalfHessianSize = StencilSize * (StencilSize + 1) / 2;

    using InterPrimitiveConstitution::InterPrimitiveConstitution;

    U64 get_uid() const noexcept override { return ConstitutionUID; }

    void do_build(BuildInfo& info) override {}

    cuda_tool::DeviceBuffer<Vector2i> topos;
    cuda_tool::DeviceBuffer<Float>    kappas;
    cuda_tool::DeviceBuffer<Float>    rest_lengths;

    vector<Vector2i> h_topos;
    vector<Float>    h_kappas;
    vector<Float>    h_rest_lengths;

    cuda_tool::CBufferView<Float>              energies;
    cuda_tool::CDoubletVectorView<Float, 3>    gradients;
    cuda_tool::CTripletMatrixView<Float, 3, 3> hessians;

    void do_init(FilteredInfo& info) override
    {
        list<Vector2i> topo_buffer;
        list<Float>    kappa_buffer;
        list<Float>    rest_length_buffer;

        auto geo_slots    = world().scene().geometries();
        using ForEachInfo = InterPrimitiveConstitutionManager::ForEachInfo;
        info.for_each(
            geo_slots,
            [&](const ForEachInfo& I, geometry::Geometry& geo)
            {
                auto topo = geo.instances().find<Vector2i>(builtin::topo);
                UIPC_ASSERT(topo, "SoftVertexStitch requires attribute `topo` on instances()");
                auto geo_ids = geo.meta().find<Vector2i>("geo_ids");
                UIPC_ASSERT(geo_ids, "SoftVertexStitch requires attribute `geo_ids` on meta()");
                auto kappa = geo.instances().find<Float>("kappa");
                UIPC_ASSERT(kappa, "SoftVertexStitch requires attribute `kappa` on instances()");
                auto rest_length = geo.instances().find<Float>("rest_length");
                UIPC_ASSERT(rest_length, "SoftVertexStitch requires attribute `rest_length` on instances()");
                Vector2i ids = geo_ids->view()[0];

                auto l_slot = info.geo_slot(ids[0]);
                auto r_slot = info.geo_slot(ids[1]);

                auto l_geo = l_slot->geometry().as<geometry::SimplicialComplex>();
                UIPC_ASSERT(l_geo,
                            "SoftVertexStitch requires simplicial complex geometry, but yours {} ({})",
                            l_slot->geometry().type(),
                            l_slot->id());
                auto r_geo = r_slot->geometry().as<geometry::SimplicialComplex>();
                UIPC_ASSERT(r_geo,
                            "SoftVertexStitch requires simplicial complex geometry, but yours {} ({})",
                            r_slot->geometry().type(),
                            r_slot->id());

                auto l_offset = l_geo->meta().find<IndexT>(builtin::global_vertex_offset);
                UIPC_ASSERT(l_offset,
                            "SoftVertexStitch requires attribute `global_vertex_offset` on meta() of geometry {} ({})",
                            l_slot->geometry().type(),
                            l_slot->id());
                IndexT l_offset_v = l_offset->view()[0];
                auto r_offset = r_geo->meta().find<IndexT>(builtin::global_vertex_offset);
                UIPC_ASSERT(r_offset,
                            "SoftVertexStitch requires attribute `global_vertex_offset` on meta() of geometry {} ({})",
                            r_slot->geometry().type(),
                            r_slot->id());
                IndexT r_offset_v = r_offset->view()[0];

                auto topo_view = topo->view();
                for(auto& v : topo_view)
                {
                    topo_buffer.push_back(Vector2i{v[0] + l_offset_v, v[1] + r_offset_v});
                }

                auto kappa_view = kappa->view();
                for(auto kappa : kappa_view)
                {
                    kappa_buffer.push_back(kappa);
                }

                auto rest_length_view = rest_length->view();
                for(auto rl : rest_length_view)
                {
                    UIPC_ASSERT(rl >= 0, "rest_length must be non-negative");
                    rest_length_buffer.push_back(rl);
                }
            });

        h_topos.resize(topo_buffer.size());
        std::ranges::move(topo_buffer, h_topos.begin());
        topos.copy_from(h_topos);

        h_kappas.resize(kappa_buffer.size());
        std::ranges::move(kappa_buffer, h_kappas.begin());
        kappas.copy_from(h_kappas);

        h_rest_lengths.resize(rest_length_buffer.size());
        std::ranges::move(rest_length_buffer, h_rest_lengths.begin());
        rest_lengths.copy_from(h_rest_lengths);
    }

    void do_report_energy_extent(EnergyExtentInfo& info) override
    {
        info.energy_count(topos.size());
    }

    void do_compute_energy(ComputeEnergyInfo& info) override
    {
        using namespace cuda_tool;

        energies = info.energies();

        auto k = SoftVertexStitch_do_compute_energy_kernel;
        int  n = (int)topos.size();
        if(n > 0)
        {
            k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                topos.cview(),
                info.positions().cviewer(),
                kappas.cview(),
                rest_lengths.cview(),
                info.energies().viewer(),
                info.dt(),
                n);
        }
    }

    void do_report_gradient_hessian_extent(GradientHessianExtentInfo& info) override
    {
        info.gradient_count(StencilSize * topos.size());

        if(info.gradient_only())
            return;

        info.hessian_count(HalfHessianSize * topos.size());
    }

    void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override
    {
        gradients = info.gradients();
        hessians  = info.hessians();
        launch_soft_vertex_stitch_gradient_hessian({topos.cview(),
                                                    info.positions().cviewer(),
                                                    kappas.cview(),
                                                    rest_lengths.cview(),
                                                    info.gradients().viewer(),
                                                    info.hessians().viewer(),
                                                    info.dt(),
                                                    info.gradient_only()});
    }
};

REGISTER_SIM_SYSTEM(SoftVertexStitch);
}  // namespace uipc::backend::cuda
