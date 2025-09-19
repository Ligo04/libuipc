#include <contact_system/global_contact_manager.h>
#include <collision_detection/global_trajectory_filter.h>
#include <sim_engine.h>
#include <contact_system/contact_reporter.h>
#include <contact_system/contact_receiver.h>
#include <uipc/common/enumerate.h>
#include <kernel_cout.h>
#include <uipc/common/unit.h>
#include <uipc/common/zip.h>

namespace uipc::backend
{
template <>
class SimSystemCreator<cuda::GlobalContactManager>
{
  public:
    static U<cuda::GlobalContactManager> create(cuda::SimEngine& engine)
    {
        auto contact_enable_attr =
            engine.world().scene().config().find<IndexT>("contact/enable");
        bool contact_enable = contact_enable_attr->view()[0] != 0;

        auto& types = engine.world().scene().constitution_tabular().types();
        bool  has_inter_primitive_constitution =
            types.find(std::string{builtin::InterPrimitive}) != types.end();

        if(contact_enable || has_inter_primitive_constitution)
            return make_unique<cuda::GlobalContactManager>(engine);
        return nullptr;
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalContactManager);

void GlobalContactManager::do_build()
{
    const auto& config = world().scene().config();

    m_impl.global_vertex_manager    = require<GlobalVertexManager>();
    m_impl.global_trajectory_filter = find<GlobalTrajectoryFilter>();


    auto d_hat_attr = config.find<Float>("contact/d_hat");
    m_impl.d_hat    = d_hat_attr->view()[0];

    auto dt_attr = config.find<Float>("dt");
    m_impl.dt    = dt_attr->view()[0];

    auto eps_velocity_attr = config.find<Float>("contact/eps_velocity");
    m_impl.eps_velocity    = eps_velocity_attr->view()[0];

    auto cfl_enable_attr = config.find<IndexT>("cfl/enable");
    m_impl.cfl_enabled   = cfl_enable_attr->view()[0] != 0;

    m_impl.kappa = world().scene().contact_tabular().default_model().resistance();
}

muda::CBuffer2DView<IndexT> GlobalContactManager::contact_mask_tabular() const noexcept
{
    return m_impl.contact_mask_tabular;
}

muda::CBuffer2DView<IndexT> GlobalContactManager::subscene_contact_mask_tabular() const noexcept
{
    return m_impl.contact_mask_tabular_subscene;
}

muda::CBCOOVectorView<Float, 3> GlobalContactManager::contact_gradient() const noexcept
{
    return m_impl.sorted_contact_gradient.view();
}

muda::CBCOOMatrixView<Float, 3> GlobalContactManager::contact_hessian() const noexcept
{
    return m_impl.sorted_contact_hessian.view();
}

void GlobalContactManager::Impl::init(WorldVisitor& world)
{
    // 1) init tabular
    auto contact_models = world.scene().contact_tabular().contact_models();
    auto subscene_contact_models =
        world.scene().contact_tabular().subscene_contact_models();

    auto attr_topo          = contact_models.find<Vector2i>("topo");
    auto attr_resistance    = contact_models.find<Float>("resistance");
    auto attr_friction_rate = contact_models.find<Float>("friction_rate");
    auto attr_enabled       = contact_models.find<IndexT>("is_enabled");

    auto attr_subscene_topo = subscene_contact_models.find<Vector2i>("topo");
    auto attr_subscene_enabled = subscene_contact_models.find<IndexT>("is_enabled");

    UIPC_ASSERT(attr_topo != nullptr, "topo is not found in contact tabular");
    UIPC_ASSERT(attr_resistance != nullptr, "resistance is not found in contact tabular");
    UIPC_ASSERT(attr_friction_rate != nullptr, "friction_rate is not found in contact tabular");
    UIPC_ASSERT(attr_enabled != nullptr, "is_enabled is not found in contact tabular");

    UIPC_ASSERT(attr_subscene_topo != nullptr, "subscene topo is not found in contact tabular");
    UIPC_ASSERT(attr_subscene_enabled != nullptr,
                "subscene is_enabled is not found in contact tabular");

    auto topo_view            = attr_topo->view();
    auto resistance_view      = attr_resistance->view();
    auto friction_rate_view   = attr_friction_rate->view();
    auto enabled_view         = attr_enabled->view();
    auto subscene_topo_view   = attr_subscene_topo->view();
    auto subscene_enable_view = attr_subscene_enabled->view();

    auto N  = world.scene().contact_tabular().element_count();
    auto SN = world.scene().contact_tabular().subscene_element_count();

    h_contact_tabular.resize(
        N * N, ContactCoeff{.kappa = resistance_view[0], .mu = friction_rate_view[0]});

    h_contact_mask_tabular.resize(N * N, 1);
    h_contact_mask_tabular_subscene.resize(SN * SN, 1);

    for(auto&& [ids, kappa, mu, is_enabled] :
        zip(topo_view, resistance_view, friction_rate_view, enabled_view))
    {

        ContactCoeff coeff{.kappa = kappa, .mu = mu};

        auto upper                    = ids.x() * N + ids.y();
        h_contact_tabular[upper]      = coeff;
        h_contact_mask_tabular[upper] = is_enabled;

        auto lower                    = ids.y() * N + ids.x();
        h_contact_tabular[lower]      = coeff;
        h_contact_mask_tabular[lower] = is_enabled;
    }

    for(auto&& [ids, is_enabled] : zip(subscene_topo_view, subscene_enable_view))
    {
        auto upper                             = ids.x() * SN + ids.y();
        h_contact_mask_tabular_subscene[upper] = is_enabled;

        auto lower                             = ids.y() * SN + ids.x();
        h_contact_mask_tabular_subscene[lower] = is_enabled;
    }

    contact_tabular.resize(muda::Extent2D{N, N});
    contact_tabular.view().copy_from(h_contact_tabular.data());

    contact_mask_tabular.resize(muda::Extent2D{N, N});
    contact_mask_tabular.view().copy_from(h_contact_mask_tabular.data());

    contact_mask_tabular_subscene.resize(muda::Extent2D{SN, SN});
    contact_mask_tabular_subscene.view().copy_from(
        h_contact_mask_tabular_subscene.data());

    // 2) vertex contact info
    vert_is_active_contact.resize(global_vertex_manager->positions().size(), 0);
    vert_disp_norms.resize(global_vertex_manager->positions().size(), 0.0);

    // 3) reporters
    auto contact_reporter_view = contact_reporters.view();
    for(auto&& [i, R] : enumerate(contact_reporter_view))
        R->init();
    for(auto&& [i, R] : enumerate(contact_reporter_view))
        R->m_index = i;

    reporter_energy_offsets_counts.resize(contact_reporter_view.size());
    reporter_gradient_offsets_counts.resize(contact_reporter_view.size());
    reporter_hessian_offsets_counts.resize(contact_reporter_view.size());

    // 4) receivers
    auto contact_receiver_view = contact_receivers.view();
    for(auto&& [i, R] : enumerate(contact_receiver_view))
        R->init();
    for(auto&& [i, R] : enumerate(contact_receiver_view))
        R->m_index = i;

    classified_contact_gradients.resize(contact_receiver_view.size());
    classified_contact_hessians.resize(contact_receiver_view.size());
}

void GlobalContactManager::Impl::compute_d_hat()
{
    // TODO: Now do nothing
}

void GlobalContactManager::Impl::compute_adaptive_kappa()
{
    // TODO: Now do nothing
}

Float GlobalContactManager::Impl::compute_cfl_condition()
{
    if(!cfl_enabled)  // if cfl is disabled, just return 1.0
        return 1.0;

    vert_is_active_contact.fill(0);  // clear the active flag

    if(global_trajectory_filter)
    {
        global_trajectory_filter->label_active_vertices();

        auto displacements = global_vertex_manager->displacements();

        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(displacements.size(),
                   [disps      = displacements.cviewer().name("disp"),
                    disp_norms = vert_disp_norms.viewer().name("disp_norm"),
                    is_contact_active = vert_is_active_contact.viewer().name(
                        "vert_is_contact_active")] __device__(int i) mutable
                   {
                       // if the contact is not active, then the displacement is ignored
                       disp_norms(i) = is_contact_active(i) ? disps(i).norm() : 0.0;
                   });

        DeviceReduce().Max(vert_disp_norms.data(),
                           max_disp_norm.data(),
                           vert_disp_norms.size());

        Float h_max_disp_norm = max_disp_norm;
        return h_max_disp_norm == 0.0 ? 1.0 : std::min(0.5 * d_hat / h_max_disp_norm, 1.0);
    }
    else
    {
        return 1.0;
    }
}

void GlobalContactManager::Impl::compute_contact()
{
    _assemble();
    _convert_matrix();
    _distribute();
}

void GlobalContactManager::Impl::_assemble()
{
    auto vertex_count = global_vertex_manager->positions().size();

    auto reporter_gradient_counts = reporter_gradient_offsets_counts.counts();
    auto reporter_hessian_counts  = reporter_hessian_offsets_counts.counts();

    for(auto&& [i, reporter] : enumerate(contact_reporters.view()))
    {
        GradientHessianExtentInfo info;
        reporter->report_gradient_hessian_extent(info);
        reporter_gradient_counts[i] = info.m_gradient_count;
        reporter_hessian_counts[i]  = info.m_hessian_count;
        spdlog::info("<{}> contact Grad3 count: {}, contact Hess3x3 count: {}",
                     reporter->name(),
                     info.m_gradient_count,
                     info.m_hessian_count);
    }

    // scan
    reporter_gradient_offsets_counts.scan();
    reporter_hessian_offsets_counts.scan();

    auto total_gradient_count = reporter_gradient_offsets_counts.total_count();
    auto total_hessian_count  = reporter_hessian_offsets_counts.total_count();

    // allocate
    loose_resize_entries(collected_contact_gradient, total_gradient_count);
    loose_resize_entries(sorted_contact_gradient, total_gradient_count);
    loose_resize_entries(collected_contact_hessian, total_hessian_count);
    loose_resize_entries(sorted_contact_hessian, total_hessian_count);
    collected_contact_gradient.reshape(vertex_count);
    collected_contact_hessian.reshape(vertex_count, vertex_count);

    // collect
    for(auto&& [i, reporter] : enumerate(contact_reporters.view()))
    {
        auto [g_offset, g_count] = reporter_gradient_offsets_counts[i];
        auto [h_offset, h_count] = reporter_hessian_offsets_counts[i];


        GradientHessianInfo info;

        info.m_gradient = collected_contact_gradient.view().subview(g_offset, g_count);
        info.m_hessian = collected_contact_hessian.view().subview(h_offset, h_count);

        reporter->assemble(info);
    }
}

void GlobalContactManager::Impl::_convert_matrix()
{
    matrix_converter.convert(collected_contact_hessian, sorted_contact_hessian);
    matrix_converter.convert(collected_contact_gradient, sorted_contact_gradient);
}

void GlobalContactManager::Impl::_distribute()
{
    using namespace muda;

    auto vertex_count = global_vertex_manager->positions().size();

    for(auto&& [i, receiver] : enumerate(contact_receivers.view()))
    {
        ClassifyInfo info;
        receiver->report(info);

        auto& classified_gradients = classified_contact_gradients[i];
        classified_gradients.reshape(vertex_count);
        auto& classified_hessians = classified_contact_hessians[i];
        classified_hessians.reshape(vertex_count, vertex_count);

        // 1) report gradient
        if(info.is_diag())
        {
            const auto N = sorted_contact_gradient.doublet_count();

            // clear the range in device
            gradient_range = Vector2i{0, 0};

            // partition
            ParallelFor()
                .kernel_name(__FUNCTION__)
                .apply(N,
                       [gradient_range = gradient_range.viewer().name("gradient_range"),
                        contact_gradient =
                            std::as_const(sorted_contact_gradient).viewer().name("contact_gradient"),
                        range = info.m_gradient_i_range] __device__(int I) mutable
                       {
                           auto in_range = [](int i, const Vector2i& range)
                           { return i >= range.x() && i < range.y(); };

                           auto&& [i, G]      = contact_gradient(I);
                           bool this_in_range = in_range(i, range);

                           //cout << "I: " << I << ", i: " << i << ", G: " << G
                           //     << ", in_range: " << this_in_range << "\n";

                           if(!this_in_range)
                           {
                               return;
                           }

                           bool prev_in_range = false;
                           if(I > 0)
                           {
                               auto&& [prev_i, prev_G] = contact_gradient(I - 1);
                               prev_in_range = in_range(prev_i, range);
                           }
                           bool next_in_range = false;
                           if(I < contact_gradient.total_doublet_count() - 1)
                           {
                               auto&& [next_i, next_G] = contact_gradient(I + 1);
                               next_in_range = in_range(next_i, range);
                           }

                           // if the prev is not in range, then this is the start of the partition
                           if(!prev_in_range)
                           {
                               gradient_range->x() = I;
                           }
                           // if the next is not in range, then this is the end of the partition
                           if(!next_in_range)
                           {
                               gradient_range->y() = I + 1;
                           }
                       });

            Vector2i h_range = gradient_range;  // copy back

            auto count = h_range.y() - h_range.x();

            loose_resize_entries(classified_gradients, count);

            // fill
            if(count > 0)
            {
                ParallelFor()
                    .kernel_name(__FUNCTION__)
                    .apply(count,
                           [contact_gradient =
                                std::as_const(sorted_contact_gradient).viewer().name("contact_gradient"),
                            classified_gradient = classified_gradients.viewer().name("classified_gradient"),
                            range = h_range] __device__(int I) mutable
                           {
                               auto&& [i, G] = contact_gradient(range.x() + I);
                               classified_gradient(I).write(i, G);
                           });
            }
        }

        // 2) report hessian
        if(!info.is_empty())
        {
            const auto N = sorted_contact_hessian.triplet_count();

            // +1 for calculate the total count
            loose_resize(selected_hessian, N + 1);
            loose_resize(selected_hessian_offsets, N + 1);

            // select
            ParallelFor()
                .kernel_name(__FUNCTION__)
                .apply(
                    N,
                    [selected_hessian = selected_hessian.view(0, N).viewer().name("selected_hessian"),
                     last =
                         VarView<IndexT>{selected_hessian.data() + N}.viewer().name("last"),
                     contact_hessian = sorted_contact_hessian.cviewer().name("contact_hessian"),
                     i_range = info.m_hessian_i_range,
                     j_range = info.m_hessian_j_range] __device__(int I) mutable
                    {
                        auto&& [i, j, H] = contact_hessian(I);

                        auto in_range = [](int i, const Vector2i& range)
                        { return i >= range.x() && i < range.y(); };

                        selected_hessian(I) =
                            in_range(i, i_range) && in_range(j, j_range) ? 1 : 0;

                        // fill the last one as 0, so that we can calculate the total count
                        // during the exclusive scan
                        if(I == 0)
                            last = 0;
                    });

            // scan
            DeviceScan().ExclusiveSum(selected_hessian.data(),
                                      selected_hessian_offsets.data(),
                                      selected_hessian.size());

            IndexT h_total_count = 0;
            VarView<IndexT>{selected_hessian_offsets.data() + N}.copy_to(&h_total_count);

            loose_resize_entries(classified_hessians, h_total_count);

            // fill
            if(h_total_count > 0)
            {
                ParallelFor()
                    .kernel_name(__FUNCTION__)
                    .apply(N,
                           [selected_hessian = selected_hessian.cviewer().name("selected_hessian"),
                            selected_hessian_offsets =
                                selected_hessian_offsets.cviewer().name("selected_hessian_offsets"),
                            contact_hessian = sorted_contact_hessian.cviewer().name("contact_hessian"),
                            classified_hessian = classified_hessians.viewer().name("classified_hessian"),
                            i_range = info.m_hessian_i_range,
                            j_range = info.m_hessian_j_range] __device__(int I) mutable
                           {
                               if(selected_hessian(I))
                               {
                                   auto&& [i, j, H] = contact_hessian(I);
                                   auto offset = selected_hessian_offsets(I);

                                   classified_hessian(offset).write(i, j, H);
                               }
                           });
            }

            ClassifiedContactInfo classified_info;

            classified_info.m_gradient = classified_gradients.view();
            classified_info.m_hessian  = classified_hessians.view();

            receiver->receive(classified_info);
        }
    }
}

void GlobalContactManager::Impl::loose_resize_entries(muda::DeviceTripletMatrix<Float, 3>& m,
                                                      SizeT size)
{
    if(size > m.triplet_capacity())
    {
        m.reserve_triplets(size * reserve_ratio);
    }
    m.resize_triplets(size);
}

void GlobalContactManager::Impl::loose_resize_entries(muda::DeviceDoubletVector<Float, 3>& v,
                                                      SizeT size)
{
    if(size > v.doublet_capacity())
    {
        v.reserve_doublets(size * reserve_ratio);
    }
    v.resize_doublets(size);
}


void GlobalContactManager::ClassifyInfo::range(const Vector2i& LRange, const Vector2i& RRange)
{
    m_type             = Type::Range;
    m_hessian_i_range  = LRange;
    m_hessian_j_range  = RRange;
    m_gradient_i_range = Vector2i::Zero();
}

void GlobalContactManager::ClassifyInfo::range(const Vector2i& Range)
{
    m_type             = Type::Range;
    m_gradient_i_range = Range;
    m_hessian_i_range  = Range;
    m_hessian_j_range  = Range;
}

bool GlobalContactManager::ClassifyInfo::is_empty() const
{
    return m_hessian_i_range[0] == m_hessian_i_range[1]
           || m_hessian_j_range[0] == m_hessian_j_range[1];
}

bool GlobalContactManager::ClassifyInfo::is_diag() const
{
    return m_gradient_i_range[0] != m_gradient_i_range[1];
}

void GlobalContactManager::ClassifyInfo::sanity_check()
{
    if(is_diag())
    {
        UIPC_ASSERT(m_gradient_i_range.x() <= m_gradient_i_range.y(),
                    "Diagonal Contact Gradient Range is invalid, [{}, {})",
                    m_gradient_i_range.x(),
                    m_gradient_i_range.y());

        UIPC_ASSERT(m_hessian_i_range == m_hessian_j_range,
                    "Diagonal Contact Hessian must have the same i_range and j_range");
    }
    else
    {
        UIPC_ASSERT(m_gradient_i_range.x() == m_gradient_i_range.y(),
                    "Off-Diagonal Contact must not have Gradient Part");
    }

    UIPC_ASSERT(m_hessian_i_range.x() <= m_hessian_i_range.y(),
                "Contact Hessian Range-i is invalid");
    UIPC_ASSERT(m_hessian_j_range.x() <= m_hessian_j_range.y(),
                "Contact Hessian Range-j is invalid");
}
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
void GlobalContactManager::compute_d_hat()
{
    m_impl.compute_d_hat();
}

void GlobalContactManager::compute_contact()
{
    m_impl.compute_contact();
}

void GlobalContactManager::compute_adaptive_kappa()
{
    m_impl.compute_adaptive_kappa();
}

Float GlobalContactManager::compute_cfl_condition()
{
    return m_impl.compute_cfl_condition();
}

void GlobalContactManager::init()
{
    m_impl.init(world());
}

Float GlobalContactManager::d_hat() const
{
    return m_impl.d_hat;
}
Float GlobalContactManager::eps_velocity() const
{
    return m_impl.eps_velocity;
}
bool GlobalContactManager::cfl_enabled() const
{
    return m_impl.cfl_enabled;
}
void GlobalContactManager::add_reporter(ContactReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    UIPC_ASSERT(reporter != nullptr, "reporter is nullptr");
    m_impl.contact_reporters.register_subsystem(*reporter);
}
void GlobalContactManager::add_receiver(ContactReceiver* receiver)
{
    check_state(SimEngineState::BuildSystems, "add_receiver()");
    UIPC_ASSERT(receiver != nullptr, "receiver is nullptr");
    m_impl.contact_receivers.register_subsystem(*receiver);
}
muda::CBuffer2DView<ContactCoeff> GlobalContactManager::contact_tabular() const noexcept
{
    return m_impl.contact_tabular;
}
}  // namespace uipc::backend::cuda