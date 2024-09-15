#include <affine_body/affine_body_dynamics.h>
#include <affine_body/abd_line_search_reporter.h>
#include <affine_body/affine_body_constitution.h>
#include <Eigen/Dense>
#include <algorithm>
#include <uipc/common/enumerate.h>
#include <uipc/common/range.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/algorithm/run_length_encode.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/common/zip.h>
#include <muda/cub/device/device_reduce.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <muda/ext/eigen/evd.h>
#include <fstream>


namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyDynamics);

void AffineBodyDynamics::do_build()
{
    const auto& scene = world().scene();
    auto&       types = scene.constitution_tabular().types();
    if(types.find(constitution::ConstitutionType::AffineBody) == types.end())
    {
        throw SimSystemException("No AffineBodyConstitution found in the scene");
    }

    // find dependent systems
    auto& dof_predictor = require<DoFPredictor>();

    // Register the action to initialize the affine body geometry
    on_init_scene([this] { m_impl.init(world()); });

    // Register the action to predict the affine body dof
    dof_predictor.on_predict(*this,
                             [this](DoFPredictor::PredictInfo& info)
                             { m_impl.compute_q_tilde(info); });

    // Register the action to compute the velocity of the affine body dof
    dof_predictor.on_compute_velocity(*this,
                                      [this](DoFPredictor::ComputeVelocityInfo& info)
                                      { m_impl.compute_q_v(info); });

    // Register the action to write the scene
    on_write_scene([this] { m_impl.write_scene(world()); });
}

bool AffineBodyDynamics::do_dump(DumpInfo& info)
{
    return m_impl.dump(info);
}

bool AffineBodyDynamics::do_try_recover(RecoverInfo& info)
{
    return m_impl.try_recover(info);
}

void AffineBodyDynamics::do_apply_recover(RecoverInfo& info)
{
    m_impl.apply_recover(info);
}

void AffineBodyDynamics::do_clear_recover(RecoverInfo& info)
{
    m_impl.clear_recover(info);
}
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
void AffineBodyDynamics::add_constitution(AffineBodyConstitution* constitution)
{
    check_state(SimEngineState::BuildSystems, "add_constitution()");
    // set the temp index, later we will sort constitution by uid
    // and reset the index
    m_impl.constitutions.register_subsystem(*constitution);
}

void AffineBodyDynamics::Impl::init(WorldVisitor& world)
{
    _build_geo_infos(world);

    _build_body_infos(world);
    _build_related_infos(world);
    _setup_geometry_attributes(world);

    _build_geometry_on_host(world);
    _build_geometry_on_device(world);

    _distribute_body_infos();
}

void AffineBodyDynamics::Impl::_build_body_infos(WorldVisitor& world)
{
    // 1) sort the constitutions by uid
    auto constitution_view = constitutions.view();

    std::ranges::sort(constitution_view,
                      [](const AffineBodyConstitution* a, AffineBodyConstitution* b)
                      { return a->constitution_uid() < b->constitution_uid(); });
    for(auto&& [i, C] : enumerate(constitution_view))  // reset the index
        C->m_index = i;

    vector<U64> filter_uids(constitution_view.size());
    std::ranges::transform(constitution_view,
                           filter_uids.begin(),
                           [](const AffineBodyConstitution* filter)
                           { return filter->constitution_uid(); });

    unordered_map<U64, IndexT> constitution_uid_to_index;
    for(auto&& [i, filter] : enumerate(constitution_view))
        constitution_uid_to_index[filter->constitution_uid()] = i;

    // 2) find the affine bodies
    list<BodyInfo> body_infos;
    auto           scene = world.scene();

    auto geo_slots = scene.geometries();
    auto N         = geo_slots.size();
    abd_body_count = 0;
    for(auto&& [i, geo_slot] : enumerate(geo_slots))
    {
        auto& geo  = geo_slot->geometry();
        auto  cuid = geo.meta().find<U64>(builtin::constitution_uid);
        if(cuid)  // if has constitution uid
        {
            auto uid            = cuid->view()[0];
            auto instance_count = geo.instances().size();
            if(std::binary_search(filter_uids.begin(), filter_uids.end(), uid))
            {
                auto* sc = geo.as<geometry::SimplicialComplex>();
                UIPC_ASSERT(sc,
                            "The geometry is not a simplicial complex (it's {}). Why can it happen?",
                            geo.type());

                auto vert_count = sc->vertices().size();
                for(auto&& j : range(instance_count))
                {
                    // push all instances of the geometry to the body_infos
                    BodyInfo info;
                    info.m_constitution_uid    = uid;
                    info.m_constitution_index  = constitution_uid_to_index[uid];
                    info.m_abd_geometry_index  = abd_geo_count;
                    info.m_geometry_slot_index = i;
                    info.m_geometry_instance_index = j;
                    info.m_affine_body_id = -1;  // keep it as -1, we will set it after sorting
                    info.m_vertex_count = vert_count;
                    info.m_vertex_offset = -1;  // keep it as -1, we will set it after sorting
                    body_infos.push_back(std::move(info));

                    abd_body_count++;
                }
                abd_geo_count++;
            }
        }
    }

    // 3) sort the body infos by (constitution uid, geometry index, geometry instance index)
    h_body_infos.resize(body_infos.size());

    std::partial_sort_copy(body_infos.begin(),
                           body_infos.end(),
                           h_body_infos.begin(),
                           h_body_infos.end(),
                           [](const BodyInfo& a, const BodyInfo& b)
                           {
                               return a.m_constitution_uid < b.m_constitution_uid
                                      || a.m_constitution_uid == b.m_constitution_uid
                                             && a.m_geometry_slot_index < b.m_geometry_slot_index
                                      || a.m_constitution_uid == b.m_constitution_uid
                                             && a.m_geometry_slot_index == b.m_geometry_slot_index
                                             && a.m_geometry_instance_index
                                                    < b.m_geometry_instance_index;
                           });

    for(auto&& [I, info] : enumerate(h_body_infos))
    {
        info.m_affine_body_id = I;
    }

    // 4) setup the vertex offset, vertex_count
    vector<SizeT> vertex_count(h_body_infos.size() + 1, 0);
    vector<SizeT> vertex_offset(h_body_infos.size() + 1);
    std::ranges::transform(h_body_infos,
                           vertex_count.begin(),
                           [&](const BodyInfo& info) -> SizeT
                           { return info.m_vertex_count; });

    std::exclusive_scan(vertex_count.begin(), vertex_count.end(), vertex_offset.begin(), 0);
    abd_vertex_count = vertex_offset.back();
    for(auto&& [i, info] : enumerate(h_body_infos))
    {
        info.m_vertex_count  = vertex_count[i];
        info.m_vertex_offset = vertex_offset[i];
    }
}

void AffineBodyDynamics::Impl::_build_related_infos(WorldVisitor& world)
{
    // Note: body_infos has been sorted by (constitution uid, geometry index, geometry instance index)


    auto constitution_view = constitutions.view();

    // 1) setup h_abd_geo_body_offsets and h_abd_geo_body_counts
    {
        h_abd_geo_body_offsets.resize(abd_geo_count + 1);
        h_abd_geo_body_counts.resize(abd_geo_count + 1);

        std::ranges::fill(h_abd_geo_body_counts, 0);

        for(auto&& body_info : h_body_infos)
            h_abd_geo_body_counts[body_info.m_abd_geometry_index]++;

        std::exclusive_scan(h_abd_geo_body_counts.begin(),
                            h_abd_geo_body_counts.end(),
                            h_abd_geo_body_offsets.begin(),
                            0);

        UIPC_ASSERT(abd_body_count == h_abd_geo_body_offsets.back(), "size mismatch");

        h_abd_geo_body_offsets.resize(abd_geo_count);
        h_abd_geo_body_counts.resize(abd_geo_count);
    }

    // 2) setup h_constitution_geo_offsets and h_constitution_geo_counts
    {
        auto constitution_view = constitutions.view();

        h_constitution_geo_offsets.resize(constitution_view.size() + 1);
        h_constitution_geo_counts.resize(constitution_view.size() + 1);

        std::ranges::fill(h_constitution_geo_counts, 0);

        for(IndexT last_geo_slot_index = -1; auto&& body_info : h_body_infos)
        {
            if(body_info.m_geometry_slot_index != last_geo_slot_index)
            {
                h_constitution_geo_counts[body_info.m_constitution_index]++;
                last_geo_slot_index = body_info.m_geometry_slot_index;
            }
        }

        std::exclusive_scan(h_constitution_geo_counts.begin(),
                            h_constitution_geo_counts.end(),
                            h_constitution_geo_offsets.begin(),
                            0);

        UIPC_ASSERT(abd_geo_count == h_constitution_geo_offsets.back(), "size mismatch");
    }

    // 3) setup h_constitution_body_offsets and h_constitution_body_counts
    {
        h_constitution_body_offsets.resize(constitution_view.size() + 1);
        h_constitution_body_counts.resize(constitution_view.size() + 1);

        std::ranges::fill(h_constitution_body_counts, 0);

        for(auto&& body_info : h_body_infos)
            h_constitution_body_counts[body_info.m_constitution_index]++;

        std::exclusive_scan(h_constitution_body_counts.begin(),
                            h_constitution_body_counts.end(),
                            h_constitution_body_offsets.begin(),
                            0);

        UIPC_ASSERT(abd_body_count == h_constitution_body_offsets.back(), "size mismatch");
    }
}

void AffineBodyDynamics::Impl::_setup_geometry_attributes(WorldVisitor& world)
{
    // set the `backend_abd_body_offset` attribute in simplicial complex geometry
    auto geo_slots = world.scene().geometries();
    for(auto&& body_offset : h_abd_geo_body_offsets)
    {
        const auto& info = h_body_infos[body_offset];
        auto&       sc   = geometry(geo_slots, info);
        auto abd_body_offset = sc.meta().find<IndexT>(builtin::backend_abd_body_offset);
        if(!abd_body_offset)
            abd_body_offset = sc.meta().create<IndexT>(builtin::backend_abd_body_offset);
        auto body_offset_view = geometry::view(*abd_body_offset);
        std::ranges::fill(body_offset_view, body_offset);
    }
}

void AffineBodyDynamics::Impl::_build_geometry_on_host(WorldVisitor& world)
{
    auto scene             = world.scene();
    auto geo_slots         = scene.geometries();
    auto constitution_view = constitutions.view();

    // 1) setup `q` for every affine body

    {
        SizeT I = 0;
        h_body_id_to_q.resize(h_body_infos.size());
        for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            { return sc.transforms().view(); },
            [&](SizeT local_i, const Matrix4x4& trans)
            {
                Vector12& q     = h_body_id_to_q[I++];
                q.segment<3>(0) = trans.block<3, 1>(0, 3);

                Float D = trans.block<3, 3>(0, 0).determinant();

                if(!Eigen::Vector<Float, 1>{D}.isApproxToConstant(1.0, 1e-6))
                    spdlog::warn("The determinant of the rotation matrix is not 1.0, but {}. Don't apply scaling on Affine Body.",
                                 D);

                q.segment<3>(3) = trans.block<1, 3>(0, 0).transpose();
                q.segment<3>(6) = trans.block<1, 3>(1, 0).transpose();
                q.segment<3>(9) = trans.block<1, 3>(2, 0).transpose();
            });
    }


    // 2) setup `J` and `mass` for every vertex
    h_vertex_id_to_J.resize(abd_vertex_count);
    h_vertex_id_to_mass.resize(abd_vertex_count);
    span Js          = h_vertex_id_to_J;
    span vertex_mass = h_vertex_id_to_mass;
    std::ranges::for_each(h_body_infos,
                          [&](BodyInfo& info)
                          {
                              auto& sc = geometry(geo_slots, info);

                              auto offset = info.m_vertex_offset;
                              auto count  = info.m_vertex_count;

                              // copy init the J from position
                              auto pos_view = sc.positions().view();
                              auto sub_Js   = Js.subspan(offset, count);
                              std::ranges::copy(pos_view, sub_Js.begin());

                              auto mass = sc.vertices().find<Float>(builtin::mass);
                              UIPC_ASSERT(mass, "The mass attribute is not found in the geometry. Why can it happen?");
                              auto mass_view = mass->view();

                              auto sub_mass = vertex_mass.subspan(offset, count);
                              std::ranges::copy(mass_view, sub_mass.begin());
                          });


    // 3) setup `vertex_id_to_body_id` and `vertex_id_to_contact_element_id`
    h_vertex_id_to_body_id.resize(abd_vertex_count);
    h_vertex_id_to_contact_element_id.resize(abd_vertex_count);
    auto v2b = span{h_vertex_id_to_body_id};
    auto v2c = span{h_vertex_id_to_contact_element_id};
    std::ranges::for_each(
        h_body_infos,
        [&](const BodyInfo& info)
        {
            auto offset = info.m_vertex_offset;
            auto count  = info.m_vertex_count;
            std::ranges::fill(v2b.subspan(offset, count), info.m_affine_body_id);
            const auto& geo = geometry(geo_slots, info);
            auto contact_element_id = geo.meta().find<IndexT>(builtin::contact_element_id);
            if(contact_element_id)
            {
                auto cid_view = contact_element_id->view();
                std::ranges::fill(v2c.subspan(offset, count), cid_view[0]);
            }
            else
            {
                std::ranges::fill(v2c.subspan(offset, count), 0);
            }
        });

    // 4) setup body_abd_mass and body_id_to_volume
    vector<ABDJacobiDyadicMass> geo_masses(abd_geo_count, ABDJacobiDyadicMass{});
    vector<Float> geo_volumes(abd_geo_count, 0.0);

    for(auto&& [I, body_offset] : enumerate(h_abd_geo_body_offsets))
    {
        const auto& info     = h_body_infos[body_offset];
        auto&       geo_mass = geo_masses[I];
        auto&       geo_vol  = geo_volumes[I];


        auto& sc = geometry(geo_slots, info);

        auto offset = info.m_vertex_offset;
        auto count  = info.m_vertex_count;

        // compute the mass of the geometry
        // sub_Js for this body
        auto sub_Js   = Js.subspan(offset, count);
        auto sub_mass = vertex_mass.subspan(offset, count);
        for(auto&& [m, J] : zip(sub_mass, sub_Js))
            geo_mass += ABDJacobiDyadicMass{m, J.x_bar()};

        // compute the volume of the geometry
        auto Vs = sc.positions().view();
        auto Ts = sc.tetrahedra().topo().view();

        for(const auto& t : Ts)
        {
            auto [p0, p1, p2, p3] = std::tuple{Vs[t[0]], Vs[t[1]], Vs[t[2]], Vs[t[3]]};

            Eigen::Matrix<Float, 3, 3> A;
            A.col(0) = p1 - p0;
            A.col(1) = p2 - p0;
            A.col(2) = p3 - p0;
            auto D   = A.determinant();
            UIPC_ASSERT(D > 0.0,
                        "The determinant of the tetrahedron is non-positive ({}), which means the tetrahedron is inverted.",
                        D);

            auto volume = D / 6.0;
            geo_vol += volume;
        }
    }

    h_body_id_to_abd_mass.resize(h_body_infos.size());
    h_body_id_to_volume.resize(h_body_infos.size());

    for(auto&& [I, info] : enumerate(h_body_infos))
    {
        auto& mass = h_body_id_to_abd_mass[info.m_affine_body_id];
        auto& vol  = h_body_id_to_volume[info.m_affine_body_id];

        mass = geo_masses[info.m_abd_geometry_index];
        vol  = geo_volumes[info.m_abd_geometry_index];
    }

    // 5) compute the inverse of the mass matrix
    h_body_id_to_abd_mass_inv.resize(h_body_infos.size());
    std::ranges::transform(h_body_id_to_abd_mass,
                           h_body_id_to_abd_mass_inv.begin(),
                           [](const ABDJacobiDyadicMass& mass) -> Matrix12x12
                           { return mass.to_mat().inverse(); });

    // 6) setup the affine body gravity
    Vector3 gravity = scene.info()["gravity"];
    h_body_id_to_abd_gravity.resize(h_body_infos.size(), Vector12::Zero());
    if(!gravity.isApprox(Vector3::Zero()))
    {
        vector<Vector12> geo_gravities(abd_geo_count, Vector12::Zero());

        std::ranges::transform(
            h_abd_geo_body_offsets,
            geo_gravities.begin(),
            [&](SizeT offset) -> Vector12
            {
                const auto& info          = h_body_infos[offset];
                auto        vertex_offset = info.m_vertex_offset;
                auto        vertex_count  = info.m_vertex_count;

                // sub_Js for this body
                auto sub_Js = Js.subspan(vertex_offset, vertex_count);
                auto sub_mass = vertex_mass.subspan(vertex_offset, vertex_count);

                Vector12 G = Vector12::Zero();

                for(auto&& [m, J] : zip(sub_mass, sub_Js))
                    G += m * (J.T() * gravity);

                const Matrix12x12& M_inv = h_body_id_to_abd_mass_inv[info.m_affine_body_id];

                return M_inv * G;
            });

        std::ranges::transform(h_body_infos,
                               h_body_id_to_abd_gravity.begin(),
                               [&](const BodyInfo& info) -> Vector12 {
                                   return geo_gravities[info.m_abd_geometry_index];
                               });
    }

    // 7) setup the boundary type
    {
        SizeT I = 0;
        h_body_id_to_is_fixed.resize(h_body_infos.size(), 0);
        for_each(
            geo_slots,
            [](geometry::SimplicialComplex& sc)
            { return sc.instances().find<IndexT>(builtin::is_fixed)->view(); },
            [&](SizeT local_i, IndexT fixed)
            { h_body_id_to_is_fixed[I++] = fixed; });
    }


    // 8) misc
    h_constitution_shape_energy.resize(constitution_view.size(), 0.0);
}

void AffineBodyDynamics::Impl::_build_geometry_on_device(WorldVisitor& world)
{
    auto async_copy = []<typename T>(span<T> src, muda::DeviceBuffer<T>& dst)
    {
        muda::BufferLaunch().resize<T>(dst, src.size());
        muda::BufferLaunch().copy<T>(dst.view(), src.data());
    };

    async_copy(span{h_body_id_to_q}, body_id_to_q);
    async_copy(span{h_vertex_id_to_J}, vertex_id_to_J);
    async_copy(span{h_vertex_id_to_body_id}, vertex_id_to_body_id);
    async_copy(span{h_body_id_to_abd_mass}, body_id_to_abd_mass);
    async_copy(span{h_body_id_to_volume}, body_id_to_volume);
    async_copy(span{h_body_id_to_abd_mass_inv}, body_id_to_abd_mass_inv);
    async_copy(span{h_body_id_to_abd_gravity}, body_id_to_abd_gravity);
    async_copy(span{h_body_id_to_is_fixed}, body_id_to_is_fixed);

    auto async_transfer = []<typename T>(const muda::DeviceBuffer<T>& src,
                                         muda::DeviceBuffer<T>&       dst)
    {
        muda::BufferLaunch().resize<T>(dst, src.size());
        muda::BufferLaunch().copy<T>(dst.view(), src.view());
    };

    async_transfer(body_id_to_q, body_id_to_q_temp);
    async_transfer(body_id_to_q, body_id_to_q_tilde);
    async_transfer(body_id_to_q, body_id_to_q_prev);

    auto async_resize = []<typename T>(muda::DeviceBuffer<T>& buf, SizeT size, const T& value)
    {
        muda::BufferLaunch().resize<T>(buf, size);
        muda::BufferLaunch().fill<T>(buf.view(), value);
    };

    async_resize(body_id_to_q_v, h_body_infos.size(), Vector12::Zero().eval());
    async_resize(body_id_to_dq, h_body_infos.size(), Vector12::Zero().eval());

    // setup body kinetic energy buffer
    async_resize(body_id_to_kinetic_energy, h_body_infos.size(), 0.0);

    async_resize(body_id_to_shape_energy, h_body_infos.size(), 0.0);

    // setup hessian and gradient buffers
    async_resize(diag_hessian, h_body_infos.size(), Matrix12x12::Zero().eval());
    async_resize(body_id_to_body_hessian,
                 h_body_infos.size(),
                 Matrix12x12::Zero().eval());
    async_resize(body_id_to_body_gradient, h_body_infos.size(), Vector12::Zero().eval());

    muda::wait_device();
}

void AffineBodyDynamics::Impl::_distribute_body_infos()
{
    // _distribute the body infos to each constitution
    for(auto&& [I, constitution] : enumerate(constitutions.view()))
    {
        FilteredInfo info{this};
        info.m_constitution_index = I;
        constitution->retrieve(info);
    }
}

void AffineBodyDynamics::Impl::write_scene(WorldVisitor& world)
{
    // 1) download from device to host
    _download_geometry_to_host();

    auto scene     = world.scene();
    auto geo_slots = scene.geometries();

    span qs = h_body_id_to_q;

    // 2) transfer from affine_body_dynamics qs to transforms
    SizeT I = 0;
    for_each(
        geo_slots,
        [](geometry::SimplicialComplex& sc) { return view(sc.transforms()); },
        [&](SizeT i, Matrix4x4& trans)
        {
            Vector12& q = h_body_id_to_q[I++];

            trans                   = Matrix4x4::Identity();
            trans.block<3, 1>(0, 3) = q.segment<3>(0);  // translation
            // rotation
            trans.block<1, 3>(0, 0) = q.segment<3>(3).transpose();
            trans.block<1, 3>(1, 0) = q.segment<3>(6).transpose();
            trans.block<1, 3>(2, 0) = q.segment<3>(9).transpose();
        });
}

void AffineBodyDynamics::Impl::_download_geometry_to_host()
{
    using namespace muda;

    auto aync_copy = []<typename T>(muda::DeviceBuffer<T>& src, span<T> dst)
    { muda::BufferLaunch().copy<T>(dst.data(), src.view()); };

    aync_copy(body_id_to_q, span{h_body_id_to_q});

    muda::wait_device();
}

void AffineBodyDynamics::Impl::compute_q_tilde(DoFPredictor::PredictInfo& info)
{
    using namespace muda;
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(body_count(),
               [is_fixed = body_id_to_is_fixed.cviewer().name("btype"),
                q_prevs  = body_id_to_q_prev.cviewer().name("q_prev"),
                q_vs     = body_id_to_q_v.cviewer().name("q_velocities"),
                q_tildes = body_id_to_q_tilde.viewer().name("q_tilde"),
                affine_gravity = body_id_to_abd_gravity.cviewer().name("affine_gravity"),
                dt = info.dt()] __device__(int i) mutable
               {
                   auto& q_prev = q_prevs(i);
                   auto& q_v    = q_vs(i);
                   auto& g      = affine_gravity(i);
                   // TODO: this time, we only consider gravity
                   if(is_fixed(i))
                   {
                       q_tildes(i) = q_prev;
                   }
                   else
                   {
                       q_tildes(i) = q_prev + q_v * dt + g * (dt * dt);
                   }
               });
}

void AffineBodyDynamics::Impl::compute_q_v(DoFPredictor::ComputeVelocityInfo& info)
{
    using namespace muda;
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(body_count(),
               [is_fixed = body_id_to_is_fixed.cviewer().name("btype"),
                qs       = body_id_to_q.cviewer().name("qs"),
                q_vs     = body_id_to_q_v.viewer().name("q_vs"),
                q_prevs  = body_id_to_q_prev.viewer().name("q_prevs"),
                dt       = info.dt()] __device__(int i) mutable
               {
                   auto& q_v    = q_vs(i);
                   auto& q_prev = q_prevs(i);

                   const auto& q = qs(i);

                   if(is_fixed(i))
                       q_v = Vector12::Zero();
                   else
                       q_v = (q - q_prev) * (1.0 / dt);

                   q_prev = q;
               });
}

bool AffineBodyDynamics::Impl::dump(DumpInfo& info)
{
    auto path = info.dump_path(__FILE__);

    return dump_q.dump(path + "q", body_id_to_q)                     //
           && dump_q_v.dump(path + "q_v", body_id_to_q_v)            //
           && dump_q_prev.dump(path + "q_prev", body_id_to_q_prev);  //
}

bool AffineBodyDynamics::Impl::try_recover(RecoverInfo& info)
{
    auto path = info.dump_path(__FILE__);
    return dump_q.load(path + "q")                //
           && dump_q_v.load(path + "q_v")         //
           && dump_q_prev.load(path + "q_prev");  //
}


void AffineBodyDynamics::Impl::apply_recover(RecoverInfo& info)
{
    auto path = info.dump_path(__FILE__);

    auto qs = dump_q.view<Vector12>();

    dump_q.apply_to(body_id_to_q);
    dump_q_v.apply_to(body_id_to_q_v);
    dump_q_prev.apply_to(body_id_to_q_prev);
}

void AffineBodyDynamics::Impl::clear_recover(RecoverInfo& info)
{
    dump_q.clean_up();
    dump_q_v.clean_up();
    dump_q_prev.clean_up();
}

geometry::SimplicialComplex& AffineBodyDynamics::Impl::geometry(
    span<S<geometry::GeometrySlot>> geo_slots, const BodyInfo& body_info)
{
    auto& geo = geo_slots[body_info.m_geometry_slot_index]->geometry();
    auto  sc  = geo.as<geometry::SimplicialComplex>();
    UIPC_ASSERT(sc,
                "The geometry is not a simplicial complex (it's {}). Why can it happen?",
                geo.type());
    return *sc;
}
}  // namespace uipc::backend::cuda