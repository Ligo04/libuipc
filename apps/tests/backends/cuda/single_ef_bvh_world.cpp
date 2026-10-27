#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/particle.h>
#include <uipc/constitution/hookean_spring.h>
#include <uipc/core/contact_system_feature.h>
#include <uipc/core/feature_collection.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>

namespace
{
using namespace uipc;
using namespace uipc::core;
using namespace uipc::geometry;
using namespace uipc::constitution;

struct FrameSample
{
    vector<Vector3>      positions;
    std::array<SizeT, 4> contact_counts{};
    std::array<Float, 4> contact_energies{};
    Float                ccd_toi = 1.0;
};

enum class GeometryKind
{
    Triangles,
    Particles,
    RodsAndParticles,
    PureCodim
};

vector<std::string> single_ef_methods()
{
    vector<std::string> methods{"single_ef_bvh"};
    const auto          available =
        Scene::config_schema().at("entries").at("collision_detection/method").at("enum");
    if(std::ranges::find(available, Json("single_ef_optix")) != available.end())
        methods.emplace_back("single_ef_optix");
    return methods;
}

vector<FrameSample> run_contact_world(std::string_view method, GeometryKind kind)
{
    const bool with_particle = kind != GeometryKind::Triangles;
    const bool with_surface  = kind != GeometryKind::PureCodim;
    const auto workspace =
        std::filesystem::path{AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE)}
        / std::to_string(static_cast<int>(kind)) / std::string{method};
    std::filesystem::create_directories(workspace);
    Engine engine{"cuda", workspace.string()};
    World  world{engine};
    auto   config                           = test::Scene::default_config();
    config["collision_detection"]["method"] = std::string{method};
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["constitution"]       = "ipc";
    config["contact"]["friction"]["enable"] = false;
    config["contact"]["d_hat"]              = 0.05;
    config["newton"]["velocity_tol"]        = 1e-4;
    Scene scene{config};
    scene.contact_tabular().default_model(0.0, 1e4);
    auto            contact = scene.contact_tabular().default_element();
    NeoHookeanShell shell;
    const auto      moduli = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.3);

    const vector<Vector3> base_vertices =
        with_surface ?
            vector<Vector3>{{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}} :
            vector<Vector3>{{-0.8, 0, 0.1}, {0.8, 0, 0.1}};
    const vector<Vector3i> base_faces = {{0, 1, 2}, {0, 2, 3}};
    auto base = with_surface ? trimesh(base_vertices, base_faces) :
                               linemesh(base_vertices, vector<Vector2i>{{0, 1}});
    if(with_surface)
        shell.apply_to(base, moduli, 1e3, 0.001);
    else
        HookeanSpring{}.apply_to(base, 1.0_MPa, 1e3, 0.004);
    label_surface(base);
    contact.apply_to(base);
    auto fixed = base.vertices().find<IndexT>(builtin::is_fixed);
    REQUIRE(fixed);
    std::ranges::fill(view(*fixed), 1);
    scene.objects().create("fixed_surface")->geometries().create(base);

    // The moving triangle starts inside d_hat, above the finite fixed surface.
    const vector<Vector3> moving_vertices =
        with_surface ?
            vector<Vector3>{{-0.2, 0.025, -0.15}, {0.2, 0.025, -0.15}, {0, 0.025, 0.2}} :
            vector<Vector3>{{0.6, 0.025, -0.2}, {0.6, 0.025, 0.4}};
    const vector<Vector3i> moving_faces = {{0, 1, 2}};
    auto moving = with_surface ? trimesh(moving_vertices, moving_faces) :
                                 linemesh(moving_vertices, vector<Vector2i>{{0, 1}});
    if(with_surface)
        shell.apply_to(moving, moduli, 1e3, 0.001);
    else
        HookeanSpring{}.apply_to(moving, 1.0_MPa, 1e3, 0.004);
    label_surface(moving);
    contact.apply_to(moving);
    auto velocity = moving.vertices().find<Vector3>(builtin::velocity);
    REQUIRE(velocity);
    std::ranges::fill(view(*velocity), Vector3{0, -0.2, 0});
    auto [moving_slot, moving_rest] =
        scene.objects().create("moving_surface")->geometries().create(moving);
    vector<S<SimplicialComplexSlot>> observed = {moving_slot};

    if(with_particle)
    {
        const Float x = with_surface ? 0.6 : -0.4;
        const vector<Vector3> particle_vertices = {{x, 0.025, 0.1}, {x + 0.02, 0.025, 0.1}};
        auto point = pointcloud(particle_vertices);
        Particle{}.apply_to(point, 1e3, 0.005);
        label_surface(point);
        contact.apply_to(point);
        auto [slot, rest] = scene.objects().create("particle")->geometries().create(point);
        observed.push_back(slot);
    }
    if(kind == GeometryKind::RodsAndParticles)
    {
        const std::array<vector<Vector3>, 2> rod_vertices = {
            vector<Vector3>{{0.4, 0.065, 0.1}, {0.8, 0.065, 0.1}},
            vector<Vector3>{{0.6, 0.085, -0.2}, {0.6, 0.085, 0.4}}};
        for(SizeT i = 0; i < rod_vertices.size(); ++i)
        {
            auto rod = linemesh(rod_vertices[i], vector<Vector2i>{{0, 1}});
            HookeanSpring{}.apply_to(rod, 1.0_MPa, 1e3, 0.004);
            label_surface(rod);
            contact.apply_to(rod);
            auto [slot, rest] =
                scene.objects().create(fmt::format("rod_{}", i))->geometries().create(rod);
            observed.push_back(slot);
        }
    }

    world.init(scene);
    REQUIRE(world.is_valid());
    auto contact_system = world.features().find<ContactSystemFeature>();
    REQUIRE(contact_system);

    constexpr std::array<std::string_view, 4> primitive_types = {"PP+N", "PE+N", "PT+N", "EE+N"};
    vector<FrameSample> samples;
    for(int frame = 1; frame <= 2; ++frame)
    {
        CAPTURE(method, kind, frame);
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
        REQUIRE(world.is_valid());
        REQUIRE(world.frame() == frame);

        const auto stats = engine.frame_stats();
        REQUIRE(stats.at("completed").get<bool>());
        REQUIRE(stats.at("line_search_trials").get<IndexT>() > 0);
        FrameSample sample;
        sample.ccd_toi = stats.at("last_ccd_toi").get<Float>();
        REQUIRE(std::isfinite(sample.ccd_toi));
        REQUIRE(sample.ccd_toi > 0.0);
        REQUIRE(sample.ccd_toi <= 1.0);
        for(const auto& slot : observed)
            for(const auto& p : slot->geometry().positions().view())
            {
                REQUIRE(p.allFinite());
                REQUIRE(p.y() > 0.002);
                sample.positions.push_back(p);
            }
        for(SizeT i = 0; i < primitive_types.size(); ++i)
        {
            Geometry energy_geometry;
            contact_system->contact_energy(primitive_types[i], energy_geometry);
            auto energy = energy_geometry.instances().find<Float>("energy");
            REQUIRE(energy);
            sample.contact_counts[i] = energy_geometry.instances().size();
            for(Float value : view(*energy))
            {
                REQUIRE(std::isfinite(value));
                REQUIRE(value >= 0.0);
                sample.contact_energies[i] += value;
            }
        }
        samples.push_back(std::move(sample));
    }
    SizeT active_contacts = 0;
    for(const auto& sample : samples)
        for(auto count : sample.contact_counts)
            active_contacts += count;
    REQUIRE(active_contacts > 0);
    REQUIRE((samples.back().positions.front() - moving_vertices.front()).norm() > 1e-8);
    return samples;
}
}  // namespace

TEST_CASE("single EF BVH consumes contact candidates in a real world",
          "[cuda][collision detection][single_ef_bvh]")
{
    const auto method =
        GENERATE_COPY(Catch::Generators::from_range(single_ef_methods()));
    CAPTURE(method);
    auto kind = GeometryKind::Triangles;
    SECTION("pure triangle surfaces") {}
    SECTION("triangle surfaces and a codimensional particle")
    {
        kind = GeometryKind::Particles;
    }
    SECTION("triangle surfaces, crossing rods and particles")
    {
        kind = GeometryKind::RodsAndParticles;
    }
    SECTION("pure rods and particles retain the baseline fallback")
    {
        kind = GeometryKind::PureCodim;
    }
    const auto baseline_method =
        method == "single_ef_optix" ? "single_ef_bvh" : "info_stackless_bvh";
    const auto baseline  = run_contact_world(baseline_method, kind);
    const auto single_ef = run_contact_world(method, kind);
    REQUIRE(single_ef.size() == baseline.size());
    for(SizeT frame = 0; frame < baseline.size(); ++frame)
    {
        CAPTURE(kind, frame);
        const auto& expected = baseline[frame];
        const auto& actual   = single_ef[frame];
        REQUIRE(actual.positions.size() == expected.positions.size());
        for(SizeT vertex = 0; vertex < expected.positions.size(); ++vertex)
            REQUIRE((actual.positions[vertex] - expected.positions[vertex]).norm() < 1e-7);
        REQUIRE(actual.contact_counts == expected.contact_counts);
        for(SizeT i = 0; i < expected.contact_energies.size(); ++i)
            REQUIRE(actual.contact_energies[i]
                    == Catch::Approx(expected.contact_energies[i]).epsilon(1e-5).margin(1e-10));
        REQUIRE(actual.ccd_toi == Catch::Approx(expected.ccd_toi).margin(1e-8));
    }
}
