#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/discrete_shell_bending.h>
#include <uipc/constitution/finite_element_external_force.h>
#include <uipc/core/finite_element_state_accessor_feature.h>
#include <algorithm>
#include <cmath>

namespace
{
using namespace uipc;
using namespace uipc::core;
using namespace uipc::geometry;
using namespace uipc::constitution;

SimplicialComplex make_tet(Float height)
{
    vector<Vector3> vertices = {
        {0, height, 0}, {0.1, height, 0}, {0, height + 0.1, 0}, {0, height, 0.1}};
    vector<Vector4i> tets = {{0, 1, 2, 3}};
    auto             mesh = tetmesh(vertices, tets);
    label_surface(mesh);
    label_triangle_orient(mesh);
    StableNeoHookean{}.apply_to(mesh, ElasticModuli::youngs_poisson(1e4, 0.3));
    return mesh;
}

TEST_CASE("MAS-PNCG config selects a nonlinear solver without PCG", "[cuda][mas_pncg]")
{
    for(auto mode : {"incremental", "frozen", "full_rebuild"})
    {
        CAPTURE(mode);
        auto   output = fmt::format("{}freeflight-{}/",
                                  AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE),
                                  mode);
        Engine engine{"cuda", output};
        World  world{engine};
        auto   config                        = Scene::default_config();
        config["nonlinear_system"]["solver"] = "mas_pncg";
        config["nonlinear_system"]["mas_pncg"]["update_mode"] = mode;
        config["contact"]["enable"]                           = 0;
        config["extras"]["strict_mode"]["enable"]             = 1;
        Scene           scene{config};
        auto            object = scene.objects().create("tet");
        auto            mesh   = make_tet(0.2);
        vector<Vector3> initial(mesh.positions().view().begin(),
                                mesh.positions().view().end());
        auto [slot, rest] = object->geometries().create(mesh);
        world.init(scene);
        REQUIRE(world.is_valid());
        for(int frame = 1; frame <= 12; ++frame)
        {
            world.advance();
            REQUIRE(world.is_valid());
            world.retrieve();
            auto stats = engine.frame_stats();
            REQUIRE(stats.at("completed") == true);
            REQUIRE(stats.at("converged") == true);
            REQUIRE(stats.at("nonlinear_solver") == "mas_pncg");
            REQUIRE(stats.at("newton_iterations") == 0);
            REQUIRE(stats.at("linear_solver_iterations") == 0);
            REQUIRE(stats.at("max_local_residual_ratio").get<Float>() <= 1);
            auto positions =
                slot->geometry().as<SimplicialComplex>()->positions().view();
            Vector3 displacement{0, -9.8 * 0.01 * 0.01 * frame * (frame + 1) / 2, 0};
            for(SizeT i = 0; i < positions.size(); ++i)
                REQUIRE((positions[i] - initial[i] - displacement).norm() < 1e-9);
        }
    }
}

TEST_CASE("MAS-PNCG stationary fixed FEM needs no artificial step", "[cuda][mas_pncg]")
{
    Engine engine{"cuda", fmt::format("{}fixed/", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE))};
    World world{engine};
    auto  config                         = Scene::default_config();
    config["nonlinear_system"]["solver"] = "mas_pncg";
    config["contact"]["enable"]          = 0;
    Scene scene{config};
    auto  mesh = make_tet(0.2);
    std::ranges::fill(view(*mesh.vertices().find<IndexT>(builtin::is_fixed)), 1);
    scene.objects().create("fixed")->geometries().create(mesh);
    world.init(scene);
    REQUIRE(world.is_valid());
    world.advance();
    REQUIRE(world.is_valid());
    auto stats = engine.frame_stats();
    REQUIRE(stats.at("converged") == true);
    REQUIRE(stats.at("accepted_steps") == 0);
    REQUIRE(stats.at("nonlinear_iterations") == 0);
    REQUIRE(stats.at("line_search_trials") == 0);
}

TEST_CASE("MAS-PNCG rejects affine coordinates and quasi-static FEM at initialization", "[cuda][mas_pncg]")
{
    for(auto mode : {"abd", "mixed", "quasi-static"})
    {
        CAPTURE(mode);
        Engine engine{"cuda",
                      fmt::format("{}unsupported-{}/",
                                  AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE),
                                  mode)};
        World  world{engine};
        auto   config                        = Scene::default_config();
        config["nonlinear_system"]["solver"] = "mas_pncg";
        config["contact"]["enable"]          = 0;
        Scene scene{config};
        auto  object = scene.objects().create("unsupported");
        auto  mesh   = make_tet(0.2);
        if(std::string_view{mode} == "quasi-static")
            std::ranges::fill(view(*mesh.vertices().find<IndexT>(builtin::is_dynamic)), 0);
        else
            AffineBodyConstitution{}.apply_to(mesh, 1e6);
        object->geometries().create(mesh);
        if(std::string_view{mode} == "mixed")
            object->geometries().create(make_tet(0.4));
        world.init(scene);
        REQUIRE_FALSE(world.is_valid());
        REQUIRE(engine.frame_stats().at("completed") == false);
    }
}

TEST_CASE("MAS-PNCG keeps native frictional plane contact feasible", "[cuda][mas_pncg]")
{
    Engine engine{"cuda", fmt::format("{}plane/", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE))};
    World world{engine};
    auto  config                              = Scene::default_config();
    config["nonlinear_system"]["solver"]      = "mas_pncg";
    config["extras"]["strict_mode"]["enable"] = 1;
    Scene scene{config};
    scene.contact_tabular().default_model(0.3, 1e6);
    auto object     = scene.objects().create("tet-and-plane");
    auto mesh       = make_tet(0.06);
    auto velocities = view(*mesh.vertices().find<Vector3>(builtin::velocity));
    std::ranges::fill(velocities, Vector3{0.05, 0, 0});
    auto [slot, rest] = object->geometries().create(mesh);
    object->geometries().create(ground(0));
    world.init(scene);
    REQUIRE(world.is_valid());
    Float final_min = 1;
    SizeT rebuilds = 0, steps = 0, columns = 0, contacts = 0;
    for(int frame = 0; frame < 45; ++frame)
    {
        world.advance();
        INFO(engine.frame_stats().dump());
        REQUIRE(world.is_valid());
        world.retrieve();
        auto stats = engine.frame_stats();
        REQUIRE(stats.at("converged") == true);
        REQUIRE(stats.at("max_local_residual_ratio").get<Float>() <= 1);
        rebuilds += stats.at("base_rebuilds").get<SizeT>();
        steps += stats.at("accepted_steps").get<SizeT>();
        columns += stats.at("contact_update_columns").get<SizeT>();
        contacts += stats.at("normal_contacts").get<SizeT>();
        final_min = 1;
        for(auto& p : slot->geometry().as<SimplicialComplex>()->positions().view())
        {
            REQUIRE(p.allFinite());
            REQUIRE(p.y() > 0);
            final_min = std::min(final_min, p.y());
        }
    }
    REQUIRE(final_min < 0.02);
    REQUIRE(steps > rebuilds);
    REQUIRE(contacts > 0);
    REQUIRE(columns > 0);
}

TEST_CASE("MAS-PNCG exhausted budget does not complete or commit velocity", "[cuda][mas_pncg]")
{
    Engine engine{"cuda", fmt::format("{}budget/", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE))};
    World world{engine};
    auto  config                         = Scene::default_config();
    config["nonlinear_system"]["solver"] = "mas_pncg";
    config["nonlinear_system"]["mas_pncg"]["max_iter"] = 1;
    Scene scene{config};
    scene.contact_tabular().default_model(0.3, 1e6);
    auto object       = scene.objects().create("tet-and-plane");
    auto [slot, rest] = object->geometries().create(make_tet(0.001));
    object->geometries().create(ground(0));
    world.init(scene);
    REQUIRE(world.is_valid());
    world.advance();
    REQUIRE_FALSE(world.is_valid());
    REQUIRE(engine.frame_stats().at("completed") == false);
    REQUIRE(engine.frame_stats().at("converged") == false);
    auto accessor = world.features().find<FiniteElementStateAccessorFeature>();
    REQUIRE(accessor);
    auto state = accessor->create_geometry();
    state.vertices().create<Vector3>(builtin::position);
    auto velocity = state.vertices().create<Vector3>(builtin::velocity);
    accessor->copy_to(state);
    for(auto& v : velocity->view())
        REQUIRE(v.norm() == 0);
    for(auto& p : state.positions().view())
    {
        REQUIRE(p.allFinite());
        REQUIRE(p.y() > 0);
    }
}

TEST_CASE("MAS-PNCG rejects numerical failures without committing native state", "[cuda][mas_pncg]")
{
    bool energy_failure = false;
    SECTION("overflowed coarse direction") {}
    SECTION("nonfinite native trial energy")
    {
        energy_failure = true;
    }
    Engine engine{"cuda", fmt::format("{}nonfinite/", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE))};
    World world{engine};
    auto  config                         = Scene::default_config();
    config["nonlinear_system"]["solver"] = "mas_pncg";
    config["contact"]["enable"]          = 0;
    config["gravity"]                    = Vector3::Zero().eval();
    if(energy_failure)
        config["sanity_check"]["enable"] = 0;
    Scene scene{config};
    auto  mesh = make_tet(energy_failure ? 0 : 0.2);
    FiniteElementExternalForce{}.apply_to(mesh, Vector3::Zero().eval());
    auto enabled = view(*mesh.vertices().find<IndexT>("external_force/is_constrained"));
    std::ranges::fill(enabled, 0);
    auto forces = view(*mesh.vertices().find<Vector3>("external_force"));
    if(energy_failure)
    {
        // Keep mass and the gradient inside the coarse float range while
        // isotropic trial deformation makes the native J^2 energy overflow.
        // This synthetic stress input disables absolute-geometry sanity checks.
        for(auto& position : view(mesh.positions()))
            position *= 1e-29;
        StableNeoHookean{}.apply_to(mesh, ElasticModuli::youngs_poisson(1e4, 0.3), 1e90);
        view(*mesh.vertices().find<IndexT>(builtin::is_fixed))[0] = 1;
        for(int i = 1; i < 4; ++i)
        {
            enabled[i] = 1;
            forces[i]  = 1e30 * Vector3::Unit(i - 1);
        }
    }
    else
    {
        enabled[0] = 1;
        forces[0]  = Vector3{1e120, 0, 0};
    }
    vector<Vector3> initial(mesh.positions().view().begin(),
                            mesh.positions().view().end());
    scene.objects().create("forced-tet")->geometries().create(mesh);
    world.init(scene);
    REQUIRE(world.is_valid());
    world.advance();
    REQUIRE_FALSE(world.is_valid());
    auto stats = engine.frame_stats();
    INFO(stats.dump());
    REQUIRE(stats.at("completed") == false);
    REQUIRE(stats.at("converged") == false);
    REQUIRE(stats.at("restarts") == 1);
    REQUIRE(stats.at("hit_line_search_limit") == energy_failure);
    if(energy_failure)
    {
        REQUIRE(stats.at("rejected_trials").get<SizeT>() >= 2);
        REQUIRE(stats.at("nonfinite_trials").get<SizeT>() >= 2);
    }
    else
    {
        REQUIRE(stats.at("line_search_trials") == 0);
        REQUIRE(stats.at("nonfinite_trials") == 0);
    }
    auto accessor = world.features().find<FiniteElementStateAccessorFeature>();
    REQUIRE(accessor);
    auto state = accessor->create_geometry();
    state.vertices().create<Vector3>(builtin::position);
    auto velocities = state.vertices().create<Vector3>(builtin::velocity);
    accessor->copy_to(state);
    auto positions = state.positions().view();
    for(SizeT i = 0; i < initial.size(); ++i)
    {
        REQUIRE((positions[i] - initial[i]).norm() == 0);
        REQUIRE(velocities->view()[i].norm() == 0);
    }
}

TEST_CASE("MAS-PNCG cloth self-contact retains fixed vertices and bending", "[cuda][mas_pncg]")
{
    Engine engine{"cuda", fmt::format("{}cloth/", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE))};
    World world{engine};
    auto  config                         = Scene::default_config();
    config["nonlinear_system"]["solver"] = "mas_pncg";
    Scene scene{config};
    scene.contact_tabular().default_model(0.3, 1e6);
    vector<Vector3>  vertices  = {{-0.05, 0, -0.05},
                                  {0.05, 0, -0.05},
                                  {0.05, 0, 0.05},
                                  {-0.05, 0, 0.05},
                                  {-0.04, 0.06, -0.04},
                                  {0.06, 0.06, -0.04},
                                  {0.06, 0.06, 0.06},
                                  {-0.04, 0.06, 0.06}};
    vector<Vector3i> triangles = {{0, 1, 2}, {0, 2, 3}, {4, 5, 6}, {4, 6, 7}};
    auto             mesh      = trimesh(vertices, triangles);
    label_surface(mesh);
    NeoHookeanShell{}.apply_to(mesh, ElasticModuli2D::youngs_poisson(1e4, 0.3), 200, 0.001);
    DiscreteShellBending{}.apply_to(mesh, 1e4, 0.3);
    auto fixed = view(*mesh.vertices().find<IndexT>(builtin::is_fixed));
    for(int i = 0; i < 4; ++i)
        fixed[i] = 1;
    auto [slot, rest] = scene.objects().create("two-patches")->geometries().create(mesh);
    world.init(scene);
    REQUIRE(world.is_valid());
    Float final_min = 1;
    for(int frame = 0; frame < 24; ++frame)
    {
        world.advance();
        INFO(engine.frame_stats().dump());
        REQUIRE(world.is_valid());
        world.retrieve();
        auto positions = slot->geometry().as<SimplicialComplex>()->positions().view();
        for(int i = 0; i < 4; ++i)
            REQUIRE((positions[i] - vertices[i]).norm() == 0);
        final_min = 1;
        for(int i = 4; i < 8; ++i)
        {
            REQUIRE(positions[i].allFinite());
            REQUIRE(positions[i].y() > 0.0019);
            final_min = std::min(final_min, positions[i].y());
        }
        REQUIRE(engine.frame_stats().at("converged") == true);
    }
    REQUIRE(final_min < 0.02);
}
}  // namespace
