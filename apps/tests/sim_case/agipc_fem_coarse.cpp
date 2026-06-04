// AGIPC coarse-mode regression test.
//
// Runs a real multi-tet FEM body with linear_system/coarse/enable=1 so that the
// algebraic-coarsening path actually aggregates nodes (the Galerkin assembly,
// union-find aggregation, coarse PCG, prolongation and fine refinement all run).
// A pure-ABD scene never aggregates (its coupling edges are all contact- or
// first-iteration-protected), so it cannot exercise these paths — a FEM body is
// required. This test guards against regressions such as the union-find
// infinite-spin that only triggers once real edge collapse happens.
#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>

TEST_CASE("agipc_fem_coarse", "[agipc]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    std::string this_output_path =
        fmt::format("{}", AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE));

    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config = Scene::default_config();

    config["gravity"]                           = Vector3{0, -9.8, 0};
    config["contact"]["enable"]                 = false;
    config["line_search"]["max_iter"]           = 8;
    config["linear_system"]["tol_rate"]         = 1e-3;
    config["linear_system"]["coarse"]["enable"] = 1;

    test::Scene::dump_config(config, this_output_path);

    SimplicialComplexIO io;
    Scene               scene{config};
    {
        StableNeoHookean snh;
        auto             object = scene.objects().create("bunny");

        auto mesh = io.read(fmt::format("{}bunny0.msh", tetmesh_dir));
        label_surface(mesh);
        label_triangle_orient(mesh);

        auto parm = ElasticModuli::youngs_poisson(1e5, 0.49);
        snh.apply_to(mesh, parm, 1e3);

        object->geometries().create(mesh);
    }

    world.init(scene);
    REQUIRE(world.is_valid());

    for(int i = 0; i < 3; ++i)
    {
        world.advance();
        REQUIRE(world.is_valid());
        world.retrieve();
    }
}
