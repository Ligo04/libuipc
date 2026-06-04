#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <cstdlib>
#include <limits>

// Stiff elastic FEM balls settling under gravity onto a ground plane.
//
// This reproduces the scene class where AGIPC reports its largest speedups
// (paper: "elastic ball E=1e7, stiff materials benefit most", 3.3x): high Young's
// modulus -> tiny deformation -> almost every interior edge is collapsible, so each
// ball aggregates into a few supernodes and the coarse system becomes much smaller
// than the fine one. Contact is sparse (ball-ball / ball-ground), unlike the
// contact-dense bunny+cloth sandwich. Set UIPC_BALLS_COARSE_ENABLE=1 to run the
// AGIPC coarse solver, =0 (default) for the baseline fine solve.
int main()
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;

    logger::set_level(spdlog::level::info);

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto this_output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);

    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config                             = Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["enable"]             = true;
    config["contact"]["friction"]["enable"] = true;
    config["contact"]["d_hat"]              = 0.01;
    config["line_search"]["max_iter"]       = 8;
    config["linear_system"]["tol_rate"]     = 1e-3;

    if(const char* env = std::getenv("UIPC_BALLS_COARSE_ENABLE"))
        config["linear_system"]["coarse"]["enable"] = (env[0] == '1') ? 1 : 0;

    // Newton convergence tolerance override (env). The paper's AGIPC criterion is
    // ‖d‖∞/Δt <= ε_d with ε_d = 1e-3·l (l = scene bbox diagonal, m/s); libuipc's
    // newton/velocity_tol is the same ‖d‖∞/Δt threshold but defaults to 0.05 with
    // no bbox scaling. Set UIPC_BALLS_VELOCITY_TOL to align with the paper.
    if(const char* v = std::getenv("UIPC_BALLS_VELOCITY_TOL"))
        config["newton"]["velocity_tol"] = std::atof(v);

    // Optional coarse-parameter sweep overrides (env).
    auto env_int = [](const char* n, auto&& set)
    {
        if(const char* v = std::getenv(n))
            set(std::atoll(v));
    };
    auto env_flt = [](const char* n, auto&& set)
    {
        if(const char* v = std::getenv(n))
            set(std::atof(v));
    };
    env_int("UIPC_COARSE_MAX_ITERS",
            [&](long long x)
            { config["linear_system"]["coarse"]["max_iterations"] = (IndexT)x; });
    env_int("UIPC_COARSE_USE_MAS",
            [&](long long x)
            { config["linear_system"]["coarse"]["use_mas_preconditioner"] = (IndexT)x; });
    env_int("UIPC_COARSE_FINE_REFINE",
            [&](long long x) {
                config["linear_system"]["coarse"]["fine_refine_iters"] = (IndexT)x;
            });
    env_int("UIPC_COARSE_MAX_AGG",
            [&](long long x) {
                config["linear_system"]["coarse"]["max_aggregate_size"] = (IndexT)x;
            });
    env_flt("UIPC_COARSE_GREEN_TAU",
            [&](double x)
            { config["linear_system"]["coarse"]["green_strain_tau"] = (Float)x; });

    test::Scene::dump_config(config, this_output_path);

    Scene scene{config};
    {
        SimplicialComplexIO io;
        StableNeoHookean    snh;
        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto default_contact = scene.contact_tabular().default_element();

        auto ball_obj  = scene.objects().create("balls");
        auto ball_mesh = io.read(fmt::format("{}ball.msh", tetmesh_dir));
        label_surface(ball_mesh);
        label_triangle_orient(ball_mesh);

        // Stiff material (paper's accelerating regime): high Young's modulus ->
        // tiny deformation -> high collapsible-edge ratio -> strong coarsening.
        auto parm = ElasticModuli::youngs_poisson(1e7, 0.45);
        snh.apply_to(ball_mesh, parm, 1e3);
        default_contact.apply_to(ball_mesh);

        // Ball bbox ~4 units, centered near origin. Scale to ~1 unit and stack a
        // vertical column of balls that settle on the ground with sparse contact.
        // Recenter the mesh to the origin first (its native center is offset), then
        // scale and stack so each ball clears the ground (y=0) and its neighbours.
        Vector3 c = Vector3::Zero();
        {
            auto pv = view(ball_mesh.positions());
            for(auto& v : pv)
                c += v;
            c /= static_cast<Float>(pv.size());
        }
        constexpr Float scale = 0.25;
        constexpr int   N     = 8;  // number of balls
        constexpr Float gap = 1.45;  // vertical spacing > scaled ball y-extent (~1.19)
        Vector3 bbox_min = Vector3::Constant(std::numeric_limits<Float>::max());
        Vector3 bbox_max = Vector3::Constant(std::numeric_limits<Float>::lowest());
        for(int i = 0; i < N; ++i)
        {
            SimplicialComplex b = ball_mesh;
            auto              p = view(b.positions());
            for(auto& v : p)
            {
                v = (v - c) * scale;  // recenter to origin, then scale
                v.y() += 0.8 + i * gap;  // lowest ball center at y=0.8 (bottom ~0.2)
                bbox_min = bbox_min.cwiseMin(v);
                bbox_max = bbox_max.cwiseMax(v);
            }
            ball_obj->geometries().create(b);
        }

        // Report the deformable-mesh bbox diagonal l so the paper's Newton tolerance
        // ε_d = 1e-3·l can be reproduced via UIPC_BALLS_VELOCITY_TOL.
        Float l = (bbox_max - bbox_min).norm();
        fmt::println("[bbox] balls deformable bbox diagonal l = {:.6f}, paper eps_d = 1e-3*l = {:.6e}",
                     l, 1e-3 * l);

        auto             ground_obj = scene.objects().create("ground");
        ImplicitGeometry g          = ground(0.0);
        ground_obj->geometries().create(g);
    }

    world.init(scene);
    if(!world.is_valid())
    {
        logger::error("fem_elastic_balls: invalid scene");
        return 1;
    }

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    SizeT max_frame = 1000;
    if(const char* v = std::getenv("UIPC_BALLS_MAX_FRAME"))
        max_frame = static_cast<SizeT>(std::atoll(v));
    bool no_write = std::getenv("UIPC_BALLS_NO_WRITE") != nullptr;

    while(world.frame() < max_frame)
    {
        world.advance();
        world.retrieve();
        if(!no_write)
            sio.write_surface(
                fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
        fmt::println("frame: {}", world.frame());
    }
    return 0;
}
