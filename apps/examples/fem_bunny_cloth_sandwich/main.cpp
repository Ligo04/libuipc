#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/neo_hookean_shell.h>
#include <uipc/constitution/discrete_shell_bending.h>
#include <cstdlib>
#include <limits>

// Two FEM (StableNeoHookean) bunnies sandwiching a FEM cloth
// (NeoHookeanShell + DiscreteShellBending), falling onto a ground plane.
//
// The stack drops a short distance and lands on the ground, so the bottom bunny
// is squeezed between the ground and the top bunny + cloth — producing sustained
// FEM-FEM-cloth contact (a richer workload than pure free fall).
//
// This is an AGIPC-friendly scene: the bodies are deformable FEM meshes, so the
// algebraic-coarsening path actually aggregates low-strain regions into coarse
// nodes (unlike a rigid/ABD scene). Set UIPC_SANDWICH_COARSE_ENABLE=1 to run with
// the AGIPC coarse solver, =0 (default) for the baseline fine solver, and compare
// 1000-frame wall-clock between the two.
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

    // AGIPC coarse-mode comparison knob (default off = baseline fine solve).
    if(const char* env = std::getenv("UIPC_SANDWICH_COARSE_ENABLE"))
        config["linear_system"]["coarse"]["enable"] = (env[0] == '1') ? 1 : 0;

    // Newton convergence tolerance override (env). The paper's AGIPC criterion is
    // ‖d‖∞/Δt <= ε_d with ε_d = 1e-3·l (l = scene bbox diagonal, m/s); libuipc's
    // newton/velocity_tol is the same ‖d‖∞/Δt threshold but defaults to 0.05 with
    // no bbox scaling. Set UIPC_SANDWICH_VELOCITY_TOL to align with the paper.
    if(const char* v = std::getenv("UIPC_SANDWICH_VELOCITY_TOL"))
        config["newton"]["velocity_tol"] = std::atof(v);

    // Optional parameter-sweep overrides (env). Let a sweep probe whether any
    // coarse configuration beats the baseline without recompiling.
    auto env_int = [](const char* name, auto&& set)
    {
        if(const char* v = std::getenv(name))
            set(std::atoll(v));
    };
    auto env_flt = [](const char* name, auto&& set)
    {
        if(const char* v = std::getenv(name))
            set(std::atof(v));
    };
    env_int("UIPC_COARSE_MAX_ITERS",
            [&](long long x)
            { config["linear_system"]["coarse"]["max_iterations"] = (IndexT)x; });
    env_int("UIPC_COARSE_FINE_REFINE",
            [&](long long x) {
                config["linear_system"]["coarse"]["fine_refine_iters"] = (IndexT)x;
            });
    env_int("UIPC_COARSE_MAX_AGG",
            [&](long long x) {
                config["linear_system"]["coarse"]["max_aggregate_size"] = (IndexT)x;
            });
    env_int("UIPC_COARSE_AFFINE_THRESH",
            [&](long long x) {
                config["linear_system"]["coarse"]["affine_threshold"] = (IndexT)x;
            });
    env_flt("UIPC_COARSE_GREEN_TAU",
            [&](double x)
            { config["linear_system"]["coarse"]["green_strain_tau"] = (Float)x; });
    env_flt("UIPC_COARSE_PCG_TOL",
            [&](double x)
            { config["linear_system"]["coarse"]["pcg_relative_tol"] = (Float)x; });

    test::Scene::dump_config(config, this_output_path);

    Scene scene{config};
    {
        SimplicialComplexIO io;

        StableNeoHookean     snh;  // 3D solid bunnies
        NeoHookeanShell      nhs;  // cloth membrane
        DiscreteShellBending dsb;  // cloth bending
        scene.contact_tabular().default_model(0.5, 1.0_GPa);
        auto default_contact = scene.contact_tabular().default_element();

        // ---- Two FEM bunnies ----
        auto bunny_obj  = scene.objects().create("bunnies");
        auto bunny_mesh = io.read(fmt::format("{}bunny0.msh", tetmesh_dir));
        label_surface(bunny_mesh);
        label_triangle_orient(bunny_mesh);
        auto bunny_parm = ElasticModuli::youngs_poisson(1e5, 0.49);
        snh.apply_to(bunny_mesh, bunny_parm, 1e3);
        default_contact.apply_to(bunny_mesh);

        // Bunny bbox ~ (0.93 x 1.44 x 1.98). Stack: bottom bunny below the cloth,
        // top bunny above it, with small gaps so the sandwich settles in free fall.
        Vector3 bbox_min = Vector3::Constant(std::numeric_limits<Float>::max());
        Vector3 bbox_max = Vector3::Constant(std::numeric_limits<Float>::lowest());
        auto place_bunny = [&](Float y_offset)
        {
            SimplicialComplex b = bunny_mesh;
            auto              p = view(b.positions());
            for(auto& v : p)
            {
                v.y() += y_offset;
                bbox_min = bbox_min.cwiseMin(v);
                bbox_max = bbox_max.cwiseMax(v);
            }
            bunny_obj->geometries().create(b);
        };
        place_bunny(-1.05);  // bottom bunny: top near y ~ -0.27
        place_bunny(+1.05);  // top bunny:   bottom near y ~ +0.43

        // ---- FEM cloth (4x area) sandwiched between the bunnies (y = 0 plane) ----
        auto cloth_obj = scene.objects().create("cloth");

        constexpr int N = 30;        // resolution per side
        constexpr Float size = 2.8;  // 2.8 x 2.8 -> 4x the original 1.4 x 1.4 area
        constexpr Float  spacing = size / N;
        vector<Vector3>  Vs;
        vector<Vector3i> Fs;
        for(int i = 0; i <= N; i++)
            for(int j = 0; j <= N; j++)
                Vs.push_back(Vector3{i * spacing - size / 2, 0.05, j * spacing - size / 2});
        for(int i = 0; i < N; i++)
            for(int j = 0; j < N; j++)
            {
                int v00 = i * (N + 1) + j;
                int v10 = (i + 1) * (N + 1) + j;
                int v01 = i * (N + 1) + (j + 1);
                int v11 = (i + 1) * (N + 1) + (j + 1);
                Fs.push_back(Vector3i{v00, v10, v11});
                Fs.push_back(Vector3i{v00, v11, v01});
            }

        auto cloth = trimesh(Vs, Fs);
        label_surface(cloth);
        auto cloth_parm = ElasticModuli2D::youngs_poisson(1.0_MPa, 0.49);
        nhs.apply_to(cloth, cloth_parm);
        dsb.apply_to(cloth, 5.0_kPa);
        default_contact.apply_to(cloth);
        cloth_obj->geometries().create(cloth);
        for(const auto& v : Vs)
        {
            bbox_min = bbox_min.cwiseMin(v);
            bbox_max = bbox_max.cwiseMax(v);
        }

        // Report the deformable-mesh bbox diagonal l so the paper's Newton tolerance
        // ε_d = 1e-3·l can be reproduced via UIPC_SANDWICH_VELOCITY_TOL.
        Float l = (bbox_max - bbox_min).norm();
        fmt::println("[bbox] sandwich deformable bbox diagonal l = {:.6f}, paper eps_d = 1e-3*l = {:.6e}",
                     l, 1e-3 * l);

        // ---- Ground plane below the stack ----
        // Bottom bunny bottom ~ y=-1.77; place the ground a bit lower so the
        // stack drops, lands, and the bottom bunny is squeezed against it.
        auto             ground_obj = scene.objects().create("ground");
        ImplicitGeometry g          = ground(-2.0);
        ground_obj->geometries().create(g);
    }

    world.init(scene);
    if(!world.is_valid())
    {
        logger::error("fem_bunny_cloth_sandwich: invalid scene, abort.");
        return 1;
    }

    SceneIO sio{scene};
    sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));

    // Sweep helpers: cap frames and optionally skip per-frame surface writes so
    // timing reflects solve cost, not disk I/O. Defaults preserve full behavior.
    SizeT max_frame = 1000;
    if(const char* v = std::getenv("UIPC_SANDWICH_MAX_FRAME"))
        max_frame = static_cast<SizeT>(std::atoll(v));
    bool no_write = std::getenv("UIPC_SANDWICH_NO_WRITE") != nullptr;

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
