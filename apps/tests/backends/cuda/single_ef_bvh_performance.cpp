#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/common/timer.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/strain_limiting_baraff_witkin.h>
#include <uipc/constitution/discrete_shell_bending.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <set>

namespace
{
using namespace uipc;
using namespace uipc::core;
using namespace uipc::geometry;
using namespace uipc::constitution;

void save_json(const std::filesystem::path& path, const Json& json)
{
    std::ofstream stream{path};
    stream << json.dump(2) << '\n';
    REQUIRE(stream.good());
}

Json loaded_files()
{
    std::ifstream         maps{"/proc/self/maps"};
    std::set<std::string> paths;
    for(std::string line; std::getline(maps, line);)
        if(const auto begin = line.find('/'); begin != std::string::npos)
            paths.insert(line.substr(begin));
    return paths;
}

Json geometry_state(span<const S<SimplicialComplexSlot>> observed)
{
    Json state = Json::array();
    for(const auto& slot : observed)
    {
        const auto& mesh      = slot->geometry();
        Json        positions = Json::array();
        for(const auto& position : mesh.positions().view())
            positions.push_back(
                Json::array({position.x(), position.y(), position.z()}));
        Json       transform = Json::array();
        const auto matrix    = mesh.transforms().view()[0];
        for(int row = 0; row < 4; ++row)
            for(int col = 0; col < 4; ++col)
                transform.push_back(matrix(row, col));
        state.push_back(Json{{"positions", std::move(positions)},
                             {"transform", std::move(transform)},
                             {"tet_count", mesh.tetrahedra().size()},
                             {"triangle_count", mesh.triangles().size()}});
    }
    return state;
}
}  // namespace

// Explicit opt-in: each process runs one method and one instrumentation mode.
TEST_CASE("single_ef_bvh_native_wrecking_ball_performance", "[.single_ef_bvh_performance]")
{
    constexpr int total_frames  = 1000;
    constexpr int warmup_frames = 30;
    const auto*   method_env    = std::getenv("UIPC_SINGLE_EF_PERF_METHOD");
    const auto*   output_env    = std::getenv("UIPC_SINGLE_EF_PERF_OUTPUT");
    REQUIRE(method_env);
    REQUIRE(output_env);
    const std::string method{method_env};
    REQUIRE((method == "info_stackless_bvh" || method == "single_ef_bvh"));
    const auto* profile_env = std::getenv("UIPC_SINGLE_EF_PERF_PROFILE");
    const bool  profile   = profile_env && std::string_view{profile_env} == "1";
    const auto* scene_env = std::getenv("UIPC_SINGLE_EF_PERF_SCENE");
    const std::string scene_name = scene_env ? scene_env : "native-wrecking-ball-574";
    const bool reference_wb = scene_name == "wb";
    const bool ex88         = scene_name == "ex88";
    REQUIRE((reference_wb || ex88 || scene_name == "native-wrecking-ball-574"));
    const auto* assets_env = std::getenv("UIPC_SINGLE_EF_PERF_ASSETS");
    if(reference_wb || ex88)
        REQUIRE(assets_env);
    const std::filesystem::path assets = assets_env ? assets_env : "";
    const std::filesystem::path output{output_env};
    REQUIRE_FALSE(std::filesystem::exists(output));
    std::filesystem::create_directories(output / "workspace");
    logger::set_level(spdlog::level::off);
    Timer::disable_all();

    Engine engine{"cuda", (output / "workspace").string()};
    World  world{engine};
    auto   config = Scene::default_config();
    if(reference_wb || ex88)
    {
        config["dt"]      = reference_wb ? 0.02 : 0.01;
        config["gravity"] = Vector3{0, reference_wb ? -9.81 : -9.8, 0};
        config["contact"]["d_hat_relative"]        = 1e-3;
        config["contact"]["eps_velocity_relative"] = 1e-2;
        config["newton"]["velocity_tol_relative"]  = 1e-2;
        config["newton"]["transrate_tol"]          = 10;
        config["linear_system"]["tol_rate"]        = 1e-4;
        if(ex88)
        {
            config["linear_system"]["fem_preconditioner"] = "mas";
            config["newton"]["semi_implicit"]["enable"]   = 1;
            config["newton"]["semi_implicit"]["beta_tol"] = 1e-2;
            config["newton"]["semi_implicit"]["K_min"]    = 6;
        }
    }
    else
    {
        config["gravity"]                       = Vector3{0, -9.8, 0};
        config["contact"]["friction"]["enable"] = true;
        config["contact"]["enable"]             = true;
        config["contact"]["d_hat"]              = 0.01;
        config["line_search"]["max_iter"]       = 8;
    }
    config["collision_detection"]["method"] = method;
    save_json(output / "config.json", config);

    Scene                            scene{config};
    vector<S<SimplicialComplexSlot>> observed;
    SizeT tracked_ball = std::numeric_limits<SizeT>::max();
    if(!ex88)
    {
        AffineBodyConstitution abd;
        if(!reference_wb)
            scene.constitution_tabular().insert(abd);
        auto contact = scene.contact_tabular().default_element();
        scene.contact_tabular().default_model(reference_wb ? 0.2 : 0.01,
                                              reference_wb ? 1e8 : 20.0_GPa);
        SimplicialComplexIO io{Transform::Identity()};
        const std::string   mesh_dir = reference_wb ?
                                           (assets / "sim_data/tetmesh").string() + "/" :
                                           std::string{AssetDir::tetmesh_path()};
        auto read_mesh = [&](const std::string& name) -> SimplicialComplex
        {
            auto mesh = io.read(mesh_dir + name);
            if(!reference_wb)
                abd.apply_to(mesh, 10.0_MPa);
            label_surface(mesh);
            label_triangle_orient(mesh);
            if(reference_wb)
                return flip_inward_triangles(mesh);
            contact.apply_to(mesh);
            return mesh;
        };
        auto       cube  = read_mesh("cube.msh");
        auto       ball  = read_mesh("ball.msh");
        auto       link  = read_mesh("link.msh");
        auto       cubes = scene.objects().create("cubes");
        auto       balls = scene.objects().create("balls");
        auto       links = scene.objects().create("links");
        const auto scene_path =
            reference_wb ? assets / "sim_data/wrecking_ball.json" :
                           std::filesystem::path{UIPC_PROJECT_DIR}
                               / "apps/examples/wrecking_ball/wrecking_ball.json";
        std::ifstream scene_stream{scene_path};
        REQUIRE(scene_stream.good());
        const auto descriptors = Json::parse(scene_stream);
        for(const auto& descriptor : descriptors)
        {
            const auto  mesh_name = descriptor.at("mesh").get<std::string>();
            const auto* source    = mesh_name == "cube.msh" ? &cube :
                                    mesh_name == "ball.msh" ? &ball :
                                                              &link;
            auto        object    = mesh_name == "cube.msh" ? cubes :
                                    mesh_name == "ball.msh" ? balls :
                                                              links;
            Vector3     position  = Vector3::Zero();
            if(descriptor.contains("position"))
                for(int i = 0; i < 3; ++i)
                    position[i] = descriptor["position"][i].get<Float>();
            Eigen::Quaternion<Float> rotation = Eigen::Quaternion<Float>::Identity();
            if(descriptor.contains("rotation"))
            {
                Vector3 angles;
                for(int i = 0; i < 3; ++i)
                    angles[i] = descriptor["rotation"][i].get<Float>();
                angles *= std::numbers::pi / 180.0;
                rotation = AngleAxis(angles.z(), Vector3::UnitZ())
                           * AngleAxis(angles.y(), Vector3::UnitY())
                           * AngleAxis(angles.x(), Vector3::UnitX());
            }
            auto transform = Transform::Identity();
            transform.translate(position).rotate(rotation);
            SimplicialComplex mesh = *source;
            if(reference_wb)
            {
                abd.apply_to(mesh, 100.0_MPa, descriptor.value("density", Float{1e3}));
                contact.apply_to(mesh);
            }
            view(mesh.transforms())[0] = transform.matrix();
            auto fixed = mesh.instances().find<IndexT>(builtin::is_fixed);
            REQUIRE(fixed);
            view(*fixed)[0]   = descriptor.value("is_dof_fixed", false) ? 1 : 0;
            auto [slot, rest] = object->geometries().create(mesh);
            if(mesh_name == "ball.msh")
                tracked_ball = observed.size();
            observed.push_back(slot);
        }
    }
    else
    {
        // Port the executed sample-88 expressions, including E=1e4 (not its docstring).
        scene.contact_tabular().default_model(0.2, 1e8);
        auto             contact = scene.contact_tabular().default_element();
        StableNeoHookean snh;
        auto             make_bunny = [&](Float y, std::string_view name)
        {
            Transform transform = Transform::Identity();
            transform.translate(Vector3{0, y, 0});
            transform.scale(0.2);
            auto raw = SimplicialComplexIO{transform}.read(
                (assets / "sim_data/tetmesh/bunny2.msh").string());
            label_surface(raw);
            label_triangle_orient(raw);
            auto mesh = flip_inward_triangles(raw);
            snh.apply_to(mesh, ElasticModuli::youngs_poisson(1e4, 0.49), 1e3);
            contact.apply_to(mesh);
            auto [slot, rest] = scene.objects().create(name)->geometries().create(mesh);
            observed.push_back(slot);
        };
        make_bunny(0.5, "fem_bunny_upper");
        make_bunny(-0.65, "fem_bunny_lower");
        auto cloth = SimplicialComplexIO{}.read(
            (assets / "sim_data/trimesh/cloth_high.obj").string());
        label_surface(cloth);
        StrainLimitingBaraffWitkinShell{}.apply_to(
            cloth,
            ElasticModuli2D::youngs_poisson(5e4, 0.49),
            ElasticModuli2D::youngs_poisson(1e1, 0.49),
            200,
            0.001,
            100);
        DiscreteShellBending{}.apply_to(cloth, 3e4, 0.49);
        auto [slot, rest] = scene.objects().create("cloth")->geometries().create(cloth);
        observed.push_back(slot);
    }
    auto plane = ground(-1.0);
    scene.objects().create("ground")->geometries().create(plane);
    REQUIRE(observed.size() == (ex88 ? 3 : 574));
    save_json(output / "initial-state.json", geometry_state(observed));
    vector<vector<Float>> inverse_rest_determinants(observed.size());
    if(ex88)
        for(SizeT geometry = 0; geometry < observed.size(); ++geometry)
        {
            const auto& mesh = observed[geometry]->geometry();
            if(mesh.tetrahedra().size() == 0)
                continue;
            const auto positions = mesh.positions().view();
            for(const auto& tet : mesh.tetrahedra().topo().view())
            {
                const Vector3 a = positions[tet[1]] - positions[tet[0]];
                const Vector3 b = positions[tet[2]] - positions[tet[0]];
                const Vector3 c = positions[tet[3]] - positions[tet[0]];
                const Float   determinant = a.dot(b.cross(c));
                REQUIRE(std::isfinite(determinant));
                REQUIRE(determinant != 0);
                inverse_rest_determinants[geometry].push_back(1.0 / determinant);
            }
        }
    world.init(scene);
    world.sync();
    save_json(output / "loaded-files-init.json", loaded_files());
    REQUIRE(world.is_valid());

    std::ofstream frames{output / "frames.jsonl"};
    REQUIRE(frames.good());
    std::ofstream timers;
    if(profile)
    {
        timers.open(output / "timers.jsonl");
        REQUIRE(timers.good());
    }
    using Clock        = std::chrono::steady_clock;
    double measured_ms = 0.0;
    int    completed   = 0;
    bool   all_finite  = true;
    bool   all_valid   = true;
    for(int frame = 1; frame <= total_frames; ++frame)
    {
        if(frame == warmup_frames + 1)
        {
            GlobalTimer::current()->clear();
            if(profile)
                Timer::enable_all();
        }
        const auto begin = Clock::now();
        world.advance();
        world.sync();
        const auto advanced = Clock::now();
        world.retrieve();
        world.sync();
        const auto   end = Clock::now();
        const double advance_ms =
            std::chrono::duration<double, std::milli>(advanced - begin).count();
        const double retrieve_ms =
            std::chrono::duration<double, std::milli>(end - advanced).count();
        const double frame_ms =
            std::chrono::duration<double, std::milli>(end - begin).count();
        // Counts and geometric observations are outside both timed intervals.
        auto  stats            = engine.frame_stats();
        bool  finite           = true;
        Float min_y            = std::numeric_limits<Float>::infinity();
        Float min_J            = std::numeric_limits<Float>::infinity();
        SizeT negative_tets    = 0;
        SizeT nonpositive_tets = 0;
        Json  centroids        = Json::array();
        for(SizeT geometry = 0; geometry < observed.size(); ++geometry)
        {
            const auto& slot      = observed[geometry];
            const auto& mesh      = slot->geometry();
            const auto  transform = mesh.transforms().view()[0];
            finite                = finite && transform.allFinite();
            Vector3 centroid      = Vector3::Zero();
            for(const auto& position : mesh.positions().view())
            {
                const Vector3 world_position = transform.topLeftCorner<3, 3>() * position
                                               + transform.topRightCorner<3, 1>();
                finite = finite && world_position.allFinite();
                min_y  = std::min(min_y, world_position.y());
                centroid += world_position;
            }
            centroid /= static_cast<Float>(mesh.positions().size());
            if(ex88 || geometry == tracked_ball)
                centroids.push_back(
                    Json::array({centroid.x(), centroid.y(), centroid.z()}));
            if(ex88 && mesh.tetrahedra().size() > 0)
            {
                const auto positions = mesh.positions().view();
                SizeT      tet_index = 0;
                for(const auto& tet : mesh.tetrahedra().topo().view())
                {
                    const Vector3 a = positions[tet[1]] - positions[tet[0]];
                    const Vector3 b = positions[tet[2]] - positions[tet[0]];
                    const Vector3 c = positions[tet[3]] - positions[tet[0]];
                    const Float   J = a.dot(b.cross(c))
                                    * inverse_rest_determinants[geometry][tet_index++];
                    finite = finite && std::isfinite(J);
                    min_J  = std::min(min_J, J);
                    negative_tets += J < 0;
                    nonpositive_tets += J <= 0;
                }
            }
        }
        const bool valid = world.is_valid() && world.frame() == frame;
        Json       sample{{"frame", frame},
                          {"measured", frame > warmup_frames},
                          {"advance_ms", advance_ms},
                          {"retrieve_ms", retrieve_ms},
                          {"frame_ms", frame_ms},
                          {"valid", valid},
                          {"finite", finite},
                          {"min_world_y", min_y},
                          {"ground_penetration", std::max(Float{0}, -1.0 - min_y)},
                          {"centroids", std::move(centroids)},
                          {"min_tet_J", ex88 ? Json(min_J) : Json(nullptr)},
                          {"negative_tets", ex88 ? Json(negative_tets) : Json(nullptr)},
                          {"nonpositive_tets", ex88 ? Json(nonpositive_tets) : Json(nullptr)},
                          {"stats", std::move(stats)}};
        frames << sample.dump() << '\n';
        // Merge and release each frame's timer tree outside the timed window.
        if(profile && frame > warmup_frames)
            timers << Json{{"frame", frame}, {"timer", Timer::report_as_json()}}.dump()
                   << '\n';
        if(frame % 10 == 0)
        {
            frames.flush();
            if(profile)
                timers.flush();
        }
        if(frame > warmup_frames)
            measured_ms += frame_ms;
        completed  = frame;
        all_finite = all_finite && finite;
        all_valid  = all_valid && valid;
        if(!finite || !valid || !sample["stats"].value("completed", false))
            break;
    }
    Timer::disable_all();
    frames.close();
    REQUIRE(frames.good());
    if(profile)
    {
        timers.close();
        REQUIRE(timers.good());
    }
    save_json(output / "loaded-files-final.json", loaded_files());
    Json final_transforms = Json::array();
    for(const auto& slot : observed)
    {
        Json       values    = Json::array();
        const auto transform = slot->geometry().transforms().view()[0];
        for(int row = 0; row < 4; ++row)
            for(int col = 0; col < 4; ++col)
                values.push_back(transform(row, col));
        final_transforms.push_back(std::move(values));
    }
    save_json(output / "final-transforms.json", final_transforms);
    save_json(output / "final-state.json", geometry_state(observed));
    save_json(output / "result.json",
              Json{{"scene", scene_name},
                   {"method", method},
                   {"profile", profile},
                   {"requested_frames", total_frames},
                   {"completed_frames", completed},
                   {"warmup_frames", warmup_frames},
                   {"measured_frame_ms", measured_ms},
                   {"finite", all_finite},
                   {"valid", all_valid}});
    REQUIRE(completed == total_frames);
    REQUIRE(all_valid);
    REQUIRE(all_finite);
}
