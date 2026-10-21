#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <uipc/common/timer.h>
#include <fstream>
#include <numbers>
#include <chrono>
#include <set>
#include <stdexcept>
#include <uipc/builtin/constants.h>

namespace
{
struct Options
{
    bool        benchmark = false;
    bool        timers    = false;
    uipc::SizeT frames    = 1000;
    uipc::SizeT warmup    = 0;
    std::string method    = "info_stackless_bvh";
    std::string result_json;
    std::string workspace;
};

Options parse_options(int argc, char* argv[])
{
    Options options;
    bool    frames_set = false, warmup_set = false;
    for(int i = 1; i < argc; ++i)
    {
        const std::string arg   = argv[i];
        auto              value = [&]() -> std::string
        {
            if(++i >= argc)
                throw std::runtime_error("Missing value for " + arg);
            return argv[i];
        };
        if(arg == "--benchmark")
            options.benchmark = true;
        else if(arg == "--timers")
            options.timers = true;
        else if(arg == "--frames")
        {
            options.frames = std::stoull(value());
            frames_set     = true;
        }
        else if(arg == "--warmup")
        {
            options.warmup = std::stoull(value());
            warmup_set     = true;
        }
        else if(arg == "--method")
            options.method = value();
        else if(arg == "--result-json")
            options.result_json = value();
        else if(arg == "--workspace")
            options.workspace = value();
        else
            throw std::runtime_error("Unknown argument: " + arg);
    }
    if(options.benchmark)
    {
        if(!frames_set)
            options.frames = 300;
        if(!warmup_set)
            options.warmup = 30;
        if(options.result_json.empty())
            throw std::runtime_error("--benchmark requires --result-json");
    }
    if(options.frames == 0 || options.warmup >= options.frames)
        throw std::runtime_error("Require frames > warmup >= 0");
    return options;
}
}  // namespace

int main(int argc, char* argv[])
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;
    namespace fs = std::filesystem;

    const auto options = parse_options(argc, argv);
    logger::set_level(options.benchmark ? spdlog::level::warn : spdlog::level::info);
    if(options.timers)
        Timer::enable_all();
    else
        Timer::disable_all();

    std::string tetmesh_dir{AssetDir::tetmesh_path()};
    auto this_output_path = AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE);
    auto this_folder      = AssetDir::folder(UIPC_RELATIVE_SOURCE_FILE);
    if(!options.workspace.empty())
        this_output_path = options.workspace;
    fs::create_directories(this_output_path);


    Engine engine{"cuda", this_output_path};
    World  world{engine};

    auto config                             = Scene::default_config();
    config["gravity"]                       = Vector3{0, -9.8, 0};
    config["contact"]["friction"]["enable"] = true;
    config["contact"]["enable"]             = true;
    config["contact"]["d_hat"]              = 0.01;
    config["line_search"]["max_iter"]       = 8;
    config["collision_detection"]["method"] = options.method;
    if(!options.benchmark)
        test::Scene::dump_config(config, this_output_path);


    Scene                            scene{config};
    vector<S<SimplicialComplexSlot>> observed;
    S<SimplicialComplexSlot>         ball_slot;
    {
        Json wrecking_ball_scene;
        {
            std::ifstream ifs(fmt::format("{}wrecking_ball.json", this_folder));
            ifs >> wrecking_ball_scene;
        }

        // create constitution and contact model
        AffineBodyConstitution abd;
        scene.constitution_tabular().insert(abd);
        auto default_contact = scene.contact_tabular().default_element();
        scene.contact_tabular().default_model(0.01, 20.0_GPa);

        Float     scale = 1;
        Transform T     = Transform::Identity();
        T.scale(scale);
        SimplicialComplexIO io{T};

        auto cube = io.read(fmt::format("{}cube.msh", tetmesh_dir));
        auto ball = io.read(fmt::format("{}ball.msh", tetmesh_dir));
        auto link = io.read(fmt::format("{}link.msh", tetmesh_dir));

        S<Object> cube_obj = scene.objects().create("cubes");
        S<Object> ball_obj = scene.objects().create("balls");
        S<Object> link_obj = scene.objects().create("links");

        abd.apply_to(cube, 10.0_MPa);
        label_surface(cube);
        label_triangle_orient(cube);
        abd.apply_to(ball, 10.0_MPa);
        label_surface(ball);
        label_triangle_orient(ball);
        abd.apply_to(link, 10.0_MPa);
        label_surface(link);
        label_triangle_orient(link);

        default_contact.apply_to(cube);
        default_contact.apply_to(ball);
        default_contact.apply_to(link);


        auto build_mesh = [&](const Json& j, Object& obj, const SimplicialComplex& mesh)
        {
            Vector3 position;

            if(j.find("position") != j.end())
            {
                position[0] = j["position"][0].get<Float>();
                position[1] = j["position"][1].get<Float>();
                position[2] = j["position"][2].get<Float>();
            }

            Eigen::Quaternion<Float> Q = Eigen::Quaternion<Float>::Identity();

            if(j.find("rotation") != j.end())
            {
                Vector3 rotation;
                rotation[0] = j["rotation"][0].get<Float>();
                rotation[1] = j["rotation"][1].get<Float>();
                rotation[2] = j["rotation"][2].get<Float>();

                rotation *= std::numbers::pi / 180.0;

                Q = AngleAxis(rotation.z(), Vector3::UnitZ())
                    * AngleAxis(rotation.y(), Vector3::UnitY())
                    * AngleAxis(rotation.x(), Vector3::UnitX());
            }

            IndexT is_fixed = 0;
            if(j.find("is_dof_fixed") != j.end())
            {
                is_fixed = j["is_dof_fixed"].get<bool>() ? 1 : 0;
            }

            position *= scale;

            Transform t = Transform::Identity();
            t.translate(position).rotate(Q);

            SimplicialComplex this_mesh     = mesh;
            view(this_mesh.transforms())[0] = t.matrix();

            auto is_fixed_attr = this_mesh.instances().find<IndexT>(builtin::is_fixed);
            view(*is_fixed_attr)[0] = is_fixed;

            auto [slot, rest] = obj.geometries().create(this_mesh);
            observed.push_back(slot);
            if(j["mesh"] == "ball.msh")
                ball_slot = slot;
        };

        //IndexT count = 0;

        for(const Json& obj : wrecking_ball_scene)
        {
            if(obj["mesh"] == "link.msh")
            {
                build_mesh(obj, *link_obj, link);
            }
            else if(obj["mesh"] == "cube.msh")
            {
                build_mesh(obj, *cube_obj, cube);
            }
            else if(obj["mesh"] == "ball.msh")
            {
                build_mesh(obj, *ball_obj, ball);
            }
        }

        constexpr bool UseMeshGround = false;

        if(UseMeshGround)
        {
            Transform pre_transform = Transform::Identity();
            pre_transform.scale(Vector3{40, 0.2, 40});

            SimplicialComplexIO io{pre_transform};
            io          = SimplicialComplexIO{pre_transform};
            auto ground = io.read(fmt::format("{}{}", tetmesh_dir, "cube.msh"));

            label_surface(ground);
            label_triangle_orient(ground);

            Transform transform = Transform::Identity();
            transform.translate(Vector3{12, -1.1, 0});
            view(ground.transforms())[0] = transform.matrix();
            abd.apply_to(ground, 10.0_MPa);

            auto is_fixed = ground.instances().find<IndexT>(builtin::is_fixed);
            view(*is_fixed)[0] = 1;

            auto ground_obj = scene.objects().create("ground");
            ground_obj->geometries().create(ground);
        }
        else
        {
            auto ground_obj = scene.objects().create("ground");
            auto g          = geometry::ground(-1.0);
            ground_obj->geometries().create(g);
        }
    }

    world.init(scene);
    if(!world.is_valid())
        throw std::runtime_error("Wrecking ball world initialization failed");
    world.sync();
    SceneIO sio{scene};
    if(!options.benchmark)
    {
        sio.write_surface(fmt::format("{}scene_surface{}.obj", this_output_path, 0));
        world.dump();
    }
    if(options.timers)
        Timer::report_as_json();

    Json samples = Json::array();
    while(world.frame() < options.frames)
    {
        const auto expected_frame = world.frame() + 1;
        const auto start          = std::chrono::steady_clock::now();
        world.advance();
        world.retrieve();
        world.sync();
        const auto stop  = std::chrono::steady_clock::now();
        const auto stats = engine.frame_stats();
        if(!world.is_valid() || world.frame() != expected_frame
           || !stats.at("completed").get<bool>())
            throw std::runtime_error("Wrecking ball frame did not complete");
        if(options.benchmark)
        {
            Json sample{{"frame", world.frame()},
                        {"wall_ms",
                         std::chrono::duration<double, std::milli>(stop - start).count()},
                        {"stats", stats}};
            if(options.timers)
                sample["timers"] = Timer::report_as_json();
            samples.push_back(std::move(sample));
            if(world.frame() % 50 == 0)
                fmt::println("benchmark {} frame {}/{}",
                             options.method,
                             world.frame(),
                             options.frames);
        }
        else
        {
            world.dump();
            sio.write_surface(
                fmt::format("{}scene_surface{}.obj", this_output_path, world.frame()));
            fmt::println("frame: {}", world.frame());
        }
    }
    if(options.benchmark)
    {
        bool finite     = true;
        Json transforms = Json::array();
        for(const auto& slot : observed)
            for(const auto& transform : slot->geometry().transforms().view())
            {
                finite = finite && transform.allFinite();
                transforms.push_back(transform);
            }
        if(!finite || !ball_slot)
            throw std::runtime_error("Invalid final wrecking ball transforms");
        const auto& transform = ball_slot->geometry().transforms().view()[0];
        Json        result{{"method", options.method},
                           {"frames", options.frames},
                           {"warmup", options.warmup},
                           {"timers_enabled", options.timers},
                           {"config", config},
                           {"frame_samples", std::move(samples)},
                           {"final_observable",
                            {{"finite", finite},
                             {"body_count", observed.size()},
                             {"ball_translation",
                              Json::array({transform(0, 3), transform(1, 3), transform(2, 3)})},
                             {"transforms", std::move(transforms)}}}};
#if defined(__linux__)
        result["executable"] = fs::read_symlink("/proc/self/exe").string();
        std::set<std::string> loaded_modules;
        std::ifstream         maps{"/proc/self/maps"};
        for(std::string line; std::getline(maps, line);)
        {
            const auto slash = line.find('/');
            if(slash != std::string::npos && line.find("libuipc", slash) != std::string::npos)
                loaded_modules.insert(line.substr(slash));
        }
        result["loaded_modules"] = loaded_modules;
#endif
        const fs::path result_path = options.result_json;
        if(!result_path.parent_path().empty())
            fs::create_directories(result_path.parent_path());
        std::ofstream out{result_path};
        out << result.dump(2) << '\n';
        if(!out)
            throw std::runtime_error("Failed to write benchmark result");
    }
}
