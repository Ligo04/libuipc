#include <app/app.h>
#include <uipc/uipc.h>
#include <uipc/constitution/affine_body_constitution.h>
#include <collision_detection/filters/simplex_query_trace.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <set>
#include <string>

namespace
{
class TraceEnvironment
{
  public:
    TraceEnvironment(const char* name, const std::string& value)
        : m_name{name}
    {
        if(const auto* previous = std::getenv(name))
            m_previous = previous;
        set(value.c_str());
    }
    ~TraceEnvironment() { set(m_previous ? m_previous->c_str() : nullptr); }

  private:
    std::string                m_name;
    std::optional<std::string> m_previous;
    void                       set(const char* value)
    {
#ifdef _WIN32
        _putenv_s(m_name.c_str(), value ? value : "");
#else
        if(value)
            setenv(m_name.c_str(), value, 1);
        else
            unsetenv(m_name.c_str());
#endif
    }
};
}  // namespace

TEST_CASE("Production simplex capture closes complete traces and skips audit repeats",
          "[cuda][simplex_query_replay][simplex_query_capture]")
{
    using namespace uipc;
    using namespace uipc::core;
    using namespace uipc::geometry;
    using namespace uipc::constitution;
    namespace trace = uipc::backend::cuda::simplex_query_trace;
    namespace fs    = std::filesystem;
    const auto root =
        fs::path{AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE)}
        / std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    REQUIRE(fs::create_directories(root));
    TraceEnvironment audit{"UIPC_CHECK_STORED_PAIRS", "1"};
    for(const auto* method : {"info_stackless_bvh", "single_ef_bvh"})
    {
        CAPTURE(method);
        const auto output = root / method;
        const auto file   = output / "queries.bin";
        TraceEnvironment capture{"UIPC_SIMPLEX_QUERY_TRACE_OUTPUT", file.string()};
        {
            REQUIRE(fs::create_directories(output / "world"));
            Engine engine{"cuda", (output / "world").string()};
            World  world{engine};
            auto   config                           = Scene::default_config();
            config["collision_detection"]["method"] = method;
            config["gravity"]                       = Vector3{0, -9.8, 0};
            Scene scene{config};
            vector<Vector3> positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
            vector<Vector4i> tetrahedra = {{0, 1, 2, 3}};
            auto             mesh       = tetmesh(positions, tetrahedra);
            label_surface(mesh);
            label_triangle_orient(mesh);
            AffineBodyConstitution{}.apply_to(mesh, 1.0_MPa);
            scene.contact_tabular().default_model(0.2, 1.0_GPa);
            scene.contact_tabular().default_element().apply_to(mesh);
            scene.objects().create("tetrahedron")->geometries().create(mesh);
            scene.objects().create("ground")->geometries().create(ground(-2.0));
            world.init(scene);
            REQUIRE(world.is_valid());
            for(SizeT frame = 1; frame <= 3; ++frame)
            {
                world.advance();
                world.sync();
                REQUIRE(world.is_valid());
                REQUIRE(world.frame() == frame);
                REQUIRE(engine.frame_stats().at("completed").get<bool>());
            }
        }
        // The per-world pipeline owns the footer lifetime. Audit recursion has
        // a default site(frame=0), so an incorrectly captured repeat fails here.
        trace::Reader   reader{file};
        trace::Request  request;
        SizeT           id = 0;
        std::set<SizeT> frames;
        while(reader.next(request))
        {
            REQUIRE(request.id == ++id);
            REQUIRE(request.site.frame >= 1);
            REQUIRE(request.site.frame <= 3);
            REQUIRE(request.positions.size() == 5);
            REQUIRE(request.metadata->v2bs.back()
                    == static_cast<IndexT>(request.metadata->body_self_collision.size()));
            frames.insert(request.site.frame);
        }
        REQUIRE(reader.summary().complete);
        REQUIRE(reader.summary().requests == id);
        const std::set<SizeT> expected_frames = {1, 2, 3};
        REQUIRE(frames == expected_frames);
    }
    fs::remove_all(root);
}
