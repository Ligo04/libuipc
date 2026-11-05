#include <type_define.h>
#include <app/app.h>
#include <collision_detection/filters/simplex_bvh_query.h>
#include <collision_detection/filters/simplex_query_trace.h>
#include <collision_detection/filters/trajectory_prefix.h>
#include <cuda_tool/cuda_tool.h>
#include <uipc/common/timer.h>
#include <cuda_profiler_api.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <limits>
#include <iterator>
#include <stdexcept>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace test_simplex_query_replay
{
using namespace uipc;
using namespace uipc::backend::cuda;
namespace ct     = uipc::backend::cuda_tool;
namespace trace  = uipc::backend::cuda::simplex_query_trace;
namespace fs     = std::filesystem;
using Clock      = std::chrono::steady_clock;
using Pair       = std::pair<IndexT, IndexT>;
using Candidates = std::array<std::vector<Pair>, 4>;
using Stats      = SimplexBVHQuery::Statistics;
using Method     = SimplexBVHQuery::Method;
constexpr std::array<const char*, 4> FamilyNames = {"pp", "pe", "ee", "pt"};
constexpr std::array<const char*, 4> StateNames  = {
    "info_natural", "single_ef_natural", "info_refresh", "single_ef_refresh"};

void save_json(const fs::path& path, const Json& value)
{
    std::ofstream stream{path};
    stream << value.dump(2) << '\n';
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

struct TemporaryDirectory
{
    fs::path path;
    TemporaryDirectory()
    {
        static std::atomic<unsigned> next{0};
        path = fs::path{AssetDir::output_path(UIPC_RELATIVE_SOURCE_FILE)}
               / ("trace-" + std::to_string(Clock::now().time_since_epoch().count())
                  + "-" + std::to_string(next++));
        REQUIRE(fs::create_directories(path));
    }
    ~TemporaryDirectory()
    {
        std::error_code error;
        fs::remove_all(path, error);
    }
};

template <typename T>
void upload(ct::DeviceBuffer<T>& device, const std::vector<T>& host)
{
    device.resize_discard(host.size());
    if(!host.empty())
        device.view().copy_from(host.data());
}

struct UploadedInputs
{
    ct::DeviceBuffer<Vector3> positions, displacements;
    ct::DeviceBuffer<Float>   d_hats, thicknesses;
    ct::DeviceBuffer<IndexT>  v2bs, dimensions, contact_ids, subscene_ids;
    ct::DeviceBuffer<IndexT> body_self_collision, codim_vertices, surface_vertices;
    ct::DeviceBuffer<Vector2i> edges;
    ct::DeviceBuffer<Vector3i> triangles;
    ct::DeviceBuffer2D<IndexT> contact_mask, subscene_mask;
    const trace::Metadata*     last_metadata = nullptr;
    SizeT                      last_epoch = std::numeric_limits<SizeT>::max();

    void send(const trace::Request& request)
    {
        REQUIRE(request.metadata);
        const auto& m = *request.metadata;
        if(last_metadata != &m || last_epoch != request.epoch)
        {
            upload(d_hats, m.d_hats);
            upload(thicknesses, m.thicknesses);
            upload(v2bs, m.v2bs);
            upload(dimensions, m.dimensions);
            upload(contact_ids, m.contact_ids);
            upload(subscene_ids, m.subscene_ids);
            upload(body_self_collision, m.body_self_collision);
            upload(codim_vertices, m.codim_vertices);
            upload(surface_vertices, m.surface_vertices);
            upload(edges, m.edges);
            upload(triangles, m.triangles);
            contact_mask.resize({m.contact_mask.rows, m.contact_mask.cols});
            subscene_mask.resize({m.subscene_mask.rows, m.subscene_mask.cols});
            if(!m.contact_mask.values.empty())
                contact_mask.view().copy_from(m.contact_mask.values.data());
            if(!m.subscene_mask.values.empty())
                subscene_mask.view().copy_from(m.subscene_mask.values.data());
            last_metadata = &m;
            last_epoch    = request.epoch;
        }
        upload(positions, request.positions);
        upload(displacements, request.displacements);
    }
};

struct Bounds
{
    ct::DeviceBuffer<AABB>   boxes;
    ct::DeviceBuffer<IndexT> bids, cids;
    void                     resize(SizeT count)
    {
        boxes.resize_discard(count);
        bids.resize_discard(count);
        cids.resize_discard(count);
    }
    SimplexBVHQuery::PrimitiveBounds view() const
    {
        return {boxes.cview(), bids.cview(), cids.cview()};
    }
};

struct PreparedRequest
{
    bvh_simplex_detail::PointPointPredicate    pp;
    bvh_simplex_detail::PointEdgePredicate     pe;
    bvh_simplex_detail::EdgeEdgePredicate      ee;
    bvh_simplex_detail::PointTrianglePredicate pt;
    bvh_simplex_detail::NodePredicate          node;
    TrajectoryPrefix                           prefix;
    SimplexBVHQuery::CodimQueries              codim;
    bool                                       new_frame = false;
};

struct ReplayState
{
    SimplexBVHQuery               query;
    InfoStacklessBVH              tree;
    InfoStacklessBVH::QueryBuffer pp, pe, ee, pt;
    // Prefix order is CP/P/E/F; result order is named PP/PE/EE/PT.
    std::array<Bounds, 4>    bounds;
    ct::DeviceBuffer<float4> f32_positions, f32_steps, f32_ends;

    ReplayState(Method method, bool force_refresh)
    {
        query.method      = method;
        query.force_fresh = force_refresh;
    }

    PreparedRequest bind(const UploadedInputs& in, const trace::Request& request)
    {
        const auto n = request.positions.size();
        f32_positions.resize_discard(n);
        f32_steps.resize_discard(n);
        f32_ends.resize_discard(n);
        bounds[0].resize(in.codim_vertices.size());
        bounds[1].resize(in.surface_vertices.size());
        bounds[2].resize(in.edges.size());
        bounds[3].resize(in.triangles.size());
        PreparedRequest result;
        auto&           prefix         = result.prefix;
        prefix.accepted_step_endpoints = true;
        prefix.positions               = in.positions.data();
        prefix.displacements           = in.displacements.data();
        prefix.d_hats                  = in.d_hats.data();
        prefix.thicknesses             = in.thicknesses.data();
        prefix.v2bs                    = in.v2bs.data();
        prefix.contact_ids             = in.contact_ids.data();
        prefix.vertices                = static_cast<int>(n);
        prefix.alpha                   = request.alpha;
        prefix.f32_positions           = f32_positions.data();
        prefix.f32_steps               = f32_steps.data();
        prefix.f32_ends                = f32_ends.data();
        for(SizeT f = 0; f < bounds.size(); ++f)
        {
            prefix.family[f].n     = static_cast<int>(bounds[f].boxes.size());
            prefix.family[f].aabbs = bounds[f].boxes.data();
            prefix.family[f].bids  = bounds[f].bids.data();
            prefix.family[f].cids  = bounds[f].cids.data();
        }
        prefix.family[0].vertices  = in.codim_vertices.data();
        prefix.family[1].vertices  = in.surface_vertices.data();
        prefix.family[2].edges     = in.edges.data();
        prefix.family[3].triangles = in.triangles.data();
        result.pp                  = {in.surface_vertices.cview(),
                                      in.codim_vertices.cview(),
                                      in.positions.cview(),
                                      in.displacements.cview(),
                                      in.thicknesses.cview(),
                                      in.dimensions.cview(),
                                      in.contact_ids.cview(),
                                      in.contact_mask.cviewer(),
                                      in.subscene_ids.cview(),
                                      in.subscene_mask.cviewer(),
                                      in.body_self_collision.cview(),
                                      in.d_hats.cview(),
                                      request.alpha,
                                      request.skin};
        result.pe                  = {in.codim_vertices.cview(),
                                      in.edges.cview(),
                                      in.positions.cview(),
                                      in.displacements.cview(),
                                      in.thicknesses.cview(),
                                      in.contact_ids.cview(),
                                      in.contact_mask.cviewer(),
                                      in.subscene_ids.cview(),
                                      in.subscene_mask.cviewer(),
                                      in.body_self_collision.cview(),
                                      in.d_hats.cview(),
                                      request.alpha,
                                      request.skin};
        result.ee                  = {in.edges.cview(),
                                      in.positions.cview(),
                                      in.displacements.cview(),
                                      in.thicknesses.cview(),
                                      in.contact_ids.cview(),
                                      in.contact_mask.cviewer(),
                                      in.subscene_ids.cview(),
                                      in.subscene_mask.cviewer(),
                                      in.body_self_collision.cview(),
                                      in.d_hats.cview(),
                                      request.alpha,
                                      request.skin};
        result.pt                  = {in.surface_vertices.cview(),
                                      in.triangles.cview(),
                                      in.positions.cview(),
                                      in.displacements.cview(),
                                      in.thicknesses.cview(),
                                      in.contact_ids.cview(),
                                      in.contact_mask.cviewer(),
                                      in.subscene_ids.cview(),
                                      in.subscene_mask.cviewer(),
                                      in.body_self_collision.cview(),
                                      in.d_hats.cview(),
                                      request.alpha,
                                      request.skin};
        if(request.alpha != 0.0 && n)
        {
            result.ee.f32_positions = f32_positions.cview();
            result.ee.f32_steps     = f32_steps.cview();
            result.pt.f32_positions = f32_positions.cview();
            result.pt.f32_steps     = f32_steps.cview();
        }
        result.node = {in.body_self_collision.cview(), in.contact_mask.cviewer()};
        result.codim     = {bounds[0].view(), result.pp, result.pe, &pp, &pe};
        result.new_frame = request.new_frame;
        return result;
    }

    bool detect(PreparedRequest& input)
    {
        return query.detect(tree,
                            input.pt,
                            input.ee,
                            input.node,
                            bounds[1].view(),
                            bounds[2].view(),
                            bounds[3].view(),
                            pt,
                            ee,
                            input.new_frame,
                            {},
                            &input.prefix,
                            &input.codim);
    }

    std::array<const InfoStacklessBVH::QueryBuffer*, 4> outputs() const
    {
        return {&pp, &pe, &ee, &pt};
    }

    Json counts() const
    {
        Json       result;
        const auto values = outputs();
        for(SizeT f = 0; f < values.size(); ++f)
            result[FamilyNames[f]] = values[f]->size();
        return result;
    }
};

std::vector<Pair> read_pairs(const InfoStacklessBVH::QueryBuffer& buffer, bool canonical)
{
    std::vector<Vector2i> raw(buffer.size());
    if(!raw.empty())
        buffer.view().copy_to(raw.data());
    std::vector<Pair> result;
    result.reserve(raw.size());
    for(const auto& pair : raw)
        result.emplace_back(canonical ? std::min(pair[0], pair[1]) : pair[0],
                            canonical ? std::max(pair[0], pair[1]) : pair[1]);
    std::sort(result.begin(), result.end());
    return result;
}

Candidates candidates(const ReplayState& state)
{
    Candidates result;
    const auto outputs = state.outputs();
    for(SizeT f = 0; f < outputs.size(); ++f)
        result[f] = read_pairs(*outputs[f], f == 2);
    return result;
}

Candidates direct_oracle(const UploadedInputs&  inputs,
                         const ReplayState&     state,
                         const PreparedRequest& request)
{
    InfoStacklessBVH              edge_tree, face_tree, codim_tree;
    InfoStacklessBVH::QueryBuffer pp, pe, ee, pt;
    const auto                    cp = state.bounds[0].view();
    const auto                    p  = state.bounds[1].view();
    const auto                    e  = state.bounds[2].view();
    const auto                    t  = state.bounds[3].view();
    if(inputs.edges.size())
    {
        edge_tree.build(e.aabbs, e.bids, e.cids);
        edge_tree.detect(inputs.contact_mask.cview(), request.node, request.ee, ee);
        if(inputs.codim_vertices.size())
            edge_tree.query(cp.aabbs,
                            cp.bids,
                            cp.cids,
                            inputs.contact_mask.cview(),
                            request.node,
                            request.pe,
                            pe);
    }
    if(inputs.triangles.size())
    {
        face_tree.build(t.aabbs, t.bids, t.cids);
        face_tree.query(p.aabbs,
                        p.bids,
                        p.cids,
                        inputs.contact_mask.cview(),
                        request.node,
                        request.pt,
                        pt);
    }
    if(inputs.codim_vertices.size())
    {
        codim_tree.build(cp.aabbs, cp.bids, cp.cids);
        codim_tree.query(p.aabbs,
                         p.bids,
                         p.cids,
                         inputs.contact_mask.cview(),
                         request.node,
                         request.pp,
                         pp);
    }
    return {read_pairs(pp, false), read_pairs(pe, false), read_pairs(ee, true), read_pairs(pt, false)};
}

Json cumulative(const Stats& s)
{
    return {{"refreshes", s.refreshes},
            {"reuses", s.reuses},
            {"topology_builds", s.topology_builds},
            {"refresh_waits", s.refresh_waits},
            {"ef_retries", s.ef_retries},
            {"typed_refills", s.typed_refills},
            {"cross_frame_reuses", s.cross_frame_reuses},
            {"fused_prefixes", s.fused_prefixes}};
}

Json gauges(const Stats& s)
{
    return {{"primary_queries", s.primary_queries},
            {"reverse_queries", s.reverse_queries},
            {"stored_pps", s.stored_pps},
            {"stored_pes", s.stored_pes},
            {"stored_ees", s.stored_ees},
            {"stored_pts", s.stored_pts},
            {"ef_pairs", s.single_ef_active ? Json(s.ef_pairs) : Json(nullptr)},
            {"pt_cover", s.pt_cover},
            {"ee_cover", s.ee_cover},
            {"cover_skin", s.cover_skin},
            {"other_skin", s.other_skin}};
}

Json statistics(const Stats& s)
{
    Json result                = cumulative(s);
    result["gauges"]           = gauges(s);
    result["single_ef_active"] = s.single_ef_active;
    return result;
}

Json delta(const Stats& before, const Stats& after)
{
    const auto a = cumulative(before), b = cumulative(after);
    Json       result;
    for(auto it = b.begin(); it != b.end(); ++it)
    {
        const auto previous = a.at(it.key()).get<SizeT>();
        const auto current  = it.value().get<SizeT>();
        REQUIRE(current >= previous);
        result[it.key()] = current - previous;
    }
    return result;
}

Json identity(const trace::Request& request)
{
    return {{"request", request.id},
            {"epoch", request.epoch},
            {"frame", request.site.frame},
            {"newton_iteration", request.site.newton_iteration},
            {"alpha", request.alpha},
            {"skin", request.skin},
            {"new_frame", request.new_frame}};
}

Json summary(const trace::Summary& value)
{
    return {{"requests", value.requests},
            {"epochs", value.epochs},
            {"bytes", value.bytes},
            {"complete", value.complete}};
}

trace::Request small_request(bool codim_only = false)
{
    auto           metadata = std::make_shared<trace::Metadata>();
    trace::Request request;
    request.site      = {1, 1};
    request.new_frame = true;
    request.positions = {
        {100, 100, 100}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 0.02}, {1, 0, 0.02}, {0, 1, 0.02}};
    request.displacements.assign(request.positions.size(), Vector3::Zero());
    metadata->d_hats.assign(request.positions.size(), 0.1);
    metadata->thicknesses.assign(request.positions.size(), 0.001);
    metadata->thicknesses[0] = 0.0;
    metadata->v2bs           = {0, 0, 0, 0, 1, 1, 1};
    metadata->dimensions     = {3, 3, 3, 3, 2, 2, 2};
    metadata->contact_ids    = {0, 0, 0, 0, 1, 1, 1};
    metadata->subscene_ids.assign(request.positions.size(), 0);
    metadata->body_self_collision = {0, 0};
    metadata->surface_vertices    = {6, 5, 4, 3, 2, 1};
    metadata->codim_vertices      = {6, 5, 4};
    metadata->edges         = {{1, 2}, {2, 3}, {3, 1}, {4, 5}, {5, 6}, {6, 4}};
    metadata->triangles     = {{1, 2, 3}, {4, 5, 6}};
    metadata->contact_mask  = {2, 2, {1, 1, 1, 1}};
    metadata->subscene_mask = {1, 1, {1}};
    if(codim_only)
    {
        metadata->edges.clear();
        metadata->triangles.clear();
        metadata->codim_vertices = metadata->surface_vertices;
        metadata->dimensions.assign(request.positions.size(), 0);
    }
    request.metadata = metadata;
    return request;
}

std::vector<trace::Request> small_sequence()
{
    auto first                   = small_request();
    auto second                  = first;
    second.alpha                 = 1.0;
    second.site.newton_iteration = 1;
    for(SizeT i = 1; i < second.displacements.size(); ++i)
        second.displacements[i] = Vector3{0.00001, -0.00001, 0};
    auto third                  = second;
    third.alpha                 = 0.5;
    third.new_frame             = false;
    third.site.newton_iteration = 2;
    auto fourth                 = third;
    fourth.site                 = {2, 1};
    fourth.new_frame            = true;
    for(SizeT i = 4; i < fourth.positions.size(); ++i)
        fourth.positions[i].z() += 0.000001;
    auto disabled          = fourth;
    disabled.site          = {3, 1};
    auto filtered_metadata = std::make_shared<trace::Metadata>(*first.metadata);
    filtered_metadata->contact_mask.values[1] = 0;
    filtered_metadata->contact_mask.values[2] = 0;
    disabled.metadata                         = filtered_metadata;
    auto reordered                            = first;
    reordered.site                            = {4, 1};
    auto reordered_metadata = std::make_shared<trace::Metadata>(*first.metadata);
    std::reverse(reordered_metadata->surface_vertices.begin(),
                 reordered_metadata->surface_vertices.end());
    std::reverse(reordered_metadata->codim_vertices.begin(),
                 reordered_metadata->codim_vertices.end());
    std::reverse(reordered_metadata->edges.begin(), reordered_metadata->edges.end());
    std::reverse(reordered_metadata->triangles.begin(),
                 reordered_metadata->triangles.end());
    reordered.metadata = reordered_metadata;
    return {first, second, third, fourth, disabled, reordered};
}

void require_same_candidates(const Candidates& expected,
                             const Candidates& actual,
                             SizeT             request,
                             const char*       state)
{
    CAPTURE(request, state);
    for(SizeT f = 0; f < FamilyNames.size(); ++f)
    {
        CAPTURE(FamilyNames[f]);
        REQUIRE(actual[f] == expected[f]);
    }
}
}  // namespace test_simplex_query_replay

TEST_CASE("Simplex query traces preserve exact epochs and request history",
          "[cuda][simplex_query_replay][simplex_query_trace]")
{
    using namespace test_simplex_query_replay;
    TemporaryDirectory directory;
    const auto         path   = directory.path / "roundtrip.bin";
    auto               first  = small_request();
    auto               second = first;
    second.alpha              = 1.0;
    second.displacements[4]   = Vector3{0.125, -0.25, 0.5};
    auto signed_zero          = second;
    auto changed = std::make_shared<trace::Metadata>(*first.metadata);
    changed->thicknesses[0] = -0.0;
    signed_zero.metadata    = changed;
    signed_zero.alpha       = 0.5;
    signed_zero.new_frame   = false;
    auto shape              = signed_zero;
    auto wider              = std::make_shared<trace::Metadata>(*changed);
    wider->contact_mask     = {3, 3, std::vector<IndexT>(9, 1)};
    shape.metadata          = wider;
    shape.site              = {2, 3};
    const std::array<trace::Request, 4> requests = {first, second, signed_zero, shape};
    {
        trace::Writer writer{path};
        for(const auto& request : requests)
            writer.append(request);
        writer.close();
        CHECK(writer.summary().requests == requests.size());
        CHECK(writer.summary().epochs == 3);
        CHECK(writer.summary().complete);
    }
    trace::Reader        reader{path};
    trace::Request       actual;
    std::array<SizeT, 4> epochs{}, ids{};
    for(SizeT i = 0; i < requests.size(); ++i)
    {
        CAPTURE(i);
        REQUIRE(reader.next(actual));
        const auto& expected = requests[i];
        epochs[i]            = actual.epoch;
        ids[i]               = actual.id;
        CHECK(actual.site.frame == expected.site.frame);
        CHECK(actual.site.newton_iteration == expected.site.newton_iteration);
        CHECK(actual.new_frame == expected.new_frame);
        CHECK(actual.alpha == expected.alpha);
        CHECK(actual.skin == expected.skin);
        REQUIRE(actual.positions.size() == expected.positions.size());
        REQUIRE(actual.displacements.size() == expected.displacements.size());
        for(SizeT v = 0; v < actual.positions.size(); ++v)
            for(int axis = 0; axis < 3; ++axis)
            {
                CHECK(std::memcmp(&actual.positions[v][axis],
                                  &expected.positions[v][axis],
                                  sizeof(Float))
                      == 0);
                CHECK(std::memcmp(&actual.displacements[v][axis],
                                  &expected.displacements[v][axis],
                                  sizeof(Float))
                      == 0);
            }
        REQUIRE(actual.metadata);
        CHECK(actual.metadata->surface_vertices == expected.metadata->surface_vertices);
        CHECK(actual.metadata->contact_mask.rows
              == expected.metadata->contact_mask.rows);
        CHECK(actual.metadata->contact_mask.cols
              == expected.metadata->contact_mask.cols);
        CHECK(actual.metadata->contact_mask.values
              == expected.metadata->contact_mask.values);
        CHECK(std::signbit(actual.metadata->thicknesses[0]) == (i >= 2));
        CHECK_FALSE(reader.summary().complete);
    }
    CHECK(epochs[0] == epochs[1]);
    CHECK(epochs[1] != epochs[2]);
    CHECK(epochs[2] != epochs[3]);
    for(SizeT i = 1; i < ids.size(); ++i)
        CHECK(ids[i] == ids[i - 1] + 1);
    CHECK_FALSE(reader.next(actual));
    CHECK(reader.summary().complete);
    CHECK(reader.summary().requests == requests.size());
    CHECK(reader.summary().epochs == 3);
    CHECK(reader.summary().bytes == fs::file_size(path));
}

TEST_CASE("Simplex query traces reject interrupted and malformed captures",
          "[cuda][simplex_query_replay][simplex_query_trace]")
{
    using namespace test_simplex_query_replay;
    TemporaryDirectory directory;
    const auto         path    = directory.path / "complete.bin";
    auto               consume = [](const fs::path& file)
    {
        trace::Reader  reader{file};
        trace::Request request;
        while(reader.next(request))
        {
        }
        if(!reader.summary().complete)
            throw std::runtime_error("Trace has no complete footer");
    };
    {
        trace::Writer writer{path};
        writer.append(small_request());
        CHECK_THROWS(consume(path));
        writer.close();
    }
    REQUIRE_NOTHROW(consume(path));
    std::ifstream           source{path, std::ios::binary};
    const std::vector<char> bytes{std::istreambuf_iterator<char>{source},
                                  std::istreambuf_iterator<char>{}};
    REQUIRE(bytes.size() > 16);
    const auto truncated = directory.path / "truncated.bin";
    {
        std::ofstream stream{truncated, std::ios::binary};
        stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size() - 8));
    }
    CHECK_THROWS(consume(truncated));
    const auto malformed = directory.path / "bad-magic.bin";
    {
        auto bad = bytes;
        bad[0] ^= 0x55;
        std::ofstream stream{malformed, std::ios::binary};
        stream.write(bad.data(), static_cast<std::streamsize>(bad.size()));
    }
    CHECK_THROWS(consume(malformed));
    const auto trailing = directory.path / "trailing.bin";
    {
        std::ofstream stream{trailing, std::ios::binary};
        stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        stream.put('x');
    }
    CHECK_THROWS(consume(trailing));
    const auto empty = directory.path / "empty.bin";
    {
        trace::Writer writer{empty};
        writer.close();
    }
    trace::Reader  reader{empty};
    trace::Request request;
    CHECK_FALSE(reader.next(request));
    CHECK(reader.summary().complete);
    CHECK(reader.summary().requests == 0);
}

TEST_CASE("Simplex query replay checks all families and cache history on identical requests",
          "[cuda][single_ef_bvh][simplex_query_replay][simplex_query_policy]")
{
    using namespace test_simplex_query_replay;
    TemporaryDirectory directory;
    const auto         path     = directory.path / "requests.bin";
    const auto         sequence = small_sequence();
    {
        trace::Writer writer{path};
        for(const auto& request : sequence)
            writer.append(request);
        writer.close();
    }
    UploadedInputs                              inputs;
    std::array<std::unique_ptr<ReplayState>, 4> states;
    for(SizeT s = 0; s < states.size(); ++s)
        states[s] = std::make_unique<ReplayState>(
            s % 2 ? Method::SingleEF : Method::InfoStackless, s >= 2);
    trace::Reader  reader{path};
    trace::Request request;
    SizeT          count = 0;
    while(reader.next(request))
    {
        inputs.send(request);
        std::array<PreparedRequest, 4> prepared;
        std::array<Candidates, 4>      actual;
        for(SizeT s = 0; s < states.size(); ++s)
        {
            prepared[s] = states[s]->bind(inputs, request);
            REQUIRE(states[s]->detect(prepared[s]));
            ct::wait_device();
            actual[s] = candidates(*states[s]);
            CHECK(states[s]->query.statistics().single_ef_active == (s % 2 != 0));
        }
        const auto expected = direct_oracle(inputs, *states[0], prepared[0]);
        for(SizeT s = 0; s < states.size(); ++s)
            require_same_candidates(expected, actual[s], request.id, StateNames[s]);
        if(count == 0)
            for(SizeT f = 0; f < FamilyNames.size(); ++f)
            {
                CAPTURE(FamilyNames[f]);
                CHECK_FALSE(expected[f].empty());
            }
        if(count == 4)
            for(const auto& family : expected)
                CHECK(family.empty());
        ++count;
    }
    REQUIRE(reader.summary().complete);
    REQUIRE(count == sequence.size());
    for(SizeT s = 0; s < states.size(); ++s)
    {
        const auto stats = states[s]->query.statistics();
        CHECK(stats.refreshes + stats.reuses == count);
        CHECK(stats.fused_prefixes == count);
        if(s >= 2)
        {
            CHECK(stats.refreshes == count);
            CHECK(stats.reuses == 0);
        }
        else
        {
            CHECK(stats.reuses > 0);
            CHECK(stats.cross_frame_reuses > 0);
            CHECK(stats.refreshes >= 3);
        }
    }
}

TEST_CASE("Simplex query replay labels codimensional fallback instead of EF execution",
          "[cuda][single_ef_bvh][simplex_query_replay][simplex_query_policy]")
{
    using namespace test_simplex_query_replay;
    TemporaryDirectory directory;
    const auto         path = directory.path / "codim.bin";
    {
        trace::Writer writer{path};
        writer.append(small_request(true));
        writer.close();
    }
    trace::Reader  reader{path};
    trace::Request request;
    REQUIRE(reader.next(request));
    UploadedInputs inputs;
    inputs.send(request);
    ReplayState info{Method::InfoStackless, false}, ef{Method::SingleEF, false};
    auto        a = info.bind(inputs, request), b = ef.bind(inputs, request);
    REQUIRE(info.detect(a));
    REQUIRE(ef.detect(b));
    ct::wait_device();
    CHECK_FALSE(info.query.statistics().single_ef_active);
    CHECK_FALSE(ef.query.statistics().single_ef_active);
    const auto expected = direct_oracle(inputs, info, a);
    require_same_candidates(expected, candidates(info), request.id, "info_natural");
    require_same_candidates(expected, candidates(ef), request.id, "single_ef_natural");
    CHECK_FALSE(expected[0].empty());
    CHECK(expected[1].empty());
    CHECK(expected[2].empty());
    CHECK(expected[3].empty());
    CHECK_FALSE(reader.next(request));
    CHECK(reader.summary().complete);
}

namespace test_simplex_query_replay
{
void verify_trace(const fs::path& input, const fs::path& output)
{
    trace::Reader                               reader{input};
    UploadedInputs                              inputs;
    std::array<std::unique_ptr<ReplayState>, 4> states;
    for(SizeT s = 0; s < states.size(); ++s)
        states[s] = std::make_unique<ReplayState>(
            s % 2 ? Method::SingleEF : Method::InfoStackless, s >= 2);
    save_json(output / "loaded-files-init.json", loaded_files());
    std::ofstream ledger{output / "requests.jsonl"};
    REQUIRE(ledger.good());
    SizeT          requests = 0;
    trace::Request request;
    while(reader.next(request))
    {
        inputs.send(request);
        Json row       = identity(request);
        row["success"] = true;
        row["states"]  = Json::object();
        std::array<Candidates, 4> all;
        PreparedRequest           oracle_request;
        for(SizeT s = 0; s < states.size(); ++s)
        {
            auto prepared = states[s]->bind(inputs, request);
            REQUIRE(states[s]->detect(prepared));
            ct::wait_device();
            if(s == 0)
                oracle_request = prepared;
            all[s]                       = candidates(*states[s]);
            const auto stats             = states[s]->query.statistics();
            row["states"][StateNames[s]] = {{"counts", states[s]->counts()},
                                            {"single_ef_active", stats.single_ef_active},
                                            {"statistics", statistics(stats)}};
        }
        const auto oracle = direct_oracle(inputs, *states[0], oracle_request);
        row["oracle_counts"]            = Json::object();
        row["independent_oracle_equal"] = true;
        for(SizeT f = 0; f < FamilyNames.size(); ++f)
            row["oracle_counts"][FamilyNames[f]] = oracle[f].size();
        for(SizeT s = 0; s < states.size(); ++s)
            for(SizeT f = 0; f < FamilyNames.size(); ++f)
                if(all[s][f] != oracle[f])
                {
                    row["success"]                  = false;
                    row["independent_oracle_equal"] = false;
                    row["mismatch"]                 = {{"state", StateNames[s]},
                                                       {"family", FamilyNames[f]},
                                                       {"expected_count", oracle[f].size()},
                                                       {"actual_count", all[s][f].size()}};
                    ledger << row.dump() << '\n';
                    ledger.flush();
                    save_json(output / "verification.json",
                              Json{{"trace_complete", false},
                                   {"requests", requests},
                                   {"success", false},
                                   {"all_candidates_equal", false},
                                   {"independent_oracle_equal", false},
                                   {"failed_request", request.id},
                                   {"mismatch", row["mismatch"]}});
                    CAPTURE(request.id, StateNames[s], FamilyNames[f]);
                    FAIL("Same-state query replay candidate multisets differ");
                }
        ledger << row.dump() << '\n';
        REQUIRE(ledger.good());
        if(++requests % 64 == 0)
            ledger.flush();
    }
    ledger.close();
    REQUIRE(ledger.good());
    const auto complete = reader.summary();
    REQUIRE(complete.complete);
    REQUIRE(complete.requests == requests);
    REQUIRE(requests > 0);
    save_json(output / "loaded-files-final.json", loaded_files());
    save_json(output / "verification.json",
              Json{{"trace_complete", complete.complete},
                   {"requests", requests},
                   {"epochs", complete.epochs},
                   {"bytes", complete.bytes},
                   {"success", true},
                   {"all_candidates_equal", true},
                   {"independent_oracle_equal", true},
                   {"states", StateNames},
                   {"trace_summary", summary(complete)}});
}

struct ProfilerRange
{
    bool active = false;
    void start()
    {
        CUDA_TOOL_CHECK(cudaProfilerStart());
        active = true;
    }
    void stop()
    {
        if(active)
        {
            CUDA_TOOL_CHECK(cudaProfilerStop());
            active = false;
        }
    }
    ~ProfilerRange()
    {
        if(active)
            cudaProfilerStop();
    }
};

void benchmark_trace(const fs::path&    input,
                     const fs::path&    output,
                     Method             method,
                     const std::string& method_name,
                     bool               force_refresh,
                     const std::string& policy_name,
                     bool               profile)
{
    trace::Reader                reader{input};
    UploadedInputs               inputs;
    std::unique_ptr<ReplayState> state;
    std::ofstream                ledger{output / "requests.jsonl"};
    REQUIRE(ledger.good());
    save_json(output / "loaded-files-init.json", loaded_files());
    ProfilerRange profiler;
    SizeT         requests = 0, measured_requests = 0;
    SizeT         measured_refreshes = 0, measured_reuses = 0;
    SizeT         first_measured_frame = 0, last_measured_frame = 0;
    double        constructor_ms = 0.0, first_detect_ms = 0.0;
    double measured_ms = 0.0, measured_refresh_ms = 0.0, measured_reuse_ms = 0.0;
    Json           measured_deltas = cumulative(Stats{});
    trace::Request request;
    while(reader.next(request))
    {
        inputs.send(request);
        ct::wait_device();
        if(!state)
        {
            const auto begin = Clock::now();
            state = std::make_unique<ReplayState>(method, force_refresh);
            ct::wait_device();
            constructor_ms =
                std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
        }
        // Device uploads, external output allocation and binding are not query cost.
        auto prepared = state->bind(inputs, request);
        ct::wait_device();
        const bool measured = request.site.frame >= 31 && request.site.frame <= 1000;
        if(profile && measured && !profiler.active)
            profiler.start();
        const auto before   = state->query.statistics();
        const auto begin    = Clock::now();
        const bool accepted = state->detect(prepared);
        ct::wait_device();
        const double query_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
        REQUIRE(accepted);
        REQUIRE(std::isfinite(query_ms));
        const auto after   = state->query.statistics();
        const auto changes = delta(before, after);
        if(requests == 0)
            first_detect_ms = query_ms;
        if(measured)
        {
            ++measured_requests;
            measured_ms += query_ms;
            if(first_measured_frame == 0)
                first_measured_frame = request.site.frame;
            last_measured_frame  = request.site.frame;
            const auto refreshes = after.refreshes - before.refreshes;
            const auto reuses    = after.reuses - before.reuses;
            REQUIRE(refreshes + reuses == 1);
            if(refreshes)
            {
                ++measured_refreshes;
                measured_refresh_ms += query_ms;
            }
            else
            {
                ++measured_reuses;
                measured_reuse_ms += query_ms;
            }
            for(auto it = changes.begin(); it != changes.end(); ++it)
                measured_deltas[it.key()] = measured_deltas.at(it.key()).get<SizeT>()
                                            + it.value().get<SizeT>();
        }
        Json row                = identity(request);
        row["measured"]         = measured;
        row["query_ms"]         = query_ms;
        row["counts"]           = state->counts();
        row["single_ef_active"] = after.single_ef_active;
        row["before"]           = cumulative(before);
        row["after"]            = cumulative(after);
        row["delta"]            = changes;
        row["gauges"]           = gauges(after);
        ledger << row.dump() << '\n';
        REQUIRE(ledger.good());
        if(++requests % 64 == 0)
            ledger.flush();
    }
    profiler.stop();
    ledger.close();
    REQUIRE(ledger.good());
    const auto complete = reader.summary();
    REQUIRE(complete.complete);
    REQUIRE(complete.requests == requests);
    REQUIRE(requests > 0);
    REQUIRE(measured_requests > 0);
    REQUIRE(state);
    save_json(output / "loaded-files-final.json", loaded_files());
    save_json(output / "result.json",
              Json{{"trace_complete", complete.complete},
                   {"requests", requests},
                   {"measured_requests", measured_requests},
                   {"method", method_name},
                   {"policy", policy_name},
                   {"profile", profile},
                   {"constructor_ms", constructor_ms},
                   {"first_detect_ms", first_detect_ms},
                   {"cold_total_ms", constructor_ms + first_detect_ms},
                   {"cold_samples", 1},
                   {"measured_query_ms", measured_ms},
                   {"measured_refresh_requests", measured_refreshes},
                   {"measured_reuse_requests", measured_reuses},
                   {"measured_refresh_query_ms", measured_refresh_ms},
                   {"measured_reuse_query_ms", measured_reuse_ms},
                   {"measured_counter_deltas", measured_deltas},
                   {"first_measured_frame", first_measured_frame},
                   {"last_measured_frame", last_measured_frame},
                   {"trace_summary", summary(complete)},
                   {"final_statistics", statistics(state->query.statistics())}});
}
}  // namespace test_simplex_query_replay

TEST_CASE("Frozen production simplex queries replay without solver evolution", "[.simplex_query_replay]")
{
    using namespace test_simplex_query_replay;
    const auto* input_env  = std::getenv("UIPC_SIMPLEX_QUERY_REPLAY_INPUT");
    const auto* output_env = std::getenv("UIPC_SIMPLEX_QUERY_REPLAY_OUTPUT");
    REQUIRE(input_env);
    REQUIRE(output_env);
    const fs::path input{input_env}, output{output_env};
    REQUIRE(fs::is_regular_file(input));
    REQUIRE_FALSE(fs::exists(output));
    REQUIRE(fs::create_directories(output));
    Timer::disable_all();
    logger::set_level(spdlog::level::off);
    ct::wait_device();
    const auto* verify_env = std::getenv("UIPC_SIMPLEX_QUERY_REPLAY_VERIFY");
    if(verify_env && std::string_view{verify_env} == "1")
        verify_trace(input, output);
    else
    {
        const auto* method_env = std::getenv("UIPC_SIMPLEX_QUERY_REPLAY_METHOD");
        const auto* policy_env = std::getenv("UIPC_SIMPLEX_QUERY_REPLAY_POLICY");
        REQUIRE(method_env);
        REQUIRE(policy_env);
        const std::string method{method_env}, policy{policy_env};
        REQUIRE((method == "info_stackless_bvh" || method == "single_ef_bvh"));
        REQUIRE((policy == "natural" || policy == "refresh"));
        const auto* profile_env = std::getenv("UIPC_SIMPLEX_QUERY_REPLAY_PROFILE");
        const bool profile = profile_env && std::string_view{profile_env} == "1";
        benchmark_trace(input,
                        output,
                        method == "single_ef_bvh" ? Method::SingleEF : Method::InfoStackless,
                        method,
                        policy == "refresh",
                        policy,
                        profile);
    }
}

TEST_CASE("Simplex query replay reports retain warmup history and verified counts",
          "[cuda][simplex_query_replay][simplex_query_policy]")
{
    using namespace test_simplex_query_replay;
    TemporaryDirectory directory;
    const auto         input    = directory.path / "history.bin";
    auto               sequence = small_sequence();
    for(SizeT i = 3; i < sequence.size(); ++i)
        sequence[i].site.frame = 31 + i - 3;
    {
        trace::Writer writer{input};
        for(const auto& request : sequence)
            writer.append(request);
        writer.close();
    }
    const auto verification_output = directory.path / "verify";
    REQUIRE(fs::create_directory(verification_output));
    verify_trace(input, verification_output);
    std::ifstream     verified_stream{verification_output / "requests.jsonl"};
    std::vector<Json> verified;
    for(std::string line; std::getline(verified_stream, line);)
        verified.push_back(Json::parse(line));
    REQUIRE(verified.size() == sequence.size());
    for(SizeT method = 0; method < 2; ++method)
    {
        const auto output = directory.path / (method ? "single" : "info");
        REQUIRE(fs::create_directory(output));
        benchmark_trace(input,
                        output,
                        method ? Method::SingleEF : Method::InfoStackless,
                        method ? "single_ef_bvh" : "info_stackless_bvh",
                        false,
                        "natural",
                        false);
        std::ifstream result_stream{output / "result.json"};
        const auto    result = Json::parse(result_stream);
        CHECK(result.at("trace_complete").get<bool>());
        CHECK(result.at("requests").get<SizeT>() == sequence.size());
        CHECK(result.at("measured_requests").get<SizeT>() == 3);
        CHECK(result.at("final_statistics").at("fused_prefixes").get<SizeT>()
              == sequence.size());
        std::ifstream ledger{output / "requests.jsonl"};
        SizeT         index = 0;
        for(std::string line; std::getline(ledger, line);)
        {
            REQUIRE(index < verified.size());
            const auto row = Json::parse(line);
            CHECK(row.at("request") == verified[index].at("request"));
            CHECK(row.at("frame") == verified[index].at("frame"));
            CHECK(row.at("new_frame") == verified[index].at("new_frame"));
            CHECK(row.at("counts")
                  == verified[index].at("states").at(StateNames[method]).at("counts"));
            CHECK(row.at("measured").get<bool>() == (index >= 3));
            ++index;
        }
        CHECK(index == verified.size());
    }
}

TEST_CASE("Simplex query traces reject negative alpha without limiting positive steps",
          "[cuda][simplex_query_replay][simplex_query_trace]")
{
    using namespace test_simplex_query_replay;
    TemporaryDirectory directory;
    const auto         path    = directory.path / "alpha.bin";
    auto               request = small_request();
    {
        trace::Writer writer{path};
        request.alpha = -0.25;
        CHECK_THROWS_WITH(writer.append(request),
                          "Simplex query trace: invalid request shape or scalar");
        CHECK(writer.summary().requests == 0);
        request.alpha = 1.5;
        REQUIRE_NOTHROW(writer.append(request));
        writer.close();
    }
    trace::Reader  reader{path};
    trace::Request restored;
    REQUIRE(reader.next(restored));
    CHECK(restored.alpha == 1.5);
    CHECK_FALSE(reader.next(restored));
    CHECK(reader.summary().complete);
}

TEST_CASE("Simplex query trace declared lengths are checked before allocation",
          "[cuda][simplex_query_replay][simplex_query_trace]")
{
    using namespace test_simplex_query_replay;
    using Word = std::uint64_t;
    TemporaryDirectory directory;
    const auto         valid    = directory.path / "valid.bin";
    const auto         original = small_request();
    {
        trace::Writer writer{valid};
        writer.append(original);
        writer.close();
    }
    std::ifstream           source{valid, std::ios::binary};
    const std::vector<char> bytes{std::istreambuf_iterator<char>{source},
                                  std::istreambuf_iterator<char>{}};
    // Corrupt the declared sizes of a valid version-1 stream, keeping its
    // header/footer and enough remaining payload to exercise the allocation cap.
    constexpr SizeT header_bytes           = 8 + 4 * sizeof(Word);
    constexpr SizeT footer_bytes           = 8 + 2 * sizeof(Word);
    constexpr SizeT metadata_length_offset = header_bytes + 1 + sizeof(Word);
    REQUIRE(bytes.size() > metadata_length_offset + sizeof(Word) + footer_bytes);
    REQUIRE(static_cast<unsigned char>(bytes[header_bytes]) == 1);
    Word metadata_bytes = 0;
    std::memcpy(&metadata_bytes, bytes.data() + metadata_length_offset, sizeof(Word));
    const SizeT request_offset = metadata_length_offset + sizeof(Word) + metadata_bytes;
    REQUIRE(request_offset < bytes.size());
    REQUIRE(static_cast<unsigned char>(bytes[request_offset]) == 2);
    const SizeT positions_length_offset =
        request_offset + 1 + 4 * sizeof(Word) + 1 + 2 * sizeof(Float);
    REQUIRE(positions_length_offset + sizeof(Word) < bytes.size() - footer_bytes);
    const Word excessive_vertices = original.positions.size() + 1;

    SECTION("metadata payload is bounded independently of sparse trace size")
    {
        const auto     path = directory.path / "large-metadata.bin";
        constexpr Word excessive_metadata = Word{256} * 1024 * 1024 + 1;
        {
            std::ofstream stream{path, std::ios::binary};
            stream.write(bytes.data(), static_cast<std::streamsize>(metadata_length_offset));
            stream.write(reinterpret_cast<const char*>(&excessive_metadata), sizeof(Word));
            // seekp creates a sparse hole; no 256 MiB host buffer is allocated.
            stream.seekp(static_cast<std::streamoff>(
                metadata_length_offset + sizeof(Word) + excessive_metadata));
            stream.write(bytes.data() + bytes.size() - footer_bytes, footer_bytes);
            REQUIRE(stream.good());
        }
        trace::Reader  reader{path};
        trace::Request request;
        CHECK_THROWS_WITH(reader.next(request),
                          "Simplex query trace: invalid metadata epoch/length");
        CHECK(request.positions.empty());
        CHECK(request.displacements.empty());
    }
    SECTION("positions cannot allocate beyond metadata vertex capacity")
    {
        auto malformed = bytes;
        std::memcpy(malformed.data() + positions_length_offset,
                    &excessive_vertices,
                    sizeof(Word));
        const auto available = malformed.size() - footer_bytes
                               - positions_length_offset - sizeof(Word);
        REQUIRE(available >= excessive_vertices * 3 * sizeof(Float));
        const auto path = directory.path / "large-positions.bin";
        {
            std::ofstream stream{path, std::ios::binary};
            stream.write(malformed.data(),
                         static_cast<std::streamsize>(malformed.size()));
        }
        trace::Reader  reader{path};
        trace::Request request;
        request.positions.assign(1, Vector3{123, 456, 789});
        CHECK_THROWS_WITH(reader.next(request),
                          "Simplex query trace: vector length exceeds record");
        CHECK(request.positions.size() == 1);
        CHECK(request.positions[0].x() == 123);
    }
    SECTION("displacements cannot allocate beyond the decoded position count")
    {
        const SizeT displacement_length_offset =
            positions_length_offset + sizeof(Word)
            + original.positions.size() * 3 * sizeof(Float);
        REQUIRE(displacement_length_offset + sizeof(Word) < bytes.size() - footer_bytes);
        auto malformed = bytes;
        std::memcpy(malformed.data() + displacement_length_offset,
                    &excessive_vertices,
                    sizeof(Word));
        malformed.insert(malformed.end() - footer_bytes, 3 * sizeof(Float), '\0');
        const auto path = directory.path / "large-displacements.bin";
        {
            std::ofstream stream{path, std::ios::binary};
            stream.write(malformed.data(),
                         static_cast<std::streamsize>(malformed.size()));
        }
        trace::Reader  reader{path};
        trace::Request request;
        request.displacements.assign(1, Vector3{123, 456, 789});
        CHECK_THROWS_WITH(reader.next(request),
                          "Simplex query trace: vector length exceeds record");
        CHECK(request.positions.size() == original.positions.size());
        CHECK(request.displacements.size() == 1);
        CHECK(request.displacements[0].x() == 123);
    }
}

TEST_CASE("Simplex query traces preserve unused vertex IDs and validate referenced IDs",
          "[cuda][simplex_query_replay][simplex_query_trace]")
{
    using namespace test_simplex_query_replay;
    TemporaryDirectory directory;
    auto               request = small_request();
    auto metadata = std::make_shared<trace::Metadata>(*request.metadata);
    // Vertex zero is deliberately absent from every primitive family. Global
    // implicit-geometry metadata need not index the simplicial query tables.
    metadata->v2bs[0]         = 31;
    metadata->contact_ids[0]  = 41;
    metadata->subscene_ids[0] = 51;
    request.positions[0]      = Vector3{17.0, -0.0, 91.5};
    request.displacements[0]  = Vector3{-0.0, 6.5, -7.0};
    request.metadata          = metadata;
    const auto path           = directory.path / "unused-global-vertex.bin";
    {
        trace::Writer writer{path};
        REQUIRE_NOTHROW(writer.append(request));
        writer.close();
    }
    trace::Reader  reader{path};
    trace::Request restored;
    REQUIRE(reader.next(restored));
    REQUIRE(restored.metadata);
    CHECK(restored.positions.size() == request.positions.size());
    CHECK(restored.displacements.size() == request.displacements.size());
    CHECK(restored.metadata->v2bs == metadata->v2bs);
    CHECK(restored.metadata->contact_ids == metadata->contact_ids);
    CHECK(restored.metadata->subscene_ids == metadata->subscene_ids);
    CHECK(restored.metadata->surface_vertices == metadata->surface_vertices);
    CHECK(restored.metadata->codim_vertices == metadata->codim_vertices);
    for(SizeT v = 0; v < request.positions.size(); ++v)
        for(int axis = 0; axis < 3; ++axis)
        {
            CHECK(std::memcmp(&restored.positions[v][axis], &request.positions[v][axis], sizeof(Float))
                  == 0);
            CHECK(std::memcmp(&restored.displacements[v][axis],
                              &request.displacements[v][axis],
                              sizeof(Float))
                  == 0);
        }
    CHECK_FALSE(reader.next(restored));
    CHECK(reader.summary().complete);

    for(int field = 0; field < 3; ++field)
        for(int family = 0; family < 4; ++family)
        {
            CAPTURE(field, family);
            auto referenced = small_request();
            auto bad = std::make_shared<trace::Metadata>(*referenced.metadata);
            if(field == 0)
                bad->v2bs[0] = 31;
            else if(field == 1)
                bad->contact_ids[0] = 41;
            else
                bad->subscene_ids[0] = 51;
            if(family == 0)
                bad->surface_vertices.push_back(0);
            else if(family == 1)
                bad->codim_vertices.push_back(0);
            else if(family == 2)
                bad->edges.emplace_back(0, 1);
            else
                bad->triangles.emplace_back(0, 1, 2);
            referenced.metadata = bad;
            const auto invalid  = directory.path
                                 / ("referenced-" + std::to_string(field) + "-"
                                    + std::to_string(family) + ".bin");
            trace::Writer writer{invalid};
            CHECK_THROWS_WITH(writer.append(referenced),
                              "Simplex query trace: metadata index exceeds its table");
            CHECK(writer.summary().requests == 0);
        }
}
