#include <type_define.h>
#include <app/app.h>
#include <collision_detection/filters/single_ef_bvh_query.h>
#include <collision_detection/segment_aabb.h>
#include <cuda_runtime.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

namespace test_single_ef_bvh
{
using namespace uipc;
using namespace uipc::backend::cuda;
namespace ct  = uipc::backend::cuda_tool;
using Pair    = std::pair<IndexT, IndexT>;
using Backend = SingleEFBVHQuery::Backend;

std::vector<Backend> query_backends()
{
    std::vector<Backend> result{Backend::Cuda};
    if(OptixEFBVH::compiled())
        result.push_back(Backend::Optix);
    return result;
}

void check_backend_execution(const SingleEFBVHQuery& query, bool supported)
{
    const auto stats = query.statistics();
    if(query.backend() == Backend::Optix && supported)
    {
        CHECK(stats.builds > 0);
        CHECK(stats.launches > 0);
        CHECK(query.last_statistics().launches > 0);
    }
    else
    {
        CHECK(stats.builds == 0);
        CHECK(stats.refits == 0);
        CHECK(stats.launches == 0);
    }
}

template <typename T>
void upload(ct::DeviceBuffer<T>& device, const std::vector<T>& host)
{
    device.resize(host.size());
    if(!host.empty())
        device.view().copy_from(host.data());
}

struct Bounds
{
    std::vector<AABB>        boxes;
    std::vector<IndexT>      bids, cids;
    ct::DeviceBuffer<AABB>   d_boxes;
    ct::DeviceBuffer<IndexT> d_bids, d_cids;

    void send()
    {
        upload(d_boxes, boxes);
        upload(d_bids, bids);
        upload(d_cids, cids);
    }
    SingleEFBVHQuery::PrimitiveBounds view() const
    {
        return {d_boxes.view(), d_bids.view(), d_cids.view()};
    }
};

struct Mesh
{
    Float                      radius = 0.1;
    std::vector<Vector3>       positions;
    std::vector<Float>         thicknesses, dhats;
    std::vector<IndexT>        vertices, bodies, contacts, subscenes;
    std::vector<Vector2i>      edges;
    std::vector<Vector3i>      triangles;
    std::vector<IndexT>        self          = {1, 1};
    std::vector<IndexT>        contact_mask  = {1, 1, 1, 1};
    std::vector<IndexT>        subscene_mask = {1, 1, 1, 1};
    ct::DeviceBuffer<Vector3>  d_positions, d_displacements;
    ct::DeviceBuffer<IndexT>   d_vertices, d_contacts, d_subscenes, d_self;
    ct::DeviceBuffer<Vector2i> d_edges;
    ct::DeviceBuffer<Vector3i> d_triangles;
    ct::DeviceBuffer<Float>    d_thicknesses, d_hats;
    ct::DeviceBuffer2D<IndexT> d_contact_mask{ct::Extent2D{2, 2}};
    ct::DeviceBuffer2D<IndexT> d_subscene_mask{ct::Extent2D{2, 2}};
    Bounds                     point_bounds, edge_bounds, triangle_bounds;

    explicit Mesh(IndexT layers)
    {
        // Padding and reversed surface order distinguish global IDs from rows.
        positions.emplace_back(100, 100, 100);
        bodies.push_back(0);
        contacts.push_back(0);
        subscenes.push_back(0);
        for(IndexT t = 0; t < layers; ++t)
        {
            IndexT base = static_cast<IndexT>(positions.size());
            Float  z    = t * 0.0005;
            positions.emplace_back(0, 0, z);
            positions.emplace_back(1, 0, z);
            positions.emplace_back(0, 1, z);
            triangles.emplace_back(base, base + 1, base + 2);
            // The first edge permits one representative to own both endpoints.
            edges.emplace_back(base, base + 1);
            edges.emplace_back(base + 1, base + 2);
            edges.emplace_back(base + 2, base);
            for(IndexT k = 0; k < 3; ++k)
            {
                vertices.push_back(base + k);
                bodies.push_back(t % 2);
                contacts.push_back(t % 2);
                subscenes.push_back((t / 2) % 2);
            }
        }
        std::reverse(vertices.begin(), vertices.end());
        thicknesses.assign(positions.size(), 0.0);
        dhats.assign(positions.size(), radius);
    }

    void add_codimensional_geometry()
    {
        const auto base = static_cast<IndexT>(positions.size());
        const std::array<Vector3, 5> added = {Vector3{0.2, 0.2, 0.025},
                                              Vector3{-0.1, 0.2, 0.01},
                                              Vector3{0.8, 0.2, 0.01},
                                              Vector3{0.3, -0.1, 0.02},
                                              Vector3{0.3, 0.8, 0.02}};
        for(SizeT i = 0; i < added.size(); ++i)
        {
            positions.push_back(added[i]);
            vertices.push_back(base + static_cast<IndexT>(i));
            bodies.push_back(i % 2);
            contacts.push_back(i % 2);
            subscenes.push_back(i % 2);
            thicknesses.push_back(0.001);
            dhats.push_back(radius);
        }
        // Put wire edges on both sides of the surface-edge indices. EE
        // coverage must not depend on which primitive happens to be smaller.
        edges.insert(edges.begin(), Vector2i{base + 1, base + 2});
        edges.emplace_back(base + 3, base + 4);
    }

    template <size_t N>
    void add_bounds(Bounds&                      out,
                    const std::array<IndexT, N>& ids,
                    const std::vector<Vector3>&  motion,
                    Float                        alpha)
    {
        AABB box;
        for(auto id : ids)
        {
            box.extend(positions[id].template cast<float>());
            if(!motion.empty())
                box.extend((positions[id] + alpha * motion[id]).template cast<float>());
        }
        Float expansion;
        if constexpr(N == 1)
            expansion = point_dcd_expansion(dhats[ids[0]]) + thicknesses[ids[0]];
        else if constexpr(N == 2)
            expansion = edge_dcd_expansion(dhats[ids[0]], dhats[ids[1]])
                        + edge_thickness(thicknesses[ids[0]], thicknesses[ids[1]]);
        else
            expansion =
                triangle_dcd_expansion(dhats[ids[0]], dhats[ids[1]], dhats[ids[2]])
                + triangle_thickness(thicknesses[ids[0]],
                                     thicknesses[ids[1]],
                                     thicknesses[ids[2]]);
        box.min().array() -= static_cast<float>(expansion);
        box.max().array() += static_cast<float>(expansion);
        out.boxes.push_back(box);
        out.bids.push_back(bodies[ids[0]]);
        out.cids.push_back(contacts[ids[0]]);
    }

    void send(const std::vector<Vector3>& motion = {}, Float alpha = 0.0)
    {
        REQUIRE((motion.empty() || motion.size() == positions.size()));
        upload(d_positions, positions);
        upload(d_displacements,
               motion.empty() ? std::vector<Vector3>(positions.size(), Vector3::Zero()) : motion);
        upload(d_vertices, vertices);
        upload(d_edges, edges);
        upload(d_triangles, triangles);
        upload(d_contacts, contacts);
        upload(d_subscenes, subscenes);
        upload(d_self, self);
        upload(d_thicknesses, thicknesses);
        upload(d_hats, dhats);
        d_contact_mask.view().copy_from(contact_mask.data());
        d_subscene_mask.view().copy_from(subscene_mask.data());
        point_bounds.boxes.clear();
        point_bounds.bids.clear();
        point_bounds.cids.clear();
        edge_bounds.boxes.clear();
        edge_bounds.bids.clear();
        edge_bounds.cids.clear();
        triangle_bounds.boxes.clear();
        triangle_bounds.bids.clear();
        triangle_bounds.cids.clear();
        for(auto v : vertices)
            add_bounds(point_bounds, std::array<IndexT, 1>{v}, motion, alpha);
        for(const auto& e : edges)
            add_bounds(edge_bounds, std::array<IndexT, 2>{e[0], e[1]}, motion, alpha);
        for(const auto& f : triangles)
            add_bounds(triangle_bounds, std::array<IndexT, 3>{f[0], f[1], f[2]}, motion, alpha);
        point_bounds.send();
        edge_bounds.send();
        triangle_bounds.send();
    }

    bvh_simplex_detail::PointTrianglePredicate pt() const
    {
        return {d_vertices.view(),
                d_triangles.view(),
                d_positions.view(),
                d_displacements.view(),
                d_thicknesses.view(),
                d_contacts.view(),
                d_contact_mask.cviewer(),
                d_subscenes.view(),
                d_subscene_mask.cviewer(),
                d_self.view(),
                d_hats.view(),
                0.0};
    }
    bvh_simplex_detail::EdgeEdgePredicate ee() const
    {
        return {d_edges.view(),
                d_positions.view(),
                d_displacements.view(),
                d_thicknesses.view(),
                d_contacts.view(),
                d_contact_mask.cviewer(),
                d_subscenes.view(),
                d_subscene_mask.cviewer(),
                d_self.view(),
                d_hats.view(),
                0.0};
    }
    bvh_simplex_detail::NodePredicate node() const
    {
        return {d_self.view(), d_contact_mask.cviewer()};
    }

    bool allowed(IndexT a, IndexT b) const
    {
        return contact_mask[contacts[a] * 2 + contacts[b]]
               && subscene_mask[subscenes[a] * 2 + subscenes[b]];
    }

    // Exhaustive CPU enumeration is independent of BVH, ownership and chunking.
    std::pair<std::vector<Pair>, std::vector<Pair>> all_pairs() const
    {
        std::vector<Pair> pts, ees;
        for(IndexT i = 0; i < static_cast<IndexT>(vertices.size()); ++i)
        {
            IndexT v = vertices[i];
            for(IndexT j = 0; j < static_cast<IndexT>(triangles.size()); ++j)
            {
                auto f = triangles[j];
                if(v == f[0] || v == f[1] || v == f[2])
                    continue;
                if(!allowed(v, f[0]) || !allowed(v, f[1]) || !allowed(v, f[2]))
                    continue;
                if(bodies[v] == bodies[f[0]] && !self[bodies[v]])
                    continue;
                Float d2;
                auto  flag = distance::point_triangle_distance_flag(
                    positions[v], positions[f[0]], positions[f[1]], positions[f[2]]);
                distance::point_triangle_distance2(
                    flag, positions[v], positions[f[0]], positions[f[1]], positions[f[2]], d2);
                Float threshold =
                    PT_d_hat(dhats[v], dhats[f[0]], dhats[f[1]], dhats[f[2]])
                    + PT_thickness(thicknesses[v],
                                   thicknesses[f[0]],
                                   thicknesses[f[1]],
                                   thicknesses[f[2]]);
                if(d2 < threshold * threshold)
                    pts.emplace_back(i, j);
            }
        }
        for(IndexT i = 0; i < static_cast<IndexT>(edges.size()); ++i)
            for(IndexT j = i + 1; j < static_cast<IndexT>(edges.size()); ++j)
            {
                auto a = edges[i];
                auto b = edges[j];
                if(a[0] == b[0] || a[0] == b[1] || a[1] == b[0] || a[1] == b[1])
                    continue;
                if(!allowed(a[0], b[0]) || !allowed(a[0], b[1])
                   || !allowed(a[1], b[0]) || !allowed(a[1], b[1]))
                    continue;
                if(bodies[a[0]] == bodies[b[0]] && !self[bodies[a[0]]])
                    continue;
                Float d2;
                auto  flag = distance::edge_edge_distance_flag(
                    positions[a[0]], positions[a[1]], positions[b[0]], positions[b[1]]);
                distance::edge_edge_distance2(flag,
                                              positions[a[0]],
                                              positions[a[1]],
                                              positions[b[0]],
                                              positions[b[1]],
                                              d2);
                Float threshold =
                    EE_d_hat(dhats[a[0]], dhats[a[1]], dhats[b[0]], dhats[b[1]])
                    + EE_thickness(thicknesses[a[0]],
                                   thicknesses[a[1]],
                                   thicknesses[b[0]],
                                   thicknesses[b[1]]);
                if(d2 < threshold * threshold)
                    ees.emplace_back(i, j);
            }
        std::sort(pts.begin(), pts.end());
        std::sort(ees.begin(), ees.end());
        return {pts, ees};
    }
};

std::vector<Pair> read_pairs(const InfoStacklessBVH::QueryBuffer& buffer, bool canonical)
{
    std::vector<Vector2i> raw(buffer.size());
    if(!raw.empty())
        buffer.view().copy_to(raw.data());
    std::vector<Pair> result;
    for(const auto& p : raw)
        result.emplace_back(canonical ? std::min(p[0], p[1]) : p[0],
                            canonical ? std::max(p[0], p[1]) : p[1]);
    std::sort(result.begin(), result.end());
    return result;
}

void check_mesh(Mesh& mesh, SizeT chunk, Backend backend, bool check_cpu = true)
{
    mesh.send();
    InfoStacklessBVH baseline_e, baseline_t, single_t;
    baseline_e.build(mesh.edge_bounds.d_boxes,
                     mesh.edge_bounds.d_bids,
                     mesh.edge_bounds.d_cids);
    baseline_t.build(mesh.triangle_bounds.d_boxes,
                     mesh.triangle_bounds.d_bids,
                     mesh.triangle_bounds.d_cids);
    InfoStacklessBVH::QueryBuffer baseline_pt, baseline_ee, pts, ees;
    baseline_e.detect(mesh.d_contact_mask.view(), mesh.node(), mesh.ee(), baseline_ee);
    baseline_t.query(mesh.point_bounds.d_boxes.view(),
                     mesh.point_bounds.d_bids.view(),
                     mesh.point_bounds.d_cids.view(),
                     mesh.d_contact_mask.view(),
                     mesh.node(),
                     mesh.pt(),
                     baseline_pt);
    pts.m_pairs.release();
    ees.m_pairs.release();
    pts.reserve(1);
    ees.reserve(1);
    SingleEFBVHQuery query;
    query.set_backend(backend);
    query.max_chunk_edges = chunk;
    REQUIRE(query.detect(single_t,
                         mesh.pt(),
                         mesh.ee(),
                         mesh.node(),
                         mesh.point_bounds.view(),
                         mesh.edge_bounds.view(),
                         mesh.triangle_bounds.view(),
                         pts,
                         ees));
    check_backend_execution(query, true);
    auto actual_pt = read_pairs(pts, false);
    auto actual_ee = read_pairs(ees, true);
    REQUIRE(!actual_pt.empty());
    REQUIRE(!actual_ee.empty());
    CHECK(actual_pt == read_pairs(baseline_pt, false));
    CHECK(actual_ee == read_pairs(baseline_ee, true));
    CHECK(std::adjacent_find(actual_pt.begin(), actual_pt.end()) == actual_pt.end());
    CHECK(std::adjacent_find(actual_ee.begin(), actual_ee.end()) == actual_ee.end());
    if(mesh.triangles.size() > 64)
    {
        CHECK(std::count_if(actual_pt.begin(),
                            actual_pt.end(),
                            [](const Pair& pair) { return pair.first == 0; })
              > 64);
        CHECK(pts.m_pairs.capacity() > 1);
        CHECK(ees.m_pairs.capacity() > 1);
    }
    if(check_cpu)
    {
        auto [cpu_pt, cpu_ee] = mesh.all_pairs();
        CHECK(actual_pt == cpu_pt);
        CHECK(actual_ee == cpu_ee);
    }
    // Reuse cached topology with changed positions and fresh bounds.
    for(auto& p : mesh.positions)
        p.z() += 0.25;
    mesh.send();
    REQUIRE(query.detect(single_t,
                         mesh.pt(),
                         mesh.ee(),
                         mesh.node(),
                         mesh.point_bounds.view(),
                         mesh.edge_bounds.view(),
                         mesh.triangle_bounds.view(),
                         pts,
                         ees,
                         true));
    check_backend_execution(query, true);
    if(backend == Backend::Optix)
        CHECK(query.last_statistics().refits > 0);
    CHECK(read_pairs(pts, false) == actual_pt);
    CHECK(read_pairs(ees, true) == actual_ee);
}

void check_unsupported(Mesh& mesh, Backend backend)
{
    mesh.send();
    InfoStacklessBVH              triangle_bvh;
    InfoStacklessBVH::QueryBuffer pts, ees;
    Vector2i                      sentinel{17, 29};
    for(auto* out : {&pts, &ees})
    {
        out->reserve(1);
        out->m_pairs.view(0, 1).copy_from(&sentinel);
        out->m_size = 1;
        out->m_cpNum.copy_from(1);
    }
    SingleEFBVHQuery query;
    query.set_backend(backend);
    CHECK_FALSE(query.detect(triangle_bvh,
                             mesh.pt(),
                             mesh.ee(),
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees));
    check_backend_execution(query, false);
    CHECK(pts.size() == 1);
    CHECK(ees.size() == 1);
    CHECK(static_cast<int>(pts.m_cpNum) == 1);
    CHECK(static_cast<int>(ees.m_cpNum) == 1);
    const std::vector<Pair> expected{{17, 29}};
    CHECK(read_pairs(pts, false) == expected);
    CHECK(read_pairs(ees, false) == expected);
}
}  // namespace test_single_ef_bvh

TEST_CASE("single EF BVH preserves DCD candidates", "[cuda][single_ef_bvh]")
{
    using namespace test_single_ef_bvh;
    const auto backend = GENERATE_COPY(Catch::Generators::from_range(query_backends()));
    CAPTURE(static_cast<int>(backend));
    SECTION("surface rows and global IDs differ; one-edge chunks")
    {
        Mesh mesh(4);
        check_mesh(mesh, 1, backend);
    }
    SECTION("more than 64 nearby faces, retry and cross-chunk uniqueness")
    {
        Mesh mesh(80);
        mesh.send();
        SizeT overlaps = 0;
        for(const auto& box : mesh.triangle_bounds.boxes)
            overlaps += mesh.edge_bounds.boxes[0].intersects(box);
        REQUIRE(overlaps > 64);
        check_mesh(mesh, 2, backend);
    }
    SECTION("EF capacity overflow replays the complete query")
    {
        Mesh mesh(160);
        check_mesh(mesh, std::numeric_limits<SizeT>::max(), backend);
    }
    SECTION("contact mask")
    {
        Mesh mesh(4);
        mesh.contact_mask = {1, 0, 0, 1};
        check_mesh(mesh, 2, backend);
    }
    SECTION("subscene mask")
    {
        Mesh mesh(4);
        mesh.subscene_mask = {1, 0, 0, 1};
        check_mesh(mesh, 2, backend);
    }
    SECTION("different primitive expansion and activation thresholds")
    {
        Mesh mesh(5);
        for(IndexT t = 0; t < static_cast<IndexT>(mesh.triangles.size()); ++t)
            for(IndexT k = 0; k < 3; ++k)
            {
                IndexT v              = mesh.triangles[t][k];
                mesh.positions[v].z() = t * 0.015;
                mesh.dhats[v]         = t % 2 ? 0.04 : 0.003;
                mesh.thicknesses[v]   = t % 2 ? 0.005 : 0.001;
            }
        check_mesh(mesh, 1, backend);
    }
    SECTION("mixed contact IDs with all contacts allowed")
    {
        Mesh mesh(4);
        for(auto v : mesh.vertices)
            mesh.contacts[v] = v % 2;
        check_mesh(mesh, 2, backend);
    }
    SECTION("slanted edges overlap corner boxes without crossing them")
    {
        Mesh mesh(4);
        for(IndexT layer = 0; layer < 2; ++layer)
        {
            const auto f         = mesh.triangles[layer];
            Float      z         = layer * 0.002;
            mesh.positions[f[0]] = Vector3{0, 0, z};
            mesh.positions[f[1]] = Vector3{1, 0.4, z};
            mesh.positions[f[2]] = Vector3{0.25, 1, z};
        }
        for(IndexT layer = 2; layer < 4; ++layer)
        {
            const auto f         = mesh.triangles[layer];
            Float      z         = (layer - 2) * 0.002;
            mesh.positions[f[0]] = Vector3{0.05, 0.35, z};
            mesh.positions[f[1]] = Vector3{0.07, 0.35, z};
            mesh.positions[f[2]] = Vector3{0.05, 0.37, z};
        }
        std::fill(mesh.dhats.begin(), mesh.dhats.end(), 0.005);
        mesh.send();
        REQUIRE(mesh.edge_bounds.boxes[0].intersects(mesh.triangle_bounds.boxes[2]));
        check_mesh(mesh, 1, backend);
    }
    SECTION("body self collision disabled")
    {
        Mesh mesh(4);
        mesh.self = {0, 0};
        check_mesh(mesh, 2, backend);
    }
    SECTION("zero-length geometry with distinct vertex IDs")
    {
        Mesh mesh(3);
        mesh.positions[2] = mesh.positions[1];
        // Flagged degenerate predicates are compared to the existing GPU path.
        check_mesh(mesh, 1, backend, false);
    }
    SECTION("wire edge connecting surface vertices")
    {
        Mesh mesh(2);
        mesh.edges.emplace_back(1, 4);
        check_mesh(mesh, 1, backend);
    }
    SECTION("isolated particle near triangle interiors")
    {
        Mesh mesh(2);
        mesh.positions[0] = Vector3{0.2, 0.2, 0.025};
        mesh.vertices.push_back(0);
        check_mesh(mesh, 1, backend);
    }
    SECTION("particles and wire edges share the PT and EE proxy tree")
    {
        Mesh mesh(4);
        mesh.add_codimensional_geometry();
        check_mesh(mesh, 1, backend);
    }
    SECTION("mixed-dimensional EF capacity overflow and chunk replay")
    {
        Mesh mesh(160);
        mesh.add_codimensional_geometry();
        check_mesh(mesh, std::numeric_limits<SizeT>::max(), backend);
    }
    SECTION("mixed-dimensional contact, subscene and self-collision masks")
    {
        Mesh mesh(8);
        mesh.add_codimensional_geometry();
        mesh.contact_mask   = {1, 0, 0, 1};
        mesh.subscene_mask  = {1, 0, 0, 1};
        mesh.self           = {0, 1};
        const auto expected = mesh.all_pairs();
        REQUIRE(!expected.first.empty());
        REQUIRE(!expected.second.empty());
        check_mesh(mesh, 2, backend);
    }
    SECTION("pure codimensional topology does not publish partial output")
    {
        Mesh mesh(0);
        mesh.add_codimensional_geometry();
        check_unsupported(mesh, backend);
    }
    SECTION("duplicate vertex does not publish partial output")
    {
        Mesh mesh(2);
        mesh.vertices.push_back(mesh.vertices.front());
        check_unsupported(mesh, backend);
    }
    SECTION("missing triangle edge does not publish partial output")
    {
        Mesh mesh(2);
        mesh.edges.pop_back();
        check_unsupported(mesh, backend);
    }
    SECTION("empty surface does not publish partial output")
    {
        Mesh mesh(0);
        check_unsupported(mesh, backend);
    }
}

TEST_CASE("single EF BVH retains reusable candidate output storage", "[cuda][single_ef_bvh]")
{
    using namespace test_single_ef_bvh;
    const auto backend = GENERATE_COPY(Catch::Generators::from_range(query_backends()));
    CAPTURE(static_cast<int>(backend));
    Mesh mesh(8);
    for(IndexT layer = 2; layer < static_cast<IndexT>(mesh.triangles.size()); ++layer)
        for(IndexT vertex : mesh.triangles[layer])
            mesh.positions[vertex].z() = static_cast<Float>(layer);
    mesh.send();

    InfoStacklessBVH baseline_e, baseline_t, single_t;
    baseline_e.build(mesh.edge_bounds.d_boxes,
                     mesh.edge_bounds.d_bids,
                     mesh.edge_bounds.d_cids);
    baseline_t.build(mesh.triangle_bounds.d_boxes,
                     mesh.triangle_bounds.d_bids,
                     mesh.triangle_bounds.d_cids);
    InfoStacklessBVH::QueryBuffer baseline_pt, baseline_ee, pts, ees;
    baseline_e.detect(mesh.d_contact_mask.view(), mesh.node(), mesh.ee(), baseline_ee);
    baseline_t.query(mesh.point_bounds.d_boxes.view(),
                     mesh.point_bounds.d_bids.view(),
                     mesh.point_bounds.d_cids.view(),
                     mesh.d_contact_mask.view(),
                     mesh.node(),
                     mesh.pt(),
                     baseline_pt);
    const auto expected_pt = read_pairs(baseline_pt, false);
    const auto expected_ee = read_pairs(baseline_ee, true);
    const auto cpu         = mesh.all_pairs();
    REQUIRE(expected_pt == cpu.first);
    REQUIRE(expected_ee == cpu.second);
    constexpr SizeT reserved_slots = 64;
    REQUIRE(!expected_pt.empty());
    REQUIRE(!expected_ee.empty());
    REQUIRE(expected_pt.size() < reserved_slots);
    REQUIRE(expected_ee.size() < reserved_slots);
    for(auto* output : {&pts, &ees})
    {
        output->m_pairs.release();
        output->reserve(reserved_slots);
    }
    SingleEFBVHQuery query;
    query.set_backend(backend);
    query.max_chunk_edges = 2;

    auto check_dcd = [&](const std::vector<Pair>& expected_pts,
                         const std::vector<Pair>& expected_ees,
                         SizeT                    pt_storage_size,
                         SizeT                    ee_storage_size)
    {
        REQUIRE(query.detect(single_t,
                             mesh.pt(),
                             mesh.ee(),
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees));
        check_backend_execution(query, true);
        CHECK(pts.m_pairs.size() >= pt_storage_size);
        CHECK(ees.m_pairs.size() >= ee_storage_size);
        CHECK(pts.size() == expected_pts.size());
        CHECK(ees.size() == expected_ees.size());
        CHECK(pts.view().size() == expected_pts.size());
        CHECK(ees.view().size() == expected_ees.size());
        CHECK(static_cast<int>(pts.m_cpNum) == expected_pts.size());
        CHECK(static_cast<int>(ees.m_cpNum) == expected_ees.size());
        CHECK(read_pairs(pts, false) == expected_pts);
        CHECK(read_pairs(ees, true) == expected_ees);
    };

    check_dcd(expected_pt, expected_ee, reserved_slots, reserved_slots);
    check_dcd(expected_pt, expected_ee, reserved_slots, reserved_slots);
    mesh.contact_mask.assign(4, 0);
    mesh.send();
    check_dcd({}, {}, reserved_slots, reserved_slots);
    mesh.contact_mask.assign(4, 1);
    mesh.send();
    check_dcd(expected_pt, expected_ee, reserved_slots, reserved_slots);

    // CCD sweeps all layers to z=0 and grows the same PT/EE output buffers.
    std::vector<Vector3> displacements(mesh.positions.size(), Vector3::Zero());
    for(IndexT vertex : mesh.vertices)
        displacements[vertex].z() = -mesh.positions[vertex].z();
    upload(mesh.d_displacements, displacements);
    auto extend_endpoints = [&](Bounds& bounds, const auto& primitives)
    {
        for(SizeT row = 0; row < primitives.size(); ++row)
            for(IndexT vertex : primitives[row])
                bounds.boxes[row].extend(
                    (mesh.positions[vertex] + displacements[vertex]).template cast<float>());
        bounds.send();
    };
    for(SizeT row = 0; row < mesh.vertices.size(); ++row)
    {
        IndexT vertex = mesh.vertices[row];
        mesh.point_bounds.boxes[row].extend(
            (mesh.positions[vertex] + displacements[vertex]).template cast<float>());
    }
    mesh.point_bounds.send();
    extend_endpoints(mesh.edge_bounds, mesh.edges);
    extend_endpoints(mesh.triangle_bounds, mesh.triangles);
    auto ccd_pt  = mesh.pt();
    auto ccd_ee  = mesh.ee();
    ccd_pt.alpha = 1.0;
    ccd_ee.alpha = 1.0;
    baseline_e.build(mesh.edge_bounds.d_boxes,
                     mesh.edge_bounds.d_bids,
                     mesh.edge_bounds.d_cids);
    baseline_t.build(mesh.triangle_bounds.d_boxes,
                     mesh.triangle_bounds.d_bids,
                     mesh.triangle_bounds.d_cids);
    baseline_e.detect(mesh.d_contact_mask.view(), mesh.node(), ccd_ee, ees);
    baseline_t.query(mesh.point_bounds.d_boxes.view(),
                     mesh.point_bounds.d_bids.view(),
                     mesh.point_bounds.d_cids.view(),
                     mesh.d_contact_mask.view(),
                     mesh.node(),
                     ccd_pt,
                     pts);
    REQUIRE(pts.size() > reserved_slots);
    REQUIRE(ees.size() > reserved_slots);
    const SizeT ccd_pt_storage = pts.m_pairs.size();
    const SizeT ccd_ee_storage = ees.m_pairs.size();
    REQUIRE(ccd_pt_storage >= pts.size());
    REQUIRE(ccd_ee_storage >= ees.size());

    // Reset motion and bounds, retaining the CCD-grown logical writable range.
    mesh.send();
    check_dcd(expected_pt, expected_ee, ccd_pt_storage, ccd_ee_storage);
    mesh.contact_mask.assign(4, 0);
    mesh.send();
    check_dcd({}, {}, ccd_pt_storage, ccd_ee_storage);
    mesh.contact_mask.assign(4, 1);
    mesh.send();
    check_dcd(expected_pt, expected_ee, ccd_pt_storage, ccd_ee_storage);

    // Grow from tiny outputs across chunks and retain every earlier chunk's pairs.
    for(IndexT layer = 0; layer < static_cast<IndexT>(mesh.triangles.size()); ++layer)
        for(IndexT vertex : mesh.triangles[layer])
            mesh.positions[vertex].z() = layer * 0.0005;
    mesh.send();
    const auto dense_expected = mesh.all_pairs();
    for(auto* output : {&pts, &ees})
    {
        output->m_pairs.release();
        output->reserve(1);
    }
    REQUIRE(dense_expected.first.size() > 1);
    REQUIRE(dense_expected.second.size() > 1);
    check_dcd(dense_expected.first, dense_expected.second, 1, 1);
    const SizeT grown_pt_storage = pts.m_pairs.size();
    const SizeT grown_ee_storage = ees.m_pairs.size();
    CHECK(grown_pt_storage >= dense_expected.first.size());
    CHECK(grown_ee_storage >= dense_expected.second.size());
    check_dcd(dense_expected.first, dense_expected.second, grown_pt_storage, grown_ee_storage);
}

namespace test_single_ef_bvh
{
void check_swept_candidates(Mesh&                       mesh,
                            const std::vector<Vector3>& motion,
                            const std::vector<Float>&   alphas,
                            Backend                     backend)
{
    InfoStacklessBVH              baseline_e, baseline_t, single_t;
    InfoStacklessBVH::QueryBuffer baseline_pt, baseline_ee, pts, ees;
    for(auto* output : {&pts, &ees})
    {
        output->m_pairs.release();
        output->reserve(1);
    }
    SingleEFBVHQuery query;
    query.set_backend(backend);
    query.max_chunk_edges = 1;
    bool saw_swept_pt = false, saw_swept_ee = false;
    auto compare = [&](const std::vector<Vector3>& dxs, Float alpha)
    {
        CAPTURE(alpha);
        mesh.send(dxs, alpha);
        auto pt_pred  = mesh.pt();
        auto ee_pred  = mesh.ee();
        pt_pred.alpha = alpha;
        ee_pred.alpha = alpha;
        baseline_e.build(mesh.edge_bounds.d_boxes,
                         mesh.edge_bounds.d_bids,
                         mesh.edge_bounds.d_cids);
        baseline_t.build(mesh.triangle_bounds.d_boxes,
                         mesh.triangle_bounds.d_bids,
                         mesh.triangle_bounds.d_cids);
        baseline_e.detect(mesh.d_contact_mask.view(), mesh.node(), ee_pred, baseline_ee);
        baseline_t.query(mesh.point_bounds.d_boxes.view(),
                         mesh.point_bounds.d_bids.view(),
                         mesh.point_bounds.d_cids.view(),
                         mesh.d_contact_mask.view(),
                         mesh.node(),
                         pt_pred,
                         baseline_pt);
        const SizeT previous_pt_storage = pts.m_pairs.size();
        const SizeT previous_ee_storage = ees.m_pairs.size();
        REQUIRE(query.detect(single_t,
                             pt_pred,
                             ee_pred,
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees,
                             true));
        check_backend_execution(query, true);
        const auto expected_pt = read_pairs(baseline_pt, false);
        const auto expected_ee = read_pairs(baseline_ee, true);
        const auto actual_pt   = read_pairs(pts, false);
        const auto actual_ee   = read_pairs(ees, true);
        CHECK(actual_pt == expected_pt);
        CHECK(actual_ee == expected_ee);
        for(const auto* pairs : {&expected_pt, &expected_ee, &actual_pt, &actual_ee})
            CHECK(std::adjacent_find(pairs->begin(), pairs->end()) == pairs->end());
        CHECK(pts.view().size() == expected_pt.size());
        CHECK(ees.view().size() == expected_ee.size());
        CHECK(static_cast<int>(pts.m_cpNum) == expected_pt.size());
        CHECK(static_cast<int>(ees.m_cpNum) == expected_ee.size());
        CHECK(pts.m_pairs.size() >= previous_pt_storage);
        CHECK(ees.m_pairs.size() >= previous_ee_storage);
        if(alpha > 0.0)
        {
            saw_swept_pt |= !expected_pt.empty();
            saw_swept_ee |= !expected_ee.empty();
        }
    };
    REQUIRE(!alphas.empty());
    for(Float alpha : alphas)
    {
        REQUIRE(alpha > 0.0);
        compare(motion, alpha);
    }
    CHECK(saw_swept_pt);
    CHECK(saw_swept_ee);
    for(SizeT vertex = 0; vertex < mesh.positions.size(); ++vertex)
        mesh.positions[vertex] += alphas.back() * motion[vertex];
    compare({}, 0.0);
}
}  // namespace test_single_ef_bvh

TEST_CASE("single EF BVH preserves swept CCD candidate multisets", "[cuda][single_ef_bvh]")
{
    using namespace test_single_ef_bvh;
    const auto backend = GENERATE_COPY(Catch::Generators::from_range(query_backends()));
    CAPTURE(static_cast<int>(backend));
    SECTION("particles and rods cross triangle surfaces during the sweep")
    {
        Mesh       mesh(2);
        const auto first_codim = mesh.positions.size();
        mesh.add_codimensional_geometry();
        std::vector<Vector3> motion(mesh.positions.size(), Vector3::Zero());
        for(SizeT v = first_codim; v < mesh.positions.size(); ++v)
        {
            mesh.positions[v].z() += 1.0;
            motion[v] = Vector3{0, 0, -2};
        }
        check_swept_candidates(mesh, motion, {0.25, 0.5, 1.0}, backend);
    }
    SECTION("deforming wires and a moving isolated particle")
    {
        Mesh       mesh(3);
        const auto first_codim = mesh.positions.size();
        mesh.add_codimensional_geometry();
        std::vector<Vector3> motion(mesh.positions.size(), Vector3::Zero());
        for(SizeT v = first_codim; v < mesh.positions.size(); ++v)
            motion[v] = Vector3{v % 2 ? 0.1 : -0.1, 0.05, v % 2 ? 0.5 : -0.5};
        check_swept_candidates(mesh, motion, {0.3, 0.8, 1.0}, backend);
    }
    SECTION("intermediate crossing without initial or final DCD contact")
    {
        Mesh mesh(2);
        for(IndexT vertex : mesh.triangles[1])
            mesh.positions[vertex].z() = 1.0;
        const auto           start = mesh.positions;
        std::vector<Vector3> motion(mesh.positions.size(), Vector3::Zero());
        for(IndexT vertex : mesh.triangles[1])
            motion[vertex].z() = -2.0;
        const auto initial = mesh.all_pairs();
        REQUIRE(initial.first.empty());
        REQUIRE(initial.second.empty());
        for(SizeT vertex = 0; vertex < mesh.positions.size(); ++vertex)
            mesh.positions[vertex] = start[vertex] + motion[vertex];
        const auto final = mesh.all_pairs();
        REQUIRE(final.first.empty());
        REQUIRE(final.second.empty());
        for(SizeT vertex = 0; vertex < mesh.positions.size(); ++vertex)
            mesh.positions[vertex] = start[vertex] + 0.5 * motion[vertex];
        REQUIRE(!mesh.all_pairs().first.empty());
        mesh.positions = start;
        check_swept_candidates(mesh, motion, {0.25, 0.75, 1.0}, backend);
    }
    SECTION("different endpoint velocities, rotating and deforming triangles")
    {
        Mesh                 mesh(3);
        std::vector<Vector3> motion(mesh.positions.size(), Vector3::Zero());
        for(IndexT vertex : mesh.triangles[1])
        {
            const auto& p  = mesh.positions[vertex];
            motion[vertex] = Vector3{1.0 - p.y(), p.x(), p.z() + 0.2} - p;
        }
        motion[mesh.triangles[2][0]] = Vector3{-0.2, 0.1, -0.3};
        motion[mesh.triangles[2][1]] = Vector3{0.1, -0.2, 0.2};
        motion[mesh.triangles[2][2]] = Vector3{0.3, 0.2, -0.1};
        for(IndexT layer = 0; layer < 3; ++layer)
            for(IndexT vertex : mesh.triangles[layer])
            {
                mesh.dhats[vertex]       = 0.03 + 0.02 * layer;
                mesh.thicknesses[vertex] = 0.001 * (layer + 1);
            }
        check_swept_candidates(mesh, motion, {0.125, 0.5, 1.0}, backend);
    }
    SECTION("positive alpha with zero displacement")
    {
        Mesh                 mesh(3);
        std::vector<Vector3> motion(mesh.positions.size(), Vector3::Zero());
        check_swept_candidates(mesh, motion, {0.25, 1.0}, backend);
    }
    SECTION("contact masks, subscenes and disabled body self collision")
    {
        Mesh mesh(4);
        mesh.self          = {0, 0};
        mesh.contact_mask  = {0, 1, 1, 0};
        mesh.subscene_mask = {1, 0, 0, 1};
        std::vector<Vector3> motion(mesh.positions.size(), Vector3::Zero());
        for(IndexT vertex : mesh.vertices)
            motion[vertex] = Vector3{0.05 * mesh.bodies[vertex], 0, -0.02};
        check_swept_candidates(mesh, motion, {0.5, 1.0}, backend);
    }
    SECTION("a collapsed edge opens during the sweep")
    {
        Mesh mesh(3);
        mesh.positions[2] = mesh.positions[1];
        std::vector<Vector3> motion(mesh.positions.size(), Vector3::Zero());
        motion[2] = Vector3{0.2, 0, 0.4};
        check_swept_candidates(mesh, motion, {0.5, 1.0}, backend);
    }
}

namespace test_single_ef_bvh
{
struct SegmentSlabCase
{
    Vector3 from, to;
    AABB    box;
    Float   radius;
    IndexT  expected;
};

__global__ void segment_slab_cases_kernel(ct::CBufferView<SegmentSlabCase> cases,
                                          ct::BufferView<IndexT> result)
{
    const int index = blockIdx.x * blockDim.x + threadIdx.x;
    if(index >= cases.size())
        return;
    const auto& c     = cases(index);
    result(2 * index) = segment_aabb_intersects(c.from, c.to, c.box, c.radius);
    result(2 * index + 1) = segment_aabb_intersects(c.to, c.from, c.box, c.radius);
}
}  // namespace test_single_ef_bvh

TEST_CASE("segment AABB slab test handles closed GPU boundaries", "[cuda][single_ef_bvh]")
{
    using namespace test_single_ef_bvh;
    AABB unit_box;
    unit_box.extend(Vector3::Zero().cast<float>()).extend(Vector3::Ones().cast<float>());
    std::vector<SegmentSlabCase> cases;
    auto add = [&](Vector3 from, Vector3 to, Float radius, IndexT expected, const AABB& box)
    { cases.push_back({from, to, box, radius, expected}); };
    // Both per-axis ranges overlap the unit box, but y=x+1.75 misses it.
    add({-1, 0.75, 0.5}, {0.75, 2.5, 0.5}, 0, 0, unit_box);
    add({-1, -1, -1}, {2, 2, 2}, 0, 1, unit_box);
    add({-1, 1, 0.5}, {2, 1, 0.5}, 0, 1, unit_box);
    add({-1, 1.01, 0.5}, {2, 1.01, 0.5}, 0, 0, unit_box);
    add({0.5, 0.5, 0.5}, {0.5, 0.5, 0.5}, 0, 1, unit_box);
    add({2, 0.5, 0.5}, {2, 0.5, 0.5}, 0, 0, unit_box);
    add({-1, 1.25, 0.5}, {2, 1.25, 0.5}, 0.25, 1, unit_box);
    add({-1, 1.251, 0.5}, {2, 1.251, 0.5}, 0.25, 0, unit_box);
    constexpr Float offset = 1e8;
    AABB            large_box;
    large_box.extend(Vector3::Constant(offset).cast<float>())
        .extend(Vector3::Constant(offset + 32).cast<float>());
    add({offset - 64, offset + 16, offset + 16}, {offset + 64, offset + 16, offset + 16}, 0, 1, large_box);
    add({offset - 64, offset + 64, offset + 16}, {offset + 64, offset + 64, offset + 16}, 0, 0, large_box);

    // Expanding this small box by a huge radius must round outward in double.
    constexpr Float huge_radius = 0x1p70;
    AABB            cancellation_box;
    cancellation_box.extend(Vector3{65536.0234375, 98304, 0}.cast<float>());
    add({-huge_radius, huge_radius, 0},
        {-huge_radius + 262144, huge_radius + 262144, 0},
        huge_radius,
        1,
        cancellation_box);

    ct::DeviceBuffer<SegmentSlabCase> d_cases;
    ct::DeviceBuffer<IndexT>          d_result;
    upload(d_cases, cases);
    d_result.resize(cases.size() * 2);
    segment_slab_cases_kernel<<<1, 32>>>(d_cases.view(), d_result.view());
    REQUIRE(cudaGetLastError() == cudaSuccess);
    std::vector<IndexT> result(cases.size() * 2);
    d_result.view().copy_to(result.data());
    for(SizeT index = 0; index < cases.size(); ++index)
    {
        CAPTURE(index);
        CHECK(result[index * 2] == cases[index].expected);
        CHECK(result[index * 2 + 1] == cases[index].expected);
    }
}

TEST_CASE("single EF slab pruning preserves EE hits after float cancellation", "[cuda][single_ef_bvh]")
{
    using namespace test_single_ef_bvh;
    const auto backend = GENERATE_COPY(Catch::Generators::from_range(query_backends()));
    CAPTURE(static_cast<int>(backend));
    Mesh mesh(2);
    mesh.positions[1] = Vector3{1.00000008, 0, 0};
    mesh.positions[2] = Vector3{1.00000008, 0, 1e-4};
    mesh.positions[3] = Vector3{1.25, 0, 0};
    mesh.positions[4] = Vector3{1e-7, 0, 0};
    mesh.positions[5] = Vector3{1.22e-7, 2, 0};
    mesh.positions[6] = Vector3{-0.5, 2, 0.25};
    std::fill(mesh.dhats.begin(), mesh.dhats.end(), 1e-12);
    for(IndexT vertex : mesh.triangles[0])
        mesh.thicknesses[vertex] = 1.0;
    for(const auto& face : mesh.triangles)
    {
        const Vector3 a = mesh.positions[face[1]] - mesh.positions[face[0]];
        const Vector3 b = mesh.positions[face[2]] - mesh.positions[face[0]];
        REQUIRE(a.cross(b).squaredNorm() > 0.0);
    }
    mesh.send();
    const Pair  witness{0, 3};
    const auto& target = mesh.triangle_bounds.boxes[0];
    REQUIRE(target.min().x() > mesh.positions[4].x());
    REQUIRE(mesh.edge_bounds.boxes[3].intersects(mesh.edge_bounds.boxes[0]));
    REQUIRE_FALSE(segment_aabb_intersects(mesh.positions[4], mesh.positions[5], target, 1e-12));
    for(IndexT vertex : mesh.edges[3])
    {
        const auto row = std::find(mesh.vertices.begin(), mesh.vertices.end(), vertex);
        REQUIRE(row != mesh.vertices.end());
        REQUIRE_FALSE(mesh.point_bounds.boxes[row - mesh.vertices.begin()].intersects(target));
    }

    const auto cpu = mesh.all_pairs();
    REQUIRE(std::binary_search(cpu.second.begin(), cpu.second.end(), witness));
    InfoStacklessBVH              baseline_e;
    InfoStacklessBVH::QueryBuffer baseline_ee;
    baseline_e.build(mesh.edge_bounds.d_boxes,
                     mesh.edge_bounds.d_bids,
                     mesh.edge_bounds.d_cids);
    baseline_e.detect(mesh.d_contact_mask.view(), mesh.node(), mesh.ee(), baseline_ee);
    const auto expected_ee = read_pairs(baseline_ee, true);
    REQUIRE(std::binary_search(expected_ee.begin(), expected_ee.end(), witness));
    // Float point bounds can reject a CPU PT hit here; use native bounds as the full oracle.
    check_mesh(mesh, 1, backend, false);
}

TEST_CASE("OptiX EF traversal preserves closed AABB overlap", "[cuda][single_ef_bvh][optix]")
{
    using namespace test_single_ef_bvh;
    if(!OptixEFBVH::compiled())
        SKIP("OptiX support is disabled in this build");
    const auto group_size = GENERATE(SizeT{1}, OptixEFBVH::default_group_size);
    CAPTURE(group_size);

    auto box = [](Vector3 lower, Vector3 upper)
    {
        AABB result;
        result.extend(lower.cast<float>()).extend(upper.cast<float>());
        return result;
    };
    const Float       after_one = std::nextafter(1.0f, 2.0f);
    constexpr Float   tiny = 1e-20, huge = 1e20;
    std::vector<AABB> leaves  = {box({0, 0, 0}, {0, 0, 0}),
                                 box({1, 1, 1}, {1, 1, 1}),
                                 box({after_one, 0, 0}, {2, 1, 1}),
                                 box({huge, huge, huge}, {huge, huge, huge}),
                                 box({tiny, -tiny, -tiny}, {2 * tiny, tiny, tiny})};
    std::vector<AABB> queries = {box({0, 0, 0}, {0, 0, 0}),
                                 box({0, 0, 0}, {1, 1, 1}),
                                 box({after_one, 1, 1}, {after_one, 1, 1}),
                                 box({huge, huge, huge}, {huge, huge, huge}),
                                 box({tiny, 0, 0}, {2 * tiny, 0, 0}),
                                 box({-1, -1, -1}, {-0.5, -0.5, -0.5}),
                                 box({1, 0, 0}, {1, 1, 1}),
                                 box({-huge, -huge, -huge}, {huge, huge, huge})};

    Mesh mesh(1);
    mesh.send();
    ct::DeviceBuffer<AABB>   d_queries, d_leaves;
    ct::DeviceBuffer<IndexT> query_tags, leaf_tags;
    upload(query_tags, std::vector<IndexT>(queries.size(), 0));
    upload(leaf_tags, std::vector<IndexT>(leaves.size(), 0));
    OptixEFBVH tree;
    tree.set_group_size(group_size);
    InfoStacklessBVH::QueryBuffer output;
    for(int pass = 0; pass < 3; ++pass)
    {
        CAPTURE(pass);
        upload(d_queries, queries);
        upload(d_leaves, leaves);
        tree.update(d_queries, query_tags, query_tags, d_leaves, leaf_tags, leaf_tags, pass == 1);
        REQUIRE_THROWS_AS(tree.launch_query(0, OptixEFBVH::max_query_count + 1, mesh.node(), output),
                          std::overflow_error);
        constexpr SizeT chunk_size = 3;
        for(SizeT start = 0; start < queries.size(); start += chunk_size)
        {
            CAPTURE(start);
            const auto count = std::min(chunk_size, queries.size() - start);
            output.m_pairs.resize_discard(count * leaves.size());
            tree.launch_query(start, count, mesh.node(), output);
            const int actual_count = static_cast<int>(output.m_cpNum);
            REQUIRE(actual_count >= 0);
            REQUIRE(static_cast<SizeT>(actual_count) <= output.m_pairs.size());
            output.m_size = actual_count;
            std::vector<Pair> expected;
            for(SizeT query = 0; query < count; ++query)
                for(SizeT leaf = 0; leaf < leaves.size(); ++leaf)
                    if(queries[start + query].intersects(leaves[leaf]))
                        expected.emplace_back(query, leaf);
            const auto actual = read_pairs(output, false);
            CHECK(actual == expected);
            CHECK(std::adjacent_find(actual.begin(), actual.end()) == actual.end());
        }
        for(auto* boxes : {&queries, &leaves})
            for(auto& bound : *boxes)
            {
                bound.min().array() += 0.125f;
                bound.max().array() += 0.125f;
            }
    }
    CHECK(tree.statistics().builds == 2);
    CHECK(tree.statistics().refits == 1);
    CHECK(tree.statistics().launches == 9);
}

namespace test_single_ef_bvh
{
struct GroupedEFCase
{
    static constexpr SizeT tag_count = 12;
    std::vector<AABB>      queries, leaves;
    std::vector<IndexT>    query_bids, query_cids, leaf_bids, leaf_cids;
    std::vector<IndexT>    self = std::vector<IndexT>(tag_count, 1);
    std::vector<IndexT>    mask = std::vector<IndexT>(tag_count * tag_count, 1);
    ct::DeviceBuffer<AABB> d_queries, d_leaves;
    ct::DeviceBuffer<IndexT> d_query_bids, d_query_cids, d_leaf_bids, d_leaf_cids, d_self;
    ct::DeviceBuffer2D<IndexT>    d_mask{ct::Extent2D{tag_count, tag_count}};
    OptixEFBVH                    tree;
    InfoStacklessBVH::QueryBuffer output;

    GroupedEFCase(SizeT leaf_count, SizeT query_count)
        : query_bids(query_count, 0)
        , query_cids(query_count, 0)
        , leaf_bids(leaf_count, 0)
        , leaf_cids(leaf_count, 0)
    {
        AABB box;
        box.extend(Vector3::Zero().cast<float>()).extend(Vector3::Ones().cast<float>());
        queries.assign(query_count, box);
        leaves.assign(leaf_count, box);
        tree.enable_culling_statistics(true);
    }

    void send_masks()
    {
        upload(d_self, self);
        d_mask.view().copy_from(mask.data());
    }

    void update(bool reuse)
    {
        upload(d_queries, queries);
        upload(d_leaves, leaves);
        upload(d_query_bids, query_bids);
        upload(d_query_cids, query_cids);
        upload(d_leaf_bids, leaf_bids);
        upload(d_leaf_cids, leaf_cids);
        send_masks();
        tree.update(d_queries, d_query_bids, d_query_cids, d_leaves, d_leaf_bids, d_leaf_cids, reuse);
    }

    void check_query(SizeT start,
                     SizeT count,
                     SizeT capacity = std::numeric_limits<SizeT>::max())
    {
        std::vector<Pair> expected;
        for(SizeT local = 0; local < count; ++local)
            for(SizeT leaf = 0; leaf < leaves.size(); ++leaf)
            {
                const auto query = start + local;
                const auto qb = query_bids[query], qc = query_cids[query];
                const auto lb = leaf_bids[leaf], lc = leaf_cids[leaf];
                const bool body_reject = qb >= 0 && lb >= 0 && qb == lb && !self[qb];
                const bool contact_reject =
                    qc >= 0 && lc >= 0 && !mask[qc * tag_count + lc];
                if(!body_reject && !contact_reject
                   && queries[query].intersects(leaves[leaf]))
                    expected.emplace_back(local, leaf);
            }
        capacity = std::min(capacity, count * leaves.size());
        output.m_pairs.resize_discard(capacity);
        const bvh_simplex_detail::NodePredicate predicate{d_self.view(), d_mask.cviewer()};
        tree.launch_query(start, count, predicate, output);
        const int required = static_cast<int>(output.m_cpNum);
        REQUIRE(required >= 0);
        REQUIRE(static_cast<SizeT>(required) == expected.size());
        if(static_cast<SizeT>(required) > capacity)
        {
            output.m_pairs.resize_discard(required);
            tree.launch_query(start, count, predicate, output);
            REQUIRE(static_cast<int>(output.m_cpNum) == required);
        }
        output.m_size     = required;
        const auto actual = read_pairs(output, false);
        CHECK(actual == expected);
        CHECK(std::adjacent_find(actual.begin(), actual.end()) == actual.end());
    }
};
}  // namespace test_single_ef_bvh

TEST_CASE("OptiX EF groups reject metadata before scanning children", "[cuda][single_ef_bvh][optix]")
{
    using namespace test_single_ef_bvh;
    if(!OptixEFBVH::compiled())
        SKIP("OptiX support is disabled in this build");

    SECTION("disabled body skips every child; changing its mask needs no GAS update")
    {
        GroupedEFCase test(48, 4);
        test.self[0] = 0;
        test.update(false);
        test.check_query(1, 2);
        const auto rejected = test.tree.culling_statistics();
        REQUIRE(rejected.group_tests > 0);
        REQUIRE(rejected.body_rejections > 0);
        REQUIRE(rejected.skipped_leaves >= 96);
        REQUIRE(rejected.leaf_tests == 0);
        REQUIRE(rejected.emitted_pairs == 0);
        test.self[0] = 1;
        test.send_masks();
        test.check_query(1, 2, 1);
        const auto allowed = test.tree.culling_statistics();
        CHECK(allowed.body_rejections == rejected.body_rejections);
        CHECK(allowed.leaf_tests > 0);
        CHECK(allowed.emitted_pairs >= 96);
        CHECK(test.tree.statistics().builds == 1);
        CHECK(test.tree.statistics().refits == 0);
    }
    SECTION("contact IDs above eight reject groups and track live mask changes")
    {
        GroupedEFCase test(32, 3);
        std::fill(test.query_cids.begin(), test.query_cids.end(), 9);
        std::fill(test.leaf_bids.begin(), test.leaf_bids.end(), 1);
        std::fill(test.leaf_cids.begin(), test.leaf_cids.end(), 11);
        test.mask[9 * test.tag_count + 11] = 0;
        test.update(false);
        test.check_query(0, 3);
        const auto rejected = test.tree.culling_statistics();
        REQUIRE(rejected.contact_rejections > 0);
        REQUIRE(rejected.body_rejections == 0);
        REQUIRE(rejected.skipped_leaves >= 96);
        REQUIRE(rejected.leaf_tests == 0);
        REQUIRE(rejected.emitted_pairs == 0);
        test.mask[9 * test.tag_count + 11] = 1;
        test.send_masks();
        test.check_query(2, 1);
        const auto allowed = test.tree.culling_statistics();
        CHECK(allowed.contact_rejections == rejected.contact_rejections);
        CHECK(allowed.leaf_tests > 0);
        CHECK(allowed.emitted_pairs == 32);
        CHECK(test.tree.statistics().refits == 0);
    }
    SECTION("twelve shuffled tag groups preserve original IDs across grouping modes")
    {
        GroupedEFCase test(96, 4);
        test.query_bids = test.query_cids = {0, 7, 9, 11};
        std::fill(test.self.begin(), test.self.end(), 0);
        for(SizeT leaf = 0; leaf < test.leaves.size(); ++leaf)
            test.leaf_bids[leaf] = test.leaf_cids[leaf] = (leaf * 5) % test.tag_count;
        for(SizeT row = 0; row < test.tag_count; ++row)
            for(SizeT column = 0; column < test.tag_count; ++column)
                test.mask[row * test.tag_count + column] = (row + column) % 3 != 0;
        for(SizeT group_size : {SizeT{1}, SizeT{8}, SizeT{16}, SizeT{64}, SizeT{1}})
        {
            CAPTURE(group_size);
            test.tree.set_group_size(group_size);
            CHECK(test.tree.group_size() == group_size);
            test.update(true);
            test.check_query(0, 2, 1);
            test.check_query(2, 2, 1);
            const auto stats = test.tree.culling_statistics();
            CHECK(stats.emitted_pairs > 0);
            if(group_size == 1)
            {
                CHECK(stats.group_tests == 0);
                CHECK(stats.body_rejections == 0);
                CHECK(stats.contact_rejections == 0);
                CHECK(stats.skipped_leaves == 0);
            }
            else
            {
                CHECK(test.tree.group_count() >= test.tag_count);
                CHECK(stats.group_tests > 0);
                CHECK(stats.body_rejections > 0);
                CHECK(stats.contact_rejections > 0);
            }
        }
    }
    SECTION("refit recomputes mixed and invalid aggregate tags")
    {
        GroupedEFCase test(48, 2);
        test.tree.set_group_size(OptixEFBVH::max_group_size);
        test.self[0] = 0;
        test.update(false);
        REQUIRE(test.tree.group_count() == 1);
        test.check_query(0, 2);
        REQUIRE(test.tree.culling_statistics().leaf_tests == 0);
        for(SizeT leaf = 0; leaf < test.leaves.size(); ++leaf)
            test.leaf_bids[leaf] = test.leaf_cids[leaf] = leaf % 2;
        test.update(true);
        test.check_query(0, 2);
        const auto mixed = test.tree.culling_statistics();
        CHECK(mixed.body_rejections == 0);
        CHECK(mixed.contact_rejections == 0);
        CHECK(mixed.leaf_tests > 0);
        CHECK(mixed.emitted_pairs == 48);
        for(SizeT leaf = 0; leaf < test.leaves.size(); ++leaf)
            test.leaf_bids[leaf] = test.leaf_cids[leaf] = leaf % 2 ? -1 : 0;
        std::fill(test.mask.begin(), test.mask.end(), 0);
        test.update(true);
        test.check_query(0, 2);
        const auto partly_invalid = test.tree.culling_statistics();
        CHECK(partly_invalid.body_rejections == 0);
        CHECK(partly_invalid.contact_rejections == 0);
        CHECK(partly_invalid.leaf_tests > 0);
        CHECK(partly_invalid.emitted_pairs == 48);
        std::fill(test.query_bids.begin(), test.query_bids.end(), -1);
        std::fill(test.query_cids.begin(), test.query_cids.end(), -1);
        std::fill(test.leaf_bids.begin(), test.leaf_bids.end(), -1);
        std::fill(test.leaf_cids.begin(), test.leaf_cids.end(), -1);
        std::fill(test.self.begin(), test.self.end(), 0);
        std::fill(test.mask.begin(), test.mask.end(), 0);
        test.update(true);
        test.check_query(0, 2);
        const auto invalid = test.tree.culling_statistics();
        CHECK(invalid.body_rejections == 0);
        CHECK(invalid.contact_rejections == 0);
        CHECK(invalid.leaf_tests > 0);
        CHECK(invalid.emitted_pairs == 96);
        CHECK(test.tree.statistics().refits == 3);
    }
}

TEST_CASE("single EF fixed-state native and OptiX timings", "[.][single_ef_optix_benchmark]")
{
    using namespace test_single_ef_bvh;
    if(!OptixEFBVH::compiled())
        SKIP("OptiX support is disabled in this build");

    Mesh             mesh(0);
    constexpr IndexT side    = 32;
    constexpr Float  spacing = 0.04, gap = 0.001;
    std::set<Pair>   unique_edges;
    for(IndexT layer = 0; layer < 2; ++layer)
    {
        const auto base = static_cast<IndexT>(mesh.positions.size());
        for(IndexT row = 0; row < side; ++row)
            for(IndexT column = 0; column < side; ++column)
            {
                mesh.vertices.push_back(static_cast<IndexT>(mesh.positions.size()));
                mesh.positions.emplace_back(column * spacing, row * spacing, layer * gap);
                mesh.bodies.push_back(layer);
                mesh.contacts.push_back(0);
                mesh.subscenes.push_back(0);
            }
        for(IndexT row = 0; row < side - 1; ++row)
            for(IndexT column = 0; column < side - 1; ++column)
            {
                const IndexT a = base + row * side + column;
                for(const Vector3i& face : {Vector3i{a, a + 1, a + side},
                                            Vector3i{a + 1, a + side + 1, a + side}})
                {
                    mesh.triangles.push_back(face);
                    for(int edge = 0; edge < 3; ++edge)
                    {
                        const auto first = face[edge], second = face[(edge + 1) % 3];
                        unique_edges.emplace(std::min(first, second),
                                             std::max(first, second));
                    }
                }
            }
    }
    for(const auto& [first, second] : unique_edges)
        mesh.edges.emplace_back(first, second);
    mesh.radius = 0.003;
    mesh.dhats.assign(mesh.positions.size(), mesh.radius);
    mesh.thicknesses.assign(mesh.positions.size(), 0.0);
    mesh.self = {0, 0};
    std::vector<Vector3> motion(mesh.positions.size(), Vector3::Zero());
    for(IndexT vertex : mesh.vertices)
        if(mesh.bodies[vertex] == 1)
            motion[vertex] = Vector3{0.005, 0, -2 * gap};

    constexpr int calls = 20, repeats = 5;
    for(Float alpha : {0.0, 1.0})
    {
        // Primitive bounds and motion stay unchanged throughout each timed scope.
        mesh.send(motion, alpha);
        auto pt_pred  = mesh.pt();
        auto ee_pred  = mesh.ee();
        pt_pred.alpha = ee_pred.alpha = alpha;
        for(bool reuse : {false, true})
        {
            const std::string scope = std::string{alpha == 0.0 ? "dcd" : "ccd"}
                                      + (reuse ? "_refit_query" : "_build_query");
            CAPTURE(scope);
            constexpr std::array<const char*, 5> methods = {"single_ef_bvh",
                                                            "single_ef_optix_flat",
                                                            "single_ef_optix_grouped8",
                                                            "single_ef_optix_grouped16",
                                                            "single_ef_optix_grouped32"};
            constexpr std::array<SizeT, 5>  group_sizes  = {0, 1, 8, 16, 32};
            std::array<SingleEFBVHQuery, 5> queries;
            std::array<InfoStacklessBVH, 5> trees;
            std::array<InfoStacklessBVH::QueryBuffer, 5> pts, ees;
            for(SizeT backend = 1; backend < queries.size(); ++backend)
            {
                queries[backend].set_backend(Backend::Optix);
                queries[backend].set_optix_group_size(group_sizes[backend]);
                queries[backend].enable_optix_culling_statistics(false);
            }
            auto detect = [&](SizeT backend)
            {
                return queries[backend].detect(trees[backend],
                                               pt_pred,
                                               ee_pred,
                                               mesh.node(),
                                               mesh.point_bounds.view(),
                                               mesh.edge_bounds.view(),
                                               mesh.triangle_bounds.view(),
                                               pts[backend],
                                               ees[backend],
                                               reuse);
            };
            for(SizeT backend = 0; backend < queries.size(); ++backend)
                for(int warmup = 0; warmup < 3; ++warmup)
                    REQUIRE(detect(backend));
            const auto expected_pt = read_pairs(pts[0], false);
            const auto expected_ee = read_pairs(ees[0], true);
            REQUIRE_FALSE(expected_pt.empty());
            REQUIRE_FALSE(expected_ee.empty());
            for(SizeT backend = 1; backend < queries.size(); ++backend)
            {
                REQUIRE(read_pairs(pts[backend], false) == expected_pt);
                REQUIRE(read_pairs(ees[backend], true) == expected_ee);
                REQUIRE_FALSE(queries[backend].optix_culling_statistics_enabled());
            }
            // Keep diagnostic atomics and their readback outside all timed samples.
            for(SizeT backend = 0; backend < queries.size(); ++backend)
            {
                queries[backend].enable_optix_culling_statistics(backend != 0);
                REQUIRE(detect(backend));
                const auto stats = queries[backend].last_culling_statistics();
                queries[backend].enable_optix_culling_statistics(false);
                REQUIRE(read_pairs(pts[backend], false) == expected_pt);
                REQUIRE(read_pairs(ees[backend], true) == expected_ee);
                if(backend != 0)
                {
                    REQUIRE(queries[backend].last_statistics().launches > 0);
                    const Json diagnostic{
                        {"method", methods[backend]},
                        {"scope", scope},
                        {"group_size", queries[backend].optix_group_size()},
                        {"group_count", queries[backend].optix_group_count()},
                        {"group_tests", stats.group_tests},
                        {"body_rejections", stats.body_rejections},
                        {"contact_rejections", stats.contact_rejections},
                        {"skipped_leaves", stats.skipped_leaves},
                        {"leaf_tests", stats.leaf_tests},
                        {"emitted_pairs", stats.emitted_pairs}};
                    std::cout << "SINGLE_EF_CULLING " << diagnostic.dump() << '\n';
                }
            }

            for(int repeat = 1; repeat <= repeats; ++repeat)
                for(SizeT backend = 0; backend < queries.size(); ++backend)
                {
                    CAPTURE(repeat, backend);
                    REQUIRE_FALSE(queries[backend].optix_culling_statistics_enabled());
                    REQUIRE(cudaDeviceSynchronize() == cudaSuccess);
                    const auto before    = queries[backend].statistics();
                    const auto start     = std::chrono::steady_clock::now();
                    bool       supported = true;
                    for(int call = 0; call < calls; ++call)
                        supported &= detect(backend);
                    const auto   status = cudaDeviceSynchronize();
                    const auto   stop   = std::chrono::steady_clock::now();
                    const double milliseconds =
                        std::chrono::duration<double, std::milli>(stop - start).count() / calls;
                    REQUIRE(status == cudaSuccess);
                    REQUIRE(supported);
                    REQUIRE(read_pairs(pts[backend], false) == expected_pt);
                    REQUIRE(read_pairs(ees[backend], true) == expected_ee);
                    check_backend_execution(queries[backend], true);
                    const auto after = queries[backend].statistics();
                    if(backend != 0)
                    {
                        REQUIRE(after.launches - before.launches >= calls);
                        REQUIRE((reuse ? after.refits - before.refits :
                                         after.builds - before.builds)
                                == calls);
                    }
                    const Json result{
                        {"method", methods[backend]},
                        {"scope", scope},
                        {"repeat", repeat},
                        {"calls", calls},
                        {"ms_per_call", milliseconds},
                        {"vertices", mesh.vertices.size()},
                        {"edges", mesh.edges.size()},
                        {"triangles", mesh.triangles.size()},
                        {"pt_count", pts[backend].size()},
                        {"ee_count", ees[backend].size()},
                        {"optix_group_size",
                         backend == 0 ? 0 : queries[backend].optix_group_size()},
                        {"optix_group_count", queries[backend].optix_group_count()},
                        {"culling_statistics_enabled",
                         queries[backend].optix_culling_statistics_enabled()},
                        {"optix_builds", after.builds - before.builds},
                        {"optix_refits", after.refits - before.refits},
                        {"optix_launches", after.launches - before.launches}};
                    std::cout << "SINGLE_EF_BENCHMARK " << result.dump() << '\n';
                }
        }
    }
}
