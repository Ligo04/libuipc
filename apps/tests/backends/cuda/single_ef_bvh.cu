#include <type_define.h>
#include <app/app.h>
#include <collision_detection/filters/simplex_bvh_query.h>
#include <collision_detection/filters/trajectory_prefix.h>
#include <cuda_runtime.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

namespace test_single_ef_bvh
{
using namespace uipc;
using namespace uipc::backend::cuda;
namespace ct = uipc::backend::cuda_tool;
using Pair   = std::pair<IndexT, IndexT>;

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
    SimplexBVHQuery::PrimitiveBounds view() const
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
                0.0,
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
                0.0,
                0.0};
    }
    bvh_simplex_detail::NodePredicate node() const
    {
        return {d_self.view(), d_contact_mask.cviewer()};
    }

    bool allowed(IndexT a, IndexT b) const
    {
        return contact_mask[contacts[a] * d_contact_mask.extent().w + contacts[b]]
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

void check_mesh(Mesh& mesh, SizeT chunk, bool check_cpu = true)
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
    SimplexBVHQuery query;
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
                         ees));
    CHECK(read_pairs(pts, false) == actual_pt);
    CHECK(read_pairs(ees, true) == actual_ee);
}

void check_unsupported(Mesh& mesh)
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
    SimplexBVHQuery query;
    CHECK_FALSE(query.detect(triangle_bvh,
                             mesh.pt(),
                             mesh.ee(),
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees));
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
    SECTION("surface rows and global IDs differ; one-edge chunks")
    {
        Mesh mesh(4);
        check_mesh(mesh, 1);
    }
    SECTION("more than 64 nearby faces, retry and cross-chunk uniqueness")
    {
        Mesh mesh(80);
        mesh.send();
        SizeT overlaps = 0;
        for(const auto& box : mesh.triangle_bounds.boxes)
            overlaps += mesh.edge_bounds.boxes[0].intersects(box);
        REQUIRE(overlaps > 64);
        check_mesh(mesh, 2);
    }
    SECTION("EF capacity overflow replays the complete query")
    {
        Mesh mesh(160);
        check_mesh(mesh, std::numeric_limits<SizeT>::max());
    }
    SECTION("contact mask")
    {
        Mesh mesh(4);
        mesh.contact_mask = {1, 0, 0, 1};
        check_mesh(mesh, 2);
    }
    SECTION("subscene mask")
    {
        Mesh mesh(4);
        mesh.subscene_mask = {1, 0, 0, 1};
        check_mesh(mesh, 2);
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
        check_mesh(mesh, 1);
    }
    SECTION("mixed contact IDs with all contacts allowed")
    {
        Mesh mesh(4);
        for(auto v : mesh.vertices)
            mesh.contacts[v] = v % 2;
        check_mesh(mesh, 2);
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
        check_mesh(mesh, 1);
    }
    SECTION("body self collision disabled")
    {
        Mesh mesh(4);
        mesh.self = {0, 0};
        check_mesh(mesh, 2);
    }
    SECTION("zero-length geometry with distinct vertex IDs")
    {
        Mesh mesh(3);
        mesh.positions[2] = mesh.positions[1];
        // Flagged degenerate predicates are compared to the existing GPU path.
        check_mesh(mesh, 1, false);
    }
    SECTION("wire edge connecting surface vertices")
    {
        Mesh mesh(2);
        mesh.edges.emplace_back(1, 4);
        check_mesh(mesh, 1);
    }
    SECTION("isolated particle near triangle interiors")
    {
        Mesh mesh(2);
        mesh.positions[0] = Vector3{0.2, 0.2, 0.025};
        mesh.vertices.push_back(0);
        check_mesh(mesh, 1);
    }
    SECTION("particles and wire edges share the PT and EE proxy tree")
    {
        Mesh mesh(4);
        mesh.add_codimensional_geometry();
        check_mesh(mesh, 1);
    }
    SECTION("mixed-dimensional EF capacity overflow and chunk replay")
    {
        Mesh mesh(160);
        mesh.add_codimensional_geometry();
        check_mesh(mesh, std::numeric_limits<SizeT>::max());
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
        check_mesh(mesh, 2);
    }
    SECTION("pure codimensional topology does not publish partial output")
    {
        Mesh mesh(0);
        mesh.add_codimensional_geometry();
        check_unsupported(mesh);
    }
    SECTION("duplicate vertex does not publish partial output")
    {
        Mesh mesh(2);
        mesh.vertices.push_back(mesh.vertices.front());
        check_unsupported(mesh);
    }
    SECTION("missing triangle edge does not publish partial output")
    {
        Mesh mesh(2);
        mesh.edges.pop_back();
        check_unsupported(mesh);
    }
    SECTION("empty surface does not publish partial output")
    {
        Mesh mesh(0);
        check_unsupported(mesh);
    }
}

TEST_CASE("single EF BVH retains reusable candidate output storage", "[cuda][single_ef_bvh]")
{
    using namespace test_single_ef_bvh;
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
    SimplexBVHQuery query;
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
                            const std::vector<Float>&   alphas)
{
    InfoStacklessBVH              baseline_e, baseline_t, single_t;
    InfoStacklessBVH::QueryBuffer baseline_pt, baseline_ee, pts, ees;
    for(auto* output : {&pts, &ees})
    {
        output->m_pairs.release();
        output->reserve(1);
    }
    SimplexBVHQuery query;
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
                             ees));
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
        check_swept_candidates(mesh, motion, {0.25, 0.5, 1.0});
    }
    SECTION("deforming wires and a moving isolated particle")
    {
        Mesh       mesh(3);
        const auto first_codim = mesh.positions.size();
        mesh.add_codimensional_geometry();
        std::vector<Vector3> motion(mesh.positions.size(), Vector3::Zero());
        for(SizeT v = first_codim; v < mesh.positions.size(); ++v)
            motion[v] = Vector3{v % 2 ? 0.1 : -0.1, 0.05, v % 2 ? 0.5 : -0.5};
        check_swept_candidates(mesh, motion, {0.3, 0.8, 1.0});
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
        check_swept_candidates(mesh, motion, {0.25, 0.75, 1.0});
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
        check_swept_candidates(mesh, motion, {0.125, 0.5, 1.0});
    }
    SECTION("positive alpha with zero displacement")
    {
        Mesh                 mesh(3);
        std::vector<Vector3> motion(mesh.positions.size(), Vector3::Zero());
        check_swept_candidates(mesh, motion, {0.25, 1.0});
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
        check_swept_candidates(mesh, motion, {0.5, 1.0});
    }
    SECTION("a collapsed edge opens during the sweep")
    {
        Mesh mesh(3);
        mesh.positions[2] = mesh.positions[1];
        std::vector<Vector3> motion(mesh.positions.size(), Vector3::Zero());
        motion[2] = Vector3{0.2, 0, 0.4};
        check_swept_candidates(mesh, motion, {0.5, 1.0});
    }
}

TEST_CASE("single EF proxy expansion preserves EE hits after float cancellation", "[cuda][single_ef_bvh]")
{
    using namespace test_single_ef_bvh;
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
    check_mesh(mesh, 1, false);
}

TEST_CASE("Simplex cache preserves fresh traversal across changes",
          "[cuda][single_ef_bvh][ef_cache][simplex_query_policy]")
{
    using namespace test_single_ef_bvh;
    const auto method = GENERATE(SimplexBVHQuery::Method::InfoStackless,
                                 SimplexBVHQuery::Method::SingleEF);
    CAPTURE(method);
    Mesh mesh(4);
    mesh.add_codimensional_geometry();
    SimplexBVHQuery query;
    query.method          = method;
    query.max_chunk_edges = 2;
    InfoStacklessBVH              tree, baseline_e, baseline_t;
    InfoStacklessBVH::QueryBuffer pts, ees, expected_pt, expected_ee;
    for(auto* output : {&pts, &ees})
    {
        output->m_pairs.release();
        output->reserve(1);
    }
    auto compare = [&](Float alpha, Float skin, Vector3 motion)
    {
        mesh.send(std::vector<Vector3>(mesh.positions.size(), motion), alpha);
        auto pt  = mesh.pt();
        auto ee  = mesh.ee();
        pt.alpha = ee.alpha = alpha;
        pt.skin = ee.skin = skin;
        baseline_e.build(mesh.edge_bounds.d_boxes,
                         mesh.edge_bounds.d_bids,
                         mesh.edge_bounds.d_cids);
        baseline_t.build(mesh.triangle_bounds.d_boxes,
                         mesh.triangle_bounds.d_bids,
                         mesh.triangle_bounds.d_cids);
        baseline_e.detect(mesh.d_contact_mask.view(), mesh.node(), ee, expected_ee);
        baseline_t.query(mesh.point_bounds.d_boxes,
                         mesh.point_bounds.d_bids,
                         mesh.point_bounds.d_cids,
                         mesh.d_contact_mask.view(),
                         mesh.node(),
                         pt,
                         expected_pt);
        REQUIRE(query.detect(tree,
                             pt,
                             ee,
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees));
        CHECK(read_pairs(pts, false) == read_pairs(expected_pt, false));
        CHECK(read_pairs(ees, true) == read_pairs(expected_ee, true));
        CHECK(cudaDeviceSynchronize() == cudaSuccess);
    };
    compare(0.0, 0.0, Vector3::Zero());
    REQUIRE(query.refresh_count() == 1);
    // All chunks must survive reuse, including the shorter last chunk.
    compare(0.0, 0.0, Vector3::Zero());
    CHECK(query.refresh_count() == 1);
    CHECK(query.reuse_count() == 1);
    // DCD refreshes must remain conservative for later swept queries.
    for(Float alpha : {0.25, 0.5, 1.0})
        compare(alpha, mesh.radius / 2, Vector3{0, 0, 0.005});
    CHECK(query.refresh_count() == 1);
    CHECK(query.reuse_count() == 4);
    // Position-free pruning is cached; in-place metadata edits must refresh
    // before previously disabled pairs can become visible.
    mesh.contact_mask.assign(4, 0);
    compare(0.0, 0.0, Vector3::Zero());
    CHECK(pts.size() == 0);
    CHECK(ees.size() == 0);
    mesh.contact_mask.assign(4, 1);
    mesh.self          = {0, 0};
    mesh.subscene_mask = {1, 0, 0, 1};
    compare(0.5, 0.0, Vector3{0, 0, 0.005});
    CHECK(query.refresh_count() == 3);
    // Same allocation and counts, different ownership.
    std::swap(mesh.edges[0], mesh.edges.back());
    std::reverse(mesh.vertices.begin(), mesh.vertices.end());
    compare(0.0, 0.0, Vector3::Zero());
    CHECK(query.refresh_count() == 4);
    // Escape at either endpoint invalidates the complete fat-proxy cache.
    compare(1.0, 0.0, Vector3{0, 0, 0.5});
    CHECK(query.refresh_count() == 5);
    query.max_chunk_edges = 1;
    compare(0.0, 0.0, Vector3::Zero());
    // Chunk partitioning belongs to EF expansion; direct Info does not use it.
    const SizeT after_chunk = method == SimplexBVHQuery::Method::SingleEF ? 6 : 5;
    CHECK(query.refresh_count() == after_chunk);
    // The validation route bypasses containment and reruns the traversal.
    query.force_fresh = true;
    compare(0.0, 0.0, Vector3::Zero());
    CHECK(query.refresh_count() == after_chunk + 1);
}

TEST_CASE("fat EF cache refreshes after leaf escape and topology growth", "[cuda][single_ef_bvh][ef_cache]")
{
    using namespace test_single_ef_bvh;
    Mesh                          mesh(3);
    SimplexBVHQuery               query;
    InfoStacklessBVH              tree;
    InfoStacklessBVH::QueryBuffer pts, ees;
    auto                          run = [&]
    {
        mesh.send();
        REQUIRE(query.detect(tree,
                             mesh.pt(),
                             mesh.ee(),
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees));
        auto expected = mesh.all_pairs();
        CHECK(read_pairs(pts, false) == expected.first);
        CHECK(read_pairs(ees, true) == expected.second);
    };
    run();
    for(auto& position : mesh.positions)
        position.x() += mesh.radius - 1e-5;
    run();
    CHECK(query.refresh_count() == 1);
    for(auto& position : mesh.positions)
        position.x() += 2e-5;
    run();
    CHECK(query.refresh_count() == 2);
    // Move a triangle and all of the primitives it owns beyond the old skin.
    for(IndexT vertex : mesh.triangles[1])
        mesh.positions[vertex].z() += 0.5;
    run();
    CHECK(query.refresh_count() == 3);
    mesh.add_codimensional_geometry();
    run();
    CHECK(query.refresh_count() == 4);
    mesh.edges[0] = mesh.edges[1];
    mesh.send();
    // Invalid topology must fall back without publishing stale partial output.
    const auto previous_pt = read_pairs(pts, false);
    const auto previous_ee = read_pairs(ees, true);
    CHECK_FALSE(query.detect(tree,
                             mesh.pt(),
                             mesh.ee(),
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees));
    CHECK(read_pairs(pts, false) == previous_pt);
    CHECK(read_pairs(ees, true) == previous_ee);
}

TEST_CASE("Simplex cover chains retain mixed-CID pairs and canonical EE order",
          "[cuda][single_ef_bvh][single_ef_queries][ef_cover][simplex_query_policy]")
{
    using namespace test_single_ef_bvh;
    const auto method = GENERATE(SimplexBVHQuery::Method::InfoStackless,
                                 SimplexBVHQuery::Method::SingleEF);
    CAPTURE(method);
    Mesh mesh(12);
    mesh.add_codimensional_geometry();
    mesh.d_contact_mask.resize(ct::Extent2D{4, 4});
    mesh.contact_mask.assign(16, 0);
    for(IndexT a = 0; a < 2; ++a)
        for(IndexT b = 2; b < 4; ++b)
            mesh.contact_mask[a * 4 + b] = mesh.contact_mask[b * 4 + a] = 1;
    for(SizeT f = 0; f < mesh.triangles.size(); ++f)
        for(IndexT k = 0; k < 3; ++k)
            mesh.contacts[mesh.triangles[f][k]] = f < 10 ? 2 + (f + k) % 2 : (f + k) % 2;
    SimplexBVHQuery query;
    query.method          = method;
    query.max_chunk_edges = 1;
    InfoStacklessBVH              tree, baseline_e, baseline_t;
    InfoStacklessBVH::QueryBuffer pts, ees, expected_pt, expected_ee;
    pts.m_pairs.release();
    pts.reserve(1);
    ees.m_pairs.release();
    ees.reserve(1);
    ct::DeviceBuffer<float4> f32_positions, f32_steps;
    int                      hooks   = 0;
    auto                     compare = [&](bool swept = false)
    {
        mesh.send();
        auto pt = mesh.pt();
        auto ee = mesh.ee();
        if(swept)
        {
            // The production prefix stores d_hat and thickness in the w lanes.
            std::vector<float4> positions, steps;
            for(SizeT i = 0; i < mesh.positions.size(); ++i)
            {
                const auto p = mesh.positions[i];
                positions.push_back(make_float4(p.x(), p.y(), p.z(), mesh.dhats[i]));
                steps.push_back(make_float4(0, 0, 0, mesh.thicknesses[i]));
            }
            upload(f32_positions, positions);
            upload(f32_steps, steps);
            pt.alpha = ee.alpha = 1.0;
        }
        // Fresh traversal deliberately uses the FP64 predicates without the
        // FP32 inputs, independently checking the certified three-stage queue.
        baseline_e.build(mesh.edge_bounds.d_boxes,
                         mesh.edge_bounds.d_bids,
                         mesh.edge_bounds.d_cids);
        baseline_t.build(mesh.triangle_bounds.d_boxes,
                         mesh.triangle_bounds.d_bids,
                         mesh.triangle_bounds.d_cids);
        baseline_e.detect(mesh.d_contact_mask.view(), mesh.node(), ee, expected_ee);
        baseline_t.query(mesh.point_bounds.d_boxes,
                         mesh.point_bounds.d_bids,
                         mesh.point_bounds.d_cids,
                         mesh.d_contact_mask.view(),
                         mesh.node(),
                         pt,
                         expected_pt);
        if(swept)
        {
            pt.f32_positions = ee.f32_positions = f32_positions.cview();
            pt.f32_steps = ee.f32_steps = f32_steps.cview();
        }
        REQUIRE(query.detect(tree,
                             pt,
                             ee,
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees,
                             false,
                             [&] { ++hooks; }));
        CHECK(read_pairs(pts, false) == read_pairs(expected_pt, false));
        CHECK(read_pairs(ees, true) == read_pairs(expected_ee, true));
        if(!swept)
        {
            const auto oracle = mesh.all_pairs();
            CHECK(read_pairs(pts, false) == oracle.first);
            CHECK(read_pairs(ees, true) == oracle.second);
        }
        CHECK(cudaDeviceSynchronize() == cudaSuccess);
    };
    compare();
    const auto stats = query.statistics();
    REQUIRE(stats.pt_cover);
    REQUIRE(stats.ee_cover);
    if(method == SimplexBVHQuery::Method::SingleEF)
    {
        CHECK(stats.primary_queries > 0);
        CHECK(stats.primary_queries < mesh.edges.size());
        CHECK(stats.reverse_queries > 0);
    }
    CHECK(stats.stored_pts >= pts.size());
    CHECK(stats.stored_ees >= ees.size());
    CHECK_FALSE(read_pairs(ees, true).empty());
    compare(true);
    CHECK(query.refresh_count() == 1);
    CHECK(query.reuse_count() == 1);
    CHECK(hooks == 2);
    // Independent in-place edits must invalidate position-free pruning.
    auto refreshes     = query.refresh_count();
    mesh.subscene_mask = {1, 0, 0, 1};
    compare();
    CHECK(query.refresh_count() == ++refreshes);
    mesh.subscenes[mesh.triangles[0][0]] ^= 1;
    compare();
    CHECK(query.refresh_count() == ++refreshes);
    mesh.self = {0, 0};
    compare();
    CHECK(query.refresh_count() == ++refreshes);
    mesh.bodies[mesh.triangles[1][0]] ^= 1;
    compare();
    CHECK(query.refresh_count() == ++refreshes);
    mesh.contacts[mesh.triangles[0][0]] = 0;
    compare();
    CHECK(query.refresh_count() == ++refreshes);
    mesh.contact_mask.assign(16, 0);
    compare();
    CHECK(query.refresh_count() == ++refreshes);
    CHECK(pts.size() == 0);
    CHECK(ees.size() == 0);
    if(method == SimplexBVHQuery::Method::SingleEF)
    {
        CHECK(query.statistics().primary_queries == 0);
        CHECK(query.statistics().reverse_queries == 0);
    }
}

TEST_CASE("Simplex skins track primitive escapes and topology retention counts frames",
          "[cuda][single_ef_bvh][single_ef_queries][ef_skin][simplex_query_policy]")
{
    using namespace test_single_ef_bvh;
    const auto method = GENERATE(SimplexBVHQuery::Method::InfoStackless,
                                 SimplexBVHQuery::Method::SingleEF);
    CAPTURE(method);
    Mesh mesh(10);
    mesh.contact_mask = {0, 1, 1, 0};
    for(SizeT f = 0; f < mesh.triangles.size(); ++f)
        for(IndexT v : mesh.triangles[f])
            mesh.contacts[v] = f == 9 ? 0 : 1;
    const auto corner         = mesh.triangles[0];
    mesh.positions[corner[1]] = Vector3{1, 1, 0};
    mesh.positions[corner[2]] = Vector3{2, 1, 0};
    SimplexBVHQuery query;
    query.method = method;
    InfoStacklessBVH              tree;
    InfoStacklessBVH::QueryBuffer pts, ees;
    auto                          run = [&](bool frame = false)
    {
        mesh.send();
        REQUIRE(query.detect(tree,
                             mesh.pt(),
                             mesh.ee(),
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees,
                             frame));
        const auto expected = mesh.all_pairs();
        CHECK(read_pairs(pts, false) == expected.first);
        CHECK(read_pairs(ees, true) == expected.second);
    };
    run(true);
    REQUIRE(query.statistics().pt_cover);
    REQUIRE(query.statistics().ee_cover);
    const auto initial_builds = query.statistics().topology_builds;
    CHECK(query.statistics().cover_skin == 1.0);
    CHECK(query.statistics().other_skin == 0.01);
    const auto old_edges = mesh.edge_bounds.boxes,
               old_faces = mesh.triangle_bounds.boxes;
    // The owned point leaves its own thin skin while all edge/face boxes
    // shrink or stay fixed. A containment check on proxy unions misses this.
    const Float delta = 0.02 * mesh.radius;
    mesh.positions[corner[0]].x() += delta;
    mesh.positions[corner[0]].y() += delta;
    run();
    auto contained = [](const AABB& current, const AABB& previous)
    {
        return (previous.min().array() <= current.min().array()).all()
               && (current.max().array() <= previous.max().array()).all();
    };
    for(SizeT i = 0; i < old_edges.size(); ++i)
        CHECK(contained(mesh.edge_bounds.boxes[i], old_edges[i]));
    for(SizeT i = 0; i < old_faces.size(); ++i)
        CHECK(contained(mesh.triangle_bounds.boxes[i], old_faces[i]));
    CHECK(query.refresh_count() == 2);
    CHECK(query.statistics().other_skin == 1.0);
    for(IndexT v : mesh.triangles[9])
        mesh.positions[v].x() += 2 * mesh.radius;
    run();
    CHECK(query.refresh_count() == 3);
    CHECK(query.statistics().cover_skin == 2.0);
    run(true);  // Frame 2 carries the previous frame's escapes.
    CHECK(query.statistics().cover_skin == 2.0);
    CHECK(query.statistics().other_skin == 1.0);
    const auto before_lower_escape = query.refresh_count();
    for(IndexT v : mesh.triangles[9])
        mesh.positions[v].x() += 1.5 * mesh.radius;
    run();  // Inside the full 2x skin but outside the next-lower 1x skin.
    CHECK(query.refresh_count() == before_lower_escape);
    run(true);  // Frame 3 retains the actual cache, including its saved shrink.
    CHECK(query.statistics().cover_skin == 2.0);
    CHECK(query.statistics().other_skin == 1.0);
    for(IndexT v : mesh.triangles[9])
        mesh.positions[v].x() -= 1.5 * mesh.radius;
    run();
    run(true);  // Frame 4 observes no lower-skin escape.
    CHECK(query.refresh_count() == before_lower_escape);
    query.force_fresh = true;
    run(true);  // Frame 5 applies the now-thin desired policy on a real refresh.
    CHECK(query.statistics().cover_skin == 1.0);
    CHECK(query.statistics().other_skin == 0.01);
    for(int i = 0; i < 4; ++i)
        run();
    query.force_fresh = false;
    CHECK(query.statistics().topology_builds == initial_builds);
    const auto before_frames = query.refresh_count();
    for(int frame = 6; frame <= 8; ++frame)
        run(true);
    CHECK(query.refresh_count() == before_frames);
    CHECK(query.statistics().topology_builds == initial_builds);
    CHECK(query.statistics().cross_frame_reuses >= 3);
    query.force_fresh = true;
    run();  // The pending eighth-frame topology rebuild is consumed here.
    CHECK(query.statistics().topology_builds > initial_builds);
    CHECK(cudaDeviceSynchronize() == cudaSuccess);
}

TEST_CASE("SingleEF query bounds exclude unused long owner edges",
          "[cuda][single_ef_bvh][single_ef_queries][ef_query_bounds]")
{
    using namespace test_single_ef_bvh;
    const bool reverse = GENERATE(false, true);
    Mesh       mesh(10);
    mesh.contact_mask = {0, 1, 1, 0};
    for(SizeT f = 0; f < mesh.triangles.size(); ++f)
        for(IndexT v : mesh.triangles[f])
        {
            mesh.contacts[v] = f == 9 ? 0 : 1;
            mesh.positions[v].x() += f == 9 ? 0 : 50;
        }
    if(reverse)
    {
        const IndexT v = mesh.positions.size();
        mesh.positions.emplace_back(0, 0, 0.03);
        mesh.positions.emplace_back(100, 0, 0.03);
        mesh.vertices.push_back(v);
        mesh.vertices.push_back(v + 1);
        mesh.edges.emplace_back(v, v + 1);
        mesh.bodies.insert(mesh.bodies.end(), {0, 1});
        mesh.contacts.insert(mesh.contacts.end(), {0, 1});
        mesh.subscenes.insert(mesh.subscenes.end(), {0, 0});
        mesh.dhats.insert(mesh.dhats.end(), {mesh.radius, mesh.radius});
        mesh.thicknesses.insert(mesh.thicknesses.end(), {0, 0});
    }
    else
    {
        const auto face        = mesh.triangles[9];
        mesh.contacts[face[1]] = mesh.contacts[face[2]] = 1;
        mesh.positions[face[1]].x()                     = 100;
    }
    mesh.send();
    SimplexBVHQuery               query;
    InfoStacklessBVH              tree;
    InfoStacklessBVH::QueryBuffer pts, ees;
    REQUIRE(query.detect(tree,
                         mesh.pt(),
                         mesh.ee(),
                         mesh.node(),
                         mesh.point_bounds.view(),
                         mesh.edge_bounds.view(),
                         mesh.triangle_bounds.view(),
                         pts,
                         ees));
    const auto expected = mesh.all_pairs();
    CHECK(read_pairs(pts, false) == expected.first);
    CHECK(read_pairs(ees, true) == expected.second);
    REQUIRE(query.statistics().pt_cover);
    REQUIRE(query.statistics().ee_cover);
    if(reverse)
    {
        CHECK(query.statistics().reverse_queries > 0);
        CHECK(query.statistics().ef_pairs == 0);
    }
    else
    {
        CHECK(query.statistics().primary_queries == 1);
        CHECK(query.statistics().reverse_queries == 0);
        CHECK(query.statistics().ef_pairs == 1);
    }
}

TEST_CASE("SingleEF batches chunks and refills bounded typed outputs",
          "[cuda][single_ef_bvh][single_ef_queries][ef_refresh_batch]")
{
    using namespace test_single_ef_bvh;
    const bool split = GENERATE(false, true);
    Mesh       mesh(160);
    mesh.send();
    SimplexBVHQuery query;
    if(split)
        query.max_chunk_edges = 17;
    InfoStacklessBVH              tree;
    InfoStacklessBVH::QueryBuffer pts, ees;
    const auto                    expected = mesh.all_pairs();
    auto                          run      = [&]
    {
        REQUIRE(query.detect(tree,
                             mesh.pt(),
                             mesh.ee(),
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees));
        CHECK(read_pairs(pts, false) == expected.first);
        CHECK(read_pairs(ees, true) == expected.second);
    };
    run();
    const auto first = query.statistics();
    CHECK(first.typed_refills > 0);
    if(split)
    {
        CHECK(first.refresh_waits == 1);
        CHECK(first.ef_retries == 0);
    }
    else
        CHECK(first.ef_retries > 0);
    query.force_fresh = true;
    run();
    CHECK(query.statistics().refresh_waits == first.refresh_waits + 1);
    CHECK(query.statistics().ef_retries == first.ef_retries);
    CHECK(query.statistics().typed_refills == first.typed_refills);
    CHECK(cudaDeviceSynchronize() == cudaSuccess);
}

TEST_CASE("Simplex fused prefix validates new primitive metadata and large tables across frames",
          "[cuda][single_ef_bvh][single_ef_queries][ef_prefix][simplex_query_policy]")
{
    using namespace test_single_ef_bvh;
    const auto method = GENERATE(SimplexBVHQuery::Method::InfoStackless,
                                 SimplexBVHQuery::Method::SingleEF);
    CAPTURE(method);
    Mesh mesh(3);
    mesh.d_contact_mask.resize(ct::Extent2D{16, 16});
    mesh.contact_mask.assign(256, 1);
    SimplexBVHQuery query;
    query.method = method;
    InfoStacklessBVH              tree;
    InfoStacklessBVH::QueryBuffer pts, ees;
    ct::DeviceBuffer<IndexT>      bodies;
    ct::DeviceBuffer<float4>      positions, steps, ends;
    TrajectoryPrefix              prefix;
    auto                          run = [&]
    {
        mesh.send();
        upload(bodies, mesh.bodies);
        positions.resize_discard(mesh.positions.size());
        steps.resize_discard(mesh.positions.size());
        ends.resize_discard(mesh.positions.size());
        prefix.accepted_step_endpoints = true;
        prefix.positions               = mesh.d_positions.data();
        prefix.displacements           = mesh.d_displacements.data();
        prefix.d_hats                  = mesh.d_hats.data();
        prefix.thicknesses             = mesh.d_thicknesses.data();
        prefix.v2bs                    = bodies.data();
        prefix.contact_ids             = mesh.d_contacts.data();
        prefix.vertices                = mesh.positions.size();
        prefix.f32_positions           = positions.data();
        prefix.f32_steps               = steps.data();
        prefix.f32_ends                = ends.data();
        auto bind                      = [&](int f, Bounds& bounds)
        {
            auto& family = prefix.family[f];
            family.n     = bounds.boxes.size();
            family.aabbs = bounds.d_boxes.data();
            family.bids  = bounds.d_bids.data();
            family.cids  = bounds.d_cids.data();
            // Stale derived outputs must not participate in the snapshot check.
            bounds.d_bids.fill(97);
            bounds.d_cids.fill(97);
        };
        bind(1, mesh.point_bounds);
        bind(2, mesh.edge_bounds);
        bind(3, mesh.triangle_bounds);
        prefix.family[1].vertices  = mesh.d_vertices.data();
        prefix.family[2].edges     = mesh.d_edges.data();
        prefix.family[3].triangles = mesh.d_triangles.data();
        REQUIRE(query.detect(tree,
                             mesh.pt(),
                             mesh.ee(),
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees,
                             true,
                             {},
                             &prefix));
        const auto expected = mesh.all_pairs();
        CHECK(read_pairs(pts, false) == expected.first);
        CHECK(read_pairs(ees, true) == expected.second);
    };
    run();
    run();
    CHECK(query.refresh_count() == 1);
    CHECK(query.statistics().cross_frame_reuses == 1);
    mesh.bodies[mesh.triangles[0][0]] ^= 1;
    run();
    CHECK(query.refresh_count() == 2);
    mesh.contact_mask.back() = 0;  // The snapshot range is larger than the vertex grid.
    run();
    CHECK(query.refresh_count() == 3);
    mesh.subscene_mask = {1, 0, 0, 1};
    run();
    CHECK(query.refresh_count() == 4);
    std::swap(mesh.edges[0], mesh.edges[1]);
    run();
    CHECK(query.refresh_count() == 5);
    for(IndexT v : mesh.triangles[1])
        mesh.positions[v].x() += 4 * mesh.radius;
    run();
    CHECK(query.refresh_count() == 6);
    mesh.add_codimensional_geometry();
    run();  // Reuse the same prefix POD after its snapshots and fat buffers grow.
    CHECK(query.refresh_count() == 7);
    CHECK(query.statistics().fused_prefixes == 8);
    CHECK(cudaDeviceSynchronize() == cudaSuccess);
}

namespace test_single_ef_bvh
{
struct IndependentPredicateInputs
{
    static constexpr int       MaskSize = 16;
    std::vector<IndexT>        contacts, subscenes, self;
    std::vector<IndexT>        contact_mask, subscene_mask;
    ct::DeviceBuffer<IndexT>   d_contacts, d_subscenes, d_self;
    ct::DeviceBuffer2D<IndexT> d_contact_mask{ct::Extent2D{MaskSize, MaskSize}};
    ct::DeviceBuffer2D<IndexT> d_subscene_mask{ct::Extent2D{MaskSize, MaskSize}};

    explicit IndependentPredicateInputs(const Mesh& mesh)
        : contacts(mesh.contacts)
        , subscenes(mesh.subscenes)
        , self(mesh.self)
        , contact_mask(MaskSize * MaskSize, 1)
        , subscene_mask(MaskSize * MaskSize, 1)
    {
        // Independently bound ranges can exceed every primitive/vertex grid.
        contacts.resize(257, 0);
        subscenes.resize(257, 0);
    }
    void send()
    {
        upload(d_contacts, contacts);
        upload(d_subscenes, subscenes);
        upload(d_self, self);
        d_contact_mask.view().copy_from(contact_mask.data());
        d_subscene_mask.view().copy_from(subscene_mask.data());
    }
};

struct AuxiliaryQueries
{
    std::vector<IndexT>           vertices, dimensions;
    ct::DeviceBuffer<IndexT>      d_vertices, d_dimensions;
    Bounds                        bounds;
    IndependentPredicateInputs    pp_inputs, pe_inputs;
    InfoStacklessBVH::QueryBuffer pps, pes;

    AuxiliaryQueries(const Mesh& mesh, bool with_codim)
        : dimensions(257, 3)
        , pp_inputs(mesh)
        , pe_inputs(mesh)
    {
        if(with_codim)
            for(IndexT i = 0; i < 5; ++i)
            {
                const IndexT v = static_cast<IndexT>(mesh.positions.size()) - 1 - i;
                vertices.push_back(v);
                dimensions[v] = i == 4 ? 0 : 1;
            }
    }
    void send(Mesh& mesh)
    {
        upload(d_vertices, vertices);
        upload(d_dimensions, dimensions);
        pp_inputs.send();
        pe_inputs.send();
        bounds.boxes.clear();
        bounds.bids.clear();
        bounds.cids.clear();
        for(IndexT v : vertices)
            mesh.add_bounds(bounds, std::array<IndexT, 1>{v}, {}, 0.0);
        bounds.send();
    }
    SimplexBVHQuery::CodimQueries bind(const Mesh& mesh)
    {
        bvh_simplex_detail::PointPointPredicate pp;
        pp.Vs                    = mesh.d_vertices.cview();
        pp.codimVs               = d_vertices.cview();
        pp.Ps                    = mesh.d_positions.cview();
        pp.dxs                   = mesh.d_displacements.cview();
        pp.thicknesses           = mesh.d_thicknesses.cview();
        pp.dimensions            = d_dimensions.cview();
        pp.contact_element_ids   = pp_inputs.d_contacts.cview();
        pp.contact_mask_tabular  = pp_inputs.d_contact_mask.cviewer();
        pp.subscene_element_ids  = pp_inputs.d_subscenes.cview();
        pp.subscene_mask_tabular = pp_inputs.d_subscene_mask.cviewer();
        pp.body_self_collision   = pp_inputs.d_self.cview();
        pp.d_hats                = mesh.d_hats.cview();
        pp.alpha                 = 0.0;
        pp.skin                  = 0.0;
        bvh_simplex_detail::PointEdgePredicate pe;
        pe.codimVs               = d_vertices.cview();
        pe.Es                    = mesh.d_edges.cview();
        pe.Ps                    = mesh.d_positions.cview();
        pe.dxs                   = mesh.d_displacements.cview();
        pe.thicknesses           = mesh.d_thicknesses.cview();
        pe.contact_element_ids   = pe_inputs.d_contacts.cview();
        pe.contact_mask_tabular  = pe_inputs.d_contact_mask.cviewer();
        pe.subscene_element_ids  = pe_inputs.d_subscenes.cview();
        pe.subscene_mask_tabular = pe_inputs.d_subscene_mask.cviewer();
        pe.body_self_collision   = pe_inputs.d_self.cview();
        pe.d_hats                = mesh.d_hats.cview();
        pe.alpha                 = 0.0;
        pe.skin                  = 0.0;
        return {bounds.view(), pp, pe, &pps, &pes};
    }
};

using FourCandidates = std::array<std::vector<Pair>, 4>;
FourCandidates fresh_candidates(Mesh& mesh, const SimplexBVHQuery::CodimQueries& codim)
{
    InfoStacklessBVH              e, t, p;
    InfoStacklessBVH::QueryBuffer pps, pes, pts, ees;
    if(!mesh.edges.empty())
    {
        e.build(mesh.edge_bounds.d_boxes,
                mesh.edge_bounds.d_bids,
                mesh.edge_bounds.d_cids);
        e.detect(mesh.d_contact_mask.view(), mesh.node(), mesh.ee(), ees);
        if(codim.pe.codimVs.size())
            e.query(codim.points.aabbs,
                    codim.points.bids,
                    codim.points.cids,
                    mesh.d_contact_mask.view(),
                    mesh.node(),
                    codim.pe,
                    pes);
    }
    if(!mesh.triangles.empty())
    {
        t.build(mesh.triangle_bounds.d_boxes,
                mesh.triangle_bounds.d_bids,
                mesh.triangle_bounds.d_cids);
        t.query(mesh.point_bounds.d_boxes,
                mesh.point_bounds.d_bids,
                mesh.point_bounds.d_cids,
                mesh.d_contact_mask.view(),
                mesh.node(),
                mesh.pt(),
                pts);
    }
    if(codim.pp.codimVs.size())
    {
        p.build(codim.points.aabbs, codim.points.bids, codim.points.cids);
        p.query(mesh.point_bounds.d_boxes,
                mesh.point_bounds.d_bids,
                mesh.point_bounds.d_cids,
                mesh.d_contact_mask.view(),
                mesh.node(),
                codim.pp,
                pps);
    }
    return {read_pairs(pps, false),
            read_pairs(pes, false),
            read_pairs(pts, false),
            read_pairs(ees, true)};
}

struct ReusedPrefix
{
    ct::DeviceBuffer<IndexT> bodies;
    ct::DeviceBuffer<float4> positions, steps, ends;
    TrajectoryPrefix         prefix;

    TrajectoryPrefix* bind(Mesh& mesh, AuxiliaryQueries& auxiliary)
    {
        upload(bodies, mesh.bodies);
        positions.resize_discard(mesh.positions.size());
        steps.resize_discard(mesh.positions.size());
        ends.resize_discard(mesh.positions.size());
        prefix.accepted_step_endpoints      = true;
        prefix.positions                    = mesh.d_positions.data();
        prefix.displacements                = mesh.d_displacements.data();
        prefix.d_hats                       = mesh.d_hats.data();
        prefix.thicknesses                  = mesh.d_thicknesses.data();
        prefix.v2bs                         = bodies.data();
        prefix.contact_ids                  = mesh.d_contacts.data();
        prefix.vertices                     = mesh.positions.size();
        prefix.f32_positions                = positions.data();
        prefix.f32_steps                    = steps.data();
        prefix.f32_ends                     = ends.data();
        const std::array<Bounds*, 4> bounds = {
            &auxiliary.bounds, &mesh.point_bounds, &mesh.edge_bounds, &mesh.triangle_bounds};
        for(SizeT f = 0; f < bounds.size(); ++f)
        {
            auto& family = prefix.family[f];
            family.n     = bounds[f]->boxes.size();
            family.aabbs = bounds[f]->d_boxes.data();
            family.bids  = bounds[f]->d_bids.data();
            family.cids  = bounds[f]->d_cids.data();
            // The producer, rather than this stale output, must be validated.
            bounds[f]->d_bids.fill(97);
            bounds[f]->d_cids.fill(97);
        }
        prefix.family[0].vertices  = auxiliary.d_vertices.data();
        prefix.family[1].vertices  = mesh.d_vertices.data();
        prefix.family[2].edges     = mesh.d_edges.data();
        prefix.family[3].triangles = mesh.d_triangles.data();
        return &prefix;
    }
};
}  // namespace test_single_ef_bvh

TEST_CASE("Simplex query policy caches all four primitive families across frames",
          "[cuda][single_ef_bvh][simplex_query_policy][simplex_codim_cache]")
{
    using namespace test_single_ef_bvh;
    const auto method = GENERATE(SimplexBVHQuery::Method::InfoStackless,
                                 SimplexBVHQuery::Method::SingleEF);
    const int  kind   = GENERATE(0, 1, 2);  // empty, pure codim, mixed
    CAPTURE(method, kind);
    Mesh mesh(kind == 2 ? 3 : 0);
    if(kind)
    {
        mesh.add_codimensional_geometry();
        const auto first          = mesh.positions.size() - 5;
        mesh.positions[first + 1] = mesh.positions[first] + Vector3{0.02, 0, 0};
    }
    AuxiliaryQueries auxiliary(mesh, kind != 0);
    SimplexBVHQuery  query;
    query.method = method;
    InfoStacklessBVH              tree;
    InfoStacklessBVH::QueryBuffer pts, ees;
    auto                          run = [&]
    {
        mesh.send();
        auxiliary.send(mesh);
        auto       codim    = auxiliary.bind(mesh);
        const auto expected = fresh_candidates(mesh, codim);
        REQUIRE(query.detect(tree,
                             mesh.pt(),
                             mesh.ee(),
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees,
                             true,
                             {},
                             nullptr,
                             &codim));
        const FourCandidates actual = {read_pairs(auxiliary.pps, false),
                                       read_pairs(auxiliary.pes, false),
                                       read_pairs(pts, false),
                                       read_pairs(ees, true)};
        CHECK(actual == expected);
        if(query.method == SimplexBVHQuery::Method::InfoStackless || kind != 2)
        {
            CHECK(query.isolated_point_count() == 0);
            CHECK(query.standalone_edge_count() == 0);
        }
        return actual;
    };
    const auto first = run();
    CHECK(query.refresh_count() == 1);
    if(kind)
    {
        REQUIRE_FALSE(first[0].empty());
        REQUIRE_FALSE(first[1].empty());
        CHECK(query.statistics().stored_pps >= first[0].size());
        CHECK(query.statistics().stored_pes >= first[1].size());
    }
    CHECK(run() == first);
    CHECK(query.refresh_count() == 1);
    CHECK(query.statistics().cross_frame_reuses == 1);
    auto refreshes = query.refresh_count();
    query.method   = method == SimplexBVHQuery::Method::InfoStackless ?
                         SimplexBVHQuery::Method::SingleEF :
                         SimplexBVHQuery::Method::InfoStackless;
    CHECK(run() == first);
    CHECK(query.refresh_count() == ++refreshes);
    query.method = method;
    CHECK(run() == first);
    CHECK(query.refresh_count() == ++refreshes);
    if(kind == 2)
    {
        REQUIRE(query.detect(tree,
                             mesh.pt(),
                             mesh.ee(),
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees,
                             true));
        CHECK(read_pairs(pts, false) == first[2]);
        CHECK(read_pairs(ees, true) == first[3]);
        CHECK(query.statistics().stored_pps == 0);
        CHECK(query.statistics().stored_pes == 0);
        CHECK(query.refresh_count() == ++refreshes);
        CHECK(run() == first);
        CHECK(query.refresh_count() == ++refreshes);
    }
    if(kind)
    {
        std::reverse(auxiliary.vertices.begin(), auxiliary.vertices.end());
        run();  // Same allocation/count, different PP/PE primitive ordinals.
        CHECK(query.refresh_count() == ++refreshes);
        mesh.positions[auxiliary.vertices.front()].z() += 1.0;
        run();  // Only a codimensional point escapes its individual fat box.
        CHECK(query.refresh_count() == ++refreshes);
    }
    CHECK(cudaDeviceSynchronize() == cudaSuccess);
}

TEST_CASE("Simplex query policy invalidates independently bound PP and PE metadata",
          "[cuda][single_ef_bvh][simplex_query_policy][simplex_codim_metadata]")
{
    using namespace test_single_ef_bvh;
    const auto method = GENERATE(SimplexBVHQuery::Method::InfoStackless,
                                 SimplexBVHQuery::Method::SingleEF);
    const bool fused  = GENERATE(false, true);
    CAPTURE(method, fused);
    Mesh mesh(3);
    mesh.add_codimensional_geometry();
    const auto first          = mesh.positions.size() - 5;
    mesh.positions[first + 1] = mesh.positions[first] + Vector3{0.02, 0, 0};
    AuxiliaryQueries auxiliary(mesh, true);
    SimplexBVHQuery  query;
    query.method = method;
    ReusedPrefix                  prefix;
    InfoStacklessBVH              tree;
    InfoStacklessBVH::QueryBuffer pts, ees;
    auto                          run = [&]
    {
        mesh.send();
        auxiliary.send(mesh);
        auto       codim    = auxiliary.bind(mesh);
        const auto expected = fresh_candidates(mesh, codim);
        REQUIRE(query.detect(tree,
                             mesh.pt(),
                             mesh.ee(),
                             mesh.node(),
                             mesh.point_bounds.view(),
                             mesh.edge_bounds.view(),
                             mesh.triangle_bounds.view(),
                             pts,
                             ees,
                             true,
                             {},
                             fused ? prefix.bind(mesh, auxiliary) : nullptr,
                             &codim));
        const FourCandidates actual = {read_pairs(auxiliary.pps, false),
                                       read_pairs(auxiliary.pes, false),
                                       read_pairs(pts, false),
                                       read_pairs(ees, true)};
        CHECK(actual == expected);
        return actual;
    };
    const auto initial = run();
    REQUIRE_FALSE(initial[0].empty());
    REQUIRE_FALSE(initial[1].empty());
    run();
    CHECK(query.refresh_count() == 1);
    const auto* pp_ids          = auxiliary.pp_inputs.d_contacts.data();
    const auto* pe_ids          = auxiliary.pe_inputs.d_contacts.data();
    auto        refreshes       = query.refresh_count();
    auxiliary.dimensions.back() = 1;
    run();
    CHECK(query.refresh_count() == ++refreshes);
    for(auto* inputs : {&auxiliary.pp_inputs, &auxiliary.pe_inputs})
    {
        inputs->contacts.back() = 1;
        run();
        CHECK(query.refresh_count() == ++refreshes);
        inputs->subscenes.back() = 1;
        run();
        CHECK(query.refresh_count() == ++refreshes);
        inputs->self[0] = 0;
        run();
        CHECK(query.refresh_count() == ++refreshes);
        inputs->contact_mask.back() = 0;
        run();
        CHECK(query.refresh_count() == ++refreshes);
        inputs->subscene_mask.back() = 0;
        run();
        CHECK(query.refresh_count() == ++refreshes);
    }
    CHECK(auxiliary.pp_inputs.d_contacts.data() == pp_ids);
    CHECK(auxiliary.pe_inputs.d_contacts.data() == pe_ids);
    std::fill(auxiliary.pp_inputs.contact_mask.begin(),
              auxiliary.pp_inputs.contact_mask.end(),
              0);
    CHECK(run()[0].empty());
    CHECK(query.refresh_count() == ++refreshes);
    std::fill(auxiliary.pp_inputs.contact_mask.begin(),
              auxiliary.pp_inputs.contact_mask.end(),
              1);
    CHECK_FALSE(run()[0].empty());  // Re-enabled pairs must enter the cache again.
    CHECK(query.refresh_count() == ++refreshes);
    std::fill(auxiliary.pe_inputs.contact_mask.begin(),
              auxiliary.pe_inputs.contact_mask.end(),
              0);
    CHECK(run()[1].empty());
    CHECK(query.refresh_count() == ++refreshes);
    std::fill(auxiliary.pe_inputs.contact_mask.begin(),
              auxiliary.pe_inputs.contact_mask.end(),
              1);
    CHECK_FALSE(run()[1].empty());
    CHECK(query.refresh_count() == ++refreshes);
    auxiliary.dimensions.resize(300, 3);
    run();  // Reuse the prefix after a snapshot-only shape change.
    CHECK(query.refresh_count() == ++refreshes);
    CHECK(cudaDeviceSynchronize() == cudaSuccess);
}

TEST_CASE("Both simplex query methods use accepted-step endpoint rounding",
          "[cuda][single_ef_bvh][simplex_query_policy][step_rounding]")
{
    using namespace test_single_ef_bvh;
    const auto method = GENERATE(SimplexBVHQuery::Method::InfoStackless,
                                 SimplexBVHQuery::Method::SingleEF);
    CAPTURE(method);
    const Float delta = std::ldexp(Float{1}, -27);
    const Float alpha = 1 + delta;
    Mesh        mesh(1);
    std::fill(mesh.positions.begin(), mesh.positions.end(), Vector3::Constant(-1));
    std::fill(mesh.dhats.begin(), mesh.dhats.end(), 0.0);
    const std::vector<Vector3> motion(mesh.positions.size(), Vector3::Constant(1 - delta));
    mesh.send(motion, alpha);
    AuxiliaryQueries auxiliary(mesh, false);
    auxiliary.send(mesh);
    ReusedPrefix prefix;
    auto*        input = prefix.bind(mesh, auxiliary);
    // The common owner must enable this contract for both methods. The generic
    // prefix POD's default stays available to independent legacy callers.
    CHECK_FALSE(TrajectoryPrefix{}.accepted_step_endpoints);
    input->accepted_step_endpoints = false;
    input->alpha                   = alpha;
    SimplexBVHQuery query;
    query.method = method;
    InfoStacklessBVH              tree;
    InfoStacklessBVH::QueryBuffer pts, ees;
    auto                          pt = mesh.pt();
    auto                          ee = mesh.ee();
    pt.alpha = ee.alpha = alpha;
    REQUIRE(query.detect(tree,
                         pt,
                         ee,
                         mesh.node(),
                         mesh.point_bounds.view(),
                         mesh.edge_bounds.view(),
                         mesh.triangle_bounds.view(),
                         pts,
                         ees,
                         false,
                         {},
                         input));
    CHECK(input->accepted_step_endpoints);
    std::vector<float4> endpoints(mesh.positions.size());
    prefix.ends.copy_to(endpoints.data());
    for(const auto endpoint : endpoints)
    {
        CHECK(endpoint.x == 0.0f);
        CHECK(endpoint.y == 0.0f);
        CHECK(endpoint.z == 0.0f);
    }
    std::vector<AABB> boxes(mesh.vertices.size());
    mesh.point_bounds.d_boxes.copy_to(boxes.data());
    for(const auto& box : boxes)
        CHECK((box.max().array() >= 0.0f).all());
    CHECK(cudaDeviceSynchronize() == cudaSuccess);
}
