# Single EF BVH

Select the optional CUDA trajectory filter with:

```cpp
auto config = Scene::default_config();
config["collision_detection"]["method"] = "single_ef_bvh";
```

The default remains `info_stackless_bvh`. This method requires no external BVH
library or OptiX SDK.

## Candidate generation

For discrete and swept detection, a supported triangle surface uses one
triangle BVH to generate edge-face pairs. Each surface vertex chooses one
incident edge, and each edge chooses one incident triangle. An edge can
represent both of its endpoints. The resulting EF pairs produce the existing
PT and EE candidate formats without duplicates.

Mixed-dimensional scenes extend those proxies without constructing degenerate
triangles. A vertex without an incident edge owns a point query proxy, and an
edge without an incident triangle owns a standalone edge leaf. Thus PT covers
particles and rod vertices against triangles, while EE covers surface edges,
rod-surface edges and rod-rod edges. Every point still has exactly one query
owner and every edge exactly one leaf owner; the same `b < e` rule removes EE
duplicates independently of primitive ordering. Point queries cannot emit EE,
and edge leaves cannot emit PT.

The original `AllP-CodimP` PP and `CodimP-AllE` PE queries run alongside this
PT/EE tree. Their predicates, body/contact masks and downstream contact/CCD
handling are unchanged. Here codimensional vertices have dimension at most
one; a two-dimensional triangle shell already uses the triangle path.

Both passes query conservative AABB proxies. An edge proxy
contains its ordinary edge box and the point boxes of its represented vertices.
A triangle proxy contains its ordinary triangle box and the edge boxes it owns.
Incident EF pairs are retained; the original PT/EE predicates reject actual
topological neighbors. This avoids one-ring correction lists.

During DCD (`alpha == 0`), an additional node predicate tests the edge segment
against the node AABB enlarged by the edge's activation expansion and thickness.
It uses a closed-interval slab test in double precision, with outward rounding
of float BVH bounds and a node-scale margin for float bound-construction error,
including cancellation. The traversal thread loads its edge geometry and owned
point boxes once, and prepares reciprocal directions before visiting nodes.
An owned vertex's original point AABB is an independent
acceptance route, so a point with a larger expansion is not lost. Body and
contact ID pruning still runs before this geometric test.

During CCD (`alpha > 0`), the primitive boxes cover both `x` and `x + alpha * dx`
before expansion. Linear motion of every vertex stays inside those swept boxes;
the proxy ownership and expansion therefore work for moving and deforming
surfaces as well. The same EF tree generates PT/EE candidates, followed by the
existing CCD broadphase predicates and TOI kernels. Static segment-AABB pruning
is disabled in this pass: the swept edge can cross a box even when neither its
initial nor its final segment intersects it. The CCD candidate multiset is
preserved rather than reduced by a tighter swept-geometry test.

The filter reuses UIPC's double precision distance predicates, contact and
subscene masks, body self-collision rules, active-feature classification,
mollifier and CCD code. Proxy body/contact tags permit early culling only when
all represented endpoints have the same tag.

EF queries run in chunks with a checked 32-bit pair-count bound. Counts and
prefix sums for PT/EE expansion use 64-bit integers. Buffers grow to fit the
results; there is no per-edge 64-candidate limit. Counts exceeding downstream
32-bit indexing produce an explicit error rather than a partial result.

PT/EE expansion keeps its GPU count, prefix-scan and fill passes. Each chunk
writes directly into the final `QueryBuffer`; GPU counters carry output offsets
between chunks. `m_size` records the valid result count while `m_pairs.size()` retains
the writable high-water mark shared by DCD and CCD. Smaller or empty DCD results
do not shrink that range. Growth follows the BVH reserve ratio and keeps the
logical write capacity within the downstream 32-bit bound.

The PT/EE query has one combined readback after all chunks finish. Expansion
launches over reusable EF capacity and zeros inactive entries using the device
EF count, so the scan length is known without an intermediate host readback.
Scan and fill stay on the GPU, including cross-chunk offsets. The scan processes
padding as well as valid entries; the extra work depends on retained capacity.
The outer filter clears unused codimensional-family counters with a device
kernel before the query, avoiding counter uploads after the final readback.
In a mixed-dimensional scene, PP/PE queries launch before PT/EE. The pipeline
then downloads their counts together in its existing four-count readback and
replays only overflowing PP/PE buffers. Thus the mixed path currently has two
count readbacks when capacities fit, while a pure triangle scene retains one.

If EF or output capacity is insufficient, writes stay within bounds, the host
grows the buffers with the BVH reserve ratio, and the entire query is replayed.
No partial result is returned. Allocation and retry can synchronize on growth;
the final readback remains necessary for the existing host-sized downstream
interfaces. This does not make the whole collision pipeline GPU autonomous.

## Supported scope

At least one triangle is required. Isolated points and edges without incident
triangles are supported. Vertex and undirected edge lists must be unique,
triangle vertex indices must be valid and distinct,
and every triangle edge must be present. Nonmanifold triangle surfaces are
allowed. Geometry may contain zero-length edges with distinct vertex indices;
the existing distance predicates still determine acceptance.

Topology ownership is prepared once for the initialized world's static surface
topology. Position and material data stay on the GPU during repeated detection.
Future support for in-place topology rebuilds must explicitly invalidate this
cache.

Scenes without triangles and unsupported surface topology use the original
informative BVH queries in both DCD and CCD. A supported pure triangle surface
builds only the proxy triangle tree in both passes. A mixed scene adds the
original codimensional-point and edge trees for PP/PE, while its PT/EE proxy
tree has `triangle_count + standalone_edge_count` leaves and
`edge_count + isolated_point_count` queries. Previously allocated
fallback-tree memory can remain allocated after a fallback pass; tree count
alone does not predict whole-step runtime or peak memory.

## Focused validation

With CUDA tests enabled:

```bash
xmake build backend_cuda core
xmake run backend_cuda '[single_ef_bvh]'
xmake run backend_cuda 'compiled collision filters match the scene schema'
xmake run core 'scene_config_schema_and_validation'
```

The focused tests compare candidate sets and multiplicity with the existing
two-tree queries and exhaustive CPU enumeration. They include more than 64
nearby triangles, chunk boundaries, reordered surface indices, geometry
degeneracy, masks and unsupported topology. Swept tests include intermediate
crossings with separated endpoints, deforming edges, multiple alpha values,
stationary positive-alpha queries, and DCD reuse after motion. World tests compare positions,
active-contact counts and energies, and CCD TOI over two frames. Mixed tests
cover particles, crossing rods and triangle surfaces using Single EF, plus
pure rods/particles using the fallback. Candidate audits compare all four
PP/PE/PT/EE families, and TOI audits compare every candidate's value as well
as the global minimum. They do not establish a performance ranking.

## Wrecking ball benchmark

Build `wrecking_ball` in Release mode with examples enabled, then use the native
C++ scene for an alternating comparison:

```bash
python scripts/benchmark_single_ef_wrecking_ball.py --phase smoke --output-dir output/benchmark-runs/single-ef
python scripts/benchmark_single_ef_wrecking_ball.py --phase throughput --frames 300 --warmup 30 --repeats 5 --output-dir output/benchmark-runs/single-ef
python scripts/benchmark_single_ef_wrecking_ball.py --phase diagnostic --frames 300 --warmup 30 --output-dir output/benchmark-runs/single-ef
python scripts/benchmark_single_ef_wrecking_ball.py --phase audit --frames 300 --warmup 30 --output-dir output/benchmark-runs/single-ef
```

The scene keeps the C++ example's configuration and 574 affine bodies. It uses
`dt=0.01`, uniform density 1000, body stiffness 10 MPa, contact friction 0.01,
and contact resistance 20 GPa. This differs from the samples repository's Python
benchmark configuration. Benchmark mode skips OBJ export and state dumps. Frame
wall time includes advance, retrieve, and explicit synchronization; initialization
and result serialization are excluded. The first 30 of 300 frames are warmup.

Throughput runs disable nested timers and candidate audits. Separate diagnostic
runs enable synchronized Timer scopes. Do not add nested timer durations together.
The driver retains each run, per-frame times, iteration counts, source/input and
binary fingerprints, and sampled GPU load.

`UIPC_SINGLE_EF_AUDIT_FILE` enables a separate correctness run. Every actual
Single-EF detect call runs an independent baseline pipeline on the same state.
The audit compares full sorted PT/EE multisets, preserving multiplicity and
canonicalizing EE direction. It also compares active PP/PE/PT/EE multisets after
feature classification. Records distinguish Single-EF DCD and CCD queries from
topology fallback. The TOI audit associates every PT/EE value with its canonical
primitive pair, compares multisets and the global minimum with absolute
tolerance `1e-12` and relative tolerance `1e-9`, and restores the actual pipeline's
minimum after the shadow computation. Missing pairs, nonfinite values, or
values outside tolerance abort the run. An unset or empty variable leaves all
shadow computation and audit downloads disabled.
