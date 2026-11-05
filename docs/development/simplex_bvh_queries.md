# Shared simplex BVH query lifecycle

`info_stackless_bvh` and `single_ef_bvh` share one `SimplexBVHQuery` owner.
Their tree/traversal and candidate-generation strategies remain distinct.
`collision_detection/method` is an initialization-time, dimensionless string
selector; its default remains `info_stackless_bvh`.

## Ownership and entry

`BVHSimplexPipeline` owns current primitive bounds/metadata, candidate outputs
and the existing active/pending-active/TOI consumers. It constructs an
unlaunched generic `TrajectoryPrefix`, original predicates and a `CodimQueries`
binding, then calls `SimplexBVHQuery::detect`. It has no cover, stored-pair,
fat-skin or judge lifecycle of its own.

The common implementation lives in
`src/backends/cuda/collision_detection/filters/simplex_bvh_query.{h,cu}`.
It owns content snapshots, cover membership, saved fat boxes, desired skin
levels, pending topology rebuilding, typed caches, three side streams, judge
scratch and final readbacks. Private EF and direct-Info refresh routines use
that state. `bvh_pair_judge.{h,cu}` provides the shared geometry kernels and
scan/scatter. The previous `SingleEFBVHQuery` class/source is removed.

## Primitive and pair IDs

The generic prefix has four families: codim points at slot 0, surface points
at 1, edges at 2 and triangles at 3. Every family writes its current AABB,
body ID and contact ID.

Cached and published pairs use uncompacted primitive-family ordinals, not
world vertex IDs:

| Pair kind | First index | Second index |
|---|---|---|
| PP | Surface-point ordinal | Codim-point ordinal |
| PE | Codim-point ordinal | Edge ordinal |
| EE | Smaller edge ordinal | Larger edge ordinal |
| PT | Surface-point ordinal | Triangle ordinal |

The common judge list/count order is PP, PE, EE, PT; its fifth legacy slot is
empty. Active and TOI consumers use the named buffers and retain their
existing PP/PE/PT/EE layout.

Info refresh initially generates cover-local traversal pairs. It maps selected
EE query IDs back to edges and canonicalizes the pair; primary PT query IDs
map to surface points; reverse PT query and leaf IDs map to surface points and
triangles. Primary/reverse PT lists merge once during refresh. PP/PE already
have the required family IDs. Reuse judging needs no cover remap.

## Cover and cache policy

PT and EE covers use the shared weighted contact-graph selection. Primitive
membership follows all relevant endpoint IDs: both edge endpoints for EE,
all triangle vertices for reverse PT. The original all-cross mask predicates
justify mixed-CID coverage. A family uses full queries when its cover is not
usable, including the existing 60% selection threshold and unsuitable masks.
SingleEF creates tight primary/reverse ownership queries from this membership;
Info creates compact primitive query lists and a cover-triangle tree.

Refresh uses position-free contact/subscene, shared-vertex and body rules to
build four typed caches. Both methods preserve PP/PE caching. The EF strategy
uses the auxiliary CP/E-tree queries for PP/PE instead of expanding them from
EF overlaps. Pure codim or unsupported EF ownership switches to direct Info
when the caller supplies the production codim binding. SingleEF-only standalone
callers without that binding retain false-return/no-published-output behavior
for unsupported topology.

Every refresh saves individual primitive fat boxes with outward rounding.
Uniform skin is 1*d_hat; cover primitives use 1 or 2*d_hat; other primitives
use 0.01 or 1*d_hat. Codim points use uniform skin. Each retained use checks
complete current swept bounds against the corresponding individual fat box.
Proxy-union containment alone is insufficient.

Frame boundaries update desired next-refresh skins and lower-skin history.
They do not invalidate a contained cache. Actual saved fat/shrink arrays remain
unchanged until refresh. Every eighth frame-start request schedules topology
rebuilding; the request remains pending until an actual refresh. Content,
shape, selected method and relevant query-policy changes invalidate the cache.

## Prefix validation and endpoint rounding

All topology and position-free pruning inputs are compared by content,
including in-place edits. Independent PT/EE/PP/PE/node bindings remain distinct.
The inventory includes PP dimensions, contact/subscene IDs and tables,
body-self-collision inputs, codim topology and generated primitive BIDs/CIDs.

The integrated prefix runs in two launches. The vertex launch initializes
device controls and prepares endpoints/FP32 inputs. The primitive launch
compares each new topology/BID/CID value at its producer, checks fat containment
and processes independent metadata in extra block ranges. Metadata work may
exceed the vertex count or exist with empty geometry. The descriptor capacity
is 32, with a compile-time inventory bound. Attached snapshot/fat pointers are
cleared before rebinding a reused prefix.

Both common production modes use `step_position` endpoints, matching actual
accepted line-search advancement even under cancellation/FMA differences.
The generic standalone `TrajectoryPrefix` default remains unchanged for its
original callers/tests. Standalone query callers without a prefix keep the
complete separate validation-kernel entry. Typed fat caches and the outer
trajectory candidate-skin contract remain separate layers.

## Streams, capacity and publication

Direct Info refresh uses three chains: CP/E queries, primary PT and reverse PT.
SingleEF refresh retains two concurrent batched EF chains. Its auxiliary PP/PE
refresh currently follows the EF refresh; these two refresh phases are not
claimed to overlap. All use the same common cover/cache/frame state.

Reuse judging schedules EE, PT and PP/PE on three independent streams, then
joins before one shared compaction. Whole-warp EE evaluation and certified FP32
screen/gap/finish retain FP64 evaluation for surviving swept pairs. Validation
and geometry gates use device allocations; only final flags/counts are
published to mapped readbacks, with the copy fallback preserved. The detection
hook runs before the final count wait.

Concurrent streams have separate persistent CUB workspace. Tree/cache mutation,
capacity growth and exception cleanup follow stream joins. EF truncation emits
no typed candidates; only failed chunks retry. Per-chain totals and saved bases
avoid double reservations. Typed discard growth refills every contributing
chunk. Output capacity is prepared before shared scatter. `force_fresh` and
`cache_skin_scale` apply to both methods; `max_chunk_edges` controls only EF
expansion and does not invalidate a direct-Info cache.

The linear-solver CUDA Graph configuration is retained. Collision detection
still has host decisions/readbacks and exceptional growth; whole-detect or
full-advance capture is not supplied by this change.

## Runtime audits

`UIPC_CHECK_STORED_PAIRS=1` repeats detection with a fresh common query and
compares candidate multisets. Its audit call does not count as a new frame.
`UIPC_CHECK_ACTIVE_SET_REUSE=1` retains the active-set reuse audit.
`UIPC_BVH_QUERY_CHECK=<file>` is migrated to common direct refresh: it compares
cooperative query output with the per-lane reference, preserves the output used
by the simulation and records JSON audit evidence. None of these audit modes
belongs in formal throughput measurements.

## Validation

Verified on RTX 5090 with CUDA 13.5 / Toolkit CCCL 3.5 and XMake Release,
using the global build lock, 16 jobs and a 32 GiB/no-swap scope. The native backend
and test executable are in `build/linux/x86_64/release`. Compiled source is
the source-file SHA-256 inventory in `verification.json`. The runtime fingerprints and reproducible commands are recorded in
`output/single-ef-unification-20261105/verification.json` and `final-checks.json`.

| Gate | Result |
|---|---|
| Focused shared-policy GPU checks | 7 cases / 1,233 assertions |
| Candidate, active-set and BVH audits | 24 cases / 843,802 assertions |
| Normal SingleEF and world compatibility | 17 cases / 520,806 assertions |
| Forced copy readbacks | 8 cases / 1,260 assertions |
| Matching Compute Sanitizer, explicit API memcheck | 13 cases / 519,316 assertions, 0 errors |
| Cooperative versus per-lane direct-query audit | 569 records, all pair multisets equal |
| Repository contracts | 13 cases passed |
| clang-format 18 and diff checks | Passed |

Coverage includes both methods, mixed CID and codim inputs, cross-frame reuse,
independent metadata edits, method/fallback transitions, context removal/rebind,
individual fat-box escape, deferred rebuilding, EF/typed overflow and reused
prefixes. Cancellation endpoints match accepted stepping for both methods.
The final source/runtime fingerprints were unchanged across the GPU checks.

Backend SHA-256: `a0d64cac57d83d8c8b0d02078de4851d4dd9a5f3d322ce61e1307301e19ccb33`.
Test executable SHA-256: `b922d30266c42073ff3d435bebaf813abda07ea488687e47d234151c66ea160f`.

Initial validation incidents are retained in the evidence directory: a const
oracle-helper compile error, a direct-launch spdlog search-path omission, and
two Info counter expectations that incorrectly applied EF-only chunk policy.
They were corrected before the final gates above. A pre-format verification
snapshot is retained separately; final gates use the final rebuilt runtime.

Evidence directory: `output/single-ef-unification-20261105/`.
CMake/Python runtime rebuilds, extended API-mode sanitizer, racecheck and synccheck
were not run. No new performance measurement or speedup claim is attached to
this structural unification. Earlier WB/EX88 reports remain dated evidence for
their own frozen runtimes.
