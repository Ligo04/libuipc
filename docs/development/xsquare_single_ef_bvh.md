# Single EF BVH on the X_square baseline

Base: `6e3386d12823c835c3206880b7758de952ccd339`.

Select the method before constructing the scene:

```python
config = Scene.default_config()
config["collision_detection"]["method"] = "single_ef_bvh"
```

`collision_detection/method` is an initialization-time, dimensionless string
selector. Its default is `info_stackless_bvh`; `single_ef_bvh` is always
available in CUDA builds, including builds with legacy collision disabled.

## Query and cache contract

One edge/face proxy tree supplies PT and EE candidates. Every surface vertex
has exactly one query owner, and every edge has exactly one leaf owner.
Isolated points become PT-only query proxies; standalone edges become EE-only
leaves. A leaf box contains its face and every owned edge. Each query chain
uses only the constituents it actually queries: selected EE edges and selected
PT points in the primary chain, outside-cover PT points in the reverse chain.
Their CIDs are recomputed from those contributors. Mixed scenes keep the existing auxiliary PP/PE queries. Pure
codimensional or unsupported topology uses the existing simplex queries.

Contact-cover queries use one EF tree with two compact query lists. The
primary list contains an edge in S_E or a proxy owning a point in S_P. The
reverse PT list contains proxies owning points outside S_P and visits only
faces entirely inside S_P. An edge belongs to S_E only when both endpoint
contact IDs belong to the cover; a face belongs to S_P only when all three
IDs do. This also covers mixed-CID primitives under the existing all-cross
mask predicates. EE pairs are emitted once and stored in canonical order.

Each refresh grows point, edge and triangle boxes individually before taking
proxy unions. Uniform skin is 1*d_hat. Cover primitives use 1*d_hat or
2*d_hat; other primitives use 0.01*d_hat or 1*d_hat. Leaving a smaller skin
raises that group's level, and leaving the next-lower skin while still
inside a thick cache keeps that level for the next frame. Bounds round
outwards. Every constituent primitive is checked against its own fat box;
proxy-union containment alone cannot justify clipping typed primitive pairs.

The two query chains have independent streams, query buffers and per-stream
CUB workspace. They read the same immutable tree and join before it changes.
Compact IDs/counts are position-free and uploaded only when the cover changes.
A single refresh-preparation launch grows all primitive boxes and writes both
compact query lists and EF leaves. Proxy threads recompute the same outward
growth from immutable current boxes, avoiding inter-block producer races.

Refresh enqueues traversal, a bounded typed count, one combined PT/EE scan and
fill for every chunk on its chain stream. Scratch is reused per chain, sized
for that chain's largest EF buffer. One aggregate status/count readback replaces
routine per-chunk waits. A truncated EF chunk emits no typed candidates; only
that traversal is retried after growth. Per-chain device totals and saved
chunk bases prevent double reservation. Typed-buffer discard growth refills
all of that chain's chunks from retained EF output without traversing again.
The final typed lists are concatenated for the existing shared judge, so
persistent typed storage includes both chain buffers and the combined cache.
Refresh expands fat EF
overlaps into position-free PT/EE lists, checking current contact/subscene
masks, shared vertices and body self-collision once. Cache reuse directly
judges those lists and never repeats ownership expansion.

`bvh_pair_judge` is shared with InfoStackless: whole-warp EE evaluation,
certified FP32 screen/gap/finish queues for swept queries, a one-pass tile
scan and kept-tile scatter. Validation atomics and geometry gates use device scratch; only final flags
and counts are published to mapped readbacks, retaining the copy fallback. Validation and both judge streams precede one host wait on the
normal cache-reuse path, and the detection hook runs before that wait.
A topology/metadata invalidation gate suppresses geometry reads until the
cache is rebuilt. Unsupported topology preserves the last published output
and returns to the existing simplex fallback.

GPU snapshots compare topology and every position-free metadata array/table
by content, including edits in the same allocation. Changes invalidate the
cover and typed cache. Frame boundaries alone do not invalidate the cache;
every retained use checks complete current swept primitive bounds. Actual
saved fat/shrink levels remain unchanged until a refresh, independently from
the next refresh's desired skin policy. Every eighth new-frame request
(`!reuse_bvh_topology`, including initialization) schedules a topology rebuild.
That request stays pending until the next actual refresh, independently of
cache refreshes within a frame. Output capacity is reserved from the
stored pair counts before scan/scatter.

`BVHSimplexPipeline` shares active filtering, two-phase pending active sets,
TOI filtering and the existing detect-skin contract between both methods.
SingleEF prefix endpoints use `step_position`, matching accepted line-search
positions even when an FMA would retain a cancelled product residual. The
legacy prefix keeps its original default. The integrated caller passes its
unlaunched generic trajectory prefix to SingleEF. Its vertex launch initializes
device controls; primitive producers compare new BID/CID and topology with
snapshots and check fat containment. An extra block range in the same second
launch compares independent metadata/table contents, even for empty geometry
or tables larger than the vertex range. This removes the separate SingleEF
validation launch. Reused prefix descriptors have every attached snapshot and
fat pointer cleared before rebinding. Standalone query callers keep the full
validation-kernel fallback. The all-filter capability check
remains in effect. The typed pair cache and the public trajectory candidate skin are separate layers.

The baseline's CUDA Graph support covers the linear solver. Collision detect
still has host readbacks and exceptional host capacity growth, as in the
baseline. Whole-detect/full-advance capture requires a separately prepared,
fixed-capacity path and is not provided by this method.

## Validation

`single_ef_bvh.cu` compares candidate multisets against fresh independent PT/EE
BVHs and exhaustive CPU DCD enumeration. Coverage includes mixed/codimensional
geometry, empty and invalid topology, more than 64 nearby faces, output/EF
overflow, unequal chunks, alpha changes, skin reuse and escape, masks, in-place
ownership changes, and floating-point cancellation. `single_ef_bvh_world.cpp`
compares positions, active contact counts and energies, and CCD TOI with the
baseline method in actual worlds.

Runtime checks can additionally enable `UIPC_CHECK_STORED_PAIRS=1` and
`UIPC_CHECK_ACTIVE_SET_REUSE=1`. The first bypasses EF cache reuse and verifies
the candidate multiset; the second verifies inherited frame-start active-set
reuse.

Initial migration validation on 2026-11-05 used an RTX 5090, driver 590.48.01, CUDA 13.4.92 and
XMake 3.1.2 in Release mode:

- Backend and CUDA device-link build passed. The initial 16-job build reached
  the 32 GiB cgroup limit; the successful retry used 4 jobs under the same
  `MemoryHigh=32G`, `MemoryMax=32G`, `MemorySwapMax=0` limits.
- The related GPU suite passed 15 cases and 842,588 assertions with both
  runtime validation modes enabled. This includes fresh candidate multiset,
  active-set, real-world energy/position/TOI, prefix, Morton, refit/order reuse
  and graph checks.
- With both extra validation modes unset, all 9 SingleEF cases and 519,596
  assertions passed, including real worlds using the persistent cache.
- Repository contract checks passed 13 cases. Clang-format 18, diff whitespace
  and Python schema-test syntax checks passed.
- CUDA 13.4 Compute Sanitizer passed 4 focused cases and 518,102 assertions
  with `--report-api-errors explicit`, reporting zero memory/API errors. Its
  default extended API mode also passed all 4 cases but emitted nine duplicate
  header-kernel registration diagnostics from the test executable and backend
  library. Those diagnostics remain recorded, so the default sanitizer mode
  is not claimed clean. The older system CUDA 12.8 sanitizer was incompatible
  with this CUDA 13.4 binary and was not used for the final memory conclusion.

The cancellation regression failed before the `step_position` change and
passed after rebuilding. CMake source ownership was checked; a CMake build and
Python runtime rebuild were not performed. This validation establishes no
scene-specific performance result. Logs and binary/source fingerprints are in
`output/single-ef-migration-20261105/`.

## Query integration validation

The query-side integration is based on migration commit `4539bdff`. On
2026-11-05 the RTX 5090 / driver 590.48.01 Release build used Toolkit 13.5.37
at `/home/ligo/.local/cuda-13.5.0`, including its CUB, Thrust and libcu++ 3.5.
XMake used 16 jobs with MemoryHigh=32G, MemoryMax=32G and MemorySwapMax=0.
The backend, device link and native test executable built successfully.

- Both candidate/active-set audit modes: 17 GPU cases, 842,782 assertions.
- Normal persistent-cache path: 11 GPU cases, 519,790 assertions.
- Forced readback-copy fallback: 2 focused cases, 194 assertions.
- Matching CUDA 13.5 Compute Sanitizer, explicit API mode: 6 focused cases,
  518,296 assertions, zero reported memory/API errors.
- Repository contracts: 13 cases. Clang-format 18 and diff checks passed.

New regressions cover mixed contact IDs, both query directions, canonical EE,
unequal/overflowing chunks, independent metadata edits, zero enabled contacts,
FP32 queues against fresh FP64 traversal, per-primitive containment, lower-skin
hysteresis during successful reuse, and frame-based topology retention.
The old EF expansion path is removed from normal cache reuse. The existing
active/pending-active and TOI functions are unchanged by this integration.

The CUDA 13.5 default extended API mode, racecheck, synccheck, a CMake build,
and a Python runtime rebuild were not run. Do not extend the explicit-mode
memcheck result to those gates. The follow-up device-control correction passed the same 17 audited, 11 normal,
2 copy-fallback and 6 memcheck cases after rebuilding under the global build
lock. Current build logs, source/runtime hashes,
commands and test results are archived in
`output/single-ef-query-optimization-20261105/`. Earlier WB/EX88 measurements
remain in `agent_docs/performance/2026-11-05-xsquare-wb-ex88.md`; they use the
previous CUDA 13.4 implementation and do not establish current performance.

The source-matched CUDA 13.5 WB/EX88 cohort at `9ba6aa88` is recorded in
[the query-integration performance report](../../agent_docs/performance/2026-11-05-xsquare-query-cuda135-wb-ex88.md).
Its single-episode timing reductions are observations with changed solver
work/trajectories and EX88 negative-J differences, not equal-quality speedup.


## Follow-up validation: bounded refresh and cross-frame reuse

The four follow-up optimizations were checked on 2026-11-05 with the same
RTX 5090 / CUDA 13.5.37 / Toolkit CCCL 3.5 environment. Configuration explicitly
selected CUDA and disabled compiler caching. Every configuration/build command
held the blocking global build lock and ran in the 16-job, 32 GiB, no-swap
systemd scope. The actual output directory is `build/linux/x86_64/release`;
XMake configuration reset the former output-directory setting.

- CUDA backend build and native test build passed. An initial test compile
  failed on a missing new test-local prefix declaration; the corrected build
  passed. The invalid initial backend target name is retained in the logs.
- Audited GPU suite: 21 cases / 842,883 assertions.
- Normal persistent-cache mode: 14 cases / 519,887 assertions.
- Forced copy readbacks: 6 cases / 295 assertions.
- Matching explicit-API memcheck: 10 cases / 518,397 assertions, zero errors.
- Repository contracts: 13 cases. Clang-format 18 and diff checks passed.

New cases cover selective bounds on long owner edges, batched unequal chunks,
EF overflow followed by selective traversal retry, typed-buffer discard growth
and complete refill, static cross-frame reuse, lower-skin history with retained
actual boxes, pending topology rebuilding, generated primitive metadata changes,
raw tables larger than the vertex grid, metadata-only empty prefixes and reuse
of the same prefix POD after topology growth. Existing candidate/active/TOI and
real-world checks continue to pass. Standalone and fused-prefix entries are
both exercised.

`Statistics::ef_pairs` reports the last refresh's EF overlap count. Refresh
waits, failed EF chunks, typed chain refills, cross-frame reuses and fused-prefix
calls are cumulative diagnostics. Skin statistics describe the actual saved
fat boxes, not a next-frame request to shrink them.

Evidence is in `output/single-ef-query-followup-20261105/verification.json` and
its build/GPU/sanitizer logs. `memcheck-old-output-directory.log` accidentally
checked the unchanged historical output and is excluded; the current ten-case
memcheck was rerun against the verified actual output directory. CMake/Python
rebuilds, extended API sanitizer, racecheck and synccheck remain unverified.
Previous 1000-frame results describe the previous version; follow-up performance
is recorded separately when available.
