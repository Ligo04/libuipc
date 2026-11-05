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

## Query strategy and common cache contract

Both `info_stackless_bvh` and `single_ef_bvh` now use `SimplexBVHQuery` in
`src/backends/cuda/collision_detection/filters/simplex_bvh_query.{h,cu}`.
The shared owner selects the cover, validates input contents, manages all four
candidate caches and fat-box/frame policy, and schedules the common judge.
See [the common query contract](simplex_bvh_queries.md) for inputs, IDs,
PP/PE handling, prefix checks, streams and validation status.

The [same-state benchmark](simplex_query_benchmark.md) captures every production
query and replays both methods on identical requests with an independent direct
oracle. It measures query cost separately from trajectory and solver work.

SingleEF retains its ownership-based candidate-generation strategy. One real
edge/face proxy tree supplies PT and EE pairs. Every surface point has one
query owner, and every edge has one leaf owner. Isolated points become
PT-only query proxies; standalone edges become EE-only leaves. Each leaf
contains its face and every owned edge. The primary query box includes only
selected EE edges and selected owned PT points; the reverse PT box includes
only outside-cover owned points. CIDs are recomputed from those contributors.

The primary list contains edges in S_E or proxies owning points in S_P. The
reverse list contains proxies owning outside-cover points and visits only
faces entirely inside S_P. An edge is selected only when both endpoint
contact IDs belong to S_E; a face is selected only when all three contact IDs
belong to S_P. This preserves coverage for mixed-CID primitives under the
original all-cross contact-mask predicates. EE pairs are emitted once in
canonical order.

Compact query IDs/counts change only with position-free inputs. A single
refresh-preparation launch grows individual primitive boxes and writes both
compact query-box lists and EF leaves. Proxy threads recompute the same
outward growth from immutable current bounds, avoiding cross-block producer
races. The two EF query chains use independent streams, query buffers and
per-stream CUB workspace. They read one immutable tree and join before it
changes.

EF refresh batches traversal, bounded typed count, combined PT/EE scan and
fill for all chunks on each chain. One aggregate status/count wait replaces
routine per-chunk waits. Truncated EF output emits no typed candidates; only
the failed traversal retries. Saved chunk bases prevent double reservation.
Typed discard growth refills every chunk of that chain from retained EF
output. The final typed lists concatenate the chain buffers, so persistent
storage includes the chain lists and combined cache.

PP/PE now share the common fat cache and judge in both methods. SingleEF
retains the auxiliary CP/E-tree queries for these pairs; their refresh
currently follows the EF refresh in the same cache transaction. Pure codim or
unsupported EF ownership uses the common direct-Info strategy when the
production caller supplies its codim binding. SingleEF-only standalone callers
retain their unsupported-topology return contract.

Both integrated methods use `step_position` endpoints and the same four-family
prefix/content validation, cross-frame reuse and three-stream judging. Their
active, pending-active and TOI consumers remain in `BVHSimplexPipeline`.
Whole-detect/full-advance CUDA Graph capture remains separate work.

## Shared query unification status (2026-11-05)

The common-owner implementation and caller migration passed the CUDA 13.5
build, 24-case candidate/active/BVH audit, 17-case normal suite, 8-case copy
fallback and 13-case explicit-API memcheck with zero errors. See
[the common query contract](simplex_bvh_queries.md#validation) for assertion
counts, source/runtime fingerprints and reproducible commands.
No new performance result is established for this unification.

The dated sections below describe the earlier migration/integration/follow-up
revisions and retain their original evidence. They do not validate the new
common-owner revision.

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
Previous 1000-frame results describe the previous version. The completed
[follow-up WB/EX88 cohort](../../agent_docs/performance/2026-11-05-singleef-query-followup-wb-ex88.md)
compares frozen `9ba6aa88` and `d4201933` runtimes, each with one 1000-frame
formal/profile process per scene. Formal frame time is higher in this group
while Newton/PCG work increases. Candidate-scope time per call is lower, but
call counts and trajectories differ. No stable whole-frame or equal-quality
speedup is established; repeatability is unassessed.
