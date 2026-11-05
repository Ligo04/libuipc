# Same-state simplex query benchmark

This benchmark compares current `InfoStackless` and `SingleEF` query strategies
on identical production requests. It separates query cost from Newton/PCG work
and independently evolving simulation trajectories. The shared Cover, fat-cache,
metadata and judging policies are described in
[the implementation contract](simplex_bvh_queries.md).

The driver supplies every actual detect over 1000 physical WB or EX88 frames.
The query benchmark retains request order, all Newton/line-search retries,
alpha/skin, metadata contents and exact topology-reuse requests. A physical frame
is recorded separately: a repeated full-step/truncated-alpha detect can issue
more than one frame-start request within a frame. Source frames 1-30 warm state;
31-1000 define the measured window, including any zero-query frames.

## Run a frozen cohort

The recorded cohort at `ba46a497` completed WB/EX88 1000-frame input captures,
four-state oracle verification of all 7,284 requests, eight formal and eight
independent profile processes. Natural query means are WB 0.484023 -> 0.368010
ms and EX88 1.198258 -> 1.028402 ms (Info -> SingleEF). Forced-refresh
reductions are 21.75% / 16.16%. Refresh/reuse decisions and stored/output counts
match; effective EF fallback is zero. Natural reuse bins are slightly slower,
cold observations are mixed, and repeatability remains unassessed. The contended
EX88 forced-refresh Info profile is retained separately from clean formal data.
See [the complete evidence](../../agent_docs/performance/2026-11-05-singleef-same-state-wb-ex88.md)
for the query-only scope, all results, profile activity certificates and limits.

Build and validate the native CUDA test executable first, following the global
build lock, CUDA 13.5, 16-job and 32 GiB/no-swap rules. The runner never builds,
installs a package, edits a scene, or calls `xmake run`. Commit source changes
before freezing a cohort. Supply a build-provenance JSON containing
`source_commit`, source-file SHA-256 values, native backend/test SHA-256 values
and the matching build/test evidence.

On this machine, use the authoritative Python explicitly:

```sh
/opt/nvidia/nsight-systems/2026.5.1/target-linux-x64/python/bin/python \
  scripts/run_simplex_query_benchmark.py \
  output/benchmark-runs/simplex-query-same-state-cuda135-20261105 \
  --build-provenance output/single-ef-same-state-20261105/build-verification.json \
  --profile-prefix-json output/single-ef-same-state-20261105/nsys-prefix.json \
  --dry-run
```

Remove `--dry-run` to execute the one planned cohort. Dry run prints arguments
and environments without creating a cohort or querying GPU. Default inputs are
the hash-frozen sample assets from the prior shared-query WB/EX88 cohort;
`--inputs` selects another frozen directory containing `assets/`. The original
sample snapshot is preserved and checked when it is available beside the inputs.
The live samples checkout is not modified.

Default execution has 20 serial processes:

1. Two input-generation drivers, one 1000-frame Info driver per scene. These
   captures are instrumented diagnostics and supply no formal scene-throughput
   result for the query experiment.
2. Two exact verification sweeps. Four persistent states (Info/SingleEF,
   natural/refresh) each match an independent unpruned direct-query oracle.
   PP/PE/EE/PT candidate vectors are CPU-sorted outside timing; only EE is
   canonicalized, and duplicate multiplicity is preserved.
3. Eight formal replay processes, one per scene/method/cache-policy combination.
4. Eight independent CUDA-only Nsight profile processes on the same traces.
   Profile timings are excluded from formal metrics.

Use `--phase capture`, then `verify`, `formal`, or `profile` to execute phases
explicitly. Later phases reuse the frozen manifest and refuse any existing
attempt label, including failed or interrupted attempts. A fresh directory is
required for a new cohort; no automatic retry or extra repetition occurs.
`--phase analyze` reads archived metrics without launching GPU work.
A failed driver, incomplete trace, oracle disagreement, changed input/runtime,
or mismatched replay ledger prevents formal eligibility.

Before every launch, the runner waits for no CUDA compute client for at least
15 seconds and GPU temperature <=45 C. It samples GPU clocks, power, temperature,
utilization, memory and compute PIDs once a second during execution. An own
exiting PID reported as `[No data]` remains an own PID; a foreign PID is retained
as interference evidence. Other jobs are not terminated. Shared graphics and
short unsampled activity remain possible. The default is one formal and one
profile run per setting; run-to-run repeatability is not established.

## Capture boundary

`UIPC_SIMPLEX_QUERY_TRACE_OUTPUT` enables one writer owned by the active
`BVHSimplexPipeline`. It snapshots the production `DetectInfo` read-only views
before normal detection and leaves the original before-readback hook intact.
Audit recursion with `direct_queries` is excluded. Disabled capture creates no
writer or diagnostic GPU work.

The stream contains exact P/dx payloads for every request and content-deduplicated
metadata epochs: topology/order, dimensions, BID/contact/subscene inputs, body
self-collision, d_hat/thickness and both mask matrices. It represents production's
shared predicate bindings, not arbitrary independently bound standalone tables.
Global arrays also contain implicit-geometry vertices outside simplex queries.
The codec checks table IDs for vertices referenced by simplex topology and keeps
unused global metadata exactly as supplied.
The codec writes scalar components, validates its version and lengths, and
requires a clean footer. One metadata epoch is bounded to 256 MiB before
allocation; there is no arbitrary whole-trace size cap. Dynamic P/dx counts are
bounded by the current metadata vertex-array capacities. WB/EX88 epochs are far
below this diagnostic-format limit. Interrupted/truncated or incompatible traces
are rejected.

A clean trace footer cannot prove that frames with no queries completed. The
runner separately checks the driver's result and its contiguous 1-1000 frame
ledger, including completed/valid/finite flags. It preserves nonlinear convergence,
limits and negative-tet-J observations as driver quality metadata. Those flags
are not converted into a query-layer physics-quality or FPS claim.

## Measurement contract

Each replay process owns one persistent query/tree and all output/prefix buffers.
It never switches a query's method or changes its tree address. Both methods
consume the same immutable trace checksum and request identities.

- `natural` retains the complete cache history and normal containment/metadata
  invalidation. It is not a stationary cache-hit-only microbenchmark.
- `refresh` sets `force_fresh=true` with persistent warmed storage. Topology
  retention/refit still applies; this is not a rebuild-every-call experiment.
- Constructor and first detect are separately timed cold observations after
  CUDA context/input preparation. They form one sample per process, not a
  repeatability study. `CUDA_MODULE_LOADING` is preserved and recorded;
  profiler injection can alter cold costs, so profile cold metrics are not
  pooled with the formal observations.

Timed work is production-prefix-inclusive `detect` with an empty solver hook,
followed by device completion across all streams. File decoding, H2D upload,
external buffer sizing/binding, output download/sort/comparison and JSON output
are outside the span. There is no solver evolution in replay. Profile capture
uses the approved absolute `nsys profile --trace=cuda --sample=none
--cpuctxsw=none --capture-range=cudaProfilerApi --capture-range-end=stop
--stats=false` prefix, with native profiler APIs bracketing measured source
frames. No NVTX dependency is introduced. That CUDA trace also sees uploads and
preparation within the captured window; its activities must not all be treated
as timed-query work.

Before a profile process can be marked validated, its expected `.nsys-rep`
file must exist and be nonempty. The attempt record preserves its absolute path,
byte size, SHA-256 and collection status; a missing or empty report fails the
attempt without an automatic rerun. `report_present_nonempty` certifies file
collection only. Usable CUDA activity remains `pending` until checked through
supported Nsight report tools, independently from the native query-time ledger.

Every timed request must match the verification request/frame/Newton/epoch,
alpha/skin/new-frame identity, four output counts, effective strategy and
cumulative counter history. `single_ef_active` identifies EF-backed cached state
versus the direct fallback. Combine it with the refresh delta to identify EF
traversal: a cache hit performs judging without traversing a tree. A selected
SingleEF method or zero EF pairs alone cannot identify the effective strategy;
fallback requests are counted explicitly.

## Evidence and interpretation

The cohort archives copied read-only native libraries/executable and non-system
shared dependencies. System and driver libraries remain installed and are
fingerprinted. All loaded native files must match the frozen runtime; Nsight
injection libraries are fingerprinted separately and cannot replace a private
`libuipc_*` library. Sources, build proof, assets, traces, commands, flags, Python,
module-loading environment and before/after loaded-file maps are recorded.
Regular indexed files retain SHA-256 checks. Gitlinks are inventoried separately
by indexed mode and pinned commit OID, without opening initialized directories
or uninitialized submodule paths as files. Each guard also compares the recorded
submodule checkout status.

`requests.jsonl` is the authoritative per-request metric ledger. `summary.json`
reports completed query cost per request and total query milliseconds per source
frame, with p50/p95, first-detect/constructor metrics, refresh/reuse bins and
counter deltas. Query requests per second are not scene FPS. Replay has no
Newton/PCG iteration or simulation-convergence metric; the driver ledger is
separate. Profile and formal observations are never pooled.

Cumulative fields include refreshes, reuses, individual tree builds, waits,
retries/refills, cross-frame reuse and prefix calls. Query counts, stored-pair
counts, EF pair count and skin/cover settings are snapshots. They remain in the
per-request ledger and are not summed across reuse calls as traversal work.
EF-only values are N/A for Info.

A valid result can establish query-layer behavior and the observed cost on this
fixed request distribution. It cannot by itself establish a stable general
speedup, uninstrumented whole-scene acceleration or equal-quality long-term
simulation. Keep any slower or inconclusive result alongside faster observations.
The Info-generated trace also bounds the trajectory distribution tested.

Run ordinary native correctness cases with `[simplex_query_replay]~[.]`.
The asset-heavy verifier/replayer uses `[.simplex_query_replay]`; the explicit
hidden-tag exclusion prevents a correctness sweep from invoking that driver.
The production capture smoke case also checks footer ownership and excludes
recursive candidate audits from the captured request stream.
Host-only orchestration gate tests are in
[`test_run_simplex_query_benchmark.py`](../../scripts/tests/test_run_simplex_query_benchmark.py).

## Harness validation (2026-11-05)

CUDA 13.5 / Toolkit CCCL 3.5 Release backend and native tests built under the
global build lock with 16 jobs and a 32 GiB/no-swap scope.

| Gate | Result |
|---|---|
| Related GPU cases with stored-pair and active-set audits | 26 cases, 521,767 assertions passed |
| New capture/replay cases under matching explicit-API memcheck | 9 cases, 941 assertions passed, zero errors |
| Host-only runner gates | 22 passed |
| Repository and archive contracts | 15 passed |
| clang-format 18 and diff whitespace | Passed |

The real tetrahedron/ground capture regression failed before the codec fix with
`metadata index exceeds its table`. The implicit ground adds a global vertex
whose body ID does not address the simplex table. Both production methods now
pass, alongside raw unused-ID roundtrips and invalid referenced-ID rejection.
Source/native hashes and commands are recorded in
`output/single-ef-same-state-20261105/build-verification.json`; logs and the
before-fix debugger evidence are beside it. CMake/Python runtime rebuilds and
extended/race/sync sanitizer modes were not run. These harness gates do not
establish a performance improvement.
