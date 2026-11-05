# 2026-11-05 Same-state InfoStackless and SingleEF queries

- Status: accepted as one controlled query-layer observation; repeatability and
  whole-scene acceleration remain unestablished.
- Both methods use source `ba46a497dae2f26ea2ad9eb76c91f8cafd94f7c0` and the same
  native executable/backend. This is a strategy comparison, not two code revisions.
- The underlying shared-query implementation is `c70878f5`. Cover selection,
  fat-cache policy, metadata checks, prefix and judging are common to both methods.

## Question

Does SingleEF reduce completed-query cost when optimized InfoStackless and
SingleEF answer exactly the same production requests? Earlier whole-frame
comparisons changed Newton/PCG work and trajectories, so they could not isolate
this cost. The new capture/replay harness removes solver evolution from timing.

## Environment

| Field | Value |
|---|---|
| GPU / driver | RTX 5090, 590.48.01, 575 W power limit |
| CUDA / CCCL | CUDA 13.5.37, Toolkit CCCL 3.5 |
| OS | Linux 6.17.0-23-generic, 32 logical CPUs |
| Build | XMake Release, `sm_120`, RDC, legacy collision enabled, ccache disabled |
| Build limits | Global flock; 16 jobs; 32 GiB memory high/max; no swap |
| Worktree | Clean for the entire frozen cohort |
| Module loading | `CUDA_MODULE_LOADING` unset, preserved and recorded |
| Profile tool | Nsight Systems 2026.5.1.161, CUDA trace, no CPU sampling/context switches |

The native backend SHA-256 is
`16af3c904bc985f4d75276fbb37936a02b951442150bb53860e2c4ca6b746a4e`;
the CUDA test executable is
`bc89bbd0a627fad182e8953658e1d6a1adda13a30bcf78403d329c434c3efb28`.
Private read-only copies include native libraries and non-system dependencies.
System/driver files and profiler injection libraries are separately fingerprinted.
Loaded-file maps are checked against those inventories for each process.

## Workload and method

The existing native WB/sample-6 and EX88/sample-88 builders use hash-frozen
assets from the preceding shared-query cohort, sample snapshot
`b70f32294e31e9e3dcc12445b72fa9f4698ff5b1`, archived in `input-origin.json`.
The live submodules are uninitialized and
their indexed OIDs are recorded separately; they do not supply these assets.

One instrumented Info driver per scene generates every actual detect over
1000 physical frames. Request ordering, raw P/dx, alpha/skin, exact metadata
epochs and the topology-reuse flag are preserved. True frame/Newton labels are
independent of that flag: multiple frame-start requests can occur within one
physical frame. Recursive validation queries are excluded.

| Scene | Physical frames completed | All captured requests | Measured requests | Trace bytes | Metadata epochs |
|---|---:|---:|---:|---:|---:|
| WB | 1000 | 3964 | 3885 | 2,381,646,505 | 1 |
| EX88 | 1000 | 3320 | 3220 | 6,793,529,073 | 1 |

Replays retain all warmup history from source frames 1-30. Frames 31-1000 are
the measured 970-frame window. Every process owns one persistent method/policy
state, tree, prefix scratch and outputs. Natural mode uses normal containment
and invalidation. Forced-refresh mode sets `force_fresh=true` with warmed
storage and retained/refitted topology. It is not a rebuild-every-call baseline;
its skin history can differ, so subtracting its cost from natural cost would
not isolate a pure cache cost.

The primary clock surrounds production-prefix-inclusive `detect` with an empty
solver hook, then device completion across all streams. It includes query host
work, tree build/refit/traversal, judging and internal growth/retries. Decoding,
H2D upload, external sizing/binding, downloads, sorting and JSON output are
outside the span. Constructor and first detect are separate cold observations
after CUDA context/input preparation. This is query cost, not scene FPS.

The cohort has two capture, two exact verification, eight formal and eight
independent profile processes. Each scene runs Info then SingleEF in natural
mode, then Info then SingleEF in forced-refresh mode. There is one formal and
one independent profile per setting, with no automatic reruns or randomized
order. Launches wait for no CUDA compute client for 15 seconds and temperature
at most 45 C. GPU clocks/power/temperature/utilization/compute PIDs are sampled
once a second. Graphics and unsampled short activity remain possible.

## Correctness and safety

All 7284 captured requests pass separate four-state verification:
Info/SingleEF times natural/forced-refresh. Every PP/PE/EE/PT candidate multiset
matches an independent unpruned direct-query oracle. EE IDs alone are
canonicalized; duplicate multiplicity and all other family ID conventions are
preserved. Both trace readers validate clean footers and record counts.

Timed sweeps perform no candidate downloads/sorts. Their request identities,
four output counts, effective strategy and cumulative counter histories match
the exact verification ledgers. Cross-method analysis additionally finds zero
per-request refresh/reuse decision differences, stored-pair count differences,
or final output-count differences for both policies and scenes. All SingleEF
requests use EF-backed state, with zero direct-fallback requests. Cache hits
judge the stored lists without EF traversal.

| Gate | Result |
|---|---|
| Related GPU suite, stored-pair and active-set audits | 26 cases / 521,767 assertions passed |
| New cases under matching explicit-API memcheck | 9 / 941, zero errors |
| Host runner gates | 22 passed |
| Repository/archive contracts | 15 passed |
| Formatting / whitespace | clang-format 18 and diff checks passed |

The production capture regression first failed on a tetrahedron plus implicit
ground with `metadata index exceeds its table`. Ground contributes an unused
global vertex whose body ID is outside the simplex-only table. The codec now
checks IDs referenced by query topology and preserves unused metadata exactly.
The real case passes for both methods, as do unused-ID roundtrips and invalid
referenced-ID rejection. This changed diagnostic validation, not collision or
solver behavior. CMake/Python runtime rebuilds and extended/race/sync sanitizers
were not run.

The driver ledger separately certifies all 1000 physical frames, including any
with no queries. Both measured windows converged 970/970 with no Newton or
line-search limit. EX88 nevertheless has negative tet J on frames
56, 57, 58, 59, 60 and 75, with minimum -0.4201906443. WB reports no negative-J
frames. These are properties of the instrumented Info input trajectory, not a
physics-quality comparison between methods. Capture throughput is not formal
scene throughput.

## Formal results

Mean completed-query wall time, source frames 31-1000. Reductions use total
query time on equal request counts.

| Scene | Policy | Requests | Info ms/query | SingleEF ms/query | Time reduction | Info -> SingleEF query ms/source frame |
|---|---|---:|---:|---:|---:|---|
| WB | Natural | 3885 | 0.484023 | 0.368010 | 23.97% | 1.938588 -> 1.473936 |
| WB | Forced refresh | 3885 | 0.858544 | 0.671779 | 21.75% | 3.438602 -> 2.690578 |
| EX88 | Natural | 3220 | 1.198258 | 1.028402 | 14.18% | 3.977724 -> 3.413870 |
| EX88 | Forced refresh | 3220 | 2.509337 | 2.103729 | 16.16% | 8.329965 -> 6.983512 |

The reduction is concentrated in refresh requests. Cache-hit observations do
not improve, which is consistent with the shared judging implementation.

| Scene | Natural-mode bin | Matched requests | Info ms/query | SingleEF ms/query | Time change |
|---|---|---:|---:|---:|---:|
| WB | Refresh | 1824 | 0.930494 | 0.683102 | -26.59% |
| WB | Reuse | 2061 | 0.088893 | 0.089151 | +0.29% |
| EX88 | Refresh | 1222 | 2.714115 | 2.250081 | -17.10% |
| EX88 | Reuse | 1998 | 0.271143 | 0.281209 | +3.71% |

Individual tree-build counts are WB 254 -> 127 for both policies, EX88
182 -> 91 for natural and 250 -> 125 for forced refresh. These are individual
tree builds, not refresh transaction counts or kernel launches. Info's E/F
trees become one EF tree in these traces. EF expansion/retry work remains
included: measured WB has one EF retry/two typed refills; EX88 has three/one.
Counters named `cross_frame_reuses` follow topology-reuse policy events; physical
frame comparisons use the separately captured frame labels. Primary/reverse
query gauges use different units across strategies and are not compared as a
common traversal count.

Cold observations are mixed. Each total below is one constructor plus first
detect after context/input preparation, outside the measured window.

| Scene / policy | Info constructor / first detect / total ms | SingleEF constructor / first detect / total ms |
|---|---|---|
| WB natural | 4.230 / 104.880 / 109.109 | 4.298 / 109.674 / 113.972 |
| WB forced refresh | 4.203 / 103.115 / 107.317 | 5.852 / 153.648 / 159.500 |
| EX88 natural | 6.440 / 156.301 / 162.742 | 5.192 / 120.252 / 125.444 |
| EX88 forced refresh | 4.165 / 111.022 / 115.187 | 4.080 / 127.296 / 131.376 |

No cold-start advantage is established. Per-request p50/p95 are retained in
`validated-metrics.json`; they describe a varying workload, not repeated-run
confidence intervals. Profile cold timings are not pooled with formal data.

## Independent CUDA profiles

All eight nonempty native reports passed SHA-256 checks and contain CUDA
runtime and kernel activity on the recorded RTX 5090. Supported report-context,
activity/kernel facts, table description, bounded per-report SQL and report
doctor produced the archived evidence. Missing NVTX and GPU-metric data are
expected for this CUDA-only collection. All 20 loaded-file maps have complete
shared-library provenance, and the final frozen-source/runtime/input guard passed.

| Scene / policy | Info -> SingleEF kernel launches | Info -> SingleEF summed kernel ms | Duration comparison |
|---|---|---|---|
| WB natural | 86,247 -> 70,607 | 1522.764 -> 1063.609 | No sampled foreign compute |
| WB forced refresh | 112,401 -> 84,395 | 2888.874 -> 1933.901 | No sampled foreign compute |
| EX88 natural | 63,684 -> 61,563 | 4009.787 -> 2793.175 | No sampled foreign compute |
| EX88 forced refresh | 94,322 -> 92,941 | 8086.644 -> 5666.166 | Info profile contended; no clean duration comparison |

All eight formal runs sampled no foreign compute PID. During the independent
EX88 forced-refresh Info profile, a foreign Python GPU process appeared in two
one-second samples, growing from 12 to 2262 MiB. That profile is retained and
excluded from clean duration comparisons. It was not rerun. This does not alter
the separately collected formal query timings.

Nsight capture begins before the first measured query and ends after the last.
It encloses inter-request decoding, upload, binding and I/O gaps. All captured
API/memory-operation totals must not be attributed to the timed query span.
Summed kernel duration can include overlap and is not critical-path latency or
measured SM utilization. Nsight Systems does not establish occupancy, spill or
memory-stall causes here.

## Interpretation and decision

This controlled request distribution supports a query-layer cost reduction
for SingleEF relative to optimized InfoStackless. Shared Cover/fat-cache/prefix/
judge policies are enabled on both sides. Exact candidate agreement and matched
refresh decisions remove differing solver paths/cache-hit ratios as explanations
for the observed mean reduction. Halved tree-build counts and faster refresh
bins are structural evidence for the remaining EF strategy benefit.

The result does not establish stable general speedup, cold-start improvement,
or whole-scene/equal-quality simulation acceleration. Cache hits are slightly
slower in this one group. The Info-generated input distribution, EX88 inversions,
shared desktop activity, fixed method order and lack of repetitions remain
limits. No production sorting, solver changes or input tuning were introduced
to obtain the result. The live-shadow alternative was rejected because its
same-process ordering, shared GPU warmth and scheduling would weaken timing
independence.

## Reproduction and artifacts

The code and command contract are in
[the benchmark guide](../../docs/development/simplex_query_benchmark.md).
The native cases are
[`simplex_query_replay.cu`](../../apps/tests/backends/cuda/simplex_query_replay.cu)
and [`simplex_query_trace_world.cu`](../../apps/tests/backends/cuda/simplex_query_trace_world.cu).
The orchestration is
[`run_simplex_query_benchmark.py`](../../scripts/run_simplex_query_benchmark.py).

Ignored evidence roots:

- `output/single-ef-same-state-20261105/`: design/review, build/gate logs,
  `build-verification.json`, `formal-analysis.json`, `formal-review.md`, supported
  profile facts and the external activity/hash certificate `profile-analysis.json`.
- `output/benchmark-runs/simplex-query-same-state-cuda135-20261105/`: frozen
  native/dependency/input manifests, 1000-frame driver ledgers, binary traces,
  exact oracle ledgers, formal/profile request metrics, PID telemetry and reports.

Use the guide's command with a fresh output directory. The original cohort
refuses overwrite/retry. Query request rates are not scene FPS; historical
whole-frame measurements remain in their original reports.
