# 2026-11-05 — SingleEF query follow-up on WB and EX88

- Status: all four requested changes are implemented and GPU-validated. This single group does not establish a whole-frame speedup. Observed formal times increased in both scenes, with different solver work and trajectories.
- Baseline code: `9ba6aa880137fe50efd01e61641b11fceb396d52`, frozen before rebuilding.
- Follow-up code: `d42019339cf0bba65ce51342803f8675962433ea`, committed and pushed on `codex/xsquare-single-ef-bvh` before measurement.
- Both sides select `single_ef_bvh`; all initial states and complete configurations match.
- Baseline CUDA backend SHA-256: `9eee70e954a9f14cdd5ec339d213049ee325cff691281989a232c662d9ca728c`.
- Follow-up CUDA backend SHA-256: `76801b7d4d2845c505b83a3a52fb7acfdc93f831dc6c47deb004ff224d138e57`.

## Implementation

1. Primary query boxes contain only selected EE edges and selected owned PT points. Reverse PT boxes contain only outside-cover owned points. CIDs are recomputed from those constituents. One actual EF tree is retained.
2. Compact query IDs/counts are prepared only when topology/cover metadata changes. Refresh streams enqueue all chunks with per-chain reusable scratch, one combined PT/EE scan and device totals. One aggregate refresh-status wait replaces routine per-chunk waits. Truncated EF chunks emit no typed output and retry selectively. Typed discard growth refills all affected-chain chunks at their saved bases without traversing or reserving twice.
3. Frame boundaries alone no longer invalidate fat caches. Exact topology/metadata snapshots and full per-primitive swept containment still gate every reuse. Saved fat/shrink values remain distinct from desired future skin policy. Eight-frame topology requests stay pending until an actual refresh.
4. SingleEF validation runs inside the generic two-launch trajectory prefix. Primitive producers compare newly generated BID/CID and topology; independent metadata/table comparisons use an additional block range. A single refresh-preparation launch grows P/E/F boxes and produces compact queries/leaves from immutable inputs. Standalone queries retain full validation fallback.

Per-chain typed outputs are concatenated into the existing shared judge cache, increasing persistent typed-pair storage. Scratch scales with each chain's largest chunk, rather than all chunk capacities. Whole-detect/FullAdvance Graph capture remains separate. The method selector default and solver/scene parameters are unchanged.

## Environment and protocol

RTX 5090, driver 590.48.01, GCC 11.5.0, XMake 3.1.2 Release/native SM 120. CUDA Toolkit 13.5.37 at `/home/ligo/.local/cuda-13.5.0`, with its CUB/Thrust/libcu++ 3.5. Configuration disabled compiler caching. Builds held the global blocking flock and used 16 jobs with MemoryHigh=32G, MemoryMax=32G, MemorySwapMax=0. Actual new build output is `build/linux/x86_64/release`.

Core, geometry, constitution, IO, sanity-check and none-backend native hashes match across frozen runtimes. CUDA backend and native test executable differ. The benchmark source is unchanged between the two code revisions. Each process verified the intended frozen backend through loaded-file hashes and the selected filter through `systems.json`.

One formal process and one separate synchronous-Timer profile process per side per scene, eight processes total. Each completed 1000 frames. Frames 1–30 are warmup; all tables use 31–1000 (970 frames). Primary time is `advance(); sync();`; secondary also includes `retrieve(); sync();`. Stats and geometry quality observations are outside those intervals. Formal Timer/profiler is disabled. Profile scopes are inclusive and excluded from formal throughput. Default linear-solver Graph settings are retained.

Inputs reuse the hash-frozen sample sources/assets at samples `b70f32294e31e9e3dcc12445b72fa9f4698ff5b1`: WB sample 6 (560 cubes, 13 links, one ball, dt=0.02); EX88 sample 88 (two FEM bunny2 meshes, E=1e4, MAS, 4225-vertex cloth, dt=0.01). Materials, contact settings, tolerances and semi-implicit settings match the [prior scene specification](2026-11-05-xsquare-wb-ex88.md).

Runs were serial. Before each launch, the runner required no CUDA compute client for at least 15 seconds and temperature <=45 C. One-second telemetry sampled no foreign CUDA compute client in any process. Shared desktop graphics and short unsampled activity cannot be excluded. Run-to-run variation and repeatability have not been assessed.

## Formal observations

| Scene | Version | Advance ms | FPS | Newton total / per frame | PCG total / per frame | Completed / converged |
|---|---|---:|---:|---:|---:|---:|
| WB | before | 8.939 | 111.873 | 3864 / 3.984 | 95170 / 98.113 | 970 / 970 of 970 |
| WB | after | 9.188 | 108.832 | 3980 / 4.103 | 96365 / 99.345 | 970 / 970 of 970 |
| EX88 | before | 30.256 | 33.051 | 3233 / 3.333 | 140850 / 145.206 | 970 / 970 of 970 |
| EX88 | after | 31.411 | 31.836 | 3574 / 3.685 | 152800 / 157.526 | 970 / 970 of 970 |

Observed advance-time increases are 2.79% for WB and 3.82% for EX88; throughput decreases are 2.72% and 3.68%. These single-episode observations do not establish a stable regression or an equal-work/equal-quality comparison.

WB: Newton work increases 3.00%, PCG work 1.26%. Including retrieve, time is 9.084 -> 9.341 ms/frame.

EX88: Newton work increases 10.55%, PCG work 8.48%. Including retrieve, time is 30.402 -> 31.562 ms/frame.

All measured frames converged at the nonlinear pipeline level; no Newton/line-search limit was hit. Frame convergence does not certify every individual PCG residual. All recorded states were finite, with no observed ground crossing.

EX88 formal baseline has five negative-J frames (56–60), min J=-0.200037; follow-up has four (57–60), min J=-0.156331. These are not strictly non-inverting or equal-quality results.
Final maximum world-vertex L2 difference for WB is 26.501249 m.
Final maximum world-vertex L2 difference for EX88 is 0.233228 m.

## Independent profile observations

| Scene | Version | Trajectory-candidate ms/frame | Calls | ms/call |
|---|---|---:|---:|---:|
| WB | before | 1.523 | 3756 | 0.393 |
| WB | after | 1.548 | 4030 | 0.373 |
| EX88 | before | 4.085 | 3163 | 1.253 |
| EX88 | after | 4.275 | 4120 | 1.006 |

Per-call candidate-scope observations are lower, while more calls leave the per-frame candidate total higher. Workloads and trajectories differ, and scopes include waits/overlap. These figures are diagnostics, not isolated BVH kernel-throughput proof. No Nsight hardware-level attribution was performed.

The EX88 profile baseline has five negative-J frames (56–60), min J=-0.170633. Follow-up has ten (56–60, 168–169, 175–177), min J=-0.204559. Thus the follow-up is not established as improving long-run physical quality. These profile processes are independent episodes and are excluded from formal statistics.

## Verification and limits

- Audited GPU suite: 21 cases / 842,883 assertions, including fresh candidate multisets, active sets, real-world energy/position/TOI and related BVH/config checks.
- Normal persistent-cache mode: 14 cases / 519,887 assertions.
- Forced copy readback: 6 cases / 295 assertions.
- Matching CUDA 13.5 explicit-API memcheck: 10 cases / 518,397 assertions, zero errors.
- Repository contracts: 13 cases. Clang-format 18 and whitespace checks passed.
- New cases cover selective long-edge query bounds, unequal chunks, EF overflow, typed discard/refill, cross-frame skin history, deferred rebuilds, produced metadata changes, large tables, empty prefixes and reused prefix PODs after topology growth.
- All eight runs loaded the expected frozen backend; configs and initial states match, source/input/runtime fingerprints stayed unchanged, and no loaded native file fell outside the manifest.

The initial invalid build target and a missing declaration in a new test were corrected; final builds passed. An initial memcheck invocation targeted the old output directory, is explicitly excluded, and was replaced by the current ten-case run. Logs are preserved. CMake/Python rebuilds, default extended API sanitizer, racecheck and synccheck remain unverified.

## Decision and artifacts

Retain the four requested mechanisms and their correctness tests. Do not advertise a whole-frame speedup from this group. Static work/launch reduction and passing same-state tests establish the implemented behavior for the tested inputs; the closed-loop episodes establish only the observations above. Run-to-run timing and trajectory variation remain unassessed.

- Implementation contract: [SingleEF on X_square](../../docs/development/xsquare_single_ef_bvh.md).
- Build/test/sanitizer and source/runtime provenance: `output/single-ef-query-followup-20261105/`.
- Cohort: `output/benchmark-runs/xsquare-query-followup-cuda135-20261105/`, containing frozen runtimes/inputs, `run_comparison.py`, preflight/telemetry, loaded files, initial/final states, per-frame stats, Timer trees, manifests, `summary.json` and `analysis.json`.
- The runner refuses to replace an existing cohort. Historical measurements remain unchanged.
