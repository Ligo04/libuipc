# 2026-11-05 — SingleEF query optimizations on WB and EX88

- Status: Single-episode observations are faster in both scenes. Stable,
  equal-work and equal-quality speedup remain unconfirmed.
- A/B source: both methods use `9ba6aa880137fe50efd01e61641b11fceb396d52` on
  `codex/xsquare-single-ef-bvh`; only `collision_detection/method` changes.
- X_square base: `6e3386d1`; migration checkpoint: `4539bdff`;
  query integration: `555efbee`; device-control correction: `9ba6aa88`.
- Samples: `b70f32294e31e9e3dcc12445b72fa9f4698ff5b1`, using the previously
  hash-frozen scene sources and assets. The live samples worktree was untouched.

## Implementation and environment

SingleEF now includes weighted contact-cover query pruning, two query/judge
streams over one EF tree, adaptive primitive skins, eight frame-start-request
topology retention, position-free PT/EE caching, whole-warp EE evaluation,
certified FP32 screen/gap/finish, one-pass tile scan/scatter, and mapped final
readbacks with a copy fallback. Mixed contact IDs and in-place metadata edits
are covered. Validation atomics and judge gates use device scratch; only
final flags/counts are published to mapped host memory. Active/pending-active
and TOI functions retain the migration implementation. Details are in
[the implementation contract](../../docs/development/xsquare_single_ef_bvh.md).

RTX 5090, driver 590.48.01, GCC 11.5.0, XMake 3.1.2 Release, native SM 120.
Toolkit is `/home/ligo/.local/cuda-13.5.0`, nvcc 13.5.37, with its own CUB,
Thrust and libcu++ 3.5. Builds used 16 jobs, MemoryHigh=32G, MemoryMax=32G,
MemorySwapMax=0. The device-control rebuild also held the global build lock.
Native backend SHA-256:
`9eee70e954a9f14cdd5ec339d213049ee325cff691281989a232c662d9ca728c`.
No Python wheel or CMake runtime was rebuilt.

## Workload and timing

WB follows sample 6: 560 cubes, 13 links, one ball, dt=0.02, gravity=-9.81,
ABD kappa=100 MPa, densities from `wrecking_ball.json`, ground at y=-1.
EX88 follows the executable sample-88 source: two `bunny2.msh` FEM bodies
(E=1e4, nu=0.49, rho=1000) and a 4225-vertex cloth (stretch E=5e4, shear E=10,
bending E=3e4, rho=200, one-sided thickness=0.001), dt=0.01 and MAS.
Sample contact, relative tolerances and semi-implicit settings are unchanged;
see [the earlier scene specification](2026-11-05-xsquare-wb-ex88.md).

Each scene/method has one formal 1000-frame process and one separate
synchronous-Timer profile process. Frames 1–30 are warmup; tables use 31–1000.
Primary wall time is `advance(); sync();`; secondary wall time also includes
`retrieve(); sync();`. Frame stats and geometry observations are outside
these intervals. Formal Timer/profiler is disabled. Profile scopes are
inclusive, synchronize CUDA, and are excluded from formal throughput.
Default linear-solver Graph configuration is retained; FullAdvance is not tested.

Before each launch, the runner waited for no other CUDA compute client for
at least 15 seconds and GPU temperature <=45 C. One-second telemetry found
no foreign CUDA compute client during any of the eight processes. This is a
shared desktop GPU; sampling does not exclude every short or graphics load.
Run-to-run variation and repeatability have not been assessed.

## Formal observations

| Scene | Method | Advance ms | FPS | Newton total / per frame | PCG total / per frame | Completed / converged |
|---|---|---:|---:|---:|---:|---:|
| WB | info_stackless_bvh | 10.008 | 99.922 | 3918 / 4.039 | 96155 / 99.129 | 970 / 970 of 970 |
| WB | single_ef_bvh | 9.499 | 105.270 | 3931 / 4.053 | 81790 / 84.320 | 970 / 970 of 970 |
| EX88 | info_stackless_bvh | 34.558 | 28.937 | 3558 / 3.668 | 149890 / 154.526 | 970 / 970 of 970 |
| EX88 | single_ef_bvh | 33.084 | 30.226 | 3532 / 3.641 | 150745 / 155.407 | 970 / 970 of 970 |

Observed advance-time reductions are 5.08% for WB and 4.27% for EX88;
corresponding throughput gains are 5.35% and 4.46%. End-to-end means including
retrieve are WB 10.368 -> 9.844 ms and EX88 34.800 -> 33.329 ms.
No measured frame hit a Newton or line-search limit. Frame-level convergence
does not establish the residual condition of every individual PCG solve.

All states were finite and no ground crossing was observed. EX88 has four
negative-J frames with Info (57–60, min J=-0.168012) and five with SingleEF
(56–60, min J=-0.210397), with at most one negative tet per frame. These are
not strictly non-inverting, equal-quality outcomes. Newton/PCG work differs;
WB PCG total is about 14.9% lower with SingleEF. Final maximum vertex L2
differences are 25.420 m for WB and 0.191 m for EX88. The closed-loop scenes
therefore do not isolate equal-state BVH throughput.

## Independent profile observations

| Scene | Method | Candidate ms/frame | Calls | ms/call |
|---|---|---:|---:|---:|
| WB | info_stackless_bvh | 1.857 | 3733 | 0.483 |
| WB | single_ef_bvh | 1.697 | 3773 | 0.436 |
| EX88 | info_stackless_bvh | 5.243 | 3695 | 1.376 |
| EX88 | single_ef_bvh | 4.355 | 3218 | 1.313 |

These are inclusive candidate-detection scopes, including waits and overlapped
work. Candidate calls and trajectories differ. They identify a smaller
candidate-stage observation in this profile cohort, not a hardware-level
explanation or a replacement for formal throughput. The profile EX88
processes also contain negative-J frames (Info: five; SingleEF: four).
No Nsight kernel-level attribution was performed.

## Validation and provenance

- Audited GPU suite: 17 cases / 842,782 assertions, candidate multisets,
  active sets, real-world position/energy/TOI and related BVH/config tests.
- Normal cache mode: 11 cases / 519,790 assertions.
- Forced readback-copy fallback: 2 cases / 194 assertions.
- Matching CUDA 13.5 Compute Sanitizer, explicit API mode: 6 cases /
  518,296 assertions, zero reported memory/API errors.
- Repository contracts: 13 cases; clang-format 18 and diff checks passed.
- Each A/B initial state is identical, and config differs only by method.
  Source/input/runtime hashes stayed unchanged. Loaded native hashes are
  covered by the preflight manifest; active filter registration was checked
  from each runtime's `systems.json`.

The CUDA 13.5 default extended API sanitizer mode, racecheck and synccheck
remain unverified. The first full CUDA 13.5 build invocation stopped before
completion; the resumed and final builds succeeded with the stated limits.
All build logs remain archived.

The earlier `555efbee` prototype cohort is preserved under
`output/benchmark-runs/xsquare-query-cuda135-20261105/`. Its WB Info run sampled
another CUDA client, and the last EX88 profile was blocked before launch by
another checkout's GPU process. Its WB difference is not a clean performance
comparison. A separate, explicitly authorized cohort at `9ba6aa88` produced
the results above after the device-control correction; previous data was not
overwritten. The earlier CUDA 13.4 records are separate historical cohorts.

## Decision and reproduction

Retain the query integration and its correctness gates. These single-process
observations do not demonstrate a stable or equal-quality general speedup.
The selector default remains `info_stackless_bvh`.

For a future build, configure CUDA explicitly and hold the global build lock:

```sh
flock /home/ligo/.cache/ligo-build.lock systemd-run --user --scope --collect \
  -p MemoryHigh=32G -p MemoryMax=32G -p MemorySwapMax=0 \
  xmake f -y -m release --cuda=/home/ligo/.local/cuda-13.5.0 --cuda_sdkver=13.5 \
  --backend_cuda=y --tests=y --pybind=n --examples=n --dev=n \
  --usd=n --vdb=n -o build/xmake-cuda135
flock /home/ligo/.cache/ligo-build.lock systemd-run --user --scope --collect \
  -p MemoryHigh=32G -p MemoryMax=32G -p MemorySwapMax=0 xmake -j16 backend_cuda
```

Entrypoint is `apps/tests/backends/cuda/single_ef_bvh_performance.cpp`, hidden
Catch2 tag `[.single_ef_bvh_performance]`. Use a fresh output directory and the
recorded frozen assets. Set `UIPC_SINGLE_EF_PERF_SCENE=wb` or `ex88`,
`UIPC_SINGLE_EF_PERF_METHOD=info_stackless_bvh` or `single_ef_bvh`,
`UIPC_SINGLE_EF_PERF_PROFILE=0` for formal timing and 1 only for a separate
profile process, plus `UIPC_SINGLE_EF_PERF_ASSETS` and
`UIPC_SINGLE_EF_PERF_OUTPUT` with absolute paths.

Artifacts:

- `output/single-ef-query-optimization-20261105/`: build/test/memcheck logs,
  compiler-header provenance and source/native hashes. Pre-correction checks
  remain in its `before-device-control/` directory.
- `output/benchmark-runs/xsquare-query-device-control-cuda135-20261105/`:
  `run_comparison.py`, frozen inputs, preflight/telemetry, per-frame stats,
  initial/final geometry, Timer trees, manifests, `summary.json` and
  `analysis.json`. The runner refuses to replace an existing cohort.
