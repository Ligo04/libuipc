# 2026-11-05 — X_square SingleEF on WB and EX88

- Status: Inconclusive for stable or equal-work BVH throughput. The observed
  formal WB episode is faster and EX88 is slower with SingleEF.
- Before / after commit: `6e3386d12823c835c3206880b7758de952ccd339` plus the
  same uncommitted migration, on `codex/xsquare-single-ef-bvh`.
- Samples: `b70f32294e31e9e3dcc12445b72fa9f4698ff5b1`; required source and
  assets were copied, hash-verified, and frozen without changing the samples
  worktree. Its unrelated dirty benchmark runner was not used.

## Question and environment

Compare `info_stackless_bvh` and `single_ef_bvh` using the requested WB and
EX88 scenes. Each scene's A/B factor is only `collision_detection/method`.
The runner ports the sample's scene construction to the current native API;
it does not load a separately installed pyuipc or run the Python GUI.

Hardware is an RTX 5090, driver 590.48.01, CUDA 13.4.92, GCC 11.5.0,
XMake 3.1.2 Release, native SM 120. The native CUDA backend SHA-256 is
`b7794492d862bb8737b9bc5ab61d0b15ed7965fbb3e6acd3a67a2f97bd3d8c8a`.
Incremental benchmark builds used 16 jobs inside a systemd user scope with
MemoryHigh=32G, MemoryMax=32G, and MemorySwapMax=0.

## Workload and method

WB follows sample 6: 560 cubes, 13 links, one ball, ground at y=-1,
dt=0.02, gravity=-9.81, ABD kappa=100 MPa, and per-object density from
`assets/sim_data/wrecking_ball.json` (1000 for cubes, 7680 for links/ball).
Contact mu=0.2, kappa=1e8, d_hat_relative=1e-3,
eps_velocity_relative=1e-2, velocity_tol_relative=1e-2,
PCG tol_rate=1e-4, and transrate_tol=10 are preserved.
This differs from the earlier native-example WB preset; their numbers are
separate experiments.

EX88 follows the executable sample-88 source: two `bunny2.msh` FEM meshes,
each with 19193 vertices and 79935 tets, baked scale=0.2, y offsets=0.5
and -0.65, StableNeoHookean E=1e4, nu=0.49, rho=1000. E=1e4 follows
the actual expression, rather than E=1e7 stated in its docstring.
The 4225-vertex `cloth_high.obj` uses stretch E=5e4, shear E=10,
nu=0.49, rho=200, one-sided thickness=0.001, strain_rate=100, and
bending E=3e4. MAS, dt=0.01, gravity=-9.8, contact mu=0.2/kappa=1e8,
the sample's relative tolerances, and semi-implicit beta_tol=1e-2/K_min=6
are preserved.

Both use default fused PCG block replay. There is no FullAdvance comparison.
Each scene/method has one completed formal 1000-frame episode and one
separate synchronous-Timer episode. Frames 1–30 are warmup; statistics use
31–1000. Processes are independent and GPU execution is serial.

Primary wall timing is `advance(); sync();`, with Timer/external profiler
off for the formal pair. Secondary timing includes `retrieve(); sync();`.
Initialization, frame stats, geometric observations, and output writes are
outside both intervals. Profile Timer is enabled only after warmup; its
CUDA sync callback is `cuda_tool::wait_device()`. Timer trees are merged
and cleared outside each timed interval. Parent/child scopes are inclusive.

GPU clocks were not locked. No external CUDA compute job was observed;
desktop graphics activity remained present. Temperature/frequency/power
samples and bounded-cooldown records are archived. Run-to-run variation
and repeatability were not evaluated.

## Formal results

All rows use the same 970-frame measurement window. FPS excludes retrieve.

| Scene | Method | Mean advance ms | FPS | Newton total / frame | PCG total / frame | Completed / converged |
|---|---|---:|---:|---:|---:|---:|
| WB | info_stackless_bvh | 10.577 | 94.543 | 4074 / 4.200 | 104930 / 108.175 | 970 / 970 |
| WB | single_ef_bvh | 9.816 | 101.871 | 3840 / 3.959 | 95555 / 98.510 | 970 / 970 |
| EX88 | info_stackless_bvh | 31.069 | 32.186 | 3244 / 3.344 | 138950 / 143.247 | 970 / 970 |
| EX88 | single_ef_bvh | 35.460 | 28.201 | 3573 / 3.684 | 154110 / 158.876 | 970 / 970 |

WB's observed mean advance time decreased 7.194% and FPS increased 7.752%.
EX88's time increased 14.130% and FPS decreased 12.381%.
Including retrieve, means are 10.766 to 9.967 ms for WB and 31.205 to
35.603 ms for EX88. Full-episode means and p50/p95 are archived separately.

All four formal episodes completed 1000 frames. In every measured window,
nonconverged, Newton-limit, line-search-limit, invalid, and nonfinite frame
counts are zero. Frame-level convergence does not establish that every PCG
solve met its residual criterion.

## Quality and interpretation

No observed vertex crossed the y=-1 ground plane. Body/body penetration
was not exhaustively checked.

Both formal EX88 runs contain five frames with one negative tet J each.
Minimum J is -0.162641 for Info and -0.195891 for SingleEF. J is the
current/rest oriented volume ratio over all 159870 bunny tets. Cloth is
excluded from tet metrics. Both fail a strict no-inversion criterion.

WB Newton/PCG totals decreased 5.74%/8.93%; EX88 increased 10.14%/10.91%.
WB solver counts first differ at frame 10. EX88 PCG first differs at frame
24 and Newton at frame 45. Maximum final vertex-position distances are
22.814 m for WB and 0.532 m for EX88. Unchanged-run trajectory variation
was not measured. Thus whole-frame changes include different evolving
workloads and cannot be attributed entirely to BVH throughput.

Separate inclusive candidate-detection Timer diagnostics:

| Scene | Method | ms/frame | Calls | ms/call |
|---|---|---:|---:|---:|
| WB | info_stackless_bvh | 2.122 | 3983 | 0.517 |
| WB | single_ef_bvh | 1.951 | 3413 | 0.554 |
| EX88 | info_stackless_bvh | 5.435 | 3077 | 1.713 |
| EX88 | single_ef_bvh | 6.781 | 3378 | 1.947 |

Calls and candidate workloads differ. These are diagnostic scope timings,
not equal-state kernel measurements, and do not replace formal throughput.
The profile groups also have five negative-J frames on each EX88 side.

## Setup failure and provenance

Four original EX88 processes stopped before `world.init()` and before any
frame timing. The new host observer accessed tet topology on the zero-tet
cloth. Two guards now skip tet observations for zero-tet geometries.
The original failure logs and manifests remain archived. Only the benchmark
host observer changed; solver code, native libraries, assets, and scene
parameters were unchanged. WB was retained from its completed cohort;
EX88 completed in a separately frozen cohort with the corrected executable.

Within each scene, all four completed processes have identical initial
states, scene configurations except for the method, and loaded runtime
hashes. Source/input/runtime fingerprints were unchanged during each cohort;
all loaded dependencies are covered by preflight hashes. Native library
hashes are identical across WB and EX88. The test executable differs after
the host-observer correction, and both executable fingerprints are retained.
Active filter registration and EX88's active FEMMASPreconditioner were
verified from runtime `systems.json`, not inferred only from configuration.

## Decision and reproduction

No solver or collision optimization was made from these observations.
There is no demonstrated general or equal-quality SingleEF speedup.
No commit or push was made.

Entrypoint: `apps/tests/backends/cuda/single_ef_bvh_performance.cpp`, hidden
Catch2 tag `[.single_ef_bvh_performance]`. A separately authorized future run
must use a fresh output directory:

```sh
UIPC_SINGLE_EF_PERF_SCENE=ex88 \
UIPC_SINGLE_EF_PERF_ASSETS=/absolute/path/to/frozen/assets \
UIPC_SINGLE_EF_PERF_METHOD=single_ef_bvh \
UIPC_SINGLE_EF_PERF_PROFILE=0 \
UIPC_SINGLE_EF_PERF_OUTPUT=/absolute/path/to/new-output-directory \
xmake run backend_cuda '[.single_ef_bvh_performance]' --reporter compact
```

Set scene to `wb` for sample 6 and PROFILE=1 only for the separate diagnostic
process. The default scene, without SCENE, remains the older native preset.

Artifacts under `output/benchmark-runs/`:

- `xsquare-wb-ex88-20261105/`: combined `REPORT.md`,
  `analysis-combined.json`, WB runs, initial zero-frame EX88 failures,
  source/asset snapshots, before/after manifests, patch, build logs, runner,
  analysis script, frame rows, states, runtime lists, Timer trees and GPU
  telemetry.
- `xsquare-ex88-20261105/`: all four completed EX88 runs, corrected-runner
  snapshot, the same frozen inputs, and independent manifests/validation.

The source metadata retains the original input-staging location; EX88's
manifest fingerprints cover its byte-identical copied input directory.
