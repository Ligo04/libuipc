# 2026-11-05 — Shared simplex BVH query policies on WB and EX88

- Status: single-group observation with mixed frame-cost results. A stable,
  equal-work or equal-quality performance advantage is not established.
- A/B measurement HEAD: `db34e286bd94e802248e0a69bb8153df3c8b5bff` on
  `codex/xsquare-single-ef-bvh`, clean at measurement start.
- Compiled implementation: `c70878f5198fa52666d0010749dcde186149c858`. The later
  measurement HEAD adds documentation only. Both methods use the same native
  executable and libraries; only `collision_detection/method` changes.
- Samples: the previously frozen inputs at
  `b70f32294e31e9e3dcc12445b72fa9f4698ff5b1`; live samples were untouched.

## Question

Measure the current InfoStackless and SingleEF methods after unifying contact
covers, four-family fat caches, cross-frame validation/skin policy and parallel
judging in `SimplexBVHQuery`. Primitive-tree and EF refresh strategies remain
method-specific. This compares the two current methods, rather than measuring
the causal before/after effect of unification on a fixed method.
See [the common contract](../../docs/development/simplex_bvh_queries.md).

## Environment

RTX 5090, driver 590.48.01, native SM 120, XMake Release, GCC 11.5.0,
CUDA Toolkit 13.5.37 at `/home/ligo/.local/cuda-13.5.0`, using its CUB,
Thrust and libcu++ 3.5 on Linux x86-64. No compiler or rebuild was invoked for
this cohort.
The existing validated native runtime was copied to a private, read-only
runtime directory and invoked directly, avoiding XMake target resolution.
The matching build configuration and prior validation manifest are archived.

Backend SHA-256: `a0d64cac57d83d8c8b0d02078de4851d4dd9a5f3d322ce61e1307301e19ccb33`.
Executable SHA-256: `b922d30266c42073ff3d435bebaf813abda07ea488687e47d234151c66ea160f`.

Before each launch, require 15 seconds without a CUDA compute client and GPU
temperature <=45 C. One-second execution telemetry sampled 0 foreign
compute observations across the eight processes. Another CUDA job delayed
launches during preflight; it was not terminated. This is a shared desktop GPU;
graphics load, brief unsampled work and clock drift cannot be excluded.
One own exiting compute context was reported as `[No data]`; its PID matches
the named benchmark PID in preceding samples. The initial name-only suspect
is preserved, and final classification uses the benchmark PID.
Run-to-run variation and repeatability have not been assessed.

## Workload and method

WB follows sample 6: 560 cubes, 13 links, one ball, dt=0.02, ground y=-1.
EX88 follows executable sample 88: two scaled bunny2 FEM meshes (E=1e4,
nu=0.49, rho=1000), MAS, 4225-vertex cloth and dt=0.01. Materials,
contact settings, tolerances and semi-implicit controls retain the
[previous scene specification](2026-11-05-xsquare-wb-ex88.md).
The native benchmark port and frozen asset hashes are unchanged.

One formal process and one separate synchronous-Timer profile process per
scene/method, eight processes total. Each requests 1000 frames. Frames 1-30
are warmup; all tabulated timing/work counts use 31-1000 (970 frames).
Order in each group is WB Info, WB SingleEF, EX88 Info, EX88 SingleEF.
No extra repetition or failed-run replacement was performed.

Primary time is `World.advance(); World.sync();`; CUDA sync calls
`cuda_tool::wait_device()`. Secondary time includes `retrieve(); sync();`.
Both exclude initialization, geometry observations, stats and file output.
Formal Timer/profiler is disabled. Default `fused_pcg` with
`linear_system/use_cuda_graph=1` is retained. This does not test complete
collision detect/advance Graph capture.

## Formal observations

| Scene | Method | Advance mean ms | FPS | Newton total / per frame | PCG total / per frame | Completed / converged |
|---|---|---:|---:|---:|---:|---:|
| WB | info_stackless_bvh | 9.578 | 104.402 | 3819 / 3.937 | 99895 / 102.985 | 970 / 970 of 970 |
| WB | single_ef_bvh | 9.409 | 106.280 | 3937 / 4.059 | 94500 / 97.423 | 970 / 970 of 970 |
| EX88 | info_stackless_bvh | 33.857 | 29.536 | 3762 / 3.878 | 160270 / 165.227 | 970 / 970 of 970 |
| EX88 | single_ef_bvh | 36.725 | 27.229 | 4046 / 4.171 | 169260 / 174.495 | 970 / 970 of 970 |

PCG totals sum all linear-solver iterations in the window, across Newton solves.
Frame convergence does not certify every PCG solve residual.

| Scene | Method | Advance p50 / p95 ms | Including retrieve mean ms |
|---|---|---:|---:|
| WB | info_stackless_bvh | 8.217 / 17.713 | 9.732 |
| WB | single_ef_bvh | 8.002 / 16.550 | 9.578 |
| EX88 | info_stackless_bvh | 24.435 / 80.255 | 34.007 |
| EX88 | single_ef_bvh | 31.306 / 80.913 | 36.876 |

## Correctness and quality

All eight processes completed 1000 valid, finite frames. Each formal measured
window completed and converged 970/970 frames, without Newton or line-search
limits. Geometry quality is a separate observation.

| Scene | Formal method | Negative-J frames | Minimum tet J | Maximum ground penetration m |
|---|---|---|---:|---:|
| WB | info_stackless_bvh | N/A (ABD) | N/A | 0 |
| WB | single_ef_bvh | N/A (ABD) | N/A | 0 |
| EX88 | info_stackless_bvh | 56, 57, 58, 59, 60 | -0.171873395 | 0 |
| EX88 | single_ef_bvh | 57, 58, 59, 60 | -0.140641552 | 0 |

EX88 therefore does not demonstrate strictly non-inverting or equal-quality
behavior. Final-state and iteration differences also prevent an equal-work
BVH throughput claim. No solver, contact setting or scene parameter was changed
to improve these observations.

The exact frozen runtime had already passed the implementation gates: 24
audited cases / 843802 assertions, 17 normal cases / 520806 assertions, 8 copy
fallback cases / 1260 assertions, and 13 explicit-API Compute Sanitizer memcheck
cases / 519316 assertions with zero errors. Those gates were not rerun as
performance measurements. CMake/Python builds, extended-API sanitizer, racecheck
and synccheck are not added by this cohort.

## Independent Timer profile

These are separate episodes. Inclusive scopes synchronize CUDA, include waits
and any overlapped work, and are excluded from formal throughput. Candidate
scopes include DCD and trajectory detection where present. Parent and child
scopes must not be summed together.

| Scene | Method | Candidate ms/frame | Calls | ms/call | Largest direct Newton child, ms/frame |
|---|---|---:|---:|---:|---|
| WB | info_stackless_bvh | 1.816 | 3745 | 0.470 | Solve Global Linear System, 4.401 |
| WB | single_ef_bvh | 1.484 | 3944 | 0.365 | Solve Global Linear System, 4.782 |
| EX88 | info_stackless_bvh | 3.800 | 3239 | 1.138 | Solve Global Linear System, 16.505 |
| EX88 | single_ef_bvh | 3.982 | 3898 | 0.991 | Solve Global Linear System, 19.721 |

These scopes identify expensive pipeline regions, rather than a kernel-level
hardware limiter. No Nsight hardware attribution was performed. Calls, Newton
work and trajectories differ between these independent profile episodes.
The largest direct Newton child is `Solve Global Linear System`, entered by
[the IPC Newton loop](../../src/backends/cuda/engine/advance_ipc.cu). Its
[global linear-system implementation](../../src/backends/cuda/linear_system/global_linear_system.cu)
includes assembly and solve, with
[FusedPCG](../../src/backends/cuda/linear_system/linear_fused_pcg.cu).
In EX88 profiles, this inclusive scope accounts for about 49.6% / 50.0% of
profile advance time for Info / SingleEF; candidate scopes account for
11.4% / 10.1%. These percentages describe the instrumented episodes only.
Profile EX88 inversion observations are also retained in the JSON artifacts:

- info_stackless_bvh: 5 negative-J frames, minimum J=-0.184682344.
- single_ef_bvh: 5 negative-J frames, minimum J=-0.216498713.

## Interpretation and decision

WB: observed SingleEF advance-time reduction +1.77%, throughput change +1.80%. Newton work changes +3.09% and PCG work -5.40%. Final maximum world-vertex L2 difference is 29.560138 m.

EX88: observed SingleEF advance-time reduction -8.47%, throughput change -7.81%. Newton work changes +7.55% and PCG work +5.61%. Final maximum world-vertex L2 difference is 0.937660 m.

Positive time reduction means SingleEF was faster; a negative value means it
was slower. These are one-episode observations, not stable performance
regressions/improvements or the isolated cost of the query implementation.
Retain the unified code and its correctness gates; this group does not select
a general performance winner or change a default.

## Provenance and reproduction

All selected filters match the requested methods. A/B initial states are
identical and complete configs differ only by method. Source/input/runtime
fingerprints stayed unchanged through the cohort, all loaded native files
are covered, and both methods loaded the expected private backend.

Artifacts are under
`output/benchmark-runs/xsquare-shared-query-cuda135-20261105/`:

- `run_comparison.py`, read-only `runtime/`, frozen `inputs/`, build/input/runtime manifests.
- Per-run logs, preflight/telemetry, loaded-file lists, effective configs, initial/final geometry, 1000 per-frame records and independent Timer trees.
- `summary.json`, `analysis.json`, `profile-summary.json`, `cohort-verification.json`.
- `summarize.py`, `analyze.py`, `verify_cohort.py` and this data-driven report writer.

The runner refuses to overwrite an existing cohort. To reproduce, copy the
runner and frozen inputs/runtime to a fresh cohort directory at the same
repository depth, preserving the intended clean source revision. It invokes
`uipc_test_backend_cuda [.single_ef_bvh_performance] --reporter compact` directly
with explicit method/scene/profile/output/asset environment variables and
`LD_LIBRARY_PATH`. Existing historical cohorts and reports are unchanged.
