# 2026-11-05 — X_square SingleEF native wrecking-ball comparison

- Status: Inconclusive for a stable performance difference; no demonstrated
  end-to-end speedup in this experiment.
- Before / after commit: both `6e3386d12823c835c3206880b7758de952ccd339`
  plus the same uncommitted SingleEF migration and benchmark entrypoint, on
  `codex/xsquare-single-ef-bvh`.
- Workload: the repository's native `apps/examples/wrecking_ball/main.cpp`
  scene, reproduced by the opt-in backend test below. This is not the Python
  `rigid-wrecking-balls` manifest workload; no samples checkout was used.

## Question

Does selecting `single_ef_bvh` improve performance over `info_stackless_bvh`
in the migrated X_square runtime? The only scene-configuration difference is
`collision_detection/method`; the two methods use the same executable and
native libraries.

## Environment

| Field | Value |
|---|---|
| GPU / driver | RTX 5090 / 590.48.01, shared display GPU |
| CUDA toolkit | 13.4.92 from `/home/ligo/.local/cuda-13.4.3` |
| OS / compiler | Ubuntu, Linux 6.17.0-23-generic / GCC 11.5.0 |
| Build | XMake 3.1.2, Release, native SM 120 |
| Worktree | Dirty migration snapshot; source/input hashes and patch archived |
| Native backend SHA-256 | `b7794492d862bb8737b9bc5ab61d0b15ed7965fbb3e6acd3a67a2f97bd3d8c8a` |
| Test executable SHA-256 | `1cda8e22e3a3bd88c4cb07b1c437f2203a41a1f46cd67a29dc2c1e4895e516b0` |

The benchmark entrypoint was built with `xmake -j16 backend_cuda` inside a
systemd user scope with MemoryHigh/MemoryMax=32G and MemorySwapMax=0.

## Workload and method

The scene contains 560 cubes, 13 links, one ball, and a half-plane at y=-1.
The native example's physics is preserved: dt=0.01 s, gravity=(0,-9.8,0),
ABD kappa=10 MPa and density=1000, contact friction=0.01, contact stiffness
20 GPa, d_hat=0.01, and line-search max_iter=8. JSON density overrides are
not consumed by the native example or this entrypoint.

Defaults remain fused PCG, tol_rate=1e-3, use_cuda_graph=1 (PCG block replay),
check_interval=5, Newton max_iter=1024, velocity_tol=0.05,
transrate_tol=0.1, and semi-implicit enable=1, beta_tol=1e-3, K_min=6.
There is no FullAdvance or collision Graph comparison here.

Each method ran once in an independent process for 1000 frames. Frames 1–30
are warmup; the primary window is 31–1000. Order was Info then SingleEF for
the formal pair, followed by Info then SingleEF for the separate Timer pair.
No extra scenes or repeated runs were added.

- Formal: Timer disabled, no external profiler, no frame dumps/rendering.
- Primary timing: `advance(); sync();` with steady-clock wall time.
- Secondary timing: `advance(); sync(); retrieve(); sync();`.
- Frame stats, geometry observations, JSON writes, and initialization are
  outside both timing intervals.
- Profile: the same frame plan with synchronous Timer enabled only after
  warmup. CUDA backend Timer synchronization calls `cuda_tool::wait_device()`.
  Each frame's merged tree is saved and cleared outside the timed interval.
  Parent/child scopes are inclusive and must not be added together.

No external CUDA compute process was observed. Desktop graphics activity
remained present (preflight utilization approximately 8–23%). Clocks were
not locked. The bounded cooldown reached 45 C for the other runs, but the
first run began at 50 C after its 65-second cooldown budget expired.
Five-second GPU samples are archived; this is not a controlled thermal or
repeatability experiment.

## Results

Formal results, frames 31–1000; FPS excludes retrieve:

| Method | Advance mean / p50 / p95 (ms) | FPS | Newton total / per frame | PCG total / per frame | Completed / converged |
|---|---:|---:|---:|---:|---:|
| info_stackless_bvh | 12.997 / 13.156 / 15.654 | 76.939 | 6492 / 6.693 | 102730 / 105.907 | 970 / 970 |
| single_ef_bvh | 13.484 / 13.362 / 17.079 | 74.162 | 6528 / 6.730 | 101355 / 104.490 | 970 / 970 |

SingleEF's observed mean advance time increased 3.745%; FPS decreased
3.610%. Mean time including retrieve increased from 13.091 to 13.654 ms
(4.299%). These figures describe this pair, not a repeatable regression.

Full 1000-frame advance means are 12.797 vs 13.288 ms, or 78.145 vs
75.255 FPS. Both full episodes completed and reported convergence on all
1000 frames. Solver counts in the table use only the 970-frame window.

Separate synchronous Timer diagnostics, also frames 31–1000:

| Inclusive scope | Info ms/frame | SingleEF ms/frame | Notes |
|---|---:|---:|---|
| Detect Trajectory Candidates | 2.668 | 2.572 | 6540 vs 6566 calls, approximately 3.62% less total time |
| Line Search | 5.238 | 5.098 | Includes candidate detection and nested iteration scopes |
| FusedPCG | 1.862 | 1.794 | Different PCG work and trajectories |
| Pipeline | 15.772 | 15.681 | Instrumented; not the formal throughput result |

The profile advance means are 15.794 vs 15.699 ms. They must not replace
the uninstrumented results or be treated as an independent repetition.
The available Timer scopes do not isolate SingleEF build/refit/cache/query
or individual kernels, so this experiment does not identify a kernel-level
cause or measure cache hit rate.

## Correctness, provenance, and limits

All four opt-in test processes passed and completed 1000 frames. In each
970-frame measurement window: completed=970, converged=970, nonconverged=0,
Newton-limit=0, line-search-limit=0, invalid=0, nonfinite=0. Frame-level
convergence does not establish that every PCG solve met a residual criterion.

No sampled body's transformed vertex crossed the y=-1 ground plane. Formal
minimum y was -0.997359 vs -0.997463. This is a ground-plane observation,
not a complete body/body penetration audit. Final affine determinant ranges
were [0.996234, 1.021881] vs [0.995050, 1.023975]; they do not certify all
intermediate states.

Formal Newton and PCG counts first differ at frame 19. The final maximum
absolute translation-component difference is 26.005805 m. The maximum
per-frame minimum-y difference is 0.004700 m. Unchanged-run trajectory
variability was not measured. Therefore the pair cannot establish equal-state
collision throughput or attribute its frame-time difference solely to BVH
work. No solver, scene, or collision implementation was changed to improve
these measurements.

The 30 frozen source/input files and 57 preflight runtime files remained
unchanged. All four processes loaded the same 24 executable/library files
with identical hashes. Preserve one provenance gap: plain `ldd` resolved
system TBB before the first run, while XMake's run environment loaded three
packaged TBB libraries. Their first hashes were captured after formal A and
checked again before formal B and after all runs. They remained identical
across all four runs, but lack a pre-formal-A hash. The original manifests
and the supplementary record retain this distinction.

## Decision

Keep the migration behavior unchanged. This experiment demonstrates no
formal whole-frame speedup. The modest Timer candidate-stage reduction
does not establish a stable or equal-work algorithmic improvement.
Run-to-run variation and repeatability were not evaluated. No commit or
push was made for this performance task.

## Reproduction and artifacts

Entrypoint:
`apps/tests/backends/cuda/single_ef_bvh_performance.cpp`.
The hidden Catch2 tag is explicitly opt-in and does not run by default.
For a future separately authorized measurement, use a fresh output directory:

```sh
UIPC_SINGLE_EF_PERF_METHOD=single_ef_bvh \
UIPC_SINGLE_EF_PERF_PROFILE=0 \
UIPC_SINGLE_EF_PERF_OUTPUT=/absolute/path/to/new-output-directory \
xmake run backend_cuda '[.single_ef_bvh_performance]' --reporter compact
```

Use `UIPC_SINGLE_EF_PERF_PROFILE=1` only for the separate diagnostic process.
The entrypoint refuses an existing output directory.

Ignored artifacts: `output/benchmark-runs/xsquare-single-ef-20261105/`.
The directory contains `REPORT.md`, `analysis.json`, before/after manifests,
`source.patch`, XMake configuration, the serialized comparison supervisor,
postprocessing script, supplementary TBB provenance, logs, five-second GPU
telemetry, and each run's configuration, frame rows, final transforms,
loaded-file lists, validation, and (for profile only) Timer trees.
