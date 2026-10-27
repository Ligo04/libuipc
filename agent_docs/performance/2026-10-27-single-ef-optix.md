# Single-EF OptiX experiment — 2026-10-27

## Decision

Keep `single_ef_optix` as an optional experiment, disabled by default. This
implementation did not demonstrate a stable end-to-end improvement on the
1,000-frame C++ wrecking-ball scene. OptiX improves DCD candidate detection in
the scene diagnostic, but increases CCD candidate-detection cost.

The implementation replaces EF enumeration only. Existing CUDA PT/EE
predicates, compaction, overflow replay, contact classification, and CCD TOI
calculations remain in use. Unsupported Single-EF topology retains the
existing informative-BVH fallback.

Machine-readable samples, validation counts, audit counters, profile evidence,
and artifact/input hashes are in
[the results file](2026-10-27-single-ef-optix-results.json).

## Source and environment

| Item | Value |
|---|---|
| Requested base | `755b3d62fa413eccd5dc977988270ab95a1aedbe` |
| Tested implementation | `b64b9f8be8c948be65be14b0eea935a4b650cc16` |
| Branch | `codex/single-ef-optix` |
| Measurement checkout | Clean; both selectors use the same executable and libraries |
| GPU / driver | RTX 5090 / 590.48.01 |
| CPU | AMD Ryzen 9 9950X, 32 logical CPUs |
| Build | Linux XMake Release, `dev=y`, `ccache=n` |
| CUDA | 13.4.92 for native code and OptiX PTX |
| Architectures | Native `sm_120`; OptiX PTX 9.4, virtual `compute_75` |
| OptiX SDK | 9.1.0; real module creation, GAS updates, and launches verified |
| Scene | Native C++ wrecking ball, 574 observed bodies |

The scene's C++ material setup uses ABD density 1000 and stiffness 10 MPa;
the JSON density setting is not consumed by that setup. Both methods retain
the same configuration apart from the collision selector. Input meshes and
configuration files, build artifacts, actual loaded modules, and repository
state are checked by the benchmark runner. No Python wheel or samples checkout
is used for these timings.

The initial compute-process snapshot was empty. This was a desktop GPU run
without locked clocks or CPU affinity; per-second GPU samples are retained.

## End-to-end result

Each method ran five times in alternating order, with 1,000 completed frames
per run. Frames 1–30 are warmup; frames 31–1,000 are measured. Each frame's
wall interval covers `World::advance`, `retrieve`, and `sync`, excluding result
serialization. Timers, audit, sanitizer, and profiler were disabled.

| Method | Median of five run means, ms/frame | Range of run means, ms/frame |
|---|---:|---:|
| `single_ef_bvh` | 38.499 | 38.016–39.193 |
| `single_ef_optix` | 38.678 | 38.215–40.015 |

The OptiX median is 0.47% higher. The ranges overlap, and paired differences
change sign. This does not establish a stable speedup or a precise regression.

Raw run means, in order:

- CUDA: 38.072027, 38.015976, 39.192815, 38.499135, 38.677560 ms/frame.
- OptiX: 38.214955, 38.898725, 38.678325, 40.014925, 38.526783 ms/frame.

Newton means range from 6.729–6.745 per frame for CUDA and 6.724–6.744 for
OptiX. Linear-solver iteration means range from 87.479–90.381 and
86.969–91.722 respectively. Independent trajectories therefore do not provide
an identical-work traversal comparison.

## Fixed-state comparison

The hidden test `[single_ef_optix_benchmark]` uses two 32-by-32 vertex layers:
2,048 vertices, 5,890 edges, and 3,844 triangles. Primitive bounds stay fixed.
Each scope warms both implementations with three calls, then alternates five
samples of twenty calls per method. CUDA synchronization bounds the host wall
interval. Timing includes the complete `SingleEFBVHQuery::detect` call,
including proxy preparation, build/refit, query, PT/EE expansion, and readback;
it excludes primitive-bound construction and initialization.

Canonical PT/EE multisets match before and after every sample. Actual OptiX
build/refit/launch counters are asserted. The test passes 402 assertions.

| Scope | CUDA median [range], ms/call | OptiX median [range], ms/call |
|---|---:|---:|
| DCD build + query | 0.444839 [0.405354–0.476288] | 0.431644 [0.385594–0.454452] |
| DCD refit + query | 0.305386 [0.303374–0.328828] | 0.302122 [0.255888–0.309656] |
| CCD build + query | 0.264866 [0.263904–0.287027] | 0.377055 [0.368910–0.405581] |
| CCD refit + query | 0.166742 [0.165967–0.198869] | 0.247082 [0.209252–0.282866] |

DCD differences are small relative to observed variation. CCD medians are
42.4% and 48.2% higher with OptiX on this fixture. These are fixture-specific
measurements, not percentages to apply to wrecking-ball frame time.

## Scene diagnostics and profiler evidence

A separate 1,000-frame run per method enables synchronized timers, again
excluding the first 30 frames. These are instrumentation results, not the
five-run throughput measurements.

| Candidate scope | CUDA ms/call | OptiX ms/call | CUDA ms/frame | OptiX ms/frame |
|---|---:|---:|---:|---:|
| DCD | 1.019426 | 0.802296 | 6.870089 | 5.408468 |
| CCD | 0.388322 | 0.743606 | 2.616972 | 5.012827 |

There are 6,537 measured calls of each scope on CUDA and 6,539 on OptiX.
The DCD reduction is outweighed by the CCD increase. The two disjoint candidate
scopes total 9.487 and 10.421 ms/frame. Parent timers include their children;
those totals are not added to parent pipeline or line-search timings.

Linear-system construction remains a substantial independent cost:
10.910 ms/frame on CUDA and 10.740 on OptiX, including subsystem and
preconditioner assembly. The linear solve itself is about 1.91 ms/frame in
both diagnostics. OptiX enumeration does not accelerate these stages.

Nsight Systems 2026.5.1 captured seconds 5–15 of separate runs, with CUDA graph
node tracing, no CPU sampling, and no application timers/audit. `--kill=none`
allows both applications to complete all 1,000 frames after collection stops.
Report context, doctor, facts, and bounded queries were obtained through the
Nsight skill gateway.

| Captured GPU operation | Mean duration, microseconds |
|---|---:|
| Native DCD `EdgeFaceNodePredicate` traversal | 1085.383 |
| Native CCD `NodePredicate` traversal | 243.251 |
| OptiX launch, DCD and CCD combined | 492.877 |
| OptiX acceleration build/update | 48.316 |
| OptiX proxy preparation | 8.165 |
| OptiX GAS-bound preparation | 8.960 |

This supports the observed DCD/CCD asymmetry: native segment-pruned DCD
traversal is more expensive than native box-overlap CCD traversal, while this
OptiX implementation uses the same ray program for both. OptiX also retains
GAS preparation/update and CPU API costs. Its recorded CPU NVTX scopes average
22.214 microseconds for `optixAccelBuild` and 13.976 for `optixLaunch`; these are
CPU scope durations, distinct from the GPU durations above.

The capture windows are not identical simulation states, so their aggregate
durations and launch counts are not speedup ratios. The captures report partial
CUDA/NVTX-event warnings; the native capture has no NVTX table. Kernel timeline
and runtime-correlation checks pass (100% and 99.990% of correlatable non-graph
kernels). This is supporting timing evidence, not a complete full-run trace.
No SM/RT-core utilization, occupancy, memory-stall, or bandwidth cause was
measured.

## Correctness and numerical limits

Each method ran a separate 1,000-frame same-state shadow audit against
`info_stackless_bvh`. Candidate and active-contact multisets match throughout.
All audited per-pair CCD TOIs match, with observed maximum absolute difference
zero and no nonfinite TOIs.

| Audit | Detect checks | Active checks | TOI checks | Accumulated PT/EE TOI pairs |
|---|---:|---:|---:|---:|
| CUDA Single-EF | 13,284 | 13,380 | 6,642 | 453,377,159 |
| OptiX Single-EF | 13,328 | 13,441 | 6,664 | 437,006,077 |

OptiX executed 1,001 builds, 12,327 refits, and 13,339 launches. Both DCD and
CCD used the requested Single-EF implementation, with no fallback in this
scene. Launch counts include capacity retries.

Independent final trajectories are not identical. Paired cross-method ball
position differences are 0.266–0.892 m. Across unchanged-method repetitions,
the corresponding ranges are 0.183–1.305 m for CUDA and 0.141–0.754 m for OptiX.
Large variation also occurs in body transforms. These observations establish
neither long-trajectory equality nor physical equivalence; the same-state
audit is the collision-correctness evidence. Parallel floating-point
accumulation was preserved.

Validation completed:

- Focused GPU tests: 3,516 assertions; full core: 1,298; full CUDA backend:
  29,473,570; full `sim_case`: 14,213. No failures or skips.
- Memcheck, including OptiX resource checks, and synccheck: 3,492 assertions
  each, with zero sanitizer errors. The first memcheck launch lacked the
  XMake spdlog runtime directory and was rerun with the resolved package path.
- CPU-only OptiX-disabled build and schema test: 730 assertions, rejecting
  `single_ef_optix`. The CUDA adapter also compiles with OptiX disabled, SDK
  variables unset, and no SDK headers in its generated dependencies.
- Python benchmark/repository contracts: 19 tests passed.
- CMake isolated PTX generation, embedding, and incremental behavior passed.
  A full-project CMake build, full CUDA-enabled/OptiX-disabled backend build,
  other GPUs/platforms, and Python wheel packaging were not tested.

## Reproduction and retained evidence

Follow [the build guide](../../docs/development/single_ef_optix.md), using
CUDA 13.4 for both compiler settings. Run the hidden test for fixed-state
measurements. For each of `audit`, `throughput`, and `diagnostic`, run:

```bash
python3 scripts/benchmark_single_ef_wrecking_ball.py \
  --phase throughput --frames 1000 --warmup 30 --repeats 5 \
  --methods single_ef_bvh single_ef_optix \
  --build-dir build/xmake-optix/linux/x86_64/release \
  --output-dir output/optix-single-ef/benchmark
```

Change only `--phase` for the other two modes; only throughput uses repeats.
Local evidence remains under `output/optix-single-ef/`, including raw test and
sanitizer logs, `fixed-state-summary.json`, `trajectory-variability.json`,
benchmark sessions, runtime hashes, and `profiles/*.nsys-rep`. Exact session
paths and compact evidence are retained in the results JSON beside this record.
