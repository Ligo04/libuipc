# MAS-PNCG fused reductions and CUDA Graph replay

## Implementation scope

The MAS-PNCG nonlinear solver now has its own fused execution path. It does not
invoke `LinearFusedPCG` or add a global inner PCG solve. Local Woodbury systems
still use direct Cholesky; the existing direct 2x2 subspace solve is shared by
host reference and device execution.

- Residual evaluation combines the fixed-mass sum, maximum local ratio and
  maximum correction in one two-stage reduction and one consolidated readback.
- Direction construction combines six dot products and sparse contact
  projections. The direct subspace solve and direction/slope update run on
  device, followed by one consolidated result readback. Historical curvature
  overflow retains the existing valid one-dimensional fallback.
- Accepted history updates combine `p = fraction * direction` and `old_z = z`.
- Fixed residual/direction kernel chains reuse `cuda_tool::GraphCapture`.
  Replay is enqueued on the producer's default stream, preserving dependencies
  with gradient and SpMV work without a separate host fence.
- Captured kernels own their reduction scratch. They do not capture CUB's
  dynamic scratch allocation, host timers, scalar readbacks, or temporary host
  upload buffers. Initialization/history have separate caches. Address, shape,
  offset and scalar changes invalidate the corresponding graph. Contact count
  is a device parameter and can change without recapture when the allocation
  and launch shape remain sufficient.
- A failed capture uses the same uncaptured fused kernels. Frame statistics
  distinguish configured graph use from actual captures, replays and fallbacks.

The reference path remains available. The two independent switches are:

```python
config["nonlinear_system"]["solver"] = "mas_pncg"
config["nonlinear_system"]["mas_pncg"]["use_fused_kernels"] = 1
config["nonlinear_system"]["mas_pncg"]["use_cuda_graph"] = 1
```

`(0, 0)` selects reference operations, `(1, 0)` selects fused operations, and
`(1, 1)` selects fused operations with graph replay. Both default to `1`.
`linear_system/solver` continues to select Newton's linear solver only.

Global CCD, objective evaluation, Armijo acceptance, restart policy, iteration
budget and convergence tolerances retain their existing rules. The nonlinear
iteration is still host-driven. SpMV and MAS preconditioner application remain
outside the captured regions, and host/device synchronization remains elsewhere
in the simulation. The active-contact count upload also remains outside capture.

## Revision and environment

| Item | Value |
|---|---|
| Worktree / branch | `/home/ligo/.codex/worktrees/mas-pncg/libuipc`, `codex/mas-pncg` |
| Before | `e5fbcb0b76ebdf85027a6148422e87328be7b497` |
| After | The revision containing this record and its fused-kernel implementation |
| Worktree state | Initially clean for tracked files; measurements used this task's uncommitted patch, fingerprinted before timing |
| Samples revision | `b70f32294e31e9e3dcc12445b72fa9f4698ff5b1`, clean frozen sample checkout |
| GPU / driver | NVIDIA GeForce RTX 5090, compute capability 12.0, driver 590.48.01 |
| Toolkit / build | CUDA 13.4, nvcc 13.4.92, Linux x86_64 release, native `sm_120` |
| CUDA options | Existing `--expt-relaxed-constexpr`, `--extended-lambda`, `-rdc=true`; no task-specific compiler optimization flags |
| Python | CPython 3.12; complete isolated `runtime/uipc` package |
| Workload | Example88, two FEM bunnies and cloth, 42,611 vertices / 127,833 DoFs |
| Shared GPU load | Unrelated Isaac Kit process, PID 918156, about 2,985 MiB; process inventory recorded before each timing attempt |

The frozen runner SHA-256 is
`e5704ed68ee1c904ab782715093ccfc9e807961ab1e2c455c62417fcae731283`.
`fingerprint.json` identifies the timing sources and native libraries;
`validation.json` identifies the final formatted sources, native libraries and
validation logs. The main checkout's unrelated modifications were preserved.

## Validation protocol

Validation uses the isolated `codex/mas-pncg` worktree and CUDA 13.4 release
build on RTX 5090. The installed original wheel is preserved; the new core and
backend are loaded from a complete isolated Python package, and their hashes
are checked. The `build/python/src` directory alone is not a Python package in
this checkout, so merely adding it to `PYTHONPATH` would still load the old wheel.

The focused native operator suite includes CPU/Eigen residual and quadratic
oracles, empty/fixed/tail inputs, nonfinite inputs, one-dimensional fallback,
accepted-history updates, real graph replay after producer updates, and cache
invalidation. Public scene tests cover contact, cloth, supported materials,
animated forces, rejection of unsupported layouts and failed-frame behavior.
Python contact trajectories compare reference, fused, and graph execution for
45 frames. GPU test selection explicitly includes `-m cuda`.

## Performance protocol

The workload is canonical Example88, 20 frames at `dt=0.01`, with the same
`gradient_atol=1e-4` setting for every variant. This is the original experiment's
successful short prefix. It is not a completed 250-frame run or a remedy for the
known nonlinear failure later in the scene.

Three runs per execution mode use balanced serial orders: reference/fused/graph,
graph/reference/fused, fused/graph/reference. Throughput runs disable native
timers and geometry capture. The measured interval is native `advance()` plus
`retrieve()`, excluding initialization, stats, serialization and postprocessing.
Each attempt starts from frame 1, with no discarded warmup frames. Graph capture
and first-frame effects inside that interval are included. Other settings retain
the same scene values, including incremental updates, `gradient_rtol=1e-5`,
`max_iter=1024`, and `line_search/max_iter=8`. The complete effective configuration
is recorded in each summary.
A separate capture compares vertex positions and true native velocities. Capture
runs do not enter timing medians. The GPU is shared with an unrelated Isaac Kit
process, so small differences are workload-specific observations rather than a
dedicated-device performance guarantee.

Raw artifacts are under `output/pncg-fused-20261022/`: `fingerprint.json`,
`matrix.json`, `analysis.json`, run summaries and frame records, isolated runtime,
CPU/GPU test logs, sanitizer logs, and capture NPZ files. Prototype runs are
excluded from formal timing. The frozen Example88 runner is unchanged; a small
wrapper injects the two execution switches into its recorded scene mutations.

## Measured timing results

| Execution | Median seconds | Observed range, seconds | Reduction vs reference |
|---|---:|---:|---:|
| Reference kernels | 41.792353 | 41.744828 to 41.869893 | 0.000% |
| Fused, no graph | 41.087302 | 40.959066 to 41.120092 | 1.687% |
| Fused + graph | 40.802621 | 40.714735 to 41.104722 | 2.368% |

All nine attempts completed 20/20 requested frames and met the same configured
nonlinear stopping checks. Graph runs captured five graphs per attempt and
replayed them 5,606 / 5,608 / 5,596 times, with no capture fallback. These
observations correspond to about 1.024x end-to-end speedup for this short prefix;
they do not establish an improvement for the complete scene or every workload.

| Execution | Nonlinear direction attempts, each run | Base rebuilds, each run | Restarts, each run |
|---|---|---|---|
| Reference | 2,786 / 2,794 / 2,787 | 380 / 382 / 381 | 24 / 25 / 24 |
| Fused, no graph | 2,793 / 2,791 / 2,795 | 382 / 381 / 382 | 25 / 24 / 25 |
| Fused + graph | 2,793 / 2,794 / 2,788 | 382 / 382 / 380 | 25 / 25 / 23 |

Summation order and existing parallel assembly variability change some nonlinear
decisions. The wall-time result therefore includes iteration-count variation; it
is not a measurement of pure kernel throughput at identical iterates.

The narrow operation changes below are source-audited, not profiler timings:

| Scope | Reference operations | Fused operations |
|---|---|---|
| Residual | Sum and two maxima reduced and read separately | Two reduction kernels, one combined result readback |
| Direction with history | Six model dot products, contact projections, host direct solve, vector update and slope dot product | Combined model/projection reduction, device direct solve and direction/slope chain, one result readback |
| Accepted history | Direction scaling and separate vector copy | One kernel updates both vectors |
| Graph region | Individual kernel launches | Cached residual or direction chain launched once per operation |

No CUDA-event microbenchmark or Nsight transfer-cost measurement was collected.
SpMV, MAS construction/application, collision handling and line search were not
timed separately in this experiment. Their cost limits the end-to-end benefit.

A formatting-only pass followed timing. The timing binary, its fingerprint and
exact input sources are retained; the final rebuild/capture verification uses
the formatted sources with identical solver expressions. Stripping whitespace
from the saved and final solver header/source produces identical text; all other
production source hashes in the timing fingerprint are unchanged. The timing
backend hash starts with `88d07648ea0d`; the final backend hash starts with
`ef85c617e3e6`.

## Numerical results and failure boundaries

Final-runtime captures compare the initial state and successful frames 1 through
20, for 21 shared states. Every captured position and velocity is finite. The
table reports the largest Euclidean per-vertex difference over those states,
not a difference between object centroids. The repeat column compares two runs
of the same reference binary and configuration.

| Object | Reference vs graph position, m | Reference repeat position, m | Reference vs graph velocity, m/s | Reference repeat velocity, m/s |
|---|---:|---:|---:|---:|
| Upper bunny | 1.06228e-7 | 1.01659e-7 | 1.06228e-5 | 1.01659e-5 |
| Lower bunny | 1.13597e-7 | 7.10142e-8 | 1.13597e-5 | 7.10142e-6 |
| Cloth | 1.22394e-9 | 3.41492e-9 | 2.50626e-8 | 5.43662e-8 |

These differences have the same order of magnitude as this single unchanged-run
control. They do not prove bitwise equality, a statistical equivalence bound or
long-horizon stability. `analysis.json` also records maximum component errors,
per-frame RMS maxima and a comparison against the preserved original binary.
The independent 45-frame Python contact regression meets `atol=1e-9, rtol=0`
between the reference, fused and graph execution variants of the same model.

The final graph runtime still fails Example88 at frame 24 with
`gradient_atol=1e-4`: 23 frames complete, then the nonlinear iteration budget is
exhausted. With the original default gradient tolerance, frame 1 still exhausts
the line-search budget and zero frames complete. Both attempts exit nonzero;
their unsuccessful frame is marked incomplete and unconverged. Their summaries
and error records are retained in `boundaries.json`. This optimization does not
resolve the previously observed nonlinear failure or complete the 250-frame
scene.

## Validation results and reproduction

| Gate | Result |
|---|---|
| Native MAS-PNCG operators | 16 cases, 3,743 assertions passed |
| Native MAS-PNCG scenes | 7 cases, 1,300 assertions passed |
| Native core | 38 cases, 1,932 assertions passed |
| Python MAS-PNCG CUDA | 15 tests passed |
| Compute Sanitizer memcheck | Full 16-case operator suite passed, 0 errors |
| Compute Sanitizer racecheck | Full 16-case operator suite passed, 0 hazards |
| Compute Sanitizer synccheck | Full 16-case operator suite passed, 0 errors |
| Source checks | clang-format 18 dry run and `git diff --check` passed |

The broader aggregate simulation suite, other CUDA architectures/toolkits, and
remote CI were not run. Capture-failure fallback is implemented, but the
successful-device tests above do not inject a capture failure.

From this worktree, using its existing configured build directory:

```sh
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake build -j2 cuda
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake build -j2 mas_pncg_operators mas_pncg core
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake run mas_pncg_operators
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake run mas_pncg
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake run core
PYTHONPATH="$PWD/output/pncg-fused-20261022/runtime" .venv/bin/python -m pytest -q -m cuda python/tests/test_mas_pncg.py
```

The local experiment wrappers and frozen runner are retained under `output/`;
they are evidence artifacts, not installed public benchmark entrypoints. The
canonical sample does not provide the registry-required `benchmark_utils.py`,
so the frozen runner executes its setup AST directly. Reproduce one attempt
with the complete isolated runtime, selecting `reference`, `fused` or `graph`:

```sh
PYTHONPATH="$PWD/output/pncg-fused-20261022/runtime" .venv/bin/python output/pncg-fused-20261022/run_variant.py --execution graph --mode mas_pncg --frames 20 --pncg-gradient-atol 1e-4 --output output/pncg-fused-reproduction
```

The first prototype captured 522 graphs in 20 frames because contact counts were
part of the by-value capture key. The accepted implementation uploads that count
to a stable device variable and keys the allocation and launch shape instead;
the formal attempts each captured five graphs. Prototype timing is excluded.
Capturing SpMV, preconditioner work or the entire nonlinear loop was left outside
this change because it needs additional workspace and host-control changes.
