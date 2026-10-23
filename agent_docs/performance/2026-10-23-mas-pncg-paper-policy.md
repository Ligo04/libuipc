# MAS-PNCG paper step, restart and termination policy

The optional `paper` policy follows Algorithm 1 and Section 4.2 of
[MAS-PNCG, arXiv:2604.19892v1](https://arxiv.org/html/2604.19892v1#S4).
It corrects the timing of orthogonality restarts and adds the paper's
fresh-preconditioner termination criterion. The existing `strict` policy remains
the default. This record does not describe the older Xingbaji/PNCG_IPC method.

## Configuration

```python
config["nonlinear_system"]["solver"] = "mas_pncg"
pncg = config["nonlinear_system"]["mas_pncg"]
pncg["iteration_policy"] = "paper"       # default: "strict"
pncg["preconditioned_gradient_tol"] = 1e-5
pncg["max_base_age"] = 0                  # disable the native periodic refresh
```

`preconditioned_gradient_tol` is a finite positive absolute length tolerance.
It uses the global L2 norm of free `z=P*g`, without RMS, mass normalization or a
relative initial-value scale. The main paper writes an unsubscripted norm; L2 is
our explicit interpretation. Its Section 5.1 specifies `1e-5` for the Teapots
comparison, not a universal tolerance for every scene.

`max_base_age` is an existing independent option. Its default remains 32; the
reported paper-policy Example88 runs use zero to isolate the paper's restart
schedule. The nonlinear direction budget remains 1,024. Newton's fused PCG is
not called by either nonlinear policy.

## Behavior

The paper policy retains the existing MAS baseline, positive contact updates,
Woodbury local solves and direct 2x2 subspace solve. It takes the resulting raw
search direction with initial trial fraction one, and does not add a separate
`-gTp/pHp` scalar step or the old PNCG-IPC `0.5*d_hat` displacement rule.

The current gradient and preconditioned gradient are evaluated at the accepted
state before the proposal. The ratio

```
r = abs(g_current.dot(z_previous)) / g_current.dot(z_current)
```

is calculated before overwriting `z_previous`. A ratio above the existing
`restart_threshold` schedules a rebuild for the next iteration after the current
valid step is accepted. It no longer discards that step or consumes an extra
same-state retry. Fresh rebuild iterations also evaluate this ratio. The initial
previous gradient is zero; an objective change clears it. Actual numerical
failures still use the existing bounded retry mechanism.

Paper mode stores the raw search direction as history. Strict mode retains its
accepted-displacement history. For global uniform step fractions these span the
same direction in exact arithmetic, so the storage change alone is not asserted
to explain a convergence improvement.

The paper convergence check uses the **pre-step** norm from an iteration that
started with a full MAS rebuild. It is applied after accepting that iteration's
position update. The engine subsequently refreshes the actual post-state
gradient/residual for reporting without replacing the cached stopping evidence.
A truly zero free gradient can terminate without an artificial update; an
explicit component check prevents norm underflow from creating that shortcut.

The new policy requires finite native energy and a positive feasible fraction;
it does not impose strict mode's Armijo comparison. Counts of accepted native
energy increases remain visible. The supplement's Section 2.5 confirms the unit
initial trial and CCD restriction. No author implementation was located, so this
is an implementation of the published algorithm and equations, not a claim of
bitwise agreement with unpublished code or every detail of its timing labels.

## Boundaries of the alignment

- Collision handling retains libuipc's global CCD, feasible-step filter and
  conditional CFL. The paper's per-subdomain conservative CCD is not implemented
  by this change. The `ccd_mode` statistic continues to say `global`.
- Existing max-age refresh, finite/descent/pivot checks, failure reporting,
  constitutions and contact model remain native choices.
- The earlier full mollified-EE curvature extension is retained.
- The main text describes double-precision simulations. Supplement Section 1.1
  describes MAS blocks assembled/stored in double and inverse matrices stored
  in single precision. This backend retains double Level-0 direct solves and
  its existing single-precision coarse MAS application; precision is not changed
  by the iteration-policy selector.

The supplement was obtained from the official
[arXiv source archive](https://arxiv.org/src/2604.19892v1), member
`MAS_PNCG_sig26_axriv_supp.pdf`. The PDF, extracted text, main TeX source and
SHA-256 provenance are preserved under the experiment's `sources/` directory.

## Statistics

- `iteration_policy`, `convergence_criterion`, and
  `independent_residual_converged` separate selected-policy success from the
  stricter native residual contract.
- `preconditioned_gradient_norm`, `preconditioned_gradient_tolerance` and
  `convergence_checked_on_rebuild` expose the cached stopping evidence.
- `orthogonality_restarts` counts next-iteration rebuild requests.
- `numerical_retries` counts same-state retries. Strict mode's existing immediate
  orthogonality rejection also uses that retry mechanism.
- `energy_increase_steps`, `last_energy_before`, `last_energy_after` expose native
  energy acceptance. An increase count alone does not measure its magnitude.

A paper-policy frame can pass its criterion while
`independent_residual_converged=false`; both results must remain visible.

## Validation and provenance

Worktree `/home/ligo/.codex/worktrees/mas-pncg/libuipc`, branch `codex/mas-pncg`,
base commit `c342910a4b5de2cf159428c79f9144e809a38ed7`. CUDA 13.4 release,
RTX 5090, native architecture. The separate runtime and source/native hashes are
recorded in `output/mas-pncg-policy-20261023/fingerprint.json`.

- Native configuration: 2 cases / 3,028 assertions passed.
- Native operators: 25 cases / 6,550 assertions passed. New cases prove delayed
  restart without direction loss, comparison across fresh rebuilds, previous-z
  initialization, graph invalidation when changing policy, and exact-zero
  component detection. Existing dense quadratic/local direct-solve tests remain.
- Native strict scenes: 7 cases / 1,300 assertions passed.
- Python configuration: 25 tests passed.
- Python CUDA: 26 tests passed. Cases include fixed/rest states, finite tet-plane
  dynamics, fresh-rebuild stopping, reference/fused/graph agreement, explicit
  budget failure, mollified EE, strict behavior and Newton solver availability.
- Compute Sanitizer: all 25 operator cases / 6,550 assertions passed under
  memcheck, racecheck and synccheck. Memcheck and synccheck report zero errors;
  racecheck reports zero hazards, errors and warnings.

The config tests now compare public schema/default JSON instead of stale fixed
entry counts. Zero-gravity Python fixtures use the supported nested 3x1 JSON
shape and verify the effective value; a flat array had retained default gravity.

No original `.venv` native file or previous validated runtime was overwritten.
The earlier experiment based on the wrong, older PNCG reference was archived in
`output/pncg-policy-20261023/superseded-source/` and `superseded.patch`, and removed
from the production diff. Its 44/250-frame result is not MAS-PNCG paper evidence.

## Example88

Uses the existing frozen canonical scene/runner, 42,611 FEM vertices,
127,833 DoFs, `dt=0.01`, paper tolerance `1e-5`, global CCD, and disabled max-age
refresh. The independent gradient tolerances remain at their defaults for
reporting. Captures include native velocities.

A first-frame run completed 403 direction attempts and accepted all 403 steps.
It scheduled 15 orthogonality restarts and had zero same-state retries. It stopped
on a fresh rebuild with pre-step `||P*g||=1.792679029e-14`; the post-state global
residual was `1.286422702e-13`, local residual ratio `9.130018901e-5`, and the
independent residual check also passed. It reported 64 energy increases, 0
unsupported-contact rebuilds and 0 graph fallbacks.

Against the prior tight Newton first-frame capture, maximum bunny vertex
position difference was 4.341 mm, RMS about 0.0764 mm. Cloth differences were at
floating-point noise. Those differences are not a statement of identical
physical accuracy or solver trajectories.

The 250-frame attempt completed **26 frames** and aborted during frame 27 with
`cudaMalloc: out of memory` (exit 134). Frame 27 has no convergence verdict; this
is not an iteration-budget failure. The runner's original summary remained at
`running` because the native abort bypassed its exception handler. A separate
`graph-250/termination.json` records the observed termination without altering
the original summary.

Across the 26 completed frames, all 8,704 direction attempts were accepted;
108 orthogonality restarts were scheduled, with no same-state numerical retries,
unsupported-contact rebuilds or graph fallbacks. All 26 also passed the
independent residual check. There were 1,267 native energy increase events;
their count alone does not establish the size or consequence of those changes.
The five graph captures occurred during initial setup; 17,434 replays followed.

The 27 captured states (initial state plus 26 steps) have finite positions and
velocities, no nonpositive tet Jacobians, no degenerate cloth triangles, and no
negative floor vertex clearance. Minimum tet Jacobians were 0.59868 for the
upper bunny and 0.30513 for the lower bunny. Minimum floor clearance was
2.1463 mm. These checks do not independently audit all mesh-mesh intersections.

At frame 26, against the tight Newton capture, lower/upper bunny maximum vertex
differences were 4.411/4.324 mm, with RMS differences 0.0682/0.0797 mm. Cloth
maximum difference was `3.39e-10 m`. The common-prefix comparison is preserved
in `prefix26-newton-comparison.json`; this is not full-episode equivalence.

The completed prefix took 132.888 s in native advance plus retrieve wall time.
It is one captured run with a different stopping policy and does not establish
an equal-accuracy speedup.

A separate 27-frame attempt with an external `cudaMalloc` tracing shim also
completed 26 frames and aborted in frame 27. The failed allocation requested
2,475,979,104 bytes; CUDA reported 2,353,922,048 free bytes immediately afterward.
The call stack identifies `InfoStacklessBVHSimplexTrajectoryFilter::Impl::filter_active`.
Successful allocations in that phase had grown from hundreds of MB to multiple
GB. This confirms the immediate failure site in candidate filtering. It does
not yet identify which proposal caused the candidate growth or establish that
the same allocation would fail with an otherwise idle GPU. Other GPU workloads
were present during these attempts; neither is an uncontended performance run.
The tracing shim did not change the production source or runtime binary.

The user subsequently requested an immediate repeat, superseding the wait for
the idle threshold. The wait automation was deleted to prevent duplicate runs.
The repeat uses the same source/native fingerprints, a fresh output directory and
five-second GPU/process snapshots, without the allocation tracing shim. Its
results are recorded separately from the previous attempts.

### Immediate repeat: 31/250 frames, frame 32 baseline validation failure

The repeat started on 2026-10-23 at 17:16:42 China Standard Time and ended at
17:22:06 with exit code 1. It completed **31/250 frames**. All 66 GPU/process
snapshots contain no competing CUDA compute application; ordinary desktop
graphics remained. The sampled peak process GPU memory was 10,512 MiB. Sampling
does not exclude an unsampled short-lived process or establish exact peak memory.

Frame 27 completed in 85.095 s with 896 accepted steps and no numerical retries.
The previous allocation failure did not reproduce in this attempt. Because
parallel floating-point accumulation can change the nonlinear trajectory, the
successful repeat alone does not establish that external occupancy caused the
earlier OOMs.

Frame 32 failed with `MAS-PNCG could not capture an SPD double MAS baseline.`
It had accepted 59 steps and completed 34 successful base rebuilds before the
failing capture. Neither the iteration-budget nor line-search-limit flag was
set; no CUDA OOM was reported. Its last reported residual was 4,930.081, versus
0.2460 at frame start, and its last computed preconditioned-gradient norm was
1.7114, above the `1e-5` tolerance. These are the last available statistics,
not a new residual evaluated after the failed capture.

The failure originates at `MASPNCGSolver::evaluate()` after
`FEMMASPreconditioner::capture_nonlinear_base()` returns false. That interface
combines Level-0 finite/symmetry/Cholesky checks, coarse inverse validation,
partition mapping and unpartitioned diagonal validation. It does not report
which condition or domain failed. Thus the message does not prove that the
double Level-0 Hessian itself is indefinite, or that FP32 coarse precision is
the cause. Those alternatives require per-check diagnostics in a later change.

Of the 31 completed frames, 30 also passed the independent native residual test.
Frame 28 passed the selected paper criterion with a fresh-rebuild pre-step norm
`5.735527237e-6` but did not pass the independent residual test. Across the
completed prefix there were 11,117 accepted direction attempts, 744 scheduled
orthogonality restarts, zero numerical retries, zero unsupported-contact
rebuilds, 9 graph captures and 22,265 graph replays with no fallback. Native
advance/retrieve wall time was 312.977 s; the failed attempt added 5.888 s.
The different criterion and incomplete episode preclude an equal-accuracy
performance claim.

The 32 captures (initial state plus frames 1-31) have finite positions/velocities,
no nonpositive tet Jacobians, no degenerate cloth triangles and no negative
floor vertex clearance. The minimum lower-bunny Jacobian is 0.30513 and minimum
floor clearance is 2.1463 mm. There is no accepted frame-32 capture, so these
checks do not validate the failed state or all mesh-mesh intersections.
At frame 31, maximum upper/lower bunny position differences from tight Newton
are 4.327/4.329 mm, RMS 0.0795/0.0840 mm; the cloth maximum is `4.75e-9 m`.

Evidence is under
`output/mas-pncg-policy-20261023/run-now-20261023-171642/`: `termination.json`,
`run.log`, `preflight.json`, `gpu.jsonl`, `analysis-summary.json`, `quality.json`
and `newton-comparison.json`, with untouched raw records and captures in `run/`.
The monitor verifies code/native hashes, removes `LD_PRELOAD`, serializes launch
with a task-local file lock and records the subprocess exit independently of
the benchmark summary. No parameter sweep or relaxed-tolerance retry followed.

## Reproduction

```bash
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake build -j2 cuda mas_pncg_operators mas_pncg core
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake run core '[scene][config]'
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake run mas_pncg_operators
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake run mas_pncg
PYTHONPATH="$PWD/output/mas-pncg-policy-20261023/runtime" .venv/bin/python -m pytest -q python/tests/test_scene_config_schema.py python/tests/test_mas_pncg.py
PYTHONPATH="$PWD/output/mas-pncg-policy-20261023/runtime" .venv/bin/python output/mas-pncg-policy-20261023/run_variant.py --execution graph --mode mas_pncg --max-base-age 0 --frames 250 --capture --output NEW_OUTPUT
```

Use a fresh output directory to preserve earlier evidence. Captured runs and
runs with different stopping criteria do not establish equal-accuracy speedups.
