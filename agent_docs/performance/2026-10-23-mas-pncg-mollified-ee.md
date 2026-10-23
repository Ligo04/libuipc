# MAS-PNCG mollified EE incremental curvature

## Problem and implementation

At revision `bb1670abebd887268b90f7175c6ded4afde85f15`, an active near-parallel
edge-edge contact was exported as unsupported by the PNCG increment builder.
Every subsequent encounter rebuilt the complete MAS baseline and cleared the
search history. In Example88 frame 24, this produced 1,024 base rebuilds and
1,023 one-dimensional directions, and exhausted the unchanged direction budget.

The EE exporter now calls the same `mollified_EE_barrier_gradient_hessian`
function as native contact assembly, checks finite derivatives and applies the
same `make_spd` projection. The global gradient already uses the native
mollified expression; exported gradients are checked rather than scattered a
second time. Ordinary and mollified EE contacts both retain their full PSD
curvature, because the restart matrix contains the full native EE Hessian.
Subtracting a rank-one surrogate from that matrix would count some old
curvature again during a transition.

For each matching canonical EE stencil, let `C_k` be its current PSD Hessian
and `C_b` its frozen restart Hessian. The update is

```text
Delta = C_k - C_b
Delta_plus = V diag(max(lambda, 0)) V^T = U U^T
H_model = H_base + sum(U U^T)
```

This computation always subtracts the restart snapshot, not the preceding
iteration's update. With exact arithmetic, `C_b + Delta_plus - C_k` is PSD.
Stiffness decreases are retained in the baseline until a normal restart.
Positive eigenvalues below `64 * epsilon * max_entry_scale(C_k, C_b)` are
discarded as numerical rank noise. The eigensolver normalizes the difference
before decomposition. Fixed degrees of freedom are masked after decomposition.

Each EE Hessian uses 78 doubles in a separate packed symmetric buffer. Ordinary
PT/PE/PP/plane contacts keep their existing compact rank-one representation.
The current and baseline buffers have independent storage. EE identities use
canonical vertex stencils and duplicate occurrences, independently of closest
features and eigenvector labels. Canonicalization permutes the Hessian's row
and column blocks together with the vertices.

An EE update can produce up to 12 sparse columns. Every column contributes to
the global direction's projected curvature, including in fused/Graph execution.
Only the local MAS preconditioner applies the existing top-k truncation, with
the existing Woodbury Cholesky/direct fallback. CCD, Armijo acceptance,
convergence tolerances and iteration budgets retain their previous values.

`Engine.frame_stats()` adds `ee_curvature_contacts`, `mollified_contacts` and
`peak_mollified_contacts`, the maximum observed across exports in that frame.
The existing `contact_update_columns` counts used increment columns, not
physical contacts.

## Provenance and validation scope

Worktree: `/home/ligo/.codex/worktrees/mas-pncg/libuipc`, branch `codex/mas-pncg`.
The tracked checkout was clean at the before revision above. Validation uses
this change's uncommitted source patch, recorded by file hash in
`output/pncg-mollified-ee-20261023/fingerprint.json`. The after revision is the
commit introducing this record. The original installed wheel and previous
experiment runtime are preserved; a complete independent Python package loads
the rebuilt native backend.

Environment: RTX 5090, driver 590.48.01, CUDA 13.4 / nvcc 13.4.92, Linux x86_64
release with native `sm_120`, CPython 3.12. Existing CUDA build options include
`--expt-relaxed-constexpr`, `--extended-lambda` and `-rdc=true`. No task-specific
compiler optimization flags were added. An unrelated Isaac Kit process shares
the GPU, so timings are scoped observations.

The new operator cases use actual energy/gradient/Hessian routines, central
finite differences with fixed rest geometry, an independent CPU Eigen PSD
oracle, noncommuting baseline differences, unchanged/softening/stiffening
states, ordinary/mollified and closest-feature transitions, endpoint
permutations, duplicate contacts, fixed masking, local direct solves and
multi-column reference/fused/Graph direction checks. Tests compare matrix
operators rather than eigenvector signs or ordering.

The near-parallel native Python scene runs four frames in each execution mode,
requires actual mollifier activation, finite positions, fixed-vertex invariance,
independent convergence and no unsupported-contact rebuild. It also requires
nonzero increment columns and two-dimensional search directions.

## Results

| Check | Result |
|---|---|
| Native operators | 23 cases, 6,467 assertions passed |
| Python CUDA suite | 16 tests passed, including the new EE scene |
| Operator memcheck / racecheck / synccheck | All 23 cases passed; 0 errors / 0 hazards |
| Native Python EE scene under memcheck | Passed; 0 errors |
| Original 7 native scenes | Three consecutive full new-binary runs passed, 1,300 assertions each; intermittent cloth failure described below |

The initial full native scene run failed in the cloth self-contact case at
frame 20. Its local residual ratio was 1.0728 and its global residual was about
1.03e-10. Armijo trials differed from the accepted energy by about 2.4e-21 J.
Alternating the preserved old binary and new binary reproduced the same class
of failure in the old binary at frame 22, with local ratio 1.3694 and global
residual about 9.13e-11. The two focused cloth runs per binary passed, followed
by three full-suite new-binary passes and two passes/one failure in the old
binary. This establishes that the symptom exists in both versions; it does not
prove equal failure probability or eliminate the numerical stability issue.
All attempts are retained in `cloth-repeats.json`, `scene-repeats.json` and logs.
No tolerance or acceptance rule was changed to obtain those passes.

### Example88 frame 24

With the same `gradient_atol=1e-4` configuration, the new 24-frame capture
completed every frame. The old run completed 23 frames before failure.

| Frame-24 metric | Before | After |
|---|---:|---:|
| Completed / converged | No | Yes |
| Nonlinear direction attempts | 1,024 | 162 |
| Base rebuilds | 1,024 | 14 |
| Unsupported-contact rebuilds | 1,023 | 0 |
| One-dimensional directions | 1,023 | 14 |
| Two-dimensional directions | 1 | 138 |
| Final local residual ratio, required <= 1 | 55.89945 | 0.74528 |

The new frame observed six mollified contacts. The step budget and stopping
checks are unchanged. This is a convergence result; the old unsuccessful
attempt's elapsed time is not treated as a valid speedup denominator.

The default gradient tolerance still fails at frame 1 on the line-search budget,
with zero completed frames. This EE integration does not resolve that existing
default-tolerance failure.

### Extended attempt

A fresh 250-frame attempt with `gradient_atol=1e-4` completed 46 frames and
failed frame 47 on the unchanged 1,024-direction limit. At failure it had
accepted 687 steps, used 338 one-dimensional and 349 two-dimensional directions,
and recorded 337 direction-retry restarts. The final local residual ratio was
35.98781. There were 24 peak observed mollified contacts, zero
unsupported-contact rebuilds, zero nonfinite trials, and zero Graph fallbacks.
The line-search limit flag was false. Per-direction reason counters were not
recorded, so the retry count alone does not identify which direction check
triggered each restart.

The 46 successful frames used 10,010 direction attempts, 1,319 base rebuilds,
11 graph captures and 20,066 graph replays. Native advance/retrieve time for
those successful frames was 215.700 s. Including the failed frame's attempt
gives 258.378 s; initialization and capture postprocessing are separate. This
is one capture run with native timers disabled on a shared GPU, with no discarded warmup
frames, not a completed 250-frame timing result or a repeated throughput claim.

The continuous 47 captured states, including frame 0, have finite positions and
velocities, no nonpositive tetrahedral J and no degenerate cloth triangles.
Minimum J is 0.59868 for the upper bunny and 0.30513 for the lower bunny. Minimum
floor PH vertex clearances are 0.37251 m, 0.002146 m and 0.003228 m for upper,
lower and cloth. These checks cover only the successful prefix, not frame 47.
See `extended-runs.json` and `graph-250-quality.json` for the failed run status
alongside successful-prefix geometry metrics.

### Captured state comparison

Common states 0 through 23 compare the old and new binaries under the relaxed
configuration. Initial positions match exactly. Maximum vertex position
differences are 3.403 mm for the upper bunny, 1.261 mm for the lower bunny and
3.699e-7 m for the cloth. Maximum per-frame position RMS differences are
3.380e-5 m, 1.736e-5 m and 3.304e-7 m respectively. Maximum vertex velocity
differences are 0.3403 m/s, 0.1257 m/s and 2.870e-6 m/s. These are algorithmic
trajectory differences under loose tolerances, not bitwise equivalence or
reduction-order noise.

Against the previously captured tighter Newton run, the maximum bunny vertex
position discrepancy is about 4.341 mm for both old and new PNCG. Maximum
per-frame RMS discrepancies change from 7.361e-5 / 7.912e-5 m to
8.099e-5 / 8.099e-5 m for upper/lower bunnies. This comparison alone does not
establish equal physical accuracy or a more accurate PNCG trajectory.

The new 24-frame capture has finite positions and velocities, no inverted or
collapsed tetrahedra, no degenerate cloth triangles, and no negative floor PH
vertex clearance. Minimum tetrahedral J is about 0.59868. Floor clearance is a
floor-only metric; it does not independently audit every self-contact or
bunny-cloth intersection. Full metrics and conventions are recorded in
`graph-24-quality.json`, `prefix-quality.json` and
`newton-prefix-comparison.json`.

Full EE Hessian evaluation and 12x12 spectral decomposition add work to each
incremental step. No isolated CUDA-event performance measurement or repeated
end-to-end throughput comparison is claimed by this record.

## Reproduction

From the worktree and its existing configured CUDA build:

```sh
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake build -j2 cuda
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake build -j2 mas_pncg_operators mas_pncg
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake run mas_pncg_operators
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake run mas_pncg
PYTHONPATH="$PWD/output/pncg-mollified-ee-20261023/runtime" .venv/bin/python -m pytest -q -m cuda python/tests/test_mas_pncg.py
```

Example88 uses the unchanged frozen runner and sample checkout from the prior
comparison, sample revision `b70f32294e31e9e3dcc12445b72fa9f4698ff5b1`.
Its wrapper injects only the execution switches. The scene has 42,611 vertices,
127,833 DoFs and `dt=0.01`. The relaxed experiment keeps `gradient_atol=1e-4`,
`gradient_rtol=1e-5`, `max_iter=1024` and eight line-search trials. The complete
effective config and loaded binary hashes are recorded per attempt.

```sh
PYTHONPATH="$PWD/output/pncg-mollified-ee-20261023/runtime" .venv/bin/python output/pncg-mollified-ee-20261023/run_variant.py --execution graph --mode mas_pncg --frames 24 --pncg-gradient-atol 1e-4 --capture --output output/pncg-ee-reproduction
```

Raw validation, alternating old/new scene runs, sanitizer logs and experiment
outputs remain under `output/pncg-mollified-ee-20261023/`. Captures and sanitizer
runs do not enter throughput comparisons. Full CUDA architecture/toolkit
coverage and remote CI are outside this local validation.
