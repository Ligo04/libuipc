# MAS-PNCG Example88 numerical recovery and bounded contact filtering

Work is isolated on `codex/mas-pncg`, based on `e68499bb7cc4424febe546d141aac40cc7f6bd5a`.
A fresh Example88 run completed all 250 frames with exit 0, the existing paper
tolerance and iteration budget. This document distinguishes paper alignment from
engineering extensions, runtime completion from physical quality, and shared-GPU
diagnostic timings from performance comparisons.

## Configuration

```python
config["nonlinear_system"]["solver"] = "mas_pncg"
pncg = config["nonlinear_system"]["mas_pncg"]
pncg["iteration_policy"] = "paper"
pncg["ccd_mode"] = "local_conservative"
pncg["energy_guard"] = "monotone"
pncg["preconditioned_gradient_tol"] = 1e-5
pncg["max_iter"] = 1024
pncg["max_base_age"] = 0
pncg["use_fused_kernels"] = 1
pncg["use_cuda_graph"] = 1
```

`energy_guard` defaults to `none`, which preserves finite paper trials. `monotone`
is an explicit engineering extension: a finite trial is accepted when its energy
is at most the initial energy plus `32*epsilon*max(abs(initial),abs(trial))`.
It uses the existing halving and one-restart limits. There is no arbitrary unit
floor in that allowance. Strict policy retains its original Armijo condition.
Paper unit initial steps, raw-direction history, orthogonality restart, fresh-base
preconditioned-gradient stopping, tolerance and direction budget are unchanged.
A paper pass need not pass the separately recorded strict residual diagnostic.

This still uses direct local systems and the direct 2x2 subspace solve, fused
reductions/vector updates and CUDA Graph. It introduces no global PCG solve.
Local CCD proposals are followed by actual-direction native global CCD. This is
not an exact reproduction of all author implementation choices; ABD remains
unsupported.

## Evidence and changes

### Active-contact compaction

The previous filter allocated candidate-sized temporary and output arrays before
selection. With capacity growth, a PT/EE candidate could cause roughly 108 bytes
of filter allocation, excluding pair/TOI storage, CUB and overlapping allocations.
The new implementation classifies fixed chunks of 262144 candidates and appends
accepted contacts in stable source order, preserving duplicates and classifiers.
Temporary storage is bounded; output storage tracks the active prefix plus one
chunk. Pair and TOI arrays still scale with all candidates, so this is not a proof
against arbitrary broad-phase OOM.

### Coarse direct recovery

A failed native24 snapshot had an SPD source coarse matrix with condition number
2.496e10. Its stored float inverse was indefinite. Recomputing the same
Gauss–Jordan inverse in double remained indefinite and had inverse residual19.1;
the CPU inverse was SPD with residual2.98e-7. Merely changing storage precision
would not repair that failure.

When coarse inverse capture fails, the recovery validates the original double
coarse matrices with the unchanged pivot threshold, caches double Cholesky
factors and applies double restriction/triangular solves/prolongation. Invalid
source matrices still fail. It leaves the ordinary PCG buffers unchanged and
resets on the next ordinary capture. The fast capture path is not newly claimed
to validate every original matrix. A coarse recovery is observable in statistics.

### Actual descent and finite-step energy

Unequal domain scales can reverse a descending raw direction. The local CCD
selector now falls back to the validated raw direction if the finite proposal
has a nonnegative slope; native global CCD then checks that selected direction.
This is an engineering extension, preserving raw history.

Descent alone does not guarantee finite-step energy decrease. After native24–25
passed, native26 generated raw direction norms5.13e5 and1.10e7, followed by up to
2.627 billion swept candidates and a candidate-allocation OOM. With the optional
monotone guard, native26 rejected a trial with energy50.3873 from0.00282707,
completed409 accepted steps, and passed the independent residual. Its maximum
swept candidate count was663587. The76 backtracks did not change tolerance or
budget. Positive accepted energy deltas within roundoff are still counted by the
raw `energy_increase_steps` diagnostic.

The initial raw candidate detection precedes energy backtracking. This safeguard
therefore prevents observed preceding energy overshoot; it cannot guarantee that
an already excessive raw direction never exhausts candidate memory.

## Reproducibility and validation boundary

Artifacts are in `output/mas-pncg-example88-goal-20261023/`. Each run freezes native
libraries, source changes and hashes; records exact process/signal exits, GPU
samples and per-frame records; preserves native checkpoints and position/native
velocity captures. Native checkpoint recovery restores friction history; an NPZ
position/velocity restore would not provide equivalent state. Fresh250 acceptance
must remain separate from checkpoint-window diagnostics.

Foreign Kit compute usage was retained and recorded. These diagnostic runs with
capture and verbose tracing are not isolated performance measurements. No
`LD_PRELOAD` was used. Historical OOM and coarse-failure evidence is unchanged.

Verified: release build; native operators48cases/9022assertions,
strict scenes7/1300, config2/4319, Pythonconfig32tests. Bounded compaction5cases/
147assertions passes matching CUDA13.4 memcheck(explicit API), racecheck and
synccheck. New coarse/descent/energy checks pass CUDA 13.4 memcheck (explicit API), racecheck and synccheck: 8 cases / 286 assertions per tool. Python CUDA passes 39 tests, including monotone reference/fused/Graph parity.
The explicit API reporting boundary is the same as the preceding report; it does
not claim old extended-API duplicate registration diagnostics were resolved.

The25–26 recovery captures have finite positions/native velocities, positive
floor PH clearance, no nonpositive tetJ and no degenerate cloth triangles. This
is not a full mesh-mesh intersection audit or a250-frame result.

## Fresh250 result

Run: `monotone-fresh250-20261023-215904`, from native frame 0 without recovery.
Started 2026-10-23 21:59:04 CST, terminated 23:18:18 CST. The independent monitor
records process exit 0; the runner summary and all 250 consecutive successful
frame records agree. Initial state plus 250 steps produce 251 captures, with
native checkpoints at every fifth frame. No tolerance or budget was changed.

| Measurement | Result |
|---|---:|
| Frames satisfying paper preconditioned-gradient criterion | 250 / 250 |
| Frames also satisfying independent strict residual | 210 / 250 |
| Accepted directions / direction attempts | 122268 / 122268 |
| Maximum accepted steps in one frame | 1006, native 134 |
| Baseline rebuilds / orthogonality restarts | 27715 / 27465 |
| Numerical retries / unsupported-contact rebuilds | 0 / 0 |
| Monotone energy backtracks | 16332 |
| Accepted positive energy deltas within the guard allowance | 7470 |
| Local descent fallbacks / accepted non-descent local directions | 1 / 0 |
| Coarse direct recoveries | 0 |
| Graph captures / replays / fallbacks | 991 / 244786 / 0 |
| Global linear-solver iterations | 0 |
| Native advance + retrieve wall time | 4742.549 s |
| Complete runner wall time | 4753.533 s |

The independent residual is false on frames
32, 46–48, 51, 53–57, 60, 63, 66, 96, 98–99, 101, 117, 122–123,
128–129, 131, 156, 159, 164–165, 168, 173, 180, 189, 194, 198, 207,
211, 218, 225, 241, 246 and 250. These are paper-policy successes, not strict
residual successes. The coarse recovery has GPU operator regression coverage,
but its zero count here does not establish that this run exercised the recovery.

The monitor recorded 946 GPU samples. Foreign Kit compute was present in all
946; other intermittent compute applications were also recorded. Sampled peak
memory for the solver process was 4450 MiB, which is not an instantaneous peak.
The capture/trace-enabled timing is a shared-GPU diagnostic, not an equal-accuracy
speed comparison. The monitor's `performance_comparison_eligible=false` overrides
the generic runner's throughput flag. Initialization, postprocessing and native
checkpoint costs are recorded separately.

## Full geometry and trajectory comparison

CPU analysis covers native captures 0–250; the pairwise comparison covers every
successful step 1–250. Positions and native velocities are finite for all three
objects, floor PH surface clearance remains positive, and no cloth triangle is
degenerate. The analyzer does not audit all mesh-mesh intersections.

| Object | Minimum J | Minimum floor PH clearance | Frames with nonpositive J |
|---|---:|---:|---|
| Upper bunny | -0.0268034 | 2.33833 mm | 236–243, 245–249 |
| Lower bunny | -0.163642 | 1.84585 mm | 57–59 |
| Cloth | not applicable | 2.00974 mm | not applicable |

Each affected solid frame contains one nonpositive-J tetrahedron. The tight
Newton reference also has lower-body inversions at 57–59 and upper-body inversions
at 236–245 (minimum J -0.143716 and -0.0299711, respectively). The lower-body
element is the same tet 11530 in both runs and returns positive at frame 60.
The earlier strict no-inversion prefix audit failed as recorded; it is not
retroactively relabeled a passing check. This comparison does not establish the
cause of the inversions or certify general physical correctness.

The canonical scene source, asset hashes and dt match the tight Newton reference.
Topology/rest geometry and collision radius compatibility are checked by the
pairwise analyzer. Solver policies differ: the reference uses Newton velocity
tolerance 1e-4, relative tolerance 0, linear tolerance rate 1e-6 and disables its
semi-implicit stopping heuristic. MAS-PNCG uses its own paper criterion; the
unused Newton defaults in its config do not govern its termination. Exact config
differences are preserved in `reference-config-comparison.json`.

| Object | Native 250 position RMS / max difference | Peak per-frame RMS | Peak vertex difference |
|---|---:|---:|---:|
| Upper bunny | 31.477 / 109.747 mm | 56.829 mm at 219 | 440.879 mm at 218 |
| Lower bunny | 6.093 / 45.838 mm | 15.634 mm at 212 | 148.446 mm at 213 |
| Cloth | 54.177 / 183.736 mm | 54.177 mm at 250 | 232.439 mm at 205 |

These are differences between solver trajectories, not error against ground
truth. The completed run does not establish Newton-equivalent accuracy, exact
paper reproduction, ABD support, or a performance advantage.

## Evidence and delivery

`fresh250-completion-audit.json` verifies the 19 changed source files, all eight
frozen native libraries and eight frozen harness files against preflight hashes.
The validated CUDA backend SHA-256 is
`2bbf5d00a662e4b29d808dd9784af3555587c5a6ee5dcbb5cb54261541b2b6d3`.
The core library SHA-256 is
`9541015c4331ccad1bb1cc4290f5710deb527fa56b9d6e403bfa1ab69f4bebfe`.
The actual run command is in the run directory's `preflight.json`; its frozen
runtime, source snapshot, harness, log, GPU samples and termination remain local.

`run/quality.json`, `comparison-tight-reference.json`, and the final figure
manifest preserve all frame metrics and input hashes. Saved captures were
rendered through Polyscope OpenGL EGL into a real 251-frame H.264 video, 960x720
at 30 fps. Its 8.367-second playback represents 2.5 seconds of simulated time,
not the solver wall time. The renderer validates fixed camera, source hashes,
nonempty pixels and ffprobe frame count.

The updated [web report](http://10.136.26.230:39961/pncg-vs-pcg/#example88-recovery)
provides the complete run, quality/trajectory tables, PNG/SVG plots, video and
downloadable evidence. Historical partial runs and failures remain visible.
No further GPU rerun is needed for this scoped change; full aggregate tests,
other devices and remote CI remain outside the verified boundary.
