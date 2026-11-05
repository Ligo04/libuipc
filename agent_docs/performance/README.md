# Performance Evidence

This directory stores durable, reviewable performance conclusions. The
chronological `handoff.md` retains raw session history; accepted/rejected
experiments should gain a focused record here so future agents can distinguish
measurement from intuition.

## Evidence hierarchy

1. Correctness and safety gates first: focused tests, full aggregate simulation,
   and sanitizer coverage appropriate to the change.
2. Measure the narrow kernel/scope that the change targets.
3. Measure the enclosing subsystem and end-to-end benchmark.
4. Repeat noisy wall measurements and report variation; do not turn one run
   into a claim.

Kernel resource counts, launch counts, or attractive assembly alone do not prove
a speedup. Conversely, contact-sensitive wall time can hide a real scoped gain
when iteration counts change. Report both and state which is the primary signal.

## Records

| Record | Status | Scope |
|---|---|---|
| [2026-09-01 cross-domain main baseline](2026-09-01-cross-domain-baseline.md) | Current reference | Three-run ABD, FEM/MAS, cloth/contact envelope and synchronized stage diagnostics |
| [2026-08-30 case2 assembly roll-up](2026-08-30-case2-assembly-rollup.md) | Accepted with caveats | Buffer growth, collision readback, line-search aggregation, contact/FEM assembly, rejected split |
| [2026-11-05 X_square SingleEF](2026-11-05-xsquare-single-ef-bvh.md) | Inconclusive | One 1000-frame native wrecking-ball A/B pair and separate synchronous Timer diagnostics |
| [2026-11-05 X_square WB and EX88](2026-11-05-xsquare-wb-ex88.md) | Inconclusive | Sample-6 WB and sample-88 FEM/cloth, 1000-frame A/B and separate Timer diagnostics, with inversion and trajectory limits |
| [2026-11-05 SingleEF query integration / CUDA 13.5](2026-11-05-xsquare-query-cuda135-wb-ex88.md) | Single-group observation | WB/EX88 1000-frame A/B and independent profile; iteration/trajectory and negative-J limits |
| [2026-11-05 SingleEF bounded refresh and cross-frame reuse](2026-11-05-singleef-query-followup-wb-ex88.md) | No whole-frame speedup established | Four implemented mechanisms; frozen SingleEF before/after, 1000-frame WB/EX88 formal/profile with work/trajectory and inversion limits |
| [2026-11-05 shared simplex query policies](2026-11-05-shared-simplex-query-wb-ex88.md) | Mixed single-group observations | Unified Info/SingleEF, WB/EX88 1000-frame formal/profile; solver-work, trajectory and negative-J limits |
| [`handoff.md`](../handoff.md) | Historical source | Earlier MAS, CUDA graph, CUB, DyTopo, lifecycle, and detailed command history |

## Required content

Every new record should identify:

- before/after commits and whether the worktree was clean;
- GPU, driver, CUDA toolkit, build type/flags, and benchmark/submodule revision;
- scene/configuration, warmup, measured frames, repetitions, and timer semantics;
- focused, subsystem, and wall metrics with units and sample counts;
- correctness/sanitizer results and observed numerical variability;
- rejected alternatives and the decision taken;
- artifact paths or commands sufficient to reproduce the run.

Use [0000-evidence-template.md](0000-evidence-template.md). Canonical large
scenes are launched through `benchmarks/manifest.json` and
`scripts/run_benchmark.py`; its archived metadata supplies the minimum runtime
and revision facts. Approved regression thresholds belong in versioned
`uipc.profile` baseline artifacts with environment compatibility checks.
The 2026-09-01 shared-WDDM baseline is deliberately a measured range rather
than an enforced threshold; promote it only on a stable dedicated runner.

## Interpretation rules

- Do not add percentages from different stages as if they were one controlled
  experiment.
- Prefer median plus distribution/repeats for wall time; keep raw means when
  comparing to historical reports that used means.
- Parent timers include children unless explicitly documented otherwise.
- A different Newton/PCG/contact count is an algorithmic-path change, not pure
  throughput evidence.
- Atomic-order trajectory drift must be compared with unchanged-run variability,
  not assumed to be either a bug or harmless.
- Record regressions and rejected designs; they prevent expensive repetition.
