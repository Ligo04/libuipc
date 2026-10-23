# MAS-PNCG local conservative CCD and baseline diagnostics

This change continues the paper-policy work from `7f06857fba4f7f87d227d758432b5fe55214c307`
in the isolated `codex/mas-pncg` worktree. It adds opt-in per-domain conservative
step proposals followed by native global CCD on the actual scaled direction.
It is a partial paper alignment, not a bitwise reproduction of the authors' code,
and it does not establish that global numerical locking has been eliminated.

## Configuration and unchanged behavior

```python
config["nonlinear_system"]["solver"] = "mas_pncg"
pncg = config["nonlinear_system"]["mas_pncg"]
pncg["iteration_policy"] = "paper"
pncg["ccd_mode"] = "local_conservative"
```

`ccd_mode` defaults to `global`; `iteration_policy` defaults to `strict`.
Local CCD requires `paper` when MAS-PNCG is selected, both in scene validation and
at engine construction. The existing global/strict path remains available.
There is no inner global PCG solve. MAS local direct solves, the direct 2x2
subspace solve, fused reductions/vector updates and CUDA Graph remain in place.
This change does not alter stopping tolerances, iteration budgets, supported
constitutions, coordinate representation, or the baseline SPD thresholds.

## Relation to the paper

Source: [MAS-PNCG v1](https://arxiv.org/html/2604.19892v1), Algorithm 1 and
Section 4.4; supplementary material Sections 3.2 and 3.4, Algorithms 1/2.
The saved primary sources are in `output/mas-pncg-policy-20261023/sources/`.

1. Compute the raw PNCG direction and retain it for the next history update.
2. Generate trajectory PT/EE candidates over the complete raw unit step.
3. For each pair, compute the tighter relative-displacement bound. Analyze the
   coplanarity cubic by derivative critical points and at most 16 bisections.
4. Reduce the safe fraction into every incident movable Level-0 domain, using
   the captured MAS mapping and its existing cluster size of 16 vertices.
   Unpartitioned vertices use separate singleton domains. Fixed vertices are
   excluded. Planes use the native 10% separation margin and contact/subscene masks.
5. Write the locally scaled direction into a separate trial vector, scatter it
   through the normal FEM interface, and recollect global displacements.
6. Run the existing feasible-step filter, regenerate candidates for that actual
   direction, then apply native global CCD and conditional CFL. Energy evaluation
   uses the same actual displacement. The original raw direction remains history.

The final displacement is `gamma * D(alpha_d) * p`, where `gamma` is the final
global fraction and `D` applies domain fractions. This extra global verification
is intentional: independently shrinking one side of a contact changes relative
motion. Two primitives translating equally have no raw relative motion, but can
collide after a third constraint slows only one domain. A device regression test
constructs that case and verifies a positive-separation native CCD bound.

Engineering extensions and limits:

- The cubic implementation handles quadratic/linear limits and tangent roots;
  it uses a roundoff uncertainty band and contracts a non-unit boundary by 10%.
  These details are not a claim of exact author implementation equivalence.
- A coplanarity polynomial alone does not certify positive-thickness clearance.
  Positive thickness, coplanarity, or uncertain derivative roots with a positive
  certified prefix use native ACCD as a fallback. Degenerate geometry, invalid
  inputs or nonpositive initial gap that yield zero prefix fail explicitly.
- Local proposals currently cover surface PT/EE and vertex/half-plane contacts.
  The final native collision pipeline retains its broader checks.
- Unequal positive scaling can change the total directional derivative. We
  record the actual derivative and count non-descent trials without adding an
  undocumented Armijo rule to the paper policy.
- The extra full-direction candidate pass, local calculations, reductions and
  global verification have costs. Local CCD is not captured into the PNCG
  direction CUDA Graph, and no acceleration is claimed without measurement.
- ABD, mixed FEM/ABD, BDF2 and unsupported constitutions remain unsupported.

## Baseline capture diagnostics

A failed snapshot now records `baseline_capture_failure` in frame statistics:
`stage`, `reason`, `epoch`, `level`, `domain`, `index`, `row`, `column`, `value`,
`threshold`. The exception also includes the identifying fields. It distinguishes
host/size/mapping failures, Level-0 matrices, coarse inverse matrices, and
unpartitioned diagonal matrices; reasons include nonfinite values, asymmetry,
nonpositive Cholesky pivots and nonfinite factors/inverses.

No acceptance threshold was weakened. Device selection is deterministic:
Level-0 domain order, then coarse cluster order, then real mapping, followed by
FEM unpartitioned vertices. Each domain has one record writer; other lanes reduce
integer failure indices before publication. Diagnostics identify a failed check,
not necessarily the upstream cause of a bad matrix.

## Validation

Focused validation on the RTX 5090, native architecture, CUDA 13.4.3 release:

- Build: CUDA backend, native config and MAS-PNCG operators, XMake.
- Native config: 2 cases / 3,634 assertions.
- Native operators: 40 cases / 8,725 assertions. Includes 9 CPU conservative
  math cases / 1,976 assertions, device parity, thick PT/EE fallback/domain
  propagation, unchanged raw history and unequal-domain relative-motion safety.
- Native strict scenes: 7 cases / 1,300 assertions.
- Python config: 27 tests; CUDA MAS-PNCG: 37 tests. Covers existing strict paths,
  paper/global and paper/local resting/fixed states, rebuild stopping, budget
  failure, reference/fused/Graph consistency and unconstrained local domains.
- CUDA 12.8 system synccheck was mistakenly used first against CUDA 13.4 code;
  it reported CUB DeviceScan missing-init errors. That attempt is retained in
  `sanitizer-cuda128/` and is not the matching-tool validation.
- CUDA 13.4.92 Compute Sanitizer 2026.3 memcheck in default extended API mode
  reported 3 duplicate template-kernel registration diagnostics while all tests
  passed. `memcheck-api-extended.log` retains this result. The comparable gate
  uses `--report-api-errors explicit`, matching the preceding implementation's
  checks. It still checks device memory and explicit application API failures,
  but does not claim the internal registration diagnostics are resolved. See
  [NVIDIA's reporting-mode definitions](https://docs.nvidia.com/compute-sanitizer/ComputeSanitizer/index.html#cuda-api-error-checking).

The matching-tool memcheck (explicit API mode), racecheck and synccheck each
passed all 40 cases, with zero reported memory errors, race hazards or sync errors
within those reporting settings.

The adversarial unequal-domain test exercises the pair CCD and domain scale
operators; it does not by itself prove the complete engine candidate-redetection
pipeline on every adversarial geometry. Full aggregate simulation, CMake runtime,
other devices and remote CI are outside this validation.
Raw logs, fresh runtime copies, source/native SHA256 fingerprints and GPU samples:
`output/mas-pncg-local-ccd-20261023/`. Earlier 26-frame OOM and 31-frame baseline
failure evidence under `output/mas-pncg-policy-20261023/` is preserved.

## Bounded Example88 results

Both runs used RTX 5090 / CUDA 13.4.92, 42,611 FEM vertices / 127,833 DoFs,
`dt=0.01`, `paper`, tolerance `1e-5`, 1,024 direction attempts, `max_base_age=0`,
fused kernels + Graph, and capture. No `LD_PRELOAD`, budget or tolerance change.
The current code and binary fingerprints were verified before each launch.

| Run | Completed / requested | Termination | Completed-prefix advance + retrieve |
|---|---:|---|---:|
| Global diagnostic, 17:57:44–18:00:31 CST | 26 / 32 | Frame27 CUDA allocation OOM; SIGABRT / exit134 | 111.291 s |
| Local conservative, 18:07:02–18:10:11 CST | 25 / 250 | Frame26 CUDA allocation OOM; SIGABRT / exit134 | 125.070 s |

Directories: `global-32-20261023-175744` and
`local_conservative-250-20261023-180702` under the output directory above.
`termination.json` is authoritative: native abort left each raw `summary.json`
with a stale `running` status. There is no accepted capture for either failed frame.
No duplicate experiment or further rerun was started.

The global attempt did not reach the earlier frame32 baseline failure. The new
matrix diagnostics pass synthetic failure tests, but the original large-scene
baseline failure remains unlocalized. Both new failures report `cudaMalloc` in
`buffer.h:363`; these logs alone do not identify the exact allocating caller or
prove the mathematical cause of memory growth. The older allocation-shim result
belongs to its older run and must not be transferred to these attempts.

The global/local monitors sampled 35/39 times at approximately five-second
intervals. Only the respective run's compute PID was observed; normal desktop
work continued. Sampled process peaks were 25,914 / 13,170 MiB. These are not
instantaneous allocation peaks, and no sampled foreign process does not prove
absence between samples. The results show that concurrent compute occupancy is
not a sufficient explanation for every observed OOM.

### Local steps and completed-prefix quality

In the 25 completed local frames:

- 8,303 accepted direction steps; 118 baseline rebuilds; 93 orthogonality restarts;
  zero numerical retries or unsupported-contact rebuilds.
- 25,597 clamped vertex-iterations, minimum local fraction `0.0924896519`.
  All-iteration PT/EE candidate evaluations total 1,629,254,488 (not unique pairs).
  These completed frames required zero native per-pair fallback calls.
- The final feasible/CCD/CFL pipeline further reduced the global fraction 12
  times; the minimum was `0.000122773773`. This confirms that local proposals
  do not remove global step restrictions. The counter includes feasible and CFL
  restrictions, so it must not be presented as 12 CCD-detected collisions.
- Four locally scaled directions had nonnegative actual directional derivative;
  these are recorded without changing paper acceptance. There were 1,198 accepted
  energy increases; counts alone do not quantify their magnitudes or consequences.
- Graph captures/replays/fallbacks: 5 / 16,631 / 0.
- All25 completed frames passed both paper and independent residual criteria.
  Initial state and frames1–25 have finite position/velocity captures, no
  nonpositive tet Jacobian, no degenerate cloth triangle, and no negative floor
  vertex clearance in the offline analyzer. Minimum upper/lower tet J:
  `0.598678 / 0.305129`; minimum floor clearance: `2.14625 mm`.

For the common completed frames1–25, maximum vertex position differences between
the two new runs were `1.20e-13 m` upper, `9.50e-14 m` lower and `3.72e-15 m`
cloth. Their accepted prefix states agree to roundoff in this scene, despite
iteration differences. This is not evidence about failed frames or future contact.
The analyzer does not perform a complete mesh-mesh intersection audit.

Common-prefix advance + retrieve totals were global `105.414 s` and local
`125.070 s`. These are single capture runs, with ordinary desktop activity and
some CPU-only build/analysis work, not repeated isolated throughput measurements.
They establish no acceleration; the additional candidate/local work has measurable
cost here. Initialization, capture postprocessing and failed-frame time are excluded
from those totals. Neither variant completed the requested long-run acceptance.

## Delivery and remaining limits

Independent implementation review found no blocking synchronization, partition
mapping or actual-displacement integration defects. The main checkout was not
modified. The option remains experimental and opt-in: do not call it a fully
validated standard MAS-PNCG implementation, use it as a replacement for a passed
Newton baseline, claim a speedup, or disable final global CCD based on this result.

Reproduce the focused checks with the build-specific XMake configuration:

```sh
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake run core '[scene][config]'
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake run mas_pncg_operators
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134" xmake run mas_pncg
PYTHONPATH="$PWD/output/mas-pncg-local-ccd-20261023/runtime" .venv/bin/python -m pytest -q python/tests/test_scene_config_schema.py
PYTHONPATH="$PWD/output/mas-pncg-local-ccd-20261023/runtime" .venv/bin/python -m pytest -q -m cuda python/tests/test_mas_pncg.py
```

The bounded replay commands, timestamps, fingerprints and exact exits are saved
in each run's `preflight.json` and `termination.json`. Do not automatically repeat
the long runs: the outstanding work is an allocation-site/minimal-reproduction
investigation and eventual localization of the old baseline validation failure.
