# Unify the nine split gradient/Hessian CUDA routes

Validated locally on 2026-10-27 in `codex/single-ef-bvh`, based on
`4c719b0b30330562250b8c4ae91056a78dc35d22`.

## Result

All nine remaining routes with separate G and H kernels now use one
`template<bool GradientOnly>` global kernel and `if constexpr(!GradientOnly)`.
A gradient-only request launches the true instance. A full derivative request
launches the false instance once, replacing the former G launch followed by H.

The CUDA source inventory is now **33 specialized derivative kernel families,
zero runtime gradient-only global kernels, and zero remaining split k1/k2
derivative kernel pairs**.

- `RevoluteJoint` in `src/backends/cuda/affine_body/constitutions/affine_body_revolute_joint.cu`.
- `DrivingRevoluteJoint` in `src/backends/cuda/affine_body/constitutions/affine_body_revolute_joint.cu`.
- `RevoluteJointLimit` in `src/backends/cuda/affine_body/constitutions/affine_body_revolute_joint.cu`.
- `PrismaticJoint` in `src/backends/cuda/affine_body/constitutions/affine_body_prismatic_joint.cu`.
- `DrivingPrismaticJoint` in `src/backends/cuda/affine_body/constitutions/affine_body_prismatic_joint.cu`.
- `PrismaticJointLimit` in `src/backends/cuda/affine_body/constitutions/affine_body_prismatic_joint.cu`.
- `SoftTransformConstraint` in `src/backends/cuda/affine_body/constraints/soft_transform_constraint.cu`.
- `ExternalArticulationConstraint` in `src/backends/cuda/affine_body/constraints/external_articulation_constraint.cu`.
- `SoftPositionConstraint` in `src/backends/cuda/finite_element/constraints/soft_position_constraint.cu`.

The shared internal production/test interfaces are declared in
`src/backends/cuda/utils/constraint_gradient_hessian_launch.h`. Public scene
configuration and material parameters are unchanged.

## Semantics preserved

- G computation remains directly in the kernel. All H declarations, calculation,
  SPD projection, shared workspaces and stores are inside compile-time full-mode
  branches.
- An unconstrained revolute or prismatic driver still explicitly writes zero G
  and zero H. Its early return occurs after both requested outputs are written.
- Passive driving, angle unwrapping, reference previous states, limit penalties,
  fixed-body handling, interpolation and existing shared EVD formulas are retained.
- Production body-id views retain the original count owners: driver masks,
  revolute limit rows and prismatic reference rows. Clipping the view to those
  extents preserves the previous host count expressions.
- External articulation keeps independent joint and joint-pair counts. The full
  grid covers their maximum; G and H separately check their own bounds.
- External articulation cross-coupling blocks can contain global lower-triangle
  indices, as in the original implementation. Tests check index bounds and output
  guards without imposing a new global-upper-triangle layout.
- Empty input is a no-op. Gradient-only mode accepts empty H views and leaves
  guarded H storage untouched.

An initial captured-lambda implementation changed prismatic G code generation:
the static SASS instruction count grew from 576 to 592 and its batch timing
regressed. Direct G statements restored the original 576 count and removed the
observed regression. This is static code and controlled batch evidence; hardware
stall/occupancy causes were not measured with Nsight Compute.

## Validation

Release XMake, CUDA compiler V13.4.92, RTX 5090 (`sm_120`), driver 590.48.01.
Every build used **`-j4`** and the dedicated
`build/xmake-cuda134-singleef` configuration.

- Complete CUDA suite: **82 root cases**, 28,946,977 assertions.
- Complete Core suite: **38 cases**, 1,295 assertions.
- **30 selected simulations**, 6,784 assertions, including single/multiple external
  articulations, revolute/prismatic driving, local joints and limit cases.
- New launch contract: **78 fixture cases** across nine families at
  `0, 1, 15, 16, 17, 31, 33, 257` elements, plus external H counts `0` and
  `2*n+1`. **224,884 assertions** check finite derivatives, sparse indices, guards,
  empty H views, gradient-only/full agreement, CPU central finite differences of
  energy primitives, and direct soft-position/soft-transform H references.
- Original split versus final full derivatives: all nine families at 33 elements,
  plus external H counts 0 and 67. Maximum absolute difference:
  **5.55e-17**. This finite sample does not establish bitwise equality generally.
- Matching CUDA 13.4 Compute Sanitizer: **memcheck 0 errors, racecheck 0 hazards,
  synccheck 0 errors**, on the final nine-family contract selection, using
  `--report-api-errors explicit`.
- Nsight Systems 2026.5.1: all **18** final true/false instances ran **48 times
  each**. The reference full call launches separate G/H kernels; the final full
  call launches one combined kernel. No NVTX or hardware GPU metrics were collected.

The installed-wheel smoke and selected Python/CUDA tests are recorded beside the
wheel in its build manifest. The full Python suite, CMake build and CI were not run.

## Compiled resources

The true instances retain the old standalone G kernels' registers, stack,
reported local memory and shared memory for **all nine families**. G-only shared
memory is zero. The full instances retain the existing EVD shared workspaces.

| Family | G registers/thread, unchanged | G stack bytes/thread, unchanged | Full shared bytes/block |
| --- | ---: | ---: | ---: |
| RevoluteJoint | 88 | 128 | 10240 |
| DrivingRevoluteJoint | 98 | 128 | 19456 |
| RevoluteJointLimit | 142 | 128 | 19456 |
| PrismaticJoint | 128 | 128 | 21760 |
| DrivingPrismaticJoint | 80 | 128 | 21760 |
| PrismaticJointLimit | 98 | 128 | 21760 |
| SoftTransformConstraint | 255 | 2160 | 0 |
| ExternalArticulationConstraint | 118 | 176 | 19456 |
| SoftPositionConstraint | 40 | 48 | 0 |

No additional gradient-only register reduction is claimed for these already
isolated routes. The structural benefit is a consistent compile-time interface
and one launch for a full G/H request.

## Logical derivative-call measurements

Five alternating processes per side at 4096 and 65536 elements, 8 warmup calls and
40 timed calls per family/mode. CUDA events include ordinary submission gaps.
Initialization, allocation, transfers and correctness checks are excluded.
External articulation uses `n` gradient rows and `2*n+1` Hessian rows.

Process lists were recorded before/after each run and GPU clocks/utilization
sampled. Another `uipc_test_backend_cuda` process was observed during both size
sets, including substantial GPU memory use in the larger set. These measurements
therefore reflect shared-device conditions. Clocks were not locked and device
ownership was not exclusive. Timings are descriptive; they do not establish
stable or whole-frame speedups.

Full G/H at 4096 elements:

| Family | Separate G + H (us/call) | Unified G/H (us/call) | Median ratio | Five-sample ranges overlap |
| --- | ---: | ---: | ---: | --- |
| revolute | 71.798 | 43.042 | 1.668× | no |
| driving_revolute | 474.431 | 415.187 | 1.143× | no |
| revolute_limit | 510.705 | 440.821 | 1.159× | no |
| prismatic | 328.780 | 281.306 | 1.169× | no |
| driving_prismatic | 232.663 | 212.507 | 1.095× | yes |
| prismatic_limit | 175.611 | 151.239 | 1.161× | no |
| soft_transform | 114.593 | 112.606 | 1.018× | yes |
| external_articulation | 1293.594 | 1198.784 | 1.079× | no |
| soft_position | 12.656 | 10.235 | 1.237× | no |

The JSON contains both sizes, both modes and all five timing samples. G-only
timings should remain near the original standalone G path; full-mode gains
depend on launch overhead versus derivative arithmetic.

[Machine-readable results](../../apps/tests/backends/cuda/constraint_gradient_hessian_results.json)
include native hashes, resource records, source inventory, snapshot differences
and all timing samples.

## Reproduction

```sh
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134-singleef" xmake build -y -j4 backend_cuda core sim_case pyuipc
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134-singleef" xmake run -w "$PWD" backend_cuda '[constraint_gradient_launch]'
UIPC_CONSTRAINT_BENCH_ELEMENTS=65536 XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134-singleef" xmake run -w "$PWD" backend_cuda '[constraint_gradient_benchmark]'
```

The hidden benchmark emits `CONSTRAINT_BENCH` JSON with time per logical derivative
call. The tagged wheel is `0.9.0+cu134.singleef.opt.gradientonly.unified` for
CPython 3.12/Linux/native RTX 5090. It retains the external `libcudart.so.13`
dependency, so self-contained wheel dependency audit does not pass.
