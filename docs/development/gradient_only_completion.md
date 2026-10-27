# Remaining gradient-only CUDA kernel specialization

Validated locally on 2026-10-27 in `codex/single-ef-bvh`, based on
`92039a26baca667fee792c65cb2f0065b0c85952`.

## Change and derivative contract

The remaining ten derivative kernel families now use
`template<bool GradientOnly>` and `if constexpr`. The production host dispatcher
chooses the true or false instance. Gradient-only instances exclude Hessian
declarations, evaluation, projection and stores. ABD ARAP's Hessian zeroing also
moved inside the full branch. Numerical expressions, constitutive models, SPD
algorithms and shared EVD implementations are unchanged.

Four families are constitutions (SNH3D, Baraff-Witkin shell, ABD ARAP and spherical
joint), two are half-plane contact models, and four are FEM/ABD BDF1/BDF2 inertia.
The source audit now finds **24 specialized derivative kernel families and zero
explicit global derivative kernels taking a runtime gradient-only flag**.
This measured snapshot preserved nine families with separate gradient/Hessian launches.
They have since been unified in the [follow-up change](gradient_only_split_unification.md).

Changed production sources:

- `src/backends/cuda/finite_element/constitutions/stable_neo_hookean_3d.cu`
- `src/backends/cuda/finite_element/constitutions/strain_limiting_baraff_witkin_shell_2d.cu`
- `src/backends/cuda/affine_body/constitutions/arap.cu`
- `src/backends/cuda/affine_body/constitutions/affine_body_spherical_joint.cu`
- `src/backends/cuda/contact_system/contact_models/ipc_vertex_half_plane_normal_contact.cu`
- `src/backends/cuda/contact_system/contact_models/ipc_vertex_half_plane_frictional_contact.cu`
- `src/backends/cuda/finite_element/bdf/finite_element_bdf1_kinetic.cu`
- `src/backends/cuda/finite_element/bdf/finite_element_bdf2_kinetic.cu`
- `src/backends/cuda/affine_body/bdf/affine_body_bdf1_kinetic.cu`
- `src/backends/cuda/affine_body/bdf/affine_body_bdf2_kinetic.cu`

`src/backends/cuda/utils/specialized_gradient_hessian_launch.h` exposes the typed
internal launch interfaces used by production and the focused tests. Empty input
is a no-op; gradient-only mode accepts an empty Hessian view and leaves guarded
Hessian storage untouched.

Both instances use the full kernel's occupancy-based block hint. The first
implementation selected each instance's automatic block independently: for 4096
SNH elements the gradient block grew from 256 to 640 threads, reducing the grid
from 16 to 7 blocks. Its observed batch time rose from about 10.3 to 20.5 us.
Retaining the full instance's launch geometry removed that small-input regression.
The final Nsight trace confirms that all 20 instances match the runtime reference
grid/block shape at 4096 elements. No microarchitectural occupancy or stall
measurements are claimed.

## Validation

Environment: RTX 5090 (`sm_120`), driver 590.48.01, CUDA compiler V13.4.92,
Release XMake, **all builds `-j4`**. The existing ordinary `.xmake` configuration
was preserved; the build used `build/xmake-cuda134-singleef`.

- Complete CUDA suite: **81 root cases**, 28,722,093 assertions, no failures.
- Complete Core suite: **38 cases**, 1,295 assertions, no failures. Catch's console
  summary confirms the case count; JUnit also includes section rows and can omit
  assertion-free cases.
- Selected simulations: **21 root cases**, 3,723 assertions, no failures.
- New derivative contract test: all ten families at **0, 1, 31, 33, 257** elements,
  182,188 assertions. It checks output indices, offset views, guard values,
  empty Hessian views, gradient-only/full agreement, and independent CPU gradient
  references for the eight non-contact families. ABD ARAP uses Eigen JacobiSVD.
- Original runtime-bool and final specialized full derivatives at 33 elements:
  **all ten saved gradient/Hessian arrays match exactly** (maximum absolute
  difference 0). This is a finite input sample, not a general floating-point proof.
- Matching CUDA 13.4 Compute Sanitizer: memcheck **0 errors**, racecheck **0 hazards**,
  synccheck **0 errors**. The first specialized revision passed the combined
  derivative/Single-EF selection; after the host launch-geometry correction the
  final ten-family contract passed all three tools again. Kernel bodies did not
  change during that correction.
- Sanitizers use `--report-api-errors explicit`. The combined Single-EF synccheck
  run used `--force-synchronization-limit 64`. Default extended API reporting is
  not an acceptance claim.
- Nsight Systems 2026.5.1 captured all **20** derivative instances with **48 launches
  each** (8 warmup + 40 timed). All 980 kernel events, including setup kernels,
  correlate with CUDA runtime calls. NVTX and hardware GPU metrics were not
  collected; no Nsight Compute counters were collected.

The runtime reference retains the original runtime-bool kernel bodies and adds
the same typed dispatch interfaces for direct, equivalent launch tests.
Machine-readable results, native hashes, all five timing samples per side and
launch metadata are in
[specialized_gradient_hessian_results.json](../../apps/tests/backends/cuda/specialized_gradient_hessian_results.json).

## Gradient-only compiled resources

`cuobjdump --dump-resource-usage` on the CUDA 13.4 backend reports:

| Family | Registers/thread, runtime → specialized | Stack bytes/thread, runtime → specialized |
| --- | ---: | ---: |
| stable_neo_hookean_3d | 255 → 94 | 1288 → 224 |
| baraff_witkin_shell | 255 → 84 | 2240 → 128 |
| abd_arap | 236 → 148 | 296 → 248 |
| affine_body_spherical_joint | 255 → 56 | 440 → 128 |
| half_plane_normal | 80 → 70 | 112 → 112 |
| half_plane_friction | 86 → 78 | 168 → 112 |
| fem_bdf1 | 37 → 34 | 96 → 48 |
| fem_bdf2 | 37 → 34 | 96 → 48 |
| abd_bdf1 | 116 → 116 | 1248 → 192 |
| abd_bdf2 | 116 → 116 | 1248 → 192 |

Shared memory and reported static local memory remain zero for these instances.
Stack size is reported separately; these values do not measure dynamic spills.

## Shared-device launch measurements

Five alternating runtime/specialized processes per side, 4096 and 65536 elements,
8 warmup launches and 40 timed launches per family/mode. CUDA events span ordinary
launches, so the measurement includes host submission gaps. Allocation, transfer,
initialization and correctness checks are outside the timed region.

`houdini-bin` occupied GPU memory throughout all before/after observations.
Clocks and utilization were sampled, but the device was neither reserved nor
clock-locked. These are descriptive shared-device batch timings; they do not
establish exclusive-device gains or frame-rate improvements.

At 65536 elements, gradient-only median times are:

| Family | Runtime reference (us) | Specialized (us) | Median ratio | Five-sample ranges overlap |
| --- | ---: | ---: | ---: | --- |
| stable_neo_hookean | 22.026 | 18.452 | 1.194× | no |
| baraff_witkin | 28.553 | 22.650 | 1.261× | no |
| abd_arap | 98.702 | 90.160 | 1.095× | yes |
| spherical_joint | 24.546 | 24.597 | 0.998× | yes |
| half_plane_normal | 86.627 | 79.969 | 1.083× | yes |
| half_plane_friction | 92.404 | 91.542 | 1.009× | yes |
| fem_bdf1 | 4.220 | 4.158 | 1.015× | yes |
| fem_bdf2 | 4.221 | 4.190 | 1.007× | yes |
| abd_bdf1 | 62.775 | 53.535 | 1.173× | yes |
| abd_bdf2 | 64.274 | 63.032 | 1.020× | yes |

At 4096 elements, SNH is 10.262 → 10.274 us and Baraff-Witkin is
14.352 → 12.494 us; most other ranges overlap. The results JSON also records
full-Hessian timings. Register and stack reductions are independently verified
compiled-resource results and do not imply equal timing improvements.

## Reproduction

Keep the CUDA 13.4 XMake configuration and run:

```sh
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134-singleef" xmake build -y -j4 backend_cuda core sim_case pyuipc
XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134-singleef" xmake run -w "$PWD" backend_cuda '[specialized_gradient_launch]'
UIPC_GRADIENT_BENCH_ELEMENTS=65536 XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134-singleef" xmake run -w "$PWD" backend_cuda '[specialized_gradient_benchmark]'
```

The hidden benchmark emits `GRADIENT_BENCH` JSON. Use matching runtime libraries
when launching test executables directly. The selected simulation names are:

`14_fem_3d_ground_contact`, `20_spring_fixed_point`, `23_kirchhoff_rod_bending`,
`37_abd_revolute_joint`, `39_soft_vertex_stitch`, `45_abd_prismatic_joint`,
`51_abd_prismatic_joint_limit`, `52_abd_revolute_joint_limit`,
`67_soft_vertex_triangle_stitch`, `68_soft_vertex_edge_stitch`,
`71_abd_driving_prismatic_joint`, `72_abd_driving_revolute_joint`,
`77_abd_fixed_joint`, `83_discrete_shell_bending_strain_plastic_residual`,
`84_strain_plastic_discrete_shell_bending_recover`,
`86_discrete_shell_bending_stress_plastic_residual`, `9_abd_multi_constitution`,
`78_abd_spherical_joint`, `76_abd_bdf2_rebound_higher_than_bdf1`,
`19_shell_fixed_point`, `21_particle_ground`.

The full Python suite, CMake build, CI and whole-frame performance are outside
this validation. The separately packaged `singleef-opt-gradientonly` wheel is
a CPython 3.12 Linux/native-architecture build and retains the existing external
`libcudart.so.13` dependency; it is not a self-contained CUDA wheel.
