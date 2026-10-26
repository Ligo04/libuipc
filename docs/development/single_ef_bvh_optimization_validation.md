# Single-EF and CUDA optimization integration validation

Validated locally on 2026-10-26 in `codex/single-ef-bvh`.

## Revisions and merge behavior

The existing seven modified Single-EF files were committed first, preserving
their original contents in `6df39cf95d8c1c27f237bc0b6757e4547d46873b`
(`feat(cuda): support mixed-dimensional Single-EF queries`). The checkpoint
passed its focused Single-EF tests before the merge.

Merge `abc93cb83e07cc0c80a210fed7b4d9f1319c6c9c` has that checkpoint as its
first parent and optimization revision
`abad8290abd66aebcc40e32f523778e9f400ce31` as its second parent. It includes
the original optimization patch and the subsequent GradientOnly/shared-EVD
changes.

The collision-filter conflict was resolved by retaining the Single-EF
pipeline and applying the optimization's refit policy to the trees each
method actually uses. Single-EF proxy queries now receive the topology-reuse
request as well. An incompatible cache triggers a full build. Mixed scenes
retain their PP/PE queries. Swept candidate tests explicitly exercise topology
and Morton-order reuse across changing CCD alphas and a subsequent DCD.

`info_stackless_bvh_v0.inl` already existed before the patch. Its merged
contents match the original patch's target blob exactly:
`f91247d6cc61f88a28fd9a2799f01a84955c1740`. The changes to that file were
provided by the original patch.

## Build and test environment

- NVIDIA RTX 5090, driver 590.48.01.
- CUDA compiler 13.4, V13.4.92, from `/home/ligo/.local/cuda-13.4.3`.
- Release XMake configuration at `build/xmake-cuda134-singleef/.xmake`.
- Every build used `-j4`; the worktree's ordinary `.xmake` configuration was
  preserved.
- Native CUDA backend SHA256:
  `50ccd6cd7fedd56df8c66c4bf0be4a4dd8296a9f835438ce247b5895e080c1b7`.

The following commands use the existing validated configuration:

```bash
export XMAKE_CONFIGDIR="$PWD/build/xmake-cuda134-singleef"
export LD_LIBRARY_PATH="/home/ligo/.local/cuda-13.4.3/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
xmake build -y -j4 backend_cuda core sim_case
xmake build -y -j4 pyuipc
xmake run backend_cuda
xmake run core
```

## Results

| Check | Coverage | Result |
| --- | --- | --- |
| CUDA default suite | 80 root test cases; 28,539,905 checks | Passed |
| Core default suite | 38 root test cases; 1,295 checks | Passed |
| Native CUDA simulation selection | 16 cases; 2,967 checks | Passed |
| Installed-wheel anisotropy Python tests | 10 tests, including CUDA cases | Passed |
| Focused memcheck | 1,228,945 checks | 0 errors |
| Focused racecheck | Same selection | 0 hazards, errors or warnings |
| Focused synccheck | Same selection; checker synchronization limit 64 | 0 errors |
| Nsight Systems execution trace | Single-EF worlds and remaining material launch contracts | Passed |

Native suite counts refer to root test cases, not Catch2's JUnit section or
assertion totals. All successful selections reported zero skips.

The simulation selection was cases 14, 20, 23, 37, 39, 45, 51, 52, 67, 68,
71, 72, 77, 83, 84 and 86. These cases explicitly use the CUDA engine and
exercise contacts, springs, rods, joints, stitches and plastic shell bending.
Case 86 retains the optimization branch's previously accepted 0.02 rad
plastic-residual lower bound.

The Python test file was `python/tests/test_baraff_witkin_anisotropy.py`, run
with `-m 'cuda or not cuda'` using a wheel installed in this worktree's
`.venv`. The wheel's CUDA library hash matched the native backend above.
Wheel: `pyuipc-0.9.0-cp312-cp312-linux_x86_64.whl`, SHA256
`a9276e17bcbc8ee74bec86ca8d60c68c0c44e417cb3c42351731904f262369c7`.
This was a local installation test, not a portable-wheel validation.

The focused sanitizer selection was:

```text
[single_ef_bvh],[remaining_material_launch],[material_gradient_only_launch],[bvh_scene_bounds]
```

It covers candidate sets and multiplicity, swept CCD and world tests,
GradientOnly/full material launch contracts, and the three BVH bounds
implementations.

## Sanitizer conditions

The successful runs used Compute Sanitizer 2026.3.0.0 from CUDA 13.4.92,
`--report-api-errors explicit`, and `--error-exitcode 99`. Synccheck also used
`--force-synchronization-limit 64` to avoid exhausting the checker's tracking
memory during the small-chunk stress test. Production execution was unchanged.

The older checker bundled with `/usr/local/cuda` (CUDA 12.8) reported memory
errors for the CUDA 13.4 binary. An independent CUB scan reproducer also
failed under that checker and passed with the matching checker. Those logs
are retained separately.

The matching checker's default `extended` API reporting emitted duplicate
template-kernel-name diagnostics for instantiations shared by the test
executable and backend library; the optimization branch also produced this
class of diagnostic. The passing results above use explicit API reporting,
which retains explicit CUDA API failures and memory/race/synchronization
checks. They do not claim a clean default extended-mode run. The unbounded
synccheck attempt exhausted checker memory and was superseded by the bounded
run; both logs are retained.

## GPU execution evidence and limits

Nsight Systems 2026.3.1 captured 16,709 kernel launches, including 454 refit
launches. In 72 sequences a Single-EF `make_leaf_proxies` launch was followed
on the same stream by `InfoStacklessBVH_refit_kernel` before the proxy query.
All eight remaining material GradientOnly kernel instantiations were captured
with both static and dynamic shared memory equal to zero. Their full-Hessian
variants used nonzero shared workspaces.

The trace establishes execution and shared-memory allocation behavior. This
integration did not measure combined Single-EF simulation FPS, add BVH CUDA
graph capture, run the full Python suite, validate CMake, or run remote CI.
Earlier material benchmark timings belong to the optimization branch and
must not be presented as combined simulation speedups.

Local raw evidence, including JUnit files, source patches, tool provenance,
the independent CUB reproducer and `.nsys-rep`/SQLite trace, is retained under
`build/verification/singleef-optimization-integration-20261026-181955`.
`tested-tree-proof.json` confirms that the production and test sources match
the merged tree; only documentation changed after those checks.
