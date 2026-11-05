# Single EF BVH on the X_square baseline

Base: `6e3386d12823c835c3206880b7758de952ccd339`.

Select the method before constructing the scene:

```python
config = Scene.default_config()
config["collision_detection"]["method"] = "single_ef_bvh"
```

`collision_detection/method` is an initialization-time, dimensionless string
selector. Its default is `info_stackless_bvh`; `single_ef_bvh` is always
available in CUDA builds, including builds with legacy collision disabled.

## Query and cache contract

One edge/face proxy tree supplies PT and EE candidates. Every surface vertex
has exactly one query owner, and every edge has exactly one leaf owner.
Isolated points become PT-only query proxies; standalone edges become EE-only
leaves. A proxy box contains its geometric primitive and every primitive it
owns. Mixed scenes keep the existing auxiliary PP/PE queries. Pure
codimensional or unsupported topology uses the existing simplex queries.

Each cache refresh grows the complete proxy AABBs by one representative
vertex's `d_hat`, rounding the new bounds outwards. It traverses only fat AABB
overlaps. It stores every chunk, converting query indices to global proxy IDs.
Static segment pruning and contact masks do not constrain this coarse cache.
While every current DCD or swept proxy remains inside its old fat box, detect
skips tree updates and traversal, then repeats the original primitive AABB,
body, contact/subscene and geometric PT/EE predicates, followed by
count/scan/fill. The inherited FP32 prefilter runs on actual PT/EE primitives.

The cache refreshes after containment escape, changed counts or pointers,
changed topology contents in the same allocation, or changed chunk size.
Topology comparisons run on the GPU before using ownership. Metadata is
reevaluated during each expansion, so changing it does not leave cached
position-free decisions. Refit retains the existing tree topology between
refreshes, with a full rebuild every eighth refresh. Output overflow grows
storage and repeats expansion across all cached chunks.

`BVHSimplexPipeline` shares active filtering, two-phase pending active sets,
TOI filtering and the existing detect-skin contract between both methods.
SingleEF prefix endpoints use `step_position`, matching accepted line-search
positions even when an FMA would retain a cancelled product residual. The
legacy prefix keeps its original default. The all-filter capability check
remains in effect. EF coarse-pair reuse and the public trajectory candidate
skin are separate layers.

The baseline's CUDA Graph support covers the linear solver. Collision detect
still has host readbacks and exceptional host capacity growth, as in the
baseline. Whole-detect/full-advance capture requires a separately prepared,
fixed-capacity path and is not provided by this method.

## Validation

`single_ef_bvh.cu` compares candidate multisets against fresh independent PT/EE
BVHs and exhaustive CPU DCD enumeration. Coverage includes mixed/codimensional
geometry, empty and invalid topology, more than 64 nearby faces, output/EF
overflow, unequal chunks, alpha changes, skin reuse and escape, masks, in-place
ownership changes, and floating-point cancellation. `single_ef_bvh_world.cpp`
compares positions, active contact counts and energies, and CCD TOI with the
baseline method in actual worlds.

Runtime checks can additionally enable `UIPC_CHECK_STORED_PAIRS=1` and
`UIPC_CHECK_ACTIVE_SET_REUSE=1`. The first bypasses EF cache reuse and verifies
the candidate multiset; the second verifies inherited frame-start active-set
reuse.

Validated on 2026-11-05 with an RTX 5090, driver 590.48.01, CUDA 13.4.92 and
XMake 3.1.2 in Release mode:

- Backend and CUDA device-link build passed. The initial 16-job build reached
  the 32 GiB cgroup limit; the successful retry used 4 jobs under the same
  `MemoryHigh=32G`, `MemoryMax=32G`, `MemorySwapMax=0` limits.
- The related GPU suite passed 15 cases and 842,588 assertions with both
  runtime validation modes enabled. This includes fresh candidate multiset,
  active-set, real-world energy/position/TOI, prefix, Morton, refit/order reuse
  and graph checks.
- With both extra validation modes unset, all 9 SingleEF cases and 519,596
  assertions passed, including real worlds using the persistent cache.
- Repository contract checks passed 13 cases. Clang-format 18, diff whitespace
  and Python schema-test syntax checks passed.
- CUDA 13.4 Compute Sanitizer passed 4 focused cases and 518,102 assertions
  with `--report-api-errors explicit`, reporting zero memory/API errors. Its
  default extended API mode also passed all 4 cases but emitted nine duplicate
  header-kernel registration diagnostics from the test executable and backend
  library. Those diagnostics remain recorded, so the default sanitizer mode
  is not claimed clean. The older system CUDA 12.8 sanitizer was incompatible
  with this CUDA 13.4 binary and was not used for the final memory conclusion.

The cancellation regression failed before the `step_position` change and
passed after rebuilding. CMake source ownership was checked; a CMake build and
Python runtime rebuild were not performed. This validation establishes no
scene-specific performance result. Logs and binary/source fingerprints are in
`output/single-ef-migration-20261105/`.
