# Optional Single-EF OptiX backend

`single_ef_optix` is an experimental alternative to `single_ef_bvh`. It is
available only in builds with OptiX enabled. The default collision method is
still `info_stackless_bvh`; the ordinary CUDA Single-EF method has no OptiX
dependency.

Set `collision_detection/method` to `single_ef_optix` before `World::init`.
The selector has no units. The scene schema lists it only when the matching
core and CUDA backend are built with OptiX support.

## Build

Provide an installed OptiX SDK. The XMake `optix` system package discovers
headers and the SDK version through the case-sensitive `OptiX_ROOT` variable.
This experiment uses SDK 9.1.0 and CUDA 13.4 for both native code and PTX.

```bash
export OptiX_ROOT=/path/to/NVIDIA-OptiX-SDK-9.1.0
export XMAKE_CONFIGDIR="$PWD/build/xmake-optix"
xmake f -y -m release -o build/xmake-optix \
  --cuda=/path/to/cuda-13.4 --ccache=n \
  --optix=y --optix_nvcc=/path/to/cuda-13.4/bin/nvcc \
  --tests=y --examples=y --pybind=n --dev=y \
  --backend_cuda=y --cuda_legacy_collision=y
xmake build -y -j4 backend_cuda core wrecking_ball
```

Pass the full configuration when changing a compiler: `xmake f` can reset
unspecified options to defaults. Verify both `cuda` and `optix_nvcc` in the
generated configuration before measuring.

CMake exposes equivalent `UIPC_WITH_OPTIX`, `UIPC_OPTIX_ROOT`, and
`UIPC_OPTIX_NVCC` settings. The PTX compiler defaults to the native CUDA
compiler. The generated module is embedded in the CUDA backend library;
the application does not load a PTX file from the SDK installation at runtime.

## Query semantics

The backend preserves the Single-EF ownership maps and AABB proxies, including
isolated-point queries and standalone-edge leaves. OptiX enumerates EF pairs;
the existing CUDA kernels apply the PT/EE predicates, count and scan output,
and replay an overflowing query after increasing reusable capacity. The
existing contact classification and CCD TOI kernels remain the consumers.

OptiX ray traversal uses a conservative box-overlap construction. Leaves are
expanded along two axes by maximum query half-extents, and each query traces
its interval along the remaining axis. Normalized, outward-rounded bounds
cover floating-point conversion. Intersection programs recheck original
proxy boxes and metadata. Any-hit collection requests one invocation per
primitive and continues traversal after recording a pair.

This construction accepts complete swept query boxes for CCD. The original
CUDA backend's DCD segment pruning is not part of OptiX enumeration; extra EF
pairs are filtered by the existing exact predicates. Scenes without triangles
or with unsupported ownership topology retain the existing informative-BVH
fallback. Intermediate pair counts, acceleration-structure memory, and
runtime can differ even when final candidate multisets agree.

## Group-root metadata culling

The OptiX BVH is opaque: this adapter cannot install the native CUDA
`NodePredicate` on its private internal nodes. Instead, each custom primitive
represents a bounded group of original EF leaves. Its intersection program
checks the group's original AABB, body ID and contact ID before any-hit scans
the children. A disabled same-body or contact-table pair rejects the complete
group. This is application-defined subtree culling, not a callback inside
OptiX's private BVH.

Leaves are spatially sorted by Morton code, then stably grouped by the full
`(body ID, contact ID)` key. Each equal-key run is split into bounded groups.
The default maximum is 16 leaves per group. Set `UIPC_OPTIX_EF_GROUP_SIZE` to
an integer in `[1, 64]` before initializing the OptiX backend; `1` selects
the flat reference implementation. The variable is an experimental process
setting with no units, not a scene configuration key. The native CUDA method
ignores it.

Rebuilds, leaf-count changes and group-size changes recreate membership.
Refits keep membership but recompute union boxes and aggregate tags: a mixed
body or contact tag becomes `-1` independently, so only a uniform tag can
reject a group. Exact per-leaf predicates remain in the child scan. Outputs
retain original leaf IDs and chunk-local query IDs; overflow sizing still
uses the original leaf count. Changing a live body/contact mask does not
require rebuilding membership. Regrouping includes one device-to-host group
count readback; refitting does not.

OptiX's single-any-hit flag limits collection to one invocation per group
primitive. Larger groups reduce primitive callbacks but loosen bounds and
can increase child AABB checks on allowed groups. Grouping is therefore not
by itself evidence of a speedup.

This representation follows NVIDIA's guidance on
[custom primitives containing groups](https://forums.developer.nvidia.com/t/aabb-for-multiple-custom-primitives/179242).

## Evidence

`UIPC_SINGLE_EF_AUDIT_FILE` runs the existing baseline on the same simulation
state. Candidate records additionally identify `requested_backend`,
`executed_backend`, and actual per-detect `optix_builds`, `optix_refits`, and
`optix_launches`, plus cumulative counters. A fallback record cannot establish
OptiX performance. Audit candidate records also contain group size/count and
optional culling counters. Counters are enabled by the audit-file variable
and disabled in ordinary runs. `group_tests`, `body_rejections` and
`contact_rejections` count intersection-program invocations. `skipped_leaves`
sums the child counts of metadata-rejected invocations; it excludes geometry
rejections and does not count unique rejected candidate pairs. `leaf_tests`
and `emitted_pairs` include all launches and overflow replays within a detect.
Flat mode has zero group-culling counters. Counter readbacks and atomics belong
only in diagnostic runs.

Use separate uninstrumented throughput runs and instrumented diagnostic runs.
Compare `single_ef_bvh` with `single_ef_optix` using the same executable and
configuration. The hidden fixed-state benchmark measures complete
`SingleEFBVHQuery::detect` calls on a deterministic two-layer grid, excluding
primitive-bound preparation and initialization. It checks candidate multisets
before and after every sample, with three warmup calls, one untimed diagnostic
call, and five interleaved samples of twenty calls for DCD/CCD build/refit
scopes. Variants are native CUDA, flat OptiX, and OptiX group sizes 8, 16 and
32. Separate `SINGLE_EF_CULLING` records describe identical-state work; all
timed samples assert that diagnostic counters are disabled.

```bash
xmake run backend_cuda '[single_ef_optix_benchmark]'
python3 scripts/benchmark_single_ef_wrecking_ball.py \
  --phase throughput --frames 1000 --warmup 30 --repeats 5 \
  --methods single_ef_bvh single_ef_optix \
  --build-dir build/xmake-optix/linux/x86_64/release \
  --output-dir output/optix-single-ef/benchmark
```

Use `--phase audit` for same-state candidates, active contacts, and pairwise
TOIs; use `--phase diagnostic` for synchronized stage timers. Both accept the
same frame and method options. Correctness and timing results are recorded
with their tested source revision and runtime hashes. Independent trajectories
can change floating-point accumulation order and solver work; their wall-time
difference alone does not isolate traversal throughput.

The original flat-mode 2026-10-27 RTX 5090 experiment used CUDA 13.4 and five alternating
1,000-frame runs per method, with 30 warmup frames. Native and OptiX medians
were 38.499 and 38.678 ms/frame, with overlapping ranges: no stable end-to-end
gain was demonstrated. Both same-state 1,000-frame collision audits passed.
Scene diagnostics found faster DCD but slower CCD candidate detection. The
option remains experimental; see the
[complete experiment record](https://github.com/Ligo04/libuipc/blob/codex/single-ef-optix/agent_docs/performance/2026-10-27-single-ef-optix.md).

The group-root follow-up passes the full suites, all three sanitizer modes
and a 1,000-frame same-state audit. Its performance comparison awaits an
uncontended GPU; see the [grouped validation record](https://github.com/Ligo04/libuipc/blob/codex/single-ef-optix/agent_docs/performance/2026-10-27-optix-group-culling.md).
