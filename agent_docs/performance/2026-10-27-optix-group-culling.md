# OptiX EF group-root culling — implementation validation

## Status and scope

The optional `single_ef_optix` backend now rejects homogeneous body/contact
subtrees before scanning their EF leaves. Each OptiX custom primitive is an
application-defined group root, with up to 16 original leaves by default.
This does not expose or modify OptiX's private BVH internal nodes.

Performance is pending: the owner requested waiting for the concurrent
Isaac Sim/Kit GPU workload to stop. No fixed-state benchmark, throughput
comparison or performance profile from this implementation has been run.
The prior flat-OptiX report remains historical evidence, not a result for
this grouped implementation.

## Implementation

- Stable Morton sorting followed by stable full-width `(bid, cid)` sorting
  creates spatial groups within equal-tag runs. Groups are bounded; a run
  boundary is never crossed merely to fill a group.
- Rebuilds, leaf-count changes and group-size changes recreate membership.
  Refits preserve membership and recompute bounds and tags. Mixed body and
  contact IDs independently become `-1`, retaining exact child predicates.
- The intersection program rejects the group AABB or homogeneous disabled
  metadata. Single-any-hit collection scans accepted groups and emits original
  leaf IDs. Chunk-local query IDs and original-leaf overflow bounds remain.
- `UIPC_OPTIX_EF_GROUP_SIZE` accepts integers 1–64; 1 is the flat reference.
  The ordinary CUDA backend ignores it. The scene's default method is unchanged.
- Audit counters are opt-in. Group/rejection counts are callback invocations;
  skipped leaves are summed group sizes for metadata rejections, not unique
  rejected candidate pairs. Counts include overflow replays. All timed paths
  disable these counters.
- Regrouping uses one integer device-to-host count transfer. Its cost belongs
  in complete detect timings. Larger groups can increase allowed-child AABB
  checks, so fewer primitives alone do not establish a performance gain.

See [the backend guide](../../docs/development/single_ef_optix.md).

## Build and test environment

Worktree: `/home/ligo/.codex/worktrees/single-ef-optix/libuipc`, branch
`codex/single-ef-optix`, preceding flat implementation/documentation revision
`90dfdc6a79fe67fe84060955e50ec8d82cac8135`. The grouped code was validated as a
working-tree change on that revision; the accompanying JSON records source
hashes. The next performance stage must use a clean committed revision and
record the actual loaded binaries.

Native CUDA and OptiX PTX both use CUDA 13.4.92 from
`/home/ligo/.local/cuda-13.4.3`, OptiX SDK 9.1.0, Release, native `sm_120`,
RTX 5090, driver 590.48.01. The separate PTX program targets virtual
`compute_75`. XMake uses `build/xmake-optix`, with ccache disabled.

| Check | Result |
|---|---|
| XMake backend/core/wrecking_ball/sim_case build | Passed |
| Focused Single-EF and compiled-filter tests | 3,706 assertions |
| Full core | 1,298 assertions |
| Full CUDA backend | 29,473,760 assertions |
| Full aggregate simulation | 14,213 assertions |
| Memcheck, including OptiX resources | 3,682 assertions; zero errors |
| Synccheck | 3,682 assertions; zero errors |
| OptiX initcheck | 233 assertions; zero errors |
| Python audit/repository contract tests | 21 passed |
| CUDA13.4 native ON, SDK-free OFF and PTX compile probes | Passed |
| clang-format-18 / diff whitespace | Passed |

New cases verify complete-group rejection without child tests, IDs beyond
8-bit visibility-mask use, live mask changes, shuffled original IDs, chunking,
overflow replay, group-size switches, and homogeneous/mixed/invalid-tag refits.
The grouped and flat paths also share closed-boundary coverage.

Full-project CMake, a full CUDA-enabled/OptiX-disabled build, other GPUs and
platforms, and Python wheel packaging remain unverified. Compile probes do
not establish those broader claims.

## Same-state 1,000-frame audit

The grouped backend completed 1,000 wrecking-ball frames. Each audited state
was checked against the existing informative BVH: 13,326 candidate
records, 13,457 active-contact records and 6,663 TOI records.
All candidate/contact comparisons agree; all 463,200,693 pairwise TOIs
agree exactly, with no nonfinite values or minimum-TOI mismatches. All CCD
records used OptiX; there was no fallback or execution error.

| Diagnostic count | Total |
|---|---:|
| GAS builds / refits / launches | 1,001 / 12,325 / 13,338 |
| Group primitives per detect | 1,308 |
| Group intersection callbacks | 7,991,551,764 |
| Body rejections | 1,472,757,678 |
| Contact-table rejections | 0 |
| Metadata-skipped child counts | 22,965,024,776 |
| Actual leaf tests | 7,604,083,060 |
| Emitted EF pairs, including replay | 1,776,792,818 |

These are callback/replay totals, not unique pairs or a flat-versus-grouped
speedup. This scene exercised body rejection; disabled contact-table rejection
is covered by the focused GPU test. Kit was active during this correctness
run, and audit atomics/readbacks were enabled. Its elapsed times are explicitly
excluded from performance evidence.

Full audit: `output/optix-group-culling/benchmark/audit-20261027T122002.325658Z`.
Numeric evidence: [validation JSON](2026-10-27-optix-group-culling-results.json).

## Sanitizer findings and fixes

The initial initcheck run reported 49 errors. Forty-seven launch-parameter
copy reports came from an unwritten 4-byte padding hole. CUDA object stores
covered `[0,84)` and `[88,112)`. An explicit zeroed reserved field fills that
hole; static assertions preserve all existing offsets and the 112-byte layout.

The remaining two reports involved the final scan element for five leaves.
A standalone CUDA13.4 CUB probe reproduced the report with seven allocated
integers (28 bytes), while the exact prefix values were correct. Nonzero
prefill proved that CUB overwrote the tail. Reserving a complete 16-byte tail
store region eliminated the report; the original 64-byte allocation did too.
The same capacity-dependent behavior reproduced with and without RDC.
This isolates the report to initialization tracking of the masked store at an
allocation boundary; it is not evidence of an unwritten logical scan element.

Only the group-offset buffer now reserves at least `round_up(count,4)` integers.
The logical scan length is unchanged. There is no output zero-fill, global
WarpSpeed disable, or sanitizer suppression. Rounded-capacity probes for
1, 2 and 5 elements and the complete final OptiX initcheck all pass.

Evidence: `output/optix-group-culling/verified-*`, `scan-*`, and
`build/optix-group-probe/final-compile.json`. The standalone reproducer is
`build/optix-group-probe/scan_initcheck.cu`.

## Remaining measurement

After Kit releases the GPU, run the hidden fixed-state benchmark for native
CUDA, flat OptiX and group sizes 8/16/32. It emits separate untimed culling
snapshots, then five interleaved samples of 20 detect calls per scope with
counters disabled. Compare DCD/CCD build and refit separately.

Then run five interleaved 1,000-frame wrecking-ball processes per variant
(native, flat, selected group size), excluding the first 30 frames from
throughput. Record solver work and both within-variant and cross-variant
trajectory variation. Keep synchronized timers and Nsight profiles separate.
The prepared local runner is `output/optix-group-culling/run_variants.py`;
it reuses the existing benchmark's validation and provenance helpers and
refuses to start while Kit or another compute application has a GPU context.
