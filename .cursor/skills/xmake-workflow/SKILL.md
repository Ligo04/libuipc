---
name: xmake-workflow
description: Build and test the libuipc project using XMake. Use when building with xmake, running xmake f / xmake build / xmake run commands, selecting release/releasedbg/debug modes, or running Catch2 tests produced by xmake.
---

# XMake Workflow for libuipc

Run all xmake commands from the repository root (`libuipc/`).

## Configure

```bash
xmake f -c
```

Add `-m releasedbg` (RelWithDebInfo equivalent) or `-m debug` to change mode. Default is `release`.

## Build

```bash
xmake build -j8
```

Keep `-j` low (e.g. `-j8`). NVCC is memory-hungry and can OOM with a high job count.

Build one target: `xmake build -j8 core`.

## Target Names vs. Binary Names

Test targets use the shared `uipc_test` rule in `apps/tests/xmake.lua`, which rewrites the binary basename to `uipc_test_<target>`. Use the **target name** for `xmake build` / `xmake run`; use the **binary name** only when executing the file directly.

| Target name | Binary name |
|-------------|-------------|
| `core`, `geometry`, `common`, `sim_case`, `regression`, `sanity_check`, `backend_cuda` | `uipc_test_<target>` |
| `hello_affine_body`, `hello_simplicial_complex`, `wrecking_ball`, `pyuipc` | same as target |

List every runnable target with `xmake run --help`.

## Testing

Tests use **Catch2**. Run via xmake (recommended):

```bash
xmake run sim_case
xmake run backend_cuda
```

Or execute the binary directly: `./build/linux/x86_64/releasedbg/uipc_test_core`.

Catch2 flags passthrough:
- `--list-tests` — list all test cases
- `[tag]` — filter by tag
- `"test name"` — run a specific test

## Quick Reference

| Step             | Command                                                  |
|------------------|----------------------------------------------------------|
| Configure        | `xmake f -c` (add `-m releasedbg` for RelWithDebInfo)    |
| Build            | `xmake build -j8`                                        |
| Build one target | `xmake build -j8 <target>`                               |
| List run targets | `xmake run --help`                                       |
| Run test         | `xmake run <target>`                                     |
| Clean            | `xmake clean` (add `-a` for all configs)                 |
