---
name: cmake-workflow
description: Build and test the libuipc project using CMake. Use when building the project, running cmake configure/build commands, compiling with RelWithDebInfo, running Catch2 tests, or when the user asks how to build or test libuipc.
---

# CMake Workflow for libuipc

## Build

The build directory is `libuipc/build`. Run all cmake commands from there.

**Configure:**
```bash
cd libuipc/build
cmake -S ..
```

**Build:**
```bash
cmake --build . -j32 --config RelWithDebInfo
```

Use `-j32` for parallel compilation. The config is always `RelWithDebInfo`.

## Testing

Tests use **Catch2**. After a successful build, run the test executable directly — no separate test runner command is needed.

Find the compiled test binary in the build output (typically under `libuipc/build/bin/` or the relevant subdirectory) and execute it:

```bash
./path/to/test_executable
```

Catch2 test binaries accept standard flags such as:
- `--list-tests` — list all test cases
- `[tag]` — filter by tag
- `"test name"` — run a specific test by name

## Quick Reference

| Step | Command |
|------|---------|
| Configure | `cmake -S ..` (from `libuipc/build`) |
| Build | `cmake --build . -j32 --config RelWithDebInfo` |
| Run tests | Execute the test binary directly |
