# pybind11 to nanobind migration plan

Status: source and primary-documentation research only. No dependency was
installed and no configure, build, import, test, wheel, or runtime command was
run for this report.

Baseline inspected: branch `codex/migrate-pybind-to-nanobind`, commit
`09c46cceba8ace6fb924ba7d9e8012080c4670b1`, 2026-08-27.

## Recommendation

Migrate the existing `pyuipc` extension to **nanobind 3.0.0**, while preserving
the Python distribution name (`pyuipc`), extension name
(`uipc._native.pyuipc`), public module layout, and Python 3.10-3.14 wheel matrix.
Nanobind 3.0.0 is the current release line inspected for this plan; it was
released on 2026-08-22, requires Python 3.10 or newer, and has internal ABI 22
([3.0.0 changelog](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)).
The repository already declares Python `>=3.10, <3.15`
(`pyproject.toml:5-15`) and builds CPython 3.10-3.14 wheels on Linux and Windows
(`.github/workflows/python-wheels.yml:47-69`).

Use nanobind's conventional linked mode for the first migration:
`nanobind_add_module(pyuipc NB_STATIC ...)`. `NB_STATIC` keeps the core runtime
inside this single extension and is nanobind's default. Nanobind also defaults
to size-oriented optimization for extension sources; keep that as the release
candidate, but build a `NOMINSIZE` comparison variant before accepting any
performance result or changing the project's optimization policy
([CMake API](https://nanobind.readthedocs.io/en/latest/api_cmake.html)). Do not
combine this migration with split mode, stable ABI, or free-threaded Python;
those are independent packaging/ABI changes introduced or expanded in 3.0.0
([3.0.0 changelog](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst),
[CMake API](https://nanobind.readthedocs.io/en/latest/api_cmake.html)).

The migration should be staged, but the product extension should switch
frameworks atomically after the central adapters are ready. Nanobind and
pybind11 may coexist in one extension during a spike, but their registered
types are not mutually interchangeable; every type that crosses a binding
boundary must move together
([porting guide](https://nanobind.readthedocs.io/en/latest/porting.html#porting-one-binding-at-a-time)).

## Version decision and CMake/XMake alignment

There is a release-channel mismatch that must be resolved before source
conversion:

| Source checked on 2026-08-27 | Highest version | Consequence |
|---|---:|---|
| nanobind upstream changelog | 3.0.0 | Target API for this migration; includes API breaks and Python >=3.10. |
| official `xmake-repo` nanobind recipe on `dev` | 2.12.0 | A plain official `add_requires("nanobind 3.0.0")` cannot currently resolve a checksum-pinned 3.0.0 recipe. |

Evidence: upstream's tagged changelog identifies 3.0.0 and its API changes
([nanobind v3.0.0 changelog](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst));
the official recipe lists `v2.12.0` as its newest entry
([xmake-repo recipe](https://github.com/xmake-io/xmake-repo/blob/dev/packages/n/nanobind/xmake.lua#L1-L16)).
The official XMake port builds a compiled `nanobind` library from nanobind's
`src/*.cpp` and exports its headers, rather than treating nanobind as a
header-only dependency
([xmake-repo port](https://github.com/xmake-io/xmake-repo/blob/dev/packages/n/nanobind/port/xmake.lua#L26-L54)).
That matches nanobind's own warning that an extension needs both a module and a
compiled library component with platform-specific compile/link handling
([nanobind CMake API](https://nanobind.readthedocs.io/en/latest/api_cmake.html)).

Recommended dependency contract:

1. Pin `nanobind==3.0.0` exactly in root `pyproject.toml` under
   `[build-system].requires`. The current isolated build requirements contain
   only scikit-build-core and setuptools-scm (`pyproject.toml:1-3`).
2. Make CMake import nanobind from the selected Python interpreter and use
   `find_package(nanobind 3.0.0 EXACT CONFIG REQUIRED)` followed by
   `nanobind_add_module`. The
   current equivalent pybind discovery is in `src/pybind/CMakeLists.txt:1-22`.
3. Add a repository-local XMake package overlay containing a checksum-pinned
   `v3.0.0` recipe and port, require exactly `nanobind 3.0.0` with
   `system = false`, and enable XMake's package lock policy. XMake officially
   supports project-local package repositories and dependency locks; it also
   documents that disabling checksum verification carries integrity and
   incomplete-package risk
   ([XMake package guide](https://xmake.io/guide/package-management/using-official-packages.html)).
   Do not use `{verify = false}` for the release build. Replace the overlay with
   the official recipe only after `xmake-repo` publishes and CI validates the
   same 3.0.0 recipe.
4. Add a repository-contract check that extracts the version from the root
   build requirement, CMake configure check, XMake exact requirement, and
   overlay recipe, and fails on any mismatch. This is deliberately redundant:
   it makes drift visible in both build systems. The owner rule already requires
   CMake and XMake dependencies and pins to stay synchronized
   (`agent_docs/rule.md:48-58`).

The XMake overlay is a **Stage 0 build-feasibility gate**, not assumed working:
nanobind 3.0.0 adds split mode and changes several APIs, while the inspected
official port is currently exercised against 2.12.0. Validate the overlay on
Linux and Windows, including an isolated probe with the repository's declared
minimum XMake 3.0.5 (`xmake.lua:1`), before converting the full binding tree.

## Current repository surface

The native Python surface is large: a static scan found 216 C++ binding files
(106 `.cpp`, 110 `.h`, 10,484 lines), 130 `py::class_` declaration lines, 25
class declarations containing the project `S<T>` shared-pointer holder, 137
`py::array_t` occurrences, 48 explicit return-value policies, eight trampoline
macros, and ten references to pybind11 internals. These counts were produced by
`find`/`rg` over `src/pybind/pyuipc/**/*.{cpp,h}` and should be regenerated on
the implementation branch before mechanical conversion.

The important concentration points are:

- The common header imports pybind11 and defines the global alias used by the
  whole tree (`src/pybind/pyuipc/pyuipc.h:1-14`).
- The module entry point creates eight submodules and exposes top-level aliases
  (`src/pybind/pyuipc/module.cpp:50-63`, `:108-162`). Preserve this registration
  order because early types are registered before consumers (`:114-144`).
- CMake discovers pybind11, creates `pyuipc`, and runs a post-build package/stub
  script (`src/pybind/CMakeLists.txt:1-22`,
  `src/pybind/pyuipc/CMakeLists.txt:1-20`, `:65-100`).
- XMake currently treats pybind11 as a header dependency, builds a shared
  extension, and copies the extension/runtime libraries after build
  (`src/pybind/xmake.lua:1-29`, `:77-112`). Its separate wheel packer installs
  the target and runs mypy stubgen (`xmake/pack.lua:21-71`).
- The public package imports `uipc._native.pyuipc`, initializes the native
  library, and re-exports its symbols (`python/src/uipc/__init__.py:1-26`). The
  wrapper modules are one-line re-exports from native submodules, for example
  `python/src/uipc/core.py:1` and `python/src/uipc/geometry.py:1`.
- There are 21 `python/tests/test_*.py` files with 89 static test-function
  matches. The wheel workflow already runs the no-GPU smoke test and portable
  pytest suite for every supported CPython/OS pair
  (`.github/workflows/python-wheels.yml:132-168`).

## Required source migrations

| Current construct and local evidence | Required nanobind treatment | Risk |
|---|---|---|
| `PYBIND11_MODULE`, `py::module`, `py::register_exception`, and `py::module::import` (`src/pybind/pyuipc/module.cpp:41-63`, `src/pybind/pyuipc/common/json.h:107-118`) | Use `NB_MODULE`, `nb::module_`, `nb::exception<T>`, and `nb::module_::import_`. The official rename table covers these API families ([porting guide](https://nanobind.readthedocs.io/en/latest/porting.html)). | Medium, because every initializer currently accepts `py::module&`. |
| `g_top_module = &m` and `top_module()` returns a wrapper reference (`src/pybind/pyuipc/module.cpp:41-52`) | Do not retain `&m`. In nanobind 3.0.0, `NB_MODULE` passes its `module_` wrapper by value, so the address is invalid after module initialization ([tagged macro source](https://github.com/wjakob/nanobind/blob/v3.0.0/include/nanobind/nb_defs.h#L201-L236)). Store a borrowed raw `PyObject *` and return a fresh borrowed `nb::module_` by value, or pass the top module explicitly. | Critical lifetime bug if mechanically renamed. |
| Holder-bearing classes such as `py::class_<PyIEngine, PyIEngine_, IEngine, S<PyIEngine>>` (`src/pybind/pyuipc/core/engine.cpp:164-171`) and other `S<T>` declarations | Remove holder template arguments. Include `nanobind/stl/shared_ptr.h` wherever APIs exchange `std::shared_ptr`; audit raw-pointer calls to `shared_from_this()`. Nanobind deliberately removed holders and has different `enable_shared_from_this` timing ([porting guide](https://nanobind.readthedocs.io/en/latest/porting.html#shared-pointers-and-holders), [ownership guide](https://nanobind.readthedocs.io/en/latest/ownership.html)). | Critical ownership/identity/lifetime behavior. |
| `py::array_t`, `py::buffer_info`, and private write-flag mutation (`src/pybind/pyuipc/as_numpy.h:14-69`, `:71-145`) | Replace the central adapter with typed `nb::ndarray` aliases for NumPy/CPU/dtype/rank/contiguity. Specify an owner for every returned view and use supported read-only annotations rather than `py::detail`. Nanobind ndarray strides count elements, while this helper currently builds byte strides ([ndarray guide](https://nanobind.readthedocs.io/en/latest/ndarray.html#dynamic-array-configurations)). | Critical data corruption, mutability, and dangling-view risk. |
| Owning arrays created with `py::array_t(shape)` and `mutable_unchecked`, e.g. `src/pybind/pyuipc/geometry/affine_body.cpp` | Add one tested allocation helper using an owning capsule or a NumPy allocation reached through the public Python API. Nanobind's documented C++-to-Python ownership mechanism is the ndarray `owner` object/capsule ([ndarray ownership](https://nanobind.readthedocs.io/en/latest/ndarray.html#data-ownership)). | High. Do not scatter ad-hoc allocation code. |
| JSON conversion depends on `py::detail::*_accessor` and `PYBIND11_TYPE_CASTER` (`src/pybind/pyuipc/common/json.h:143-225`) | Rewrite using public nanobind wrapper APIs and `NB_TYPE_CASTER`. Implement `from_python(handle, uint32_t flags, cleanup_list*) noexcept` and `from_cpp(..., rv_policy, cleanup_list*) noexcept`, including the documented asymmetric Python-error handling ([porting guide](https://nanobind.readthedocs.io/en/latest/porting.html#type-casters), [3.0.0 caster change](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)). | Critical; JSON is used across configuration and metadata APIs. |
| Eight `PYBIND11_OVERRIDE_PURE` calls (`src/pybind/pyuipc/core/pyengine.h:31-90`) | Add `NB_TRAMPOLINE(PyIEngine)` to the alias class and convert to `NB_OVERRIDE_PURE`. Nanobind 3 no longer needs a trampoline slot count ([porting guide](https://nanobind.readthedocs.io/en/latest/porting.html#trampoline-classes), [3.0.0 changelog](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)). | High; verify Python subclass dispatch and pure-virtual failures. |
| Factory lambda passed to `py::init` captures Python callbacks (`src/pybind/pyuipc/backend/buffer.cpp:8-49`) | Convert to nanobind's placement-new `__init__` pattern. Explicitly test callback reference cycles, GIL acquisition, exceptions, destruction, and interpreter shutdown ([custom constructors](https://nanobind.readthedocs.io/en/latest/porting.html#custom-constructors), [reference leaks](https://nanobind.readthedocs.io/en/latest/refleaks.html)). | High. |
| Resident-thread callback captures `py::function` and later acquires the GIL (`src/pybind/pyuipc/common/resident_thread.cpp:14-44`) | Convert wrappers/GIL guards and check guard validity before using Python during shutdown. Nanobind 3 documents that `gil_scoped_acquire` may fail when Python is finalizing ([3.0.0 changelog](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)). | High concurrency/shutdown risk. |
| `py::make_iterator(begin, end)` (`src/pybind/pyuipc/common/span.h:6-44`, `src/pybind/pyuipc/geometry/attribute_slot.cpp:40-76`) | Include `nanobind/make_iterator.h` and provide the required scope and installed iterator name ([iterator porting](https://nanobind.readthedocs.io/en/latest/porting.html#iterator-bindings)). | Medium. |
| `.def_readwrite`, `py::return_value_policy`, `py::doc`, and `py::none`/typed wrapper mixing (`src/pybind/pyuipc/builtin/uid_info.cpp:7-20`, `src/pybind/pyuipc/geometry/attribute_slot.cpp:128-164`) | Use `.def_rw`, `nb::rv_policy`, direct doc strings, and explicit `nb::object` return types where a branch can return `None`. Audit every nullable argument because nanobind rejects `None` unless annotated or defaulted ([porting guide](https://nanobind.readthedocs.io/en/latest/porting.html#none-null-arguments), [3.0.0 changelog](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)). | Medium API compatibility risk. |
| Eigen casters are included in two external-force bindings (`src/pybind/pyuipc/constitution/affine_body_external_force.cpp:1-15`, `src/pybind/pyuipc/constitution/finite_element_external_force.cpp:1-15`) | Use `nanobind/eigen/dense.h` and verify reference/copy behavior rather than relying on include-name substitution ([Eigen guide](https://nanobind.readthedocs.io/en/latest/eigen.html)). | Medium. |

No clear product use of pybind11 multiple inheritance was found by the static
scan; the most complex declaration combines one C++ base, one trampoline, and
one holder (`src/pybind/pyuipc/core/engine.cpp:164-171`). This is not compile
proof. Nanobind does not support pybind11-style multiple inheritance
([removed features](https://nanobind.readthedocs.io/en/latest/porting.html#removed-features)),
so the converted declaration inventory must be checked before the first full
compile.

## Build, packaging, and stub generation

### CMake path

Replace `pybind11_add_module` and `pybind11::module` in
`src/pybind/pyuipc/CMakeLists.txt:1-5` with the pinned nanobind target. Preserve
all backend dependencies, `LINK_DEPENDS`, compile-time build metadata, output
directory, RPATH, install, and runtime-library copy behavior
(`src/pybind/pyuipc/CMakeLists.txt:6-63`, `:93-100`). Remove
`PYBIND11_DETAILED_ERROR_MESSAGES` (`:19`) rather than inventing an equivalent.

The current CMake post-build script stages Python sources and runtime libraries,
generates stubs with `pybind11_stubgen`, and optionally installs the development
package (`scripts/after_build_pyuipc.py:54-97`, `:99-142`, `:143-188`). Keep
that ordering, but replace the generator with the stable nanobind CLI, not the
experimental `nanobind.stubgen.StubGen` Python API
([stubgen CLI](https://nanobind.readthedocs.io/en/latest/typing.html#command-line-interface)).
The official CMake `nanobind_add_stub` helper is valid, but it imports the target
module and therefore must depend on the fully staged extension and runtime
libraries
([CMake stub generation](https://nanobind.readthedocs.io/en/latest/api_cmake.html#stub-generation)).
For this repository, a small shared CLI wrapper called after staging is less
likely to regress the existing backend-copy/development-install behavior.

### XMake path

XMake must link the compiled nanobind package rather than only changing headers
in `src/pybind/xmake.lua:1-29`. Preserve the Python extension suffix rule and
runtime library copy (`src/pybind/xmake.lua:77-139`). Validate that the local
3.0.0 overlay propagates every platform definition/link option from its port on
Windows and Linux.

XMake stubs are generated separately at wheel-package time: `xmake/pack.lua`
currently installs the extension, installs `mypy`/NumPy if needed, and invokes
`scripts/stubgen.py` (`xmake/pack.lua:21-71`); that script calls mypy's
inspection-based API (`scripts/stubgen.py:1-60`). Replace this with the same
repository wrapper around:

```text
python -m nanobind.stubgen \
  -m uipc._native.pyuipc \
  -r \
  -O <staged-package>/src/uipc/_native \
  -M <staged-package>/src/uipc/py.typed
```

Nanobind documents `-r`, `-O`, `-M`, and repeatable pattern files as stable CLI
features ([typing/stubgen](https://nanobind.readthedocs.io/en/latest/typing.html#command-line-interface)).
Use a shared output manifest/pattern file for CMake and XMake, then assert the
same generated file set and normalized contents from both paths. Do not assume
`add_packages("nanobind")` alone makes `python -m nanobind.stubgen` importable;
the Stage 0 XMake probe must verify or explicitly add the overlay's Python
directory to the selected interpreter environment.

Importing the dotted native module executes `uipc/__init__.py`, which currently
calls `init()` unconditionally (`python/src/uipc/__init__.py:6-24`). The
nanobind CLI sets `NB_STUBGEN=1` specifically so packages can skip runtime/device
initialization ([stubgen detection](https://nanobind.readthedocs.io/en/latest/typing.html#detecting-stub-generation)).
Guard only the runtime `init()` call during stub generation; keep native module
registration and re-exports active. Add a normal-import regression test to
prove production initialization remains unchanged.

The wheel gate must inspect the installed archive for the native extension,
recursive native stubs, public wrapper modules, and `uipc/py.typed`. Existing
stub tests encode an expected `pyuipc` stub directory but are not in the active
wheel command (`scripts/test_wheel.py:19-67`,
`scripts/test_user_experience.py:118-167` versus
`.github/workflows/python-wheels.yml:132-168`), so add a maintained assertion to
`scripts/smoke_test_wheel.py` rather than relying on those legacy scripts.

## Staged implementation and gates

1. **Stage 0 -- freeze behavior and prove both builds.** Record an API/stub
   manifest from the current pybind11 extension. Add focused tests for ndarray
   conversion/view ownership, shared ownership, trampoline callbacks, JSON,
   thread callbacks, exceptions, module aliases, and clean interpreter exit.
   Build a tiny non-product nanobind 3.0.0 probe with CMake and the local XMake
   overlay on Linux and Windows. Gate: exact dependency version and importable
   probe in both systems.
2. **Stage 1 -- dependency/build/stub infrastructure.** Add the exact version
   contract, CMake integration, XMake overlay/compiled-core link, shared stub CLI
   wrapper, `NB_STUBGEN` guard, and version-drift contract test. Keep the product
   binding on pybind11 until the nanobind product source is ready. Gate: both
   build descriptions configure the same version and produce the expected probe
   artifact/stubs.
3. **Stage 2 -- central high-risk adapters.** Implement and test ndarray
   ownership/shape/stride/mutability, JSON caster, shared-pointer exchange,
   trampoline, custom Buffer constructor, iterator helper, and top-module
   lifetime. Gate: focused tests pass under sanitizers where supported and a
   subprocess exit test emits no unexpected nanobind leak warnings.
4. **Stage 3 -- mechanical binding conversion.** Convert leaf modules in
   registration order, remove holder arguments, update fields/policies/docs and
   nullable arguments, then switch the product entry point to `NB_MODULE`.
   Update the constitution source checker, which currently hard-codes
   `py::class_`, `py::module`, and “pybind module”
   (`scripts/check_constitution_api.py:12-28`, `:44-76`, `:115-124`) and its
   pybind-specific fixtures (`scripts/tests/test_repository_contracts.py:36-76`).
   Gate: no active pybind11 API/include remains and the full portable suite
   passes.
5. **Stage 4 -- package and wheel parity.** Generate identical stubs through
   CMake and XMake, build fresh wheels, install into clean environments, compare
   the API/stub manifest against Stage 0, and run wheel smoke/pytest for
   CPython 3.10-3.14 on Linux and Windows. Gate: wheel contents, imports,
   `build_info()`, `Engine("none")`, stubs, and shutdown all pass.
6. **Stage 5 -- runtime/performance validation.** Run CUDA-marked tests on a
   supported GPU and compare clean build time, extension size, import time,
   representative binding-call latency, and ndarray view throughput against the
   frozen pybind11 baseline. Measure nanobind's default size optimization and a
   `NOMINSIZE` variant before selecting the release policy. Gate: no unexplained
   semantic or material performance regression; document accepted differences.

Keep the existing option and source-directory names (`UIPC_BUILD_PYBIND`,
`--pybind`, `src/pybind`) as compatibility names during this migration. Renaming
them is a separate cleanup that would add downstream and CI churn without
proving binding parity.

## Required regression tests

- **Arrays:** list/NumPy inputs; accepted/rejected dtype conversions; C/F and
  non-contiguous layouts; 1D/2D/3D shape failures; empty spans; writable versus
  read-only views; `OWNDATA`/base owner; use after the originating C++ wrapper is
  deleted. Existing array behavior is exercised in
  `python/tests/test_attribute.py:1-25` and type/value basics in
  `python/tests/test_typing.py:1-16`, but neither is a complete ownership suite.
- **Ownership:** `Scene`/geometry/attribute shared objects, base/derived returns,
  Python-created `PyIEngine`, identity on repeated returns, and destruction
  order. Existing scene coverage starts at `python/tests/test_scene.py:1-40`.
- **Callbacks/GIL:** every `PyIEngine` pure virtual, Buffer resize/view callbacks,
  ResidentThread success/exception/destruction, and subprocess shutdown while a
  callback is queued. Keep nanobind leak warnings enabled in CI; the official
  guide explains that wrappers, functions, and shared ownership can form
  uncollectable cycles ([reference-leak guide](https://nanobind.readthedocs.io/en/latest/refleaks.html)).
- **API/stubs:** module/class/function names, signatures, defaults, docstrings,
  exceptions, top-level aliases, recursive native stubs, `py.typed`, and a real
  static type-check sample. The existing `test_typing.py` is a runtime value
  test, not a stub/type-checker validation (`python/tests/test_typing.py:1-16`).
- **Wheel/runtime:** fresh-environment import, compatibility metadata, native
  library colocation, `Engine("none")`, CUDA backend load on a GPU runner, and
  clean interpreter exit. The existing smoke test provides the first four
  checks except stubs and exit diagnostics (`scripts/smoke_test_wheel.py:13-73`).

The current general XMake workflow never enables `--pybind=y`
(`.github/workflows/xmake.yml:79-93`), while the wheel workflow ignores all
`xmake.lua` changes (`.github/workflows/python-wheels.yml:9-37`). Add a dedicated
XMake Python-binding configure/build/import/stub job; otherwise CMake wheel
success cannot validate the required build-system parity.

## Main residual risks

1. **XMake 3.0.0 packaging is unproven.** The official recipe lag is a hard
   migration gate, not a reason to silently build 2.12.0 with XMake.
2. **Array semantics may change silently.** Byte-versus-element strides,
   conversion defaults, read-only flags, and owner objects require runtime
   assertions and negative tests.
3. **Holder removal changes lifetime semantics.** Compilation success does not
   prove `shared_from_this`, identity, or base/derived ownership behavior.
4. **Python callback cycles/shutdown are high risk.** Buffer and ResidentThread
   retain Python callables across C++ lifetimes; leak warnings and subprocess
   exit tests are required.
5. **Stub layout can regress independently in CMake and XMake.** Treat generated
   file manifests and type-checker output as release artifacts, not incidental
   build by-products.
6. **No build has been run for this report.** Exact nanobind template spellings,
   platform flags, the 3.0.0 XMake overlay, and all runtime semantics remain to
   be proven by the staged gates above.

## Primary sources

- [nanobind 3.0.0 changelog](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)
- [nanobind pybind11 porting guide](https://nanobind.readthedocs.io/en/latest/porting.html)
- [nanobind CMake API](https://nanobind.readthedocs.io/en/latest/api_cmake.html)
- [nanobind build guide](https://nanobind.readthedocs.io/en/latest/building.html)
- [nanobind packaging guide](https://nanobind.readthedocs.io/en/latest/packaging.html)
- [nanobind ndarray guide](https://nanobind.readthedocs.io/en/latest/ndarray.html)
- [nanobind ownership guide](https://nanobind.readthedocs.io/en/latest/ownership.html)
- [nanobind advanced ownership guide](https://nanobind.readthedocs.io/en/latest/ownership_adv.html)
- [nanobind Eigen guide](https://nanobind.readthedocs.io/en/latest/eigen.html)
- [nanobind typing and stubgen guide](https://nanobind.readthedocs.io/en/latest/typing.html)
- [nanobind reference-leak guide](https://nanobind.readthedocs.io/en/latest/refleaks.html)
- [nanobind 3.0.0 module macro source](https://github.com/wjakob/nanobind/blob/v3.0.0/include/nanobind/nb_defs.h#L201-L236)
- [official xmake-repo nanobind recipe](https://github.com/xmake-io/xmake-repo/blob/dev/packages/n/nanobind/xmake.lua)
- [official xmake-repo nanobind port](https://github.com/xmake-io/xmake-repo/blob/dev/packages/n/nanobind/port/xmake.lua)
- [XMake official package-management guide](https://xmake.io/guide/package-management/using-official-packages.html)
- [pybind11 NumPy guide](https://pybind11.readthedocs.io/en/stable/advanced/pycpp/numpy.html)
- [pybind11 smart-pointer guide](https://pybind11.readthedocs.io/en/stable/advanced/smart_ptrs.html)

## Static verification performed

- Inspected Git branch, commit, and worktree status before writing this report.
- Read both binding build descriptions, both stub-generation/package paths,
  root and development Python metadata, module initialization, central ndarray
  and JSON adapters, shared-holder declarations, trampoline/callback code,
  Python package re-exports, test inventory, repository-contract checker, and
  wheel/XMake workflows.
- Counted binding files and high-risk API tokens with `find` and `rg`.
- Cross-checked every external migration claim against upstream nanobind 3.0.0
  documentation/source, upstream pybind11 documentation where current behavior
  matters, and the official XMake repository/documentation.
- Did **not** fetch, switch branches, install dependencies, configure, build,
  import the extension, generate stubs, run tests, build wheels, or execute a
  simulator/runtime validation.
