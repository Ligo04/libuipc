# pybind11 到 nanobind 迁移方案

状态：`src/nanobind` 是默认的 `nanobind 3.0.0` 绑定实现，`src/pybind`
保留迁移前的 pybind11 实现；CMake 和 XMake 均可在唯一入口选择一个适配器。
Linux x86_64 上，两套适配器已经通过 CMake/XMake 编译、链接、导入、stub
和运行时验证。根目录 PEP 517/scikit-build-core 入口已生成 CPython
3.10.20、3.11.15、3.12.3、3.13.12 和 3.14.3 的五个 CPU release wheel；
每个 wheel 均在独立干净环境中安装，包含匹配 ABI 的扩展、10 个递归 stub
及 `py.typed`，通过 `Engine("none")` 冒烟和可移植测试集（84 项通过、仅
可选 Warp 适配器 1 项跳过、54 项按标记排除）。CPython 3.12.3 CUDA
release 产品构建、SM 120 cubin/PTX、CUDA Engine 冒烟和 `wrecking_balls`
性能对照也已通过。Windows 和完整 CUDA pytest 仍未验证。

调研与实施基线：分支 `codex/migrate-pybind-to-nanobind`，迁移拆分提交
`c9e5229249cc8383dd0a3bb4e811cd31714cf41b`，日期 2026-08-28。

## 建议

建议将现有 `pyuipc` 扩展迁移到 **nanobind 3.0.0**，同时保持 Python
发行包名称（`pyuipc`）、扩展名称（`uipc._native.pyuipc`）、公开模块布局
以及 Python 3.10-3.14 wheel 矩阵不变。本方案调研时 nanobind 3.0.0
是最新发布版本；该版本发布于 2026-08-22，要求 Python 3.10 或更高版本，
内部 ABI 为 22
（[3.0.0 更新日志](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)）。
本仓库已经声明 Python `>=3.10, <3.15`（`pyproject.toml:5-15`），并在
Linux 和 Windows 上构建 CPython 3.10-3.14 wheel
（`.github/workflows/python-wheels.yml:47-69`）。

首次迁移使用 nanobind 的常规链接模式：
`nanobind_add_module(pyuipc NB_STATIC ...)`。`NB_STATIC` 将核心运行时保留在
这一个扩展内，也是 nanobind 的默认配置。Nanobind 默认还会对扩展源文件
采用面向体积的优化；将其作为发布候选，但在接受任何性能结论或修改项目
优化策略前，额外构建一个 `NOMINSIZE` 对照版本
（[CMake API](https://nanobind.readthedocs.io/en/latest/api_cmake.html)）。
本次迁移不要同时引入 split mode、Stable ABI 或 free-threaded Python；
这些是 3.0.0 新增或扩展的独立打包/ABI 变更
（[3.0.0 更新日志](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)、
[CMake API](https://nanobind.readthedocs.io/en/latest/api_cmake.html)）。

迁移过程应当分阶段实施，但中央适配器准备好后，产品扩展应一次性切换
绑定框架。试验阶段可以让 nanobind 与 pybind11 共存于同一个扩展中，
但两者注册的类型不能相互交换；所有跨越绑定边界的类型必须一起迁移
（[迁移指南](https://nanobind.readthedocs.io/en/latest/porting.html#porting-one-binding-at-a-time)）。

## 版本决策与 CMake/XMake 对齐

在转换源代码之前，必须先解决以下发布渠道不一致问题：

| 2026-08-27 检查的来源 | 最高版本 | 影响 |
|---|---:|---|
| nanobind 上游更新日志 | 3.0.0 | 本次迁移的目标 API；包含 API 破坏性变更，要求 Python >=3.10。 |
| 官方 `xmake-repo` 的 `dev` 分支 nanobind 配方 | 2.12.0 | 直接使用官方 `add_requires("nanobind 3.0.0")` 暂时无法解析到带校验和的 3.0.0 配方。 |
| 本仓库 XMake overlay | 3.0.0 | 精确标签和 SHA-256 已写入本地配方；Linux 隔离探针已验证。 |

依据：上游带标签的更新日志明确列出了 3.0.0 及其 API 变更
（[nanobind v3.0.0 更新日志](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)）；
官方配方当前最新条目为 `v2.12.0`
（[xmake-repo 配方](https://github.com/xmake-io/xmake-repo/blob/dev/packages/n/nanobind/xmake.lua#L1-L16)）。
官方 XMake port 会从 nanobind 的 `src/*.cpp` 构建一个已编译的
`nanobind` 库并导出其头文件，而不是把 nanobind 当作纯头文件依赖
（[xmake-repo port](https://github.com/xmake-io/xmake-repo/blob/dev/packages/n/nanobind/port/xmake.lua#L26-L54)）。
这与 nanobind 自身的说明一致：一个扩展既需要模块，也需要已编译的库组件，
并且要处理平台相关的编译与链接选项
（[nanobind CMake API](https://nanobind.readthedocs.io/en/latest/api_cmake.html)）。

建议采用以下依赖契约：

1. 在根目录 `pyproject.toml` 的 `[build-system].requires` 中精确固定
   `nanobind==3.0.0`。当前隔离构建依赖只有 scikit-build-core 和
   setuptools-scm（`pyproject.toml:1-3`）。
2. 让 CMake 从选定的 Python 解释器导入 nanobind，使用
   `find_package(nanobind 3.0.0 EXACT CONFIG REQUIRED)`，随后调用
   `nanobind_add_module`。现有对应的 pybind 发现逻辑位于
   `src/pybind/CMakeLists.txt:1-22`。
3. 仓库内 XMake 包 overlay 已放在
   `xmake/repository/packages/n/nanobind`，根 `xmake.lua` 已注册
   `libuipc-packages` 本地仓库。配方包含带 SHA-256 的 `v3.0.0` 和适配后的
   port。产品绑定切换时，通过 `system = false` 精确要求
   `nanobind 3.0.0`，并启用 XMake 包锁定策略。XMake 官方支持项目本地
   包仓库和依赖锁，
   也明确说明禁用校验和会带来完整性与包不完整风险
   （[XMake 包管理指南](https://xmake.io/guide/package-management/using-official-packages.html)）。
   发布构建不得使用 `{verify = false}`。只有在 `xmake-repo` 发布相同的
   3.0.0 配方并通过 CI 验证后，才用官方配方替换该 overlay。
4. 增加一个仓库契约检查，从根构建依赖、CMake 配置检查、XMake 精确版本
   要求和 overlay 配方中提取版本，发现任何不一致时直接失败。这里有意
   保留冗余，以便两套构建系统中的版本漂移都能被发现。项目规则已经要求
   CMake 与 XMake 的依赖和版本固定保持同步
   （`agent_docs/rule.md:48-58`）。

XMake overlay 是一个**阶段 0 构建可行性门槛**。当前 Linux 探针已经证明：
本地仓库能解析精确的 3.0.0，port 能编译 nanobind 核心，CPython 扩展能
链接和导入，并且扩展的 ELF `NEEDED` 项不包含 `libpython`。这还不是完整的
跨平台验收；在转换完整绑定树之前，仍应在 Windows 上验证该 overlay，
仓库声明的最低 XMake 3.0.5（`xmake.lua:1`）兼容性应由 CI 矩阵持续覆盖；
本次本地开发与验收按工作区最新 XMake 执行。

本地 overlay 保留官方配方的自动依赖解析行为，同时匹配 nanobind 各版本线的
实际 Python 下限：2.10.0 之前为 `>=3.8`，2.10.0-2.12.x 为 `>=3.9`，
3.x 为 `>=3.10`。官方当前 2.x 配方统一声明 `>=3.8`，本地 overlay 修正了
其中 2.10.0 及以上版本过宽的下限。具体解释器仍可由产品层
`add_requireconfs`、`python_system` 和外部构建环境约束，但不再作为
nanobind 包自身的配置进入包哈希。`libnanobind` 静态核心依赖 CPython ABI；
CPython 3.12 wheel 验证曾捕获旧的 Python 3.14.3 核心被错误复用并导致导入时
缺少 `PyThreadState_GetUnchecked`。因此切换 Python 次版本时必须隔离或清理
nanobind 包缓存，CI 矩阵也不能跨 ABI 复用同一个已编译核心。清空既有 3.0.0
安装后，CPython 3.12.3 配置自动解析并重建了无 ABI 配置的包哈希；manifest
记录 Python 3.12.3，随后产品构建、wheel 导入和 `Engine("none")` 冒烟通过。

## 当前仓库范围

原生 Python 绑定规模较大：静态扫描发现 216 个 C++ 绑定文件
（106 个 `.cpp`、110 个 `.h`，共 10,484 行）、130 行 `py::class_`
声明、25 个包含项目 `S<T>` 共享指针 holder 的类声明、137 处
`py::array_t`、48 个显式返回值策略、8 个 trampoline 宏，以及 10 处
对 pybind11 内部实现的引用。这些统计由 `find`/`rg` 扫描
`src/pybind/pyuipc/**/*.{cpp,h}` 得到；开始机械转换前，应在实际实现分支
重新生成统计结果。

需要重点处理的位置如下：

- 公共头文件导入 pybind11，并定义整棵绑定树使用的全局别名
  （`src/pybind/pyuipc/pyuipc.h:1-14`）。
- 模块入口创建 8 个子模块并暴露顶层别名
  （`src/pybind/pyuipc/module.cpp:50-63`、`:108-162`）。必须保留注册顺序，
  因为前面的类型会先于其使用方完成注册（`:114-144`）。
- CMake 发现 pybind11、创建 `pyuipc`，并运行构建后的包整理/stub 脚本
  （`src/pybind/CMakeLists.txt:1-22`、
  `src/pybind/pyuipc/CMakeLists.txt:1-20`、`:65-100`）。
- XMake 当前把 pybind11 当作头文件依赖，构建共享扩展，并在构建后复制
  扩展及运行时库（`src/pybind/xmake.lua:1-29`、`:77-112`）。其独立的
  wheel 打包器会安装目标并运行 mypy stubgen（`xmake/pack.lua:21-71`）。
- 公开包导入 `uipc._native.pyuipc`、初始化原生库并重新导出其中的符号
  （`python/src/uipc/__init__.py:1-26`）。包装模块只是从原生子模块执行
  单行重新导出，例如 `python/src/uipc/core.py:1` 和
  `python/src/uipc/geometry.py:1`。
- `python/tests/test_*.py` 下共有 21 个文件，静态匹配到 89 个测试函数。
  wheel 工作流已经针对每组受支持的 CPython/操作系统组合运行无 GPU
  smoke test 和可移植 pytest 测试集
  （`.github/workflows/python-wheels.yml:132-168`）。

## 必须实施的源代码迁移

| 当前结构与本地依据 | 所需 nanobind 处理 | 风险 |
|---|---|---|
| `PYBIND11_MODULE`、`py::module`、`py::register_exception` 和 `py::module::import`（`src/pybind/pyuipc/module.cpp:41-63`、`src/pybind/pyuipc/common/json.h:107-118`） | 使用 `NB_MODULE`、`nb::module_`、`nb::exception<T>` 和 `nb::module_::import_`。官方重命名表覆盖了这些 API 系列（[迁移指南](https://nanobind.readthedocs.io/en/latest/porting.html)）。 | 中等，因为每个初始化器当前都接收 `py::module&`。 |
| `g_top_module = &m`，且 `top_module()` 返回包装对象引用（`src/pybind/pyuipc/module.cpp:41-52`） | 不得保留 `&m`。在 nanobind 3.0.0 中，`NB_MODULE` 按值传入 `module_` 包装对象，因此模块初始化结束后该地址会失效（[带标签的宏源码](https://github.com/wjakob/nanobind/blob/v3.0.0/include/nanobind/nb_defs.h#L201-L236)）。应保存借用的原始 `PyObject *`，并按值返回新构造的借用 `nb::module_`；或者显式传递顶层模块。 | 如果只机械重命名，会产生严重生命周期错误。 |
| 带 holder 的类，例如 `py::class_<PyIEngine, PyIEngine_, IEngine, S<PyIEngine>>`（`src/pybind/pyuipc/core/engine.cpp:164-171`）以及其他 `S<T>` 声明 | 移除 holder 模板参数。凡 API 交换 `std::shared_ptr` 的位置都应包含 `nanobind/stl/shared_ptr.h`；审计通过裸指针调用 `shared_from_this()` 的位置。Nanobind 有意移除了 holder，并且 `enable_shared_from_this` 的生效时机不同（[迁移指南](https://nanobind.readthedocs.io/en/latest/porting.html#shared-pointers-and-holders)、[所有权指南](https://nanobind.readthedocs.io/en/latest/ownership.html)）。 | 严重的所有权、对象标识和生命周期风险。 |
| `py::array_t`、`py::buffer_info` 和对可写标志的私有修改（`src/pybind/pyuipc/as_numpy.h:14-69`、`:71-145`） | 用带 NumPy/CPU/dtype/维度/连续性约束的 `nb::ndarray` 类型别名重写中央适配器。为每个返回视图指定 owner，并使用受支持的只读标注，不再使用 `py::detail`。Nanobind ndarray 的 stride 单位是元素，而当前辅助函数构造的是字节 stride（[ndarray 指南](https://nanobind.readthedocs.io/en/latest/ndarray.html#dynamic-array-configurations)）。 | 严重的数据损坏、可变性和悬空视图风险。 |
| 使用 `py::array_t(shape)` 和 `mutable_unchecked` 创建自有数据数组，例如 `src/pybind/pyuipc/geometry/affine_body.cpp` | 增加一个经过测试的统一分配辅助函数，使用拥有数据的 capsule，或通过公开 Python API 创建 NumPy 分配。Nanobind 文档规定的 C++ 到 Python 所有权机制是 ndarray 的 `owner` 对象/capsule（[ndarray 所有权](https://nanobind.readthedocs.io/en/latest/ndarray.html#data-ownership)）。 | 高。不要把临时分配代码散落到各处。 |
| JSON 转换依赖 `py::detail::*_accessor` 和 `PYBIND11_TYPE_CASTER`（`src/pybind/pyuipc/common/json.h:143-225`） | 使用公开 nanobind 包装 API 和 `NB_TYPE_CASTER` 重写。实现 `from_python(handle, uint32_t flags, cleanup_list*) noexcept` 和 `from_cpp(..., rv_policy, cleanup_list*) noexcept`，并遵循文档规定的非对称 Python 错误处理（[迁移指南](https://nanobind.readthedocs.io/en/latest/porting.html#type-casters)、[3.0.0 caster 变更](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)）。 | 严重；JSON 广泛用于配置和元数据 API。 |
| 8 处 `PYBIND11_OVERRIDE_PURE` 调用（`src/pybind/pyuipc/core/pyengine.h:31-90`） | 在别名类中增加 `NB_TRAMPOLINE(PyIEngine)`，并转换为 `NB_OVERRIDE_PURE`。Nanobind 3 不再需要 trampoline 槽位数量（[迁移指南](https://nanobind.readthedocs.io/en/latest/porting.html#trampoline-classes)、[3.0.0 更新日志](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)）。 | 高；必须验证 Python 子类分派和纯虚函数失败行为。 |
| 传给 `py::init` 的工厂 lambda 捕获 Python 回调（`src/pybind/pyuipc/backend/buffer.cpp:8-49`） | 转换为 nanobind 的 placement-new `__init__` 模式。显式测试回调引用环、GIL 获取、异常、析构和解释器关闭（[自定义构造函数](https://nanobind.readthedocs.io/en/latest/porting.html#custom-constructors)、[引用泄漏](https://nanobind.readthedocs.io/en/latest/refleaks.html)）。 | 高。 |
| 常驻线程回调捕获 `py::function`，随后再获取 GIL（`src/pybind/pyuipc/common/resident_thread.cpp:14-44`） | 转换包装对象和 GIL guard，并在解释器关闭期间调用 Python 前检查 guard 是否有效。Nanobind 3 明确说明 Python 正在 finalizing 时 `gil_scoped_acquire` 可能失败（[3.0.0 更新日志](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)）。 | 高并发/关闭风险。 |
| `py::make_iterator(begin, end)`（`src/pybind/pyuipc/common/span.h:6-44`、`src/pybind/pyuipc/geometry/attribute_slot.cpp:40-76`） | 包含 `nanobind/make_iterator.h`，并传入必需的作用域和已安装迭代器名称（[迭代器迁移](https://nanobind.readthedocs.io/en/latest/porting.html#iterator-bindings)）。 | 中等。 |
| `.def_readwrite`、`py::return_value_policy`、`py::doc`，以及 `py::none` 与强类型包装对象混用（`src/pybind/pyuipc/builtin/uid_info.cpp:7-20`、`src/pybind/pyuipc/geometry/attribute_slot.cpp:128-164`） | 使用 `.def_rw`、`nb::rv_policy`、直接文档字符串；当某个分支可能返回 `None` 时，使用显式 `nb::object` 返回类型。审计每个可空参数，因为除非显式标注或提供默认值，否则 nanobind 会拒绝 `None`（[迁移指南](https://nanobind.readthedocs.io/en/latest/porting.html#none-null-arguments)、[3.0.0 更新日志](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)）。 | 中等 API 兼容风险。 |
| 两个外力绑定包含 Eigen caster（`src/pybind/pyuipc/constitution/affine_body_external_force.cpp:1-15`、`src/pybind/pyuipc/constitution/finite_element_external_force.cpp:1-15`） | 使用 `nanobind/eigen/dense.h`，并验证引用/复制行为，不能只替换 include 名称（[Eigen 指南](https://nanobind.readthedocs.io/en/latest/eigen.html)）。 | 中等。 |

静态扫描没有发现产品代码明确使用 pybind11 多重继承；最复杂的声明组合了
一个 C++ 基类、一个 trampoline 和一个 holder
（`src/pybind/pyuipc/core/engine.cpp:164-171`）。这不能作为编译通过的证明。
Nanobind 不支持 pybind11 风格的多重继承
（[已移除功能](https://nanobind.readthedocs.io/en/latest/porting.html#removed-features)），
因此首次完整编译前必须检查转换后的声明清单。

## 构建、打包与 stub 生成

### CMake 路径

将 `src/pybind/pyuipc/CMakeLists.txt:1-5` 中的 `pybind11_add_module` 和
`pybind11::module` 替换为固定版本的 nanobind 目标。保留所有后端依赖、
`LINK_DEPENDS`、编译期构建元数据、输出目录、RPATH、安装和运行时库复制行为
（`src/pybind/pyuipc/CMakeLists.txt:6-63`、`:93-100`）。删除
`PYBIND11_DETAILED_ERROR_MESSAGES`（`:19`），不要虚构等价配置。

当前 CMake 构建后脚本会暂存 Python 源文件和运行时库，使用
`pybind11_stubgen` 生成 stub，并按需安装开发包
（`scripts/after_build_pyuipc.py:54-97`、`:99-142`、`:143-188`）。
保留这个执行顺序，但将生成器替换为稳定的 nanobind CLI，而不是实验性的
`nanobind.stubgen.StubGen` Python API
（[stubgen CLI](https://nanobind.readthedocs.io/en/latest/typing.html#command-line-interface)）。
官方 CMake `nanobind_add_stub` 辅助函数可以使用，但它会导入目标模块，
因此必须依赖已经完整暂存的扩展和运行时库
（[CMake stub 生成](https://nanobind.readthedocs.io/en/latest/api_cmake.html#stub-generation)）。
对于本仓库，在暂存完成后调用一个两套构建共用的小型 CLI 包装脚本，
更不容易破坏现有的后端复制/开发安装行为。

### XMake 路径

本地配方复制自 `xmake-repo` 的
`e063b2eb1633b9c5d37fbfe9a51b39b5115883fa`，并增加 3.0.0 标签校验和。
port 针对 3.0.0 增加 `NB_BUILD` 和非 Windows 的
`-fno-strict-aliasing`，同时排除只能用于 split mode 的
`nb_backend.cpp`；通配源文件仍会纳入 3.0.0 新增的 `nb_datetime.cpp`。
隔离探针执行过清理包缓存后的完整重装，确认这些定义、选项和源文件实际
进入编译命令。仓库规则要求禁用 ccache，因此 port 显式设置了
`build.ccache=false`。

产品切换时，XMake 必须链接这个已编译的 nanobind 包，不能只修改
`src/pybind/xmake.lua:1-29` 中的头文件。保留 Python 扩展后缀规则和运行时库
复制逻辑（`src/pybind/xmake.lua:77-139`）。Linux 的包级验证已经通过，
Windows 平台定义和链接选项仍需验证。

XMake 的 stub 在 wheel 打包阶段单独生成：`xmake/pack.lua` 当前会安装
扩展、按需安装 `mypy`/NumPy，并调用 `scripts/stubgen.py`
（`xmake/pack.lua:21-71`）；该脚本调用 mypy 基于自省的 API
（`scripts/stubgen.py:1-60`）。将它替换为对以下命令的同一个仓库包装脚本：

```text
python -m nanobind.stubgen \
  -m uipc._native.pyuipc \
  -r \
  -O <staged-package>/src/uipc/_native \
  -M <staged-package>/src/uipc/py.typed
```

Nanobind 将 `-r`、`-O`、`-M` 和可重复指定的 pattern file 记录为稳定的
CLI 功能
（[typing/stubgen](https://nanobind.readthedocs.io/en/latest/typing.html#command-line-interface)）。
CMake 与 XMake 应使用同一个输出清单/pattern file，随后断言两条路径生成的
文件集合及规范化内容一致。不要假定只调用 `add_packages("nanobind")`
就能导入 `python -m nanobind.stubgen`；阶段 0 的 XMake 探针必须验证这一点，
或者把 overlay 的 Python 目录显式加入选定解释器的环境。

导入带点号的原生模块会执行 `uipc/__init__.py`，它当前无条件调用 `init()`
（`python/src/uipc/__init__.py:6-24`）。Nanobind CLI 会专门设置
`NB_STUBGEN=1`，使包能够跳过运行时/设备初始化
（[stubgen 检测](https://nanobind.readthedocs.io/en/latest/typing.html#detecting-stub-generation)）。
stub 生成期间只保护运行时 `init()` 调用；原生模块注册和重新导出必须继续
执行。增加正常导入回归测试，证明生产环境初始化行为没有改变。

wheel 门槛必须检查已安装归档中是否包含原生扩展、递归原生 stub、公开包装
模块和 `uipc/py.typed`。现有 stub 测试编码了预期的 `pyuipc` stub 目录，
但没有被当前 wheel 命令执行（`scripts/test_wheel.py:19-67`、
`scripts/test_user_experience.py:118-167`，对比
`.github/workflows/python-wheels.yml:132-168`），因此应在
`scripts/smoke_test_wheel.py` 中增加持续维护的断言，而不是依赖这些旧脚本。

## 分阶段实施与门槛

1. **阶段 0（Linux 已完成）——冻结行为并验证两套构建。** Linux CMake、
   XMake 隔离探针及产品扩展已经通过；Windows 和完整行为基线仍待完成。从迁移前 pybind11
   扩展记录 API/stub 清单。为 ndarray 转换/视图所有权、共享所有权、
   trampoline 回调、JSON、线程回调、异常、模块别名和解释器正常退出增加
   聚焦测试。
   在 Linux 和 Windows 上，分别用 CMake 与本地 XMake overlay 构建一个
   不属于产品代码的最小 nanobind 3.0.0 探针。门槛：两套构建系统都使用
   精确依赖版本，且探针可以导入。
2. **阶段 1（Linux 已完成）——依赖/构建/stub 基础设施。** 精确版本、CMake
   描述、XMake overlay、已编译核心链接、共用 stub CLI 包装脚本、
   `NB_STUBGEN` 保护及版本漂移契约测试已经完成。门槛：两份构建描述配置
   相同版本，并生成预期的探针产物/stub。
3. **阶段 2（XMake 已验证）——中央高风险适配器。** ndarray 的所有权/形状/
   stride/可变性、JSON caster、共享指针交换、trampoline、自定义 Buffer
   构造函数、迭代器辅助函数和顶层模块生命周期。门槛：在支持的平台上，
   聚焦测试通过 sanitizer；子进程退出测试没有产生非预期的 nanobind
   泄漏警告。
4. **阶段 3（Linux CMake/XMake 已验证）——机械转换绑定。** 已按注册顺序转换叶子模块并移除 holder 参数，
   更新字段/策略/文档及可空参数，最后将产品入口切换为 `NB_MODULE`。
   更新 constitution 源码检查器；该检查器当前硬编码了 `py::class_`、
   `py::module` 和“pybind module”
   （`scripts/check_constitution_api.py:12-28`、`:44-76`、`:115-124`），
   以及对应的 pybind 专用 fixture
   （`scripts/tests/test_repository_contracts.py:36-76`）。
   当前活动绑定源码中不再残留 pybind11 API/include，CMake/XMake release
   的默认可移植测试集、CUDA 产品构建和代表性样例通过；Windows 和完整
   CUDA pytest 验证仍是阶段门槛的一部分。
5. **阶段 4（Linux 已完成）——包与 wheel 对等。** 通过 CMake 和 XMake 生成一致的 stub，
   构建全新 wheel，安装到干净环境，与阶段 0 的 API/stub 清单对比，并在
   Linux 和 Windows 上针对 CPython 3.10-3.14 运行 wheel smoke/pytest。
   门槛：wheel 内容、导入、`build_info()`、`Engine("none")`、stub 和关闭
   流程全部通过。
6. **阶段 5——运行时/性能验证。** 在受支持的 GPU 上运行带 CUDA 标记的
   测试，并与冻结的 pybind11 基线比较干净构建耗时、扩展体积、导入耗时、
   代表性绑定调用延迟和 ndarray 视图吞吐量。在确定发布策略前，同时测量
   nanobind 默认的体积优化和 `NOMINSIZE` 版本。门槛：没有无法解释的语义
   或显著性能退化；记录所有接受的差异。

迁移期间保留现有启用选项（`UIPC_BUILD_PYBIND`、`--pybind`）作为兼容名称，
但将实现拆成两个独立适配器：`src/nanobind` 是默认实现，`src/pybind` 保留
迁移前的 pybind11 实现。CMake 通过
`UIPC_PYTHON_BINDING=nanobind|pybind11`，XMake 通过
`--python_binding=nanobind|pybind11` 在唯一入口选择一个目录；两边仍输出同名
`pyuipc` 扩展，不能同时加入同一个构建图。

## 必需的回归测试

- **数组：** list/NumPy 输入；接受/拒绝的 dtype 转换；C/F 连续和不连续
  布局；1D/2D/3D 形状失败；空 span；可写与只读视图；`OWNDATA`/base owner；
  删除来源 C++ 包装对象后继续使用视图。现有数组行为在
  `python/tests/test_attribute.py:1-25` 中有所覆盖，类型/值基础行为在
  `python/tests/test_typing.py:1-16` 中有所覆盖，但两者都不是完整的
  所有权测试集。
- **所有权：** `Scene`/geometry/attribute 共享对象、基类/派生类返回值、
  Python 创建的 `PyIEngine`、重复返回时的对象标识，以及析构顺序。现有
  Scene 覆盖从 `python/tests/test_scene.py:1-40` 开始。
- **回调/GIL：** `PyIEngine` 的每个纯虚函数、Buffer resize/view 回调、
  ResidentThread 成功/异常/析构，以及回调仍在队列中时的子进程关闭。
  在 CI 中保持 nanobind 泄漏警告开启；官方指南说明包装对象、函数和共享
  所有权可能形成无法回收的环
  （[引用泄漏指南](https://nanobind.readthedocs.io/en/latest/refleaks.html)）。
- **API/stub：** 模块/类/函数名称、签名、默认值、文档字符串、异常、
  顶层别名、递归原生 stub、`py.typed`，以及真实的静态类型检查样例。
  现有 `test_typing.py` 是运行时值测试，不是 stub/类型检查器验证
  （`python/tests/test_typing.py:1-16`）。
- **Wheel/运行时：** 在全新环境中导入、兼容性元数据、原生库共置、
  `Engine("none")`、在 GPU runner 上加载 CUDA 后端，以及解释器正常退出。
  现有 smoke test 覆盖了前四项中的大部分，但不检查 stub 和退出诊断
  （`scripts/smoke_test_wheel.py:13-73`）。

迁移分支的 CMake 与 XMake 工作流现已显式选择 nanobind。XMake 在 Linux
和 Windows 上使用 CPython 3.12 构建、打包、安装并检查 ABI、递归 stub、
`py.typed` 和 `Engine("none")`；CMake 同样构建并导入 nanobind 产品扩展。
wheel 工作流在该分支运行 Linux/Windows CPython 3.10-3.14 矩阵，但普通
分支 push 只上传 Actions artifact，TestPyPI/PyPI 发布仍仅允许 tag/release。
这些门槛只有在远端 Actions 完成后才能计为跨平台验证通过。

## 主要残余风险

1. **XMake 的 3.0.0 打包仅完成 Linux 验证。** 官方配方滞后已经由仓库内
   精确版本 overlay 处理，产品依赖也已切换；overlay 与官方行为一致地自动
   解析 Python 依赖，因此切换 Python 次版本时必须隔离或清理 ABI 相关缓存。
   Windows 仍是迁移门槛，不能静默回退到 2.12.0。
2. **数组语义可能无声改变。** 字节 stride 与元素 stride、转换默认值、
   只读标志和 owner 对象都需要运行时断言及负向测试。
3. **移除 holder 会改变生命周期语义。** 编译成功不能证明
   `shared_from_this`、对象标识或基类/派生类所有权行为正确。
4. **Python 回调引用环和关闭过程风险很高。** Buffer 和 ResidentThread
   会跨越 C++ 生命周期保留 Python 可调用对象；必须检查泄漏警告，并运行
   子进程退出测试。
5. **CMake 与 XMake 的 stub 布局可能独立退化。** 应将生成文件清单和
   类型检查器输出视为发布产物，而不是偶然生成的构建副产品。
6. **产品验证仍未覆盖 Windows 发布矩阵。** Linux CMake/XMake 产品构建、
   递归 stub、CPython 3.10-3.14 CMake wheel 安装/导入/默认可移植测试、
   CUDA Engine 和代表性模拟样例已通过；Windows、完整 CUDA pytest 和所有
   产品运行时语义仍须通过上述分阶段门槛验证。

## 一手资料

- [nanobind 3.0.0 更新日志](https://github.com/wjakob/nanobind/blob/v3.0.0/docs/changelog.rst)
- [nanobind 的 pybind11 迁移指南](https://nanobind.readthedocs.io/en/latest/porting.html)
- [nanobind CMake API](https://nanobind.readthedocs.io/en/latest/api_cmake.html)
- [nanobind 构建指南](https://nanobind.readthedocs.io/en/latest/building.html)
- [nanobind 打包指南](https://nanobind.readthedocs.io/en/latest/packaging.html)
- [nanobind ndarray 指南](https://nanobind.readthedocs.io/en/latest/ndarray.html)
- [nanobind 所有权指南](https://nanobind.readthedocs.io/en/latest/ownership.html)
- [nanobind 高级所有权指南](https://nanobind.readthedocs.io/en/latest/ownership_adv.html)
- [nanobind Eigen 指南](https://nanobind.readthedocs.io/en/latest/eigen.html)
- [nanobind 类型与 stubgen 指南](https://nanobind.readthedocs.io/en/latest/typing.html)
- [nanobind 引用泄漏指南](https://nanobind.readthedocs.io/en/latest/refleaks.html)
- [nanobind 3.0.0 模块宏源码](https://github.com/wjakob/nanobind/blob/v3.0.0/include/nanobind/nb_defs.h#L201-L236)
- [官方 xmake-repo nanobind 配方](https://github.com/xmake-io/xmake-repo/blob/dev/packages/n/nanobind/xmake.lua)
- [官方 xmake-repo nanobind port](https://github.com/xmake-io/xmake-repo/blob/dev/packages/n/nanobind/port/xmake.lua)
- [XMake 官方包管理指南](https://xmake.io/guide/package-management/using-official-packages.html)
- [pybind11 NumPy 指南](https://pybind11.readthedocs.io/en/stable/advanced/pycpp/numpy.html)
- [pybind11 智能指针指南](https://pybind11.readthedocs.io/en/stable/advanced/smart_ptrs.html)

## 已完成的验证

- 编写报告前检查了 Git 分支、提交和工作区状态。
- 阅读了两套绑定构建描述、两条 stub 生成/打包路径、根目录和开发环境的
  Python 元数据、模块初始化、中央 ndarray 与 JSON 适配器、共享 holder
  声明、trampoline/回调代码、Python 包重新导出、测试清单、仓库契约检查器，
  以及 wheel/XMake 工作流。
- 使用 `find` 和 `rg` 统计了绑定文件和高风险 API token。
- 将每项外部迁移结论与 nanobind 3.0.0 上游文档/源码、涉及当前行为时的
  pybind11 上游文档，以及 XMake 官方仓库/文档进行了交叉核对。
- 从上游 `v3.0.0` 标签归档独立计算 SHA-256，并写入本地配方。
- 清除本次安装的 nanobind 包缓存后，使用工作区最新的
  XMake 3.1.1+HEAD.3ba37a0 和本地 overlay 重新安装并构建
  nanobind 3.0.0；编译命令包含 `NB_BUILD`、`NB_COMPACT_ASSERTIONS` 和
  `-fno-strict-aliasing`，编译了 `nb_datetime.cpp`，未编译
  `nb_backend.cpp`。
- 使用 XMake `python.module` 构建最小 CPython 3.13 扩展，导入后函数调用
  返回预期值；`ldd`/`readelf` 确认扩展不依赖 `libpython`。
- 根工程 `xmake show -l targets` 和 `git diff --check` 通过。
- 完成 Linux XMake 产品绑定切换及 CPython 3.13 debug/release 构建；默认
  可移植测试集为 80 项通过、54 项按标记排除。
- 使用项目 `.venv` 的 CPython 3.12.3 构建并解包 XMake wheel；wheel 可在
  不设置 `LD_LIBRARY_PATH` 的情况下导入，包含 10 个递归 stub，并通过默认
  可移植测试集。
- 使用 CPython 3.12.3 完成 pybind11/nanobind 的 XMake CUDA release 构建、
  CUDA Engine 冒烟，以及 `wrecking_balls` 各 5 次 300 帧对照；按 Newton
  工作量归一后的模拟耗时相同，nanobind 场景创建中位数减少约 8.6 ms。
- 将 overlay 改为官方式自动依赖解析，并按上游要求为 2.10.0 前、2.10.0
  起和 3.x 分别声明 `python >=3.8`、`>=3.9`、`>=3.10`；移走旧 3.0.0 包
  缓存后重新配置，manifest、编译头文件和 wheel ABI 均为 CPython 3.12，
  XMake release 构建、解包导入及 `Engine("none")` 冒烟通过。
- 使用根目录 PEP 517/scikit-build-core 入口生成 Linux CPython 3.10-3.14
  CPU wheel 矩阵；五个 wheel 均在独立环境中安装，ABI、元数据、10 个 stub、
  `py.typed`、`Engine("none")` 和可移植 pytest 通过。
- **未执行** Windows 或完整 CUDA pytest 矩阵验证。
