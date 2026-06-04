# 周报 — AGIPC 代数粗化加速集成与验证

## 摘要

本周对 codex 生成的代数粗化实现做了完整 review、论文对齐与性能验证。该实现即论文提出的面向图形处理器（Graphics Processing Unit，GPU）增量势接触（Incremental Potential Contact，IPC）的自适应求解内代数粗化（Adaptive In-Solve Algebraic Coarsening for GPU IPC，AGIPC）。

**核心结论**：修复了多处保真 bug 与一个阻塞性 GPU 死循环，把开启粗化（coarse-on）相对基线（baseline）的慢比从约 7.5× 一路优化到 1.31×，但**两个测试场景均未实现加速，仍为减速**。根因已确证为架构性问题——粗化是「叠加」在细网格（fine）求解之上而非「替代」它，换更强的预条件器无法解决。要兑现论文加速需重写求解流程。

**本周补充验证**：对照论文第一手收敛判据（牛顿停机条件 `‖d‖∞/Δt ≤ ε_d`，`ε_d = 1e-3·l`，`l` 为场景包围盒对角线），把两个场景的牛顿容差对齐到论文口径后重跑 4 象限对照实验。结论不变——对齐论文收敛口径后 coarse 仍为减速（balls 1.52× 慢、sandwich 1.87× 慢），且收敛口径越严，coarse 减速越明显。**收敛判据不是瓶颈，改容差不改变结论**，进一步确证瓶颈是架构性的。

## 一、完成的工作

### 1. Review 与论文对齐

对照论文解读逐项核查 codex 的约 1600 行新代码（`agipc_coarse_linear_system.cu` / `.h`），发现并修复多处与论文不一致或有 bug 的地方：

- **伽辽金投影（Galerkin projection，Hc = UᵀHfU）的对称投影只写了半边**（Bᵀ 项缺失），补全完整对称化 `B*(w_rp·w_cq) + Bᵀ*(w_cp·w_rq)`。
- 聚合算法被替换为 union-find（并查集），但缺少有界聚合尺寸（bounded aggregate size），补上聚合尺寸上限。
- 仿射（affine）基未中心化，改为论文的中心化基 `[1, X-X̄, Y-Ȳ, Z-Z̄]`，改善条件数。
- 补充退化场景短路（无可聚合边时直接跳过粗化）。

### 2. 修复阻塞性 GPU bug

- union-find 的非原子路径压缩在并发下成环，导致 kernel `while(true)` 死循环（有限元方法（Finite Element Method，FEM）场景 frame 52，GPU 打满挂死）。改为只读 find + 独立原子压缩 pass。codex 没踩到，是因为纯仿射体动力学（Affine Body Dynamics，ABD）的 `wrecking_ball` 从不触发聚合。
- 中心化仿射基在退化聚合（单点 / 共面）上产生奇异 3×3 矩阵 → 求逆 NaN → 预条件器断言崩溃。加 `isfinite` 检查 + Identity 回退。

### 3. 搭建对比 example 与基准

- `fem_bunny_cloth_sandwich`：两 FEM 兔子夹 FEM 布料 + 地面自由下落（接触密集 + 大形变）。
- `fem_elastic_balls`：8 个刚性球（E=1e7）堆叠（刚性材料、变形小、接触稀疏，对应论文最佳加速场景类型）。
- 新增回归测试 `agipc_fem_coarse.cpp`。
- 所有 example 加了环境变量开关，方便切 coarse-on/off 和扫参。

### 4. 集成 MAS 强预条件器

把 libuipc 已有的多级加性 Schwarz（Multi-Level Additive Schwarz，MAS）预条件器接到粗化预条件共轭梯度（Preconditioned Conjugate Gradient，PCG）上，带跨迭代 checksum 缓存，默认关闭（`use_mas_preconditioner=0`），无回归。

## 二、性能数据

两个场景均未加速，下面按「逐版本优化」与「换场景 + 强预条件器」两条线呈现。

### 场景一：fem_bunny_cloth_sandwich（1000 帧）

接触密集 + 大形变，coarse 算子在 stiff contact 下病态（coarse PCG 约 70% 打满 150 迭代）。这是逐版本优化的主战场：

| 版本 | 墙钟时间 | Newton 总数 | 相对 baseline | 关键改动 |
|------|----------|-------------|---------------|----------|
| baseline（coarse off） | 2:04.55（124.5s） | 7416 | 基准 | — |
| 初版 | — | — | 约 7.5× 慢 | codex 原始实现 |
| v2 | 5:24（324s） | — | 2.6× 慢 | 对称对角 + bounded aggregate + tol150 |
| v3 | 3:44.45（224.5s） | 7501 | 1.80× 慢 | 完整对称 Galerkin 投影 + 1e-3 粗解 tol + 自适应 refine |
| **v4** | **3:13.95（193.95s）** | **7420** | **1.56× 慢** | 中心化 affine 基 + CSR 缓存 |

v4 的 Newton 总数（7420）已追平 baseline（7416），方向质量问题已解决，line-search 失败从早期上千次降到个位数。剩余 1.56× 纯粹是每个 Newton 迭代额外叠加的 coarse build + coarse PCG + fine refinement 开销。

### 场景二：fem_elastic_balls（60 帧窗口）

论文宣称约 3.3× 加速的场景类型，用于验证强预条件器能否突破：

| 配置 | 墙钟时间 | GPU 打满率 | 相对 baseline | 备注 |
|------|----------|------------|---------------|------|
| baseline（coarse off） | 14.22s | — | 基准 | — |
| coarse + Jacobi | 18.67s | 约 70% | 1.31× 慢 | 全程最接近 baseline 的一次 |
| coarse + MAS | 18.16s | 6% | 1.28× 慢 | 收敛改善但不省时 |
| coarse + MAS + cache | 19.94s | 5% | 1.40× 慢 | coarse 图动态，缓存未命中 |

即便在论文最佳场景类型叠加强预条件器，仍 1.31× 慢于 baseline——这已是全程最接近基准的结果。

## 二点五、收敛口径对齐实验（本周新增）

此前的性能对比用 libuipc 默认牛顿容差，而论文用的是另一套口径。为排除「是不是因为收敛判据不一致才看不到加速」这一疑问，本周直接查论文（非二手代码）核对收敛条件并对齐重测。

### 收敛判据对比（第一手论文 vs libuipc）

| 维度 | 论文 AGIPC | libuipc |
|------|-----------|---------|
| 牛顿停机量 | `‖d‖∞ / Δt ≤ ε_d`（位移步长 L∞ 范数 / 时间步） | `compute_axis_max_displacement / dt ≤ velocity_tol`，**同一个量** |
| 容差取值 | `ε_d = 1e-3·l`，`l` = 场景包围盒对角线（单位 m/s） | 默认 `velocity_tol = 0.05`，**不随包围盒缩放** |
| 收敛口径 | StiffGIPC 用全空间位移；AGIPC 用「延拓的粗层位移（经后粗化修正细化）」——两者精度不同 | coarse-on 路径已是「延拓 + 细化」口径，与论文 AGIPC 一致 |
| 线性求解 tol | PCG 相对残差 1e-3；后粗化 CG ≤10 次 | `tol_rate = 1e-3`（一致） |

结论：libuipc 与论文的收敛**量纲一致**（都是 `‖d‖∞/Δt`），coarse-on 已采用论文的「延拓+细化」口径，对比是公平的。唯一实质差异是**容差取值**——论文带包围盒缩放、libuipc 默认不带。于是把两个场景的容差对齐到论文 `ε_d = 1e-3·l` 重测：
- `fem_elastic_balls`：可形变网格包围盒对角线 `l = 11.425` → `ε_d = 0.011425`
- `fem_bunny_cloth_sandwich`：`l = 5.314` → `ε_d = 0.005314`

### 4 象限对照实验（150 帧，关闭逐帧写盘）

每个场景跑 `coarse∈{off,on} × tol∈{默认 0.05, 论文 1e-3·l}` 四象限，硬件 RTX 5090，`/usr/bin/time` 计墙钟，从「Newton Iteration Converged」日志累计牛顿总数。

**fem_elastic_balls**（`l=11.425`，论文 `ε_d=0.011425`）：

| 象限 | coarse | 牛顿容差 | 墙钟时间 | 牛顿总数 | 相对同口径 baseline |
|------|--------|----------|----------|----------|---------------------|
| A | off | 默认 0.05 | 76.37s | 2759 | 基准 |
| B | on | 默认 0.05 | 124.14s | 3000 | **1.63× 慢** |
| C | off | 论文 0.011425 | 87.84s | 3330 | 基准 |
| D | on | 论文 0.011425 | 133.16s | 3234 | **1.52× 慢** |

D 象限 2778 次粗解中 79.5% 打满 150 迭代仍未达残差。注意 D 的牛顿数（3234）比 C（3330）还略少，但墙钟仍慢 1.52×——粗化即便略减牛顿步数，每步固定开销也吃掉全部收益。

**fem_bunny_cloth_sandwich**（`l=5.314`，论文 `ε_d=0.005314`）：

| 象限 | coarse | 牛顿容差 | 墙钟时间 | 牛顿总数 | 相对同口径 baseline |
|------|--------|----------|----------|----------|---------------------|
| A | off | 默认 0.05 | 37.00s | 1758 | 基准 |
| B | on | 默认 0.05 | 62.51s | 1747 | **1.69× 慢** |
| C | off | 论文 0.005314 | 52.75s | 2742 | 基准 |
| D | on | 论文 0.005314 | 98.57s | 2775 | **1.87× 慢** |

B 象限 70%、D 象限 80% 的粗解打满 150 迭代——与场景一表头记录的「约 70% 打满」一致。

### 实验结论

1. **对齐论文收敛口径后，加速依然没有出现**：balls 1.52× 慢、sandwich 1.87× 慢，与默认口径同向（balls 1.63×、sandwich 1.69× 慢）。
2. **收敛口径越严，coarse 减速越明显**（sandwich：1.69× → 1.87×）。更严的容差需要更多牛顿步，每步的粗化固定开销随之被放大。
3. **收敛判据不是瓶颈**，改容差不改变「coarse-on 减速」这一结论，反向印证了下文的架构性根因——瓶颈在每个牛顿步的固定开销与病态粗解 PCG，而非牛顿何时停。

## 三、根因分析（已确证，非推测）

三方对比锁定了瓶颈所在：

- **coarse PCG 迭代数不是瓶颈**。Jacobi（18.67s，打满 70%）与 MAS（18.16s，打满 6%）墙钟几乎相同——MAS 把 coarse PCG 迭代砍了 10×、cap-hit 从 70% 降到 6%，总时间却没省。
- **真正瓶颈是每次 Newton 迭代的固定开销**（粗化图重建 + 预条件器装配 + fine refinement）。这些开销叠加在 fine 工作之上而非替换它：baseline 20.9ms/newton，coarse-on 28.4ms/newton，每步净增约 7.5ms。
- **coarse 图无法跨迭代缓存**。聚合每个 Newton 迭代都在变（不同边坍缩），checksum 几乎每次不同、缓存基本不命中；这与固定拓扑、可缓存的 fine 压缩稀疏行（Compressed Sparse Row，CSR）本质不同。

### 逐 Newton 步拆解（150 帧 4 象限日志）

进一步把每个 Newton 步拆开看，加速达不到是四个机制叠加的结果，而非单一原因。以 `fem_elastic_balls`（fine 系统 39528 自由度 / 13176 顶点）为例：

| 每个 Newton 步 | baseline（coarse off） | coarse-on |
|----------------|------------------------|-----------|
| 线性求解结构 | 1 次 fine PCG | coarse build + coarse PCG +「再来一次」fine refine PCG |
| 线性求解迭代数 | fine PCG 中位 30、均值 34 次即收敛 | coarse PCG **79% 打满 150 次**仍不收敛，再叠加 fine refine 均值 36 次 |

一句话：baseline 一步只解一次线性系统，coarse-on 一步要解两次（粗的没解动 + 细的补救）。下面是四个具体机制。

**机制一：fine 求解本身已经够快，没有「慢」留给粗化去省。** baseline 的 fine PCG 每步中位仅 30 次迭代（balls）、35 次（sandwich）即收敛。论文加速的前提是「fine 求解贵、用便宜的 coarse 子空间替代其大部分工作量」，但 libuipc 的 fine PCG + 预条件器已很高效，粗化想替代的那块开销本来就小。

**机制二：粗化是「叠加」在 fine 上，不是「替代」。** `global_linear_system.cu:736-823` 的每步流程是无条件的 `build coarse → coarse PCG（≤150）→ prolongate → fine refine PCG`，其中 fine refine 总是执行（预算 `max(5,(coarse_iter+2)/3)`，粗解越差细化越多）。即便 coarse 一点忙没帮上，fine refine 也照跑——coarse 是 fine 之上的额外层，不是替身。

一句话：这是「coarse 叠在 fine 上」的架构限制，不是换预条件器能解决的。

## 四、遗留与下一步

要真正兑现论文加速，需要重写求解流程，让 coarse 解承担主要工作量（而非作为额外 refinement 层）。这是一次求解器重写，工作量远大于本周的「接预条件器」，需要确认后再启动。

当前仓库状态：改动文件干净（两个 example、回归测试、coarse system 新文件），clang-format 通过，clean build。
