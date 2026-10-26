# GradientOnly 编译分支与 shared EVD 联合优化

这次修改基于 `ffd572193cfbe2eb02e7c44a3f9890e14cd619d7`。下文的“优化前”指这个版本的相应 kernel，已经包含原始优化 patch；比较两侧的其他优化保持一致。

目标表中的 14 处 6×6、9×9、12×12 投影已接入 shared EVD。8 个运行时 `gradient_only` kernel 拆成 `<true>`、`<false>` 实例；转动、滑动关节原本已有独立梯度/Hessian kernel，保持原来的拆分。3×3 和摩擦专用 2×2 保留原实现。

梯度实例通过 `if constexpr(!GradientOnly)` 完全移除 Hessian 声明、生成、投影、写入和 shared 工作区，host 继续按 runtime bool 选择一次 launch。空输入、空 H view、稀疏索引和塑性历史语义保持不变。CPU ARAP 参考计算需要的三个 dependent `segment` 调用补了 `template` 限定符，公式未改。

完整计算按原来的投影空间处理。Spring、杆、点点/点边缝合直接用特征分解装配；ARAP、点三角形缝合和关节保留完整矩阵重建及原来的 Jacobian 映射。应变塑性弯曲具有四顶点共同平移不变性，使用已有 `12→9` 约化，再恢复 12 维输出；测试仍以完整 12×12 CPU 投影为参考。

## CUDA-event 对照

RTX 5090 / sm_120 / driver 590.48.01 / CUDA 13.4.3 / FP64 Release。每种模型使用 65,537 个确定性、互相独立的图元。计时包含实际生产 kernel 的导数生成、PSD 投影、装配和输出写入，排除分配、传输和 CPU 校验。每项先持续预热至少 250 ms；每次样本平均 20 次 launch，进程内 7 次样本，优化前/后交替运行 5 组独立进程，共 1,120 条正式样本。

CTA 调优与正式测量分开，两侧各做 2 轮。梯度候选为默认、32、64、128、256；优化前完整计算候选为默认、8、16、32、64、128，shared 完整计算候选按固定 workspace pitch 限制为默认、8、16。默认结果距离最佳不超过 3% 时保留默认。正式优化版使用代码中的生产默认值，优化前使用其测得配置。

| 模型 | GradientOnly：前→后 ms/批 | 前/后 | 完整计算：前→后 ms/批 | 前/后 |
|---|---:|---:|---:|---:|
| 固定关节 | 0.028710 → 0.024642 | 1.165× | 2.432608 → 2.142029 | 1.136× |
| ARAP | 0.183843 → 0.145376 | 1.265× | 1.615739 → 1.330461 | 1.214× |
| Hookean 弹簧 | 0.045451 → 0.041013 | 1.108× | 0.464163 → 0.413005 | 1.124× |
| Kirchhoff 杆弯曲 | 0.160854 → 0.134674 | 1.194× | 1.650387 → 1.303584 | 1.266× |
| 应变塑性壳弯曲 | 0.034864 → 0.026704 | 1.306× | 3.514257 → 2.907942 | 1.209× |
| 点点缝合 | 0.036054 → 0.032872 | 1.097× | 0.486206 → 0.474250 | 1.025× |
| 点边缝合 | 0.064248 → 0.061419 | 1.046× | 1.233270 → 1.052205 | 1.172× |
| 点三角形缝合 | 0.080835 → 0.046706 | 1.731× | 2.022608 → 1.669456 | 1.212× |

表中取五个进程中位数的中位数。点点缝合完整计算的变化只有约 2.5%，接近进程间波动，不视为可靠的大幅收益。其余结果也只覆盖这些输入和设备；不代表整帧 FPS。

| 模型 / 模式 | 优化前进程中位数范围 ms | 优化后进程中位数范围 ms |
|---|---:|---:|
| 固定关节 / G | 0.028706–0.028712 | 0.024626–0.024922 |
| 固定关节 / G+H | 2.419579–2.457447 | 2.105787–2.154098 |
| ARAP / G | 0.182259–0.190424 | 0.142789–0.149574 |
| ARAP / G+H | 1.608560–1.632811 | 1.308934–1.343371 |
| Hookean 弹簧 / G | 0.045282–0.045550 | 0.041003–0.041042 |
| Hookean 弹簧 / G+H | 0.460555–0.469026 | 0.408998–0.419730 |
| Kirchhoff 杆弯曲 / G | 0.160723–0.162523 | 0.131894–0.139274 |
| Kirchhoff 杆弯曲 / G+H | 1.620088–1.662237 | 1.292021–1.308112 |
| 应变塑性壳弯曲 / G | 0.034835–0.034872 | 0.026702–0.026786 |
| 应变塑性壳弯曲 / G+H | 3.509725–3.598817 | 2.897680–2.942510 |
| 点点缝合 / G | 0.035734–0.036170 | 0.032797–0.033339 |
| 点点缝合 / G+H | 0.477818–0.490160 | 0.463736–0.477283 |
| 点边缝合 / G | 0.063915–0.064370 | 0.061406–0.061461 |
| 点边缝合 / G+H | 1.223096–1.244998 | 1.038221–1.080570 |
| 点三角形缝合 / G | 0.080834–0.081474 | 0.046205–0.046949 |
| 点三角形缝合 / G+H | 2.002192–2.034434 | 1.648031–1.689491 |

正式运行采样 GPU 状态 139 次，未发现其他 CUDA compute 进程；桌面图形负载仍可能存在。Nsight 与 sanitizer 的结果均未混入计时。首次直接套用 shared 的 12×12 弯曲版本曾回退，已采用平移约化；gradient-only 的 occupancy 默认 CTA 也曾回退，改为逐实例实测配置。上述探索记录保留在本地验证目录。

## 编译资源与执行证据

| 模型 | 原 kernel 寄存器/线程 | 梯度实例寄存器/线程 | 原 STACK B | 梯度 STACK B | 梯度 CTA |
|---|---:|---:|---:|---:|---:|
| 固定关节 | 255 | 124 | 16488 | 128 | 128 |
| ARAP | 255 | 148 | 7064 | 1176 | 64 |
| Hookean 弹簧 | 166 | 52 | 560 | 128 | 32 |
| Kirchhoff 杆弯曲 | 255 | 90 | 2784 | 128 | 32 |
| 应变塑性壳弯曲 | 255 | 94 | 7088 | 128 | 32 |
| 点点缝合 | 164 | 52 | 560 | 128 | 32 |
| 点边缝合 | 255 | 96 | 2512 | 128 | 32 |
| 点三角形缝合 | 255 | 94 | 7120 | 1152 | 64 |

8 个梯度实例的 shared memory 均为 0。独立 Nsight Systems trace 记录 40,776 次 kernel 执行，确认 16 个新实例及其 CTA，完整计算使用 32 线程，约化弯曲使用 16。STACK 是编译资源，不是实测 spill 流量；部分完整实例的 STACK 反而增加。未采集硬件 counter，不能将收益归因于单一硬件瓶颈。

六个独立的转动/滑动关节 Hessian kernel 已完成资源审计和真实仿真验证，本次没有它们的独立 CUDA-event 吞吐测量；上表的固定关节是另一个合并 kernel。

## 验证

- `xmake build -j4 backend_cuda`、`xmake build -j4 sim_case` 通过。
- 默认 CUDA 套件 74 个用例、28,538,379 条断言，0 failure、0 error。
- 新的生产 launch 测试覆盖 8 类本构，数量 0、1、15、16、17、31、32、33、257、1025，共 1,226,926 条断言；两侧都通过。
- CPU Eigen 完整 PSD 投影参考、梯度对照、稀疏索引、非零输出偏移、首尾保护区、空 H、H canary、塑性历史检查。误差界为 `2e-11 + 2e-10 * scale`。
- CUDA memcheck：0 errors；racecheck：0 errors、0 warnings。
- 15 个实际仿真场景，两侧共 30 次运行全部通过；包括 ARAP、弹簧、杆、三种缝合、固定/转动/滑动关节、驱动与限位、应变塑性残余角和恢复测试。场景输出隔离并恢复原有输出。

这些仿真是回归检查，每侧只运行一次，wall time 包含初始化和场景输出；没有据此给出整帧性能结论。CMake、其他 GPU 与其他 CUDA 版本未验证。

## 复现

Linux 下使用已经配置的 Release/CUDA 构建：

```sh
xmake build -j4 backend_cuda
xmake run -w "$PWD" backend_cuda '[remaining_material_launch]' --log-level warn
```

优化前需要同 ABI 的 launch shim。以下命令在单独的干净 checkout 中只导出已有 kernel 的 launch 接口及 CTA 控制，保持原 device body 和原本构 dispatch：

```sh
git worktree add --detach ../libuipc-evd-reference ffd572193cfbe2eb02e7c44a3f9890e14cd619d7
python3 apps/tests/backends/cuda/benchmark_constitution_launches.py prepare-baseline ../libuipc-evd-reference
```

随后在该 checkout 配置相同 CUDA、Eigen、Release 选项并执行 `xmake build -j4 cuda`，取生成的 `libuipc_backend_cuda.so`。对比时使用优化后的测试二进制及匹配的其他动态库：

```sh
python3 apps/tests/backends/cuda/benchmark_constitution_launches.py compare \
  --baseline /absolute/path/to/reference/libuipc_backend_cuda.so \
  --baseline-blocks apps/tests/backends/cuda/remaining_material_baseline_blocks.json \
  --runs 5 --output /absolute/path/to/new-results
```

需要时用 `--binary` 指定测试程序，`--library-path` 补充 CUDA/dependency 库目录。比较脚本逐次保存 CUDA-event 样本、JUnit、GPU 状态、程序/库 SHA256，并核对实际加载的 backend 库。默认 optimized 使用生产 CTA；内部调优可用 `UIPC_REMAINING_TUNE_MODE`、`UIPC_REMAINING_TUNE_CTA`，完整 shared launch 不可超过 workspace lane pitch。

结构化结果见 [remaining_material_results.json](remaining_material_results.json)。本机完整日志、Nsight、sanitizer、所有场景输出及探索记录保留在 `build/verification/gradient-shared-evd-20261026/`。

优化后库 SHA256：`7fa52800b63e178cbf898dc54464a76d7e0a7403b71944255749169ac3a1a74c`。测试程序 SHA256：`74cb66c9b3241a57c32652a4f1a4921f817d3a05050120a5f20a8a7886116aa8`。
