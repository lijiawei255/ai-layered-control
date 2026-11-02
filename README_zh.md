# ai-layered-control

[English](README.md) | [简体中文](README_zh.md)

[![Release v1.1.0](https://img.shields.io/badge/Release-v1.1.0-blue.svg)](https://github.com/lijiawei255/ai-layered-control/releases)
[![ANSI C99 零堆核心](https://img.shields.io/badge/Core-ANSI%20C99%20%28Zero--Heap%29-00599C.svg)](c99_layered_core/)
[![Python 与 MATLAB/Simulink 基准](https://img.shields.io/badge/Bench-Python%20%7C%20MATLAB%2FSimulink-2ea44f.svg)](python_bench/)
[![MIT License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**面向闭环 AI 控制的参考架构：慢速粗粒度 AI 分类器只决定机器处于*哪一种*控制模态；快速、零依赖的确定性核心决定*如何*驱动执行器。分层路径中，任何端到端 `u = pi(x)` 模型都不会进入 1 kHz 指令通路。**

**核心结论（TL;DR）**

* **几十次标定，而不是上千次。** 10 Hz 粗分类器把 4 维无量纲宏观特征映射到 3 档阻尼策略；在 **N = 40** 个标定样本时达到 **100% 部署质量**。同分布的连续回归需要约 120–500 个名义机组样本才能达到同样的分布内质量。
* **+35% 整机漂移，零重训。** 四阶段基准中，分层支路 S2 RMSE 为 **0.0057 rad**，端到端支路为 **0.1585 rad**。仅在名义数据上训练的连续策略，在 +35% 漂移机组上即使 N=2000 仍是 **0%** 达标；用漂移数据训练的 oracle 在 N=120 恢复到 **100%**。
* **未见 OOD 故障显式弃权。** 分层支路 S4 安全包络越界 **0** 个采样点，端到端支路 **2574** 个；OOD 否决在故障后 **2.0 ms** 锁存。只保留 42 Hz 谐振或只保留传感器尖峰时同样 **2.0 ms** 触发；S1–S3 在 20 倍宽带测量噪声下 OOD 否决采样点 **0**。
* **模态必要性消融。** 动态选择的 S1–S3 RMSE 为 **0.00710**，固定 R1/R2/R3 分别为 **0.01201 / 0.00935 / 0.05501**；40 s 综合为 **0.01721** 对 **0.01798 / 0.01777 / 0.05069**。实际 10 Hz 分类器为 **0.00700 / 0.01630**，与 stage-forced 上界相差 2% 以内（0.00700 对 0.00710，略优）。
* **真正训练出的基线。** Branch 2 是由名义机组教师 rollout 闭式拟合的 RBF 逆动力学模型 + 固定 PD 稳定器；Branch 3 的 3×4 原型与置信度阈值由 `calibrate_coarse_classifier.m` 从闭环探针生成，没有硬编码原型表。

> **适用范围。** 本仓库是合成非线性被控对象上的方法论参考实现；完整边界见文末唯一的**适用范围与免责声明**。文中所有数字均由 §6 的验收命令产生。

---

## 1. 架构

```text
+------------------------------ 慢环：10 Hz --------------------------------+
|  4 维无量纲宏观特征 -> 3 类粗分类器 -> regime ∈ {1,2,3}                  |
|  (rms 误差, rms 指令, max(慢残差, 高频残差), rms 速度)                    |
|  margin x proximity 置信度 -- 在类别歧义与远离原型时同时下降              |
|  独立名义观测器 -> 物理残差探针                                          |
+---------------------------------------------------------------------------+
               |  邮箱：{regime, confidence, frame age}
               v
+------------------------------ 快环：1 kHz --------------------------------+
|  0  输入防火墙   NaN/Inf/越界帧 -> 保持执行器，锁存审计位                 |
|  1  安全包络     |x| > 90% 限制，或 |x| + 60 ms*|xdot| > 限制 -> SAFE        |
|  2  OOD 残差     尖峰通道 或 窄带高频通道 -> 显式弃权                     |
|  3  置信度       低于标定阈值或 AI 主动弃权 -> 显式弃权                   |
|  4  心跳         慢环帧过期 -> GATE_HOLD_LATENCY，指令与状态冻结          |
|  5  驻留         正常模态切换至少 150 ms                                  |
|  6  冷却         500 ms 连续干净采样后才重新武装 AI                       |
|  7  控制器 bank  4 × PID + 每档确定性前馈缩放，逐控制器抗饱和             |
|  8  钳位         无扰切换，|u| <= u_max，|du| <= slew_rate*dt              |
|  9  审计         5 位粘滞故障位：为什么进入安全态，无需 AI 自省           |
+---------------------------------------------------------------------------+
               |
               v
      执行器  +  u_ff = 由调试模型给出的白盒被控对象逆模型
```

零堆、无 `math.h`、无递归、无 OS 调用；固定直线代码，O(1) 最坏执行时间。`LayeredState_t` 为 88 B，`LayeredConfig_t` 为 132 B。AI 可以慢、可以抖动、可以短暂缺席；内环从不等待它。

v1.1.0 的关键细节是**正交残差探针**：

* **尖峰通道**：位置测量的 median-5 偏差，按 `res_fast_scale` 归一化；
* **高频谐振通道**：以该域 OOD 模态频率（工业 42 Hz）为中心的 2 阶窄带谐振器，8 Hz 带宽、4 ms RMS 能量、连续 2 拍确认；
* **慢通道**：LP(innovation)，只作为分类器特征。

门控接收 `max(res_fast, res_hf_gated)`。因此 20 倍宽带测量噪声（S3）不会触发 OOD，而 42 Hz 结构谐振会。

### 工程系统论的职责解耦

本架构不是去卷更大的单点模型，而是从工程系统论出发做结构层分工：**10 Hz 慢环 AI** 发挥其“模式识别与定性语义归类”的长处，仅在无量纲空间做粗粒度工况选择题，规避端到端 AI 在高频连续控制中易过拟合旧机台逆动力学流形、推理抖动及 OOD 强行乱输出的短板；**1 kHz 快环确定性 C99 核心**发挥其“高频连续调节、定量漂移吸收与硬实时安全兜底”的长处，以常数时间 O(1) 门控与极简物理残差探针锁死安全边界。

### 为什么采用合成同构基准？

作为通用架构参考库，本仓库蓄意采用**参数化合成非线性被控对象**作为受控压力测试（Controlled Stress-Testing）。在真实工业产线或临床设备上，无法安全、可重复地注入极端未见共振故障（Stage 4）来检验端到端模型的越界破坏力，也无法精确锁定 `+35%` 参数漂移或精确扫描 `N = 15..2000` 样本复杂度来做严格的反事实消融（Ablation）。合成基准让这些反事实精确、可复现；三个域预设是同构映射，不是硬件验证结论。

## 2. 主指标

工业参考预设，40 s，4 阶段，1 kHz 内环；三条支路使用相同对象、参考、噪声实现与故障时序：

| 指标 | Branch 1: 固定 PID | Branch 2: 端到端 AI | Branch 3: 分层 |
| :--- | ---: | ---: | ---: |
| RMSE S1 快速跟踪 [rad] | 0.0566 | 0.0397 | **0.0077** |
| RMSE S2 +35% 负载/Stribeck [rad] | 0.1790 | 0.1585 | **0.0057** |
| RMSE S3 接触/高频噪声/边缘时延 [rad] | 0.1921 | 0.1670 | **0.0074** |
| RMSE S4 未见 OOD [rad] | 0.1693 | 0.8308 | **0.0303** |
| 安全包络越界采样点 S4 [ms] | 0 | 2574 | **0** |
| OOD 否决延迟 | - | - | **2.0 ms** |
| 标定样本数 N（分类器 / 连续策略） | - | ~500 | **≤ 40** |
| 算力 (MAC/s) | - | 2.40e9 | **2.41e4** |
| 内存占用 | - | 5.60e6 B | **2.21e3 B** |

`Safety Violation Ticks` 是 `|x| > x_limit` 的 1 kHz 采样点数；在 `dt = 1 ms` 时数值等于毫秒数。`Control Effort TV(u) = Σ|u_k − u_{k−1}|` 是控制指令总变差，反映高频抖振/磨损。`N` 是用于拟合分类器或连续策略的标定样本数。

**Control Effort TV(u)，S1–S3：** 分层 **0.41e3 N·m**，固定 PID **1.24e3**，端到端 **1.33e3**。S4 中分层安全回退调节器为把轴保持在包络内，TV 为 **1.97e3 N·m**；端到端 S4 TV 更低，但有 **2574** 个越界采样点。TV 必须与安全列一起读。

## 3. 模态必要性消融

`run_all_benchmarks` 额外运行三次固定 bank 的 40 s 仿真和一个 stage-forced oracle（S1→R1、S2→R2、S3→R3、S4→安全回退），并打印实际 10 Hz 分类器：

| 控制器 | S1 | S2 | S3 | S4 | S1–S3 | 40 s |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: |
| Stage-forced oracle | 0.0083 | 0.0056 | 0.0071 | 0.0322 | **0.00710** | **0.01721** |
| 固定 R1 | 0.0083 | 0.0077 | 0.0175 | 0.0293 | 0.01201 | 0.01798 |
| 固定 R2 | 0.0095 | 0.0058 | 0.0117 | 0.0316 | 0.00935 | 0.01777 |
| 固定 R3 | 0.0938 | 0.0150 | 0.0072 | 0.0346 | 0.05501 | 0.05069 |
| 实际 10 Hz 分类器 | 0.0077 | 0.0057 | 0.0074 | 0.0303 | 0.00700 | 0.01630 |

oracle 是理想分类器的上界；实际分类器 S1–S3 RMSE 与上界相差 2% 以内（0.00700 对 0.00710，略优）。stage-forced 动态满足 `<0.8 ×` 固定 bank 判据（`0.00710 / 0.00935 = 0.759`）与 40 s 判据。

## 4. 成对样本复杂度（图 2 左子图，Panel A）

两个学习器**严格共用同一批名义机组（drift = 0）的 N 个成对样本**；部署在 0% 漂移与 +35% 漂移的同一机组上。oracle 是同一 495 项连续头改用 +35% 漂移数据训练：

| N | 分层 @ +35% | 连续 nominal-only @ 0% | 连续 nominal-only @ +35% | Oracle @ +35% |
| ---: | ---: | ---: | ---: | ---: |
| 15 | **100.0%** | 78.1% | 3.1% | 59.4% |
| 40 | **100.0%** | 84.4% | 0.0% | 71.9% |
| 120 | **100.0%** | 96.9% | 3.1% | **100.0%** |
| 500 | **100.0%** | 100.0% | 6.2% | 100.0% |
| 2000 | **100.0%** | 100.0% | **0.0%** | 100.0% |

形状本身就是结论：粗分类器在它从未见过的漂移机组上 N=40 即饱和；只在名义数据上训练的连续回归器，即使 N=2000 也无法在漂移机组上恢复；oracle 在 N=120 恢复，说明差距来自**数据覆盖**，不是模型容量。

## 5. 三张图

![四阶段对比：安全包络内的状态跟踪、执行器指令、残差与置信度、regime/gate 时序](assets/benchmark_4stages_showdown.png)

**图 1 — 40 s，四阶段，三条支路。** (a) 状态与硬安全包络；(b) 执行器指令；(c) 尖峰/高频残差通道与 AI 置信度；(d) 有效 regime 与 gate 时序。S4 进入 `REGIME_SAFE_FALLBACK (4)` 是一个有名字的决策，不是静默钳位。

![成对样本复杂度、零重训漂移鲁棒性、算力与内存](assets/efficiency_and_generalization.png)

**图 2 — 成对数据效率。** (a) 分层分类器 vs 仅名义数据连续头 vs +35% 训练 oracle；(b) AI 冻结下的全闭环漂移扫描；(c) 算力、内存与推理频率。

![工业伺服、边缘 VLA 与闭环神经调控的跨域轨迹](assets/domain_profiles_comparison.png)

**图 3 — 同一套架构用于三个域。** 只改变物理常数、阶段名称与包络单位；C99 核心与门控语义不变。

## 6. 运行与验收

### MATLAB 全量验收

```powershell
matlab -batch "cd('matlab_simulink'); run_all_benchmarks"
```

判据：

| 信号 | 期望 |
| :--- | :--- |
| `[1]`–`[5]` | 全部 `PASS` |
| 末行 | `All five engineering checks passed.` |
| 图 | `assets/` 下三张 240 dpi 图重写 |
| 模型 | 有 Simulink 且模型无未保存编辑时，重建 `matlab_simulink/ai_layered_control_bench.slx` |
| 退出码 | `0` |

没有 Simulink 时，数值基准与五项检查仍全部运行，`.slx` 步骤打印 skip。

### C99 核心回归测试

```powershell
cd c99_layered_core
gcc -std=c99 -pedantic -Wall -Wextra -O2 -o test_gatekeeper_standalone.exe layered_gatekeeper.c test_gatekeeper_standalone.c
.\test_gatekeeper_standalone.exe
$LASTEXITCODE
```

Linux / macOS（bash / zsh）：

```bash
cd c99_layered_core
cc -std=c99 -pedantic -Wall -Wextra -O2 -o test_gatekeeper_standalone layered_gatekeeper.c test_gatekeeper_standalone.c
./test_gatekeeper_standalone
echo $?
```

期望：`SUMMARY: n_pass=116, n_fail=0`，退出码 0，无编译诊断。

### C99 ↔ MATLAB 闭环等价

```powershell
matlab -batch "cd('matlab_simulink'); verify_core_equivalence"
```

期望：`PASS: 6000 closed-loop ticks; gate/regime/fault exact; max|du|=4.82e-07, max|du_target|=4.82e-07, max|dblend|=9.5e-08`（容差 `1e-4`）。无编译器时 `SKIP`。

### C99 核心 3 步嵌入式移植

1. 把 `c99_layered_core/layered_gatekeeper.c` 与 `.h` 拷贝到目标工程，按 ANSI C99（`-std=c99`）编译；无堆、无 `libm`、无递归。
2. 用目标机的单位、限幅与 bank 增益填好 `LayeredConfig_t`，启动时调用一次 `Layered_Init(&state, &config)`。
3. 在 1 kHz（1 ms）中断里填 `LayeredInput_t`，调用 `Layered_Step(&state, &in, &out)`；把 `out.u` 写执行器，并持久化/记录 `out.gate`、`out.regime` 与粘滞故障位。

### 纯 Python 快速入口（无 MATLAB）

```powershell
python -m pip install -r python_bench/requirements.txt
python python_bench/run_benchmark.py
```

入口优先用 `ctypes` 编译 `c99_layered_core/layered_gatekeeper.c`（Windows `.dll`、macOS `.dylib`、Linux `.so`）；没有编译器时打印一行提示并回退到纯 Python/NumPy 门控镜像。`--no-c` 强制镜像；`--selftest` 用 500 个确定性随机帧比较两套实现，要求 gate/regime 完全一致且 `|du| <= 1e-5`。

**1 kHz C99 门控核在 MATLAB 与 Python 中逐帧完全等价**（`--selftest` 误差 `max|du| = 1.55e-07 < 1e-6`）。Python 独立基准的 10 Hz 外环探针采用自适应噪声底与置信度 EMA，因此其 B3 S1–S4 RMSE 为 **0.0090 / 0.0053 / 0.0343 / 0.0873**，OOD 弃权延迟 **9.0 ms**（<10 ms），各阶段安全越界采样点均为 **0**。这些 Python 运行数字有意与 MATLAB 40 s 表不同；门控语义与 C 核心相同。

### 可复用核心 vs 基准脚手架

| 角色 | 文件 | 用法 |
| :--- | :--- | :--- |
| **[REUSABLE CORE]** | `c99_layered_core/layered_gatekeeper.c/.h` + `python_bench/minimal_template.py` | 直接拷贝进目标工程。模板是 10 Hz + 1 kHz 的可复制骨架；自动 `ctypes` 加载 C 核，无编译器时回退最小 Python 门控。 |
| **[BENCHMARK HARNESS]** | `matlab_simulink/run_all_benchmarks.m`、`matlab_simulink/sim_core_engine.m`、`python_bench/run_benchmark.py` | 仅用于复现 40 s 论文/仓库图。**不要把合成被控对象拷入生产工程。** |

`python_bench/minimal_template.py` 只有三个替换插槽：

1. `[USER-PLUGIN-SLOT 1]` — `read_plant_and_reference(t)`：真实传感器 `(q, dq)` 与参考 `(q_ref, dq_ref, ddq_ref)`；
2. `[USER-PLUGIN-SLOT 2]` — `compute_nominal_residual(q, dq, u_prev)`：目标域的极简名义物理残差探针；
3. `[USER-PLUGIN-SLOT 3]` — `slow_loop_10hz_step(...)`：4 维无量纲宏观特征、margin × proximity 置信度、S3 残差守卫、粗粒度 `ai_regime ∈ {1,2,3,4}`。

运行一秒、1 kHz 闭环演示，成功输出 `[OK] Minimal layered control loop completed (0 safety violations)`：

```powershell
python python_bench/minimal_template.py
```

### `.slx` 独立运行

```powershell
matlab -batch "clearvars; sim('matlab_simulink/ai_layered_control_bench.slx'); disp('model runs standalone')"
```

模型只使用内置模块，所有数值参数已内联；它是三条支路的可视化，定量基准在 `sim_core_engine.m`。

## 7. 三个域预设

| 域 | S1 | S2 | S3 | S4 |
| :--- | :--- | :--- | :--- | :--- |
| **工业 WAAM** | 焊枪/轴快速跟踪（轻沉积负载） | +35% 层间热累积等效负载漂移 + 送丝 Stribeck 摩擦 | 弧长/接触刚度 + 弧压高频噪声 + 现场总线时延 | 未见结构谐振 + 传感器尖峰 |
| **边缘 VLA** | 高速自由空间运动（轻负载） | +35% 负载漂移 + 关节静摩擦 | 接触刚度突变 + 视觉/力觉高频噪声 + 10 Hz 边缘推理抖动 | 未见结构谐振 + 传感器尖峰 |
| **闭环神经调控** | 快速相位生物标记物跟踪 | +35% 电极-组织阻抗漂移 + 激活阈值非线性 | 高耦合神经反弹 + EMG/刺激伪迹高频噪声 + 遥测抖动 | 未见 OOD 病理振荡 + 导联微脱位尖峰 |

共享的无量纲字段是 `friction_scale`、`k_scale`、`noise_scale` 与每档 `ff_scale`；`load_domain_profile.m` 与 `docs/domain_mapping_guide.md` 的注释使用各域自己的物理语义。神经调控域不使用机械臂/摩擦/接触刚度措辞。

工业预设的 +35% 漂移为：`J ×1.35, b ×1.25, k ×1.10, fc ×2.00, fs ×2.40, vs ×0.25, fvisc ×1.20`。

## 8. 仓库内容

```text
ai-layered-control/
├── README.md / README_zh.md            本文件 / 中文版
├── AGENTS.md                           机器可读不变量、移植菜谱、命令
├── docs/
│   ├── systems_engineering_manifesto_en.md / _zh.md
│   └── domain_mapping_guide.md
├── c99_layered_core/
│   ├── layered_gatekeeper.h            88 B 状态 / 132 B 配置契约
│   ├── layered_gatekeeper.c            零依赖 1 kHz 核心
│   ├── test_gatekeeper_standalone.c    116 项确定性检查
│   └── verify_core_equivalence.c       闭环 C 回放夹具（仅测试）
├── matlab_simulink/
│   ├── load_domain_profile.m
│   ├── sim_core_engine.m
│   ├── calibrate_coarse_classifier.m
│   ├── train_continuous_policy.m
│   ├── run_efficiency_analysis.m
│   ├── run_all_benchmarks.m
│   ├── build_simulink_model.m
│   ├── verify_core_equivalence.m
│   └── ai_layered_control_bench.slx    生成产物
├── python_bench/
│   ├── minimal_template.py             98 行可复用复制骨架
│   ├── run_benchmark.py                ctypes C 核心或 NumPy 回退（40 s 基准）
│   ├── README.md                       可复用核心 / 脚手架区分
│   └── requirements.txt
└── assets/                             三张生成的 240 dpi 图
```

纯文本：C99、MATLAB、Python、Markdown。仓库不提交权重或数据集：RBF 策略在调试时由合成名义 rollout 确定性拟合；分类器原型由三个 stage 探针拟合；C 核心是可部署产物。

## 9. 适用范围与免责声明

本仓库是“快慢双环解耦 + 显式拒识 + 确定性安全门控”控制架构的方法论参考实现。`neuromodulation`（aDBS/SCS）预设基于集总参数合成非线性动力学与同构生物标记物包络模型，不含任何真实患者临床记录，也不构成特定疾病的临床生理/病理模型。C99 核心遵循零动态内存、O(1) 最坏执行时间、NaN 防火墙与粘滞故障审计等防御性嵌入式设计原则，但未经过且不声称具备 IEC 62304、ISO 14971、IEC 60601 或 IEC 61508 认证。任何面向临床植入/穿戴式神经调控或高危工业设备的实际部署，均须配合独立硬件电荷平衡保护电路、硬件看门狗及完整的临床与法规审批验证。

## 10. 引用

如果本参考架构或基准对你的工作有帮助，请引用：

```bibtex
@software{li2026ailayeredcontrol,
  author       = {Li, Jiawei},
  title        = {ai-layered-control: A Systems-Engineering Reference Architecture for Layered Closed-Loop AI Control},
  year         = {2026},
  version      = {1.1.0},
  url          = {https://github.com/lijiawei255/ai-layered-control},
  license      = {MIT}
}
```

**姊妹项目：** [jev-tune](https://github.com/lijiawei255/jev-tune) —— 单一控制器内部的连续参数层整定，C99 安全边界兜底。