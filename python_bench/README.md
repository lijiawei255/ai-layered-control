# Python quick start and integration guide — v2.0.2

This directory is the Python entry point of the repository: it hosts the integration template and the
benchmark harness. Four reusable modules carry the integration, and each is complete on its own.
The 1 kHz ANSI C99 core is the zero-trust safety arbiter of the 10 Hz slow interface: it validates
every slow-loop decision, measures the frame lease on its own clock when `outer_seq` is enabled, and
vetoes independently of the slow loop.

## Directory role and the four reusable modules

| Module | Size | Role |
| :--- | :--- | :--- |
| [minimal_template.py](minimal_template.py) | 119 lines | 10 Hz qualitative small-sample calibration (`auto_calibrate`, 36 windows / 15 scalars / 60 B), low-confidence abstention, frozen-AI three-knob field tuning (`auto_field_tune`, 20 lines for both helpers) and the SLOT 1..3 integration skeleton |
| [gate_template.c](../c99_layered_core/gate_template.c) | 120 lines | 1 kHz MCU bare-metal ISR template: 32 B `LayeredProbe_t` (median glitch rejection + cascaded high-pass innovation envelope), `blanking_active` hardware blanking window and the `outer_seq` commit-marker pattern |
| [layered_gatekeeper.h](../c99_layered_core/layered_gatekeeper.h) / [layered_gatekeeper.c](../c99_layered_core/layered_gatekeeper.c) | 96 B state | 1 kHz zero-dependency ANSI C99 safety arbitration and four-bank anti-windup control micro-kernel; the ABI occupies 96/140/12/40/28 B for state/config/tuning/input/output |
| [sfun_layered_gatekeeper.c](../simulink_sfunc/sfun_layered_gatekeeper.c) | 685 lines | Level-2 C-MEX S-Function: `DWork` holds the 96 B state, zero global variables, zero dynamic memory, 11 inputs (10-field ABI + ack rising edge) and 7 outputs |

[run_benchmark.py](run_benchmark.py) is the benchmark harness: it builds the shared physical fixture,
trains B2, runs the branch comparisons and field-tuning experiments, and writes the result tables.
Authoritative numerical results live in [README.md](../README.md) and
[README_zh.md](../README_zh.md); MIT reuse and optional citation are described in the root
[license and citation guide](../README.md#license-and-citation), and
[CITATION.cff](../CITATION.cff) contains the author and citation metadata.

## Run

From the repository root:

```powershell
python -m pip install -r python_bench/requirements.txt
python python_bench/minimal_template.py
python python_bench/run_benchmark.py
python python_bench/run_benchmark.py --no-c
python python_bench/run_benchmark.py --selftest
```

- `minimal_template.py` runs the self-contained template mission. It requires Python 3 plus the
  packages in `requirements.txt`, and a C99 compiler (`gcc` by default, or the one named by `CC`,
  looked up on `PATH`). It rebuilds the dynamic library whenever `layered_gatekeeper.c` or
  `.h` changes, and compilation errors propagate directly.
- `run_benchmark.py` executes the full benchmark suite under the same prerequisites.
- `--no-c` selects the complete Python core mirror when no C compiler is available.
- `--selftest` compares the Python mirror against the compiled C core.

Generated libraries and the Python preview figure remain ignored build artifacts. The template's
own PASS criteria are zero state-envelope violations, zero abnormal gate/R4/corruption ticks and
RMSE below 0.02 state units; the frozen classifier also serves field tuning.

## Graded branches and the current fixture

The harness grades five branches on one 40 s, four-stage fixture (S1 nominal, S2 +35 % load/stiction, S3 unilateral contact, S4 unmodelled flexible mode): B1 (fixed-parameter white-box), B2 (10 Hz context + 1 kHz conditioned RBF action head whose policy input carries 10 features, including one clamped error-integral history feature normalized by the integrator limit), B3 (10 Hz qualitative routing + 1 kHz C99 gate), and the two fairness controls B1+C99 Gate / B2+C99 Gate, where the identical 96 B gate is mounted transparently on top of B1/B2 (zero normal-bank gains, unit feedforward, `blend_alpha = 1`, identical envelope/residual/latency/hold/R4/slew parameters). The comparison therefore separates the universal runtime-assurance shell from the slow qualitative layer.

The contact probe is position-independent: it combines a resistive effort/velocity product with a 20 ms stalled-velocity one-signed tracking-error term and keeps an 8 s surface memory against real contact. Release is physical, not a slow tail: a 250 ms reference-step blanking rejects inertial command transients, the remembered contact point is abandoned once the measured state passes it with position-independent speed, the free-state decay is 0.10 s, and a genuine resistive blockage re-arms the surface immediately. The selftest includes a wall-position generalisation check at wall positions 0.30 / 0.43 / 0.55 rad with the reference amplitude shifted by the same 0.07 rad and every controller/probe parameter unchanged, plus a closed-loop `GATE_HOLD_LATENCY` parity test and an independent nominal control against false confidence vetoes. S4 no longer injects a first-tick spike; the OOD veto rests on the real unmodelled-flexible-mode residual and the random glitch process. The frozen-AI tuning study validates each candidate on two wall phases and grades B1 through the same C99 gate. The off-manifold audit runs at a normal amplitude and commissioning noise with an in-box coupled constant load-torque bias, a 1 % per-frame split-conformal budget plus 5-frame persistence, and a data-derived (not algebraic) single-dimension support box.

## Adapt three slots and the core API

The three slots in the template mark the device-specific code:

1. **SLOT 1** — replace the sensor, trajectory and actuator I/O with the device's measured state and
   command path.
2. **SLOT 2** — define the commissioned nominal predictor, the independent residual probe and the
   normalized dimensionless features.
3. **SLOT 3** — supply the labeled commissioning windows and a safe device-trial callback for
   field tuning.

Any AI coding assistant (DeepSeek Harness / Kimi Code / OpenCode / ZCode / Qoder / Codex / Claude Code / Cursor) can implement these three slots directly; reuse or customize `auto_calibrate` and `auto_field_tune` as needed.

`Core(config, tuning=(1,1,1))` accepts the exact C configuration names; the bank fields `kp`, `kd`,
`ki` and `ff_scale` each contain four values. One call

```text
step(ai_regime, ai_confidence, outer_age_s, physics_residual, x, x_dot, r, r_dot, u_ff, outer_seq)
```

takes 10 inputs — `ai_regime, ai_confidence, outer_age_s, physics_residual, x, x_dot, r, r_dot,
u_ff, outer_seq` — and returns 7 outputs: `(u, gate, regime, blend, fault_audit_bitmask, corrupted,
u_target)`. `acknowledge_fault_latch()` calls `Layered_AcknowledgeFaultLatch()` and is the only
release path for `GATE_LATCHED_SAFE`; the lifetime veto counters and the 7-bit
`fault_audit_bitmask` survive the call.

`auto_calibrate(samples, labels)` expects four dimensionless features per window and labels 1–3,
and returns three prototypes and three confidence parameters. Freeze these parameters, then call
`auto_field_tune(evaluate, current)`. The trial callback returns `(measured_cost, valid)`; an
invalid candidate cannot win, and if every candidate fails the constraints the search returns the
current knobs. The fixed search evaluates six joint candidates — gain/damping pairs `(0.75,1)`,
`(1,1)`, `(1.25,1)`, `(2,1)`, `(1.25,0.5)` and `(1.25,0.75)` — each independently under the same
constraints, so an infeasible higher-damping trial does not exclude its lower-damping counterpart.
Gain/damping lie in [0.10,4.0] and the margin scale in [0.10,1.0]; the safety-margin scale stays
fixed during the automated gain/damping search. Include load/stiction and contact trials and
constrain contact force, veto fraction and R4 occupancy. R4 ignores these gain scalings, trajectory
references and feedforward; it uses fixed feedback toward the safe zero.

Frame age advances every tick from the source timestamp. With `outer_seq` nonzero the core measures
age on its own 1 kHz clock: a new sequence opens a fresh lease, an unchanged one ages by one tick,
and a frozen or forged `outer_age_s` cannot mask a stalled mailbox; 0 selects the legacy age mode.
Native HOLD retains the applied actuator command and is bounded by `max_hold_ticks`, after which the
core enters `GATE_SAFE_ZERO` and ramps the command to zero in steps of
`max(u_slew_max_per_s * dt_s, 1 binary32 ULP(|u_prev|))` — the one-ULP floor applies only when the
legal slew budget is below one representable step, so a sub-ULP budget still de-energises in finite
time. On timeout the adapter sends R4 with zero confidence while retaining the real age; malformed
slow mailbox fields or invalid field knobs run closed-loop R4 with `corrupted=1`. The
`blanking_active` input of the probe template freezes every internal filter state and holds the
pre-blanking residual (`return p->residual`) during a scheduled stimulation pulse or commutation
edge: a healthy periodic pulse cannot raise the residual, a sustained fault spanning periodic
blanking keeps one OOD latch episode, and the residual decays along the release envelope outside
the blanking window. Three safety/OOD veto episodes latch
`GATE_LATCHED_SAFE` permanently until acknowledged; count safety violations from the true/measured
state, independently of gate activity.

## Verification summary

The C99 standalone regression passes 265 checks with zero failures; Python/C `--selftest` replays
6,000 frames bit-exact in binary32 with max |du| = 0; MATLAB/C equivalence replays 40,000 normal and
40,000 corrupted-mailbox frames plus illegal-knob runs at max |du| = 5e-9, and a 10,000-tick
`outer_seq` lease replay stays within max |du| = 3.6e-8 (binary32 values bit-identical after text round-trip); the Level-2 S-Function replays 1,100
frames with all seven outputs at max diff = 0.0.

For interface adaptation order, multi-axis ownership and timeout behavior, see [AGENTS.md](../AGENTS.md) and
[domain_mapping_guide.md](../docs/domain_mapping_guide.md). Full numerical evidence is maintained in
the root [README.md](../README.md) and [README_zh.md](../README_zh.md).

> **Scope:** This repository is an uncertified engineering reference architecture maintained by an independent developer; real-equipment integration requires deployer-owned hazard validation and independent hardware safety protection (E-stop, STO, hardware charge-balance limiters; see the root [README.md License and citation](../README.md#license-and-citation)). The application mappings are directions for adaptation, not hardware, clinical or functional-safety verification or certification, and they do not replace certification or lifecycle processes under IEC 61508, ISO 13849, ISO 26262 or IEC 62304.

---

# Python 快速上手与接入指南 — v2.0.2

本目录是仓库的 Python 入口：集成模板与基准测试框架都放在这里。接入工作由四个可复用模块承担，
每个模块都可以独立使用。
1 kHz 的 ANSI C99 内核是 10 Hz 慢环接口的零信任安全仲裁：它对每一条慢环决策执行合法性
校验，在启用 `outer_seq` 时以本地时钟计量帧龄，并独立执行物理否决。

## 目录定位与四大模块概览表

| 模块 | 规模 | 职责 |
| :--- | :--- | :--- |
| [minimal_template.py](minimal_template.py) | 119 行 | 10 Hz 定性小样本标定（`auto_calibrate`，36 窗口 / 15 标量 / 60 B）、低置信度弃权、冻结 AI 的三旋钮现场整定（`auto_field_tune`，两个辅助函数合计 20 行），以及 SLOT 1..3 接入骨架 |
| [gate_template.c](../c99_layered_core/gate_template.c) | 120 行 | 1 kHz MCU 裸机中断模板：32 B `LayeredProbe_t`（中值抗毛刺 + 级联高通包络，提取标称模型预测残差（新息））、`blanking_active` 硬件消隐窗口与 `outer_seq` 提交标记写法 |
| [layered_gatekeeper.h](../c99_layered_core/layered_gatekeeper.h) / [layered_gatekeeper.c](../c99_layered_core/layered_gatekeeper.c) | 96 B 状态 | 1 kHz 零依赖 ANSI C99 安全仲裁与四档控制律库抗饱和微内核；ABI 为 状态/配置/整定/输入/输出 96/140/12/40/28 B |
| [sfun_layered_gatekeeper.c](../simulink_sfunc/sfun_layered_gatekeeper.c) | 685 行 | Level-2 C-MEX S-Function：`DWork` 持有 96 B 状态，零全局变量、零动态内存，11 输入（10 字段 ABI + 确认上升沿）/ 7 输出 |

[run_benchmark.py](run_benchmark.py) 是基准测试框架：搭建共享物理测试台架、训练 B2、运行分支对比
与现场整定实验，并输出结果表。权威数值结果见根目录 [README.md](../README.md) 与
[README_zh.md](../README_zh.md)；MIT 复用与可选引用说明见根目录的
[license and citation guide](../README.md#license-and-citation)，作者与引用元数据在
[CITATION.cff](../CITATION.cff) 中。

## 分级分支与当前测试台架

测试框架在一条 40 s、四阶段测试台架（S1 标称、S2 +35 % 带载/静摩擦、S3 单边接触、S4 未建模柔性模态）上评估五个分支：B1（定参白盒）、B2（10 Hz 上下文 + 1 kHz 条件 RBF 动作头，其策略输入含 10 个特征，包括一个按积分器限幅归一化的钳位误差积分历史特征）、B3（10 Hz 定性路由 + 1 kHz C99 门控），以及两个公平性对照 B1+C99 Gate / B2+C99 Gate——同一 96 B 门控透明地叠加在 B1/B2 之上（零正常档增益、单位前馈、`blend_alpha = 1`，包络/残差/延迟/保持/R4/斜率参数完全一致）。因此该对比把通用运行时保障外壳与慢环定性层分开。

接触探针与位置无关：它把阻性力矩/速度乘积与 20 ms 停转速度单符号跟踪误差项结合，对真实接触保持 8 s 表面记忆。释放是物理判据而非慢尾：250 ms 参考阶跃消隐切断惯性指令瞬态；对象带位置无关速度越过记忆接触点即放弃该表面；自由态衰减为 0.10 s；真实阻性阻塞会立即重新武装表面。自检包含墙体位置泛化检查（墙体位置 0.30 / 0.43 / 0.55 rad，参考幅值同移 0.07 rad，所有控制器/探针参数不变）、闭环 `GATE_HOLD_LATENCY` 逐帧一致性回放测试，以及独立标称低误报校验。S4 不再注入首拍尖峰；OOD 否决依赖真实的未建模柔性模态残差与随机毛刺过程。冻结 AI 整定研究在两个墙体相位上验证每个候选，并用同一 C99 门控给 B1 打分。流形外审计以正常幅值与建档噪声运行，叠加盒内耦合常值负载偏置、1% 逐帧分裂共形预算 + 5 帧持续性，以及数据导出的单维支撑盒。

## 运行命令

在仓库根目录执行：

```powershell
python -m pip install -r python_bench/requirements.txt
python python_bench/minimal_template.py
python python_bench/run_benchmark.py
python python_bench/run_benchmark.py --no-c
python python_bench/run_benchmark.py --selftest
```

- `minimal_template.py` 运行自包含的模板任务。前置条件是 Python 3 加 `requirements.txt` 中的
  依赖，以及一个 C99 编译器（默认 `gcc`，也可用 `CC` 指定，在 `PATH` 上查找）。
  `layered_gatekeeper.c` 或 `.h` 有变动时会自动重建动态库，编译错误直接抛出。
- `run_benchmark.py` 在同样的前置条件下运行完整基准套件。
- `--no-c` 在没有 C 编译器时改用完整的 Python 内核镜像。
- `--selftest` 把 Python 镜像与编译出的 C 内核逐帧对拍。

生成的动态库与 Python 预览图均为被忽略的构建产物，不会入库。模板自带的 PASS 判据是：
零状态包络越界、零异常门控 / R4 / 损坏 tick，且 RMSE 低于 0.02 状态单位；冻结后的分类器
同时用于现场整定。

## 三槽位接口适配与核心 API

模板中的三个槽位对应需要对接设备侧接口的三处：

1. **SLOT 1**：把传感器、轨迹与执行器 I/O 换成设备的实测状态与指令通道。
2. **SLOT 2**：定义已标定的标称预测器、独立残差探针，以及归一化的无量纲特征。
3. **SLOT 3**：提供带标签的标定窗口，以及一个安全的设备试验回调用于现场整定。

以上三个槽位可由任意 AI 编程助手（DeepSeek Harness / Kimi Code / OpenCode / ZCode / Qoder / Codex / Claude Code / Cursor）直接实现；`auto_calibrate` 与 `auto_field_tune` 可直接复用或按现场需求定制。

`Core(config, tuning=(1,1,1))` 接受与 C 完全一致的配置名；`kp`、`kd`、`ki`、`ff_scale`
四个控制档位字段各含四个值。一次调用

```text
step(ai_regime, ai_confidence, outer_age_s, physics_residual, x, x_dot, r, r_dot, u_ff, outer_seq)
```

接收 10 个输入——`ai_regime, ai_confidence, outer_age_s, physics_residual, x, x_dot, r, r_dot,
u_ff, outer_seq`——返回 7 个输出：`(u, gate, regime, blend, fault_audit_bitmask, corrupted,
u_target)`。`acknowledge_fault_latch()` 内部调用 `Layered_AcknowledgeFaultLatch()`，是解除
`GATE_LATCHED_SAFE` 的唯一路径；终身否决计数与 7 位 `fault_audit_bitmask` 在调用后保留。

`auto_calibrate(samples, labels)` 要求每个窗口给出四个无量纲特征，标签取 1–3，返回三个原型
与三个置信度参数。冻结这些参数后调用 `auto_field_tune(evaluate, current)`。试验回调返回
`(measured_cost, valid)`：无效候选不能胜出；全部候选都不合格时，保留当前旋钮。固定搜索评估
六组联合候选——增益/阻尼组合 `(0.75,1)`、`(1,1)`、`(1.25,1)`、`(2,1)`、`(1.25,0.5)`、
`(1.25,0.75)`——每组在相同约束下独立判定，高阻尼试验不可行不会牵连低阻尼的对应项。
增益/阻尼范围 [0.10,4.0]，裕度缩放系数范围 [0.10,1.0]；自动搜索增益/阻尼期间安全裕度缩放系数保持
冻结。试验要覆盖带载/静摩擦与接触工况，并约束接触力、否决占比与 R4 驻留时间。R4 忽略这些
增益缩放、轨迹参考与前馈，只做指向安全零点的固定反馈。

帧龄从源时间戳起每拍推进。`outer_seq` 非零时，内核用自己的 1 kHz 时钟测龄：新序号开启新租约，
序号不变则每拍老化一次，冻结或伪造的 `outer_age_s` 无法掩盖慢环通信停滞；取 0 退回传统帧龄模式。
慢环帧过期但快环传感器有效时，内核报告 `GATE_HOLD_LATENCY` 并继续按固定 R4 律闭环；只有快环输入不可用才开环保持上一条指令。超过 `max_hold_ticks` 后内核进入 `GATE_SAFE_ZERO`，
按 `max(u_slew_max_per_s * dt_s, 1 binary32 ULP(|u_prev|))` 的步长把指令归零——单 ULP 保底步长
只在合法斜率预算不足一个可表示步进时生效，因此亚 ULP 的预算也能在有限时间内完成能量泄放归零。
超时后适配层发送零置信度的 R4，同时保留真实帧龄；慢环通信帧字段畸形或现场旋钮非法时，
以 `corrupted=1` 运行闭环 R4。探针模板的 `blanking_active` 输入在计划内刺激脉冲或换向边沿期间
冻结全部内部滤波器状态并保持消隐前的残差（`return p->residual`）：正常周期性脉冲不会抬升残差；
跨越多个消隐窗口的持续异常只计一次 OOD 锁存事件；消隐窗口之外，残差按释放包络自然衰减。安全/OOD 否决累计三次即
永久锁存 `GATE_LATCHED_SAFE`，直到确认解除；安全违规的统计以真实/实测状态为准，与门控动作无关。

## 基准验证摘要

C99 独立回归 265 项全部通过、零失败；Python/C `--selftest` 回放 6,000 帧，binary32 下
max |du| = 0 位级一致；MATLAB/C 等价性回放 40,000 帧正常帧与 40,000 帧慢环通信损坏帧，并叠加非法
旋钮注入，max |du| = 5e-9；10,000 拍的 `outer_seq` 租约回放 max |du| = 3.6e-8（文本往返后 binary32 值逐位一致）；Level-2
S-Function 回放 1,100 帧，7 路输出 max diff = 0.0。

接口适配顺序、多轴归属与超时行为见 [AGENTS.md](../AGENTS.md) 与
[domain_mapping_guide.md](../docs/domain_mapping_guide.md)。完整数值证据维护在根目录
[README.md](../README.md) 与 [README_zh.md](../README_zh.md) 中。

> **适用范围：** 本仓库为个人开发者维护的非认证工程参考架构；真实设备集成须由部署方独立完成危险分析与安全验证，并保留独立于本软件的硬件安全保护（E-stop、STO、硬件电荷平衡限制器，详见根目录 [README_zh.md 许可与引用](../README_zh.md#许可与引用)）。应用映射是适配方向，不是硬件、临床或功能安全验证或认证，亦不替代 IEC 61508、ISO 13849、ISO 26262 或 IEC 62304 等标准下的认证与生命周期过程要求。
