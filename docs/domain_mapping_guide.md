# Domain mapping / 领域映射

The architecture is a generic division of control authority. Industrial WAAM / PLC servo,
Edge VLA / fast-slow robotics and closed-loop aDBS / SCS are three equal application examples;
the mechanical benchmark is one common test fixture for all of them.

本架构提供通用的控制权限分工。工业 WAAM／PLC 伺服、边缘具身智能、植入式闭环神经调控是并列实例，
机械基准是三者共用的一套试验台。

## The interface / 接口

| Contract | Device-specific definition |
| :--- | :--- |
| `x, x_dot` | Measured/estimated controlled state and rate, with explicit units |
| `r, r_dot` | Reference and reference rate in the same units |
| `u_ff` | Deterministic commissioned-model feedforward |
| `physics_residual` | Dimensionless independent nominal-physics innovation |
| `ai_regime, ai_confidence` | Normal regime 1–3 or explicit abstention 4, and confidence in [0,1] |
| `outer_age_s` | Transported age of the source frame, updated every fast tick |
| `outer_seq` | uint32 slow-frame sequence number; 0 selects the legacy `outer_age_s` mode |
| `FieldTuning_t` | 12 B physical knobs: gain, damping and commissioned safety-margin scales |

| ABI | Size |
| :--- | :--- |
| `LayeredState_t` / `LayeredConfig_t` | 96 B / 140 B |
| `FieldTuning_t` / `LayeredInput_t` / `LayeredOutput_t` | 12 B / 40 B / 28 B |
| `LayeredProbe_t` (`gate_template.c`) | 32 B |

Define the actuator mapping, state envelope, slew bounds and fallback objective together.
Express the commissioned fallback point as state zero for the fixed R4 law.
Nominal-model mismatch informs the probe; the classifier cannot redefine that evidence.
Choose dimensionless macro-features from measured signals and commission their scales.
No deployed mode decision may use benchmark stage labels or scheduled fault times.

执行器映射、状态包络、斜率限制与回退目标应一起定义。标称模型残差由独立探针计算，
分类器不能改变这份物理证据。宏观特征来自可测信号，尺度通过设备标定确定。
部署路径不读取阶段真值或预定故障时刻。

The 1 kHz probe template in [gate_template.c](../c99_layered_core/gate_template.c) runs a three-point
median followed by a cascaded high-pass innovation envelope and fits in a 32 B `LayeredProbe_t`.
Its `blanking_active` input freezes every probe state and returns 0.0f during a scheduled stimulation
pulse or commutation edge, so an elevated residual cannot hold the OOD veto through a hardware blanking
window; the device masks the gate for the same window and the probe resumes afterwards.

[gate_template.c](../c99_layered_core/gate_template.c) 提供 1 kHz 探针模板：三点中值后接级联高通
新息包络，状态仅 32 B。`blanking_active` 有效时冻结全部探针状态并返回 0.0f，使计划内的刺激脉冲
或换向边沿不会把已抬高的残差带过硬件消隐窗口；同一窗口内设备侧屏蔽门控输出，消隐结束后探针恢复。

## Gate decisions and audit bits / 门控决策与审计位

| Code | Gate | Meaning |
| :---: | :--- | :--- |
| 0 | `GATE_NORMAL` | Supervisor regime accepted; the selected bank runs |
| 1 | `GATE_HOLD_LATENCY` | Stale frame or unusable fast input; last command held |
| 2 | `GATE_VETO_OOD_RESIDUAL` | Physics innovation out of distribution; abstain to R4 |
| 3 | `GATE_VETO_LOW_CONFIDENCE` | Confidence below threshold or explicit regime 4; abstain to R4 |
| 4 | `GATE_VETO_SAFETY_ENVELOPE` | State envelope or look-ahead exceeded; R4 |
| 5 | `GATE_SAFE_ZERO` | Native HOLD past `max_hold_ticks = 100`: integrators cleared, command ramped to zero at `u_slew_max_per_s` |
| 6 | `GATE_LATCHED_SAFE` | Cumulative severe faults (`max_fault_vetoes_before_latch = 3`): permanently in R4 until acknowledged |

| Bit | Fault |
| :---: | :--- |
| 1<<0 | `LATENCY_TIMEOUT` |
| 1<<1 | `OOD_RESIDUAL` |
| 1<<2 | `LOW_CONFIDENCE` |
| 1<<3 | `ENVELOPE_BREACH` |
| 1<<4 | `NAN_CORRUPTION` |
| 1<<5 | `HOLD_TIMEOUT` |
| 1<<6 | `PERMANENT_LATCH` |

A safety or OOD veto episode increments the latch counter; the third episode latches
`GATE_LATCHED_SAFE`. `Layered_AcknowledgeFaultLatch()` is the only release path: it clears the veto and
permanent latches, resets the episode counter, keeps the lifetime veto counters and the 7-bit
`fault_audit_bitmask` for forensics, and re-arms the recovery cooldown before normal regimes can be
accepted again. A veto or the permanent latch selects R4 with the blend filter bypassed (blend = 1.0),
so emergency damping is not delayed by transfer smoothing.

安全或残差否决的一个 episode 增加一次锁存计数，累计 3 次即进入 `GATE_LATCHED_SAFE`。
唯一解除路径是 `Layered_AcknowledgeFaultLatch()`：清除否决与永久锁存、重置 episode 计数，
保留终身否决计数与 7 位 `fault_audit_bitmask` 供事后追溯，并重新武装恢复冷却期后才允许接受正常工况。
否决激活或永久锁存切入 R4 时旁路 blend 滤波（blend = 1.0），紧急阻尼不受一阶无扰切换的延迟影响。

## Three parallel examples / 三类并列实例

| Application | Slow semantic decision | Fast quantitative responsibility |
| :--- | :--- | :--- |
| Industrial WAAM / PLC servo | Process regime, load/contact condition, sensor validity | Process or servo regulation, actuator authority, contact/output limits |
| Edge VLA / fast-slow robotics | Task/environment context, contact class, abstention | Torque/impedance loop, disturbance rejection, explicit fallback |
| Closed-loop neuromodulation aDBS / SCS | Biomarker regime, electrode/interface state, abstention | Device-defined feedback, output envelope, pulse/timing constraints |

For WAAM, choose an observable process state and its causal actuator model;
for a servo, define the actual drive units and contact limits.
For robotics, separate task semantics from the fast measured-state control interface.
For neuromodulation, define device-specific states and timing constraints;
mechanical PID gains and torque limits do not transfer by renaming variables.

工业过程控制先选可观测过程量及其执行器模型；伺服先确定驱动单位与接触约束。
机器人把任务语义与快速状态闭环分开。
神经调控按设备定义状态、输出和时序约束，不能把机械增益和力矩阈值直接改名套用。

## Commission, freeze, tune / 建档、冻结、整定

1. Define the signals, units, nominal model, controller bank and explicit fallback.
2. Collect labeled macro-feature windows for `auto_calibrate(samples, labels)`.
3. Freeze prototypes, distance-decay coefficient and confidence thresholds.
4. Run `auto_field_tune(evaluate, current)` with a device-side safe trial callback.

The callback returns measured cost and constraint validity. Invalid candidates are rejected;
if all candidates are invalid, the current knobs remain in use.
Evaluate both load/stiction and contact behavior; constrain force, veto fraction and R4 occupancy.
Keep the safety-margin scale fixed during automated gain/damping searches.
Gain/damping are bounded to [0.10,4.0] and margin to [0.10,1.0].
The fixed search evaluates six joint candidates, each independently under the same constraints.
A new unit within the same interface can retain the semantic archive.
A new physical domain starts with a new interface and commissioning archive.

每次候选试验返回实测误差与约束是否满足；不合格候选不能被选中，
全部不合格时保留当前旋钮。自动增益／阻尼搜索保持已确定的安全裕度不变。
试验覆盖重载静摩擦与接触工况，同时限制接触力、否决率和 R4 占比。
增益／阻尼范围为 [0.10,4.0]，安全裕度范围为 [0.10,1.0]。
固定搜索评估 6 组联合候选，每组都在相同约束下独立判定。
同场景换机台可冻结语义档案，全新物理领域先建立新接口与标定档案。

## Multi-axis and timeout / 多轴与超时

Each channel owns its state, probe, tuning and source timestamp.
Share read-only configurations only for channels with identical units and control definitions.
The adapter evaluates coupled constraints and coordinates fallback.
The core remains a single-axis API.

On a stale frame, preserve the real age and send explicit R4 abstention.
This keeps deterministic fallback running while the latency fault remains observable.
A fresh frame restores normal decisions only after the recovery cooldown (350 ms timeout and 300 ms
cooldown in the reference).
Malformed slow fields also select closed-loop R4, preserving envelope/residual priority.
With `outer_seq` nonzero, the core measures frame age on its own 1 kHz clock: a new sequence opens a
fresh lease, an unchanged one ages it by one tick, and a frozen or forged `outer_age_s` cannot mask a
stalled mailbox; a sequence falling back to zero is treated as corruption, not as a fresh frame.
Native HOLD preserves the applied command, not a mechanical position; past `max_hold_ticks` the core
enters `GATE_SAFE_ZERO` and ramps to zero in steps of `max(u_slew_max_per_s * dt_s, 1 binary32 ULP(|u_prev|))`.
R4 uses fixed gains, zero-state error and no feedforward, independent of field tuning.

每通道独立持有状态、探针、旋钮与源时间戳。
只有单位和控制定义相同的通道才能共享只读配置。
适配层处理耦合约束与协调回退，C API 保持单轴形式。
超时后保留真实帧龄并显式发送 R4，既运行确定性回退，又保留延迟故障的可观测性。
新鲜帧经过恢复冷却期后才重新启用正常模态。
慢环字段损坏也进入闭环 R4，保留包络与残差优先级。
`outer_seq` 非零时，内核用本地 1 kHz 时钟测量帧龄：序号变化即打开新租约，序号不变则每 tick 老化一次，
冻结或伪造的 `outer_age_s` 无法掩盖停滞的邮箱。原生 HOLD 保持的是执行器指令而非机械位置；
超过 `max_hold_ticks` 后进入 `GATE_SAFE_ZERO`，按 `max(u_slew_max_per_s * dt_s, 1 binary32 ULP(|u_prev|))` 的步长严格单调归零。
R4 固定增益、零状态参考和零前馈，不受现场增益旋钮缩放。

## Verification / 验证

C99 standalone regression passes 235 checks with zero failures; `gate_template.c` prints PASS.
Python/C `--selftest` replays 6,000 frames with max |du| = 0 in binary32, MATLAB/C equivalence
keeps max |du| = 5e-9 across normal, corrupted-mailbox and illegal-knob runs, a 10,000-tick
`outer_seq` lease replay stays within max |du| = 4.5e-8, and the Level-2
S-Function replays 1,100 frames with all seven outputs at max diff = 0.0.

C99 独立回归 235 项全部通过、0 失败；`gate_template.c` 自检输出 PASS。
Python/C `--selftest` 回放 6,000 帧，binary32 下 max |du| = 0；MATLAB/C 等价性在正常帧、损坏邮箱帧
与非法旋钮注入下均保持 max |du| = 5e-9；10,000 tick 的 `outer_seq` 租约回放 max |du| = 4.5e-8；Level-2 S-Function 回放 1,100 帧，7 路输出 max diff = 0.0。

Commands and exact API semantics are in [AGENTS.md](../AGENTS.md).
Numerical evidence is maintained in [README.md](../README.md) / [README_zh.md](../README_zh.md).
