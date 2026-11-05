# Domain mapping / 领域映射

The architecture is a generic division of control authority. Industrial WAAM / PLC servo,
Edge VLA / fast-slow robotics and closed-loop aDBS / SCS are three equal application examples;
the mechanical benchmark is one common test fixture for all of them.

本架构只规定控制权限如何划分，与具体被控对象无关。工业 WAAM／PLC 伺服、边缘具身智能、
闭环神经调控（aDBS／SCS）是三个地位并列的应用实例，机械基准则是三者共用的同一套试验台。

## The interface / 接口

| Contract | Device-specific definition |
| :--- | :--- |
| `x, x_dot` | Measured/estimated controlled state and rate, with explicit units（实测或估计的受控状态及其变化率，单位必须显式声明） |
| `r, r_dot` | Reference and reference rate in the same units（与状态同单位的参考量及其变化率） |
| `u_ff` | Deterministic commissioned-model feedforward（基于已标定模型的确定性前馈） |
| `physics_residual` | Dimensionless independent nominal-physics innovation（由独立通道计算的无量纲标称模型预测残差（新息）） |
| `ai_regime, ai_confidence` | Normal regime 1–3 or explicit abstention 4, and confidence in [0,1]（正常工况编号 1–3，或显式弃权 4；置信度取值 [0,1]） |
| `outer_age_s` | Transported age of the source frame, updated every fast tick（随帧传输的源帧龄，每个快拍更新一次） |
| `outer_seq` | uint32 slow-frame sequence number; 0 selects the legacy `outer_age_s` mode（uint32 慢帧序号；取 0 时退回传统 `outer_age_s` 模式） |
| `FieldTuning_t` | 12 B physical knobs: gain, damping and commissioned safety-margin scales（12 B 物理旋钮：增益、阻尼与已标定的安全裕度缩放系数） |

| ABI | Size |
| :--- | :--- |
| `LayeredState_t` / `LayeredConfig_t` | 96 B / 140 B（内核状态 / 配置） |
| `FieldTuning_t` / `LayeredInput_t` / `LayeredOutput_t` | 12 B / 40 B / 28 B（整定旋钮 / 输入 / 输出） |
| `LayeredProbe_t` (`gate_template.c`) | 32 B（探针状态） |

Define the actuator mapping, state envelope, slew bounds and fallback objective together.
Express the commissioned fallback point as state zero for the fixed R4 law.
Nominal-model mismatch informs the probe; the classifier cannot redefine that evidence.
Choose dimensionless macro-features from measured signals and commission their scales.
No deployed mode decision may use benchmark stage labels or scheduled fault times.

执行器映射、状态包络、斜率上限与回退目标要放在一起定义，不能分开决定。固定的 R4 回退律
以标定好的回退点为状态零点。标称模型失配由独立探针量化为残差，分类器无权修改这份物理证据。
无量纲宏观特征必须取自可测信号，其尺度在设备标定时确定。任何部署态的模态决策都不得使用
基准的阶段标签或预设故障时刻。

The 1 kHz probe template in [gate_template.c](../c99_layered_core/gate_template.c) runs a three-point
median followed by a cascaded high-pass innovation envelope and fits in a 32 B `LayeredProbe_t`.
Its `blanking_active` input freezes every internal filter state and holds the pre-blanking residual
(`return p->residual`) during a scheduled stimulation pulse or commutation edge: a healthy periodic
pulse cannot raise the residual, a sustained fault spanning periodic blanking keeps one OOD latch
episode, and the residual decays along the release envelope outside the blanking window; no
device-side gating masking is needed.

[gate_template.c](../c99_layered_core/gate_template.c) 提供 1 kHz 探针模板：先做三点中值滤波，
再串联高通预测残差包络，全部状态只占 32 B。`blanking_active` 输入有效时，探针冻结全部内部
滤波器状态并保持消隐前的残差（`return p->residual`）：正常周期性脉冲不会抬升残差；跨越多个
消隐窗口的持续异常只计一次 OOD 锁存事件；消隐窗口之外，残差按释放包络自然衰减。设备侧无需屏蔽门控输出。

## Gate decisions and audit bits / 门控决策与审计位

| Code | Gate | Meaning |
| :---: | :--- | :--- |
| 0 | `GATE_NORMAL` | Supervisor regime accepted; the selected bank runs（接受决策层工况，运行所选控制档位） |
| 1 | `GATE_HOLD_LATENCY` | Stale slow mailbox with healthy fast sensors（帧过期但快环传感器有效：报告延迟门并继续闭环 R4）；only an unusable fast input holds the last command open-loop（仅快环输入不可用时开环保持上一条指令） |
| 2 | `GATE_VETO_OOD_RESIDUAL` | Physics innovation out of distribution; abstain to R4（物理预测残差超出分布，弃权并切入 R4） |
| 3 | `GATE_VETO_LOW_CONFIDENCE` | Confidence below threshold or explicit regime 4; abstain to R4（置信度低于阈值或显式工况 4，弃权并切入 R4） |
| 4 | `GATE_VETO_SAFETY_ENVELOPE` | State envelope or look-ahead exceeded; R4（状态包络或前视检查越界，切入 R4） |
| 5 | `GATE_SAFE_ZERO` | Native HOLD past `max_hold_ticks = 100`: integrators cleared, command ramped to zero in steps of `max(u_slew_max_per_s * dt_s, 1 binary32 ULP(\|u_prev\|))`; the one-ULP floor toward zero applies only when the legal slew budget is below one representable step, only reduces \|u\| and never re-energises（内核级 HOLD（指令保持）超过 `max_hold_ticks = 100` 后：积分器清零，指令按上述步长单调归零；单 ULP 保底步长仅在合法斜率预算不足一个可表示步进时生效，只减小 \|u\|，绝不反向注入能量） |
| 6 | `GATE_LATCHED_SAFE` | Cumulative severe faults (`max_fault_vetoes_before_latch = 3`): permanently in R4 until acknowledged（严重故障累计达 `max_fault_vetoes_before_latch = 3` 次：永久驻留 R4，直到人工确认解除） |

| Bit | Fault |
| :---: | :--- |
| 1<<0 | `LATENCY_TIMEOUT`（帧龄/传输超时） |
| 1<<1 | `OOD_RESIDUAL`（物理预测残差越界） |
| 1<<2 | `LOW_CONFIDENCE`（置信度过低） |
| 1<<3 | `ENVELOPE_BREACH`（状态包络越界） |
| 1<<4 | `NAN_CORRUPTION`（NaN 数据损坏） |
| 1<<5 | `HOLD_TIMEOUT`（保持超时） |
| 1<<6 | `PERMANENT_LATCH`（永久锁存） |

A safety or OOD veto episode increments the latch counter; the third episode latches
`GATE_LATCHED_SAFE`. `Layered_AcknowledgeFaultLatch()` is the only release path: it clears the veto and
permanent latches, resets the episode counter, keeps the lifetime veto counters and the 7-bit
`fault_audit_bitmask` for forensics, and re-arms the recovery cooldown before normal regimes can be
accepted again. A veto or the permanent latch selects R4 with the blend filter bypassed (blend = 1.0),
so emergency damping is not delayed by transfer smoothing.

安全否决或残差否决每发生一次完整事件，锁存计数加一；累计第 3 次即锁存为 `GATE_LATCHED_SAFE`。
唯一的解除路径是调用 `Layered_AcknowledgeFaultLatch()`：它清除否决锁存与永久锁存、把事件计数
归零，但保留终身否决计数和 7 位 `fault_audit_bitmask` 供事后追溯，并重新启动恢复冷却期，
冷却结束后才允许再次接受正常工况。一旦否决生效或永久锁存，控制器直接切入 R4 并旁路 blend
滤波（blend = 1.0），紧急阻尼不会被无扰切换的平滑过程拖延。

The two runtime-assurance layers a derived device must re-calibrate against its own reachable sets
are the signed 1D ZCBF envelope look-ahead (`|x + T * x_dot|` plus the direction-blind instantaneous
guard, the boundary condition `h_dot + h/T >= 0` for `h = x_max - |x|`) and the split-conformal calibration
abstention threshold (`q = min(1, ceil((N+1)(1-alpha))/N)` in `auto_calibrate`). Both are
device-specific: the envelope depends on the actuator's reachable set and detection delay, and the
conformal threshold depends on the commissioning archive.

派生设备必须针对自身可达集重新标定的两层运行时保障是：有符号 1D ZCBF 包络前视（`|x + T * x_dot|`
加方向无关的瞬时保护，即 `h = x_max - |x|` 下的边界条件 `h_dot + h/T >= 0`）与分裂共形校准弃权阈值
（`auto_calibrate` 中的 `q = min(1, ceil((N+1)(1-alpha))/N)`）。两者都与设备相关：包络取决于执行器
可达集与检测延迟，共形校准阈值取决于标定档案。

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

工业 WAAM 要先选定一个可观测的过程量及其因果执行器模型；伺服应用则要先明确驱动侧的
实际单位与接触约束。机器人应把任务语义与基于实测状态的快速闭环接口分开，两层各管各的。
神经调控按设备自身定义状态、输出与时序约束——机械基准里的 PID 增益和力矩限值，
不会因为改了变量名就能直接套用。

## Commission, freeze, tune / 建档、冻结、整定

1. Define the signals, units, nominal model, controller bank and explicit fallback.
2. Collect labeled macro-feature windows for `auto_calibrate(samples, labels)`.
3. Freeze prototypes, distance-decay coefficient and the split-conformal calibration confidence trip/clear levels
   (order `q = min(1, ceil((N+1)(1-alpha))/N)`).
4. Run `auto_field_tune(evaluate, current)` with a device-side safe trial callback.

The callback returns measured cost and constraint validity. Invalid candidates are rejected;
if all candidates are invalid, the current knobs remain in use.
Evaluate both load/stiction and contact behavior; constrain force, veto fraction and R4 occupancy.
Keep the safety-margin scale fixed during automated gain/damping searches.
Gain/damping are bounded to [0.10,4.0] and margin to [0.10,1.0].
The fixed search evaluates six joint candidates, each independently under the same constraints.
A new unit within the same interface can retain the semantic archive.
A new physical domain starts with a new interface and commissioning archive.

中文操作顺序与上文一致：

1. 定义信号、单位、标称模型、控制器组与显式回退。
2. 采集带标签的宏观特征窗口，调用 `auto_calibrate(samples, labels)`。
3. 冻结原型、距离衰减系数与置信度阈值。
4. 挂上设备侧的安全试验回调，运行 `auto_field_tune(evaluate, current)`。

回调返回实测代价与约束是否满足：约束不合格的候选直接淘汰；全部候选都不合格时，
沿用当前旋钮。试验要同时覆盖带载/静摩擦与接触工况，并对接触力、否决占比和 R4 驻留
时间设限。自动搜索增益/阻尼期间，安全裕度缩放系数保持冻结。增益/阻尼限在 [0.10,4.0]，
裕度限在 [0.10,1.0]。固定搜索共评估 6 组联合候选，每组都在相同约束下独立判定。
同一接口内更换机台可以沿用已冻结的语义档案；进入全新物理领域则要重新建立接口与
标定档案。

When commissioning, verify the two degradation semantics explicitly: a stale slow mailbox with
healthy fast sensors keeps the measured 1 kHz loop closed on the fixed R4 law (it does not hold the
last command), and only an unusable fast input holds the last command open-loop. The 1 kHz slew path
is binary32-only: the slew limiter uses the binary32 TwoSum error-free transformation, and the core
contains no binary64 keyword.

标定时要显式验证两条降级语义：慢环通信帧过期但快环传感器健康时，实测 1 kHz 回路保持在固定 R4 律上
闭环运行（不保持上一条指令）；只有快环输入不可用时才开环保持上一条指令。1 kHz 斜率路径为纯 binary32：
斜率限幅器使用 binary32 TwoSum 无误差变换，内核不含任何 binary64 关键字。

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
A stale slow mailbox with healthy fast sensors reports `GATE_HOLD_LATENCY` and keeps the measured 1 kHz
loop closed on the fixed R4 law; only an unusable fast input holds the last applied command open-loop.
Past `max_hold_ticks` the core enters `GATE_SAFE_ZERO` and ramps to zero in steps of `max(u_slew_max_per_s * dt_s, 1 binary32 ULP(|u_prev|))`.
R4 uses fixed gains, zero-state error and no feedforward, independent of field tuning.

每个通道各自持有状态、探针、整定参数与源时间戳。只有单位和控制定义完全相同的通道才允许
共享只读配置。耦合约束的评估与各轴回退的协调由适配层负责，核心 C API 保持单轴形态。

帧过期时，保留真实帧龄并显式发送 R4 弃权：确定性回退继续运行，延迟故障也依然可观测。
新帧到达后，要经过恢复冷却期（本参考架构为 350 ms 超时加 300 ms 冷却）才恢复正常决策。
慢环字段畸形同样切入闭环 R4，包络与残差的优先级不受影响。`outer_seq` 非零时，内核用自己的
1 kHz 时钟测量帧龄：新序号开启一份新租约，序号不变则每拍老化一次；冻结或伪造的
`outer_age_s` 无法掩盖慢环通信停滞，而序号回落到 0 会被视为数据损坏，而不是新帧。
慢环帧过期但快环传感器有效时，内核报告 `GATE_HOLD_LATENCY` 并继续按固定 R4 律闭环；只有快环输入不可用才开环保持上一条指令。
超过 `max_hold_ticks` 后内核进入 `GATE_SAFE_ZERO`，按 `max(u_slew_max_per_s * dt_s, 1 binary32 ULP(|u_prev|))` 的步长把指令
严格单调地归零。R4 使用固定增益、零状态误差、零前馈，与现场整定无关。

## Verification / 验证

C99 standalone regression passes 265 checks with zero failures; `gate_template.c` prints PASS.
Python/C `--selftest` replays 6,000 frames with max |du| = 0 in binary32, MATLAB/C equivalence
keeps max |du| = 5e-9 across normal, corrupted-mailbox and illegal-knob runs, a 10,000-tick
`outer_seq` lease replay stays within max |du| = 3.6e-8 (the underlying binary32 values are
bit-identical after text round-trip), and the Level-2
S-Function replays 1,100 frames with all seven outputs at max diff = 0.0.

C99 独立回归 265 项全部通过、零失败，`gate_template.c` 自检打印 PASS。Python/C `--selftest`
回放 6,000 帧，binary32 下 max |du| = 0；MATLAB/C 等价性在正常帧、慢环通信损坏帧与非法旋钮
注入下均保持 max |du| = 5e-9；10,000 拍的 `outer_seq` 租约回放 max |du| = 3.6e-8（文本往返后 binary32 值逐位一致）；
Level-2 S-Function 回放 1,100 帧，全部 7 路输出 max diff = 0.0。

Commands and exact API semantics are in [AGENTS.md](../AGENTS.md).
Numerical evidence is maintained in [README.md](../README.md) / [README_zh.md](../README_zh.md).

> **适用范围：** 本仓库为个人开发者维护的非认证工程参考架构；真实设备集成须由部署方独立完成危险分析与安全验证，并保留独立于本软件的硬件安全保护（E-stop、STO、硬件电荷平衡限制器，详见根目录 [README_zh.md 许可与引用](../README_zh.md#许可与引用)）。应用映射是适配方向，不是硬件、临床或功能安全验证或认证，亦不替代 IEC 61508、ISO 13849、ISO 26262 或 IEC 62304 等标准下的认证与生命周期过程要求。

> **Scope:** This repository is an uncertified engineering reference architecture maintained by an independent developer; real-equipment integration requires deployer-owned hazard validation and independent hardware safety protection (E-stop, STO, hardware charge-balance limiters; see the root [README.md License and citation](../README.md#license-and-citation)). The application mappings are directions for adaptation, not hardware, clinical or functional-safety verification or certification, and they do not replace certification or lifecycle processes under IEC 61508, ISO 13849, ISO 26262 or IEC 62304.
