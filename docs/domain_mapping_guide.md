# Domain mapping / 领域映射

The architecture is a generic division of control authority.
Industrial WAAM / PLC servo, Edge VLA / fast-slow robotics and closed-loop aDBS / SCS
are three equal application examples. They do not delimit its use.
The mechanical benchmark provides one common test fixture; application names do not turn that
fixture into a process model, robot digital twin or physiological model.

本架构提供通用的控制权限分工。工业 WAAM／PLC 伺服、边缘具身智能、
植入式闭环神经调控是并列实例。移植以物理接口为起点，不以领域名称替换为起点。

## The interface / 接口

| Contract | Device-specific definition |
| :--- | :--- |
| `x, x_dot` | Measured/estimated controlled state and rate, with explicit units |
| `r, r_dot` | Reference and reference rate in the same units |
| `u_ff` | Deterministic commissioned-model feedforward |
| `physics_residual` | Dimensionless independent nominal-physics innovation |
| `ai_regime, ai_confidence` | Normal regime 1–3 or explicit abstention 4, and confidence |
| `outer_age_s` | Actual age of the source frame, updated every fast tick |
| `FieldTuning_t` | Gain, damping and commissioned safety-margin scales |

Define the actuator mapping, state envelope, slew bounds and fallback objective together.
Nominal-model mismatch informs the probe; the classifier cannot redefine that evidence.
Choose dimensionless macro-features from measured signals and commission their scales.
No deployed mode decision may use benchmark stage labels or scheduled fault times.

执行器映射、状态包络、斜率限制与回退目标应一起定义。
名义模型残差由独立探针计算，分类器不能改变这份物理证据。
宏观特征来自可测信号，尺度通过设备标定确定。
部署路径不读取阶段真值或预定故障时刻。

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
神经调控按设备定义状态、输出和时序约束，不能将机械增益和力矩阈值直接改名套用。

## Commission, freeze, tune / 建档、冻结、整定

1. Define the signals, units, nominal model, controller bank and explicit fallback.
2. Collect labeled macro-feature windows for `auto_calibrate(samples, labels)`.
3. Freeze prototypes, distance-decay coefficient and confidence thresholds.
4. Run `auto_field_tune(evaluate, current)` with a device-side safe trial callback.

The callback returns measured cost and constraint validity. Invalid candidates are rejected;
if all candidates are invalid, the current knobs remain in use.
Keep the safety-margin scale fixed during automated gain/damping searches.
A new unit within the same interface can retain the semantic archive.
A new physical domain starts with a new interface and commissioning archive.

每次候选试验返回实测误差与约束是否满足；不合格候选不能被选中，
全部不合格时保留当前旋钮。自动增益／阻尼搜索保持已确定的安全裕度不变。
同场景换机台可冻结语义档案，全新物理领域先建立新接口与标定档案。

## Multi-axis and timeout / 多轴与超时

Each channel owns its state, probe, tuning and source timestamp.
Share read-only configurations only for channels with identical units and control definitions.
The adapter evaluates coupled constraints and coordinates fallback.
The core remains a single-axis API.

On a stale frame, preserve actual age and send explicit R4 abstention.
This lets deterministic fallback run while the latency fault remains observable.
Native C HOLD preserves the applied command; its physical effect depends on the device.
Restore normal decisions only after fresh frames satisfy the recovery conditions.

每通道独立持有状态、探针、旋钮与源时间戳。
只有单位和控制定义相同的通道才能共享只读配置。
适配层处理耦合约束与协调回退，C API 保持单轴形式。
超时后保留真实帧龄并显式发送 R4；新鲜帧经过恢复条件后才重新启用正常模态。

Commands and exact API semantics are in [AGENTS.md](../AGENTS.md).
Numerical evidence and the scope statement are maintained in
[README.md](../README.md) / [README_zh.md](../README_zh.md).
