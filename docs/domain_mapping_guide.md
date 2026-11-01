# Cross-Domain Mapping Guide — Layered AI Control in Three Real Deployment Targets

**Bilingual document / 中英双语文档**

| Part | English | 中文 |
| :--- | :--- | :--- |
| 0 | One skeleton, three parameter sets | 同一副骨架，三套参数 |
| 1 | Industrial real-time control (WAAM wire-feed axis / high-precision servo) | 工业实时控制（WAAM 送丝轴 / 高精度伺服） |
| 2 | Edge embodied AI (10 Hz VLA over a 1 kHz compliant arm) | 边缘具身智能（10 Hz VLA 控制 1 kHz 柔顺机械臂） |
| 3 | Closed-loop neuromodulation (aDBS / SCS) | 闭环神经调控（aDBS / SCS） |
| 4 | Cross-domain isomorphism table | 跨域同构映射总表 |
| 5 | Porting checklist | 移植检查清单 |
| 6 | Common misuse, per domain | 各域常见误用 |

Every number quoted in this guide is a **shipped constant** of
`matlab_simulink/load_domain_profile.m` (`'industrial'`, `'edge_vla'`, `'neuromodulation'`), or a
gatekeeper constant of `c99_layered_core/layered_gatekeeper.h`. If a number here disagrees with those two
files, the code wins — those files are the ground truth (see `AGENTS.md` §2).

本文引用的所有数字都来自 `matlab_simulink/load_domain_profile.m` 的三套预设或
`c99_layered_core/layered_gatekeeper.h` 的门控常数。若与代码不一致，以代码为准（真值地图见 `AGENTS.md` §2）。

---

## Scope and disclaimer / 适用范围与免责声明

**Scope and disclaimer.** This document is a conceptual mapping guide for a **reference implementation and
proof-of-concept benchmark**. Every number quoted here comes from a **parameterised synthetic nonlinear
plant** (second-order drive, Stribeck friction, sensor noise, drift, edge-compute latency, injected OOD
mode); the three domains reuse that same skeleton with different units and magnitudes. No clinical patient
data and no production-machine data are used, and nothing here has been validated on hardware. The C core
borrows defensive embedded-programming principles (zero dynamic allocation, bounded execution steps,
input sanitisation, fault-bit logging) as a structure reference; the code in this repository is **not**
certified functional-safety or medical-device firmware. Before applying this mapping to a real
industrial, robotic or neuromodulation system, run your own FMEA/STPA, hardware-in-the-loop testing and
the regulatory certification your domain requires.

**适用范围与免责声明。** 本文件是**参考实现与概念验证基准**的概念映射指南。文中所有数字都来自**参数化合成非线性
被控对象**（二阶传动、Stribeck 摩擦、传感器噪声、漂移、边缘算力时延、注入式 OOD 模态）；三个目标域只是同一骨架在
不同单位与量级下的复用。不含任何真实患者临床数据或产线机台数据，也没有在硬件上验证。C 核心借鉴防御式嵌入式编程
原则（零动态分配、有界执行步数、输入清洗、故障位记录）作为结构参考，但本仓库代码**不是**经过认证的功能安全或医疗
器械固件。把本映射用于真实工业/机器人/神经调控系统之前，必须自行完成 FMEA/STPA、硬件在环测试以及所在领域的法规认证。

---
## Part 0 — One skeleton, three parameter sets / 同一副骨架，三套参数

### 0.1 English

All three domains instantiate **the same second-order plant with non-smooth Stribeck friction** and the
**same layered controller**; only the units and the magnitudes change:

```text
J·ẍ + b·ẋ + k·x + F_friction(ẋ) + d_ood(t) = u
F_friction(ẋ) = f_visc·ẋ + f_c·tanh(ẋ/v_ε) + (f_s − f_c)·exp(−|ẋ|/v_s)·tanh(ẋ/v_ε)     [v_ε = 0.02·v_s]
```

| Layer | Invariant across domains | Domain-specific |
| :--- | :--- | :--- |
| Slow loop (10 Hz) | 3-class coarse regime + confidence from 4 normalized features; never emits `u` | what the 3 regimes *mean physically*, the feature scales, the class means |
| Residual probe | `r(t) = \|y − ŷ\|` on a frozen nominal model, fast + slow channels, fast channel vetoes | `res_fast_scale` (state units), the nominal model, the OOD morphology |
| Fast loop (1 kHz) | zero-heap C99 core: firewall → priority chain → dwell → cooldown → anti-windup → bumpless → clamps | gain bank, `u_max`, slew limit, safety envelope, feedforward model |
| Field tuning | 3 scalars orthogonal to the AI | the physical meaning of each knob and its safe range |
| Acceptance | 4 engineering checks (drift, latency, OOD abstention, data/compute) | thresholds that are *engineering* thresholds, not tuned per run |

The shared normalized feature vector (used by the nearest-centroid coarse classifier) is:

| # | Feature | Normalization |
| :---: | :--- | :--- |
| 1 | RMS tracking error | `/ ref.amp` |
| 2 | RMS control effort | `/ act.u_max` |
| 3 | slow residual channel | `/ ai.res_slow_scale` |
| 4 | RMS velocity | `/ ref.v_scale` (= peak velocity of the quintic move) |

The shared gatekeeper timing constants (identical in all three presets) are: outer loop 10 Hz,
heartbeat timeout 0.30 s, residual trip 1.0 / clear 0.80, confidence trip < 0.45 / clear 0.55,
anti-chatter dwell 150 ms, post-fault cooldown 500 ms, anti-windup time constant 50 ms,
bumpless `blend_alpha` 0.10, envelope trip 0.90 / clear 0.75 with 60 ms look-ahead.

### 0.2 中文

三个目标域实例化的是**同一条二阶被控对象 + 同一套非光滑 Stribeck 摩擦**，以及**同一个分层控制器**；
不同的只是单位与量级：

```text
J·ẍ + b·ẋ + k·x + F_friction(ẋ) + d_ood(t) = u
F_friction(ẋ) = f_visc·ẋ + f_c·tanh(ẋ/v_ε) + (f_s − f_c)·exp(−|ẋ|/v_s)·tanh(ẋ/v_ε)     [v_ε = 0.02·v_s]
```

| 分层 | 跨域不变的部分 | 与域相关的部分 |
| :--- | :--- | :--- |
| 慢环（10 Hz） | 3 类粗粒度模态 + 由 4 个归一化特征给出的置信度；绝不输出 `u` | 3 个模态**在物理上各代表什么**、特征尺度、类均值 |
| 残差探针 | 冻结名义模型上的 `r(t) = \|y − ŷ\|`，快/慢双通道，仅快通道触发否决 | `res_fast_scale`（状态量单位）、名义模型、OOD 形态 |
| 快环（1 kHz） | 零堆 C99 核心：防火墙 → 优先级链 → 驻留 → 冷却 → 抗饱和 → 无扰切换 → 钳位 | 增益库、`u_max`、斜率限制、安全包络、前馈模型 |
| 现场整定 | 与 AI 正交的 3 个标量 | 每个旋钮的物理含义与安全范围 |
| 验收 | 四条工程断言（漂移、时延、OOD 弃权、数据/算力） | 属于"工程阈值"，不是逐次调出来的数字 |

慢环分类器共用的归一化特征向量（最近类均值分类器）：

| # | 特征 | 归一化 |
| :---: | :--- | :--- |
| 1 | 跟踪误差 RMS | `/ ref.amp` |
| 2 | 控制量 RMS | `/ act.u_max` |
| 3 | 慢残差通道 | `/ ai.res_slow_scale` |
| 4 | 速度 RMS | `/ ref.v_scale`（五次曲线换段的峰值速度） |

三套预设共用同一组门控时序常数：慢环 10 Hz、心跳超时 0.30 s、残差触发 1.0 / 清除 0.80、
置信度触发 < 0.45 / 清除 0.55、抗乒乓驻留 150 ms、故障后冷却 500 ms、抗饱和时间常数 50 ms、
无扰切换系数 `blend_alpha` 0.10、安全包络触发 0.90 / 清除 0.75、前视 60 ms。

---

## Part 1 — Industrial real-time control / 工业实时控制

**Reference target:** a rotary servo joint driving a WAAM torch carriage, or a high-precision
positioning axis. Thermal growth and wire-feed load shift the effective inertia by up to +35 % between
machines; Stribeck friction at reversal is the dominant non-smooth boundary.
**参考对象：** 驱动 WAAM 焊枪小车的旋转伺服关节，或高精度定位轴。热变形与送丝负载使有效惯量在不同机台间
漂移 +35%；换向时的 Stribeck 摩擦是最主要的非光滑边界。

### 1.1 English

| Item | Mapping |
| :--- | :--- |
| **Controlled variable / states** | `x` = joint position [rad], `ẋ` = velocity [rad/s]; measured by an encoder (σ = 2.5 mrad, derivative low-pass τ = 12 ms). Reference `r` = quintic point-to-point move, amplitude 0.50 rad, 1.50 s move + 1.50 s dwell (the dwell segments are where stiction matters). |
| **Actuator** | motor torque `u` [N·m], `u_max = 2.0 N·m`, slew limit 200 N·m/s, driven through the drive's torque interface. |
| **Slow-loop AI semantics** | `REGIME_NOMINAL (1)` = factory-nominal machine, nominal load; `REGIME_HIGH_LOAD_DAMPED (2)` = thermal growth / heavier feed load, larger effective inertia ⇒ more damping, slightly less proportional action; `REGIME_BOUNDARY_COMPLIANT (3)` = reversal / dwell near a non-smooth friction boundary ⇒ softer P, more D, integral de-emphasized. The AI never sees raw positions — only the 4 normalized features. |
| **Fast-loop control law** | 1 kHz bank of PD+I tracking controllers with deterministic inverse-dynamics feedforward `u_ff = Ĵ·r̈ + b̂·ṙ + k̂·r + F̂_friction(ṙ)` from the **commissioning** model. Gains are derived at load time from the target bandwidth `f_n = 3 Hz`, `ζ = 0.80`, `K_p = J·ω_n²`, `K_d = max(2ζJω_n − b, 0.15Jω_n)`, `K_i = K_p·ω_n/6`, then per-regime relative gains rows (columns `Kp, Kd, Ki`) `[1.00 1.00 1.00] / [1.40 1.80 5.00] / [0.90 1.50 1.30] / [1.00 3.00 0.00]` (the last row = safe fallback: pure PD, no integral). Every controller owns an integrator with tracking anti-windup (`τ = 50 ms`). |
| **Safety-envelope definition** | motion envelope `\|x\| ≤ 1.0 rad`; trip when `\|x\| > 0.90 rad` **or** `\|x\| + 0.06 s·\|ẋ\| > 1.0 rad`; sticky until `\|x\| < 0.75 rad` **and** the prediction recedes. Actuator envelope in the same layer: `\|u\| ≤ 2.0 N·m`, `\|Δu\| ≤ 0.2 N·m` per millisecond. |
| **The three field knobs** | `gain_scale` = servo stiffness for a *specific* machine (↑ for heavier/stiffer coupling; ↓ if you see overshoot or a reversal limit cycle) — a fine trim, not a commissioning substitute; `damping_scale` = anti-hunting aggressiveness (↑ where stiction and backlash are larger, which is exactly the +35 % drift case); `safety_margin_scale` = shrink the motion envelope for new tooling/fixtures (e.g. 0.9 when clearance is smaller than the nominal machine). |
| **Typical OOD forms + residual threshold** | unmodelled parasitic structural resonance (shipped fault: 42 Hz mode, ζ = 0.03, coupled to both `u` and `x`), encoder spikes (0.20 rad outliers at ≈4/s), collision/stall, feed interruption. Threshold advice: the shipped `res_fast_scale = 0.050 rad` with a normalized trip of 1.0 means "abstain when the fast residual exceeds ≈50 mrad ≈ 20 encoder σ" — start from `max(8·σ_sensor, 1.5× the largest benign transient you measure during a reversal)`, then verify that a real collision still trips. Hysteresis is 20 % (clear at 0.80). |

### 1.2 中文

| 项目 | 映射 |
| :--- | :--- |
| **被控量 / 状态量** | `x` = 关节位置 [rad]，`ẋ` = 速度 [rad/s]；编码器测量（σ = 2.5 mrad，微分低通 τ = 12 ms）。参考 `r` 为五次多项式点到点运动，幅值 0.50 rad，1.50 s 运动 + 1.50 s 驻停（驻停段正是静摩擦起决定作用的区间）。 |
| **执行器** | 电机转矩 `u` [N·m]，`u_max = 2.0 N·m`，斜率限制 200 N·m/s，经驱动器转矩接口下发。 |
| **慢环 AI 的分类语义** | `REGIME_NOMINAL (1)`：出厂标定机台、额定负载；`REGIME_HIGH_LOAD_DAMPED (2)`：热变形/送丝负载增大导致等效惯量上升 ⇒ 加大阻尼、略减比例；`REGIME_BOUNDARY_COMPLIANT (3)`：换向或驻停靠近非光滑摩擦边界 ⇒ 软化 P、加强 D、弱化积分。AI 只看 4 个归一化特征，从不接触原始位置信号。 |
| **快环控制律** | 1 kHz 的 PD+I 跟踪控制器库 + 标定模型给出的确定性逆动力学前馈 `u_ff = Ĵ·r̈ + b̂·ṙ + k̂·r + F̂_friction(ṙ)`。增益由目标带宽 `f_n = 3 Hz`、`ζ = 0.80` 在线导出：`K_p = J·ω_n²`、`K_d = max(2ζJω_n − b, 0.15Jω_n)`、`K_i = K_p·ω_n/6`，再乘以逐个模态的相对行（列依次为 `Kp, Kd, Ki`）`[1.00 1.00 1.00] / [1.40 1.80 5.00] / [0.90 1.50 1.30] / [1.00 3.00 0.00]`（最后一行为安全回退：纯 PD、无积分）。**每个**控制器各带积分器并做跟踪抗饱和（`τ = 50 ms`）。 |
| **安全包络定义** | 运动包络 `\|x\| ≤ 1.0 rad`；当 `\|x\| > 0.90 rad` 或 `\|x\| + 0.06 s·\|ẋ\| > 1.0 rad` 时触发；粘滞直到 `\|x\| < 0.75 rad` 且预测值回收。执行器包络在同一层：`\|u\| ≤ 2.0 N·m`，每毫秒 `\|Δu\| ≤ 0.2 N·m`。 |
| **现场整定三旋钮的物理含义** | `gain_scale` = 针对**某台具体机床**的伺服刚度（耦合更重/更硬时增大；出现超调或换向极限环时减小）——是精修，不能替代标定；`damping_scale` = 抗"爬行/极限环"的激进程度（静摩擦与间隙越大越要加，正是 +35% 漂移场景）；`safety_margin_scale` = 为新工装/夹具收缩运动包络（例如间隙比标定机台小时取 0.9）。 |
| **该域 OOD 典型形态与残差阈值建议** | 未建模的寄生结构谐振（仓库内置故障：42 Hz 模态、ζ = 0.03，同时受 `u` 与 `x` 激励）、编码器尖峰（0.20 rad 离群点，约 4 个/秒）、碰撞/堵转、送丝中断。阈值建议：仓库默认 `res_fast_scale = 0.050 rad` 且归一化触发值 1.0，等价于"快残差超过约 50 mrad（约 20 倍编码器 σ）即弃权"；工程上从 `max(8·σ_sensor, 1.5× 换向时实测的最大良性瞬态)` 起步，再验证真实碰撞仍能触发。回差 20%（0.80 清除）。 |

---

## Part 2 — Edge embodied AI / 边缘具身智能

**Reference target:** a 10 Hz vision-language-action (VLA) model supervising a compliant robot joint
whose torque loop closes at 1 kHz, on a power/thermal-limited edge NPU. Payload changes shift link
inertia by +35 %; contact transition and harmonic-drive friction are the non-smooth boundaries.
**参考对象：** 10 Hz 视觉-语言-动作（VLA）模型指挥一条 1 kHz 闭环转矩的柔顺机械臂关节，算力受功耗与散热限制。
负载变化使连杆惯量漂移 +35%；接触切换与谐波减速器摩擦是最主要的非光滑边界。

### 2.1 English

| Item | Mapping |
| :--- | :--- |
| **Controlled variable / states** | joint position `x` [rad] and velocity `ẋ` [rad/s] from the joint encoder (σ = 4.0 mrad); reference = quintic move of 0.60 rad with 1.20 s move + 1.20 s dwell. In the real robot this is a joint-space trajectory segment issued by the task planner, not by the VLA. |
| **Actuator** | joint torque `u` [N·m], `u_max = 12 N·m`, slew limit 1200 N·m/s (harmonic drive, torque-controlled). |
| **Slow-loop AI semantics** | `REGIME_NOMINAL (1)` = free-space motion with a known payload; `REGIME_HIGH_LOAD_DAMPED (2)` = unknown/heavier payload (inertia +35 %) ⇒ damped bank; `REGIME_BOUNDARY_COMPLIANT (3)` = contact / constrained-motion phase ⇒ compliant bank. The VLA's job is to say *which phase the task is in* and *how sure it is*; it is not allowed to say "apply 3.2 N·m now". |
| **Fast-loop control law** | 1 kHz bank on the same structure as Part 1, with `f_n = 4 Hz`, `ζ = 0.85`, the same relative bank rows (columns `Kp, Kd, Ki`); the feedforward is the commissioned rigid-body model of the arm (gravity, Coriolis, friction approximated at commissioning). Contact phase uses the softer bank so that a mis-classified contact transition does not become a stiff collision. |
| **Safety-envelope definition** | joint envelope `\|x\| ≤ 1.0 rad`; trip at 0.90 rad or when `\|x\| + 0.06 s·\|ẋ\|` exceeds the limit; torque envelope `\|u\| ≤ 12 N·m` and the slew limit above; additionally the task-level workspace limit is enforced *outside* this layer, so a mis-classified regime cannot leave the workspace. |
| **The three field knobs** | `gain_scale` = joint stiffness for a new payload class (↑ for heavier payloads, ↓ for fragile objects); `damping_scale` = contact softness/robustness (↑ when contact transitions are abrupt or the end-effector is heavy); `safety_margin_scale` = shrink the joint envelope for a new end-effector or a cluttered cell (e.g. 0.85 when the tool leaves less clearance). |
| **Typical OOD forms + residual threshold** | unexpected rigid contact or a jam, abrupt payload release, camera occlusion / VLA hallucination, NPU thermal throttling (shipped fault: base latency 60 ms + 0–60 ms jitter, 15 % frame drops), encoder spikes (0.22 rad). Unmodelled structural resonance (38 Hz) is the shipped OOD mode. Threshold: `res_fast_scale = 0.060 rad` ⇒ abstain above ≈3.4° of fast residual; in practice set it from the worst benign contact transient (measure it, then ≈1.5–2×) and confirm it still trips on an unexpected contact within the < 10 ms criterion. |

### 2.2 中文

| 项目 | 映射 |
| :--- | :--- |
| **被控量 / 状态量** | 关节位置 `x` [rad] 与速度 `ẋ` [rad/s]（关节编码器，σ = 4.0 mrad）；参考为 0.60 rad 的五次多项式运动（1.20 s 运动 + 1.20 s 驻停）。在真实机器人上，这一段来自任务规划器的关节空间轨迹，**不是** VLA 输出。 |
| **执行器** | 关节转矩 `u` [N·m]，`u_max = 12 N·m`，斜率限制 1200 N·m/s（谐波减速器、转矩控制）。 |
| **慢环 AI 的分类语义** | `REGIME_NOMINAL (1)`：已知负载的自由空间运动；`REGIME_HIGH_LOAD_DAMPED (2)`：未知/更重负载（惯量 +35%）⇒ 阻尼型增益库；`REGIME_BOUNDARY_COMPLIANT (3)`：接触/受限运动阶段 ⇒ 柔顺型增益库。VLA 的职责是判断"任务处于哪个阶段、有多确定"，**不允许**说"现在施加 3.2 N·m"。 |
| **快环控制律** | 与第 1 部分同结构的 1 kHz 增益库，`f_n = 4 Hz`、`ζ = 0.85`，相对增益行相同（列依次为 `Kp, Kd, Ki`）；前馈为标定得到的机械臂刚体模型（重力、科氏力、标定时的摩擦近似）。接触阶段走更软的增益库，使一次被误分类的接触切换不会变成刚性碰撞。 |
| **安全包络定义** | 关节包络 `\|x\| ≤ 1.0 rad`；`\|x\| > 0.90 rad` 或 `\|x\| + 0.06 s·\|ẋ\|` 越限即触发；转矩包络 `\|u\| ≤ 12 N·m` 与上述斜率限制；此外任务级工作空间限制在本层**之外**再设一道，使误分类也不会离开工作空间。 |
| **现场整定三旋钮的物理含义** | `gain_scale` = 针对新负载类别的关节刚度（负载更重加大、易碎物体减小）；`damping_scale` = 接触柔顺/稳健性（接触切换剧烈或末端较重时加大）；`safety_margin_scale` = 换新末端执行器或进入拥挤工位时收缩关节包络（例如 0.85）。 |
| **该域 OOD 典型形态与残差阈值建议** | 意外刚性接触/卡阻、负载突然脱落、相机遮挡或 VLA 幻觉、NPU 热降频（仓库内置故障：基础时延 60 ms + 0–60 ms 抖动、15% 丢帧）、编码器尖峰（0.22 rad）；未建模结构谐振（38 Hz）是内置 OOD 模态。阈值建议：`res_fast_scale = 0.060 rad` ⇒ 快残差超过约 3.4° 即弃权；实际工程上先实测最恶劣的良性接触瞬态，取约 1.5–2 倍，并验证意外接触仍在 < 10 ms 判据内触发。 |

---

## Part 3 — Closed-loop neuromodulation / 闭环神经调控

**Reference target:** a wearable/implantable adaptive deep-brain or spinal-cord stimulator. A 10 Hz
biomarker classifier decides the therapy regime; a 1 kHz deterministic core shapes the stimulation
envelope inside strict clinical bounds; electrode impedance drifts +35 % between patients.
**参考对象：** 可穿戴/植入式自适应脑深部或脊髓刺激器。10 Hz 生物标志物分类器决定治疗模态；1 kHz 确定性核心
在严格临床边界内塑造刺激包络；电极阻抗在患者间漂移 +35%。

> All quantities in this preset are **normalized** (`state/rate/input` are dimensionless) because the
> mapping from biomarker to charge delivery is device- and indication-specific. Keep the normalized
> formulation until you have a certified dosing model.
> 该预设中所有量都是**归一化**的（`state/rate/input` 均为无量纲），因为"生物标志物 → 电荷输出"的映射依器械与
> 适应症而定。在取得经过认证的剂量模型之前，请保持归一化形式。

### 3.1 English

| Item | Mapping |
| :--- | :--- |
| **Controlled variable / states** | normalized biomarker state `x` (dimensionless) and its rate `ẋ` [1/s]; reference = a therapeutic target trajectory (0.40 normalized amplitude, 1.50 s segments). Sensor σ = 0.0030 normalized units. |
| **Actuator** | normalized stimulation authority `u` (dimensionless), `u_max = 12` in normalized units, slew limit 600 units/s. In hardware this maps to pulse amplitude/width/rate — that mapping belongs to the certified dosing layer, not to the AI. |
| **Slow-loop AI semantics** | `REGIME_NOMINAL (1)` = known responder, nominal impedance; `REGIME_HIGH_LOAD_DAMPED (2)` = raised impedance / raised activation threshold ⇒ damped bank (avoid overshoot into side-effect territory); `REGIME_BOUNDARY_COMPLIANT (3)` = operating close to the therapeutic/side-effect boundary ⇒ most conservative normal bank. The classifier sees only normalized macro-features of the past seconds — never a raw LFP/ECAP trace (that would be an end-to-end model in disguise, with no clinical audit path). |
| **Fast-loop control law** | 1 kHz bank, deliberately **low bandwidth** (`f_n = 1.2 Hz`, `ζ = 0.90`, `K_i = K_p·ω_n/7`) because the plant is slow biomarker dynamics; deterministic feedforward from the commissioned patient model; the safe-fallback bank is **pure PD (`K_i = 0`)**, so an abstention can never integrate its way into a runaway stimulation. |
| **Safety-envelope definition** | normalized envelope `\|x\| ≤ 1.0`, trip at 0.90 with a 60 ms look-ahead; stimulation authority `\|u\| ≤ 12`, slew 600 units/s. In firmware the envelope limits are **constants**, not configuration: a device that can be reconfigured into a wider envelope by a software update is not clinically certifiable. |
| **The three field knobs** | `gain_scale` = therapeutic gain (how aggressively the biomarker error drives the envelope) — bounded by tolerability, not by performance; `damping_scale` = envelope smoothing (↑ reduces paresthesia/side-effect transients, at the cost of slower adaptation); `safety_margin_scale` = per-patient safety derating (e.g. 0.8 for a fragile patient or a fresh lead; it scales the envelope limit, it never enlarges it). |
| **Typical OOD forms + residual threshold** | electrode impedance step (lead maturation, fibrosis), motion/sweat artefacts, stimulation-evoked artefacts, connector/lead fault, unmodelled lead resonance (shipped: 33 Hz with the **largest** plant coupling of the three presets, 0.020), artefact spikes of 0.40 normalized units. Unmodelled artefact is expected in every session, so the threshold must tolerate the benign tail: `res_fast_scale = 0.050` normalized with trip 1.0; set the trip above the 99.9th percentile of benign artefacts and below the smallest clinically relevant fault. A false abstention temporarily reduces therapy, so do not set it tight "for safety". |

### 3.2 中文

| 项目 | 映射 |
| :--- | :--- |
| **被控量 / 状态量** | 归一化生物标志物状态 `x`（无量纲）及其变化率 `ẋ` [1/s]；参考为目标治疗轨迹（归一化幅值 0.40，每段 1.50 s）。传感器 σ = 0.0030（归一化）。 |
| **执行器** | 归一化刺激权限 `u`（无量纲），`u_max = 12`（归一化），斜率限制 600 单位/秒。落到硬件上是脉冲幅值/脉宽/频率——该映射属于经认证的剂量层，**不属于** AI。 |
| **慢环 AI 的分类语义** | `REGIME_NOMINAL (1)`：已知应答者、标称阻抗；`REGIME_HIGH_LOAD_DAMPED (2)`：阻抗升高/激活阈值升高 ⇒ 阻尼型增益库（避免过冲进入副作用区间）；`REGIME_BOUNDARY_COMPLIANT (3)`：工作点靠近疗效/副作用边界 ⇒ 最保守的正常增益库。分类器只看过去数秒的归一化宏观特征，**绝不**直接吃原始 LFP/ECAP 波形（那等于把端到端模型伪装成分类器，且没有临床审计路径）。 |
| **快环控制律** | 1 kHz 增益库，带宽**故意做低**（`f_n = 1.2 Hz`、`ζ = 0.90`、`K_i = K_p·ω_n/7`），因为对象是慢速生物标志物动力学；前馈来自标定过的患者模型；安全回退增益库是**纯 PD（`K_i = 0`）**，因此一次弃权不可能通过积分累积演变成刺激失控。 |
| **安全包络定义** | 归一化包络 `\|x\| ≤ 1.0`，0.90 触发并带 60 ms 前视；刺激权限 `\|u\| ≤ 12`，斜率 600 单位/秒。落地时这些包络限制是**固件常量**而非配置文件——能被软件升级放宽的包络不具备临床可认证性。 |
| **现场整定三旋钮的物理含义** | `gain_scale` = 治疗增益（生物标志物误差驱动包络的激进程度），上界由耐受性而非"性能"决定；`damping_scale` = 包络平滑度（加大可减少感觉异常/副作用瞬态，代价是适应变慢）；`safety_margin_scale` = 逐患者安全降额（体弱患者或新电极取 0.8；它只会把包络**缩小**，绝不放大）。 |
| **该域 OOD 典型形态与残差阈值建议** | 电极阻抗阶跃（导线成熟、纤维化）、运动/汗液伪迹、刺激诱发伪迹、接头/导线故障、未建模导线谐振（内置：33 Hz，且是三个预设中与对象耦合**最强**的，0.020）、0.40 归一化单位的伪迹尖峰。伪迹在每次治疗中都会出现，阈值必须容忍良性尾部：`res_fast_scale = 0.050`（归一化）配触发值 1.0；工程上把触发值设在良性伪迹的 99.9 分位之上、最小临床相关故障之下。**误弃权会减少治疗时间**，不要为"保险"而调得过紧。 |

---

## Part 4 — Cross-domain isomorphism table / 跨域同构映射总表

### 4.1 English

| Dimension | Industrial (WAAM / servo) | Edge VLA arm | Neuromodulation (aDBS / SCS) |
| :--- | :--- | :--- | :--- |
| State | joint angle [rad] | joint angle [rad] | normalized biomarker state |
| Actuator | motor torque [N·m] | joint torque [N·m] | normalized stimulation authority |
| Actuator limits | 2.0 N·m, 200 N·m/s | 12 N·m, 1200 N·m/s | 12 norm., 600 norm./s |
| Envelope | 1.0 rad, trip 0.90, clear 0.75 | 1.0 rad, trip 0.90, clear 0.75 | 1.0 norm., trip 0.90, clear 0.75 |
| Slow-loop rate | 10 Hz | 10 Hz | 10 Hz |
| Fast-loop rate | 1 kHz | 1 kHz | 1 kHz |
| Regime 1 semantics | nominal machine / load | free-space motion | nominal responder |
| Regime 2 semantics | thermal + load drift (+35 %) | unknown payload (+35 % inertia) | raised impedance / threshold |
| Regime 3 semantics | reversal / stiction boundary | contact / constrained motion | near therapeutic/side-effect boundary |
| Bank (Kp, Kd, Ki) relative | nominal 1.00/1.00/1.00 | nominal 1.00/1.00/1.00 | nominal 1.00/1.00/1.00 |
| Bank for regime 2 | 1.40/1.80/5.00 | 1.40/1.80/5.00 | 1.40/1.80/5.00 |
| Bank for regime 3 | 0.90/1.50/1.30 | 0.90/1.50/1.30 | 0.90/1.50/1.30 |
| Safe fallback bank | 1.00/3.00/0.00 (PD only) | 1.00/3.00/0.00 (PD only) | 1.00/3.00/0.00 (PD only) |
| Feedforward source | commissioning inverse dynamics | commissioned rigid-body model | commissioned patient model |
| Slow-loop bandwidth | 3.0 Hz design BW | 4.0 Hz design BW | 1.2 Hz design BW (deliberate) |
| Observer bandwidth | 4.0 Hz | 5.0 Hz | 1.5 Hz |
| Sensor noise σ | 2.5 mrad | 4.0 mrad | 0.0030 norm. |
| Drift (S2) | J×1.35, f_s×2.4, v_s×0.25 | J×1.35, f_s×2.4, v_s×0.25 | J×1.35, f_s×4.0, v_s×0.12 |
| Edge latency (S3) | 60–120 ms, 15 % drops | 60–120 ms, 15 % drops | 60–120 ms, 15 % drops |
| OOD mode (S4) | 42 Hz resonance, 0.20 rad spikes | 38 Hz resonance, 0.22 rad spikes | 33 Hz resonance (strongest coupling), 0.40 norm. spikes |
| `res_fast_scale` | 0.050 rad | 0.060 rad | 0.050 norm. |
| Residual trip → physical | ≈50 mrad ≈ 20 σ | ≈60 mrad ≈ 15 σ | ≈0.05 norm. ≈ 17 σ |
| Field-knob risk | over-stiffness ⇒ reversal limit cycle | over-stiffness ⇒ rigid contact | over-gain ⇒ side-effect transients |
| Fail-safe action | hold command, safe PD bank | hold command, safe PD bank | hold command, safe PD bank (Ki = 0) |

### 4.2 中文

| 维度 | 工业（WAAM / 伺服） | 边缘 VLA 机械臂 | 神经调控（aDBS / SCS） |
| :--- | :--- | :--- | :--- |
| 状态量 | 关节角 [rad] | 关节角 [rad] | 归一化生物标志物状态 |
| 执行器 | 电机转矩 [N·m] | 关节转矩 [N·m] | 归一化刺激权限 |
| 执行器限制 | 2.0 N·m，200 N·m/s | 12 N·m，1200 N·m/s | 12（归一化），600（归一化）/s |
| 安全包络 | 1.0 rad，触发 0.90，清除 0.75 | 1.0 rad，触发 0.90，清除 0.75 | 1.0（归一化），触发 0.90，清除 0.75 |
| 慢环频率 | 10 Hz | 10 Hz | 10 Hz |
| 快环频率 | 1 kHz | 1 kHz | 1 kHz |
| 模态 1 语义 | 标称机台/负载 | 自由空间运动 | 已知应答者 |
| 模态 2 语义 | 热变形 + 负载漂移（+35%） | 未知负载（惯量 +35%） | 阻抗/阈值升高 |
| 模态 3 语义 | 换向/静摩擦边界 | 接触/受限运动 | 靠近疗效/副作用边界 |
| 增益库（Kp, Kd, Ki）相对值 | 标称 1.00/1.00/1.00 | 标称 1.00/1.00/1.00 | 标称 1.00/1.00/1.00 |
| 模态 2 增益库 | 1.40/1.80/5.00 | 1.40/1.80/5.00 | 1.40/1.80/5.00 |
| 模态 3 增益库 | 0.90/1.50/1.30 | 0.90/1.50/1.30 | 0.90/1.50/1.30 |
| 安全回退增益库 | 1.00/3.00/0.00（纯 PD） | 1.00/3.00/0.00（纯 PD） | 1.00/3.00/0.00（纯 PD） |
| 前馈来源 | 标定的逆动力学模型 | 标定的刚体模型 | 标定的患者模型 |
| 慢环设计带宽 | 3.0 Hz | 4.0 Hz | 1.2 Hz（有意做低） |
| 观测器带宽 | 4.0 Hz | 5.0 Hz | 1.5 Hz |
| 传感器噪声 σ | 2.5 mrad | 4.0 mrad | 0.0030（归一化） |
| 漂移（S2） | J×1.35，f_s×2.4，v_s×0.25 | J×1.35，f_s×2.4，v_s×0.25 | J×1.35，f_s×4.0，v_s×0.12 |
| 边缘时延（S3） | 60–120 ms，15% 丢帧 | 60–120 ms，15% 丢帧 | 60–120 ms，15% 丢帧 |
| OOD 模态（S4） | 42 Hz 谐振，0.20 rad 尖峰 | 38 Hz 谐振，0.22 rad 尖峰 | 33 Hz 谐振（耦合最强），0.40 尖峰 |
| `res_fast_scale` | 0.050 rad | 0.060 rad | 0.050（归一化） |
| 残差触发值折算 | ≈50 mrad ≈ 20 σ | ≈60 mrad ≈ 15 σ | ≈0.05 ≈ 17 σ |
| 现场旋钮风险 | 刚度过大 ⇒ 换向极限环 | 刚度过大 ⇒ 刚性接触 | 增益过大 ⇒ 副作用瞬态 |
| 失效行为 | 保持指令 + 安全 PD 库 | 保持指令 + 安全 PD 库 | 保持指令 + 安全 PD 库（Ki = 0） |

---

## Part 5 — Porting checklist / 移植检查清单

### 5.1 English

**A. Contract**
- [ ] `LayeredInput_t` field → my signal mapping written down, with units, reviewed before coding.
- [ ] `dt_s` chosen; `min_dwell_ticks` and `ood_recovery_cooldown_ticks` derived from 150 ms / 500 ms.
- [ ] `outer_age_s` computed from a timestamp/sequence number, not from "last value seen".
- [ ] Every quantity carries one unit system; normalization factors documented.

**B. Deterministic core**
- [ ] Core compiled on the target toolchain; no heap, no `math.h`, no recursion in the 1 kHz path.
- [ ] Shipped regression test green on the host: `SUMMARY: n_pass=105, n_fail=0`.
- [ ] Target port re-proves: `|u| ≤ u_max`, `|Δu| ≤ slew_max·dt_s` on **every** tick, `blend ∈ [0,1]`.
- [ ] NaN/Inf/out-of-range frame ⇒ hold + `NAN_CORRUPTION` bit, integrators untouched.
- [ ] Stale heartbeat ⇒ `GATE_HOLD_LATENCY`, command bit-identical to the last applied one.
- [ ] Sticky `fault_audit_bitmask` persisted (flash/log) and readable by a service tool.

**C. Residual probe + coarse classifier**
- [ ] Nominal plant model frozen at commissioning (no online adaptation of the reference model).
- [ ] Two residual channels with fast/slow split; only the fast channel vetoes.
- [ ] 3–6 dimensionless features; class means from ≤ 50 calibration runs; no raw-signal classifier.
- [ ] Confidence is a margin measure (`d₂/(d₁+d₂)`), not a softmax of an unverifiable logit.
- [ ] Artificial OOD injection ⇒ veto within 10 ms and no envelope breach **in the synthetic benchmark conditions**.

**D. Safety and behaviour**
- [ ] Envelope trip 0.90 / clear 0.75 with 60 ms look-ahead verified against `|ẋ|`.
- [ ] Every non-normal tick carries a gate state or a fault bit (no silent clamping).
- [ ] Bumpless transfer verified across every bank switch (bounded, no ringing).
- [ ] Per-controller anti-windup verified: inactive banks track the applied command.
- [ ] Post-fault cooldown verified: re-arm only after the full consecutive clean window.

**E. Field tuning and release**
- [ ] The three knobs move the command in the expected direction (monotone / D-action / earlier trip).
- [ ] Drift sweep 0 → +50 % keeps tracking flat **with the AI frozen**.
- [ ] No retraining path exists in production; the AI artifact is versioned and read-only.
- [ ] Acceptance command reproducible in CI (fixed seeds; no GUI, no toolbox, no MEX).
- [ ] Figures and metrics archived from the exact release commit.

### 5.2 中文

**A. 契约**
- [ ] 写下 `LayeredInput_t` 字段 → 我的信号的映射（含单位），**先评审再写代码**。
- [ ] 选定 `dt_s`，并由 150 ms / 500 ms 反算出 `min_dwell_ticks` 与 `ood_recovery_cooldown_ticks`。
- [ ] `outer_age_s` 由时间戳/序号计算，而不是"我最后一次看到的值的年龄"。
- [ ] 全链路单位统一；归一化因子有文档。

**B. 确定性核心**
- [ ] 核心在**目标工具链**上编译通过；1 kHz 路径无堆、无 `math.h`、无递归。
- [ ] 仓库自带回归测试在主机上全绿：`SUMMARY: n_pass=105, n_fail=0`。
- [ ] 目标移植重新证明：**每一个** tick 都满足 `|u| ≤ u_max`、`|Δu| ≤ slew_max·dt_s`、`blend ∈ [0,1]`。
- [ ] NaN/Inf/越界帧 ⇒ 保持指令 + `NAN_CORRUPTION` 位，且积分器未被更新。
- [ ] 心跳过期 ⇒ `GATE_HOLD_LATENCY`，且指令与上一次实际下发值**逐位相同**。
- [ ] 粘滞 `fault_audit_bitmask` 已持久化（Flash/日志），维修工具可读。

**C. 残差探针 + 粗分类器**
- [ ] 名义对象模型在标定后**冻结**（参考模型不做在线自适应）。
- [ ] 残差快/慢双通道；只有快通道参与否决。
- [ ] 3–6 维无量纲特征；类均值来自 ≤ 50 次标定；不做原始信号分类器。
- [ ] 置信度是边际量（`d₂/(d₁+d₂)`），不是无法核验的 logit softmax。
- [ ] 人工注入 OOD ⇒ 10 ms 内弃权，且在**合成基准工况下**安全包络零越界。

**D. 安全与行为**
- [ ] 包络触发 0.90 / 清除 0.75、60 ms 前视已用 `|ẋ|` 实测验证。
- [ ] 每个非正常 tick 都有门控状态或故障位（**没有静默钳位**）。
- [ ] 每次增益库切换都验证过无扰切换（有界、无回环）。
- [ ] 逐控制器抗饱和已验证：未激活的增益库跟随实际下发指令。
- [ ] 故障后冷却已验证：必须整段连续干净窗口之后才重新接管。

**E. 现场整定与发布**
- [ ] 三个旋钮的方向性已验证（单调 / D 作用 / 更早触发）。
- [ ] 0 → +50% 漂移扫掠下，**AI 冻结**仍保持跟踪平稳。
- [ ] 生产中不存在重训路径；AI 产物只读且版本受控。
- [ ] 验收命令可在 CI 复现（固定种子；无 GUI、无工具箱、无 MEX）。
- [ ] 图与指标从**发布提交**归档。

---

## Part 6 — Common misuse per domain / 各域常见误用

### 6.1 Industrial / 工业

**English**
1. **Feeding vision/model output straight into a 1 kHz torque command.** That is end-to-end control with
   extra steps: it reintroduces boundary overfitting, drift fragility and unsafe extrapolation. The AI's
   output must stay `{regime, confidence}`.
2. **Using `FieldTuning_t` as a general-purpose gain multiplier.** It is a fine trim (±20–30 %) on top of
   a properly commissioned model and feedforward; it is not a substitute for commissioning, and it must
   never be used to mask a bad feedforward model.
3. **Setting `safety_margin_scale > 1.0` to "unlock more workspace".** That silently removes the physical
   meaning of the envelope; if the machine needs more travel, change the declared envelope and re-validate.

**中文**
1. **把视觉/模型输出直接变成 1 kHz 转矩指令。** 这就是"多加了几步的端到端控制"，会把边界过拟合、漂移脆弱性
   与不安全外推全部带回来。AI 的输出必须始终只有 `{模态, 置信度}`。
2. **把 `FieldTuning_t` 当成万能增益倍数。** 它是在**已正确标定**的模型与前馈之上的精修（±20–30%），
   不能替代标定，更不能用来掩盖糟糕的前馈模型。
3. **把 `safety_margin_scale` 设成 > 1.0 来"解锁更多行程"。** 这会悄悄抽掉包络的物理含义；若机台确实需要更大行程，
   应正式修改声明的包络并重新验证。

### 6.2 Edge VLA / 边缘 VLA

**English**
1. **Calling the VLA from the real-time thread / blocking on Python.** GIL contention and transport jitter
   turn the 1 kHz loop's phase margin into a lottery. The VLA must be a separate process communicating
   through a timestamped mailbox.
2. **Using the VLA's own confidence as the only OOD check.** A hallucinating policy is confident; the
   independent residual probe is what makes abstention meaningful.
3. **Retraining the VLA per robot, fixture or payload class.** The architecture expects three scalar knobs
   per deployment; if a new fixture needs retraining, the feature normalization or the bank is wrong.

**中文**
1. **在实时线程里调用 VLA / 阻塞等 Python。** GIL 争用与传输抖动会把 1 kHz 环的相位裕度变成抽奖。VLA 必须是
   独立进程，只通过带时间戳的邮箱通信。
2. **只拿 VLA 自身的置信度当 OOD 判据。** 产生幻觉的策略照样"自信"；独立的残差探针才让弃权有意义。
3. **每换一台机器人/夹具/负载类别就重训 VLA。** 本架构要求每个部署只有三个标量旋钮；如果换夹具需要重训，
   说明特征归一化或增益库本身出了问题。

### 6.3 Neuromodulation / 神经调控

**English**
1. **Letting the AI write a continuous stimulation amplitude.** No gate, no envelope, no audit trail — this
   is not a deployable clinical design, regardless of how good the offline metrics look.
2. **Auto-clearing the envelope/abstention latch, or not persisting the fault bits.** The sticky latch and
   the audit bitmask *are* the clinical incident report; clearing them automatically destroys the evidence
   and can silently re-arm stimulation after a fault.
3. **Retraining the classifier per patient, or setting the residual trip so tight that normal artefacts abort
   therapy.** Use the three field knobs for patient adaptation, and set the trip above the benign artefact
   tail (99.9th percentile) — false abstentions reduce therapy time and are a real clinical harm.

**中文**
1. **让 AI 直接输出连续刺激幅值。** 没有门控、没有包络、没有审计链——无论离线指标多好，这都不是可部署的临床设计。
2. **自动清除包络/弃权粘滞位，或不持久化故障位。** 粘滞锁存与故障位掩码**就是**临床事件报告；自动清除等于销毁证据，
   并可能在故障后悄悄重新开启刺激。
3. **逐患者重训分类器，或把残差触发调得过紧以致正常伪迹就中断治疗。** 患者适配用三个现场旋钮；触发值应高于良性伪迹尾部
   （99.9 分位）——误弃权会减少治疗时间，是真实的临床伤害。

---

## Part 7 — Where these numbers come from / 这些数字从哪里来

| What you need | Where it lives |
| :--- | :--- |
| all three domain presets (units, plant, limits, drift, latency, OOD) | `matlab_simulink/load_domain_profile.m` |
| the 4-regime / 5-gate contract, `FieldTuning_t`, firewall and priority order | `c99_layered_core/layered_gatekeeper.h` |
| the per-tick execution order and hysteresis behaviour | `c99_layered_core/layered_gatekeeper.c` |
| how the presets are used in closed loop (observer, features, classifier) | `matlab_simulink/sim_core_engine.m` |
| sample complexity / drift / compute evidence | `matlab_simulink/run_efficiency_analysis.m` |
| the four engineering checks and the figures | `matlab_simulink/run_all_benchmarks.m` |
| reproducible seeds | four-stage benchmark `seed = 7`, efficiency study `seed = 17`, C regression test uses no `rand()` (see `AGENTS.md` §7) |
| the rules an AI assistant must not break | `AGENTS.md` |

**中文提示：** 修改任何域参数时，只改 `matlab_simulink/load_domain_profile.m` 中对应的 preset，
不要在 `sim_core_engine.m` 或 `run_efficiency_analysis.m` 里写域相关常数；改完重跑 `AGENTS.md` §4 的验收命令。
