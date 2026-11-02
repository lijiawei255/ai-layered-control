# AGENTS.md — Machine-Readable Ground Truth for an Agent-Friendly Repository

**Repository:** `ai-layered-control` — reference architecture for *slow coarse AI + fast deterministic
control*, with a deployable zero-dependency C99 core and a MATLAB/Simulink benchmark.
**Audience:** AI coding assistants (agentic IDE and CLI assistants alike) and the
engineers who point them at this repository.
**Purpose of this file:** describe the architecture's **non-negotiable invariants**, the **plain-text
files that are the ground truth**, and the **exact commands** that prove you reproduced it.
Reference implementation and proof-of-concept benchmark only — see **§9 Scope and disclaimer**.

> **Read this file before editing anything.**
> Everything you need is plain text. The `.slx` model is a visualization artifact, never the source of
> truth. Do not try to parse it.

---

## 0. TL;DR — the 60-second version

| Question | Answer |
| :--- | :--- |
| What is this? | A layered closed-loop architecture: a **10 Hz coarse AI classifier** that only picks one of **3 semantic regimes**, an **independent physics-residual probe** that can **abstain**, and a **1 kHz zero-heap C99 gatekeeper + controller bank** that actually owns the actuator. |
| What is *not* here? | The **layered path** has no continuous `u(t) = π_θ(x(t))` policy and never lets AI touch `u`/`u_ff`; the benchmark additionally runs a trained RBF end-to-end baseline for comparison. No committed weights or datasets, no MEX, no toolbox, no `.mat` files, no large binaries. |
| Where is the truth? | `c99_layered_core/layered_gatekeeper.h` → `.c` → `matlab_simulink/sim_core_engine.m` → `matlab_simulink/load_domain_profile.m` (see §2). |
| How do I know it works? | One command (§4): the five engineering checks print `[1] … PASS` through `[5] … PASS` and the run ends with `All five engineering checks passed.`; the C test prints `SUMMARY: n_pass=116, n_fail=0`. |
| How do I port it? | Follow the four-step playbook in §3. |

**中文小结：** 本仓库是"粗粒度慢环 AI + 确定性快环 C99 核心"的参考架构。AI 只做 3 类模态分类，
物理残差探针负责 OOD 弃权，1 kHz 门控核心独占执行器。真正的实现全部是纯文本（C99 + MATLAB），
`.slx` 只是可视化产物。先读 `§2` 的文件地图，再按 `§3` 的四步菜谱移植；适用范围与免责声明见 `§9`。

---

## 1. Architectural invariants (violating any one of them means you are no longer running this architecture)

These four invariants are the architecture. Everything else in the repository is an implementation
choice. If a change breaks one of them, the change is wrong even if the checks still pass.

### Invariant 1 — The slow loop classifies, it never generates a continuous command

The 10 Hz AI layer outputs **one discrete label** (`ai_regime ∈ {1, 2, 3, 4}`) and **one scalar
confidence** in `[0, 1]`. It does **not** emit `u(t)`, it does **not** emit `u_ff`, and it does not run
at 1 kHz. It works only on **dimensionless, normalized macro-features** (four of them in the shipped
implementation: normalized RMS tracking error, normalized RMS control effort, normalized slow residual,
normalized RMS velocity) — never on raw sensor streams.

* Enforced by: `LayeredInput_t` (`ai_regime`, `ai_confidence` are the only AI-supplied fields), the
  input firewall that rejects `ai_regime ∉ [1, 4]` and `ai_confidence ∉ [0, 1]` as **corrupted frames**,
  and by the fact that the command path (`lg_pid_output` + `u_ff`) never reads any AI object.
* Consequence you must respect when porting: the AI is allowed to be slow, jittery, and occasionally
  absent. Its only failure mode is *bad input*, which the core handles without falling over.

### Invariant 2 — There is an independent nominal-physics residual probe, and it can abstain

A nominal observer (a **frozen** physical model of the plant, commissioned on the nominal unit) produces
the prediction error `r(t) = |y(t) − ŷ(t)|`. When the residual exceeds a threshold, or when the
classifier's confidence is too low, the system must **explicitly abstain**: latch
`REGIME_SAFE_FALLBACK (4)` and report `GATE_VETO_OOD_RESIDUAL (2)` or `GATE_VETO_LOW_CONFIDENCE (3)`.
Abstention is a *decision with a name*, not a silent clamp.

* Enforced by: `LayeredInput_t::physics_residual`, `max_physics_residual`, `residual_clear_ratio`
  (hysteresis), `ood_latched`, `conf_latched`, the sticky audit bit
  `LAYERED_FAULT_BIT_OOD_RESIDUAL`, and the priority chain inside `Layered_Step` (§1.3, item 2).
* In the MATLAB engine the probe is **three-channel** (`matlab_simulink/sim_core_engine.m`, `local_run_layered`): a **spike channel** (median-5 deviation), a **narrowband HF resonance channel** at the profile OOD mode, and a **slow LP channel** used only as a classifier feature. The gate receives `max(res_fast, res_hf_gated)`; only the spike/HF channels veto.
* Consequence: an OOD event is *supposed* to look like "gate changes state, command stays sane", never
  like "output explodes".

### Invariant 3 — The 1 kHz fast loop is a zero-heap, O(1) deterministic core that owns the actuator

Everything that touches the actuator quantity runs at 1 kHz in `c99_layered_core/layered_gatekeeper.c`:
no `malloc`, no `math.h`, no recursion, no OS calls, no file-static mutable state, only `<stdint.h>`.
Per tick it performs, in this exact order:

| # | Duty | Code anchor |
| :--- | :--- | :--- |
| 0 | **Input firewall** — NaN/Inf/out-of-range IPC frame is rejected; actuator holds; `NAN_CORRUPTION` bit latched; no integrator is touched | `lg_frame_is_corrupt` |
| 1 | **Hard safety-envelope veto** (priority 1) — `\|x\| > trip_ratio·limit` or `\|x\| + predict_s·\|ẋ\| > limit`; sticky until both recede | steps 1 in `Layered_Step` |
| 2 | **OOD residual veto** (priority 2) — sticky with `residual_clear_ratio` hysteresis | step 2 |
| 3 | **Low-confidence / explicit-abstention veto** (priority 3) — sticky; clears only above `confidence_clear_ratio` **and** a normal regime | step 3 |
| 4 | **Heartbeat watchdog** (priority 4) — stale `outer_age_s > outer_timeout_s` ⇒ `GATE_HOLD_LATENCY`, actuator frozen | step 4 |
| 5 | **Anti-chatter minimum dwell** — a transition between normal regimes (1/2/3) only after `min_dwell_ticks` | step 5 |
| 6 | **Post-fault recovery cooldown** — after a veto clears, the AI is re-armed only after `ood_recovery_cooldown_ticks` **consecutive clean** ticks | step 5 |
| 7 | **Controller bank + per-controller anti-windup + bumpless transfer + magnitude/slew clamps** | step 6 |
| 8 | **Sticky fault audit bitmask** — the black box that explains the last incident without AI introspection | `fault_audit_bitmask` |

Non-negotiables inside this layer:

* **Every** bank controller keeps its own integrator, and **every** integrator is continuously
  back-calculated toward the *actually applied* (clamped) command — so an inactive controller can never
  hide windup and a later bank switch releases no energy.
* Bumpless transfer is a first-order follow with `blend_alpha`; a bank switch restarts it, so the command
  step is bounded by `blend_alpha·u_max + slew_max·dt_s`.
* The command is clamped in **magnitude first, then slew rate**, every single tick.
* `u_ff` is a **deterministic feedforward** from the commissioning model / trajectory generator. It is
  part of the white-box core. It is never produced by the AI.

### Invariant 4 — Field tuning is orthogonal to AI weights (zero retraining across units)

Moving the same firmware to a new machine or a new patient is **three scalars**, not a retraining job:

```c
typedef struct {
    float gain_scale;          /* scales all P/I gains,  1.0 = factory nominal */
    float damping_scale;       /* scales all D gains,    1.0 = factory nominal */
    float safety_margin_scale; /* scales the safety limit, 1.0 = factory nominal */
} FieldTuning_t;
```

The AI stays frozen; the operator adjusts three physical knobs; the deterministic bank absorbs the
uncalibrated drift. `test_gatekeeper_standalone.c` stage 5 proves all three knobs actually move the
command (gain ↑ ⇒ steady-state `|u|` ↑; damping ↑ ⇒ command moves along `+ė`; margin ↓ ⇒ envelope trips
earlier).

### 1.5 Anti-patterns that silently break the architecture

| If a change is about to … | Why it breaks the architecture |
| :--- | :--- |
| make the AI output `u(t)` at 100 Hz–1 kHz | Invariant 1 — you are back to end-to-end control; the drift/latency/OOD failure modes return |
| let the AI output `u_ff` | Invariant 1/3 — the plant-inverse term must stay white-box and commissioned, never learned |
| replace the residual probe with "AI confidence only" | Invariant 2 — confidence is not evidence; a mis-calibrated classifier is confident |
| clamp the output without reporting a gate state | Invariant 2/3 — silent clamping destroys debuggability and the audit trail |
| delete the dwell timer or the recovery cooldown because the benchmark is faster without it | Invariant 3 — you get boundary ping-pong and immediate re-arm after a fault |
| retrain / fine-tune the model per machine or per patient | Invariant 4 — the entire point is three scalar knobs and a frozen model |
| add `malloc`, `math.h`, floating-point library calls, or dynamic dispatch to the core | Invariant 3 — WCET is no longer O(1) |

**中文小结（四大不变量）：**
① 慢环只做 3 类模态分类，绝不输出连续 `u(t)`；② 必须有独立的名义物理残差探针 `|y−ŷ|`，OOD 或低置信度时**显式弃权**；
③ 快环由零堆、O(1) 的 C99 门控独占执行器（NaN/IPC 防火墙 → 优先级链 → 抗乒乓驻留 → 故障冷却 → 逐控制器抗饱和 → 无扰切换 → 幅值/斜率钳位）；
④ 现场整定只有三个标量旋钮（`gain_scale` / `damping_scale` / `safety_margin_scale`），与 AI 权重正交，跨机台/跨患者免重训。

---

## 2. Plain-text ground truth file map

### 2.1 Read in this order (≈4,900 lines total, all plain text)

| Order | File | Why you read it | What you get |
| :---: | :--- | :--- | :--- |
| 1 | `c99_layered_core/layered_gatekeeper.h` | the **contract** | 4 regimes, 5 gate states, `FieldTuning_t`, `LayeredConfig_t`, `LayeredInput_t`/`LayeredOutput_t`, `LayeredState_t`, 5 fault-audit bits, and the full 0→7 semantics of `Layered_Step` |
| 2 | `c99_layered_core/layered_gatekeeper.c` | the **execution order** | what actually happens per tick, in which order, with which hysteresis; the only mutable state is the struct you pass in |
| 3 | `matlab_simulink/sim_core_engine.m` | the **closed-loop meaning** | the 3-branch / 4-stage benchmark and a line-by-line MATLAB mirror of the C core (`local_core_step`), plus the residual probe, observer and coarse classifier |
| 4 | `matlab_simulink/load_domain_profile.m` | the **physical numbers** | every constant for the three domains, in real units: inertia, friction, actuator limits, safety envelope, noise, drift, latency, OOD fault, gatekeeper timings, classifier class means |
| 5 | `c99_layered_core/test_gatekeeper_standalone.c` | the **behavioral spec** | 116 C checks across 9+ stages: priority chain, dwell, cooldown, firewall, anti-windup, `FieldTuning_t`, feedforward, confidence latch |
| 6 | `matlab_simulink/run_all_benchmarks.m` | the **acceptance gate** | the five engineering checks and the three 240 dpi figures |
| 7 | `matlab_simulink/run_efficiency_analysis.m` | the **data/compute claims** | sample complexity (Panel A), zero-retraining drift sweep (Panel B), compute/memory footprint (Panel C) |
| 8 | `matlab_simulink/build_simulink_model.m` | the **`.slx` definition** | the model is generated from this file; this is what the `.slx` actually contains |

Supporting reading (all plain text): `README.md` (English overview), `README_zh.md` (中文总览),
`docs/systems_engineering_manifesto_en.md` / `docs/systems_engineering_manifesto_zh.md` (why this beats
single-point AI optimization), `docs/domain_mapping_guide.md` (cross-domain mapping for industrial,
edge-VLA and neuromodulation targets).

### 2.2 Directory map

```text
ai-layered-control/
├── AGENTS.md                          # this file
├── README.md / README_zh.md           # human overview (EN / 中文)
├── docs/
│   ├── systems_engineering_manifesto_en.md
│   ├── systems_engineering_manifesto_zh.md
│   └── domain_mapping_guide.md        # cross-domain isomorphism + checklist (bilingual)
├── c99_layered_core/                  # deployable core: C99, zero dependency, no heap
│   ├── layered_gatekeeper.h           # contract      (read first)
│   ├── layered_gatekeeper.c           # implementation (read second)
│   └── test_gatekeeper_standalone.c   # 116-check regression test
├── matlab_simulink/                   # benchmark + domain presets + model builder
│   ├── load_domain_profile.m
│   ├── sim_core_engine.m
│   ├── run_efficiency_analysis.m
│   ├── build_simulink_model.m
│   ├── run_all_benchmarks.m           # one-click acceptance run
│   ├── calibrate_coarse_classifier.m
│   ├── train_continuous_policy.m
│   ├── verify_core_equivalence.m
│   └── ai_layered_control_bench.slx   # generated artifact (do not treat as source)
├── python_bench/
│   ├── run_benchmark.py               # ctypes C core or NumPy fallback
│   └── requirements.txt
└── assets/                            # three 240 dpi figures (produced by run_all_benchmarks)
    ├── benchmark_4stages_showdown.png
    ├── efficiency_and_generalization.png
    └── domain_profiles_comparison.png
```

### 2.3 Do not parse the `.slx` — it is a visualization artifact

`matlab_simulink/ai_layered_control_bench.slx` is **generated** by
`matlab_simulink/build_simulink_model.m` and is committed so a human can open it and look at the three
parallel branches. Practical consequences for an agent:

* The `.slx` is a binary/zip container. Grep, XML readers, `strings`, and diff tools do **not** reveal
  its semantics reliably. If you need to know what the model contains, **read
  `build_simulink_model.m`** — it is the model's source code.
* The model uses **built-in Simulink blocks only** (`Sum`, `Gain`, `Integrator`, `Derivative`,
  `Saturation`, `Transport Delay`, `Zero-Order Hold`, `Sine Wave`, `Fcn`, `Mux`, `Scope`). No MATLAB
  Function blocks, no Stateflow, no toolbox blocks, no MEX — so it opens and runs without a compiler and
  without any toolbox.
* The model is a *sketch* of the architecture (three branches side by side, 20 s horizon). The
  quantitative benchmark lives in `sim_core_engine.m`. Do not try to make the `.slx` reproduce the
  numbers.
* **All parameters are inlined at build time**, so the model has no workspace dependency: after
  `clearvars` you can double-click **Run** in Simulink, or run
  `sim('matlab_simulink/ai_layered_control_bench.slx')` from the repository root.
* Do not hand-edit the model; change `build_simulink_model.m` and regenerate:
  `build_simulink_model()` from `matlab_simulink/`, or just run the acceptance command in §4.

### 2.4 What the repository contains

Plain-text sources — C99 (`c99_layered_core/`), MATLAB (`matlab_simulink/`) and Markdown (`docs/`,
`README*.md`, this file) — plus **one generated Simulink model** and **three generated 240 dpi figures**.
No weights are committed: the Branch 2 RBF is fitted deterministically at run time from a synthetic
nominal-unit rollout, and the coarse-classifier prototypes plus confidence threshold are fitted by
`calibrate_coarse_classifier.m` from three short closed-loop probes. The repository ships sources, one
generated model and three generated figures; if your port produces weights, checkpoints or logs, keep
them outside this repository.

**中文小结（纯文本真值地图）：** 阅读顺序 `.h` → `.c` → `sim_core_engine.m` → `load_domain_profile.m`；
`.slx` 由 `build_simulink_model.m` 生成（参数已内联，`clearvars` 后可直接运行），仅含内置 Simulink 模块，
是可视化产物——**不要**去解析它。仓库内容 = 纯文本源码 + 一个生成的 `.slx` + 三张 240 dpi 图；
RBF 权重与分类器原型都在运行时由确定性标定生成，不提交权重或数据集。

---

## 3. Four-step porting playbook

The four steps are ordered by risk: get the **contract** right before the **code**, and get the **code**
right before the **AI**. Each step lists *what to read*, *what to change*, and *which signals verify it*.

### Step 1 — Freeze the contract (mailbox + units), do not write control code yet

* **Read:** `c99_layered_core/layered_gatekeeper.h` — nothing else. Pay attention to `LayeredInput_t`
  (9 fields), `LayeredOutput_t` (7 fields) and the `Layered_Step` comment block.
* **Change / do:**
  1. Write down, in your project, the mapping *your stack* → `LayeredInput_t`:
     `ai_regime` (1–3, or 4 for self-declared abstention), `ai_confidence ∈ [0,1]`,
     `outer_age_s` (age of the freshest slow-loop frame — you must be able to compute it),
     `physics_residual` (normalized `|y − ŷ|`), `x`, `x_dot`, `r`, `r_dot`, `u_ff`.
  2. Fix the units and normalization once. Every gain, limit and threshold below is meaningless without
     it; mix radians and millimetres here and nothing will work later.
  3. Decide `dt_s` (1 kHz in the reference) and the safety envelope `state_safety_limit` in the same
     units as `x`.
* **Verify (signals):** log the frame for ≥ 10 s of idle operation and assert
  `ai_regime ∈ [1,4]`, `ai_confidence ∈ [0,1]`, `physics_residual ≥ 0`, `outer_age_s ≥ 0`, all fields
  finite, and `outer_age_s` stays below `outer_timeout_s` while the AI is healthy. Also assert
  `step_count == round(T/dt)` and `corrupted_frame_count == 0` for a healthy run.

### Step 2 — Port the 1 kHz core unchanged (this is the boring, deterministic part)

* **Read:** `c99_layered_core/layered_gatekeeper.c` (blocks 0→7) and skim
  `test_gatekeeper_standalone.c` for the expected *behaviors* (not just the numbers).
* **Change / do:**
  1. Keep the algorithm's semantics; adapt only the glue: how you call `Layered_Step` from your 1 ms
     task, how you own `LayeredState_t` (static, one instance per axis), how you write the actuator, how
     you log `LayeredOutput_t`.
  2. Compile on the **target** toolchain, not just the host. The module needs only `<stdint.h>`; no
     linker flags, no libm, no heap.
  3. Keep the sticky fault bits: persist them if the platform has non-volatile storage; they are your
     incident report.
* **Verify (signals):**
  * rebuild and run the shipped regression test — expect `SUMMARY: n_pass=116, n_fail=0` and exit code 0;
  * on the target: `|u| ≤ u_max` and `|u[k] − u[k−1]| ≤ slew_max·dt_s` on **every** tick (not on average);
  * `blend ∈ [0, 1]`; `u` is finite even after injecting NaN/Inf into the mailbox;
  * a deliberate stale heartbeat produces `GATE_HOLD_LATENCY` with a **bit-identical** held command.

### Step 3 — Build only the two things the AI is allowed to do: normalized features + residual probe

* **Read:** `matlab_simulink/sim_core_engine.m` (`local_run_layered`: observer, three-channel residual,
  feature EMAs, `local_classify`), and in `matlab_simulink/load_domain_profile.m` the fields
  `ai.feature_scales`, `ai.prototypes`, `ai.res_fast_scale`, `ai.res_slow_scale`, `ai.lp_tau_s`,
  `ai.window_tau_s`, `observer.*`.
* **Change / do:**
  1. Commission a **nominal** plant model (frozen at deployment) and run a nominal observer on it. The
     residual probe is only as good as this model — but it must stay *nominal*, not adaptive.
  2. Choose 3–6 **dimensionless** features (divide by amplitude, actuator limit, velocity scale …).
  3. Compute the class means from a **small** calibration set. In the shipped study the coarse
     classifier reaches its full deployment accuracy by **N = 40** calibration runs (87.5 % at N = 15);
     the prototypes and threshold are generated by `calibrate_coarse_classifier.m` from three short closed-loop stage probes.
  4. Attach a **margin x proximity confidence** (`margin = (d2-d1)/(d2+d1)`, `proximity = exp(-gamma*d1)`); both ambiguity and novelty collapse the confidence.
* **Verify (signals):** regime stays correct per stage; confidence visibly collapses during the OOD
  stage; the fast residual crosses `max_physics_residual` within a few milliseconds of fault onset
  (the benchmark criterion is veto latency < 10 ms).

### Step 4 — Wire field tuning + the acceptance checks, then freeze the AI

* **Read:** `FieldTuning_t` in the header, stage 5–6 of `test_gatekeeper_standalone.c`, the
  `P.drift.*` / `P.ood.*` / `P.edge.*` blocks of `load_domain_profile.m`, and Panel B of
  `run_efficiency_analysis.m`.
* **Change / do:**
  1. Enter the three field knobs per unit/patient; never touch the class means.
  2. Reproduce the five engineering checks **on your plant** (drift robustness, latency decoupling,
     OOD abstention, data/compute efficiency).
  3. Record the sticky fault bits during commissioning; they become your regression baseline.
* **Verify (signals):** the three knobs move the command in the expected direction (monotone in
  `gain_scale`, D-action in `damping_scale`, earlier trip for smaller `safety_margin_scale`); the drift
  sweep 0 → +50 % keeps RMSE flat **with the AI frozen**; the envelope-breach count stays at zero **in
  the synthetic four-stage benchmark of this repository**.

### 3.1 Stack notes A — industrial real-time control (PLC / C++ servo drive)

* **Where the loops live:** the 1 kHz core belongs in the drive/motion controller or a deterministic
  PLC task (1 ms); many PLCs run 1–4 ms tasks — set `dt_s` accordingly and re-derive the dwell/cooldown
  tick counts (`min_dwell_ticks = round(0.150/dt_s)`, `ood_recovery_cooldown_ticks = round(0.500/dt_s)`).
* **Read / port:** the `.c` file compiles as C++ with a thin `extern "C"` wrapper; keep `LayeredState_t`
  as plain old data inside your axis object. No exceptions, no RTTI, no allocation in the cyclic task.
* **Mailbox:** the classifier usually runs on an edge box, so the mailbox becomes an EtherCAT/PROFINET
  PDO, an OPC UA subscription, or a shared-memory block refreshed at ~10 Hz. Staleness must come from a
  counter/timestamp in the PDO, not from "the last value we saw".
* **Verify:** PDO age histogram, `corrupted_frame_count == 0`, envelope trip at the configured
  `trip_ratio`, dwell/cooldown counters behaving as configured, actuator `|Δu|` bound respected at the
  drive, and `FieldTuning_t` response after a machine swap.
* **Do not:** let the edge box write the torque/velocity setpoint directly; it writes regime + confidence.

### 3.2 Stack notes B — edge embodied AI (10 Hz VLA over a 1 kHz compliant arm, ROS 2 / Python)

* **Where the loops live:** the 1 kHz core is a real-time C++ node (e.g. a `ros2_control` hardware
  interface or a dedicated RT thread); the VLA runs as a **separate** Python process. Never call Python
  from the RT thread; never allocate on it.
* **Mailbox:** a 10 Hz topic (best-effort QoS) carrying `{regime, confidence, stamp, seq}`; derive
  `outer_age_s` from the header stamp so a dropped or late frame becomes a **watchdog age**, not a crash.
  QoS best-effort + a sequence number is exactly the "dropped frame" case the core is built for.
* **Residual probe:** the nominal model is the commissioned rigid-body model of the arm; `res_fast_scale`
  is in state units (the reference uses ≈0.06 rad for this profile), so pick it from your encoder
  resolution and structural compliance, not from the classifier output.
* **Verify:** kill/pause the VLA process during a move — the arm must **hold** (`GATE_HOLD_LATENCY`,
  frozen command), not sag or runaway; then resume and confirm it re-arms only after the cooldown.
  Check that transport jitter of the topic does not appear as chattering in `u`: the 1 kHz loop is
  decoupled from the outer rate by construction.
* **Do not:** put the VLA in the 1 kHz path "just for the last few millimetres"; that is the failure
  mode this architecture removes.

### 3.3 Stack notes C — closed-loop neuromodulation (aDBS/SCS, STM32 / FPGA, µW budget)

* **Where the loops live:** a 1 ms timer ISR on an MCU (Cortex-M4F, single precision) or a decimated
  FPGA pipeline. The core is malloc-free and libm-free on purpose; keep it that way (float32 only,
  no double promotion in the hot path).
* **Mailbox:** a framed UART/SPI/DMA message from the biomarker classifier at 10 Hz
  (`seq`, `timestamp`, `regime`, `confidence`, CRC). A missing frame must increase the age — never block;
  the watchdog converts it into a hold.
* **Clinical envelope:** `state_safety_limit`, `u_max`, `u_slew_max_per_s` are *firmware* constants, not
  configuration files. Latch and persist `fault_audit_bitmask` to flash as the audit trail; require an
  explicit clinician/service reset after `LAYERED_FAULT_BIT_ENVELOPE_BREACH` and
  `LAYERED_FAULT_BIT_OOD_RESIDUAL`.
* **Power:** the AI runs at 10 Hz (or duty-cycled), the core runs at ~1 kHz but is a fixed straight-line
  branch sequence of a few dozen operations — that is the whole point of the layering.
* **Verify:** inject a synthetic electrode-impedance step and an artefact burst: expect an explicit
  abstention within milliseconds, a smooth (bumpless) transition into the safe regulator, and no envelope
  breach under the synthetic benchmark conditions; then confirm the cooldown prevents immediate re-arm.
* **Do not:** ship an "AI-tuned" stimulation amplitude as a continuous function; do not allow the
  classifier to bypass the safety envelope.

**中文小结（四步菜谱）：**
① 先冻结契约与单位（把 `LayeredInput_t` 的 9 个字段接到你的栈上，先只打日志验证）；
② 原样移植 1 kHz 核心，只改调用/执行器/日志的胶水层，先跑通 116 项回归测试；
③ 只做两件 AI 允许做的事——归一化无量纲特征 + 名义残差探针（3–6 维特征；实测粗分类 N=40 次标定即达满精度、N=15 时 87.5%；边际置信度）；
④ 接入现场三旋钮 + 四项工程检查，之后冻结 AI。三种目标栈（PLC/C++ 伺服、ROS2/Python VLA 机械臂、STM32/FPGA 神经调控）的要点见 §3.1–§3.3。

---

## 4. Run and verify (Windows PowerShell, copy-paste ready)

### 4.1 Prerequisites

* **MATLAB with Simulink** (the reference run used **R2024b**). No toolbox, no Simulink Coder, no MEX
  compiler: the model is built from built-in blocks and all analysis is plain MATLAB.
* **Any C99 compiler** for the standalone test (MinGW-w64 `gcc`, clang, MSVC, `arm-none-eabi-gcc`, …).
  If you have no C compiler, skip step 4.3 — the MATLAB run does not need one.
* Run everything **from the repository root**.

### 4.2 Full benchmark + figures + engineering checks (the acceptance run)

```powershell
# from the repository root
matlab -batch "cd('matlab_simulink'); run_all_benchmarks"
```

Equivalent form (explicit script path, also from the repository root):

```powershell
matlab -batch "run('matlab_simulink/run_all_benchmarks.m')"
```

Success criteria — all of them:

| Signal | Expected |
| :--- | :--- |
| five check lines | `[1] S2 drift robustness … PASS`, `[2] S3 latency decoupling … PASS`, `[3] S4 OOD abstention and envelope safety … PASS`, `[4] Paired data and compute efficiency: PASS …`, `[5] Mode necessity: PASS …` |
| final line | `All five engineering checks passed.` |
| timing block | `=== TIMING (wall clock, this MATLAB process) ===` with a `total wall-clock time: <n> s` line; in the reference R2024b session the acceptance run measured **32.56 s** total (1.93 s model, 7.86 s four-stage, 4.18 s efficiency, 7.89 s ablation/probes, 10.69 s cross-domain); a cold `matlab -batch` adds the one-off Simulink/model load |
| figures | `assets/benchmark_4stages_showdown.png`, `assets/efficiency_and_generalization.png`, `assets/domain_profiles_comparison.png` rewritten, 240 dpi (within the 200–300 dpi budget) |
| model | `matlab_simulink/ai_layered_control_bench.slx` rebuilt (built-in blocks only) |
| exit code | `0` — the script raises an error (non-zero exit) if any check fails |

The `[1/5] … [5/5]` lines are progress markers for the five work blocks; the acceptance verdict is the
`[1]`–`[5]` check block that follows them. What the run does, in order: (1) builds the `.slx` and
smoke-tests it for 0.1 s; (2) runs the 40 s four-stage benchmark on the industrial reference profile and
evaluates checks 1–3; (3) runs the data/drift/compute efficiency study and evaluates check 4; (4) runs
all three domain presets and writes the cross-domain figure; (5) prints a Markdown metrics table.

The 40 s benchmark is: `dt = 1 ms`, 3 branches × 4 stages × 10 s —
S1 fast light-load tracking, S2 +35 % load/Stribeck, S3 contact/HF noise/edge latency, S4 unseen OOD resonance and spikes.

### 4.3 C99 core regression test (independent of MATLAB)

```powershell
cd c99_layered_core
gcc -std=c99 -pedantic -Wall -Wextra -O2 -o test_gatekeeper_standalone.exe layered_gatekeeper.c test_gatekeeper_standalone.c
.\test_gatekeeper_standalone.exe
$LASTEXITCODE   # 0 = every check passed
cd ..
```

Success criteria:

| Signal | Expected |
| :--- | :--- |
| every check line of the test log | starts with `[PASS]` (no `[FAIL]`, no `[INV ]` violation report) |
| last line | `SUMMARY: n_pass=116, n_fail=0` |
| `$LASTEXITCODE` | `0` |
| compiler diagnostics | none (the module compiles warning-free with `-pedantic -Wall -Wextra`) |

The test is fully deterministic (no `rand()`, no `<math.h>`; the reference sine is generated with a
Taylor-series rotation matrix), so the same 116 checks and the same numbers reproduce everywhere. It
covers: the 40 s mission schedule, priority chain, anti-chatter dwell, post-fault cooldown, NaN/Inf/
out-of-range firewall, tracking anti-windup across a saturated bank switch, the three `FieldTuning_t`
knobs, deterministic feedforward `u_ff`, the sticky confidence latch, global actuator invariants, and the
API/self-healing/null-pointer paths.

> Cross-toolchain note: the module includes only `<stdint.h>` and uses no libm, so the same flags work
> with `arm-none-eabi-gcc` / IAR / Keil MDK. This repository only exercises the host compiler; treat the
> cross-compile as your own first porting task (see §3.1–§3.3).

### 4.4 Simulink model as a standalone artifact

The builder inlines every numeric parameter, so the committed model has no workspace dependency:

```powershell
matlab -batch "clearvars; sim('matlab_simulink/ai_layered_control_bench.slx'); disp('model runs standalone')"
```

You can equally open the `.slx`, press **Run**, and inspect the three branches (fixed PID, end-to-end AI
with its 100 ms delay + 10 Hz zero-order hold, layered core with hard saturation). This is the
inspectable counterpart of the plain-text implementation, not the quantitative benchmark.

### 4.5 Optional: individual experiments

```powershell
# efficiency / drift / compute study only (also evaluates check [4] and writes figure 2)
matlab -batch "cd('matlab_simulink'); E = run_efficiency_analysis('industrial'); disp(E.check4)"

# one domain preset, verbose per-stage summary
matlab -batch "cd('matlab_simulink'); out = sim_core_engine('neuromodulation', struct('verbose', true));"

# rebuild the Simulink model only
matlab -batch "cd('matlab_simulink'); build_simulink_model();"
```

**中文小结（运行与验收）：** 一条命令 `matlab -batch "cd('matlab_simulink'); run_all_benchmarks"` 完成建模、40 s 四阶段基准、
效率实验、三域对比与三张 240 dpi 图；成功判据是 `[1]`–`[5]` 五项检查全部 `PASS`、末行 `All five engineering checks passed.`、退出码 0。
参考机本次实测 `run_all_benchmarks` 总耗时 **32.56 s**（模型 1.93 s、四阶段 7.86 s、效率 4.18 s、消融/探针 7.89 s、三域 10.69 s）；冷启动 `matlab -batch` 另加一次 Simulink 加载开销。
C 核心用 gcc 编译后运行，判据是 `SUMMARY: n_pass=116, n_fail=0` 且退出码 0。`.slx` 参数已内联，`clearvars` 后可独立运行。

---

## 5. Common pitfalls (things that look harmless and are not)

1. **Never introduce a GUI dependency.** Everything is headless: figures are built with
   `'Visible','off'` and exported with `exportgraphics` at 240 dpi (within the 200–300 dpi budget);
   the entry point is always `matlab -batch`. Adding a dialog, `input()`, `uiwait`, or a live figure
   breaks CI and breaks agents.
2. **The model must stay built from built-in Simulink blocks.** No MATLAB Function blocks, no Stateflow,
   no toolbox blocks, no MEX/S-function. Otherwise opening or running `ai_layered_control_bench.slx`
   suddenly requires a compiler or a paid toolbox — exactly what this model is designed to avoid.
   `build_simulink_model.m` fixes the block list; keep it that way.
3. **Do not hand-edit the generated model.** Every acceptance run rebuilds the `.slx` from
   `build_simulink_model.m`, so hand edits are lost. Change the builder and regenerate instead.
4. **Keep the content textual.** The repository ships sources, one generated model and three generated
   figures. Do not add model weights, datasets or logs; if your port produces them, keep them out of the
   repository.
5. **Keep the AI out of the command path.** `u` and `u_ff` come from the deterministic bank /
   commissioning model only. An AI-written `u_ff` is an end-to-end controller in disguise.
6. **Freeze the AI when you change hardware.** New machine, new patient, new payload ⇒ three
   `FieldTuning_t` scalars. If your fix requires retraining, you have left the architecture.
7. **Do not compile the core with `-ffast-math`.** The NaN/Inf firewall relies on IEEE comparisons;
   fast-math can legally optimize them away. If you need the GCC flags, keep `-ffast-math` off for this
   translation unit.
8. **Do not run two MATLAB sessions against the same model file concurrently.** The build script closes a
   loaded model before saving, but concurrent runs on one `.slx` will race. Serialize CI jobs.
9. **Do not mix units.** Radians vs millimetres, normalized vs physical actuator commands — pick one and
   document it next to `LayeredConfig_t`. All thresholds (`state_safety_limit`, `u_max`,
   `physics_residual`) live in those units.
10. **Do not silently clamp.** If you add a new limit, keep reporting it through a gate state or a fault
    bit so a field engineer can see *why*. The architecture's debuggability depends on "every
    non-normal tick has a name".
11. **Do not remove determinism from the benchmark.** Fixed seeds, no `rand()` in the C test, no
    wall-clock dependence in the metrics. Reproducible numbers are what makes the five engineering
    checks meaningful.

**中文小结（常见坑）：** 只用 `matlab -batch`（不引入任何 GUI 依赖）；模型只用内置 Simulink 模块（无 MEX/无工具箱）；
`.slx` 由 `build_simulink_model.m` 生成，不要手改；仓库内容保持纯文本 + 一个生成模型 + 三张图（分类器只有 16 个数）；
AI 不进入 `u`/`u_ff` 通路；换机台只调三个旋钮；不要用 `-ffast-math`（会破坏 NaN 防火墙）；不要并行跑两个 MATLAB 会话
写同一个 `.slx`；单位不要混用；不要静默钳位（每个非正常 tick 都要有名字）；保持基准的确定性。

---

## 6. Prompt templates (copy-paste)

### 6.1 "Port this architecture into my project"

```text
You are working in my repository. Read ./AGENTS.md from this repository first
(if it is not vendored here, read the reference architecture at ai-layered-control),
then follow its four-step porting playbook in order.

My target stack: <industrial PLC + C++ servo | ROS2 + Python VLA arm | STM32/FPGA closed-loop
neuromodulation>. My sensor/actuator units: <units>. My control rate: <Hz>.

Rules:
1. Do not write control code before you have written down the mailbox contract
   (LayeredInput_t field -> my signal, with units) and shown it to me.
2. The 1 kHz core must stay: no heap, no math.h, O(1), NaN/Inf firewall, priority chain,
   dwell, post-fault cooldown, per-controller anti-windup, bumpless transfer, magnitude and
   slew clamps, sticky fault bits.
3. The AI may only produce a discrete regime (1..3, or 4 = abstain) and a confidence in [0,1],
   from normalized dimensionless features. It must never produce u(t) or u_ff.
4. There must be an independent residual probe r(t) = |y - y_hat| on a frozen nominal model,
   and it must be able to force an explicit abstention.
5. Field adaptation is FieldTuning_t (gain_scale / damping_scale / safety_margin_scale) only;
   never retrain or fine-tune the model.
6. After each step, tell me the exact signals to log and the pass/fail criterion to check,
   and show the evidence (log excerpt, test output, exit code) for every step you claim is done.
```

### 6.2 "Review a change against the four invariants"

```text
Read ./AGENTS.md in this repository (section 1: Architectural invariants) and the reference
implementation files it lists. Then review the diff I am about to commit.

For each of the four invariants, answer explicitly:
  - Is it still satisfied? Cite the specific code lines / signals that prove it.
  - If not, what is the smallest change that restores it?
Also check the mechanical contracts: |u| <= u_max, |du| <= slew_max*dt every tick,
blend in [0,1], u finite after a NaN/Inf frame, held command bit-identical under a stale
heartbeat, dwell and cooldown honoured, every non-normal tick carries a gate state or fault bit.
Report violations as a numbered list with file:line, severity, and the failure mode it creates.
```

### 6.3 "Add a new domain / a fourth regime without breaking the architecture"

```text
Read ./AGENTS.md and docs/domain_mapping_guide.md. I want to add <new domain preset | a new
regime>.

Constraints:
  - Do not change the gate/enum contract in layered_gatekeeper.h unless the new regime can be
    classified by the 10 Hz coarse AI and absorbed by the 1 kHz bank (show me both).
  - Every new physical constant goes into matlab_simulink/load_domain_profile.m with units,
    next to the existing presets, never scattered in the engine.
  - Re-run the acceptance commands from AGENTS.md section 4 and paste the exact output lines
    (the five engineering checks + the C test summary). If a check cannot hold for the new
    domain, say so and explain why instead of loosening the threshold.
```

**中文小结（提示词模板）：** 粘贴 `@AGENTS.md` 后，让助手先读四步菜谱再改代码：① 移植模板（先交契约再写控制代码，AI 只出模态+置信度，
三旋钮免重训）；② 审查模板（逐条对照四大不变量 + 机械契约，给出 file:line 与失败模式）；③ 扩展模板（新增域/新增模态时保持契约不变，
常量进 `load_domain_profile.m`，重跑 §4 验收命令并贴出确切输出）。

---

## 7. Reference numbers (all reproduced by the commands in §4)

| Quantity | Value |
| :--- | :--- |
| Inner-loop rate / sample time | 1 kHz / `dt_s = 0.001 s` |
| Outer-loop rate | 10 Hz (coarse classifier), heartbeat timeout 0.30 s |
| Benchmark horizon | 40 s = 4 stages × 10 s |
| Branches compared | 3 (fixed PID / end-to-end continuous AI / layered) |
| Operating regimes | 4 (`NOMINAL 1`, `HIGH_LOAD_DAMPED 2`, `BOUNDARY_COMPLIANT 3`, `SAFE_FALLBACK 4`); the AI may only emit 1–3 |
| Gate states | 5 (`NORMAL 0`, `HOLD_LATENCY 1`, `VETO_OOD_RESIDUAL 2`, `VETO_LOW_CONFIDENCE 3`, `VETO_SAFETY_ENVELOPE 4`) |
| Field tuning knobs | 3 (`gain_scale`, `damping_scale`, `safety_margin_scale`) |
| Anti-chatter dwell | 150 ms (`min_dwell_ticks = 150` at 1 kHz; 2 frames at 10 Hz in Panel A) |
| Post-fault recovery cooldown | 500 ms (`ood_recovery_cooldown_ticks = 500` at 1 kHz) |
| Residual / confidence hysteresis | residual trip 1.0 → clear 0.80; confidence uses the calibrated `margin x exp(-gamma*d1)` value (industrial commissioning threshold in the 1e-5 range, printed by `calibrate_coarse_classifier`; the C test separately exercises 0.45/0.55 synthetic frames) |
| Safety envelope | trip at 90 % of limit, clear at 75 %, look-ahead 60 ms via `\|ẋ\|` |
| Bumpless transfer | `blend_alpha = 0.10` per tick (regime switch restarts it) |
| OOD veto latency | criterion < 10 ms; measured **2.0 ms** in the shipped run |
| C regression test | **116 checks, 0 failures** (`SUMMARY: n_pass=116, n_fail=0`) |
| Full MATLAB acceptance run | **≈33 s** in this MATLAB session (measured 32.56 s; cold `matlab -batch` adds the one-off Simulink/model load); five engineering checks PASS; three 240 dpi figures; `.slx` rebuilt |
| Sample complexity (Panel A) | Paired nominal-only samples; layered at +35 % drift: 100 % at N = 15 / 40 / 120 / 500 / 2000. Continuous nominal-only at +35 % drift: 3.1 / 0 / 3.1 / 6.2 / **0.0 %**. Drifted-data oracle at +35 %: 59.4 / 71.9 / **100** / 100 / 100 %. Continuous nominal-only at 0 % drift: 78.1 / 84.4 / 96.9 / 100 / 100 %. |
| Compute / memory (analytic, shipped model) | ≈1.0 × 10⁵ fewer MAC/s and ≈2.5 × 10³ less memory than a 1.2 M-parameter policy evaluated at 1 kHz (the shipped check requires at least 10³× compute reduction) |
| Cross-unit drift robustness | 0 → +50 % uncalibrated drift with the AI **frozen** (Panel B of the efficiency study) |
| Reproducibility | four-stage benchmark `seed = 7` (`sim_core_engine.m`), efficiency study `seed = 17` (`run_efficiency_analysis.m`), C regression test uses no `rand()`; identical numbers on the same MATLAB release |
| Domain presets | 3 (`industrial`, `edge_vla`, `neuromodulation`) |

---

## 8. Related documents

| Document | What it answers |
| :--- | :--- |
| `README.md` / `README_zh.md` | the human-facing overview in English / 中文: the motivation, the architecture, the figures, the metrics |
| `docs/systems_engineering_manifesto_en.md` / `docs/systems_engineering_manifesto_zh.md` | the systems-engineering argument: why coarse classification + deterministic control beats single-point AI optimization |
| `docs/domain_mapping_guide.md` | how the same architecture maps onto industrial servo/WAAM, edge VLA arms and closed-loop neuromodulation, with per-domain thresholds, OOD failure modes, a porting checklist and the most common misuse per domain |
| `c99_layered_core/layered_gatekeeper.h` | the contract; read it before asking any question about behavior |
| `matlab_simulink/run_all_benchmarks.m` | the acceptance gate and the metrics table |

### 8.1 Sister project — [jev-tune](https://github.com/lijiawei255/jev-tune)

[jev-tune](https://github.com/lijiawei255/jev-tune) works one level down: it concentrates on
**continuous parameter-layer tuning inside a single controller**, with a C99 safety-boundary fallback
underneath. This repository steps up to the **discrete structural layer** — semantic regime
classification across operating conditions, calibration from a few dozen runs, compute decoupling
between the two time scales, and explicit OOD refusal. Read together, they form one engineering-systems
exploration of closed-loop control from the parameter layer up to the structural layer.

**中文小结（姊妹项目）：** [jev-tune](https://github.com/lijiawei255/jev-tune) 解决"单一控制器内部的连续参数层整定 + C99 安全边界兜底"；
本仓库上移到离散结构层（跨工况语义分类、极小样本标定、算力解耦、OOD 显式拒识）。两者合起来构成
从参数层到结构层的工程系统论闭环控制探索。

---

## 9. Scope and disclaimer

This repository is a **reference implementation and a proof-of-concept benchmark**, not a product and
not a certified component.

* Every experiment runs on a **parameterised synthetic nonlinear plant** — a second-order drive with
  Stribeck friction, sensor noise, cross-unit drift, edge-compute latency and an injected OOD mode. The
  three domain presets (`industrial`, `edge_vla`, `neuromodulation`) reuse that same skeleton with
  different units and magnitudes; they are conceptual mappings, not validated models of a specific
  machine or a specific patient.
* **No real data is included**: no clinical patient data and no production-machine data. Nothing here
  has been validated on hardware.
* The C core borrows defensive embedded-programming principles (zero dynamic allocation, bounded
  execution steps, input sanitisation, fault-bit logging) as a **structure reference** for later
  engineering work; the code in this repository is **not** certified functional-safety or medical-device
  firmware.
* Before using this architecture in a real industrial, robotic or neuromodulation system, run your own
  **FMEA/STPA**, hardware-in-the-loop testing, and the regulatory certification required by your domain.
  The safety statements produced by the benchmark describe the synthetic scenarios defined in
  `matlab_simulink/load_domain_profile.m`, not any physical system.

**中文小结（适用范围与免责声明）：** 本仓库是**参考实现与概念验证基准**，不是产品、也不是经过认证的部件。
所有实验都运行在**参数化合成非线性被控对象**上（二阶传动 + Stribeck 摩擦 + 传感器噪声 + 漂移 + 时延 + 注入式 OOD），
三个域预设只是同一骨架在不同单位下的概念映射；**不含真实患者临床数据或产线机台数据**，未在硬件上验证。
C 核心借鉴防御式嵌入式编程原则（零动态分配、有界执行步数、输入清洗、故障位记录）作为后续工程的结构参考，
但本仓库代码**不是**经过认证的功能安全或医疗器械固件。用于真实工业/机器人/神经调控系统前，必须自行完成
FMEA/STPA、硬件在环测试以及所在领域的法规认证——基准中的安全结论仅针对
`matlab_simulink/load_domain_profile.m` 所定义的合成场景。
