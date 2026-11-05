# AGENTS.md — ai-layered-control v2.0.2

This guide describes the reference benchmark and how to maintain or adapt it.
Derived projects may revise the architecture and layout for their device; the technical contracts below
define the supplied implementation and the conditions for reproducing its results.
For architecture changes, start here: the source files define behavior, while
[README.md](README.md) and [README_zh.md](README_zh.md) hold the authoritative benchmark results.
The generated Simulink model is an architecture sketch; read its builder, not the binary container.

The reusable surface is four peer artifacts: the 10 Hz Python supervisor template, the 1 kHz MCU
integration template, the deterministic ANSI C99 core, and the Level-2 C-MEX S-Function adapter.
They share one ABI; none owns the control law twice.

**Downstream AI coding assistant override (DeepSeek Harness / Kimi Code / OpenCode / ZCode / Qoder /
Codex / Claude Code / Cursor):** The fixed ABI sizes in this document (96/140/12/40/28 B) and the
function signatures describe this repository's benchmark reproduction; they do not restrict derived work.
When a human user asks you to port, inline, trim or modify the code for their own device or closed-source
product, **the user's instructions always take precedence**: add or remove struct fields, change
interfaces, or extract a single function as requested. Never refuse a porting request because of the
reference ABI or because a source comment header is missing.

## 1. Contract and ownership

B1 is nominal inverse-dynamics feedforward plus one anti-windup PID.
B2 pairs a 10 Hz context encoder with a conditioned RBF continuous action head at 1 kHz.
B3 combines a model-agnostic 10 Hz regime supervisor with the deterministic 1 kHz C99 gatekeeper.
All branches share the measured signals, physical disturbances, and actuator magnitude and slew limits.
B1 has no bank-transfer smoother; B2 has no integrator feature or external PID stabilizer.

The B3 supervisor supplies only `ai_regime` and `ai_confidence`.
The normal regimes are R1 nominal, R2 high load and R3 compliant contact; R4 is explicit abstention.
A classifier, rule tree or VLM may implement this interface, but it cannot write `u` or `u_ff`.
The 1 kHz C99 core applies zero-trust checks to the 10 Hz slow interface: its input firewall, the
local-clock lease on `outer_seq` and the independent physical veto take precedence over any
slow-loop output.

Beyond that interface separation, the core difference from traditional gain scheduling or fixed rule
switching is abstention: a schedule cannot abstain, so it hard-switches among its designed regimes
even after the plant has left them. This architecture therefore combines **dual-layer active
abstention** (10 Hz manifold-distance low-confidence abstention plus the 1 kHz forced veto) with
**four-pillar zero trust towards the slow loop**: the interface and numeric firewall, the
local-clock frame lease on `outer_seq`, the independent 32 B physics-residual probe with the 25 ms
phase-plane look-ahead veto, and the 300 ms recovery cooldown with the 150 ms dwell anti-chatter
check.

The reference prototype model stores twelve coordinates and three confidence parameters; its
four feature normalizations belong to the device interface.

The C99 ABI is fixed: state/config/tuning/input/output occupy 96/140/12/40/28 B.
`LayeredProbe_t` in the MCU template is a separate fixed 32 B struct.
In this reference benchmark, preserve field order, types, function signatures and the sole dependency
on `<stdint.h>`.
There is no heap, no OS call, no mutable global state and no libm dependency (on Linux toolchains that default to stack protection, add `-fno-stack-protector` for bare-metal `nm -u` checks).
Compile without `-ffast-math`: IEEE NaN comparisons are part of the input firewall.

```c
LayeredState_t state;
LayeredConfig_t config;
LayeredInput_t input;
FieldTuning_t field_tuning = {1.0f, 1.0f, 1.0f};
/* Populate config and all ten input fields from the commissioned device interface. */
Layered_Init(&state, &config, &field_tuning);
LayeredOutput_t output = Layered_Step(&state, &config, &field_tuning, &input);
```

The ten input fields are `ai_regime`, `ai_confidence`, `outer_age_s`, `physics_residual`,
`x`, `x_dot`, `r`, `r_dot`, `u_ff` and `uint32_t outer_seq`.
A non-zero `outer_seq` selects the local frame lease: a changed sequence opens a fresh lease,
an unchanged one ages it by one control period, and a sequence that falls back to zero is corruption.
With `outer_seq == 0`, the transported `outer_age_s` remains authoritative.

## 2. Nine fast-loop duties and seven gate states

| Order | Duty |
| :---: | :--- |
| 0 | Invalid fast inputs/configuration hold; invalid field knobs or slow mailboxes latch corruption and run closed-loop R4 using valid sensors. A corrupt configuration uses the fixed 100-tick HOLD budget, then `GATE_SAFE_ZERO`. |
| 1 | Select R4 when the state envelope or velocity look-ahead trips, with hysteresis; veto transitions bypass the dwell timer. |
| 2 | Latch the independent physics-residual OOD veto with release hysteresis. |
| 3 | Latch low confidence or explicit R4 abstention; clear only on a confident normal regime. |
| 4 | Detect stale slow-loop frames on both the age path and the `outer_seq` lease, retain the latency fault bit, and past `max_hold_ticks` escalate to a slew-limited zero command. |
| 5 | Enforce minimum dwell between normal regimes. |
| 6 | Require consecutive clean ticks during recovery cooldown. |
| 7 | Run the controller bank, per-bank tracking anti-windup, bumpless transfer (emergency R4 bypass), magnitude and slew limits. |
| 8 | Record sticky fault bits and event counters; latch `GATE_LATCHED_SAFE` after the configured cumulative severe-fault episodes. |

| Code | Gate | Contract |
| :---: | :--- | :--- |
| 0 | `GATE_NORMAL` | Accepted AI regime; the selected bank runs with field-tuned gains and deterministic feedforward. |
| 1 | `GATE_HOLD_LATENCY` | Stale mailbox or unusable fast input; the last applied command is held. Integrators, target and blend hold. |
| 2 | `GATE_VETO_OOD_RESIDUAL` | Residual above threshold; sticky until it recedes below `residual_clear_ratio`. |
| 3 | `GATE_VETO_LOW_CONFIDENCE` | Confidence below threshold or explicit `ai_regime = 4`; cleared only by a confident normal regime. |
| 4 | `GATE_VETO_SAFETY_ENVELOPE` | Measured or predicted state at the envelope; sticky until both fall below `safety_clear_ratio`. |
| 5 | `GATE_SAFE_ZERO` | Native HOLD past `max_hold_ticks = 100`: target and integrators cleared, command ramped to zero in steps of `max(u_slew_max_per_s * dt_s, 1 binary32 ULP(\|u_prev\|))`. |
| 6 | `GATE_LATCHED_SAFE` | `max_fault_vetoes_before_latch = 3` safety/OOD episodes lock R4 until `Layered_AcknowledgeFaultLatch()`. |

Priority runs envelope, residual, confidence, then native latency HOLD.
All bank integrators, including inactive ones, track the actually applied command.
Dwell, latency and diagnostic counters saturate at UINT32_MAX; they never wrap.
`blend_alpha` smooths the controller target on every control tick after initialization;
it is not a switch-only filter. Magnitude limiting comes before hardware slew limiting.
While a veto is active, or the permanent latch is set with the active regime at R4, the blend filter is
bypassed: the target is the raw R4 output and `blend = 1.0`, so the emergency damping response carries no
first-order phase lag. Magnitude and slew limits still apply.
Nominal feedforward is deterministic and is shared by the B1/B3 normal banks.
R4 always uses a zero reference, fixed bank gains and no feedforward, independently of field tuning.
Its integral and anti-windup act on the same zero-state error; `ff_scale[3]` is ignored.
Invalid field knobs fall back to the internal nominal scales (1,1,1), including for the normal-bank
tracking integrators.
The nominal residual predictor is frozen independently of supervisor weights.

Slew contract: on every normal control tick `|u(k) - u(k-1)| <= u_slew_max_per_s * dt_s`.
`GATE_SAFE_ZERO` is the single documented exception, with a per-tick step of
`max(u_slew_max_per_s * dt_s, 1 binary32 ULP(|u_prev|))`; it only ever reduces `|u|`, and only
while the budget is positive. A configuration whose binary32 `u_slew_max_per_s * dt_s` product
underflows to `0.0f` or overflows to infinity is rejected as corrupt: the first cannot be honoured by
the quantisation floor, the second makes every command reachable in one tick.

The sticky 7-bit audit mask records `LATENCY_TIMEOUT`, `OOD_RESIDUAL`, `LOW_CONFIDENCE`,
`ENVELOPE_BREACH`, `NAN_CORRUPTION`, `HOLD_TIMEOUT` and `PERMANENT_LATCH`.
`Layered_AcknowledgeFaultLatch()` clears the three independent veto latches (safety envelope, physics
residual, low confidence) together with `hold_ticks`, `permanent_latched` and `latch_episode_count`, and
re-arms the recovery cooldown. It never clears the audit mask or the lifetime counters. A heartbeat-only
timeout latches `HOLD_TIMEOUT` but never arms the permanent latch.

## 3. Plain-text source map

| File | Role |
| :--- | :--- |
| `c99_layered_core/layered_gatekeeper.h` | `[REUSABLE CORE / TEMPLATE]` ABI, units, regimes, seven gates and function contract |
| `c99_layered_core/layered_gatekeeper.c` | `[REUSABLE CORE / TEMPLATE]` Deterministic safety arbiter and four-bank controller |
| `c99_layered_core/gate_template.c` | `[REUSABLE CORE / TEMPLATE]` 1 kHz MCU integration sample: 32 B residual probe, `blanking_active`, `outer_seq` commit pattern |
| `python_bench/minimal_template.py` | `[REUSABLE CORE / TEMPLATE]` 10 Hz supervisor, C binding, `auto_calibrate`, `auto_field_tune` and `[USER-PLUGIN-SLOT] 1..3` |
| `simulink_sfunc/sfun_layered_gatekeeper.c` | `[REUSABLE CORE / TEMPLATE]` Level-2 C-MEX S-Function around the frozen core: `DWork` 96 B state, 11 inputs, 7 outputs, 685 lines |
| `python_bench/run_benchmark.py` | `[BENCHMARK HARNESS - DO NOT COPY]` Physical fixture, full Python mirror, training and experiments |
| `c99_layered_core/test_gatekeeper_standalone.c` | `[BENCHMARK HARNESS - DO NOT COPY]` Core behavioral regression |
| `c99_layered_core/verify_core_equivalence.c` | `[BENCHMARK HARNESS - DO NOT COPY]` Test-only frame replay |
| `matlab_simulink/load_domain_profile.m` | `[BENCHMARK HARNESS - DO NOT COPY]` MATLAB constants and application interface names |
| `matlab_simulink/sim_core_engine.m` | `[BENCHMARK HARNESS - DO NOT COPY]` MATLAB physical fixture, deployment logic and core mirror |
| `matlab_simulink/calibrate_coarse_classifier.m` | `[BENCHMARK HARNESS - DO NOT COPY]` Small-sample prototype calibration |
| `matlab_simulink/train_continuous_policy.m` | `[BENCHMARK HARNESS - DO NOT COPY]` Conditioned continuous action policy |
| `matlab_simulink/run_efficiency_analysis.m` | `[BENCHMARK HARNESS - DO NOT COPY]` Action-sample budgets and frozen-AI field tuning |
| `matlab_simulink/verify_core_equivalence.m` | `[BENCHMARK HARNESS - DO NOT COPY]` MATLAB/C replay verification |
| `matlab_simulink/run_all_benchmarks.m` | `[BENCHMARK HARNESS - DO NOT COPY]` Acceptance run and three figures |
| `matlab_simulink/build_simulink_model.m` | `[BENCHMARK HARNESS - DO NOT COPY]` Source for the built-in-block Simulink sketch |

The three `SLOT 1..3` comments inside `minimal_template.py` are the `[USER-PLUGIN-SLOT]` markers:
slot 1 is sensor/reference/actuator I/O, slot 2 is the commissioned predictor and its dimensionless
features, and slot 3 is the labeled commissioning set with the constrained device-trial callback.

Keep the shared physical constants aligned between the existing MATLAB and Python files, and use the
same 32-bit LCG and random-number consumption order in both engines.
The fixture uses Karnopp sticking, unilateral contact and a positive-stiffness, positive-damping two-inertia mode.
S3 measurement noise has a single scale and a bounded white distribution; no periodic tone is added.
Stage labels belong only to plant generation, calibration labels and offline statistics: deployment
controllers and residual probes cannot read stage labels, time schedules or fault frequency, and the
broadband innovation envelope and independent spike probe must work without a fault-frequency input.

The Simulink adapter is a thin bridge, and its parameter contract is part of the reusable template.
`mdlCheckParameters` is compiled unconditionally — `#define MDL_CHECK_PARAMETERS` before `simulink.c`
is included, never inside an `#ifdef` of itself — and `mdlInitializeSizes` calls it once the
S-function parameter count matches. The same readers run again in `mdlStart`, so a malformed block
parameter is rejected when the model is updated (`set_param(model,'SimulationCommand','update')`) and
again before the first sample. Every floating parameter element must be finite as a double and must
convert to a finite binary32 value; every tick-count element must be a finite integer in
`[0, 4294967295]`; each vector must be a real 1-D double array. `single`, integer, sparse, complex,
empty, wrong-length, matrix-shaped and 3-D variants are refused, and no value is ever saturated or
silently zeroed. Runtime input ports keep the opposite contract: a NaN or out-of-range signal reaches
the frozen core, which classifies it through its own corruption path. `outer_seq` is validated at the
adapter boundary because the ABI carries a `uint32_t`: a sample that is not a finite whole number in
`[0, 4294967295]` is forwarded as explicit slow-mailbox corruption — the core sets `corrupted = 1`,
latches `NAN_CORRUPTION` and runs closed-loop R4 — rather than being truncated or saturated into a legal
sequence, while an integral zero keeps the legacy semantics. The block sample time is taken only from a
positive `dt_s` below `1e6 s`; `mdlCheckParameters` refuses a larger period so the core period and the
Simulink rate cannot diverge, and a finite non-positive `dt_s` inherits the model rate and is classified
by the core as a corrupt configuration.

## 4. Interface adaptation and porting sequence

### Step 1 — Define signals, units and fallback

Map all ten input fields: `ai_regime`, `ai_confidence`, `outer_age_s`, `outer_seq`,
`physics_residual`, `x`, `x_dot`, `r`, `r_dot`, `u_ff`.
State and reference share units; derivatives share units per second.
Residuals and macro-features are dimensionless after device-defined normalization.
Specify the fallback objective and measurable output limits before tuning controllers.
Choose state coordinates whose zero is the commissioned safe fallback point.
The upstream planner or reference governor must limit `r` to the state safety envelope before calling the core.
The benchmark's radians, torques and mechanical reference are one example of such an interface.

### Step 2 — Commission the nominal fast loop

For direct reuse of the verified core, start with it unchanged and define the four controller banks
in the device adapter. Adaptations may instead revise the core and validate the resulting device contract.
The MIT copyright and permission notice in `LICENSE` must accompany copies or substantial portions.
Commission the nominal model, the deterministic feedforward and the independent innovation probe.
The MCU template in `gate_template.c` is the 1 kHz integration reference: it derives a 32 B probe
from a measured signal, freezes probe and HAL and holds the pre-blanking residual while
`blanking_active`, and commits each slow frame by incrementing `outer_seq` after the payload is
visible.
Verify the command magnitude/slew bounds, corruption handling, gate priorities, the HOLD → `SAFE_ZERO`
escalation, the cumulative latch and the acknowledgement path.
Measure target execution time separately from host ctypes call time.

### Step 3 — Build a small labeled macro-feature archive

`auto_calibrate(samples, labels)` accepts finite normalized feature rows and labels 1, 2 or 3;
supply every class. It returns `(prototypes, gamma, confidence_trip, confidence_clear)`.
The reference calibration uses short measured closed-loop probes, then samples macro windows.
Freeze the resulting prototypes and confidence parameters before unit-to-unit adaptation.

### Step 4 — Tune the physical knobs

`auto_field_tune(evaluate, current)` calls `evaluate(knobs) -> (cost, valid)`.
Run each candidate inside the C gatekeeper and return a measured tracking cost.
Set `valid=False` on any state, contact, actuator or device-specific constraint violation.
Each candidate must cover load/stiction and contact conditions, including harmonic and ramped references.
Constrain contact force, gate-veto fraction and R4 occupancy, and repeat those checks after tuning.
The helper rejects invalid or nonfinite scores and retains `current` when all candidates fail.
Evaluate six fixed joint (gain, damping) candidates: (0.75,1), (1,1), (1.25,1), (2,1),
(1.25,0.5), (1.25,0.75). Each pair is evaluated independently under the same constraints.
The benchmark evaluates six full 40 s missions per branch/unit (240 s of search trajectories).
Candidate validity requires zero breaches throughout, nominal tracking, and normal-stage force,
veto fraction and R4 occupancy limits. Independent validation adds two 6 s S2 runs and a 40 s mission
(52 s per branch/unit). Device trajectory time and host simulation time are separate quantities.
The reference benchmark runs the same four commissioned conditions in both engines — forward drift 0.35,
forward drift 0.50, cross mismatch (J +40%, b −30%) and reverse mismatch (J, b, fc, fs −25%) — and the
accepted B3 tuples, together with their search and mission metrics, are identical in Python and MATLAB; only
the rejected B1 counterfactual peaks carry the ~1e-14 double-precision sensitivity described in README
section 3.1, which changes no feasibility verdict.
The safety-margin knob stays fixed during this search; its limit comes from the device contract.
The three knobs remain `gain_scale / damping_scale / safety_margin_scale`.
Gain and damping must lie in [0.10,4.0] and safety margin in [0.10,1.0], so tuning can only tighten
the envelope.
These are input-validity bounds; closed-loop suitability requires the constrained trials above.
Record trial duration and host computation time separately, and freeze the AI throughout the search.
A new physical domain requires a new signal/model/controller definition before calibration.

### Step 5 — Answer unstructured reuse questions with three layers

A downstream request rarely arrives in this repository's shape: a user describes an existing
multi-axis arm, a WAAM machine, an aDBS stimulator or a PLC/PID loop and asks what can be reused and
what has to be cut out. Do not answer by porting the benchmark wholesale. Answer along three
decoupled layers — drop-in reuse, mechanism slicing and fixture removal — as a heuristic reference
path, not as a rigid doctrine. The fixed C ABI sizes (96/140/12/40/28 B) and the template line
counts (119/120/685 lines) stay unchanged only when maintaining this repository's own reproduction
benchmark. When helping a downstream user adapt the code to their device, their real hardware,
control period and existing code structure always take precedence: refactor the interfaces, merge or
split functions, change the bank count, or borrow only the core ideas (dual-layer active abstention,
the zero-trust firewall, tracking anti-windup) and rewrite them for the target platform. Never limit
a user's engineering changes on the grounds that they violate the reference architecture.

1. **Drop-in reuse (plug and play).** `c99_layered_core/layered_gatekeeper.c` and
   `layered_gatekeeper.h`, and the Level-2 `simulink_sfunc/sfun_layered_gatekeeper.c` adapter, are
   independent of the physical domain: no units, no plant model and no AI model live inside them. For
   a multi-axis or multi-channel device, instantiate one `LayeredState_t` (96 B) per channel, share a
   read-only configuration only between identical actuator classes, and call `Layered_Step()` from the
   existing 1 kHz control task. The core runs inside the existing loop instead of replacing it.
2. **Mechanism slicing.** If the device already owns a control law, do not replace it: lift one
   orthogonal mechanism out of the core and leave the rest in place. The usual slices are the `outer_seq`
   heartbeat lease with local 1 kHz age timing (so a stalled or forged slow frame cannot mask the
   timeout), the `blanking_active` artefact-blanking probe, the per-bank tracking anti-windup, and the
   three-episode permanent latch with its `Layered_AcknowledgeFaultLatch()` release state machine.
   Each slice keeps its own contract and can be validated on its own.
3. **The benchmark fixture does not constrain interface adaptation.** The 1-DOF rotary motor, the
   unilateral spring wall and the four fixed bank parameter sets inside `python_bench/run_benchmark.py`
   and `matlab_simulink/sim_core_engine.m` exist only to reproduce this repository's own benchmark runs.
   A downstream device maps its real physical signals through
   [domain mapping](docs/domain_mapping_guide.md); none of those fixture choices (radians, N m, wall
   phase, noise scale, bank gains) belongs to the reusable contract. Replacing or discarding the
   fixture is normal adaptation, never an interface-adaptation failure, and never a reason to keep an
   ABI element the device does not need.

#### Hardware adaptation notes

When porting to target hardware, resolve these three configuration points before commissioning:

1. **Define the device's fail-safe action.** The default `R4` (zero reference, fixed low-gain PD
   toward `x = 0`) and `GATE_SAFE_ZERO` (command ramp to zero) match the horizontal rotary fixture.
   For a vertical gravity-loaded axis, WAAM welding or closed-loop aDBS/SCS, replace both branches
   with the device's physical safe state (drop-prevention brake or position hold; retract torch /
   break arc; nonnegative stimulation amplitude retreating to clinical open-loop baseline parameters
   or controlled shutdown).
2. **Calibrate the envelope look-ahead and the smoother against actuator limits.** The `25 ms`
   scalar linear look-ahead (`|x| + T * |x_dot|`, where `T` is the look-ahead horizon) is a
   lightweight heuristic; high-inertia or strong-disturbance devices must calibrate the switching
   margin against actuator saturation and detection delay (reachable sets or control barrier
   functions, CBF). The always-on `blend_alpha` smoother explains most of B3's TV reduction; for
   phase-sensitive high-bandwidth loops set it to `1.0` and rely only on the four-bank reverse
   tracking anti-windup for bumpless switching. The 32 B model-free high-pass envelope probe in
   `gate_template.c` and the fixture-model observer in `run_benchmark.py` differ in complexity;
   calibrate the filter coefficients to the target sample rate.
3. **Synchronize cross-interrupt data exchange and structs.** `outer_seq` defines the cross-loop
   logical lease; on a bare-metal MCU use double buffering, interrupt-disabled copy or atomic
   publication ordering so the 1 kHz interrupt cannot tear the slow-loop mailbox. When C-side struct
   fields are added or removed, update the Python `ctypes` binding, and re-evaluate the `double`
   inward-rounding helper used by the slew limiter on single-precision FPU MCUs.

#### Bilingual terminology bridge for Chinese prompts

When a Chinese-speaking user describes the device in Chinese, map each Chinese anchor to the symbol
before answering; keep the user's Chinese wording in the reply while using the exact identifiers below.

- `跨环异步通信缓冲 / 慢环通信帧` = slow-frame mailbox (`ai_regime`, `ai_confidence`, `outer_age_s`, `outer_seq`).
- `离散定性路由与流形外主动弃权` = discrete qualitative routing & OOD abstention (`ai_regime = 4` / `GATE_VETO_LOW_CONFIDENCE`).
- `流形外主动弃权 / 低置信度弃权` = OOD / low-confidence abstention (`ai_regime = 4` / `GATE_VETO_LOW_CONFIDENCE`).
- `标称模型预测残差（新息）探针` = 32 B innovation probe (`LayeredProbe_t` / `physics_residual`).
- `硬件消隐窗口` = `blanking_active`.
- `内核级 HOLD（指令保持）` = native command hold (`GATE_HOLD_LATENCY`).
- `能量泄放归零` = slew-limited ramp-to-zero (`GATE_SAFE_ZERO`).
- `锁存保持故障掩码` = sticky `fault_audit_bitmask`.
- `接口适配` = interface adaptation (`SLOT 1..3` / `FieldTuning_t`).

## 5. Multi-axis pattern and slow-loop timeout

Create one `LayeredState_t`, probe, tuning tuple, `outer_seq` counter and slow-frame timestamp per
axis/channel. Share read-only configurations only when units, limits and controller definitions are
identical. A shared supervisor may publish distinct decisions to each channel.
Compute coupled constraints in the device adapter and translate them into per-channel abstention
or a coordinated device fallback. In this reference benchmark, keep the single-axis C API unchanged.

Compute `outer_age_s = now - source_timestamp` on every fast tick.
A nonfinite or out-of-range regime, confidence or age marks the mailbox corrupt and runs R4;
the envelope and residual checks retain priority, with valid sensors driving the closed loop.
The reference timeout is 350 ms. Once stale, the adapter sends an explicit R4 with confidence zero
while preserving the real age. This selects the deterministic fallback and also records latency.
A fresh frame restores eligibility only after the core's clean recovery cooldown, 300 ms here.
With a non-zero `outer_seq`, the core ages the frame on its own clock, so a stalled producer or a forged
age cannot mask the timeout. A sequence that falls back to zero is treated as corruption, not as a
fresh frame. Native `GATE_HOLD_LATENCY` holds the last actuator command, not mechanical position.
Do not describe a held torque, current or stimulation command as a held physical state.
Past `max_hold_ticks` the core enters `GATE_SAFE_ZERO`: target and integrators clear, the command is
ramped to zero in steps of `max(u_slew_max_per_s * dt_s, 1 binary32 ULP(|u_prev|))`, and 
`HOLD_TIMEOUT` latches. The cumulative safety/OOD latch (`GATE_LATCHED_SAFE`) is separate and
requires the explicit acknowledgement API plus cooldown.
A persistent fast-sensor failure requires a hardware-enforced maximum HOLD duration and safe shutdown
(safe torque off, STO, for a drive); the C core does not supply that independent hardware function.

## 6. Three equal application examples

| Application | Slow interface | Deterministic fast interface |
| :--- | :--- | :--- |
| Industrial WAAM / PLC servo | Process, load and contact regime; abstention | Process/servo regulation, output limits and field tuning |
| Edge VLA / fast-slow robotics | Task and environment context; abstention | Torque/impedance regulation, disturbance rejection and fallback |
| Closed-loop neuromodulation aDBS / SCS | Biomarker and interface regime; abstention | Device-defined regulation, output envelope and timing limits |

These examples map a generic architecture; other devices follow the same interface adaptation sequence.
Keep physical interfaces and units explicit rather than renaming mechanical states as biological ones.
See [domain mapping](docs/domain_mapping_guide.md) for the adaptation contract.

## 7. Reproduce from the repository root

Python uses only the packages already listed in `python_bench/requirements.txt`.
The minimal template requires a working C99 compiler, and build failures propagate directly.
Its dynamic library rebuilds when either the C source or the header is newer.
The benchmark additionally offers its complete Python mirror.

```powershell
python -m pip install -r python_bench/requirements.txt
python python_bench/minimal_template.py
python python_bench/run_benchmark.py
python python_bench/run_benchmark.py --no-c
python python_bench/run_benchmark.py --selftest
gcc -std=c99 -pedantic -Wall -Wextra -O2 -o c99_layered_core/test_gatekeeper_standalone.exe c99_layered_core/layered_gatekeeper.c c99_layered_core/test_gatekeeper_standalone.c
.\c99_layered_core\test_gatekeeper_standalone.exe
gcc -std=c99 -pedantic -Wall -Wextra -O2 -o c99_layered_core/gate_template.exe c99_layered_core/layered_gatekeeper.c c99_layered_core/gate_template.c
.\c99_layered_core\gate_template.exe
matlab -batch "cd('matlab_simulink'); verify_core_equivalence"
matlab -batch "cd('matlab_simulink'); run_all_benchmarks"
```

The C regression reports 245 passes and zero failures with exit code zero, and the MCU template prints PASS.
Replay compares gate, regime, fault bits and corruption flags exactly.
Python/C command agreement is bit-exact (max|du| = 0 over 6,000 frames); MATLAB/C agreement is
max|du| = 5e-9 over 40,000 normal, 40,000 mailbox-fault and four 40,000-frame tuning-injection replays,
plus 4.19e-8 over a 10,000-tick `outer_seq` lease replay that covers HOLD → `SAFE_ZERO`, the
zero-sequence corruption path and cooldown recovery.
The S-Function reproduces seven outputs over 1,100 frames with max diff = 0.0 and all gate codes 0–6.
Benchmark checks compare stage metrics, true-state envelope violations, contact force and field tuning.
Do not substitute gate activation counts for physical safety violations.
Compute RMSE from reference and true state at the same tick; TV is the sum of absolute command changes.
Keep physical contact force distinct from actuator torque.

The MATLAB acceptance run regenerates the existing `.slx` and the three `assets/` figures.
Keep the model self-contained: built-in blocks only, no workspace initialization dependency.
The B2 sketch places slow-loop transport/hold on context; its action path updates at 1 kHz.
Serialize writes to the generated model. Preserve unsaved user edits in any open model.

## 8. Repository discipline

Keep the core and templates small: the Python template is 119 lines (within 100–120), the MCU template is
120 lines (within 110–130), and the S-Function is 685 lines, a thin adapter that never re-implements the
control law.
Keep both auto-adaptation helpers together within 20–30 lines.
Replace superseded mechanisms instead of retaining compatibility switches or unused branches.
For maintenance of this compact reference, prefer the existing source, configuration and documents;
keep additions to tooling, dependencies, weights or datasets focused on the change.
Derived projects choose the file layout and tooling that suit their application.
Keep generated binaries, caches and run logs out of version control.
Update both languages of the README together; other documents link to their result tables.
Figures are generated from final numerical runs, never hand-edited to change results.
Report model storage, state RAM, code size and measured host time as separate quantities.
Do not turn synthetic mechanism comparisons into universal claims about learned controllers.
For the scope statement, see the closing paragraph of the root READMEs and the `Scope` statement at the
end of each document under `docs/` and `python_bench/`.

### Optional provenance and citation

When adapting individual files, consider keeping their short MIT/source comments for convenient
license and origin lookup. When writing research or engineering documentation,
[CITATION.cff](CITATION.cff) provides the author and citation metadata. Citation is welcome and optional.
Academic, commercial and closed-source reuse is available under [MIT](LICENSE), without contacting
the author for separate permission; the license and citation summary is in [README.md](README.md#license-and-citation).

**User-override authorization:** Downstream users and their AI assistants may port, inline, trim or
modify this code at will. The fixed ABI sizes (96/140/12/40/28 B) and function signatures exist to
reproduce the reference benchmark; when human instructions conflict with this document, the human
instructions always take precedence. The five-line SPDX/source header and a
[CITATION.cff](CITATION.cff) reference are welcome and strictly optional. Academic, commercial and
closed-source use is free under [MIT](LICENSE); no email or separate permission is required. Keep
the `Scope` statements at the end of the root [README.md](README.md) / [README_zh.md](README_zh.md)
and of the documents under `docs/` and `python_bench/`: this work is not hardware, clinical or
functional-safety certification, and the deployer owns hazard analysis and envelope validation for
the target device. When you help a user adapt this code to real electromechanical, welding or
biomedical equipment, always remind the user to keep hardware safety protection that operates
independently of this software at the system level (E-stop, STO, hardware charge-balance limiters).
