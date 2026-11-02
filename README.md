# ai-layered-control · v2.0.0

[English](README.md) | [简体中文](README_zh.md) · [MIT](LICENSE) · [Agent guide](AGENTS.md)

**A general reference architecture for slow qualitative AI, fast deterministic control, and adaptation without AI retraining.** The slow loop selects a regime or explicitly abstains. A zero-dependency C99 core owns the actuator at 1 kHz.

Three application families motivate the same separation: **industrial WAAM / PLC servo**, **edge VLA / fast-slow robotics**, and **closed-loop aDBS / SCS neuromodulation**. They are examples of a common control contract, open to other systems with scarce samples, unit variation, intermittent contact, uncertain context and strict output constraints.

## 1. Three competing control paths

| Branch | Slow context | Continuous actuator command at 1 kHz |
|---|---|---|
| B1 · modern white-box baseline | Single fixed regime | Nominal inverse-dynamics feedforward + high-gain anti-windup PID |
| B2 · learned fast-slow policy | Four measured macro-features at 10 Hz, delayed independently | Conditioned 512-center RBF action head, `u = π(x, x_dot, r, r_dot, r_ddot, z)` |
| B3 · qualitative / quantitative separation | Model-agnostic regime + confidence + timestamp at 10 Hz | C99 controller bank, nine duties of protection, deterministic feedforward and field tuning |

B1 and B3 receive the **same nominal feedforward**. All branches share actuator magnitude and slew limits, sensor processing, noise, reference, plant and fault realization. B1 has no `blend_alpha` smoother. B2 performs **40,000 action evaluations in 40 s**; latency applies only to context. Its policy contains no integrator input or external PD stabilizer. B3's first-order command smoothing operates on every tick.

Learning-based fast-slow systems are an active engineering direction; [Helix 02](https://www.figure.ai/news/helix-02) is one example. B2 is a reproducible representative of the continuous-action route, not a reproduction of that system. The design horizon is **2027+ deployment across constrained devices**.

## 2. Four-stage benchmark and field tuning

40 s, four 10 s stages, a 1 ms control period and two 0.5 ms physical substeps. Actuator limits are **20 N·m** and **2,000 N·m/s**. The state envelope is **|x| ≤ 1 rad**; S3's contact-force budget is **20 N**, using a 0.25 m lever arm.

RMSE compares reference and true state at the same tick. TV is the within-stage sum of absolute command changes. A violation tick means the true state lies outside the envelope; it is separate from a gate activation. Tables use the Python + C99 run, seed 7. MATLAB independently reproduces the metrics within the declared 1% tolerance.

| Stage | Branch | RMSE / rad | TV(u) / N·m | Violation ticks | Peak contact / N |
|---|---|---:|---:|---:|---:|
| S1 | B1 | 0.000664 | 8217.67 | 0 | — |
| S1 | B2 | 0.004663 | 5804.16 | 0 | — |
| S1 | B3 | 0.000630 | 1567.40 | 0 | — |
| S2 | B1 | 0.017125 | 8100.68 | 0 | — |
| S2 | B2 | 0.052245 | 5927.32 | 0 | — |
| S2 | B3 | 0.020819 | 2083.14 | 0 | — |
| S3 | B1 | 0.030182 | 18257.53 | 0 | 74.49 |
| S3 | B2 | 0.036630 | 16743.89 | 0 | 18.95 |
| S3 | B3 | 0.049949 | 9002.59 | 0 | 12.72 |
| S4 | B1 | 0.412180 | 10393.22 | 880 | — |
| S4 | B2 | 0.322109 | 11891.10 | 218 | — |
| S4 | B3 | 0.275673 | 397.19 | 0 | — |

**Zero-retraining unit adaptation, adjacent to the main result.** The classifier remains frozen. The following independent S2 validation uses a 6 s trajectory of amplitude 0.43 rad, seed 811; B1, factory B3 and tuned B3 use that same trajectory. These values are not mixed with the 10 s S2 window above. RMSE is in rad; TV is in N·m.

| Inertia drift | B1 fixed PID | B3 factory `(1,1,1)` | B3 tuned `(2,1,1)` | RMSE reduction | B3 TV: factory → tuned | Violations |
|---|---:|---:|---:|---:|---:|---:|
| +35% | 0.016682 | 0.020331 | **0.009443** | 53.6% | 1249.03 → 1523.91 | 0 → 0 |
| +50% | 0.023648 | 0.027846 | **0.013028** | 53.2% | 1254.20 → 1536.43 | 0 → 0 |

Each search evaluates four gains `{0.75,1,1.5,2}`, then two damping values `{0.5,0.75}` at the best gain. Six 4 s trials give **24 s of device-side search**, followed by two 6 s validation rollouts. Host simulation time and device time are distinct. The safety-margin knob stays at 1.0. Rejected trials cannot become the selected candidate; if all candidates fail, the current knobs are retained.

The tuned full 40 s missions also have **zero violations**, with S3 peak forces **14.38 / 13.23 N** for +35% / +50% inertia. B1 is more accurate than factory B3 in S2; tuned B3 is more accurate than B1 on the independent validation. In S3, B3 accepts more position error to reduce contact force and command variation. B2 also meets the S3 force budget in this run.

![Four-stage comparison](assets/benchmark_4stages_showdown.png)

### Physical fixture

| Stage | Physical mechanism |
|---|---|
| S1 · nominal tracking | Inertia 0.010 kg·m², viscous damping 0.060 N·m·s/rad, ground stiffness 0.300 N·m/rad; dynamic/static friction 0.6 / 1.0 N·m |
| S2 · unit-to-unit gap | Inertia +35%; dynamic/static friction 3.09 / 6.2625 N·m. Karnopp zero-speed band 0.003 rad/s; insufficient drive locks the state, breakaway switches to sliding friction |
| S3 · contact and edge jitter | One-sided wall: stiffness 300 N·m/rad, damping 0.8 N·m·s/rad; position 0.43 + 0.025 sin(2π·0.73t + phase) rad with continuous approach. Measurement white-noise standard deviation 0.015 rad; context delay 60–120 ms and 8% frame drops |
| S4 · unseen compound OOD | At 30.037 s, a passive motor/load pair with 20%/80% inertia split replaces the rigid actuator; positive stiffness/damping create a 12→24 Hz swept mode. Load sensing also receives 0.025 rad bias and 0.18 rad spikes |

The inertia change persists through S3/S4; elevated friction belongs to S2. Both engines use the same 32-bit LCG and bounded, unit-variance white-noise construction. There is no sinusoidal measurement-noise tone. The flexible plant has a decaying zero-input response: the fault introduces no negative-damping source.

B2 trains on **16,000 nominal-model action labels**, spanning tracking errors, reference derivatives and context. Contact-like context softens its nominal action target. Strong static friction, the moving wall and the flexible actuator are deployment shifts. Panel A separately trains on shifted-unit labels to measure adaptation; no test trajectory is used as a training label. The comparison tests this memoryless action head and training distribution, not every possible learned controller.

### Controller bank

| Controller | Kp | Ki | Kd | Feedforward scale |
|---|---:|---:|---:|---:|
| B1 | 110 | 300 | 1.5 | 1 |
| B3 R1 · nominal | 60 | 60 | 1.2 | 1 |
| B3 R2 · load / friction | 110 | 300 | 1.5 | 1 |
| B3 R3 · compliant contact | 8 | 2 | 0.8 | 1 |
| B3 R4 · safe fallback | 2 | 0 | 0.35 | **0** |

R4 suppresses nominal trajectory feedforward by design. Its feedback law and the physical damping remain deterministic.

## 3. Mode necessity and independent protection

Fixed-bank ablations retain the **same fast safety gates**. Dynamic classification and physical rules read only measured features. All six variants below have zero state-envelope violations.

| Supervisor | S1 RMSE | S2 RMSE | S3 RMSE | S4 RMSE | S3 TV | S3 peak / N |
|---|---:|---:|---:|---:|---:|---:|
| R1 | 0.000630 | 0.048769 | 0.038107 | 0.276310 | 12993.85 | 35.12 |
| R2 | 0.000715 | 0.019624 | 0.030081 | 0.276936 | 14779.48 | 71.34 |
| R3 | 0.002368 | 0.416069 | 0.066815 | 0.275729 | 8899.58 | 8.76 |
| R4 | 0.244245 | 0.491956 | 0.245783 | 0.297341 | 3904.54 | 0.00 |
| dynamic | 0.000630 | 0.020819 | 0.049949 | 0.275673 | 9002.59 | 12.72 |
| rules | 0.000630 | 0.020819 | 0.049949 | 0.275673 | 9002.59 | 12.72 |

The fixed-bank results expose the load/contact tradeoff. The rule supervisor and the prototype classifier produce the same main-run result, supporting the **model-agnostic slow interface**. A rule tree or compact VLM can replace the prototype classifier without changing actuator ownership.

The probe combines a median-based spike channel with a **broadband high-frequency innovation envelope**: time-aligned nominal acceleration prediction, two-pole 25 Hz low-pass processing, 5 Hz high-pass innovation, an 8 ms energy window, normalization by 500 rad/s² and a 1 s release envelope. A separate slow innovation feature supports classification. No fault frequency enters the deployed probe.

The full compound fault's first spike triggers a veto in the same control tick. **Anomaly detection delay and core response time are different measurements.** With the spike channel disabled, a 0.2 rad sensor oscillation at 12 / 18 / 37 / 83 / 137 Hz is detected after **30 / 6 / 4 / 3 / 3 ms**, following a quiet initialization interval. With spikes, confidence, envelope and timeout vetoes removed from the compound-fault test, the independent broadband probe detects after **947 ms**, retains zero violations and limits maximum |x| to **0.7482 rad**.

The main run selects R1 throughout S1, R2 for 96.8% of S2 and R3 for 98.22% of S3; the remaining normal-stage ticks cover transitions. S4 stays in R4 after fault onset. The main run has no normal-stage low-confidence veto. In five additional noise/phase runs, B1/B2 S4 violations range over **798–862 / 131–199 ticks**, while B3 remains at zero; B3 S3 peak force is **7.64–9.68 N**. Two runs each contain one fresh low-confidence frame at the S3 transition, producing 103 / 111 veto ticks; stale-frame ticks and recovery cooldown are counted separately.

Removing only OOD veto leaves other gates active but produces **75 violation ticks**, maximum **|x| = 1.4819 rad** and S4 TV **2,894.99 N·m**. Restoring R4 feedforward in a separate ablation still gives zero violations, so zero feedforward is a fallback design convention rather than a uniquely necessary safety mechanism. Higher sweep bands 18→36 and 24→48 Hz do not produce B2 envelope breaches. Removing context jitter changes B2's S3 peak force only from **18.95 to 18.86 N**; the dominant comparison here is friction, contact and measurement noise. One/two/four-substep runs retain the main safety and contact conclusions.

## 4. Small-sample commissioning and resource accounting

`auto_calibrate(samples, labels)` builds three four-dimensional prototypes and confidence parameters from **36 labeled macro-windows**, twelve from each of three 10 s commissioning probes. Windows are not independent experiments. The resulting model has **15 stored scalars**: twelve prototype values, a distance-decay coefficient and two confidence thresholds.

`auto_field_tune(evaluate, current)` freezes that model and searches physical gains using measured error and an explicit validity result. The two reusable helpers fit in 20–30 lines inside the [117-line template](python_bench/minimal_template.py).

![Sample budgets, field tuning and storage](assets/efficiency_and_generalization.png)

Panel A uses the actual conditional RBF at 64 / 256 / 1,024 / 4,096 / 16,000 action labels, with up to 512 centers. It reports nominal performance, frozen transfer and retrained transfer. The curves are non-monotonic; retraining improves some budgets without restoring nominal accuracy. Action labels and macro-windows are distinct sample units and are not divided into a sample-efficiency ratio.

| Resource category | Size |
|---|---:|
| Conditional RBF, including normalization, centers, width and output coefficients | 5,149 scalars; 41,192 B FP64 / 20,596 B FP32 storage |
| Prototype classifier | 15 scalars; 60 B FP32 storage |
| C99 state / configuration / field knobs | **88 / 132 / 12 B** |
| C99 object text, GCC 4.9.2 x86-64 | 3,880 B at `-Os`; 4,568 B at `-O2` |

FP32 model bytes are a storage calculation; numerical RBF runs use FP64. State RAM, model storage and code size are separate quantities. Python prints measured RBF and C-call host times, including ctypes overhead; they are not target WCET. The core uses only `<stdint.h>`, fixed-size arithmetic and no heap, I/O, libm or mutable globals.

## 5. Port the contract to a new system

| Application example | Slow qualitative decision | Deterministic fast loop |
|---|---|---|
| Industrial WAAM / PLC servo | Process, load and contact regime | Process/servo law, actuator constraints, field tuning |
| Edge VLA / fast-slow robotics | Task and environment context | Torque/impedance law, disturbance handling, fallback |
| Closed-loop aDBS / SCS | Biomarker and interface-state class | Device-defined control law, output envelope and timing |

![Three mappings of the common control contract](assets/domain_profiles_comparison.png)

Define variables and a safe fallback → commission the nominal model and controller bank → calibrate measured macro-features → freeze AI and field-tune. A new physical domain needs new units, limits, control laws and probes. A new unit in the same domain can retain the commissioned classifier.

```c
LayeredState_t state;
LayeredConfig_t config;       /* device-defined units, limits and bank */
FieldTuning_t field_tuning = {1.0f, 1.0f, 1.0f};
LayeredInput_t input;         /* measured signals and timestamped regime frame */
/* Populate config and input before calling. */
Layered_Init(&state, &config, &field_tuning);
LayeredOutput_t output = Layered_Step(&state, &config, &field_tuning, &input);
```

The nine core duties are input/configuration validation, predictive envelope veto, residual veto, confidence/abstention veto, heartbeat watchdog, minimum dwell, recovery cooldown, controller-bank execution with anti-windup and output limits, and sticky fault reporting. Each integrator tracks the applied command; output magnitude is clamped before slew. The C ABI and function signatures remain fixed.

Use **one state, probe, tuning tuple and timestamp per axis/channel**. Share read-only configuration only where units and limits agree; coupled constraints belong in the device adapter. On a **350 ms** slow-loop timeout the adapter sends explicit R4 abstention and preserves real frame age. Clean recovery passes through the core's **300 ms** cooldown. Native C `HOLD` freezes the **actuator command**, not mechanical position.

See [AGENTS.md](AGENTS.md), the [domain mapping guide](docs/domain_mapping_guide.md), and the [systems engineering rationale](docs/systems_engineering_manifesto_en.md). Deployment code does not accept stage labels, injected fault frequency or simulator truth.

## 6. Reproduce

Python 3 + the existing NumPy/matplotlib requirements; C99 compiler for the core-backed path:

```powershell
python -m pip install -r python_bench/requirements.txt
python python_bench/run_benchmark.py
python python_bench/run_benchmark.py --no-c
python python_bench/run_benchmark.py --selftest
python python_bench/minimal_template.py
gcc -std=c99 -pedantic -Wall -Wextra -O2 -o c99_layered_core/test_gatekeeper_standalone.exe c99_layered_core/layered_gatekeeper.c c99_layered_core/test_gatekeeper_standalone.c
.\c99_layered_core\test_gatekeeper_standalone.exe
```

The template recompiles when either C source or header changes; compiler failure is explicit. The benchmark's `--no-c` path uses a complete Python mirror. C regression: **120 passes, zero failures**. Python/C shared-input replay: **6,000 frames**, exact gate/regime/fault agreement, maximum command difference below `1e-4`.

MATLAB R2024b + Simulink, built-in functions and blocks:

```powershell
matlab -batch "cd('matlab_simulink'); run_all_benchmarks"
matlab -batch "cd('matlab_simulink'); verify_core_equivalence"
matlab -batch "clearvars; sim('matlab_simulink/ai_layered_control_bench.slx'); disp('standalone model PASS')"
```

The acceptance run prints five PASS verdicts, rebuilds the existing model and three 240 dpi figures, and emits a JSON report. MATLAB/C replay uses **40,000 common frames** with exact discrete outputs. The `.slx` is a self-contained architectural sketch; the numerical engines generate the quantitative results. Its B2 delay/hold is on the context path, while the action path runs at 1 kHz. Read [the builder](matlab_simulink/build_simulink_model.m) as the model's source.

The safety partition relates to Simplex / runtime assurance; see [AFRL RTA](https://github.com/act3-ace/run-time-assurance) and [ISO/IEC TR 5469:2024](https://www.iso.org/standard/81283.html).

**Scope.** This repository supplies a reference control contract and reproducible numerical evidence for the specified fixture. Application mappings do not constitute hardware, clinical or functional-safety validation or certification; deployment requires device-specific hazard analysis, envelope definition and validation.
