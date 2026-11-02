# AGENTS.md — ai-layered-control v2.0.0

Read this file before changing the architecture. The source files define behavior;
[README.md](README.md) and [README_zh.md](README_zh.md) contain the authoritative benchmark results.
The generated Simulink model is an architecture sketch; read its builder, not the binary container.

## 1. Contract and ownership

B1 is nominal inverse-dynamics feedforward plus one anti-windup PID.
B2 is a 10 Hz context encoder plus a conditioned RBF continuous action head at 1 kHz.
B3 is a model-agnostic 10 Hz regime supervisor plus the deterministic 1 kHz C99 gatekeeper.
All branches share measured signals, physical disturbances, actuator magnitude and slew limits.
B1 has no bank-transfer smoother; B2 has no integrator feature or external PID stabilizer.

The B3 supervisor supplies only `ai_regime` and `ai_confidence`.
Normal regimes are R1 nominal, R2 high load and R3 compliant contact; R4 is explicit abstention.
A classifier, rule tree or VLM may implement this interface. It cannot write `u` or `u_ff`.
The reference prototype model stores twelve coordinates and three confidence parameters.
Its four feature normalizations belong to the device interface.

The C99 ABI is fixed: state/config/tuning/input/output occupy 88/132/12/36/28 B.
Preserve field order, types, function signatures and the sole dependency on `<stdint.h>`.
There is no heap, OS call, mutable global state or libm dependency.
Compile without `-ffast-math`; IEEE NaN comparisons are part of the input firewall.

```c
LayeredState_t state;
LayeredConfig_t config;
LayeredInput_t input;
FieldTuning_t field_tuning = {1.0f, 1.0f, 1.0f};
/* Populate config and input from the commissioned device interface. */
Layered_Init(&state, &config, &field_tuning);
LayeredOutput_t output = Layered_Step(&state, &config, &field_tuning, &input);
```

## 2. Nine fast-loop duties

| Order | Duty |
| :---: | :--- |
| 0 | Invalid fast inputs/configuration hold; invalid field knobs or slow mailboxes latch corruption and run closed-loop R4 using valid sensors. |
| 1 | Select R4 when the state envelope or velocity look-ahead trips, with hysteresis. |
| 2 | Latch independent physics-residual OOD veto with release hysteresis. |
| 3 | Latch low confidence or explicit R4 abstention; clear only on a confident normal regime. |
| 4 | Detect stale slow-loop frames and retain the latency fault bit. |
| 5 | Enforce minimum dwell between normal regimes. |
| 6 | Require consecutive clean ticks during recovery cooldown. |
| 7 | Run the controller bank, per-bank tracking anti-windup, transfer smoothing, magnitude and slew limits. |
| 8 | Record sticky fault bits and event counters. |

Priority is envelope, residual, confidence, then native latency HOLD.
All bank integrators track the actually applied command, including inactive banks.
`blend_alpha` smooths the controller target on every control tick after initialization;
it is not a switch-only filter. Magnitude limiting precedes hardware slew limiting.
Nominal feedforward is deterministic and shared by B1/B3 normal banks.
R4 always uses zero reference, fixed bank gains and no feedforward, independently of field tuning.
Its integral and anti-windup use the same zero-state error; `ff_scale[3]` is ignored.
Invalid field knobs use internal nominal scales (1,1,1), including normal-bank tracking integrators.
The nominal residual predictor is frozen independently of supervisor weights.

## 3. Plain-text source map

| File | Role |
| :--- | :--- |
| `c99_layered_core/layered_gatekeeper.h` | ABI, units, regimes, gates and function contract |
| `c99_layered_core/layered_gatekeeper.c` | Reusable deterministic controller |
| `c99_layered_core/test_gatekeeper_standalone.c` | Core behavioral regression |
| `c99_layered_core/verify_core_equivalence.c` | Test-only frame replay |
| `python_bench/minimal_template.py` | Standalone C binding and three device adaptation slots |
| `python_bench/run_benchmark.py` | Physical fixture, full Python mirror, training and experiments |
| `matlab_simulink/load_domain_profile.m` | MATLAB constants and application interface names |
| `matlab_simulink/sim_core_engine.m` | MATLAB physical fixture, deployment logic and core mirror |
| `matlab_simulink/calibrate_coarse_classifier.m` | Small-sample prototype calibration |
| `matlab_simulink/train_continuous_policy.m` | Conditioned continuous action policy |
| `matlab_simulink/run_efficiency_analysis.m` | Action-sample budgets and frozen-AI field tuning |
| `matlab_simulink/verify_core_equivalence.m` | MATLAB/C replay verification |
| `matlab_simulink/run_all_benchmarks.m` | Acceptance run and three figures |
| `matlab_simulink/build_simulink_model.m` | Source for the built-in-block Simulink sketch |

Keep shared physical constants aligned in the existing MATLAB and Python files.
Use the same 32-bit LCG and random-number consumption order in both engines.
The fixture uses Karnopp sticking, unilateral contact and a positive-stiffness, positive-damping two-inertia mode.
S3 measurement noise has one scale and a bounded white distribution; no periodic tone is added.
Stage labels belong only to plant generation, calibration labels and offline statistics.
Deployment controllers and residual probes cannot read stage labels, time schedules or fault frequency.
The broadband innovation envelope and independent spike probe must work without a fault-frequency input.

## 4. Porting sequence

### Step 1 — Define signals, units and fallback

Map all nine input fields: `ai_regime`, `ai_confidence`, `outer_age_s`,
`physics_residual`, `x`, `x_dot`, `r`, `r_dot`, `u_ff`.
State and reference share units; derivatives share units per second.
Residuals and macro-features are dimensionless after device-defined normalization.
Specify the fallback objective and measurable output limits before tuning controllers.
Choose state coordinates whose zero is the commissioned safe fallback point.
The upstream planner or reference governor must limit `r` to the state safety envelope before calling the core.
The benchmark's radians, torques and mechanical reference are one interface example.

### Step 2 — Commission the nominal fast loop

Copy the C99 core unchanged and define the four controller banks in the device adapter.
Commission the nominal model, deterministic feedforward and independent innovation probe.
Verify command magnitude/slew bounds, corruption handling, gate priorities and recovery.
Measure target execution time separately from host ctypes call time.

### Step 3 — Build a small labeled macro-feature archive

`auto_calibrate(samples, labels)` accepts finite normalized feature rows and labels 1, 2 or 3.
Supply every class. It returns `(prototypes, gamma, confidence_trip, confidence_clear)`.
The reference calibration uses short measured closed-loop probes, then samples macro windows.
Window counts and independent experiment counts are distinct quantities.
Freeze the resulting prototypes and confidence parameters before unit-to-unit adaptation.

### Step 4 — Tune the physical knobs

`auto_field_tune(evaluate, current)` calls `evaluate(knobs) -> (cost, valid)`.
Run each candidate inside the C gatekeeper and return a measured tracking cost.
Set `valid=False` on any state, contact, actuator or device-specific constraint violation.
Each candidate must cover load/stiction and contact conditions, including harmonic and ramped references.
Constrain contact force, gate-veto fraction and R4 occupancy; repeat those checks after tuning.
The helper rejects invalid/nonfinite scores and retains `current` when all candidates fail.
Evaluate six fixed joint (gain, damping) candidates: (0.75,1), (1,1), (1.25,1), (2,1),
(1.25,0.5), (1.25,0.75). Each pair is evaluated independently under the same constraints.
The benchmark evaluates six full 40 s missions per branch/unit (240 s of search trajectories).
Candidate validity requires zero breaches throughout, nominal tracking, and normal-stage force,
veto fraction and R4 occupancy limits. Independent validation adds two 6 s S2 runs and a 40 s mission
(52 s per branch/unit). Device trajectory time and host simulation time are separate quantities.
The safety-margin knob stays fixed during this search; its limit comes from the device contract.
The three knobs remain `gain_scale / damping_scale / safety_margin_scale`.
Gain and damping must lie in [0.10,4.0]; safety margin in [0.10,1.0], so tuning only tightens the envelope.
These are input-validity bounds; closed-loop suitability requires the constrained trials above.
Record trial duration and host computation time separately; freeze AI throughout the search.
A new physical domain requires a new signal/model/controller definition before calibration.

## 5. Multi-axis pattern and slow-loop timeout

Create one `LayeredState_t`, residual probe, tuning tuple and slow-frame timestamp per axis/channel.
Share read-only configurations only when units, limits and controller definitions are identical.
A shared supervisor may publish distinct decisions to each channel.
Compute coupled constraints in the device adapter and translate them into per-channel abstention
or a coordinated device fallback. Keep the single-axis C API unchanged.

Compute `outer_age_s = now - source_timestamp` on every fast tick.
Nonfinite/out-of-range regime, confidence or age marks the mailbox corrupt and runs R4;
envelope and residual checks retain priority, with valid sensors driving the closed loop.
The reference timeout is 350 ms. Once stale, the adapter sends explicit R4 with confidence zero
while preserving the real age. This selects deterministic fallback and also records latency.
A fresh frame restores eligibility only after the core's clean recovery cooldown, 300 ms here.
Native `GATE_HOLD_LATENCY` holds the last actuator command, not mechanical position.
Do not describe a held torque, current or stimulation command as a held physical state.
Persistent fast-sensor failure requires a hardware-enforced maximum HOLD duration and safe shutdown
(safe torque off, STO, for a drive); the C core does not supply that independent hardware function.

## 6. Three equal application examples

| Application | Slow interface | Deterministic fast interface |
| :--- | :--- | :--- |
| Industrial WAAM / PLC servo | Process, load and contact regime; abstention | Process/servo regulation, output limits and field tuning |
| Edge VLA / fast-slow robotics | Task and environment context; abstention | Torque/impedance regulation, disturbance rejection and fallback |
| Closed-loop neuromodulation aDBS / SCS | Biomarker and interface regime; abstention | Device-defined regulation, output envelope and timing limits |

These examples map a generic architecture; other devices use the same porting sequence.
Keep physical interfaces and units explicit rather than renaming mechanical states as biological ones.
See [domain mapping](docs/domain_mapping_guide.md) for the adaptation contract.

## 7. Reproduce from the repository root

Python uses only the packages already listed in `python_bench/requirements.txt`.
The minimal template requires a working C99 compiler; build failures propagate directly.
Its dynamic library rebuilds when either the C source or header is newer.
The benchmark additionally offers its complete Python mirror.

```powershell
python -m pip install -r python_bench/requirements.txt
python python_bench/minimal_template.py
python python_bench/run_benchmark.py
python python_bench/run_benchmark.py --no-c
python python_bench/run_benchmark.py --selftest
gcc -std=c99 -pedantic -Wall -Wextra -O2 -o c99_layered_core/test_gatekeeper_standalone.exe c99_layered_core/layered_gatekeeper.c c99_layered_core/test_gatekeeper_standalone.c
.\c99_layered_core\test_gatekeeper_standalone.exe
matlab -batch "cd('matlab_simulink'); verify_core_equivalence"
matlab -batch "cd('matlab_simulink'); run_all_benchmarks"
```

C regression succeeds with no failures and exit code zero. Replay compares gate, regime,
fault bits and corruption flags exactly; command agreement uses the declared floating-point tolerance.
Benchmark checks compare stage metrics, true-state envelope violations, contact force and field tuning.
Do not substitute gate activation counts for physical safety violations.
Compute RMSE from reference and true state at the same tick; TV is the sum of absolute command changes.
Keep physical contact force distinct from actuator torque.

The MATLAB acceptance run regenerates the existing `.slx` and three `assets/` figures.
Keep the model self-contained, with built-in blocks and no workspace initialization dependency.
The B2 sketch places slow-loop transport/hold on context; its action path updates at 1 kHz.
Serialize writes to the generated model. Preserve unsaved user edits in any open model.

## 8. Repository discipline

Keep the core and minimal template small; the template stays within 100–120 lines.
Keep both auto-adaptation helpers together within 20–30 lines.
Replace superseded mechanisms rather than retaining compatibility switches or unused branches.
Add no source/configuration/document files, build systems, CI, dependencies, weights or datasets.
Keep generated binaries, caches and run logs out of version control.
Update both languages of the README together; other documents link to their result tables.
Figures are generated from final numerical runs, never hand-edited to change results.
Report model storage, state RAM, code size and measured host time as separate quantities.
Do not turn synthetic mechanism comparisons into universal claims about learned controllers.
For the scope statement, see the closing paragraph of [README.md](README.md).
