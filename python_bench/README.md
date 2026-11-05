# Python entry point — v2.0.2

This project is a full-stack dual-loop architecture: a 10 Hz qualitative supervisor feeds a 1 kHz
deterministic C99 controller, and four reusable modules carry the integration. Each module is complete
on its own: the Python template and the MCU probe are working references, the core is the deployable
kernel, and the S-Function is the Simulink bridge. This directory hosts the template and the benchmark
harness.

| Module | Size | Role |
| :--- | :--- | :--- |
| [minimal_template.py](minimal_template.py) | 119 lines | 10 Hz qualitative small-sample calibration (`auto_calibrate`, 36 windows / 15 scalars / 60 B), low-confidence abstention, frozen-AI three-knob field tuning (`auto_field_tune`, 20 lines for both helpers) and the SLOT 1..3 integration skeleton |
| [gate_template.c](../c99_layered_core/gate_template.c) | 120 lines | 1 kHz MCU bare-metal ISR template: 32 B `LayeredProbe_t` (median glitch rejection + cascaded high-pass innovation envelope), `blanking_active` hardware blanking window and the `outer_seq` commit-marker pattern |
| [layered_gatekeeper.h](../c99_layered_core/layered_gatekeeper.h) / [layered_gatekeeper.c](../c99_layered_core/layered_gatekeeper.c) | 96 B state | 1 kHz zero-dependency ANSI C99 safety arbitration and four-bank anti-windup control micro-kernel |
| [sfun_layered_gatekeeper.c](../simulink_sfunc/sfun_layered_gatekeeper.c) | 685 lines | Level-2 C-MEX S-Function: `DWork` holds the 96 B state, zero global variables, zero dynamic memory, 11 inputs (10-field ABI + ack rising edge) and 7 outputs |

[run_benchmark.py](run_benchmark.py) is the benchmark harness: it builds the shared physical fixture,
trains B2, runs the branch comparisons and field-tuning experiments, and writes the result tables.
Authoritative numerical results are in [README.md](../README.md) and [README_zh.md](../README_zh.md).
MIT reuse and optional citation are described in the root
[license and citation guide](../README.md#license-and-citation); [CITATION.cff](../CITATION.cff)
contains the author and citation metadata.

## Run

From the repository root:

```powershell
python -m pip install -r python_bench/requirements.txt
python python_bench/minimal_template.py
python python_bench/run_benchmark.py
python python_bench/run_benchmark.py --no-c
python python_bench/run_benchmark.py --selftest
```

The template requires a C99 compiler (`gcc` by default, or `CC`).
It rebuilds the dynamic library when either `layered_gatekeeper.c` or `.h` changes.
Compilation errors propagate directly. The benchmark's `--no-c` option selects its complete
Python core mirror; `--selftest` compares the mirror with compiled C.
Generated libraries and the Python preview figure remain ignored build artifacts.

## Adapt three slots

1. Replace the sensor, trajectory and actuator I/O with the device's measured state and command.
2. Define the commissioned nominal predictor, independent residual probe and normalized features.
3. Supply labeled commissioning windows and a safe device-trial callback.

`Core(config, tuning=(1,1,1))` accepts the exact C configuration names.
Its `step(ai_regime, ai_confidence, outer_age_s, physics_residual, x, x_dot, r, r_dot, u_ff, outer_seq)`
returns `(u, gate, regime, blend, fault_audit_bitmask, corrupted, u_target)`.
The configuration bank fields `kp`, `kd`, `ki` and `ff_scale` each contain four values.
`acknowledge_fault_latch()` calls `Layered_AcknowledgeFaultLatch()` and is the only release path for
`GATE_LATCHED_SAFE`; the lifetime veto counters and the 7-bit `fault_audit_bitmask` survive the call.

`auto_calibrate(samples, labels)` expects four dimensionless features and labels 1–3.
It returns three prototypes and three confidence parameters.
Freeze these parameters, then call `auto_field_tune(evaluate, current)`.
The trial callback returns `(measured_cost, valid)`; reject constraint violations.
Include load/stiction and contact trials, contact-force limits, veto fraction and R4 occupancy.
Invalid candidates cannot win, and an entirely invalid search returns the current knobs.
The fixed six-trial joint grid uses gain/damping pairs `(0.75,1)`, `(1,1)`, `(1.25,1)`,
`(2,1)`, `(1.25,0.5)` and `(1.25,0.75)`. Safety margin stays fixed.
Each pair is evaluated independently: an infeasible higher-damping trial does not exclude its
lower-damping counterpart. Gain/damping lie in [0.10,4.0], margin in [0.10,1.0]. R4 ignores these gain
scalings, trajectory references and feedforward; it uses fixed feedback toward the safe zero.

Frame age advances every tick from the source timestamp. With `outer_seq` nonzero the core measures age
on its own 1 kHz clock: a new sequence opens a fresh lease, an unchanged one ages by one tick, and a
frozen or forged `outer_age_s` cannot mask a stalled mailbox; 0 selects the legacy age mode.
On timeout the adapter sends R4 with zero confidence while retaining the real age.
Native HOLD retains an actuator command and is bounded by `max_hold_ticks`, after which the core enters
`GATE_SAFE_ZERO` and ramps the command to zero in steps of `max(u_slew_max_per_s * dt_s, 1 binary32 ULP(|u_prev|))`, so a legal sub-ULP slew budget still de-energises in finite time.
Malformed slow mailbox fields or invalid field knobs run closed-loop R4 with `corrupted=1`.
Invalid knobs use internal nominal scales; invalid configuration or fast sensors retain native HOLD.
Three safety/OOD veto episodes latch `GATE_LATCHED_SAFE` permanently until acknowledged.
Count safety violations from true/measured state, independently of gate activity.

The template commissions three viscous-load classes from measured velocity changes and applied effort.
Its smooth trajectory starts at rest. PASS requires zero state-envelope violations, zero abnormal
gate/R4/corruption ticks and RMSE below 0.02 state units; the frozen classifier also serves field tuning.

## Benchmark contract

The three branches are white-box feedforward plus PID, conditioned RBF continuous action,
and a coarse supervisor plus deterministic gatekeeper. Both learned continuous actions and C control
run at 1 kHz; only context/supervisor updates run at 10 Hz.
The physical fixture combines Karnopp friction, unit drift, moving unilateral contact,
white measurement noise and a variable-frequency flexible mode with positive stiffness and damping.
The benchmark regenerates its training samples and classifier archive deterministically.
It reports the four-stage metrics, mode ablations, frozen-AI tuning and action-sample efficiency.
Per branch/unit, six full 40 s candidate missions provide 240 s of search trajectories;
independent validation uses two 6 s S2 runs plus a 40 s mission, totaling 52 s.
Search candidates must have zero breaches, nominal tracking and acceptable normal-stage contact
force, veto fraction and R4 occupancy. Host simulation time is reported separately from trajectory time.
The deployment path reads measured features; stage labels are confined to plant generation and scoring.

The layered branch records S4 RMSE 0.539985 rad and TV 1545.01 N·m in the main comparison.
C99 standalone regression passes 245 checks with zero failures, and the 120-line `gate_template.c`
self-check prints PASS. Python/C `--selftest` replays 6,000 frames (R1 = 901, R2 = 900, R3 = 304,
R4 = 3,895, including acknowledgement re-arming and the 7→5→3→1→0 ramp-down) with identical discrete
outputs and max |du| = 0, bit-exact in binary32. MATLAB/C `verify_core_equivalence` covers 40,000 normal
frames, 40,000 corrupted-mailbox frames and four 40,000-frame runs with illegal knob injection at
max |du| = 5e-9; a 10,000-tick `outer_seq` lease replay covering HOLD → `SAFE_ZERO` and the
zero-sequence corruption path stays within max |du| = 4.19e-8. The Level-2 S-Function replays 1,100
frames with all seven outputs at max diff = 0.0.
Object text measures 6,572 B at `-Os` and 7,744 B at `-O2` with GCC 4.9.2 TDM64 on x86-64.
The ABI occupies 96/140/12/40/28 B for state/config/tuning/input/output, plus the 32 B probe.

Porting order, multi-axis ownership and timeout behavior are specified in
[AGENTS.md](../AGENTS.md). The domain mapping contract is in
[domain_mapping_guide.md](../docs/domain_mapping_guide.md).
