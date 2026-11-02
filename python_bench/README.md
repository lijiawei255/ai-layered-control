# Python entry point — v2.0.0

The reusable implementation is the [C99 core](../c99_layered_core/layered_gatekeeper.h).
[minimal_template.py](minimal_template.py) supplies a compact C binding, the slow/fast loop and
two adaptation helpers. [run_benchmark.py](run_benchmark.py) runs the shared physical benchmark.
Authoritative numerical results are in [README.md](../README.md) and [README_zh.md](../README_zh.md).

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
Its `step(regime, confidence, age, residual, x, x_dot, r, r_dot, u_ff)` returns
`(u, gate, regime, blend, fault, corrupted, target)`.
The configuration bank fields `kp`, `kd`, `ki` and `ff_scale` each contain four values.

`auto_calibrate(samples, labels)` expects four dimensionless features and labels 1–3.
It returns three prototypes and three confidence parameters.
Freeze these parameters, then call `auto_field_tune(evaluate, current)`.
The trial callback returns `(measured_cost, valid)`; reject constraint violations.
Invalid candidates cannot win, and an entirely invalid search returns the current knobs.
The search adjusts gain and damping while retaining the commissioned safety-margin scale.

Frame age advances every tick from the source timestamp.
On timeout the adapter sends R4 with zero confidence while retaining the real age.
C's native HOLD retains an actuator command; the device fallback defines the desired physical behavior.
Count safety violations from true/measured state, independently of gate activity.

## Benchmark contract

The three branches are white-box feedforward plus PID, conditioned RBF continuous action,
and a coarse supervisor plus deterministic gatekeeper. Both learned continuous actions and C control
run at 1 kHz; only context/supervisor updates run at 10 Hz.
The physical fixture combines Karnopp friction, unit drift, moving unilateral contact,
white measurement noise and a passive variable-frequency flexible mode.
The benchmark regenerates its training samples and classifier archive deterministically.
It reports the four-stage metrics, mode ablations, frozen-AI tuning and action-sample efficiency.
The deployment path reads measured features; stage labels are confined to plant generation and scoring.

Porting order, multi-axis ownership and timeout behavior are specified in
[AGENTS.md](../AGENTS.md). Scope is stated at the end of the root README.
