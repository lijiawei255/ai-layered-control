# ai-layered-control

[English](README.md) | [简体中文](README_zh.md)

[![Release v1.1.0](https://img.shields.io/badge/Release-v1.1.0-blue.svg)](https://github.com/lijiawei255/ai-layered-control/releases)
[![ANSI C99 zero-heap core](https://img.shields.io/badge/Core-ANSI%20C99%20%28Zero--Heap%29-00599C.svg)](c99_layered_core/)
[![Python and MATLAB/Simulink benchmark](https://img.shields.io/badge/Bench-Python%20%7C%20MATLAB%2FSimulink-2ea44f.svg)](python_bench/)
[![MIT License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**A reference architecture for closed-loop AI control: a slow, coarse AI classifier decides *which* control regime a machine is in; a fast, zero-dependency deterministic core decides *how* to move the actuator. In the layered path, no end-to-end `u = pi(x)` model ever touches the 1 kHz command path.**

**TL;DR**

* **A few dozen calibration runs, not a thousand.** The 10 Hz coarse classifier maps four dimensionless macro-features onto one of three damping schedules. It reaches **100 % deployment quality with N = 40** calibration samples. The paired continuous regression needs ~120–500 nominal-unit samples for the same in-distribution quality.
* **+35 % unit-to-unit drift, zero retraining.** On the four-stage benchmark the layered branch tracks at **0.0057 rad RMSE in S2** versus **0.1585 rad** for the end-to-end branch. The continuous policy trained only on nominal data is at **0 %** deployable success on the +35 % drifted unit even at N = 2000; a drifted-data oracle recovers to **100 %** at N = 120.
* **Explicit abstention on an unseen OOD fault.** **0** safety-envelope violation ticks for the layered branch in S4, versus **2,574** for the end-to-end branch; the OOD veto latches **2.0 ms** after fault onset. The same 2.0 ms is measured for the resonance-only and sensor-spike-only probes, while S1–S3 carry **0** OOD veto ticks under 20× broadband measurement noise.
* **A stage-forced mode-necessity ablation.** Dynamic selection gives **0.00710** S1–S3 RMSE, versus **0.01201 / 0.00935 / 0.05501** for any fixed R1 / R2 / R3; the 40 s totals are **0.01721** versus **0.01798 / 0.01777 / 0.05069**. The deployed 10 Hz classifier lands at **0.00700** / **0.01630**, marginally better than the stage-forced bound.
* **A real trained baseline.** Branch 2 is a closed-form RBF inverse-dynamics model fitted to a nominal-unit teacher rollout plus a fixed PD stabilizer. Branch 3's 3×4 prototypes and confidence threshold are commissioned by `calibrate_coarse_classifier.m`; no prototype table is hard-coded.

> **Scope.** This is a methodological reference implementation on a synthetic nonlinear plant; see the single **Scope and disclaimer** statement at the end. All numbers below are produced by the acceptance run in §6.

---

## 1. The architecture

```text
+------------------------------ slow outer loop: 10 Hz ------------------------------+
|  4 dimensionless macro-features -> coarse 3-class classifier -> regime in {1,2,3}  |
|  (rms error, rms effort, max(slow residual, HF residual), rms velocity)             |
|  margin x proximity confidence  -- collapses on ambiguity and on novelty            |
|  independent nominal observer -> residual probe                                    |
+-------------------------------------------------------------------------------------+
               |  mailbox: {regime, confidence, frame age}
               v
+------------------------------ fast inner loop: 1 kHz ------------------------------+
|  0  input firewall   NaN/Inf/out-of-range frame -> hold actuator, latch audit bit   |
|  1  safety envelope  |x| > 90 % of limit, or |x| + 60 ms*|xdot| > limit -> SAFE      |
|  2  OOD residual     spike channel OR narrowband HF channel -> ABSTAIN             |
|  3  confidence       confidence below calibrated threshold or AI self-abstains      |
|  4  heartbeat        stale frame -> GATE_HOLD_LATENCY, command and state frozen     |
|  5  dwell            150 ms minimum between normal regime changes                   |
|  6  cooldown         500 ms of consecutive clean ticks before the AI is re-armed    |
|  7  controller bank  4 x PID + per-bank deterministic feedforward scale, anti-windup|
|  8  clamps           bumpless transfer, |u| <= u_max, |du| <= slew_rate*dt           |
|  9  audit            5 sticky fault bits: why the unit went safe, without the AI    |
+-------------------------------------------------------------------------------------+
               |
               v
      actuator   +   u_ff = white-box plant inverse from the commissioning model
```

Zero heap. No `math.h`. No recursion. No OS calls. Straight-line code, O(1) worst-case execution time. `LayeredState_t` is 88 B; `LayeredConfig_t` is 132 B. The AI is allowed to be slow, jittery and occasionally absent; the inner loop never waits on it.

The key v1.1.0 detail is the **orthogonal residual probe**:

* **spike channel** — median-5 deviation of the position measurement, normalized by `res_fast_scale`;
* **HF resonance channel** — a 2nd-order narrowband resonator at the profile's OOD mode (42 Hz industrial), 8 Hz bandwidth, 4 ms RMS energy, 2-tick confirmation;
* **slow channel** — LP(innovation), used only as a classifier feature.

The gate receives `max(res_fast, res_hf_gated)`. This is why 20× broadband measurement noise (S3) does not trip the OOD veto, while a 42 Hz structural resonance does.

## 2. Headline numbers

Industrial reference preset, 40 s, 4 stages, 1 kHz inner loop, three branches on the identical plant, reference, noise realisation and fault schedule:

| Metric | Branch 1: fixed PID | Branch 2: end-to-end AI | Branch 3: layered |
| :--- | ---: | ---: | ---: |
| RMSE S1 fast tracking [rad] | 0.0566 | 0.0397 | **0.0077** |
| RMSE S2 +35 % load/Stribeck [rad] | 0.1790 | 0.1585 | **0.0057** |
| RMSE S3 contact/HF noise/edge latency [rad] | 0.1921 | 0.1670 | **0.0074** |
| RMSE S4 unseen OOD [rad] | 0.1693 | 0.8308 | **0.0303** |
| Safety Violation Ticks S4 [ms] | 0 | 2574 | **0** |
| OOD veto latency | - | - | **2.0 ms** |
| Calibration Samples N (classifier / continuous) | - | ~500 | **≤ 40** |
| Compute (MAC/s) | - | 2.40e9 | **2.41e4** |
| Memory footprint | - | 5.60e6 B | **2.21e3 B** |

`Safety Violation Ticks` are 1 kHz samples with `|x| > x_limit`; at `dt = 1 ms` the count equals milliseconds. `Control Effort TV(u) = Σ|u_k − u_{k−1}|` is the actuator-command total variation. `N` is the number of calibration samples used to fit the classifier or the continuous head.

**Control Effort TV(u), S1–S3:** layered **0.41e3 N·m** vs 1.24e3 (B1) and 1.33e3 (B2). In S4 the layered safe-fallback regulator spends **1.97e3 N·m** because it holds the axis inside the envelope while the OOD mode is active; the end-to-end branch has lower S4 TV but 2,574 violation ticks. Read TV together with the safety column, never alone.

## 3. Mode-necessity ablation

`run_all_benchmarks` runs three fixed-bank 40 s simulations and one stage-forced oracle (S1→R1, S2→R2, S3→R3, S4→safe fallback), plus the deployed 10 Hz classifier:

| controller | S1 | S2 | S3 | S4 | S1–S3 | 40 s |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: |
| Stage-forced oracle | 0.0083 | 0.0056 | 0.0071 | 0.0322 | **0.00710** | **0.01721** |
| Fixed R1 | 0.0083 | 0.0077 | 0.0175 | 0.0293 | 0.01201 | 0.01798 |
| Fixed R2 | 0.0095 | 0.0058 | 0.0117 | 0.0316 | 0.00935 | 0.01777 |
| Fixed R3 | 0.0938 | 0.0150 | 0.0072 | 0.0346 | 0.05501 | 0.05069 |
| Deployed 10 Hz classifier | 0.0077 | 0.0057 | 0.0074 | 0.0303 | 0.00700 | 0.01630 |

The oracle is the ideal-classifier bound; the deployed classifier is within 2 % of its S1–S3 RMSE (0.00700 vs 0.00710). The stage-forced dynamic passes the `<0.8 ×` fixed-bank criterion (`0.00710 / 0.00935 = 0.759`) and the 40 s criterion.

## 4. Paired sample complexity (Panel A)

Both learners receive the **same N paired samples from the nominal unit** (`drift = 0`); the test deployment is the same unit at 0 % drift and a +35 % drifted unit. The oracle is the same 495-term continuous head retrained on +35 % drifted data.

| N | Layered @ +35 % | Continuous nominal-only @ 0 % | Continuous nominal-only @ +35 % | Oracle @ +35 % |
| ---: | ---: | ---: | ---: | ---: |
| 15 | **100.0 %** | 78.1 % | 3.1 % | 59.4 % |
| 40 | **100.0 %** | 84.4 % | 0.0 % | 71.9 % |
| 120 | **100.0 %** | 96.9 % | 3.1 % | **100.0 %** |
| 500 | **100.0 %** | 100.0 % | 6.2 % | 100.0 % |
| 2000 | **100.0 %** | 100.0 % | **0.0 %** | 100.0 % |

The shape is the argument: the coarse classifier saturates at N = 40 on a unit it never saw; the continuous regression trained only on nominal data never recovers on the drifted unit, even with 2,000 samples. The oracle recovers at N = 120, proving that the gap is **data coverage**, not model capacity.

## 5. The three figures

![Four-stage showdown: state tracking inside the safety envelope, actuator command, residual and confidence, regime and gate timeline](assets/benchmark_4stages_showdown.png)

**Figure 1 — 40 s, four stages, three branches.** (a) state tracking against the hard envelope; (b) actuator command; (c) spike and HF residual channels with the AI confidence; (d) active regime and gate timeline. The S4 transition into `REGIME_SAFE_FALLBACK (4)` is a named decision, not a silent clamp.

![Paired sample complexity, zero-retraining drift robustness, compute and memory footprint](assets/efficiency_and_generalization.png)

**Figure 2 — paired data efficiency.** (a) layered classifier vs nominal-only continuous head vs +35 %-trained oracle; (b) full-loop drift sweep with the AI frozen; (c) compute, memory and inference rate.

![Cross-domain trajectories for industrial servo, edge VLA and closed-loop neuromodulation](assets/domain_profiles_comparison.png)

**Figure 3 — the same architecture on three domains.** Only the physical constants, stage names and envelope units change; the C99 core and gate semantics do not.

## 6. Run and verify

### Full MATLAB acceptance run

```powershell
matlab -batch "cd('matlab_simulink'); run_all_benchmarks"
```

Success criteria:

| Signal | Expected |
| :--- | :--- |
| checks `[1]`–`[5]` | all `PASS` |
| final line | `All five engineering checks passed.` |
| figures | `assets/benchmark_4stages_showdown.png`, `assets/efficiency_and_generalization.png`, `assets/domain_profiles_comparison.png` rewritten at 240 dpi |
| model | `matlab_simulink/ai_layered_control_bench.slx` rebuilt (built-in blocks only) if Simulink is installed and the model is not open with unsaved edits |
| exit code | `0` |

If Simulink is not installed, the numeric benchmark and all five checks still run; the `.slx` step prints a skip line. A loaded model with unsaved edits is never closed or overwritten.

### C99 core regression test

```powershell
cd c99_layered_core
gcc -std=c99 -pedantic -Wall -Wextra -O2 -o test_gatekeeper_standalone.exe layered_gatekeeper.c test_gatekeeper_standalone.c
.\test_gatekeeper_standalone.exe
$LASTEXITCODE
```

Expected: `SUMMARY: n_pass=116, n_fail=0`, exit code 0, no compiler diagnostics.

### C99 ↔ MATLAB closed-loop equivalence

```powershell
matlab -batch "cd('matlab_simulink'); verify_core_equivalence"
```

Expected: `PASS: 6000 closed-loop ticks; gate/regime/fault exact; max|du|=4.82e-07, max|du_target|=4.82e-07, max|dblend|=9.5e-08` (tolerances `1e-4`). No compiler => `SKIP`.

### 3-step C99 porting

1. Copy `c99_layered_core/layered_gatekeeper.c` and `.h` into the target; compile as ANSI C99 (`-std=c99`), with no heap, no `libm`, no recursion.
2. Fill `LayeredConfig_t` with the target's units, limits and bank gains, then call `Layered_Init(&state, &config)` once at boot.
3. In the 1 kHz (1 ms) ISR fill `LayeredInput_t` and call `Layered_Step(&state, &in, &out)`; write `out.u` to the actuator and persist/log `out.gate`, `out.regime` and the sticky fault bits.

### Python-only quickstart (no MATLAB)

```powershell
python -m pip install -r python_bench/requirements.txt
python python_bench/run_benchmark.py
```

The entry tries to compile `c99_layered_core/layered_gatekeeper.c` with `ctypes` (Windows `.dll`, macOS `.dylib`, Linux `.so`); if no compiler is present it prints one line and falls back to the pure-Python/NumPy gatekeeper mirror. `--no-c` forces the mirror; `--selftest` replays 500 deterministic frames through both implementations and requires exact gate/regime and `|du| <= 1e-5`.

The **1 kHz C99 gatekeeper is frame-for-frame equivalent** between the MATLAB engine and the Python implementation (`--selftest` max `|du| = 1.55e-07 < 1e-6`). The standalone Python benchmark's 10 Hz outer probe uses adaptive noise floors and a confidence EMA; therefore its B3 S1–S4 RMSE is **0.0090 / 0.0053 / 0.0343 / 0.0873**, OOD veto **9.0 ms** (< 10 ms), and its safety-violation tick count is **0** in every stage. Those Python-run numbers intentionally differ from the MATLAB 40 s table; the gate semantics and C core do not.

### Simulink model as a standalone artifact

```powershell
matlab -batch "clearvars; sim('matlab_simulink/ai_layered_control_bench.slx'); disp('model runs standalone')"
```

The model is built from built-in blocks only, with every numeric parameter inlined. It is a visualization of the three branches; the quantitative benchmark lives in `sim_core_engine.m`.

## 7. The three domain presets

| Domain | S1 | S2 | S3 | S4 |
| :--- | :--- | :--- | :--- | :--- |
| **industrial WAAM** | Fast torch/axis tracking (light deposition load) | +35 % layer-heat equivalent load drift & wire-feed Stribeck friction | Arc/contact stiffness + arc-voltage HF noise + fieldbus latency | Unseen structural resonance & sensor spikes |
| **edge VLA** | High-speed free-space motion (light payload) | +35 % payload drift & joint static friction | Contact stiffness step + vision/force HF noise + 10 Hz edge inference jitter | Unseen structural resonance & sensor spikes |
| **neuromodulation** | Fast phasic biomarker tracking | +35 % electrode–tissue impedance drift & activation-threshold nonlinearity | High-coupling neural rebound + EMG/artifact HF noise + telemetry jitter | Unseen OOD pathological oscillation & lead micro-dislodgement spikes |

The shared dimensionless fields are `friction_scale`, `k_scale`, `noise_scale` and per-bank `ff_scale`; the domain comments in `load_domain_profile.m` and `docs/domain_mapping_guide.md` use the domain's own physics. Neuromodulation never uses mechanical-arm wording.

+35 % drift in the industrial preset means: `J ×1.35, b ×1.25, k ×1.10, fc ×2.00, fs ×2.40, vs ×0.25, fvisc ×1.20`.

## 8. Repository contents

```text
ai-layered-control/
├── README.md / README_zh.md            this file / 中文版
├── AGENTS.md                           machine-readable invariants, playbook, commands
├── docs/
│   ├── systems_engineering_manifesto_en.md / _zh.md
│   └── domain_mapping_guide.md
├── c99_layered_core/
│   ├── layered_gatekeeper.h            88 B state / 132 B config contract
│   ├── layered_gatekeeper.c            zero-dependency 1 kHz core
│   ├── test_gatekeeper_standalone.c    116 deterministic checks
│   └── verify_core_equivalence.c       closed-loop C replay harness (test-only)
├── matlab_simulink/
│   ├── load_domain_profile.m
│   ├── sim_core_engine.m
│   ├── calibrate_coarse_classifier.m
│   ├── train_continuous_policy.m
│   ├── run_efficiency_analysis.m
│   ├── run_all_benchmarks.m
│   ├── build_simulink_model.m
│   ├── verify_core_equivalence.m
│   └── ai_layered_control_bench.slx    generated artifact
├── python_bench/
│   ├── run_benchmark.py                ctypes C core or NumPy fallback
│   └── requirements.txt
└── assets/                             three generated 240 dpi figures
```

Plain text: C99, MATLAB, Python, Markdown. No committed weights or datasets: the RBF policy is fitted deterministically at commissioning time from a synthetic nominal-unit rollout; the classifier prototypes are fitted from three stage probes; the C core is the deployable artifact.

## 9. Scope and disclaimer

This repository is a methodological reference implementation of a fast/slow-loop decoupled control architecture: a coarse semantic classifier, an independent physics-residual abstention probe, and a deterministic 1 kHz safety gate. The `neuromodulation` (aDBS/SCS) preset is a lumped-parameter synthetic nonlinear-dynamics and isomorphic biomarker-envelope model; it contains no patient records and is not a clinical physiological or pathological model of any specific disease. The C99 core follows defensive embedded principles (zero dynamic allocation, O(1) worst-case execution, NaN firewall, sticky fault audit), but it has not been certified and makes no claim of compliance with IEC 62304, ISO 14971, IEC 60601, or IEC 61508. Any clinical implant/wearable neuromodulation or high-risk industrial deployment must add independent hardware charge-balance protection, an external watchdog, and complete clinical/regulatory validation.

## 10. Citation

If you use this reference architecture or benchmark in your work, please cite:

```bibtex
@software{li2026ailayeredcontrol,
  author       = {Li, Jiawei},
  title        = {ai-layered-control: A Systems-Engineering Reference Architecture for Layered Closed-Loop AI Control},
  year         = {2026},
  version      = {1.1.0},
  url          = {https://github.com/lijiawei255/ai-layered-control},
  license      = {MIT}
}
```

**Sister project:** [jev-tune](https://github.com/lijiawei255/jev-tune) — continuous parameter-layer tuning inside a single controller, with a C99 safety-boundary fallback.