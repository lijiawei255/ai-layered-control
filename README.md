# ai-layered-control

[English](README.md) | [简体中文](README_zh.md)

**A reference architecture for closed-loop AI control: a slow, coarse AI classifier decides *which*
control regime a machine is in; a fast, zero-dependency deterministic core decides *how* to move the
actuator. No end-to-end `u = pi(x)` model ever touches the 1 kHz path.**

**TL;DR**

* **A few dozen calibration runs, not a thousand.** The 10 Hz AI only has to map four normalised
  macro-features onto one of three damping schedules: it reaches 100 % deployment quality with 40
  calibration runs. The end-to-end alternative needs about 120 runs on the unit it was trained on - and
  scores 0 % on the same task once that unit is deployed with +35 % drift, at any sample count.
* **~100,000x less compute.** 2.41e4 MAC/s and 2.21e3 B, against 2.40e9 MAC/s and 5.60e6 B for a
  1.2 M-parameter policy running at 1 kHz (99,502x / 2,536x). It fits a microcontroller that is already
  on the board.
* **+35 % unit-to-unit drift, zero retraining.** RMSE 0.0122 rad vs 0.0516 rad for the end-to-end
  branch (4.22x). Across a full 0 -> +50 % drift sweep the AI stays frozen: 0.0111 -> 0.0151 rad.
* **Explicit abstention on an unseen OOD fault.** Zero envelope excursions in the four-stage synthetic
  benchmark of this repository, against 1,733 for the end-to-end branch; the layered controller latches
  its abstention 2.0 ms after fault onset and holds the axis inside the envelope.
* **4.2-4.9x less actuator travel.** Total variation TV(u) over 40 s: 342.0 N*m vs 1,442.9 N*m
  (end-to-end) and 1,664.8 N*m (fixed PID). Same task, a fraction of the wear.

> **Scope, before you read the numbers.** This is a *synthetic* benchmark: a second-order plant with
> Stribeck friction, simulated at `dt = 1 ms` for 40 s under a fixed disturbance schedule, with a frozen
> computed-torque policy standing in for a learned continuous controller. The compute and memory figures
> are analytic (weights x rate), not profiled on hardware. Nothing here is a claim about a specific
> machine. What is real: the C99 core (105 deterministic assertions, 0 failures), the four engineering
> checks, and the fact that every number below is produced by the acceptance run in section 6 (the
> console prints some of them to one decimal; the full-precision values live in the returned structs).

---

## 1. The architecture in one picture

```text
+-------------------------------- slow outer loop: 10 Hz --------------------------------+
|  4 normalised macro-features -> coarse 3-class classifier -> regime in {1, 2, 3}       |
|  (rms error, rms effort, slow residual, rms velocity; all dimensionless)               |
|  margin confidence d2/(d1+d2) in [0, 1]  -- collapses where the call is ambiguous      |
|  frozen nominal observer -> residual r(t) = |y - y_hat| -- independent OOD evidence    |
+----------------------------------------------------------------------------------------+
              |  mailbox: {regime, confidence, frame age}   (may be late, missing, bad)
              v
+-------------------------------- fast inner loop: 1 kHz --------------------------------+
|  0  input firewall   NaN/Inf/out-of-range frame -> hold actuator, latch audit bit      |
|  1  safety envelope  |x| > 90 % of limit, or |x| + 60 ms*|xdot| > limit -> SAFE        |
|  2  OOD residual     residual above threshold -> ABSTAIN (GATE_VETO_OOD_RESIDUAL)      |
|  3  confidence       confidence < 0.45 or AI self-abstains -> ABSTAIN                  |
|  4  heartbeat        stale frame -> GATE_HOLD_LATENCY, command and state frozen        |
|  5  dwell            150 ms minimum between normal regime changes (anti-chatter)       |
|  6  cooldown         500 ms of consecutive clean ticks before the AI is re-armed       |
|  7  controller bank  4 x PID + deterministic feedforward u_ff, per-bank anti-windup    |
|  8  clamps           bumpless transfer, |u| <= u_max, |du| <= slew_rate * dt           |
|  9  audit            5 sticky fault bits: why the unit went safe, without the AI       |
+----------------------------------------------------------------------------------------+
              |
              v
      actuator     +     u_ff = white-box plant inverse from the commissioning model
```

Zero heap. No `math.h`. No recursion. No OS calls. One 84 B state struct, straight-line code, O(1)
worst-case execution time. The AI is allowed to be slow, jittery and occasionally absent; the inner loop
never waits on it.

## 2. Headline numbers

Industrial reference preset, 40 s, 4 stages, 1 kHz inner loop, three branches on the identical plant,
reference, noise realisation and fault schedule:

| Metric | Branch 1: fixed PID | Branch 2: end-to-end AI | Branch 3: layered |
| :--- | ---: | ---: | ---: |
| RMSE S1 nominal [rad] | 0.0733 | 0.0087 | 0.0098 |
| RMSE S2 +35 % drift [rad] | 0.1266 | 0.0516 | 0.0122 |
| RMSE S3 edge compute limited [rad] | 0.1442 | 0.0933 | 0.0125 |
| RMSE S4 unseen OOD fault [rad] | 0.1372 | 0.6152 | 0.0295 |
| Phase-lag chattering index S3 [norm.] | 0.089 | 0.123 | 0.013 |
| Actuator total variation TV(u), 40 s [N*m] | 1664.8 | 1442.9 | 342.0 |
| Envelope excursions S1 / S2 / S3 / S4 [count] | 0 / 0 / 0 / 0 | 0 / 0 / 0 / **1733** | 0 / 0 / 0 / 0 |
| OOD abstention response time | - | - | **2.0 ms** |
| Calibration samples required | - | ~120 (nominal unit) | <= 50 |
| Compute (MAC/s) | - | 2.40e9 | 2.41e4 |
| Memory footprint | - | 5.60e6 B | 2.21e3 B |

All four engineering checks hold - the run ends with `All four engineering checks passed.`:

1. **S2 anti-overfitting:** 0.0516 vs 0.0122 rad (4.22x), AI frozen, +35 % uncalibrated drift.
2. **S3 latency decoupling:** chattering 0.123 vs 0.013 (9.50x); RMSE 0.0933 vs 0.0125 rad under 10 Hz,
   60-120 ms jitter and 15 % dropped frames.
3. **S4 OOD abstention:** 1,733 vs **0** envelope excursions in the four-stage synthetic benchmark of
   this repository, veto latched 2.0 ms after onset, safe-fallback regulator for the rest of the stage.
4. **Data and compute:** the coarse classifier is already at 100 % deployment quality with `N <= 50`
   (saturated at `N = 40`); the continuous policy needs about `N = 120` on the nominal unit (81.2 % at
   `N = 40`, 100 % at `N = 120`) and stays at 0 % when the same unit is deployed with +35 % drift, even
   at `N = 2000`; 99,502x compute and 2,536x memory reduction.

Sample-complexity curves (`N = 15 / 40 / 120 / 250 / 500 / 1000 / 2000`). Deployment quality = the
damping actually applied stays within 10 % of the closed-loop optimum at that operating point, with the
150 ms classifier dwell applied to the per-frame decisions first:

| Learner / deployment condition | 15 | 40 | 120 | 250 | 500 | 1000 | 2000 |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Layered coarse classifier, deployed at +35 % drift | 87.5 % | **100 %** | 100 % | 100 % | 100 % | 100 % | 100 % |
| Continuous policy, nominal unit (0 % drift) | 28.1 % | 81.2 % | 100 % | 100 % | 100 % | 100 % | 100 % |
| Continuous policy, deployed at +35 % drift | 3.1 % | 0 % | 0 % | 0 % | 0 % | 0 % | 0 % |

The shape of that table is the argument in one glance: the coarse classifier saturates within a few
tens of calibration runs, the continuous policy needs ~120 runs on the machinery it was trained on, and
on a unit with +35 % drift it never clears the bar - extra nominal-unit data does not help, because the
constant it is missing is physical, not statistical.

Zero-retraining drift sweep (AI frozen, uncalibrated plant drift 0 / 17 / 33 / 50 %): the end-to-end
branch rises 0.0101 -> 0.0783 rad (7.75x); the layered branch stays at 0.0111 -> 0.0151 rad (1.36x).

## 3. The three figures

![Four-stage showdown: state tracking inside the safety envelope, actuator command, physics residual and confidence, regime and gate timeline](assets/benchmark_4stages_showdown.png)

**Figure 1 - 40 s, four stages, three branches.** (a) state tracking against the hard envelope;
(b) actuator command; (c) normalised fast residual and coarse-AI confidence for the layered branch;
(d) active regime and gate-state timeline. Read (d) for the whole story: steady `1 -> 2 -> 3` regime
changes, then at `t = 30 s` a clean switch into `REGIME_SAFE_FALLBACK (4)` with the gate latched at
`2` (OOD residual) - an explained decision, not a silent clamp.

![Sample complexity of coarse classification vs a continuous policy, zero-retraining drift robustness, and compute / memory footprint on a log scale](assets/efficiency_and_generalization.png)

**Figure 2 - why the layering is cheap.** (a) deployment quality against the number of calibration runs:
the coarse classifier is already at 100 % with 40 runs, the continuous policy needs about 120 runs on
the unit it trained on, and it sits at 0 % once that unit is deployed with +35 % drift - at any sample
count. (b) with the AI frozen, the layered branch stays flat across the drift sweep while the end-to-end
branch rises monotonically. (c) compute, memory and inference rate on a log scale (99,502x / 2,536x).

![Cross-domain trajectories for industrial servo, edge VLA and closed-loop neuromodulation](assets/domain_profiles_comparison.png)

**Figure 3 - the same architecture on three domains.** State and command traces for industrial
servo/WAAM, the edge VLA preset (10 Hz policy over a 1 kHz joint loop) and closed-loop neuromodulation.
Only the physical constants change; the architecture does not.

## 4. Five engineering bottlenecks of end-to-end continuous control - and the structural fix

The failure of a monolithic `u = pi_theta(x)` policy in a real loop is not a model-capacity problem. It
is five separate structural problems, and each one is dissolved by *where* the AI sits, not by making the
AI bigger.

The research lineage matters here. Most AI-plus-control work in the literature stays inside one
continuous controller - tuning its parameters, or replacing it with an end-to-end network that regresses
the physical inverse dynamics directly. This repository proposes the step *above* that layer: instead of
fitting a continuous parameter manifold, it arbitrates between a small set of discrete structures and
rejects the operating points it does not recognise.

### 4.1 Boundary ill-conditioning: the regression target is discontinuous where it matters

*Symptom.* Hunting and steady-state error at friction reversal, contact transition, backlash, actuator
saturation.

*Cause.* Inverse dynamics blow up on a non-smooth boundary. In this bench the break-away friction
transition is 0.4 mrad/s wide on a 0.45 N*m jump, so the inverse gain there is set by physics you cannot
average over - and any finite-capacity learner must smooth the jump. The smoothing is wrong the moment
the boundary moves.

*Fix.* The AI does not invert the plant. It selects one of 4 pre-verified controller banks; the
plant-inverse term `u_ff` stays a white-box commissioning model; an integrator with tracking anti-windup
absorbs the unknown bias. **Evidence:** S2 RMSE 0.0122 vs 0.0516 rad.

### 4.2 Data hunger and unit-to-unit drift

*Symptom.* Works on the pilot machine, fails on machine #2 by 30-40 %; fixing it needs a dataset and a
GPU, per unit.

*Cause.* A continuous policy encodes the *constants* of the plant it trained on; a classifier on
normalised features encodes the *shape* of the operating point. Sample complexity differs by two orders
of magnitude, and only the second transfers across units.

*Fix.* Frozen AI plus three scalar field knobs (`gain_scale`, `damping_scale`, `safety_margin_scale`).
**Evidence:** 100 % deployment quality from 40 calibration runs against ~120 runs for the continuous
policy on its own unit, and 0 % for that same policy once the unit carries +35 % drift; 0.0111 -> 0.0151
rad across the +50 % drift sweep with no retraining.

### 4.3 Compute, power, and latency jitter inside the loop

*Symptom.* The loop runs late, phase margin disappears, the axis chatters, the edge box throttles.

*Cause.* A large model at 1 kHz is 2.4e9 MAC/s with data-dependent inference time; a 10 Hz policy puts
60-120 ms of transport delay inside the feedback path. At a 3 Hz crossover that costs 65-130 deg of pure
delay before zero-order-hold effects and dropped frames.

*Fix.* The delay is moved out of the feedback path into a *decision* channel; the 1 kHz core is O(1)
with a bounded worst case; a stale frame becomes a watchdog hold instead of a bad command.
**Evidence:** chatter 0.013 vs 0.123 (9.50x), RMSE 0.0125 vs 0.0933 rad.

### 4.4 Unsafe forced extrapolation under OOD

*Symptom.* On an unseen fault the black box keeps producing confident commands, saturates, and pushes the
axis outside its envelope.

*Cause.* A learned function is defined off its training support; "confidence" is another learned output
trained on the same data, not an evidence channel.

*Fix.* An independent physics-residual hypothesis test against a frozen nominal model, with an explicit
abstention state, sticky latches, hysteresis and a re-arm cooldown. **Evidence:** 1,733 vs 0 envelope
excursions in the four-stage synthetic benchmark of this repository; veto 2.0 ms after onset; 0.0295 rad
RMSE afterwards under a PD-only safe regulator.

### 4.5 Zero field debuggability

*Symptom.* A technician cannot tune what they cannot see; any change requires a new model version and a
revalidation.

*Cause.* The tuning surface is the learned parameters. There is no orthogonal knob, no incident record,
no way to know *why* the unit went safe.

*Fix.* Named gate states (every non-normal tick is a decision, never a silent clamp), a sticky 5-bit
fault audit mask, 4 orthogonal banks, 3 physical knobs, and 105 deterministic assertions.
**Evidence:** the C suite pins all of it - priority chain, dwell, cooldown, firewall, anti-windup, field
knobs - with zero random inputs.

For the control-theoretic version of this section (time-scale separation, delay-margin arithmetic,
inverse ill-posedness, support and abstention, verifiability) see
[`docs/systems_engineering_manifesto_en.md`](docs/systems_engineering_manifesto_en.md).

## 5. Cross-domain isomorphic mapping

The same three layers carry over unchanged; only the units, the physical constants and the meaning of
the regime label change. All three presets ship in
[`matlab_simulink/load_domain_profile.m`](matlab_simulink/load_domain_profile.m).

| Dimension | Industrial servo / WAAM axis | Edge VLA (10 Hz policy over a 1 kHz arm) | Closed-loop neuromodulation (aDBS / SCS) |
| :--- | :--- | :--- | :--- |
| State / command | joint angle [rad] / torque [N*m] | joint angle [rad] / torque [N*m] | normalised biomarker / normalised stimulation envelope |
| What the 10 Hz layer decides | thermal growth, wire-feed load, friction regime | compliance / contact regime for the joint | biomarker regime for the stimulation pattern |
| What the 1 kHz core owns | torque loop, envelope, actuator limits | torque loop, envelope, actuator limits | stimulation envelope inside strict clinical bounds |
| Drift to absorb without retraining | +35 % inertia, 2.4x break-away friction | +35 % link/payload inertia | +35 % electrode impedance, scar-tissue boundary |
| OOD event to abstain on | 42 Hz parasitic structural resonance + sensor spikes | 38 Hz structural mode + sensor spikes | 33 Hz lead resonance + stimulation artefacts |

Measured on all three presets with the same engine (synthetic dynamics presets, no hardware in the loop):

| preset | S2 RMSE B2 | S2 RMSE B3 | S3 RMSE B2 | S3 RMSE B3 | S4 envelope excursions B2 | S4 envelope excursions B3 | TV(u) B2 | TV(u) B3 |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `industrial` (rad) | 0.0516 | 0.0122 | 0.0933 | 0.0125 | 1733 | 0 | 1442.9 | 342.0 |
| `edge_vla` (rad) | 0.0353 | 0.0086 | 0.2887 | 0.0102 | 1637 | 0 | 22484.5 | 18848.6 |
| `neuromodulation` (norm.) | 0.0551 | 0.0269 | 0.0711 | 0.0238 | 182 | 0 | 11953.3 | 13824.0 |

The pattern repeats across all three: under +35 % drift the layered branch tracks 4.2x, 4.1x and 2.0x
better, under edge latency the gap widens, and only the end-to-end branch leaves the envelope. The
actuator-travel column is deliberately left in - it shows that the layer that stays inside the envelope
is *not* the one that works the actuator hardest (the neuromodulation preset is the case where the
end-to-end branch happens to be smoother; its cost is the 182 excursions).

Figure 3 above shows the three presets side by side. Per-domain thresholds, a porting checklist and the
most common misuse per domain are in
[`docs/domain_mapping_guide.md`](docs/domain_mapping_guide.md).

## 6. Run it (Windows PowerShell / cmd.exe, copy-paste ready)

**Prerequisites:** MATLAB with Simulink (the reference run used R2024b; no toolbox, no MEX compiler
required), and optionally any C99 compiler (`gcc`, `clang`, MSVC, `arm-none-eabi-gcc`) for the standalone
core test. Run both from the repository root.

**6.1 The full acceptance run** - builds the model, simulates the 40 s four-stage bench, runs the
efficiency study and the three domains, exports the three figures, and evaluates the four engineering
checks. Everything is plain MATLAB on the CPU, and the script prints its own `=== TIMING ===` block. On
the reference laptop (R2024b, idle) a cold `matlab -batch` takes about **27 s**, of which **~11.5 s is
the one-off Simulink model build / first load**; in an already-warm MATLAB session the same work takes
about **10 s**:

```powershell
matlab -batch "cd('matlab_simulink'); run_all_benchmarks"
```

Success signals: the four `[1]`-`[4] ... PASS` lines under `=== ENGINEERING CHECKS ===`, the final line
`All four engineering checks passed.`, three PNGs rewritten in `assets/` at 240 dpi (within the 200-300
dpi budget), and exit code `0` (the script raises if any check fails). The run is deterministic: the
four-stage benchmark uses `seed = 7`, the efficiency study `seed = 17`, and the C test uses no random
number generator at all, so the printed numbers reproduce digit for digit. Individual experiments
(efficiency only, one domain, model only) are listed in [`AGENTS.md`](AGENTS.md) section 4.4.

**6.2 The standalone C99 core** - 105 deterministic assertions, no MATLAB involved:

```powershell
gcc -std=c99 -pedantic -Wall -Wextra -O2 c99_layered_core/test_gatekeeper_standalone.c c99_layered_core/layered_gatekeeper.c -o rta_test.exe
.\rta_test.exe
```

Success signals: every check line starts with `[PASS]`, the last line is
`SUMMARY: n_pass=105, n_fail=0`, and the exit code is `0`. (`rta_test.exe` is gitignored, and the module
compiles warning-free under `-pedantic -Wall -Wextra`.)

**6.3 The Simulink model** - `matlab_simulink/ai_layered_control_bench.slx` comes from
`build_simulink_model.m` (built-in blocks only), and its parameters are inlined in the model, so it
needs no workspace variables: after `clearvars` you can double-click Run, or run
`sim('matlab_simulink/ai_layered_control_bench.slx')`. Treat it as a generated artefact and do not edit
it by hand - change the builder and regenerate.

That is the whole loop: one command proves the architecture's claims end to end, one command proves the
deployable core in isolation. If either fails, the repository is broken - that is the contract.

## 7. Using this repository from your AI assistant

The repository is written to be read by agents first: plain-text C99 and MATLAB, no hidden weights, no
datasets, a machine-readable ground-truth file, and two commands with exact pass strings. Paste this
prompt into your AI coding assistant with `@AGENTS.md` attached:

```text
Read @AGENTS.md first. It is the ground truth for this repository: the four architectural
invariants, the plain-text file map, the porting playbook, the acceptance commands and the
reference numbers.

Then help me port this architecture to my plant: <describe plant, units, control rate,
actuator limits, safety envelope, how state is measured>.

Rules for you:
1. Start with the mailbox contract (LayeredInput_t field -> my signal, with units) and show
   it to me before writing any control code.
2. Keep the 1 kHz core exactly as it is: no heap, no math.h, O(1), NaN/Inf firewall,
   priority chain, dwell, post-fault cooldown, per-controller anti-windup, bumpless transfer,
   magnitude and slew clamps, sticky fault bits.
3. The AI may only emit a regime (1..3, or 4 = abstain) and a confidence in [0,1], computed
   from dimensionless macro-features. It must never emit u(t) or u_ff.
4. Keep the physics-residual probe independent of the classifier, and able to force abstention.
5. Field adaptation is the three FieldTuning_t scalars only. Never retrain or fine-tune.
6. After each step, give me the signals to log and the pass/fail criterion. Do not claim
   success without pasting the evidence (log lines, test output, exit code).
```

Adapt the prompt rather than the architecture: the four invariants in `AGENTS.md` are what make the
drift, latency and OOD results hold, and any change that breaks one of them quietly returns you to
end-to-end control.

## 8. Deep dives

<details>
<summary><strong>Second-by-second walkthrough of the four stages</strong> (why each branch fails or holds)</summary>

**Setup.** `dt = 1 ms`, 40 s, three branches on one plant and one disturbance schedule (single seed, so
the comparison is paired). Industrial preset: `J = 0.010 kg*m^2`, viscous damping 0.060 N*m*s/rad,
compliance 0.300 N*m/rad, `u_max = 2.0 N*m`, envelope `|x| = 1.0 rad`, position noise 0.0025 rad
(1 sigma). The reference is a quintic point-to-point move (+/-0.5 rad, 1.5 s move, 1.5 s dwell) - the
dwells are where stiction actually bites.

**S1, 0-10 s - nominal distribution.** All three branches track. The end-to-end branch is the best
tracker (0.0087 rad) because it has memorised the exact inverse dynamics of *this* unit; the layered
branch is close behind (0.0098 rad) with a well-tuned PID plus white-box feedforward; the conservative
fixed PID is 7.5x the layered branch's error (0.0733 rad). Watch the commands instead: the end-to-end
branch already spends
660.3 N*m of total variation against 51.3 N*m for the layered branch, because it is continuously
inverting a noisy measurement. Nominal performance is a *local* property; the cost is not.

**S2, 10-20 s - the machine is swapped.** Inertia x1.35, Coulomb friction x2.0, break-away x2.4,
Stribeck velocity x0.25, ramped over 1 s. The end-to-end branch's frozen inverse is now wrong by about
0.63 N*m of break-away on an axis whose limit is 2.0 N*m, and it has no integral action to absorb the
error - by construction, a policy trained for pure inverse dynamics carries `Ki = 0`. Result: 0.0516 rad
versus 0.0122 rad. The layered branch's classifier sees the normalised feature vector drift, changes to
`REGIME_HIGH_LOAD_DAMPED` (higher gains, much higher damping, heavy integral), and the integrator absorbs
the rest. Nothing is retrained.

**S3, 20-30 s - the edge chip, or the network, is slow.** From here the end-to-end branch is throttled to
10 Hz with 60-120 ms of transport delay and 15 % dropped frames, while the layered branch keeps its inner
loop at 1 kHz and only receives labels at that rate. At the 3 Hz design crossover, 60/80/120 ms of delay
costs 65/86/130 deg of phase, plus about 54 deg from a 10 Hz zero-order hold, plus up to another 100 ms if
a frame is dropped - against a 60-70 deg margin. The loop chatters: 0.123 vs 0.013 (9.50x), RMSE 0.0933
vs 0.0125 rad. The layered branch is unaffected because the delay sits in the decision channel, and the
150 ms dwell timer stops label noise from reaching the command at all.

**S4, 30-40 s - an unseen fault arrives.** A 42 Hz parasitic mode (damping 0.03) excited by both command
and motion, plus 0.20 rad sensor outliers at roughly 4 per second (an 80-sigma excursion against the noise
floor). The end-to-end branch forces extrapolation: it saturates, then drives the axis past the 1.0 rad
envelope 1,733 times in 10 s - RMSE 0.6152 rad, chatter 0.986. The layered branch's fast residual channel
crosses its threshold 2.0 ms after onset, latches `GATE_VETO_OOD_RESIDUAL`, switches to the PD-only safe
regulator with a bumpless transfer, stays inside the envelope, and finishes the stage at 0.0295 rad. It
stays in fallback for a 500 ms cooldown after the fault subsides; the test suite pins the exact re-arm
tick.

**Total picture.** The monolithic branch wins 0.0087 to 0.0098 on the stage where its assumption holds,
and loses by 4.22x, 7.5x and 1,733 envelope excursions on the three stages where it does not. That trade
is the whole argument, and it is why the layered design is the default for machines that run unattended.

</details>

<details>
<summary><strong>The four mechanisms, in engineering terms</strong> (time scales, delay margin, inversion, abstention)</summary>

**Time-scale separation.** Inner 1 ms, outer 100 ms, plant drift that settles over about 1 s (content
well below 1 Hz): two decades of separation. The outer loop sees a quasi-steady inner loop; the inner loop sees a piecewise-constant
regime. That is what licenses "the AI may be slow, jittery and occasionally absent" as an architectural
statement rather than an aspiration, and it is why the coarse layer only reads slow variables (feature
time constants 0.25-0.50 s, i.e. 3-5 outer frames).

**Delay margin.** For a loop with crossover `f_c`, a transport delay `T_d` costs `360 * f_c * T_d` of
phase. At `f_c = 3 Hz` that is 108 deg per 100 ms - more than the entire margin. A 1 kHz inner loop with
sub-millisecond delay costs under 1 deg. The layered design therefore keeps *all* delay out of the
feedback path and pays for the AI's slowness in adaptation speed, on a time scale (seconds) where nothing
matters. A stale heartbeat becomes a frozen command (`GATE_HOLD_LATENCY`) that leaves no trace in any
integrator, blend or dwell counter.

**Inversion is ill-posed on non-smooth boundaries.** `u = f^-1(..)` when `f` contains a jump (stiction,
backlash, contact, dead-band) has unbounded gain at the jump; a finite-capacity learner must smooth it,
and a smooth compensator in series with a relay-like nonlinearity produces a limit cycle rather than a
fixed point. The robust cures are classical: integral action to absorb the unknown bias, a bank of
controllers to change loop gain around the boundary, and hard limits to contain the limit cycle. The
architecture schedules exactly those three with a slow, low-dimensional decision.

**Abstention is a hypothesis test, not a mood.** A residual against a *frozen* nominal model has a null
distribution you can characterise from commissioning noise, so the threshold has an arguable false-alarm
rate and can be verified before deployment. Softmax confidence has none of those properties: it is a
function of the same policy, trained on the same data, and it is usually high exactly where it is wrong.
The architecture uses the residual as the primary OOD channel and keeps confidence as a secondary, sticky
veto. Hysteresis (trip 1.0 / clear 0.80 for the residual; trip below 0.45 / clear at 0.55 for confidence)
plus a 500 ms re-arm cooldown give the switching law dwell-time behaviour instead of chatter.

The full argument - including what would falsify it, and when an end-to-end policy is the better
engineering choice - is in
[`docs/systems_engineering_manifesto_en.md`](docs/systems_engineering_manifesto_en.md).

</details>

## 9. Deterministic core: zero heap, no `malloc`, O(1) worst-case execution time

Everything that can move the actuator lives in
[`c99_layered_core/layered_gatekeeper.c`](c99_layered_core/layered_gatekeeper.c) behind the contract in
[`c99_layered_core/layered_gatekeeper.h`](c99_layered_core/layered_gatekeeper.h), whose comments specify
the full per-tick semantics.

* **Zero dependencies.** The only include is `<stdint.h>`. No `math.h`, no libm, no `printf`, no OS calls.
  `|x|`, clamping and the finite check are hand-written (a NaN fails both comparisons, an infinity fails
  one), so no library call can be optimised in or out.
* **Zero heap, zero recursion, no hidden state.** No allocation, no recursion, no file-static mutable
  data. All persistence is the caller-owned `LayeredState_t` - **84 B** on the reference toolchain, with
  `LayeredConfig_t` 116 B, `FieldTuning_t` 12 B, `LayeredInput_t` 36 B and `LayeredOutput_t` 28 B
  (measured with `sizeof` on gcc 4.9.2, `-std=c99 -pedantic -Wall -Wextra`). Several axes can run side by
  side; nothing is shared.
* **O(1) WCET.** One step is a fixed straight-line sequence of branches and arithmetic: no
  data-dependent loops, no dispatch, no memory walks. Worst case equals average case. It compiles clean
  with `-pedantic -Wall -Wextra` and with `-Wdeclaration-after-statement`, so it drops into ISR code that
  has to stay boring.
* **Fails safe on bad input.** A NaN/Inf/out-of-range mailbox frame is rejected *before* any state is
  touched: the actuator holds, `corrupted_frame_count` increments, a sticky bit latches, and no
  integrator is poisoned.
* **Bounded output, always.** Magnitude clamp then slew clamp, every tick; every bank controller's
  integrator is back-calculated toward the *actually applied* command, so an inactive bank cannot hide
  windup and a switch releases no energy; bumpless transfer bounds the step at
  `blend_alpha * u_max + slew_rate * dt`.
* **Pinned by tests, not by inspection.**
  [`test_gatekeeper_standalone.c`](c99_layered_core/test_gatekeeper_standalone.c) composes 105
  assertions over a 40 s mission schedule: priority chain, dwell, cooldown, NaN/Inf firewall,
  anti-windup across a saturated bank switch, the three field knobs, feedforward handling, sticky
  confidence latch, global actuator invariants, and the API / self-healing / null-pointer paths. It is
  deterministic (no `rand()`, no `<math.h>`; a truncated-Taylor rotation matrix generates the reference
  sine), so it prints `SUMMARY: n_pass=105, n_fail=0` everywhere.

**Certification posture.** The core borrows defensive embedded-programming principles - zero dynamic
allocation, bounded execution steps, input sanitisation, fault-bit logging - as a structure reference for
later engineering work; the code in this repository is not certified functional-safety or medical-device
firmware. The same principles are why the deterministic part is easy to test exhaustively: one
straight-line C function, fixed state, a range-checked interface to the learned layer, and a runtime audit
trail. That is a starting point for a safety case, not a substitute for a hazard analysis, hardware-in-the-
loop testing, or the certification work of the target industry.

## 10. Fault audit bit mask

A sticky 5-bit mask in `LayeredState_t` records *why* the unit left normal operation. It is OR-ed with
every new event and never cleared by the core, so it survives the incident and the platform can push it
to non-volatile storage.

| Bit | Macro | Latched when | Meaning in the field |
| :--- | :--- | :--- | :--- |
| 0 | `LAYERED_FAULT_BIT_LATENCY_TIMEOUT` | slow-loop heartbeat older than `outer_timeout_s` (0.30 s) | the AI process stalled, was preempted, or a frame was lost in transport |
| 1 | `LAYERED_FAULT_BIT_OOD_RESIDUAL` | `\|y - y_hat\|` above threshold (sticky, clears with hysteresis) | the plant left the commissioned model - fault, wear, or a new operating mode |
| 2 | `LAYERED_FAULT_BIT_LOW_CONFIDENCE` | confidence below 0.45, or the AI self-abstains (`regime == 4`) | the classifier does not vouch for this frame |
| 3 | `LAYERED_FAULT_BIT_ENVELOPE_BREACH` | state beyond 90 % of the hard limit, or the 60 ms prediction does | the axis reached the edge of what the firmware allows |
| 4 | `LAYERED_FAULT_BIT_NAN_CORRUPTION` | any NaN/Inf/out-of-range mailbox field | upstream software or IPC corruption - investigate the producer |

Per-cause counters (`safety_veto_count`, `ood_veto_count`, `low_confidence_veto_count`,
`latency_hold_count`, `corrupted_frame_count`, `step_count`) turn the bits into a timeline: how many
episodes, how often, in which order. That is the black box a field engineer needs - and it needs nothing
from the AI.

## 11. Field tuning: three knobs, no retraining

Moving the same firmware to a new machine, a new patient or a new payload is a calibration, not a
machine-learning project:

```c
typedef struct {
    float gain_scale;          /* scales all P/I gains;    1.0 = factory nominal */
    float damping_scale;       /* scales all D gains;      1.0 = factory nominal */
    float safety_margin_scale; /* scales the safety limit; 1.0 = factory nominal */
} FieldTuning_t;               /* 12 B: the entire field-adaptation surface */
```

The AI stays frozen. The deterministic bank absorbs the uncalibrated drift; the integral action plus the
drift tolerance shown in S2 is what buys the technician time to do the three-knob procedure properly. The
C suite's stage 5 proves all three knobs actually move the command (gain up => steady-state `|u|` up;
damping up => command moves along `+e_dot`; margin down => the envelope trips earlier), and stage 6
proves the drift is absorbed without touching the classifier. Anything that cannot be fixed with these
three numbers is a commissioning problem, and it should be fixed at commissioning - not papered over with
a retrained model that nobody can re-verify.

## 12. Actuator wear: total variation TV(u)

TV(u) is the sum of `|u(k) - u(k-1)|` over the run - a proxy for how hard the actuator, gearbox and
mounts are being worked. Over the 40 s benchmark:

| Branch | TV(u), 40 s [N*m] | Relative to layered |
| :--- | ---: | ---: |
| Branch 1: fixed PID | 1664.8 | 4.87x |
| Branch 2: end-to-end AI | 1442.9 | 4.22x |
| Branch 3: layered | **342.0** | 1.00x |

Per stage, the layered branch stays between 51.3 and 148.3 N*m while the alternatives run 409-426 N*m
(fixed PID) and 17.4-660.3 N*m (end-to-end). Two cautions before quoting these as "smoothness": (1) the
units are domain-specific - compare branches within one benchmark, not across domains; (2) low TV is not
automatically good: in S3 the end-to-end branch has the *lowest* TV (17.4 N*m) because its command is
frozen between 10 Hz frames, which is a stale actuator, not a well-behaved one. Read TV together with
tracking error and envelope excursions, never alone.

## 13. Repository contents and further reading

The repository is plain text - C99, MATLAB, Markdown - plus one generated Simulink model and three
generated figures. Nothing learned is shipped: the coarse classifier is a 16-float prototype table plus
four feature scales.

```text
ai-layered-control/
├── README.md / README_zh.md            this file / 中文版
├── AGENTS.md                           machine-readable ground truth: invariants, playbook, commands
├── docs/
│   ├── systems_engineering_manifesto_en.md   the systems-engineering argument (EN)
│   ├── systems_engineering_manifesto_zh.md   same, 中文
│   └── domain_mapping_guide.md               cross-domain mapping + porting checklist
├── c99_layered_core/                   deployable core: C99, zero dependency, no heap
│   ├── layered_gatekeeper.h            contract (read this first)
│   ├── layered_gatekeeper.c            the 1 kHz implementation
│   └── test_gatekeeper_standalone.c    105 deterministic assertions
├── matlab_simulink/                    benchmark, presets, model builder
│   ├── load_domain_profile.m           the three domain presets, in physical units
│   ├── sim_core_engine.m               4-stage, 3-branch engine (+ MATLAB mirror of the C core)
│   ├── run_efficiency_analysis.m       sample complexity, drift sweep, compute/memory
│   ├── build_simulink_model.m          generates the .slx from built-in blocks
│   ├── run_all_benchmarks.m            one-click acceptance run
│   └── ai_layered_control_bench.slx    generated artefact (do not treat as source)
└── assets/                             three generated 240 dpi figures
```

| Document | Answers |
| :--- | :--- |
| [`docs/systems_engineering_manifesto_en.md`](docs/systems_engineering_manifesto_en.md) | why coarse classification + deterministic control beats single-point AI optimisation, when it does not, and what would falsify the claim |
| [`AGENTS.md`](AGENTS.md) | the four non-negotiable invariants, the plain-text ground-truth file map, a four-step porting playbook, the acceptance commands |
| [`docs/domain_mapping_guide.md`](docs/domain_mapping_guide.md) | the same mapping applied to industrial servo/WAAM, edge VLA and neuromodulation, with per-domain thresholds and misuse patterns |
| [`c99_layered_core/layered_gatekeeper.h`](c99_layered_core/layered_gatekeeper.h) | the contract: 4 regimes, 5 gate states, 5 fault bits, the full per-tick semantics |

### Sister project: jev-tune

[`jev-tune`](https://github.com/lijiawei255/jev-tune) works one level lower, inside a single controller:
continuous parameter-layer tuning with a C99 safety boundary as the backstop. This repository works one
level higher, at the discrete structural layer - coarse semantic classification across operating
conditions, convergence from very few calibration runs, compute decoupling between the slow and fast
loops, and explicit rejection of operating points it does not recognise. The two are complementary:
parameter-layer fitting and structural-layer arbitration, together forming one engineering-systems path
from "tune the controller" to "decide which controller, and refuse the ones you cannot vouch for".

## License

MIT. See [LICENSE](LICENSE).

## Scope and disclaimer

This repository is a **reference implementation and conceptual benchmark** for academic exchange and
architectural exploration. Every experiment in it runs on a **parameterised synthetic nonlinear plant**;
it contains no clinical data from real patients and no production data from real machines, and nothing
in it has been validated on hardware.

Before any of this is used on a real industrial machine, robot or neuromodulation system, the full
engineering process for that application has to be carried out: hazard analysis (FMEA / STPA, and the
equivalent clinical risk analysis for medical use), hardware-in-the-loop testing, and the certification
required by the relevant industrial-safety or medical-device regulations. The numbers in this README
describe a simulation; they are not a safety claim about any device.
