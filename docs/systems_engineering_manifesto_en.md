# The Layered Control Manifesto

### A systems-engineering argument for coarse classification + deterministic control over end-to-end continuous policies in closed-loop physical systems

[English](systems_engineering_manifesto_en.md) | [简体中文](systems_engineering_manifesto_zh.md)

**Scope and honesty statement (read this before the numbers).** Every quantitative claim below comes
from the *synthetic* bench in this repository: a second-order plant with Stribeck friction, simulated at
`dt = 1 ms` for 40 s under a fixed disturbance schedule. No number in this document was measured on real
hardware, no real dataset was used, and the "end-to-end AI" branch is a *stand-in* for a learned
continuous policy (a frozen computed-torque law plus learned feedback gains), not a trained network. The
compute and memory figures are analytic (weight counts x inference rate), not profiled on a device. What
the bench does provide is a *controlled comparison*: same plant, same reference, same noise realisation,
same fault schedule, three architectures. That is enough to make architecture-level statements, and not
enough to make device-level ones. Section 8 lists what would falsify the argument.

---

## 1. The claim, in one page

The dominant research programme for AI in closed-loop control is *single-point optimisation of the
policy*: train one large model to approximate the continuous map `u(t) = pi_theta(x(t))`, run it at or
near the loop rate, and keep the result inside the physical envelope with soft penalty terms.

This document argues that for physical control loops this is the wrong place to put the intelligence,
and that the failure is structural rather than a matter of insufficient data or model capacity:

> **A continuous policy has to be correct everywhere the loop can go. A discrete decision only has to be
> correct at the few places the system actually operates, and the deterministic controller that consumes
> the decision only has to be correct in the sense a control engineer can prove.**

The layered architecture in this repository implements that restatement literally:

| Layer | Rate | What it does | What it is *not* allowed to do |
| :--- | ---: | :--- | :--- |
| Coarse classifier ("the AI") | 10 Hz | Emits one label in `{1, 2, 3, 4}` plus one confidence in `[0, 1]`, from four normalised, dimensionless macro-features | Emit `u(t)`, emit a feedforward term, see raw sensor streams |
| Physics-residual probe | 1 kHz | Computes `r(t) = \|y(t) - y_hat(t)\|` against a **frozen nominal model** and can force an explicit abstention | Adapt the model online |
| Deterministic core | 1 kHz | Gatekeeper (firewall, priority chain, dwell, cooldown) + controller bank + bumpless transfer + hard clamps; owns the actuator | Allocate memory, call `math.h`, recurse, dispatch dynamically |

The three claims this buys, each with the benchmark evidence that supports it:

1. **Sample efficiency and drift tolerance are structural, not statistical.** 40-ish calibration runs
   saturate the classifier (79.75 % of its 85.13 % ceiling at `N <= 50`), while the continuous regressor
   still needs 1,000-2,000 samples (91.88 % / 97.63 %). Under +35 % uncalibrated drift with the AI
   **frozen**, the layered branch holds RMSE at 0.0122 rad against 0.0516 rad for the end-to-end branch
   (4.22x), and across the whole 0 -> +50 % drift sweep (4 points: 0 / 17 / 33 / 50 %) it moves
   0.0111 -> 0.0151 (1.36x) against 0.0101 -> 0.0783 (7.75x).
2. **Latency decoupling is structural.** At 10 Hz with 60-120 ms jitter and 15 % dropped frames, the
   end-to-end branch burns its phase margin (chatter index 0.123 vs 0.013, 9.50x; RMSE 0.0933 vs
   0.0125), while the 1 kHz inner loop never sees the delay at all.
3. **Abstention is possible, and only because the model is white-box.** On an unseen 42 Hz structural
   resonance with sensor outliers, the end-to-end branch breaches the 1.0 rad hard envelope 1,733 times
   (peak error 0.6152 rad RMSE); the layered branch vetoes 2.0 ms after onset, never breaches the
   envelope, and continues at 0.0295 rad RMSE under a PD-only safe regulator.

Two further consequences are engineering, not performance: actuator travel over the whole 40 s run is
342.0 N*m for the layered branch against 1,664.8 N*m (fixed PID) and 1,442.9 N*m (end-to-end), and the
deployment footprint drops from an analytic 2.40e9 MAC/s and 5.60e6 B to 2.41e4 MAC/s and 2.21e3 B
(99,502x and 2,536x), which is the difference between a GPU-class edge box inside the 1 ms budget and a
microcontroller that was already on the board.

This is not an argument that large models are useless in control. It is an argument about *where* in a
control architecture a learned component can be placed so that its failure modes are bounded, its
behaviour is auditable, and its upkeep does not scale with the number of machines you ship. Section 7
states the cases where the monolithic design is the better engineering choice.

---

## 2. Five failure modes of end-to-end continuous control

Each failure mode below is stated as a mechanism, not as an opinion, and matched to the bench evidence
that isolates it.

### 2.1 Boundary ill-conditioning: the regression target is discontinuous where it matters most

A physical plant is not smooth everywhere that a control engineer cares about. The bench plant is

```
J * xdd + b * xd + k * x + F_f(xd) + d_ood(t) = u
F_f(v) = fvisc * v + fc * tanh(v / veps) + (fs - fc) * exp(-|v| / vs) * tanh(v / veps)
veps = max(0.02 * vs, 1e-9)
```

With the industrial preset (`fs = 0.45 N*m`, `fc = 0.20 N*m`, `vs = 0.02 rad/s`), the break-away
transition happens across roughly 0.4 mrad/s of velocity - four orders of magnitude below the reference
velocity scale - which is a stand-in for stiction: on a real axis it is a genuine jump.

Now consider what a continuous policy has to learn. On the nominal unit it must reproduce the inverse
`u = J*r_dd + b*r_d + k*r + F_f(r_d)`. The inverse of a jump is unbounded: an infinitesimal change of
velocity across the boundary requires a finite force change of `fs - fc`. Any learner that is trained
with a smooth loss and has finite capacity must *smooth* that jump. On the unit it was trained on, the
smoothed inverse is good enough: in Stage 1 the end-to-end branch is the best tracker of the three
(RMSE 0.0087 rad vs 0.0098 rad for the layered branch, 0.0733 rad for the fixed PID).

The cost is not visible in Stage 1. It becomes visible the moment the boundary moves. In Stage 2 the
bench applies `fs x 2.40`, `fc x 2.00`, `vs x 0.25` and `J x 1.35`. The learned inverse now under-estimates
break-away by roughly 0.63 N*m on an axis whose actuator limit is 2.0 N*m - a 31 % command authority
error, applied every time the axis tries to reverse - and, because the policy carries no integral action
(`Ki = 0` in the end-to-end branch, exactly as a policy trained for pure inverse dynamics would), nothing
absorbs the bias. The result is not a graceful degradation but hunting: RMSE 0.0516 rad vs 0.0122 rad,
a 4.22x ratio, with limit-cycle content visible in both the state and the command trace.

The deeper point: **the manifold a continuous policy is trained on is a smoothification of the real
plant, and the smoothification is exactly what fails at the boundary.** This is not fixed by more data,
because the boundary is not a data-density problem; it is a property of the target function.

### 2.2 Data hunger and unit-to-unit drift

The second failure mode is an accounting one. A continuous policy is trained to be correct on a
*distribution of trajectories*; a classifier is trained to be correct on a *partition of a low-dimensional
feature space*. Those two requirements have wildly different sample complexity, and the bench measures
the difference.

Both learners in Panel A of the efficiency study see the same normalised macro-features and are trained
from the same calibration pool. The coarse learner is a nearest-class-mean classifier over 3 classes
(the only thing the 10 Hz AI is asked to learn: 3 x 4 = 12 class-mean coordinates). The continuous
learner is a ridge regression on a degree-8 monomial basis, 495 terms, standing in for a dense policy
head that would in reality carry `1e4` to `1e6` weights - so the comparison flatters the continuous
learner, if anything.

| Calibration runs `N` | 15 | 40 | 120 | 250 | 500 | 1000 | 2000 |
| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Coarse 3-class classifier | 72.56 % | **79.75 %** | 84.38 % | 84.13 % | 83.44 % | 85.13 % | 84.31 % |
| Continuous policy (damping within 8 %) | 12.75 % | 26.06 % | 39.31 % | 78.75 % | 78.75 % | 91.88 % | 97.63 % |

Two things are worth noticing. First, the classifier is already at 93.7 % of its own ceiling with 40
samples, and its ceiling (85.13 %, reached at `N = 1000`) is set by the *quantisation* of the damping
range into three banks, not by data: it is non-monotonic after `N = 120`
(84.38 -> 84.13 -> 83.44 -> 85.13 -> 84.31) precisely because additional data cannot buy back
quantisation error. Second, the continuous learner needs more than a thousand samples to reach 91.88 %,
and even at 2,000 it is at 97.63 % - and that is on a *synthetic* operating-point model with a smooth,
well-conditioned target.

Now the deployment question. The parameter that changes between units is not a label, it is a physical
constant (`+35 %` inertia, `2.4x` break-away friction, sharper Stribeck velocity). A classifier that
operates on normalised features sees the same *shape* of feature vector on every unit, so it transfers;
a continuous policy encodes the constants themselves, so it does not. Panel B measures this directly:

| Uncalibrated drift | 0 % | ... | +50 % |
| :--- | ---: | :---: | ---: |
| End-to-end branch RMSE | 0.0101 | (rises monotonically) | 0.0783 |
| Layered branch RMSE | 0.0111 | (flat) | 0.0151 |

A 7.75x degradation against 1.36x, with **no retraining** and no dataset upload in either case. The three
floats of `FieldTuning_t` (`gain_scale`, `damping_scale`, `safety_margin_scale`) are the entire field
adaptation procedure; the trained part of the system never changes.

### 2.3 Compute, power, and the latency budget inside the loop

A 1 kHz loop gives you 1 ms of budget, every millisecond, for the whole path from sample to actuator:
sensor read, filtering, policy inference, safety logic, bus write. A 1.2 M-parameter dense policy is
`2.4e9` MAC/s if it runs at 1 kHz, and `5.6 MB` of float32 weights working set - before activations.
The layered architecture is `2.41e4` MAC/s (3 prototypes x 4 features at 10 Hz, plus 24 MAC/step at
1 kHz) and `2.21e3` B (64 B of learned parameters + 96 B of state + 2 kB of code). The ratio is 99,502x
in compute and 2,536x in memory.

Those numbers matter twice. Once as cost and power: for an implantable or a battery-powered edge device,
2.4 GMAC/s is simply not a candidate, while 24 kMAC/s is below the noise floor of the rest of the
system. And once as *jitter*: a large model's inference time is data-dependent and cache-sensitive, so
the loop's effective delay is a random variable. Real-time control does not care about mean latency; it
cares about the worst case against the deadline. The 1 kHz core here is O(1) with no heap, no `math.h`,
no recursion and no loops over memory, so its worst-case execution time is bounded by a fixed
straight-line path - which is a *verifiable* statement, not a benchmark average.

Stage 3 makes the consequence concrete. The 10 Hz outer loop is given 60-120 ms of transport latency and
a 15 % frame-drop probability, and the end-to-end branch is throttled to that 10 Hz channel by
construction. Two separate effects hit it:

* **Phase.** A transport delay `T_d` contributes `360 deg * f * T_d` of phase loss at frequency `f`. At
  the industrial preset's designed crossover (`f_c = 3 Hz`), 60 ms costs 65 deg, 80 ms costs 86 deg and
  120 ms costs 130 deg. A zero-order hold at 10 Hz adds on average another half-frame, 50 ms, i.e. 54 deg
  at `f_c`. A loop designed with 60-70 deg of phase margin is under water as soon as the jitter is
  considered, before any frame is dropped at all. Dropping one frame doubles the inter-sample interval
  (200 ms), which is 216 deg at `f_c`.
* **Discretisation.** A 10 Hz policy can only react to what it saw up to 100 ms ago; commands are held
  between frames. The loop's effective closed-loop bandwidth is capped well below the plant's, and the
  error signal that reaches the policy is already aliased.

The measured result is a 9.50x chatter index and a 7.5x RMSE penalty (0.0933 vs 0.0125 rad). Note what
the layered branch does *not* need: it does not need the classifier to be fast or even reliably present.
The 10 Hz layer's output is a *label*; the gatekeeper's watchdog converts a late or missing label into
`GATE_HOLD_LATENCY` with a frozen actuator, and the inner loop's phase margin is untouched because the
delay lives in a decision channel, not in the feedback path.

### 2.4 Forced extrapolation: a black box has no "I do not know" output

A function approximator is defined off its training support. Feed it a state it has never seen and it
returns a number with the same confidence and the same magnitude as any other number. There is no
internal signal that means "undefined". A softmax confidence is not that signal: it is a function of the
same weights, trained on the same data, and it is routinely high on off-support inputs.

Stage 4 injects an unmodelled 42 Hz parasitic structural mode (`zeta = 0.03`, excited by both the command
and the motion) plus random sensor outliers of 0.20 rad at 4 per second - an 80-sigma excursion against a
0.0025 rad noise floor - on a 1.0 rad hard envelope. The end-to-end branch keeps producing commands,
saturates, and drives the axis past the envelope **1,733** times in 10 s; its RMSE is 0.6152 rad and its
chatter index 0.986, i.e. it has stopped controlling and is oscillating.

The layered branch does something categorically different: it *abstains*. The nominal observer (a frozen
model of the plant, 4 Hz bandwidth, deliberately slower than the inner loop) produces a two-channel
residual - a slow channel `|LP(innovation)|` for drift and a fast channel `|innovation - LP(innovation)|`
for transients. The fast channel crosses the veto threshold within 2.0 ms of fault onset; the priority
chain latches `GATE_VETO_OOD_RESIDUAL`, the core switches to `REGIME_SAFE_FALLBACK` (PD-only, heavy
damping), and the envelope is never breached (0 violations, RMSE 0.0295 rad).

The epistemics matter more than the numbers. A residual against a white-box nominal model is a
*hypothesis test*. Under the null hypothesis ("the nominal model, plus known noise, explains this
measurement") the residual's distribution is known well enough from commissioning noise statistics to
place a threshold with a defensible false-alarm rate. That is a design calculation you can do before
deployment and verify with a test. "The policy seems unsure" is not a hypothesis test; it is another
learned output, and it is unverifiable by construction.

### 2.5 No white-box debugging, and no change control

A field engineer facing a misbehaving black-box controller has one lever: retrain or fine-tune, with all
the risks that entails (data collection, catastrophic forgetting, revalidation, and - in a regulated
context - a new trained-model version). A field engineer facing the layered architecture has:

* five named gate states (`NORMAL`, `HOLD_LATENCY`, `VETO_OOD_RESIDUAL`, `VETO_LOW_CONFIDENCE`,
  `VETO_SAFETY_ENVELOPE`) - every non-normal tick is a decision with a name, never a silent clamp;
* a sticky 5-bit fault-audit mask that survives the incident and tells you *why* the unit went safe
  without asking the AI anything;
* four orthogonal controller banks selected by a label, plus three scalar field knobs that are physical
  quantities (gain, damping, margin), so a technician can reason about them;
* 105 deterministic assertions that pin the behaviour of everything that touches the actuator.

That last item is the one that changes the engineering economics. Verification effort scales with the
number of behaviours you must argue about. In the layered design, the safety-relevant behaviour lives in
a straight-line C function with a fixed 84 B state struct, so it can be unit-tested exhaustively at the
boundaries (stage 3-4 of the test suite drive every veto, every hysteresis edge, the dwell timer, the
cooldown, the NaN firewall and all three field knobs). The learned behaviour lives in *16 floats of
prototype table* over a 4-dimensional feature space, so it can be characterised by a confusion matrix
over the operating envelope rather than by a loss curve. The interface between them is a 9-field struct
with range contracts, and any frame that violates a contract is rejected before it can touch state.

Section 6.2 of this document spells out the relationship to IEC 61508 / IEC 62304 honestly: the
architecture is *shaped to be compatible with* those lifecycles (separable safety function, deterministic
core, explicit interfaces, audit trail), and this repository does not claim a certified component.

---

## 3. The cybernetic argument

The five failure modes above share one root: they all come from putting a *high-bandwidth, unbounded-gain,
unverifiable* element inside the feedback path. The layered architecture is not a better learner; it is a
different allocation of responsibility. The classical arguments that justify it are old and still correct.

### 3.1 Time-scale separation (singular perturbation)

Write the closed loop with two rates: `eps = T_inner / T_outer = 1e-3 / 1e-1 = 0.01`. In singular
perturbation form the fast subsystem reaches a quasi-steady state on the slow time scale, so the outer
loop sees an *averaged* plant and the inner loop sees a *constant* setpoint. The standard requirement is
that the outer bandwidth be well below the inner one; here it is two decades below, and the decision
layer's own content is slower still (the Stage-2 drift ramps over 1 s, so its content sits well below
1 Hz).

That separation is what makes "the AI is allowed to be slow, jittery and occasionally absent" a
statement about the *architecture* rather than a hope. It also tells you what the coarse layer is allowed
to depend on: only the slow manifold. The four shipped features are exactly that - normalised RMS
tracking error, normalised RMS control effort, the slow residual channel, and normalised RMS velocity -
all computed with time constants of 0.25-0.50 s, i.e. three to five outer frames. They are dimensionless
because dimensionless features are the ones that survive a unit change.

### 3.2 Bandwidth and delay margin

For a loop with crossover `f_c` and loop delay `T_d`, the phase margin consumed by delay is
`phi_delay = 360 * f_c * T_d`. This is the single most useful formula in the whole argument, and it is
why the two designs fail differently:

| Architecture | Delay inside the feedback path | Phase lost at `f_c = 3 Hz` |
| :--- | :--- | :--- |
| 1 kHz monolithic policy | about 0.5-2 ms (ZOH + compute) | 0.5-2 deg |
| 10 Hz monolithic policy | 100 ms ZOH + 60-120 ms transport | 170-240 deg |
| Layered, inner loop | about 0.5-2 ms | 0.5-2 deg |
| Layered, outer loop | 100 ms ZOH + 60-120 ms transport | 170-240 deg, **outside the feedback path** |

The last row is the design. The same delay that destroys the monolithic system is harmless in the layered
system because the outer layer produces a *decision*, and the decision is consumed by a supervisory
state machine with hysteresis (150 ms minimum dwell, 500 ms post-fault cooldown) whose job is precisely
to not react to every frame. Delay in a decision channel costs adaptation speed on a time scale where
adaptation speed does not matter - the plant drift takes seconds, the classification takes 100 ms.

The corollary is a design rule: **the inner loop must never wait on the AI.** In the C core, a stale
heartbeat freezes the actuator (`GATE_HOLD_LATENCY`) and touches nothing else - no integrator, no dwell
counter, no blend - so a 200 ms AI hiccup leaves no trace in the controller state. That property is
tested directly (stage 3 of the C regression suite).

### 3.3 Inversion is ill-posed across non-smooth boundaries

Model-based control wants `u = f^{-1}(...)`. When `f` contains a jump - stiction, backlash, contact
transition, saturation, a valve dead-band - the inverse is unbounded at the jump and the gain of the
inverse around it is set by how fast the transition happens, not by the physics you can measure. The
bench's Stribeck layer has slope `(fs - fc) / vs = 12.5 N*m/(rad/s)` in absolute terms, sharpened by the
`tanh(v / 0.02 vs)` term to an effective transition width of 0.4 mrad/s.

Three consequences, all of them visible in the data:

1. A learner trained on the nominal plant smooths the jump, and the smoothing is wrong as soon as the
   jump moves (Stage 2: `fs x 2.4`).
2. A smooth compensator in series with a relay-like nonlinearity produces a limit cycle, not a fixed
   point; the amplitude is set by the intersection of the compensator's frequency response with the
   nonlinearity's describing function. That is exactly what appears as "hunting" at reversal.
3. The only robust cures are the classical ones: an integrator with anti-windup (to absorb the unknown
   bias), a bank of controllers that changes the loop gain around the boundary, and hard limits that
   prevent the limit cycle from leaving the envelope. The layered architecture is a systematic way to
   schedule those three cures with a slow, low-dimensional decision.

This is also why the architecture insists that the feedforward `u_ff` - the plant-inverse term - is
produced by the *commissioning model*, white-box, and never by the AI. The AI is not allowed to learn the
inverse; it is only allowed to say which of four known-good inverses to use.

### 3.4 Out-of-distribution: support, abstention, and dwell-time stability

A learned policy is a function on the whole state space but is only meaningful on the support of its
training distribution. Off-support it extrapolates arbitrarily - not "smoothly wrong" but *arbitrarily*
wrong, with the same numerical confidence. This is not a statistical tail problem you can buy your way
out of; it is a statement about what a function is.

The layered architecture replaces "hope the input is in-distribution" with an explicit supervision
decision:

* **Detection** is done by the residual probe, which is an independent channel: it does not share the
  classifier's features, its model, or its failure modes. Detection latency in the bench is 2.0 ms,
  because the probe runs at 1 kHz on the innovation of a nominal observer.
* **Abstention** is a first-class state with a priority above every other decision: the C core's
  priority chain is safety envelope > OOD residual > low confidence > stale heartbeat > normal. A veto
  is *sticky*, with hysteresis ratios (residual trip 1.0 / clear 0.80; confidence trip below 0.45 /
  clear at 0.55), so a single noisy sample cannot toggle the system.
* **Re-arm** is gated by a cooldown: after a veto clears, the AI is not trusted again until 500 ms of
  *consecutive clean* samples have elapsed. This is the discrete-time analogue of dwell-time switching
  in supervisory control, and it is what prevents the common failure of an OOD detector that chatters
  between "safe" and "AI" modes at the fault boundary. In the C test, stage 6 pins the exact tick at
  which the regime is restored.

One more detail that matters in practice: the classifier's confidence is defined as a *margin* in
distance space, `d2 / (d1 + d2)` where `d1` and `d2` are the distances to the nearest and runner-up
prototype. It is not a softmax over logits; it collapses to 0 exactly when two classes are equally close,
which is the ambiguous situation you want a veto on. The architecture does not need the confidence to be
a calibrated probability - only to be monotone in ambiguity - and that is a much weaker requirement.

### 3.5 Verifiability, not just performance

The layered design changes what you can *prove*. Three concrete properties:

* The whole actuator path is deterministic and allocation-free. `Layered_Step` is a straight-line
  sequence of branches over an 84 B state struct; the block comment in the header enumerates its
  semantics in order (firewall, safety, residual, confidence, heartbeat, dwell, cooldown, bank,
  anti-windup, bumpless transfer, clamps). That is a specification you can test assertion by assertion,
  and the shipped suite does: 105 checks, 0 failures, deterministic (no `rand()`, no `math.h`, no clock).
* The learned part is *tiny and inspectable*. The shipped classifier is a prototype table of 12 numbers
  plus 4 feature scales. Its behaviour over the operating envelope can be characterised exhaustively;
  there is no hidden layer to sample.
* The interface is a contract. Nine input fields, seven output fields, with range contracts enforced by
  the input firewall before any persistent state is touched. A corrupted IPC frame (NaN, Inf, out-of-range
  enum) cannot poison an integrator - it is rejected, the actuator holds, and a sticky bit records it.

Contrast that with the monolithic design, where the verification object is "the behaviour of 1.2 M
parameters over the reachable state space". There is no way to enumerate that, so the only available
evidence is statistical and distribution-dependent - which is fine for a perception benchmark and
insufficient for a safety function.

---

## 4. The experiment: four assertions, and what they do not prove

### 4.1 Setup

Three branches, one plant, one reference, one disturbance schedule (same RNG seed for noise, spikes,
latency and drops - so the comparison is paired, not averaged):

| Branch | What it is | Rate |
| :--- | :--- | :--- |
| B1 fixed PID | conservative 1 kHz PID, no adaptation, no fault detection | 1 kHz |
| B2 end-to-end AI | computed-torque policy with gains learned/frozen on the **nominal** unit, `Ki = 0`; throttled to the 10 Hz channel in Stages 3-4 | 1 kHz -> 10 Hz |
| B3 layered | 10 Hz coarse classifier + residual probe + 1 kHz C99 core (mirrored in MATLAB line for line) | 10 Hz + 1 kHz |

Stages: S1 0-10 s nominal; S2 10-20 s cross-unit `+35 %` drift and a sharper friction boundary;
S3 20-30 s edge compute scarcity (10 Hz, 60-120 ms jitter, 15 % drops); S4 30-40 s unseen 42 Hz
resonance plus sensor outliers. The industrial preset is a servo/WAAM-quality axis: `J = 0.010 kg*m^2`,
`u_max = 2.0 N*m`, hard envelope `|x| = 1.0 rad`, position noise `0.0025 rad`.

### 4.2 The four assertions

| # | Claim | Criterion | Measured | verdict |
| :---: | :--- | :--- | :--- | :--- |
| 1 | Drift resilience without retraining | `RMSE_S2(B2) > 2 x RMSE_S2(B3)` | 0.0516 vs 0.0122 (4.22x) | PASS |
| 2 | Latency decoupling | `chatter_S3(B2) > 3 x chatter_S3(B3)` and `RMSE_S3(B2) > 2 x RMSE_S3(B3)` | 0.123 vs 0.013 (9.50x); 0.0933 vs 0.0125 | PASS |
| 3 | OOD abstention and safety | `violations_S4(B2) > 0`, `violations_S4(B3) = 0`, `veto < 10 ms` | 1,733 vs 0; 2.0 ms | PASS |
| 4 | Data and compute efficiency | classifier at `N <= 50` within 10 % of its ceiling; continuous `< 90 %` at `N = 500`, `< 95 %` at `N = 1000`, `>= 95 %` at `N = 2000`; compute `> 1000x` | 79.75 % of 85.13 %; 78.75 / 91.88 / 97.63 %; 99,502x | PASS |

### 4.3 What the bench does *not* prove

This is the part that most architecture write-ups omit, so it is stated explicitly:

* **It is a simulation.** The plant is two states plus one parasitic mode; there is no backlash, no
  quantisation, no actuator rate-dependent efficiency, no thermal model, no bus jitter distribution from
  a real stack. The numbers are internally consistent and reproducible - they are not extrapolations to
  a specific machine.
* **One seed, one fault realisation.** `seed = 7` for the four-stage run; the efficiency study averages
  4 seeds per sample size. The veto latency of 2.0 ms is a property of this fault onset (a step in the
  resonance amplitude), not a distribution.
* **The "end-to-end AI" is a stand-in.** A frozen computed-torque law with learned gains is a *fair*
  model of the failure mode (it encodes the nominal constants and has no integral action, no
  abstention), but it is not a network, and a real network might be better behaved - or worse, and
  nobody would know which without measuring.
* **The compute and memory figures are analytic.** They are derived from weight counts and inference
  rates, not profiled. The 99,502x is a design-space statement, not a benchmark.
* **The classifier's prototypes are hand-set** from a calibration grid, not learned by an optimiser.
  The sample-complexity study measures how many calibration runs would recover such class means, which
  is the relevant quantity for a nearest-mean classifier - but a different learner would have a
  different curve.
* **No hardware-in-the-loop, no certification evidence.** The IEC language in Section 6.2 is about
  architecture shape, not compliance.

The honest summary of the evidence: *within a synthetic but paired and reproducible bench, the layered
architecture dominates the monolithic one on exactly the axes that the mechanism analysis predicts
(drift, latency, OOD, actuator travel, footprint), and loses narrowly on the axis where the monolithic
one is supposed to be good (nominal tracking).* Both halves of that sentence are load-bearing.

---

## 5. Nominal tracking: where the monolithic model actually wins

In Stage 1 - the plant it was trained on - the end-to-end branch is the best tracker of the three:
RMSE 0.0087 rad against 0.0098 rad for the layered branch. That is a real result and it is not an
accident: a continuous policy that has memorised the exact inverse dynamics of the nominal plant beats a
three-bank PID that is merely well tuned. This is the regime the end-to-end literature reports, and it is
reported for the same reason: the training and test distributions coincide.

The same stage also shows the price. Over those 10 s the end-to-end branch spends 660.3 N*m of total
variation on the command against 51.3 N*m for the layered branch (12.9x) and 411.7 N*m for the fixed PID.
The end-to-end command is continuously active because it is continuously inverting a noisy measurement;
the layered branch's command is quiet because a PID around a commissioning feedforward only has to
correct the mismatch. Over the whole 40 s the totals are 1,442.9 / 1,664.8 / 342.0 N*m - the layered
branch is 4.2-4.9x lower than either alternative, which on a real axis is gear wear, motor heating and
audible noise.

A caution about that metric: low total variation is not by itself good. In Stage 3 the end-to-end branch
has the *lowest* TV of all (17.4 N*m) precisely because its command is frozen between 10 Hz frames - a
stale actuator is not a well-behaved actuator. Actuator wear has to be read together with tracking and
safety, never alone. (The repository reports per-stage TV, chatter and violations side by side for
exactly this reason.)

---

## 6. Engineering properties that follow from the structure

### 6.1 Zero heap, no allocation, O(1) worst-case execution time

`c99_layered_core/layered_gatekeeper.c` includes only its own header (which includes `<stdint.h>`). No
`<math.h>`, no `malloc`, no recursion, no file-static mutable state, no loops over memory. `|.|`,
clamping and the finite check are hand-written (a NaN fails both comparisons, an infinity fails one), so
no libm call is needed anywhere. The per-tick cost is a fixed straight-line sequence of branches and
arithmetic; there are no data-dependent loops, so the worst-case and the average case are the same
expression. Persistent state is a single `LayeredState_t` (84 B on the reference toolchain) passed in by
the caller, so multiple axes can run side by side and `sizeof` is known at compile time. Field tuning is
a 12 B struct. That is the entire mutable footprint of the safety-relevant code.

### 6.2 Relationship to IEC 61508 and IEC 62304 (stated carefully)

The architecture is *shaped* to fit a functional-safety lifecycle; it is not a certified element and this
repository makes no SIL or medical-device class claim. With that caveat, the shape is:

* **The safety function is separable and deterministic.** Everything that can move the actuator inside
  the layered design is in the C core: the hard envelope veto (trip at 90 % of the limit, clear at 75 %,
  with a 60 ms velocity look-ahead), the magnitude and slew limits, the watchdog hold, and the
  bumpless/anti-windup path. A separable deterministic element is what a safety case can be built
  around; a distributed one is not.
* **The learned element is an input, not an authority.** In IEC 61508 terms the classifier sits at the
  boundary of the safety function with a defined, range-checked interface, and its worst case is handled
  by an independent monitor (the residual probe) plus a fallback controller. The AI can be wrong; the
  system still has to be safe. That is a much easier argument than "the network is correct".
* **Change control is parameterised.** In IEC 62304 terms, adapting to a new unit or patient is a change
  to three scalar parameters with a documented physical meaning, regression-tested by the same
  deterministic suite plus the four bench assertions - not a new trained-model version requiring a new
  dataset, retraining and revalidation. This is the difference between a service visit and a software
  release.
* **The audit trail exists at runtime.** The sticky 5-bit fault mask plus per-cause counters
  (`safety_veto_count`, `ood_veto_count`, `low_confidence_veto_count`, `latency_hold_count`,
  `corrupted_frame_count`) make a post-incident reconstruction possible without instrumenting the AI at
  all.

None of that replaces a hazard analysis, a SIL verification, or a clinical evaluation. It removes the
part of the problem that is usually unsolvable: proving a black box correct.

### 6.3 Where the architectural argument stops

The argument above is about *allocation of responsibility* in a feedback loop with a non-smooth plant, a
hard envelope, a delay budget and a per-unit calibration burden. It does not say that coarse
classification is a universal interface to intelligence, and it does not say that learned components
belong only at 10 Hz. Section 7 is the honest boundary.

---

## 7. When you should still choose an end-to-end model

The layered architecture is a strong default for *safety-relevant physical control with a commissioning
model*, and a poor fit in several important cases. Choose the monolithic design - or a hybrid - when:

1. **The policy is the plant model.** In contact-rich or soft-body manipulation, the relevant physics may
   not admit a usable nominal model; if you cannot commission `y_hat`, you have no residual probe and no
   white-box feedforward, and the architecture's two central mechanisms are unavailable. Learning the
   dynamics *is* the task in that case.
2. **The task parameter is high-dimensional and continuous.** A three-way regime vocabulary is a
   straitjacket for dexterous in-hand manipulation, where every millisecond needs a different force
   distribution. If the "regime" is not identifiable from a handful of slow scalar features, do not
   pretend it is.
3. **The plant is slow and latency-tolerant.** If the dominant dynamics are below 1 Hz and the loop runs
   at 20 Hz, delay margin is not the binding constraint and a 10-50 Hz learned policy is perfectly
   sound. Much of process control and most "physical AI" demos are in this class.
4. **The data is genuinely abundant and the fleet is large.** If you have tens of thousands of units
   producing diverse data continuously and a training pipeline that can exploit it, the fleet-wide model
   may beat a per-unit commissioning procedure - the classifier's advantage (frozen model + 3 knobs) is
   largest when data per unit is scarce and engineering time per unit is expensive.
5. **You are learning a skill, not a servo.** If the hard part is *what* to do (grasp choice, gait phase,
   tool selection) rather than *how* to hold a torque loop, then put the learned part there - which is,
   note, still a layered design: the learned layer decides, the deterministic layer executes.
6. **The regime taxonomy is unavailable but the residual is usable.** A legitimate hybrid: keep the 1 kHz
   deterministic core and the envelope/abstention mechanics, and replace the coarse classifier with a
   learned policy that outputs *parameters of the certified controller family* (gains, setpoints, feedforward
   weights) rather than a torque. You keep the safety argument and the delay immunity; you lose the
   16-float auditability. That trade is often the right one.

The pattern across all six: **the monolithic design wins when the learned component's output is a
decision or a model, and loses when its output is the actuator command inside a hard-real-time loop.**

---

## 8. What would falsify this argument

An engineering argument is only useful if it can be wrong. This one would be weakened or broken by:

* A paired bench, on a real axis, where a continuous 1 kHz policy survives `+35 %` drift and `60-120 ms`
  jitter with zero retraining, zero envelope violations and no increase in actuator travel. That would
  remove the structural claim and reduce the layered design to a cost/verifiability trade.
* Evidence that a white-box nominal model cannot be commissioned to within the accuracy the residual
  probe needs - i.e. that the false-alarm rate of the OOD channel is unacceptable across the operating
  envelope. Then abstention becomes unusable and the architecture loses its safety mechanism.
* Evidence that regime labels are not identifiable from a few slow scalar features on a target domain
  (measured confusion is at chance), which would invalidate the coarse-layer premise for that domain.
* Evidence that per-unit commissioning cost dominates at fleet scale, i.e. that the three-knob procedure
  is more expensive in practice than a retraining pipeline at the numbers you actually ship.
* A demonstration that an end-to-end policy can be given a *verifiable* abstention mechanism and a
  *bounded* worst-case execution path at 1 kHz without giving up its nominal advantage. Nothing in
  current practice suggests this is close, but it is the natural counter-argument and should be watched.

If the first or the fifth happened, the honest conclusion would change: the layered architecture is not
"the" answer, it is the answer that keeps every failure mode bounded at the cost of some nominal
tracking. That trade is worth making for a machine that runs unattended for years; it is not worth making
for a demo that runs for ten seconds on the plant it was trained on.

---

## 9. Two paradigms, side by side

| Dimension | End-to-end continuous policy | Layered (coarse AI + deterministic core) |
| :--- | :--- | :--- |
| What the learned part outputs | `u(t)`, continuously, at loop rate | one label `{1..4}` + confidence, at 10 Hz |
| Nominal tracking | best (0.0087 rad in the bench) | slightly worse (0.0098 rad) |
| Under `+35 %` uncalibrated drift | 0.0516 rad, 4.22x worse | 0.0122 rad, unchanged |
| Across `0 -> +50 %` drift, AI frozen | 7.75x degradation | 1.36x degradation |
| Under 10 Hz + 60-120 ms + 15 % drops | chatter 0.123, RMSE 0.0933 | chatter 0.013, RMSE 0.0125 |
| Unseen OOD fault | 1,733 envelope breaches | 0 breaches, abstains in 2.0 ms |
| Calibration data needed | > 1,000 samples | ~50 samples |
| Compute / memory | 2.40e9 MAC/s, 5.60e6 B | 2.41e4 MAC/s, 2.21e3 B |
| Actuator travel over 40 s | 1,442.9 N*m | 342.0 N*m |
| Worst-case execution time | data- and cache-dependent | O(1), straight-line, no allocation |
| Failure mode when wrong | saturation, oscillation, envelope breach | named veto, sticky audit bit, safe regulator |
| Field adaptation | data collection + retraining | three scalar knobs, AI frozen |
| Verification object | behaviour of 1.2 M parameters | 105 deterministic assertions + a 16-float table |
| Where it is the right choice | perception-action coupling, no usable model, slow loops, abundant data | safety-relevant physical control with a commissioning model |

---

## 10. Reproduce every number in this document

From the repository root (Windows PowerShell; MATLAB with Simulink, R2024b used for the reference run):

```powershell
# 40 s four-stage bench + efficiency study + 3 domains + 3 figures + the 4 assertions
matlab -batch "cd('matlab_simulink'); run_all_benchmarks"

# standalone C99 core: 105 assertions, no MATLAB involved
gcc -std=c99 -pedantic -Wall -Wextra -O2 c99_layered_core/test_gatekeeper_standalone.c c99_layered_core/layered_gatekeeper.c -o rta_test.exe
.\rta_test.exe
```

The acceptance strings are `ALL FOUR ASSERTIONS PASSED.` (exit code 0) and
`SUMMARY: n_pass=105, n_fail=0`. The full metrics table is printed by the first command; the per-stage,
per-branch values quoted above are those printed lines, and the sample-complexity percentages are the
`panelA` fields returned by the efficiency study (the console table rounds them to one decimal).

Read next: `README.md` (overview and quick start), `docs/domain_mapping_guide.md` (the same architecture
on industrial servo/WAAM, edge VLA and closed-loop neuromodulation),
`c99_layered_core/layered_gatekeeper.h` (the contract), `AGENTS.md` (ground truth for coding agents).
