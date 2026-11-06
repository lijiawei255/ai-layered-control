# Systems Engineering Manifesto: Layered Control Architecture and Deterministic Safety Boundaries

The central decision is who owns the actuator. The answer is a split: a slow semantic model selects
the operating regime, and a deterministic controller owns the quantitative command, the integral
correction and the recovery behavior. This split turns "learn a description of the environment" and
"maintain a physical invariant under disturbances" into two separate problems, each designed and
verified on its own.

Three routes are available. A white-box controller combines nominal inverse dynamics with feedback
and anti-windup. It is strong in familiar conditions, but a single fixed gain set must trade
disturbance rejection against contact compliance and measurement-noise amplification at the same
time. A fast-slow learned controller separates context inference from a continuous action head.
Because the head runs at the fast rate, perception latency stays out of the actuation path; the cost
is that its local mapping depends on the dynamics covered by training data, and uncovered regimes
carry no guarantee. The layered route assigns regime selection to the slow interface and
quantitative feedback to a deterministic bank: integral states absorb persistent mismatch, physical
knobs remain available after deployment, and a residual probe can reject unfamiliar operation. The
slow interface is model-agnostic. A prototype classifier, a rule supervisor or a VLM can all
implement the same contract.
Traditional gain scheduling or fixed-threshold rule trees only partition known regimes; when they
encounter unmodeled mechanical resonance or an out-of-distribution (OOD) disturbance, they still emit
the nearest regime's control law. Unlike a traditional Simplex / runtime assurance (RTA) wrapper that
only intercepts a continuous AI command at the safety boundary, this architecture strips continuous
actuation from the slow loop altogether and fixes the authority boundary as a cross-loop contract via
fast-slow dual-layer active abstention: the 10 Hz slow loop performs manifold-distance low-confidence
abstention and retreats to R4 damping, and the 1 kHz fast loop executes a millisecond-scale forced veto
from an independent physical residual probe.

## Layered system boundaries (L0–L4)

| Layer | Responsibility |
| :--- | :--- |
| L0 | Hardware safety channel: an independent safety PLC (ISO 13849 Cat.3/4, 1oo2D), dual-channel sensor cross-check, and hardware STO / IEC 60204-1 Stop 0. |
| L1 | Deterministic fast inner loop: four-bank PID with full-branch reverse-tracking anti-windup plus nominal feedforward; DOB/ESO/ADRC or bounded adaptive compensation can be added in the control-law slot or via `u_ff`. |
| L2 | Runtime safety arbitration and degradation chain: 96 B C99 Simplex state machine, 1D ZCBF signed envelope look-ahead, blanking hold, per-axis R4 → SAFE_ZERO → LATCHED_SAFE; multi-axis aggregation of `gate_state` plus a shared ack bus for Stop 1/2. (Under ISO/IEC TR 5469 the physical travel/torque/slew/timeout checks are exact a-priori inequalities, and the main-control AI plus the supervisory AI must not extrapolate blindly together out of distribution; this layer is therefore deterministic code, never a second black-box model.) |
| L3 | Slow qualitative supervision and calibrated abstention: 10 Hz discrete regime advice plus calibrated distance-based confidence. |
| L4 | Fleet lifecycle closure: frozen AI weights during operation, append-only 7-bit fault audit mask as trigger tags for OOD data capture, offline prototype supplementation and safety-case update. |

## Six field problems and the mechanisms that answer them

### 1. Scarce samples and the compute cost of continuous policies

**Problem:** A continuous action model needs dense labeled rollouts. In this benchmark the
conditioned RBF action head (B2), which represents the fast-slow learned route, is trained on 16,000
nominal simulation samples. Figure 2 Panel A shows that its nominal RMSE has already converged at
64-1,024 samples, yet without fast-loop safety gating it still cannot remove the S4 envelope
violations. The sample count illustrates how high the engineering threshold of dense trial-and-error
sampling plus retraining remains on real hardware; it is not a measured machine-time or wear budget.

**Where the alternatives fail:** Retraining on the target machine is the natural response to scarce
samples, but the data cost and the trial risk scale with every unit and every operating condition. A
fixed rule tree avoids labels yet cannot express regime-dependent continuous dynamics. Both routes
stall at the same point.

**Mechanism:** Continuous quantitative feedback and trajectory tracking are already carried by the
four pre-commissioned deterministic controller banks and the nominal feedforward, so the 10 Hz slow
loop only has to perform coarse discrete regime classification and manifold-distance abstention. It
therefore needs only qualitative coverage: 36 labeled macro windows and 15 statistical scalars (60 bytes)
calibrated by `auto_calibrate()` on a CPU. The archive establishes the regime prototypes and the
confidence thresholds. No GPU and no action labels are involved. The template's built-in 15-statistical-scalar
nearest-centroid classifier and six-point constrained search (`auto_field_tune`) are a compact
NumPy implementation without a machine-learning framework; in a real machine the same interface can host
a physical rule tree with an out-of-range fallback branch, safe Bayesian optimization, or another
classification model. B2 serves as the comparison point that quantifies the data cost of the
continuous route: it evaluates a continuous action head over **5,664 scalars / 22,656 B** on every
1 ms tick, whereas this architecture only classifies discrete regimes plus a manifold-distance
confidence at 10 Hz with a model of **15 statistical scalars / 60 B** — reducing the slow-loop task from 1 kHz
continuous regression to 10 Hz discrete classification and cutting both sample complexity and online
inference compute.

### 2. Multi-condition conflict: stiction versus compliant contact

**Problem:** One task requires breaking static friction under heavy load and yielding under
unilateral contact. These two objectives pull the same gain set in opposite directions, so no fixed
tuning can satisfy both.

**Where the alternatives fail:** The fixed-gain white-box baseline B1 shares the identical nominal
inverse-dynamics feedforward, anti-windup and limits with the layered branch B3. B1 reaches the
lowest raw S2 RMSE of the comparison, 0.017125 rad, but its fixed tuning exhibits a waterbed effect:
S3 contact reaches 64.59 N at the wall, an alpha = 0.3 output low-pass still leaves 63.47 N, and all
six candidate tunings fail the contact criterion.

**Mechanism:** The bottom layer holds four deterministic banks: R1 nominal, R2 high load, R3
compliant contact and R4 safe fallback. The active bank's integrator is back-calculated against the
magnitude/slew-limited command before the blend filter, and inactive banks track the applied command,
so a bank switch releases no stored integral energy. When a veto or the permanent latch selects R4,
the core bypasses the blend filter and sets blend = 1.0, so the emergency damping acts on the first
tick, without the delay of first-order bumpless transfer.

### 3. Unknown manifolds and incipient oscillation

**Problem:** A new installation can operate outside the calibrated manifold. The evaluated benchmark
case runs at a normal amplitude and commissioning noise with an in-box coupled constant load-torque bias and a 1 % per-frame split-conformal false-positive budget plus 5-frame persistence;
once the operating point leaves the calibrated manifold, the loop must not keep applying a
confident-looking command.

**Where the alternatives fail:** The fixed-threshold rule tree, or a
fuzzy controller, encodes decision boundaries but - without an explicitly embedded
manifold-distance or density model - carries no notion of distance from the calibrated
distribution. The physical rule supervisor agrees with the prototype classifier on the main S1/S2
decisions and the 19.9 N S3 contact peak, yet in the off-manifold case it cannot abstain. Abstention
first requires a measure of how far the current feature point lies from known operation, and the
rule tree has none.

**Mechanism:** The prototype model exposes `ai_confidence` as a distance-based measure. Outside the
manifold the confidence collapses and triggers an active low-confidence abstention: the 1 kHz core
registers GATE_VETO_LOW_CONFIDENCE and runs R4 with fixed damping until a normal regime is
confidently reported again — abstention becomes an observable, adjudicable and deterministic event.

### 4. Unit, batch and unit-to-unit drift without retraining compute

**Problem:** A forward heavy load of +35% and +50%, a cross mismatch of J +40% and b -30%, and a
reverse light load of -25% all move the plant away from its commissioned model, while the field has
no retraining budget.

**Where the alternatives fail:** Scheduled recalibration assumes a dataset and a training run per
condition; a learned action head adds label collection on top. Neither route works for a technician
equipped with a torque wrench and a stopwatch.

**Mechanism:** The design decouples two objects orthogonally: the frozen AI manifold topology, which
describes qualitative regimes, and the 12-byte `FieldTuning_t` physical knobs (`gain_scale`,
`damping_scale`, `safety_margin_scale`), which absorb quantitative displacement. `auto_field_tune()`
runs six constrained trials over a fixed gain/damping grid, accepts only candidates that satisfy
load, contact and envelope constraints, and keeps the classifier frozen throughout.

### 5. The 100 ms blind window, transients and link loss

**Problem:** The 100 ms blind window comes from the 10 Hz slow-loop context update: the slow loop
sees a new frame only every 100 ms. Stimulation pulses, commutation edges and sensor transients all
fall inside that window, and a dropped link can leave a stale command in place.

**Where the alternatives fail:** A slower supervisory scan only widens the interval. B2's 1 kHz
continuous action head reads fast-loop state every millisecond, but its context feature z is still
constrained by the 100 ms slow-loop period and by transport delay. A transported frame age can be
forged or frozen; without local evidence the loop cannot tell a quiet plant from a silent mailbox.

**Mechanism:** A 32-byte probe runs at 1 kHz: a three-point median rejects single-tick glitches, a
cascaded high-pass extracts broadband innovation, and attack/release envelopes normalize it into a
dimensionless residual. During a scheduled artifact, `blanking_active` freezes every internal filter
state and holds the pre-blanking residual (`return p->residual`): a healthy periodic pulse cannot
raise the residual, a sustained fault spanning periodic blanking keeps one OOD latch episode, and the
residual decays along the release envelope outside the blanking window. The
96-byte core adds three further layers: a 25 ms signed envelope look-ahead, an independent residual
veto, and a local clock lease on `outer_seq` so a frozen transported age cannot mask a stalled
mailbox. A stale slow mailbox with healthy fast sensors keeps the loop closed on the fixed R4 law;
past `max_hold_ticks` (100 ticks in the reference) the core enters GATE_SAFE_ZERO, clears the
integrators and ramps the command to zero in per-tick steps of
`max(u_slew_max_per_s * dt_s, 1 binary32 ULP(|u_prev|))`. The normal path obeys
`|u(k) - u(k-1)| <= u_slew_max_per_s * dt_s`; the single documented exception is this one-ULP floor
toward zero, which applies only when the legal slew budget is below one representable step, only
reduces the command magnitude and never re-energises.

### 6. Bare-metal limits and incident accountability

**Problem:** The control task must run in a 1 kHz interrupt on a fixed-memory target, with no heap
and no operating system. An incident report must also state which decision and which root cause
placed the actuator in a safe state.

**Where the alternatives fail:** A Python or inference runtime brings allocation, scheduling jitter
and untraceable state. A veto without persistent evidence cannot answer the post-incident question.

**Mechanism:** The core is ANSI C99 with zero dynamic memory and statically sized state. Its ABI occupies
96/140/12/40/28 B for state/config/tuning/input/output, plus the 32 B probe. The 7-bit
`fault_audit_bitmask` records seven event classes: latency, residual OOD, low confidence, envelope
breach, NaN corruption, HOLD timeout and permanent latch. Three cumulative safety/OOD episodes latch
GATE_LATCHED_SAFE permanently; only an explicit `Layered_AcknowledgeFaultLatch()` releases the latch,
restores the episode tolerance after the cooling period, and preserves the lifetime counters.

## Bandwidth and authority are separate choices

A fast inference rate says when a command is computed; whether the command is admissible is decided
by another set of conditions. A complete architecture specifies which state estimates, residuals,
actuator limits and recovery conditions can override that command. The 1 kHz deterministic core
arbitrates the 10 Hz slow loop under zero trust: the slow loop emits only operating-regime proposals
awaiting review, while numeric validity, frame-age authenticity (with `outer_seq` enabled), the physical-residual envelope and
the recovery cooldown are all verified independently by the fast loop.

The gatekeeper applies a fixed priority chain: state envelope first, then residual, then confidence,
and finally native latency HOLD. The observable states are explicit — GATE_NORMAL,
GATE_HOLD_LATENCY, GATE_VETO_OOD_RESIDUAL, GATE_VETO_LOW_CONFIDENCE, GATE_VETO_SAFETY_ENVELOPE,
GATE_SAFE_ZERO and GATE_LATCHED_SAFE — and frame age remains observable. Dwell and recovery cooling
constrain re-arming. Tracking anti-windup keeps every inactive controller consistent with the
applied command. Magnitude and slew limits belong to the actuator contract on every tick: the normal
path obeys `|u(k) - u(k-1)| <= u_slew_max_per_s * dt_s`, and GATE_SAFE_ZERO's one-ULP floor toward
zero is the single documented exception.

Safe fallback is an explicit control regime, and its reference behavior, damping and feedforward
permissions must be defined for the target device. A stale slow mailbox with healthy fast sensors
keeps the measured 1 kHz loop closed on the fixed R4 law; only an unusable fast input holds the last
actuator command open-loop. A held command is a command; its physical consequence is defined by the
device adapter. The core's R4 regulates
toward a commissioned safe zero with fixed gains and no feedforward. When slow decisions are corrupt,
that closed-loop law keeps running as long as valid fast measurements remain active.

## Adaptation belongs to identifiable physical parameters

A new device can preserve the semantic regime definition while changing quantitative dynamics.
Small-sample calibration first establishes the regime prototypes and confidence thresholds. Field
tuning then freezes that archive and searches bounded gain and damping adjustments under the same
deterministic safety controls. Trial acceptance jointly constrains load/stiction error, contact
force and fallback occupancy.

This separation makes commissioning evidence directly readable: the record shows which physical
knobs changed, which trial constraints were checked and how tracking changed. A genuinely new
physical domain first defines its signals, a nominal model and a controller bank, then builds its
own semantic archive. Continuous-policy retraining remains a complementary comparison: after
collecting target-unit action labels, measure nominal accuracy, frozen transfer and recovery.

## Experiments must isolate causes

The experimental design lets each causal chain be tested on its own. Static friction needs a model
that actually sticks at the chosen integration step. Unilateral contact needs a force metric as well
as position error. Unknown flexible modes need a broadband residual probe that takes no hidden
fault-frequency input. Noise, context delay and contact stiffness should have independent controls.

The comparison keeps the baselines at full strength. B1 and B3 share the identical nominal
inverse-dynamics feedforward, anti-windup and limits, and B1's S2 result remains the lowest raw error
in the table. B2 runs its action head at the full 1 kHz rate (40,000 inferences, zero fast-loop
state delay); with its clamped error-integral history feature but no external deterministic PID stabilizer, and no physical gate, it carries larger S2 error, 65.71 N S3 contact and 824 S4 violation ticks. The rule tree and the prototype classifier agree on the main
S1/S2/S3/S4 results, including the 19.9 N S3 contact peak, but in the off-manifold case the rule tree
cannot abstain. The layered branch records S4 RMSE 0.534138 rad with TV 1528.1 N·m and zero S4
violations, while B1 keeps the lower raw S2 error; these observations define the interpretation of
the benchmark.

Replay and regression evidence is collected in [README §3.6 Reproduce](../README.md#36-reproduce).

## A generic interface for constrained devices

Industrial WAAM and PLC servo systems use the slow interface for process and load conditions, with
deterministic regulation and actuator constraints below it. Edge VLA systems place semantic
task/context decisions above torque or impedance control. Closed-loop aDBS/SCS systems map
biomarker/interface states to device-defined regulation, output envelopes and timing constraints.

The three domains differ, but each is an equal instance of the same ownership pattern. Other systems
fit when they can define measured states, a commissioned model, explicit fallback and quantitative
limits. Coupled channels retain individual state, probes and timestamps, with shared constraints
handled by the device adapter.

Reproducible results and resource measurements are maintained in [README.md](../README.md).
[AGENTS.md](../AGENTS.md) defines the implementation contract, and
[the domain guide](domain_mapping_guide.md) defines how to map it.

> **Scope:** This repository is an uncertified engineering reference architecture maintained by an independent developer; real-equipment integration requires deployer-owned hazard validation and independent hardware safety protection (E-stop, STO, hardware charge-balance limiters; see the root [README.md License and citation](../README.md#license-and-citation)). The application mappings are directions for adaptation, not hardware, clinical or functional-safety verification or certification, and they do not replace certification or lifecycle processes under IEC 61508, ISO 13849, ISO 26262 or IEC 62304.
