# Systems engineering for qualitative AI and quantitative control

The central decision is who owns the actuator. A slow semantic model can choose an operating regime;
a deterministic controller should own the quantitative command, the integral correction and the
recovery behavior. That split keeps learning an environment description separate from maintaining a
physical invariant under disturbances.

Three routes are available. A white-box controller combines nominal inverse dynamics with feedback
and anti-windup; it is strong in familiar conditions, but one fixed tuning must trade disturbance
rejection against contact compliance and measurement-noise amplification. A fast-slow learned
controller separates context inference from a continuous action head; running the head at the fast
rate keeps perception latency out of the actuation path, yet its local mapping depends on the dynamics
represented in training data. The layered route assigns context selection to the slow interface and
quantitative feedback to a deterministic bank; integral states absorb persistent mismatch, physical
knobs remain available after deployment, and the residual probe can reject unfamiliar operation.
The slow interface is model-agnostic: a prototype classifier, a rule supervisor or a VLM can implement
the same contract.

## Six field problems and the mechanisms that answer them

### 1. Scarce samples, expensive trials

**Problem.** A continuous action model needs dense labeled rollouts. The conditioned RBF action head
(B2) is trained on 16,000 nominal simulation samples in this benchmark. Figure 2 Panel A shows that its
nominal RMSE has already converged at 64-1,024 samples, yet without fast-loop safety gating it still
cannot remove the S4 envelope violations. The sample count illustrates how high the engineering
threshold of dense trial-and-error sampling plus retraining remains on real hardware; it is not a
measured machine-time or wear budget.

**Where the alternatives fail.** Retraining on the target machine is the natural response, but the
data cost and the trial risk scale with every unit and every operating condition. A fixed rule tree
avoids labels yet cannot express regime-dependent continuous dynamics.

**Mechanism.** The 10 Hz loop needs only qualitative coverage: 36 labeled macro windows, 15 scalars,
60 bytes, calibrated by `auto_calibrate()` on a CPU. The archive establishes the regime prototypes and
the confidence thresholds; no GPU and no action labels are involved. B2 remains the comparison point
that quantifies the data cost of the continuous route.

### 2. Multi-condition conflict: stiction versus compliant contact

**Problem.** One task requires breaking static friction under heavy load and yielding under unilateral
contact. Those objectives pull the same gain set in opposite directions.

**Where the alternatives fail.** B1 is the white-box baseline with the same nominal inverse-dynamics
feedforward, anti-windup and limits as B3. It reaches the lowest raw S2 RMSE of the comparison,
0.017125 rad, but its fixed tuning exhibits a waterbed: S3 contact reaches 64.59 N at the wall, an
alpha = 0.3 output low-pass still leaves 63.47 N, and all six candidate tunings fail the contact
criterion.

**Mechanism.** The bottom layer holds four deterministic banks: R1 nominal, R2 high load, R3 compliant
contact and R4 safe fallback. Every bank runs tracking anti-windup back-calculated from the actually
applied command, so a bank switch releases no stored energy. When a veto or the permanent latch selects
R4, the blend filter is bypassed and blend = 1.0, so the emergency damping acts on the first tick.

### 3. Unknown manifolds and incipient oscillation

**Problem.** A new installation can produce behavior outside the calibrated manifold: an unmodeled dead
zone plus a 4 Hz mounting mode is one evaluated benchmark case. The loop must not keep applying a
confident-looking command while the structure oscillates.

**Where the alternatives fail.** The fixed-threshold rule tree compared in this benchmark, or a fuzzy
controller, encodes decision boundaries but - without an explicitly embedded manifold-distance or
density model - carries no notion of distance from the calibrated distribution. The physical rule
supervisor agrees with the prototype classifier on the main S1/S2 decisions and the 19.47 N S3 contact
peak, yet in the off-manifold case it cannot abstain, because abstention requires a measure of how far
the current feature point lies from known operation.

**Mechanism.** The prototype model exposes `ai_confidence` as a distance-based measure. Outside the
manifold the confidence collapses, the 1 kHz core registers GATE_VETO_LOW_CONFIDENCE and runs R4 with
fixed damping until a normal regime is confidently reported again.

### 4. Unit, batch and unit-to-unit drift without retraining compute

**Problem.** A forward heavy load of +35% and +50%, a cross mismatch of J +40% and b -30%, and a reverse
light load of -25% all move the plant away from its commissioned model, while the field has no
retraining budget.

**Where the alternatives fail.** Scheduled recalibration assumes a dataset and a training run per
condition; a learned action head needs the same and adds label collection. Neither fits a technician
with a torque wrench and a stopwatch.

**Mechanism.** The design decouples two objects orthogonally: the frozen AI manifold topology, which
describes qualitative regimes, and the 12-byte `FieldTuning_t` physical knobs (`gain_scale`,
`damping_scale`, `safety_margin_scale`), which absorb quantitative displacement. `auto_field_tune()`
runs six constrained trials over a fixed gain/damping grid, accepts only candidates that satisfy load,
contact and envelope constraints, and keeps the classifier frozen throughout.

### 5. The 100 ms blind window, transients and link loss

**Problem.** The 100 ms blind window belongs to the 10 Hz slow-loop context update, which sees a new
frame every 100 ms. Stimulation pulses, commutation edges and sensor transients fall inside that window,
and a dropped link can leave a stale command in place.

**Where the alternatives fail.** A slower supervisory scan only widens the interval. B2's 1 kHz
continuous action head reads fast-loop state every millisecond, but its context feature z is still
constrained by the 100 ms slow-loop period and by transport delay. A transported frame age can be
forged or frozen, and without local evidence the loop cannot tell a quiet plant from a silent mailbox.

**Mechanism.** A 32-byte probe runs at 1 kHz: a three-point median rejects single-tick glitches, a
cascaded high-pass extracts broadband innovation, and attack/release envelopes convert it into a
dimensionless residual. During a scheduled artifact `blanking_active` freezes every probe state and
returns 0.0f, so an already elevated residual cannot hold the veto through the blanking window. The
96-byte core adds a 25 ms phase-plane look-ahead, an independent residual veto, and a local clock lease
on `outer_seq` so a frozen transported age cannot mask a stalled mailbox. Native HOLD is bounded: past
`max_hold_ticks` (100 ticks in the reference) the core enters GATE_SAFE_ZERO, clears the integrators
and ramps the command to zero at the configured `u_slew_max_per_s` limit.

### 6. Bare-metal limits and incident accountability

**Problem.** The control task must run in a 1 kHz interrupt on a fixed-memory target, with no heap and no
operating system. An incident report must also explain which decision and which root cause placed the
actuator in a safe state.

**Where the alternatives fail.** Deploying on a Python or inference runtime adds allocation, scheduling
jitter and untraceable state. A veto without persistent evidence cannot answer the post-incident
question.

**Mechanism.** The core is ANSI C99 with zero dynamic memory, statically sized state and object text of
6,144 bytes at `-Os` and 7,172 bytes at `-O2` (GCC 4.9.2 TDM64, x86-64). Its ABI occupies
96/140/12/40/28 B for state/config/tuning/input/output, plus the 32 B probe. The 7-bit
`fault_audit_bitmask` records latency, residual OOD, low confidence, envelope breach, NaN corruption,
HOLD timeout and permanent latch. Three cumulative safety/OOD episodes latch GATE_LATCHED_SAFE
permanently; only an explicit `Layered_AcknowledgeFaultLatch()` releases the latch, restores the episode
tolerance after the cooling period, and preserves the lifetime counters.

## Bandwidth and authority are separate choices

A fast inference rate says when a command is computed; it does not establish the command's
admissibility. A complete architecture also specifies which state estimates, residuals, actuator limits
and recovery conditions can override that command.

The gatekeeper applies a fixed priority chain: state envelope, residual, confidence, then native
latency HOLD. The observable states are explicit — GATE_NORMAL, GATE_HOLD_LATENCY,
GATE_VETO_OOD_RESIDUAL, GATE_VETO_LOW_CONFIDENCE, GATE_VETO_SAFETY_ENVELOPE, GATE_SAFE_ZERO and
GATE_LATCHED_SAFE. Frame age remains observable. Dwell and recovery prevent unstable re-arming, while
tracking anti-windup keeps every inactive controller consistent with the applied command. Magnitude and
slew limits belong to the actuator contract on every tick.

Safe fallback is an explicit control regime. Its reference behavior, damping and feedforward permissions
must be defined for the target device. A native HOLD holds the last actuator command, not a mechanical
position or a physiological state; the device adapter defines the physical consequence. The core's R4
returns toward a commissioned safe zero with fixed gains and no feedforward, and corrupt slow decisions
select that closed-loop law while valid fast measurements remain active.

## Adaptation belongs to identifiable physical parameters

A new device can preserve the semantic regime definition while changing quantitative dynamics.
Small-sample calibration establishes the regime prototypes and confidence thresholds. Subsequent field
tuning freezes that archive and searches bounded gain and damping adjustments under the same
deterministic safety controls. Trial acceptance jointly constrains load/stiction error, contact force
and fallback occupancy.

This separation makes commissioning evidence readable: the record shows which physical knobs changed,
which trial constraints were checked and how tracking changed. A genuinely new physical domain requires
new signal definitions, a nominal model and a controller bank; its semantic archive is calibrated only
after that interface exists. Continuous-policy retraining remains a complementary comparison: measure
nominal accuracy, frozen transfer and recovery after collecting target-unit action labels.

## Experiments must isolate causes

Static friction needs a model that actually sticks at the chosen integration step. Unilateral contact
needs a force metric as well as position error. Unknown flexible modes need a broadband residual probe
without a hidden fault-frequency input. Noise, context delay and contact stiffness should have
independent controls.

The comparison keeps the baselines at full strength. B1 and B3 share the identical nominal
inverse-dynamics feedforward, anti-windup and limits, and B1's S2 result remains the lowest raw error
in the table. B2 runs its action head at the full 1 kHz rate (40,000 inferences, zero fast-loop state
delay) and records the lowest S3 contact force at 12.52 N; without an explicit integrator or physical
gate it carries larger S2 error and 589 S4 violation ticks. The rules tree and the prototype
classifier agree on the main S1/S2 results and on the 19.47 N S3 contact peak; their S3 R3 occupancy
differs by 150 ticks (93.49% versus 94.99%), visible in the third decimal of the S3/S4 RMSE
(0.075100 / 0.540437 versus 0.069478 / 0.539985). The layered branch records S4 RMSE 0.539985 rad with
TV 1545.01 N·m, while B1 keeps the lower raw S2 error and B2 keeps its S3 contact advantage; these
observations define the interpretation of the benchmark.

Cross-language replay closes the verification loop. The C99 standalone regression passes 233 checks with
zero failures, and the 120-line `gate_template.c` self-check prints PASS. The Python/C `--selftest`
replays 6,000 frames (R1 = 901, R2 = 900, R3 = 304, R4 = 3,895, including acknowledgement re-arming and
the 7→5→3→1→0 ramp-down) with identical discrete outputs and max |du| = 0, bit-exact in binary32. The
MATLAB/C equivalence run covers 40,000 normal frames, 40,000 corrupted-mailbox frames and four
40,000-frame runs with illegal knob injection at max |du| = 5e-9. A 10,000-tick `outer_seq` lease
replay adds native HOLD → `SAFE_ZERO`, the zero-sequence corruption path and cooldown recovery at
max |du| = 4.5e-8. The Level-2 S-Function replays 1,100
frames with all seven outputs at max diff = 0.0.

## A generic interface for constrained devices

Industrial WAAM and PLC servo systems use the slow interface for process and load conditions, with
deterministic regulation and actuator constraints below it. Edge VLA systems use semantic task/context
decisions above torque or impedance control. Closed-loop aDBS/SCS systems map biomarker/interface states
to device-defined regulation, output envelopes and timing constraints.

These are equal examples of the same ownership pattern. Other systems fit when they can define measured
states, a commissioned model, explicit fallback and quantitative limits. Coupled channels retain
individual state, probes and timestamps, with shared constraints handled by the device adapter.

Reproducible results and resource measurements are maintained in [README.md](../README.md).
[AGENTS.md](../AGENTS.md) defines the implementation contract, and
[the domain guide](domain_mapping_guide.md) defines how to map it.
