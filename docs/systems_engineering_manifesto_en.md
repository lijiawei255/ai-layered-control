# Systems engineering for qualitative AI and quantitative control

The central design decision is ownership of the actuator. A slow semantic model can select an
operating regime while a deterministic controller owns the quantitative command, integral correction
and recovery behavior. This division separates learning a description of the environment from
maintaining a physical invariant under disturbances.

## The three control routes

A modern white-box controller combines nominal inverse dynamics with feedback and anti-windup.
It offers a strong reference in familiar operating conditions. A single fixed tuning must still
trade disturbance rejection against contact compliance and measurement-noise amplification.

A fast-slow learned controller separates context inference from a continuous action head.
Running the action head at the fast-loop rate avoids putting perception latency directly in the
actuation path. Its local mapping nevertheless depends on the dynamics represented by training data.
Friction discontinuities, load changes and flexible modes test that mapping under closed-loop feedback.

The layered route assigns context selection to the slow interface and quantitative feedback to a
deterministic controller bank. Its integral states absorb persistent mismatch, its physical knobs
remain available after deployment, and its residual probe can reject unfamiliar operation.
The slow interface is model-agnostic: a prototype classifier and a physical rule supervisor can
implement the same contract.

## Bandwidth and authority are separate choices

A fast inference rate says when a command is computed; it does not establish the command's
admissibility. A complete control architecture also specifies which state estimates, residuals,
actuator limits and recovery conditions can override that command.

The gatekeeper applies a fixed priority chain. State-envelope and residual vetoes dominate semantic
confidence. Frame age remains observable. Dwell and recovery prevent unstable re-arming, while
tracking anti-windup keeps inactive controller states consistent with the applied command.
Every tick's magnitude and slew limits belong to the actuator contract.

Safe fallback is an explicit control regime. Its reference behavior, damping and feedforward
permissions must be defined for the target device. Holding the previous actuator command is a
different behavior from holding a mechanical position or a physiological state.
The core's R4 returns toward a commissioned safe zero with fixed gains and no feedforward.
Corrupt slow decisions select that closed-loop law while valid fast measurements remain active.

## Adaptation belongs to identifiable physical parameters

A new device can preserve the semantic regime definition while changing quantitative dynamics.
Small-sample calibration establishes the regime prototypes and confidence thresholds.
Subsequent field tuning freezes that archive and searches bounded gain and damping adjustments
under the same deterministic safety controls.
Trial acceptance jointly constrains load/stiction error, contact force and fallback occupancy.

This separation makes commissioning evidence readable: the record shows which physical knobs
changed, what trial constraints were checked and how tracking changed.
A genuinely new physical domain requires new signal definitions, a nominal model and a controller bank.
Its semantic archive is calibrated only after that interface exists.

Continuous-policy retraining is a complementary comparison. Sample-efficiency experiments should
measure nominal accuracy, frozen transfer and recovery after collecting target-unit action labels.
Action labels and macro-feature windows are different sample units; their counts are reported separately.

## Experiments must isolate causes

Static friction needs a model that actually sticks at the chosen integration step.
Unilateral contact needs a force metric as well as position error.
Unknown flexible modes need a broadband residual probe without a hidden fault-frequency input.
Noise, context delay and contact stiffness should have independent controls.

Compare fixed controller modes, a measured-feature classifier and a physical rule supervisor.
Compare full residual gating with spike-free and residual-only operation.
Check passive free response and integration-step convergence before attributing an instability
to the controller. Distinguish anomaly-detection delay from gate response once evidence arrives.

Safety-envelope breaches, actuator variation and contact force answer different questions.
A compliant controller may accept position error to reduce contact loading.
A bounded command can still excite an unsafe physical trajectory.
These observations define the interpretation of the benchmark rather than a single scalar ranking.

## A generic interface for constrained devices

Industrial WAAM and PLC servo systems use the slow interface for process and load conditions,
with deterministic regulation and actuator constraints below it.
Edge VLA systems use semantic task/context decisions above torque or impedance control.
Closed-loop aDBS/SCS systems map biomarker/interface states to device-defined regulation,
output envelopes and timing constraints.

These are equal examples of the same ownership pattern.
Other systems fit when they can define measured states, a commissioned model, explicit fallback
and quantitative limits. Coupled channels retain individual state, probes and timestamps,
with shared constraints handled by the device adapter.

The reproducible evidence and resource measurements are maintained only in
[README.md](../README.md). [AGENTS.md](../AGENTS.md) defines the implementation contract;
[the domain guide](domain_mapping_guide.md) defines how to map it.
The repository's scope statement is the closing paragraph of the README.
