// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jiawei Li
// Source: https://github.com/lijiawei255/ai-layered-control
// License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
// Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
/*
 * layered_gatekeeper.h
 * ---------------------------------------------------------------------------
 * Zero-dependency ANSI C99 reference core for the layered AI control
 * architecture ("slow coarse AI + fast deterministic core").
 *
 * The whole point of this module is that it is boring to deploy and boring to
 * certify:
 *   - no floating-point library functions, no dynamic memory (malloc-free),
 *     no OS calls, no recursion; bounded, data-independent O(1) control flow;
 *     target WCET is established by measurement;
 *   - the only include is <stdint.h>; select C99 mode on the target compiler
 *     and verify its floating-point behavior and build options;
 *   - fixed, statically-sized state (LayeredState_t), safe for 1 kHz ISRs;
 *   - sticky black-box fault audit bitmask for post-incident forensics
 *     with explicit gate decisions.
 *
 * Architecture (see README.md for the full picture):
 *
 *   slow outer loop (10 Hz, any Linux/Android/edge accelerator)
 *        |  mailbox: regime in {1,2,3}, confidence in [0,1], heartbeat
 *        v
 *   THIS MODULE (1 kHz, deterministic, 96 B state + 140 B config + 12 B tuning)
 *        - fast-input firewall; bad slow mailboxes/tuning request closed-loop R4
 *        - heartbeat watchdog: survives 10 Hz jitter / dropped frames; a stale
 *          mailbox keeps the measured loop closed on the fixed R4 bank instead
 *          of freezing the last torque
 *        - physics-residual + confidence veto: explicit ABSTENTION
 *        - predictive state-envelope veto: signed |x + T*x_dot| look-ahead plus
 *          the direction-blind instantaneous guard
 *        - anti-chatter minimum dwell time on normal regime transitions
 *        - post-fault recovery cooldown before AI is re-armed
 *        - controller bank with per-controller anti-windup and bumpless
 *          transfer
 *        - magnitude + slew-rate clamping on the actuator command
 *        - sticky fault_audit_bitmask (root cause of the last incidents)
 *
 * The whole 1 kHz path is binary32 arithmetic: no binary64, no extended
 * precision and no libm.  The slew limiter recovers the sign of its own
 * rounding residual through the TwoSum error-free transformation instead of
 * promoting to a wider type, so a single-precision FPU never calls into the
 * __aeabi_d* software library.
 *
 * Deterministic C99 runtime-assurance and authority-arbitration core:
 * hardware independent and unit tested by test_gatekeeper_standalone.c.
 * ---------------------------------------------------------------------------
 */

#ifndef LAYERED_GATEKEEPER_H
#define LAYERED_GATEKEEPER_H

#include <stdint.h>

#if defined(__FAST_MATH__)
#error "layered_gatekeeper requires strict IEEE-754 NaN/Inf semantics; do not compile with -ffast-math."
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Discrete operating regimes produced by the slow coarse classifier.        */
/* ------------------------------------------------------------------------- */
typedef enum {
    REGIME_NOMINAL            = 1, /* nominal tracking                        */
    REGIME_HIGH_LOAD_DAMPED   = 2, /* high load / static-friction rejection    */
    REGIME_BOUNDARY_COMPLIANT = 3, /* compliant unilateral contact            */
    REGIME_SAFE_FALLBACK      = 4  /* fixed-gain return toward the safe zero   */
} layered_regime_t;

/* ------------------------------------------------------------------------- */
/* Gatekeeper states.  These are the *observable* decisions of the 1 kHz     */
/* core and are logged by the field data recorder; gate != NORMAL is         */
/* always a deliberate, explainable decision, never a silent clamp.          */
/* ------------------------------------------------------------------------- */
typedef enum {
    GATE_NORMAL                = 0, /* AI regime accepted, bank-controller on  */
    GATE_HOLD_LATENCY          = 1, /* stale heartbeat or unusable fast input  */
                                    /* With valid fast sensors the fixed R4    */
                                    /* bank keeps the loop closed during the   */
                                    /* bounded grace; only a lost fast sensor  */
                                    /* holds the last command open-loop        */
    GATE_VETO_OOD_RESIDUAL     = 2, /* physics innovation too large: ABSTAIN    */
    GATE_VETO_LOW_CONFIDENCE   = 3, /* AI self-confidence too low: ABSTAIN     */
    GATE_VETO_SAFETY_ENVELOPE  = 4, /* state at/approaching hard safety limit  */
    GATE_SAFE_ZERO             = 5, /* native HOLD ran past its budget:        */
                                    /* command ramped to zero in steps of      */
                                    /* max(slew * dt, 1 binary32 ULP) so the   */
                                    /* de-energise always finishes; integrators*/
                                    /* cut                                     */
    GATE_LATCHED_SAFE          = 6  /* cumulative severe faults: permanently   */
                                    /* locked in R4 until an explicit external */
                                    /* acknowledgement re-arms the AI         */
} layered_gate_state_t;

/* ------------------------------------------------------------------------- */
/* Sticky fault audit bitmask: set once, never cleared, OR-ed across reboots */
/* if the platform persists it.  This is the "black box" that tells a field  */
/* engineer WHY the unit entered a safe state, without any AI introspection. */
/* ------------------------------------------------------------------------- */
#define LAYERED_FAULT_BIT_LATENCY_TIMEOUT  ((uint32_t)1u << 0)
#define LAYERED_FAULT_BIT_OOD_RESIDUAL     ((uint32_t)1u << 1)
#define LAYERED_FAULT_BIT_LOW_CONFIDENCE   ((uint32_t)1u << 2)
#define LAYERED_FAULT_BIT_ENVELOPE_BREACH  ((uint32_t)1u << 3)
#define LAYERED_FAULT_BIT_NAN_CORRUPTION   ((uint32_t)1u << 4)
#define LAYERED_FAULT_BIT_HOLD_TIMEOUT     ((uint32_t)1u << 5)
#define LAYERED_FAULT_BIT_PERMANENT_LATCH  ((uint32_t)1u << 6)

/* ------------------------------------------------------------------------- */
/* On-site field calibration: the ONLY three numbers a technician touches    */
/* when the same firmware is moved to a new device. There is                 */
/* no retraining and no dataset upload -- the AI stays frozen.               */
/* ------------------------------------------------------------------------- */
typedef struct {
    float gain_scale;          /* normal-bank P/I scale, [0.10, 4.0]           */
    float damping_scale;       /* normal-bank D scale, [0.10, 4.0]             */
    float safety_margin_scale; /* tightens the state envelope, [0.10, 1.0]     */
} FieldTuning_t;

#define LAYERED_NUM_REGIMES 4

/* ------------------------------------------------------------------------- */
/* Deployment constants (set once at boot). Positive: dt, timeout, residual   */
/* and state limits, magnitude, slew. Nonnegative: integral bound, anti-windup */
/* time and prediction horizon. Ratios: 0 <= safety_clear <= safety_trip <= 1, */
/* safety_trip > 0; residual_clear in [0,1]; 0 <= confidence_trip <= clear <= 1.*/
/* The binary32 product u_slew_max_per_s * dt_s must stay positive and finite:  */
/* a product that underflows to zero cannot be honoured by the SAFE_ZERO        */
/* quantisation floor and one that overflows makes every command reachable in   */
/* a single tick, so both are rejected as corrupt configurations.               */
/* ------------------------------------------------------------------------- */
typedef struct {
    float dt_s;                 /* inner-loop period, 0.001 s = 1 kHz          */
    float outer_timeout_s;      /* watchdog trip age of slow-loop heartbeat    */

    float max_physics_residual; /* residual threshold (normalized, 1.0 = trip) */
    float residual_clear_ratio; /* hysteresis: clear veto below ratio*thresh   */

    float min_ai_confidence;    /* confidence veto threshold, in [0,1]         */
    float confidence_clear_ratio; /* ABSOLUTE confidence level (0..1) at/above */
                                /* which the sticky confidence veto is cleared */
                                /*  (e.g. 0.55 when min_ai_confidence is 0.45) */

    float state_safety_limit;   /* absolute hard limit on |x|                  */
    float safety_trip_ratio;    /* trip when |x| > ratio*limit (e.g. 0.90)     */
    float safety_clear_ratio;   /* clear when |x| < ratio*limit (e.g. 0.75)    */
    float safety_predict_s;     /* look-ahead horizon: trip on |x + T*x_dot|   */

    float u_max;                /* hard actuator magnitude limit               */
    float u_slew_max_per_s;     /* hard actuator slew-rate limit               */
    float blend_alpha;          /* per-step transfer coefficient, (0,1]        */
    float integ_limit;          /* absolute clamp on every integral state      */
    float windup_tau_s;         /* tracking anti-windup time constant          */

    uint32_t min_dwell_ticks;   /* anti-chatter dwell on NORMAL regime changes */
    uint32_t ood_recovery_cooldown_ticks; /* clean ticks before AI re-armed    */

    float kp[LAYERED_NUM_REGIMES]; /* controller bank: index = regime - 1      */
    float kd[LAYERED_NUM_REGIMES];
    float ki[LAYERED_NUM_REGIMES];
    float ff_scale[LAYERED_NUM_REGIMES]; /* normal-bank feedforward scaling;    */
                                         /* index 3 is reserved, R4 ignores it  */

    uint32_t max_hold_ticks;    /* maximum consecutive native HOLD ticks;      */
                                /* beyond it the core forces GATE_SAFE_ZERO;   */
                                /* must be > 0 and < UINT32_MAX, otherwise the */
                                /* configuration is invalid (the counter       */
                                /* saturates, so UINT32_MAX never times out)   */
    uint32_t max_fault_vetoes_before_latch; /* safety+OOD veto EPISODES (since  */
                                /* Layered_Init or the last acknowledgement)   */
                                /* that latch GATE_LATCHED_SAFE; 0 disables    */
                                /* the cumulative permanent latch              */
} LayeredConfig_t;

/* ------------------------------------------------------------------------- */
/* One 1 kHz sample: plant measurement + freshest slow-loop mailbox entry.   */
/* ------------------------------------------------------------------------- */
typedef struct {
    float ai_regime;        /* 1..3 from the coarse classifier (4 = abstain)  */
    float ai_confidence;    /* classifier confidence in [0,1]                 */
    float outer_age_s;      /* age of the freshest slow-loop frame            */
    float physics_residual; /* normalized nominal-physics innovation envelope */
    float x;                /* measured state                                 */
    float x_dot;            /* measured state derivative                      */
    float r;                /* reference state                                */
    float r_dot;            /* reference state derivative                     */
    float u_ff;             /* deterministic feedforward command produced by  */
                            /* the commissioning model / trajectory generator */
                            /* (0.0 when the plant has no feedforward term)   */
    uint32_t outer_seq;     /* slow-frame sequence number; 0 = not sequenced, */
                            /* outer_age_s is authoritative                   */
} LayeredInput_t;

/* ------------------------------------------------------------------------- */
/* One 1 kHz sample of core output (what the actuator and the logger see).   */
/* ------------------------------------------------------------------------- */
typedef struct {
    float u;                     /* command actually applied this step          */
    float u_target;              /* pre-clamp target (for diagnostics only)     */
    layered_gate_state_t gate;   /* gate decision for this step                 */
    layered_regime_t regime;     /* regime actually used by the controller bank */
    float blend;                 /* bumpless-transfer progress in [0,1]         */
    uint32_t fault_audit_bitmask;/* sticky snapshot of all root-cause flags     */
    int corrupted;               /* 1 = invalid fast input/settings or mailbox */
} LayeredOutput_t;

/* ------------------------------------------------------------------------- */
/* Persistent core state (statically allocatable, no heap).                  */
/* ------------------------------------------------------------------------- */
typedef struct {
    int initialized;
    float u_applied;             /* previous actuator command                   */
    float u_target;              /* previous pre-clamp target                   */
    float integ[LAYERED_NUM_REGIMES]; /* one integrator per bank controller     */
    float blend;                 /* bumpless-transfer progress in [0,1]         */
    int control_started;         /* 0 until the first clean control step        */
    layered_regime_t regime;     /* last effective regime                       */
    uint8_t ood_latched;         /* sticky OOD veto                             */
    uint8_t safety_latched;      /* sticky safety-envelope veto                 */
    uint8_t conf_latched;        /* sticky low-confidence veto                  */
    uint8_t permanent_latched;   /* cumulative-fault latch, cleared by the API  */
    uint32_t dwell_ticks;        /* ticks since the last normal regime change   */
    uint32_t recovery_ticks;     /* remaining clean ticks in recovery cooldown  */
    uint32_t fault_audit_bitmask;/* sticky root-cause flags                     */
    uint32_t step_count;
    uint32_t safety_veto_count;
    uint32_t ood_veto_count;
    uint32_t low_confidence_veto_count;
    uint32_t latency_hold_count;
    uint32_t corrupted_frame_count;
    uint32_t hold_ticks;         /* consecutive ticks in GATE_HOLD_LATENCY      */
    uint32_t last_outer_seq;     /* last accepted slow-frame sequence number    */
    uint32_t outer_age_ticks;    /* locally measured slow-frame age, in ticks   */
    uint32_t latch_episode_count;/* new safety+OOD veto episodes since          */
                                 /* Layered_Init or the last acknowledgement;   */
                                 /* drives the cumulative permanent latch       */
} LayeredState_t;

/* ------------------------------------------------------------------------- */
/* API                                                                       */
/* ------------------------------------------------------------------------- */

/* Zeroes the state and puts the core in a defined, safe starting condition.
 * Safe starting condition = NOMINAL regime, no latches, no fault bits,
 * u = 0, blend = 0, control_started = 0, recovery cooldown expired, dwell
 * counter = 0. */
void Layered_Init(LayeredState_t *state,
                  const LayeredConfig_t *config,
                  const FieldTuning_t *tuning);

/* Executes exactly one 1 kHz step and returns the command actually applied.
 * The function is O(1): fixed-size checks and controller arithmetic, no
 * allocation, no library calls.
 *
 * 0. SPLIT INPUT FIREWALL
 *    Invalid configuration or nonfinite fast inputs (x, x_dot, r,
 *    r_dot, u_ff, physics_residual) hold the last command without advancing
 *    controller state. Negative residual is invalid; signed state/reference
 *    and command quantities are valid.
 *    Invalid tuning is replaced locally by (1,1,1) for every bank and the
 *    state envelope, and forces closed-loop R4. Caller settings are unchanged.
 *    Invalid slow mailbox fields (nonfinite values, regime outside [1,4],
 *    confidence outside [0,1], negative age) force explicit R4 abstention.
 *    Valid sensors still run the fast closed loop, including envelope/OOD
 *    priority. Both paths set corrupted=1, latch NAN_CORRUPTION, increment
 *    corrupted_frame_count and re-arm recovery cooldown.
 *    ai_regime == 4 is *not* corruption: it is an explicit AI abstention and
 *    is handled as GATE_VETO_LOW_CONFIDENCE.
 *    A native HOLD counts consecutive held ticks in state->hold_ticks. The
 *    corrupt-frame path (invalid fast sensor, or invalid configuration with its
 *    fixed LAYERED_CORRUPT_CONFIG_MAX_HOLD_TICKS = 100 budget) has no trustworthy
 *    feedback and therefore holds the last command open-loop; past the budget it
 *    escalates to GATE_SAFE_ZERO: target and integrators are cleared, the
 *    command is ramped to zero under the commissioned slew limit, one binary32
 *    ULP at a time when that budget is below one ULP of the command (immediately
 *    zeroed when the slew configuration itself is untrusted), and
 *    HOLD_TIMEOUT plus PERMANENT_LATCH latch. A stale heartbeat with healthy
 *    fast sensors (section d) uses config->max_hold_ticks, keeps the measured
 *    loop closed on the fixed R4 bank, and latches HOLD_TIMEOUT only.
 *
 * 1. DECISION PRIORITY (highest first, evaluated every clean step):
 *      a) GATE_VETO_SAFETY_ENVELOPE : |x| > safety_trip_ratio*limit_eff, or
 *         |x + safety_predict_s*x_dot| > limit_eff.  The prediction projects
 *         the measured state ALONG its own velocity, so a state that is inside
 *         the band and retreating toward zero is not an envelope approach; the
 *         instantaneous guard keeps its direction-blind contract.  Sticky: it
 *         clears only when BOTH |x| and the signed prediction are below
 *         safety_clear_ratio*limit.
 *         -> latches LAYERED_FAULT_BIT_ENVELOPE_BREACH.
 *      b) GATE_VETO_OOD_RESIDUAL    : residual above threshold. Sticky until
 *         it recedes below residual_clear_ratio*threshold.
 *         -> latches LAYERED_FAULT_BIT_OOD_RESIDUAL.
 *      c) GATE_VETO_LOW_CONFIDENCE  : confidence below min_ai_confidence, or
 *         an explicit ai_regime == 4 abstention.  Sticky: the latch clears
 *         only once confidence rises to confidence_clear_ratio AND the
 *         classifier reports a normal regime again.
 *         -> latches LAYERED_FAULT_BIT_LOW_CONFIDENCE.
 *      d) GATE_HOLD_LATENCY         : heartbeat older than outer_timeout_s.
 *         With healthy fast sensors the fixed R4 bank keeps the loop closed on
 *         the measured state: freezing a torque while valid feedback exists is
 *         not a safe response to a lost 10 Hz producer.  The gate and the
 *         LATENCY_TIMEOUT audit bit still report the timeout, the blend filter
 *         is bypassed so the damping law carries no first-order lag, and the
 *         grace is bounded by max_hold_ticks.  An unusable FAST sensor is the
 *         only case with no feedback at all; that path (section 0) holds the
 *         last applied command open-loop.  Neither case holds position.
 *         -> latches LAYERED_FAULT_BIT_LATENCY_TIMEOUT.
 *         After max_hold_ticks consecutive HOLD ticks the decision escalates
 *         to GATE_SAFE_ZERO: u_target = 0, all integrators cleared, the
 *         command ramped to zero under the commissioned slew rate, and
 *         HOLD_TIMEOUT latches (a heartbeat-only timeout never arms the
 *         permanent latch). No controller runs while the fault persists.
 *         QUANTISATION CONTRACT: the commanded change per tick is
 *         max(u_slew_max_per_s * dt_s, 1 binary32 ULP(|u_prev|)), and the
 *         one-ULP floor applies only when the slew budget is positive.  A legal
 *         budget smaller than half an ULP of the remaining command would
 *         otherwise round back to the same binary32 value and the actuator could
 *         stay non-zero forever, so the floor trades a bounded, one-directional
 *         quantisation overshoot for a guaranteed finite-time de-energise.  The
 *         core rejects any configuration whose binary32 slew*dt product
 *         underflows to 0 or overflows to infinity, so the floor is never the
 *         whole budget.  This is the only path where the applied command may
 *         change by more than the configured slew budget.
 *      e) GATE_NORMAL               : AI regime accepted.
 *    A cumulative permanent latch (max_fault_vetoes_before_latch > 0) beats
 *    all of the above: once the configured number of safety/OOD veto EPISODES
 *    has accumulated, the gate reports GATE_LATCHED_SAFE, the regime is
 *    REGIME_SAFE_FALLBACK and only Layered_AcknowledgeFaultLatch() releases
 *    the core. Cleared residual/envelope conditions or an expired recovery
 *    cooldown do not release it.
 *    All vetoes are emergency transitions and bypass the dwell timer.
 *
 * 2. ANTI-CHATTER DWELL TIME
 *    A transition between two *normal* regimes (1,2,3) is only applied after
 *    the current regime has been held for min_dwell_ticks samples.  The
 *    counter counts the current tick as well: the earliest switch is exactly
 *    min_dwell_ticks ticks after the previous switch.  Before that the
 *    previous regime is kept (the gate stays GATE_NORMAL).  This is what
 *    stops 1->2->1->2 mode ping-ponging on a noisy boundary.
 *
 * 3. POST-FAULT RECOVERY COOLDOWN
 *    Any veto, native latency hold or corrupt frame re-arms
 *    ood_recovery_cooldown_ticks.  After the fault clears the core keeps
 *    REGIME_SAFE_FALLBACK (gate GATE_NORMAL, no veto) until that many
 *    CONSECUTIVE clean samples have elapsed; only then is the AI regime
 *    switching re-enabled.  A single clean tick is never enough.
 *
 * 3b. LOCAL FRAME LEASE (outer_seq)
 *    With input->outer_seq != 0 the slow-frame age is measured by the L1 clock
 *    instead of the transported outer_age_s: a changed sequence number opens a
 *    fresh lease (outer_age_ticks = 0), an unchanged one ages it by one inner
 *    period. A forged or frozen outer_age_s therefore cannot mask a stalled
 *    mailbox.  If the sequence was active and suddenly reads 0, the mailbox is
 *    treated as anomalous and the core runs closed-loop R4.  With outer_seq
 *    always 0 the legacy outer_age_s mode is used unchanged.
 *
 * 3c. CUMULATIVE SEVERE-FAULT LATCH
 *    When max_fault_vetoes_before_latch > 0, the safety/OOD veto EPISODE that
 *    brings state->latch_episode_count to the configured threshold sets
 *    permanent_latched and latches PERMANENT_LATCH.  latch_episode_count
 *    counts only episodes since Layered_Init or the last successful
 *    Layered_AcknowledgeFaultLatch, so an acknowledgement restores the full
 *    tolerance.  safety_veto_count / ood_veto_count remain lifetime audit
 *    history and are never reset by the latch API; they therefore cannot
 *    re-arm the latch by themselves.
 *
 * 4. CONTROLLER BANK + ANTI-WINDUP + BUMPLESS TRANSFER
 *    u_pid[i] = kp[i]*e + kd[i]*ed + ki[i]*integ[i] + ff_scale[i]*u_ff
 *    for normal banks i=0..2, with gains scaled by FieldTuning_t.
 *    R4 uses kp[3]*(-x) + kd[3]*(-x_dot) + ki[3]*integ[3], fixed gains,
 *    zero reference and no feedforward. Its anti-windup uses that same law
 *    and error -x. The field knobs never scale R4 gains.
 *    u_ff is the deterministic feedforward command produced
 *    by the commissioning model / trajectory generator, so the plant-inverse
 *    term stays white-box and is never produced by the AI.  ff_scale[i] is a
 *    per-bank commissioned gain on that same white-box term; it is not an AI
 *    output and does not change the feedforward's deterministic origin.
 *    EVERY controller in the bank owns an integrator and EVERY integrator is
 *    continuously back-calculated toward the command that was really applied
 *    (tracking anti-windup with windup_tau_s), so an inactive controller can
 *    never accumulate hidden windup and a later switch releases no energy.
 *    The ACTIVE bank's integrator is back-calculated toward the command after
 *    the magnitude and slew limits but BEFORE the blend filter: a first-order
 *    transfer lag is not actuator saturation and must not silently detune the
 *    running bank.  The inactive banks keep tracking the final applied command
 *    so a later switch is bumpless.
 *    On the first clean control step the pre-clamp target snaps to the active
 *    bank output and blend = 1, so the command does not ramp up from zero.
 *    On later steps it follows u_pid[i] through a first-order filter with
 *    coefficient blend_alpha, and a bank switch restarts blend at 0.
 *    EMERGENCY BYPASS: while a veto is active (or the permanent latch is set or
 *    the slow mailbox is stale) and the active regime is REGIME_SAFE_FALLBACK,
 *    the blend filter is bypassed: the target is the R4 bank output directly
 *    and blend = 1.  This removes the first-order phase lag from every
 *    emergency damping response.  The magnitude and slew limits below still
 *    apply to every command.
 *    Finally the command is hard clamped in magnitude and slew rate.
 *    SLEW CONTRACT: on every normal control tick |u(k) - u(k-1)| is bounded by
 *    u_slew_max_per_s * dt_s.  Both bound computations and the limiter itself
 *    stay in binary32: TwoSum recovers the exact sign of the rounding residual
 *    so a bound that nearest-even rounding placed outside the budget is pulled
 *    one representable value inward.  The single documented exception is the
 *    GATE_SAFE_ZERO de-energise above, whose per-tick step is
 *    max(u_slew_max_per_s * dt_s, 1 binary32 ULP(|u_prev|)) so that a legal yet
 *    sub-ULP budget still reaches exactly 0.0f in finite time.  It only ever
 *    reduces |u|, never re-energises, and never applies when the budget is 0.
 */
LayeredOutput_t Layered_Step(LayeredState_t *state,
                             const LayeredConfig_t *config,
                             const FieldTuning_t *tuning,
                             const LayeredInput_t *input);

/* External acknowledgement of a permanent safe latch.
 *
 * Checks the pointers and the configuration (same validity rules as
 * Layered_Step) and, on success, clears the three independent veto latches
 * (envelope, residual, low confidence) plus permanent_latched, hold_ticks and
 * latch_episode_count, and re-arms the recovery cooldown
 * (ood_recovery_cooldown_ticks) so the AI cannot be re-armed on the
 * very next tick.  fault_audit_bitmask, step_count and all *_veto_count
 * counters are AUDIT HISTORY and are never cleared here; resetting the
 * episode counter is what gives the unit its full max_fault_vetoes_before_latch
 * tolerance after a field acknowledgement.
 *
 * Returns 1 when the latch was released, 0 for a null/corrupt argument. */
int Layered_AcknowledgeFaultLatch(LayeredState_t *state,
                                  const LayeredConfig_t *config);

/* Human-readable names for logs / test harness (never NULL). */
const char *Layered_GateName(layered_gate_state_t gate);
const char *Layered_RegimeName(layered_regime_t regime);

#ifdef __cplusplus
}
#endif

#endif /* LAYERED_GATEKEEPER_H */
