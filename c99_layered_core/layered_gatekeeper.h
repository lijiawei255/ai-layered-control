/*
 * layered_gatekeeper.h
 * ---------------------------------------------------------------------------
 * Zero-dependency ANSI C99 reference core for the layered AI control
 * architecture ("slow coarse AI + fast deterministic core").
 *
 * The whole point of this module is that it is boring to deploy and boring to
 * certify:
 *   - no floating-point library functions, no dynamic memory (malloc-free),
 *     no OS calls, no recursion -> strict O(1) Worst-Case Execution Time;
 *   - the only include is <stdint.h>, so it compiles on
 *     gcc / clang / arm-none-eabi-gcc / Keil MDK / IAR without a single flag
 *     beyond -std=c99 -pedantic -Wall -Wextra ;
 *   - fixed, statically-sized state (LayeredState_t), safe for 1 kHz ISRs;
 *   - sticky black-box fault audit bitmask for post-incident forensics
 *     with explicit gate decisions.
 *
 * Architecture (see README.md for the full picture):
 *
 *   slow outer loop (10 Hz, any Linux/Android/edge accelerator)
 *        |  mailbox: regime in {1,2,3}, confidence in [0,1], heartbeat
 *        v
 *   THIS MODULE (1 kHz, deterministic, 88 B state + 132 B config + 12 B tuning)
 *        - fast-input firewall; corrupt slow mailboxes request closed-loop R4
 *        - heartbeat watchdog: survives 10 Hz jitter / dropped frames
 *        - physics-residual + confidence veto: explicit ABSTENTION
 *        - predictive state-envelope veto: selects fixed-gain R4
 *        - anti-chatter minimum dwell time on normal regime transitions
 *        - post-fault recovery cooldown before AI is re-armed
 *        - controller bank with per-controller anti-windup and bumpless
 *          transfer
 *        - magnitude + slew-rate clamping on the actuator command
 *        - sticky fault_audit_bitmask (root cause of the last incidents)
 *
 * Purely a reference implementation for the benchmark in ../matlab_simulink/:
 * it is hardware independent and unit tested by test_gatekeeper_standalone.c.
 * ---------------------------------------------------------------------------
 */

#ifndef LAYERED_GATEKEEPER_H
#define LAYERED_GATEKEEPER_H

#include <stdint.h>

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
    GATE_VETO_OOD_RESIDUAL     = 2, /* physics innovation too large: ABSTAIN    */
    GATE_VETO_LOW_CONFIDENCE   = 3, /* AI self-confidence too low: ABSTAIN     */
    GATE_VETO_SAFETY_ENVELOPE  = 4  /* state at/approaching hard safety limit  */
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
    float safety_predict_s;     /* look-ahead horizon used with |x_dot|        */

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
    int ood_latched;             /* sticky OOD veto                             */
    int safety_latched;          /* sticky safety-envelope veto                 */
    int conf_latched;            /* sticky low-confidence veto                  */
    uint32_t dwell_ticks;        /* ticks since the last normal regime change   */
    uint32_t recovery_ticks;     /* remaining clean ticks in recovery cooldown  */
    uint32_t fault_audit_bitmask;/* sticky root-cause flags                     */
    uint32_t step_count;
    uint32_t safety_veto_count;
    uint32_t ood_veto_count;
    uint32_t low_confidence_veto_count;
    uint32_t latency_hold_count;
    uint32_t corrupted_frame_count;
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
 *    Invalid configuration/tuning or nonfinite fast inputs (x, x_dot, r,
 *    r_dot, u_ff, physics_residual) hold the last command without advancing
 *    controller state. Negative residual is invalid; signed state/reference
 *    and command quantities are valid.
 *    Invalid slow mailbox fields (nonfinite values, regime outside [1,4],
 *    confidence outside [0,1], negative age) force explicit R4 abstention.
 *    Valid sensors still run the fast closed loop, including envelope/OOD
 *    priority. Both paths set corrupted=1, latch NAN_CORRUPTION, increment
 *    corrupted_frame_count and re-arm recovery cooldown.
 *    ai_regime == 4 is *not* corruption: it is an explicit AI abstention and
 *    is handled as GATE_VETO_LOW_CONFIDENCE.
 *
 * 1. DECISION PRIORITY (highest first, evaluated every clean step):
 *      a) GATE_VETO_SAFETY_ENVELOPE : |x| > safety_trip_ratio*limit_eff, or
 *         |x| + safety_predict_s*|x_dot| > limit_eff.  Sticky: it clears only
 *         when BOTH |x| and the prediction are below safety_clear_ratio*limit.
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
 *         The last *applied* command is held; this does not hold position.
 *         Integrators/target/blend hold; dwell advances and recovery re-arms.
 *         -> latches LAYERED_FAULT_BIT_LATENCY_TIMEOUT.
 *      e) GATE_NORMAL               : AI regime accepted.
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
 *    continuously back-calculated toward the ACTUAL clamped actuator output
 *    (tracking anti-windup with windup_tau_s), so an inactive controller can
 *    never accumulate hidden windup and a later switch releases no energy.
 *    On the first clean control step the pre-clamp target snaps to the active
 *    bank output and blend = 1, so the command does not ramp up from zero.
 *    On later steps it follows u_pid[i] through a first-order filter with
 *    coefficient blend_alpha, and a bank switch restarts blend at 0.  Finally
 *    the command is hard clamped in magnitude and slew rate.
 */
LayeredOutput_t Layered_Step(LayeredState_t *state,
                             const LayeredConfig_t *config,
                             const FieldTuning_t *tuning,
                             const LayeredInput_t *input);

/* Human-readable names for logs / test harness (never NULL). */
const char *Layered_GateName(layered_gate_state_t gate);
const char *Layered_RegimeName(layered_regime_t regime);

#ifdef __cplusplus
}
#endif

#endif /* LAYERED_GATEKEEPER_H */
