// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jiawei Li
// Source: https://github.com/lijiawei255/ai-layered-control
// License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
// Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
/*
 * layered_gatekeeper.c
 * ---------------------------------------------------------------------------
 * Zero-dependency ANSI C99 implementation of the deterministic 1 kHz core
 * declared in layered_gatekeeper.h.
 *
 * Deployment constraints honoured on purpose (this file must be boring to
 * certify and safe to drop into a 1 kHz ISR):
 *   - the only include is the module header (which pulls in <stdint.h>);
 *   - no <math.h> / libm: |.|, clamping and the whole decision logic are hand
 *     written, no exp()/sqrt()/trig is ever needed;
 *   - no allocation, no I/O, no recursion: fixed-size checks and controller
 *     arithmetic bound the work per tick; target WCET requires measurement;
 *   - no file-static mutable state: every byte of persistence lives in
 *     LayeredState_t, so several instances can run side by side.
 *
 * One step, in this order:
 *   0. split firewall             bad fast/config holds; bad mailbox/tuning runs R4
 *   1. safety envelope veto       sticky, latches ENVELOPE_BREACH
 *   2. OOD residual veto          sticky, latches OOD_RESIDUAL
 *   3. confidence / abstention    sticky latch, latches LOW_CONFIDENCE
 *   3b. local frame lease         outer_seq ages the slow frame on the L1 clock
 *   3c. cumulative fault latch    safety/OOD episode count -> permanent R4
 *   4. heartbeat watchdog         bounded hold, then mandatory safe zero
 *   5. dwell timer + post-fault recovery cooldown for the normal AI regime
 *   6. controller bank (+ deterministic feedforward u_ff), tracking
 *      anti-windup, emergency R4 blend bypass, hard clamps
 * ---------------------------------------------------------------------------
 */

#include "layered_gatekeeper.h"

/* Magnitudes above 3e38 are rejected along with NaN/Inf. The comparison-only
 * firewall leaves headroom below the float32 representable maximum. */
#define LAYERED_FINITE_LIMIT 3.0e38f

/* A corrupt configuration has no trustworthy hold budget or slew rate.  The
 * core therefore keeps the last command for at most this many consecutive
 * rejected ticks and then de-energises immediately (no slew ramp, because the
 * configured slew limit itself cannot be trusted). */
#define LAYERED_CORRUPT_CONFIG_MAX_HOLD_TICKS ((uint32_t)100u)

/* Build-time guards: the straight-line bank code below is written out for
 * exactly LAYERED_NUM_REGIMES controllers, and the byte-exact ABI is part of
 * the device contract (the Python/ctypes mirror and the field logger depend on
 * these sizes).  If a structure ever changes, the build fails instead of
 * silently shifting an offset.  C99 has no _Static_assert, so negative-size
 * typedefs are used. */
typedef char lg_bank_size_check[(LAYERED_NUM_REGIMES == 4) ? 1 : -1];
typedef char lg_state_size_check[(sizeof(LayeredState_t) == 96) ? 1 : -1];
typedef char lg_config_size_check[(sizeof(LayeredConfig_t) == 140) ? 1 : -1];
typedef char lg_tuning_size_check[(sizeof(FieldTuning_t) == 12) ? 1 : -1];
typedef char lg_input_size_check[(sizeof(LayeredInput_t) == 40) ? 1 : -1];
typedef char lg_output_size_check[(sizeof(LayeredOutput_t) == 28) ? 1 : -1];

/* ========================================================================= */
/* Dependency-free float helpers                                             */
/* ========================================================================= */

static float lg_absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

static float lg_clampf(float v, float lo, float hi)
{
    if (v != v) {
        v = 0.0f;
    }
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/* NaN fails both comparisons, +Inf / -Inf fail one of them: no <math.h>. */
static int lg_is_finite(float v)
{
    return (v >= -LAYERED_FINITE_LIMIT) && (v <= LAYERED_FINITE_LIMIT);
}

static uint32_t lg_inc_sat(uint32_t value)
{
    return (value < UINT32_MAX) ? value + 1u : value;
}

static int lg_config_is_corrupt(const LayeredConfig_t *c)
{
    int i;
    if (!lg_is_finite(c->dt_s) || !lg_is_finite(c->outer_timeout_s) ||
        !lg_is_finite(c->max_physics_residual) || !lg_is_finite(c->residual_clear_ratio) ||
        !lg_is_finite(c->min_ai_confidence) || !lg_is_finite(c->confidence_clear_ratio) ||
        !lg_is_finite(c->state_safety_limit) || !lg_is_finite(c->safety_trip_ratio) ||
        !lg_is_finite(c->safety_clear_ratio) || !lg_is_finite(c->safety_predict_s) ||
        !lg_is_finite(c->u_max) || !lg_is_finite(c->u_slew_max_per_s) ||
        !lg_is_finite(c->blend_alpha) || !lg_is_finite(c->integ_limit) ||
        !lg_is_finite(c->windup_tau_s)) {
        return 1;
    }
    for (i = 0; i < LAYERED_NUM_REGIMES; ++i) {
        if (!lg_is_finite(c->kp[i]) || !lg_is_finite(c->kd[i]) ||
            !lg_is_finite(c->ki[i]) || !lg_is_finite(c->ff_scale[i])) {
            return 1;
        }
    }
    if (c->dt_s <= 0.0f || c->outer_timeout_s <= 0.0f ||
        c->max_physics_residual <= 0.0f || c->state_safety_limit <= 0.0f ||
        c->u_max <= 0.0f || c->u_slew_max_per_s <= 0.0f ||
        c->blend_alpha <= 0.0f || c->blend_alpha > 1.0f ||
        c->integ_limit < 0.0f || c->windup_tau_s < 0.0f || c->safety_predict_s < 0.0f ||
        c->max_hold_ticks == 0u || c->max_hold_ticks == UINT32_MAX ||
        c->residual_clear_ratio < 0.0f || c->residual_clear_ratio > 1.0f ||
        c->min_ai_confidence < 0.0f || c->min_ai_confidence > 1.0f ||
        c->confidence_clear_ratio < c->min_ai_confidence || c->confidence_clear_ratio > 1.0f ||
        c->safety_trip_ratio <= 0.0f || c->safety_trip_ratio > 1.0f ||
        c->safety_clear_ratio < 0.0f || c->safety_clear_ratio > c->safety_trip_ratio) {
        return 1;
    }
    return 0;
}

static int lg_tuning_is_corrupt(const FieldTuning_t *t)
{
    return !lg_is_finite(t->gain_scale) || !lg_is_finite(t->damping_scale) ||
           !lg_is_finite(t->safety_margin_scale) ||
           t->gain_scale < 0.10f || t->gain_scale > 4.0f ||
           t->damping_scale < 0.10f || t->damping_scale > 4.0f ||
           t->safety_margin_scale < 0.10f || t->safety_margin_scale > 1.0f;
}

/* ========================================================================= */
/* Mailbox decoding                                                          */
/* ========================================================================= */

/* The firewall has already guaranteed 1.0 <= ai_regime <= 4.0.
 * 4.0 is an explicit abstention and maps to the safe fallback bank. */
static layered_regime_t lg_mailbox_regime(float ai_regime)
{
    if (ai_regime < 1.5f) {
        return REGIME_NOMINAL;
    }
    if (ai_regime < 2.5f) {
        return REGIME_HIGH_LOAD_DAMPED;
    }
    if (ai_regime < 3.5f) {
        return REGIME_BOUNDARY_COMPLIANT;
    }
    return REGIME_SAFE_FALLBACK;
}

static int lg_mailbox_abstains(float ai_regime)
{
    return (ai_regime >= 3.5f);
}

/* ========================================================================= */
/* Input firewall                                                            */
/* ========================================================================= */

/* Fast measurements must remain usable before any controller can execute.
 * Signed state, reference and actuator quantities are valid; residual is
 * the only nonnegative fast input. */
static int lg_fast_input_is_corrupt(const LayeredInput_t *input)
{
    if (!lg_is_finite(input->physics_residual) ||
        !lg_is_finite(input->x) ||
        !lg_is_finite(input->x_dot) ||
        !lg_is_finite(input->r) ||
        !lg_is_finite(input->r_dot) ||
        !lg_is_finite(input->u_ff)) {
        return 1;
    }
    return input->physics_residual < 0.0f;
}

/* Slow mailbox corruption requests closed-loop fallback, never a hold. */
static int lg_mailbox_is_corrupt(const LayeredInput_t *input)
{
    if (!lg_is_finite(input->ai_regime) ||
        !lg_is_finite(input->ai_confidence) ||
        !lg_is_finite(input->outer_age_s)) {
        return 1;
    }
    if (input->ai_regime < 1.0f || input->ai_regime > 4.0f) {
        return 1;
    }
    if (input->ai_confidence < 0.0f || input->ai_confidence > 1.0f) {
        return 1;
    }
    if (input->outer_age_s < 0.0f) {
        return 1;
    }
    return 0;
}

/* ========================================================================= */
/* Controller bank helpers                                                   */
/* ========================================================================= */

/* Normal banks: kp[i]*e + kd[i]*ed + ki[i]*integ[i] + ff_scale[i]*u_ff, with the two
 * field-tuning knobs applied to the gains (gain_scale scales P and I,
 * damping_scale D).  u_ff is the deterministic white-box feedforward produced
 * by the commissioning model / trajectory generator. R4 has its own zero-state
 * error, fixed gains and no feedforward; its integral tracks that same law. */
static float lg_pid_output(const LayeredConfig_t *config,
                           const FieldTuning_t *tuning,
                           int bank,
                           float e,
                           float ed,
                           float integ,
                           float u_ff)
{
    if (bank == 3) {
        return config->kp[3] * e + config->kd[3] * ed + config->ki[3] * integ;
    }
    return (config->kp[bank] * tuning->gain_scale) * e
         + (config->kd[bank] * tuning->damping_scale) * ed
         + (config->ki[bank] * tuning->gain_scale) * integ
         + config->ff_scale[bank] * u_ff;
}

/* Tracking anti-windup (back-calculation) for ONE integrator.
 *
 *   integ += dt*e + (u_actual - u_pid) * dt / (ki_eff * windup_tau_s)
 *
 * Every integrator of the bank follows the command that was really applied, so
 * an inactive controller can never store hidden windup and a later switch
 * releases no energy.  When ki_eff == 0 (or the time constant is disabled) the
 * back-calculation term has no defined gain and is skipped: the integrator is
 * then a pure accumulator and still clamped. */
static void lg_integrate(float *integ,
                         float ki_eff,
                         float e,
                         float dt,
                         float u_pid,
                         float u_actual,
                         float windup_tau,
                         float limit)
{
    float next = *integ + e * dt;

    if (ki_eff != 0.0f && windup_tau > 0.0f) {
        next += (u_actual - u_pid) * dt / (ki_eff * windup_tau);
    }
    *integ = lg_clampf(next, -limit, limit);
}

/* ========================================================================= */
/* API                                                                       */
/* ========================================================================= */

void Layered_Init(LayeredState_t *state,
                  const LayeredConfig_t *config,
                  const FieldTuning_t *tuning)
{
    (void)config; /* the core never copies its configuration */
    (void)tuning;

    if (state == 0) {
        return;
    }

    state->initialized = 1;
    state->u_applied = 0.0f;
    state->u_target = 0.0f;
    state->integ[0] = 0.0f;
    state->integ[1] = 0.0f;
    state->integ[2] = 0.0f;
    state->integ[3] = 0.0f;
    state->blend = 0.0f;
    state->control_started = 0;
    state->regime = REGIME_NOMINAL;
    state->ood_latched = 0u;
    state->safety_latched = 0u;
    state->conf_latched = 0u;
    state->permanent_latched = 0u;
    state->dwell_ticks = 0u;
    state->recovery_ticks = 0u;
    state->fault_audit_bitmask = 0u;
    state->step_count = 0u;
    state->safety_veto_count = 0u;
    state->ood_veto_count = 0u;
    state->low_confidence_veto_count = 0u;
    state->latency_hold_count = 0u;
    state->corrupted_frame_count = 0u;
    state->hold_ticks = 0u;
    state->last_outer_seq = 0u;
    state->outer_age_ticks = 0u;
    state->latch_episode_count = 0u;
}

/* External release of the cumulative safe latch.  Historical audit data is
 * deliberately preserved so a post-incident review still sees why the unit
 * latched even after the field engineer acknowledged it. */
int Layered_AcknowledgeFaultLatch(LayeredState_t *state,
                                  const LayeredConfig_t *config)
{
    if (state == 0 || config == 0 || lg_config_is_corrupt(config)) {
        return 0;
    }
    state->permanent_latched = 0u;
    state->ood_latched = 0u;
    state->safety_latched = 0u;
    state->conf_latched = 0u;
    state->hold_ticks = 0u;
    /* The lifetime veto counters are audit history and stay untouched; only
     * the episode counter restarts, which restores the full fault tolerance. */
    state->latch_episode_count = 0u;
    state->recovery_ticks = config->ood_recovery_cooldown_ticks;
    return 1;
}

LayeredOutput_t Layered_Step(LayeredState_t *state,
                             const LayeredConfig_t *config,
                             const FieldTuning_t *tuning,
                             const LayeredInput_t *input)
{
    LayeredOutput_t out;
    const FieldTuning_t nominal_tuning = {1.0f, 1.0f, 1.0f};
    float u_max;
    float integ_limit;
    float du_max;
    float dt_eff;
    float windup_tau;
    float limit_eff;
    float x_abs;
    float xdot_abs;
    float predicted_abs;
    float residual_abs;
    float e;
    float ed;
    float u_pid0;
    float u_pid1;
    float u_pid2;
    float u_pid3;
    float u_active;
    float u_cmd;
    float outer_age_eff;
    int snap;
    int conf_before;
    int safety_before;
    int ood_before;
    int conf_veto;
    int latency_hold;
    int veto_active;
    int do_control;
    int bank;
    int tuning_corrupt;
    int outer_corrupt;
    int fast_input_bad;
    int config_bad;
    int seq_mailbox_bad;
    layered_gate_state_t gate;
    layered_regime_t regime;
    layered_regime_t mailbox_regime;

    /* Fail-safe answer for a broken caller (null pointer): no command, the
     * strictest gate reported.  Never dereferences a null pointer. */
    out.u = 0.0f;
    out.u_target = 0.0f;
    out.gate = GATE_VETO_SAFETY_ENVELOPE;
    out.regime = REGIME_SAFE_FALLBACK;
    out.blend = 0.0f;
    out.fault_audit_bitmask = 0u;
    out.corrupted = 0;

    if (state == 0 || config == 0 || tuning == 0 || input == 0) {
        return out;
    }

    /* Default initialization: when initialized != 1 the core runs the same
     * initialization inline and continues computing in this same tick, so a
     * forgotten Layered_Init() cannot produce a wild actuator command on the
     * first call.  The caller must still call Layered_Init at startup: that is
     * what makes the initial state deterministic instead of depending on
     * whatever bytes the struct happened to carry. */
    if (!state->initialized) {
        Layered_Init(state, config, tuning);
    }

    /* ---------------- 0. INPUT FIREWALL -------------------------------- */
    /* Invalid fast inputs/configuration stop here: no integrator, no
     * latch, no blend, no dwell counter is touched. The applied command
     * remains unchanged and the sticky NAN_CORRUPTION bit
     * records that this IPC frame must be investigated.  The recovery
     * cooldown is re-armed because a corrupt frame is not a clean sample; it
     * must never count towards "consecutive clean ticks".
     * A native HOLD is bounded.  A valid configuration uses max_hold_ticks;
     * an invalid configuration uses the fixed corrupt-config budget and is
     * then de-energised immediately, because its slew limit is untrusted.
     * Both timeout paths clear every integrator, force the target to zero,
     * latch HOLD_TIMEOUT | PERMANENT_LATCH and arm permanent_latched: a
     * sensor or configuration that stays broken must not be recovered
     * automatically. */
    fast_input_bad = lg_fast_input_is_corrupt(input);
    config_bad = lg_config_is_corrupt(config);
    if (fast_input_bad || config_bad) {
        uint32_t hold_budget;

        state->fault_audit_bitmask |= LAYERED_FAULT_BIT_NAN_CORRUPTION;
        state->corrupted_frame_count = lg_inc_sat(state->corrupted_frame_count);
        state->recovery_ticks = config->ood_recovery_cooldown_ticks;
        state->step_count = lg_inc_sat(state->step_count);

        /* The hold budget always advances.  An invalid configuration must not
         * hold the last command forever: that would leave a non-zero actuator
         * command alive with no trustworthy control law behind it. */
        hold_budget = config_bad ? LAYERED_CORRUPT_CONFIG_MAX_HOLD_TICKS
                                 : config->max_hold_ticks;
        state->hold_ticks = lg_inc_sat(state->hold_ticks);
        if (state->hold_ticks > hold_budget) {
            state->fault_audit_bitmask |= LAYERED_FAULT_BIT_HOLD_TIMEOUT |
                                          LAYERED_FAULT_BIT_PERMANENT_LATCH;
            state->permanent_latched = 1u;
            state->u_target = 0.0f;
            state->integ[0] = 0.0f;
            state->integ[1] = 0.0f;
            state->integ[2] = 0.0f;
            state->integ[3] = 0.0f;
            state->blend = 0.0f;
            state->regime = REGIME_SAFE_FALLBACK;
            if (config_bad) {
                /* The slew limit belongs to the rejected configuration. */
                state->u_applied = 0.0f;
            } else {
                /* Trusted settings: the de-energise must still respect
                 * u_slew_max_per_s, so walk the command to zero at the
                 * commissioned rate instead of stepping it in one tick. */
                float corrupt_du_max = lg_absf(config->u_slew_max_per_s) *
                                       lg_absf(config->dt_s);
                state->u_applied = lg_clampf(0.0f,
                                             state->u_applied - corrupt_du_max,
                                             state->u_applied + corrupt_du_max);
            }

            out.u = state->u_applied;
            out.u_target = 0.0f;
            out.gate = GATE_SAFE_ZERO;
            out.regime = REGIME_SAFE_FALLBACK;
            out.blend = state->blend;
            out.fault_audit_bitmask = state->fault_audit_bitmask;
            out.corrupted = 1;
            return out;
        }

        out.u = state->u_applied;
        out.u_target = state->u_target;
        out.gate = GATE_HOLD_LATENCY;
        out.regime = state->regime;
        out.blend = state->blend;
        out.fault_audit_bitmask = state->fault_audit_bitmask;
        out.corrupted = 1;
        return out;
    }
    tuning_corrupt = lg_tuning_is_corrupt(tuning);
    if (tuning_corrupt) {
        tuning = &nominal_tuning;
    }
    outer_corrupt = tuning_corrupt || lg_mailbox_is_corrupt(input);
    if (outer_corrupt) {
        state->fault_audit_bitmask |= LAYERED_FAULT_BIT_NAN_CORRUPTION;
        state->corrupted_frame_count = lg_inc_sat(state->corrupted_frame_count);
    }

    snap = (state->control_started == 0);
    conf_before = state->conf_latched;
    safety_before = state->safety_latched;
    ood_before = state->ood_latched;

    u_max = lg_absf(config->u_max);
    integ_limit = lg_absf(config->integ_limit);
    dt_eff = lg_absf(config->dt_s);
    du_max = lg_absf(config->u_slew_max_per_s) * dt_eff;
    windup_tau = config->windup_tau_s;

    /* ---------------- 0b. L1 slow-frame lease (outer_seq) --------------- */
    /* With a live sequence number the fast core times the slow frame itself.
     * A changed number is a fresh frame; an unchanged one ages the local lease
     * by one inner period, so a forged outer_age_s cannot hide a stalled
     * mailbox.  A sequence that was active and then reads 0 is a mailbox
     * anomaly and forces closed-loop R4. */
    outer_age_eff = input->outer_age_s;
    seq_mailbox_bad = 0;
    if (input->outer_seq != 0u) {
        if (state->last_outer_seq == 0u ||
            input->outer_seq != state->last_outer_seq) {
            state->last_outer_seq = input->outer_seq;
            state->outer_age_ticks = 0u;
        } else {
            state->outer_age_ticks = lg_inc_sat(state->outer_age_ticks);
        }
        outer_age_eff = (float)state->outer_age_ticks * dt_eff;
    } else if (state->last_outer_seq != 0u) {
        seq_mailbox_bad = 1;
    }
    /* A live sequence that falls back to zero is corruption of the same
     * mailbox, so it must be reported as corruption as well as forcing the
     * closed-loop R4 path.  The earlier mailbox check already counted this
     * frame when the transported fields were also invalid; only merge and
     * count when it did not. */
    if (seq_mailbox_bad != 0 && outer_corrupt == 0) {
        outer_corrupt = 1;
        state->fault_audit_bitmask |= LAYERED_FAULT_BIT_NAN_CORRUPTION;
        state->corrupted_frame_count = lg_inc_sat(state->corrupted_frame_count);
    }

    /* ---------------- 1. hard safety envelope -------------------------- */
    limit_eff = lg_absf(config->state_safety_limit * tuning->safety_margin_scale);
    x_abs = lg_absf(input->x);
    xdot_abs = lg_absf(input->x_dot);
    predicted_abs = x_abs + lg_absf(config->safety_predict_s) * xdot_abs;

    if (state->safety_latched) {
        float clear_level = config->safety_clear_ratio * limit_eff;
        /* Sticky until BOTH the measurement and the prediction recede. */
        if (x_abs < clear_level && predicted_abs < clear_level) {
            state->safety_latched = 0;
        }
    } else if (x_abs > config->safety_trip_ratio * limit_eff ||
               predicted_abs > limit_eff) {
        state->safety_latched = 1;
        state->safety_veto_count = lg_inc_sat(state->safety_veto_count);
        state->latch_episode_count = lg_inc_sat(state->latch_episode_count);
        state->fault_audit_bitmask |= LAYERED_FAULT_BIT_ENVELOPE_BREACH;
    }

    /* ---------------- 2. physics-residual (OOD) veto -------------------- */
    residual_abs = lg_absf(input->physics_residual);
    if (state->ood_latched) {
        if (residual_abs < config->residual_clear_ratio * config->max_physics_residual) {
            state->ood_latched = 0;
        }
    } else if (residual_abs > config->max_physics_residual) {
        state->ood_latched = 1;
        state->ood_veto_count = lg_inc_sat(state->ood_veto_count);
        state->latch_episode_count = lg_inc_sat(state->latch_episode_count);
        state->fault_audit_bitmask |= LAYERED_FAULT_BIT_OOD_RESIDUAL;
    }

    /* ---------------- 3. confidence / explicit abstention --------------- */
    /* STICKY latch, as the contract describes:
     *   - trip  : confidence < min_ai_confidence, or an explicit ai_regime == 4
     *             abstention (the AI says "I do not vouch for this frame");
     *   - clear : confidence >= confidence_clear_ratio (an ABSOLUTE level in
     *             [0,1], not a multiple) AND the classifier reports a normal
     *             regime again;
     *   - in between the previous decision is kept (hysteresis).
     * confidence_clear_ratio must therefore be chosen above min_ai_confidence,
     * e.g. min 0.45 / clear 0.55. */
    mailbox_regime = outer_corrupt ? REGIME_SAFE_FALLBACK : lg_mailbox_regime(input->ai_regime);
    if (outer_corrupt || seq_mailbox_bad != 0 ||
        input->ai_confidence < config->min_ai_confidence ||
        lg_mailbox_abstains(input->ai_regime) != 0) {
        state->conf_latched = 1;
        state->fault_audit_bitmask |= LAYERED_FAULT_BIT_LOW_CONFIDENCE;
    } else if (input->ai_confidence >= config->confidence_clear_ratio &&
               lg_mailbox_abstains(input->ai_regime) == 0) {
        state->conf_latched = 0;
    }
    conf_veto = state->conf_latched;
    if (conf_veto && !conf_before) {
        /* Diagnostics only: one count per veto EPISODE (rising edge of the
         * latch), never one per held tick. */
        state->low_confidence_veto_count = lg_inc_sat(state->low_confidence_veto_count);
    }

    /* ---------------- 3b. cumulative severe-fault latch ----------------- */
    /* Only a NEW safety/OOD veto episode can arm the permanent latch, and the
     * tolerance is counted on episodes since the last acknowledgement.  The
     * lifetime veto counters survive Layered_AcknowledgeFaultLatch(), so a
     * pure level test over them would re-latch on the next tick and the
     * external release could never take effect. */
    if (config->max_fault_vetoes_before_latch > 0u &&
        (((state->safety_latched != 0) && (safety_before == 0)) ||
         ((state->ood_latched != 0) && (ood_before == 0))) &&
        state->latch_episode_count >= config->max_fault_vetoes_before_latch) {
        state->permanent_latched = 1u;
        state->fault_audit_bitmask |= LAYERED_FAULT_BIT_PERMANENT_LATCH;
    }

    /* ---------------- 4. heartbeat watchdog ----------------------------- */
    /* Sequenced mailboxes are timed by the local lease (0b); legacy ones by
     * the transported age.  Both are held to exactly one rule here. */
    latency_hold = lg_is_finite(outer_age_eff) &&
                   (outer_age_eff > config->outer_timeout_s);
    if (latency_hold) {
        state->fault_audit_bitmask |= LAYERED_FAULT_BIT_LATENCY_TIMEOUT;

        /* A stale heartbeat is not a clean sample: re-arm the recovery
         * cooldown so the AI can only be re-armed after a fresh run of
         * consecutive clean ticks. */
        state->recovery_ticks = config->ood_recovery_cooldown_ticks;
    }

    /* ---------------- 5. decision priority ------------------------------ */
    veto_active = (state->safety_latched != 0) ||
                  (state->ood_latched != 0) ||
                  (conf_veto != 0);

    if (state->safety_latched) {
        gate = GATE_VETO_SAFETY_ENVELOPE;
        regime = REGIME_SAFE_FALLBACK;
    } else if (state->ood_latched) {
        gate = GATE_VETO_OOD_RESIDUAL;
        regime = REGIME_SAFE_FALLBACK;
    } else if (conf_veto) {
        gate = GATE_VETO_LOW_CONFIDENCE;
        regime = REGIME_SAFE_FALLBACK;
    } else if (latency_hold) {
        gate = GATE_HOLD_LATENCY;
        regime = state->regime; /* keep the last valid bank */
    } else {
        gate = GATE_NORMAL;

        if (state->recovery_ticks > 0u) {
            /* Post-fault cooldown: the fault is gone but the AI is not
             * re-armed yet, so the safe regulator keeps the plant.  Only a
             * fully clean tick consumes cooldown. */
            state->recovery_ticks--;
            regime = REGIME_SAFE_FALLBACK;
        } else if (mailbox_regime == state->regime) {
            regime = state->regime;
        } else if (lg_inc_sat(state->dwell_ticks) >= config->min_dwell_ticks) {
            regime = mailbox_regime; /* dwell satisfied: accept the switch */
        } else {
            regime = state->regime;  /* anti-chatter: keep the old regime */
        }
    }

    if (veto_active) {
        /* Any active veto re-arms the cooldown, so the AI is never re-armed
         * on the very next clean sample. */
        state->recovery_ticks = config->ood_recovery_cooldown_ticks;
    }

    /* The cumulative latch outranks every other decision: a unit that has
     * accumulated the configured number of severe episodes stays in R4 with
     * GATE_LATCHED_SAFE until the field service explicitly acknowledges it. */
    if (state->permanent_latched != 0u) {
        gate = GATE_LATCHED_SAFE;
        regime = REGIME_SAFE_FALLBACK;
    }

    /* ---------------- 5b. bounded native HOLD --------------------------- */
    /* Every consecutive held tick advances hold_ticks; entering any real
     * closed loop (R1..R4, including a veto-driven R4) clears it.  Once the
     * hold budget is exhausted the actuator target is zeroed, the branches
     * are cut and the command is ramped to zero under the commissioned slew
     * limit (a single-frame step would itself violate u_slew_max_per_s). */
    if (gate == GATE_HOLD_LATENCY) {
        /* Diagnostics only: count one hold EPISODE on the exact 0 -> 1 entry
         * edge of hold_ticks, not one count per held tick and not one per
         * age window.  hold_ticks is cleared on every non-HOLD tick, so this
         * edge is the native-HOLD entry itself. */
        if (state->hold_ticks == 0u) {
            state->latency_hold_count = lg_inc_sat(state->latency_hold_count);
        }
        state->hold_ticks = lg_inc_sat(state->hold_ticks);
        if (state->hold_ticks > config->max_hold_ticks) {
            gate = GATE_SAFE_ZERO;
        }
    } else {
        state->hold_ticks = 0u;
    }
    if (gate == GATE_SAFE_ZERO) {
        state->fault_audit_bitmask |= LAYERED_FAULT_BIT_HOLD_TIMEOUT;
        state->u_target = 0.0f;
        state->integ[0] = 0.0f;
        state->integ[1] = 0.0f;
        state->integ[2] = 0.0f;
        state->integ[3] = 0.0f;
        state->blend = 0.0f;
        state->regime = REGIME_SAFE_FALLBACK;
        regime = REGIME_SAFE_FALLBACK;
        /* Only a valid configuration reaches this path (the input firewall
         * above rejects invalid settings), so du_max is trustworthy. */
        state->u_applied = lg_clampf(0.0f,
                                     state->u_applied - du_max,
                                     state->u_applied + du_max);
    }

    /* ---------------- 6. controller bank -------------------------------- */
    do_control = (gate != GATE_HOLD_LATENCY) && (gate != GATE_SAFE_ZERO);

    if (do_control) {
        e = input->r - input->x;
        ed = input->r_dot - input->x_dot;

        /* The whole bank is evaluated every tick at a fixed cost: the inactive
         * controllers need their own u_pid for the tracking anti-windup.  The
         * normal banks use scaled nominal feedforward; R4 returns to zero. */
        u_pid0 = lg_pid_output(config, tuning, 0, e, ed, state->integ[0], input->u_ff);
        u_pid1 = lg_pid_output(config, tuning, 1, e, ed, state->integ[1], input->u_ff);
        u_pid2 = lg_pid_output(config, tuning, 2, e, ed, state->integ[2], input->u_ff);
        u_pid3 = lg_pid_output(config, tuning, 3, -input->x, -input->x_dot, state->integ[3], 0.0f);

        bank = (int)regime - 1;
        if (bank < 0) {
            bank = 0;
        }
        if (bank > 3) {
            bank = 3;
        }
        switch (bank) {
        case 0:
            u_active = u_pid0;
            break;
        case 1:
            u_active = u_pid1;
            break;
        case 2:
            u_active = u_pid2;
            break;
        default:
            u_active = u_pid3;
            break;
        }

        /* Bumpless transfer: first-order follow of the active controller.
         * Emergency fallback (an active veto or the permanent latch while the
         * core runs R4) bypasses this filter so the damping authority reaches
         * the actuator without one filter time constant of phase lag.  The
         * magnitude and slew limits below still apply. */
        if (snap) {
            /* Very first step: take the target as-is so the command never has
             * to ramp up from zero. */
            state->u_target = u_active;
            state->blend = 1.0f;
        } else if ((veto_active != 0 || state->permanent_latched != 0) &&
                   regime == REGIME_SAFE_FALLBACK) {
            state->u_target = u_active;
            state->blend = 1.0f;
        } else {
            if (regime != state->regime) {
                state->blend = 0.0f; /* a bank switch restarts the transfer */
            }
            state->u_target += config->blend_alpha * (u_active - state->u_target);
            state->blend = lg_clampf(state->blend + config->blend_alpha, 0.0f, 1.0f);
        }
        if (state->u_target != state->u_target) {
            state->u_target = 0.0f;
        }

        /* Hard actuator limits: magnitude first, then slew rate. */
        u_cmd = lg_clampf(state->u_target, -u_max, u_max);
        u_cmd = lg_clampf(u_cmd, state->u_applied - du_max, state->u_applied + du_max);

        /* Tracking anti-windup for every controller of the bank. */
        lg_integrate(&state->integ[0],
                     config->ki[0] * tuning->gain_scale,
                     e, dt_eff, u_pid0, u_cmd, windup_tau, integ_limit);
        lg_integrate(&state->integ[1],
                     config->ki[1] * tuning->gain_scale,
                     e, dt_eff, u_pid1, u_cmd, windup_tau, integ_limit);
        lg_integrate(&state->integ[2],
                     config->ki[2] * tuning->gain_scale,
                     e, dt_eff, u_pid2, u_cmd, windup_tau, integ_limit);
        lg_integrate(&state->integ[3],
                     config->ki[3],
                     -input->x, dt_eff, u_pid3, u_cmd, windup_tau, integ_limit);

        state->u_applied = u_cmd;
        state->control_started = 1;

        out.u = u_cmd;
        out.u_target = state->u_target;
        out.blend = state->blend;
    } else {
        /* GATE_HOLD_LATENCY holds command, target, blend and integrators;
         * GATE_SAFE_ZERO reports the forced de-energised output written above.
         * Dwell bookkeeping still advances; the watchdog re-arms recovery.
         * A held actuator command does not imply a held physical position. */
        out.u = state->u_applied;
        out.u_target = state->u_target;
        out.blend = state->blend;
    }

    /* ---------------- 7. dwell bookkeeping + output --------------------- */
    if (regime != state->regime) {
        state->dwell_ticks = 0u;
    } else {
        state->dwell_ticks = lg_inc_sat(state->dwell_ticks);
    }
    state->regime = regime;

    out.gate = gate;
    out.regime = regime;
    out.fault_audit_bitmask = state->fault_audit_bitmask;
    out.corrupted = outer_corrupt;

    state->step_count = lg_inc_sat(state->step_count);
    return out;
}

const char *Layered_GateName(layered_gate_state_t gate)
{
    switch (gate) {
    case GATE_NORMAL:
        return "GATE_NORMAL";
    case GATE_HOLD_LATENCY:
        return "GATE_HOLD_LATENCY";
    case GATE_VETO_OOD_RESIDUAL:
        return "GATE_VETO_OOD_RESIDUAL";
    case GATE_VETO_LOW_CONFIDENCE:
        return "GATE_VETO_LOW_CONFIDENCE";
    case GATE_VETO_SAFETY_ENVELOPE:
        return "GATE_VETO_SAFETY_ENVELOPE";
    case GATE_SAFE_ZERO:
        return "GATE_SAFE_ZERO";
    case GATE_LATCHED_SAFE:
        return "GATE_LATCHED_SAFE";
    default:
        break;
    }
    return "GATE_UNKNOWN";
}

const char *Layered_RegimeName(layered_regime_t regime)
{
    switch (regime) {
    case REGIME_NOMINAL:
        return "REGIME_NOMINAL";
    case REGIME_HIGH_LOAD_DAMPED:
        return "REGIME_HIGH_LOAD_DAMPED";
    case REGIME_BOUNDARY_COMPLIANT:
        return "REGIME_BOUNDARY_COMPLIANT";
    case REGIME_SAFE_FALLBACK:
        return "REGIME_SAFE_FALLBACK";
    default:
        break;
    }
    return "REGIME_UNKNOWN";
}
