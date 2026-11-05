// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jiawei Li
// Source: https://github.com/lijiawei255/ai-layered-control
// License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
// Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
/*
 * test_gatekeeper_standalone.c
 * ---------------------------------------------------------------------------
 * Deterministic standalone regression test for layered_gatekeeper.c.
 *
 *   gcc -std=c99 -pedantic -Wall -Wextra -Wdouble-promotion -Werror -O2
 *       -o test_gatekeeper_standalone.exe layered_gatekeeper.c
 *       test_gatekeeper_standalone.c
 *   ./test_gatekeeper_standalone.exe        exit code 0 = every check passed
 *
 * Determinism: no rand(), no clock(), no <math.h>.  The reference sine is
 * produced with a per-step rotation matrix built from truncated Taylor series,
 * so the whole run is a pure function of this source file.
 *
 * Coverage
 *   stage 1   nominal plant, slow reference   -> GATE_NORMAL / REGIME_NOMINAL
 *   stage 2   +35 % load drift                -> REGIME_HIGH_LOAD_DAMPED
 *   stage 3   jittered/dropped heartbeat      -> GATE_HOLD_LATENCY + closed-loop R4
 *   stage 4   OOD residual + envelope breach  -> veto < 10 ticks + hysteresis
 *   stage 5   FieldTuning_t: the three knobs really move the command
 *   stage 6   anti-chatter dwell, recovery cooldown, sticky fault audit bits
 *   stage 7   NaN/Inf/out-of-range IPC firewall
 *   stage 8   tracking anti-windup across a saturated bank switch
 *   stage 10  deterministic feedforward u_ff
 *   stage 11  sticky confidence veto
 *   stage 12  first-step snap, latency cooldown, per-bank ff_scale
 *   phase A   bounded HOLD -> slew-limited GATE_SAFE_ZERO incl. corrupt-config
 *             de-energise (A1), cumulative latch + explicit acknowledgement and
 *             post-ack episode-quota re-arm (A2), local outer_seq frame lease
 *             (A3), emergency R4 blend bypass (A4), signed envelope look-ahead
 *             (A8), closed-loop R4 during the latency grace (A9), normal-path
 *             slew bound rounded inward at the 1.0f / 7.0e-8 boundary (A6), and
 *             the GATE_SAFE_ZERO de-energise budget rounded inward at the
 *             reachable 1.5f / 0.1f boundary (A7)
 * ---------------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdarg.h>

#include "layered_gatekeeper.h"

#define TEST_TWO_PI 6.283185307179586
#define MISSION_STEPS 40000
#define STAGE4_START 30000
#define TRACK_STEPS 6000
#define TRACK_WINDOW 2000

/* ========================================================================= */
/* Check harness                                                             */
/* ========================================================================= */

static int g_pass = 0;
static int g_fail = 0;

static void check(int ok, const char *fmt, ...)
{
    va_list ap;

    if (ok) {
        g_pass++;
        fputs("[PASS] ", stdout);
    } else {
        g_fail++;
        fputs("[FAIL] ", stdout);
    }
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fputc('\n', stdout);
}

static float t_absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

/* Exact absolute difference of two binary32 values held in double: comparing
 * |u(k) - u(k-1)| in binary32 would round the difference itself and could hide
 * a sub-ULP overshoot, so the slew assertions below use this helper. */
static double t_absd(double v)
{
    return (v < 0.0) ? -v : v;
}

/* Finite binary32 check without <math.h>: NaN fails v == v, and +/-Inf makes
 * (v - v) NaN, which fails the second test. */
static int t_is_finite(float v)
{
    return (v == v) && ((v - v) == 0.0f);
}

/* One binary32 step from v toward zero, matching the core's
 * lg_next_toward_zero(): GATE_SAFE_ZERO uses exactly this step as its
 * quantisation floor when the legal slew budget is below one ULP. */
static float t_step_toward_zero(float v)
{
    union {
        uint32_t u;
        float f;
    } bits;
    uint32_t magnitude;

    if (v == 0.0f || v != v) {
        return 0.0f;
    }
    bits.f = v;
    magnitude = bits.u & 0x7FFFFFFFu;
    if (magnitude <= 1u) {
        return 0.0f;
    }
    bits.u = (bits.u & 0x80000000u) | (magnitude - 1u);
    return bits.f;
}

/* Fully initialised frames: the legacy scenarios fill the nine physical
 * fields and leave outer_seq at 0 (no local sequence lease).  Spelling every
 * field out keeps -Wmissing-field-initializers quiet on strict compilers. */
static LayeredInput_t zero_input(void)
{
    LayeredInput_t in = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0u};
    return in;
}

static LayeredOutput_t zero_output(void)
{
    LayeredOutput_t out = {0.0f, 0.0f, GATE_NORMAL, REGIME_NOMINAL, 0.0f, 0u, 0};
    return out;
}

/* ========================================================================= */
/* Deterministic signal generator (no <math.h>)                              */
/* ========================================================================= */

typedef struct {
    double s;   /* sin(phase) */
    double c;   /* cos(phase) */
    double ds;  /* sin(dphase), one inner period */
    double dc;  /* cos(dphase) */
    double amp;
    double omega;
} sine_gen_t;

static void sine_gen_init(sine_gen_t *g, double amp, double omega, double dt)
{
    double a = omega * dt;
    double a2 = a * a;

    g->s = 0.0;
    g->c = 1.0;
    g->ds = a - (a * a2) / 6.0 + (a * a2 * a2) / 120.0;
    g->dc = 1.0 - a2 / 2.0 + (a2 * a2) / 24.0;
    g->amp = amp;
    g->omega = omega;
}

static void sine_gen_next(sine_gen_t *g, float *value, float *slope)
{
    double s_next = g->c * g->ds + g->s * g->dc;
    double c_next = g->c * g->dc - g->s * g->ds;

    *value = (float)(g->amp * g->s);
    *slope = (float)(g->amp * g->omega * g->c);
    g->s = s_next;
    g->c = c_next;
}

/* Exact bit patterns: no libm, no float division by zero, fully portable. */
static float make_nan(void)
{
    union {
        uint32_t u;
        float f;
    } v;

    v.u = 0x7FC00000u; /* quiet NaN */
    return v.f;
}

static float make_inf(void)
{
    union {
        uint32_t u;
        float f;
    } v;

    v.u = 0x7F800000u; /* +Inf */
    return v.f;
}

/* ========================================================================= */
/* Scenario configuration                                                    */
/* ========================================================================= */

static void build_config(LayeredConfig_t *c)
{
    c->dt_s = 0.001f;
    c->outer_timeout_s = 0.250f;

    c->max_physics_residual = 1.0f;
    c->residual_clear_ratio = 0.80f;

    /* confidence_clear_ratio is an ABSOLUTE confidence level (0..1), not a
     * multiple: the sticky veto trips below 0.45 and clears at/above 0.55. */
    c->min_ai_confidence = 0.45f;
    c->confidence_clear_ratio = 0.55f;

    c->state_safety_limit = 1.0f;
    c->safety_trip_ratio = 0.90f;
    c->safety_clear_ratio = 0.75f;
    c->safety_predict_s = 0.06f;

    c->u_max = 2.0f;
    c->u_slew_max_per_s = 200.0f;
    c->blend_alpha = 0.02f;
    c->integ_limit = 2.0f;
    c->windup_tau_s = 0.05f;

    c->min_dwell_ticks = 150u;
    c->ood_recovery_cooldown_ticks = 500u;
    c->max_hold_ticks = 100u;
    c->max_fault_vetoes_before_latch = 3u;

    c->kp[0] = 1.20f;
    c->kp[1] = 0.90f;
    c->kp[2] = 0.70f;
    c->kp[3] = 0.50f;
    c->kd[0] = 0.05f;
    c->kd[1] = 0.12f;
    c->kd[2] = 0.10f;
    c->kd[3] = 0.02f;
    c->ki[0] = 0.20f;
    c->ki[1] = 0.15f;
    c->ki[2] = 0.10f;
    c->ki[3] = 0.05f;
    c->ff_scale[0] = 1.0f;
    c->ff_scale[1] = 1.0f;
    c->ff_scale[2] = 1.0f;
    c->ff_scale[3] = 1.0f;
}

static void build_tuning(FieldTuning_t *t, float gain, float damping, float margin)
{
    t->gain_scale = gain;
    t->damping_scale = damping;
    t->safety_margin_scale = margin;
}

static void print_header(const LayeredConfig_t *c)
{
    printf("layered_gatekeeper standalone test\n");
    printf("  cfg: dt=%.4f s  outer_timeout=%.3f s  residual_trip=%.2f clear=%.2f\n",
           (double)c->dt_s, (double)c->outer_timeout_s,
           (double)c->max_physics_residual, (double)c->residual_clear_ratio);
    printf("       conf_trip=%.2f conf_clear=%.2f  safety_limit=%.2f trip=%.2f clear=%.2f predict=%.3f\n",
           (double)c->min_ai_confidence, (double)c->confidence_clear_ratio,
           (double)c->state_safety_limit,
           (double)c->safety_trip_ratio, (double)c->safety_clear_ratio,
           (double)c->safety_predict_s);
    printf("       u_max=%.2f slew=%.1f/s alpha=%.3f integ_limit=%.2f windup_tau=%.3f s\n",
           (double)c->u_max, (double)c->u_slew_max_per_s, (double)c->blend_alpha,
           (double)c->integ_limit, (double)c->windup_tau_s);
    printf("       dwell=%u ticks  recovery_cooldown=%u ticks  max_hold=%u ticks  latch_after=%u veto episodes\n",
           (unsigned)c->min_dwell_ticks, (unsigned)c->ood_recovery_cooldown_ticks,
           (unsigned)c->max_hold_ticks, (unsigned)c->max_fault_vetoes_before_latch);
    printf("       kp=%.2f/%.2f/%.2f/%.2f  kd=%.2f/%.2f/%.2f/%.2f  ki=%.2f/%.2f/%.2f/%.2f\n",
           (double)c->kp[0], (double)c->kp[1], (double)c->kp[2], (double)c->kp[3],
           (double)c->kd[0], (double)c->kd[1], (double)c->kd[2], (double)c->kd[3],
           (double)c->ki[0], (double)c->ki[1], (double)c->ki[2], (double)c->ki[3]);
}

/* ========================================================================= */
/* Invariant tracker                                                         */
/* ========================================================================= */

typedef struct {
    int violations;
    int nan_count;
    int reported;
    float worst_abs_u;
    float worst_abs_du;
} inv_t;

static void inv_reset(inv_t *inv)
{
    inv->violations = 0;
    inv->nan_count = 0;
    inv->reported = 0;
    inv->worst_abs_u = 0.0f;
    inv->worst_abs_du = 0.0f;
}

/* Every single step of every scenario goes through here, so the hard actuator
 * contract is verified on all 40 000+ ticks of the test, not on a sample.
 *
 * The slew bound is the published strict inequality
 * |u(k) - u(k-1)| <= u_slew_max_per_s * dt_s on every normal tick.  It is
 * compared in double arithmetic: a binary32 subtraction would round the
 * difference itself and could hide a sub-ULP overshoot.  GATE_SAFE_ZERO keeps
 * its separate documented exception, a per-tick step of
 * max(du_max, 1 binary32 ULP(|u_prev|)) that only ever reduces |u|. */
static LayeredOutput_t step_checked(inv_t *inv,
                                    LayeredState_t *st,
                                    const LayeredConfig_t *cfg,
                                    const FieldTuning_t *tun,
                                    const LayeredInput_t *in)
{
    LayeredOutput_t out;
    float prev_u = st->u_applied;
    float du_max = cfg->u_slew_max_per_s * cfg->dt_s;
    float abs_u;
    float abs_du;
    double exact_du;
    double budget;
    int bad = 0;

    out = Layered_Step(st, cfg, tun, in);

    if (!t_is_finite(out.u) || !t_is_finite(out.u_target) || !t_is_finite(out.blend)) {
        inv->nan_count++;
        bad = 1;
    }

    abs_u = t_absf(out.u);
    if (abs_u > inv->worst_abs_u) {
        inv->worst_abs_u = abs_u;
    }
    if (!(abs_u <= cfg->u_max + 1e-6f)) {
        bad = 1;
    }

    abs_du = t_absf(out.u - prev_u);
    if (abs_du > inv->worst_abs_du) {
        inv->worst_abs_du = abs_du;
    }
    exact_du = (double)out.u - (double)prev_u;
    if (exact_du < 0.0) {
        exact_du = -exact_du;
    }
    if (out.gate == GATE_SAFE_ZERO) {
        budget = (double)du_max;
        if (t_step_toward_zero(prev_u) != prev_u) {
            double floor_step = (double)t_step_toward_zero(prev_u) - (double)prev_u;
            if (floor_step < 0.0) {
                floor_step = -floor_step;
            }
            if (floor_step > budget) {
                budget = floor_step;
            }
        }
        if (!(exact_du <= budget) || !(t_absf(out.u) <= t_absf(prev_u))) {
            bad = 1;
        }
    } else if (!(exact_du <= (double)du_max)) {
        bad = 1;
    }

    if (!(out.blend >= 0.0f && out.blend <= 1.0f)) {
        bad = 1;
    }

    if (bad) {
        if (inv->reported < 5) {
            inv->reported++;
            printf("[INV ] step=%u pin=%.6f u=%.6f prev_u=%.6f du=%.6f gate=%s\n",
                   (unsigned)st->step_count, (double)in->x, (double)out.u,
                   (double)prev_u, (double)abs_du, Layered_GateName(out.gate));
        }
        inv->violations++;
    }
    return out;
}

/* ========================================================================= */
/* Stage 1..4: the 40 s mission                                              */
/* ========================================================================= */

static float mission_x(double t, float r, float r_dot, float *x_dot)
{
    if (t < 10.0) { /* nominal plant: mild amplitude loss */
        *x_dot = 0.60f * r_dot;
        return 0.60f * r;
    }
    if (t < 30.0) { /* +35 % load: more lag, bigger tracking error */
        *x_dot = 0.50f * r_dot;
        return 0.50f * r;
    }
    if (t < 31.0) {
        *x_dot = 0.50f * r_dot;
        return 0.50f * r;
    }
    if (t < 34.0) { /* safety stage: a slow demand walk up to 0.97 */
        *x_dot = 0.25666667f;
        return 0.2f + (float)((t - 31.0) * 0.2566666666666667);
    }
    if (t < 36.0) { /* parked just under the hard limit */
        *x_dot = 0.0f;
        return 0.97f;
    }
    if (t < 36.5) { /* walk back inside the envelope */
        *x_dot = -1.14f;
        return 0.97f - (float)((t - 36.0) * 1.14);
    }
    *x_dot = 0.0f;
    return 0.40f;
}

static void test_mission(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    inv_t inv;
    sine_gen_t ref;
    LayeredOutput_t out;
    LayeredOutput_t last;
    int k;

    int stage1_bad = 0;
    int stage2_bad = 0;
    int stage2_veto_ok = 1;
    unsigned snap_safety = 0u;
    unsigned snap_ood = 0u;
    unsigned snap_conf = 0u;
    unsigned snap_latency = 0u;

    int hold_steps = 0;
    int hold_episodes = 0;
    int hold_regime_bad = 0;
    int hold_motion = 0;
    int hold_recoveries = 0;
    int hold_bad_recovery = 0;
    int prev_hold = 0;
    float hold_prev_u = 0.0f;

    int first_ood = -1;
    int first_safety = -1;
    int safety_cleared_step = -1;
    int ood_regime_bad = 0;
    int conf_gate_in_fault = 0;
    int safety_after_ood = 0;
    int ood_after_safety = 0;
    int hysteresis_bad = 0;
    float x_at_first_safety = 0.0f;
    float pred_at_first_safety = 0.0f;
    float x_at_clear = 0.0f;

    int clean_after_fault = -1;
    int cooldown_ticks = 0;
    int regime_restored = -1;
    int cooldown_bad = 0;

    float age = 0.005f;
    int age_countdown = 0;
    int frame_no = 0;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    inv_reset(&inv);
    sine_gen_init(&ref, 0.5, TEST_TWO_PI * 0.5, (double)cfg.dt_s);

    for (k = 0; k < MISSION_STEPS; ++k) {
        double t = (double)k * (double)cfg.dt_s;
        LayeredInput_t in = zero_input();
        float r;
        float r_dot;
        float x_dot;

        sine_gen_next(&ref, &r, &r_dot);

        in.r = r;
        in.r_dot = r_dot;
        in.x = mission_x(t, r, r_dot, &x_dot);
        in.x_dot = x_dot;
        in.u_ff = 0.0f; /* the mission runs without a commissioning feedforward */
        in.ai_regime = (t < 10.0) ? 1.0f : 2.0f;

        if (t < 10.0) {
            in.ai_confidence = 0.95f;
            in.physics_residual = 0.10f;
        } else if (t < 30.0) {
            in.ai_confidence = 0.90f;
            in.physics_residual = 0.35f;
        } else if (t < 36.5) {
            in.ai_confidence = 0.20f;
            in.physics_residual = 3.00f;
        } else if (t < 37.0) {
            in.ai_confidence = 0.92f;
            in.physics_residual = 0.90f; /* inside the hysteresis band */
        } else {
            in.ai_confidence = 0.92f;
            in.physics_residual = 0.70f; /* below residual_.clear_ratio*trip */
        }

        if (t < 20.0 || t >= 30.0) {
            in.outer_age_s = 0.005f;
        } else {
            /* deterministic jittered heartbeat with occasional dropped frames */
            age += cfg.dt_s;
            if (age_countdown > 0) {
                age_countdown--;
            }
            if (age_countdown == 0) {
                frame_no++;
                if ((frame_no % 17) == 5) {
                    age_countdown = 200; /* frame dropped: stale for ~300 ms */
                } else {
                    age = 0.005f + 0.0015f * (float)(frame_no % 5);
                    age_countdown = 100 + 2 * (frame_no % 7);
                }
            }
            in.outer_age_s = age;
        }

        out = step_checked(&inv, &st, &cfg, &tun, &in);
        last = out;

        /* ---------------- stage 1 ---------------- */
        if (k < 10000) {
            if (out.gate != GATE_NORMAL || out.regime != REGIME_NOMINAL) {
                stage1_bad++;
            }
        }

        /* ---------------- stage 2 ---------------- */
        if (k == 10000) {
            snap_safety = st.safety_veto_count;
            snap_ood = st.ood_veto_count;
            snap_conf = st.low_confidence_veto_count;
            snap_latency = st.latency_hold_count;
        }
        if (k >= 10000 && k < 20000) {
            if (out.gate != GATE_NORMAL || out.regime != REGIME_HIGH_LOAD_DAMPED) {
                stage2_bad++;
            }
        }
        if (k == 19999) {
            if (st.safety_veto_count != snap_safety || st.ood_veto_count != snap_ood ||
                st.low_confidence_veto_count != snap_conf ||
                st.latency_hold_count != snap_latency) {
                stage2_veto_ok = 0;
            }
        }

        /* ---------------- stage 3 ---------------- */
        if (k >= 20000 && k < 30000) {
            if (out.gate == GATE_HOLD_LATENCY) {
                hold_steps++;
                if (!prev_hold) {
                    hold_episodes++;
                }
                /* The slow mailbox is stale but the fast sensors are healthy:
                 * the deterministic fallback must close the loop, not freeze
                 * the last torque. */
                if (out.regime != REGIME_SAFE_FALLBACK) {
                    hold_regime_bad++;
                }
                if (out.u != hold_prev_u) {
                    hold_motion++;
                }
            } else {
                if (prev_hold) {
                    hold_recoveries++;
                    if (out.gate != GATE_NORMAL) {
                        hold_bad_recovery++;
                    }
                }
            }
            prev_hold = (out.gate == GATE_HOLD_LATENCY) ? 1 : 0;
            hold_prev_u = out.u;
        }

        /* ---------------- stage 4 ---------------- */
        if (k >= STAGE4_START) {
            if (out.gate == GATE_VETO_LOW_CONFIDENCE) {
                /* the sticky confidence latch is set during the fault, but
                 * OOD/safety must always outrank it */
                conf_gate_in_fault++;
            }
            if (out.gate == GATE_VETO_OOD_RESIDUAL) {
                if (first_ood < 0) {
                    first_ood = k;
                }
                if (out.regime != REGIME_SAFE_FALLBACK) {
                    ood_regime_bad++;
                }
                if (first_safety >= 0) {
                    ood_after_safety = 1;
                }
            }
            if (out.gate == GATE_VETO_SAFETY_ENVELOPE) {
                if (first_safety < 0) {
                    first_safety = k;
                    safety_after_ood = (first_ood >= 0) ? 1 : 0;
                    x_at_first_safety = in.x;
                    pred_at_first_safety = t_absf(in.x + cfg.safety_predict_s * in.x_dot);
                }
            } else if (first_safety >= 0 && safety_cleared_step < 0) {
                safety_cleared_step = k;
                x_at_clear = in.x;
            }
            if (!(in.physics_residual > cfg.max_physics_residual) &&
                in.physics_residual > cfg.residual_clear_ratio * cfg.max_physics_residual &&
                out.gate != GATE_VETO_OOD_RESIDUAL && safety_cleared_step >= 0) {
                hysteresis_bad++;
            }

            /* post-fault recovery cooldown measurement */
            if (clean_after_fault < 0 && out.gate == GATE_NORMAL) {
                clean_after_fault = k;
            }
            if (clean_after_fault >= 0) {
                if (out.regime == REGIME_SAFE_FALLBACK) {
                    cooldown_ticks++;
                } else if (regime_restored < 0) {
                    regime_restored = k;
                }
            }
        }

        /* keep the compiler honest about the counter snapshots */
        if (k == MISSION_STEPS - 1) {
            if (cooldown_ticks <= 0) {
                cooldown_bad++;
            }
        }
    }

    /* ---------------- stage 1..4 verdicts ---------------- */
    check(stage1_bad == 0,
          "stage 1 (0-10 s): gate==GATE_NORMAL / regime==REGIME_NOMINAL on all 10000 ticks (violations %d)",
          stage1_bad);
    check(stage2_bad == 0,
          "stage 2 (10-20 s): gate==GATE_NORMAL / regime==REGIME_HIGH_LOAD_DAMPED on all 10000 ticks (violations %d)",
          stage2_bad);
    check(stage2_veto_ok != 0,
          "stage 2: no veto/hold counter moved while the physics drifted (+35%%)");

    check(hold_episodes >= 1,
          "stage 3 (20-30 s): %d GATE_HOLD_LATENCY episode(s) over %d held ticks",
          hold_episodes, hold_steps);
    check(hold_episodes == (int)st.latency_hold_count,
          "stage 3: latency_hold_count (%u) matches the %d episodes seen by the harness",
          (unsigned)st.latency_hold_count, hold_episodes);
    check(hold_regime_bad == 0 && hold_motion > 0,
          "stage 3: every one of the %d held ticks runs REGIME_SAFE_FALLBACK in closed loop "
          "(command changes on %d ticks, i.e. it is not a frozen torque)",
          hold_steps, hold_motion);
    check(hold_bad_recovery == 0 && hold_recoveries == hold_episodes,
          "stage 3: every hold episode returns to GATE_NORMAL on the first fresh heartbeat (%d/%d)",
          hold_recoveries, hold_episodes);

    check(first_ood >= 0 && (first_ood - STAGE4_START) <= 10,
          "stage 4 (30-40 s): first GATE_VETO_OOD_RESIDUAL %d tick(s) after the fault injection (limit 10)",
          first_ood - STAGE4_START);
    check(ood_regime_bad == 0,
          "stage 4: every OOD veto tick commands REGIME_SAFE_FALLBACK (violations %d)", ood_regime_bad);
    check(first_safety >= 0 && safety_after_ood != 0,
          "stage 4: envelope veto asserted at tick %d once |x| approached the limit",
          first_safety);
    check(pred_at_first_safety > 0.90f && x_at_first_safety > 0.899f &&
          x_at_first_safety < 0.91f,
          "stage 4: envelope trip rule fired at |x|=%.5f pred=%.5f (trip ratio 0.90 of limit 1.0)",
          (double)x_at_first_safety, (double)pred_at_first_safety);
    check(safety_cleared_step > 0 && x_at_clear < 0.75f,
          "stage 4: the sticky envelope latch only cleared at |x|=%.5f (clear ratio 0.75)",
          (double)x_at_clear);
    check(ood_after_safety != 0,
          "stage 4: OOD veto is still latched after the envelope clears (priority chain intact)");
    check(hysteresis_bad == 0,
          "stage 4: residual 0.90 (trip 1.00, clear 0.80) keeps the OOD latch alive (violations %d)",
          hysteresis_bad);
    check(cooldown_ticks == (int)cfg.ood_recovery_cooldown_ticks,
          "stage 4: after the last veto the core stays in REGIME_SAFE_FALLBACK for %d clean ticks (cooldown %u)",
          cooldown_ticks, (unsigned)cfg.ood_recovery_cooldown_ticks);
    check(regime_restored == clean_after_fault + (int)cfg.ood_recovery_cooldown_ticks,
          "stage 4: the AI bank is re-armed exactly at tick %d (= first clean tick %d + cooldown)",
          regime_restored, clean_after_fault);
    check(cooldown_bad == 0,
          "stage 4: the recovery cooldown was actually consumed (not a zero-length window)");
    check(st.ood_veto_count == 1u && st.safety_veto_count == 1u,
          "counters: exactly one OOD and one envelope episode were counted (%u, %u)",
          (unsigned)st.ood_veto_count, (unsigned)st.safety_veto_count);
    check(st.low_confidence_veto_count == 1u && conf_gate_in_fault == 0,
          "counters: the sticky confidence latch entered once (%u) but never became the decisive gate (%d ticks)",
          (unsigned)st.low_confidence_veto_count, conf_gate_in_fault);

    /* ---------------- sticky fault audit ---------------- */
    check((st.fault_audit_bitmask & LAYERED_FAULT_BIT_LATENCY_TIMEOUT) != 0u &&
          (st.fault_audit_bitmask & LAYERED_FAULT_BIT_OOD_RESIDUAL) != 0u &&
          (st.fault_audit_bitmask & LAYERED_FAULT_BIT_ENVELOPE_BREACH) != 0u,
          "fault audit: stage 3 latched LATENCY_TIMEOUT, stage 4 latched OOD_RESIDUAL+ENVELOPE_BREACH (mask 0x%02X)",
          (unsigned)st.fault_audit_bitmask);
    check((st.fault_audit_bitmask & LAYERED_FAULT_BIT_NAN_CORRUPTION) == 0u,
          "fault audit: a clean mission never latches NAN_CORRUPTION");
    check(last.fault_audit_bitmask == st.fault_audit_bitmask,
          "fault audit: the per-step output snapshot equals the sticky state (0x%02X)",
          (unsigned)last.fault_audit_bitmask);
    check(last.gate == GATE_NORMAL && last.regime == REGIME_HIGH_LOAD_DAMPED,
          "fault audit: the mission ends healthy in %s / %s",
          Layered_GateName(last.gate), Layered_RegimeName(last.regime));

    /* ---------------- global invariants ---------------- */
    check(inv.violations == 0,
          "stage 9: |u|<=u_max and |du|<=slew*dt on all %d ticks (worst |u|=%.6f, worst |du|=%.6f)",
          MISSION_STEPS, (double)inv.worst_abs_u, (double)inv.worst_abs_du);
    check(inv.nan_count == 0,
          "stage 9: u / u_target / blend stay finite over the whole mission");
    check(st.step_count == (uint32_t)MISSION_STEPS,
          "stage 9: exactly %u steps processed", (unsigned)st.step_count);
}

/* ========================================================================= */
/* Stage 5: FieldTuning_t                                                    */
/* ========================================================================= */

static void run_tracking(const LayeredConfig_t *cfg,
                         const FieldTuning_t *tun,
                         int n_steps,
                         float plant_gain,
                         float *u_log,
                         float *ed_log,
                         double *mean_abs_u,
                         inv_t *inv)
{
    LayeredState_t st;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    sine_gen_t ref;
    int k;
    double sum = 0.0;
    int n_sum = 0;
    int window_start = n_steps - TRACK_WINDOW;

    Layered_Init(&st, cfg, tun);
    sine_gen_init(&ref, 0.5, TEST_TWO_PI * 0.5, (double)cfg->dt_s);

    for (k = 0; k < n_steps; ++k) {
        float r;
        float r_dot;

        sine_gen_next(&ref, &r, &r_dot);
        in.ai_regime = 1.0f;
        in.ai_confidence = 0.95f;
        in.outer_age_s = 0.005f;
        in.physics_residual = 0.10f;
        in.r = r;
        in.r_dot = r_dot;
        in.x = plant_gain * r;
        in.x_dot = plant_gain * r_dot;
        in.u_ff = 0.0f;

        out = step_checked(inv, &st, cfg, tun, &in);

        if (u_log != 0) {
            u_log[k] = out.u;
        }
        if (ed_log != 0) {
            ed_log[k] = in.r_dot - in.x_dot;
        }
        if (k >= window_start) {
            sum += (double)t_absf(out.u);
            n_sum++;
        }
    }
    *mean_abs_u = sum / (double)n_sum;
}

static void test_tuning_gain(void)
{
    LayeredConfig_t cfg;
    FieldTuning_t t_low;
    FieldTuning_t t_high;
    inv_t inv;
    double mean_low = 0.0;
    double mean_high = 0.0;

    build_config(&cfg);
    build_tuning(&t_low, 1.0f, 1.0f, 1.0f);
    build_tuning(&t_high, 1.3f, 1.0f, 1.0f);

    inv_reset(&inv);
    run_tracking(&cfg, &t_low, TRACK_STEPS, 0.60f, 0, 0, &mean_low, &inv);
    check(inv.violations == 0,
          "stage 5: the actuator invariants hold on the first gain_scale sweep");
    inv_reset(&inv);
    run_tracking(&cfg, &t_high, TRACK_STEPS, 0.60f, 0, 0, &mean_high, &inv);

    check(mean_high > 1.15 * mean_low,
          "stage 5: gain_scale 1.3 lifts the steady-state |u| by %.1f%% (mean|u| %.4f -> %.4f)",
          100.0 * (mean_high / mean_low - 1.0), mean_low, mean_high);
    check(inv.violations == 0,
          "stage 5: the actuator invariants hold while gain_scale sweeps");
}

static void test_tuning_damping(void)
{
    static float u_soft[TRACK_STEPS];
    static float u_stiff[TRACK_STEPS];
    static float ed_log[TRACK_STEPS];
    LayeredConfig_t cfg;
    FieldTuning_t t_soft;
    FieldTuning_t t_stiff;
    inv_t inv;
    double mean_soft = 0.0;
    double mean_stiff = 0.0;
    double corr = 0.0;
    float max_diff = 0.0f;
    int k;

    build_config(&cfg);
    build_tuning(&t_soft, 1.0f, 0.5f, 1.0f);
    build_tuning(&t_stiff, 1.0f, 1.5f, 1.0f);

    inv_reset(&inv);
    run_tracking(&cfg, &t_soft, TRACK_STEPS, 0.60f, u_soft, ed_log, &mean_soft, &inv);
    check(inv.violations == 0,
          "stage 5: the actuator invariants hold on the first damping_scale sweep");
    inv_reset(&inv);
    run_tracking(&cfg, &t_stiff, TRACK_STEPS, 0.60f, u_stiff, ed_log, &mean_stiff, &inv);

    for (k = 0; k < TRACK_STEPS; ++k) {
        float diff = u_stiff[k] - u_soft[k];

        if (t_absf(diff) > max_diff) {
            max_diff = t_absf(diff);
        }
        corr += (double)diff * (double)ed_log[k];
    }

    check(max_diff > 1.0e-3f,
          "stage 5: damping_scale 0.5 vs 1.5 changes the command (max |du| = %.6f)",
          (double)max_diff);
    check(corr > 0.0,
          "stage 5: extra damping pushes the command along +ed, i.e. it really is a D action (corr %.4f)",
          corr);
    check(inv.violations == 0,
          "stage 5: the actuator invariants hold while damping_scale sweeps");
}

static int run_safety_ramp(const LayeredConfig_t *cfg, const FieldTuning_t *tun)
{
    LayeredState_t st;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    inv_t inv;
    int k;
    int trip = -1;

    Layered_Init(&st, cfg, tun);
    inv_reset(&inv);

    for (k = 0; k < 6000; ++k) {
        in.ai_regime = 1.0f;
        in.ai_confidence = 0.95f;
        in.outer_age_s = 0.005f;
        in.physics_residual = 0.10f;
        in.r = 0.5f;
        in.r_dot = 0.0f;
        in.x = 0.05f + 0.2f * (float)k * cfg->dt_s; /* slow walk into the envelope */
        in.x_dot = 0.2f;
        in.u_ff = 0.0f;

        out = step_checked(&inv, &st, cfg, tun, &in);
        if (out.gate == GATE_VETO_SAFETY_ENVELOPE && trip < 0) {
            trip = k;
        }
    }
    check(inv.violations == 0,
          "stage 5: the actuator invariants hold through the whole safety ramp");
    return trip;
}

static void test_tuning_safety_margin(void)
{
    LayeredConfig_t cfg;
    FieldTuning_t t_nom;
    FieldTuning_t t_tight;
    int trip_nom;
    int trip_tight;

    build_config(&cfg);
    build_tuning(&t_nom, 1.0f, 1.0f, 1.0f);
    build_tuning(&t_tight, 1.0f, 1.0f, 0.8f);

    trip_nom = run_safety_ramp(&cfg, &t_nom);
    trip_tight = run_safety_ramp(&cfg, &t_tight);

    check(trip_nom > 0 && trip_tight > 0,
          "stage 5: both safety ramps tripped (margin 1.0 at tick %d, margin 0.8 at tick %d)",
          trip_nom, trip_tight);
    check(trip_tight < trip_nom,
          "stage 5: safety_margin_scale 0.8 trips %d ticks earlier than 1.0 (%d vs %d)",
          trip_nom - trip_tight, trip_tight, trip_nom);
}

/* ========================================================================= */
/* Stage 5b: anti-chatter dwell                                              */
/* ========================================================================= */

static void test_dwell(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    inv_t inv;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    sine_gen_t ref;
    int k;
    int switches = 0;
    int toggles = 0;
    int min_gap = 1 << 30;
    int last_switch = -1;
    int prev_regime = 0;
    int prev_wanted = 1;
    int gate_bad = 0;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    inv_reset(&inv);
    sine_gen_init(&ref, 0.5, TEST_TWO_PI * 0.5, (double)cfg.dt_s);
    prev_regime = (int)st.regime;

    for (k = 0; k < 10000; ++k) {
        float r;
        float r_dot;
        int wanted = (((k / 10) % 2) == 0) ? 1 : 2; /* 50 Hz square wave (100 transitions/s) */

        sine_gen_next(&ref, &r, &r_dot);
        in.ai_regime = (float)wanted;
        in.ai_confidence = 0.95f;
        in.outer_age_s = 0.005f;
        in.physics_residual = 0.10f;
        in.r = r;
        in.r_dot = r_dot;
        in.x = 0.30f * r;
        in.x_dot = 0.30f * r_dot;
        in.u_ff = 0.0f;

        out = step_checked(&inv, &st, &cfg, &tun, &in);

        if (wanted != prev_wanted) {
            toggles++;
            prev_wanted = wanted;
        }
        if ((int)out.regime != prev_regime) {
            switches++;
            if (last_switch >= 0 && (k - last_switch) < min_gap) {
                min_gap = k - last_switch;
            }
            last_switch = k;
            prev_regime = (int)out.regime;
        }
        if (out.gate != GATE_NORMAL) {
            gate_bad++;
        }
    }

    check(gate_bad == 0,
          "stage 6 dwell: the gate stays GATE_NORMAL while the AI regime chatters (violations %d)",
          gate_bad);
    check(switches > 0 && min_gap >= (int)cfg.min_dwell_ticks,
          "stage 6 dwell: %d effective regime switches, minimum spacing %d ticks (dwell %u)",
          switches, (min_gap == (1 << 30)) ? -1 : min_gap, (unsigned)cfg.min_dwell_ticks);
    check(min_gap == (int)cfg.min_dwell_ticks,
          "stage 6 dwell: the minimum spacing is exactly min_dwell_ticks=%u ticks",
          (unsigned)cfg.min_dwell_ticks);
    check(switches * 3 <= toggles,
          "stage 6 dwell: %d switches for %d AI regime-change requests (anti-chatter keeps the bank)",
          switches, toggles);
    check(inv.violations == 0,
          "stage 6 dwell: the actuator invariants hold through the chatter");
}

/* ========================================================================= */
/* Stage 6b: post-fault recovery cooldown                                    */
/* ========================================================================= */

static void test_recovery_cooldown(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    inv_t inv;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int k;
    int fallback_ticks = 0;
    int fallback_bad = 0;
    int switched_at = -1;
    int i;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    inv_reset(&inv);

    /* 200 healthy ticks: the dwell timer is already satisfied */
    for (i = 0; i < 200; ++i) {
        in.ai_regime = 1.0f;
        in.ai_confidence = 0.95f;
        in.outer_age_s = 0.005f;
        in.physics_residual = 0.10f;
        in.r = 0.0f;
        in.r_dot = 0.0f;
        in.x = 0.0f;
        in.x_dot = 0.0f;
        in.u_ff = 0.0f;
        step_checked(&inv, &st, &cfg, &tun, &in);
    }

    /* 5 ticks of obvious OOD fault */
    for (i = 0; i < 5; ++i) {
        in.ai_regime = 1.0f;
        in.ai_confidence = 0.95f;
        in.outer_age_s = 0.005f;
        in.physics_residual = 3.0f;
        in.r = 0.0f;
        in.r_dot = 0.0f;
        in.x = 0.0f;
        in.x_dot = 0.0f;
        in.u_ff = 0.0f;
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_VETO_OOD_RESIDUAL && out.regime == REGIME_SAFE_FALLBACK,
          "stage 6 cooldown: a 5-tick OOD burst forces REGIME_SAFE_FALLBACK");

    /* the fault is gone and the AI immediately asks for another bank */
    for (k = 0; k < (int)cfg.ood_recovery_cooldown_ticks + 100; ++k) {
        in.ai_regime = 2.0f;
        in.ai_confidence = 0.95f;
        in.outer_age_s = 0.005f;
        in.physics_residual = 0.10f;
        in.r = 0.0f;
        in.r_dot = 0.0f;
        in.x = 0.0f;
        in.x_dot = 0.0f;
        in.u_ff = 0.0f;
        out = step_checked(&inv, &st, &cfg, &tun, &in);

        if (k < (int)cfg.ood_recovery_cooldown_ticks) {
            if (out.gate == GATE_NORMAL && out.regime == REGIME_SAFE_FALLBACK) {
                fallback_ticks++;
            } else {
                fallback_bad++;
            }
        }
        if (switched_at < 0 && out.regime == REGIME_HIGH_LOAD_DAMPED) {
            switched_at = k;
        }
    }

    check(fallback_bad == 0 && fallback_ticks == (int)cfg.ood_recovery_cooldown_ticks,
          "stage 6 cooldown: %d/%u clean ticks keep REGIME_SAFE_FALLBACK with gate GATE_NORMAL",
          fallback_ticks, (unsigned)cfg.ood_recovery_cooldown_ticks);
    check(switched_at == (int)cfg.ood_recovery_cooldown_ticks,
          "stage 6 cooldown: the AI bank is re-armed on tick %d, i.e. exactly after the cooldown",
          switched_at);
    check(st.recovery_ticks == 0u,
          "stage 6 cooldown: recovery_ticks is down to 0 once the AI is re-armed");
    check(inv.violations == 0,
          "stage 6 cooldown: the actuator invariants hold through the whole recovery window");
}

/* ========================================================================= */
/* Stage 7: NaN / Inf / out-of-range firewall                                */
/* ========================================================================= */

static void test_firewall(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    inv_t inv;
    LayeredInput_t good = zero_input();
    LayeredInput_t bad = zero_input();
    LayeredOutput_t out;
    float integ_snapshot[LAYERED_NUM_REGIMES];
    float saved_u;
    float saved_target;
    float saved_blend;
    layered_regime_t saved_regime;
    uint32_t saved_dwell;
    uint32_t saved_corrupt;
    int saved_ood;
    int saved_safety;
    int i;
    int corrupted_count = 0;
    int i_finite;
    int finite_ok = 1;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    inv_reset(&inv);

    good.ai_regime = 1.0f;
    good.ai_confidence = 0.95f;
    good.outer_age_s = 0.005f;
    good.physics_residual = 0.10f;
    good.r = 0.3f;
    good.r_dot = 0.0f;
    good.x = 0.1f;
    good.x_dot = 0.0f;
    good.u_ff = 0.0f;

    /* warm up so that the integrators / command are non-trivial */
    for (i = 0; i < 200; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &good);
    }
    check(out.corrupted == 0 && out.gate == GATE_NORMAL,
          "stage 7 firewall: 200 healthy frames are accepted and processed normally");

    for (i = 0; i < 5; ++i) {
        int j;

        bad = good;
        switch (i) {
        case 0:
            bad.x_dot = make_nan();
            break;
        case 1:
            bad.physics_residual = make_inf();
            break;
        case 2:
            bad.r_dot = make_inf();
            break;
        case 3:
            bad.x = make_nan();
            break;
        default:
            bad.u_ff = make_nan();
            break;
        }

        saved_u = st.u_applied;
        saved_target = st.u_target;
        saved_blend = st.blend;
        saved_regime = st.regime;
        saved_dwell = st.dwell_ticks;
        saved_corrupt = st.corrupted_frame_count;
        saved_ood = st.ood_latched;
        saved_safety = st.safety_latched;
        for (j = 0; j < LAYERED_NUM_REGIMES; ++j) {
            integ_snapshot[j] = st.integ[j];
        }

        out = Layered_Step(&st, &cfg, &tun, &bad);
        corrupted_count++;

        check(out.corrupted == 1 && out.gate == GATE_HOLD_LATENCY,
              "stage 7 firewall case %d: corrupt frame rejected (corrupted=1, gate=%s)",
              i + 1, Layered_GateName(out.gate));
        check(out.u == saved_u && st.u_applied == saved_u &&
              st.u_target == saved_target && st.blend == saved_blend &&
              st.regime == saved_regime,
              "stage 7 firewall case %d: actuator held at %.6f and every controller output frozen",
              i + 1, (double)saved_u);
        check(st.integ[0] == integ_snapshot[0] && st.integ[1] == integ_snapshot[1] &&
              st.integ[2] == integ_snapshot[2] && st.integ[3] == integ_snapshot[3],
              "stage 7 firewall case %d: not a single integrator was updated",
              i + 1);
        check(st.ood_latched == saved_ood && st.safety_latched == saved_safety &&
              st.dwell_ticks == saved_dwell &&
              st.recovery_ticks == cfg.ood_recovery_cooldown_ticks,
              "stage 7 firewall case %d: no latch/dwell state was touched, and the recovery cooldown was re-armed to %u",
              i + 1, (unsigned)st.recovery_ticks);
        check(st.corrupted_frame_count == saved_corrupt + 1u &&
              (st.fault_audit_bitmask & LAYERED_FAULT_BIT_NAN_CORRUPTION) != 0u,
              "stage 7 firewall case %d: corrupted_frame_count=%u and NAN_CORRUPTION latched",
              i + 1, (unsigned)st.corrupted_frame_count);
    }

    /* the core must recover: healthy frames still produce a sane command */
    for (i = 0; i < 200; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &good);
        if (!t_is_finite(out.u) || t_absf(out.u) > cfg.u_max) {
            finite_ok = 0;
        }
    }
    i_finite = t_is_finite(st.integ[0]) && t_is_finite(st.integ[1]) &&
               t_is_finite(st.integ[2]) && t_is_finite(st.integ[3]);

    check(finite_ok && i_finite,
          "stage 7 firewall: after %d corrupt frames the core keeps producing a finite command",
          corrupted_count);
    check(out.corrupted == 0 && out.gate == GATE_NORMAL &&
          st.corrupted_frame_count == (uint32_t)corrupted_count,
          "stage 7 firewall: healthy traffic resumes and corrupted_frame_count stays at %u",
          (unsigned)st.corrupted_frame_count);
    check(inv.violations == 0 && inv.nan_count == 0,
          "stage 7 firewall: the actuator invariants hold across the whole injection");
}

/* ========================================================================= */
/* Stage 8: tracking anti-windup across a saturated bank switch              */
/* ========================================================================= */

static void test_antiwindup(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    inv_t inv;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int k;
    float u_before;
    float u_after;
    float du;
    float e;
    float u_pid0;
    float u_pid1;
    float du_bound;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    inv_reset(&inv);

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.r = 2.0f;
    in.r_dot = 0.0f;
    in.x = 0.0f;
    in.x_dot = 0.0f;
    in.u_ff = 0.0f;

    /* 2000 ticks of a demand the actuator cannot follow: hard saturation */
    for (k = 0; k < 2000; ++k) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(t_absf(out.u - cfg.u_max) <= 1e-6f,
          "stage 8 anti-windup: the command is pinned at u_max=%.2f after 2 s of saturation",
          (double)out.u);
    check(st.integ[2] == cfg.integ_limit && st.integ[3] == cfg.integ_limit,
          "stage 8 anti-windup: the un-reachable banks sit on the +/-integ_limit clamp (%.2f / %.2f)",
          (double)st.integ[2], (double)st.integ[3]);

    e = in.r - in.x;
    u_pid0 = (cfg.kp[0] * tun.gain_scale) * e +
             (cfg.kd[0] * tun.damping_scale) * (in.r_dot - in.x_dot) +
             (cfg.ki[0] * tun.gain_scale) * st.integ[0];
    u_pid1 = (cfg.kp[1] * tun.gain_scale) * e +
             (cfg.kd[1] * tun.damping_scale) * (in.r_dot - in.x_dot) +
             (cfg.ki[1] * tun.gain_scale) * st.integ[1];
    check(t_absf(u_pid0 - out.u) < 0.05f && t_absf(u_pid1 - out.u) < 0.05f,
          "stage 8 anti-windup: inactive bank 2 tracks the applied command (|u_pid-u|=%.4f/%.4f < 0.05)",
          (double)t_absf(u_pid0 - out.u), (double)t_absf(u_pid1 - out.u));

    /* now switch the bank: no energy may be released */
    u_before = out.u;
    in.ai_regime = 2.0f;
    out = step_checked(&inv, &st, &cfg, &tun, &in);
    u_after = out.u;
    du = t_absf(u_after - u_before);
    du_bound = cfg.blend_alpha * cfg.u_max + cfg.u_slew_max_per_s * cfg.dt_s;

    check(out.regime == REGIME_HIGH_LOAD_DAMPED,
          "stage 8 anti-windup: the bank switch really happened (%s)",
          Layered_RegimeName(out.regime));
    check(du <= du_bound,
          "stage 8 anti-windup: |du| = %.6f on the switch <= blend_alpha*u_max + slew*dt = %.6f",
          (double)du, (double)du_bound);

    /* and the command must stay put for a while, not ring back */
    for (k = 0; k < 200; ++k) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(t_absf(out.u - u_after) <= 0.05f,
          "stage 8 anti-windup: no post-switch ringing (|u-u_switch| = %.6f after 200 ticks)",
          (double)t_absf(out.u - u_after));
    check(inv.violations == 0,
          "stage 8 anti-windup: the actuator invariants hold across the saturated switch");
}

/* ========================================================================= */
/* Priority chain (one-shot steps)                                           */
/* ========================================================================= */

static void priority_case(const char *what,
                          float ai_regime,
                          float confidence,
                          float age,
                          float residual,
                          float x,
                          layered_gate_state_t expected_gate,
                          layered_regime_t expected_regime)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);

    in.ai_regime = ai_regime;
    in.ai_confidence = confidence;
    in.outer_age_s = age;
    in.physics_residual = residual;
    in.x = x;
    in.x_dot = 0.0f;
    in.r = 0.0f;
    in.r_dot = 0.0f;
    in.u_ff = 0.0f;

    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == expected_gate && out.regime == expected_regime,
          "priority: %-32s -> %-26s / %s",
          what, Layered_GateName(out.gate), Layered_RegimeName(out.regime));
}

static void test_priority(void)
{
    priority_case("safety + ood + conf + stale", 1.0f, 0.20f, 0.50f, 3.0f, 0.95f,
                  GATE_VETO_SAFETY_ENVELOPE, REGIME_SAFE_FALLBACK);
    priority_case("ood + conf + stale", 1.0f, 0.20f, 0.50f, 3.0f, 0.10f,
                  GATE_VETO_OOD_RESIDUAL, REGIME_SAFE_FALLBACK);
    priority_case("conf + stale", 1.0f, 0.20f, 0.50f, 0.10f, 0.10f,
                  GATE_VETO_LOW_CONFIDENCE, REGIME_SAFE_FALLBACK);
    priority_case("stale only", 1.0f, 0.95f, 0.50f, 0.10f, 0.10f,
                  GATE_HOLD_LATENCY, REGIME_SAFE_FALLBACK);
    priority_case("healthy", 1.0f, 0.95f, 0.005f, 0.10f, 0.10f,
                  GATE_NORMAL, REGIME_NOMINAL);
    priority_case("explicit AI abstention (regime 4)", 4.0f, 0.99f, 0.005f, 0.10f, 0.10f,
                  GATE_VETO_LOW_CONFIDENCE, REGIME_SAFE_FALLBACK);
}

/* ========================================================================= */
/* API / init / self-healing                                                 */
/* ========================================================================= */

static void test_api(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int i;
    int integ_zero = 1;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);

    for (i = 0; i < LAYERED_NUM_REGIMES; ++i) {
        if (st.integ[i] != 0.0f) {
            integ_zero = 0;
        }
    }

    check(st.initialized == 1 && st.regime == REGIME_NOMINAL,
          "api: Layered_Init leaves the core initialised in %s", Layered_RegimeName(st.regime));
    check(st.ood_latched == 0 && st.safety_latched == 0 && st.conf_latched == 0 &&
          st.permanent_latched == 0u && st.fault_audit_bitmask == 0u,
          "api: Layered_Init clears the safety/OOD/confidence/permanent latches and the fault audit bitmask");
    check(st.u_applied == 0.0f && st.u_target == 0.0f && st.blend == 0.0f && integ_zero != 0,
          "api: Layered_Init zeroes the command, the blend and every integrator");
    check(st.dwell_ticks == 0u && st.recovery_ticks == 0u && st.step_count == 0u &&
          st.corrupted_frame_count == 0u && st.hold_ticks == 0u &&
          st.last_outer_seq == 0u && st.outer_age_ticks == 0u,
          "api: Layered_Init zeroes the dwell/recovery/step/corruption/hold/sequence counters");
    check(st.safety_veto_count == 0u && st.ood_veto_count == 0u &&
          st.low_confidence_veto_count == 0u && st.latency_hold_count == 0u,
          "api: Layered_Init zeroes every veto counter");

    check(Layered_GateName(GATE_NORMAL) != 0 && Layered_GateName(GATE_HOLD_LATENCY) != 0 &&
          Layered_GateName(GATE_VETO_OOD_RESIDUAL) != 0 &&
          Layered_GateName(GATE_VETO_LOW_CONFIDENCE) != 0 &&
          Layered_GateName(GATE_VETO_SAFETY_ENVELOPE) != 0 &&
          Layered_GateName(GATE_SAFE_ZERO) != 0 &&
          Layered_GateName(GATE_LATCHED_SAFE) != 0 &&
          Layered_RegimeName(REGIME_NOMINAL) != 0 &&
          Layered_RegimeName(REGIME_HIGH_LOAD_DAMPED) != 0 &&
          Layered_RegimeName(REGIME_BOUNDARY_COMPLIANT) != 0 &&
          Layered_RegimeName(REGIME_SAFE_FALLBACK) != 0,
          "api: every gate/regime has a human-readable name");
    printf("       names: %s | %s | %s | %s | %s | %s | %s\n",
           Layered_GateName(GATE_NORMAL), Layered_GateName(GATE_HOLD_LATENCY),
           Layered_GateName(GATE_VETO_OOD_RESIDUAL),
           Layered_GateName(GATE_VETO_LOW_CONFIDENCE),
           Layered_GateName(GATE_VETO_SAFETY_ENVELOPE),
           Layered_GateName(GATE_SAFE_ZERO), Layered_GateName(GATE_LATCHED_SAFE));

    /* self-healing: a caller that forgot Layered_Init() */
    st.initialized = 0;
    st.u_applied = 0.0f;
    st.u_target = 0.0f;
    st.integ[0] = 0.0f;
    st.integ[1] = 0.0f;
    st.integ[2] = 0.0f;
    st.integ[3] = 0.0f;
    st.blend = 0.0f;
    st.regime = REGIME_NOMINAL;
    st.ood_latched = 0;
    st.safety_latched = 0;
    st.conf_latched = 0;
    st.permanent_latched = 0u;
    st.dwell_ticks = 0u;
    st.recovery_ticks = 0u;
    st.fault_audit_bitmask = 0u;
    st.step_count = 0u;
    st.safety_veto_count = 0u;
    st.ood_veto_count = 0u;
    st.low_confidence_veto_count = 0u;
    st.latency_hold_count = 0u;
    st.corrupted_frame_count = 0u;
    st.hold_ticks = 0u;
    st.last_outer_seq = 0u;
    st.outer_age_ticks = 0u;

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.x = 0.0f;
    in.x_dot = 0.0f;
    in.u_ff = 0.0f;
    in.r = 0.1f;
    in.r_dot = 0.0f;

    out = Layered_Step(&st, &cfg, &tun, &in);
    check(st.initialized == 1 && out.gate == GATE_NORMAL && out.regime == REGIME_NOMINAL &&
          out.corrupted == 0,
          "api: Layered_Step self-heals a state whose initialized flag was lost");

    /* and a null-pointer call must not crash */
    out = Layered_Step(0, 0, 0, 0);
    check(out.u == 0.0f && out.gate == GATE_VETO_SAFETY_ENVELOPE && out.corrupted == 0,
          "api: a null-pointer call returns the fail-safe output instead of crashing");
}

/* ========================================================================= */
/* Stage 10: deterministic feedforward u_ff                                  */
/* ========================================================================= */

/* Runs a zero-error steady scenario with a constant feedforward and returns the
 * settled command.  The final state/last output are handed back so the caller
 * can re-derive the bank equations. */
static float run_ff_steady(const LayeredConfig_t *cfg,
                           const FieldTuning_t *tun,
                           float u_ff,
                           int n_steps,
                           LayeredState_t *st_out,
                           LayeredOutput_t *out_last,
                           inv_t *inv)
{
    LayeredState_t st;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int k;

    Layered_Init(&st, cfg, tun);

    /* Zero tracking error: the ONLY thing that moves the command is u_ff. */
    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.r = 0.20f;
    in.r_dot = 0.0f;
    in.x = 0.20f;
    in.x_dot = 0.0f;
    in.u_ff = u_ff;

    for (k = 0; k < n_steps; ++k) {
        out = step_checked(inv, &st, cfg, tun, &in);
    }
    *st_out = st;
    *out_last = out;
    return out.u;
}

static void test_feedforward(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    inv_t inv;
    LayeredOutput_t out;
    int i;
    float u_no_ff;
    float u_ff_small;
    float u_ff_big;
    float e;
    float ed;
    int track_ok = 1;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);

    /* run A: no feedforward */
    inv_reset(&inv);
    u_no_ff = run_ff_steady(&cfg, &tun, 0.0f, 600, &st, &out, &inv);
    check(inv.violations == 0 && t_absf(u_no_ff) <= 1e-6f,
          "stage 10 feedforward: with u_ff=0 and zero error the command settles at 0 (%.6f)",
          (double)u_no_ff);

    /* run B: u_ff inside the per-tick slew capability -> exact identity */
    inv_reset(&inv);
    u_ff_small = run_ff_steady(&cfg, &tun, 0.15f, 600, &st, &out, &inv);
    check(t_absf((u_ff_small - u_no_ff) - 0.15f) <= 1e-5f,
          "stage 10 feedforward: u_ff=0.15 shifts the steady command by %.6f (error %.8f <= 1e-5)",
          (double)(u_ff_small - u_no_ff), (double)t_absf((u_ff_small - u_no_ff) - 0.15f));

    /* run C: u_ff needs two slew-limited ticks -> the integrator absorbs that
     * startup transient (normal for tracking anti-windup), so the offset is
     * u_ff up to a small residual. */
    check(inv.violations == 0,
          "stage 10 feedforward: the actuator invariants hold on the middle feedforward level (u_ff=0.15)");
    inv_reset(&inv);
    u_ff_big = run_ff_steady(&cfg, &tun, 0.35f, 600, &st, &out, &inv);
    check(t_absf((u_ff_big - u_no_ff) - 0.35f) <= 0.01f,
          "stage 10 feedforward: u_ff=0.35 shifts the steady command by %.6f (error %.6f <= 0.01)",
          (double)(u_ff_big - u_no_ff), (double)t_absf((u_ff_big - u_no_ff) - 0.35f));
    check(t_absf(u_no_ff) <= cfg.u_max && t_absf(u_ff_small) <= cfg.u_max &&
          t_absf(u_ff_big) <= cfg.u_max,
          "stage 10 feedforward: |u| stays inside u_max=%.2f for every feedforward level (%.5f / %.5f / %.5f)",
          (double)cfg.u_max, (double)u_no_ff, (double)u_ff_small, (double)u_ff_big);

    /* Normal-bank anti-windup includes the deterministic feedforward. */
    /* the scenario keeps r == x, so both tracking errors are exactly zero */
    e = 0.0f;
    ed = 0.0f;
    for (i = 0; i < 3; ++i) {
        float u_pid = (cfg.kp[i] * tun.gain_scale) * e +
                      (cfg.kd[i] * tun.damping_scale) * ed +
                      (cfg.ki[i] * tun.gain_scale) * st.integ[i] +
                      0.35f;
        if (t_absf(u_pid - out.u) > 0.05f) {
            track_ok = 0;
        }
    }
    check(track_ok != 0,
          "stage 10 feedforward: normal bank outputs include u_ff and track the applied command within 0.05");
    check(inv.violations == 0,
          "stage 10 feedforward: the actuator invariants hold with feedforward enabled");
}

/* ========================================================================= */
/* Stage 11: sticky confidence veto                                          */
/* ========================================================================= */

static void test_confidence_latch(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    inv_t inv;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int k;

    build_config(&cfg); /* min_ai_confidence 0.45 / confidence_clear_ratio 0.55 */
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    inv_reset(&inv);

    in.ai_regime = 1.0f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.r = 0.20f;
    in.r_dot = 0.0f;
    in.x = 0.20f;
    in.x_dot = 0.0f;
    in.u_ff = 0.0f;

    /* 1) healthy: 0.60 >= min 0.45 -> accepted */
    in.ai_confidence = 0.60f;
    for (k = 0; k < 50; ++k) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_NORMAL && st.conf_latched == 0,
          "stage 11 confidence: confidence 0.60 is accepted, no veto");

    /* 2) 0.40 < min 0.45 -> veto latches */
    in.ai_confidence = 0.40f;
    for (k = 0; k < 20; ++k) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_VETO_LOW_CONFIDENCE && st.conf_latched == 1 &&
          out.regime == REGIME_SAFE_FALLBACK,
          "stage 11 confidence: 0.40 < 0.45 trips GATE_VETO_LOW_CONFIDENCE / REGIME_SAFE_FALLBACK");

    /* 3) 0.50 is above the trip level but below the clear level 0.55 */
    in.ai_confidence = 0.50f;
    for (k = 0; k < 20; ++k) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_VETO_LOW_CONFIDENCE && st.conf_latched == 1,
          "stage 11 confidence: 0.50 is above the trip level but below 0.55, the veto is HELD (hysteresis)");

    /* 4) 0.56 >= clear 0.55 with a normal regime -> latch clears */
    in.ai_confidence = 0.56f;
    for (k = 0; k < 20; ++k) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_NORMAL && st.conf_latched == 0,
          "stage 11 confidence: 0.56 >= 0.55 clears the latch, the gate returns to GATE_NORMAL");

    /* 5) explicit abstention: ai_regime 4 vetoes even at confidence 1.0 */
    in.ai_regime = 4.0f;
    in.ai_confidence = 1.0f;
    for (k = 0; k < 20; ++k) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_VETO_LOW_CONFIDENCE && st.conf_latched == 1,
          "stage 11 confidence: ai_regime 4 abstention vetoes even with confidence 1.0");

    /* 6) normal regime again + high confidence -> clears */
    in.ai_regime = 2.0f;
    in.ai_confidence = 1.0f;
    for (k = 0; k < 20; ++k) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_NORMAL && st.conf_latched == 0,
          "stage 11 confidence: a normal regime plus high confidence clears the latch again");

    check(st.low_confidence_veto_count == 2u,
          "stage 11 confidence: low_confidence_veto_count counts exactly 2 episodes (%u)",
          (unsigned)st.low_confidence_veto_count);
    check(inv.violations == 0,
          "stage 11 confidence: the actuator invariants hold through the whole latch sequence");
}

/* ========================================================================= */
/* main                                                                      */
/* ========================================================================= */

/* ========================================================================= */
/* Stage 12: first clean control step snap                                   */
/* ========================================================================= */

static void test_first_control_step(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    inv_t inv;
    LayeredInput_t in = zero_input();
    LayeredInput_t bad = zero_input();
    LayeredOutput_t out;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    inv_reset(&inv);

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.r = 0.10f;
    in.r_dot = 0.0f;
    in.x = 0.0f;
    in.x_dot = 0.0f;
    in.u_ff = 0.0f;

    /* A corrupt first frame must not consume the snap. */
    Layered_Init(&st, &cfg, &tun);
    bad = in;
    bad.x = make_nan();
    out = Layered_Step(&st, &cfg, &tun, &bad);
    check(out.corrupted == 1 && st.control_started == 0,
          "stage 12 first step: a corrupt first frame is rejected and does not consume the snap");

    out = step_checked(&inv, &st, &cfg, &tun, &in);
    check(out.blend == 1.0f && st.control_started == 1 && out.u == 0.0f &&
          out.regime == REGIME_SAFE_FALLBACK,
          "stage 12 first step: the first clean tick snaps to zero-state R4 during recovery (u=%.6f)",
          (double)out.u);
    check(inv.violations == 0,
          "stage 12 first step: the actuator invariants hold on the first recovered tick");

    /* A normal first frame also snaps. */
    Layered_Init(&st, &cfg, &tun);
    inv_reset(&inv);
    out = step_checked(&inv, &st, &cfg, &tun, &in);
    check(out.blend == 1.0f && st.control_started == 1 && out.u > 0.0f,
          "stage 12 first step: a healthy first frame snaps the command (u=%.6f)",
          (double)out.u);
    check(inv.violations == 0,
          "stage 12 first step: actuator invariants hold on the snap tick");
}

/* ========================================================================= */
/* Stage 12b: latency / corrupt frames re-arm the recovery cooldown          */
/* ========================================================================= */

static void test_cooldown_latency(void)
{
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredState_t st;
    inv_t inv;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int i;
    int fallback_ticks = 0;
    int fallback_bad = 0;
    int switched_at = -1;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    inv_reset(&inv);

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.r = 0.0f;
    in.r_dot = 0.0f;
    in.x = 0.0f;
    in.x_dot = 0.0f;
    in.u_ff = 0.0f;

    for (i = 0; i < 200; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }

    /* OOD burst, then a few clean ticks. */
    in.physics_residual = 3.0f;
    for (i = 0; i < 5; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    in.physics_residual = 0.10f;
    for (i = 0; i < 50; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(st.recovery_ticks > 0u && st.recovery_ticks < cfg.ood_recovery_cooldown_ticks,
          "stage 12b cooldown: 50 clean ticks consume part of the cooldown (%u left)",
          (unsigned)st.recovery_ticks);

    /* A latency hold must re-arm the full cooldown. */
    in.outer_age_s = 0.400f;
    for (i = 0; i < 3; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(st.recovery_ticks == cfg.ood_recovery_cooldown_ticks &&
          out.gate == GATE_HOLD_LATENCY,
          "stage 12b cooldown: 3 stale ticks re-arm the full %u-tick cooldown",
          (unsigned)cfg.ood_recovery_cooldown_ticks);

    /* Now count the safe fallback ticks; the AI must be re-armed only after
     * the full cooldown of consecutive clean samples. */
    in.outer_age_s = 0.005f;
    in.ai_regime = 2.0f;
    for (i = 0; i < (int)cfg.ood_recovery_cooldown_ticks + 20; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
        if (switched_at < 0 && out.regime == REGIME_HIGH_LOAD_DAMPED) {
            switched_at = i;
        }
        if (i < (int)cfg.ood_recovery_cooldown_ticks) {
            if (out.gate == GATE_NORMAL && out.regime == REGIME_SAFE_FALLBACK) {
                fallback_ticks++;
            } else {
                fallback_bad++;
            }
        }
    }
    check(fallback_bad == 0 && fallback_ticks == (int)cfg.ood_recovery_cooldown_ticks,
          "stage 12b cooldown: %d/%u clean ticks stay in REGIME_SAFE_FALLBACK",
          fallback_ticks, (unsigned)cfg.ood_recovery_cooldown_ticks);
    check(switched_at == (int)cfg.ood_recovery_cooldown_ticks,
          "stage 12b cooldown: the AI bank is re-armed on tick %d after the last stale tick",
          switched_at);
    check(inv.violations == 0,
          "stage 12b cooldown: the actuator invariants hold across the stale-frame recovery");
}

/* ========================================================================= */
/* Stage 12c: per-regime feedforward scale                                   */
/* ========================================================================= */

static void test_ff_scale(void)
{
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredState_t st;
    inv_t inv;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int i;

    build_config(&cfg);
    cfg.ff_scale[0] = 0.5f;
    cfg.ff_scale[1] = 0.25f;
    /* Disable the integrators so the test observes the feedforward gain
     * directly, without the (correct) inactive-bank tracking term. */
    cfg.ki[0] = 0.0f;
    cfg.ki[1] = 0.0f;
    cfg.ki[2] = 0.0f;
    cfg.ki[3] = 0.0f;
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    inv_reset(&inv);

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.r = 0.40f;
    in.r_dot = 0.0f;
    in.x = 0.40f;
    in.x_dot = 0.0f;
    in.u_ff = 0.40f;

    for (i = 0; i < 100; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(t_absf(out.u - 0.20f) < 0.02f,
          "stage 12c ff_scale: bank 1 applies 0.5*u_ff=0.20 (u=%.5f)",
          (double)out.u);

    /* Request bank 2 and wait out the 150-tick dwell. */
    in.ai_regime = 2.0f;
    for (i = 0; i < 250; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(t_absf(out.u - 0.10f) < 0.02f,
          "stage 12c ff_scale: bank 2 applies 0.25*u_ff=0.10 (u=%.5f)",
          (double)out.u);
    check(inv.violations == 0,
          "stage 12c ff_scale: the actuator invariants hold across the per-bank scaling");
}

/* Finite configuration / field tuning and NaN arithmetic at the clamp. */
static void test_settings_and_clamp(void)
{
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredState_t st;
    LayeredInput_t in = {1.0f, 0.95f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0u};
    LayeredOutput_t out;
    float *fields[34];
    int n = 0, i, j, rejected = 0, fallback = 0;
    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
#define FIELD(name) fields[n++] = &cfg.name
    FIELD(dt_s); FIELD(outer_timeout_s); FIELD(max_physics_residual);
    FIELD(residual_clear_ratio); FIELD(min_ai_confidence); FIELD(confidence_clear_ratio);
    FIELD(state_safety_limit); FIELD(safety_trip_ratio); FIELD(safety_clear_ratio);
    FIELD(safety_predict_s); FIELD(u_max); FIELD(u_slew_max_per_s);
    FIELD(blend_alpha); FIELD(integ_limit); FIELD(windup_tau_s);
#undef FIELD
    for (i = 0; i < 4; ++i) {
        fields[n++] = &cfg.kp[i]; fields[n++] = &cfg.kd[i];
        fields[n++] = &cfg.ki[i]; fields[n++] = &cfg.ff_scale[i];
    }
    fields[n++] = &tun.gain_scale;
    fields[n++] = &tun.damping_scale;
    fields[n++] = &tun.safety_margin_scale;
    for (i = 0; i < n; ++i) {
        float saved = *fields[i];
        for (j = 0; j < 2; ++j) {
            float held, integral;
            Layered_Init(&st, &cfg, &tun);
            out = Layered_Step(&st, &cfg, &tun, &in);
            held = out.u; integral = st.integ[0];
            *fields[i] = j ? make_inf() : make_nan();
            out = Layered_Step(&st, &cfg, &tun, &in);
            if (i < 31) {
                rejected += out.corrupted && out.gate == GATE_HOLD_LATENCY &&
                    out.u == held && st.integ[0] == integral &&
                    (out.fault_audit_bitmask & LAYERED_FAULT_BIT_NAN_CORRUPTION) != 0;
            } else {
                /* A NaN-only test (out.u == out.u) would let a +/-Inf
                 * regression escape; every published value of the fallback
                 * tick must be finite, not merely non-NaN. */
                fallback += out.corrupted && out.regime == REGIME_SAFE_FALLBACK &&
                    out.gate == GATE_VETO_LOW_CONFIDENCE &&
                    t_is_finite(out.u) && t_is_finite(out.u_target) &&
                    t_is_finite(out.blend) &&
                    (out.fault_audit_bitmask & LAYERED_FAULT_BIT_NAN_CORRUPTION) != 0;
            }
            *fields[i] = saved;
        }
    }
    check(rejected == 62, "config firewall: 31 floating fields reject NaN and Inf while holding state");
    check(fallback == 6, "tuning firewall: three NaN/Inf knobs enter closed-loop R4");
    check(sizeof(LayeredState_t) == 96 && sizeof(LayeredConfig_t) == 140 &&
          sizeof(FieldTuning_t) == 12, "binary contract: state/config/tuning = 96/140/12 bytes");
    Layered_Init(&st, &cfg, &tun);
    st.control_started = 1; st.u_applied = 1.0f; st.u_target = 1.0f;
    cfg.kp[0] = 1.0e38f; cfg.kd[0] = -1.0e38f;
    in.r = 100.0f; in.r_dot = 100.0f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    /* Inf - Inf in the bank sum is not a value the core may store: the bank
     * output is sanitized to 0, the blend keeps the target finite and the slew
     * limit still decides the applied command. */
    check(t_is_finite(out.u_target) && t_is_finite(out.u) &&
          out.u == out.u_target && t_absf(out.u_target - 0.98f) <= 1.0e-6f,
          "NaN arithmetic: an overflowing bank sum keeps a finite target (%.6f) and command (%.6f)",
          (double)out.u_target, (double)out.u);
    build_config(&cfg);
    in.r = 0.1f; in.r_dot = 0.0f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.u_target > 0.0f && out.u_target < cfg.u_max && out.u > 0.0f,
          "NaN recovery: normal arithmetic restores a finite positive target and command");
}

/* Slow mailbox faults retain fast feedback and safety priority. */
static void test_mailbox_fallback(void)
{
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredState_t st;
    LayeredInput_t good = {1.0f, 0.95f, 0.0f, 0.0f, 0.2f, 0.1f, 20.0f, 5.0f, 100.0f, 0u};
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int i, k, accepted = 0, continuous = 0, cooldown = 0;
    float peak, velocity;
    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    for (i = 0; i < 12; ++i) {
        in = good;
        switch (i) {
        case 0: in.ai_regime = make_nan(); break;
        case 1: in.ai_regime = make_inf(); break;
        case 2: in.ai_regime = 0.0f; break;
        case 3: in.ai_regime = 5.0f; break;
        case 4: in.ai_confidence = make_nan(); break;
        case 5: in.ai_confidence = make_inf(); break;
        case 6: in.ai_confidence = -0.1f; break;
        case 7: in.ai_confidence = 1.1f; break;
        case 8: in.outer_age_s = make_nan(); break;
        case 9: in.outer_age_s = make_inf(); break;
        case 10: in.outer_age_s = -0.1f; break;
        default: in.ai_regime = -make_inf(); break;
        }
        Layered_Init(&st, &cfg, &tun);
        out = Layered_Step(&st, &cfg, &tun, &in);
        accepted += out.corrupted && out.regime == REGIME_SAFE_FALLBACK &&
            out.gate == GATE_VETO_LOW_CONFIDENCE && out.u < 0.0f &&
            st.corrupted_frame_count == 1u &&
            (out.fault_audit_bitmask & LAYERED_FAULT_BIT_NAN_CORRUPTION) != 0u;
        for (k = 0; k < 20; ++k) out = Layered_Step(&st, &cfg, &tun, &in);
        continuous += st.step_count == 21u && st.corrupted_frame_count == 21u &&
            out.u < 0.0f && st.integ[3] < 0.0f &&
            st.recovery_ticks == cfg.ood_recovery_cooldown_ticks;
    }
    check(accepted == 12, "mailbox firewall: all malformed slow fields enter R4 with valid feedback");
    check(continuous == 12, "mailbox firewall: repeated faults keep integrating R4 and re-arm cooldown");
    in = good; in.ai_confidence = make_nan(); in.x = 1.1f;
    Layered_Init(&st, &cfg, &tun);
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.corrupted && out.gate == GATE_VETO_SAFETY_ENVELOPE && out.u < 0.0f,
          "mailbox firewall: envelope priority survives corrupt AI and an outward reference");
    in.x = good.x; in.physics_residual = 2.0f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.corrupted && out.gate == GATE_VETO_OOD_RESIDUAL && out.u < 0.0f,
          "mailbox firewall: independent OOD priority survives corrupt AI");
    in = good; in.ai_regime = 3.0f;
    for (k = 0; k < (int)cfg.ood_recovery_cooldown_ticks; ++k) {
        out = Layered_Step(&st, &cfg, &tun, &in);
        cooldown += out.regime == REGIME_SAFE_FALLBACK && out.gate == GATE_NORMAL && !out.corrupted;
    }
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(cooldown == (int)cfg.ood_recovery_cooldown_ticks &&
          out.regime == REGIME_BOUNDARY_COMPLIANT && out.gate == GATE_NORMAL,
          "mailbox firewall: normal R3 resumes only after the full clean cooldown");
    cfg.ki[3] = 0.0f;
    in = good; in.ai_confidence = make_nan(); in.x = 0.4f; in.x_dot = 0.0f;
    Layered_Init(&st, &cfg, &tun);
    st.control_started = 1; st.u_applied = 1.34f; st.u_target = 1.34f;
    peak = in.x; velocity = 0.0f;
    for (k = 0; k < 5000; ++k) {
        out = Layered_Step(&st, &cfg, &tun, &in);
        velocity += cfg.dt_s * (out.u - 0.2f * velocity) / 0.1f;
        in.x += cfg.dt_s * velocity; in.x_dot = velocity;
        if (t_absf(in.x) > peak) peak = t_absf(in.x);
    }
    check(peak < 1.0f && t_absf(in.x) < 0.4f && out.corrupted && st.step_count == 5000u,
          "mailbox firewall: persistent NaN AI retains a bounded closed-loop return (peak=%.4f)",
          (double)peak);
}

/* Physical-domain settings bounds and unchanged binary layout. */
static void test_settings_bounds(void)
{
    LayeredConfig_t cfg, valid;
    FieldTuning_t tun;
    LayeredState_t st;
    LayeredInput_t in = {1.0f, 0.95f, 0.0f, 0.0f, -0.02f, -0.03f, -0.05f, -0.01f, -0.01f, 0u};
    LayeredOutput_t out;
    float *knobs[3];
    float upper[3] = {4.0f, 4.0f, 1.0f};
    int rejected = 0, bounds = 0, i, j;
    build_config(&valid); cfg = valid;
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
#define BAD_SETTING(name, value) do { cfg = valid; cfg.name = (value); \
    Layered_Init(&st, &cfg, &tun); out = Layered_Step(&st, &cfg, &tun, &in); \
    rejected += out.corrupted && out.gate == GATE_HOLD_LATENCY; } while (0)
    BAD_SETTING(dt_s, 0.0f); BAD_SETTING(dt_s, -0.001f);
    BAD_SETTING(outer_timeout_s, 0.0f); BAD_SETTING(max_physics_residual, 0.0f);
    BAD_SETTING(state_safety_limit, 0.0f); BAD_SETTING(u_max, 0.0f);
    BAD_SETTING(u_slew_max_per_s, 0.0f); BAD_SETTING(blend_alpha, 0.0f);
    BAD_SETTING(blend_alpha, 1.01f); BAD_SETTING(integ_limit, -1.0f);
    BAD_SETTING(windup_tau_s, -1.0f); BAD_SETTING(safety_predict_s, -1.0f);
    BAD_SETTING(residual_clear_ratio, -0.01f); BAD_SETTING(residual_clear_ratio, 1.01f);
    BAD_SETTING(min_ai_confidence, -0.01f); BAD_SETTING(min_ai_confidence, 1.01f);
    BAD_SETTING(confidence_clear_ratio, 0.44f); BAD_SETTING(confidence_clear_ratio, 1.01f);
    BAD_SETTING(safety_trip_ratio, 0.0f); BAD_SETTING(safety_trip_ratio, 1.01f);
    BAD_SETTING(safety_clear_ratio, -0.01f); BAD_SETTING(safety_clear_ratio, 0.91f);
    BAD_SETTING(max_hold_ticks, 0u);
#undef BAD_SETTING
    check(rejected == 23, "settings domains: nonphysical timing, limits, smoothing, ratios and a zero hold budget are rejected");
    cfg = valid;
    knobs[0] = &tun.gain_scale; knobs[1] = &tun.damping_scale; knobs[2] = &tun.safety_margin_scale;
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 4; ++j) {
            *knobs[i] = j == 0 ? 0.10f : j == 1 ? upper[i] : j == 2 ? 0.099f : upper[i] + 0.001f;
            Layered_Init(&st, &cfg, &tun);
            out = Layered_Step(&st, &cfg, &tun, &in);
            bounds += j < 2 ? !out.corrupted :
                out.corrupted && out.regime == REGIME_SAFE_FALLBACK && out.gate != GATE_HOLD_LATENCY;
        }
        *knobs[i] = 1.0f;
    }
    check(bounds == 12, "field knobs: gain/damping [0.10,4], margin [0.10,1] include endpoints only");
    cfg.blend_alpha = 1.0f; cfg.residual_clear_ratio = 0.0f; cfg.integ_limit = 0.0f;
    cfg.windup_tau_s = 0.0f; cfg.safety_predict_s = 0.0f; cfg.safety_clear_ratio = 0.0f;
    cfg.safety_trip_ratio = 1.0f; cfg.min_ai_confidence = 0.0f; cfg.confidence_clear_ratio = 1.0f;
    Layered_Init(&st, &cfg, &tun);
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(!out.corrupted && out.gate == GATE_NORMAL,
          "settings domains: allowed boundary settings and signed state/reference/feedforward are valid");
    in.physics_residual = -0.01f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.corrupted && out.gate == GATE_HOLD_LATENCY,
          "fast firewall: residual alone must be nonnegative");
    check(sizeof(LayeredState_t) == 96 && sizeof(LayeredConfig_t) == 140 &&
          sizeof(FieldTuning_t) == 12 && sizeof(LayeredInput_t) == 40 && sizeof(LayeredOutput_t) == 28,
          "binary contract: state/config/tuning/input/output = 96/140/12/40/28 bytes");
}

static void test_safe_bank_and_r3(void)
{
    LayeredConfig_t cfg;
    FieldTuning_t low, high;
    LayeredState_t a, b;
    LayeredInput_t ia = {4.0f, 1.0f, 0.0f, 0.0f, 0.2f, 0.1f, 20.0f, 10.0f, 100.0f, 0u};
    LayeredInput_t ib = zero_input();
    LayeredOutput_t oa, ob;
    float expected;
    int k, same = 1;
    build_config(&cfg);
    build_tuning(&low, 0.10f, 0.10f, 1.0f); build_tuning(&high, 4.0f, 4.0f, 1.0f);
    cfg.ff_scale[3] = 4.0f;
    ib = ia; ib.r = -20.0f; ib.r_dot = -10.0f; ib.u_ff = -100.0f;
    Layered_Init(&a, &cfg, &low); Layered_Init(&b, &cfg, &high);
    for (k = 0; k < 100; ++k) {
        oa = Layered_Step(&a, &cfg, &low, &ia); ob = Layered_Step(&b, &cfg, &high, &ib);
        same &= oa.u == ob.u && oa.u_target == ob.u_target && a.integ[3] == b.integ[3];
        if (k == 0) {
            check(t_absf(a.integ[3] + ia.x * cfg.dt_s) < 1.0e-7f,
                  "R4 anti-windup: fixed gain and zero-state error define the safe integral");
        }
    }
    check(same && oa.u < 0.0f, "R4 authority: knobs, unsafe references and nonzero ff_scale cannot change fallback");
    ia.ai_regime = 1.0f; ia.x = 1.1f; ia.physics_residual = 0.0f;
    Layered_Init(&a, &cfg, &high); oa = Layered_Step(&a, &cfg, &high, &ia);
    check(oa.gate == GATE_VETO_SAFETY_ENVELOPE && oa.u < 0.0f,
          "R4 envelope: outward trajectory demand cannot override zero-state return");
    build_config(&cfg); cfg.min_dwell_ticks = 0;
    build_tuning(&low, 1.5f, 0.75f, 1.0f);
    ia.ai_regime = 3.0f; ia.x = 0.1f; ia.x_dot = 0.03f; ia.r = 0.2f; ia.r_dot = 0.05f; ia.u_ff = 0.15f;
    Layered_Init(&a, &cfg, &low); oa = Layered_Step(&a, &cfg, &low, &ia);
    expected = cfg.kp[2]*low.gain_scale*(ia.r-ia.x) +
        cfg.kd[2]*low.damping_scale*(ia.r_dot-ia.x_dot) + cfg.ff_scale[2]*ia.u_ff;
    check(oa.regime == REGIME_BOUNDARY_COMPLIANT && oa.gate == GATE_NORMAL &&
          t_absf(oa.u_target - expected) < 1.0e-7f && oa.u > 0.0f,
          "R3 controller: compliant bank executes its own scaled gains and feedforward");
}

/* Runtime tuning faults use nominal internal gains and retain closed-loop R4. */
static void test_runtime_tuning_faults(void)
{
    LayeredConfig_t cfg;
    FieldTuning_t tun, nominal;
    LayeredState_t st, reference;
    LayeredInput_t in = zero_input(), ref_in = zero_input();
    LayeredOutput_t out, ref_out;
    int fault, k, bank;
    build_config(&cfg);
    build_tuning(&nominal, 1.0f, 1.0f, 1.0f);
    for (fault = 0; fault < 4; ++fault) {
        int holds = 0, safe = 0, violations = 0, polluted = 0, recovery = 0;
        float peak = 0.0f;
        tun = nominal;
        in.ai_regime = 1.0f; in.ai_confidence = 0.95f; in.outer_age_s = 0.0f;
        in.physics_residual = 0.0f; in.x = 0.0f; in.x_dot = 0.0f;
        in.r = 0.4f; in.r_dot = 0.0f; in.u_ff = 0.0f;
        Layered_Init(&st, &cfg, &tun);
        for (k = 0; k < 10000; ++k) {
            if (k == 5000) {
                reference = st;
                if (fault == 0) tun.gain_scale = 5.0f;
                else if (fault == 1) tun.gain_scale = -1.0f;
                else if (fault == 2) tun.gain_scale = make_nan();
                else tun.safety_margin_scale = 1.5f;
            }
            if (k == 7500) tun = nominal;
            out = Layered_Step(&st, &cfg, &tun, &in);
            if (k >= 5000 && k < 7500) {
                ref_in = in; ref_in.ai_regime = 4.0f; ref_in.ai_confidence = 0.0f;
                ref_out = Layered_Step(&reference, &cfg, &nominal, &ref_in);
                holds += out.gate == GATE_HOLD_LATENCY;
                safe += out.regime == REGIME_SAFE_FALLBACK && out.corrupted;
                polluted += out.u != ref_out.u || out.u_target != ref_out.u_target;
                for (bank = 0; bank < 4; ++bank) {
                    polluted += st.integ[bank] != reference.integ[bank] ||
                        t_absf(st.integ[bank]) > cfg.integ_limit;
                }
            }
            if (k >= 7500 && k < 8000) recovery +=
                out.regime == REGIME_SAFE_FALLBACK && !out.corrupted && out.gate == GATE_NORMAL;
            in.x_dot += cfg.dt_s * (out.u - 0.2f * in.x_dot) / 0.1f;
            in.x += cfg.dt_s * in.x_dot;
            if (t_absf(in.x) > peak) peak = t_absf(in.x);
            /* A non-finite plant state is a violation in its own right: test it
             * with t_is_finite so a +/-Inf regression cannot slip through a
             * NaN-only comparison. */
            violations += !t_is_finite(in.x) || t_absf(in.x) > cfg.state_safety_limit;
        }
        check(holds == 0 && safe == 2500 && violations == 0,
              "runtime tuning case %d: fault at 5 s gives 0 HOLD, 2500 R4 ticks and 0 breaches (peak=%.4f)",
              fault + 1, (double)peak);
        check(polluted == 0, "runtime tuning case %d: every bank integral matches nominal-knob R4", fault + 1);
        check(recovery == 500 && out.regime == REGIME_NOMINAL && !out.corrupted &&
              st.corrupted_frame_count == 2500u &&
              (out.fault_audit_bitmask & LAYERED_FAULT_BIT_NAN_CORRUPTION) != 0u,
              "runtime tuning case %d: restored knobs recover after the full clean cooldown", fault + 1);
    }
    tun = nominal; tun.safety_margin_scale = 1.5f; in.x = 0.95f; in.x_dot = 0.0f;
    Layered_Init(&st, &cfg, &tun);
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.corrupted && out.gate == GATE_VETO_SAFETY_ENVELOPE && out.regime == REGIME_SAFE_FALLBACK,
          "runtime tuning: invalid margin uses the nominal envelope and preserves veto priority");
    in.x = make_nan();
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.corrupted && out.gate == GATE_HOLD_LATENCY,
          "runtime tuning: invalid fast measurement still takes the HOLD firewall");
}

static void test_counter_boundaries(void)
{
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredState_t st;
    LayeredInput_t in = {1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0u};
    LayeredInput_t bad = zero_input();
    LayeredOutput_t out;
    int i, immediate = 0;
    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    st.dwell_ticks = UINT32_MAX - 1u; st.step_count = UINT32_MAX - 1u;
    for (i = 0; i < 3; ++i) out = Layered_Step(&st, &cfg, &tun, &in);
    check(st.dwell_ticks == UINT32_MAX && st.step_count == UINT32_MAX && out.gate == GATE_NORMAL,
          "counter boundary: normal dwell and tick counts saturate without changing control");
    in.ai_regime = 2.0f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.regime == REGIME_HIGH_LOAD_DAMPED && st.dwell_ticks == 0u,
          "counter boundary: saturated dwell still permits a normal bank switch");
    for (i = 0; i < 2; ++i) {
        cfg.min_dwell_ticks = (uint32_t)i;
        Layered_Init(&st, &cfg, &tun);
        out = Layered_Step(&st, &cfg, &tun, &in);
        immediate += out.regime == REGIME_HIGH_LOAD_DAMPED;
    }
    check(immediate == 2, "counter boundary: dwell thresholds zero and one switch on the first tick");
    cfg.min_dwell_ticks = UINT32_MAX;
    Layered_Init(&st, &cfg, &tun); st.dwell_ticks = UINT32_MAX - 2u;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.regime == REGIME_NOMINAL && st.dwell_ticks == UINT32_MAX - 1u,
          "counter boundary: maximum dwell threshold waits until its final tick");
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.regime == REGIME_HIGH_LOAD_DAMPED && st.dwell_ticks == 0u,
          "counter boundary: maximum dwell threshold switches at MAX minus one");
    Layered_Init(&st, &cfg, &tun); st.dwell_ticks = UINT32_MAX;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.regime == REGIME_HIGH_LOAD_DAMPED,
          "counter boundary: maximum dwell threshold accepts a saturated count");

    build_config(&cfg); cfg.ood_recovery_cooldown_ticks = 0u;
    /* This block exercises saturating audit counters and the original gate
     * priority chain; the cumulative permanent latch has its own dedicated
     * regression below, so it is disabled here to keep the counters/bitmask
     * expectations focused on saturation arithmetic. */
    cfg.max_fault_vetoes_before_latch = 0u;
    in.ai_regime = 1.0f; bad = in;
    bad.ai_regime = make_nan(); bad.x = 1.1f; bad.physics_residual = 2.0f;
    bad.outer_age_s = cfg.outer_timeout_s + cfg.dt_s * 0.5f;
    Layered_Init(&st, &cfg, &tun);
    st.step_count = st.corrupted_frame_count = UINT32_MAX - 1u;
    st.safety_veto_count = st.ood_veto_count = UINT32_MAX - 1u;
    st.low_confidence_veto_count = st.latency_hold_count = UINT32_MAX - 1u;
    for (i = 0; i < 3; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &bad);
        if (i < 2) (void)Layered_Step(&st, &cfg, &tun, &in);
    }
    check(st.step_count == UINT32_MAX && st.corrupted_frame_count == UINT32_MAX &&
          st.safety_veto_count == UINT32_MAX && st.ood_veto_count == UINT32_MAX &&
          st.low_confidence_veto_count == UINT32_MAX,
          "counter boundary: repeated fault episodes saturate all diagnostic counters");
    /* The fault frame is veto-masked (envelope outranks the latency HOLD), so
     * it must not count a native-HOLD entry any more. */
    check(st.latency_hold_count == UINT32_MAX - 1u,
          "counter boundary: a veto-masked stale frame does not count a HOLD episode");
    check(out.gate == GATE_VETO_SAFETY_ENVELOPE && out.regime == REGIME_SAFE_FALLBACK &&
          out.corrupted && out.fault_audit_bitmask == 31u,
          "counter boundary: saturation preserves gate priority and all sticky fault bits");
    bad.x = make_nan();
    for (i = 0; i < 2; ++i) out = Layered_Step(&st, &cfg, &tun, &bad);
    check(out.gate == GATE_HOLD_LATENCY && st.step_count == UINT32_MAX &&
          st.corrupted_frame_count == UINT32_MAX,
          "counter boundary: fast-input firewall counters also remain saturated");

    /* latency_hold_count counts native-HOLD entry edges, so its saturation is
     * covered separately with a pure stale mailbox that no veto masks. */
    Layered_Init(&st, &cfg, &tun);
    st.latency_hold_count = UINT32_MAX - 1u;
    in.ai_regime = 1.0f; in.ai_confidence = 0.95f;
    in.x = 0.0f; in.physics_residual = 0.0f;
    in.outer_age_s = cfg.outer_timeout_s + cfg.dt_s * 0.5f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_HOLD_LATENCY && st.latency_hold_count == UINT32_MAX &&
          st.hold_ticks == 1u,
          "counter boundary: one native-HOLD entry saturates latency_hold_count");
}

/* ========================================================================= */
/* Phase A1: bounded native HOLD -> mandatory safe de-energise               */
/* ========================================================================= */

static void test_hold_timeout(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    LayeredConfig_t bad_cfg;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    inv_t inv;
    int i;
    int hold_bad = 0;
    int hold_motion = 0;
    int integ_zero = 1;
    int zero_bad = 0;
    int zero_step = -1;
    float prev_u;
    float du_max;
    /* Fixed HOLD budget the core uses when the configuration itself is
     * rejected; the header documents the value, so mirror it here. */
    const uint32_t corrupt_config_hold_ticks = 100u;

    build_config(&cfg); /* max_hold_ticks == 100, max_fault_vetoes_before_latch == 3 */
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.r = 0.4f;
    in.r_dot = 0.0f;
    in.x = 0.20f;
    in.x_dot = 0.0f;
    in.u_ff = 0.0f;
    in.outer_seq = 0u;

    /* A1a: a corrupt fast sensor must hold the last command for the whole
     * budget and then de-energise under the commissioned slew limit instead of
     * stepping the actuator in one tick.  Every tick of this segment runs
     * through step_checked(), so magnitude and slew compliance are verified on
     * each sample rather than on the endpoint alone. */
    inv_reset(&inv);
    for (i = 0; i < 200; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_NORMAL && t_absf(out.u) > 0.0f && st.hold_ticks == 0u,
          "A1 hold: healthy frames run the closed loop (u=%.6f, hold_ticks=0)", (double)out.u);

    /* Corrupt the fast measurement: the first max_hold_ticks ticks must hold,
     * the next one must start the slew-limited de-energise. */
    in.x = make_nan();
    for (i = 0; i < (int)cfg.max_hold_ticks; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
        if (out.gate != GATE_HOLD_LATENCY) {
            hold_bad++;
        }
    }
    check(hold_bad == 0 && st.hold_ticks == cfg.max_hold_ticks,
          "A1 hold: the first %u consecutive corrupted ticks stay in GATE_HOLD_LATENCY (hold_ticks=%u)",
          (unsigned)cfg.max_hold_ticks, (unsigned)st.hold_ticks);
    prev_u = out.u; /* the command held through the whole HOLD window */
    du_max = cfg.u_slew_max_per_s * cfg.dt_s;
    out = step_checked(&inv, &st, &cfg, &tun, &in);
    for (i = 0; i < LAYERED_NUM_REGIMES; ++i) {
        if (st.integ[i] != 0.0f) {
            integ_zero = 0;
        }
    }
    check(out.gate == GATE_SAFE_ZERO && out.u_target == 0.0f &&
          st.u_target == 0.0f &&
          t_absd((double)out.u - (double)prev_u) <= (double)du_max &&
          t_absf(out.u) < t_absf(prev_u) && integ_zero != 0 &&
          st.permanent_latched == 1u &&
          (st.fault_audit_bitmask &
           (LAYERED_FAULT_BIT_HOLD_TIMEOUT | LAYERED_FAULT_BIT_PERMANENT_LATCH)) ==
          (LAYERED_FAULT_BIT_HOLD_TIMEOUT | LAYERED_FAULT_BIT_PERMANENT_LATCH),
          "A1 hold: tick %u (budget %u + 1) forces GATE_SAFE_ZERO and ramps the command under "
          "the slew limit (u %.6f -> %.6f, u_target=0, permanent=%u, mask=0x%02X)",
          (unsigned)cfg.max_hold_ticks + 1u, (unsigned)cfg.max_hold_ticks,
          (double)prev_u, (double)out.u, (unsigned)st.permanent_latched,
          (unsigned)st.fault_audit_bitmask);
    check(integ_zero != 0,
          "A1 hold: every branch integrator is cleared before the zero command");
    check((st.fault_audit_bitmask & LAYERED_FAULT_BIT_HOLD_TIMEOUT) != 0u &&
          (st.fault_audit_bitmask & LAYERED_FAULT_BIT_NAN_CORRUPTION) != 0u,
          "A1 hold: HOLD_TIMEOUT and NAN_CORRUPTION are latched (mask 0x%02X)",
          (unsigned)st.fault_audit_bitmask);
    check(st.permanent_latched == 1u &&
          (st.fault_audit_bitmask & LAYERED_FAULT_BIT_PERMANENT_LATCH) != 0u,
          "A1 hold: a sensor-caused hold timeout also arms the permanent latch");

    /* Same sensor fault but a valid configuration: the remaining command is
     * walked to zero one du_max per tick, then the zero output is held. */
    zero_step = -1;
    zero_bad = 0;
    for (i = 1; i <= 8; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
        if (out.gate != GATE_SAFE_ZERO) {
            zero_bad++;
        }
        if (out.u == 0.0f) {
            zero_step = i;
            break;
        }
    }
    check(zero_step >= 1 && zero_bad == 0,
          "A1 hold: the slew-limited de-energise reaches exactly u == 0.0f after %d extra "
          "tick(s) and stays in GATE_SAFE_ZERO", zero_step);
    out = step_checked(&inv, &st, &cfg, &tun, &in);
    check(out.gate == GATE_SAFE_ZERO && out.u == 0.0f,
          "A1 hold: a still-corrupt sensor keeps the de-energised output");
    in.x = 0.20f;
    out = step_checked(&inv, &st, &cfg, &tun, &in);
    check(out.gate == GATE_LATCHED_SAFE && out.regime == REGIME_SAFE_FALLBACK,
          "A1 hold: recovering from a sensor-timeout safe zero needs the latch API (%s)",
          Layered_GateName(out.gate));
    check(inv.violations == 0 && inv.nan_count == 0,
          "A1 hold: every valid-configuration tick obeys |u|<=u_max and |du|<=u_slew*dt "
          "(worst |u|=%.6f, |du|=%.6f)",
          (double)inv.worst_abs_u, (double)inv.worst_abs_du);

    /* A1b: an untrusted configuration is held for the fixed corrupt-config
     * budget and then de-energised immediately: its slew limit cannot be used.
     * Direct Layered_Step() is deliberate here. */
    Layered_Init(&st, &cfg, &tun);
    inv_reset(&inv);
    for (i = 0; i < 200; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_NORMAL && t_absf(out.u) > 0.0f && inv.violations == 0,
          "A1 config_bad: the healthy prelude energises the actuator (u=%.6f)",
          (double)out.u);
    bad_cfg = cfg;
    bad_cfg.kp[0] = make_nan();
    hold_bad = 0;
    for (i = 0; i < (int)corrupt_config_hold_ticks; ++i) {
        out = Layered_Step(&st, &bad_cfg, &tun, &in);
        if (out.gate != GATE_HOLD_LATENCY) {
            hold_bad++;
        }
    }
    prev_u = out.u;
    check(hold_bad == 0 && st.hold_ticks == corrupt_config_hold_ticks,
          "A1 config_bad: the first %u corrupt-config ticks hold the last command (hold_ticks=%u)",
          (unsigned)corrupt_config_hold_ticks, (unsigned)st.hold_ticks);
    out = Layered_Step(&st, &bad_cfg, &tun, &in);
    check(out.gate == GATE_SAFE_ZERO && out.u == 0.0f && st.u_applied == 0.0f &&
          st.permanent_latched == 1u &&
          (st.fault_audit_bitmask &
           (LAYERED_FAULT_BIT_HOLD_TIMEOUT | LAYERED_FAULT_BIT_PERMANENT_LATCH)) ==
          (LAYERED_FAULT_BIT_HOLD_TIMEOUT | LAYERED_FAULT_BIT_PERMANENT_LATCH),
          "A1 config_bad: tick %u de-energises immediately (u %.6f -> 0.0) because the "
          "rejected configuration's slew limit is untrusted",
          (unsigned)corrupt_config_hold_ticks + 1u, (double)prev_u);

    /* A1b': UINT32_MAX can never time out (the counter saturates), so it must
     * be rejected as a corrupt configuration on the very first tick. */
    bad_cfg = cfg;
    bad_cfg.max_hold_ticks = UINT32_MAX;
    Layered_Init(&st, &bad_cfg, &tun);
    out = Layered_Step(&st, &bad_cfg, &tun, &in);
    check(out.corrupted == 1 && out.gate == GATE_HOLD_LATENCY && st.hold_ticks == 1u,
          "A1 config_bad: max_hold_ticks == UINT32_MAX is rejected as a corrupt configuration "
          "(corrupted=%d, gate=%s, hold_ticks=%u)",
          out.corrupted, Layered_GateName(out.gate), (unsigned)st.hold_ticks);

    /* A1c: a stale slow mailbox with healthy fast sensors must keep the 1 kHz
     * loop closed on the fixed R4 law for the whole grace window, report
     * GATE_HOLD_LATENCY, and only then de-energise WITHOUT the permanent latch
     * and without breaking the commissioned slew limit.  Freezing the last
     * torque while valid feedback exists is exactly the defect this contract
     * removes. */
    Layered_Init(&st, &cfg, &tun);
    in.x = 0.20f;
    in.x_dot = 0.0f;
    inv_reset(&inv);
    for (i = 0; i < 200; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
    }
    prev_u = out.u; /* the command that must be walked down, not stepped down */
    in.outer_age_s = 10.0f; /* heartbeat gone for good */
    hold_bad = 0;
    hold_motion = 0;
    {
        float previous = prev_u;
        for (i = 0; i < (int)cfg.max_hold_ticks; ++i) {
            out = step_checked(&inv, &st, &cfg, &tun, &in);
            if (out.gate != GATE_HOLD_LATENCY || out.regime != REGIME_SAFE_FALLBACK) {
                hold_bad++;
            }
            if (out.u != previous) {
                hold_motion++;
            }
            previous = out.u;
        }
        prev_u = previous;
    }
    check(hold_bad == 0 && st.hold_ticks == cfg.max_hold_ticks && hold_motion > 0,
          "A1 hold: all %u stale-heartbeat grace ticks stay in GATE_HOLD_LATENCY / "
          "REGIME_SAFE_FALLBACK and the command keeps moving (%d changes, hold_ticks=%u)",
          (unsigned)cfg.max_hold_ticks, hold_motion, (unsigned)st.hold_ticks);
    du_max = cfg.u_slew_max_per_s * cfg.dt_s;
    out = step_checked(&inv, &st, &cfg, &tun, &in);
    check(out.gate == GATE_SAFE_ZERO && out.u_target == 0.0f && st.u_target == 0.0f &&
          t_absd((double)out.u - (double)prev_u) <= (double)du_max &&
          t_absf(out.u) <= t_absf(prev_u) &&
          (st.fault_audit_bitmask & LAYERED_FAULT_BIT_HOLD_TIMEOUT) != 0u &&
          st.permanent_latched == 0u,
          "A1 hold: a latency-only timeout -> %s u=%.6f (walked from %.6f by <= %.6f) "
          "permanent=%u hold_ticks=%u",
          Layered_GateName(out.gate), (double)out.u, (double)prev_u, (double)du_max,
          (unsigned)st.permanent_latched, (unsigned)st.hold_ticks);
    zero_step = -1;
    zero_bad = 0;
    for (i = 1; i <= 8; ++i) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
        if (out.gate != GATE_SAFE_ZERO) {
            zero_bad++;
        }
        if (out.u == 0.0f) {
            zero_step = i;
            break;
        }
    }
    check(zero_step >= 1 && zero_bad == 0,
          "A1 hold: a latency-only timeout reaches exactly u == 0.0f after %d extra "
          "slew-limited tick(s)", zero_step);
    in.outer_age_s = 0.005f;
    out = step_checked(&inv, &st, &cfg, &tun, &in);
    check(out.gate == GATE_NORMAL && st.hold_ticks == 0u &&
          out.regime == REGIME_SAFE_FALLBACK,
          "A1 hold: a fresh heartbeat leaves safe zero and clears hold_ticks (%s / %s)",
          Layered_GateName(out.gate), Layered_RegimeName(out.regime));
    check(inv.violations == 0 && inv.nan_count == 0,
          "A1 hold: every valid-configuration tick obeys |u|<=u_max and |du|<=u_slew*dt "
          "(worst |u|=%.6f, |du|=%.6f)",
          (double)inv.worst_abs_u, (double)inv.worst_abs_du);

    /* A1d: latency_hold_count counts the exact native-HOLD entry edge.  A first
     * frame already 10 periods past the timeout is one episode, not zero. */
    Layered_Init(&st, &cfg, &tun);
    in.x = 0.20f;
    in.outer_age_s = cfg.outer_timeout_s + 10.0f * cfg.dt_s;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_HOLD_LATENCY && st.latency_hold_count == 1u &&
          st.hold_ticks == 1u,
          "A1 hold: a first stale frame 10 periods past the timeout counts one HOLD entry "
          "(gate=%s, count=%u, hold_ticks=%u)",
          Layered_GateName(out.gate), (unsigned)st.latency_hold_count,
          (unsigned)st.hold_ticks);

    /* A1e: five constant frames half a period past the timeout stay one episode. */
    Layered_Init(&st, &cfg, &tun);
    in.x = 0.20f;
    in.outer_age_s = cfg.outer_timeout_s + 0.5f * cfg.dt_s;
    hold_bad = 0;
    for (i = 0; i < 5; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
        if (out.gate != GATE_HOLD_LATENCY) {
            hold_bad++;
        }
    }
    check(hold_bad == 0 && st.latency_hold_count == 1u && st.hold_ticks == 5u,
          "A1 hold: five constant stale frames stay one HOLD episode (count=%u, hold_ticks=%u)",
          (unsigned)st.latency_hold_count, (unsigned)st.hold_ticks);
}

/* ========================================================================= */
/* Phase A2: cumulative severe-fault latch + explicit acknowledgement        */
/* ========================================================================= */

static void test_permanent_latch(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    LayeredConfig_t bad_cfg;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out = zero_output();
    int episode;
    int i;
    int latched_through_cooldown = 1;
    uint32_t ood_history;
    uint32_t safety_history;
    uint32_t mask_history;
    uint32_t step_history;

    build_config(&cfg); /* max_fault_vetoes_before_latch == 3 */
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.x = 0.0f;
    in.x_dot = 0.0f;
    in.r = 0.0f;
    in.r_dot = 0.0f;
    in.u_ff = 0.0f;
    in.outer_seq = 0u;

    for (episode = 0; episode < (int)cfg.max_fault_vetoes_before_latch; ++episode) {
        in.physics_residual = 3.0f;
        for (i = 0; i < 5; ++i) {
            out = Layered_Step(&st, &cfg, &tun, &in);
        }
        in.physics_residual = 0.10f;
        for (i = 0; i < 5; ++i) {
            out = Layered_Step(&st, &cfg, &tun, &in);
        }
        if (episode + 1 < (int)cfg.max_fault_vetoes_before_latch) {
            check(st.permanent_latched == 0u && out.gate != GATE_LATCHED_SAFE,
                  "A2 latch: %d veto episode(s) stay recoverable (latched=0)",
                  episode + 1);
        }
    }
    check(st.ood_veto_count == cfg.max_fault_vetoes_before_latch &&
          st.permanent_latched == 1u &&
          (st.fault_audit_bitmask & LAYERED_FAULT_BIT_PERMANENT_LATCH) != 0u &&
          out.gate == GATE_LATCHED_SAFE && out.regime == REGIME_SAFE_FALLBACK,
          "A2 latch: %u accumulated OOD episodes latch GATE_LATCHED_SAFE in R4",
          (unsigned)st.ood_veto_count);

    for (i = 0; i < (int)cfg.ood_recovery_cooldown_ticks + 100; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
        if (out.gate != GATE_LATCHED_SAFE || out.regime != REGIME_SAFE_FALLBACK) {
            latched_through_cooldown = 0;
        }
    }
    check(latched_through_cooldown != 0 && st.recovery_ticks == 0u,
          "A2 latch: the latch survives the full %u-tick recovery cooldown",
          (unsigned)cfg.ood_recovery_cooldown_ticks);

    bad_cfg = cfg;
    bad_cfg.max_hold_ticks = 0u;
    check(Layered_AcknowledgeFaultLatch(0, 0) == 0 &&
          Layered_AcknowledgeFaultLatch(&st, 0) == 0 &&
          Layered_AcknowledgeFaultLatch(&st, &bad_cfg) == 0,
          "A2 latch: the acknowledgement API rejects null pointers and a corrupt configuration");

    ood_history = st.ood_veto_count;
    safety_history = st.safety_veto_count;
    mask_history = st.fault_audit_bitmask;
    step_history = st.step_count;
    check(Layered_AcknowledgeFaultLatch(&st, &cfg) == 1,
          "A2 latch: the explicit acknowledgement releases the latch");
    check(st.permanent_latched == 0u && st.ood_latched == 0u &&
          st.safety_latched == 0u && st.conf_latched == 0u &&
          st.hold_ticks == 0u &&
          st.recovery_ticks == cfg.ood_recovery_cooldown_ticks,
          "A2 latch: ack clears latches/hold_ticks and re-arms the %u-tick cooldown",
          (unsigned)st.recovery_ticks);
    check(st.ood_veto_count == ood_history && st.safety_veto_count == safety_history &&
          st.fault_audit_bitmask == mask_history && st.step_count == step_history,
          "A2 latch: ack preserves the audit history (%u OOD episodes, mask 0x%02X)",
          (unsigned)st.ood_veto_count, (unsigned)st.fault_audit_bitmask);

    for (i = 0; i <= (int)cfg.ood_recovery_cooldown_ticks; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_NORMAL && out.regime == REGIME_NOMINAL &&
          st.permanent_latched == 0u,
          "A2 latch: after ack the AI bank is re-armed once the clean cooldown expires (%s / %s)",
          Layered_GateName(out.gate), Layered_RegimeName(out.regime));

    check(st.latch_episode_count == 0u && st.ood_veto_count == ood_history,
          "A2 latch: ack resets the episode quota (episodes=%u) but keeps the %u OOD "
          "episodes in the audit history",
          (unsigned)st.latch_episode_count, (unsigned)st.ood_veto_count);

    /* The acknowledgement restores the full tolerance: only three NEW OOD
     * episodes may re-arm the latch, while the lifetime audit counter keeps
     * counting across the release. */
    for (episode = 1; episode <= (int)cfg.max_fault_vetoes_before_latch; ++episode) {
        in.physics_residual = 3.0f;
        for (i = 0; i < 5; ++i) {
            out = Layered_Step(&st, &cfg, &tun, &in);
        }
        in.physics_residual = 0.10f;
        for (i = 0; i < 5; ++i) {
            out = Layered_Step(&st, &cfg, &tun, &in);
        }
        if (episode < (int)cfg.max_fault_vetoes_before_latch) {
            check(st.permanent_latched == 0u &&
                  st.latch_episode_count == (uint32_t)episode &&
                  out.gate != GATE_LATCHED_SAFE,
                  "A2 latch: after ack, new OOD episode %d stays recoverable "
                  "(episodes=%u, latched=%u)",
                  episode, (unsigned)st.latch_episode_count,
                  (unsigned)st.permanent_latched);
        }
    }
    check(st.ood_veto_count == 2u * cfg.max_fault_vetoes_before_latch &&
          st.latch_episode_count == cfg.max_fault_vetoes_before_latch &&
          st.permanent_latched == 1u && out.gate == GATE_LATCHED_SAFE,
          "A2 latch: after ack, %u fresh OOD episodes re-arm the latch "
          "(audit OOD episodes=%u, episodes=%u)",
          (unsigned)cfg.max_fault_vetoes_before_latch, (unsigned)st.ood_veto_count,
          (unsigned)st.latch_episode_count);
}

/* ========================================================================= */
/* Phase A3: L1 local slow-frame lease (outer_seq)                           */
/* ========================================================================= */

static void test_outer_sequence_lease(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int i;
    int first_hold = -1;
    int early_hold = 0;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.0f; /* forged: the mailbox claims a brand-new frame */
    in.physics_residual = 0.10f;
    in.x = 0.0f;
    in.x_dot = 0.0f;
    in.r = 0.0f;
    in.r_dot = 0.0f;
    in.u_ff = 0.0f;
    in.outer_seq = 7u;

    /* The sequence number never changes: only the local clock may age it. */
    for (i = 0; i < 300; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
        if (out.gate == GATE_HOLD_LATENCY && first_hold < 0) {
            first_hold = i;
        }
        if (i < 200 && out.gate == GATE_HOLD_LATENCY) {
            early_hold++;
        }
    }
    check(early_hold == 0 && first_hold >= 240 && first_hold <= 260,
          "A3 seq: forged outer_age_s=0.0 cannot mask an unchanged sequence (local hold at tick %d)",
          first_hold);
    check(st.outer_age_ticks == 299u && st.last_outer_seq == 7u,
          "A3 seq: 300 identical sequence numbers age the local lease to %u ticks",
          (unsigned)st.outer_age_ticks);

    in.outer_seq = 8u;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate != GATE_HOLD_LATENCY && st.outer_age_ticks == 0u,
          "A3 seq: a changed sequence number opens a fresh local lease (%s)",
          Layered_GateName(out.gate));
    for (i = 0; i < 200; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    check(out.gate != GATE_HOLD_LATENCY && st.outer_age_ticks == 200u,
          "A3 seq: 201 ticks on the fresh lease stay inside the timeout (age_ticks=%u)",
          (unsigned)st.outer_age_ticks);

    /* A sequence that was live and then reads 0 is a mailbox anomaly. */
    in.outer_seq = 0u;
    in.outer_age_s = 0.005f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_VETO_LOW_CONFIDENCE &&
          out.regime == REGIME_SAFE_FALLBACK && st.conf_latched == 1u,
          "A3 seq: a sequence falling back to 0 is treated as mailbox corruption and forces R4");
    check(out.corrupted == 1 && st.corrupted_frame_count == 1u,
          "A3 seq: the sequence fallback is reported as one corrupt frame (corrupted=%d, count=%u)",
          out.corrupted, (unsigned)st.corrupted_frame_count);
    check((st.fault_audit_bitmask & LAYERED_FAULT_BIT_NAN_CORRUPTION) != 0u,
          "A3 seq: the NAN_CORRUPTION audit bit records the sequence fallback (mask 0x%02X)",
          (unsigned)st.fault_audit_bitmask);

    /* Legacy mode stays bit-compatible when outer_seq is never used. */
    Layered_Init(&st, &cfg, &tun);
    in.outer_seq = 0u;
    in.outer_age_s = 0.50f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_HOLD_LATENCY && st.last_outer_seq == 0u,
          "A3 seq: outer_seq == 0 keeps the legacy outer_age_s timeout");
}

/* ========================================================================= */
/* Phase A5: numerical-domain bounds                                         */
/* ========================================================================= */

/* The slow-frame lease is timed on the fast clock, so an unusable fast frame
 * must not pause it: alternating corrupt frames must still trip the 250 ms
 * timeout after 250 ms of wall time, not after twice as many ticks. */
static void test_lease_under_corrupt_frames(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int i;
    int first_clean_hold = -1;

    build_config(&cfg); /* dt = 1 ms, outer_timeout = 0.250 s */
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.0f; /* forged: only the local lease may time the frame */
    in.physics_residual = 0.10f;
    in.x = 0.10f;
    in.x_dot = 0.0f;
    in.r = 0.10f;
    in.r_dot = 0.0f;
    in.u_ff = 0.0f;
    in.outer_seq = 1u;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_NORMAL && st.outer_age_ticks == 0u,
          "A5 lease: a first live sequence frame opens the local lease at age 0");

    for (i = 0; i < 400; ++i) {
        in.x = ((i % 2) == 0) ? make_nan() : 0.10f;
        out = Layered_Step(&st, &cfg, &tun, &in);
        if (out.corrupted == 0 && out.gate == GATE_HOLD_LATENCY && first_clean_hold < 0) {
            first_clean_hold = i;
        }
    }
    check(first_clean_hold >= 249 && first_clean_hold <= 253,
          "A5 lease: the 250 ms timeout trips at fast tick %d although every second "
          "frame has an unusable fast sensor",
          first_clean_hold + 2);
    check(st.outer_age_ticks == 400u && st.last_outer_seq == 1u,
          "A5 lease: the corrupt frames still advanced the lease to %u ticks",
          (unsigned)st.outer_age_ticks);
    check(st.corrupted_frame_count == 200u,
          "A5 lease: all 200 alternating corrupt fast frames stay audited (%u)",
          (unsigned)st.corrupted_frame_count);
}

/* A legal but tiny ki_eff * windup_tau_s underflows to 0.0f in binary32.  The
 * tracking anti-windup must skip that correction instead of dividing by zero,
 * which would drive every integrator into the clamp. */
static void test_antiwindup_underflow(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int i;
    int finite = 1;

    build_config(&cfg);
    cfg.ki[0] = 1.0e-30f;
    cfg.ki[1] = 1.0e-30f;
    cfg.ki[2] = 1.0e-30f;
    cfg.ki[3] = 0.0f;
    cfg.windup_tau_s = 1.0e-20f; /* ki_eff * windup_tau underflows to 0.0f */
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.0f;
    in.x = 0.0f;
    in.x_dot = 0.0f;
    in.r = 0.1f;
    in.r_dot = 0.0f;
    in.u_ff = 0.0f;
    for (i = 0; i < 200; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
        if (!t_is_finite(out.u) || !t_is_finite(out.u_target) ||
            !t_is_finite(st.integ[0]) || !t_is_finite(st.integ[1]) ||
            !t_is_finite(st.integ[2]) || !t_is_finite(st.integ[3])) {
            finite = 0;
        }
    }
    /* Without the guard the integral would have pinned at the clamp; the pure
     * accumulator only sees dt * e = 1e-4 per tick. */
    check(finite != 0 && t_absf(st.integ[0] - 0.02f) < 1.0e-3f,
          "A5 underflow: an underflowed ki*windup_tau_s skips the back-calculation "
          "(integ[0]=%.6f, all states finite)",
          (double)st.integ[0]);
    check(t_is_finite(out.u) && t_absf(out.u) <= cfg.u_max,
          "A5 underflow: the command stays finite and inside u_max (u=%.6f)",
          (double)out.u);
}

/* Extreme but legal parameters can overflow an intermediate product.  The
 * published target must stay finite, a saturated local lease must still trip the
 * timeout, and a slew budget below one ULP must still walk the command to zero. */
static void test_extreme_numeric_domain(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int i;
    float previous;

    /* (a) kp * gain_scale * e overflows binary32: kp[0] = 3e38, scale = 4. */
    build_config(&cfg);
    cfg.kp[0] = 3.0e38f;
    cfg.kd[0] = 0.0f;
    cfg.ki[0] = 0.0f;
    build_tuning(&tun, 4.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.0f;
    in.x = 0.0f;
    in.x_dot = 0.0f;
    in.r = 0.1f;
    in.r_dot = 0.0f;
    in.u_ff = 0.0f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(t_is_finite(out.u_target) && out.u_target == cfg.u_max &&
          t_is_finite(out.u) && t_absf(out.u) <= cfg.u_max,
          "A5 overflow: an overflowing bank product clamps the published target to u_max "
          "(target=%.6f, u=%.6f)",
          (double)out.u_target, (double)out.u);

    /* (b) outer_age_ticks * dt overflows: the saturated lease must still time out.
     * The slew budget is scaled down so that slew * dt stays a finite positive
     * binary32 product, i.e. the extreme inner period remains a legal
     * configuration and the lease path is exercised instead of the config
     * firewall. */
    build_config(&cfg);
    cfg.dt_s = 3.0e38f;
    cfg.u_slew_max_per_s = 1.0e-30f; /* 1e-30 * 3e38 = 3e8, finite */
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    in.outer_age_s = 0.0f;
    in.outer_seq = 5u;
    out = Layered_Step(&st, &cfg, &tun, &in); /* lease opens, age 0 */
    check(out.gate == GATE_NORMAL && st.outer_age_ticks == 0u,
          "A5 overflow: the extreme-period lease starts at age 0 (%s)",
          Layered_GateName(out.gate));
    out = Layered_Step(&st, &cfg, &tun, &in); /* age = 3.0e38 s */
    check(out.gate == GATE_HOLD_LATENCY,
          "A5 overflow: the extreme-period lease is stale on its first aged tick");
    out = Layered_Step(&st, &cfg, &tun, &in); /* product overflows to +Inf */
    check(out.gate == GATE_HOLD_LATENCY && st.outer_age_ticks == 2u,
          "A5 overflow: an overflowing lease age saturates instead of releasing the HOLD "
          "(gate=%s, age_ticks=%u)",
          Layered_GateName(out.gate), (unsigned)st.outer_age_ticks);

    /* (c) du_max = slew * dt below one ULP of the remaining command. */
    build_config(&cfg);
    cfg.u_slew_max_per_s = 1.0e-6f; /* du_max = 1e-9 s^-1, far below ULP(1.0f) */
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    st.u_applied = 1.0f;
    st.u_target = 1.0f;
    st.control_started = 1;
    in.outer_age_s = cfg.outer_timeout_s + 10.0f * cfg.dt_s; /* heartbeat gone for good */
    in.outer_seq = 0u;
    for (i = 0; i <= (int)cfg.max_hold_ticks; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_SAFE_ZERO && st.u_applied < 1.0f && st.u_applied > 0.0f,
          "A5 sub-ULP: the first de-energise tick moves the command below 1.0 (u=%.9f)",
          (double)st.u_applied);
    previous = st.u_applied;
    for (i = 0; i < 200; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
        if (!(st.u_applied < previous && st.u_applied > 0.0f)) {
            break;
        }
        previous = st.u_applied;
    }
    check(i == 200 && st.u_applied < 0.99999f && st.u_applied > 0.9999f,
          "A5 sub-ULP: 200 further ticks decrease the command by one ULP each (u=%.9f)",
          (double)st.u_applied);

    /* (d) The binary32 slew budget slew * dt must stay positive and finite.
     * A product that underflows to zero cannot be honoured by the quantisation
     * floor, and one that overflows lets a single tick reach any command; both
     * are degenerate configurations and must be refused as corrupt instead of
     * being silently reinterpreted. */
    in = zero_input();
    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    build_config(&cfg);
    cfg.dt_s = 1.0e-6f;
    cfg.u_slew_max_per_s = 1.0e-40f; /* 1e-46 rounds to 0.0f in binary32 */
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.corrupted != 0 && out.gate == GATE_HOLD_LATENCY,
          "A5 slew budget: slew * dt underflowing to 0 is rejected as corrupt (%s)",
          Layered_GateName(out.gate));
    build_config(&cfg);
    cfg.dt_s = 3.0e38f;
    cfg.u_slew_max_per_s = 3.0e38f; /* the product overflows to +Inf */
    Layered_Init(&st, &cfg, &tun);
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.corrupted != 0 && out.gate == GATE_HOLD_LATENCY,
          "A5 slew budget: slew * dt overflowing to infinity is rejected as corrupt (%s)",
          Layered_GateName(out.gate));
}

/* A corrupt fast frame advances hold_ticks without being a latency timeout, so
 * the following slow-mailbox timeout must still count exactly one episode. */
static void test_mixed_hold_episode(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int i;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.x = 0.10f;
    in.x_dot = 0.0f;
    in.r = 0.10f;
    in.r_dot = 0.0f;
    in.u_ff = 0.0f;
    for (i = 0; i < 200; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_NORMAL && st.latency_hold_count == 0u,
          "A5 mixed HOLD: healthy frames open no latency episode (count=%u)",
          (unsigned)st.latency_hold_count);

    in.x = make_nan();
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.corrupted == 1 && out.gate == GATE_HOLD_LATENCY && st.hold_ticks == 1u &&
          st.latency_hold_count == 0u,
          "A5 mixed HOLD: one corrupt fast frame advances hold_ticks to %u without "
          "counting a latency episode", (unsigned)st.hold_ticks);

    in.x = 0.10f;
    in.outer_age_s = cfg.outer_timeout_s + 0.5f * cfg.dt_s;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.corrupted == 0 && out.gate == GATE_HOLD_LATENCY && st.hold_ticks == 2u &&
          st.latency_hold_count == 1u,
          "A5 mixed HOLD: the following slow-mailbox timeout counts one latency episode "
          "(hold_ticks=%u, count=%u)",
          (unsigned)st.hold_ticks, (unsigned)st.latency_hold_count);
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(st.latency_hold_count == 1u && st.hold_ticks == 3u,
          "A5 mixed HOLD: the continued stale frame does not count a second episode");
}

/* ========================================================================= */
/* Phase A4: emergency R4 entry bypasses the slow blend filter               */
/* ========================================================================= */

static void test_emergency_r4_bypass(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int i;
    float prev_u;
    float integ3;
    float expected;
    float du_max;

    build_config(&cfg);
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.x = 0.10f;
    in.x_dot = 0.02f;
    in.r = 0.20f;
    in.r_dot = 0.0f;
    in.u_ff = 0.0f;
    in.outer_seq = 0u;

    for (i = 0; i < 300; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }

    /* Force a partial blend and a stale target: the emergency path must
     * ignore both and write the raw R4 target. */
    st.blend = 0.25f;
    st.u_target = 0.0f;
    integ3 = st.integ[3];
    prev_u = st.u_applied;
    du_max = cfg.u_slew_max_per_s * cfg.dt_s;

    in.physics_residual = 3.0f; /* independent OOD veto -> immediate R4 */
    out = Layered_Step(&st, &cfg, &tun, &in);

    expected = cfg.kp[3] * (-in.x) + cfg.kd[3] * (-in.x_dot) + cfg.ki[3] * integ3;
    check(out.gate == GATE_VETO_OOD_RESIDUAL && out.regime == REGIME_SAFE_FALLBACK,
          "A4 bypass: an OOD veto enters R4 on the same tick (%s / %s)",
          Layered_GateName(out.gate), Layered_RegimeName(out.regime));
    check(out.blend == 1.0f && t_absf(out.u_target - expected) <= 1.0e-7f,
          "A4 bypass: blend=%.3f and the target is the raw R4 law (diff %.9f)",
          (double)out.blend, (double)t_absf(out.u_target - expected));
    check(t_absf(out.u) <= cfg.u_max + 1.0e-6f &&
          t_absd((double)out.u - (double)prev_u) <= (double)du_max,
          "A4 bypass: magnitude and slew limits still apply (|u|=%.6f, |du|=%.6f)",
          (double)t_absf(out.u), (double)t_absf(out.u - prev_u));

    /* Staying in R4 keeps the direct target as well. */
    st.blend = 0.20f;
    st.u_target = 0.0f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.blend == 1.0f && out.regime == REGIME_SAFE_FALLBACK,
          "A4 bypass: an already active R4 keeps blend at 1.0");

    /* A healthy normal-regime step must still use the slow filter. */
    Layered_Init(&st, &cfg, &tun);
    in.physics_residual = 0.10f;
    for (i = 0; i < 300; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    st.blend = 0.25f;
    st.u_target = 0.0f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_NORMAL && out.blend > 0.25f && out.blend < 1.0f,
          "A4 bypass: normal tracking still follows blend_alpha (blend=%.3f)",
          (double)out.blend);
}

/* ========================================================================= */
/* Phase A6: normal-path slew bound rounded inward                           */
/* ========================================================================= */

/* Slew-bound counterexample: with dt = 0.001 s and u_slew_max_per_s = 0.00007 /s
 * the binary32 per-tick budget is 7.0e-8, between 0.5 ULP and 1 ULP of 1.0f.
 * Nearest-even rounding puts 1.0f + 7.0e-8f at 1.0000001192f, a 1.1920929e-7
 * step, so a limiter that clamps to the rounded bound can publish almost twice
 * the legal budget on an ordinary GATE_NORMAL tick.  The ramp below is driven
 * from Layered_Init; every tick passes through
 * step_checked(), so the strict double-arithmetic bound is verified on the
 * 16.5 million ticks that reach the boundary and on the ticks after it, where
 * the inward-rounded bound must hold the command instead of overshooting. */
static void test_normal_slew_rounding(void)
{
    LayeredConfig_t cfg;
    LayeredState_t st;
    FieldTuning_t tun;
    LayeredInput_t in;
    inv_t inv;
    LayeredOutput_t out;
    float prev_u;
    double du_max;
    long k;
    int reached = 0;
    int held = 0;
    int i;

    build_config(&cfg);
    cfg.u_slew_max_per_s = 0.00007f; /* 7.0e-8 per 1 ms tick */
    cfg.u_max = 2.0f;
    cfg.blend_alpha = 1.0f;
    cfg.kp[0] = 1.0f;
    for (i = 1; i < LAYERED_NUM_REGIMES; ++i) {
        cfg.kp[i] = 0.0f;
    }
    for (i = 0; i < LAYERED_NUM_REGIMES; ++i) {
        cfg.kd[i] = 0.0f;
        cfg.ki[i] = 0.0f;
        cfg.ff_scale[i] = 0.0f;
    }
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);

    in = zero_input();
    in.ai_regime = 1.0f;
    in.ai_confidence = 1.0f;
    in.outer_age_s = 0.0f;
    in.r = 2.0f;

    du_max = (double)(cfg.u_slew_max_per_s * cfg.dt_s);
    Layered_Init(&st, &cfg, &tun);
    inv_reset(&inv);

    /* Drive the command from rest until it first sits exactly at 1.0f, then
     * keep stepping: the next representable value above 1.0f is outside the
     * 7.0e-8 budget, so the strict bound holds the command at 1.0f. */
    out = zero_output();
    for (k = 0; k < 20000000L; ++k) {
        out = step_checked(&inv, &st, &cfg, &tun, &in);
        if (st.u_applied == 1.0f) {
            if (reached != 0) {
                if (++held >= 100) {
                    break;
                }
            } else {
                reached = 1;
            }
        } else if (reached != 0) {
            break;
        }
    }

    check(reached != 0 && st.u_applied == 1.0f && out.gate == GATE_NORMAL,
          "A6 slew rounding: the 7.0e-8 ramp reaches u=1.0f on a normal tick (u=%.9f after %ld ticks)",
          (double)st.u_applied, k);
    check(inv.violations == 0 && inv.nan_count == 0,
          "A6 slew rounding: every tick honours |du| <= du_max strictly (worst |du|=%.9g, budget=%.9g)",
          (double)inv.worst_abs_du, du_max);

    /* Direct frame of the counterexample: previous command 1.0f, target
     * above it.  The rounded upper bound is 1.0000001192f, so the earlier
     * limiter published |du| = 1.19e-7 against a 7.0e-8 budget; the
     * inward-rounded bound must instead hold 1.0f. */
    Layered_Init(&st, &cfg, &tun);
    for (k = 0; k < 20; ++k) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    st.u_applied = 1.0f;
    prev_u = 1.0f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_NORMAL && out.u == prev_u,
          "A6 slew rounding: u_prev=1.0f holds at 1.0f instead of stepping to 1.0000001192f (u=%.9f)",
          (double)out.u);

    /* Negative mirror: -1.0f - 7.0e-8f rounds to -1.0000001192f, also outside
     * the budget, so the lower bound is pulled back to -1.0f as well. */
    st.u_applied = -1.0f;
    st.u_target = -2.0f;
    in.r = -2.0f;
    prev_u = -1.0f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_NORMAL && out.u == prev_u,
          "A6 slew rounding: u_prev=-1.0f holds at -1.0f instead of stepping to -1.0000001192f (u=%.9f)",
          (double)out.u);
}

/* ========================================================================= */
/* Phase A7: GATE_SAFE_ZERO de-energise budget rounded inward                */
/* ========================================================================= */

/* Reachable counterexample from the cold-start audit: dt = 0.001 s and
 * u_slew_max_per_s = 100 /s give the binary32 per-tick budget
 * 0.10000000149011612, which is larger than one ULP of the 1.5f command
 * (1.1920928955078125e-7).  Nearest-even rounding of 1.5f - 0.1f lands on
 * 1.3999999761581421f, an exact double difference of 0.10000002384185791 --
 * 2.235174179e-8 over the declared bound -- so a de-energise walk that clamps
 * to the rounded value publishes a step wider than the budget it claims to
 * honour.  The walk must round inward exactly like the normal path and step to
 * 1.4000000953674316f (0.09999990463256836) instead, while the sub-ULP case
 * keeps its documented one-ULP floor. */
static void test_safe_zero_rounding(void)
{
    LayeredConfig_t cfg;
    LayeredState_t st;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    float prev_u;
    float naive;
    double du_max;
    double naive_delta;
    int i;

    build_config(&cfg);
    cfg.dt_s = 0.001f;
    cfg.u_slew_max_per_s = 100.0f;
    cfg.max_hold_ticks = 1u;
    cfg.u_max = 2.0f;
    cfg.blend_alpha = 1.0f;
    cfg.kp[0] = 1.0f;
    for (i = 1; i < LAYERED_NUM_REGIMES; ++i) {
        cfg.kp[i] = 0.0f;
    }
    for (i = 0; i < LAYERED_NUM_REGIMES; ++i) {
        cfg.kd[i] = 0.0f;
        cfg.ki[i] = 0.0f;
        cfg.ff_scale[i] = 0.0f;
    }
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);

    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.r = 1.5f;

    Layered_Init(&st, &cfg, &tun);
    out = zero_output();
    for (i = 0; i < 100; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    prev_u = st.u_applied;
    check(out.gate == GATE_NORMAL && prev_u == 1.5f,
          "A7 safe-zero rounding: the closed loop parks the command exactly at 1.5f (u=%.9f)",
          (double)prev_u);

    du_max = (double)(cfg.u_slew_max_per_s * cfg.dt_s);
    naive = prev_u - cfg.u_slew_max_per_s * cfg.dt_s;
    naive_delta = t_absd((double)naive - (double)prev_u);
    check(naive_delta > du_max,
          "A7 safe-zero rounding: the unguarded subtraction lands outside the budget "
          "(naive step=%.17g > bound=%.17g)", naive_delta, du_max);

    /* Trigger path A: a stale slow mailbox runs the fallback bank in closed
     * loop.  Its first step is limited by the same inward-rounded slew bound,
     * so the posted command is 1.4000000953674316f (a 0.0999999046 step that
     * stays inside the 0.1000000015 budget) rather than the unguarded
     * 1.3999999761581421f (a 0.1000000238 step). */
    in.outer_age_s = 10.0f; /* heartbeat gone for good */
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_HOLD_LATENCY && out.regime == REGIME_SAFE_FALLBACK &&
          t_absd((double)out.u - 1.4000000953674316) == 0.0 &&
          t_absd((double)out.u - (double)prev_u) <= du_max,
          "A7 safe-zero rounding: the first latency-grace tick steps 1.5f -> %.9f "
          "(exact |du|=%.17g <= %.17g, R4 closed loop)",
          (double)out.u, t_absd((double)out.u - (double)prev_u), du_max);

    /* Trigger path B: with no valid fast feedback the core holds the last
     * command open-loop, so the exhausted budget then starts the documented
     * GATE_SAFE_ZERO de-energise walk from exactly 1.5f. */
    Layered_Init(&st, &cfg, &tun);
    in = zero_input();
    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.r = 1.5f;
    for (i = 0; i < 100; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    prev_u = st.u_applied;
    check(out.gate == GATE_NORMAL && prev_u == 1.5f,
          "A7 safe-zero rounding: the closed loop parks the command exactly at 1.5f (u=%.9f)",
          (double)prev_u);
    in.x = make_nan(); /* fast sensor lost: no trustworthy feedback */
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_HOLD_LATENCY && st.hold_ticks == 1u && out.u == prev_u,
          "A7 safe-zero rounding: a corrupt fast sensor holds the command open-loop "
          "(gate=%s, u=%.9f)", Layered_GateName(out.gate), (double)out.u);

    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_SAFE_ZERO,
          "A7 safe-zero rounding: the exhausted budget enters GATE_SAFE_ZERO (gate=%s, u=%.9f)",
          Layered_GateName(out.gate), (double)out.u);
    check(t_absd((double)out.u - (double)prev_u) <= du_max &&
          t_absf(out.u) > t_absf(naive) && t_absf(out.u) < t_absf(prev_u),
          "A7 safe-zero rounding: 1.5f -> %.9f is the inward-rounded step "
          "(exact |du|=%.17g <= %.17g; unguarded would be %.9f)",
          (double)out.u, t_absd((double)out.u - (double)prev_u), du_max, (double)naive);

    /* The following ticks keep honouring the exact budget while the magnitude
     * decreases monotonically. */
    prev_u = out.u;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_SAFE_ZERO &&
          t_absd((double)out.u - (double)prev_u) <= du_max &&
          t_absf(out.u) < t_absf(prev_u),
          "A7 safe-zero rounding: the next de-energise tick also lands inside the budget "
          "(u %.9f -> %.9f, exact |du|=%.17g)",
          (double)prev_u, (double)out.u, t_absd((double)out.u - (double)prev_u));
}

/* ========================================================================= */
/* Phase A8: signed envelope look-ahead                                      */
/* ========================================================================= */

/* The predictive envelope term projects the measured state ALONG its own
 * velocity: |x + T*x_dot|.  The old direction-blind |x| + T*|x_dot| treated a
 * state that was already inside the band and retreating toward zero as if it
 * were still approaching the boundary, which vetoed the safest possible
 * recovery motion and charged a permanent-latch episode for it.  An outward
 * sprint must still trip ahead of the instantaneous guard, and the
 * instantaneous guard itself stays deliberately direction-blind. */
static void test_envelope_lookahead(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    int i;
    float blind_prediction;

    build_config(&cfg); /* limit 1.0, trip ratio 0.90, look-ahead 0.06 s */
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);

    /* (a) Outward sprint from inside the instantaneous band: x = 0.88 < 0.90,
     * but the projected position 0.88 + 0.06*2.5 = 1.03 crosses the limit, so
     * the signed look-ahead must trip on the first tick. */
    Layered_Init(&st, &cfg, &tun);
    in = zero_input();
    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.x = 0.88f;
    in.x_dot = 2.5f;
    in.r = 0.88f;
    in.r_dot = 0.0f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_VETO_SAFETY_ENVELOPE && st.safety_veto_count == 1u &&
          st.latch_episode_count == 1u,
          "A8 look-ahead: an outward sprint at |x|=0.880 trips the signed prediction "
          "|x+T*v|=1.030 before the instantaneous guard (gate=%s, episodes=%u)",
          Layered_GateName(out.gate), (unsigned)st.latch_episode_count);

    /* (b) The same velocity magnitude pointing back toward zero: the projected
     * position 0.88 - 0.15 = 0.73 is inside every limit, so no veto and no
     * episode may be charged.  The direction-blind expression still reads
     * 1.030 and would have vetoed. */
    Layered_Init(&st, &cfg, &tun);
    in.x = 0.88f;
    in.x_dot = -2.5f;
    blind_prediction = t_absf(in.x) + cfg.safety_predict_s * t_absf(in.x_dot);
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(blind_prediction > cfg.state_safety_limit &&
          out.gate == GATE_NORMAL && st.safety_veto_count == 0u &&
          st.latch_episode_count == 0u &&
          (st.fault_audit_bitmask & LAYERED_FAULT_BIT_ENVELOPE_BREACH) == 0u,
          "A8 look-ahead: the same 2.5 rad/s retreat toward zero is not an envelope approach "
          "(signed |x+T*v|=0.730 vs direction-blind %.3f; gate=%s)",
          (double)blind_prediction, Layered_GateName(out.gate));

    /* (c) Cold-start report reproduction: x = 0.97, x_dot = -1.14 with the
     * instantaneous trip ratio at 0.99.  The direction-blind prediction is
     * 0.97 + 0.06*1.14 = 1.038 and vetoes; the signed prediction is 0.9016 and
     * must not charge an episode for a recovery motion. */
    cfg.safety_trip_ratio = 0.99f;
    Layered_Init(&st, &cfg, &tun);
    in.x = 0.97f;
    in.x_dot = -1.14f;
    in.r = 0.97f;
    blind_prediction = t_absf(in.x) + cfg.safety_predict_s * t_absf(in.x_dot);
    for (i = 0; i < 50; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    check(blind_prediction > cfg.state_safety_limit &&
          out.gate == GATE_NORMAL && st.safety_veto_count == 0u &&
          st.latch_episode_count == 0u,
          "A8 look-ahead: the reported x=0.970 / x_dot=-1.14 retreat charges no envelope episode "
          "(signed |x+T*v|=0.902 vs direction-blind %.3f, gate=%s)",
          (double)blind_prediction, Layered_GateName(out.gate));

    /* (d) The instantaneous guard keeps its direction-blind contract: a state
     * whose |x| is already above the trip ratio is vetoed while retreating. */
    build_config(&cfg);
    Layered_Init(&st, &cfg, &tun);
    in.x = 0.95f;
    in.x_dot = -1.14f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_VETO_SAFETY_ENVELOPE && st.safety_veto_count == 1u,
          "A8 look-ahead: the instantaneous |x| > 0.90*limit guard still vetoes an "
          "above-band state regardless of velocity direction");

    /* (e) A genuine outward trip followed by an inward retreat must charge
     * exactly one episode: the sticky latch survives while |x| is above the
     * band, clears once both the measurement and the signed prediction fall
     * below the clear ratio, and the retreat never adds a new rising edge. */
    Layered_Init(&st, &cfg, &tun);
    in.x = 0.88f;
    in.x_dot = 2.5f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_VETO_SAFETY_ENVELOPE, "A8 look-ahead: episode setup trips");
    in.x = 0.95f;
    in.x_dot = -1.14f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_VETO_SAFETY_ENVELOPE && st.latch_episode_count == 1u,
          "A8 look-ahead: a retreating state that is still above the band keeps the latch");
    in.x = 0.70f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(st.safety_latched == 0u,
          "A8 look-ahead: the sticky latch clears once |x| and the signed prediction are "
          "below the clear ratio");
    in.x = 0.88f;
    in.x_dot = -2.5f;
    for (i = 0; i < 20; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_NORMAL && st.safety_veto_count == 1u &&
          st.latch_episode_count == 1u,
          "A8 look-ahead: the inward retreat charges no second episode (episodes=%u)",
          (unsigned)st.latch_episode_count);
}

/* ========================================================================= */
/* Phase A9: closed-loop R4 during the bounded latency grace                 */
/* ========================================================================= */

/* A stale 10 Hz mailbox with healthy 1 kHz sensors must not freeze the last
 * torque.  The core keeps GATE_HOLD_LATENCY as the reported decision (and
 * latches LATENCY_TIMEOUT) but runs the fixed R4 bank on the measured state
 * for up to max_hold_ticks, then de-energises through GATE_SAFE_ZERO.  Only a
 * lost fast sensor -- no feedback at all -- holds the command open-loop. */
static void test_latency_closed_loop(void)
{
    LayeredState_t st;
    LayeredConfig_t cfg;
    FieldTuning_t tun;
    LayeredInput_t in = zero_input();
    LayeredOutput_t out;
    float integ_before;
    float expected;
    float du_max;
    float previous;
    int i;
    int motion = 0;

    build_config(&cfg); /* max_hold_ticks 100, kp3=0.50, kd3=0.02, ki3=0.05 */
    build_tuning(&tun, 1.0f, 1.0f, 1.0f);
    Layered_Init(&st, &cfg, &tun);
    in = zero_input();
    in.ai_regime = 1.0f;
    in.ai_confidence = 0.95f;
    in.outer_age_s = 0.005f;
    in.physics_residual = 0.10f;
    in.x = 0.20f;
    in.x_dot = 0.02f;
    in.r = 0.40f;
    for (i = 0; i < 200; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_NORMAL && out.regime == REGIME_NOMINAL,
          "A9 latency: the healthy prelude runs the AI bank");

    /* Stall the mailbox.  The reported gate stays GATE_HOLD_LATENCY, but the
     * regime is R4 and the blend filter is bypassed so the damping law reaches
     * the actuator without a first-order lag. */
    in.outer_age_s = 10.0f;
    integ_before = st.integ[3];
    previous = st.u_applied;
    out = Layered_Step(&st, &cfg, &tun, &in);
    expected = cfg.kp[3] * (-in.x) + cfg.kd[3] * (-in.x_dot) +
               cfg.ki[3] * integ_before;
    check(out.gate == GATE_HOLD_LATENCY && out.regime == REGIME_SAFE_FALLBACK &&
          out.corrupted == 0 && st.hold_ticks == 1u &&
          st.latency_hold_count == 1u &&
          (st.fault_audit_bitmask & LAYERED_FAULT_BIT_LATENCY_TIMEOUT) != 0u &&
          (st.fault_audit_bitmask & LAYERED_FAULT_BIT_NAN_CORRUPTION) == 0u,
          "A9 latency: a stale mailbox reports GATE_HOLD_LATENCY / R4 with the latency audit "
          "bit and no corruption (hold_ticks=%u)", (unsigned)st.hold_ticks);
    check(out.blend == 1.0f && t_absf(out.u_target - expected) <= 1.0e-6f,
          "A9 latency: the grace tick publishes the raw R4 law (blend=%.3f, target=%.6f, "
          "raw=%.6f)", (double)out.blend, (double)out.u_target, (double)expected);

    /* The grace keeps the loop closed for the whole budget: command motion,
     * live R4 integrator, slew compliance. */
    du_max = cfg.u_slew_max_per_s * cfg.dt_s;
    previous = out.u;
    {
        int grace_bad = 0;
        int slew_bad = 0;
        float integ_prev = st.integ[3];

        for (i = 1; i < (int)cfg.max_hold_ticks; ++i) {
            integ_prev = st.integ[3];
            out = Layered_Step(&st, &cfg, &tun, &in);
            if (out.gate != GATE_HOLD_LATENCY || out.regime != REGIME_SAFE_FALLBACK) {
                grace_bad++;
            }
            if (t_absf(out.u - previous) > du_max) {
                slew_bad++;
            }
            if (out.u != previous) {
                motion++;
            }
            previous = out.u;
        }
        check(grace_bad == 0 && slew_bad == 0 && motion > 0,
              "A9 latency: all %d grace ticks stay GATE_HOLD_LATENCY / R4, honour the slew "
              "bound and keep the command moving (%d changes)",
              (int)cfg.max_hold_ticks, motion);
        expected = cfg.kp[3] * (-in.x) + cfg.kd[3] * (-in.x_dot) + cfg.ki[3] * integ_prev;
    }
    check(st.hold_ticks == cfg.max_hold_ticks && st.integ[3] != integ_before &&
          t_absf(out.u_target - expected) <= 1.0e-6f,
          "A9 latency: the R4 integrator stays live through the grace and the published "
          "target tracks the live R4 law (integ=%.6f)", (double)st.integ[3]);

    /* One tick past the budget: mandatory slew-limited de-energise. */
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_SAFE_ZERO && out.u_target == 0.0f &&
          st.permanent_latched == 0u &&
          (st.fault_audit_bitmask & LAYERED_FAULT_BIT_HOLD_TIMEOUT) != 0u,
          "A9 latency: past max_hold_ticks the core enters GATE_SAFE_ZERO without the "
          "permanent latch");
    for (i = 0; i < 8 && out.u != 0.0f; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
        check(out.gate == GATE_SAFE_ZERO, "A9 latency: the de-energise walk stays in SAFE_ZERO");
    }
    check(out.u == 0.0f, "A9 latency: the de-energise walk reaches exactly zero");

    /* A fresh mailbox restores the AI only after the clean cooldown, and the
     * contrast: a corrupt fast sensor still holds the command open-loop. */
    in.outer_age_s = 0.005f;
    in.x = 0.20f;
    in.x_dot = 0.02f;
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.gate == GATE_NORMAL && out.regime == REGIME_SAFE_FALLBACK &&
          st.hold_ticks == 0u,
          "A9 latency: a fresh mailbox returns to GATE_NORMAL / recovery R4");
    for (i = 0; i < (int)cfg.ood_recovery_cooldown_ticks; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    check(out.gate == GATE_NORMAL && out.regime == REGIME_NOMINAL,
          "A9 latency: the AI bank is re-armed after the clean cooldown");

    Layered_Init(&st, &cfg, &tun);
    for (i = 0; i < 50; ++i) {
        out = Layered_Step(&st, &cfg, &tun, &in);
    }
    previous = st.u_applied;
    in.x = make_nan();
    out = Layered_Step(&st, &cfg, &tun, &in);
    check(out.corrupted == 1 && out.gate == GATE_HOLD_LATENCY &&
          out.regime == REGIME_NOMINAL && out.u == previous,
          "A9 latency: a corrupt fast sensor has no feedback, so the command is held open-loop "
          "and the last valid bank is reported");
}

int main(void)
{
    LayeredConfig_t cfg;

    build_config(&cfg);
    print_header(&cfg);

    printf("\n-- stage 1..4: 40 s mission (nominal / +35%% drift / heartbeat / faults) --\n");
    test_mission();

    printf("\n-- stage 5: FieldTuning_t (gain / damping / safety margin) --\n");
    test_tuning_gain();
    test_tuning_damping();
    test_tuning_safety_margin();

    printf("\n-- stage 6: anti-chatter dwell + post-fault recovery cooldown --\n");
    test_dwell();
    test_recovery_cooldown();

    printf("\n-- stage 7: NaN/Inf/out-of-range IPC firewall --\n");
    test_firewall();

    printf("\n-- stage 8: tracking anti-windup across a saturated bank switch --\n");
    test_antiwindup();

    printf("\n-- stage 10: deterministic feedforward u_ff --\n");
    test_feedforward();

    printf("\n-- stage 11: sticky confidence veto --\n");
    test_confidence_latch();

    printf("\n-- stage 12: first-step snap / latency cooldown / ff_scale --\n");
    test_first_control_step();
    test_cooldown_latency();
    test_ff_scale();

    printf("\n-- priority chain + API --\n");
    test_priority();
    test_api();
    test_settings_and_clamp();
    test_mailbox_fallback();
    test_settings_bounds();
    test_safe_bank_and_r3();
    test_runtime_tuning_faults();
    test_counter_boundaries();

    printf("\n-- phase A: bounded HOLD / permanent latch / seq lease / R4 bypass --\n");
    test_hold_timeout();
    test_permanent_latch();
    test_outer_sequence_lease();
    test_emergency_r4_bypass();

    printf("\n-- phase A5: numerical-domain bounds --\n");
    test_lease_under_corrupt_frames();
    test_antiwindup_underflow();
    test_extreme_numeric_domain();
    test_mixed_hold_episode();

    printf("\n-- phase A6: normal-path slew bound rounded inward --\n");
    test_normal_slew_rounding();

    printf("\n-- phase A7: GATE_SAFE_ZERO de-energise budget rounded inward --\n");
    test_safe_zero_rounding();

    printf("\n-- phase A8: signed envelope look-ahead --\n");
    test_envelope_lookahead();

    printf("\n-- phase A9: closed-loop R4 during the bounded latency grace --\n");
    test_latency_closed_loop();

    printf("\nSUMMARY: n_pass=%d, n_fail=%d\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
