// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jiawei Li
// Source: https://github.com/lijiawei255/ai-layered-control
// License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
// Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
/*
 * gate_template.c -- MCU-side 1 kHz integration template: fixed-size innovation
 * probe plus the deterministic core; the 10 Hz mailbox is timed by outer_seq.
 * Copy the pattern into the firmware and replace the toy plant with the ADC/PWM HAL.
 */
#include <stdio.h>
#include "layered_gatekeeper.h"

/* Probe calibration: the high-pass poles set the ~16 Hz / ~8 Hz corners at
 * 1 kHz, attack/release set the envelope, gains map sensor scale to the 1.0 trip. */
#define PROBE_HP1 0.90f
#define PROBE_HP2 0.95f
#define PROBE_ATTACK 0.25f
#define PROBE_RELEASE 0.02f
#define PROBE_GAIN1 4.0f
#define PROBE_GAIN2 4.0f

typedef struct { /* fixed size, zero allocation: one instance per axis */
    float h0, h1;     /* raw history; the current sample is the raw argument */
    float med_prev;   /* previous median output */
    float hp1, hp2;   /* 1st-order HP state; cascade state of the 2nd section */
    float env1, env2; /* attack/release innovation envelopes */
    float residual;   /* last reported normalized residual */
} LayeredProbe_t;
typedef char probe_size_check[sizeof(LayeredProbe_t) == 32u ? 1 : -1];

float LayeredProbe_Step(LayeredProbe_t *p, float raw, int blanking_active)
{
    float t, a, b, c, med, innov1, innov2, mag1, mag2;

    if (blanking_active) {
        /* Scheduled artefact (stimulation pulse, commutation edge): freezing
         * every state keeps the broadband event out of the IIR sections.  Return
         * zero, not the last residual: an already elevated residual must not
         * hold the OOD veto through the blanking window; the HAL masks the gate. */
        return 0.0f;
    }
    /* 3-point median: a one-tick EMI glitch cannot seed the IIR section. */
    a = raw; b = p->h0; c = p->h1; p->h1 = b; p->h0 = raw;
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }
    med = b;
    /* First-order HP innovation: reject the tracking trend, keep broadband. */
    innov1 = PROBE_HP1 * (p->hp1 + med - p->med_prev);
    /* Second-order (cascaded) HP: only fastest content, so slow motion cannot
     * hide a short sensor or actuator fault; the old hp1 is the one-tick delay. */
    innov2 = PROBE_HP2 * (p->hp2 + innov1 - p->hp1);
    p->med_prev = med; p->hp1 = innov1; p->hp2 = innov2;
    mag1 = (innov1 < 0.0f) ? -innov1 : innov1;
    mag2 = (innov2 < 0.0f) ? -innov2 : innov2;
    /* Fast attack catches the fault, slow release stops veto chatter. */
    p->env1 += ((mag1 > p->env1) ? PROBE_ATTACK : PROBE_RELEASE) * (mag1 - p->env1);
    p->env2 += ((mag2 > p->env2) ? PROBE_ATTACK : PROBE_RELEASE) * (mag2 - p->env2);
    p->residual = PROBE_GAIN1 * p->env1 + PROBE_GAIN2 * p->env2;
    return p->residual;
}

int main(void)
{
    LayeredState_t state;
    LayeredConfig_t cfg = {
        .dt_s = 0.001f, .outer_timeout_s = 0.25f, .max_physics_residual = 1.0f, .residual_clear_ratio = 0.80f,
        .min_ai_confidence = 0.45f, .confidence_clear_ratio = 0.55f, .state_safety_limit = 1.0f,
        .safety_trip_ratio = 0.90f, .safety_clear_ratio = 0.75f, .safety_predict_s = 0.06f,
        .u_max = 2.0f, .u_slew_max_per_s = 2000.0f, .blend_alpha = 0.20f, .integ_limit = 2.0f,
        .windup_tau_s = 0.05f, .min_dwell_ticks = 100u, .ood_recovery_cooldown_ticks = 100u,
        .kp = {25.0f, 35.0f, 12.0f, 2.0f}, .kd = {8.0f, 10.0f, 8.0f, 2.0f},
        .ki = {2.0f, 8.0f, 1.0f, 0.0f}, .ff_scale = {1.0f, 1.0f, 1.0f, 0.0f},
        .max_hold_ticks = 8u, .max_fault_vetoes_before_latch = 3u};
    FieldTuning_t tuning = {1.0f, 1.0f, 1.0f};
    LayeredProbe_t probe = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    LayeredInput_t in = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0u};
    LayeredOutput_t out = {0.0f, 0.0f, GATE_NORMAL, REGIME_NOMINAL, 0.0f, 0u, 0};
    float q = 0.0f, v = 0.0f, q_f = 0.0f, q_prev = 0.0f, q_hold = 0.0f, v_meas = 0.0f, q_meas, frozen = 0.0f;
    uint32_t tick, seq = 0u;
    uint32_t saw_normal = 0u, saw_hold = 0u, saw_safe_zero = 0u, saw_ood = 0u, saw_latched = 0u;
    int blank, glitch, blank_ok = 1, ack_ok = 0, ok;

    Layered_Init(&state, &cfg, &tuning);
    printf("MCU C99 Template: state=%u B config=%u B tuning=%u B input=%u B output=%u B probe=%u B\n",
           (unsigned)sizeof(LayeredState_t), (unsigned)sizeof(LayeredConfig_t), (unsigned)sizeof(FieldTuning_t),
           (unsigned)sizeof(LayeredInput_t), (unsigned)sizeof(LayeredOutput_t), (unsigned)sizeof(LayeredProbe_t));
    in.r = 0.12f; in.r_dot = 0.0f; in.u_ff = 0.0f; in.outer_age_s = 0.005f;
    /* Mission: healthy, stalled mailbox (HOLD -> SAFE_ZERO), 3 OOD pulses, ack. */
    for (tick = 0u; tick < 2850u; ++tick) {
        if ((tick < 1200u || tick >= 1600u) && tick % 100u == 0u) {
            in.ai_regime = 1.0f; in.ai_confidence = 0.95f;
            in.outer_seq = ++seq; /* seq is the commit marker: payload is visible first */
        }
        blank = (tick >= 400u && tick < 410u);
        glitch = (tick >= 1800u && tick < 2404u && (tick % 300u) < 4u);
        q_meas = q + ((blank || glitch) ? 0.7f : 0.0f); /* one ADC anomaly into probe and core */
        if (blank) q_meas = q_hold; else q_hold = q_meas; /* zero-order hold while the HAL masks */
        q_f += 0.02f * (q_meas - q_f); v_meas = (q_f - q_prev) / cfg.dt_s; q_prev = q_f;
        if (blank && tick == 400u) frozen = probe.residual;
        in.physics_residual = LayeredProbe_Step(&probe, q_meas, blank);
        in.x = q_f; in.x_dot = v_meas;
        out = Layered_Step(&state, &cfg, &tuning, &in);
        v += cfg.dt_s * (out.u - 0.4f * v); q += cfg.dt_s * v;
        if (blank && (in.physics_residual != 0.0f || probe.residual != frozen)) blank_ok = 0;
        if (tick == 410u && in.physics_residual == 0.0f) blank_ok = 0; /* probe resumed, not stuck at 0 */
        if (out.gate == GATE_NORMAL) saw_normal = 1u; if (out.gate == GATE_HOLD_LATENCY) saw_hold = 1u;
        if (out.gate == GATE_SAFE_ZERO) saw_safe_zero = 1u; if (out.gate == GATE_VETO_OOD_RESIDUAL) saw_ood = 1u;
        if (out.gate == GATE_LATCHED_SAFE) saw_latched = 1u;
        if (tick == 2650u) ack_ok = Layered_AcknowledgeFaultLatch(&state, &cfg);
    }
    ok = saw_normal && saw_hold && saw_safe_zero && saw_ood && saw_latched && ack_ok && blank_ok;
    ok = ok && (out.gate == GATE_NORMAL) && (out.regime == REGIME_NOMINAL);
    ok = ok && (state.fault_audit_bitmask & LAYERED_FAULT_BIT_HOLD_TIMEOUT) != 0u;
    ok = ok && (state.fault_audit_bitmask & LAYERED_FAULT_BIT_PERMANENT_LATCH) != 0u;
    ok = ok && (state.ood_veto_count == 3u); /* exactly the three OOD episodes */
    printf("MCU C99 Template: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
