// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jiawei Li
// Source: https://github.com/lijiawei255/ai-layered-control
// License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
// Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
/* gate_template.c -- MCU-side 1 kHz integration template: fixed-size innovation
 * probe plus the deterministic core; the 10 Hz mailbox is timed by outer_seq. */
#include <stdio.h>
#include "layered_gatekeeper.h"

/* Probe calibration: ~16 Hz / ~8 Hz corners at 1 kHz, envelope rates, 1.0 trip. */
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
    float a, b, c, med, innov1, innov2, mag1, mag2;
    if (blanking_active) {
        /* Scheduled artefact (stimulation pulse, commutation edge): freeze every
         * state and hold the residual. Zero-order hold, not zero: an elevated
         * residual must stay latched so one sustained fault is not fragmented. */
        return p->residual;
    }
    /* 3-point median: a one-tick EMI glitch cannot seed the IIR section. */
    a = raw;
    b = p->h0;
    c = p->h1;
    p->h1 = b;
    p->h0 = raw;
    med = a > b ? (b > c ? b : (a > c ? c : a)) : (a > c ? a : (b > c ? c : b));
    /* First-order HP innovation: reject the tracking trend, keep broadband. */
    innov1 = PROBE_HP1 * (p->hp1 + med - p->med_prev);
    /* Cascaded HP: only fastest content; hp1 is the one-tick delay. */
    innov2 = PROBE_HP2 * (p->hp2 + innov1 - p->hp1);
    p->med_prev = med;
    p->hp1 = innov1;
    p->hp2 = innov2;
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
        .kp = {25.0f, 35.0f, 12.0f, 2.0f}, .kd = {8.0f, 10.0f, 8.0f, 2.0f}, .ki = {2.0f, 8.0f, 1.0f, 0.0f},
        .ff_scale = {1.0f, 1.0f, 1.0f, 0.0f}, .max_hold_ticks = 8u, .max_fault_vetoes_before_latch = 3u};
    FieldTuning_t tuning = {1.0f, 1.0f, 1.0f};
    LayeredProbe_t probe = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    LayeredInput_t in = {1.0f, 0.95f, 0.005f, 0.0f, 0.0f, 0.0f, 0.12f, 0.0f, 0.0f, 0u};
    LayeredOutput_t out = {0.0f, 0.0f, GATE_NORMAL, REGIME_NOMINAL, 0.0f, 0u, 0};
    float q = 0.0f, v = 0.0f, q_f = 0.0f, q_prev = 0.0f;
    float q_hold = 0.0f, v_meas = 0.0f;
    float q_meas, frozen = 0.0f;
    uint32_t tick, seq = 0u, gates_seen = 0u, latch_pre = 0u;
    int blank, blank2, glitch, blank_ok = 1, ack_ok = 0, ok;
    Layered_Init(&state, &cfg, &tuning);
    printf("MCU C99 Template: state=%u B config=%u B tuning=%u B input=%u B output=%u B probe=%u B\n",
           (unsigned)sizeof(LayeredState_t), (unsigned)sizeof(LayeredConfig_t), (unsigned)sizeof(FieldTuning_t),
           (unsigned)sizeof(LayeredInput_t), (unsigned)sizeof(LayeredOutput_t), (unsigned)sizeof(LayeredProbe_t));
    /* Mission: blanked artefacts in the normal state and under OOD, a stalled
     * mailbox (HOLD -> SAFE_ZERO), three OOD pulses, then acknowledgement. */
    for (tick = 0u; tick < 2850u; ++tick) {
        if ((tick < 1200u || tick >= 1600u) && tick % 100u == 0u) in.outer_seq = ++seq;
        blank = (tick >= 400u && tick < 410u);
        blank2 = (tick >= 1804u && tick < 1814u);
        glitch = (tick >= 1800u && tick < 2404u && (tick % 300u) < 4u);
        q_meas = q + ((glitch || blank || blank2) ? 0.7f : 0.0f);
        q_hold = (blank || blank2 || glitch) ? q_hold : q_meas;
        q_meas = (blank || blank2) ? q_hold : q_meas;
        q_f += 0.02f * (q_meas - q_f);
        v_meas = (q_f - q_prev) / cfg.dt_s;
        q_prev = q_f;
        if ((blank && tick == 400u) || (blank2 && tick == 1804u)) frozen = probe.residual;
        if (blank2 && tick == 1804u) latch_pre = state.latch_episode_count;
        in.physics_residual = LayeredProbe_Step(&probe, q + ((glitch || blank || blank2) ? 0.7f : 0.0f),
                                                blank || blank2);
        in.x = q_f;
        in.x_dot = v_meas;
        out = Layered_Step(&state, &cfg, &tuning, &in);
        v += cfg.dt_s * (out.u - 0.4f * v);
        q += cfg.dt_s * v;
        if ((blank || blank2) && in.physics_residual != frozen) blank_ok = 0;
        if (blank && (out.gate == GATE_VETO_OOD_RESIDUAL || frozen >= cfg.max_physics_residual)) blank_ok = 0;
        if (blank2 && state.ood_latched != 1u) blank_ok = 0;
        if (blank2 && tick == 1813u && state.latch_episode_count != latch_pre) blank_ok = 0;
        if (tick == 410u && probe.h0 != q_meas) blank_ok = 0;
        gates_seen |= 1u << (unsigned)out.gate;
        if (tick == 2650u) ack_ok = Layered_AcknowledgeFaultLatch(&state, &cfg);
    }
    /* Bits 0,1,2,5,6: NORMAL, HOLD_LATENCY, OOD, SAFE_ZERO and LATCHED_SAFE. */
    ok = ((gates_seen & 0x67u) == 0x67u) && ack_ok && blank_ok;
    ok = ok && (out.gate == GATE_NORMAL) && (out.regime == REGIME_NOMINAL);
    ok = ok && (state.fault_audit_bitmask & LAYERED_FAULT_BIT_HOLD_TIMEOUT) != 0u;
    ok = ok && (state.fault_audit_bitmask & LAYERED_FAULT_BIT_PERMANENT_LATCH) != 0u;
    ok = ok && (state.ood_veto_count == 3u); /* exactly the three OOD episodes */
    printf("MCU C99 Template: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
