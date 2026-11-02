/*
 * verify_core_equivalence.c
 * ---------------------------------------------------------------------------
 * Test-only replay harness for the C/MATLAB closed-loop equivalence check.
 * It is NOT part of the deployable core: stdio is allowed here.
 *
 * Input file format produced by matlab_simulink/verify_core_equivalence.m:
 *   N
 *   dt timeout max_res res_clear min_conf conf_clear x_limit trip clear
 *   predict u_max slew alpha integ_limit windup min_dwell recovery
 *   kp0 kp1 kp2 kp3 kd0..kd3 ki0..ki3 ff0..ff3
 *   then N lines of 12 float32 bit patterns: three FieldTuning_t scales
 *   followed by the nine LayeredInput_t fields.
 *
 * Output: N lines of
 *   u u_target gate regime blend fault_audit_bitmask corrupted
 */
#include <stdio.h>
#include <stdlib.h>

#include "layered_gatekeeper.h"

static float input_value(unsigned bits)
{
    union { uint32_t bits; float value; } input;
    input.bits = (uint32_t)bits;
    return input.value;
}

int main(int argc, char **argv)
{
    FILE *f;
    FILE *g;
    int N;
    int k;
    int j;
    LayeredConfig_t cfg;
    LayeredState_t st;
    FieldTuning_t tun;

    if (argc < 3) {
        fprintf(stderr, "usage: %s input.txt output.txt\n", argv[0]);
        return 2;
    }
    f = fopen(argv[1], "r");
    if (f == NULL) {
        fprintf(stderr, "cannot open input %s\n", argv[1]);
        return 2;
    }

    if (fscanf(f, "%d", &N) != 1 || N < 0) {
        fprintf(stderr, "bad N\n");
        fclose(f);
        return 2;
    }
    if (fscanf(f, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %u %u",
               &cfg.dt_s, &cfg.outer_timeout_s,
               &cfg.max_physics_residual, &cfg.residual_clear_ratio,
               &cfg.min_ai_confidence, &cfg.confidence_clear_ratio,
               &cfg.state_safety_limit, &cfg.safety_trip_ratio,
               &cfg.safety_clear_ratio, &cfg.safety_predict_s,
               &cfg.u_max, &cfg.u_slew_max_per_s, &cfg.blend_alpha,
               &cfg.integ_limit, &cfg.windup_tau_s,
               &cfg.min_dwell_ticks, &cfg.ood_recovery_cooldown_ticks) != 17) {
        fprintf(stderr, "bad config header\n");
        fclose(f);
        return 2;
    }
    for (j = 0; j < LAYERED_NUM_REGIMES; ++j) {
        if (fscanf(f, "%f", &cfg.kp[j]) != 1) { fclose(f); return 2; }
    }
    for (j = 0; j < LAYERED_NUM_REGIMES; ++j) {
        if (fscanf(f, "%f", &cfg.kd[j]) != 1) { fclose(f); return 2; }
    }
    for (j = 0; j < LAYERED_NUM_REGIMES; ++j) {
        if (fscanf(f, "%f", &cfg.ki[j]) != 1) { fclose(f); return 2; }
    }
    for (j = 0; j < LAYERED_NUM_REGIMES; ++j) {
        if (fscanf(f, "%f", &cfg.ff_scale[j]) != 1) { fclose(f); return 2; }
    }

    tun.gain_scale = 1.0f;
    tun.damping_scale = 1.0f;
    tun.safety_margin_scale = 1.0f;
    Layered_Init(&st, &cfg, &tun);

    g = fopen(argv[2], "w");
    if (g == NULL) {
        fprintf(stderr, "cannot open output %s\n", argv[2]);
        fclose(f);
        return 2;
    }
    for (k = 0; k < N; ++k) {
        LayeredInput_t in;
        LayeredOutput_t out;
        unsigned bits[12];
        if (fscanf(f, "%u %u %u %u %u %u %u %u %u %u %u %u",
                   &bits[0], &bits[1], &bits[2], &bits[3], &bits[4],
                   &bits[5], &bits[6], &bits[7], &bits[8],
                   &bits[9], &bits[10], &bits[11]) != 12) {
            fprintf(stderr, "bad trace row %d\n", k);
            fclose(f);
            fclose(g);
            return 2;
        }
        tun.gain_scale = input_value(bits[0]);
        tun.damping_scale = input_value(bits[1]);
        tun.safety_margin_scale = input_value(bits[2]);
        in.ai_regime = input_value(bits[3]);
        in.ai_confidence = input_value(bits[4]);
        in.outer_age_s = input_value(bits[5]);
        in.physics_residual = input_value(bits[6]);
        in.x = input_value(bits[7]);
        in.x_dot = input_value(bits[8]);
        in.r = input_value(bits[9]);
        in.r_dot = input_value(bits[10]);
        in.u_ff = input_value(bits[11]);
        out = Layered_Step(&st, &cfg, &tun, &in);
        fprintf(g, "%.9g %.9g %d %d %.9g %u %d\n",
                (double)out.u, (double)out.u_target,
                (int)out.gate, (int)out.regime, (double)out.blend,
                (unsigned)out.fault_audit_bitmask, (int)out.corrupted);
    }
    fclose(f);
    fclose(g);
    return 0;
}
