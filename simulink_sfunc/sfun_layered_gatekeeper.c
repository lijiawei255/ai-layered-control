// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Jiawei Li
// Source: https://github.com/lijiawei255/ai-layered-control
// License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
// Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
/*
 * sfun_layered_gatekeeper.c
 * ---------------------------------------------------------------------------
 * Level-2 C-MEX S-function adapter for the frozen deterministic 1 kHz core in
 * ../c99_layered_core/layered_gatekeeper.c.
 *
 * The block is a thin, stateful bridge: one call per sample maps the eleven
 * scalar input ports onto one LayeredInput_t, runs Layered_Step() once and
 * publishes the seven output ports from the returned LayeredOutput_t.  No
 * control law, no clamping and no decision logic is re-implemented here; the
 * frozen core remains the single source of truth, so a Simulink run and a
 * direct C / Python-ctypes run of the same frame sequence must agree exactly.
 *
 * BUILD (from this directory, MATLAB R2024b + MinGW64 8.1.0 already selected
 * with `mex -setup C`):
 *
 *     mex -v -I../c99_layered_core sfun_layered_gatekeeper.c ../c99_layered_core/layered_gatekeeper.c
 *
 * The frozen core is compiled and linked into the MEX file, it is not copied
 * or re-implemented.  `mex -v` shows layered_gatekeeper.c on the compile line
 * and the link step; products (*.mexw64, *.obj) are ignored by .gitignore.
 *
 * BLOCK PORTS
 *   Inputs  (all scalar, double, direct feedthrough):
 *     in1  ai_regime          -> LayeredInput_t.ai_regime        (1..3, 4 = abstain)
 *     in2  ai_confidence      -> LayeredInput_t.ai_confidence    ([0,1])
 *     in3  outer_age_s        -> LayeredInput_t.outer_age_s      (s)
 *     in4  physics_residual   -> LayeredInput_t.physics_residual (>= 0, normalized)
 *     in5  x                  -> LayeredInput_t.x                (state)
 *     in6  x_dot              -> LayeredInput_t.x_dot            (state derivative)
 *     in7  r                  -> LayeredInput_t.r                (reference)
 *     in8  r_dot              -> LayeredInput_t.r_dot            (reference derivative)
 *     in9  u_ff               -> LayeredInput_t.u_ff             (deterministic feedforward)
 *     in10 outer_seq          -> LayeredInput_t.outer_seq        (slow-frame sequence, 0 = legacy)
 *     in11 ack                -> external fault-latch acknowledgement (see below)
 *   Outputs (all scalar, double):
 *     out1 u                   command actually applied this tick
 *     out2 u_target            pre-clamp target (diagnostics only)
 *     out3 gate                layered_gate_state_t code (0..6)
 *     out4 regime              layered_regime_t code (1..4)
 *     out5 blend               bumpless-transfer progress in [0,1]
 *     out6 fault_audit_bitmask sticky root-cause bitmask (uint32, cast to double)
 *     out7 corrupted           1 = invalid fast input/configuration/mailbox this tick
 *
 * ACKNOWLEDGEMENT PATH
 *   The eleventh input `ack` is the external release of the cumulative fault
 *   latch (Layered_AcknowledgeFaultLatch).  A finite level above zero is HIGH;
 *   every other finite value, including zero and negatives, is LOW.  A rising
 *   edge - HIGH on a sample whose previous sample was LOW - calls the API
 *   once, BEFORE Layered_Step() of the same tick, so that tick already runs
 *   with the latch released and the recovery cooldown re-armed.  A level held
 *   HIGH does not repeat the call.  A non-finite sample (NaN or +/-Inf) is
 *   ignored entirely and the stored previous level is kept, so forcing NaN
 *   between two HIGH samples cannot forge a second rising edge.  Feed a
 *   constant 0 when the model does not need it.
 *
 * BLOCK PARAMETERS (eight non-tunable vectors, in struct field order):
 *   P1  cfg_float  15 values: dt_s, outer_timeout_s, max_physics_residual,
 *                   residual_clear_ratio, min_ai_confidence,
 *                   confidence_clear_ratio, state_safety_limit,
 *                   safety_trip_ratio, safety_clear_ratio, safety_predict_s,
 *                   u_max, u_slew_max_per_s, blend_alpha, integ_limit,
 *                   windup_tau_s
 *   P2  cfg_ticks_a 2 values: min_dwell_ticks, ood_recovery_cooldown_ticks
 *   P3  kp          4 values (bank index = regime - 1)
 *   P4  kd          4 values
 *   P5  ki          4 values
 *   P6  ff_scale    4 values (R4 ignores ff_scale[3])
 *   P7  cfg_ticks_b 2 values: max_hold_ticks, max_fault_vetoes_before_latch
 *   P8  tuning      3 values: gain_scale, damping_scale, safety_margin_scale
 *
 *   Example "S-function parameters" string, entered as one line (the wrap
 *   below is only for page width), using the benchmark configuration:
 *     [0.001 0.35 1 .5 .01 .02 1 .9 .75 .025 20 2000 .3 .5 .1],[150 300],
 *     [60 110 8 2],[1.2 1.5 0.8 0.35],[60 300 2 0],[1 1 1 0],[100 3],[1 1 1]
 *
 *   The block sample time is taken from cfg_float(1) (dt_s) so the core's
 *   period and the Simulink step cannot drift apart.  If dt_s is not finite
 *   and positive the block inherits its sample time and the frozen core
 *   reports the invalid configuration through its own corruption path.
 *
 * NUMERIC CONVERSION AT THE ADAPTER BOUNDARY
 *   Inputs and parameters arrive as double.  A double that cannot be
 *   represented in float32 saturates to +/-FLT_MAX, which the frozen firewall
 *   already classifies as non-finite (its limit is 3.0e38), so the core's
 *   NaN/Inf corruption semantics are preserved instead of invoking undefined
 *   double->float behaviour.  NaN passes through unchanged.
 *
 *   The tick-count parameters P2 and P7 are counts, not measurements, and are
 *   validated when the adapter reads its block parameters: every element must
 *   be a finite integer in [0, 4294967295] or the parameter is rejected.
 *   A NaN, -Inf or +Inf element is a configuration error, never a silent 0 or
 *   UINT32_MAX: a silently zeroed max_fault_vetoes_before_latch would disable
 *   the permanent safety latch, and a silently zeroed max_hold_ticks would
 *   disable the HOLD -> SAFE_ZERO budget.  Fractional and out-of-range values
 *   are refused for the same reason.  An explicit legal 0 stays legal: zero
 *   vetoes is the documented way to disable the cumulative latch.  Every
 *   parameter vector must also be a real 1-D array, so a matrix with the right
 *   element count is refused instead of being read in column-major order.
 *   `mdlCheckParameters` runs the same readers, so an invalid vector is
 *   reported when the block is configured or the model is updated, and again
 *   in `mdlStart` before the first step.
 *   The uint32 conversion helper remains for the `outer_seq` input port, which
 *   is a runtime signal: a NaN or non-positive level becomes 0 (the core then
 *   falls back to the transported outer_age_s), a level above UINT32_MAX
 *   saturates and a fractional level truncates.
 *
 * STORAGE (no mutable global state, no malloc/free)
 *   DW_STATE  : LayeredState_t (96 B) held in a uint32-typed DWork vector,
 *               wider than the struct only by the alignment rounding.
 *   DW_CONFIG : LayeredConfig_t (140 B), filled from the block parameters in
 *               mdlStart; the core never copies its configuration.
 *   DW_TUNING : FieldTuning_t (12 B), likewise.
 *   DW_BOOK   : one double remembering the previous ack level (edge detect).
 *   The pointers are re-fetched from ssGetDWork() on every call, so multiple
 *   instances of the block in one model are independent.
 *
 * ABI GUARD
 *   The static assertions below pin the frozen ABI sizes; if the header ever
 *   changes, this adapter fails to compile instead of silently reinterpreting
 *   the work vectors.
 * ---------------------------------------------------------------------------
 */

#define S_FUNCTION_NAME  sfun_layered_gatekeeper
#define S_FUNCTION_LEVEL 2

#include <string.h>

#include "simstruc.h"
#include "layered_gatekeeper.h"

/* ------------------------------------------------------------------------- */
/* Block parameter map (dialog order)                                        */
/* ------------------------------------------------------------------------- */
enum {
    P_CFG_FLOAT = 0,
    P_CFG_TICKS_A,
    P_KP,
    P_KD,
    P_KI,
    P_FF_SCALE,
    P_CFG_TICKS_B,
    P_TUNING,
    P_NUM_PARAMS
};

enum {
    N_CFG_FLOAT = 15,
    N_CFG_TICKS_A = 2,
    N_BANK = 4,
    N_CFG_TICKS_B = 2,
    N_TUNING = 3
};

/* ------------------------------------------------------------------------- */
/* Port map                                                                  */
/* ------------------------------------------------------------------------- */
enum {
    IN_AI_REGIME = 0,
    IN_AI_CONFIDENCE,
    IN_OUTER_AGE_S,
    IN_PHYSICS_RESIDUAL,
    IN_X,
    IN_X_DOT,
    IN_R,
    IN_R_DOT,
    IN_U_FF,
    IN_OUTER_SEQ,
    IN_ACK,
    N_INPUTS
};

enum {
    OUT_U = 0,
    OUT_U_TARGET,
    OUT_GATE,
    OUT_REGIME,
    OUT_BLEND,
    OUT_FAULT_AUDIT,
    OUT_CORRUPTED,
    N_OUTPUTS
};

/* ------------------------------------------------------------------------- */
/* DWork map                                                                 */
/* ------------------------------------------------------------------------- */
enum {
    DW_STATE = 0,
    DW_CONFIG,
    DW_TUNING,
    DW_BOOK,
    N_DWORK
};

enum {
    DW_WORDS_STATE = (sizeof(LayeredState_t) + 3u) / 4u,
    DW_WORDS_CONFIG = (sizeof(LayeredConfig_t) + 3u) / 4u,
    DW_WORDS_TUNING = (sizeof(FieldTuning_t) + 3u) / 4u,
    DW_WORDS_BOOK = 1
};

/* Frozen-ABI guards (C99 has no _Static_assert; negative-size typedefs). */
typedef char lg_sfun_state_size_check[(sizeof(LayeredState_t) == 96) ? 1 : -1];
typedef char lg_sfun_config_size_check[(sizeof(LayeredConfig_t) == 140) ? 1 : -1];
typedef char lg_sfun_tuning_size_check[(sizeof(FieldTuning_t) == 12) ? 1 : -1];
typedef char lg_sfun_input_size_check[(sizeof(LayeredInput_t) == 40) ? 1 : -1];
typedef char lg_sfun_output_size_check[(sizeof(LayeredOutput_t) == 28) ? 1 : -1];

/* ========================================================================= */
/* Conversion helpers: double -> float32 / uint32 without undefined behaviour */
/* ========================================================================= */

/* float32 maximum; larger magnitudes saturate here so the frozen firewall
 * (which rejects anything outside +/-3.0e38) keeps its NaN/Inf semantics. */
#define LG_SFUN_FLOAT_MAX 3.402823466e38

static float lg_sfun_to_float(real_T v)
{
    if (v != v) {
        return (float)v; /* NaN reaches the core unchanged */
    }
    if (v > 3.0e38 || v < -3.0e38) {
        return (v > 0.0) ? (float)LG_SFUN_FLOAT_MAX : -(float)LG_SFUN_FLOAT_MAX;
    }
    return (float)v;
}

/* Runtime-signal conversion only (the `outer_seq` input port).  Block
 * parameters never take this path: they are validated element by element in
 * lg_sfun_read_u32_vector, so the saturating behaviour below cannot hide a
 * bad tick budget. */
static uint32_t lg_sfun_to_u32(real_T v)
{
    if (v != v) {
        return 0u;
    }
    if (v <= 0.0) {
        return 0u;
    }
    if (v >= 4294967295.0) {
        return (uint32_t)4294967295u;
    }
    return (uint32_t)v; /* in range: truncation toward zero is defined */
}

/* Finite test without a library call: NaN fails the first comparison and
 * +/-Inf fails the second because Inf - Inf is NaN. */
static int_T lg_sfun_is_finite(real_T v)
{
    return (v == v) && ((v - v) == 0.0);
}

/* HIGH = finite and strictly positive.  A negative level is LOW, never a
 * release, and a non-finite sample is not a level at all: the caller keeps the
 * stored previous level so NaN cannot fabricate a rising edge. */
static int_T lg_sfun_ack_asserted(real_T v)
{
    return lg_sfun_is_finite(v) && (v > 0.0);
}

/* ========================================================================= */
/* Block-parameter readers                                                   */
/* ========================================================================= */

/* A block-parameter vector is accepted only as a real, full, double, 1-D
 * array of exactly n elements.  A scalar is 1x1 and still passes; a matrix
 * with the right element count does not, because its column-major layout would
 * not match the documented field order. */
static int_T lg_sfun_param_is_vector(const mxArray *p, int_T n)
{
    return (p != NULL) && !mxIsEmpty(p) && mxIsDouble(p) && !mxIsComplex(p) &&
           !mxIsSparse(p) && (int_T)mxGetNumberOfElements(p) == n &&
           mxGetNumberOfDimensions(p) == 2 &&
           (mxGetM(p) == 1 || mxGetN(p) == 1);
}

static int_T lg_sfun_read_float_vector(SimStruct *S, int_T pidx, int_T n,
                                       float *dst, const char *msg)
{
    const mxArray *p = ssGetSFcnParam(S, pidx);
    const real_T *v;
    int_T i;

    if (!lg_sfun_param_is_vector(p, n)) {
        ssSetErrorStatus(S, msg);
        return 0;
    }
    v = mxGetPr(p);
    for (i = 0; i < n; ++i) {
        dst[i] = lg_sfun_to_float(v[i]);
    }
    return 1;
}

static int_T lg_sfun_read_u32_vector(SimStruct *S, int_T pidx, int_T n,
                                     uint32_t *dst, const char *msg)
{
    const mxArray *p = ssGetSFcnParam(S, pidx);
    const real_T *v;
    int_T i;

    if (!lg_sfun_param_is_vector(p, n)) {
        ssSetErrorStatus(S, msg);
        return 0;
    }
    v = mxGetPr(p);
    /* Validate every element before converting.  A tick budget is a safety
     * number: a NaN or -Inf must not decay to 0 (that would disable the
     * permanent latch or the HOLD budget) and +Inf must not become
     * UINT32_MAX.  The range test also makes the uint32 cast well defined,
     * since a double outside [0, UINT32_MAX] cannot be converted in C. */
    for (i = 0; i < n; ++i) {
        const real_T t = v[i];
        if (!lg_sfun_is_finite(t) || t < 0.0 || t > 4294967295.0 ||
            (real_T)(uint32_t)t != t) {
            ssSetErrorStatus(S, msg);
            return 0;
        }
        dst[i] = (uint32_t)t;
    }
    return 1;
}

static int_T lg_sfun_load_config(SimStruct *S, LayeredConfig_t *cfg)
{
    float f[N_CFG_FLOAT];
    float kp[N_BANK];
    float kd[N_BANK];
    float ki[N_BANK];
    float ff[N_BANK];
    uint32_t ta[N_CFG_TICKS_A];
    uint32_t tb[N_CFG_TICKS_B];

    if (!lg_sfun_read_float_vector(S, P_CFG_FLOAT, N_CFG_FLOAT, f,
            "S-function parameter 1 (cfg_float) must be a real 1-D 15-element vector.")) {
        return 0;
    }
    if (!lg_sfun_read_u32_vector(S, P_CFG_TICKS_A, N_CFG_TICKS_A, ta,
            "S-function parameter 2 (cfg_ticks_a) must be a real 1-D 2-element vector of integers in [0, 4294967295].")) {
        return 0;
    }
    if (!lg_sfun_read_float_vector(S, P_KP, N_BANK, kp,
            "S-function parameter 3 (kp) must be a real 1-D 4-element vector.")) {
        return 0;
    }
    if (!lg_sfun_read_float_vector(S, P_KD, N_BANK, kd,
            "S-function parameter 4 (kd) must be a real 1-D 4-element vector.")) {
        return 0;
    }
    if (!lg_sfun_read_float_vector(S, P_KI, N_BANK, ki,
            "S-function parameter 5 (ki) must be a real 1-D 4-element vector.")) {
        return 0;
    }
    if (!lg_sfun_read_float_vector(S, P_FF_SCALE, N_BANK, ff,
            "S-function parameter 6 (ff_scale) must be a real 1-D 4-element vector.")) {
        return 0;
    }
    if (!lg_sfun_read_u32_vector(S, P_CFG_TICKS_B, N_CFG_TICKS_B, tb,
            "S-function parameter 7 (cfg_ticks_b) must be a real 1-D 2-element vector of integers in [0, 4294967295].")) {
        return 0;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->dt_s = f[0];
    cfg->outer_timeout_s = f[1];
    cfg->max_physics_residual = f[2];
    cfg->residual_clear_ratio = f[3];
    cfg->min_ai_confidence = f[4];
    cfg->confidence_clear_ratio = f[5];
    cfg->state_safety_limit = f[6];
    cfg->safety_trip_ratio = f[7];
    cfg->safety_clear_ratio = f[8];
    cfg->safety_predict_s = f[9];
    cfg->u_max = f[10];
    cfg->u_slew_max_per_s = f[11];
    cfg->blend_alpha = f[12];
    cfg->integ_limit = f[13];
    cfg->windup_tau_s = f[14];

    cfg->min_dwell_ticks = ta[0];
    cfg->ood_recovery_cooldown_ticks = ta[1];

    cfg->kp[0] = kp[0];
    cfg->kp[1] = kp[1];
    cfg->kp[2] = kp[2];
    cfg->kp[3] = kp[3];
    cfg->kd[0] = kd[0];
    cfg->kd[1] = kd[1];
    cfg->kd[2] = kd[2];
    cfg->kd[3] = kd[3];
    cfg->ki[0] = ki[0];
    cfg->ki[1] = ki[1];
    cfg->ki[2] = ki[2];
    cfg->ki[3] = ki[3];
    cfg->ff_scale[0] = ff[0];
    cfg->ff_scale[1] = ff[1];
    cfg->ff_scale[2] = ff[2];
    cfg->ff_scale[3] = ff[3];

    cfg->max_hold_ticks = tb[0];
    cfg->max_fault_vetoes_before_latch = tb[1];
    return 1;
}

static int_T lg_sfun_load_tuning(SimStruct *S, FieldTuning_t *tune)
{
    float t[N_TUNING];

    if (!lg_sfun_read_float_vector(S, P_TUNING, N_TUNING, t,
            "S-function parameter 8 (tuning) must be a real 1-D 3-element vector.")) {
        return 0;
    }
    /* Assignment in frozen field order: gain, damping, safety margin. */
    tune->gain_scale = t[0];
    tune->damping_scale = t[1];
    tune->safety_margin_scale = t[2];
    return 1;
}

/* Block sample time: the commissioned inner-loop period, cfg_float(1).  An
 * unusable value inherits the model rate; the core then rejects the invalid
 * configuration itself (corruption path) rather than this adapter guessing. */
static real_T lg_sfun_sample_time(SimStruct *S)
{
    const mxArray *p = ssGetSFcnParam(S, P_CFG_FLOAT);
    real_T dt;

    if (p != NULL && mxIsDouble(p) && !mxIsComplex(p) && !mxIsSparse(p) &&
        mxGetNumberOfElements(p) >= 1) {
        dt = mxGetPr(p)[0];
        if (dt == dt && dt > 0.0 && dt < 1.0e6) {
            return dt;
        }
    }
    return (real_T)(-1.0); /* INHERITED_SAMPLE_TIME */
}

/* ========================================================================= */
/* Simulink callbacks                                                        */
/* ========================================================================= */

#ifdef MDL_CHECK_PARAMETERS
#define MDL_CHECK_PARAMETERS
static void mdlCheckParameters(SimStruct *S)
{
    LayeredConfig_t cfg;
    FieldTuning_t tune;

    /* The same readers used at start validate the block parameters here, so a
     * matrix-shaped vector or a non-finite/fractional tick budget is reported
     * when the block is configured or the model is updated, not only when the
     * simulation runs.  The readers raise the error message themselves. */
    if (!lg_sfun_load_config(S, &cfg) || !lg_sfun_load_tuning(S, &tune)) {
        return;
    }
}
#endif

static void mdlInitializeSizes(SimStruct *S)
{
    int_T i;

    ssSetNumSFcnParams(S, P_NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Simulink reports the parameter-count mismatch with its own dialog. */
        return;
    }
#ifdef MDL_CHECK_PARAMETERS
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) {
        return;
    }
#endif
    for (i = 0; i < P_NUM_PARAMS; ++i) {
        ssSetSFcnParamTunable(S, i, 0);
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, N_INPUTS)) {
        return;
    }
    /* Port names are not part of the C-MEX API in R2024b; the mapping above
     * (IN_*) is by port index and is documented in the file header. */
    for (i = 0; i < N_INPUTS; ++i) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDataType(S, i, SS_DOUBLE);
        ssSetInputPortDirectFeedThrough(S, i, 1);
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    if (!ssSetNumOutputPorts(S, N_OUTPUTS)) {
        return;
    }
    for (i = 0; i < N_OUTPUTS; ++i) {
        ssSetOutputPortWidth(S, i, 1);
        ssSetOutputPortDataType(S, i, SS_DOUBLE);
    }

    ssSetNumDWork(S, N_DWORK);
    ssSetDWorkWidth(S, DW_STATE, DW_WORDS_STATE);
    ssSetDWorkDataType(S, DW_STATE, SS_UINT32);
    ssSetDWorkName(S, DW_STATE, "core_state");
    ssSetDWorkWidth(S, DW_CONFIG, DW_WORDS_CONFIG);
    ssSetDWorkDataType(S, DW_CONFIG, SS_UINT32);
    ssSetDWorkName(S, DW_CONFIG, "core_config");
    ssSetDWorkWidth(S, DW_TUNING, DW_WORDS_TUNING);
    ssSetDWorkDataType(S, DW_TUNING, SS_UINT32);
    ssSetDWorkName(S, DW_TUNING, "field_tuning");
    ssSetDWorkWidth(S, DW_BOOK, DW_WORDS_BOOK);
    ssSetDWorkDataType(S, DW_BOOK, SS_DOUBLE);
    ssSetDWorkName(S, DW_BOOK, "ack_bookkeeping");

    ssSetNumSampleTimes(S, 1);

    /* mdlOutputs is straight-line arithmetic on DWork and fixed-size locals:
     * no allocation, no I/O, no library call that can raise.  The frozen core
     * guarantees the same, so the exception-free contract holds here too. */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, lg_sfun_sample_time(S));
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    LayeredState_t *state = (LayeredState_t *)ssGetDWork(S, DW_STATE);
    LayeredConfig_t *cfg = (LayeredConfig_t *)ssGetDWork(S, DW_CONFIG);
    FieldTuning_t *tune = (FieldTuning_t *)ssGetDWork(S, DW_TUNING);
    real_T *book = (real_T *)ssGetDWork(S, DW_BOOK);

    if (!lg_sfun_load_config(S, cfg) || !lg_sfun_load_tuning(S, tune)) {
        return;
    }
    book[0] = 0.0;
    Layered_Init(state, cfg, tune);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    LayeredState_t *state = (LayeredState_t *)ssGetDWork(S, DW_STATE);
    const LayeredConfig_t *cfg = (const LayeredConfig_t *)ssGetDWork(S, DW_CONFIG);
    const FieldTuning_t *tune = (const FieldTuning_t *)ssGetDWork(S, DW_TUNING);
    real_T *book = (real_T *)ssGetDWork(S, DW_BOOK);
    const real_T *sig;
    LayeredInput_t in;
    LayeredOutput_t out;
    int_T ack_on;

    (void)tid; /* single sample time: nothing rate-specific to do */

    memset(&in, 0, sizeof(in));
    in.ai_regime = lg_sfun_to_float(ssGetInputPortRealSignal(S, IN_AI_REGIME)[0]);
    in.ai_confidence = lg_sfun_to_float(ssGetInputPortRealSignal(S, IN_AI_CONFIDENCE)[0]);
    in.outer_age_s = lg_sfun_to_float(ssGetInputPortRealSignal(S, IN_OUTER_AGE_S)[0]);
    in.physics_residual = lg_sfun_to_float(ssGetInputPortRealSignal(S, IN_PHYSICS_RESIDUAL)[0]);
    in.x = lg_sfun_to_float(ssGetInputPortRealSignal(S, IN_X)[0]);
    in.x_dot = lg_sfun_to_float(ssGetInputPortRealSignal(S, IN_X_DOT)[0]);
    in.r = lg_sfun_to_float(ssGetInputPortRealSignal(S, IN_R)[0]);
    in.r_dot = lg_sfun_to_float(ssGetInputPortRealSignal(S, IN_R_DOT)[0]);
    in.u_ff = lg_sfun_to_float(ssGetInputPortRealSignal(S, IN_U_FF)[0]);
    in.outer_seq = lg_sfun_to_u32(ssGetInputPortRealSignal(S, IN_OUTER_SEQ)[0]);

    /* External acknowledgement: a finite level above zero is HIGH, every other
     * finite level (zero, negatives) is LOW, and a rising edge is applied
     * before this tick's step so the release takes effect immediately.  A
     * non-finite sample is ignored and the stored level is left untouched, so
     * a forced 1 -> NaN -> 1 sequence cannot re-trigger the API.  The API
     * validates the configuration itself and ignores the call when it is
     * corrupt. */
    sig = ssGetInputPortRealSignal(S, IN_ACK);
    if (lg_sfun_is_finite(sig[0])) {
        ack_on = lg_sfun_ack_asserted(sig[0]);
        if (ack_on != 0 && book[0] == 0.0) {
            (void)Layered_AcknowledgeFaultLatch(state, cfg);
        }
        book[0] = (ack_on != 0) ? 1.0 : 0.0;
    }

    out = Layered_Step(state, cfg, tune, &in);

    ssGetOutputPortRealSignal(S, OUT_U)[0] = (real_T)out.u;
    ssGetOutputPortRealSignal(S, OUT_U_TARGET)[0] = (real_T)out.u_target;
    ssGetOutputPortRealSignal(S, OUT_GATE)[0] = (real_T)out.gate;
    ssGetOutputPortRealSignal(S, OUT_REGIME)[0] = (real_T)out.regime;
    ssGetOutputPortRealSignal(S, OUT_BLEND)[0] = (real_T)out.blend;
    ssGetOutputPortRealSignal(S, OUT_FAULT_AUDIT)[0] = (real_T)out.fault_audit_bitmask;
    ssGetOutputPortRealSignal(S, OUT_CORRUPTED)[0] =
        (out.corrupted != 0) ? (real_T)1.0 : (real_T)0.0;
}

static void mdlTerminate(SimStruct *S)
{
    /* All persistence lives in DWork; nothing was allocated, nothing to free. */
    (void)S;
}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c"  /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"   /* Code generation registration function */
#endif
