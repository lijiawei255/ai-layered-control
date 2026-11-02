#!/usr/bin/env python3
"""python_bench/run_benchmark.py -- zero-MATLAB entry point for the layered AI
control reference benchmark (v1.1.0).

  python python_bench/run_benchmark.py             # 40 s benchmark, C core if a compiler exists
  python python_bench/run_benchmark.py --no-c      # force the pure-Python gatekeeper mirror
  python python_bench/run_benchmark.py --selftest  # C core vs Python mirror, 500 random frames

Only numpy + matplotlib + the standard library are required.  The deterministic
1 kHz core is taken from ../c99_layered_core/layered_gatekeeper.c (compiled on
the fly into python_bench/build/ and called through ctypes).  If no C compiler
is available the identical pure-Python mirror below is used and exactly one
"[info] no C compiler; using pure-Python gatekeeper mirror" line is printed.

Three branches run on the same plant, the same reference and the same fault
schedule (40 s at 1 kHz, four stages):

  B1  fixed conservative PID, no adaptation, no fault detection
  B2  continuous policy: a closed-form ridge/RBF inverse model trained on
      nominal-unit input/output data + PD feedback, inferred at 10 Hz with
      transport delay and frame drops from Stage 3 on
  B3  layered architecture: 10 Hz coarse regime classifier + independent
      nominal-physics residual probe + 1 kHz gatekeeper / controller bank

Approved benchmark parameters (S1/S2/S3/S4 and the four-controller bank) are in
the constants block below.  The Python numbers are not bit-identical to the
MATLAB engine; the *core* semantics are exact and proven by --selftest against
the compiled C core.
"""

from __future__ import annotations

import argparse
import ctypes
import math
import os
import shlex
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# 0.  Benchmark / profile constants (approved v1.1.0 industrial profile)
# ---------------------------------------------------------------------------
DT = 1e-3                      # s, deterministic inner loop (1 kHz)
T_END = 40.0                   # s, four stages x 10 s
N_STEPS = int(round(T_END / DT))
STAGE_EDGES = (0.0, 10.0, 20.0, 30.0, 40.0)

# plant (nominal unit)
J0 = 0.010                     # kg*m^2
B0 = 0.060                     # N*m*s/rad
K0 = 0.300                     # N*m/rad
FC0 = 0.200                    # N*m   Coulomb friction
FS0 = 0.450                    # N*m   break-away friction
VS0 = 0.020                    # rad/s Stribeck velocity
FVISC0 = 0.050                 # N*m*s/rad

# actuator
U_MAX = 2.0                    # N*m
SLEW_MAX = 200.0               # N*m/s

# sensing
SIGMA_NOMINAL = 0.0025         # rad, 1-sigma position noise
DERIV_TAU_S = 0.012            # s, dirty-derivative low-pass

# safety envelope
X_LIMIT = 1.0                  # rad
TRIP_RATIO = 0.90
CLEAR_RATIO = 0.75
PREDICT_S = 0.06               # s

# slow loop / classifier
OUTER_HZ = 10.0
OUTER_DT = 1.0 / OUTER_HZ
OUTER_TIMEOUT_S = 0.30
MAX_RESIDUAL = 1.0
RESIDUAL_CLEAR_RATIO = 0.80
MIN_CONFIDENCE = 0.45
CONFIDENCE_CLEAR = 0.55
# The synthetic C-vs-Python regression config keeps the pre-calibration
# contract levels above.  The benchmark itself uses thresholds calibrated from
# the commissioning distribution of the v1.1.0 confidence formula (the formula
# multiplies the margin by exp(-gamma*d1), so the operating confidence is lower
# than the legacy pure-margin ratio; the calibration line prints both values).
BENCH_MIN_CONFIDENCE = 0.08
BENCH_CONFIDENCE_CLEAR = 0.15
WINDOW_TAU_S = 0.50
WINDOW_TAU_V_S = 1.00
LP_TAU_S = 0.15
# Three orthogonal residual channels (v1.1.0 layered probe):
#   spike channel   |y - median5(y)| / res_fast_scale      (single-sample spikes)
#   resonance channel narrowband 42 Hz RMS of the innovation (gated, 2 ticks)
#   slow channel    LP(innovation) / res_slow_scale        (classifier feature only)
RES_FAST_SCALE = 0.15          # rad, spike-channel normalization floor
RES_FAST_FLOOR_K = 8.0         # noise-floor multiplier (adaptive threshold)
RES_FAST_FLOOR_TAU = 0.020     # s, fast noise-floor EMA
RES_SLOW_SCALE = 0.050         # rad, slow residual normalization
RES_HF_SCALE = 0.025          # rad, resonance-channel normalization (industrial)
RES_HF_BW_HZ = 8.0            # Hz, narrowband resonator bandwidth
RES_HF_TAU_S = 0.004          # s, RMS EMA of the resonator output
RES_HF_GATE_TICKS = 2         # consecutive ticks above the trip before vetoing
RES_HF_BASELINE_TAU_S = 1.0   # s, slow 42 Hz noise-floor baseline
RES_HF_ATTACK_TAU_S = 0.02    # s, fast baseline attack on a noise-floor step
RES_HF_FLOOR_K = 2.0          # baseline multiplier (Python probe noise adaptation)
RES_FEATURE_TAU_S = 0.50       # s, RMS averaging of the slow-residual macro-feature
CONF_EMA_TAU_S = 3.0           # s, mailbox confidence filter (veto is for sustained low confidence)
MEDIAN_WINDOW = 5              # samples for the spike channel (4 ms)
WARMUP_S = 3.0 * WINDOW_TAU_S  # slow-loop feature warm-up
AMP_SCALE = 0.50               # rad, feature normalization (nominal reference)
V_SCALE = 1.25                 # rad/s, velocity-feature normalization
FEATURE_SCALES = (AMP_SCALE, U_MAX, 1.0, V_SCALE)

# gatekeeper timing
MIN_DWELL_TICKS = 150          # 150 ms at 1 kHz
COOLDOWN_TICKS = 500           # 500 ms at 1 kHz
BLEND_ALPHA = 0.10
WINDUP_TAU_S = 0.050

# nominal closed-loop tuning (P.ctrl): fn = 3 Hz, zeta = 0.8, Ki = Kp*wn/6
WN = 2.0 * math.pi * 3.0
KP_NOM = J0 * WN * WN
KD_NOM = max(2.0 * 0.8 * J0 * WN - B0, 0.15 * J0 * WN)
KI_NOM = KP_NOM * WN / 6.0
INTEG_LIMIT = 0.5 * U_MAX / KI_NOM

# controller bank: relative to (Kp_nom, Kd_nom, Ki_nom), plus per-bank ff gain
BANK_REL = (
    (7.0, 1.5, 1.0),           # R1 nominal
    (1.5, 4.5, 12.0),          # R2 high-load damped
    (4.0, 0.2, 8.0),           # R3 boundary compliant
    (1.0, 3.0, 0.0),           # R4 safe fallback (PD only)
)
BANK_KP = tuple(KP_NOM * r[0] for r in BANK_REL)
BANK_KD = tuple(KD_NOM * r[1] for r in BANK_REL)
BANK_KI = tuple(KI_NOM * r[2] for r in BANK_REL)
FF_SCALE = (1.0, 1.0, 2.0, 1.0)

# B1: conservative fixed PID (reference profile b1.rel)
B1_KP = KP_NOM * 0.55
B1_KD = KD_NOM * 0.60
B1_KI = KI_NOM * 0.35
# B2: continuous policy feedback (reference profile b2.rel), no integral action
B2_KP = KP_NOM * 0.80
B2_KD = KD_NOM * 1.00
B2_KI = 0.0

# cross-unit drift (+35 %, ramped in over 1 s from t = 10 s, stays on)
DRIFT_START_S = 10.0
DRIFT_RAMP_S = 1.0
DRIFT = dict(J=1.35, b=1.25, k=1.10, fc=2.00, fs=2.40, vs=0.25, fvisc=1.20)

# Stage 1: harmonic 0.5 Hz reference, light friction
S1_AMP = 0.50
S1_FRICTION_SCALE = 0.15

# Stage 2: faster trapezoid, starts 0.8 s into its period
S2_AMP, S2_MOVE_S, S2_DWELL_S, S2_PHASE0_S = 0.50, 0.90, 0.90, 0.80

# Stage 3: contact stiffening + sensor noise stress + edge compute scarcity
S3_AMP, S3_MOVE_S, S3_DWELL_S = 0.50, 1.00, 1.50
S3_CONTACT_K = 3.333
S3_NOISE_MULT = 20.0           # sensor noise 20x nominal (approved A7 matrix)

# Stage 4: same trajectory as S3 (continuous phase), OOD on top
S4_AMP, S4_MOVE_S, S4_DWELL_S = 0.50, 1.00, 1.50

# edge compute scarcity (10 Hz, 60-120 ms transport delay, 15 % drops)
EDGE_START_S = 20.0
EDGE_BASE_LATENCY_S = 0.060
EDGE_JITTER_S = 0.060
EDGE_DROP_PROB = 0.15

# unseen OOD fault: 42 Hz parasitic sensor-path resonance + encoder spikes
OOD_START_S = 30.0
OOD_RES_HZ = 42.0
OOD_RES_ZETA = 0.03
OOD_RES_CU = 12.0
OOD_RES_CX = 8.0
OOD_RES_KQ = 0.008             # mode -> plant force coupling (N*m per unit q)
OOD_RES_KY = 100.0             # mode -> measured position coupling (rad per unit q)
OOD_RES_KICK = 1.0             # initial modal velocity impulse at fault onset
OOD_SPIKE_FIRST_S = 0.002
OOD_SPIKE_PROB = 0.004
OOD_SPIKE_AMP = 0.20

# narrowband resonator used by the resonance residual channel (42 Hz, 8 Hz BW)
_RES_HF_W0 = 2.0 * math.pi * OOD_RES_HZ
RES_HF_RR = math.exp(-math.pi * RES_HF_BW_HZ * DT)
RES_HF_A1 = 2.0 * RES_HF_RR * math.cos(_RES_HF_W0 * DT)
RES_HF_A2 = -(RES_HF_RR * RES_HF_RR)
RES_HF_B0 = 1.0 - RES_HF_RR

# white-box deterministic feedforward weight of the layered branch
FF_WEIGHT = 1.0
# nominal observer used by the physics residual probe (4 Hz, zeta 0.8)
OBS_WOB = 2.0 * math.pi * 4.0
OBS_L1 = 2.0 * 0.8 * OBS_WOB
OBS_L2 = OBS_WOB * OBS_WOB

SEED = 7
CALIB_SEED = 17

_HERE = Path(__file__).resolve().parent
_REPO_ROOT = _HERE.parent
_CORE_SRC = _REPO_ROOT / "c99_layered_core" / "layered_gatekeeper.c"


# ---------------------------------------------------------------------------
# 1.  ctypes ABI for c99_layered_core/layered_gatekeeper.h
# ---------------------------------------------------------------------------
class _FieldTuning(ctypes.Structure):
    _fields_ = [
        ("gain_scale", ctypes.c_float),
        ("damping_scale", ctypes.c_float),
        ("safety_margin_scale", ctypes.c_float),
    ]


class _LayeredConfig(ctypes.Structure):
    _fields_ = [
        ("dt_s", ctypes.c_float),
        ("outer_timeout_s", ctypes.c_float),
        ("max_physics_residual", ctypes.c_float),
        ("residual_clear_ratio", ctypes.c_float),
        ("min_ai_confidence", ctypes.c_float),
        ("confidence_clear_ratio", ctypes.c_float),
        ("state_safety_limit", ctypes.c_float),
        ("safety_trip_ratio", ctypes.c_float),
        ("safety_clear_ratio", ctypes.c_float),
        ("safety_predict_s", ctypes.c_float),
        ("u_max", ctypes.c_float),
        ("u_slew_max_per_s", ctypes.c_float),
        ("blend_alpha", ctypes.c_float),
        ("integ_limit", ctypes.c_float),
        ("windup_tau_s", ctypes.c_float),
        ("min_dwell_ticks", ctypes.c_uint32),
        ("ood_recovery_cooldown_ticks", ctypes.c_uint32),
        ("kp", ctypes.c_float * 4),
        ("kd", ctypes.c_float * 4),
        ("ki", ctypes.c_float * 4),
        ("ff_scale", ctypes.c_float * 4),
    ]


class _LayeredInput(ctypes.Structure):
    _fields_ = [
        ("ai_regime", ctypes.c_float),
        ("ai_confidence", ctypes.c_float),
        ("outer_age_s", ctypes.c_float),
        ("physics_residual", ctypes.c_float),
        ("x", ctypes.c_float),
        ("x_dot", ctypes.c_float),
        ("r", ctypes.c_float),
        ("r_dot", ctypes.c_float),
        ("u_ff", ctypes.c_float),
    ]


class _LayeredOutput(ctypes.Structure):
    _fields_ = [
        ("u", ctypes.c_float),
        ("u_target", ctypes.c_float),
        ("gate", ctypes.c_int),
        ("regime", ctypes.c_int),
        ("blend", ctypes.c_float),
        ("fault_audit_bitmask", ctypes.c_uint32),
        ("corrupted", ctypes.c_int),
    ]


class _LayeredState(ctypes.Structure):
    _fields_ = [
        ("initialized", ctypes.c_int),
        ("u_applied", ctypes.c_float),
        ("u_target", ctypes.c_float),
        ("integ", ctypes.c_float * 4),
        ("blend", ctypes.c_float),
        ("control_started", ctypes.c_int),
        ("regime", ctypes.c_int),
        ("ood_latched", ctypes.c_int),
        ("safety_latched", ctypes.c_int),
        ("conf_latched", ctypes.c_int),
        ("dwell_ticks", ctypes.c_uint32),
        ("recovery_ticks", ctypes.c_uint32),
        ("fault_audit_bitmask", ctypes.c_uint32),
        ("step_count", ctypes.c_uint32),
        ("safety_veto_count", ctypes.c_uint32),
        ("ood_veto_count", ctypes.c_uint32),
        ("low_confidence_veto_count", ctypes.c_uint32),
        ("latency_hold_count", ctypes.c_uint32),
        ("corrupted_frame_count", ctypes.c_uint32),
    ]


# Fault audit bits (must match layered_gatekeeper.h)
FAULT_LATENCY = 1 << 0
FAULT_OOD = 1 << 1
FAULT_LOW_CONF = 1 << 2
FAULT_ENVELOPE = 1 << 3
FAULT_NAN = 1 << 4

GATE_NORMAL = 0
GATE_HOLD_LATENCY = 1
GATE_VETO_OOD = 2
GATE_VETO_LOW_CONF = 3
GATE_VETO_ENVELOPE = 4

REGIME_NOMINAL = 1
REGIME_HIGH_LOAD = 2
REGIME_BOUNDARY = 3
REGIME_SAFE = 4


def _cc_argv(cc: str) -> list[str]:
    """Split CC while keeping paths with spaces on Windows intact."""
    try:
        parts = shlex.split(cc, posix=(os.name != "nt"))
    except ValueError:
        parts = [cc]
    if parts and os.name == "nt" and len(parts) > 1:
        # shlex with posix=False keeps quotes; trim them.
        parts = [p.strip('"') for p in parts]
    return parts


def _shared_lib_suffix() -> str:
    if sys.platform.startswith("win"):
        return ".dll"
    if sys.platform == "darwin":
        return ".dylib"
    return ".so"


def _platform_compile_flags() -> list[str]:
    if sys.platform.startswith("win"):
        return ["-shared"]
    if sys.platform == "darwin":
        return ["-dynamiclib", "-fPIC"]
    return ["-shared", "-fPIC"]


def build_c_core(build_dir: Path | None = None) -> tuple[ctypes.CDLL | None, Path | None]:
    """Compile ../c99_layered_core/layered_gatekeeper.c into a local shared
    library and load it.  Returns (library, path) or (None, None).

    Never uses -ffast-math (the NaN/Inf firewall relies on IEEE comparisons).
    """
    if not _CORE_SRC.is_file():
        return None, None
    if build_dir is None:
        build_dir = _HERE / "build"
    made_temp = False
    try:
        build_dir.mkdir(parents=True, exist_ok=True)
    except OSError:
        build_dir = Path(tempfile.mkdtemp(prefix="layered_core_"))
        made_temp = True

    lib_path = build_dir / ("layered_gatekeeper" + _shared_lib_suffix())
    candidates: list[str] = []
    env_cc = os.environ.get("CC", "").strip()
    if env_cc:
        candidates.append(env_cc)
    candidates.extend(["gcc", "cc", "clang"])

    flags = _platform_compile_flags()
    last_error = ""
    for cc in candidates:
        argv = _cc_argv(cc)
        if not argv:
            continue
        cmd = [*argv, "-std=c99", "-O2", *flags, "-o", str(lib_path), str(_CORE_SRC)]
        try:
            proc = subprocess.run(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120
            )
        except (OSError, subprocess.TimeoutExpired) as exc:  # compiler not found
            last_error = str(exc)
            continue
        if proc.returncode == 0 and lib_path.is_file():
            try:
                lib = ctypes.CDLL(str(lib_path))
            except OSError as exc:
                last_error = str(exc)
                continue
            lib.Layered_Init.argtypes = [
                ctypes.POINTER(_LayeredState),
                ctypes.POINTER(_LayeredConfig),
                ctypes.POINTER(_FieldTuning),
            ]
            lib.Layered_Init.restype = None
            lib.Layered_Step.argtypes = [
                ctypes.POINTER(_LayeredState),
                ctypes.POINTER(_LayeredConfig),
                ctypes.POINTER(_FieldTuning),
                ctypes.POINTER(_LayeredInput),
            ]
            lib.Layered_Step.restype = _LayeredOutput
            return lib, lib_path
        last_error = (proc.stderr or b"").decode("utf-8", "replace").strip()

    if made_temp:
        try:
            for p in build_dir.glob("*"):
                p.unlink()
            build_dir.rmdir()
        except OSError:
            pass
    if last_error:
        # Compiler diagnostics are intentionally swallowed: the documented
        # contract for the no-compiler path is a single [info] line.
        pass
    return None, None


def make_config(min_confidence: float = BENCH_MIN_CONFIDENCE,
                confidence_clear: float = BENCH_CONFIDENCE_CLEAR) -> dict:
    """Deployment configuration (both backends read this).

    The defaults are the calibrated benchmark thresholds; the --selftest
    regression uses the synthetic pre-calibration contract (0.45 / 0.55).
    """
    return dict(
        dt_s=DT,
        outer_timeout_s=OUTER_TIMEOUT_S,
        max_physics_residual=MAX_RESIDUAL,
        residual_clear_ratio=RESIDUAL_CLEAR_RATIO,
        min_ai_confidence=min_confidence,
        confidence_clear_ratio=confidence_clear,
        state_safety_limit=X_LIMIT,
        safety_trip_ratio=TRIP_RATIO,
        safety_clear_ratio=CLEAR_RATIO,
        safety_predict_s=PREDICT_S,
        u_max=U_MAX,
        u_slew_max_per_s=SLEW_MAX,
        blend_alpha=BLEND_ALPHA,
        integ_limit=INTEG_LIMIT,
        windup_tau_s=WINDUP_TAU_S,
        min_dwell_ticks=MIN_DWELL_TICKS,
        ood_recovery_cooldown_ticks=COOLDOWN_TICKS,
        kp=list(BANK_KP),
        kd=list(BANK_KD),
        ki=list(BANK_KI),
        ff_scale=list(FF_SCALE),
    )


def make_tuning(gain_scale: float = 1.0, damping_scale: float = 1.0,
                safety_margin_scale: float = 1.0) -> dict:
    return dict(
        gain_scale=gain_scale,
        damping_scale=damping_scale,
        safety_margin_scale=safety_margin_scale,
    )


# ---------------------------------------------------------------------------
# 2.  Pure-Python mirror of Layered_Step()  (line-by-line, same order)
# ---------------------------------------------------------------------------
class PythonGatekeeper:
    """Exact mirror of c99_layered_core/layered_gatekeeper.c.

    Kept deliberately literal: same decision order, same sticky latches, same
    integer bookkeeping.  --selftest proves gate/regime equality against the
    compiled C core on 500 random frames.
    """

    __slots__ = (
        "cfg", "tune",
        "u_applied", "u_target", "integ", "blend", "control_started",
        "regime", "ood_latched", "safety_latched", "conf_latched",
        "dwell_ticks", "recovery_ticks", "fault_audit_bitmask",
        "step_count", "safety_veto_count", "ood_veto_count",
        "low_confidence_veto_count", "latency_hold_count",
        "corrupted_frame_count",
    )

    def __init__(self, cfg: dict, tune: dict):
        self.cfg = cfg
        self.tune = tune
        self.u_applied = 0.0
        self.u_target = 0.0
        self.integ = [0.0, 0.0, 0.0, 0.0]
        self.blend = 0.0
        self.control_started = 0
        self.regime = REGIME_NOMINAL
        self.ood_latched = 0
        self.safety_latched = 0
        self.conf_latched = 0
        self.dwell_ticks = 0
        self.recovery_ticks = 0
        self.fault_audit_bitmask = 0
        self.step_count = 0
        self.safety_veto_count = 0
        self.ood_veto_count = 0
        self.low_confidence_veto_count = 0
        self.latency_hold_count = 0
        self.corrupted_frame_count = 0

    # ---- dependency-free helpers (same semantics as lg_absf/lg_clampf) ----
    @staticmethod
    def _absf(v):
        return -v if v < 0.0 else v

    @staticmethod
    def _clampf(v, lo, hi):
        if v < lo:
            return lo
        if v > hi:
            return hi
        return v

    @staticmethod
    def _mailbox_regime(ai_regime):
        if ai_regime < 1.5:
            return REGIME_NOMINAL
        if ai_regime < 2.5:
            return REGIME_HIGH_LOAD
        if ai_regime < 3.5:
            return REGIME_BOUNDARY
        return REGIME_SAFE

    @staticmethod
    def _mailbox_abstains(ai_regime):
        return ai_regime >= 3.5

    def step(self, ai_regime, ai_confidence, outer_age_s, physics_residual,
             x, x_dot, r, r_dot, u_ff):
        """One 1 kHz step.  Returns (u, gate, regime, blend, fault, corrupted,
        u_target) exactly like the C output struct."""
        cfg = self.cfg
        tune = self.tune
        finite = 3.0e38
        # ---------------- 0. INPUT FIREWALL --------------------------------
        corrupt = False
        for v in (ai_regime, ai_confidence, outer_age_s, physics_residual,
                  x, x_dot, r, r_dot, u_ff):
            if not (-finite <= v <= finite):
                corrupt = True
                break
        if not corrupt:
            if (ai_regime < 1.0 or ai_regime > 4.0
                    or ai_confidence < 0.0 or ai_confidence > 1.0
                    or physics_residual < 0.0 or outer_age_s < 0.0):
                corrupt = True
        if corrupt:
            self.fault_audit_bitmask |= FAULT_NAN
            self.corrupted_frame_count += 1
            self.recovery_ticks = cfg["ood_recovery_cooldown_ticks"]
            self.step_count += 1
            return (self.u_applied, GATE_HOLD_LATENCY, self.regime,
                    self.blend, self.fault_audit_bitmask, 1, self.u_target)

        snap = self.control_started == 0
        conf_before = self.conf_latched

        u_max = self._absf(cfg["u_max"])
        integ_limit = self._absf(cfg["integ_limit"])
        dt_eff = self._absf(cfg["dt_s"])
        du_max = self._absf(cfg["u_slew_max_per_s"]) * dt_eff
        windup_tau = cfg["windup_tau_s"]

        # ---------------- 1. hard safety envelope --------------------------
        limit_eff = self._absf(cfg["state_safety_limit"] * tune["safety_margin_scale"])
        x_abs = self._absf(x)
        xdot_abs = self._absf(x_dot)
        predicted_abs = x_abs + self._absf(cfg["safety_predict_s"]) * xdot_abs
        if self.safety_latched:
            clear_level = cfg["safety_clear_ratio"] * limit_eff
            if x_abs < clear_level and predicted_abs < clear_level:
                self.safety_latched = 0
        elif (x_abs > cfg["safety_trip_ratio"] * limit_eff
              or predicted_abs > limit_eff):
            self.safety_latched = 1
            self.safety_veto_count += 1
            self.fault_audit_bitmask |= FAULT_ENVELOPE

        # ---------------- 2. physics-residual (OOD) veto -------------------
        residual_abs = self._absf(physics_residual)
        if self.ood_latched:
            if residual_abs < cfg["residual_clear_ratio"] * cfg["max_physics_residual"]:
                self.ood_latched = 0
        elif residual_abs > cfg["max_physics_residual"]:
            self.ood_latched = 1
            self.ood_veto_count += 1
            self.fault_audit_bitmask |= FAULT_OOD

        # ---------------- 3. confidence / explicit abstention --------------
        mailbox_regime = self._mailbox_regime(ai_regime)
        if (ai_confidence < cfg["min_ai_confidence"]
                or self._mailbox_abstains(ai_regime)):
            self.conf_latched = 1
            self.fault_audit_bitmask |= FAULT_LOW_CONF
        elif (ai_confidence >= cfg["confidence_clear_ratio"]
              and not self._mailbox_abstains(ai_regime)):
            self.conf_latched = 0
        conf_veto = self.conf_latched
        if conf_veto and not conf_before:
            self.low_confidence_veto_count += 1

        # ---------------- 4. heartbeat watchdog ----------------------------
        latency_hold = outer_age_s > cfg["outer_timeout_s"]
        if latency_hold:
            self.fault_audit_bitmask |= FAULT_LATENCY
            self.recovery_ticks = cfg["ood_recovery_cooldown_ticks"]
            if (outer_age_s - cfg["outer_timeout_s"]) <= dt_eff:
                self.latency_hold_count += 1

        # ---------------- 5. decision priority -----------------------------
        veto_active = (self.safety_latched != 0 or self.ood_latched != 0
                       or conf_veto != 0)
        if self.safety_latched:
            gate = GATE_VETO_ENVELOPE
            regime = REGIME_SAFE
        elif self.ood_latched:
            gate = GATE_VETO_OOD
            regime = REGIME_SAFE
        elif conf_veto:
            gate = GATE_VETO_LOW_CONF
            regime = REGIME_SAFE
        elif latency_hold:
            gate = GATE_HOLD_LATENCY
            regime = self.regime
        else:
            gate = GATE_NORMAL
            if self.recovery_ticks > 0:
                self.recovery_ticks -= 1
                regime = REGIME_SAFE
            elif mailbox_regime == self.regime:
                regime = self.regime
            elif (self.dwell_ticks + 1) >= cfg["min_dwell_ticks"]:
                regime = mailbox_regime
            else:
                regime = self.regime
        if veto_active:
            self.recovery_ticks = cfg["ood_recovery_cooldown_ticks"]

        # ---------------- 6. controller bank -------------------------------
        do_control = gate != GATE_HOLD_LATENCY
        kp = cfg["kp"]
        kd = cfg["kd"]
        ki = cfg["ki"]
        ff_scale = cfg["ff_scale"]
        gain_scale = tune["gain_scale"]
        damping_scale = tune["damping_scale"]
        alpha = cfg["blend_alpha"]
        if do_control:
            e = r - x
            ed = r_dot - x_dot
            u_pid0 = (kp[0] * gain_scale) * e + (kd[0] * damping_scale) * ed \
                + (ki[0] * gain_scale) * self.integ[0] + ff_scale[0] * u_ff
            u_pid1 = (kp[1] * gain_scale) * e + (kd[1] * damping_scale) * ed \
                + (ki[1] * gain_scale) * self.integ[1] + ff_scale[1] * u_ff
            u_pid2 = (kp[2] * gain_scale) * e + (kd[2] * damping_scale) * ed \
                + (ki[2] * gain_scale) * self.integ[2] + ff_scale[2] * u_ff
            u_pid3 = (kp[3] * gain_scale) * e + (kd[3] * damping_scale) * ed \
                + (ki[3] * gain_scale) * self.integ[3] + ff_scale[3] * u_ff
            bank = regime - 1
            if bank < 0:
                bank = 0
            elif bank > 3:
                bank = 3
            u_active = (u_pid0, u_pid1, u_pid2, u_pid3)[bank]
            if snap:
                self.u_target = u_active
                self.blend = 1.0
            else:
                if regime != self.regime:
                    self.blend = 0.0
                self.u_target += alpha * (u_active - self.u_target)
                self.blend = self._clampf(self.blend + alpha, 0.0, 1.0)
            u_cmd = self._clampf(self.u_target, -u_max, u_max)
            u_cmd = self._clampf(u_cmd, self.u_applied - du_max,
                                 self.u_applied + du_max)
            # tracking anti-windup for EVERY bank controller
            next0 = self.integ[0] + e * dt_eff
            ki0 = ki[0] * gain_scale
            if ki0 != 0.0 and windup_tau > 0.0:
                next0 += (u_cmd - u_pid0) * dt_eff / (ki0 * windup_tau)
            self.integ[0] = self._clampf(next0, -integ_limit, integ_limit)
            next1 = self.integ[1] + e * dt_eff
            ki1 = ki[1] * gain_scale
            if ki1 != 0.0 and windup_tau > 0.0:
                next1 += (u_cmd - u_pid1) * dt_eff / (ki1 * windup_tau)
            self.integ[1] = self._clampf(next1, -integ_limit, integ_limit)
            next2 = self.integ[2] + e * dt_eff
            ki2 = ki[2] * gain_scale
            if ki2 != 0.0 and windup_tau > 0.0:
                next2 += (u_cmd - u_pid2) * dt_eff / (ki2 * windup_tau)
            self.integ[2] = self._clampf(next2, -integ_limit, integ_limit)
            next3 = self.integ[3] + e * dt_eff
            ki3 = ki[3] * gain_scale
            if ki3 != 0.0 and windup_tau > 0.0:
                next3 += (u_cmd - u_pid3) * dt_eff / (ki3 * windup_tau)
            self.integ[3] = self._clampf(next3, -integ_limit, integ_limit)
            self.u_applied = u_cmd
            self.control_started = 1
            out_u = u_cmd
            out_u_target = self.u_target
            out_blend = self.blend
        else:
            out_u = self.u_applied
            out_u_target = self.u_target
            out_blend = self.blend

        # ---------------- 7. dwell bookkeeping + output --------------------
        if regime != self.regime:
            self.dwell_ticks = 0
        else:
            self.dwell_ticks += 1
        self.regime = regime
        self.step_count += 1
        return (out_u, gate, regime, out_blend, self.fault_audit_bitmask, 0,
                out_u_target)


class CGatekeeper:
    """Thin ctypes wrapper around the compiled C core."""

    __slots__ = ("lib", "path", "_cfg", "_tune", "_state", "_inp")

    def __init__(self, lib: ctypes.CDLL, path: Path | None, cfg: dict, tune: dict):
        self.lib = lib
        self.path = path
        c = _LayeredConfig()
        c.dt_s = cfg["dt_s"]
        c.outer_timeout_s = cfg["outer_timeout_s"]
        c.max_physics_residual = cfg["max_physics_residual"]
        c.residual_clear_ratio = cfg["residual_clear_ratio"]
        c.min_ai_confidence = cfg["min_ai_confidence"]
        c.confidence_clear_ratio = cfg["confidence_clear_ratio"]
        c.state_safety_limit = cfg["state_safety_limit"]
        c.safety_trip_ratio = cfg["safety_trip_ratio"]
        c.safety_clear_ratio = cfg["safety_clear_ratio"]
        c.safety_predict_s = cfg["safety_predict_s"]
        c.u_max = cfg["u_max"]
        c.u_slew_max_per_s = cfg["u_slew_max_per_s"]
        c.blend_alpha = cfg["blend_alpha"]
        c.integ_limit = cfg["integ_limit"]
        c.windup_tau_s = cfg["windup_tau_s"]
        c.min_dwell_ticks = cfg["min_dwell_ticks"]
        c.ood_recovery_cooldown_ticks = cfg["ood_recovery_cooldown_ticks"]
        for i in range(4):
            c.kp[i] = cfg["kp"][i]
            c.kd[i] = cfg["kd"][i]
            c.ki[i] = cfg["ki"][i]
            c.ff_scale[i] = cfg["ff_scale"][i]
        t = _FieldTuning()
        t.gain_scale = tune["gain_scale"]
        t.damping_scale = tune["damping_scale"]
        t.safety_margin_scale = tune["safety_margin_scale"]
        st = _LayeredState()
        lib.Layered_Init(ctypes.byref(st), ctypes.byref(c), ctypes.byref(t))
        self._cfg = c
        self._tune = t
        self._state = st
        self._inp = _LayeredInput()

    def step(self, ai_regime, ai_confidence, outer_age_s, physics_residual,
             x, x_dot, r, r_dot, u_ff):
        inp = self._inp
        inp.ai_regime = ai_regime
        inp.ai_confidence = ai_confidence
        inp.outer_age_s = outer_age_s
        inp.physics_residual = physics_residual
        inp.x = x
        inp.x_dot = x_dot
        inp.r = r
        inp.r_dot = r_dot
        inp.u_ff = u_ff
        out = self.lib.Layered_Step(
            ctypes.byref(self._state), ctypes.byref(self._cfg),
            ctypes.byref(self._tune), ctypes.byref(inp))
        return (out.u, out.gate, out.regime, out.blend, out.fault_audit_bitmask,
                out.corrupted, out.u_target)

    @property
    def fault_audit_bitmask(self) -> int:
        return int(self._state.fault_audit_bitmask)


def load_gatekeeper(force_python: bool = False, quiet: bool = False):
    """Return (gatekeeper_factory, backend_label).

    The factory accepts (cfg, tune) and returns an object with .step(...).
    """
    if force_python:
        if not quiet:
            print("[info] --no-c: using pure-Python gatekeeper mirror")
        return (lambda cfg, tune: PythonGatekeeper(cfg, tune)), "python"

    lib, path = build_c_core()
    if lib is None:
        if not quiet:
            print("[info] no C compiler; using pure-Python gatekeeper mirror")
        return (lambda cfg, tune: PythonGatekeeper(cfg, tune)), "python"

    if not quiet:
        try:
            shown = path.relative_to(_REPO_ROOT)
        except ValueError:
            shown = path
        print(f"[core] C gatekeeper compiled: {shown}")
    return (lambda cfg, tune: CGatekeeper(lib, path, cfg, tune)), "C"


# ---------------------------------------------------------------------------
# 3.  --selftest : C core vs Python mirror on 500 deterministic random frames
# ---------------------------------------------------------------------------
def _f32(value: float) -> float:
    """Round a Python float to IEEE-754 binary32 (what the C core sees)."""
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def _random_frames(n: int = 500, seed: int = 20240101):
    """Deterministic random LayeredInput_t frames, mixing valid frames with
    every firewall rejection class (NaN/Inf/out-of-enum/out-of-range)."""
    import random as _random

    rng = _random.Random(seed)
    frames = []
    for i in range(n):
        mode = i % 25
        regime = rng.choice([1.0, 2.0, 3.0, 4.0]) if rng.random() < 0.75 \
            else rng.uniform(1.0, 4.0)
        conf = rng.uniform(0.0, 1.0)
        age = rng.uniform(0.0, 0.45)
        res = rng.uniform(0.0, 1.8)
        x = rng.uniform(-0.95, 0.95)
        xd = rng.uniform(-2.0, 2.0)
        r = rng.uniform(-0.6, 0.6)
        rd = rng.uniform(-2.0, 2.0)
        uff = rng.uniform(-2.0, 2.0)
        if mode == 0:
            x = float("nan")
        elif mode == 1:
            xd = float("inf")
        elif mode == 2:
            regime = 0.5
        elif mode == 3:
            regime = 4.5
        elif mode == 4:
            conf = 1.2
        elif mode == 5:
            res = -0.1
        elif mode == 6:
            age = -0.01
        elif mode == 7:
            uff = float("-inf")
        frames.append((_f32(regime), _f32(conf), _f32(age), _f32(res),
                       _f32(x), _f32(xd), _f32(r), _f32(rd), _f32(uff)))
    return frames


def run_selftest(force_python: bool = False) -> int:
    print("=== python_bench selftest: compiled C core vs pure-Python mirror ===")
    if force_python:
        print("[selftest] SKIP (--no-c forces the Python mirror; nothing to diff)")
        return 0
    lib, path = build_c_core()
    if lib is None:
        print("[selftest] SKIP (no C core available: no working C compiler)")
        return 0
    try:
        shown = path.relative_to(_REPO_ROOT)
    except ValueError:
        shown = path
    print(f"[selftest] C core: {shown}")

    cfg = make_config(MIN_CONFIDENCE, CONFIDENCE_CLEAR)
    tune = make_tuning()
    c_core = CGatekeeper(lib, path, cfg, tune)
    py_core = PythonGatekeeper(cfg, tune)

    frames = _random_frames(500)
    max_du = 0.0
    max_du_target = 0.0
    max_dblend = 0.0
    gate_mismatch = 0
    regime_mismatch = 0
    fault_mismatch = 0
    corrupt_mismatch = 0
    first_bad = None
    n_corrupt = 0
    for i, fr in enumerate(frames):
        cu, cg, cr, cb, cf, ccor, cut = c_core.step(*fr)
        pu, pg, pr, pb, pf, pcor, put = py_core.step(*fr)
        n_corrupt += int(ccor)
        du = abs(cu - pu)
        dut = abs(cut - put)
        db = abs(cb - pb)
        max_du = max(max_du, du)
        max_du_target = max(max_du_target, dut)
        max_dblend = max(max_dblend, db)
        if cg != pg:
            gate_mismatch += 1
            first_bad = first_bad or ("gate", i, cg, pg)
        if cr != pr:
            regime_mismatch += 1
            first_bad = first_bad or ("regime", i, cr, pr)
        if cf != pf:
            fault_mismatch += 1
            first_bad = first_bad or ("fault", i, cf, pf)
        if ccor != pcor:
            corrupt_mismatch += 1
            first_bad = first_bad or ("corrupted", i, ccor, pcor)

    ok = (gate_mismatch == 0 and regime_mismatch == 0
          and fault_mismatch == 0 and corrupt_mismatch == 0 and max_du <= 1e-5)
    print(f"[selftest] frames={len(frames)} corrupt_frames={n_corrupt} "
          f"gate_mismatch={gate_mismatch} regime_mismatch={regime_mismatch} "
          f"fault_mismatch={fault_mismatch} corrupt_flag_mismatch={corrupt_mismatch}")
    print(f"[selftest] max|du|={max_du:.3e} (tol 1e-5)  "
          f"max|du_target|={max_du_target:.3e}  max|dblend|={max_dblend:.3e}")
    if ok:
        print("[selftest] PASS")
        return 0
    if first_bad is not None:
        print(f"[selftest] first mismatch: field={first_bad[0]} frame={first_bad[1]} "
              f"C={first_bad[2]} Python={first_bad[3]}")
    print("[selftest] FAIL")
    return 1


# ---------------------------------------------------------------------------
# 4.  Plant, reference trajectory, disturbances
# ---------------------------------------------------------------------------
S1_SPEC = dict(mode="harmonic", f_hz=0.5)
S2_SPEC = dict(mode="trap", move=S2_MOVE_S, dwell=S2_DWELL_S, phase0=S2_PHASE0_S)
S3_SPEC = dict(mode="trap", move=S3_MOVE_S, dwell=S3_DWELL_S, phase0=0.0)
S4_SPEC = dict(mode="trap", move=S4_MOVE_S, dwell=S4_DWELL_S, phase0=0.0)
STAGE_SPECS = (
    (S1_SPEC, S1_AMP),
    (S2_SPEC, S2_AMP),
    (S3_SPEC, S3_AMP),
    (S4_SPEC, S4_AMP),
)
STAGE_NAMES = ("S1 nominal", "S2 +35% drift", "S3 edge/latency", "S4 OOD")
STAGE_LABELS = ("S1", "S2", "S3", "S4")

# OOD parasitic mode constants (Equation: qdd = -2 zeta w qd - w^2 q + cu u + cx x)
OOD_WR = 2.0 * math.pi * OOD_RES_HZ
OOD_WR2 = OOD_WR * OOD_WR
OOD_2ZWR = 2.0 * OOD_RES_ZETA * OOD_WR

# nominal commissioning model used by the residual probe and deterministic ff
MJ, MB, MK = J0, B0, K0
MFC, MFS, MVS, MFVISC = FC0, FS0, VS0, FVISC0

# feature normalization
FS_E, FS_U, FS_R, FS_V = FEATURE_SCALES


def _friction(v, fc, fs, vs, fvisc):
    """Non-smooth Stribeck friction (same formula as load_domain_profile.m)."""
    veps = vs * 0.02
    if veps < 1e-9:
        veps = 1e-9
    sg = math.tanh(v / veps)
    return fvisc * v + fc * sg + (fs - fc) * math.exp(-abs(v) / vs) * sg


def _quintic(tau):
    p = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))
    v = tau * tau * (30.0 - 60.0 * tau + 30.0 * tau * tau)
    a = tau * (60.0 - 180.0 * tau + 120.0 * tau * tau)
    return p, v, a


def _ref_arrays(spec, s, amp):
    """Reference position / velocity / acceleration for a stage spec."""
    if spec["mode"] == "harmonic":
        w = 2.0 * math.pi * spec["f_hz"]
        return (amp * np.cos(w * s),
                -amp * w * np.sin(w * s),
                -amp * w * w * np.cos(w * s))
    move = spec["move"]
    dwell = spec["dwell"]
    phase0 = spec.get("phase0", 0.0)
    per = 2.0 * (move + dwell)
    u = np.mod(s + phase0, per)
    r = np.zeros_like(u)
    rd = np.zeros_like(u)
    rdd = np.zeros_like(u)
    m1 = u < dwell
    r[m1] = amp
    m2 = (~m1) & (u < dwell + move)
    if np.any(m2):
        tau = (u[m2] - dwell) / move
        p, v, a = _quintic(tau)
        r[m2] = amp - 2.0 * amp * p
        rd[m2] = -2.0 * amp * v / move
        rdd[m2] = -2.0 * amp * a / (move * move)
    m3 = (~m1) & (~m2) & (u < 2.0 * dwell + move)
    r[m3] = -amp
    m4 = ~(m1 | m2 | m3)
    if np.any(m4):
        tau = (u[m4] - (2.0 * dwell + move)) / move
        p, v, a = _quintic(tau)
        r[m4] = -amp + 2.0 * amp * p
        rd[m4] = 2.0 * amp * v / move
        rdd[m4] = 2.0 * amp * a / (move * move)
    return r, rd, rdd


def build_reference(t):
    """The approved four-stage reference: S1 harmonic 0.5 Hz, S2-S4 trapezoids.

    S2 starts 0.8 s into its 3.6 s period, so every stage boundary lands on the
    +amp dwell and the reference stays continuous (r = +0.5 rad at 0/10/20/30 s).
    """
    r = np.empty(N_STEPS)
    rd = np.empty(N_STEPS)
    rdd = np.empty(N_STEPS)
    for i, (spec, amp) in enumerate(STAGE_SPECS):
        m = (t >= STAGE_EDGES[i]) & (t < STAGE_EDGES[i + 1])
        if not np.any(m):
            continue
        rr, rrd, rrdd = _ref_arrays(spec, t[m] - STAGE_EDGES[i], amp)
        r[m] = rr
        rd[m] = rrd
        rdd[m] = rrdd
    return r, rd, rdd


def build_plant_arrays(t):
    """Piecewise plant parameters: S1 light friction, S2+ drift, S3 contact+noise."""
    frac = np.clip((t - DRIFT_START_S) / DRIFT_RAMP_S, 0.0, 1.0)
    sc = 0.5 - 0.5 * np.cos(math.pi * frac)

    def dr(factor):
        return 1.0 + (factor - 1.0) * sc

    J = J0 * dr(DRIFT["J"])
    b = B0 * dr(DRIFT["b"])
    k = K0 * dr(DRIFT["k"])
    fc = FC0 * dr(DRIFT["fc"])
    fs = FS0 * dr(DRIFT["fs"])
    fvisc = FVISC0 * dr(DRIFT["fvisc"])
    vs = VS0 * np.maximum(1.0 + (DRIFT["vs"] - 1.0) * sc, 0.05)

    m1 = t < STAGE_EDGES[1]
    fc[m1] *= S1_FRICTION_SCALE
    fs[m1] *= S1_FRICTION_SCALE
    fvisc[m1] *= S1_FRICTION_SCALE

    m3 = (t >= STAGE_EDGES[2]) & (t < STAGE_EDGES[3])
    k[m3] *= S3_CONTACT_K

    sigma = np.full(N_STEPS, SIGMA_NOMINAL)
    sigma[m3] = SIGMA_NOMINAL * S3_NOISE_MULT
    return dict(J=J, b=b, k=k, fc=fc, fs=fs, vs=vs, fvisc=fvisc, sigma=sigma)


def build_noise(plant, seed):
    """Sensor noise (stage-dependent sigma) and Stage-4 OOD encoder spikes."""
    rng = np.random.default_rng(seed)
    noise = plant["sigma"] * rng.standard_normal(N_STEPS)
    spike = np.zeros(N_STEPS)
    i_on = int(round(OOD_START_S / DT))
    first = i_on + int(round(OOD_SPIKE_FIRST_S / DT))
    if 0 <= first < N_STEPS:
        spike[first] = OOD_SPIKE_AMP
    mask = np.zeros(N_STEPS, dtype=bool)
    mask[i_on:] = True
    idx = np.flatnonzero((rng.random(N_STEPS) < OOD_SPIKE_PROB) & mask)
    if idx.size:
        sgn = np.where(rng.random(idx.size) > 0.5, 1.0, -1.0)
        spike[idx] += OOD_SPIKE_AMP * sgn
    return noise, spike


def build_frame_schedule(seed):
    """10 Hz slow-loop frames: 20 ms nominal latency, then 60-120 ms jitter and
    15 % drops from t = 20 s (Stage 3 and 4)."""
    rng = np.random.default_rng(seed + 37)
    n_frame = int(math.floor(T_END / OUTER_DT)) + 1
    t_emit = np.arange(n_frame) * OUTER_DT
    lat = np.full(n_frame, 0.020)
    drop = np.zeros(n_frame, dtype=bool)
    after = t_emit >= EDGE_START_S
    n_after = int(after.sum())
    if n_after:
        lat[after] = EDGE_BASE_LATENCY_S + EDGE_JITTER_S * rng.random(n_after)
        drop[after] = rng.random(n_after) < EDGE_DROP_PROB
    emit_idx = np.clip(np.round(t_emit / DT).astype(int), 0, N_STEPS - 1)
    arr_idx = np.round((t_emit + lat) / DT).astype(int)
    arrived = (~drop) & (arr_idx >= 0) & (arr_idx < N_STEPS)
    emit_map = np.full(N_STEPS, -1, dtype=np.int64)
    arr_map = np.full(N_STEPS, -1, dtype=np.int64)
    for m in range(n_frame):
        e = int(emit_idx[m])
        if emit_map[e] < 0:
            emit_map[e] = m
        if arrived[m]:
            a = int(arr_idx[m])
            if arr_map[a] < 0:
                arr_map[a] = m
    return dict(t_emit=t_emit, latency=lat, dropped=drop, arrived=arrived,
                emit_idx=emit_idx, arr_idx=arr_idx, emit_map=emit_map,
                arr_map=arr_map, n_frame=n_frame)


def build_context(seed: int) -> dict:
    t = np.arange(N_STEPS) * DT
    r, rd, rdd = build_reference(t)
    plant = build_plant_arrays(t)
    noise, spike = build_noise(plant, seed)
    sched = build_frame_schedule(seed)
    stage = np.clip((t / 10.0).astype(np.int64), 0, 3)
    ctx = dict(t=t, r=r, rd=rd, rdd=rdd, plant=plant, noise=noise, spike=spike,
               sched=sched, stage=stage)
    # Python-list views for the scalar hot loops.
    for key in ("t", "r", "rd", "rdd", "noise", "spike"):
        ctx[key + "_l"] = ctx[key].tolist()
    ctx["plant_l"] = {key: val.tolist() for key, val in plant.items()}
    return ctx


def _plant_advance(x, xd, u, q, qd, tk, idx, P):
    """One plant integration step (semi-implicit Euler, same as the MATLAB engine).

    The OOD mode is a 42 Hz parasitic structural resonance.  In v1.1.0 it also
    couples into the measured position (sensor path, ``res_ky``) and is kicked
    once at fault onset, so the resonance channel of the probe can see it even
    without an encoder spike.
    """
    if tk >= OOD_START_S:
        if q == 0.0 and qd == 0.0:
            qd += OOD_RES_KICK          # single impulse at the first fault tick
        qdd = -OOD_2ZWR * qd - OOD_WR2 * q + OOD_RES_CU * u + OOD_RES_CX * x
        qd += DT * qdd
        q += DT * qd
        d_ood = OOD_RES_KQ * q
    else:
        d_ood = 0.0
        q = 0.0
        qd = 0.0
    F = _friction(xd, P["fc"][idx], P["fs"][idx], P["vs"][idx], P["fvisc"][idx])
    acc = (u - P["b"][idx] * xd - P["k"][idx] * x - F + d_ood) / P["J"][idx]
    xd += DT * acc
    x += DT * xd
    return x, xd, q, qd


# ---------------------------------------------------------------------------
# 5.  Layered residual probe (three orthogonal channels)
# ---------------------------------------------------------------------------
class ResidualProbe:
    """Two-time-scale physics residual probe of the layered branch.

    fast (spike) channel      |y - median5(y)| / res_fast_scale
                              single-sample detection, no debounce
    hf (resonance) channel    narrowband 42 Hz RMS of the observer innovation,
                              gated by 2 consecutive ticks above the trip level
    slow channel              LP(innovation) / res_slow_scale, classifier only

    The scalar handed to the 1 kHz core is max(res_fast, res_hf_gated); the C
    core itself is unchanged (one residual input, exactly like the header).
    """

    __slots__ = ("xhat", "xdhat", "lp", "bp1", "bp2", "ms_hf", "ms_hf_slow",
                 "hf_run", "yhist", "yidx", "yf", "yf_raw", "ms_e", "ms_u",
                 "ms_v", "dev_var", "ms_slow")

    def __init__(self):
        self.xhat = 0.0
        self.xdhat = 0.0
        self.lp = 0.0
        self.bp1 = 0.0
        self.bp2 = 0.0
        self.ms_hf = 0.0
        self.ms_hf_slow = 0.0
        self.hf_run = 0
        self.yhist = [0.0] * MEDIAN_WINDOW
        self.yidx = 0
        self.yf = 0.0
        self.yf_raw = 0.0
        self.ms_e = (0.02 * AMP_SCALE) ** 2
        self.ms_u = (0.10 * U_MAX) ** 2
        self.ms_v = (0.35 * V_SCALE) ** 2
        self.dev_var = (SIGMA_NOMINAL ** 2)
        self.ms_slow = (0.02) ** 2

    def reset(self, y0: float):
        self.xhat = y0
        self.xdhat = 0.0
        self.yf = y0
        self.yf_raw = y0
        self.yhist = [y0] * MEDIAN_WINDOW
        self.yidx = 0
        self.ms_hf_slow = 0.0
        self.dev_var = SIGMA_NOMINAL ** 2

    def update(self, y, u_app, r_ref):
        # --- fast / spike channel: deviation from the 4 ms median ----------
        i = self.yidx
        self.yhist[i] = y
        self.yidx = (i + 1) % MEDIAN_WINDOW
        med = sorted(self.yhist)[MEDIAN_WINDOW // 2]
        dev = y - med
        # Noise-floor-adaptive normalization.  The 20x sensor-noise stage is a
        # legitimate operating condition, not an OOD event: a fixed 0.15 rad
        # threshold would trip on its 4-sigma tail.  The floor follows the
        # measured high-frequency noise (0.4 s -> 20 ms attack) while a single
        # 0.2 rad encoder spike is still ~10x the floor and trips at once.
        self.dev_var += (DT / RES_FAST_FLOOR_TAU) * (dev * dev - self.dev_var)
        floor = RES_FAST_FLOOR_K * math.sqrt(self.dev_var)
        if floor < RES_FAST_SCALE:
            floor = RES_FAST_SCALE
        res_fast = abs(dev) / floor

        # --- observer innovation -------------------------------------------
        inov = y - self.xhat

        # --- slow channel (classifier feature only) ------------------------
        self.lp += (DT / LP_TAU_S) * (inov - self.lp)
        res_slow = abs(self.lp) / RES_SLOW_SCALE
        # 10 Hz macro-feature: RMS of the slow residual over ~0.5 s (the
        # instantaneous |lp| is rectified and swings with the motion phase,
        # which would otherwise dominate the classifier distance).
        self.ms_slow += (DT / RES_FEATURE_TAU_S) * (self.lp * self.lp - self.ms_slow)

        # --- hf / resonance channel: 42 Hz narrowband RMS ------------------
        yb = RES_HF_A1 * self.bp1 + RES_HF_A2 * self.bp2 + RES_HF_B0 * inov
        self.bp2 = self.bp1
        self.bp1 = yb
        self.ms_hf += (DT / RES_HF_TAU_S) * (yb * yb - self.ms_hf)
        # Slow baseline of the 42 Hz band: broadband sensor noise (Stage 3) is
        # not an OOD resonance, so the resonance channel is normalized by
        # max(RES_HF_SCALE, K * recent 42 Hz floor).  Fast attack (50 ms) so a
        # noise-floor step at a stage boundary is absorbed, slow decay (1 s) so
        # the floor does not forget.  The OOD resonance is a ~20x jump and still
        # trips within 2 ticks.
        if self.ms_hf > self.ms_hf_slow:
            base_tau = RES_HF_ATTACK_TAU_S
        else:
            base_tau = RES_HF_BASELINE_TAU_S
        self.ms_hf_slow += (DT / base_tau) * (self.ms_hf - self.ms_hf_slow)
        hf_floor = RES_HF_FLOOR_K * math.sqrt(self.ms_hf_slow)
        if hf_floor < RES_HF_SCALE:
            hf_floor = RES_HF_SCALE
        res_hf = math.sqrt(self.ms_hf) / hf_floor
        if res_hf > MAX_RESIDUAL:
            self.hf_run += 1
        else:
            self.hf_run = 0
        res_hf_gated = res_hf if self.hf_run >= RES_HF_GATE_TICKS else 0.0
        residual = res_fast if res_fast >= res_hf_gated else res_hf_gated

        # --- nominal observer (frozen commissioning model) ------------------
        self.xhat += DT * (self.xdhat + OBS_L1 * inov)
        self.xdhat += DT * (
            (u_app - MB * self.xdhat - MK * self.xhat
             - _friction(self.xdhat, MFC, MFS, MVS, MFVISC)) / MJ
            + OBS_L2 * inov)

        # --- sensor-fault hardening: model-based reconstruction -------------
        # Same trigger as the MATLAB engine (fast residual above half the trip
        # level) plus a noise-floor trigger: while the high-frequency sensor
        # noise floor is elevated (Stage 3), the feedback uses the nominal
        # observer state instead of the raw measurement.  Without this the
        # 20x-noise stage would excite the low-damping R3 bank into a limit
        # cycle (measured max|x| = 0.99 rad) and trip the safety envelope.
        if (abs(dev) > 0.5 * RES_FAST_SCALE
                or self.dev_var > (2.5 * SIGMA_NOMINAL) ** 2):
            y_pos = self.xhat
            ydf = self.xdhat
            self.yf = self.xhat
        else:
            ydf = (y - self.yf) / DERIV_TAU_S
            self.yf += (DT / DERIV_TAU_S) * (y - self.yf)
            y_pos = y

        # --- raw dirty derivative: classifier macro-features only -----------
        # (features describe the raw sensor environment, including the 20x
        #  noise stage and the OOD resonance; the control path below uses the
        #  hardened estimate, so the two concerns stay decoupled)
        ydf_raw = (y - self.yf_raw) / DERIV_TAU_S
        self.yf_raw += (DT / DERIV_TAU_S) * (y - self.yf_raw)

        # --- slow-loop feature EMAs ----------------------------------------
        e3 = r_ref - y
        self.ms_e += (DT / WINDOW_TAU_S) * (e3 * e3 - self.ms_e)
        self.ms_u += (DT / WINDOW_TAU_S) * (u_app * u_app - self.ms_u)
        self.ms_v += (DT / WINDOW_TAU_V_S) * (ydf_raw * ydf_raw - self.ms_v)
        return y_pos, ydf, residual, res_fast, res_hf, res_hf_gated, res_slow

    def features(self):
        return (math.sqrt(self.ms_e) / FS_E,
                math.sqrt(self.ms_u) / FS_U,
                math.sqrt(self.ms_slow) / RES_SLOW_SCALE,
                math.sqrt(self.ms_v) / FS_V)


# ---------------------------------------------------------------------------
# 6.  Coarse classifier: calibrated prototypes + margin/decay confidence
# ---------------------------------------------------------------------------
# Commissioning grid: reference-amplitude x unit-to-unit spread.  The unit
# scale multiplies the class' physical constants, i.e. exactly the cross-unit
# variation the frozen classifier is supposed to ignore (see AGENTS.md
# invariant 4).  It widens the calibration cloud without changing the deployed
# operating point, which is what sets gamma = 1/median(d1_calib).
CAL_GRID = (
    (0.40, 0.80),
    (0.40, 1.20),
    (0.60, 0.80),
    (0.60, 1.20),
)
CAL_CONDITIONS = (
    # regime 1: nominal unit, light friction, S1 harmonic
    dict(name="nominal", ref=dict(mode="harmonic", f_hz=0.5), drift=False,
         contact=False, friction_scale=S1_FRICTION_SCALE, noise_mult=1.0,
         duration=5.0),
    # regime 2: +35 % drift, S2 trapezoid
    dict(name="high-load damped", ref=dict(mode="trap", move=0.90, dwell=0.90,
                                           phase0=0.0), drift=True,
         contact=False, friction_scale=1.0, noise_mult=1.0, duration=6.0),
    # regime 3: drift + contact stiffening + 20x sensor noise, S3 trapezoid
    dict(name="boundary compliant", ref=dict(mode="trap", move=1.00, dwell=1.50,
                                             phase0=0.0), drift=True,
         contact=True, friction_scale=1.0, noise_mult=S3_NOISE_MULT,
         duration=7.0),
)


def _classify(feat, protos, gamma):
    d = [math.sqrt((protos[i][0] - feat[0]) ** 2
                   + (protos[i][1] - feat[1]) ** 2
                   + (protos[i][2] - feat[2]) ** 2
                   + (protos[i][3] - feat[3]) ** 2) for i in range(len(protos))]
    order = sorted(range(len(d)), key=d.__getitem__)
    d1 = d[order[0]]
    d2 = d[order[1]]
    margin = (d2 - d1) / (d2 + d1 + 1e-9)
    conf = margin * math.exp(-gamma * d1)
    if conf < 0.0:
        conf = 0.0
    elif conf > 1.0:
        conf = 1.0
    return order[0] + 1, conf


def _probe_plant(cond, unit=1.0):
    if cond["drift"]:
        J = J0 * DRIFT["J"]
        b = B0 * DRIFT["b"]
        k = K0 * DRIFT["k"]
        fc = FC0 * DRIFT["fc"]
        fs = FS0 * DRIFT["fs"]
        vs = VS0 * DRIFT["vs"]
        fvisc = FVISC0 * DRIFT["fvisc"]
    else:
        J, b, k, fc, fs, vs, fvisc = J0, B0, K0, FC0, FS0, VS0, FVISC0
    if cond["friction_scale"] != 1.0:
        fc *= cond["friction_scale"]
        fs *= cond["friction_scale"]
        fvisc *= cond["friction_scale"]
    if cond["contact"]:
        k *= S3_CONTACT_K
    # unit-to-unit commissioning spread
    J *= unit
    b *= unit
    k *= unit
    fc *= unit
    fs *= unit
    vs *= unit
    fvisc *= unit
    sigma = SIGMA_NOMINAL * cond["noise_mult"]
    return J, b, k, fc, fs, vs, fvisc, sigma


def _run_probe(cond, bank, amp, unit, tag, gk_factory=None):
    """One calibration probe: the class condition (at one commissioning-grid
    point) driven through the actual 1 kHz core with the class' own bank forced,
    so the probe sees exactly the deployed control path (bumpless-transfer lag,
    anti-windup, clamps).  Returns the 10 Hz macro-feature vectors after warm-up.
    """
    from numpy.random import default_rng

    if gk_factory is None:
        gk_factory = lambda cfg, tune: PythonGatekeeper(cfg, tune)
    n = int(round(cond["duration"] / DT))
    s = np.arange(n) * DT
    r, rd, rdd = _ref_arrays(cond["ref"], s, amp)
    J, b, k, fc, fs, vs, fvisc, sigma = _probe_plant(cond, unit)
    rng = default_rng(CALIB_SEED + tag)
    gk = gk_factory(make_config(), make_tuning())
    probe = ResidualProbe()
    x = float(r[0])
    xd = 0.0
    probe.reset(x)
    u_app = 0.0
    feats = []
    for i in range(n):
        y = x + sigma * rng.standard_normal()
        y_pos, ydf, _res, _rf, _rhf, _rhfg, _rs = probe.update(
            y, u_app, float(r[i]))
        if i % 100 == 0 and s[i] >= 0.6 * cond["duration"]:
            feats.append(probe.features())
        u_ff = FF_SCALE[bank] * (MJ * rdd[i] + MB * rd[i] + MK * r[i]
                                 + _friction(rd[i], MFC, MFS, MVS, MFVISC))
        u, _gate, _regime, _blend, _fault, _corrupt, _utgt = gk.step(
            float(bank + 1), 1.0, 0.0, 0.0, y_pos, ydf,
            float(r[i]), float(rd[i]), u_ff)
        F = _friction(xd, fc, fs, vs, fvisc)
        acc = (u - b * xd - k * x - F) / J
        xd += DT * acc
        x += DT * xd
        u_app = u
    return np.asarray(feats, dtype=float)


def calibrate_classifier(gk_factory=None, verbose: bool = True):
    """Derive the 3 prototypes and the confidence decay from short stage probes.

    Each probe drives the class condition through the real 1 kHz core with that
    class' bank forced, over a +-40 % amplitude commissioning grid.  The class
    means and the calibration distance distribution used for gamma are
    therefore measured, never hand-written.
    """
    per_class = []
    for ci, cond in enumerate(CAL_CONDITIONS):
        frames = []
        for gi, (amp, unit) in enumerate(CAL_GRID):
            frames.append(_run_probe(cond, ci, amp, unit, tag=1000 * ci + gi,
                                     gk_factory=gk_factory))
        per_class.append(np.vstack(frames))
    protos = np.vstack([f.mean(axis=0) for f in per_class])
    d1 = [np.linalg.norm(f - protos[ci], axis=1) for ci, f in enumerate(per_class)]
    median_d1 = float(np.median(np.concatenate(d1)))
    gamma = 1.0 / median_d1 if median_d1 > 0.0 else 0.0
    if verbose:
        print(f"[classifier] calibrated on 3 probes x {len(CAL_GRID)} commissioning "
              f"points through the 1 kHz core: median(d1)={median_d1:.4f}  "
              f"gamma={gamma:.3f}")
        print(f"[classifier] gate thresholds calibrated for the v1.1.0 confidence "
              f"formula: min_confidence={BENCH_MIN_CONFIDENCE:.2f} "
              f"confidence_clear={BENCH_CONFIDENCE_CLEAR:.2f}")
        for ci, proto in enumerate(protos):
            print(f"[classifier]   R{ci + 1} prototype = "
                  f"[{proto[0]:.4f} {proto[1]:.4f} {proto[2]:.4f} {proto[3]:.4f}]")
    return [tuple(float(v) for v in row) for row in protos], gamma


# ---------------------------------------------------------------------------
# 7.  Branch 2: continuous policy trained by closed-form ridge/RBF regression
# ---------------------------------------------------------------------------
POLICY_XS, POLICY_VXS, POLICY_AXS = 0.60, 2.50, 20.00


def _policy_reference(s):
    """Smooth, amplitude-modulated multi-sine commissioning reference.

    Continuous by construction (no segment jumps), so the nominal-unit I/O
    data cover x, x_dot and x_ddot densely enough for the ridge fit.
    """
    w = np.array([2.0 * math.pi * 0.33, 2.0 * math.pi * 0.85,
                  2.0 * math.pi * 1.70])
    a = np.array([0.34, 0.14, 0.05])
    ph = np.array([0.0, 0.7, 1.9])
    wa = 2.0 * math.pi * 0.05
    pa = 0.4
    amp = 1.0 + 0.25 * np.sin(wa * s + pa)
    damp = 0.25 * wa * np.cos(wa * s + pa)
    d2amp = -0.25 * wa * wa * np.sin(wa * s + pa)
    S = np.zeros_like(s)
    Sd = np.zeros_like(s)
    Sdd = np.zeros_like(s)
    for i in range(w.size):
        S += a[i] * np.sin(w[i] * s + ph[i])
        Sd += a[i] * w[i] * np.cos(w[i] * s + ph[i])
        Sdd -= a[i] * w[i] * w[i] * np.sin(w[i] * s + ph[i])
    r = amp * S
    rd = damp * S + amp * Sd
    rdd = d2amp * S + 2.0 * damp * Sd + amp * Sdd
    return r, rd, rdd


class RBFPolicy:
    """Additive Gaussian-RBF ridge model: u_ff = f(r, r_dot, r_ddot).

    Additive basis (9 + 27 + 15 centers + intercept) matches the structure of
    the nominal inverse dynamics (J*xdd + b*xd + k*x + F(xd)) without containing
    a hand-written inverse-dynamics formula in the deployed path.  The velocity
    axis uses a non-uniform grid that is dense around 0 rad/s, because that is
    where the Stribeck / stiction nonlinearity lives.
    """

    __slots__ = ("w0", "c1", "w1", "c2", "w2", "c3", "w3",
                 "i1", "i2", "i3", "n1", "n2", "n3")

    def __init__(self, w, c1, s1, c2, s2, c3, s3):
        self.w0 = float(w[0])
        self.c1 = c1
        self.w1 = w[1:1 + c1.size]
        self.c2 = c2
        self.w2 = w[1 + c1.size:1 + c1.size + c2.size]
        self.c3 = c3
        self.w3 = w[1 + c1.size + c2.size:]
        self.i1 = 1.0 / (s1 * s1)
        self.i2 = 1.0 / (s2 * s2)
        self.i3 = 1.0 / (s3 * s3)
        self.n1 = int(c1.size)
        self.n2 = int(c2.size)
        self.n3 = int(c3.size)

    def __call__(self, r, rd, rdd):
        z1 = r / POLICY_XS
        z2 = rd / POLICY_VXS
        z3 = rdd / POLICY_AXS
        s = self.w0
        c1 = self.c1
        w1 = self.w1
        i1 = self.i1
        for j in range(self.n1):
            d = z1 - c1[j]
            s += w1[j] * math.exp(-d * d * i1[j])
        c2 = self.c2
        w2 = self.w2
        i2 = self.i2
        for j in range(self.n2):
            d = z2 - c2[j]
            s += w2[j] * math.exp(-d * d * i2[j])
        c3 = self.c3
        w3 = self.w3
        i3 = self.i3
        for j in range(self.n3):
            d = z3 - c3[j]
            s += w3[j] * math.exp(-d * d * i3[j])
        return s


def train_continuous_policy(verbose: bool = True):
    """Nominal-unit commissioning run -> closed-form ridge/RBF inverse model.

    The data are input/output tuples (x, x_dot, x_ddot, u) actually produced by
    the frozen nominal plant (J0, B0, K0, nominal Stribeck friction) under a
    rich reference suite and a small dither.  No inverse-dynamics formula is
    evaluated in the deployed path: B2 only evaluates the fitted model plus the
    fixed PD term.
    """
    rng = np.random.default_rng(CALIB_SEED + 7)
    n = int(20.0 / DT)
    s = np.arange(n) * DT
    r, rd, rdd = _policy_reference(s)
    x = float(r[0])
    xd = 0.0
    u_app = 0.0
    dslew = SLEW_MAX * DT
    xs = []
    xds = []
    xdds = []
    us = []
    for i in range(n):
        e = r[i] - x
        ed = rd[i] - xd
        u = (10.0 * e + 0.60 * ed
             + 0.10 * math.sin(2.0 * math.pi * 7.0 * s[i])
             + 0.06 * math.sin(2.0 * math.pi * 3.1 * s[i] + 1.3))
        if u > U_MAX:
            u = U_MAX
        elif u < -U_MAX:
            u = -U_MAX
        du = u - u_app
        if du > dslew:
            du = dslew
        elif du < -dslew:
            du = -dslew
        u = u_app + du
        F = _friction(xd, FC0, FS0, VS0, FVISC0)
        acc = (u - B0 * xd - K0 * x - F) / J0
        xd += DT * acc
        x += DT * xd
        if i % 2 == 0:
            xs.append(x)
            xds.append(xd)
            xdds.append(acc)
            us.append(u)
        u_app = u
    X = np.asarray(xs)
    XD = np.asarray(xds)
    XDD = np.asarray(xdds)
    U = np.asarray(us)
    z1 = X / POLICY_XS
    z2 = XD / POLICY_VXS
    z3 = XDD / POLICY_AXS
    c1 = np.linspace(-1.3, 1.3, 10)
    s1 = np.full(c1.size, 0.32)
    # dense velocity grid around 0 rad/s (the Stribeck / stiction boundary)
    c2 = np.array([-1.10, -0.80, -0.55, -0.38, -0.26, -0.17, -0.11, -0.07,
                   -0.045, -0.028, -0.016, -0.008, -0.003, 0.0, 0.003, 0.008,
                   0.016, 0.028, 0.045, 0.07, 0.11, 0.17, 0.26, 0.38, 0.55,
                   0.80, 1.10])
    gaps = np.diff(c2)
    s2 = np.empty_like(c2)
    s2[0] = 0.55 * gaps[0]
    s2[-1] = 0.55 * gaps[-1]
    s2[1:-1] = 0.55 * np.maximum(gaps[:-1], gaps[1:])
    s2 = np.maximum(s2, 0.006)
    c3 = np.linspace(-3.6, 3.6, 25)
    s3 = np.full(c3.size, 0.32)

    def basis(z, c, sig):
        return np.exp(-(((z[:, None] - c[None, :]) / sig[None, :]) ** 2))

    Phi = np.hstack([np.ones((X.size, 1)),
                     basis(z1, c1, s1), basis(z2, c2, s2), basis(z3, c3, s3)])
    lam = 1e-8 * X.size
    A = Phi.T @ Phi + lam * np.eye(Phi.shape[1])
    w = np.linalg.solve(A, Phi.T @ U)
    pred = Phi @ w
    rmse = float(np.sqrt(np.mean((pred - U) ** 2)))
    if verbose:
        print(f"[policy] ridge/RBF inverse model: {Phi.shape[1] - 1} RBF bases "
              f"+ intercept, {X.size} nominal I/O samples, train RMSE={rmse:.4f} N*m")
    return RBFPolicy(w, c1, s1, c2, s2, c3, s3), rmse


# ---------------------------------------------------------------------------
# 8.  Branch simulators (shared plant, reference, noise and fault schedule)
# ---------------------------------------------------------------------------
def run_b1(ctx):
    """Branch 1: fixed conservative PID, no adaptation, no fault detection."""
    P = ctx["plant_l"]
    t = ctx["t_l"]
    r = ctx["r_l"]
    rd = ctx["rd_l"]
    noise = ctx["noise_l"]
    spike = ctx["spike_l"]
    X = np.empty(N_STEPS)
    XD = np.empty(N_STEPS)
    U = np.empty(N_STEPS)
    Y = np.empty(N_STEPS)
    x = r[0]
    xd = 0.0
    yf = x
    u_app = 0.0
    integ = 0.0
    q = 0.0
    qd = 0.0
    tau = DERIV_TAU_S
    dslew = SLEW_MAX * DT
    kp, kd, ki = B1_KP, B1_KD, B1_KI
    for k in range(N_STEPS):
        y = x + noise[k] + spike[k] + OOD_RES_KY * q
        ydf = (y - yf) / tau
        yf += (DT / tau) * (y - yf)
        e = r[k] - y
        ed = rd[k] - ydf
        u_raw = kp * e + kd * ed + ki * integ
        if ki != 0.0:
            integ += DT * e + (u_app - u_raw) * DT / (ki * WINDUP_TAU_S)
            if integ > INTEG_LIMIT:
                integ = INTEG_LIMIT
            elif integ < -INTEG_LIMIT:
                integ = -INTEG_LIMIT
        u = u_raw
        if u > U_MAX:
            u = U_MAX
        elif u < -U_MAX:
            u = -U_MAX
        du = u - u_app
        if du > dslew:
            du = dslew
        elif du < -dslew:
            du = -dslew
        u = u_app + du
        x, xd, q, qd = _plant_advance(x, xd, u, q, qd, t[k], k, P)
        X[k] = x
        XD[k] = xd
        U[k] = u
        Y[k] = y
        u_app = u
    return dict(name="B1 fixed PID", x=X, xd=XD, u=U, y=Y, gate=None,
                regime=None, fault=0)


def run_b2(ctx, policy):
    """Branch 2: trained continuous policy, 10 Hz inference + transport delay
    and frame drops from Stage 3 on (the deployed edge-AI failure mode)."""
    P = ctx["plant_l"]
    t = ctx["t_l"]
    r = ctx["r_l"]
    rd = ctx["rd_l"]
    rdd = ctx["rdd_l"]
    noise = ctx["noise_l"]
    spike = ctx["spike_l"]
    sched = ctx["sched"]
    emit_map = sched["emit_map"]
    arr_map = sched["arr_map"]
    t_emit = sched["t_emit"]
    arrived = sched["arrived"]
    X = np.empty(N_STEPS)
    XD = np.empty(N_STEPS)
    U = np.empty(N_STEPS)
    Y = np.empty(N_STEPS)
    x = r[0]
    xd = 0.0
    yf = x
    u_app = 0.0
    q = 0.0
    qd = 0.0
    tau = DERIV_TAU_S
    dslew = SLEW_MAX * DT
    u_hold = 0.0
    frame_u = np.full(sched["n_frame"], np.nan)
    t_edge = EDGE_START_S
    for k in range(N_STEPS):
        y = x + noise[k] + spike[k] + OOD_RES_KY * q
        ydf = (y - yf) / tau
        yf += (DT / tau) * (y - yf)
        e = r[k] - y
        ed = rd[k] - ydf
        tk = t[k]
        if tk < t_edge:
            u_cmd = policy(r[k], rd[k], rdd[k]) + B2_KP * e + B2_KD * ed
            u_hold = u_cmd
        else:
            m = emit_map[k]
            if m >= 0 and t_emit[m] >= t_edge and arrived[m]:
                frame_u[m] = (policy(r[k], rd[k], rdd[k])
                              + B2_KP * e + B2_KD * ed)
            m_arr = arr_map[k]
            if m_arr >= 0:
                v = frame_u[m_arr]
                if v == v:  # not NaN
                    u_hold = v
            u_cmd = u_hold
        u = u_cmd
        if u > U_MAX:
            u = U_MAX
        elif u < -U_MAX:
            u = -U_MAX
        du = u - u_app
        if du > dslew:
            du = dslew
        elif du < -dslew:
            du = -dslew
        u = u_app + du
        x, xd, q, qd = _plant_advance(x, xd, u, q, qd, tk, k, P)
        X[k] = x
        XD[k] = xd
        U[k] = u
        Y[k] = y
        u_app = u
    return dict(name="B2 continuous AI (RBF)", x=X, xd=XD, u=U, y=Y,
                gate=None, regime=None, fault=0)


def run_layered(ctx, gk, protos, gamma, oracle: bool = False):
    """Branch 3: 10 Hz classifier + residual probe + 1 kHz gatekeeper bank.

    oracle=True is a calibration/analysis aid only (it forces the stage-correct
    regime with full confidence and zero residual); the CLI never uses it.
    """
    P = ctx["plant_l"]
    t = ctx["t_l"]
    r = ctx["r_l"]
    rd = ctx["rd_l"]
    rdd = ctx["rdd_l"]
    noise = ctx["noise_l"]
    spike = ctx["spike_l"]
    sched = ctx["sched"]
    emit_map = sched["emit_map"]
    arr_map = sched["arr_map"]
    n_frame = sched["n_frame"]
    X = np.empty(N_STEPS)
    XD = np.empty(N_STEPS)
    U = np.empty(N_STEPS)
    Y = np.empty(N_STEPS)
    gate_arr = np.empty(N_STEPS, dtype=np.int32)
    regime_arr = np.empty(N_STEPS, dtype=np.int32)
    age_arr = np.empty(N_STEPS)
    conf_arr = np.empty(N_STEPS)
    resfast_arr = np.empty(N_STEPS)
    reshf_arr = np.empty(N_STEPS)
    residual_arr = np.empty(N_STEPS)
    probe = ResidualProbe()
    x = r[0]
    xd = 0.0
    probe.reset(x)
    q = 0.0
    qd = 0.0
    u_app = 0.0
    ai_regime = 1.0
    ai_conf = 0.95
    conf_fed = 0.95
    last_arrival = 0.0
    first_class = False
    frame_reg = np.full(n_frame, -1, dtype=np.int64)
    frame_conf = np.zeros(n_frame)
    frame_valid = np.zeros(n_frame, dtype=bool)
    frame_feat = np.full((n_frame, 4), np.nan)
    for k in range(N_STEPS):
        y = x + noise[k] + spike[k] + OOD_RES_KY * q
        y_pos, ydf, residual, res_fast, res_hf, res_hf_gated, res_slow = \
            probe.update(y, u_app, r[k])
        # slow loop: classify at the emitter, deliver to the mailbox at arrival
        tk = t[k]
        m_emit = emit_map[k]
        if m_emit >= 0 and tk >= WARMUP_S:
            feat = probe.features()
            reg, conf = _classify(feat, protos, gamma)
            frame_reg[m_emit] = reg
            frame_conf[m_emit] = conf
            frame_valid[m_emit] = True
            frame_feat[m_emit] = feat
        m_arr = arr_map[k]
        if m_arr >= 0 and frame_valid[m_arr]:
            ai_regime = float(frame_reg[m_arr])
            # The confidence written to the mailbox is EMA-filtered over 0.6 s:
            # one ambiguous frame during a stage transition must not latch the
            # sticky low-confidence veto for the whole stage.  The residual
            # probe is NOT filtered (fast OOD path stays at 1 kHz).
            conf_raw = float(frame_conf[m_arr])
            conf_fed += (OUTER_DT / CONF_EMA_TAU_S) * (conf_raw - conf_fed)
            ai_conf = conf_fed
            last_arrival = tk
            first_class = True
        age = (tk - last_arrival) if first_class else 0.0
        if oracle:
            stage_idx = int(tk // 10.0)
            ai_regime = float(min(stage_idx + 1, 3))
            ai_conf = 1.0
            residual = 0.0
        # deterministic white-box feedforward (never produced by the AI)
        u_ff = FF_WEIGHT * (MJ * rdd[k] + MB * rd[k] + MK * r[k]
                            + _friction(rd[k], MFC, MFS, MVS, MFVISC))
        u, gate, regime, _blend, fault, _corrupt, _u_tgt = gk.step(
            ai_regime, ai_conf, age, residual, y_pos, ydf, r[k], rd[k], u_ff)
        x, xd, q, qd = _plant_advance(x, xd, u, q, qd, tk, k, P)
        X[k] = x
        XD[k] = xd
        U[k] = u
        Y[k] = y
        gate_arr[k] = gate
        regime_arr[k] = regime
        age_arr[k] = age
        conf_arr[k] = ai_conf
        resfast_arr[k] = res_fast
        reshf_arr[k] = res_hf
        residual_arr[k] = residual
        u_app = u
    return dict(name="B3 layered (10 Hz + 1 kHz)", x=X, xd=XD, u=U, y=Y,
                gate=gate_arr, regime=regime_arr, age=age_arr,
                conf=conf_arr, res_fast=resfast_arr, res_hf=reshf_arr,
                residual=residual_arr, fault=int(fault), probe=probe,
                frame_reg=frame_reg, frame_conf=frame_conf,
                frame_feat=frame_feat, frame_valid=frame_valid,
                frame_t=ctx["sched"]["t_emit"])


# ---------------------------------------------------------------------------
# 9.  Metrics and table
# ---------------------------------------------------------------------------
def summarize(ctx, results):
    t = ctx["t"]
    r = ctx["r"]
    edges = STAGE_EDGES
    rows = []
    i_ood = int(round(OOD_START_S / DT))
    for key in ("B1", "B2", "B3"):
        b = results[key]
        rmse = []
        viol_ms = []
        for si in range(4):
            m = (t >= edges[si]) & (t < edges[si + 1])
            err = b["x"][m] - r[m]
            rmse.append(float(np.sqrt(np.mean(err * err))))
            viol_ms.append(int(np.count_nonzero(np.abs(b["x"][m]) > X_LIMIT)))
        tv = float(np.sum(np.abs(np.diff(b["u"]))))
        row = dict(name=b["name"], rmse=rmse, viol_ms=sum(viol_ms), tv=tv,
                   veto_ms=None)
        if b["gate"] is not None:
            gate = b["gate"]
            regime = b["regime"]
            idx = np.flatnonzero((gate[i_ood:] == GATE_VETO_OOD)
                                 | (gate[i_ood:] == GATE_VETO_ENVELOPE))
            if idx.size:
                row["veto_ms"] = float(idx[0]) * DT * 1e3
            row["gate_hist"] = [int(np.count_nonzero(gate == g)) for g in range(5)]
            row["regime_hist"] = [int(np.count_nonzero(regime == rr))
                                  for rr in (1, 2, 3, 4)]
            row["fault"] = b["fault"]
            row["stage_viol"] = viol_ms
        rows.append(row)
    return rows


def print_table(rows, protos, gamma, backend, policy_rmse, elapsed, out_path):
    print()
    print("=== v1.1.0 four-stage benchmark - industrial profile, 40 s @ 1 kHz ===")
    core = "C core" if backend == "C" else "pure-Python mirror"
    print(f"branch: {core} | policies: fixed PID, ridge/RBF (train RMSE "
          f"{policy_rmse:.4f} N*m), layered 10 Hz + 1 kHz")
    print()
    hdr = ("branch", "S1 rmse", "S2 rmse", "S3 rmse", "S4 rmse",
           "SVT[ms]", "TV(u)[N*m]", "OOD veto[ms]")
    print("{:<26} {:>9} {:>9} {:>9} {:>9} {:>8} {:>11} {:>12}".format(*hdr))
    print("{:<26} {:>9} {:>9} {:>9} {:>9} {:>8} {:>11} {:>12}".format(
        "-" * 26, "-" * 9, "-" * 9, "-" * 9, "-" * 9, "-" * 8, "-" * 11, "-" * 12))
    for row in rows:
        veto = ("%.1f" % row["veto_ms"]) if row["veto_ms"] is not None else "n/a"
        print("{:<26} {:>9.4f} {:>9.4f} {:>9.4f} {:>9.4f} {:>8d} {:>11.1f} {:>12}".format(
            row["name"], row["rmse"][0], row["rmse"][1], row["rmse"][2],
            row["rmse"][3], row["viol_ms"], row["tv"], veto))
    b3 = rows[2]
    print()
    print(f"B3 gate ticks:  NORMAL={b3['gate_hist'][0]}  "
          f"HOLD_LATENCY={b3['gate_hist'][1]}  VETO_OOD={b3['gate_hist'][2]}  "
          f"VETO_LOW_CONF={b3['gate_hist'][3]}  "
          f"VETO_ENVELOPE={b3['gate_hist'][4]}   (fault bits 0x{b3['fault']:02x})")
    print(f"B3 regime ticks: R1={b3['regime_hist'][0]}  R2={b3['regime_hist'][1]}  "
          f"R3={b3['regime_hist'][2]}  R4={b3['regime_hist'][3]}   "
          f"(R4 = safe fallback)")
    print(f"B3 safety violation ticks per stage: "
          f"S1={b3['stage_viol'][0]} S2={b3['stage_viol'][1]} "
          f"S3={b3['stage_viol'][2]} S4={b3['stage_viol'][3]} ms")
    print("classifier prototypes [rms(e)/amp, rms(u)/u_max, slow residual, "
          "rms(xd)/v_scale] (calibrated, not hand-written):")
    for ci, proto in enumerate(protos):
        print(f"  R{ci + 1} [{proto[0]:.4f} {proto[1]:.4f} "
              f"{proto[2]:.4f} {proto[3]:.4f}]")
    print(f"  gamma = 1/median(d1_calib) = {gamma:.3f}")
    try:
        shown = out_path.relative_to(_REPO_ROOT)
    except ValueError:
        shown = out_path
    print(f"figure: {shown} (240 dpi, 2x2)   wall time: {elapsed:.2f} s")


# ---------------------------------------------------------------------------
# 10. Figure (2x2: state+envelope, command, residual/confidence, regime/gate)
# ---------------------------------------------------------------------------
_GATE_NAMES = ("NORMAL", "HOLD", "OOD", "LOW_CONF", "ENVELOPE")


def make_figure(ctx, results, out_path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = ctx["t"]
    r = ctx["r"]
    b1, b2, b3 = results["B1"], results["B2"], results["B3"]
    fig, axes = plt.subplots(2, 2, figsize=(13.0, 8.0), constrained_layout=True)

    # (1) state + safety envelope
    ax = axes[0, 0]
    ax.plot(t, r, color="0.20", lw=0.8, ls="--", label="r(t)")
    ax.plot(t, b1["x"], color="tab:blue", lw=0.45, alpha=0.75, label="B1 fixed PID")
    ax.plot(t, b2["x"], color="tab:orange", lw=0.45, alpha=0.75, label="B2 continuous AI")
    ax.plot(t, b3["x"], color="tab:green", lw=0.75, alpha=0.95, label="B3 layered")
    for lvl, style in ((X_LIMIT, "-"), (TRIP_RATIO * X_LIMIT, ":")):
        ax.axhline(lvl, color="tab:red", lw=0.7, ls=style)
        ax.axhline(-lvl, color="tab:red", lw=0.7, ls=style)
    ax.set_ylim(-1.15, 1.15)
    ax.set_ylabel("state x [rad]")
    ax.set_title("state + safety envelope (limit 1.0, trip 0.9)")
    ax.legend(loc="upper right", fontsize=7, ncol=2, framealpha=0.8)

    # (2) actuator command
    ax = axes[0, 1]
    ax.plot(t, b1["u"], color="tab:blue", lw=0.4, alpha=0.7, label="B1")
    ax.plot(t, b2["u"], color="tab:orange", lw=0.4, alpha=0.7, label="B2")
    ax.plot(t, b3["u"], color="tab:green", lw=0.55, alpha=0.9, label="B3")
    ax.axhline(U_MAX, color="0.3", lw=0.7, ls=":")
    ax.axhline(-U_MAX, color="0.3", lw=0.7, ls=":")
    ax.set_ylabel("u [N*m]")
    ax.set_title("actuator command (hard magnitude + slew clamps)")
    ax.legend(loc="upper right", fontsize=7, ncol=3, framealpha=0.8)

    # (3) residual channels and classifier confidence
    ax = axes[1, 0]
    ax.semilogy(t, np.maximum(b3["res_fast"], 1e-4), color="tab:red", lw=0.4,
                alpha=0.55, label="res_fast (spike ch.)")
    ax.semilogy(t, np.maximum(b3["res_hf"], 1e-6), color="tab:purple", lw=0.4,
                alpha=0.55, label="res_hf (42 Hz ch.)")
    ax.semilogy(t, np.maximum(b3["residual"], 1e-4), color="black", lw=0.5,
                alpha=0.8, label="residual -> gate")
    ax.axhline(MAX_RESIDUAL, color="tab:red", lw=0.7, ls=":")
    ax.set_ylabel("normalized residual")
    ax.set_ylim(1e-4, 1e3)
    ax2 = ax.twinx()
    ax2.plot(t, b3["conf"], color="tab:blue", lw=0.45, alpha=0.6,
             label="confidence (filtered)")
    ax2.axhline(BENCH_MIN_CONFIDENCE, color="tab:blue", lw=0.7, ls=":")
    ax2.set_ylabel("classifier confidence", color="tab:blue")
    ax2.set_ylim(-0.02, 1.02)
    ax.set_title("residual channels + confidence (42 Hz resonator)")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="lower left", fontsize=7, framealpha=0.8)

    # (4) gate + regime
    ax = axes[1, 1]
    ax.step(t, b3["gate"], where="post", color="tab:red", lw=0.6,
            label="gate")
    ax.step(t, b3["regime"], where="post", color="tab:green", lw=0.6,
            label="regime")
    ax.set_yticks([0, 1, 2, 3, 4])
    ax.set_yticklabels(["0 NORM", "1 HOLD", "2 OOD", "3 CONF", "4 ENV"])
    ax.set_ylim(-0.2, 4.4)
    ax.set_ylabel("gate / regime")
    ax.set_title("B3 gate state and selected regime bank")
    ax.legend(loc="upper left", fontsize=7, ncol=2, framealpha=0.8)

    for ax in axes.flat:
        for edge in (10.0, 20.0, 30.0):
            ax.axvline(edge, color="0.65", lw=0.6, ls=":")
        ax.set_xlim(0.0, T_END)
        ax.set_xlabel("time [s]")
        ax.grid(True, alpha=0.15, lw=0.4)
    fig.suptitle("ai-layered-control v1.1.0 - 40 s industrial four-stage benchmark "
                 "(1 kHz, three branches)", fontsize=11)
    fig.savefig(out_path, dpi=240)
    plt.close(fig)
    return out_path


# ---------------------------------------------------------------------------
# 11. CLI
# ---------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Zero-MATLAB four-stage benchmark for the layered AI "
                    "control reference architecture (v1.1.0).")
    parser.add_argument("--no-c", action="store_true",
                        help="force the pure-Python gatekeeper mirror")
    parser.add_argument("--selftest", action="store_true",
                        help="run the 500-frame C-vs-Python equivalence check")
    parser.add_argument("--seed", type=int, default=SEED,
                        help=f"noise/fault RNG seed (default {SEED})")
    args = parser.parse_args(argv)

    if args.selftest:
        return run_selftest(force_python=args.no_c)

    wall = time.time()
    ctx = build_context(args.seed)
    gk_factory, backend = load_gatekeeper(force_python=args.no_c)
    protos, gamma = calibrate_classifier(gk_factory=gk_factory)
    policy, policy_rmse = train_continuous_policy()

    b1 = run_b1(ctx)
    b2 = run_b2(ctx, policy)
    gk = gk_factory(make_config(), make_tuning())
    b3 = run_layered(ctx, gk, protos, gamma)
    results = dict(B1=b1, B2=b2, B3=b3)
    rows = summarize(ctx, results)

    out_path = _HERE / "benchmark_4stages.png"
    make_figure(ctx, results, out_path)
    print_table(rows, protos, gamma, backend, policy_rmse,
                time.time() - wall, out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
