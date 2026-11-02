#!/usr/bin/env python3
"""Minimal copy-paste layered-control skeleton (1 kHz core + 10 Hz slow loop)."""
# [AGENT-CONTRACT] Copy c99_layered_core/layered_gatekeeper.{c,h} + this file only; do NOT copy the
# 40 s benchmark harness (run_benchmark.py / sim_core_engine.m). Replace [USER-PLUGIN-SLOT 1..3].
import ctypes, os, subprocess, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]; CORE = ROOT / "c99_layered_core" / "layered_gatekeeper.c"; OUT = Path(__file__).resolve().parent / "build"
F, U, I = ctypes.c_float, ctypes.c_uint32, ctypes.c_int
FLOAT_FIELDS = "dt_s outer_timeout_s max_physics_residual residual_clear_ratio min_ai_confidence confidence_clear_ratio state_safety_limit safety_trip_ratio safety_clear_ratio safety_predict_s u_max u_slew_max_per_s blend_alpha integ_limit windup_tau_s".split()

class Config(ctypes.Structure):
    _fields_ = [(n, F) for n in FLOAT_FIELDS] + [("min_dwell_ticks", U), ("ood_recovery_cooldown_ticks", U)] + [(g + str(i), F) for g in ("kp", "kd", "ki", "ff_scale") for i in range(4)]
class Input(ctypes.Structure):
    _fields_ = [(n, F) for n in "ai_regime ai_confidence outer_age_s physics_residual x x_dot r r_dot u_ff".split()]
class Output(ctypes.Structure):
    _fields_ = [("u", F), ("u_target", F), ("gate", I), ("regime", I), ("blend", F), ("fault", U), ("corrupted", I)]
class State(ctypes.Structure):
    _fields_ = [("initialized", I), ("u_applied", F), ("u_target", F), ("integ", F * 4), ("blend", F), ("control_started", I), ("regime", I), ("ood_latched", I), ("safety_latched", I), ("conf_latched", I)] + [(n, U) for n in "dwell_ticks recovery_ticks fault_audit_bitmask step_count safety_veto_count ood_veto_count low_confidence_veto_count latency_hold_count corrupted_frame_count".split()]
class Tuning(ctypes.Structure):
    _fields_ = [(n, F) for n in "gain_scale damping_scale safety_margin_scale".split()]

def load_core():
    ext = ".dll" if os.name == "nt" else (".dylib" if sys.platform == "darwin" else ".so")
    lib_path = OUT / ("layered_gatekeeper" + ext)
    try:
        if not lib_path.exists():
            OUT.mkdir(parents=True, exist_ok=True); cc = os.environ.get("CC", "gcc")
            flags = ["-shared"] if os.name == "nt" else (["-dynamiclib", "-fPIC"] if sys.platform == "darwin" else ["-shared", "-fPIC"])
            subprocess.run([cc, "-std=c99", "-O2", *flags, "-o", str(lib_path), str(CORE)], check=True, capture_output=True)
        lib = ctypes.CDLL(str(lib_path))
        lib.Layered_Init.argtypes = [ctypes.POINTER(State), ctypes.POINTER(Config), ctypes.POINTER(Tuning)]
        lib.Layered_Step.argtypes = [ctypes.POINTER(State), ctypes.POINTER(Config), ctypes.POINTER(Tuning), ctypes.POINTER(Input)]
        lib.Layered_Step.restype = Output
        return lib
    except Exception as exc:
        print(f"[fallback] C99 core unavailable ({exc}); using minimal Python gate")
        return None

class FallbackCore:
    def __init__(self): self.u = 0.0
    def step(self, inp):
        e, ed = inp.r - inp.x, inp.r_dot - inp.x_dot
        self.u = max(-20.0, min(20.0, 25.0 * e + 8.0 * ed))
        return type("O", (), {"u": self.u, "gate": 4 if abs(inp.x) > 1.35 or inp.physics_residual > 0.65 else 0, "regime": 1})()

def read_plant_and_reference(t):
    # [USER-PLUGIN-SLOT 1]: replace with real sensor (q, dq) and trajectory (q_ref, dq_ref, ddq_ref).
    global Q, DQ
    w = 2.0 * np.pi * 1.0
    return Q, DQ, 0.4 * np.sin(w * t), 0.4 * w * np.cos(w * t), -0.4 * w * w * np.sin(w * t)

def compute_nominal_residual(q, dq, u_prev):
    # [USER-PLUGIN-SLOT 2]: replace with a domain-specific nominal model residual |y - y_hat|.
    return float(abs(q - (q - dq * 0.001))) / 0.5

def slow_loop_10hz_step(macro_features, prototypes, gamma):
    # [USER-PLUGIN-SLOT 3]: 4-D dimensionless features -> coarse regime {1,2,3,4}.
    d = np.linalg.norm(np.asarray(prototypes) - np.asarray(macro_features), axis=1)
    order = np.argsort(d); d1, d2 = d[order[0]], d[order[1]]
    conf = float(np.clip(((d2 - d1) / (d2 + d1 + 1e-9)) * np.exp(-gamma * d1), 0.0, 1.0))
    regime = int(order[0]) + 1
    if regime == 3 and macro_features[2] < 0.42: regime = 2  # S3 physical-residual mode guard
    if conf < 0.08: regime = 4
    return regime, conf

def main():
    global Q, DQ
    Q, DQ, u, violations = 0.0, 0.0, 0.0, 0
    lib, dt, protos = load_core(), 0.001, np.array([[0.05, 0.05, 0.05, 0.1], [0.2, 0.2, 0.3, 0.3], [0.8, 0.5, 0.9, 1.2]])
    cfg = Config(**{n: 0.0 for n in FLOAT_FIELDS})
    cfg.dt_s, cfg.outer_timeout_s, cfg.max_physics_residual, cfg.residual_clear_ratio = dt, 0.30, 0.65, 0.80
    cfg.min_ai_confidence, cfg.confidence_clear_ratio, cfg.state_safety_limit = 0.01, 0.02, 1.5
    cfg.safety_trip_ratio, cfg.safety_clear_ratio, cfg.safety_predict_s = 0.90, 0.75, 0.06
    cfg.u_max, cfg.u_slew_max_per_s, cfg.blend_alpha, cfg.integ_limit, cfg.windup_tau_s = 20.0, 1000.0, 0.10, 10.0, 0.02
    cfg.min_dwell_ticks, cfg.ood_recovery_cooldown_ticks = 150, 500
    for i, (kp, kd) in enumerate([(25.0, 8.0), (18.0, 6.0), (12.0, 5.0), (8.0, 4.0)]):
        setattr(cfg, "kp" + str(i), kp); setattr(cfg, "kd" + str(i), kd); setattr(cfg, "ff_scale" + str(i), 1.0)
    state, fallback, tuning = State(), FallbackCore(), Tuning(1.0, 1.0, 1.0)
    if lib is not None: lib.Layered_Init(ctypes.byref(state), ctypes.byref(cfg), ctypes.byref(tuning))
    err_hist, res_hist = [], []
    for k in range(1000):
        t = k * dt; q, dq, qr, dqr, _ = read_plant_and_reference(t)
        if k % 100 == 0:
            macro = np.array([np.sqrt(np.mean(np.square(err_hist))) / 0.4 if err_hist else 0.0, abs(u) / 20.0, np.sqrt(np.mean(np.square(res_hist))) if res_hist else 0.0, abs(dq) / 2.0])
            regime, conf, age = *slow_loop_10hz_step(macro, protos, 1.5), 0.0
            err_hist, res_hist = [], []
        residual = compute_nominal_residual(q, dq, u)
        inp = Input(regime, conf, age, residual, q, dq, qr, dqr, 0.0)
        out = lib.Layered_Step(ctypes.byref(state), ctypes.byref(cfg), ctypes.byref(tuning), ctypes.byref(inp)) if lib is not None else fallback.step(inp)
        u = float(out.u); violations += int(out.gate == 4)
        err_hist.append(qr - q); res_hist.append(residual)
        Q += DQ * dt; DQ += u * dt  # trivial double-integrator plant for the template demo only
    print(f"[OK] Minimal layered control loop completed ({violations} safety violations)")
    return 0 if violations == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
