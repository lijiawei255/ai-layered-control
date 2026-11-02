#!/usr/bin/env python3
"""Reusable 10 Hz supervisor + 1 kHz C99 loop; replace the three device slots."""
import ctypes, os, subprocess, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "c99_layered_core" / "layered_gatekeeper.c"
BUILD = Path(__file__).resolve().parent / "build"
F, U, I = ctypes.c_float, ctypes.c_uint32, ctypes.c_int
FLOAT_FIELDS = "dt_s outer_timeout_s max_physics_residual residual_clear_ratio min_ai_confidence confidence_clear_ratio state_safety_limit safety_trip_ratio safety_clear_ratio safety_predict_s u_max u_slew_max_per_s blend_alpha integ_limit windup_tau_s".split()
BANK_FIELDS = ("kp", "kd", "ki", "ff_scale")
class Config(ctypes.Structure):
    _fields_ = [(n, F) for n in FLOAT_FIELDS] + [("min_dwell_ticks", U), ("ood_recovery_cooldown_ticks", U)] + [(n, F * 4) for n in BANK_FIELDS]
class Input(ctypes.Structure):
    _fields_ = [(n, F) for n in "ai_regime ai_confidence outer_age_s physics_residual x x_dot r r_dot u_ff".split()]
class Output(ctypes.Structure):
    _fields_ = [("u", F), ("u_target", F), ("gate", I), ("regime", I), ("blend", F), ("fault_audit_bitmask", U), ("corrupted", I)]
class State(ctypes.Structure):
    _fields_ = [("initialized", I), ("u_applied", F), ("u_target", F), ("integ", F * 4), ("blend", F), ("control_started", I), ("regime", I), ("ood_latched", I), ("safety_latched", I), ("conf_latched", I)] + [(n, U) for n in "dwell_ticks recovery_ticks fault_audit_bitmask step_count safety_veto_count ood_veto_count low_confidence_veto_count latency_hold_count corrupted_frame_count".split()]
class Tuning(ctypes.Structure):
    _fields_ = [(n, F) for n in "gain_scale damping_scale safety_margin_scale".split()]

def load_core():
    extension = ".dll" if os.name == "nt" else (".dylib" if sys.platform == "darwin" else ".so")
    path = BUILD / ("layered_gatekeeper" + extension)
    if not path.exists() or max(SOURCE.stat().st_mtime_ns, SOURCE.with_suffix(".h").stat().st_mtime_ns) > path.stat().st_mtime_ns:
        BUILD.mkdir(exist_ok=True)
        flags = ["-dynamiclib", "-fPIC"] if sys.platform == "darwin" else ["-shared"] + ([] if os.name == "nt" else ["-fPIC"])
        subprocess.run([os.environ.get("CC", "gcc"), "-std=c99", "-O2", *flags, "-o", str(path), str(SOURCE)], check=True)
    lib = ctypes.CDLL(str(path))
    lib.Layered_Init.argtypes = [ctypes.POINTER(t) for t in (State, Config, Tuning)]
    lib.Layered_Init.restype = None
    lib.Layered_Step.argtypes = [ctypes.POINTER(t) for t in (State, Config, Tuning, Input)]
    lib.Layered_Step.restype = Output
    return lib

class Core:
    def __init__(self, cfg, tuning=(1, 1, 1)):
        self.lib = load_core()
        self.cfg = Config(**{n: (F * 4)(*v) if n in BANK_FIELDS else v for n, v in cfg.items()})
        self.tuning, self.state = Tuning(*tuning), State()
        self.lib.Layered_Init(ctypes.byref(self.state), ctypes.byref(self.cfg), ctypes.byref(self.tuning))
    def step(self, ai_regime, ai_confidence, outer_age_s, physics_residual, x, x_dot, r, r_dot, u_ff):
        inp = Input(ai_regime, ai_confidence, outer_age_s, physics_residual, x, x_dot, r, r_dot, u_ff)
        out = self.lib.Layered_Step(ctypes.byref(self.state), ctypes.byref(self.cfg), ctypes.byref(self.tuning), ctypes.byref(inp))
        return out.u, out.gate, out.regime, out.blend, out.fault_audit_bitmask, out.corrupted, out.u_target

def auto_calibrate(samples, labels):
    samples, labels = np.asarray(samples), np.asarray(labels)
    prototypes = np.array([samples[labels == k].mean(axis=0) for k in (1, 2, 3)])
    distances = np.sort(np.linalg.norm(samples[:, None, :] - prototypes, axis=2), axis=1)
    first, second = distances[:, 0], distances[:, 1]
    gamma = 1 / max(float(np.quantile(first, .95)), 1e-6)
    confidence = (second - first) / (second + first + 1e-9) * np.exp(-gamma * first)
    threshold = .15 * float(np.quantile(confidence, .01))
    return prototypes, gamma, threshold, 1.25 * threshold

def auto_field_tune(evaluate, current=(1, 1, 1)):
    """Six bounded trials; device-side validity includes every operating regime."""
    best, score = current, float("inf")
    grid = ((.75, 1.), (1., 1.), (1.25, 1.),
            (2., 1.), (1.25, .5), (1.25, .75))
    for gain, damping in grid:
        candidate = (gain, damping, current[2])
        cost, valid = evaluate(candidate)
        if valid and np.isfinite(cost) and cost < score:
            best, score = candidate, cost
    return best

def classify(features, model):
    prototypes, gamma, _, _ = model
    distance = np.linalg.norm(prototypes - features, axis=1)
    order = np.argsort(distance); a, b = distance[order[:2]]
    return int(order[0]) + 1, float((b - a) / (b + a + 1e-9) * np.exp(-gamma * a))

def rollout(cfg, tuning=(1, 1, 1), model=None, damping=.4):
    core = Core(cfg, tuning); dt = cfg["dt_s"]
    q = velocity = previous_v = load = prediction = u = error_sum = 0.; violations = faults = 0
    regime, confidence, stamp, samples = 1, 1., 0., []
    for tick in range(1300):
        t = tick * dt
        # SLOT 1: real sensor/reference and actuator I/O; this demo uses a damped axis.
        r, rd, rdd = .1*(1-np.cos(np.pi*t)), .1*np.pi*np.sin(np.pi*t), .1*np.pi**2*np.cos(np.pi*t)
        # SLOT 2: commissioned predictor and dimensionless measured features.
        residual = abs(q - prediction) / .005
        if abs(previous_v) > .001: load = max(0., (u-(velocity-previous_v)/dt)/previous_v)
        features = np.array([abs(r-q)/.2, abs(u)/20, residual, load/(1+load)])
        if tick and tick % 100 == 0:
            samples.append(features); stamp = t
            if model is not None: regime, confidence = classify(features, model)
        age = t - stamp
        ai, conf = (4, 0.) if age > cfg["outer_timeout_s"] else (regime, confidence)
        out = core.step(ai, conf, age, residual, q, velocity, r, rd, rdd + .4*rd); u = out[0]
        error_sum += (r-q)**2; violations += int(abs(q) > cfg["state_safety_limit"]); faults += int(out[1] != 0 or out[2] == 4 or out[5])
        prediction = q + dt * (velocity + dt * (u - .4*velocity))
        previous_v = velocity; velocity += dt * (u - damping*velocity); q += dt*velocity
    error = (error_sum/1300)**.5
    return error, violations == faults == 0 and error < .02, np.array(samples), (violations, faults)

def main():
    cfg = dict(zip(FLOAT_FIELDS, (.001, .35, 1., .5, .01, .02, 1., .9, .75, .025, 20., 2000., .3, .5, .1)))
    cfg.update(min_dwell_ticks=150, ood_recovery_cooldown_ticks=300, kp=[25,35,12,2], kd=[8,10,8,2], ki=[2,8,1,0], ff_scale=[1,1,1,0])
    # SLOT 3: labeled commissioning observations and a measured, safety-checked trial.
    observations = np.vstack([rollout(cfg, damping=d)[2] for d in (.4, 2., 5.)])
    model = auto_calibrate(observations, np.repeat((1, 2, 3), 12))
    cfg.update(min_ai_confidence=model[2], confidence_clear_ratio=model[3])
    evaluate = lambda tuning: rollout(cfg, tuning, model, damping=2.5)[:2]
    tuning = auto_field_tune(evaluate); error, valid, _, counts = rollout(cfg, tuning, model, damping=2.5)
    print(f"Template: tuning={tuning}, RMSE={error:.6f}, breaches/gate_faults={counts}, {'PASS' if valid else 'FAIL'}")
    return 0 if valid else 1

if __name__ == "__main__":
    sys.exit(main())
