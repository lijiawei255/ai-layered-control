"""V2.0.0: white-box PID, conditioned continuous policy, and deterministic layered control."""
import ctypes
import json
import math
import sys
import time
from pathlib import Path
import numpy as np
STAGE_SPECS = [dict(mode='harmonic', f_hz=0.5), dict(mode='trap', move=0.9, dwell=0.9, phase0=0.8), dict(mode='trap', move=1.0, dwell=1.5, phase0=0.0), dict(mode='trap', move=1.0, dwell=1.5, phase0=0.0)]
DT = 0.001
PARAM = dict(J=0.01, b=0.06, k=0.3, fc=0.6, fs=1.0)
CFG = dict(
    dt_s=DT, min_ai_confidence=0.01, confidence_clear_ratio=0.02,
    kp=[60.0, 110.0, 8.0, 2.0], kd=[1.2, 1.5, 0.8, 0.35],
    ki=[60.0, 300.0, 2.0, 0.0], ff_scale=[1.0, 1.0, 1.0, 0.0],
    u_max=20.0, u_slew_max_per_s=2000.0, state_safety_limit=1.0,
    safety_trip_ratio=0.9, safety_clear_ratio=0.75, safety_predict_s=0.025,
    blend_alpha=0.3, integ_limit=0.5, windup_tau_s=0.1,
    min_dwell_ticks=150, ood_recovery_cooldown_ticks=300, outer_timeout_s=0.35,
    max_physics_residual=1.0, residual_clear_ratio=0.5)
BASE = (110.0, 1.5, 300.0)
FSCALE = np.array([0.5, 20.0, 0.006, 0.005])
FAULT_LATENCY, FAULT_OOD, FAULT_LOW_CONF, FAULT_ENVELOPE, FAULT_NAN = (1, 2, 4, 8, 16)
GATE_NORMAL, GATE_HOLD_LATENCY, GATE_VETO_OOD, GATE_VETO_LOW_CONF, GATE_VETO_ENVELOPE = range(5)
REGIME_NOMINAL, REGIME_HIGH_LOAD, REGIME_BOUNDARY, REGIME_SAFE = range(1, 5)
USE_PYTHON = False

def make_core(cfg, tuning):
    if USE_PYTHON:
        return PythonGatekeeper(cfg, dict(zip(('gain_scale', 'damping_scale', 'safety_margin_scale'), tuning)))
    from minimal_template import Core
    return Core(cfg, tuning)

class PythonGatekeeper:
    """Behavioral mirror of c99_layered_core/layered_gatekeeper.c.

    Kept deliberately literal: same decision order, same sticky latches, same
    integer bookkeeping.  --selftest proves gate/regime equality against the
    compiled C core on 6,000 shared random frames.
    """
    __slots__ = ('cfg', 'tune', 'u_applied', 'u_target', 'integ', 'blend', 'control_started', 'regime', 'ood_latched', 'safety_latched', 'conf_latched', 'dwell_ticks', 'recovery_ticks', 'fault_audit_bitmask', 'step_count', 'safety_veto_count', 'ood_veto_count', 'low_confidence_veto_count', 'latency_hold_count', 'corrupted_frame_count')

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

    @staticmethod
    def _absf(v):
        return -v if v < 0.0 else v

    @staticmethod
    def _clampf(v, lo, hi):
        if v != v:
            v = 0.0
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

    def step(self, ai_regime, ai_confidence, outer_age_s, physics_residual, x, x_dot, r, r_dot, u_ff):
        """One 1 kHz step.  Returns (u, gate, regime, blend, fault, corrupted,
        u_target) exactly like the C output struct."""
        cfg = self.cfg
        tune = self.tune
        finite = 3e+38
        values = [x for v in cfg.values() for x in (v if isinstance(v, list) else [v])] + list(tune.values())
        corrupt = any((not -finite <= v <= finite for v in values))
        for v in (ai_regime, ai_confidence, outer_age_s, physics_residual, x, x_dot, r, r_dot, u_ff):
            if not -finite <= v <= finite:
                corrupt = True
                break
        if not corrupt:
            if ai_regime < 1.0 or ai_regime > 4.0 or ai_confidence < 0.0 or (ai_confidence > 1.0) or (physics_residual < 0.0) or (outer_age_s < 0.0):
                corrupt = True
        if corrupt:
            self.fault_audit_bitmask |= FAULT_NAN
            self.corrupted_frame_count += 1
            self.recovery_ticks = cfg['ood_recovery_cooldown_ticks']
            self.step_count += 1
            return (self.u_applied, GATE_HOLD_LATENCY, self.regime, self.blend, self.fault_audit_bitmask, 1, self.u_target)
        snap = self.control_started == 0
        conf_before = self.conf_latched
        u_max = self._absf(cfg['u_max'])
        integ_limit = self._absf(cfg['integ_limit'])
        dt_eff = self._absf(cfg['dt_s'])
        du_max = self._absf(cfg['u_slew_max_per_s']) * dt_eff
        windup_tau = cfg['windup_tau_s']
        limit_eff = self._absf(cfg['state_safety_limit'] * tune['safety_margin_scale'])
        x_abs = self._absf(x)
        xdot_abs = self._absf(x_dot)
        predicted_abs = x_abs + self._absf(cfg['safety_predict_s']) * xdot_abs
        if self.safety_latched:
            clear_level = cfg['safety_clear_ratio'] * limit_eff
            if x_abs < clear_level and predicted_abs < clear_level:
                self.safety_latched = 0
        elif x_abs > cfg['safety_trip_ratio'] * limit_eff or predicted_abs > limit_eff:
            self.safety_latched = 1
            self.safety_veto_count += 1
            self.fault_audit_bitmask |= FAULT_ENVELOPE
        residual_abs = self._absf(physics_residual)
        if self.ood_latched:
            if residual_abs < cfg['residual_clear_ratio'] * cfg['max_physics_residual']:
                self.ood_latched = 0
        elif residual_abs > cfg['max_physics_residual']:
            self.ood_latched = 1
            self.ood_veto_count += 1
            self.fault_audit_bitmask |= FAULT_OOD
        mailbox_regime = self._mailbox_regime(ai_regime)
        if ai_confidence < cfg['min_ai_confidence'] or self._mailbox_abstains(ai_regime):
            self.conf_latched = 1
            self.fault_audit_bitmask |= FAULT_LOW_CONF
        elif ai_confidence >= cfg['confidence_clear_ratio'] and (not self._mailbox_abstains(ai_regime)):
            self.conf_latched = 0
        conf_veto = self.conf_latched
        if conf_veto and (not conf_before):
            self.low_confidence_veto_count += 1
        latency_hold = outer_age_s > cfg['outer_timeout_s']
        if latency_hold:
            self.fault_audit_bitmask |= FAULT_LATENCY
            self.recovery_ticks = cfg['ood_recovery_cooldown_ticks']
            if outer_age_s - cfg['outer_timeout_s'] <= dt_eff:
                self.latency_hold_count += 1
        veto_active = self.safety_latched != 0 or self.ood_latched != 0 or conf_veto != 0
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
            elif self.dwell_ticks + 1 >= cfg['min_dwell_ticks']:
                regime = mailbox_regime
            else:
                regime = self.regime
        if veto_active:
            self.recovery_ticks = cfg['ood_recovery_cooldown_ticks']
        do_control = gate != GATE_HOLD_LATENCY
        kp = cfg['kp']
        kd = cfg['kd']
        ki = cfg['ki']
        ff_scale = cfg['ff_scale']
        gain_scale = tune['gain_scale']
        damping_scale = tune['damping_scale']
        alpha = cfg['blend_alpha']
        if do_control:
            e = r - x
            ed = r_dot - x_dot
            u_pid0 = kp[0] * gain_scale * e + kd[0] * damping_scale * ed + ki[0] * gain_scale * self.integ[0] + ff_scale[0] * u_ff
            u_pid1 = kp[1] * gain_scale * e + kd[1] * damping_scale * ed + ki[1] * gain_scale * self.integ[1] + ff_scale[1] * u_ff
            u_pid2 = kp[2] * gain_scale * e + kd[2] * damping_scale * ed + ki[2] * gain_scale * self.integ[2] + ff_scale[2] * u_ff
            u_pid3 = kp[3] * gain_scale * e + kd[3] * damping_scale * ed + ki[3] * gain_scale * self.integ[3] + ff_scale[3] * u_ff
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
            if self.u_target != self.u_target:
                self.u_target = 0.0
            u_cmd = self._clampf(self.u_target, -u_max, u_max)
            u_cmd = self._clampf(u_cmd, self.u_applied - du_max, self.u_applied + du_max)
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
        if regime != self.regime:
            self.dwell_ticks = 0
        else:
            self.dwell_ticks += 1
        self.regime = regime
        self.step_count += 1
        return (out_u, gate, regime, out_blend, self.fault_audit_bitmask, 0, out_u_target)

def _quintic(tau):
    p = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))
    v = tau * tau * (30.0 - 60.0 * tau + 30.0 * tau * tau)
    a = tau * (60.0 - 180.0 * tau + 120.0 * tau * tau)
    return (p, v, a)

def _ref_arrays(spec, s, amp):
    """Reference position / velocity / acceleration for a stage spec."""
    if spec['mode'] == 'harmonic':
        w = 2.0 * math.pi * spec['f_hz']
        return (amp * np.cos(w * s), -amp * w * np.sin(w * s), -amp * w * w * np.cos(w * s))
    move = spec['move']
    dwell = spec['dwell']
    phase0 = spec.get('phase0', 0.0)
    per = 2.0 * (move + dwell)
    u = np.mod(s + phase0, per)
    r = np.zeros_like(u)
    rd = np.zeros_like(u)
    rdd = np.zeros_like(u)
    m1 = u < dwell
    r[m1] = amp
    m2 = ~m1 & (u < dwell + move)
    if np.any(m2):
        tau = (u[m2] - dwell) / move
        p, v, a = _quintic(tau)
        r[m2] = amp - 2.0 * amp * p
        rd[m2] = -2.0 * amp * v / move
        rdd[m2] = -2.0 * amp * a / (move * move)
    m3 = ~m1 & ~m2 & (u < 2.0 * dwell + move)
    r[m3] = -amp
    m4 = ~(m1 | m2 | m3)
    if np.any(m4):
        tau = (u[m4] - (2.0 * dwell + move)) / move
        p, v, a = _quintic(tau)
        r[m4] = -amp + 2.0 * amp * p
        rd[m4] = 2.0 * amp * v / move
        rdd[m4] = 2.0 * amp * a / (move * move)
    return (r, rd, rdd)

def classify(feat, protos, gamma):
    d = [math.sqrt((protos[i][0] - feat[0]) ** 2 + (protos[i][1] - feat[1]) ** 2 + (protos[i][2] - feat[2]) ** 2 + (protos[i][3] - feat[3]) ** 2) for i in range(len(protos))]
    order = sorted(range(len(d)), key=d.__getitem__)
    d1 = d[order[0]]
    d2 = d[order[1]]
    margin = (d2 - d1) / (d2 + d1 + 1e-09)
    conf = margin * math.exp(-gamma * d1)
    if conf < 0.0:
        conf = 0.0
    elif conf > 1.0:
        conf = 1.0
    return (order[0] + 1, conf)

def clamp(x, a, b):
    return min(max(x, a), b)

def friction(v, p):
    return math.copysign(p['fc'], v) if abs(v) > 1e-10 else 0.0

def advance(x, v, u, p, dt=0.0005, force=0.0):
    drive = u - p['b'] * v - p['k'] * x - force
    if abs(v) < 0.003 and abs(drive) <= p['fs']:
        return (x, 0.0, 1)
    f = math.copysign(p['fc'], v if abs(v) > 1e-10 else drive)
    vn = v + dt * (drive - f) / p['J']
    if v * vn < 0 and abs(drive) <= p['fs']:
        vn = 0.0
    return (x + dt * vn, vn, 0)

def lcg(seed, n):
    a = np.empty(n)
    state = int(seed) & 4294967295
    for k in range(n):
        state = 1664525 * state + 1013904223 & 4294967295
        a[k] = (state + 0.5) / 4294967296.0
    return a

def context(seed=7, drift=0.35, scenario=None, duration=40.0, substeps=2, ood=True, wallphase=0.4, noise_scale=1.0, fault_f=(12.0, 24.0), zeta=0.012, wallk=300.0, wallx=0.43):
    n = round(duration / DT)
    t = np.arange(n) * DT
    stage = np.minimum((t // 10).astype(int), 3) if scenario is None else np.full(n, scenario)
    r = np.zeros(n)
    rd = r.copy()
    rdd = r.copy()
    for s in range(4):
        mask = stage == s
        tr = t[mask] - (10 * s if scenario is None else 0.0)
        if not mask.any():
            continue
        spec = STAGE_SPECS[s]
        rr, vv, aa = _ref_arrays(spec, tr, 0.5 if scenario is None else 0.46)
        r[mask] = rr
        rd[mask] = vv
        rdd[mask] = aa
    dd = np.where(stage >= 1, drift, 0.0)
    p = {key: np.full(n, val, dtype=float) for key, val in PARAM.items()}
    p['J'] *= 1 + dd
    p['b'] *= 1 + 0.3 * dd
    p['fc'][stage == 1] = 1.2 * (1 + 4.5 * drift)
    p['fs'][stage == 1] = 2.5 * (1 + 4.3 * drift)
    ran = lcg(seed + 11, 6 * n).reshape(6, n)
    white = (ran[0] + ran[1] + ran[2] - 1.5) * 2
    noise = white * np.where(stage == 2, 0.015, 0.0015) * noise_scale
    fault = (stage == 3) & ood & (t >= (30.037 if scenario is None else 0.037))
    sfault = np.where(fault, 0.025, 0.0)
    spike = np.where(fault & (ran[3] < 0.004), np.where(ran[4] > 0.5, 0.18, -0.18), 0.0)
    onset = np.flatnonzero(fault)
    if len(onset):
        spike[onset[0]] = 0.18
    ct = np.maximum(t - (20 if scenario is None else 0.0), 0.0)
    approach = 0.24 * np.exp(-ct / 0.25)
    wall = wallx + 0.025 * np.sin(2 * np.pi * 0.73 * t + wallphase) + approach
    wv = 0.025 * 2 * np.pi * 0.73 * np.cos(2 * np.pi * 0.73 * t + wallphase) - approach / 0.25
    wc = np.where(stage == 2, wallk, 0.0)
    nf = (n + 99) // 100
    emit = np.arange(nf) * 100
    rn = lcg(seed + 37, 2 * nf).reshape(2, nf)
    delays = np.where(stage[emit] >= 2, 0.06 + 0.06 * rn[0], 0.02)
    drops = (stage[emit] >= 2) & (rn[1] < 0.08)
    arr = {}
    for i, a in enumerate(emit + np.rint(delays / DT).astype(int)):
        if a < n and (not drops[i]):
            arr.setdefault(int(a), []).append(i)
    ft = np.maximum(t - (30 if scenario is None else 0.0), 0.0)
    freq = fault_f[0] + (fault_f[1] - fault_f[0]) * np.clip(ft / 10, 0, 1)
    return dict(t=t, stage=stage, r=r, rd=rd, rdd=rdd, p=p, noise=noise, sfault=sfault, spike=spike, wall=wall, wv=wv, wc=wc, emit={int(k): i for i, k in enumerate(emit)}, arr=arr, stamp=emit * DT, substeps=substeps, fault=fault, freq=freq, zeta=zeta)

class Probe:

    def __init__(self, x):
        self.xh = x
        self.vh = 0.0
        self.lp = 0.0
        self.buf = []
        self.yf = x
        self.vf = 0.0
        self.me = 0.0
        self.mu = 0.0
        self.hf = 0.0
        self.sp = 0.0
        self.rough = 0.0017 ** 2
        self.noisef = x
        self.slowenergy = 0.0
        self.veto = 0.0
        self.y1 = x
        self.y2 = x
        self.yold = x
        self.yolder = x
        self.a1 = 0.0
        self.a2 = 0.0
        self.alp = 0.0
        self.ae = 0.0
        self.apbuf = []

    def step(self, y, u, r):
        self.buf.append(y)
        if len(self.buf) > 5:
            self.buf.pop(0)
        med = sorted(self.buf)[len(self.buf) // 2]
        self.sp = abs(y - med) / 0.12
        ym = med if self.sp > 1 else y
        xp, vp, _ = advance(self.xh, self.vh, u, PARAM, DT)
        innov = ym - xp
        self.lp += DT / 0.1 * (innov - self.lp)
        self.slowenergy += DT / 0.35 * (self.lp * self.lp - self.slowenergy)
        aa = DT / (DT + 1 / (2 * np.pi * 25))
        self.y1 += aa * (med - self.y1)
        self.y2 += aa * (self.y1 - self.y2)
        actual = (self.y2 - 2 * self.yold + self.yolder) / (DT * DT)
        self.yolder = self.yold
        self.yold = self.y2
        predicted = (u - PARAM['b'] * self.vf - PARAM['k'] * ym - friction(self.vf, PARAM)) / PARAM['J']
        self.apbuf.append(predicted)
        if len(self.apbuf) > 3:
            self.apbuf.pop(0)
        self.a1 += aa * (self.apbuf[0] - self.a1)
        self.a2 += aa * (self.a1 - self.a2)
        ar = actual - self.a2
        self.alp += DT / (DT + 1 / (2 * np.pi * 5)) * (ar - self.alp)
        self.ae += DT / 0.008 * ((ar - self.alp) ** 2 - self.ae)
        self.hf = math.sqrt(max(self.ae, 0)) / 500.0
        self.xh = xp + DT * 40.0 * innov
        self.vh = vp + DT * 400.0 * innov
        vd = (ym - self.yf) / 0.002
        self.yf += DT / 0.002 * (ym - self.yf)
        self.vf += DT / 0.002 * (vd - self.vf)
        self.noisef += DT / 0.003 * (ym - self.noisef)
        self.rough += DT / 0.15 * ((ym - self.noisef) ** 2 - self.rough)
        self.me += DT / 0.25 * ((r - ym) ** 2 - self.me)
        self.mu += DT / 0.25 * (u * u - self.mu)
        self.veto = max(self.sp, self.hf, self.veto * (1 - DT / 1.0))
        return (ym, self.vf, self.veto)

    def features(self):
        q = np.array([math.sqrt(max(self.me, 0)), math.sqrt(max(self.mu, 0)), math.sqrt(max(self.slowenergy, 0)), math.sqrt(max(self.rough, 0))])
        return q / (q + FSCALE) * np.array([1.0, 1.0, 0.5, 1.4])

def fullfeat(x, v, r, rd, rdd, macro):
    return np.r_[(r - x) / 0.1, rd - v, r / 0.5, rd / 2.0, rdd / 6.0, macro]

def simulate(c, mode='b1', policy=None, clf=None, tuning=(1, 1, 1), bank=None, ood_gate=True, conf_gate=True, ffsafe=0.0, baseline=None):
    n = len(c['t'])
    x = float(c['r'][0])
    v = 0.0
    xm = x
    vm = 0.0
    wasflex = False
    u0 = 0.0
    integ = 0.0
    probe = Probe(x)
    ctx = np.zeros(4)
    frames = {}
    last_stamp = 0.0
    cfg = dict(CFG)
    cfg['ff_scale'] = list(CFG['ff_scale'])
    cfg['ff_scale'][3] = ffsafe
    if clf is not None:
        cfg.update(min_ai_confidence=clf[2], confidence_clear_ratio=clf[3])
    core = make_core(cfg, tuning)
    ai = 1
    conf = 1.0
    calls = 0
    log = np.zeros((n, 14))
    macros = []
    for k, tk in enumerate(c['t']):
        r, rd, rdd = (c['r'][k], c['rd'][k], c['rdd'][k])
        y = x + c['noise'][k] + c['sfault'][k] + c['spike'][k]
        ym, yd, res = probe.step(y, u0, r)
        if k in c['emit']:
            mi = c['emit'][k]
            frames[mi] = probe.features()
            macros.append((tk, *frames[mi]))
        for mi in c['arr'].get(k, []):
            if c['stamp'][mi] >= last_stamp:
                last_stamp = c['stamp'][mi]
                ctx = frames[mi]
                if clf is not None:
                    ai, conf = classify(ctx, clf[0], clf[1])
        e = r - ym
        ed = rd - yd
        uff = PARAM['J'] * rdd + PARAM['b'] * rd + PARAM['k'] * r + friction(rd, PARAM)
        gate = 0
        reg = 0
        if mode == 'b2':
            raw = float(policy(fullfeat(ym, yd, r, rd, rdd, ctx)))
            calls += 1
            u = clamp(clamp(raw, -20, 20), u0 - 2.0, u0 + 2.0)
        elif mode == 'b1':
            kp, kd, ki = BASE if baseline is None else baseline
            raw = kp * e + kd * ed + ki * integ + uff
            u = clamp(clamp(raw, -20, 20), u0 - 2.0, u0 + 2.0)
            if ki:
                integ = clamp(integ + DT * e + (u - raw) * DT / (ki * 0.1), -0.5, 0.5)
        else:
            if bank is not None:
                ai = int(bank)
                conf = 1.0
            if mode == 'rules':
                ai = 3 if ctx[3] > 0.65 else 2 if ctx[2] > 0.25 or ctx[1] > 0.55 else 1
                conf = 1.0
            age = tk - last_stamp
            ci = conf if conf_gate else 1.0
            mai = 4 if age > cfg['outer_timeout_s'] else ai
            if mai == 4:
                ci = 0.0
            u, gate, reg, _, _, _, raw = core.step(mai, ci, age, res if ood_gate else 0.0, ym, yd, r, rd, uff)
        sample_x = x
        sample_xm = xm
        p = {key: float(a[k]) for key, a in c['p'].items()}
        contact = 0.0
        stuck = 0
        peak = 0.0
        if c['fault'][k] and (not wasflex):
            xm = x
            vm = v
            wasflex = True
        for j in range(c['substeps']):
            dt = DT / c['substeps']
            delta = x - c['wall'][k]
            contact = max(0.0, c['wc'][k] * delta + 0.8 * (v - c['wv'][k])) if delta > 0 and c['wc'][k] > 0 else 0.0
            peak = max(peak, contact / 0.25)
            if c['fault'][k]:
                jm = p['J'] * 0.2
                jl = p['J'] * 0.8
                red = jm * jl / (jm + jl)
                w = 2 * np.pi * c['freq'][k]
                kc = red * w * w
                dc = 2 * c['zeta'] * w * red
                spring = kc * (xm - x) + dc * (vm - v)
                vm += dt * (u - 0.1 * vm - spring) / jm
                xm += dt * vm
                pl = dict(p)
                pl['J'] = jl
                x, v, stuck = advance(x, v, spring, pl, dt, contact)
            else:
                x, v, stuck = advance(x, v, u, p, dt, contact)
        log[k] = [sample_x, u, gate, reg, peak, stuck, res, conf, probe.hf, ai, ym, sample_xm, yd, integ]
        u0 = u
    return dict(log=log, macro=np.asarray(macros), calls=calls)

def metrics(c, z):
    a = z['log']
    out = {}
    for s in np.unique(c['stage']):
        q = c['stage'] == s
        out['S' + str(s + 1)] = dict(rmse=round(float(np.sqrt(np.mean((a[q, 0] - c['r'][q]) ** 2))), 6), tv=round(float(np.abs(np.diff(a[q, 1])).sum()), 2), breach=int(np.sum(np.abs(a[q, 0]) > 1)), peak_N=round(float(np.max(a[q, 4])), 2), stick=int(a[q, 5].sum()), ood=int(np.sum(a[q, 2] == 2)), safe=int(np.sum(a[q, 3] == 4)))
    return out

class Policy:

    def __init__(self, X, Y, nc=128, ridge=1e-05):
        self.mu = X.mean(0)
        self.sd = np.maximum(X.std(0), 0.05)
        Z = (X - self.mu) / self.sd
        idx = [0]
        dist = ((Z - Z[0]) ** 2).sum(1)
        for j in range(1, nc):
            i = int(np.argmax(dist))
            idx.append(i)
            dist = np.minimum(dist, ((Z - Z[i]) ** 2).sum(1))
        self.centers = Z[idx]
        d = ((self.centers[:, None, :] - self.centers[None, :, :]) ** 2).sum(2)
        np.fill_diagonal(d, np.inf)
        self.width = float(np.median(np.min(d, axis=1)) ** 0.5 * 2)
        phi = self.basis(Z)
        self.w = np.linalg.solve(phi.T @ phi + ridge * len(X) * np.eye(phi.shape[1]), phi.T @ Y)

    def basis(self, Z):
        rbf = np.exp(-np.maximum((Z * Z).sum(1)[:, None] + (self.centers * self.centers).sum(1)[None, :] - 2 * Z @ self.centers.T, 0) / (2 * self.width ** 2))
        return np.c_[np.ones(len(Z)), Z, rbf]

    def __call__(self, f):
        z = (f - self.mu) / self.sd
        phi = np.exp(-np.sum((self.centers - z) ** 2, axis=1) / (2 * self.width ** 2))
        return self.w[0] + z @ self.w[1:1 + len(z)] + phi @ self.w[1 + len(z):]

def calibrate(groups, n=12):
    from minimal_template import auto_calibrate
    X = np.vstack([g[10:, 1:][np.linspace(0, len(g) - 11, n).astype(int)] for g in groups])
    return auto_calibrate(X, np.repeat(np.arange(1, 4), n))

def train_policy(n=16000, centers=512, drift=0.0):
    X = lcg(933, 9 * n).reshape(n, 9) * 2 - 1
    X[:, 5:] = (X[:, 5:] + 1) / 2
    X[:, 8] *= 1.4
    h = np.clip((X[:, 8] - 0.65) / 0.35, 0, 1)
    kp = 60 * (1 - h) + 8 * h
    kd = 1.2 * (1 - h) + 0.8 * h
    ff = PARAM['J'] * (1 + drift) * 6 * X[:, 4] + PARAM['b'] * 2 * X[:, 3] + PARAM['k'] * 0.5 * X[:, 2] + PARAM['fc'] * np.sign(X[:, 3])
    Y = kp * 0.1 * X[:, 0] + kd * X[:, 1] + ff
    if drift:
        Y += (1.2 * (1 + 4.5 * drift) - PARAM["fc"]) * np.sign(2 * X[:, 3] - X[:, 1])
    return Policy(X, Y, min(centers, n), 1e-07)

def commission():
    groups = [simulate(context(seed=101, drift=0.0, scenario=s, duration=10.0, ood=False), 'fixed', bank=s + 1)['macro'] for s in range(3)]
    return calibrate(groups)

def print_table(c, results):
    print('| Stage | Branch | RMSE [rad] | TV [N m] | Violation ticks | Peak contact [N] |')
    print('|---|---|---:|---:|---:|---:|')
    for name, z in results.items():
        for stage, m in metrics(c, z).items():
            print(f"| {stage} | {name} | {m['rmse']:.6f} | {m['tv']:.2f} | {m['breach']} | {m['peak_N']:.2f} |")

def tuning_study(clf):
    from minimal_template import auto_field_tune
    rows = []
    for drift in (0.35, 0.5):
        trial = context(seed=501, drift=drift, scenario=1, duration=4.0, ood=False)

        def evaluate(knobs):
            z = simulate(trial, 'dynamic', clf=clf, tuning=knobs)
            a = z['log']
            return (float(np.sqrt(np.mean((trial['r'] - a[:, 10]) ** 2))), bool(np.all(np.abs(a[:, 0]) <= 1) and np.all(np.isfinite(a[:, 1])) and (not np.any(np.isin(a[:, 2], (2, 4))))))
        start = time.perf_counter()
        tuned = auto_field_tune(evaluate, (1.0, 1.0, 1.0))
        valid = context(seed=811, drift=drift, scenario=1, duration=6.0, ood=False)
        for name in ('r', 'rd', 'rdd'):
            valid[name] *= 0.43 / 0.46
        before = simulate(valid, 'dynamic', clf=clf)
        after = simulate(valid, 'dynamic', clf=clf, tuning=tuned)
        whitebox = simulate(valid, 'b1')
        mission = context(seed=811, drift=drift)
        check = metrics(mission, simulate(mission, 'dynamic', clf=clf, tuning=tuned))
        rows.append(dict(drift=drift, knobs=list(tuned), before=metrics(valid, before)['S2'], after=metrics(valid, after)['S2'], b1=metrics(valid, whitebox)['S2'], mission=check, search_seconds=24, validation_seconds=12, compute_seconds=time.perf_counter() - start))
    return rows

def efficiency_study(clf):
    rows = []
    nominal = context(seed=303, drift=0.0, scenario=0, duration=6.0, ood=False)
    shifted = context(seed=303, drift=0.35, scenario=1, duration=6.0, ood=False)
    for n in (64, 256, 1024, 4096, 16000):
        model = train_policy(n, min(512, max(8, n // 8)))
        retrained = train_policy(n, min(512, max(8, n // 8)), drift=.35)
        rows.append(dict(samples=n, centers=len(model.centers), nominal=metrics(nominal, simulate(nominal, 'b2', policy=model))['S1']['rmse'], frozen=metrics(shifted, simulate(shifted, 'b2', policy=model))['S2']['rmse'], retrained=metrics(shifted, simulate(shifted, 'b2', policy=retrained))['S2']['rmse']))
    return rows

def resource_study(policy):
    """Separate model storage from state RAM and Python-host call timings."""
    count = policy.mu.size + policy.sd.size + policy.centers.size + policy.w.size + 1
    core = make_core(CFG, (1, 1, 1))
    feature = np.zeros(9)
    times = []
    for call in (lambda: policy(feature), lambda: core.step(1, 1, 0, 0, 0, 0, .1, 0, 0)):
        start = time.perf_counter()
        for _ in range(10000):
            call()
        times.append((time.perf_counter() - start) * 100)
    return dict(model_scalars=count, model_fp64_bytes=8*count, model_fp32_bytes=4*count,
                classifier_scalars=15, state_bytes=88, config_bytes=132, tuning_bytes=12,
                action_us=times[0], core_call_us=times[1], core_backend='Python' if USE_PYTHON else 'C99 via ctypes')

def selftest():
    from minimal_template import Core, State, Config, Tuning
    assert (ctypes.sizeof(State), ctypes.sizeof(Config), ctypes.sizeof(Tuning)) == (88, 132, 12)
    cfg = {k: [float(np.float32(x)) for x in v] if isinstance(v, list) else float(np.float32(v)) for k, v in CFG.items()}
    for k in ('min_dwell_ticks', 'ood_recovery_cooldown_ticks'):
        cfg[k] = int(cfg[k])
    c = Core(cfg)
    p = PythonGatekeeper(cfg, dict(gain_scale=1.0, damping_scale=1.0, safety_margin_scale=1.0))
    r = lcg(2026, 6000 * 6).reshape(6000, 6)
    maxerr = 0.0
    for k, z in enumerate(r):
        frame = [1 + int(z[0] * 3), float(z[1]), float(z[2] * 0.5), float(z[3] * 1.4), float((z[4] - 0.5) * 2.2), float((z[5] - 0.5) * 3), 0.3, 0.0, 0.1]
        if k in (100, 500, 1000):
            frame[4] = float('nan')
        frame = [float(np.float32(v)) for v in frame]
        a = c.step(*frame)
        b = p.step(*frame)
        assert a[1:3] == b[1:3] and a[4:6] == b[4:6], (k, a, b)
        maxerr = max(maxerr, abs(a[0] - b[0]))
    assert maxerr < 0.0001, maxerr
    for bad in (float('nan'), float('inf'), -float('inf')):
        q = dict(CFG)
        q['kp'] = [bad, *CFG['kp'][1:]]
        a = Core(q).step(1, 1, 0, 0, 0, 0, 0.1, 0, 0)
        b = PythonGatekeeper(q, dict(gain_scale=1.0, damping_scale=1.0, safety_margin_scale=1.0)).step(1, 1, 0, 0, 0, 0, 0.1, 0, 0)
        assert a[5] == b[5] == 1 and math.isfinite(a[0])
    print(f'PASS: 6000 frames; gate/regime/fault exact; max|du|={maxerr:.3g}')

def physics_checks():
    for drive in (0.5, 1.01):
        x, v, stuck = advance(0.0, 0.0, drive, PARAM)
        assert (stuck == 1) == (drive <= PARAM['fs'])
    for hz in (12, 18, 24):
        jm, jl = (0.0135 * 0.2, 0.0135 * 0.8)
        w = 2 * np.pi * hz
        red = jm * jl / (jm + jl)
        k = red * w * w
        d = 2 * 0.012 * w * red
        A = np.array([[0, 1, 0, 0], [-k / jm, -(0.1 + d) / jm, k / jm, d / jm], [0, 0, 0, 1], [k / jl, d / jl, -(k + 0.3) / jl, -(d + 0.0663) / jl]])
        assert np.max(np.linalg.eigvals(A).real) < 0
    assert lcg(18, 1000)[-1] == 0.16194512567017227

def plot_benchmark(c, results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for name, z in results.items():
        axes[0].plot(c['t'], z['log'][:, 0], label=name, lw=0.8)
        axes[1].plot(c['t'], z['log'][:, 1], label=name, lw=0.5, alpha=0.8)
    axes[0].plot(c['t'], c['r'], 'k--', lw=0.8, label='reference')
    for limit in (-1, 1):
        axes[0].axhline(limit, color='r', ls=':', lw=0.8)
    z = results['B3']['log']
    axes[2].plot(c['t'], z[:, 6], label='physics residual')
    axes[2].axhline(1, color='r', ls=':')
    axes[3].plot(c['t'], z[:, 3], label='regime')
    axes[3].plot(c['t'], z[:, 2], label='gate', alpha=0.6)
    for ax, label in zip(axes, ('State [rad]', 'Command [N m]', 'Normalized residual', 'Discrete state')):
        ax.set_ylabel(label)
        ax.grid(alpha=0.2)
        ax.legend(loc='upper right', ncol=3)
        for edge in (10, 20, 30):
            ax.axvline(edge, color='gray', lw=0.6)
    axes[-1].set_xlabel('Time [s]')
    fig.tight_layout()
    fig.savefig(Path(__file__).with_name('benchmark_4stages.png'), dpi=240)
    plt.close(fig)

def main():
    global USE_PYTHON
    USE_PYTHON = '--no-c' in sys.argv
    if '--selftest' in sys.argv:
        selftest()
        return 0
    start = time.perf_counter()
    physics_checks()
    clf = commission()
    policy = train_policy()
    c = context()
    results = {name: simulate(c, mode, policy=policy, clf=clf) for name, mode in (('B1', 'b1'), ('B2', 'b2'), ('B3', 'dynamic'))}
    ablation = {f'R{bank}': metrics(c, simulate(c, 'fixed', bank=bank)) for bank in range(1, 5)}
    ablation['dynamic'] = metrics(c, results['B3'])
    ablation['rules'] = metrics(c, simulate(c, 'rules', clf=clf))
    tune = tuning_study(clf)
    eff = efficiency_study(clf)
    report = dict(version='2.0.0', metrics={k: metrics(c, v) for k, v in results.items()}, ablation=ablation, tuning=tune, efficiency=eff, classifier=[clf[0].tolist(), *map(float, clf[1:])], calls=results['B2']['calls'], resources=resource_study(policy))
    m = report['metrics']
    assert results['B2']['calls'] == 40000
    assert all((m['B3'][s]['breach'] == 0 for s in m['B3']))
    assert m['B1']['S3']['peak_N'] > 20 and m['B3']['S3']['peak_N'] < 20
    assert m['B1']['S4']['breach'] > 0 and m['B2']['S4']['breach'] > 0
    assert all((row['after']['rmse'] < 0.6 * row['before']['rmse'] for row in tune))
    assert all((s['breach'] == 0 for row in tune for s in row['mission'].values()))
    if '--json' in sys.argv:
        print(json.dumps(report))
        return 0
    print_table(c, results)
    print('\nMode ablation')
    for name, row in ablation.items():
        print(name, json.dumps(row))
    print('\nFrozen-AI field tuning')
    for row in tune:
        print(json.dumps(row))
    print('\nSample efficiency (action labels)')
    for row in eff:
        print(json.dumps(row))
    print('\nResources (host timing includes language bindings)')
    print(json.dumps(report['resources']))
    plot_benchmark(c, results)
    print(f'All engineering checks passed. Wall time: {time.perf_counter() - start:.2f} s')
    return 0
if __name__ == '__main__':
    raise SystemExit(main())
