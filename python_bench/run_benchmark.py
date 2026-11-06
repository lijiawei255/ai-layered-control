# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Jiawei Li
# Source: https://github.com/lijiawei255/ai-layered-control
# License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
# Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
"""V2.0.2: white-box PID, conditioned continuous policy, and deterministic layered control."""
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
    max_physics_residual=1.0, residual_clear_ratio=0.5,
    # Three safety-envelope or OOD episodes latch GATE_LATCHED_SAFE; a field
    # acknowledgement releases it (--selftest exercises the release path).
    max_hold_ticks=100, max_fault_vetoes_before_latch=3)
BASE = (110.0, 1.5, 300.0)
FSCALE = np.array([0.5, 20.0, 0.006])
FAULT_LATENCY, FAULT_OOD, FAULT_LOW_CONF, FAULT_ENVELOPE, FAULT_NAN = (1, 2, 4, 8, 16)
FAULT_HOLD_TIMEOUT, FAULT_PERMANENT_LATCH = 32, 64
GATE_NORMAL, GATE_HOLD_LATENCY, GATE_VETO_OOD, GATE_VETO_LOW_CONF, GATE_VETO_ENVELOPE = range(5)
GATE_SAFE_ZERO, GATE_LATCHED_SAFE = 5, 6
REGIME_NOMINAL, REGIME_HIGH_LOAD, REGIME_BOUNDARY, REGIME_SAFE = range(1, 5)
# Off-manifold audit fixture: a NORMAL-amplitude, NORMAL-noise S1 trajectory
# with a non-zero payload offset --- a coupled constant load torque that the
# nominal predictor does not know about.  Each macro feature individually stays
# inside the empirical commissioning support, so the single-dimension range rule
# almost never retreats; the joint signature (elevated slow innovation together
# with a one-sided effort/contact-like trace, a combination no nominal prototype
# occupies) is only visible in the joint manifold geometry.
OFF_MANIFOLD = dict(scenario=0, ood=False, amplitude=0.46, load_bias=0.08)
USE_PYTHON = False

_F32_MAX = 3.4028234663852886e+38
# The frozen core tests finiteness with LAYERED_FINITE_LIMIT = 3.0e38f, i.e. the
# binary32 value below; comparing against the double 3e38 would disagree for the
# few representable values between 3e38 and 3.0e38f.
_FINITE_LIMIT = float(np.float32(3.0e38))

def _f32(v):
    """Round one host float to the binary32 value carried by the C ABI."""
    if v > _F32_MAX:
        return float('inf')
    if v < -_F32_MAX:
        return float('-inf')
    return float(np.float32(v))

def make_core(cfg, tuning):
    if USE_PYTHON:
        return PythonGatekeeper(cfg, dict(zip(('gain_scale', 'damping_scale', 'safety_margin_scale'), tuning)))
    from minimal_template import Core
    return Core(cfg, tuning)

class PythonGatekeeper:
    """Behavioral mirror of c99_layered_core/layered_gatekeeper.c: same decision
    order, same sticky latches, same integer bookkeeping, and every host float
    quantized to the binary32 value the ctypes boundary would carry.  --selftest
    proves gate/regime/fault and episode-counter equality on 6,000 shared frames
    plus the fault-path scenarios below.
    """
    __slots__ = ('cfg', 'tune', 'u_applied', 'u_target', 'integ', 'blend', 'control_started', 'regime', 'ood_latched', 'safety_latched', 'conf_latched', 'permanent_latched', 'dwell_ticks', 'recovery_ticks', 'fault_audit_bitmask', 'step_count', 'safety_veto_count', 'ood_veto_count', 'low_confidence_veto_count', 'latency_hold_count', 'corrupted_frame_count', 'hold_ticks', 'last_outer_seq', 'outer_age_ticks', 'latch_episode_count', 'latency_open')

    def __init__(self, cfg: dict, tune: dict):
        # The C ABI carries binary32 only, so quantize every host value before
        # it can affect a decision.  The caller's dicts are left untouched.
        int_fields = ('min_dwell_ticks', 'ood_recovery_cooldown_ticks',
                      'max_hold_ticks', 'max_fault_vetoes_before_latch')
        self.cfg = {k: (int(v) if k in int_fields else
                        [_f32(x) for x in v] if isinstance(v, list) else _f32(v))
                    for k, v in cfg.items()}
        self.set_tuning(tune)
        self.u_applied = self.u_target = self.blend = 0.0
        self.integ = [0.0, 0.0, 0.0, 0.0]
        self.regime = REGIME_NOMINAL
        # No native latency HOLD is open before the first tick.
        self.latency_open = False
        # Every integer state starts at zero: latches, counters, lease clocks.
        for name in ('control_started', 'ood_latched', 'safety_latched', 'conf_latched',
                     'permanent_latched', 'dwell_ticks', 'recovery_ticks', 'fault_audit_bitmask',
                     'step_count', 'safety_veto_count', 'ood_veto_count', 'low_confidence_veto_count',
                     'latency_hold_count', 'corrupted_frame_count', 'hold_ticks', 'last_outer_seq',
                     'outer_age_ticks', 'latch_episode_count'):
            setattr(self, name, 0)

    def set_tuning(self, tune: dict):
        """Quantize the field knobs to the same binary32 boundary as the C ABI."""
        self.tune = {key: _f32(value) for key, value in tune.items()}

    @staticmethod
    def _absf(v):
        return -v if v < 0.0 else v

    @staticmethod
    def _inc_sat(v):
        return min(v + 1, 4294967295)

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
    def _guard(v, limit):
        """Mirror of lg_guard_value(): keep an overflowed binary32 value finite."""
        if v != v:
            return 0.0
        if v > _FINITE_LIMIT:
            return limit
        if v < -_FINITE_LIMIT:
            return -limit
        return v

    @staticmethod
    def _next_toward_zero(v):
        """Mirror of lg_next_toward_zero(): one binary32 step toward zero."""
        if v == 0.0 or v != v:
            return 0.0
        bits = int(np.float32(v).view(np.uint32))
        magnitude = bits & 0x7FFFFFFF
        if magnitude <= 1:
            return 0.0
        bits = (bits & 0x80000000) | (magnitude - 1)
        return float(np.uint32(bits).view(np.float32))

    @staticmethod
    def _next_toward(frm, to):
        """Mirror of lg_next_toward(): one binary32 step from frm toward to."""
        if frm == to:
            return to
        bits = int(np.float32(frm).view(np.uint32))
        if frm < to:
            if bits == 0x80000000:
                bits = 0x00000001
            elif bits & 0x80000000:
                bits -= 1
            else:
                bits += 1
        elif bits == 0x00000000:
            bits = 0x80000001
        elif bits & 0x80000000:
            bits += 1
        else:
            bits -= 1
        return float(np.uint32(bits).view(np.float32))

    @staticmethod
    def _bound_rounded_away(base, delta):
        """Mirror of lg_bound_rounded_away(): did fl(base+delta) round away?

        TwoSum recovers the exact sign of the binary32 rounding residual; the
        overflow and underflow ranges answer conservatively so the caller can
        only shorten the slew bound.
        """
        if delta == 0.0:
            return False
        total = _f32(base + delta)
        if not -_F32_MAX <= total <= _F32_MAX:
            return True
        if total == 0.0:
            return False
        base_part = _f32(total - base)
        if not -_F32_MAX <= base_part <= _F32_MAX:
            return True
        error = _f32(_f32(base - _f32(total - base_part)) + _f32(delta - base_part))
        if not -_F32_MAX <= error <= _F32_MAX:
            return True
        return error < 0.0 if delta > 0.0 else error > 0.0

    @staticmethod
    def _ramp_toward_zero(u_applied, du_max):
        """Mirror of lg_ramp_toward_zero(): slew-limited walk to exactly 0.0.

        The de-energise step is rounded inward exactly like the normal slew
        bounds: nearest-even rounding of u_applied - du_max can land just
        outside the exact budget (at 1.5f with a 0.1f budget the subtraction
        yields a 0.100000024 step), so a bound that rounded away is pulled one
        binary32 step back toward u_applied.  The documented one-ULP floor
        still de-energises when even that bound cannot move the command, and it
        only applies while the budget itself is positive.
        """
        nxt = PythonGatekeeper._clampf(0.0, _f32(u_applied - du_max),
                                       _f32(u_applied + du_max))
        if du_max > 0.0 and u_applied != 0.0:
            delta = -du_max if u_applied > 0.0 else du_max
            if nxt != 0.0 and PythonGatekeeper._bound_rounded_away(u_applied, delta):
                nxt = PythonGatekeeper._next_toward(nxt, u_applied)
            if nxt == u_applied:
                nxt = PythonGatekeeper._next_toward_zero(u_applied)
        return nxt

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

    @staticmethod
    def _config_corrupt(cfg):
        finite = _FINITE_LIMIT
        values = [x for v in cfg.values() for x in (v if isinstance(v, list) else [v])]
        if any(not -finite <= v <= finite for v in values):
            return True
        positive = ('dt_s', 'outer_timeout_s', 'max_physics_residual', 'state_safety_limit',
                    'u_max', 'u_slew_max_per_s', 'max_hold_ticks')
        if any(cfg[k] <= 0 for k in positive):
            return True
        # hold_ticks saturates at the same value, so a UINT32_MAX budget can
        # never expire; the C core rejects it as a corrupt configuration.
        if cfg['max_hold_ticks'] == 4294967295:
            return True
        # The lease age saturates at UINT32_MAX ticks: a timeout beyond that
        # reach can never trip and is rejected exactly as in the C core.
        if cfg['outer_timeout_s'] >= _f32(_f32(4294967296.0) * cfg['dt_s']):
            return True
        if not (0 < cfg['blend_alpha'] <= 1 and 0 <= cfg['residual_clear_ratio'] <= 1
                and 0 <= cfg['min_ai_confidence'] <= cfg['confidence_clear_ratio'] <= 1
                and 0 < cfg['safety_trip_ratio'] <= 1 and 0 <= cfg['safety_clear_ratio'] <= cfg['safety_trip_ratio']):
            return True
        if any(cfg[k] < 0 for k in ('integ_limit', 'windup_tau_s', 'safety_predict_s')):
            return True
        # The per-tick slew budget is the binary32 product slew * dt.  One that
        # underflows to zero cannot be honoured by the SAFE_ZERO quantisation
        # floor and one that overflows makes any command reachable in one tick,
        # so both are degenerate configurations the core rejects.
        if cfg['u_slew_max_per_s'] > 0 and cfg['dt_s'] > 0:
            slew_dt = _f32(cfg['u_slew_max_per_s'] * cfg['dt_s'])
            if not slew_dt > 0 or not -finite <= slew_dt <= finite:
                return True
        return False

    @staticmethod
    def _pid(kp, kd, ki, ff, gain_scale, damping_scale, e, ed, integ, u_ff, u_max):
        """Normal-bank law, rounded exactly where the C expression rounds.

        C evaluates ``(kp*gain)*e + (kd*damp)*ed + (ki*gain)*integ + ff*u_ff``
        left to right with a float rounding after every product and every sum.
        """
        term_p = _f32(_f32(kp * gain_scale) * e)
        term_d = _f32(_f32(kd * damping_scale) * ed)
        term_i = _f32(_f32(ki * gain_scale) * integ)
        term_f = _f32(ff * u_ff)
        return PythonGatekeeper._guard(
            _f32(_f32(_f32(term_p + term_d) + term_i) + term_f), u_max)

    @staticmethod
    def _pid_safe(kp, kd, ki, e, ed, integ, u_max):
        """R4 law: fixed gains, no field scaling and no feedforward term."""
        return PythonGatekeeper._guard(
            _f32(_f32(_f32(kp * e) + _f32(kd * ed)) + _f32(ki * integ)), u_max)

    @staticmethod
    def _integrate(integ, ki_eff, e, dt, u_pid, u_actual, windup_tau, limit):
        """Tracking anti-windup with the binary32 rounding of ``lg_integrate``.

        A legal ki_eff * windup_tau_s that underflows to zero in binary32 would
        make the back-calculation a division by zero, so only a strictly
        positive finite denominator contributes a correction term.
        """
        nxt = _f32(integ + _f32(e * dt))
        if ki_eff != 0.0 and windup_tau > 0.0:
            denom = _f32(ki_eff * windup_tau)
            if denom > 0.0 and denom <= _FINITE_LIMIT:
                nxt = _f32(nxt + _f32(_f32(_f32(u_actual - u_pid) * dt) / denom))
        return PythonGatekeeper._clampf(nxt, -limit, limit)

    def step(self, ai_regime, ai_confidence, outer_age_s, physics_residual, x, x_dot, r, r_dot, u_ff, outer_seq):
        """One 1 kHz step.  Returns (u, gate, regime, blend, fault, corrupted,
        u_target) exactly like the C output struct.  Every arithmetic expression
        is rounded to binary32 at the point the C core rounds it: at dt = 0.001 s
        and a 2000 rad/s^2 slew limit, 2000.0f * 0.001f is exactly 2.0f, so the
        SAFE_ZERO ramp must be 7.0 -> 5.0 -> 3.0 -> 1.0 -> 0.0.
        """
        ai_regime, ai_confidence, outer_age_s = _f32(ai_regime), _f32(ai_confidence), _f32(outer_age_s)
        physics_residual, x, x_dot = _f32(physics_residual), _f32(x), _f32(x_dot)
        r, r_dot, u_ff = _f32(r), _f32(r_dot), _f32(u_ff)
        seq = int(outer_seq)
        if seq != outer_seq or not 0 <= seq <= 0xFFFFFFFF:
            raise ValueError("outer_seq must be a uint32 integer")
        outer_seq = seq
        cfg = self.cfg
        tune = self.tune
        finite = _FINITE_LIMIT
        fast_bad = (any(not -finite <= v <= finite for v in (physics_residual, x, x_dot, r, r_dot, u_ff))
                    or physics_residual < 0)
        config_bad = self._config_corrupt(cfg)
        if fast_bad or config_bad:
            firewall_du_max = _f32(self._absf(cfg['u_slew_max_per_s']) * self._absf(cfg['dt_s']))
            self.fault_audit_bitmask |= FAULT_NAN
            self.corrupted_frame_count = self._inc_sat(self.corrupted_frame_count)
            self.recovery_ticks = cfg['ood_recovery_cooldown_ticks']
            self.step_count = self._inc_sat(self.step_count)
            # An unusable fast frame must still advance the slow-frame lease:
            # the latency timeout is timed on this clock, so pausing it would
            # stretch the effective slow-loop timeout under alternating bad
            # fast frames.
            if not config_bad:
                if outer_seq != 0:
                    if self.last_outer_seq == 0 or outer_seq != self.last_outer_seq:
                        self.last_outer_seq = outer_seq
                        self.outer_age_ticks = 0
                    else:
                        self.outer_age_ticks = self._inc_sat(self.outer_age_ticks)
                elif self.last_outer_seq != 0:
                    self.outer_age_ticks = self._inc_sat(self.outer_age_ticks)
            # The hold budget always advances; a rejected configuration has no
            # trustworthy slew budget, so it gets the fixed corrupt-config budget.
            hold_budget = 100 if config_bad else cfg['max_hold_ticks']
            self.hold_ticks = self._inc_sat(self.hold_ticks)
            if self.hold_ticks > hold_budget:
                self.fault_audit_bitmask |= FAULT_HOLD_TIMEOUT | FAULT_PERMANENT_LATCH
                self.permanent_latched = 1
                self.u_target = 0.0
                self.integ = [0.0, 0.0, 0.0, 0.0]
                self.blend = 0.0
                self.regime = REGIME_SAFE
                if config_bad:
                    self.u_applied = 0.0
                else:
                    self.u_applied = self._ramp_toward_zero(self.u_applied, firewall_du_max)
                return (self.u_applied, GATE_SAFE_ZERO, REGIME_SAFE, self.blend,
                        self.fault_audit_bitmask, 1, 0.0)
            return (self.u_applied, GATE_HOLD_LATENCY, self.regime, self.blend, self.fault_audit_bitmask, 1, self.u_target)
        tuning_bad = not (.1 <= tune['gain_scale'] <= 4 and .1 <= tune['damping_scale'] <= 4 and .1 <= tune['safety_margin_scale'] <= 1)
        if tuning_bad:
            tune = dict(gain_scale=1.0, damping_scale=1.0, safety_margin_scale=1.0)
        mailbox_bad = (tuning_bad or any(not -finite <= v <= finite for v in (ai_regime, ai_confidence, outer_age_s))
                       or not 1 <= ai_regime <= 4 or not 0 <= ai_confidence <= 1 or outer_age_s < 0)
        if mailbox_bad:
            self.fault_audit_bitmask |= FAULT_NAN
            self.corrupted_frame_count = self._inc_sat(self.corrupted_frame_count)
            ai_regime, ai_confidence = 4.0, 0.0
        snap = self.control_started == 0
        conf_before = self.conf_latched
        safety_before = self.safety_latched
        ood_before = self.ood_latched
        u_max = self._absf(cfg['u_max'])
        integ_limit = self._absf(cfg['integ_limit'])
        dt_eff = self._absf(cfg['dt_s'])
        du_max = _f32(self._absf(cfg['u_slew_max_per_s']) * dt_eff)
        windup_tau = cfg['windup_tau_s']

        # L1 slow-frame lease: a live outer_seq is timed by the fast core itself.
        outer_age_eff = outer_age_s
        age_usable = -_FINITE_LIMIT <= outer_age_eff <= _FINITE_LIMIT
        seq_mailbox_bad = False
        if outer_seq != 0:
            if self.last_outer_seq == 0 or outer_seq != self.last_outer_seq:
                self.last_outer_seq = outer_seq
                self.outer_age_ticks = 0
            else:
                self.outer_age_ticks = self._inc_sat(self.outer_age_ticks)
            # C evaluates (float)ticks * dt, i.e. the uint32 count is rounded to
            # binary32 *before* the multiply; do the same here, otherwise a lease
            # beyond 2^24 ticks drifts by up to one tick from the core.
            outer_age_eff = _f32(_f32(float(self.outer_age_ticks)) * dt_eff)
            # A legal but extreme inner period can overflow that product.  It
            # saturates instead of reading as invalid input, so the timeout still
            # trips when the locally timed frame is that old.
            if not (-_FINITE_LIMIT <= outer_age_eff <= _FINITE_LIMIT):
                outer_age_eff = 3.402823466e+38
            age_usable = True
        elif self.last_outer_seq != 0:
            seq_mailbox_bad = True
        # A live sequence that falls back to zero is corruption of the same
        # mailbox: merge it into the corruption flag and count the frame once.
        if seq_mailbox_bad and not mailbox_bad:
            mailbox_bad = True
            self.fault_audit_bitmask |= FAULT_NAN
            self.corrupted_frame_count = self._inc_sat(self.corrupted_frame_count)

        limit_eff = _f32(cfg['state_safety_limit'] * tune['safety_margin_scale'])
        x_abs = self._absf(x)
        # Signed look-ahead: project the measured state along its own velocity.
        # The direction-blind |x| + T*|v| would veto a safe retreat toward zero.
        predicted_abs = self._absf(_f32(x + _f32(cfg['safety_predict_s'] * x_dot)))
        if self.safety_latched:
            clear_level = _f32(cfg['safety_clear_ratio'] * limit_eff)
            if x_abs < clear_level and predicted_abs < clear_level:
                self.safety_latched = 0
        elif x_abs > _f32(cfg['safety_trip_ratio'] * limit_eff) or predicted_abs > limit_eff:
            self.safety_latched = 1
            self.safety_veto_count = self._inc_sat(self.safety_veto_count)
            self.latch_episode_count = self._inc_sat(self.latch_episode_count)
            self.fault_audit_bitmask |= FAULT_ENVELOPE
        residual_abs = self._absf(physics_residual)
        if self.ood_latched:
            if residual_abs < _f32(cfg['residual_clear_ratio'] * cfg['max_physics_residual']):
                self.ood_latched = 0
        elif residual_abs > cfg['max_physics_residual']:
            self.ood_latched = 1
            self.ood_veto_count = self._inc_sat(self.ood_veto_count)
            self.latch_episode_count = self._inc_sat(self.latch_episode_count)
            self.fault_audit_bitmask |= FAULT_OOD
        mailbox_regime = REGIME_SAFE if mailbox_bad else self._mailbox_regime(ai_regime)
        if mailbox_bad or seq_mailbox_bad or ai_confidence < cfg['min_ai_confidence'] or self._mailbox_abstains(ai_regime):
            self.conf_latched = 1
            self.fault_audit_bitmask |= FAULT_LOW_CONF
        elif ai_confidence >= cfg['confidence_clear_ratio'] and (not self._mailbox_abstains(ai_regime)):
            self.conf_latched = 0
        conf_veto = self.conf_latched
        if conf_veto and (not conf_before):
            self.low_confidence_veto_count = self._inc_sat(self.low_confidence_veto_count)

        # Cumulative severe-fault latch: only a NEW safety/OOD episode arms it,
        # and the tolerance counts EPISODES since the last acknowledge.
        if (cfg['max_fault_vetoes_before_latch'] > 0
                and ((self.safety_latched and not safety_before) or (self.ood_latched and not ood_before))
                and self.latch_episode_count >= cfg['max_fault_vetoes_before_latch']):
            self.permanent_latched = 1
            self.fault_audit_bitmask |= FAULT_PERMANENT_LATCH

        latency_hold = age_usable and outer_age_eff > cfg['outer_timeout_s']
        if latency_hold:
            self.fault_audit_bitmask |= FAULT_LATENCY
            self.recovery_ticks = cfg['ood_recovery_cooldown_ticks']
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
            # Stale mailbox, healthy fast sensors: the deterministic fallback
            # closes the loop instead of freezing the last torque.
            gate = GATE_HOLD_LATENCY
            regime = REGIME_SAFE
        else:
            gate = GATE_NORMAL
            if self.recovery_ticks > 0:
                self.recovery_ticks -= 1
                regime = REGIME_SAFE
            elif mailbox_regime == self.regime:
                regime = self.regime
            elif self._inc_sat(self.dwell_ticks) >= cfg['min_dwell_ticks']:
                regime = mailbox_regime
            else:
                regime = self.regime
        if veto_active:
            self.recovery_ticks = cfg['ood_recovery_cooldown_ticks']
        # The cumulative latch outranks every other decision.
        if self.permanent_latched:
            gate = GATE_LATCHED_SAFE
            regime = REGIME_SAFE
        # Bounded native HOLD: past max_hold_ticks the command is de-energised.
        # latency_hold_count counts one latency-HOLD episode when the core enters
        # the hold, never one count per held tick or per age window.  The open
        # episode is tracked separately from hold_ticks because a corrupt fast
        # frame also advances hold_ticks without being a latency timeout.
        if gate == GATE_HOLD_LATENCY:
            if not self.latency_open:
                self.latency_hold_count = self._inc_sat(self.latency_hold_count)
                self.latency_open = True
            self.hold_ticks = self._inc_sat(self.hold_ticks)
            if self.hold_ticks > cfg['max_hold_ticks']:
                gate = GATE_SAFE_ZERO
        else:
            self.hold_ticks = 0
            self.latency_open = False
        if gate == GATE_SAFE_ZERO:
            self.fault_audit_bitmask |= FAULT_HOLD_TIMEOUT
            self.u_target = 0.0
            self.integ = [0.0, 0.0, 0.0, 0.0]
            self.blend = 0.0
            self.regime = REGIME_SAFE
            regime = REGIME_SAFE
            # Only a valid configuration reaches this path, so du_max is
            # trustworthy and the command ramps to zero instead of stepping.
            self.u_applied = self._ramp_toward_zero(self.u_applied, du_max)
        do_control = gate != GATE_SAFE_ZERO
        kp = cfg['kp']
        kd = cfg['kd']
        ki = cfg['ki']
        ff_scale = cfg['ff_scale']
        gain_scale = tune['gain_scale']
        damping_scale = tune['damping_scale']
        alpha = cfg['blend_alpha']
        if do_control:
            e = _f32(r - x)
            ed = _f32(r_dot - x_dot)
            u_pid0 = self._pid(kp[0], kd[0], ki[0], ff_scale[0], gain_scale, damping_scale, e, ed, self.integ[0], u_ff, u_max)
            u_pid1 = self._pid(kp[1], kd[1], ki[1], ff_scale[1], gain_scale, damping_scale, e, ed, self.integ[1], u_ff, u_max)
            u_pid2 = self._pid(kp[2], kd[2], ki[2], ff_scale[2], gain_scale, damping_scale, e, ed, self.integ[2], u_ff, u_max)
            u_pid3 = self._pid_safe(kp[3], kd[3], ki[3], -x, -x_dot, self.integ[3], u_max)
            bank = regime - 1
            if bank < 0:
                bank = 0
            elif bank > 3:
                bank = 3
            u_active = (u_pid0, u_pid1, u_pid2, u_pid3)[bank]
            if snap:
                self.u_target = u_active
                self.blend = 1.0
            elif (veto_active or self.permanent_latched or gate == GATE_HOLD_LATENCY) and regime == REGIME_SAFE:
                # Emergency R4 entry (veto, permanent latch or stale mailbox)
                # bypasses the slow blend filter.
                self.u_target = u_active
                self.blend = 1.0
            else:
                if regime != self.regime:
                    self.blend = 0.0
                self.u_target = _f32(self.u_target + _f32(alpha * _f32(u_active - self.u_target)))
                self.blend = self._clampf(_f32(self.blend + alpha), 0.0, 1.0)
            if not (-_FINITE_LIMIT <= self.u_target <= _FINITE_LIMIT):
                # An extreme but legal gain product must never store a
                # non-finite target: clamp it by sign into the actuator range.
                self.u_target = self._guard(self.u_target, u_max)
            u_cmd = self._clampf(self.u_target, -u_max, u_max)
            # The two slew bounds are binary32; nearest-even rounding of
            # u_applied +/- du_max can land outside the exact budget, so a bound
            # that rounded away is pulled one representable value back toward
            # u_applied (mirror of the C core's TwoSum inward rounding).
            u_hi = _f32(self.u_applied + du_max)
            u_lo = _f32(self.u_applied - du_max)
            if self._bound_rounded_away(self.u_applied, du_max):
                u_hi = self._next_toward(u_hi, self.u_applied)
            if self._bound_rounded_away(self.u_applied, -du_max):
                u_lo = self._next_toward(u_lo, self.u_applied)
            u_cmd = self._clampf(u_cmd, u_lo, u_hi)
            # The active bank's anti-windup tracks the magnitude/slew-limited
            # command BEFORE the blend filter, so transfer lag is never mistaken
            # for actuator saturation; inactive banks track the applied command.
            u_bank_limited = self._clampf(self._clampf(u_active, -u_max, u_max), u_lo, u_hi)
            self.integ[0] = self._integrate(self.integ[0], _f32(ki[0] * gain_scale), e, dt_eff,
                                            u_pid0, u_bank_limited if bank == 0 else u_cmd,
                                            windup_tau, integ_limit)
            self.integ[1] = self._integrate(self.integ[1], _f32(ki[1] * gain_scale), e, dt_eff,
                                            u_pid1, u_bank_limited if bank == 1 else u_cmd,
                                            windup_tau, integ_limit)
            self.integ[2] = self._integrate(self.integ[2], _f32(ki[2] * gain_scale), e, dt_eff,
                                            u_pid2, u_bank_limited if bank == 2 else u_cmd,
                                            windup_tau, integ_limit)
            self.integ[3] = self._integrate(self.integ[3], ki[3], -x, dt_eff,
                                            u_pid3, u_bank_limited if bank == 3 else u_cmd,
                                            windup_tau, integ_limit)
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
            self.dwell_ticks = self._inc_sat(self.dwell_ticks)
        self.regime = regime
        self.step_count = self._inc_sat(self.step_count)
        return (out_u, gate, regime, out_blend, self.fault_audit_bitmask, int(mailbox_bad), out_u_target)

    def acknowledge_fault_latch(self):
        """Mirror of Layered_AcknowledgeFaultLatch(): release the latch, reset the
        episode quota and keep the audit history (fault bits, step_count and
        every veto counter)."""
        if self._config_corrupt(self.cfg):
            return 0
        self.permanent_latched = 0
        self.ood_latched = 0
        self.safety_latched = 0
        self.conf_latched = 0
        self.hold_ticks = 0
        self.latch_episode_count = 0
        self.recovery_ticks = self.cfg['ood_recovery_cooldown_ticks']
        return 1

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
    m3 = ~m1 & ~m2 & (u < 2.0 * dwell + move)
    r[m3] = -amp
    m4 = ~(m1 | m2 | m3)
    # The ramp offset is subtracted in the same association as the MATLAB mirror:
    # the return ramp is ((u - 2*dwell) - move)/move, not (u - (2*dwell+move))/move.
    # Both spellings are mathematically equal but round the quintic time ratio
    # differently by one binary64 ULP, which the contact fixture would amplify.
    for mask, sgn, second in ((m2, -1.0, False), (m4, 1.0, True)):
        if not np.any(mask):
            continue
        tau = (((u[mask] - 2.0 * dwell) - move) if second
               else (u[mask] - dwell)) / move
        p5, v5, a5 = _quintic(tau)
        r[mask] = sgn * (-amp + 2.0 * amp * p5)
        rd[mask] = sgn * 2.0 * amp * v5 / move
        rdd[mask] = sgn * 2.0 * amp * a5 / (move * move)
    return (r, rd, rdd)

def classify(feat, protos, gamma):
    """Nearest-prototype regime plus the split-conformal manifold confidence.

    The confidence collapses when the sample is far from the manifold (small
    exp(-gamma*d1)) OR when it sits between two prototypes (small margin), so a
    joint anomaly that mixes two otherwise normal conditions trips the same
    threshold that single-dimension range rules cannot see.
    """
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

def context(seed=7, drift=0.35, scenario=None, duration=40.0, substeps=2, ood=True, wallphase=0.4, noise_scale=1.0, fault_f=(12.0, 24.0), zeta=0.012, wallk=300.0, wallx=0.43, contact_noise=.006, amplitude=None,
            act_gain=1.0, act_f=0.0, act_zeta=1.0, act_dz=0.0, j_scale=1.0, b_scale=1.0, f_scale=1.0, load_bias=0.0):
    """Build one benchmark fixture.

    act_gain/act_f/act_zeta/act_dz model an unmodelled slow actuator: a force
    scale error, a second-order response (zeta >= 1 lag, zeta < 1 a lightly
    damped compliance mode) and a linkage dead zone; the default is the
    commissioned actuator, so the nominal mission is unchanged.
    """
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
        amp = (0.5 if scenario is None else 0.46) if amplitude is None else amplitude
        rr, vv, aa = _ref_arrays(spec, tr, amp)
        r[mask] = rr
        rd[mask] = vv
        rdd[mask] = aa
    dd = np.where(stage >= 1, drift, 0.0)
    p = {key: np.full(n, val, dtype=float) for key, val in PARAM.items()}
    p['J'] *= 1 + dd
    p['b'] *= 1 + 0.3 * dd
    # The stiction rise must stay physical for a negative drift: the raw
    # 1 + 4.3*drift crosses zero at drift = -0.233, so clamp the positive scale.
    p['fc'][stage == 1] = 1.2 * max(0.25, 1 + 4.5 * drift)
    p['fs'][stage == 1] = 2.5 * max(0.25, 1 + 4.3 * drift)
    # Per-parameter device mismatch on the loaded/contact stages only, so every
    # physical parameter stays positive and S1 remains the commissioned plant.
    p['J'][stage >= 1] *= j_scale
    p['b'][stage >= 1] *= b_scale
    p['fc'][stage >= 1] *= f_scale
    p['fs'][stage >= 1] *= f_scale
    ran = lcg(seed + 11, 6 * n).reshape(6, n)
    white = (ran[0] + ran[1] + ran[2] - 1.5) * 2
    noise = white * np.where(stage == 2, contact_noise, 0.0015) * noise_scale
    fault = (stage == 3) & ood & (t >= (30.037 if scenario is None else 0.037))
    sfault = np.where(fault, 0.025, 0.0)
    # S4 OOD detection must stand on the real unmodelled-flexible-mode residual
    # and the random glitch process alone: no first-tick spike is injected, so
    # the reported detection delay and the envelope counts are honest.
    spike = np.where(fault & (ran[3] < 0.004), np.where(ran[4] > 0.5, 0.18, -0.18), 0.0)
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
    return dict(t=t, stage=stage, r=r, rd=rd, rdd=rdd, p=p, noise=noise, sfault=sfault, spike=spike, wall=wall, wv=wv, wc=wc, emit={int(k): i for i, k in enumerate(emit)}, arr=arr, stamp=emit * DT, substeps=substeps, fault=fault, freq=freq, zeta=zeta,
                act_gain=act_gain, act_f=act_f, act_zeta=act_zeta, act_dz=act_dz, load_bias=load_bias, mode=fault)

class Probe:

    def __init__(self, x):
        # Every filter state starts at the measured position / at rest.
        for name in ('vh lp vf me mu hf sp power load_energy contact slowenergy veto '
                     'a1 a2 alp ae').split():
            setattr(self, name, 0.0)
        self.lp2 = self.errlp = self.biaslp = 0.0
        self.xh = self.y1 = self.y2 = self.yold = self.yolder = self.yf = x
        self.cy, self.free, self.buf, self.apbuf = 1.0, 0, [], []
        self.retreat_s = self.stall_ticks = self.block_run = 0.0
        # Position-free release state: a 50 ms low-pass measured velocity gates
        # the "passed the remembered contact point under its own speed" test,
        # pass_run times that sustained crossing, and nsign is the sign of the
        # tracking error that last accumulated contact evidence.  A released
        # surface may only be re-armed by a command that still presses into it.
        self.vlp = self.pass_run = 0.0
        self.r_prev = x
        self.nsign = 0.0

    def step(self, y, u, r):
        previous_position = self.y2
        self.buf.append(y)
        if len(self.buf) > 5:
            self.buf.pop(0)
        med = sorted(self.buf)[len(self.buf) // 2]
        self.sp = abs(y - med) / 0.12
        ym = med if self.sp > 1 else y
        xp, vp, _ = advance(self.xh, self.vh, u, PARAM, DT)
        innov = ym - xp
        self.lp += DT / 0.1 * (innov - self.lp)
        self.lp2 += DT / 0.5 * (innov - self.lp2)
        self.errlp += DT / 0.5 * ((r - ym) - self.errlp)
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
        velocity = (self.y2 - previous_position) / DT
        self.vlp += DT / .05 * (velocity - self.vlp)
        self.power += DT / .05 * (PARAM['J'] * self.alp * velocity - self.power)
        self.load_energy += DT / .05 * ((PARAM['J'] * self.alp) ** 2 - self.load_energy)
        ratio = self.power / (self.load_energy + .25)
        # Contact evidence is built from position-free physics only, so the same
        # probe works at any wall position and never keys off an absolute
        # coordinate.  Two independent signatures:
        #   (a) a resistive effort/velocity product (the actuator does work
        #       against a blockage), normalized by the load energy;
        #   (b) a stalled measured velocity combined with a SUSTAINED one-signed
        #       tracking error.  Dry friction opposes the direction of motion and
        #       cancels in the signed average; a unilateral constraint does not,
        #       so the 0.5 s error mean separates S3 contact from S2 stiction.
        # A released but remembered surface may only be re-armed by a command
        # that still pushes into it.  A reference moving away (or already on the
        # inner side of the remembered contact) while the plant follows is a
        # commanded unload at any speed: no single-step reference threshold.
        apart = (self.nsign * (r - self.r_prev) < 0.0) or (self.nsign * (r - ym) < -0.02)
        self.unload = self.free and apart and self.nsign * self.vlp <= 0.0
        stall = 1.0 / (1.0 + (self.vf / 0.05) ** 2)
        self.stall_ticks = self.stall_ticks + 1 if abs(self.vf) < 0.05 else 0
        track_err = abs(r - ym)
        lead = abs(self.errlp) - 0.03
        evidence = 0.0
        if self.power > .10 and ratio > .03 and not self.unload:
            evidence = self.power - .10
        if lead > 0.0 and self.stall_ticks >= 20:
            # The one-signed-error term only counts after the measured velocity
            # has stayed genuinely stalled for 20 ms; it is capped so a sustained
            # blockage charges the memory to the feature-saturation level while a
            # single tracking transient can neither fake a contact nor keep the
            # surface release engaged.
            evidence = max(evidence, min(0.02, lead * stall))
        blocked = evidence > 0.0
        self.block_run = self.block_run + 1 if blocked else 0
        self.retreat_s = self.retreat_s + DT if max(ym, r) < self.cy - .08 else 0.0
        if blocked:
            self.cy = ym
            self.nsign = 1.0 if r >= ym else -1.0
        # Release: a remembered contact point is abandoned when the measured state
        # passes it with position-independent speed, or after 3 s of clear
        # retreat.  The free-state decay is 0.1 s, so the 10 Hz classifier sees
        # the contact feature collapse within one frame; a genuine 100 ms
        # re-blockage still re-latches.
        if ym > self.cy + 0.04 and self.vlp > 0.05:
            self.pass_run += 1
        else:
            self.pass_run = 0
        if self.free:
            # A genuine resistive-effort blockage re-arms the surface; the
            # commanded-unload rule above keeps an inertial reference transient
            # from doing so.
            if blocked:
                self.free = 0
        elif self.retreat_s > 3.0 or self.pass_run >= 100:
            self.free = 1
        tau = 0.10 if self.free else 8.0
        if evidence >= self.contact:
            self.contact = max(self.contact * (1 - DT / tau), evidence, 0.0)
        else:
            self.contact = max(self.contact * (1 - DT / tau), 0.0)
        self.me += DT / 0.25 * ((r - ym) ** 2 - self.me)
        self.mu += DT / 0.25 * (u * u - self.mu)
        self.veto = max(self.sp, self.hf, self.veto * (1 - DT / 1.0))
        self.r_prev = r
        return (ym, self.vf, self.veto)

    def features(self):
        q = np.array([math.sqrt(max(self.me, 0)), math.sqrt(max(self.mu, 0)), math.sqrt(max(self.slowenergy, 0))])
        return np.r_[q / (q + FSCALE) * [1.0, 1.0, .5], 1.4 * min(self.contact / .01, 1.0)]

def fullfeat(x, v, r, rd, rdd, macro, integ):
    """B2 conditioning vector: tracking kinematics, 10 Hz context and the
    clamped error integral (the history state a memoryless PD regression
    structurally lacks)."""
    return np.r_[(r - x) / 0.1, rd - v, r / 0.5, rd / 2.0, rdd / 6.0, macro, integ / 0.5]

# Safety shell for the fair baseline comparison: the SAME 96 B C99 core is
# mounted around the B1/B2 control law.  The normal banks are made transparent
# (zero P/I/D, unit feedforward, blend_alpha = 1) so the shell never silently
# re-tunes the graded baseline; every safety parameter -- state envelope,
# residual OOD threshold, hold budget, recovery cooldown, R4 escape law and the
# magnitude/slew limits -- is byte-identical to the B3 deployment.
def gate_shell_cfg(cfg):
    return dict(cfg, kp=[0.0, 0.0, 0.0, cfg['kp'][3]], kd=[0.0, 0.0, 0.0, cfg['kd'][3]],
                ki=[0.0, 0.0, 0.0, cfg['ki'][3]], ff_scale=[1.0, 1.0, 1.0, 0.0],
                blend_alpha=1.0)

def simulate(c, mode='b1', policy=None, clf=None, tuning=(1, 1, 1), bank=None, ood_gate=True, conf_gate=True, baseline=None, smoothing=False, corrupt_slow_at=math.inf, log_features=False, supervised=True, rule_bounds=None, skip_first_frame=False, conf_persist=5):
    n = len(c['t'])
    x = float(c['r'][0]); v = vm = u0 = integ = 0.0; xm = x; wasflex = False
    probe, ctx, frames = Probe(x), np.zeros(4), {}
    last_stamp, outer_seq = 0.0, 0
    wrapped = mode in ('b1g', 'b2g')
    base_mode = mode[:-1] if wrapped else mode
    cfg = dict(CFG)
    if not supervised:
    # Envelope-only ablation: neutralise the envelope and the cumulative latch,
    # nothing else; whether the residual reaches the core stays a separate switch.
        cfg.update(state_safety_limit=1.0e6, max_fault_vetoes_before_latch=0)
    if clf is not None:
        cfg.update(min_ai_confidence=clf[2], confidence_clear_ratio=clf[3])
    core = make_core(cfg, tuning)
    shell = make_core(gate_shell_cfg(cfg), (1., 1., 1.)) if wrapped else None
    ai, conf, calls = 1, 1.0, 0
    low_run = 0
    ctx_ready = not skip_first_frame
    log = np.zeros((n, 14))
    feats = np.zeros((n, 4)) if log_features else None
    macros, target, act, act_v = [], None, 0.0, 0.0
    for k, tk in enumerate(c['t']):
        r, rd, rdd = (c['r'][k], c['rd'][k], c['rdd'][k])
        y = x + c['noise'][k] + c['sfault'][k] + c['spike'][k]
        ym, yd, res = probe.step(y, u0, r)
        if log_features:
            feats[k] = probe.features()
        if k in c['emit']:
            mi = c['emit'][k]
            frames[mi] = probe.features()
            macros.append((tk, *frames[mi]))
        for mi in c['arr'].get(k, []):
            if c['stamp'][mi] >= last_stamp:
                last_stamp = c['stamp'][mi]
                ctx = frames[mi]
        # A delivered frame advances the sequence number; a dropped one
        # freezes it, so the core's L1 lease ages locally.
                outer_seq += 1
                if not (skip_first_frame and mi == 0):
                    ctx_ready = True
                if clf is not None and not (skip_first_frame and mi == 0):
                    ai, conf = classify(ctx, clf[0], clf[1])
                    # Temporal persistence: an isolated run of 10 Hz frames below
                    # the conformal trip level is a filter transient, not a
                    # regime loss.  Only `conf_persist` consecutive low frames
                    # reach the 1 kHz arbitration; the C core's own latch stays
                    # immediate.
                    if conf < clf[2]:
                        low_run += 1
                        if low_run < conf_persist:
                            conf = 1.0
                    else:
                        low_run = 0
        e = r - ym
        ed = rd - yd
        uff = PARAM['J'] * rdd + PARAM['b'] * rd + PARAM['k'] * r + friction(rd, PARAM)
        gate = 0
        reg = 0
        if base_mode == 'b2':
            raw = float(policy(fullfeat(ym, yd, r, rd, rdd, ctx, integ)))
            calls += 1
            u = clamp(clamp(raw, -20, 20), u0 - 2.0, u0 + 2.0)
            # The integral state is the branch's own anti-windup history: it is
            # a measured/derived feature, like the filter states the 10 Hz
            # context already carries, never an AI output.
            integ = clamp(integ + DT * e + (u - raw) * DT / (BASE[2] * 0.1), -0.5, 0.5)
        elif base_mode == 'b1':
            kp, kd, ki = BASE if baseline is None else baseline
            raw = kp * e + kd * ed + ki * integ + uff
            target = raw if target is None or not smoothing else target + cfg['blend_alpha'] * (raw - target)
            u = clamp(clamp(target, -20, 20), u0 - 2.0, u0 + 2.0)
            if ki:
                integ = clamp(integ + DT * e + (u - raw) * DT / (ki * 0.1), -0.5, 0.5)
        else:
            if bank is not None:
                ai = int(bank)
                conf = 1.0
            if mode == 'rules':
                # The hard rule tree is the fair single-dimension comparator: it
                # classifies with the same three thresholds and now also
                # retreats to R4 when any single feature leaves the data-derived
                # [min,max] band.  It has no joint (multi-feature) manifold vote.
                # Until the first real slow frame has arrived it stays nominal,
                # so the silent startup sample cannot fake an out-of-box retreat.
                if ctx_ready:
                    ai = 3 if ctx[3] > 0.65 else 2 if ctx[2] > 0.25 or ctx[1] > 0.55 else 1
                    conf = 1.0
                    if rule_bounds is not None:
                        lo, hi = rule_bounds
                        if np.any(ctx < lo) or np.any(ctx > hi):
                            ai = 4
                else:
                    ai, conf = 1, 1.0
            age = tk - last_stamp
            ci = conf if conf_gate else 1.0
            # The slow-loop timeout is NOT rewritten into an AI abstention here:
            # the core must see the real age and run its own GATE_HOLD_LATENCY
            # closed-loop fallback.
            if tk >= corrupt_slow_at:
                ci = float('nan')
            u, gate, reg, _, _, _, raw = core.step(ai, ci, age, res if ood_gate else 0.0, ym, yd, r, rd, uff, outer_seq)
        if shell is not None:
            # The baseline command is the feedforward term of a transparent
            # core: the same 96 B gate applies its envelope, residual, latency,
            # magnitude, slew and R4-escape logic to it unchanged.
            age = tk - last_stamp
            u = shell.step(1.0, 1.0, age, res if ood_gate else 0.0, ym, yd, r, rd, u, outer_seq)[0]
        sample_x, sample_xm = x, xm
        p = {key: float(a[k]) for key, a in c['p'].items()}
        # Unmodelled slow actuator: the real effort blends the commanded force
        # with a lightly damped compliance mode, so the nominal predictor sees a
        # persistent slow innovation and only a small fast signature.
        if c['act_f']:
            w = 2.0 * math.pi * c['act_f']
            act_v += DT * (w * w * (u - act) - 2.0 * c['act_zeta'] * w * act_v)
            act += DT * act_v
        else:
            act = u
        u_dev = u + c['act_gain'] * (act - u) - c['load_bias']
        if c['act_dz']:
            u_dev -= clamp(u_dev, -c['act_dz'], c['act_dz'])
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
            force = contact
            if c['mode'][k]:
                jm = p['J'] * 0.2
                jl = p['J'] * 0.8
                red = jm * jl / (jm + jl)
                w = 2 * np.pi * c['freq'][k]
                kc = red * w * w
                dc = 2 * c['zeta'] * w * red
                spring = kc * (xm - x) + dc * (vm - v)
                vm += dt * (u_dev - 0.1 * vm - spring) / jm
                xm += dt * vm
                pl = dict(p)
                pl['J'] = jl
                x, v, stuck = advance(x, v, spring, pl, dt, force)
            else:
                x, v, stuck = advance(x, v, u_dev, p, dt, force)
        log[k] = [sample_x, u, gate, reg, peak, stuck, res, conf, probe.hf, ai, ym, sample_xm, yd, integ]
        u0 = u
    return dict(log=log, macro=np.asarray(macros), calls=calls, feats=feats)

def metrics(c, z):
    a = z['log']
    out = {}
    for s in np.unique(c['stage']):
        q = c['stage'] == s
        out['S' + str(s + 1)] = dict(rmse=round(float(np.sqrt(np.mean((a[q, 0] - c['r'][q]) ** 2))), 6), tv=round(float(np.abs(np.diff(a[q, 1])).sum()), 2), breach=int(np.sum(np.abs(a[q, 0]) > 1)), peak_N=round(float(np.max(a[q, 4])), 2), peak_u=round(float(np.max(np.abs(a[q, 1]))), 2), latched=int(np.sum(a[q, 2] == 6)), stick=int(a[q, 5].sum()), ood=int(np.sum(a[q, 2] == 2)), safe=int(np.sum(a[q, 3] == 4)), veto_rate=float(np.mean(a[q, 2] != 0)), r3_rate=float(np.mean(a[q, 3] == 3)))
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

def calibration_windows(groups, n=12):
    """The labelled macro windows auto_calibrate() consumes, in engine order."""
    return np.vstack([g[10:, 1:][np.linspace(0, len(g) - 11, n).astype(int)] for g in groups])

def calibrate(groups, n=12):
    from minimal_template import auto_calibrate
    return auto_calibrate(calibration_windows(groups, n), np.repeat(np.arange(1, 4), n))

def feature_support(groups, n=12):
    """Empirical commissioning support of the four dimensionless macro features.

    The bounds are the observed per-dimension min/max of the same labelled
    calibration windows that fit the prototypes, expanded by 25 % of each
    feature's observed span.  They are therefore data-derived operating limits,
    not the algebraic saturation caps of the normalization: the single-dimension
    rule tree sees a real box and can hand control back when a feature genuinely
    leaves it, while it still cannot see the joint manifold geometry.
    """
    windows = calibration_windows(groups, n)
    lo0, hi0 = windows.min(axis=0), windows.max(axis=0)
    margin = 0.25 * (hi0 - lo0)
    return np.maximum(0.0, lo0 - margin), hi0 + margin

def conformal_level(a, n):
    """Finite-sample split-conformal order q = min(1, ceil((N+1)*a)/N)."""
    return min(1.0, math.ceil((n + 1) * a) / n)

CONF_PERSIST_FRAMES = 5

def off_manifold_threshold(clf):
    """Split-conformal abstention threshold for the audit operating condition.

    The audit grants the manifold detector a 1 % per-frame split-conformal
    false-positive budget on independent NOMINAL probes of the same operating
    condition.  The first emitted slow frame (the probe's silent startup sample)
    is excluded exactly as auto_calibrate() excludes it; a five-frame temporal
    persistence then keeps one filter transient from latching the 1 kHz core,
    while a genuine regime loss is still reported within 500 ms.  The clear
    level is a reachable 1.05x trip strictly below 1.0, so the confidence veto
    can recover instead of latching forever.
    """
    protos, gamma = clf[0], clf[1]
    nominal = context(seed=211, duration=20.0, scenario=0, ood=False, amplitude=0.46)
    z = simulate(nominal, 'dynamic', clf=clf)
    macro = z['macro']
    keep = macro[:, 0] >= 0.1
    confidence = np.array([classify(frame, protos, gamma)[1] for frame in macro[keep, 1:]])
    n = len(confidence)
    trip = float(np.sort(confidence)[max(0, math.ceil(0.01 * n) - 1)])  # 1% lower-tail order statistic
    return trip, float(min(0.999, 1.05 * trip)), n

def nominal_control(clf, audit_clf, seeds=(0, 1)):
    """Independent nominal runs under the audit threshold: no false abstention."""
    rows = []
    for seed in seeds:
        fx = context(seed=seed, duration=20.0, scenario=0, ood=False, amplitude=0.46)
        z = simulate(fx, 'dynamic', clf=audit_clf, skip_first_frame=True,
                     conf_persist=CONF_PERSIST_FRAMES)
        a, q = z['log'], fx['t'] >= 0.1
        rows.append(dict(seed=seed,
                         low_confidence_ticks=int(np.sum(a[q, 2] == GATE_VETO_LOW_CONF)),
                         r4_ticks=int(np.sum(a[q, 3] == REGIME_SAFE))))
    return rows

def train_policy(n=16000, centers=512, drift=0.0):
    X = lcg(933, 10 * n).reshape(n, 10) * 2 - 1
    # Three quarters concentrate on nominal trajectory kinematics and sensor-scale errors.
    near = np.arange(n) % 4 != 0
    for kind in (0, 1):
        q = near & (np.arange(n) % 2 == kind)
        r, rd, rdd = _ref_arrays(STAGE_SPECS[kind], (X[q, 4] + 1) * 5, .5)
        X[q, 0] *= .06
        X[q, 1] = 15 * X[:, 0][q] + .4 * X[q, 1]
        X[q, 2:5] = np.c_[r / .5, rd / 2, rdd / 6]
    X[:, 5:9] = (X[:, 5:9] + 1) / 2
    X[:, 8] *= 1.4
    X[near, 5:8] *= [.03, .10, .06]
    X[near, 8] = 0.0
    # Column 9 is the clamped error-integral history state, normalized by the
    # integrator limit so the training support covers the full [integ_limit,
    # +integ_limit] range.  The training target carries the same integral gain
    # as the B1 baseline (300), so the action head can represent integral
    # action; the previous memoryless PD target structurally could not.
    h = np.clip((X[:, 8] - 0.65) / 0.35, 0, 1)
    kp = 60 * (1 - h) + 8 * h
    kd = 1.2 * (1 - h) + 0.8 * h
    ff = PARAM['J'] * (1 + drift) * 6 * X[:, 4] + PARAM['b'] * 2 * X[:, 3] + PARAM['k'] * 0.5 * X[:, 2] + PARAM['fc'] * np.sign(X[:, 3])
    Y = kp * 0.1 * X[:, 0] + kd * X[:, 1] + BASE[2] * 0.5 * X[:, 9] + ff
    if drift:
        Y += (1.2 * (1 + 4.5 * drift) - PARAM["fc"]) * np.sign(2 * X[:, 3] - X[:, 1])
    return Policy(X, Y, min(centers, n), 1e-07)

def commission():
    groups = [simulate(context(seed=101, drift=0.0, scenario=s, duration=10.0, ood=False), 'fixed', bank=s + 1)['macro'] for s in range(3)]
    return calibrate(groups)

def print_table(c, results):
    print('| Stage | Branch | RMSE [rad] | TV [N m] | Violation ticks | Peak contact [N] |\n|---|---|---:|---:|---:|---:|')
    for name, z in results.items():
        for stage, m in metrics(c, z).items():
            print(f"| {stage} | {name} | {m['rmse']:.6f} | {m['tv']:.2f} | {m['breach']} | {m['peak_N']:.2f} |")

def tuning_study(clf):
    from minimal_template import auto_field_tune
    rows = []
    cases = (('drift=0.35', dict(drift=.35)), ('drift=0.50', dict(drift=.5)),
             ('cross J+40% b-30%', dict(drift=0., j_scale=1.4, b_scale=.7)),
             ('reverse J/b/fc/fs-25%', dict(drift=0., j_scale=.75, b_scale=.75, f_scale=.75)))
    for label, fx in cases:
        trial = context(seed=7, **fx)
        # Second, independently wall-phased contact probe: a candidate that only
        # clears the 20 N contact budget for one wall phase is not an accepted
        # tuning.  The two-seed validity is part of the commissioning record.
        contact_trial = context(seed=811, scenario=2, duration=10.0, ood=False, amplitude=.5, **fx)
        histories = {}
        selected = {}
        start = time.perf_counter()
        for mode in ('dynamic', 'b1g'):
            candidates = []
            def evaluate(knobs):
                baseline = (BASE[0]*knobs[0], BASE[1]*knobs[1], BASE[2]*knobs[0])
                # B1 is mounted on the same 96 B C99 safety gate as B3; the gain
                # search therefore starts from the same safety floor.
                z = simulate(trial, mode, clf=clf, tuning=knobs, baseline=baseline)
                m = metrics(trial, z)
                z_contact = simulate(contact_trial, mode, clf=clf, tuning=knobs, baseline=baseline)
                mc = metrics(contact_trial, z_contact)
                q = trial['stage'] == 1
                cost = float(np.sqrt(np.mean((trial['r'][q]-z['log'][q,10])**2)))
                valid = all(v['breach'] == 0 for v in m.values()) and m['S1']['rmse'] < .01
                valid &= all(m[k]['peak_N'] < 20 and m[k]['veto_rate'] <= .05 and m[k]['safe'] <= 1000 for k in ('S1','S2','S3'))
                valid &= all(v['breach'] == 0 for v in mc.values())
                valid &= all(mc[k]['peak_N'] < 20 and mc[k]['veto_rate'] <= .05 and mc[k]['safe'] <= 1000 for k in mc)
                candidates.append(dict(knobs=list(knobs), cost=cost, valid=bool(valid), metrics=m,
                                       contact_metrics=mc))
                return cost, valid
            selected[mode] = auto_field_tune(evaluate, (1.,1.,1.))
            histories[mode] = candidates
        tuned = selected['dynamic']
        # The gated single-stage baseline is graded on the same constraints.  If
        # no candidate satisfies them, the frozen search keeps the commissioned
        # knobs, which is reported as such instead of being asserted away.
        b1_valid = [c for c in histories['b1g'] if c['valid']]
        b1_valid_candidates = len(b1_valid)
        b1_knobs = min(histories['b1g'], key=lambda c: c['cost'])['knobs']
        valid = context(seed=811, scenario=1, duration=6., ood=False, amplitude=.43, **fx)
        baseline = (BASE[0]*b1_knobs[0],BASE[1]*b1_knobs[1],BASE[2]*b1_knobs[0])
        before = metrics(valid, simulate(valid, 'dynamic', clf=clf))['S2']
        after = metrics(valid, simulate(valid, 'dynamic', clf=clf, tuning=tuned))['S2']
        b1 = metrics(valid, simulate(valid, 'b1g'))['S2']
        b1_after = metrics(valid, simulate(valid, 'b1g', baseline=baseline))['S2']
        mission = context(seed=811, **fx)
        check = metrics(mission, simulate(mission, 'dynamic', clf=clf, tuning=tuned))
        b1_check = metrics(mission, simulate(mission, 'b1g', baseline=baseline))
        rows.append(dict(label=label, drift=fx['drift'], knobs=list(tuned), before=before, after=after, b1=b1,
            b1_tuned=b1_after, b1_knobs=list(b1_knobs), b1_feasible=b1_valid_candidates > 0,
            b1_valid_candidates=b1_valid_candidates,
            search_cost_before=histories['dynamic'][1]['cost'],
            search_cost_after=next(v['cost'] for v in histories['dynamic'] if v['knobs'] == list(tuned)),
            mission=check, mission_before=metrics(mission, simulate(mission, 'dynamic', clf=clf)),
            b1_mission=b1_check, candidates=histories,
            search_seconds=240, validation_seconds=52, compute_seconds=time.perf_counter()-start))
    return rows

def _off_row(c, z):
    """One slow-loop abstention row: tracking, abstention and fast-residual evidence."""
    a = z['log']
    return dict(rmse=round(float(np.sqrt(np.mean((a[:, 0] - c['r']) ** 2))), 6),
                peak_error=round(float(np.max(np.abs(a[:, 0] - c['r']))), 6),
                low_confidence_ticks=int(np.sum(a[:, 2] == GATE_VETO_LOW_CONF)),
                r4_ticks=int(np.sum(a[:, 3] == REGIME_SAFE)),
                max_residual=round(float(np.max(a[:, 6])), 3),
                max_abs_state=round(float(np.max(np.abs(a[:, 0]))), 3),
                breach=int(np.sum(np.abs(a[:, 0]) > 1)))

def reverse_context(seed=5):
    """10 s of S3 wall contact, then the wall is withdrawn and S1 free tracking resumes."""
    c = context(seed=seed, duration=20.0, scenario=0, ood=False)
    t, m1 = c['t'], c['t'] < 10.0
    c['stage'] = np.where(m1, 2, 0)
    for mask, spec, off in ((m1, STAGE_SPECS[2], 0.0), (~m1, STAGE_SPECS[0], 10.0)):
        c['r'][mask], c['rd'][mask], c['rdd'][mask] = _ref_arrays(spec, t[mask] - off, 0.5)
    c['wall'] = np.where(m1, c['wall'], 9.0)
    c['wc'] = np.where(m1, 300.0, 0.0)
    return c

def retreat_context(seed=5):
    """10 s of S3 wall contact, then the reference returns to the interior zero
    while the wall STAYS physically in place: only physical retreat can release
    the remembered contact (there is no wall-coordinate shortcut)."""
    c = context(seed=seed, duration=30.0, scenario=0, ood=False)
    t, m1 = c['t'], c['t'] < 10.0
    c['stage'] = np.where(m1, 2, 0)
    c['r'][m1], c['rd'][m1], c['rdd'][m1] = _ref_arrays(STAGE_SPECS[2], t[m1], 0.5)
    c['r'][~m1], c['rd'][~m1], c['rdd'][~m1] = 0.0, 0.0, 0.0
    c['wc'] = np.full(len(t), 300.0)
    return c

def audit_study(clf, c):
    low, no_wall, report = context(contact_noise=.0015), context(wallk=0), {}
    for name, fixture, mode in (('low_noise',low,'dynamic'), ('low_noise_rules',low,'rules'),
                               ('noise_only',no_wall,'dynamic'), ('noise_only_rules',no_wall,'rules')):
        report[name] = metrics(fixture, simulate(fixture, mode, clf=clf))
    report['b1_smoothed'] = metrics(c, simulate(c, 'b1', smoothing=True))
    corrupt = simulate(c, 'dynamic', clf=clf, corrupt_slow_at=22.)
    report['slow_corruption'] = metrics(c, corrupt)
    report['slow_corruption_maxabs'] = float(np.max(np.abs(corrupt['log'][:,0])))
    assert all(v['breach'] == 0 for v in report['slow_corruption'].values())
    for key in ('low_noise','low_noise_rules'):
        # Qualitative robustness check for the reduced-noise fixture; the strict
        # 20 N device budget is asserted on the main four-stage mission.
        assert report[key]['S3']['peak_N'] < 25 and report[key]['S3']['r3_rate'] > .90
    assert report['noise_only']['S3']['r3_rate'] == report['noise_only_rules']['S3']['r3_rate'] == 0
    # ---- Off-manifold abstention -------------------------------------------
    # Normal-amplitude, normal-noise trajectory + an unmodelled coupled constant
    # load torque.  The audit compares three slow-loop supervisors on the same
    # frames:
    #   * b3      : prototype / manifold-distance classifier with an independent
    #               1 % split-conformal threshold on nominal probes;
    #   * rules   : hard rule tree with data-derived single-dimension min/max
    #               retreat bands from the SAME calibration windows;
    #   * no_conf : the same b3 with the confidence veto disabled.
    # The fixture is deliberately in-box per dimension: the honest comparison is
    # the frame detection rate of the joint detector against the range rule, not
    # a tautological in_box == 1.0.
    groups = [simulate(context(seed=101, drift=0.0, scenario=s, duration=10.0, ood=False), 'fixed', bank=s + 1)['macro'] for s in range(3)]
    bounds = feature_support(groups)
    lo, hi = bounds
    off_trip, off_clear, off_cal_n = off_manifold_threshold(clf)
    audit_clf = (clf[0], clf[1], off_trip, off_clear)
    nominal_rows = nominal_control(clf, audit_clf)
    rows = []
    for seed in range(5):
        fx = context(seed=seed, duration=20.0, **OFF_MANIFOLD)
        zs = dict(b3=simulate(fx, 'dynamic', clf=audit_clf, skip_first_frame=True,
                              conf_persist=CONF_PERSIST_FRAMES),
                  no_conf=simulate(fx, 'dynamic', clf=audit_clf, conf_gate=False,
                                   log_features=True, skip_first_frame=True),
                  rules=simulate(fx, 'rules', clf=clf, rule_bounds=bounds,
                                 skip_first_frame=True))
        frames_no = zs['no_conf']['macro']
        keep = frames_no[:, 0] >= 0.1
        boxed = np.all((frames_no[keep, 1:] >= lo - 1e-9) & (frames_no[keep, 1:] <= hi + 1e-9), axis=1)
        in_box = float(np.mean(boxed))
        frames_b3 = zs['b3']['macro']
        conf = np.array([classify(frame, clf[0], clf[1])[1] for frame in frames_b3[keep, 1:]])
        b3_detection = float(np.mean(conf < off_trip))
        q = fx['t'] >= 0.1
        rules_retreat = float(np.mean(zs['rules']['log'][q, 3] == REGIME_SAFE))
        row = dict(seed=seed, in_box_fraction=round(in_box, 4),
                   b3_detection_frames=round(b3_detection, 4),
                   rules_retreat_rate=round(rules_retreat, 4),
                   **{name: _off_row(fx, z) for name, z in zs.items()})
        rows.append(row)
    report['off_manifold_abstention'] = dict(
        fixture=dict(OFF_MANIFOLD), calibration_frames=off_cal_n,
        conformal_trip=round(off_trip, 5), conformal_clear=round(off_clear, 5),
        confidence_persistence_frames=CONF_PERSIST_FRAMES,
        band=[lo.tolist(), hi.tolist()], nominal_control=nominal_rows, seeds=rows)
    for row in rows:
        # Honest in-box property: most single-dimension frames stay inside the
        # data-derived commissioning box, so the range rule rarely retreats.
        assert row['in_box_fraction'] >= 0.90, (row['seed'], row['in_box_fraction'])
        assert row['rules_retreat_rate'] < 0.10, row['seed']
        # The joint manifold detector must detect the coupled load bias at a far
        # higher frame rate than the single-dimension rule tree.
        assert row['b3_detection_frames'] > 0.80, row['seed']
        assert row['b3_detection_frames'] > max(5.0 * row['rules_retreat_rate'], 0.50), row['seed']
        # The physical envelope must hold, and the confidence ablation must not
        # abstain (the veto is the only difference between b3 and no_conf).
        assert row['b3']['low_confidence_ticks'] > 0, row['seed']
        assert row['b3']['breach'] == 0 and row['b3']['max_abs_state'] < 1.0, row['seed']
        assert row['no_conf']['low_confidence_ticks'] == 0, row['seed']
    for row in nominal_rows:
        # Nominal control: no offset, same detector -> no false abstention.
        assert row['low_confidence_ticks'] == 0, row
        assert row['r4_ticks'] == 0, row
    # ---- S3 -> S1 reverse transition: the compliant bank must let go ------
    rev = reverse_context()
    a = simulate(rev, 'dynamic', clf=clf)['log']
    q1, q2, tail = rev['stage'] == 2, rev['stage'] == 0, rev['t'] >= 18.0
    r3 = np.flatnonzero(q2 & (a[:, 3] == REGIME_BOUNDARY))
    report['reverse_s3_to_s1'] = dict(
        s3_peak_N=round(float(np.max(a[q1, 4])), 2),
        s1_r3_rate=round(float(np.mean(a[q2, 3] == REGIME_BOUNDARY)), 3),
        last2_r3_rate=round(float(np.mean(a[tail, 3] == REGIME_BOUNDARY)), 3),
        release_s=round(float(rev['t'][r3[-1]] - 10.0), 3) if len(r3) else 0.0)
    rv = report['reverse_s3_to_s1']
    # Functional release contract: once the wall is withdrawn the compliant bank
    # lets go and the late S1 window carries no residual contact routing.
    assert rv['s3_peak_N'] < 20.0 and rv['release_s'] < 0.35 and rv['last2_r3_rate'] == 0.0, rv
    # ---- S3 -> interior retreat with the wall still in place: the contact
    # memory must release on physical retreat alone (no absolute-coordinate cue).
    ret = retreat_context()
    a = simulate(ret, 'dynamic', clf=clf)['log']
    q2, tail = ret['stage'] == 0, ret['t'] >= 25.0
    r3 = np.flatnonzero(q2 & (a[:, 3] == REGIME_BOUNDARY))
    report['retreat_s3_to_s1'] = dict(
        s1_r3_rate=round(float(np.mean(a[q2, 3] == REGIME_BOUNDARY)), 3),
        release_s=round(float(ret['t'][r3[-1]] - 10.0), 3) if len(r3) else 0.0,
        last5_r3_rate=round(float(np.mean(a[tail, 3] == REGIME_BOUNDARY)), 3),
        s1_rmse=round(float(np.sqrt(np.mean((a[q2, 0] - ret['r'][q2]) ** 2))), 5))
    rv = report['retreat_s3_to_s1']
    assert rv['release_s'] < 0.50 and rv['last5_r3_rate'] == 0.0 and rv['s1_rmse'] < 0.05, rv
    return report

def efficiency_study():
    rows = []
    nominal = context(seed=303, drift=0.0, scenario=0, duration=6.0, ood=False)
    shifted = context(seed=303, drift=0.35, scenario=1, duration=6.0, ood=False)
    for n in (64, 256, 1024, 4096, 16000):
        model = train_policy(n, min(512, max(8, n // 8)))
        retrained = train_policy(n, min(512, max(8, n // 8)), drift=.35)
        rows.append(dict(samples=n, centers=len(model.centers),
                         nominal=metrics(nominal, simulate(nominal, 'b2', policy=model))['S1']['rmse'],
                         frozen=metrics(shifted, simulate(shifted, 'b2', policy=model))['S2']['rmse'],
                         retrained=metrics(shifted, simulate(shifted, 'b2', policy=retrained))['S2']['rmse']))
    return rows

def resource_study(policy):
    # Model storage, state RAM and host call timings are separate quantities.
    count = policy.mu.size + policy.sd.size + policy.centers.size + policy.w.size + 1
    core = make_core(CFG, (1, 1, 1))
    feature = np.zeros(10)
    times = []
    for call in (lambda: policy(feature), lambda: core.step(1, 1, 0, 0, 0, 0, .1, 0, 0, 0)):
        start = time.perf_counter()
        for _ in range(10000):
            call()
        times.append((time.perf_counter() - start) * 100)
    return dict(model_scalars=count, model_fp64_bytes=8*count, model_fp32_bytes=4*count,
                classifier_scalars=15, state_bytes=96, config_bytes=140, tuning_bytes=12, input_bytes=40,
                action_us=times[0], core_call_us=times[1], core_backend='Python' if USE_PYTHON else 'C99 via ctypes')

def selftest():
    from minimal_template import Core, State, Config, Tuning, Input, Output, auto_field_tune
    assert tuple(ctypes.sizeof(t) for t in (State, Config, Tuning, Input, Output)) == (96, 140, 12, 40, 28)
    cfg = {k: [float(np.float32(x)) for x in v] if isinstance(v, list) else float(np.float32(v)) for k, v in CFG.items()}
    for k in ('min_dwell_ticks', 'ood_recovery_cooldown_ticks', 'max_hold_ticks', 'max_fault_vetoes_before_latch'):
        cfg[k] = int(cfg[k])
    c = Core(cfg)
    for count in (0, 8, 9):
        try:
            c.step(*([0.] * count))
        except TypeError:
            pass
        else:
            raise AssertionError('Core.step must require exactly ten input fields')
    assert c.state.step_count == 0
    p = PythonGatekeeper(cfg, dict(gain_scale=1.0, damping_scale=1.0, safety_margin_scale=1.0))
    r = lcg(2026, 6000 * 6).reshape(6000, 6)
    maxerr, latched, seen = 0.0, 0, np.zeros(5, int)
    slew_limit = float(np.float32(cfg['u_slew_max_per_s'] * cfg['dt_s']))
    prev_u = 0.0
    for k, z in enumerate(r):
        if k % 1000 < 350:
            # Random inputs plus NaN injection keep every gate, bank and latch busy.
            frame = [1 + int(z[0] * 3), float(z[1]), float(z[2] * 0.5), float(z[3] * 1.4),
                     float((z[4] - 0.5) * 2.2), float((z[5] - 0.5) * 3), 0.3, 0.0, 0.1]
            if k in (100, 500, 1000):
                frame[4] = float('nan')
            if k % 300 == 299:
                frame[(k // 300) % 3] = float('nan')
        else:
            # Clean window: the 300-tick cooldown drains, then the mailbox walks
            # R1 -> R2 -> R3 every 150 ticks (a 110-tick band is unreachable).
            if k % 1000 == 350 and c.state.permanent_latched:
                assert c.acknowledge_fault_latch() == p.acknowledge_fault_latch() == 1
            frame = [float(max(k % 1000 - 650, 0) // 150 % 3 + 1), 0.95, 0.005, 0.1,
                     float((z[4] - 0.5) * 0.4), float((z[5] - 0.5) * 0.4), 0.3, 0.0, 0.1]
        frame = [float(np.float32(v)) for v in frame] + [0]
        a = c.step(*frame)
        b = p.step(*frame)
        # The cumulative latch must not freeze a whole window: re-arm both together.
        if c.state.permanent_latched:
            latched += 1
            if latched >= 20:
                assert c.acknowledge_fault_latch() == p.acknowledge_fault_latch() == 1
                latched = 0
        else:
            latched = 0
        seen[a[2]] += 1
        assert a[1:3] == b[1:3] and a[4:6] == b[4:6], (k, a, b)
        # The normal-path slew contract is strict: |u(k)-u(k-1)| must not
        # exceed the binary32 budget even by half an ULP.  GATE_SAFE_ZERO is the
        # only documented exception (its single-ULP floor is checked separately).
        if a[1] != GATE_SAFE_ZERO:
            assert abs(a[0] - prev_u) <= slew_limit, (k, a[0], prev_u, slew_limit)
        prev_u = a[0]
        maxerr = max(maxerr, abs(a[0] - b[0]))
    assert maxerr == 0.0, maxerr
    assert all(seen[1:5] >= 300), seen
    maximum = 4294967295
    for threshold, dwell, expected in ((0,0,2), (1,0,2), (150,maximum,2),
                                      (maximum,maximum-2,1), (maximum,maximum-1,2), (maximum,maximum,2)):
        q = dict(cfg, min_dwell_ticks=threshold)
        c = Core(q); p = PythonGatekeeper(q, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
        c.state.dwell_ticks = p.dwell_ticks = dwell
        a, b = c.step(2,1,0,0,0,0,0,0,0,0), p.step(2,1,0,0,0,0,0,0,0,0)
        assert a[2] == b[2] == expected and c.state.dwell_ticks == p.dwell_ticks
    sat_cfg = dict(cfg, max_fault_vetoes_before_latch=0)
    c = Core(sat_cfg); p = PythonGatekeeper(sat_cfg, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    counters = ('dwell_ticks', 'step_count', 'safety_veto_count', 'ood_veto_count',
                'low_confidence_veto_count', 'latency_hold_count', 'corrupted_frame_count')
    for name in counters:
        setattr(c.state, name, maximum-1); setattr(p, name, maximum-1)
    clean = (1.,1.,0.,0.,0.,0.,0.,0.,0.,0)
    fault = (math.nan,1.,cfg['outer_timeout_s']+.5*cfg['dt_s'],2.,1.1,0.,0.,0.,0.,0)
    for frame in (clean, clean, fault, clean, fault, clean, fault):
        frame = tuple(float(np.float32(v)) for v in frame[:-1]) + (int(frame[-1]),)
        a, b = c.step(*frame), p.step(*frame)
        assert a[1:3] == b[1:3] and a[4:6] == b[4:6]
        assert all(getattr(c.state,name) == getattr(p,name) for name in counters)
    # The fault frame is veto-masked (envelope outranks the latency HOLD), so it
    # no longer counts a native-HOLD entry: latency_hold_count stays at MAX-1.
    assert c.state.latency_hold_count == p.latency_hold_count == maximum - 1
    saturated = [name for name in counters[1:] if name != 'latency_hold_count']
    assert all(getattr(c.state,name) == maximum for name in saturated) and a[4] == 31
    for _ in range(2):
        a, b = c.step(1,1,0,0,math.nan,0,0,0,0,0), p.step(1,1,0,0,math.nan,0,0,0,0,0)
        assert a[1] == b[1] == GATE_HOLD_LATENCY
    assert c.state.step_count == p.step_count == c.state.corrupted_frame_count == p.corrupted_frame_count == maximum
    # latency_hold_count counts native-HOLD entry edges; cover its saturation
    # with a pure stale mailbox that no veto masks.
    c = Core(sat_cfg); p = PythonGatekeeper(sat_cfg, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    c.state.latency_hold_count = p.latency_hold_count = maximum - 1
    stale = (1., .95, cfg['outer_timeout_s'] + .5*cfg['dt_s'], 0., 0., 0., 0., 0., 0., 0)
    a, b = c.step(*stale), p.step(*stale)
    assert a[1] == b[1] == GATE_HOLD_LATENCY
    assert c.state.latency_hold_count == p.latency_hold_count == maximum
    assert c.state.hold_ticks == p.hold_ticks == 1
    # One exact HOLD-entry event per episode, on both timeout patterns.
    c = Core(sat_cfg); p = PythonGatekeeper(sat_cfg, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    jump = (1., .95, cfg['outer_timeout_s'] + 10*cfg['dt_s'], 0., 0., 0., 0., 0., 0., 0)
    a, b = c.step(*jump), p.step(*jump)
    assert a[1] == b[1] == GATE_HOLD_LATENCY
    assert c.state.latency_hold_count == p.latency_hold_count == 1
    assert c.state.hold_ticks == p.hold_ticks == 1
    c = Core(sat_cfg); p = PythonGatekeeper(sat_cfg, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    constant = (1., .95, cfg['outer_timeout_s'] + .5*cfg['dt_s'], 0., 0., 0., 0., 0., 0., 0)
    for _ in range(5):
        a, b = c.step(*constant), p.step(*constant)
        assert a[1] == b[1] == GATE_HOLD_LATENCY
        assert c.state.latency_hold_count == p.latency_hold_count == 1
    assert c.state.hold_ticks == p.hold_ticks == 5
    for bad in (float('nan'), float('inf'), -float('inf')):
        q = dict(CFG)
        q['kp'] = [bad, *CFG['kp'][1:]]
        a = Core(q).step(1, 1, 0, 0, 0, 0, 0.1, 0, 0, 0)
        b = PythonGatekeeper(q, dict(gain_scale=1.0, damping_scale=1.0, safety_margin_scale=1.0)).step(1, 1, 0, 0, 0, 0, 0.1, 0, 0, 0)
        assert a[5] == b[5] == 1 and math.isfinite(a[0])
    for field, bad in (('gain_scale', 5.), ('gain_scale', -1.), ('gain_scale', math.nan), ('safety_margin_scale', 1.5)):
        c, nominal = Core(cfg), Core(cfg)
        p = PythonGatekeeper(cfg, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
        x, v, holds, breaches = .2, 0., 0, 0
        for k in range(10000):
            if k in (5000, 9000):
                value = bad if k == 5000 else 1.
                setattr(c.tuning, field, value); p.set_tuning({**p.tune, field: value})
            t = k * DT; r = .2 * math.cos(math.pi*t); rd = -.2 * math.pi * math.sin(math.pi*t)
            ff = PARAM['J']*(-math.pi**2*r) + PARAM['b']*rd + PARAM['k']*r + friction(rd, PARAM)
            frame = [float(np.float32(z)) for z in (1, 1, 0, 0, x, v, r, rd, ff)] + [0]
            a, b = c.step(*frame), p.step(*frame)
            injected = 5000 <= k < 9000
            nominal.step(*([4., 0., *frame[2:]] if injected else frame))
            assert a[1:3] == b[1:3] and a[4:6] == b[4:6]
            assert abs(a[0]-b[0]) < 1e-4 and np.allclose(c.state.integ, nominal.state.integ, atol=1e-6)
            assert np.all(np.isfinite(p.integ)) and np.allclose(c.state.integ, p.integ, atol=1e-5)
            if injected:
                assert a[2] == REGIME_SAFE and a[5] == 1 and a[4] & FAULT_NAN
            holds += int(a[1] == GATE_HOLD_LATENCY); breaches += int(abs(x) > 1.)
            for _ in range(2):
                x, v, _ = advance(x, v, a[0], PARAM)
        assert holds == breaches == 0 and a[2] == REGIME_NOMINAL
        assert c.state.corrupted_frame_count == p.corrupted_frame_count == 4000
    # A2: cumulative latch, cooldown survival, explicit acknowledgement, audit history.
    latched = dict(cfg, max_fault_vetoes_before_latch=3)
    c = Core(latched); p = PythonGatekeeper(latched, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    for _ in range(3):
        for _ in range(5):
            a, b = c.step(1,1,.005,3.,0.,0.,0.,0.,0.,0), p.step(1,1,.005,3.,0.,0.,0.,0.,0.,0)
        for _ in range(5):
            a, b = c.step(1,1,.005,.1,0.,0.,0.,0.,0.,0), p.step(1,1,.005,.1,0.,0.,0.,0.,0.,0)
    assert a[1:3] == b[1:3] == (GATE_LATCHED_SAFE, REGIME_SAFE) and a[4] & FAULT_PERMANENT_LATCH
    for _ in range(latched['ood_recovery_cooldown_ticks'] + 1):
        a, b = c.step(1,1,.005,.1,0.,0.,0.,0.,0.,0), p.step(1,1,.005,.1,0.,0.,0.,0.,0.,0)
    assert a[1] == b[1] == GATE_LATCHED_SAFE and c.state.recovery_ticks == p.recovery_ticks == 0
    history = (c.state.ood_veto_count, c.state.fault_audit_bitmask, c.state.step_count)
    assert c.acknowledge_fault_latch() == p.acknowledge_fault_latch() == 1
    assert (c.state.ood_veto_count, c.state.fault_audit_bitmask, c.state.step_count) == history
    assert c.state.permanent_latched == p.permanent_latched == 0
    for _ in range(latched['ood_recovery_cooldown_ticks'] + 1):
        a, b = c.step(1,1,.005,.1,0.,0.,0.,0.,0.,0), p.step(1,1,.005,.1,0.,0.,0.,0.,0.,0)
    assert a[1:3] == b[1:3] == (GATE_NORMAL, REGIME_NOMINAL)
    # Fix 1 mirror parity: the acknowledge restores the episode quota only.
    assert c.state.latch_episode_count == p.latch_episode_count == 0
    for episode in (1, 2, 3):
        for _ in range(5):
            a, b = c.step(1,1,.005,3.,0.,0.,0.,0.,0.,0), p.step(1,1,.005,3.,0.,0.,0.,0.,0.,0)
        for _ in range(5):
            a, b = c.step(1,1,.005,.1,0.,0.,0.,0.,0.,0), p.step(1,1,.005,.1,0.,0.,0.,0.,0.,0)
        assert a[1:3] == b[1:3] and a[4] == b[4]
        assert c.state.latch_episode_count == p.latch_episode_count == episode
        assert c.state.ood_veto_count == p.ood_veto_count == 3 + episode
        assert c.state.permanent_latched == p.permanent_latched == (1 if episode == 3 else 0)
    assert a[1] == b[1] == GATE_LATCHED_SAFE and a[4] & FAULT_PERMANENT_LATCH
    # A3: local frame lease from outer_seq, including the forged-age and dropout cases.
    c = Core(sat_cfg); p = PythonGatekeeper(sat_cfg, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    for _ in range(400):
        a, b = c.step(1,1,0.,0.,0.,0.,0.,0.,0.,7), p.step(1,1,0.,0.,0.,0.,0.,0.,0.,7)
        assert a[1:3] == b[1:3]
    assert a[1] == b[1] == GATE_HOLD_LATENCY and c.state.outer_age_ticks == p.outer_age_ticks == 399
    a, b = c.step(1,1,0.,0.,0.,0.,0.,0.,0.,8), p.step(1,1,0.,0.,0.,0.,0.,0.,0.,8)
    assert a[1:3] == b[1:3] and c.state.outer_age_ticks == p.outer_age_ticks == 0
    a, b = c.step(1,1,.005,0.,0.,0.,0.,0.,0.,0), p.step(1,1,.005,0.,0.,0.,0.,0.,0.,0)
    assert a[1] == b[1] == GATE_VETO_LOW_CONF and a[2] == b[2] == REGIME_SAFE
    # The sequence fallback is one corrupt frame with the NAN bit.
    assert a[5] == b[5] == 1
    assert a[4] & FAULT_NAN and b[4] & FAULT_NAN
    assert c.state.corrupted_frame_count == p.corrupted_frame_count == 1
    # A5: numerical-domain bounds.  The local lease keeps ageing through
    # alternating corrupt fast frames, an underflowed anti-windup denominator is
    # skipped, an overflowing product stays finite, a saturated lease still times
    # out, a sub-ULP slew budget still de-energises and a corrupt frame followed
    # by a slow-mailbox timeout counts exactly one latency episode.
    c = Core(sat_cfg); p = PythonGatekeeper(sat_cfg, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    a, b = c.step(1, .95, 0., .1, .1, 0., .1, 0., 0., 10), p.step(1, .95, 0., .1, .1, 0., .1, 0., 0., 10)
    assert a[1] == b[1] == GATE_NORMAL
    first_clean_hold = None
    for k in range(1, 401):
        frame = (1., .95, 0., .1, math.nan if k % 2 else .1, 0., .1, 0., 0., 10)
        a, b = c.step(*frame), p.step(*frame)
        assert a[1:3] == b[1:3] and a[4:6] == b[4:6]
        if a[5] == 0 and a[1] == GATE_HOLD_LATENCY and first_clean_hold is None:
            first_clean_hold = k
    assert c.state.outer_age_ticks == p.outer_age_ticks == 400
    assert 348 <= first_clean_hold <= 352, first_clean_hold
    q = dict(sat_cfg, ki=[1e-30, 1e-30, 1e-30, 0.], windup_tau_s=1e-20)
    c = Core(q); p = PythonGatekeeper(q, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    under = (1., .95, .005, 0., 0., 0., .1, 0., 0., 0)
    for _ in range(200):
        a, b = c.step(*under), p.step(*under)
        assert a[1:3] == b[1:3] and a[4:6] == b[4:6]
    assert abs(c.state.integ[0] - .02) < 1e-3 and list(c.state.integ) == p.integ
    q = dict(cfg, kp=[3e38] + cfg['kp'][1:], kd=[0.] + cfg['kd'][1:], ki=[0.] + cfg['ki'][1:])
    c = Core(q, (4., 1., 1.)); p = PythonGatekeeper(q, dict(gain_scale=4., damping_scale=1., safety_margin_scale=1.))
    a, b = c.step(1, .95, .005, 0., 0., 0., .1, 0., 0., 0), p.step(1, .95, .005, 0., 0., 0., .1, 0., 0., 0)
    assert a == b and a[6] == cfg['u_max'] == 20.0, (a, b, cfg['u_max'])
    q = dict(cfg, dt_s=3e38, u_slew_max_per_s=1e-30)  # 1e-30 * 3e38 = 3e8, finite
    c = Core(q); p = PythonGatekeeper(q, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    slow = (1., .95, 0., 0., 0., 0., .1, 0., 0., 5)
    a, b = c.step(*slow), p.step(*slow)
    assert a == b and a[1] == GATE_NORMAL
    a, b = c.step(*slow), p.step(*slow)
    assert a == b and a[1] == GATE_HOLD_LATENCY
    a, b = c.step(*slow), p.step(*slow)
    assert a == b and a[1] == GATE_HOLD_LATENCY
    assert c.state.outer_age_ticks == p.outer_age_ticks == 2
    q = dict(cfg, u_slew_max_per_s=1e-6)
    c = Core(q); p = PythonGatekeeper(q, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    c.state.u_applied = p.u_applied = 1.0
    c.state.u_target = p.u_target = 1.0
    c.state.control_started = p.control_started = 1
    sub_ulp = (1., .95, cfg['outer_timeout_s'] + 10*cfg['dt_s'], 0., .2, 0., .4, 0., 0., 0)
    for _ in range(101):
        a, b = c.step(*sub_ulp), p.step(*sub_ulp)
        assert a[1:3] == b[1:3] and a[4:6] == b[4:6]
    assert a[1] == b[1] == GATE_SAFE_ZERO and a[0] == b[0] and 0.0 < a[0] < 1.0
    previous = a[0]
    for _ in range(200):
        a, b = c.step(*sub_ulp), p.step(*sub_ulp)
        assert a[0] == b[0] and 0.0 < a[0] < previous and a[6] == b[6] == 0.0
        previous = a[0]
    c = Core(sat_cfg); p = PythonGatekeeper(sat_cfg, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    good = (1., .95, .005, .1, .1, 0., .1, 0., 0., 0)
    for _ in range(200):
        a, b = c.step(*good), p.step(*good)
    assert c.state.latency_hold_count == p.latency_hold_count == 0
    a, b = c.step(1., .95, .005, .1, math.nan, 0., .1, 0., 0., 0), p.step(1., .95, .005, .1, math.nan, 0., .1, 0., 0., 0)
    assert a[1] == b[1] == GATE_HOLD_LATENCY and a[5] == b[5] == 1
    assert c.state.hold_ticks == p.hold_ticks == 1
    assert c.state.latency_hold_count == p.latency_hold_count == 0
    mixed = (1., .95, cfg['outer_timeout_s'] + .5*cfg['dt_s'], .1, .1, 0., .1, 0., 0., 0)
    a, b = c.step(*mixed), p.step(*mixed)
    assert a[1] == b[1] == GATE_HOLD_LATENCY and a[5] == b[5] == 0
    assert c.state.hold_ticks == p.hold_ticks == 2
    assert c.state.latency_hold_count == p.latency_hold_count == 1
    a, b = c.step(*mixed), p.step(*mixed)
    assert c.state.latency_hold_count == p.latency_hold_count == 1
    assert c.state.hold_ticks == p.hold_ticks == 3
    # A8: signed envelope look-ahead.  The same velocity magnitude must trip
    # outward and stay clear inward, and the reported x=0.97 / x_dot=-1.14
    # retreat must not charge a latch episode.
    look_cfg = dict(cfg, safety_trip_ratio=0.99, safety_predict_s=0.06)
    look_tuning = dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.)
    c = Core(look_cfg); p = PythonGatekeeper(look_cfg, look_tuning)
    a, b = c.step(1., .95, .005, .1, .88, 2.5, .88, 0., 0., 0), p.step(1., .95, .005, .1, .88, 2.5, .88, 0., 0., 0)
    assert a == b and a[1] == b[1] == GATE_VETO_ENVELOPE
    assert c.state.safety_veto_count == p.safety_veto_count == 1
    c = Core(look_cfg); p = PythonGatekeeper(look_cfg, look_tuning)
    a, b = c.step(1., .95, .005, .1, .88, -2.5, .88, 0., 0., 0), p.step(1., .95, .005, .1, .88, -2.5, .88, 0., 0., 0)
    assert a == b and a[1] == b[1] == GATE_NORMAL
    assert c.state.safety_veto_count == p.safety_veto_count == 0
    c = Core(look_cfg); p = PythonGatekeeper(look_cfg, look_tuning)
    retreat = (1., .95, .005, .1, .97, -1.14, .97, 0., 0., 0)
    for _ in range(50):
        a, b = c.step(*retreat), p.step(*retreat)
        assert a == b
    assert a[1] == b[1] == GATE_NORMAL
    assert c.state.safety_veto_count == p.safety_veto_count == 0
    assert c.state.latch_episode_count == p.latch_episode_count == 0
    print('PASS: A8 signed envelope look-ahead (outward trip, inward retreat, reported 0.97/-1.14 case)')
    # A9: a stale slow mailbox with healthy fast sensors keeps the loop closed
    # on the fixed R4 bank during the whole grace, then de-energises.
    lat_tuning = dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.)
    c = Core(cfg); p = PythonGatekeeper(cfg, lat_tuning)
    healthy = (1., .95, .005, .1, .2, .02, .4, 0., 0., 0)
    for _ in range(200):
        a, b = c.step(*healthy), p.step(*healthy)
        assert a == b
    stale = healthy[:2] + (10.0,) + healthy[3:]
    integ_before = c.state.integ[3]
    previous_u = a[0]
    for _ in range(100):
        a, b = c.step(*stale), p.step(*stale)
        assert a == b
        assert a[1] == b[1] == GATE_HOLD_LATENCY and a[2] == b[2] == REGIME_SAFE
        assert a[3] == b[3] == 1.0
    assert a[4] & FAULT_LATENCY and not (a[4] & FAULT_NAN)
    assert c.state.hold_ticks == p.hold_ticks == 100
    assert c.state.latency_hold_count == p.latency_hold_count == 1
    assert c.state.integ[3] == p.integ[3] != integ_before
    assert a[0] == b[0] != previous_u
    a, b = c.step(*stale), p.step(*stale)
    assert a == b and a[1] == b[1] == GATE_SAFE_ZERO and a[6] == b[6] == 0.0
    assert c.state.permanent_latched == p.permanent_latched == 0
    fresh = (1., .95, .005, .1, .2, .02, .4, 0., 0., 0)
    for _ in range(int(cfg['ood_recovery_cooldown_ticks']) + 2):
        a, b = c.step(*fresh), p.step(*fresh)
        assert a == b
    assert a[1] == b[1] == GATE_NORMAL and a[2] == b[2] == REGIME_NOMINAL
    print('PASS: A9 stale mailbox runs closed-loop R4 for the grace, then SAFE_ZERO and re-arm')
    # A1: bounded HOLD -> de-energised zero at the exact hold-budget boundary.
    c = Core(sat_cfg); p = PythonGatekeeper(sat_cfg, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    for _ in range(101):
        a, b = c.step(1,1,.005,.1,math.nan,0.,.1,0.,0.,0), p.step(1,1,.005,.1,math.nan,0.,.1,0.,0.,0)
        assert a[1:3] == b[1:3]
    assert a[1] == b[1] == GATE_SAFE_ZERO and a[0] == b[0] == 0.0
    assert list(c.state.integ) == p.integ == [0.0, 0.0, 0.0, 0.0]
    assert a[4] & FAULT_HOLD_TIMEOUT and c.state.permanent_latched == p.permanent_latched == 1
    # Fix 2 mirror parity: a corrupt configuration holds for 100 ticks then stops.
    held_frame = (1., 1., .005, .1, .2, 0., .4, 0., 0., 0)
    c = Core(latched)
    p = PythonGatekeeper(latched, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    for _ in range(200):
        a, b = c.step(*held_frame), p.step(*held_frame)
    assert a[1:3] == b[1:3] == (GATE_NORMAL, REGIME_NOMINAL) and a[0] > 0.0
    c.cfg.kp[0] = math.nan  # the configuration is re-read on every call
    p.cfg['kp'][0] = math.nan
    for _ in range(100):
        a, b = c.step(*held_frame), p.step(*held_frame)
        assert a[1:3] == b[1:3] == (GATE_HOLD_LATENCY, REGIME_NOMINAL)
    assert c.state.hold_ticks == p.hold_ticks == 100
    a, b = c.step(*held_frame), p.step(*held_frame)
    assert a[1:3] == b[1:3] == (GATE_SAFE_ZERO, REGIME_SAFE)
    assert a[0] == b[0] == 0.0 and a[4] == b[4]
    assert c.state.permanent_latched == p.permanent_latched == 1
    assert c.state.hold_ticks == p.hold_ticks == 101
    assert a[4] & (FAULT_HOLD_TIMEOUT | FAULT_PERMANENT_LATCH)
    # Fix 2 mirror parity: UINT32_MAX can never time out and is rejected.
    saturated = dict(latched, max_hold_ticks=4294967295)
    c = Core(saturated)
    p = PythonGatekeeper(saturated, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    a, b = c.step(*held_frame), p.step(*held_frame)
    assert a[5] == b[5] == 1 and a[1] == b[1] == GATE_HOLD_LATENCY
    assert c.state.hold_ticks == p.hold_ticks == 1
    # A stale heartbeat with healthy fast sensors keeps the loop closed on the
    # fixed R4 bank for the whole grace, then de-energises under the slew limit.
    c = Core(sat_cfg)
    p = PythonGatekeeper(sat_cfg, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    for _ in range(200):
        a, b = c.step(*held_frame), p.step(*held_frame)
    stale_frame = held_frame[:2] + (10.0,) + held_frame[3:]
    for _ in range(100):
        a, b = c.step(*stale_frame), p.step(*stale_frame)
        assert a == b and a[1:3] == (GATE_HOLD_LATENCY, REGIME_SAFE)
        assert a[2] == b[2] == REGIME_SAFE
    assert c.state.latency_hold_count == p.latency_hold_count == 1
    prev_u_c, prev_u_p = a[0], b[0]
    a, b = c.step(*stale_frame), p.step(*stale_frame)
    du_max = sat_cfg['u_slew_max_per_s'] * sat_cfg['dt_s']
    assert a == b and a[1:3] == (GATE_SAFE_ZERO, REGIME_SAFE)
    assert a[6] == b[6] == 0.0
    assert abs(a[0] - prev_u_c) <= du_max + 1e-6
    assert abs(b[0] - prev_u_p) <= du_max + 1e-6
    for _ in range(20):
        if a[0] == 0.0:
            break
        a, b = c.step(*stale_frame), p.step(*stale_frame)
        assert a[1] == b[1] == GATE_SAFE_ZERO
        assert a[0] == b[0]
    assert a[0] == b[0] == 0.0
    assert c.state.permanent_latched == p.permanent_latched == 0
    # The binary32 SAFE_ZERO ramp: 2000.0f * 0.001f is exactly 2.0f, so a 7.0
    # command steps 7 -> 5 -> 3 -> 1 -> 0 across the HOLD budget; both trigger
    # paths must agree bit for bit with the C core.
    ramp_cfg = dict(cfg, blend_alpha=1.0, kp=[0.0] * 4, kd=[0.0] * 4, ki=[0.0] * 4,
                    ff_scale=[1.0, 1.0, 1.0, 0.0])
    ramp_expected = [7.0, 5.0, 3.0, 1.0, 0.0, 0.0]
    def ramp_tail(backend, frame):
        for _ in range(4):
            backend.step(1., 1., 0., 0., 0., 0., 0., 0., 7., 0)
        return [backend.step(*frame)[0] for _ in range(105)]
    ramp_tuning = dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.)
    nan_frame = (1., 1., 0., 0., np.nan, 0., 0., 0., 7., 0)
    # The stale-age variant uses a 0.25 s watchdog so the constant 0.26 s
    # transported age is genuinely past the lease from the first fast tick.
    stale_cfg = dict(ramp_cfg, outer_timeout_s=0.25)
    stale_frame = (1., 1., 0.26, 0., 0., 0., 0., 0., 7., 0)
    c_nan = ramp_tail(Core(ramp_cfg), nan_frame)
    p_nan = ramp_tail(PythonGatekeeper(ramp_cfg, ramp_tuning), nan_frame)
    assert c_nan[99:105] == p_nan[99:105] == ramp_expected, (c_nan[99:105], p_nan[99:105])
    assert c_nan == p_nan
    # The stale-mailbox path has healthy feedback, so the fallback bank closes
    # the loop and the same 2.0-per-tick slew limiter walks 7 -> 5 -> 3 -> 1 ->
    # 0 during the grace; the budget boundary then reports the de-energised zero.
    c_stale = ramp_tail(Core(stale_cfg), stale_frame)
    p_stale = ramp_tail(PythonGatekeeper(stale_cfg, ramp_tuning), stale_frame)
    assert c_stale[:6] == p_stale[:6] == [5.0, 3.0, 1.0, 0.0, 0.0, 0.0], (c_stale[:6], p_stale[:6])
    assert c_stale[99:105] == p_stale[99:105] == [0.0] * 6
    assert c_stale == p_stale
    print('PASS: 1 kHz SAFE_ZERO ramp 7->5->3->1->0 on the corrupt-sensor path and the '
          'closed-loop fallback ramp on the stale-mailbox path')
    # F1 reachable case: a held command of 1.5 with a 0.1 binary32 budget.  The
    # rounded bound 1.5f - 0.1f is 1.3999999762f, a 0.100000024 step that
    # exceeds the budget, so the de-energise must pull the bound one ULP back
    # toward the held command and step to 1.4000000953674316, bit-exact on both
    # engines.  A corrupt fast sensor is used so the hold is exact.
    inward_cfg = dict(cfg, dt_s=float(np.float32(0.001)),
                      u_slew_max_per_s=float(np.float32(100.0)), max_hold_ticks=1,
                      u_max=float(np.float32(2.0)), blend_alpha=1.0,
                      kp=[1.0, 0.0, 0.0, 0.0], kd=[0.0] * 4, ki=[0.0] * 4,
                      ff_scale=[0.0] * 4)
    inward_tuning = dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.)
    c = Core(inward_cfg)
    p = PythonGatekeeper(inward_cfg, inward_tuning)
    live = (1., .95, .005, 0., 0., 0., 1.5, 0., 0., 0)
    for _ in range(100):
        a, b = c.step(*live), p.step(*live)
        assert a == b
    assert a[0] == b[0] == 1.5
    dead_sensor = (1., .95, .005, 0., np.nan, 0., 1.5, 0., 0., 0)
    a, b = c.step(*dead_sensor), p.step(*dead_sensor)
    assert a == b and a[1] == b[1] == GATE_HOLD_LATENCY and a[0] == b[0] == 1.5
    a, b = c.step(*dead_sensor), p.step(*dead_sensor)
    inward_du = float(np.float32(100.0 * 0.001))
    assert a == b and a[1] == b[1] == GATE_SAFE_ZERO
    assert a[0] == b[0] == 1.4000000953674316, (a[0], b[0])
    assert abs(a[0] - 1.5) <= inward_du
    print('PASS: F1 SAFE_ZERO inward rounding 1.5 -> 1.4000000953674316 within the 0.1 budget')
    # Normal-path slew-bound regression: the two bounds are rounded inward.  With
    # a binary32 budget of 7.0e-8 between 0.5 ULP and 1 ULP of 1.0f, the rounded
    # upper bound 1.0000001192f would allow a 1.1920929e-7 step; the limiter must
    # hold 1.0f and report a zero step, and the Python mirror must agree.
    fine_cfg = dict(cfg, u_slew_max_per_s=7e-5, blend_alpha=1.0,
                    kp=[1.0, 0.0, 0.0, 0.0], kd=[0.0] * 4, ki=[0.0] * 4,
                    ff_scale=[0.0] * 4)
    fine_tuning = dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.)
    fine_du = float(np.float32(fine_cfg['u_slew_max_per_s'] * fine_cfg['dt_s']))
    assert float(np.float32(1.0 + fine_du)) - 1.0 > fine_du
    c = Core(fine_cfg); p = PythonGatekeeper(fine_cfg, fine_tuning)
    up_frame = (1., 1., 0., 0., 0., 0., 2., 0., 0., 0)
    for _ in range(20):
        a, b = c.step(*up_frame), p.step(*up_frame)
    c.state.u_applied = p.u_applied = 1.0
    a, b = c.step(*up_frame), p.step(*up_frame)
    assert a == b and a[1] == b[1] == GATE_NORMAL and a[0] == b[0] == 1.0, (a, b)
    assert abs(a[0] - 1.0) <= fine_du
    down_frame = (1., 1., 0., 0., 0., 0., -2., 0., 0., 0)
    c.state.u_applied = p.u_applied = -1.0
    c.state.u_target = p.u_target = -2.0
    a, b = c.step(*down_frame), p.step(*down_frame)
    assert a == b and a[1] == b[1] == GATE_NORMAL and a[0] == b[0] == -1.0, (a, b)
    print('PASS: normal-path slew bound rounded inward at the 1.0f / 7.0e-8 boundary')
    trials = []
    def evaluate(knobs):
        trials.append(knobs)
        return (knobs[0]-1.25)**2 + (knobs[1]-.5)**2, knobs[:2] != (1.25, 1.)
    assert auto_field_tune(evaluate, (.9,.8,.7)) == (1.25, .5, .7)
    assert trials == [(.75,1.,.7), (1.,1.,.7), (1.25,1.,.7), (2.,1.,.7), (1.25,.5,.7), (1.25,.75,.7)]
    assert auto_field_tune(lambda _: (math.nan, False), (.9,.8,.7)) == (.9,.8,.7)
    assert auto_field_tune(lambda _: (math.inf, True), (.9,.8,.7)) == (.9,.8,.7)
    # F2 boundary: the local lease age is (float)ticks * dt in the C core, so a
    # lease count of 2^24+1 must round to 16777216 before the multiply.  Both
    # engines therefore read 16777.216796875 s and stay below a timeout equal to
    # that value; a double-precision product would read 16777.21875 s and trip.
    lease_cfg = dict(cfg, outer_timeout_s=16777.216796875)
    c = Core(lease_cfg)
    p = PythonGatekeeper(lease_cfg, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    c.state.last_outer_seq = p.last_outer_seq = 7
    c.state.outer_age_ticks = p.outer_age_ticks = 16777216
    a, b = c.step(1, .95, 0., 0., 0., 0., 0., 0., 0., 7), p.step(1, .95, 0., 0., 0., 0., 0., 0., 0., 7)
    assert a == b and a[1] == GATE_NORMAL and a[5] == 0, (a, b)
    assert c.state.outer_age_ticks == p.outer_age_ticks == 16777217
    print('PASS: F2 lease boundary 2^24+1 ticks rounds the uint32 count to binary32 first')
    # F3 boundary: a legal UINT32_MAX seed shifts by +11/+37 in the context, so
    # the offset must wrap modulo 2^32 before the uint32 conversion, exactly as
    # the MATLAB mirror does.
    assert lcg(4294967295 + 11, 1)[0] == 0.23994349723216146
    assert lcg(4294967295 + 37, 1)[0] == 0.25001986033748835
    print('PASS: F3 LCG seed 4294967295 wraps modulo 2^32 at both context offsets')
    # The ctypes boundary carries binary32 only: a raw float64 residual a hair
    # above the trip threshold must round to the same frame on both backends.
    c0 = Core(cfg)
    p0 = PythonGatekeeper(cfg, dict(gain_scale=1., damping_scale=1., safety_margin_scale=1.))
    a0 = c0.step(1, 1, 0, 1.00000001, 0, 0, 0, 0, 0, 1)
    b0 = p0.step(1, 1, 0, 1.00000001, 0, 0, 0, 0, 0, 1)
    assert a0 == b0, (a0, b0)
    for bad in (2**32, -1, 1.5):
        for obj in (c0, p0):
            try:
                obj.step(1, 1, 0, 0, 0, 0, 0, 0, 0, bad)
            except ValueError:
                pass
            else:
                raise AssertionError(f'outer_seq {bad!r} was not rejected')
    print('PASS: outer_seq rejects non-uint32 values before ctypes/mirror conversion')
    print(f'PASS: 6000 frames, R1={seen[1]} R2={seen[2]} R3={seen[3]} R4={seen[4]}; '
          f'gate/regime/fault exact; max|du|={maxerr:.3g}')
    print('PASS: uint32 counter saturation, dwell boundary semantics and ten-field step contract')
    print('PASS: four mid-run tuning faults; zero HOLD/breaches, finite integrators, six-candidate search')
    print('PASS: A1 bounded HOLD / A2 latch+ack+episode quota / A3 outer_seq lease parity vs C99 core')
    print('PASS: config_bad budget, UINT32_MAX rejection and slew-limited SAFE_ZERO agree with the C core')
    print('PASS: A5 numerical-domain bounds (lease ageing, underflow, overflow, sub-ULP, mixed HOLD)')
    # Wall-position generalisation: the same commissioned classifier, probe and
    # controller must hold the contact budget when the unilateral surface moves.
    # Only the wall position and the reference amplitude shift together (same
    # commanded 0.07 rad penetration); no controller or probe parameter changes.
    clf = commission()
    wall_peaks = []
    for wall in (0.30, 0.43, 0.55):
        c = context(seed=7, scenario=2, duration=10.0, ood=False, wallx=wall, amplitude=wall + 0.07)
        m = metrics(c, simulate(c, 'dynamic', clf=clf))['S3']
        assert m['breach'] == 0 and m['peak_N'] < 20.0 and m['r3_rate'] > 0.90, (wall, m)
        wall_peaks.append(m['peak_N'])
    print(f'PASS: wall-position generalisation 0.30 / 0.43 / 0.55 rad (peaks {wall_peaks[0]:.2f} / '
          f'{wall_peaks[1]:.2f} / {wall_peaks[2]:.2f} N, R3 > 0.9, zero breaches)')
    # A commanded smooth retreat is the same physical unload as a reference step:
    # the directional rule must release it inside the 0.35 s contract without a
    # re-latch (the retired 250 ms step blanking used to be the only guard).
    ret = retreat_context()
    q1 = ret['t'] >= 10.0
    tau = np.clip((ret['t'][q1] - 10.0) / 0.6, 0.0, 1.0)
    p5 = 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5
    dp5 = 30.0 * tau ** 2 - 60.0 * tau ** 3 + 30.0 * tau ** 4
    ret['r'][q1], ret['rd'][q1], ret['rdd'][q1] = 0.5 * (1.0 - p5), -0.5 * dp5 / 0.6, 0.0
    assert float(np.max(np.abs(np.diff(ret['r'][q1])))) < 0.003
    a = simulate(ret, 'dynamic', clf=clf)['log']
    r3 = np.flatnonzero((ret['stage'] == 0) & (a[:, 3] == REGIME_BOUNDARY))
    smooth_release = float(ret['t'][r3[-1]] - 10.0) if len(r3) else 0.0
    assert smooth_release < 0.35 and float(np.mean(a[ret['t'] >= 25.0, 3] == REGIME_BOUNDARY)) == 0.0, smooth_release
    print(f'PASS: smooth quintic retreat releases in {smooth_release:.3f} s (< 0.35 s), no re-latch')
    # Off-manifold audit sanity: the same conformal detector must stay quiet on
    # independent NOMINAL runs while still detecting the coupled load bias.
    audit_trip, audit_clear, _ = off_manifold_threshold(clf)
    audit_clf = (clf[0], clf[1], audit_trip, audit_clear)
    nom = nominal_control(clf, audit_clf)
    nom_ticks = max(r['low_confidence_ticks'] for r in nom)
    assert nom_ticks == 0, nom
    fx = context(seed=0, duration=6.0, **OFF_MANIFOLD)
    zb = simulate(fx, 'dynamic', clf=audit_clf, skip_first_frame=True,
                  conf_persist=CONF_PERSIST_FRAMES)
    bias_ticks = int(np.sum(zb['log'][:, 2] == GATE_VETO_LOW_CONF))
    assert bias_ticks > 0, bias_ticks
    print(f'PASS: off-manifold nominal control ({nom_ticks} false-veto ticks) vs '
          f'coupled load bias ({bias_ticks} low-confidence ticks)')

def physics_checks():
    for drive in (0.5, 1.01):
        assert (advance(0.0, 0.0, drive, PARAM)[2] == 1) == (drive <= PARAM['fs'])
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
        ax.set_ylabel(label); ax.grid(alpha=0.2); ax.legend(loc='upper right', ncol=3)
        for edge in (10, 20, 30):
            ax.axvline(edge, color='gray', lw=0.6)
    axes[-1].set_xlabel('Time [s]'); fig.tight_layout()
    fig.savefig(Path(__file__).with_name('benchmark_4stages.png'), dpi=240)
    plt.close(fig)

def _ablation_summary(c, z):
    """Stage metrics plus the supervision-loss evidence used by the ablation rows."""
    m = metrics(c, z)
    return dict(m, summary=dict(safety_breaches=sum(v['breach'] for v in m.values()),
                tv=round(sum(v['tv'] for v in m.values()), 1), s4_rmse=m['S4']['rmse'],
                s4_latched=m['S4']['latched'], s4_peak_u=m['S4']['peak_u'],
                max_abs_state=round(float(np.max(np.abs(z['log'][:, 0]))), 3)))

def main():
    global USE_PYTHON
    if '-h' in sys.argv or '--help' in sys.argv:
        print("usage: run_benchmark.py [--selftest] [--json] [--no-c]")
        print("  --selftest  core/parity self-checks without the full benchmark")
        print("  --json      print the full report as JSON instead of tables/plots")
        print("  --no-c      use the Python core mirror instead of the C99 build")
        return 0
    USE_PYTHON = '--no-c' in sys.argv
    if '--selftest' in sys.argv:
        selftest()
        return 0
    start = time.perf_counter()
    physics_checks()
    clf = commission()
    policy = train_policy()
    c = context()
    # Main table: the two baseline families are graded twice - once as the
    # plain controller and once with the SAME 96 B C99 core mounted as an
    # independent safety shell - so the comparison reports what the runtime
    # assurance shell buys and what it does not.
    branches = (('B1', 'b1'), ('B2', 'b2'), ('B3', 'dynamic'),
                ('B1+C99 Gate', 'b1g'), ('B2+C99 Gate', 'b2g'))
    results = {name: simulate(c, mode, policy=policy, clf=clf) for name, mode in branches}
    ablation = {f'R{bank}': metrics(c, simulate(c, 'fixed', bank=bank)) for bank in range(1, 5)}
    ablation['dynamic'] = metrics(c, results['B3'])
    ablation['rules'] = metrics(c, simulate(c, 'rules', clf=clf))
    # "Wild dual loop": slow bank selection + multi-bank PID, no 1 kHz supervision.
    ablation['unguarded_dual'] = _ablation_summary(c, simulate(c, 'dynamic', clf=clf, ood_gate=False, conf_gate=False, supervised=False))
    # Envelope-only dual loop: envelope + latch are the only supervision left.
    ablation['envelope_only_dual'] = _ablation_summary(c, simulate(c, 'dynamic', clf=clf, ood_gate=False, conf_gate=False))
    tune = tuning_study(clf)
    eff = efficiency_study()
    report = dict(version='2.0.2', metrics={k: metrics(c, v) for k, v in results.items()}, ablation=ablation, tuning=tune, efficiency=eff, classifier=[clf[0].tolist(), *map(float, clf[1:])], calls=results['B2']['calls'], resources=resource_study(policy), audit=audit_study(clf,c))
    m = report['metrics']
    # Functional and physical-safety assertions only.  The comparative outcome
    # of every branch is measured and reported, never asserted: no threshold
    # here may encode "B3 must beat Bx", so the tables cannot be pre-shaped.
    assert results['B2']['calls'] == 40000
    assert all(m['B3'][stage]['breach'] == 0 for stage in m['B3'])
    assert m['B3']['S3']['peak_N'] < 20
    assert m['B3']['S4']['latched'] == 0
    for row in tune:
        dynamic_trials = row['candidates']['dynamic']
        valid = [trial['knobs'] for trial in dynamic_trials if trial['valid']]
        if valid:
            assert row['knobs'] in valid
            assert row['search_cost_after'] == min(trial['cost'] for trial in dynamic_trials if trial['valid'])
        else:
            assert row['knobs'] == [1.0, 1.0, 1.0]
        assert all(np.isfinite(v) for stage in row['mission'].values() for v in stage.values())
    assert all(row['mission'][stage]['breach'] == 0 for row in tune for stage in ('S1', 'S2', 'S3', 'S4'))
    if '--json' in sys.argv:
        print(json.dumps(report))
        return 0
    print_table(c, results)
    print('\nMode ablation')
    for name, row in ablation.items():
        print(name, json.dumps(row))
    print('\nFrozen-AI field tuning')
    for row in tune:
        drop = 100.0 * (1.0 - row['after']['rmse'] / row['before']['rmse'])
        print(json.dumps(row))
        print(f"  {row['label']}: before_rmse={row['before']['rmse']:.6f} -> after_rmse={row['after']['rmse']:.6f} "
              f"({drop:.1f}% change); breaches before/after={row['before']['breach']}/{row['after']['breach']}; "
              f"b1_gated_valid_candidates={row['b1_valid_candidates']} b1_gated_feasible={row['b1_feasible']}")
    print('\nOff-manifold abstention (manifold classifier / single-dimension rule tree / no-confidence ablation)')
    off = report['audit']['off_manifold_abstention']
    print(f"  fixture={off['fixture']} conformal trip={off['conformal_trip']} clear={off['conformal_clear']} "
          f"calibration frames={off['calibration_frames']} persistence={off['confidence_persistence_frames']} "
          f"band={off['band']}")
    print('| Seed | Branch | In-box | B3 detect | Rules retreat | RMSE [rad] | Low-conf ticks | R4 ticks | Max |x| | Breach |')
    print('|---|---|---:|---:|---:|---:|---:|---:|---:|---:|')
    for row in off['seeds']:
        for name in ('b3', 'no_conf', 'rules'):
            z = row[name]
            print(f"| {row['seed']} | {name} | {row['in_box_fraction']:.3f} | {row['b3_detection_frames']:.3f} | "
                  f"{row['rules_retreat_rate']:.3f} | {z['rmse']:.6f} | {z['low_confidence_ticks']} | "
                  f"{z['r4_ticks']} | {z['max_abs_state']:.3f} | {z['breach']} |")
    print('  nominal control (no load bias, same audit detector): '
          + ', '.join(f"seed {r['seed']}: {r['low_confidence_ticks']} low-conf / {r['r4_ticks']} R4 ticks"
                      for r in off['nominal_control']))
    rv = report['audit']['reverse_s3_to_s1']
    rt = report['audit']['retreat_s3_to_s1']
    print(f"  reverse S3->S1 (wall withdrawn): peak contact={rv['s3_peak_N']} N, S1 r3_rate={rv['s1_r3_rate']}, release={rv['release_s']} s")
    print(f"  retreat S3->interior (wall kept): S1 r3_rate={rt['s1_r3_rate']}, release={rt['release_s']} s, last5 r3={rt['last5_r3_rate']}, rmse={rt['s1_rmse']}")
    print('\nSample efficiency (action labels)')
    for row in eff:
        print(json.dumps(row))
    print('\nContact, smoothing and slow-mailbox controls')
    print(json.dumps(report['audit']))
    print('\nResources (host timing includes language bindings)')
    print(json.dumps(report['resources']))
    plot_benchmark(c, results)
    print(f'All engineering checks passed. Wall time: {time.perf_counter() - start:.2f} s')
    return 0
if __name__ == '__main__':
    raise SystemExit(main())
