function P = load_domain_profile(name)
%LOAD_DOMAIN_PROFILE  Engineering presets for the three isomorphic domains.
%
%   P = load_domain_profile()                  % 'industrial'
%   P = load_domain_profile('industrial')      % WAAM wire-feed / servo axis
%   P = load_domain_profile('edge_vla')        % 10 Hz VLA governing a 1 kHz arm
%   P = load_domain_profile('neuromodulation') % aDBS / SCS closed-loop therapy
%
%   All three presets share ONE second-order plant model with non-smooth
%   Stribeck friction:
%
%       J*xdd + b*xd + k*x + F_friction(xd) + d_ood(t) = u
%
%   and differ only in physical units, inertia / impedance, actuator
%   authority, safety envelope, sensor noise and disturbance severity.  The
%   benchmark engine (sim_core_engine.m) and the efficiency study
%   (run_efficiency_analysis.m) are therefore completely domain agnostic: they
%   read everything from this struct.
%
%   The returned struct is intentionally flat and documented so that a field
%   engineer can diff two units/patients without reading MATLAB.
%
%   See also SIM_CORE_ENGINE, RUN_EFFICIENCY_ANALYSIS, RUN_ALL_BENCHMARKS.

if nargin < 1 || isempty(name)
    name = 'industrial';
end
if ~(ischar(name) || (isstring(name) && isscalar(name)))
    error('load_domain_profile:badInput', ...
        'Profile name must be a character vector or a string scalar.');
end
key = lower(strtrim(char(name)));

switch key
    case {'industrial', 'waam', 'servo', 'industrial_servo'}
        P = preset_industrial();
    case {'edge_vla', 'vla', 'robot', 'robotics', 'arm'}
        P = preset_edge_vla();
    case {'neuromodulation', 'neuro', 'adbs', 'scs'}
        P = preset_neuromodulation();
    otherwise
        error('load_domain_profile:unknownProfile', ...
            ['Unknown domain profile "%s".  Available profiles: ' ...
             'industrial | edge_vla | neuromodulation'], key);
end

P = finalize_profile(P);
end

% =========================================================================
%  PRESET 1 -- Industrial servo / Wire-Arc Additive Manufacturing axis
% =========================================================================
function P = preset_industrial()
P.kind        = 'industrial';
P.name        = 'Industrial servo / WAAM wire-feed axis';
P.summary     = ['Rotary servo joint driving a WAAM torch carriage. Thermal ' ...
                 'growth and wire-feed load shift the effective inertia by ' ...
                 '+35% between machines; Stribeck friction at reversal is ' ...
                 'the dominant non-smooth boundary.'];
P.units       = struct('state', 'rad', 'rate', 'rad/s', 'input', 'N*m', ...
                       'inertia', 'kg*m^2', 'increment', 'rad');

% --- sampling / benchmark timeline --------------------------------------
P.dt          = 1e-3;          % 1 kHz deterministic inner loop
P.stages.edges = [0 10 20 30 40];
P.stages.names = { ...
    'Stage 1 - nominal distribution', ...
    'Stage 2 - cross-unit +35% drift and non-smooth boundary', ...
    'Stage 3 - edge compute scarcity (10 Hz, 60-120 ms jitter)', ...
    'Stage 4 - unseen OOD structural resonance and sensor outliers'};
P.stages.short = {'S1 nominal', 'S2 drift', 'S3 latency', 'S4 OOD'};

% --- reference trajectory ------------------------------------------------
% Point-to-point (quintic S-curve) moves with dwells: dwell segments are where
% stiction and the non-smooth friction boundary actually matter.
P.ref.mode    = 'trapezoid';   % 'trapezoid' | 'harmonic'
P.ref.amp     = 0.50;          % rad, peak commanded position
P.ref.dwell_s = 1.50;          % s, hold at each end point
P.ref.move_s  = 1.50;          % s, quintic transition between end points
P.ref.f1_hz   = 0.12;          % Hz, dominant harmonic (harmonic mode)
P.ref.f2_hz   = 0.31;          % Hz, secondary harmonic (harmonic mode)
P.ref.weights = [0.68 0.32];   % harmonic weights (sum to 1)
P.ref.phi     = [0.00 0.90];   % rad, phase of each harmonic

% --- plant ---------------------------------------------------------------
P.plant.J     = 0.010;         % kg*m^2, rotor + torch carriage inertia
P.plant.b     = 0.060;         % N*m*s/rad, viscous damping
P.plant.k     = 0.300;         % N*m/rad, cable / gearbox compliance
P.plant.fc    = 0.200;         % N*m, Coulomb friction
P.plant.fs    = 0.450;         % N*m, static (break-away) friction
P.plant.vs    = 0.020;         % rad/s, Stribeck velocity
P.plant.fvisc = 0.050;         % N*m*s/rad, viscous friction term

% --- actuator ------------------------------------------------------------
P.act.u_max   = 2.0;           % N*m, hard torque limit
P.act.slew_max = 200.0;        % N*m/s, hard slew-rate limit

% --- safety envelope -----------------------------------------------------
P.safety.x_limit   = 1.0;      % rad, hard motion envelope
P.safety.trip_ratio = 0.90;    % veto declared above 90% of the envelope
P.safety.clear_ratio = 0.75;   % veto cleared below 75% of the envelope
P.safety.predict_s = 0.06;     % s, look-ahead used with |x_dot|

% --- sensing -------------------------------------------------------------
P.sensor.sigma    = 0.0025;    % rad, position noise 1-sigma
P.sensor.deriv_tau_s = 0.012;  % s, derivative low-pass time constant

% --- nominal closed-loop tuning -----------------------------------------
P.ctrl.fn_hz     = 3.0;        % Hz, target closed-loop bandwidth
P.ctrl.zeta       = 0.80;      % target closed-loop damping ratio
P.ctrl.int_ratio  = 6.0;       % Ki = Kp*wn/int_ratio
P.ctrl.bank_rel   = [1.00 1.00 1.00; ...   % regime 1: nominal
                     1.40 1.80 5.00; ...   % regime 2: high-load damped
                     0.90 1.50 1.30; ...   % regime 3: boundary compliant
                     1.00 3.00 0.00];      % regime 4: safe fallback (PD only)
P.b1.rel          = [0.55 0.60 0.35];      % Branch 1: conservative fixed PID
P.b2.rel          = [0.80 1.00 0.00];      % Branch 2: overfitted continuous AI

% --- slow coarse AI loop -------------------------------------------------
P.ai.outer_hz          = 10;       % Hz, coarse classifier rate
P.ai.outer_timeout_s   = 0.30;     % s, heartbeat watchdog trip
P.ai.max_residual      = 1.0;      % normalized fast-residual veto threshold
P.ai.residual_clear_ratio = 0.80;  % hysteresis to clear the OOD veto
P.ai.min_confidence    = 0.45;     % confidence veto threshold
P.ai.confidence_clear  = 0.55;     % hysteresis to clear the confidence veto
P.ai.window_tau_s      = 0.25;     % s, EMA window for error/effort features
P.ai.window_tau_v_s    = 0.50;     % s, slower EMA for the velocity feature
P.ai.lp_tau_s          = 0.15;     % s, slow (drift) residual channel
P.ai.res_fast_scale    = 0.050;    % normalization of the fast residual channel
P.ai.res_slow_scale    = 0.050;    % normalization of the slow residual channel
P.ai.feature_scales    = [P.ref.amp, P.act.u_max, 1.0, ...
                          P.ref.amp*2*pi*P.ref.f1_hz*2.0];
P.ai.regime_names      = {'REGIME_NOMINAL', 'REGIME_HIGH_LOAD_DAMPED', ...
                          'REGIME_BOUNDARY_COMPLIANT', 'REGIME_SAFE_FALLBACK'};
% Class means of the coarse classifier, in normalized feature units
% [rms(error)/amp, rms(u)/u_max, slow residual, rms(xd)/v_scale].
% Recovered from the 40-sample coarse calibration grid (see
% run_efficiency_analysis.m Panel A for the sample-complexity study).
P.ai.prototypes        = [0.012 0.096 0.002 0.650; ...
                          0.018 0.150 0.200 0.650; ...
                          0.045 0.200 0.120 0.650];

% --- gatekeeper ----------------------------------------------------------
P.gate.blend_alpha = 0.10;     % per-step bumpless transfer coefficient (1 kHz)
P.gate.integ_ratio = 0.50;     % Ki*integ_limit = integ_ratio*u_max

% --- nominal observer used by the physics residual probe -----------------
P.observer.wob_hz  = 4.0;      % Hz, observer bandwidth (slow on purpose: it
                              % acts as a model-based band-split for the two
                              % residual channels, it is not the controller)
P.observer.zeta    = 0.80;

% --- Stage 2 disturbance: cross-unit drift + sharper boundary ------------
P.drift.t_start = 10.0;        % s
P.drift.ramp_s  = 1.0;         % s, smooth redeployment ramp
P.drift.J       = 1.35;        % +35% effective inertia / impedance
P.drift.b       = 1.25;
P.drift.k       = 1.10;
P.drift.fc      = 2.00;
P.drift.fs      = 2.40;        % break-away friction 2.4x higher
P.drift.vs      = 0.25;        % Stribeck velocity 4x sharper
P.drift.fvisc   = 1.20;

% --- Stage 3 disturbance: edge compute scarcity --------------------------
P.edge.t_start   = 20.0;       % s
P.edge.base_latency_s = 0.060; % s, minimum inference latency
P.edge.jitter_s       = 0.060; % s, uniform additional jitter (60-120 ms)
P.edge.drop_prob      = 0.15;  % frame drop probability at 10 Hz

% --- Stage 4 disturbance: unseen OOD fault -------------------------------
P.ood.t_start     = 30.0;      % s
P.ood.res_hz      = 42.0;      % Hz, unmodelled parasitic structural mode
P.ood.res_zeta    = 0.03;      % modal damping
P.ood.res_cu      = 12.0;      % mode excitation from actuator command
P.ood.res_cx      = 8.0;       % mode excitation from motion
P.ood.res_kq      = 0.004;     % mode -> plant force coupling
P.ood.spike_first_s = 0.002;   % s after onset, deterministic first outlier
P.ood.spike_prob  = 0.004;     % outliers per 1 ms sample (4 per second)
P.ood.spike_amp   = 0.20;      % rad, outlier amplitude
P.ood.spike_tau_s = 0.001;     % s, outlier sensor impulse width

% --- metrics -------------------------------------------------------------
P.metrics.chatter_window_s = 0.02;
P.metrics.success_factor   = 1.50;  % closed-loop success = RMSE < 1.5x ideal
end

% =========================================================================
%  PRESET 2 -- Edge embodied AI / 10 Hz VLA on a 1 kHz compliant arm
% =========================================================================
function P = preset_edge_vla()
P.kind    = 'edge_vla';
P.name    = 'Edge VLA (10 Hz) governing a 1 kHz compliant joint';
P.summary = ['A vision-language-action model at 10 Hz overrides a compliant ' ...
             'robot joint whose torque loop closes at 1 kHz. Payload changes ' ...
             'shift the link inertia by +35%; contact transition and the ' ...
             'harmonic drive friction are the non-smooth boundaries.'];
P.units   = struct('state', 'rad', 'rate', 'rad/s', 'input', 'N*m', ...
                   'inertia', 'kg*m^2', 'increment', 'rad');

P.dt           = 1e-3;
P.stages.edges = [0 10 20 30 40];
P.stages.names = { ...
    'Stage 1 - nominal pick-and-place distribution', ...
    'Stage 2 - unknown payload (+35% inertia) and contact boundary', ...
    'Stage 3 - on-robot edge NPU: 10 Hz, 60-120 ms jitter', ...
    'Stage 4 - unseen structural resonance and sensor faults'};
P.stages.short = {'S1 nominal', 'S2 drift', 'S3 latency', 'S4 OOD'};

P.ref.mode    = 'trapezoid';
P.ref.amp     = 0.60;
P.ref.dwell_s = 1.20;
P.ref.move_s  = 1.20;
P.ref.f1_hz   = 0.15;
P.ref.f2_hz   = 0.37;
P.ref.weights = [0.66 0.34];
P.ref.phi     = [0.00 0.70];

P.plant.J     = 0.120;         % kg*m^2, link + payload
P.plant.b     = 0.350;
P.plant.k     = 0.800;         % compliant joint
P.plant.fc    = 3.00;          % N*m, harmonic-drive Coulomb friction
P.plant.fs    = 8.00;          % N*m, break-away friction
P.plant.vs    = 0.050;
P.plant.fvisc = 0.500;

P.act.u_max   = 12.0;
P.act.slew_max = 1200.0;

P.safety.x_limit    = 1.0;
P.safety.trip_ratio = 0.90;
P.safety.clear_ratio = 0.75;
P.safety.predict_s  = 0.06;

P.sensor.sigma       = 0.0040;
P.sensor.deriv_tau_s = 0.006;

P.ctrl.fn_hz    = 4.0;
P.ctrl.zeta     = 0.85;
P.ctrl.int_ratio = 6.0;
P.ctrl.bank_rel = [1.00 1.00 1.00; ...
                   1.40 1.80 5.00; ...
                   0.90 1.50 1.30; ...
                   1.00 3.00 0.00];
P.b1.rel = [0.55 0.60 0.35];
P.b2.rel = [0.80 1.00 0.00];

P.ai.outer_hz        = 10;
P.ai.outer_timeout_s = 0.30;
P.ai.max_residual    = 1.0;
P.ai.residual_clear_ratio = 0.80;
P.ai.min_confidence  = 0.45;
P.ai.confidence_clear = 0.55;
P.ai.window_tau_s    = 0.25;
P.ai.window_tau_v_s  = 0.50;
P.ai.lp_tau_s        = 0.15;
P.ai.res_fast_scale  = 0.060;
P.ai.res_slow_scale  = 0.060;
P.ai.feature_scales  = [P.ref.amp, P.act.u_max, 1.0, ...
                        P.ref.amp*2*pi*P.ref.f1_hz*2.0];
P.ai.regime_names    = {'REGIME_NOMINAL', 'REGIME_HIGH_LOAD_DAMPED', ...
                        'REGIME_BOUNDARY_COMPLIANT', 'REGIME_SAFE_FALLBACK'};
% Class means of the coarse classifier are set per domain in FINALIZE_PROFILE
% from the measured feature clusters of each preset (kept out of the preset
% body so the numbers cannot drift away from the code that produces them).
P.ai.prototypes      = zeros(3,4);

P.gate.blend_alpha = 0.10;
P.gate.integ_ratio = 0.50;

P.observer.wob_hz = 5.0;
P.observer.zeta   = 0.80;

P.drift.t_start = 10.0;
P.drift.ramp_s  = 1.0;
P.drift.J       = 1.35;
P.drift.b       = 1.20;
P.drift.k       = 1.15;
P.drift.fc      = 2.00;
P.drift.fs      = 2.40;
P.drift.vs      = 0.25;
P.drift.fvisc   = 1.20;

P.edge.t_start         = 20.0;
P.edge.base_latency_s  = 0.060;
P.edge.jitter_s        = 0.060;
P.edge.drop_prob       = 0.15;

P.ood.t_start     = 30.0;
P.ood.res_hz      = 38.0;
P.ood.res_zeta    = 0.03;
P.ood.res_cu      = 10.0;
P.ood.res_cx      = 7.0;
P.ood.res_kq      = 0.004;
P.ood.spike_first_s = 0.002;
P.ood.spike_prob  = 0.004;
P.ood.spike_amp   = 0.22;
P.ood.spike_tau_s = 0.001;

P.metrics.chatter_window_s = 0.02;
P.metrics.success_factor   = 1.50;
end

% =========================================================================
%  PRESET 3 -- Closed-loop neuromodulation (aDBS / SCS)
% =========================================================================
function P = preset_neuromodulation()
P.kind    = 'neuromodulation';
P.name    = 'Closed-loop neuromodulation (aDBS / SCS)';
P.summary = ['Wearable/implantable adaptive deep-brain or spinal-cord ' ...
             'stimulator. The 10 Hz biomarker classifier decides the ' ...
             'stimulation regime; a 1 kHz deterministic core shapes the ' ...
             'stimulation envelope inside strict clinical safety bounds. ' ...
             'Electrode impedance drifts +35% between patients.'];
P.units   = struct('state', 'norm.', 'rate', '1/s', 'input', 'norm.', ...
                   'inertia', 'norm.', 'increment', 'norm.');

P.dt           = 1e-3;
P.stages.edges = [0 10 20 30 40];
P.stages.names = { ...
    'Stage 1 - nominal patient responder', ...
    'Stage 2 - +35% impedance drift and scar-tissue boundary', ...
    'Stage 3 - sub-mW edge compute: 10 Hz, 60-120 ms jitter', ...
    'Stage 4 - unseen artefact / lead res                          -onance'};
P.stages.short = {'S1 nominal', 'S2 drift', 'S3 latency', 'S4 OOD'};

P.ref.mode    = 'trapezoid';
P.ref.amp     = 0.40;
P.ref.dwell_s = 1.50;
P.ref.move_s  = 1.50;
P.ref.f1_hz   = 0.08;
P.ref.f2_hz   = 0.21;
P.ref.weights = [0.70 0.30];
P.ref.phi     = [0.00 1.10];

P.plant.J     = 0.800;         % effective biomarker dynamics
P.plant.b     = 0.900;
P.plant.k     = 0.150;
P.plant.fc    = 1.20;
P.plant.fs    = 3.20;
P.plant.vs    = 0.040;
P.plant.fvisc = 0.200;

P.act.u_max   = 12.0;          % normalized stimulation authority
P.act.slew_max = 600.0;

P.safety.x_limit    = 1.0;
P.safety.trip_ratio = 0.90;
P.safety.clear_ratio = 0.75;
P.safety.predict_s  = 0.06;

P.sensor.sigma       = 0.0030;
P.sensor.deriv_tau_s = 0.010;

P.ctrl.fn_hz    = 1.2;         % deliberately low bandwidth: 1-minute budget
P.ctrl.zeta     = 0.90;
P.ctrl.int_ratio = 7.0;
P.ctrl.bank_rel = [1.00 1.00 1.00; ...
                   1.40 1.80 5.00; ...
                   0.90 1.50 1.30; ...
                   1.00 3.00 0.00];
P.b1.rel = [0.55 0.60 0.35];
P.b2.rel = [0.60 1.00 0.00];

P.ai.outer_hz        = 10;
P.ai.outer_timeout_s = 0.30;
P.ai.max_residual    = 1.0;
P.ai.residual_clear_ratio = 0.80;
P.ai.min_confidence  = 0.45;
P.ai.confidence_clear = 0.55;
P.ai.window_tau_s    = 0.25;
P.ai.window_tau_v_s  = 0.50;
P.ai.lp_tau_s        = 0.15;
P.ai.res_fast_scale  = 0.050;
P.ai.res_slow_scale  = 0.050;
P.ai.feature_scales  = [P.ref.amp, P.act.u_max, 1.0, ...
                        P.ref.amp*2*pi*P.ref.f1_hz*2.0];
P.ai.regime_names    = {'REGIME_NOMINAL', 'REGIME_HIGH_LOAD_DAMPED', ...
                        'REGIME_BOUNDARY_COMPLIANT', 'REGIME_SAFE_FALLBACK'};
% Class means of the coarse classifier are set per domain in FINALIZE_PROFILE
% from the measured feature clusters of each preset (kept out of the preset
% body so the numbers cannot drift away from the code that produces them).
P.ai.prototypes      = zeros(3,4);

P.gate.blend_alpha = 0.10;
P.gate.integ_ratio = 0.50;

P.observer.wob_hz = 1.5;
P.observer.zeta   = 0.80;

P.drift.t_start = 10.0;
P.drift.ramp_s  = 1.0;
P.drift.J       = 1.35;
P.drift.b       = 1.20;
P.drift.k       = 1.25;
P.drift.fc      = 2.60;
P.drift.fs      = 4.00;
P.drift.vs      = 0.12;
P.drift.fvisc   = 1.20;

P.edge.t_start        = 20.0;
P.edge.base_latency_s = 0.060;
P.edge.jitter_s       = 0.060;
P.edge.drop_prob      = 0.15;

P.ood.t_start     = 30.0;
P.ood.res_hz      = 33.0;
P.ood.res_zeta    = 0.03;
P.ood.res_cu      = 8.0;
P.ood.res_cx      = 6.0;
P.ood.res_kq      = 0.020;
P.ood.spike_first_s = 0.002;
P.ood.spike_prob  = 0.004;
P.ood.spike_amp   = 0.40;
P.ood.spike_tau_s = 0.001;

P.metrics.chatter_window_s = 0.02;
P.metrics.success_factor   = 1.50;
end

% =========================================================================
%  Shared derivation of gains, feature scales, ceilings
% =========================================================================
function P = finalize_profile(P)

% ---- benchmark timeline -------------------------------------------------
P.stage.edges = P.stages.edges;
P.stage.T     = P.stage.edges(end);
P.stage.names = P.stages.names;
P.stage.short = P.stages.short;
P.stage.count = numel(P.stage.edges) - 1;

% ---- reference trajectory coefficients ----------------------------------
P.ref.w  = 2*pi*[P.ref.f1_hz, P.ref.f2_hz];
P.ref.a  = P.ref.weights(:)' * P.ref.amp;
P.ref.phi = P.ref.phi(:)';
if ~isfield(P.ref, 'mode') || isempty(P.ref.mode)
    P.ref.mode = 'trapezoid';
end
if strcmpi(P.ref.mode, 'trapezoid')
    P.ref.period = 2*(P.ref.dwell_s + P.ref.move_s);
    P.ref.v_peak = 1.875*P.ref.amp/max(P.ref.move_s, eps);
    P.ref.v_scale = P.ref.v_peak;
else
    P.ref.period = 1/max(P.ref.f1_hz, eps);
    P.ref.v_scale = P.ref.amp*2*pi*P.ref.f1_hz*2.0;
end

% ---- nominal controller gains ------------------------------------------
wn = 2*pi*P.ctrl.fn_hz;
P.ctrl.wn = wn;
P.ctrl.Kp = P.plant.J*wn^2;
P.ctrl.Kd = max(2*P.ctrl.zeta*P.plant.J*wn - P.plant.b, 0.15*P.plant.J*wn);
P.ctrl.Ki = P.ctrl.Kp*wn/P.ctrl.int_ratio;

% ---- controller bank (4 regimes x {Kp,Kd,Ki}) ---------------------------
rel = P.ctrl.bank_rel;
P.bank.Kp = P.ctrl.Kp*rel(:,1);
P.bank.Kd = P.ctrl.Kd*rel(:,2);
P.bank.Ki = P.ctrl.Ki*rel(:,3);
P.bank.names = P.ai.regime_names;

% ---- Branch 1: conservative fixed PID (no adaptation) -------------------
P.b1.Kp = P.ctrl.Kp*P.b1.rel(1);
P.b1.Kd = P.ctrl.Kd*P.b1.rel(2);
P.b1.Ki = P.ctrl.Ki*P.b1.rel(3);

% ---- Branch 2: end-to-end continuous AI policy --------------------------
P.b2.Kp = P.ctrl.Kp*P.b2.rel(1);
P.b2.Kd = P.ctrl.Kd*P.b2.rel(2);
P.b2.Ki = P.ctrl.Ki*P.b2.rel(3);
% The policy learned the inverse dynamics of the NOMINAL unit and froze them.
P.b2.model = struct('J', P.plant.J, 'b', P.plant.b, 'k', P.plant.k, ...
                    'fc', P.plant.fc, 'fs', P.plant.fs, ...
                    'vs', P.plant.vs, 'fvisc', P.plant.fvisc);

% ---- Branch 3: deterministic (white-box) feedforward --------------------
% The 1 kHz core runs the nominal inverse-dynamics feedforward from the
% commissioning model.  It is part of the deterministic core -- the AI never
% produces u(t) -- and it is exactly the term whose mismatch the coarse regime
% classifier + integral action have to absorb after a machine/patient swap.
P.b3.ff_weight = 1.0;
P.b3.model = P.b2.model;

% ---- gatekeeper derived constants --------------------------------------
% Anti-chatter minimum dwell time between normal regimes (150 ms at 1 kHz).
P.gate.min_dwell_s = 0.150;
% Post-fault recovery cooldown: consecutive clean samples before the AI is
% re-armed (500 ms at 1 kHz).
P.gate.ood_recovery_cooldown_s = 0.500;
% Tracking anti-windup time constant of every bank controller.
P.gate.windup_tau_s = 0.050;
P.gate.integ_limit = P.gate.integ_ratio*P.act.u_max/max(P.ctrl.Ki, eps);
P.gate.max_residual = P.ai.max_residual;
P.gate.residual_clear_ratio = P.ai.residual_clear_ratio;
P.gate.min_confidence = P.ai.min_confidence;
P.gate.confidence_clear = P.ai.confidence_clear;
P.gate.x_limit = P.safety.x_limit;
P.gate.trip_ratio = P.safety.trip_ratio;
P.gate.clear_ratio = P.safety.clear_ratio;
P.gate.predict_s = P.safety.predict_s;
P.gate.u_max = P.act.u_max;
P.gate.slew_max = P.act.slew_max;
P.gate.dt = P.dt;

% ---- observer gains -----------------------------------------------------
wob = 2*pi*P.observer.wob_hz;
P.observer.L1 = 2*P.observer.zeta*wob;
P.observer.L2 = wob^2;
P.observer.wn = wob;

% ---- coarse-classifier class means (per domain) -------------------------
% Frozen deployment table of the 10 Hz coarse classifier: class means in the
% normalized feature space [rms(e)/amp, rms(u)/u_max, slow residual,
% rms(xd)/v_scale].  run_efficiency_analysis.m quantifies how many
% calibration runs are needed to recover such means.
switch P.kind
    case 'industrial'
        P.ai.prototypes = [0.012 0.096 0.002 0.650; ...
                           0.018 0.150 0.200 0.650; ...
                           0.045 0.200 0.120 0.650];
    case 'edge_vla'
        P.ai.prototypes = [0.012 0.271 0.003 1.130; ...
                           0.018 0.398 0.148 1.143; ...
                           0.035 0.330 0.040 1.138];
    case 'neuromodulation'
        P.ai.prototypes = [0.009 0.139 0.002 1.020; ...
                           0.029 0.234 0.220 1.024; ...
                           0.060 0.250 0.060 1.020];
    otherwise
        error('load_domain_profile:unknownKind', ...
            'No coarse-classifier calibration table for domain "%s".', P.kind);
end

% ---- slow-loop feature normalization -----------------------------------
P.ai.outer_dt = 1/P.ai.outer_hz;
P.ai.n_prototypes = size(P.ai.prototypes, 1);
P.ai.feature_scales = P.ai.feature_scales(:)';
P.ai.feature_scales(4) = P.ref.v_scale;

% ---- metric bookkeeping -------------------------------------------------
P.metrics.dt = P.dt;
P.metrics.stage_edges = P.stage.edges;

% ---- profile catalogue (used by run_all_benchmarks) --------------------
P.profiles = {'industrial', 'edge_vla', 'neuromodulation'};
end
