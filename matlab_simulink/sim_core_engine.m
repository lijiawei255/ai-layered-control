function out = sim_core_engine(profile, opts)
%SIM_CORE_ENGINE  Four-stage closed-loop benchmark engine for the layered
%   AI control reference architecture.
%
%   OUT = SIM_CORE_ENGINE()            % industrial profile, full 40 s run
%   OUT = SIM_CORE_ENGINE('edge_vla')
%   OUT = SIM_CORE_ENGINE(P)           % pass a struct from LOAD_DOMAIN_PROFILE
%   OUT = SIM_CORE_ENGINE(P, OPTS)
%
%   Three branches are simulated side by side on the SAME plant, the SAME
%   reference trajectory and the SAME fault schedule so the comparison is
%   apples to apples:
%
%     Branch 1 - Fixed Baseline        conservative 1 kHz PID, no adaptation.
%     Branch 2 - End-to-End AI         high-frequency continuous policy that
%                                      learned the inverse dynamics of the
%                                      NOMINAL unit; inference is throttled to
%                                      10 Hz with jitter/drops in Stage 3-4.
%     Branch 3 - Layered (proposed)    10 Hz coarse regime classifier +
%                                      physics residual probe + 1 kHz C99
%                                      gatekeeper / controller bank.
%
%   Stage timeline (default 40 s, dt = 1 ms):
%     Stage 1  0-10 s  nominal distribution
%     Stage 2 10-20 s  cross-unit +35% drift and non-smooth Stribeck boundary
%     Stage 3 20-30 s  edge compute scarcity: 10 Hz, 60-120 ms jitter, drops
%     Stage 4 30-40 s  unseen OOD structural resonance + sensor outliers
%
%   OPTS fields (all optional):
%     .t_end          simulation horizon in seconds   (default P.stage.T)
%     .branches       1xN branches to run             (default [1 2 3])
%     .seed           RNG seed for noise/faults       (default 7)
%     .drift_scale    override drift magnitude, e.g. 0.35 for +35 %
%     .drift_t_start  override drift onset time
%     .edge_on        enable Stage 3 edge constraints (default true)
%     .ood_on         enable Stage 4 OOD fault        (default true)
%     .verbose        print a short stage summary     (default false)
%
%   The gatekeeper implemented in LOCAL_CORE_STEP is a 1:1 MATLAB mirror of
%   c99_layered_core/layered_gatekeeper.c (same decision order, dwell timer,
%   post-fault cooldown, per-controller anti-windup, NaN firewall and fault
%   bitmask) so the figure-producing benchmark and the deployable C core can
%   be reviewed against each other line by line.
%
%   See also LOAD_DOMAIN_PROFILE, RUN_EFFICIENCY_ANALYSIS, RUN_ALL_BENCHMARKS.

% ------------------------------------------------------------------------
% 0. Input handling
% ------------------------------------------------------------------------
if nargin < 1 || isempty(profile)
    profile = 'industrial';
end
if nargin < 2 || isempty(opts)
    opts = struct();
end
if ischar(profile) || (isstring(profile) && isscalar(profile))
    P = load_domain_profile(char(profile));
else
    P = profile;
end
opts = local_defaults(opts, P);

dt = P.dt;
N  = round(opts.t_end/dt);
t  = (0:N-1)'*dt;

% Reference trajectory (analytic derivatives).
[r, rd, rdd] = local_ref(P, t);

% Stage index of every sample.
stage = local_stage_index(t, P.stage.edges);

% ------------------------------------------------------------------------
% 1. Disturbance schedule (identical for all three branches)
% ------------------------------------------------------------------------
plant = local_plant_arrays(P, opts, t);

% Sensor noise and outliers: same realisation for every branch.  Outliers
% belong to the Stage-4 OOD fault only.
rng(opts.seed + 11, 'twister');
noise = P.sensor.sigma*randn(N,1);
spike = zeros(N,1);
if opts.ood_on
    i_onset = min(N, max(1, round(P.ood.t_start/dt) + 1));
    first   = min(N, i_onset + round(P.ood.spike_first_s/dt));
    if first >= 1 && first <= N
        spike(first) = P.ood.spike_amp;                       % deterministic
    end
    mask = false(N,1);
    mask(i_onset:end) = true;
    rnd  = rand(N,1);
    idx  = find(rnd < P.ood.spike_prob & mask);
    if ~isempty(idx)
        sgn = 2*(rand(numel(idx),1) > 0.5) - 1;
        spike(idx) = spike(idx) + P.ood.spike_amp*sgn;
    end
end

% Edge-compute frame schedule: identical for Branch 2 inference and for the
% Branch 3 coarse classifier.
sched = local_frame_schedule(P, opts, dt, N);

% OOD parasitic mode parameters.
ood = struct('t_start', P.ood.t_start, 'wr', 2*pi*P.ood.res_hz, ...
             'zr', P.ood.res_zeta, 'cu', P.ood.res_cu, ...
             'cx', P.ood.res_cx, 'kq', P.ood.res_kq, 'on', opts.ood_on);

% Branch 3 core configuration (mirror of the C99 config struct).
cfg = local_core_config(P);

% ------------------------------------------------------------------------
% 2. Run the requested branches
% ------------------------------------------------------------------------
out = struct();
out.opts     = opts;
out.profile  = P;
out.t        = t;
out.stage    = stage;
out.ref      = struct('r', r, 'rd', rd, 'rddot', rdd);
out.schedule = sched;
out.b1 = [];
out.b2 = [];
out.b3 = [];
if any(opts.branches == 1)
    out.b1 = local_run_fixed(P, t, r, rd, noise, spike, plant, ood, r(1));
end
if any(opts.branches == 2)
    out.b2 = local_run_e2e(P, t, r, rd, rdd, noise, spike, plant, ood, ...
                           sched, opts, r(1));
end
if any(opts.branches == 3)
    out.b3 = local_run_layered(P, t, r, rd, rdd, noise, spike, plant, ood, ...
                               sched, opts, cfg, r(1));
end

% ------------------------------------------------------------------------
% 3. Engineering metrics
% ------------------------------------------------------------------------
out.metrics = local_metrics(P, out, opts);
out.metrics.assertions = local_assertions(P, out);

if opts.verbose
    local_print_summary(P, out);
end
end

% =========================================================================
%  OPTIONS / CONFIG
% =========================================================================
function opts = local_defaults(opts, P)
if ~isfield(opts, 't_end') || isempty(opts.t_end)
    opts.t_end = P.stage.T;
end
if ~isfield(opts, 'branches') || isempty(opts.branches)
    opts.branches = [1 2 3];
end
if ~isfield(opts, 'seed') || isempty(opts.seed)
    opts.seed = 7;
end
if ~isfield(opts, 'drift_scale')
    opts.drift_scale = [];
end
if ~isfield(opts, 'drift_t_start') || isempty(opts.drift_t_start)
    opts.drift_t_start = P.drift.t_start;
end
if ~isfield(opts, 'edge_on') || isempty(opts.edge_on)
    opts.edge_on = true;
end
if ~isfield(opts, 'ood_on') || isempty(opts.ood_on)
    opts.ood_on = true;
end
if ~isfield(opts, 'verbose') || isempty(opts.verbose)
    opts.verbose = false;
end
if ~isfield(opts, 'fixed_regime') || isempty(opts.fixed_regime)
    opts.fixed_regime = NaN;      % tuning aid: force a constant regime
end
% Trim the benchmark horizon if a caller asked for a shorter run.
opts.t_end = min(opts.t_end, P.stage.T);
end

function cfg = local_core_config(P)
%LOCAL_CORE_CONFIG  Mirror of c99_layered_core LayeredConfig_t + gains.
cfg = struct();
cfg.dt            = P.dt;
cfg.timeout_s     = P.ai.outer_timeout_s;
cfg.max_residual  = P.ai.max_residual;
cfg.res_clear     = P.ai.residual_clear_ratio;
cfg.min_confidence= P.ai.min_confidence;
cfg.conf_clear    = P.ai.confidence_clear;
cfg.x_limit       = P.safety.x_limit;
cfg.trip_ratio    = P.safety.trip_ratio;
cfg.clear_ratio   = P.safety.clear_ratio;
cfg.predict_s     = P.safety.predict_s;
cfg.u_max         = P.act.u_max;
cfg.slew_max      = P.act.slew_max;
cfg.blend_alpha   = P.gate.blend_alpha;
cfg.integ_limit   = P.gate.integ_limit;
cfg.windup_tau_s  = P.gate.windup_tau_s;
cfg.min_dwell     = max(1, round(P.gate.min_dwell_s/P.dt));
cfg.recovery      = max(0, round(P.gate.ood_recovery_cooldown_s/P.dt));
cfg.kp            = P.bank.Kp(:);
cfg.kd            = P.bank.Kd(:);
cfg.ki            = P.bank.Ki(:);
cfg.gain_scale    = 1.0;
cfg.damping_scale = 1.0;
cfg.safety_margin_scale = 1.0;
end

% =========================================================================
%  SIGNALS
% =========================================================================
function [r, rd, rdd] = local_ref(P, t)
r = zeros(size(t)); rd = zeros(size(t)); rdd = zeros(size(t));
if strcmpi(P.ref.mode, 'trapezoid')
    % Quintic point-to-point moves separated by dwells.  The dwells are the
    % interesting part for this benchmark: at zero velocity the stiction
    % boundary dominates and a controller without integral action parks on a
    % steady-state error.
    A  = P.ref.amp; dw = P.ref.dwell_s; mv = P.ref.move_s;
    per = 2*(dw + mv);
    s = mod(t, per);
    m1 = s < dw;                          % dwell at +A
    r(m1) = A;
    m2 = ~m1 & (s < dw + mv);             % +A -> -A
    tau = (s(m2) - dw)/mv;
    [p, v, a] = local_quintic(tau);
    r(m2)   = A - 2*A*p;
    rd(m2)  = -2*A*v/mv;
    rdd(m2) = -2*A*a/mv^2;
    m3 = ~m1 & ~m2 & (s < 2*dw + mv);     % dwell at -A
    r(m3) = -A;
    m4 = ~(m1 | m2 | m3);                 % -A -> +A
    tau = (s(m4) - (2*dw + mv))/mv;
    [p, v, a] = local_quintic(tau);
    r(m4)   = -A + 2*A*p;
    rd(m4)  = 2*A*v/mv;
    rdd(m4) = 2*A*a/mv^2;
else
    for i = 1:numel(P.ref.w)
        arg = P.ref.w(i)*t + P.ref.phi(i);
        s = sin(arg); c = cos(arg);
        r   = r   + P.ref.a(i)*s;
        rd  = rd  + P.ref.a(i)*P.ref.w(i)*c;
        rdd = rdd - P.ref.a(i)*P.ref.w(i)^2*s;
    end
end
end

function [p, v, a] = local_quintic(tau)
% 5th-order polynomial: zero velocity and acceleration at both ends.
p = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
v = 30*tau.^2 - 60*tau.^3 + 30*tau.^4;
a = 60*tau - 180*tau.^2 + 120*tau.^3;
end

function stage = local_stage_index(t, edges)
stage = zeros(size(t));
for s = 1:numel(edges)-1
    stage(t >= edges(s) & t < edges(s+1)) = s;
end
stage(t >= edges(end)) = numel(edges) - 1;
end

function F = local_friction(v, fc, fs, vs, fvisc)
% Non-smooth Stribeck friction, evaluated with a very sharp smooth sign so
% the model stays continuous but behaves like a real stiction boundary.
veps = max(vs*0.02, 1e-9);
sg   = tanh(v/veps);
F    = fvisc*v + fc*sg + (fs - fc)*exp(-abs(v)/vs)*sg;
end

function plant = local_plant_arrays(P, opts, t)
% Local uncalibrated drift: the machine/patient is swapped at t_start and the
% parameters settle to the drifted values over ramp_s seconds.
sc = zeros(size(t));
if P.drift.ramp_s <= 0
    sc(t >= opts.drift_t_start) = 1;
else
    f = (t - opts.drift_t_start)/P.drift.ramp_s;
    f = min(max(f, 0), 1);
    sc = 0.5 - 0.5*cos(pi*f);      % smooth cosine redeployment ramp
end
if isempty(opts.drift_scale)
    amp = 1.0;
else
    amp = opts.drift_scale/max(P.drift.J - 1, 1e-6);
end
dr = @(factor) 1 + (factor - 1)*amp.*sc;
plant.J    = P.plant.J*dr(P.drift.J);
plant.b    = P.plant.b*dr(P.drift.b);
plant.k    = P.plant.k*dr(P.drift.k);
plant.fc   = P.plant.fc*dr(P.drift.fc);
plant.fs   = P.plant.fs*dr(P.drift.fs);
plant.vs   = P.plant.vs*max(1 + (P.drift.vs - 1)*amp*sc, 0.05);
plant.fvisc= P.plant.fvisc*dr(P.drift.fvisc);
end

function sched = local_frame_schedule(P, opts, dt, N)
% 10 Hz slow-loop / edge-inference frame schedule.
n_frame = floor(opts.t_end/P.ai.outer_dt) + 1;
t_emit  = (0:n_frame-1)'*P.ai.outer_dt;
rng(opts.seed + 37, 'twister');
lat  = 0.020*ones(n_frame,1);
drop = false(n_frame,1);
if opts.edge_on
    after = t_emit >= P.edge.t_start;
    n_after = nnz(after);
    if n_after > 0
        lat(after)  = P.edge.base_latency_s + ...
                      P.edge.jitter_s*rand(n_after,1);
        drop(after) = rand(n_after,1) < P.edge.drop_prob;
    end
end
emit_idx = min(N, max(1, round(t_emit/dt) + 1));
arr_idx  = min(N, max(1, round((t_emit + lat)/dt) + 1));
arr_idx(drop) = 0;

% index look-up tables: sample -> frame id (0 = none)
emit_map = zeros(N,1);
arr_map  = zeros(N,1);
for m = 1:n_frame
    if emit_idx(m) >= 1 && emit_idx(m) <= N
        if emit_map(emit_idx(m)) == 0
            emit_map(emit_idx(m)) = m;
        end
    end
    if arr_idx(m) >= 1 && arr_idx(m) <= N
        if arr_map(arr_idx(m)) == 0
            arr_map(arr_idx(m)) = m;
        end
    end
end
sched = struct('t_emit', t_emit, 'latency', lat, 'dropped', drop, ...
               'emit_idx', emit_idx, 'arr_idx', arr_idx, ...
               'emit_map', emit_map, 'arr_map', arr_map, ...
               'n_frame', n_frame);
end

% =========================================================================
%  BRANCH 1 -- conservative fixed PID (no adaptation, no fault detection)
% =========================================================================
function B = local_run_fixed(P, t, r, rd, noise, spike, plant, ood, x0)
N  = numel(t); dt = P.dt;
x = x0; xd = 0; q = 0; qd = 0;
u_app = 0; y_prev = 0; yf = x0; ydf = 0; integ = 0;

Kp = P.b1.Kp; Kd = P.b1.Kd; Ki = P.b1.Ki;
u_max = P.act.u_max; dslew = P.act.slew_max*dt;
tau_d = P.sensor.deriv_tau_s;
wind  = P.gate.windup_tau_s; ilim = P.gate.integ_limit;

X = zeros(N,1); XD = zeros(N,1); U = zeros(N,1); Y = zeros(N,1);
for k = 1:N
    y = x + noise(k) + spike(k);

    % filtered derivative ("dirty derivative"): unity DC gain, no 1/dt noise
    % amplification.
    ydf = (y - yf)/tau_d;
    yf  = yf + (dt/tau_d)*(y - yf);

    e  = r(k)  - y;
    ed = rd(k) - ydf;
    u_raw = Kp*e + Kd*ed + Ki*integ;

    % tracking anti-windup
    if Ki > 0
        integ = integ + dt*e + (u_app - u_raw)*dt/(Ki*wind);
        integ = min(max(integ, -ilim), ilim);
    end

    u = min(max(u_raw, -u_max), u_max);
    du = min(max(u - u_app, -dslew), dslew);
    u = u_app + du;

    [d_ood, q, qd] = local_ood_step(ood, t(k), q, qd, u, x, dt);
    F = local_friction(xd, plant.fc(k), plant.fs(k), plant.vs(k), plant.fvisc(k));
    acc = (u - plant.b(k)*xd - plant.k(k)*x - F + d_ood)/plant.J(k);
    xd = xd + dt*acc;
    x  = x  + dt*xd;

    X(k) = x; XD(k) = xd; U(k) = u; Y(k) = y;
    u_app = u;
end
B = struct('name', 'Fixed baseline', 'x', X, 'xd', XD, 'u', U, 'y', Y, ...
           'gate', nan(N,1), 'regime', nan(N,1), 'res_fast', nan(N,1), ...
           'res_slow', nan(N,1), 'ai_conf', nan(N,1), 'faults', uint32(0));
end

% =========================================================================
%  BRANCH 2 -- end-to-end continuous AI policy (overfitted inverse dynamics)
% =========================================================================
function B = local_run_e2e(P, t, r, rd, rdd, noise, spike, plant, ood, ...
                           sched, opts, x0)
N  = numel(t); dt = P.dt;
x = x0; xd = 0; q = 0; qd = 0;
u_app = 0; y_prev = 0; yf = x0; ydf = 0;

m  = P.b2.model;                       % frozen inverse-dynamics model
Kp = P.b2.Kp; Kd = P.b2.Kd;
u_max = P.act.u_max; dslew = P.act.slew_max*dt;
tau_d = P.sensor.deriv_tau_s;

X = zeros(N,1); XD = zeros(N,1); U = zeros(N,1); Y = zeros(N,1);
u_hold = 0;
frame_u = nan(sched.n_frame,1);
edge_on = opts.edge_on;
t_edge  = P.edge.t_start;

for k = 1:N
    tk = t(k);
    y = x + noise(k) + spike(k);
    ydf = (y - yf)/tau_d;
    yf  = yf + (dt/tau_d)*(y - yf);

    e  = r(k)  - y;
    ed = rd(k) - ydf;

    if (~edge_on) || (tk < t_edge)
        % high-frequency (1 kHz) continuous policy: computed torque on the
        % frozen nominal model + learned feedback.
        Fm = local_friction(rd(k), m.fc, m.fs, m.vs, m.fvisc);
        u_cmd = m.J*rdd(k) + m.b*rd(k) + m.k*r(k) + Fm + Kp*e + Kd*ed;
        u_hold = u_cmd;
    else
        % edge deployment: inference at 10 Hz, transport delay, frame drops.
        m_emit = sched.emit_map(k);
        if m_emit > 0 && sched.t_emit(m_emit) >= t_edge && sched.arr_idx(m_emit) > 0
            Fm = local_friction(rd(k), m.fc, m.fs, m.vs, m.fvisc);
            frame_u(m_emit) = m.J*rdd(k) + m.b*rd(k) + m.k*r(k) + ...
                              Fm + Kp*e + Kd*ed;
        end
        m_arr = sched.arr_map(k);
        if m_arr > 0 && ~isnan(frame_u(m_arr))
            u_hold = frame_u(m_arr);
        end
        u_cmd = u_hold;
    end

    u = min(max(u_cmd, -u_max), u_max);
    du = min(max(u - u_app, -dslew), dslew);
    u = u_app + du;

    [d_ood, q, qd] = local_ood_step(ood, tk, q, qd, u, x, dt);
    F = local_friction(xd, plant.fc(k), plant.fs(k), plant.vs(k), plant.fvisc(k));
    acc = (u - plant.b(k)*xd - plant.k(k)*x - F + d_ood)/plant.J(k);
    xd = xd + dt*acc;
    x  = x  + dt*xd;

    X(k) = x; XD(k) = xd; U(k) = u; Y(k) = y;
    u_app = u;
end
B = struct('name', 'End-to-end AI', 'x', X, 'xd', XD, 'u', U, 'y', Y, ...
           'gate', nan(N,1), 'regime', nan(N,1), 'res_fast', nan(N,1), ...
           'res_slow', nan(N,1), 'ai_conf', nan(N,1), 'faults', uint32(0));
end

% =========================================================================
%  BRANCH 3 -- proposed layered architecture
% =========================================================================
function B = local_run_layered(P, t, r, rd, rdd, noise, spike, plant, ood, ...
                               sched, opts, cfg, x0)
N  = numel(t); dt = P.dt;
x = x0; xd = 0; q = 0; qd = 0;
u_app = 0; y_prev = 0;
yf = x0; ydf = 0;

% nominal observer (frozen nominal model, deployment-time)
m = P.b2.model;
L1 = P.observer.L1; L2 = P.observer.L2;
xhat = x0; xdhat = 0;

% residual probe: two channels (slow drift channel + fast OOD channel)
tau_lp = P.ai.lp_tau_s;
lp = 0;
res_fast = 0; res_slow = 0;
guard = false;

% slow-loop feature EMAs (warm started from the nominal operating point)
tau_w  = P.ai.window_tau_s;
tau_v  = P.ai.window_tau_v_s;
ms_e  = (0.02*P.ref.amp)^2;
ms_u  = (0.10*P.act.u_max)^2;
ms_v  = (0.35*P.ref.v_scale)^2;
fsc   = P.ai.feature_scales;
proto = P.ai.prototypes;
warmup = 3*tau_w;

% slow-loop mailbox
ai_regime = 1; ai_conf = 0.95; last_arrival = 0; t_first_class = inf;
frame_regime = nan(sched.n_frame,1);
frame_conf   = nan(sched.n_frame,1);
frame_feat   = nan(sched.n_frame,4);

core = local_core_init(P, cfg);

X = zeros(N,1); XD = zeros(N,1); U = zeros(N,1); Y = zeros(N,1);
GA = zeros(N,1); RG = zeros(N,1); RF = zeros(N,1); RS = zeros(N,1);
CF = zeros(N,1); AG = zeros(N,1); RC = zeros(N,1); DW = zeros(N,1);

for k = 1:N
    tk = t(k);
    y = x + noise(k) + spike(k);

    % ---- physics residual probe (two-channel) --------------------------
    inov = y - xhat;
    lp = lp + (dt/tau_lp)*(inov - lp);
    res_slow = abs(lp)/P.ai.res_slow_scale;
    res_fast = abs(inov - lp)/P.ai.res_fast_scale;

    % ---- nominal observer ---------------------------------------------
    xhat  = xhat  + dt*(xdhat + L1*inov);
    xdhat = xdhat + dt*((u_app - m.b*xdhat - m.k*xhat - ...
                         local_friction(xdhat, m.fc, m.fs, m.vs, m.fvisc))/m.J ...
                        + L2*inov);

    % ---- sensor-fault hardening: model-based reconstruction -------------
    guard = res_fast > 0.5*cfg.max_residual;
    if guard
        y_pos = xhat;
        y_tmp = xhat;
        ydf   = xdhat;
        yf    = xhat;
    else
        ydf   = (y - yf)/P.sensor.deriv_tau_s;      % dirty derivative
        yf    = yf + (dt/P.sensor.deriv_tau_s)*(y - yf);
        y_pos = y;
        y_tmp = y;
    end
    y_prev = y_tmp;

    % ---- macro features for the coarse classifier ----------------------
    e3 = r(k) - y_pos;
    ms_e = ms_e + (dt/tau_w)*(e3^2 - ms_e);
    ms_u = ms_u + (dt/tau_w)*(u_app^2 - ms_u);
    ms_v = ms_v + (dt/tau_v)*(ydf^2 - ms_v);

    % ---- slow-loop frames: emit (classify) / arrive (mailbox) -----------
    m_emit = sched.emit_map(k);
    if m_emit > 0 && tk >= warmup
        feat = [sqrt(ms_e)/fsc(1), sqrt(ms_u)/fsc(2), ...
                res_slow/fsc(3), sqrt(ms_v)/fsc(4)];
        [reg, conf] = local_classify(feat, proto);
        frame_regime(m_emit) = reg;
        frame_conf(m_emit)   = conf;
        frame_feat(m_emit,:) = feat;
    end
    m_arr = sched.arr_map(k);
    if m_arr > 0 && ~isnan(frame_regime(m_arr))
        ai_regime  = frame_regime(m_arr);
        ai_conf    = min(max(frame_conf(m_arr), 0), 1);
        last_arrival = tk;
        t_first_class = min(t_first_class, tk);
    end
    if ~isnan(opts.fixed_regime)
        ai_regime = opts.fixed_regime;
        ai_conf   = 1.0;
    end
    if isinf(t_first_class)
        age = 0;                       % no frame expected yet: mailbox is warm
    else
        age = tk - last_arrival;
    end

    % ---- 1 kHz deterministic core --------------------------------------
    % White-box feedforward from the commissioning model: the plant-inverse
    % term is NEVER produced by the AI, it is part of the deterministic core.
    if P.b3.ff_weight ~= 0
        u_ff = P.b3.ff_weight*(m.J*rdd(k) + m.b*rd(k) + m.k*r(k) + ...
               local_friction(rd(k), m.fc, m.fs, m.vs, m.fvisc));
    else
        u_ff = 0;
    end
    in = struct('ai_regime', ai_regime, 'confidence', ai_conf, ...
                'age', age, 'residual', res_fast, ...
                'x', y_pos, 'xd', ydf, 'r', r(k), 'rd', rd(k), ...
                'u_ff', u_ff);
    [core, u, gate, regime] = local_core_step(core, cfg, in);

    % ---- plant ---------------------------------------------------------
    [d_ood, q, qd] = local_ood_step(ood, tk, q, qd, u, x, dt);
    F = local_friction(xd, plant.fc(k), plant.fs(k), plant.vs(k), plant.fvisc(k));
    acc = (u - plant.b(k)*xd - plant.k(k)*x - F + d_ood)/plant.J(k);
    xd = xd + dt*acc;
    x  = x  + dt*xd;

    X(k) = x; XD(k) = xd; U(k) = u; Y(k) = y;
    GA(k) = gate; RG(k) = regime; RF(k) = res_fast; RS(k) = res_slow;
    CF(k) = ai_conf; AG(k) = age;
    RC(k) = core.recovery_ticks; DW(k) = core.dwell_ticks;
    u_app = u;
end

B = struct('name', 'Layered AI', 'x', X, 'xd', XD, 'u', U, 'y', Y, ...
           'gate', GA, 'regime', RG, 'res_fast', RF, 'res_slow', RS, ...
           'ai_conf', CF, 'faults', core.fault_bits, ...
           'core_counters', core.counters, ...
           'age', AG, 'recovery_ticks', RC, 'dwell_ticks', DW, ...
           'frame_regime', frame_regime, 'frame_conf', frame_conf, ...
           'frame_feat', frame_feat);
end

% =========================================================================
%  1 kHz CORE -- MATLAB mirror of layered_gatekeeper.c
% =========================================================================
function core = local_core_init(P, cfg)
core = struct();
core.u_applied = 0;
core.u_target  = 0;
core.integ     = zeros(4,1);
core.blend     = 0;
core.regime    = 1;              % REGIME_NOMINAL
core.gate      = 0;              % GATE_NORMAL
core.ood_latched    = false;
core.safety_latched = false;
core.conf_latched   = false;
core.dwell_ticks    = 0;
core.recovery_ticks = 0;
core.fault_bits     = uint32(0);
core.counters = struct('safety', 0, 'ood', 0, 'conf', 0, 'latency', 0, ...
                       'corrupted', 0, 'steps', 0);
end

function [core, u, gate, regime] = local_core_step(core, cfg, in)
% Straight port of Layered_Step() from c99_layered_core/layered_gatekeeper.c.

FAULT_LATENCY = uint32(1);   % 1u<<0
FAULT_OOD     = uint32(2);   % 1u<<1
FAULT_CONF    = uint32(4);   % 1u<<2
FAULT_ENVELOPE= uint32(8);   % 1u<<3
FAULT_NAN     = uint32(16);  % 1u<<4

u = core.u_applied; gate = core.gate; regime = core.regime;
core.counters.steps = core.counters.steps + 1;

% ---- 0. INPUT FIREWALL (before touching any controller state) -----------
vals = [in.ai_regime, in.confidence, in.age, in.residual, in.x, in.xd, ...
        in.r, in.rd, in.u_ff];
bad = any(~isfinite(vals)) || in.ai_regime < 1 || in.ai_regime > 4 || ...
      in.confidence < 0 || in.confidence > 1 || in.age < 0 || in.residual < 0;
if bad
    core.fault_bits = bitor(core.fault_bits, FAULT_NAN);
    core.counters.corrupted = core.counters.corrupted + 1;
    core.gate = 1;                       % GATE_HOLD_LATENCY: fail-safe hold
    gate = 1;
    return;                              % integrators NOT updated
end

% ---- 1. latches ---------------------------------------------------------
limit_eff = cfg.x_limit*cfg.safety_margin_scale;
pred = abs(in.x) + cfg.predict_s*abs(in.xd);
if abs(in.x) > cfg.trip_ratio*limit_eff || pred > limit_eff
    if ~core.safety_latched
        core.counters.safety = core.counters.safety + 1;
    end
    core.safety_latched = true;
    core.fault_bits = bitor(core.fault_bits, FAULT_ENVELOPE);
elseif abs(in.x) < cfg.clear_ratio*limit_eff && pred < cfg.trip_ratio*limit_eff
    core.safety_latched = false;
end

if in.residual > cfg.max_residual
    if ~core.ood_latched
        core.counters.ood = core.counters.ood + 1;
    end
    core.ood_latched = true;
    core.fault_bits = bitor(core.fault_bits, FAULT_OOD);
elseif in.residual < cfg.res_clear*cfg.max_residual
    core.ood_latched = false;
end

if (in.confidence < cfg.min_confidence) || (in.ai_regime == 4)
    if ~core.conf_latched
        core.counters.conf = core.counters.conf + 1;
    end
    core.conf_latched = true;
    core.fault_bits = bitor(core.fault_bits, FAULT_CONF);
elseif in.confidence >= cfg.conf_clear && in.ai_regime <= 3
    core.conf_latched = false;
end

stale = in.age > cfg.timeout_s;

% ---- 2. gate decision (priority: safety > OOD > conf > latency > normal) -
if core.safety_latched
    gate = 4;
elseif core.ood_latched
    gate = 2;
elseif core.conf_latched
    gate = 3;
elseif stale
    gate = 1;
    core.counters.latency = core.counters.latency + 1;
    core.fault_bits = bitor(core.fault_bits, FAULT_LATENCY);
else
    gate = 0;
end

% ---- 3. recovery cooldown ----------------------------------------------
if gate ~= 0
    core.recovery_ticks = cfg.recovery;
elseif core.recovery_ticks > 0
    core.recovery_ticks = core.recovery_ticks - 1;
end

% ---- 4. regime selection with anti-chatter dwell -------------------------
if gate == 1
    % heartbeat hold: keep the last regime, hold the actuator command
    core.gate = gate;
    return;
elseif gate == 2 || gate == 3 || gate == 4 || core.recovery_ticks > 0
    target = 4;                       % REGIME_SAFE_FALLBACK
else
    cand = round(in.ai_regime);
    core.dwell_ticks = core.dwell_ticks + 1;
    if cand == core.regime
        target = core.regime;
    elseif core.dwell_ticks >= cfg.min_dwell
        core.dwell_ticks = 0;
        target = cand;
    else
        target = core.regime;         % anti-chatter: keep current regime
    end
end

% ---- 5. controller bank + bumpless transfer + clamps --------------------
i  = target;
kp = cfg.kp(i)*cfg.gain_scale;
kd = cfg.kd(i)*cfg.damping_scale;
ki = cfg.ki(i)*cfg.gain_scale;
e  = in.r  - in.x;
ed = in.rd - in.xd;

u_pid = kp*e + kd*ed + ki*core.integ(i) + in.u_ff;

alpha = min(max(cfg.blend_alpha, 1e-3), 1);
core.blend = core.blend + alpha*(1 - core.blend);
u_tgt = core.u_target + alpha*(u_pid - core.u_target);

u_c = min(max(u_tgt, -cfg.u_max), cfg.u_max);
dslew = cfg.slew_max*cfg.dt;
du = min(max(u_c - core.u_applied, -dslew), dslew);
u = core.u_applied + du;

% ---- 6. tracking anti-windup for EVERY bank controller ------------------
for j = 1:4
    kij = cfg.ki(j)*cfg.gain_scale;
    if kij > eps
        u_pid_j = cfg.kp(j)*cfg.gain_scale*e + cfg.kd(j)*cfg.damping_scale*ed + ...
                  kij*core.integ(j) + in.u_ff;
        core.integ(j) = core.integ(j) + cfg.dt*e + ...
                        (u - u_pid_j)*cfg.dt/(kij*cfg.windup_tau_s);
        core.integ(j) = min(max(core.integ(j), -cfg.integ_limit), cfg.integ_limit);
    end
end

core.u_applied = u;
core.u_target  = u_tgt;
core.regime    = target;
core.gate      = gate;
regime = target;
end

function [regime, conf] = local_classify(feat, proto)
% Nearest-centroid coarse classifier with a standard margin confidence:
%   conf = d2 / (d1 + d2) in [0,1], where d1 is the distance to the winning
% prototype and d2 the distance to the runner-up.  conf -> 1 when the sample
% sits on one prototype, conf -> 0 when two classes are equally close (which
% is exactly the OOD / ambiguous situation the gatekeeper must abstain on).
d  = sqrt(sum((proto - feat).^2, 2));
[d1, i1] = min(d);
d2 = min(d([1:i1-1, i1+1:end]));
regime = i1;
conf = d2/max(d1 + d2, 1e-9);
conf = min(max(conf, 0), 1);
end

function [d_ood, q, qd] = local_ood_step(ood, tk, q, qd, u, x, dt)
if ood.on && tk >= ood.t_start
    qdd = -2*ood.zr*ood.wr*qd - ood.wr^2*q + ood.cu*u + ood.cx*x;
    qd  = qd + dt*qdd;
    q   = q  + dt*qd;
    d_ood = ood.kq*q;
else
    d_ood = 0;
    q = 0; qd = 0;
end
end

% =========================================================================
%  METRICS
% =========================================================================
function M = local_metrics(P, out, opts)
edges = P.stage.edges;
edges = edges(edges < opts.t_end + 1e-12);
if isempty(edges) || edges(end) < opts.t_end
    edges = [edges, opts.t_end];
end
n_s   = numel(edges) - 1;
names = {'B1 fixed', 'B2 end-to-end AI', 'B3 layered'};
branches = {out.b1, out.b2, out.b3};
u_max = P.act.u_max;
dt = P.dt;
win = max(3, round(P.metrics.chatter_window_s/dt));

M = struct();
M.stage_edges = edges;
M.stage_names = names;
M.rmse      = nan(3, n_s);
M.maxabs    = nan(3, n_s);
M.violations= nan(3, n_s);
M.chatter   = nan(3, n_s);   % phase-lag limit-cycle index (error above 1 Hz)
M.chatter_u = nan(3, n_s);   % high-frequency content of the command
M.tv        = nan(3, n_s);   % actuator wear: sum |u_k - u_{k-1}|
M.sat_frac  = nan(3, n_s);

% First-order high-pass used by the phase-lag chattering index.
tau_hp = 0.16;                      % s -> ~1 Hz corner
ahp = dt/tau_hp;

for b = 1:3
    B = branches{b};
    if isempty(B)
        continue;
    end
    u_hp = B.u - movmean(B.u, win);
    err  = B.x - out.ref.r;
    % y = filter(alpha, [1, -(1-alpha)], x)  is a first-order low-pass
    err_lp = filter(ahp, [1, -(1-ahp)], err);
    err_hp = err - err_lp;
    for s = 1:n_s
        idx = out.t >= edges(s) & out.t < edges(s+1);
        if ~any(idx)
            continue;
        end
        M.rmse(b,s)       = sqrt(mean(err(idx).^2));
        M.maxabs(b,s)     = max(abs(B.x(idx)));
        M.violations(b,s) = sum(abs(B.x(idx)) > P.safety.x_limit);
        M.chatter(b,s)    = sqrt(mean(err_hp(idx).^2))/P.ref.amp;
        M.chatter_u(b,s)  = sqrt(mean(u_hp(idx).^2))/u_max;
        M.tv(b,s)         = sum(abs(diff(B.u(idx))));
        M.sat_frac(b,s)   = mean(abs(B.u(idx)) >= 0.999*u_max);
    end
end

% OOD detection latency of Branch 3 (first veto after fault onset).
M.veto_latency_ms = NaN;
M.recovery_ms     = NaN;
M.gate_hist       = zeros(5,1);
M.regime_hist     = zeros(4,1);
if ~isempty(out.b3)
    i_on = round(P.ood.t_start/dt) + 1;
    if opts.ood_on && i_on <= numel(out.t)
        veto = find(out.b3.gate(i_on:end) == 2 | out.b3.gate(i_on:end) == 4, 1);
        if ~isempty(veto)
            M.veto_latency_ms = (veto - 1)*dt*1e3;
        end
        % recovery: last veto sample -> first sample back on a normal regime
        last_veto = find(out.b3.gate(i_on:end) ~= 0, 1, 'last');
        if ~isempty(last_veto)
            idx0 = i_on + last_veto - 1;
            back = find(out.b3.regime(idx0:end) <= 3, 1);
            if ~isempty(back)
                M.recovery_ms = (back - 1)*dt*1e3;
            end
        end
    end
    g = out.b3.gate;
    for v = 0:4
        M.gate_hist(v+1) = nnz(g == v);
    end
    rg = out.b3.regime;
    for v = 1:4
        M.regime_hist(v) = nnz(rg == v);
    end
end

% Metrics that the README quotes directly.
M.stage_names_short = P.stage.short;
M.ref = out.ref;
end

function A = local_assertions(P, out)
% Engineering assertions from the execution plan.  Returns a struct with
% logical flags and the numbers behind them.  Assertions that need a stage the
% caller did not simulate are reported as not evaluable (false + NaN) instead
% of raising, so short/partial runs (calibration, panel B) still work.
A = struct();
M = out.metrics;
edge = P.stage.edges;
n_s  = size(M.rmse, 2);
has_stage = @(s) (s <= n_s) && (out.t(end) >= edge(s+1) - P.dt - 1e-9);

% Assertion 1: Stage 2, B2 RMSE > 2x B3 RMSE.
if ~isempty(out.b2) && ~isempty(out.b3) && has_stage(2)
    idx2 = out.t >= edge(2) & out.t < edge(3);
    A.stage2_rmse_b2 = sqrt(mean((out.b2.x(idx2)-out.ref.r(idx2)).^2));
    A.stage2_rmse_b3 = sqrt(mean((out.b3.x(idx2)-out.ref.r(idx2)).^2));
    A.assert1 = A.stage2_rmse_b2 > 2*A.stage2_rmse_b3;
else
    A.stage2_rmse_b2 = NaN; A.stage2_rmse_b3 = NaN; A.assert1 = false;
end

% Assertion 2: Stage 3, B2 phase-lag chattering and tracking both degrade
% far beyond B3 under 10 Hz + 60-120 ms jitter + frame drops.
if ~isempty(out.b2) && ~isempty(out.b3) && has_stage(3)
    A.stage3_chatter_b2 = M.chatter(2, 3);
    A.stage3_chatter_b3 = M.chatter(3, 3);
    A.stage3_rmse_b2 = M.rmse(2, 3);
    A.stage3_rmse_b3 = M.rmse(3, 3);
    A.assert2 = (A.stage3_chatter_b2 > 3*A.stage3_chatter_b3) && ...
                (A.stage3_rmse_b2 > 2*A.stage3_rmse_b3);
else
    A.stage3_chatter_b2 = NaN; A.stage3_chatter_b3 = NaN;
    A.stage3_rmse_b2 = NaN; A.stage3_rmse_b3 = NaN;
    A.assert2 = false;
end

% Assertion 3: Stage 4, B2 breaches the envelope, B3 has zero violations
% and abstains within 10 ms.
if ~isempty(out.b2) && ~isempty(out.b3) && has_stage(4)
    idx4 = out.t >= edge(4);
    A.stage4_violations_b2 = sum(abs(out.b2.x(idx4)) > P.safety.x_limit);
    A.stage4_violations_b3 = sum(abs(out.b3.x(idx4)) > P.safety.x_limit);
    A.stage4_maxabs_b2 = max(abs(out.b2.x(idx4)));
    A.stage4_maxabs_b3 = max(abs(out.b3.x(idx4)));
    A.veto_latency_ms = M.veto_latency_ms;
    A.assert3 = (A.stage4_violations_b2 > 0) && ...
                (A.stage4_violations_b3 == 0) && ...
                (A.veto_latency_ms < 10);
else
    A.stage4_violations_b2 = NaN; A.stage4_violations_b3 = NaN;
    A.stage4_maxabs_b2 = NaN; A.stage4_maxabs_b3 = NaN;
    A.veto_latency_ms = NaN; A.assert3 = false;
end
A.all = A.assert1 && A.assert2 && A.assert3;
end

function local_print_summary(P, out)
fprintf('\n--- sim_core_engine: %s (%s) ---\n', P.name, P.kind);
fprintf('%-18s %10s %10s %10s\n', 'branch', 'S1 RMSE', 'S2 RMSE', 'S4 max|x|');
lab = {'B1 fixed', 'B2 e2e AI', 'B3 layered'};
for b = 1:3
    fprintf('%-18s %10.4f %10.4f %10.4f\n', lab{b}, ...
        out.metrics.rmse(b,1), out.metrics.rmse(b,2), out.metrics.maxabs(b,4));
end
fprintf('assert1=%d assert2=%d assert3=%d (veto latency %.2f ms)\n', ...
    out.metrics.assertions.assert1, out.metrics.assertions.assert2, ...
    out.metrics.assertions.assert3, out.metrics.assertions.veto_latency_ms);
end
