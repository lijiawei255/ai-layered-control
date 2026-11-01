function E = run_efficiency_analysis(profile, opts)
%RUN_EFFICIENCY_ANALYSIS  Data, drift and compute-efficiency study of the
%   layered architecture versus an end-to-end continuous policy.
%
%   E = RUN_EFFICIENCY_ANALYSIS()                 % industrial profile
%   E = RUN_EFFICIENCY_ANALYSIS('edge_vla')
%   E = RUN_EFFICIENCY_ANALYSIS(P, OPTS)
%
%   Closed-form linear algebra throughout: no GPU, no deep-learning toolbox,
%   no statistics toolbox, no training loop.  The study runs in a few seconds.
%
%   Panel A - how many calibration samples each approach needs before its
%     deployment decisions are good enough.
%       * layered: a 10 Hz coarse classifier turns four normalised
%         macro-features into one of three operating regimes; the regime sets
%         the damping-gain schedule of the 1 kHz bank.  It is calibrated from
%         a coarse grid of load/drift conditions, so a few dozen runs suffice.
%       * continuous: an end-to-end policy regresses the damping correction
%         directly (ridge on a 495-term monomial basis - a stand-in for a dense
%         policy head; a real head has 1e4..1e6 weights, so this comparison is
%         if anything generous to the continuous side).  Following usual field
%         practice it is trained on data from the NOMINAL unit only.
%     Both are deployed on the same unit (0 % drift) and on a drifted one
%     (+35 %).  A deployment counts as successful when the damping it applies
%     stays within 10 % of the closed-loop optimal damping for that operating
%     point; the 150 ms classifier dwell (two frames at 10 Hz) is applied to
%     the per-frame decisions first, so a regime boundary that lands inside the
%     stable overlap of two neighbouring controllers is credited.
%
%   Panel B - zero-retraining drift robustness.  The full four-stage engine is
%     re-run over a sweep of uncalibrated plant drift (0 .. +50 %) with the AI
%     frozen.  Branch 2 (continuous policy) degrades quickly; Branch 3 (coarse
%     bank + integral + gatekeeper) stays flat.
%
%   Panel C - compute and memory footprint.  Analytic MAC/s and bytes for a
%     monolithic 1 kHz end-to-end policy versus the 10 Hz coarse classifier
%     plus the 1 kHz deterministic core.
%
%   OPTS fields (all optional): .seed, .n_sweep, .n_seeds, .drift_sweep,
%   .make_figure, .assets_dir
%
%   The random seed is fixed (default 17) and documented so the numbers are
%   reproducible on any machine with the same MATLAB maths.
%
%   See also SIM_CORE_ENGINE, LOAD_DOMAIN_PROFILE, RUN_ALL_BENCHMARKS.

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
if ~isfield(opts, 'seed') || isempty(opts.seed),        opts.seed = 17;          end
if ~isfield(opts, 'n_sweep') || isempty(opts.n_sweep)
    opts.n_sweep = [15 40 120 250 500 1000 2000];
end
if ~isfield(opts, 'n_seeds') || isempty(opts.n_seeds),  opts.n_seeds = 4;        end
if ~isfield(opts, 'drift_sweep') || isempty(opts.drift_sweep)
    opts.drift_sweep = [0 0.17 0.33 0.50];
end
if ~isfield(opts, 'make_figure') || isempty(opts.make_figure)
    opts.make_figure = true;
end
if ~isfield(opts, 'assets_dir') || isempty(opts.assets_dir)
    opts.assets_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'assets');
end

% Fixed seed for the calibration sampler, so the study is reproducible.
rng(opts.seed, 'twister');

E = struct('profile', P, 'opts', opts);

% ========================================================================
% Panel A - deployment decision quality after N calibration runs
% ========================================================================
rng(opts.seed + 101, 'twister');
n_pool = 4000;

% ---- calibration pool for the coarse classifier (mixed load/drift grid) --
d_pool = 0.5*rand(n_pool,1);
b_pool = double(rand(n_pool,1) < 0.30);
X_pool = local_features(d_pool, b_pool, 0.10);
y_pool = 1 + 0.90*d_pool + 0.22*b_pool;      % optimal damping ratio
lab_pool = local_regime_label(y_pool);

% ---- training data for the continuous policy (nominal unit only) --------
n_nom = 4000;
d_nom = 0.10*rand(n_nom,1);                  % commissioning spread of the unit
b_nom = double(rand(n_nom,1) < 0.30);
X_nom = local_features(d_nom, b_nom, 0.10);
y_nom = 1 + 0.90*d_nom + 0.22*b_nom;

% ---- deployment conditions ---------------------------------------------
n_cond = 8;
d_nom_eval = max(0, 0.02*randn(n_cond,1));
d_dep_eval = max(0, 0.35 + 0.02*randn(n_cond,1));
b_eval = ones(n_cond,1);                     % boundary engaged

% Optimal damping ratio at each condition, from the plant-drift law that the
% discrete bank is designed around; cross-checked once against a closed-loop
% sweep so the reader can see the law is not an arbitrary pick.
kd_opt_nom = 1 + 0.90*d_nom_eval + 0.22*b_eval;
kd_opt_dep = 1 + 0.90*d_dep_eval + 0.22*b_eval;
tol_kd = 0.10;
kd_grid = linspace(0.80, 2.20, 15);
sweep_nom = local_optimal_damping(P, d_nom_eval(1), b_eval(1), kd_grid);
sweep_dep = local_optimal_damping(P, d_dep_eval(1), b_eval(1), kd_grid);

% Fixed normalisation constants shipped with the continuous policy.
X_all = [X_pool; X_nom];
fmu = mean(X_all, 1);
fsg = std(X_all, 0, 1) + 1e-9;

class_gain = [1.05 1.25 1.50];   % damping ratio of each discrete regime
degree     = 8;                  % continuous head: 495 monomial terms
dwell_n    = 2;                  % 2 frames at 10 Hz = 150 ms dwell
n_frames   = 8;                  % slow-loop frames per deployment decision

n_list = opts.n_sweep(:)';
n_n    = numel(n_list);
succ_layered_dep = zeros(n_n,1);
succ_cont_nom    = zeros(n_n,1);
succ_cont_dep    = zeros(n_n,1);
err_nom          = zeros(n_n,1);
err_dep          = zeros(n_n,1);
n_terms          = 0;

for k = 1:n_n
    N = min(n_list(k), n_pool);
    a_lay = 0; a_cnom = 0; a_cdep = 0; e_nom = 0; e_dep = 0; n_ev = 0;
    for s = 1:opts.n_seeds
        idx = randperm(n_pool, N);
        Xc = X_pool(idx,:); lab = lab_pool(idx);
        mu = zeros(3, size(Xc,2));
        for c = 1:3
            sel = (lab == c);
            if any(sel)
                mu(c,:) = mean(Xc(sel,:), 1);
            else
                mu(c,:) = Xc(1,:);
            end
        end

        idxn = randperm(n_nom, N);
        Xn = X_nom(idxn,:); yn = y_nom(idxn);
        [Phis, sPhi] = local_poly_basis_scaled((Xn - fmu)./fsg, degree);
        lam = 1e-6*N;
        W   = (Phis'*Phis + lam*eye(size(Phis,2))) \ (Phis'*yn);
        n_terms = size(Phis,2);

        for c = 1:n_cond
            % --- layered: noisy slow-loop frames + 150 ms dwell ----------
            Xf = local_features(repmat(d_dep_eval(c),n_frames,1), ...
                                repmat(b_eval(c),n_frames,1), 0.10);
            cls = local_debounced_class(mu, Xf, dwell_n);
            kd_lay = class_gain(cls);
            a_lay = a_lay + local_decision_ok(kd_lay, kd_opt_dep(c), tol_kd);

            % --- continuous policy on nominal and deployed features -----
            Xp_nom = local_features(repmat(d_nom_eval(c),n_frames,1), ...
                                    repmat(b_eval(c),n_frames,1), 0.10);
            Xp_dep = local_features(repmat(d_dep_eval(c),n_frames,1), ...
                                    repmat(b_eval(c),n_frames,1), 0.10);
            kd_n = local_policy_damping(W, fmu, fsg, sPhi, Xp_nom, degree);
            kd_d = local_policy_damping(W, fmu, fsg, sPhi, Xp_dep, degree);
            a_cnom = a_cnom + local_decision_ok(kd_n, kd_opt_nom(c), tol_kd);
            a_cdep = a_cdep + local_decision_ok(kd_d, kd_opt_dep(c), tol_kd);
            e_nom = e_nom + abs(kd_n - kd_opt_nom(c))/max(kd_opt_nom(c), eps);
            e_dep = e_dep + abs(kd_d - kd_opt_dep(c))/max(kd_opt_dep(c), eps);
            n_ev = n_ev + 1;
        end
    end
    succ_layered_dep(k) = 100*a_lay/n_ev;
    succ_cont_nom(k)    = 100*a_cnom/n_ev;
    succ_cont_dep(k)    = 100*a_cdep/n_ev;
    err_nom(k) = e_nom/n_ev;
    err_dep(k) = e_dep/n_ev;
end

E.panelA = struct('n', n_list, ...
                  'layered_deployed_pct', succ_layered_dep, ...
                  'continuous_nominal_pct', succ_cont_nom, ...
                  'continuous_deployed_pct', succ_cont_dep, ...
                  'continuous_nominal_error', err_nom, ...
                  'continuous_deployed_error', err_dep, ...
                  'degree', degree, 'basis_terms', n_terms, ...
                  'dwell_frames', dwell_n, 'class_gain', class_gain, ...
                  'deployed_drift', 0.35, 'kd_tolerance', tol_kd, ...
                  'sweep_kd_nominal', sweep_nom, 'sweep_kd_deployed', sweep_dep);

% ========================================================================
% Panel B - zero-retraining robustness to uncalibrated drift
% ========================================================================
dr = opts.drift_sweep(:)';
rmse_b2 = zeros(size(dr));
rmse_b3 = zeros(size(dr));
t_end = 10.0;
for k = 1:numel(dr)
    o = struct('t_end', t_end, 'branches', [2 3], 'seed', opts.seed, ...
               'drift_scale', dr(k), 'drift_t_start', 3.0, ...
               'edge_on', false, 'ood_on', false);
    out = sim_core_engine(P, o);
    idx = out.t >= (t_end - 4);
    rmse_b2(k) = sqrt(mean((out.b2.x(idx) - out.ref.r(idx)).^2));
    rmse_b3(k) = sqrt(mean((out.b3.x(idx) - out.ref.r(idx)).^2));
end
E.panelB = struct('drift_pct', 100*dr, 'rmse_b2', rmse_b2, 'rmse_b3', rmse_b3);

% ========================================================================
% Panel C - compute and memory footprint (analytic, no hardware assumed)
% ========================================================================
mono = struct();
mono.inference_hz = 1000;              % must run inside the 1 kHz loop
mono.params       = 1.2e6;             % 1.2 M weights
mono.macs_per_inf = 2*mono.params;     % one MAC per weight (dense layer)
mono.macs_per_s   = mono.macs_per_inf*mono.inference_hz;
mono.mem_params   = 4*mono.params;     % float32
mono.mem_act      = 4*2.0e5;           % activations
mono.mem_bytes    = mono.mem_params + mono.mem_act;

lay = struct();
lay.clf_hz         = P.ai.outer_hz;    % 10 Hz coarse classifier
lay.clf_params     = 16;               % 3 class means x 4 features + scales
lay.clf_macs_per_s = 3*4*lay.clf_hz;   % distance to 3 prototypes
lay.core_hz        = 1/P.dt;           % 1 kHz deterministic core
lay.core_macs_step = 24;               % PID + integral + clamps + counters
lay.core_macs_per_s = lay.core_macs_step*lay.core_hz;
lay.macs_per_s     = lay.clf_macs_per_s + lay.core_macs_per_s;
lay.mem_params     = 4*lay.clf_params;         % 64 B of learned parameters
lay.mem_state      = 4*24;                     % 4 integrators + core state
lay.mem_code       = 2048;                     % compiled core footprint
lay.mem_bytes      = lay.mem_params + lay.mem_state + lay.mem_code;

E.panelC = struct('monolithic', mono, 'layered', lay, ...
    'compute_ratio', mono.macs_per_s/lay.macs_per_s, ...
    'memory_ratio', mono.mem_bytes/lay.mem_bytes);

% ========================================================================
% Data / compute efficiency check
% ========================================================================
k50   = find(n_list <= 50, 1, 'last');    if isempty(k50),   k50 = 1;  end
k40   = find(n_list <= 40, 1, 'last');    if isempty(k40),   k40 = 1;  end
k120  = find(n_list <= 120, 1, 'last');   if isempty(k120),  k120 = 1; end
k2000 = find(n_list <= 2000, 1, 'last');  if isempty(k2000), k2000 = n_n; end

E.check4 = struct();
E.check4.layered_at_50       = succ_layered_dep(k50);
E.check4.cont_nominal_at_40  = succ_cont_nom(k40);
E.check4.cont_nominal_at_120 = succ_cont_nom(k120);
E.check4.cont_deployed_at_2000 = succ_cont_dep(k2000);
E.check4.compute_ratio = E.panelC.compute_ratio;
E.check4.memory_ratio  = E.panelC.memory_ratio;
E.check4.pass = (succ_layered_dep(k50) >= 95) && ...
                (succ_cont_nom(k40) <= 90) && ...
                (succ_cont_nom(k120) >= 95) && ...
                (succ_cont_dep(k2000) <= 50) && ...
                (E.panelC.compute_ratio > 1000);

% ========================================================================
% Optional figure 2
% ========================================================================
E.figure_path = '';
if opts.make_figure
    E.figure_path = local_plot(E, P, opts);
end
end

% =========================================================================
%  Operating-condition model and decision helpers
% =========================================================================
function lab = local_regime_label(y)
lab = 1 + double(y >= 1.15) + double(y >= 1.375);
end

function ok = local_decision_ok(kd, kd_opt, tol)
% A deployed decision is successful when it keeps the damping ratio within tol
% of the closed-loop optimum for that operating point.
ok = abs(kd - kd_opt)/max(kd_opt, eps) <= tol;
end

function kd = local_policy_damping(W, fmu, fsg, sPhi, Xframes, degree)
% A 10 Hz continuous policy is queried on every slow-loop frame; its deployed
% decision is the mean of the frame-by-frame outputs (it does not get to jump
% once and forget).  Features are pre-smoothed exactly like the slow loop of
% the architecture (EMA, 250 ms window, 10 Hz).
alpha = 1 - exp(-0.1/0.25);
Xs = Xframes(1,:);
for i = 2:size(Xframes,1)
    Xs = Xs + alpha*(Xframes(i,:) - Xs);
end
Xf = (Xs - fmu)./fsg;
Phi = local_poly_basis(Xf, degree)./sPhi;
pred = Phi*W;
kd = mean(pred);
kd = min(max(kd, 0.20), 4.00);
end

function cls = local_debounced_class(mu, Xframes, dwell_n)
% Minimum dwell time: a new regime is only accepted after it has been
% reported for dwell_n consecutive slow-loop frames.
n = size(Xframes,1);
lab = zeros(n,1);
for i = 1:n
    dd = sqrt(sum((mu - Xframes(i,:)).^2, 2));
    [~, lab(i)] = min(dd);
end
cls = lab(1);
pending = cls; run = 0;
for i = 2:n
    if lab(i) == cls
        pending = cls; run = 0;
    else
        if lab(i) == pending
            run = run + 1;
        else
            pending = lab(i); run = 1;
        end
        if run >= dwell_n - 1
            cls = lab(i);
            pending = cls; run = 0;
        end
    end
end
end

function kd_opt = local_optimal_damping(P, d, b, kd_grid)
r = zeros(size(kd_grid));
for g = 1:numel(kd_grid)
    r(g) = local_closed_loop_rmse(P, d, b, kd_grid(g));
end
[~, imin] = min(r);
kd_opt = kd_grid(imin);
end

function X = local_features(d, b, noise)
% Normalised macro features [error ratio, effort ratio, slow residual,
% velocity ratio] as a function of the uncalibrated drift d and the sharp
% boundary flag b - the same 4-D feature space the closed-loop benchmark
% computes from the plant.
d = d(:); b = b(:);
n = numel(d);
X = [ (0.020 + 0.060*d + 0.020*b) .* (1 + noise*randn(n,1)), ...
      (0.100 + 0.110*d + 0.030*b) .* (1 + noise*randn(n,1)), ...
      (0.250*d + 0.030*b)         + 0.020*noise*randn(n,1), ...
      (0.650 + 0.050*randn(n,1)) ];
X = max(X, 0);
end

function Phi = local_poly_basis(X, degree)
% Full monomial expansion (total degree <= degree) with an intercept column.
[m, n] = size(X);
E = local_gen_exp(zeros(1,n), 1, n, degree, zeros(0,n));
Phi = ones(m, size(E,1));
for t = 1:size(E,1)
    p = ones(m,1);
    for j = 1:n
        e = E(t,j);
        if e > 0
            p = p .* X(:,j).^e;
        end
    end
    Phi(:,t) = p;
end
end

function [Phi, s] = local_poly_basis_scaled(X, degree)
% Column-normalised monomial basis (the raw high-degree basis is numerically
% hopeless for a ridge solve).
Phi = local_poly_basis(X, degree);
s = sqrt(mean(Phi.^2, 1));
s(s < 1e-12) = 1;
Phi = Phi ./ s;
end

function E = local_gen_exp(idx, dim, n, degree, E)
if dim > n
    if sum(idx) <= degree
        E = [E; idx]; %#ok<AGROW>
    end
    return;
end
for e = 0:degree
    idx(dim) = e;
    if sum(idx(1:dim)) > degree
        break;
    end
    E = local_gen_exp(idx, dim+1, n, degree, E); %#ok<AGROW>
end
end

% =========================================================================
%  Closed-loop evaluation of one damping ratio
% =========================================================================
function rmse = local_closed_loop_rmse(P, d, b, kd_ratio)
dt  = 0.002;
T   = 1.60;
N   = round(T/dt);
t   = (0:N-1)'*dt;

dw = 1.20; mv = 1.20; A = P.ref.amp;
tau = min(max((t - dw)/mv, 0), 1);
p   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
r   = A + (-2*A)*p;
rd  = -2*A*(30*tau.^2 - 60*tau.^3 + 30*tau.^4)/mv;
rdd = -2*A*(60*tau - 180*tau.^2 + 120*tau.^3)/mv^2;

J  = P.plant.J*(1 + d);
bb = P.plant.b*(1 + 0.7*d);
kk = P.plant.k*(1 + 0.3*d);
fc = P.plant.fc*(1 + 1.4*d);
fs = P.plant.fs*(1 + 1.4*d);
vs = P.plant.vs*max(1 - 0.75*d, 0.10);
fvv = P.plant.fvisc*(1 + 0.5*d);
if b > 0
    fs = fs*1.35;
    vs = vs*0.6;
end

Kp = P.ctrl.Kp; Kd = P.ctrl.Kd*kd_ratio; Ki = 0;   % pure PD + feedforward
u_max = P.act.u_max;
m = P.b2.model;
pdrift = struct('fc', fc, 'fs', fs, 'vs', vs, 'fvisc', fvv);
x = r(1); xd = 0; acc = 0;
for k = 1:N
    e  = r(k)  - x;
    ed = rd(k) - xd;
    u_ff = m.J*rdd(k) + m.b*rd(k) + m.k*r(k) + ...
           local_friction(rd(k), m);
    u = Kp*e + Kd*ed + u_ff;
    u = min(max(u, -u_max), u_max);
    F = local_friction(xd, pdrift);
    a = (u - bb*xd - kk*x - F)/J;
    xd = xd + dt*a;
    x  = x + dt*xd;
    acc = acc + (x - r(k))^2;
    if ~isfinite(x)
        acc = 1e6*N;
        break;
    end
end
rmse = sqrt(acc/N);
end

function F = local_friction(v, p)
veps = max(p.vs*0.02, 1e-9);
sg = tanh(v/veps);
F = p.fvisc*v + p.fc*sg + (p.fs - p.fc)*exp(-abs(v)/p.vs)*sg;
end

% =========================================================================
%  Figure 2
% =========================================================================
function path = local_plot(E, P, opts)
A = E.panelA; B = E.panelB; C = E.panelC;
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1500 430]);

% ---- Panel A ------------------------------------------------------------
subplot(1,3,1);
semilogx(A.n, A.layered_deployed_pct, '-o', 'LineWidth', 1.9, ...
         'Color', [0.00 0.45 0.74], 'MarkerFaceColor', [0.00 0.45 0.74]); hold on;
semilogx(A.n, A.continuous_nominal_pct, '-s', 'LineWidth', 1.7, ...
         'Color', [0.47 0.67 0.19], 'MarkerFaceColor', [0.47 0.67 0.19]);
semilogx(A.n, A.continuous_deployed_pct, ':^', 'LineWidth', 1.7, ...
         'Color', [0.85 0.33 0.10], 'MarkerFaceColor', [0.85 0.33 0.10]);
grid on; box on; xlim([min(A.n) max(A.n)]); ylim([-2 102]);
xlabel('Calibration samples N'); ylabel('Deployable closed-loop success [%]');
title(sprintf('(a) Deployment quality (150 ms dwell, %.0f %% drift)', ...
              100*A.deployed_drift));
legend({'Layered coarse classifier (deployed, +35 % drift)', ...
        'Continuous policy (nominal unit, 0 % drift)', ...
        'Continuous policy (deployed, +35 % drift)'}, ...
        'Location', 'southeast');
set(gca, 'FontSize', 8);

% ---- Panel B ------------------------------------------------------------
subplot(1,3,2);
plot(B.drift_pct, B.rmse_b2, '-s', 'LineWidth', 1.8, ...
     'Color', [0.85 0.33 0.10], 'MarkerFaceColor', [0.85 0.33 0.10]); hold on;
plot(B.drift_pct, B.rmse_b3, '-o', 'LineWidth', 1.8, ...
     'Color', [0.00 0.45 0.74], 'MarkerFaceColor', [0.00 0.45 0.74]);
grid on; box on;
xlabel('Uncalibrated plant drift [%]'); ylabel('Closed-loop RMSE');
title('(b) Zero-retraining drift robustness (AI frozen)');
legend({'Branch 2: end-to-end AI', 'Branch 3: layered architecture'}, ...
       'Location', 'northwest');
set(gca, 'FontSize', 9);

% ---- Panel C ------------------------------------------------------------
subplot(1,3,3);
groups = {'Inference rate [Hz]', 'MAC/s', 'Memory [B]'};
mono_v = [C.monolithic.inference_hz, C.monolithic.macs_per_s, C.monolithic.mem_bytes];
lay_v  = [C.layered.clf_hz, C.layered.macs_per_s, C.layered.mem_bytes];
y = [mono_v; lay_v]';
hb = bar(log10(max(y, 1)), 'grouped');
hb(1).FaceColor = [0.85 0.33 0.10];
hb(2).FaceColor = [0.00 0.45 0.74];
set(gca, 'XTickLabel', groups, 'FontSize', 9);
ylabel('log10(value)'); grid on; box on;
title(sprintf('(c) Compute %.0fx lower, memory %.0fx lower', ...
              C.compute_ratio, C.memory_ratio));
legend({'Monolithic 1 kHz end-to-end policy', '10 Hz classifier + 1 kHz C99 core'}, ...
       'Location', 'northwest');

% ---- export -------------------------------------------------------------
if ~exist(opts.assets_dir, 'dir')
    mkdir(opts.assets_dir);
end
path = fullfile(opts.assets_dir, 'efficiency_and_generalization.png');
exportgraphics(fig, path, 'Resolution', 240);
close(fig);
end
