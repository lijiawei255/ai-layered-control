function E = run_efficiency_analysis(profile, opts)
%RUN_EFFICIENCY_ANALYSIS  Data / drift / compute-efficiency study of the
%   layered architecture versus an end-to-end continuous policy.
%
%   E = RUN_EFFICIENCY_ANALYSIS()                 % industrial profile
%   E = RUN_EFFICIENCY_ANALYSIS('edge_vla')
%   E = RUN_EFFICIENCY_ANALYSIS(P, OPTS)
%
%   Everything below is closed-form linear algebra: no GPU, no Deep Learning
%   Toolbox, no Statistics and Machine Learning Toolbox, no training loop.
%   The whole study (including the drift sweep) runs in a few seconds on a
%   plain CPU laptop.
%
%   Panel A - sample complexity.  Both learners see the same normalized macro
%     features and are trained from the same calibration pool:
%       (i)  the coarse 3-class regime classifier (nearest class mean, i.e.
%            the only thing the 10 Hz AI has to learn);
%       (ii) a continuous damping-policy regressor (ridge on a degree-8
%            monomial basis, 495 terms - a stand-in for a dense policy head;
%            a real dense head has 1e4..1e6 weights, so this comparison is if
%            anything conservative).
%     They are judged by their DEPLOYMENT action on held-out calibration runs:
%     the correct regime for the classifier, and damping within 8 % of optimal
%     for the continuous policy.
%
%   Panel B - zero-retraining drift robustness.  The full 4-stage engine is
%     re-run for a sweep of uncalibrated plant drift (0 .. +50 %) with the AI
%     frozen.  Branch 2 (continuous policy, no integral action, no abstention)
%     degrades quickly; Branch 3 (coarse bank + integral + gatekeeper) stays
%     flat.
%
%   Panel C - compute and memory footprint.  Analytic MAC/s and bytes for a
%     monolithic 1 kHz end-to-end policy versus the 10 Hz coarse classifier
%     plus the 1 kHz deterministic core.
%
%   OPTS fields (all optional): .seed, .n_sweep, .n_seeds, .drift_sweep,
%   .make_figure, .assets_dir
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

E = struct('profile', P, 'opts', opts);

% ========================================================================
% Panel A - sample complexity of the deployment action
% ========================================================================
rng(opts.seed + 101, 'twister');
n_pool = 4000;
n_val  = 400;

d_pool = 0.5*rand(n_pool,1);                    % uncalibrated drift fraction
b_pool = double(rand(n_pool,1) < 0.30);         % sharp-boundary flag
X_pool = local_features(d_pool, b_pool, 0.10);
y_pool = 1 + 0.90*d_pool + 0.22*b_pool;         % optimal damping ratio

% The coarse regime label is defined on the OPTIMAL damping, not on the raw
% drift: a sharp boundary at low drift also needs the damped bank.
lab_pool = 1 + double(y_pool >= 1.20) + double(y_pool >= 1.50);

d_val = 0.5*rand(n_val,1);
b_val = double(rand(n_val,1) < 0.30);
X_val = local_features(d_val, b_val, 0.10);
y_val = 1 + 0.90*d_val + 0.22*b_val;
lab_val = 1 + double(y_val >= 1.20) + double(y_val >= 1.50);

class_gain = [1.05 1.35 1.65];  % discrete damping ratio per coarse class
degree     = 8;                 % continuous-policy head: 495 basis terms
tol_gain   = 0.08;              % deployment tolerance on the damping

% The polynomial basis only depends on the (fixed) feature pools, so build it
% once: the sample-complexity sweep then only solves 495x495 systems.
fmu = mean(X_pool, 1);
fsg = std(X_pool, 0, 1) + 1e-9;
Phi_pool = local_poly_basis((X_pool - fmu)./fsg, degree);
Phi_val  = local_poly_basis((X_val  - fmu)./fsg, degree);
n_terms  = size(Phi_val, 2);

n_list = opts.n_sweep(:)';
n_n    = numel(n_list);
acc_cls  = zeros(n_n,1);
acc_gain = zeros(n_n,1);
err_gain = zeros(n_n,1);

for k = 1:n_n
    N = min(n_list(k), n_pool);
    a_c = 0; a_g = 0; e_g = 0;
    for s = 1:opts.n_seeds
        idx = randperm(n_pool, N);
        X = X_pool(idx,:); y = y_pool(idx); lab = lab_pool(idx);

        % ---- learner 1: coarse 3-class nearest-mean classifier ----------
        mu = zeros(3, size(X,2));
        for c = 1:3
            sel = (lab == c);
            if any(sel)
                mu(c,:) = mean(X(sel,:), 1);
            else
                mu(c,:) = X(1,:);      % empty class fallback
            end
        end
        dist = sqrt(sum((reshape(X_val,n_val,1,4) - reshape(mu,1,3,4)).^2, 3));
        [~, cls] = min(dist, [], 2);
        a_c = a_c + mean(cls == lab_val);

        % ---- learner 2: continuous policy on a polynomial head ----------
        Phi = Phi_pool(idx,:);
        lam = 1e-6*max(trace(Phi'*Phi)/max(N,1), 1e-12);
        W   = (Phi'*Phi + lam*eye(n_terms)) \ (Phi'*y);

        pred = Phi_val*W;
        rel  = abs(pred - y_val)./y_val;
        a_g  = a_g + mean(rel < tol_gain);
        e_g  = e_g + mean(rel);
    end
    acc_cls(k)  = 100*a_c/opts.n_seeds;
    acc_gain(k) = 100*a_g/opts.n_seeds;
    err_gain(k) = e_g/opts.n_seeds;
end

% Closed-loop cross-check: the quantised coarse bank must also keep the loop
% inside the tracking bar at every test operating point (this is the same
% success_factor used by the 4-stage benchmark).
d_test = [0.10 0.18 0.25 0.32 0.38 0.42 0.48];
b_test = ones(1, numel(d_test));
ideal_rmse = zeros(1, numel(d_test));
kd_grid = [0.90 1.20 1.60];
for c = 1:numel(d_test)
    r = zeros(1, numel(kd_grid));
    for g = 1:numel(kd_grid)
        r(g) = local_closed_loop_rmse(P, d_test(c), b_test(c), kd_grid(g));
    end
    ideal_rmse(c) = min(r);
end
loop_ok = zeros(1, numel(d_test));
for c = 1:numel(d_test)
    kd_opt = 1 + 0.90*d_test(c) + 0.22*b_test(c);
    kd_q = class_gain(1 + double(kd_opt >= 1.20) + double(kd_opt >= 1.50));
    rmse = local_closed_loop_rmse(P, d_test(c), b_test(c), kd_q);
    loop_ok(c) = rmse < P.metrics.success_factor*ideal_rmse(c);
end

E.panelA = struct('n', n_list, 'classifier_acc_pct', acc_cls, ...
                  'continuous_gain_acc_pct', acc_gain, ...
                  'continuous_mean_gain_err', err_gain, ...
                  'degree', degree, 'basis_terms', n_terms, ...
                  'tol_gain', tol_gain, 'class_gain', class_gain, ...
                  'closed_loop_bank_ok_pct', 100*mean(loop_ok));

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
% Assertion 4 - coarse classification is saturated with a few tens of
% samples, the continuous policy still needs more than a thousand.
% ========================================================================
k50   = find(n_list <= 50, 1, 'last');    if isempty(k50),   k50 = 1;  end
k500  = find(n_list <= 500, 1, 'last');   if isempty(k500),  k500 = 1; end
k1000 = find(n_list <= 1000, 1, 'last');  if isempty(k1000), k1000 = 1; end
k2000 = find(n_list <= 2000, 1, 'last');  if isempty(k2000), k2000 = n_n; end

ceil_c = max(acc_cls);
E.assert4 = struct();
E.assert4.coarse_at_50    = acc_cls(k50);
E.assert4.coarse_ceiling  = ceil_c;
E.assert4.regressor_at_500  = acc_gain(k500);
E.assert4.regressor_at_1000 = acc_gain(k1000);
E.assert4.regressor_at_2000 = acc_gain(k2000);
E.assert4.compute_ratio   = E.panelC.compute_ratio;
E.assert4.memory_ratio    = E.panelC.memory_ratio;
E.assert4.closed_loop_bank_ok = E.panelA.closed_loop_bank_ok_pct;
E.assert4.pass = (acc_cls(k50) >= 0.90*ceil_c) && ...
                 (acc_gain(k500) < 90) && ...
                 (acc_gain(k1000) < 95) && ...
                 (acc_gain(k2000) >= 95) && ...
                 (E.panelA.closed_loop_bank_ok_pct == 100) && ...
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
%  Macro features shared with the 4-stage benchmark
% =========================================================================
function X = local_features(d, b, noise)
% Normalized macro features [error ratio, effort ratio, slow residual,
% velocity ratio] as a function of the uncalibrated drift d and the sharp
% boundary flag b.  Same 4-D feature space the closed-loop benchmark computes
% from the plant; the synthetic operating-point model keeps the study fast.
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
%  Closed-loop evaluation of one damping ratio (cross-check only)
% =========================================================================
function rmse = local_closed_loop_rmse(P, d, b, kd_ratio)
dt  = 0.002;
T   = 2.40;
N   = round(T/dt);
t   = (0:N-1)'*dt;

dw = 1.20; mv = 1.20; A = P.ref.amp;
tau = min(max((t - dw)/mv, 0), 1);
p   = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
r   = A + (-2*A)*p;                     % move from +A down to -A
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

Kp = P.ctrl.Kp; Kd = P.ctrl.Kd*kd_ratio; Ki = P.ctrl.Ki;
u_max = P.act.u_max; dslew = P.act.slew_max*dt;
m = P.b2.model;
x = r(1); xd = 0; u_app = 0; integ = 0;
win = P.gate.integ_limit;
acc = 0;
for k = 1:N
    e  = r(k)  - x;
    ed = rd(k) - xd;
    u_ff = m.J*rdd(k) + m.b*rd(k) + m.k*r(k) + ...
           local_friction(rd(k), m.fc, m.fs, m.vs, m.fvisc);
    u_raw = Kp*e + Kd*ed + Ki*integ + u_ff;
    if Ki > 0
        integ = integ + dt*e + (u_app - u_raw)*dt/(Ki*P.gate.windup_tau_s);
        integ = min(max(integ, -win), win);
    end
    u = min(max(u_raw, -u_max), u_max);
    du = min(max(u - u_app, -dslew), dslew);
    u = u_app + du;
    F = local_friction(xd, fc, fs, vs, fvv);
    acc_dyn = (u - bb*xd - kk*x - F)/J;
    xd = xd + dt*acc_dyn;
    x  = x + dt*xd;
    acc = acc + (x - r(k))^2;
    u_app = u;
    if ~isfinite(x)
        acc = 1e6*N;
        break;
    end
end
rmse = sqrt(acc/N);
end

function F = local_friction(v, fc, fs, vs, fvisc)
veps = max(vs*0.02, 1e-9);
sg = tanh(v/veps);
F = fvisc*v + fc*sg + (fs - fc)*exp(-abs(v)/vs)*sg;
end

% =========================================================================
%  Figure 2
% =========================================================================
function path = local_plot(E, P, opts)
A = E.panelA; B = E.panelB; C = E.panelC;
fig = figure('Visible', 'off', 'Color', 'w', ...
             'Position', [100 100 1500 430]);

% ---- Panel A ------------------------------------------------------------
subplot(1,3,1);
semilogx(A.n, A.classifier_acc_pct, '-o', 'LineWidth', 1.8, ...
         'Color', [0.00 0.45 0.74], 'MarkerFaceColor', [0.00 0.45 0.74]); hold on;
semilogx(A.n, A.continuous_gain_acc_pct, '-s', 'LineWidth', 1.8, ...
         'Color', [0.85 0.33 0.10], 'MarkerFaceColor', [0.85 0.33 0.10]);
grid on; box on; xlim([min(A.n) max(A.n)]); ylim([-2 102]);
xlabel('Calibration samples N'); ylabel('Deployment action accuracy [%]');
title('(a) Sample complexity: coarse regime vs continuous policy');
legend({'Coarse 3-class regime classifier', ...
        sprintf('Continuous policy (damping within %.0f %%)', 100*A.tol_gain)}, ...
        'Location', 'southeast');
set(gca, 'FontSize', 9);

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


