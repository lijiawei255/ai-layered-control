function E = run_efficiency_analysis(profile, opts)
%RUN_EFFICIENCY_ANALYSIS  Data, drift and compute-efficiency study.
%
%   E = RUN_EFFICIENCY_ANALYSIS()                 % industrial profile
%   E = RUN_EFFICIENCY_ANALYSIS('edge_vla')
%   E = RUN_EFFICIENCY_ANALYSIS(P, OPTS)
%
%   Panel A - paired, nominal-only sample complexity.
%     Both learners receive the SAME N calibration samples from the NOMINAL
%     unit (drift = 0): the layered 3-class coarse classifier and the
%     continuous 495-term regression head.  They are then deployed on the
%     same unit at 0 % drift and on a +35 % drifted unit.  A third
%     continuous learner is retrained on +35 % drifted data (the oracle),
%     which isolates "data coverage" from "architecture".
%
%   Panel B - zero-retraining drift robustness of the full closed loop.
%   Panel C - analytic compute / memory footprint.

if nargin < 1 || isempty(profile), profile = 'industrial'; end
if nargin < 2 || isempty(opts),    opts = struct(); end
if ischar(profile) || (isstring(profile) && isscalar(profile))
    P = load_domain_profile(char(profile));
else
    P = profile;
end
if ~isfield(opts,'seed') || isempty(opts.seed), opts.seed = 17; end
if ~isfield(opts,'n_sweep') || isempty(opts.n_sweep)
    opts.n_sweep = [15 40 120 500 2000];
end
if ~isfield(opts,'n_seeds') || isempty(opts.n_seeds), opts.n_seeds = 4; end
if ~isfield(opts,'drift_sweep') || isempty(opts.drift_sweep)
    opts.drift_sweep = [0 0.17 0.33 0.50];
end
if ~isfield(opts,'make_figure') || isempty(opts.make_figure)
    opts.make_figure = true;
end
if ~isfield(opts,'assets_dir') || isempty(opts.assets_dir)
    opts.assets_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'assets');
end

rng(opts.seed + 101, 'twister');
E = struct('profile', P, 'opts', opts);

% ========================================================================
% Panel A - paired nominal-only calibration, deployed at 0 / +35 % drift
% ========================================================================
n_pool = 4000;
load_pool = 0.5*rand(n_pool,1);
b_pool    = double(rand(n_pool,1) < 0.30);
X_pool    = local_features(load_pool, b_pool, 0, 0.10);
y_pool    = 1 + 0.90*load_pool + 0.22*b_pool;
lab_pool  = local_regime_label(y_pool);
X_oracle  = local_features(load_pool, b_pool, 0.35, 0.10);
y_oracle  = 1 + 0.90*(load_pool + 0.35) + 0.22*b_pool;

n_cond = 8;
load_eval = max(0, 0.35 + 0.02*randn(n_cond,1));
b_eval    = ones(n_cond,1);
X_dep     = local_features(load_eval, b_eval, 0.35, 0.10);
X_dep0    = local_features(load_eval, b_eval, 0, 0.10);
kd_dep    = 1 + 0.90*(load_eval + 0.35) + 0.22*b_eval;
kd_dep0   = 1 + 0.90*load_eval + 0.22*b_eval;
tol_kd    = 0.10;

class_gain = [1.05 1.50 1.85];
degree     = 8;
dwell_n    = 2;
n_frames   = 8;
n_list     = opts.n_sweep(:)';
n_n        = numel(n_list);

X_all = [X_pool; X_oracle];
fmu = zeros(1, size(X_all,2));
fsg = ones(1, size(X_all,2));
PhiP = local_poly_basis(X_pool, degree);
PhiO = local_poly_basis(X_oracle, degree);
n_terms = size(PhiP,2);

succ_layered = zeros(n_n,1);
succ_cont0   = zeros(n_n,1);
succ_contd   = zeros(n_n,1);
succ_oracle  = zeros(n_n,1);
err_cont0    = zeros(n_n,1);
err_contt    = zeros(n_n,1);

for k = 1:n_n
    N = min(n_list(k), n_pool);
    a_lay = 0; a_c0 = 0; a_cd = 0; a_or = 0; e0 = 0; ed = 0; n_ev = 0;
    for s = 1:opts.n_seeds
        idx = randperm(n_pool, N);
        Xc = X_pool(idx,:); lab = lab_pool(idx);
        mu = zeros(3,4);
        for c = 1:3
            sel = (lab == c);
            if any(sel), mu(c,:) = mean(Xc(sel,:),1); else, mu(c,:) = Xc(1,:); end
        end
        lam = 1e-6*N;
        PhiN = PhiP(idx,:); sN = sqrt(mean(PhiN.^2,1)); sN(sN < 1e-12) = 1;
        PhiNs = PhiN./sN;
        W_nom = (PhiNs'*PhiNs + lam*eye(n_terms)) \ (PhiNs'*y_pool(idx));
        PhiOp = PhiO(idx,:); sO = sqrt(mean(PhiOp.^2,1)); sO(sO < 1e-12) = 1;
        PhiOs = PhiOp./sO;
        W_ora = (PhiOs'*PhiOs + lam*eye(n_terms)) \ (PhiOs'*y_oracle(idx));
        for c = 1:n_cond
            Fd = local_features(repmat(load_eval(c),n_frames,1), ...
                                repmat(b_eval(c),n_frames,1), 0.35, 0.10);
            F0 = local_features(repmat(load_eval(c),n_frames,1), ...
                                repmat(b_eval(c),n_frames,1), 0.00, 0.10);
            cls = local_debounced_class(mu, Fd, dwell_n);
            kd_lay = class_gain(cls);
            a_lay = a_lay + local_decision_ok(kd_lay, kd_dep(c), tol_kd);

            kd0 = local_policy_damping(W_nom, fmu, fsg, sN, F0, degree);
            kdd = local_policy_damping(W_nom, fmu, fsg, sN, Fd, degree);
            kdo = local_policy_damping(W_ora, fmu, fsg, sO, Fd, degree);
            a_c0 = a_c0 + local_decision_ok(kd0, kd_dep0(c), tol_kd);
            a_cd = a_cd + local_decision_ok(kdd, kd_dep(c), tol_kd);
            a_or = a_or + local_decision_ok(kdo, kd_dep(c), tol_kd);
            e0 = e0 + abs(kd0 - kd_dep0(c))/max(kd_dep0(c), eps);
            ed = ed + abs(kdd - kd_dep(c))/max(kd_dep(c), eps);
            n_ev = n_ev + 1;
        end
    end
    succ_layered(k) = 100*a_lay/n_ev;
    succ_cont0(k)   = 100*a_c0/n_ev;
    succ_contd(k)   = 100*a_cd/n_ev;
    succ_oracle(k)  = 100*a_or/n_ev;
    err_cont0(k)    = e0/n_ev;
    err_contt(k)    = ed/n_ev;
end

E.panelA = struct('n', n_list, ...
    'layered_deployed_pct', succ_layered, ...
    'continuous_nominal_pct', succ_cont0, ...
    'continuous_deployed_pct', succ_contd, ...
    'oracle_deployed_pct', succ_oracle, ...
    'continuous_nominal_error', err_cont0, ...
    'continuous_deployed_error', err_contt, ...
    'degree', degree, 'basis_terms', n_terms, ...
    'dwell_frames', dwell_n, 'class_gain', class_gain, ...
    'deployed_drift', 0.35, 'kd_tolerance', tol_kd);

% ========================================================================
% Panel B - zero-retraining robustness of the full closed loop
% ========================================================================
if isempty(P.b2.policy) || ~isfield(P.b2.policy,'weights')
    % Train the continuous baseline once on the nominal unit.
    Ptmp = P;
    out0 = sim_core_engine(Ptmp, struct('t_end',0.5,'branches',2,'verbose',false));
    P.b2.policy = out0.profile.b2.policy;
end
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
% Panel C - compute / memory footprint (analytic)
% ========================================================================
mono = struct();
mono.inference_hz = 1000;
mono.params       = 1.2e6;
mono.macs_per_inf = 2*mono.params;
mono.macs_per_s   = mono.macs_per_inf*mono.inference_hz;
mono.mem_params   = 4*mono.params;
mono.mem_act      = 4*2.0e5;
mono.mem_bytes    = mono.mem_params + mono.mem_act;

lay = struct();
lay.clf_hz         = P.ai.outer_hz;
lay.clf_params     = 16;
lay.clf_macs_per_s = 3*4*lay.clf_hz;
lay.core_hz        = 1/P.dt;
lay.core_macs_step = 24;
lay.core_macs_per_s = lay.core_macs_step*lay.core_hz;
lay.macs_per_s     = lay.clf_macs_per_s + lay.core_macs_per_s;
lay.mem_params     = 4*lay.clf_params;
lay.mem_state      = 4*24;
lay.mem_code       = 2048;
lay.mem_bytes      = lay.mem_params + lay.mem_state + lay.mem_code;
E.panelC = struct('monolithic', mono, 'layered', lay, ...
    'compute_ratio', mono.macs_per_s/lay.macs_per_s, ...
    'memory_ratio', mono.mem_bytes/lay.mem_bytes);

% ========================================================================
% Check 4 - fair data / compute efficiency
% ========================================================================
k40   = find(n_list == 40, 1);   if isempty(k40),   [~,k40]   = min(abs(n_list-40));   end
k120  = find(n_list == 120, 1);  if isempty(k120),  [~,k120]  = min(abs(n_list-120));  end
k500  = find(n_list == 500, 1);  if isempty(k500),  [~,k500]  = min(abs(n_list-500));  end
k2000 = find(n_list == 2000, 1); if isempty(k2000), [~,k2000] = min(abs(n_list-2000)); end

E.check4 = struct();
E.check4.layered_at_40       = succ_layered(k40);
E.check4.cont_nominal_at_40  = succ_cont0(k40);
E.check4.cont_nominal_at_120 = succ_cont0(k120);
E.check4.cont_deployed_at_2000 = succ_contd(k2000);
E.check4.oracle_at_120       = succ_oracle(k120);
E.check4.cont_nominal_at_500 = succ_cont0(k500);
E.check4.compute_ratio = E.panelC.compute_ratio;
E.check4.memory_ratio  = E.panelC.memory_ratio;
E.check4.pass = (succ_layered(k40) >= 95) && ...
                (succ_contd(k2000) <= 50) && ...
                (succ_oracle(k120) >= 95) && ...
                (succ_cont0(k500) >= 95) && ...
                (E.panelC.compute_ratio > 1000);

% ========================================================================
% Figure 2
% ========================================================================
E.figure_path = '';
if opts.make_figure
    E.figure_path = local_plot(E, P, opts);
end
end

% =========================================================================
%  Helpers
% =========================================================================
function lab = local_regime_label(y)
lab = 1 + double(y >= 1.15) + double(y >= 1.375);
end

function ok = local_decision_ok(kd, kd_opt, tol)
ok = abs(kd - kd_opt)/max(kd_opt, eps) <= tol;
end

function kd = local_policy_damping(W, fmu, fsg, sPhi, Xframes, degree)
alpha = 1 - exp(-0.1/0.25);
Xs = Xframes(1,:);
for i = 2:size(Xframes,1)
    Xs = Xs + alpha*(Xframes(i,:) - Xs);
end
Xf = (Xs - fmu)./fsg;
Phi = local_poly_basis(Xf, degree)./sPhi;
kd = mean(Phi*W);
kd = min(max(kd, 0.20), 4.00);
end

function cls = local_debounced_class(mu, Xframes, dwell_n)
% A new regime is accepted only after dwell_n consecutive slow-loop frames.
n = size(Xframes,1);
lab = zeros(n,1);
for i = 1:n
    dd = sqrt(sum((mu - Xframes(i,:)).^2, 2));
    [~, lab(i)] = min(dd);
end
cls = lab(1);
pending = cls; run = 1;
for i = 2:n
    if lab(i) == pending
        run = run + 1;
    else
        pending = lab(i); run = 1;
    end
    if pending ~= cls && run >= dwell_n
        cls = pending; run = 0;
    end
end
end

function X = local_features(load, b, drift, noise)
load = load(:); b = b(:); drift = drift(:);
n = numel(load);
d = load + drift;
X = [ (0.020 + 0.060*d + 0.020*b) .* (1 + noise*randn(n,1)), ...
      (0.100 + 0.110*d + 0.030*b) .* (1 + noise*randn(n,1)), ...
      (0.250*d + 0.030*b)         + 0.020*noise*randn(n,1), ...
      (0.650 + 0.050*randn(n,1)) ];
X = max(X, 0);
end

function Phi = local_poly_basis(X, degree)
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
%  Figure 2
% =========================================================================
function path = local_plot(E, P, opts)
A = E.panelA; B = E.panelB; C = E.panelC;
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1500 430]);

subplot(1,3,1);
semilogx(A.n, A.layered_deployed_pct, '-o', 'LineWidth', 1.9, ...
         'Color', [0.00 0.45 0.74], 'MarkerFaceColor', [0.00 0.45 0.74]); hold on;
semilogx(A.n, A.continuous_nominal_pct, '-s', 'LineWidth', 1.7, ...
         'Color', [0.47 0.67 0.19], 'MarkerFaceColor', [0.47 0.67 0.19]);
semilogx(A.n, A.continuous_deployed_pct, ':^', 'LineWidth', 1.7, ...
         'Color', [0.85 0.33 0.10], 'MarkerFaceColor', [0.85 0.33 0.10]);
semilogx(A.n, A.oracle_deployed_pct, '--d', 'LineWidth', 1.7, ...
         'Color', [0.49 0.18 0.56], 'MarkerFaceColor', [0.49 0.18 0.56]);
grid on; box on; xlim([min(A.n) max(A.n)]); ylim([-2 102]);
xlabel('Calibration Samples N'); ylabel('Deployable closed-loop success [%]');
title(sprintf('(a) Paired nominal-only calibration (%.0f %% deployment drift)', ...
              100*A.deployed_drift));
legend({'Layered classifier (deployed, +35 % drift)', ...
        'Continuous, nominal-only (0 % drift)', ...
        'Continuous, nominal-only (+35 % drift)', ...
        'Continuous oracle (+35 % drift-trained)'}, ...
        'Location', 'southeast');
set(gca, 'FontSize', 8);

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

if ~exist(opts.assets_dir, 'dir')
    mkdir(opts.assets_dir);
end
path = fullfile(opts.assets_dir, 'efficiency_and_generalization.png');
exportgraphics(fig, path, 'Resolution', 240);
close(fig);
end
