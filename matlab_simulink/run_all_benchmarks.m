%RUN_ALL_BENCHMARKS  One-click entry point for the whole reference benchmark.
%
%   Run it from anywhere:
%       matlab -batch "run('matlab_simulink/run_all_benchmarks.m')"
%
%   What it does, in order:
%     1. builds matlab_simulink/ai_layered_control_bench.slx (built-in blocks
%        only) and smoke-tests it for 0.1 s, if Simulink is installed and the
%        model is not open with unsaved edits;
%     2. runs the 40 s four-stage benchmark of the industrial reference
%        profile and verifies checks 1-3;
%     3. runs the paired, nominal-only data / drift / compute study and
%        verifies check 4;
%     4. runs the three domain presets;
%     5. runs the stage-forced mode-necessity ablation (check 5) and the
%        OOD probe cross-checks;
%     6. exports the three publication figures into assets/ at 240 dpi and
%        prints a Markdown metrics table to stdout.
%
%   Everything is plain MATLAB + Simulink: no toolbox, no GPU, no MEX.

here  = fileparts(mfilename('fullpath'));
root  = fullfile(here, '..');
addpath(here);
assets = fullfile(root, 'assets');
if ~exist(assets, 'dir')
    mkdir(assets);
end
t_start = tic;
fprintf('=== ai-layered-control reference benchmark (v1.1.0) ===\n');

% ------------------------------------------------------------------------
% 1. Simulink model (optional) + smoke test
% ------------------------------------------------------------------------
mdl_path = fullfile(here, 'ai_layered_control_bench.slx');
[mdl, mdl_status] = build_simulink_model(mdl_path);
t_model = toc(t_start);
switch mdl_status
    case 'built'
        load_system(mdl);
        set_param(mdl, 'StopTime', '0.1');
        sim(mdl);
        close_system(mdl, 0);
        fprintf('[1/5] Simulink model built and smoke-tested: matlab_simulink/%s.slx\n', mdl);
    case 'skipped_dirty'
        fprintf('[1/5] Simulink model is open with unsaved edits: skipped rebuild; numeric benchmark continues.\n');
    otherwise
        fprintf('[1/5] Simulink not installed: skipped .slx build; numeric benchmark continues.\n');
end

% ------------------------------------------------------------------------
% 2. Four-stage benchmark (industrial reference profile)
% ------------------------------------------------------------------------
P    = load_domain_profile('industrial');
P    = calibrate_coarse_classifier(P);
out  = sim_core_engine(P, struct('verbose', false));
P    = out.profile;                  % calibrated P + trained B2 policy
A    = out.metrics.checks;
fig1 = fullfile(assets, 'benchmark_4stages_showdown.png');
local_plot_stages(out, P, fig1);
fprintf('[2/5] 40 s four-stage benchmark: figure -> assets/benchmark_4stages_showdown.png\n');
t_bench = toc(t_start) - t_model;

% ------------------------------------------------------------------------
% 3. Paired nominal-only data / drift / compute study
% ------------------------------------------------------------------------
E  = run_efficiency_analysis(P, struct('assets_dir', assets, 'make_figure', true));
A4 = E.check4;
fprintf('[3/5] efficiency study: figure -> assets/efficiency_and_generalization.png\n');
t_eff = toc(t_start) - t_model - t_bench;

% ------------------------------------------------------------------------
% 4. Stage-forced mode-necessity ablation + OOD probes (check 5 / check 3)
% ------------------------------------------------------------------------
fixed = cell(1,3);
for r = 1:3
    fixed{r} = sim_core_engine(P, struct('branches', 3, 'fixed_regime', r, ...
                                         'verbose', false));
end
oracle = sim_core_engine(P, struct('branches', 3, 'stage_regime', [1 2 3 4], ...
                                   'verbose', false));
actual = out;
s123 = @(M) sqrt(mean(M(1:3).^2));
s40  = @(M) sqrt(mean(M.^2));
fixed_s123 = [s123(fixed{1}.metrics.rmse(3,:)), ...
              s123(fixed{2}.metrics.rmse(3,:)), ...
              s123(fixed{3}.metrics.rmse(3,:))];
fixed_40   = [s40(fixed{1}.metrics.rmse(3,:)), ...
              s40(fixed{2}.metrics.rmse(3,:)), ...
              s40(fixed{3}.metrics.rmse(3,:))];
oracle_s123 = s123(oracle.metrics.rmse(3,:));
oracle_40   = s40(oracle.metrics.rmse(3,:));
actual_s123 = s123(actual.metrics.rmse(3,:));
actual_40   = s40(actual.metrics.rmse(3,:));
A5 = struct();
A5.fixed_s123 = fixed_s123;
A5.fixed_40 = fixed_40;
A5.oracle_s123 = oracle_s123;
A5.oracle_40 = oracle_40;
A5.actual_s123 = actual_s123;
A5.actual_40 = actual_40;
A5.check5 = (oracle_s123 < 0.8*min(fixed_s123)) && ...
            (oracle_40 < min(fixed_40));

% OOD probes: resonance-only, spike-only, and S1-S3 false-veto count.
Pr = P; Pr.ood.t_start = 1.0; Pr.ood.spike_amp = 0; Pr.ood.spike_prob = 0;
out_r = sim_core_engine(Pr, struct('t_end', 6, 'branches', 3, ...
    'ood_spikes', false, 'ood_resonance', true, 'edge_on', false, 'verbose', false));
Ps = P; Ps.ood.t_start = 1.0; Ps.ood.res_kq = 0; Ps.ood.res_ky = 0; Ps.ood.res_kick = 0;
out_s = sim_core_engine(Ps, struct('t_end', 6, 'branches', 3, ...
    'ood_spikes', true, 'ood_resonance', false, 'edge_on', false, 'verbose', false));
resonance_ms = out_r.metrics.veto_latency_ms;
spike_ms     = out_s.metrics.veto_latency_ms;
s123_ood     = sum(out.b3.gate(out.t < 30) == 2);
A.check3_probe = (resonance_ms < 10) && (spike_ms < 10) && (s123_ood == 0);
A.check3 = A.check3 && A.check3_probe;

fprintf('[4/5] mode-necessity ablation + OOD probes complete.\n');
t_ablate = toc(t_start) - t_model - t_bench - t_eff;

% ------------------------------------------------------------------------
% 5. The three domain presets
% ------------------------------------------------------------------------
profiles = {'industrial', 'edge_vla', 'neuromodulation'};
outs = cell(1, numel(profiles));
for i = 1:numel(profiles)
    Pi = load_domain_profile(profiles{i});
    Pi = calibrate_coarse_classifier(Pi);
    outs{i} = sim_core_engine(Pi, struct('verbose', false));
end
fig3 = fullfile(assets, 'domain_profiles_comparison.png');
local_plot_domains(outs, fig3);
fprintf('[5/5] cross-domain comparison: figure -> assets/domain_profiles_comparison.png\n');
t_dom = toc(t_start) - t_model - t_bench - t_eff - t_ablate;

% ------------------------------------------------------------------------
% Engineering checks + Markdown metrics table
% ------------------------------------------------------------------------
M = out.metrics;
fprintf('\n=== ENGINEERING CHECKS ===\n');
fprintf('[1] S2 drift robustness (B2 RMSE vs B3):      %s  (%.4f vs %.4f, ratio %.2f)\n', ...
    local_ok(A.check1), A.stage2_rmse_b2, A.stage2_rmse_b3, ...
    A.stage2_rmse_b2/max(A.stage2_rmse_b3, eps));
fprintf('[2] S3 latency decoupling (chatter / RMSE):   %s  (%.3f vs %.3f, ratio %.2f; RMSE %.4f vs %.4f)\n', ...
    local_ok(A.check2), A.stage3_chatter_b2, A.stage3_chatter_b3, ...
    A.stage3_chatter_b2/max(A.stage3_chatter_b3, eps), ...
    A.stage3_rmse_b2, A.stage3_rmse_b3);
fprintf('[3] S4 OOD abstention and envelope safety:    %s  (B2 breaches %d, B3 breaches %d, veto %.1f ms; spike %.1f ms / resonance %.1f ms; S1-S3 OOD ticks %d)\n', ...
    local_ok(A.check3), A.stage4_violations_b2, A.stage4_violations_b3, ...
    A.veto_latency_ms, spike_ms, resonance_ms, s123_ood);
fprintf('[4] Paired data and compute efficiency:      %s  (layered %.1f%% at N=40, continuous nominal-only %.1f%% at N=40 / %.1f%% at N=120, +35%% drift %.1f%% at N=2000, oracle %.1f%% at N=120, compute %.0fx)\n', ...
    local_ok(A4.pass), A4.layered_at_40, A4.cont_nominal_at_40, ...
    A4.cont_nominal_at_120, A4.cont_deployed_at_2000, ...
    A4.oracle_at_120, A4.compute_ratio);
fprintf('[5] Mode necessity (stage-forced oracle):     %s  (S1-S3 %.5f vs fixed %.5f/%.5f/%.5f; 40 s %.5f vs %.5f/%.5f/%.5f; deployed classifier S1-S3 %.5f, 40 s %.5f)\n', ...
    local_ok(A5.check5), A5.oracle_s123, fixed_s123(1), fixed_s123(2), fixed_s123(3), ...
    A5.oracle_40, fixed_40(1), fixed_40(2), fixed_40(3), ...
    A5.actual_s123, A5.actual_40);

fprintf('\n=== MODE-NECESSITY ABLATION (40 s, S1/S2/S3/S4 RMSE) ===\n');
fprintf('| controller | S1 | S2 | S3 | S4 | S1-S3 | 40 s | TV(u) |\n');
fprintf('| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n');
ab_names = {'Stage-forced oracle', 'Fixed R1', 'Fixed R2', 'Fixed R3'};
ab_outs  = {oracle, fixed{1}, fixed{2}, fixed{3}};
for i = 1:4
    Mi = ab_outs{i}.metrics;
    fprintf('| %s | %.4f | %.4f | %.4f | %.4f | %.5f | %.5f | %.1f |\n', ...
        ab_names{i}, Mi.rmse(3,1), Mi.rmse(3,2), Mi.rmse(3,3), Mi.rmse(3,4), ...
        s123(Mi.rmse(3,:)), s40(Mi.rmse(3,:)), sum(Mi.tv(3,:)));
end
fprintf('| deployed 10 Hz classifier | %.4f | %.4f | %.4f | %.4f | %.5f | %.5f | %.1f |\n', ...
    M.rmse(3,1), M.rmse(3,2), M.rmse(3,3), M.rmse(3,4), actual_s123, actual_40, sum(M.tv(3,:)));

fprintf('\n=== QUANTITATIVE METRICS (industrial reference profile) ===\n');
fprintf('| metric | Branch 1: fixed PID | Branch 2: end-to-end AI | Branch 3: layered |\n');
fprintf('| :--- | ---: | ---: | ---: |\n');
stages = P.stages.short;
for s = 1:4
    fprintf('| RMSE %s [%s] | %.4f | %.4f | %.4f |\n', stages{s}, P.units.state, ...
        M.rmse(1,s), M.rmse(2,s), M.rmse(3,s));
end
for s = 1:4
    fprintf('| chatter %s [norm.] | %.3f | %.3f | %.3f |\n', stages{s}, ...
        M.chatter(1,s), M.chatter(2,s), M.chatter(3,s));
end
for s = 1:4
    fprintf('| Control Effort TV(u) %s [%s] | %.1f | %.1f | %.1f |\n', stages{s}, P.units.input, ...
        M.tv(1,s), M.tv(2,s), M.tv(3,s));
end
for s = 1:4
    fprintf('| Safety Violation Ticks %s [ms] | %d | %d | %d |\n', stages{s}, ...
        M.violations(1,s), M.violations(2,s), M.violations(3,s));
end
fprintf('| total Control Effort TV(u) over 40 s [%s] | %.1f | %.1f | %.1f |\n', ...
    P.units.input, sum(M.tv(1,:)), sum(M.tv(2,:)), sum(M.tv(3,:)));
fprintf('| OOD veto latency | - | - | %.1f ms |\n', M.veto_latency_ms);
fprintf('| Calibration Samples N (classifier / continuous) | - | ~500 | <= 40 |\n');
fprintf('| compute load (MAC/s) | - | %.2e | %.2e |\n', ...
    E.panelC.monolithic.macs_per_s, E.panelC.layered.macs_per_s);
fprintf('| memory footprint (B) | - | %.2e | %.2e |\n', ...
    E.panelC.monolithic.mem_bytes, E.panelC.layered.mem_bytes);
fprintf('| compute / memory reduction | - | - | %.0fx / %.0fx |\n', ...
    E.panelC.compute_ratio, E.panelC.memory_ratio);
fprintf('> +35%% drift = J x1.35, b x1.25, k x1.10, fc x2.00, fs x2.40, vs x0.25, fvisc x1.20 (industrial).\n');

fprintf('\n=== CROSS-DOMAIN SYNTHETIC PRESET METRICS ===\n');
fprintf('| preset | S2 RMSE B2 | S2 RMSE B3 | S3 RMSE B2 | S3 RMSE B3 | S4 breaches B2 | S4 breaches B3 | Control Effort TV(u) B2 | Control Effort TV(u) B3 |\n');
fprintf('| :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n');
for i = 1:numel(profiles)
    Mi = outs{i}.metrics;
    fprintf('| `%s` (%s) | %.4f | %.4f | %.4f | %.4f | %d | %d | %.1f | %.1f |\n', ...
        outs{i}.profile.kind, outs{i}.profile.units.state, ...
        Mi.rmse(2,2), Mi.rmse(3,2), Mi.rmse(2,3), Mi.rmse(3,3), ...
        Mi.violations(2,4), Mi.violations(3,4), ...
        sum(Mi.tv(2,:)), sum(Mi.tv(3,:)));
end

fprintf('\nfigures written to: %s\n', assets);
fprintf('\n=== TIMING (wall clock, this MATLAB process) ===\n');
fprintf('  model build + smoke test      : %6.2f s\n', t_model);
fprintf('  four-stage benchmark + figure : %6.2f s\n', t_bench);
fprintf('  efficiency study + figure     : %6.2f s\n', t_eff);
fprintf('  ablation / probes             : %6.2f s\n', t_ablate);
fprintf('  three domain presets + figure : %6.2f s\n', t_dom);
fprintf('  total wall-clock time         : %6.2f s\n', toc(t_start));
if ~(A.check1 && A.check2 && A.check3 && A4.pass && A5.check5)
    error('run_all_benchmarks:check', ...
          'At least one engineering check failed - see the table above.');
end
fprintf('All five engineering checks passed.\n');

% =========================================================================
%  Figure 1 - four-stage showdown
% =========================================================================
function local_plot_stages(out, P, path)
t = out.t;
col1 = [0.55 0.55 0.55];
col2 = [0.85 0.33 0.10];
col3 = [0.00 0.45 0.74];
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1100 700]);

subplot(2,2,1);
plot(t, out.ref.r, 'k--', 'LineWidth', 1.0); hold on;
plot(t, out.b1.x, 'Color', col1, 'LineWidth', 1.0);
plot(t, out.b2.x, 'Color', col2, 'LineWidth', 1.0);
plot(t, out.b3.x, 'Color', col3, 'LineWidth', 1.2);
yline(P.safety.x_limit, 'r-', 'LineWidth', 1.2);
yline(-P.safety.x_limit, 'r-', 'LineWidth', 1.2);
grid on; box on; xlim([0 t(end)]);
xlabel('time [s]'); ylabel(sprintf('state [%s]', P.units.state));
title('(a) State tracking and hard safety envelope');
legend({'reference', 'Branch 1 fixed PID', 'Branch 2 end-to-end AI', ...
        'Branch 3 layered'}, 'Location', 'southwest');

subplot(2,2,2);
plot(t, out.b1.u, 'Color', col1, 'LineWidth', 0.8); hold on;
plot(t, out.b2.u, 'Color', col2, 'LineWidth', 0.8);
plot(t, out.b3.u, 'Color', col3, 'LineWidth', 1.0);
grid on; box on; xlim([0 t(end)]);
xlabel('time [s]'); ylabel(sprintf('command [%s]', P.units.input));
title('(b) Actuator command');
legend({'Branch 1', 'Branch 2', 'Branch 3'}, 'Location', 'southwest');

subplot(2,2,3);
yyaxis left;
plot(t, out.b3.res_fast, 'Color', [0.49 0.18 0.56], 'LineWidth', 0.9); hold on;
plot(t, out.b3.res_hf, 'Color', [0.00 0.45 0.74], 'LineWidth', 0.9);
yline(P.ai.max_residual, 'r--', 'LineWidth', 1.0);
ylabel('normalized residual'); ylim([0 2.5]);
yyaxis right;
plot(t, out.b3.ai_conf, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.0); hold on;
yline(P.ai.min_confidence, 'k:', 'LineWidth', 1.0);
ylabel('classifier confidence'); ylim([0 1]);
grid on; box on; xlim([0 t(end)]);
xlabel('time [s]');
title('(c) Spike / HF residual channels and AI confidence');
legend({'spike channel', 'HF resonance channel', 'OOD threshold', 'confidence'}, ...
       'Location', 'northeast');

subplot(2,2,4);
stairs(t, out.b3.regime, 'Color', col3, 'LineWidth', 1.2); hold on;
stairs(t, out.b3.gate, 'Color', col2, 'LineWidth', 0.9);
grid on; box on; xlim([0 t(end)]);
ylim([-0.2 4.5]);
xlabel('time [s]'); ylabel('index');
title('(d) Active regime and gate state timeline');
legend({'regime (1 nominal, 2 load, 3 boundary, 4 fallback)', ...
        'gate (0 normal, 1 hold, 2 OOD, 3 confidence, 4 envelope)'}, ...
        'Location', 'southwest');

if ~exist(fileparts(path), 'dir')
    mkdir(fileparts(path));
end
exportgraphics(fig, path, 'Resolution', 240);
close(fig);
end

% =========================================================================
%  Figure 3 - cross-domain trajectories
% =========================================================================
function local_plot_domains(outs, path)
col2 = [0.85 0.33 0.10];
col3 = [0.00 0.45 0.74];
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1100 640]);
titles = {'Industrial servo / WAAM', 'Edge VLA (10 Hz over 1 kHz)', ...
          'Closed-loop neuromodulation'};
for i = 1:numel(outs)
    o = outs{i}; P = o.profile;
    t = o.t;
    subplot(2, numel(outs), i);
    plot(t, o.ref.r, 'k--', 'LineWidth', 1.0); hold on;
    plot(t, o.b2.x, 'Color', col2, 'LineWidth', 0.9);
    plot(t, o.b3.x, 'Color', col3, 'LineWidth', 1.1);
    yline(P.safety.x_limit, 'r-', 'LineWidth', 1.0);
    yline(-P.safety.x_limit, 'r-', 'LineWidth', 1.0);
    grid on; box on; xlim([0 t(end)]);
    xlabel('time [s]'); ylabel(sprintf('state [%s]', P.units.state));
    title(sprintf('%s: state', titles{i}));
    if i == 1
        legend({'reference', 'Branch 2 end-to-end AI', 'Branch 3 layered'}, ...
               'Location', 'southwest');
    end

    subplot(2, numel(outs), numel(outs)+i);
    plot(t, o.b2.u, 'Color', col2, 'LineWidth', 0.8); hold on;
    plot(t, o.b3.u, 'Color', col3, 'LineWidth', 0.9);
    grid on; box on; xlim([0 t(end)]);
    xlabel('time [s]'); ylabel(sprintf('command [%s]', P.units.input));
    title(sprintf('%s: command', titles{i}));
end
exportgraphics(fig, path, 'Resolution', 240);
close(fig);
end

function s = local_ok(flag)
if flag
    s = 'PASS';
else
    s = 'FAIL';
end
end
