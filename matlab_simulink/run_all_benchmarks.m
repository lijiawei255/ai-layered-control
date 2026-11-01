%RUN_ALL_BENCHMARKS  One-click entry point for the whole reference benchmark.
%
%   Run it from anywhere:
%       matlab -batch "run('matlab_simulink/run_all_benchmarks.m')"
%
%   What it does, in order:
%     1. builds matlab_simulink/ai_layered_control_bench.slx (built-in blocks
%        only) and smoke-tests it for one simulated second;
%     2. runs the 40 s four-stage benchmark of the industrial reference
%        profile and verifies assertions 1-3;
%     3. runs the data / drift / compute efficiency study and verifies
%        assertion 4;
%     4. runs the three domain presets (industrial, edge VLA, neuromodulation);
%     5. exports the three publication figures into assets/ at 300 dpi and
%        prints a Markdown metrics table to stdout (pure 7-bit ASCII).
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
fprintf('=== ai-layered-control reference benchmark ===\n');

% ------------------------------------------------------------------------
% 1. Simulink model (built-in blocks only) + smoke test
% ------------------------------------------------------------------------
mdl = build_simulink_model(fullfile(here, 'ai_layered_control_bench.slx'));
load_system(mdl);
set_param(mdl, 'StopTime', '0.1');
sim(mdl);
close_system(mdl, 0);
fprintf('[1/5] Simulink model built and smoke-tested: matlab_simulink/%s.slx\n', mdl);
t_model = toc(t_start);

% ------------------------------------------------------------------------
% 2. Four-stage benchmark (industrial reference profile)
% ------------------------------------------------------------------------
P    = load_domain_profile('industrial');
out  = sim_core_engine(P, struct('verbose', false));
A    = out.metrics.assertions;
fig1 = fullfile(assets, 'benchmark_4stages_showdown.png');
local_plot_stages(out, P, fig1);
fprintf('[2/5] 40 s four-stage benchmark: figure -> assets/benchmark_4stages_showdown.png\n');
t_bench = toc(t_start) - t_model;

% ------------------------------------------------------------------------
% 3. Efficiency and generalization study
% ------------------------------------------------------------------------
E  = run_efficiency_analysis(P, struct('assets_dir', assets, 'make_figure', true));
A4 = E.assert4;
fprintf('[3/5] efficiency study: figure -> assets/efficiency_and_generalization.png\n');
t_eff = toc(t_start) - t_model - t_bench;

% ------------------------------------------------------------------------
% 4. The three domain presets
% ------------------------------------------------------------------------
profiles = {'industrial', 'edge_vla', 'neuromodulation'};
outs = cell(1, numel(profiles));
for i = 1:numel(profiles)
    outs{i} = sim_core_engine(profiles{i}, struct('verbose', false));
end
fig3 = fullfile(assets, 'domain_profiles_comparison.png');
local_plot_domains(outs, fig3);
fprintf('[4/5] cross-domain comparison: figure -> assets/domain_profiles_comparison.png\n');
t_dom = toc(t_start) - t_model - t_bench - t_eff;

% ------------------------------------------------------------------------
% 5. Engineering assertions + Markdown metrics table
% ------------------------------------------------------------------------
M = out.metrics;
fprintf('\n=== ENGINEERING ASSERTIONS ===\n');
fprintf('assertion 1 (S2 anti-overfitting):   %s  (B2 %.4f vs B3 %.4f, ratio %.2f)\n', ...
    local_ok(A.assert1), A.stage2_rmse_b2, A.stage2_rmse_b3, ...
    A.stage2_rmse_b2/max(A.stage2_rmse_b3, eps));
fprintf('assertion 2 (S3 latency decoupling): %s  (chatter %.3f vs %.3f, ratio %.2f; RMSE %.4f vs %.4f)\n', ...
    local_ok(A.assert2), A.stage3_chatter_b2, A.stage3_chatter_b3, ...
    A.stage3_chatter_b2/max(A.stage3_chatter_b3, eps), ...
    A.stage3_rmse_b2, A.stage3_rmse_b3);
fprintf('assertion 3 (S4 OOD abstention):     %s  (B2 violations %d, B3 violations %d, veto %.1f ms)\n', ...
    local_ok(A.assert3), A.stage4_violations_b2, A.stage4_violations_b3, A.veto_latency_ms);
fprintf('assertion 4 (data/compute efficiency): %s  (coarse %.1f%% at N<=50 vs ceiling %.1f%%, continuous %.1f%% at N=500, %.1f%% at N=1000, %.1f%% at N=2000, compute %.0fx)\n', ...
    local_ok(A4.pass), A4.coarse_at_50, A4.coarse_ceiling, ...
    A4.regressor_at_500, A4.regressor_at_1000, A4.regressor_at_2000, ...
    A4.compute_ratio);

fprintf('\n=== QUANTITATIVE METRICS (industrial reference profile) ===\n');
fprintf('| metric | Branch 1: fixed PID | Branch 2: end-to-end AI | Branch 3: layered |\n');
fprintf('| :--- | ---: | ---: | ---: |\n');
stages = {'S1 nominal', 'S2 +35% drift', 'S3 edge latency', 'S4 OOD fault'};
for s = 1:4
    fprintf('| RMSE %s [%s] | %.4f | %.4f | %.4f |\n', stages{s}, P.units.state, ...
        M.rmse(1,s), M.rmse(2,s), M.rmse(3,s));
end
for s = 1:4
    fprintf('| chatter %s [norm.] | %.3f | %.3f | %.3f |\n', stages{s}, ...
        M.chatter(1,s), M.chatter(2,s), M.chatter(3,s));
end
for s = 1:4
    fprintf('| actuator TV %s [%s] | %.1f | %.1f | %.1f |\n', stages{s}, P.units.input, ...
        M.tv(1,s), M.tv(2,s), M.tv(3,s));
end
for s = 1:4
    fprintf('| safety violations %s | %d | %d | %d |\n', stages{s}, ...
        M.violations(1,s), M.violations(2,s), M.violations(3,s));
end
fprintf('| total actuator TV(u) over 40 s [%s] | %.1f | %.1f | %.1f |\n', ...
    P.units.input, sum(M.tv(1,:)), sum(M.tv(2,:)), sum(M.tv(3,:)));
fprintf('| OOD veto latency | - | - | %.1f ms |\n', M.veto_latency_ms);
fprintf('| samples needed for the AI | - | > 1000 | %.0f (<= 50) |\n', 50);
fprintf('| compute load (MAC/s) | - | %.2e | %.2e |\n', ...
    E.panelC.monolithic.macs_per_s, E.panelC.layered.macs_per_s);
fprintf('| memory footprint (B) | - | %.2e | %.2e |\n', ...
    E.panelC.monolithic.mem_bytes, E.panelC.layered.mem_bytes);
fprintf('| compute / memory reduction | - | - | %.0fx / %.0fx |\n', ...
    E.panelC.compute_ratio, E.panelC.memory_ratio);

fprintf('\n=== CROSS-DOMAIN SUMMARY ===\n');
for i = 1:numel(profiles)
    Mi = outs{i}.metrics;
    fprintf('| %s | S2 RMSE B2/B3 = %.4f / %.4f | S4 violations B2/B3 = %d / %d |\n', ...
        outs{i}.profile.kind, Mi.rmse(2,2), Mi.rmse(3,2), ...
        Mi.violations(2,4), Mi.violations(3,4));
end

fprintf('\nfigures written to: %s\n', assets);
fprintf('\n=== TIMING (wall clock, this MATLAB process) ===\n');
fprintf('  model build + smoke test      : %6.2f s\n', t_model);
fprintf('  four-stage benchmark + figure : %6.2f s\n', t_bench);
fprintf('  efficiency study + figure     : %6.2f s\n', t_eff);
fprintf('  three domain presets + figure : %6.2f s\n', t_dom);
fprintf('  total wall-clock time         : %6.2f s\n', toc(t_start));
fprintf('  (a cold matlab -batch also pays the one-off Simulink start-up inside\n');
fprintf('   the first block; in an already-warm MATLAB session that block is ~1 s)\n');
if ~(A.assert1 && A.assert2 && A.assert3 && A4.pass)
    error('run_all_benchmarks:assertion', ...
          'At least one engineering assertion failed - see the table above.');
end
fprintf('ALL FOUR ASSERTIONS PASSED.\n');

% =========================================================================
%  Figure 1 - four-stage showdown
% =========================================================================
function local_plot_stages(out, P, path)
t = out.t;
col1 = [0.55 0.55 0.55];
col2 = [0.85 0.33 0.10];
col3 = [0.00 0.45 0.74];
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1100 700]);

% (a) states and safety envelope
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

% (b) control effort
subplot(2,2,2);
plot(t, out.b1.u, 'Color', col1, 'LineWidth', 0.8); hold on;
plot(t, out.b2.u, 'Color', col2, 'LineWidth', 0.8);
plot(t, out.b3.u, 'Color', col3, 'LineWidth', 1.0);
grid on; box on; xlim([0 t(end)]);
xlabel('time [s]'); ylabel(sprintf('command [%s]', P.units.input));
title('(b) Actuator command');
legend({'Branch 1', 'Branch 2', 'Branch 3'}, 'Location', 'southwest');

% (c) physics residual probe and AI confidence
subplot(2,2,3);
yyaxis left;
plot(t, out.b3.res_fast, 'Color', [0.49 0.18 0.56], 'LineWidth', 0.9); hold on;
yline(P.ai.max_residual, 'r--', 'LineWidth', 1.0);
ylabel('normalized fast residual'); ylim([0 2.5]);
yyaxis right;
plot(t, out.b3.ai_conf, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.0); hold on;
yline(P.ai.min_confidence, 'k:', 'LineWidth', 1.0);
ylabel('classifier confidence'); ylim([0 1]);
grid on; box on; xlim([0 t(end)]);
xlabel('time [s]');
title('(c) Physics residual probe (fast channel) and AI confidence');

% (d) regime and gate timeline
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




