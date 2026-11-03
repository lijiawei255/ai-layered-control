% SPDX-License-Identifier: MIT
% Copyright (c) 2026 Jiawei Li
% Source: https://github.com/lijiawei255/ai-layered-control
% License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
% Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
%RUN_ALL_BENCHMARKS V2 physical benchmark, adaptation and engineering checks.
started=tic; here=fileparts(mfilename('fullpath')); addpath(here);
assets_dir=fullfile(here,'..','assets');
if ~exist('generate_figures','var'), generate_figures=true; end
P=load_domain_profile(); P.classifier=calibrate_coarse_classifier(P);
P.policy=train_continuous_policy(P);
fprintf('[1/5] Shared four-stage fixture and continuous 1 kHz actions\n');
[main,D]=sim_core_engine(P); M=main.metrics;
D.regime_fraction=zeros(4,4); D.gate_ticks=zeros(4,5);
for s=0:3
    selected=main.stage==s;
    for r=1:4, D.regime_fraction(s+1,r)=mean(main.b3.regime(selected)==r); end
    for g=0:4, D.gate_ticks(s+1,g+1)=sum(main.b3.gate(selected)==g); end
end
fresh=[true; diff(main.b3.age)<-.0005] & main.stage<3 & main.b3.age<=P.cfg.outer_timeout_s;
D.fresh_low_confidence_frames=sum(fresh & main.b3.confidence<P.classifier.trip);
D.normal_timeout_ticks=sum(main.stage<3 & main.b3.age>P.cfg.outer_timeout_s);
D.fresh_gate3_ticks=sum(main.stage<3 & main.b3.age<=P.cfg.outer_timeout_s & main.b3.gate==3);
D.confidence_thresholds=[P.classifier.trip P.classifier.clear];
D.main_contact=contact_stats(main);
fprintf('[2/5] Fixed controller banks and model-agnostic supervision\n');
A=struct();
for bank=1:4
    z=sim_core_engine(P,struct('branches',3,'fixed_regime',bank));
    A.(sprintf('R%d',bank))=z.metrics;
end
z=sim_core_engine(P,struct('branches',3,'mode','rules')); A.rules=z.metrics;
A.dynamic=M;
fprintf('[3/5] Action sample budgets and frozen-AI field tuning\n');
E=run_efficiency_analysis(P,struct('make_figure',generate_figures,'assets_dir',assets_dir));
fprintf('[4/5] Independent seeds and physical sensitivity\n');
seeds=[107 211 307 401 503]; phases=[.3 1.3 2.3 3.3 4.3];
H=repmat(struct('seed',0,'wallphase',0,'metrics',M,'fresh_gate3_ticks',0, ...
    'timeout_ticks',0,'fresh_low_confidence_frames',0),1,5);
for j=1:5
    z=sim_core_engine(P,struct('seed',seeds(j),'wallphase',phases(j)));
    normal=z.stage<3; fresh_age=z.b3.age<=P.cfg.outer_timeout_s;
    received=[true; diff(z.b3.age)<-.0005] & normal & fresh_age;
    H(j)=struct('seed',seeds(j),'wallphase',phases(j),'metrics',z.metrics, ...
        'fresh_gate3_ticks',sum(normal & fresh_age & z.b3.gate==3), ...
        'timeout_ticks',sum(normal & ~fresh_age), ...
        'fresh_low_confidence_frames',sum(received & z.b3.confidence<P.classifier.trip));
end
cases={'no_jitter','no_spike','no_ood_veto','band_18_36','band_24_48','one_substep','four_substeps', ...
    'low_noise','low_noise_rules','noise_without_contact','b1_same_lowpass'};
settings={struct('jitter',false),struct('branches',3,'spikes',false), ...
    struct('branches',3,'ood_gate',false), ...
    struct('fault_f',[18 36]),struct('fault_f',[24 48]),struct('substeps',1),struct('substeps',4), ...
    struct('branches',3,'s3_noise',.0015),struct('branches',3,'s3_noise',.0015,'mode','rules'), ...
    struct('branches',3,'wallk',0),struct('branches',1,'smoothing',true)};
V=struct();
for j=1:numel(cases)
    z=sim_core_engine(P,settings{j}); V.(cases{j})=z.metrics;
    if any(strcmp(cases{j},{'low_noise','low_noise_rules','noise_without_contact'}))
        D.(cases{j})=contact_stats(z);
    end
end
Q=P; Q.cfg.state_safety_limit=100; Q.cfg.outer_timeout_s=100;
z=sim_core_engine(Q,struct('branches',3,'spikes',false,'probe_spikes',false,'conf_gate',false));
V.broadband_only=z.metrics;
first=find(z.b3.gate==2 & z.context.fault,1); onset=find(z.context.fault,1);
D.broadband_only_detect_ms=1000*(z.t(first)-z.t(onset));
z=sim_core_engine(P,struct('branches',3,'corrupt_slow_at',22,'log_core',true));
V.mailbox_corruption=z.metrics; damaged=z.t>=22;
D.mailbox_corruption=struct('corrupted_ticks',sum(z.b3.core_out(:,7)), ...
    'r4_fraction',mean(z.b3.regime(damaged)==4), ...
    'changing_command_ticks',sum(abs(diff(z.b3.u(damaged)))>1e-10));
assert(all(z.b3.gate(damaged)~=1) && all(z.metrics.violations(3,:)==0),'Damaged slow mailbox froze control or breached safety.');
fprintf('[5/5] C99 input replay, acceptance and generated artifacts\n');
replay=verify_core_equivalence(P);
mailbox_replay=verify_core_equivalence(P,struct('corrupt_slow_at',22));
tuning_faults=[5 1 1;-1 1 1;NaN 1 1;1 1 1.5];
names={'gain_high','gain_negative','gain_nan','margin_high'}; tuning_replays=struct();
damaged=main.t>=5;
for j=1:size(tuning_faults,1)
    tuning=repmat([1 1 1],numel(main.t),1);
    tuning(damaged,:)=repmat(tuning_faults(j,:),sum(damaged),1);
    options=struct('branches',3,'tuning',tuning,'log_core',true);
    z=sim_core_engine(P,options); b=z.b3;
    assert(all(b.regime(damaged)==4) && all(b.gate(damaged)~=1) && ...
        all(b.core_out(damaged,7)==1) && all(bitand(uint32(b.core_out(damaged,6)),uint32(16))~=0), ...
        'Invalid tuning did not enter audited closed-loop R4.');
    assert(b.integrators_finite && all(isfinite(b.core_out),'all') && ...
        max(abs(b.u))<=20.00001 && max(abs(diff([0;b.u])))<=2.00001, ...
        'Invalid tuning corrupted the controller state or actuator limits.');
    assert(all(z.metrics.violations(3,:)==0) && any(abs(diff(b.u(damaged)))>1e-10), ...
        'Invalid tuning froze the actuator or breached the state envelope.');
    D.runtime_tuning.(names{j})=struct('corrupted_ticks',sum(b.core_out(:,7)), ...
        'r4_fraction',mean(b.regime(damaged)==4),'maxabs',max(abs(b.x)), ...
        'changing_command_ticks',sum(abs(diff(b.u(damaged)))>1e-10));
    tuning_replays.(names{j})=verify_core_equivalence(P,options);
end
for b=1:3
    u=main.(sprintf('b%d',b)).u;
    assert(all(isfinite(u)) && max(abs(u))<=20.00001,'Actuator magnitude failed.');
    assert(max(abs(diff([0;u])))<=2.00001,'Actuator slew failed.');
end
assert(main.b2.calls==40000 && all(M.rmse(:,1)<.01),'Nominal accuracy or action rate failed.');
assert(D.lcg_last==.16194512567017227,'LCG sequence differs.');
assert(D.counter_bounds,'Counter saturation boundaries failed.');
fprintf('[1] Equal 1 kHz action rate, nominal tracking and actuator bounds: PASS\n');
assert(M.rmse(2,2)>M.rmse(1,2) && M.rmse(3,2)<.03,'S2 transfer result failed.');
assert(M.peak_contact(1,3)>20 && M.peak_contact(3,3)<20 && M.tv(3,3)<M.tv(1,3),'Contact tradeoff failed.');
assert(V.low_noise.peak_contact(3,3)<20 && V.low_noise_rules.peak_contact(3,3)<20,'Low-noise contact recognition failed.');
assert(D.low_noise.r3_fraction>.8 && D.low_noise_rules.r3_fraction>.8,'Low-noise contact regime occupancy failed.');
fprintf('[2] S2 load/friction and S3 contact/noise tradeoff: PASS\n');
assert(all(M.violations(3,:)==0) && all(M.violations(1:2,4)>0),'Composite fault result failed.');
for j=1:numel(H), assert(all(H(j).metrics.violations(3,:)==0),'Holdout B3 safety failed.'); end
assert(all(isfinite(D.sensor_detect_ms)) && all(D.sensor_detect_ms<=1000),'Broadband sensor probe failed.');
assert(all(D.free_energy_ratio<.01) && all(D.free_peak_ratio<1.01),'Free structural response failed.');
assert(all(V.broadband_only.violations(3,:)==0) && ~isempty(first),'Independent broadband veto failed.');
fprintf('[3] Composite OOD and five holdout B3 state-envelope checks: PASS\n');
assert(E.retrained(end)<E.frozen(end),'Retrained action-head recovery failed.');
expected_improvement=[.187 .165];
for j=1:numel(E.tuning)
    a=E.tuning(j); chosen=find(cellfun(@(c) isequal(c.knobs,a.knobs),a.candidates),1);
    assert(~isempty(chosen) && a.candidates{chosen}.score<=a.candidates{2}.score, ...
        'Field tuning did not preserve or improve the commissioning objective.');
    assert(isequal(a.knobs,[1.25 .5 1]) && ...
        abs(1-a.after_rmse/a.before_rmse-expected_improvement(j))<.005, ...
        'Joint field-tuning validation differs from the commissioned benchmark.');
    assert(E.tuning(j).mission_constraints.valid,'Tuned mission constraints failed.');
end
fprintf('[4] Frozen-AI tuning and action-label sample budget: PASS\n');
assert(A.R2.rmse(3,2)<A.R1.rmse(3,2) && A.R2.rmse(3,2)<A.R3.rmse(3,2),'Load-bank necessity failed.');
assert(A.R3.peak_contact(3,3)<A.R1.peak_contact(3,3) && A.R3.peak_contact(3,3)<A.R2.peak_contact(3,3),'Compliance-bank necessity failed.');
assert(all(A.rules.violations(3,:)==0) && A.rules.peak_contact(3,3)<20,'Rules supervisor safety failed.');
fprintf('[5] Controller-bank necessity and supervisor interchangeability: PASS\n');
if generate_figures
    draw_showdown(main,assets_dir); draw_mappings(main,assets_dir);
    [model,status]=build_simulink_model();
    assert(strcmp(status,'built'),'Simulink is required to regenerate the model.');
    sim(model,'StopTime','0.1'); close_system(model,0);
end
fprintf('\n| Stage | Branch | RMSE [rad] | TV [N m] | Violation ticks | Peak contact [N] |\n');
fprintf('|---|---|---:|---:|---:|---:|\n');
for s=1:4
    for b=1:3
        fprintf('| S%d | B%d | %.6f | %.2f | %d | %.2f |\n',s,b,M.rmse(b,s),M.tv(b,s),M.violations(b,s),M.peak_contact(b,s));
    end
end
fprintf('\n| Supervisor | S1 RMSE | S2 RMSE | S3 RMSE | S4 RMSE | S3 TV | S3 peak N |\n');
fprintf('|---|---:|---:|---:|---:|---:|---:|\n');
for key={'R1','R2','R3','R4','dynamic','rules'}
    a=A.(key{1}); fprintf('| %s | %.6f | %.6f | %.6f | %.6f | %.2f | %.2f |\n',key{1},a.rmse(3,:),a.tv(3,3),a.peak_contact(3,3));
end
fprintf('\n| Unit drift | B3 factory -> tuned RMSE | B1 factory -> S2-best RMSE | B3 S3 peak N | B1 S2-best S3 peak N | B3 knobs | B1 S2-best feasible |\n');
fprintf('|---|---|---|---:|---:|---|---|\n');
for j=1:2
    a=E.tuning(j); fprintf('| +%.0f%% | %.6f -> %.6f | %.6f -> %.6f | %.2f | %.2f | %.2f / %.2f / %.2f | %d |\n', ...
        100*a.drift,a.before_rmse,a.after_rmse,a.b1_rmse,a.b1_best_rmse,a.mission.peak_contact(3,3), ...
        a.b1_best_mission.peak_contact(1,3),a.knobs,a.b1_best_feasible);
end
report=struct('main',M,'ablation',A,'efficiency',E,'holdout',H,'sensitivity',V,'diagnostics',D, ...
    'replay',replay,'mailbox_replay',mailbox_replay,'tuning_replays',tuning_replays);
fprintf('\nV2_RESULTS_JSON=%s\n',jsonencode(report));
fprintf('All five engineering checks passed.\nTotal wall-clock time: %.2f s\n',toc(started));

function a=contact_stats(z)
q=z.stage==2; touched=q & z.b3.contact_force>0;
a=struct('r3_fraction',mean(z.b3.regime(q)==3),'r3_contact_fraction',mean(z.b3.regime(touched)==3), ...
    'fallback_fraction',mean(z.b3.regime(q)==4),'veto_fraction',mean(z.b3.gate(q)~=0));
end

function draw_showdown(z,folder)
f=figure('Visible','off','Color','w','Position',[80 80 1450 1250]);
tiledlayout(f,5,1,'TileSpacing','compact','Padding','compact');
colors=[.15 .4 .7;.85 .33 .1;.1 .55 .35]; keep=1:5:numel(z.t);
nexttile; hold on;
for b=1:3, plot(z.t(keep),z.(sprintf('b%d',b)).x(keep),'Color',colors(b,:),'LineWidth',.8); end
plot(z.t(keep),z.r(keep),'k--','LineWidth',.8); yline(1,':r'); yline(-1,':r');
ylabel('State [rad]'); title('Four-stage physical stress test'); legend('B1 FF + PID','B2 conditional RBF','B3 layered core','Reference','Location','northoutside','Orientation','horizontal','AutoUpdate','off');
nexttile; hold on;
for b=1:3, plot(z.t(keep),z.(sprintf('b%d',b)).u(keep),'Color',colors(b,:),'LineWidth',.45); end
ylabel('Command [N m]');
nexttile; hold on;
for b=1:3, plot(z.t,z.(sprintf('b%d',b)).contact_force,'Color',colors(b,:),'LineWidth',.6); end
yline(20,':r','Contact budget'); ylabel('Contact force [N]');
nexttile; plot(z.t,z.b3.residual,'Color',colors(3,:),'LineWidth',.8); hold on; yline(1,':r');
ylabel('Normalized residual'); ylim([0 4]);
nexttile; stairs(z.t,z.b3.regime,'Color',colors(3,:),'LineWidth',1.1); hold on;
stairs(z.t,z.b3.gate,'Color',[.55 .35 .65],'LineWidth',.7); ylabel('Discrete state'); xlabel('Time [s]');
legend('Regime','Gate','Location','northwest','AutoUpdate','off'); yticks(0:4);
axeslist=findall(f,'Type','axes');
for ax=axeslist', grid(ax,'on'); xlim(ax,[0 40]); for t=[10 20 30], xline(ax,t,':','Color',[.55 .55 .55]); end; end
exportgraphics(f,fullfile(folder,'benchmark_4stages_showdown.png'),'Resolution',240); close(f);
end

function draw_mappings(z,folder)
f=figure('Visible','off','Color','w','Position',[100 100 1500 650]);
tiledlayout(f,2,3,'TileSpacing','compact','Padding','compact');
titles={'Industrial WAAM / PLC servo','Edge VLA / fast-slow robotics','Closed-loop aDBS / SCS'};
slow={'Process / load / contact class','Task / environment context','Biomarker / interface class'};
fast={'Process / servo law + actuator limits','Torque / impedance + safe fallback','Device law + output / timing limits'};
for j=1:3
    nexttile; axis off; title(titles{j},'FontWeight','bold');
    text(.03,.82,'10 Hz supervisor','FontWeight','bold','FontSize',13);
    text(.03,.65,slow{j},'FontSize',11); text(.03,.47,'Regime + confidence + timestamp','FontSize',10);
    text(.03,.28,'1 kHz deterministic core','FontWeight','bold','FontSize',13);
    text(.03,.09,fast{j},'FontSize',10);
end
for j=1:3
    nexttile; stairs(z.t,z.b3.regime,'LineWidth',1); grid on; ylim([.8 4.2]); yticks(1:4);
    xlabel('Common fixture time [s]'); ylabel('Selected regime'); title('Shared normalized interface');
end
sgtitle('Three application mappings of a general control contract','FontWeight','bold');
exportgraphics(f,fullfile(folder,'domain_profiles_comparison.png'),'Resolution',240); close(f);
end
