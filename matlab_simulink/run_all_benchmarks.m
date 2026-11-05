% SPDX-License-Identifier: MIT
% Copyright (c) 2026 Jiawei Li
% Source: https://github.com/lijiawei255/ai-layered-control
% License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
% Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
%RUN_ALL_BENCHMARKS V2 physical benchmark, adaptation and engineering checks.
started=tic; here=fileparts(mfilename('fullpath')); addpath(here);
assets_dir=fullfile(here,'..','assets');
if ~exist('generate_figures','var'), generate_figures=true; end
P=load_domain_profile(); [P.classifier,calib_samples]=calibrate_coarse_classifier(P);
P.policy=train_continuous_policy(P);
branch_names={'B1','B2','B3','B1+C99 Gate','B2+C99 Gate'};
fprintf('[1/6] Shared four-stage fixture, five graded branches and continuous 1 kHz actions\n');
[main,D]=sim_core_engine(P); M=main.metrics;
D.regime_fraction=zeros(4,4); D.gate_ticks=zeros(4,7);
for s=0:3
    selected=main.stage==s;
    for r=1:4, D.regime_fraction(s+1,r)=mean(main.b3.regime(selected)==r); end
    for g=0:6, D.gate_ticks(s+1,g+1)=sum(main.b3.gate(selected)==g); end
end
fresh=[true; diff(main.b3.age)<-.0005] & main.stage<3 & main.b3.age<=P.cfg.outer_timeout_s;
D.fresh_low_confidence_frames=sum(fresh & main.b3.confidence<P.classifier.trip);
D.normal_timeout_ticks=sum(main.stage<3 & main.b3.age>P.cfg.outer_timeout_s);
D.fresh_gate3_ticks=sum(main.stage<3 & main.b3.age<=P.cfg.outer_timeout_s & main.b3.gate==3);
D.confidence_thresholds=[P.classifier.trip P.classifier.clear];
D.main_contact=contact_stats(main);
fprintf('[2/6] Fixed controller banks and model-agnostic supervision\n');
A=struct();
for bank=1:4
    z=sim_core_engine(P,struct('branches',3,'fixed_regime',bank));
    A.(sprintf('R%d',bank))=z.metrics;
end
z=sim_core_engine(P,struct('branches',3,'mode','rules')); A.rules=z.metrics;
A.dynamic=M;
fprintf('[3/6] Action sample budgets and frozen-AI field tuning\n');
E=run_efficiency_analysis(P,struct('make_figure',generate_figures,'assets_dir',assets_dir));
fprintf('[4/6] Independent seeds and physical sensitivity\n');
seeds=[107 211 307 401 503]; phases=[.3 1.3 2.3 3.3 4.3];
H=repmat(struct('seed',0,'wallphase',0,'metrics',M,'fresh_gate3_ticks',0, ...
    'timeout_ticks',0,'fresh_low_confidence_frames',0),1,5);
for j=1:5
    z=sim_core_engine(P,struct('branches',3,'seed',seeds(j),'wallphase',phases(j)));
    normal=z.stage<3; fresh_age=z.b3.age<=P.cfg.outer_timeout_s;
    received=[true; diff(z.b3.age)<-.0005] & normal & fresh_age;
    H(j)=struct('seed',seeds(j),'wallphase',phases(j),'metrics',z.metrics, ...
        'fresh_gate3_ticks',sum(normal & fresh_age & z.b3.gate==3), ...
        'timeout_ticks',sum(normal & ~fresh_age), ...
        'fresh_low_confidence_frames',sum(received & z.b3.confidence<P.classifier.trip));
end
cases={'no_jitter','no_spike','no_ood_veto','band_18_36','band_24_48','one_substep','four_substeps', ...
    'low_noise','low_noise_rules','noise_without_contact','b1_same_lowpass'};
settings={struct('branches',3,'jitter',false),struct('branches',3,'spikes',false), ...
    struct('branches',3,'ood_gate',false), ...
    struct('branches',3,'fault_f',[18 36]),struct('branches',3,'fault_f',[24 48]), ...
    struct('branches',3,'substeps',1),struct('branches',3,'substeps',4), ...
    struct('branches',3,'s3_noise',.0015),struct('branches',3,'s3_noise',.0015,'mode','rules'), ...
    struct('branches',3,'wallk',0),struct('branches',1,'smoothing',true)};
V=struct();
for j=1:numel(cases)
    z=sim_core_engine(P,settings{j}); V.(cases{j})=z.metrics;
    if any(strcmp(cases{j},{'low_noise','low_noise_rules','noise_without_contact'}))
        D.(cases{j})=contact_stats(z);
    end
end
% Broadband probe isolation: neutralise the envelope limit and the cumulative
% permanent latch so only the independent physics-residual veto can trip.
Q=P; Q.cfg.state_safety_limit=100; Q.cfg.outer_timeout_s=100;
Q.cfg.max_fault_vetoes_before_latch=0;
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
fprintf('[5/6] Off-manifold abstention and S3 contact-memory audits\n');
D.off_manifold=off_manifold_audit(P,calib_samples);
D.reverse_s3_to_s1=contact_memory_audit(P,'reverse');
D.retreat_s3_to_s1=contact_memory_audit(P,'retreat');
fprintf('[6/6] C99 input replay, acceptance and generated artifacts\n');
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
% Every graded branch must honour the actuator magnitude and slew contract.
for b=1:5
    u=main.(sprintf('b%d',b)).u;
    assert(all(isfinite(u)) && max(abs(u))<=20.00001,'Actuator magnitude failed.');
    assert(max(abs(diff([0;u])))<=2.00001,'Actuator slew failed.');
end
assert(main.b2.calls==40000 && main.b5.calls==40000,'Equal 1 kHz action rate failed.');
assert(all(isfinite(M.rmse(:))) && M.rmse(2,1)<.01,'Nominal B2 tracking failed.');
assert(D.lcg_last==.16194512567017227,'LCG sequence differs.');
assert(D.counter_bounds,'Counter saturation boundaries failed.');
fprintf('[1] Equal 1 kHz action rate, nominal tracking and actuator bounds: PASS\n');
% Physical safety of the deterministic layer: no envelope breach, contact within
% budget and no permanent latch for B3 (functional/physical checks only; the
% comparative outcome of every branch is reported, never asserted).
assert(all(M.violations(3,:)==0),'B3 state-envelope breach failed.');
assert(M.peak_contact(3,3)<20,'B3 S3 contact budget failed.');
assert(sum(main.b3.gate(main.stage==3)==6)==0,'B3 latched safe during the nominal mission.');
% Qualitative robustness check for the reduced-noise fixture; the strict 20 N
% device budget is asserted on the main four-stage mission (Python audit_study).
assert(V.low_noise.peak_contact(3,3)<25 && V.low_noise_rules.peak_contact(3,3)<25,'Low-noise contact recognition failed.');
assert(D.low_noise.r3_fraction>.8 && D.low_noise_rules.r3_fraction>.8,'Low-noise contact regime occupancy failed.');
fprintf('[2] S2 load/friction and S3 contact/noise tradeoff: PASS\n');
for j=1:numel(H), assert(all(H(j).metrics.violations(3,:)==0),'Holdout B3 safety failed.'); end
assert(all(isfinite(D.sensor_detect_ms)) && all(D.sensor_detect_ms<=1000),'Broadband sensor probe failed.');
assert(all(D.free_energy_ratio<.01) && all(D.free_peak_ratio<1.01),'Free structural response failed.');
assert(all(V.broadband_only.violations(3,:)==0) && ~isempty(first),'Independent broadband veto failed.');
fprintf('[3] Composite OOD and five holdout B3 state-envelope checks: PASS\n');
% Frozen-AI tuning: report the measured outcome of every condition, asserting
% only the search contract (an accepted candidate must be feasible and
% lowest-cost) and the physical mission constraints.
for j=1:numel(E.tuning)
    a=E.tuning(j); cand=a.candidates; validMask=cellfun(@(c) c.valid,cand);
    if any(validMask)
        assert(any(cellfun(@(c) isequal(c.knobs,a.knobs),cand(validMask))), ...
            'The accepted B3 knobs are not a feasible candidate.');
        assert(abs(a.search_cost_after-min(cellfun(@(c) c.score,cand(validMask))))<1e-9, ...
            'The accepted B3 knobs are not the lowest-cost feasible candidate.');
    else
        assert(isequal(a.knobs,[1 1 1]),'With no feasible candidate the search must keep the commissioned knobs.');
    end
    % The mission metrics struct carries all five branch rows; only this branch's
    % row is meaningful for the tuned B3 mission.
    mv=[a.mission.rmse(3,:),a.mission.tv(3,:),a.mission.violations(3,:), ...
        a.mission.peak_contact(3,:),a.mission.maxabs(3,:)];
    assert(all(isfinite(mv)),'Tuned mission metrics are not finite.');
    assert(all(a.mission.violations(3,:)==0),'Tuned mission breached the state envelope.');
end
fprintf('[4] Frozen-AI tuning and action-label sample budget: PASS\n');
assert(all(A.rules.violations(3,:)==0),'Rule-tree supervisor breached the state envelope.');
fprintf('[5] Controller-bank behaviour and supervisor interchangeability: PASS\n');
if generate_figures
    draw_showdown(main,assets_dir); draw_mappings(main,assets_dir);
    [model,status]=build_simulink_model();
    assert(strcmp(status,'built'),'Simulink is required to regenerate the model.');
    sim(model,'StopTime','0.1'); close_system(model,0);
end
fprintf('\n| Stage | Branch | RMSE [rad] | TV [N m] | Violation ticks | Peak contact [N] |\n');
fprintf('|---|---|---:|---:|---:|---:|\n');
for s=1:4
    for b=1:5
        fprintf('| S%d | %s | %.6f | %.2f | %d | %.2f |\n',s,branch_names{b}, ...
            M.rmse(b,s),M.tv(b,s),M.violations(b,s),M.peak_contact(b,s));
    end
end
fprintf('\n| Supervisor | S1 RMSE | S2 RMSE | S3 RMSE | S4 RMSE | S3 TV | S3 peak N |\n');
fprintf('|---|---:|---:|---:|---:|---:|---:|\n');
for key={'R1','R2','R3','R4','dynamic','rules'}
    a=A.(key{1}); fprintf('| %s | %.6f | %.6f | %.6f | %.6f | %.2f | %.2f |\n',key{1},a.rmse(3,:),a.tv(3,3),a.peak_contact(3,3));
end
fprintf('\n| Unit mismatch | B3 factory -> tuned RMSE | B1+C99 factory -> lowest-cost RMSE | B3 S3 peak N | B1+C99 S3 peak N | B3 knobs | B1+C99 valid candidates |\n');
fprintf('|---|---|---|---:|---:|---|---:|\n');
for j=1:numel(E.tuning)
    a=E.tuning(j); fprintf('| %s | %.6f -> %.6f | %.6f -> %.6f | %.2f | %.2f | %.2f / %.2f / %.2f | %d |\n', ...
        a.label,a.before_rmse,a.after_rmse,a.b1_rmse,a.b1_best_rmse, ...
        a.mission.peak_contact(3,3),a.b1_best_mission.peak_contact(4,3), ...
        a.knobs,a.b1_valid_candidates);
end
off=D.off_manifold;
fprintf('\nOff-manifold abstention: fixture={scenario 0, ood false, amplitude 0.46, load_bias 0.08}\n');
fprintf('  conformal trip=%.5f clear=%.5f calibration frames=%d persistence=%d band=[%s] / [%s]\n', ...
    off.conformal_trip,off.conformal_clear,off.calibration_frames, ...
    off.confidence_persistence_frames,num2str(off.band{1}),num2str(off.band{2}));
fprintf('| Seed | Branch | In-box | B3 detect | Rules retreat | RMSE [rad] | Low-conf ticks | R4 ticks | Max |x| | Breach |\n');
fprintf('|---|---|---:|---:|---:|---:|---:|---:|---:|---:|\n');
for j=1:numel(off.seeds)
    row=off.seeds(j);
    for name={'b3','no_conf','rules'}
        a=row.(name{1});
        fprintf('| %d | %s | %.3f | %.3f | %.3f | %.6f | %d | %d | %.3f | %d |\n',row.seed,name{1}, ...
            row.in_box_fraction,row.b3_detection_frames,row.rules_retreat_rate, ...
            a.rmse,a.low_confidence_ticks,a.r4_ticks,a.max_abs_state,a.breach);
    end
end
for j=1:numel(off.nominal_control)
    n=off.nominal_control(j);
    fprintf('  nominal control seed %d: %d low-conf / %d R4 ticks\n',n.seed,n.low_confidence_ticks,n.r4_ticks);
end
rv=D.reverse_s3_to_s1; rt=D.retreat_s3_to_s1;
fprintf('  reverse S3->S1 (wall withdrawn): peak contact=%.2f N, S1 r3_rate=%.3f, release=%.3f s, last2 r3=%.3f\n', ...
    rv.s3_peak_N,rv.s1_r3_rate,rv.release_s,rv.last_r3_rate);
fprintf('  retreat S3->interior (wall kept): S1 r3_rate=%.3f, release=%.3f s, last5 r3=%.3f, rmse=%.5f\n', ...
    rt.s1_r3_rate,rt.release_s,rt.last_r3_rate,rt.s1_rmse);
report=struct('main',M,'ablation',A,'efficiency',E,'holdout',H,'sensitivity',V,'diagnostics',D, ...
    'classifier',[P.classifier.gamma,P.classifier.trip,P.classifier.clear], ...
    'replay',replay,'mailbox_replay',mailbox_replay,'tuning_replays',tuning_replays);
fprintf('\nV2_RESULTS_JSON=%s\n',jsonencode(report));
fprintf('All five engineering checks passed.\nTotal wall-clock time: %.2f s\n',toc(started));

function a=contact_stats(z)
q=z.stage==2; touched=q & z.b3.contact_force>0;
a=struct('r3_fraction',mean(z.b3.regime(q)==3),'r3_contact_fraction',mean(z.b3.regime(touched)==3), ...
    'fallback_fraction',mean(z.b3.regime(q)==4),'veto_fraction',mean(z.b3.gate(q)~=0));
end

function A=off_manifold_audit(P,calib_samples)
% Python audit_study() off-manifold block: one normal-amplitude, normal-noise
% S1 trajectory with an unmodelled coupled constant load torque.  The support
% box is the empirical per-dimension span of the SAME labelled calibration
% windows (25 % margin), never the algebraic saturation caps of the feature
% normalization.  Most single-dimension frames stay inside that box, so the
% range rule rarely retreats; the joint manifold signature is what the
% split-conformal confidence detects.
lo0=min(calib_samples,[],1); hi0=max(calib_samples,[],1); margin=.25*(hi0-lo0);
lo=max(0,lo0-margin); hi=hi0+margin;
nominal=struct('branches',3,'seed',211,'duration',20,'scenario',0,'ood',false, ...
    'amplitude',.46,'mode','dynamic');
z=sim_core_engine(P,nominal);
macro=z.b3.macro(2:end,2:end); n=size(macro,1);
conf=zeros(n,1);
for j=1:n, [~,conf(j)]=classify_feat(macro(j,:),P.classifier); end
trip=linear_quantile(sort(conf),conformal_level(.01,n));
clear=min(.999,1.05*trip);
audit_clf=P.classifier; audit_clf.trip=trip; audit_clf.clear=clear;
Pa=P; Pa.classifier=audit_clf;
nom=nominal_control(Pa);
A=struct('fixture',struct(),'calibration_frames',n,'conformal_trip',trip, ...
    'conformal_clear',clear,'confidence_persistence_frames',5, ...
    'band',{{lo,hi}},'nominal_control',nom,'seeds',[]);
A.fixture=struct('scenario',0,'ood',false,'amplitude',.46,'load_bias',.08);
rows=repmat(struct('seed',0,'in_box_fraction',0,'b3_detection_frames',0, ...
    'rules_retreat_rate',0,'b3',struct(),'no_conf',struct(),'rules',struct()),1,5);
for seed=0:4
    fx=struct('seed',seed,'duration',20,'scenario',0,'ood',false, ...
        'amplitude',.46,'load_bias',.08,'branches',3, ...
        'skip_first_frame',true,'conf_persist',5);
    z_b3=sim_core_engine(Pa,fx);
    fxn=fx; fxn.conf_gate=false; fxn.log_features=true;
    z_no=sim_core_engine(Pa,fxn);
    fxr=fx; fxr.mode='rules'; fxr.rule_bounds={lo,hi};
    z_rules=sim_core_engine(P,fxr);
    F=z_no.b3.macro(2:end,2:end);
    in_box=mean(all(F>=lo-1e-9 & F<=hi+1e-9,2));
    M=z_b3.b3.macro(2:end,2:end); nf=size(M,1); cf=zeros(nf,1);
    for j=1:nf, [~,cf(j)]=classify_feat(M(j,:),P.classifier); end
    det=mean(cf<trip);
    rr=mean(z_rules.b3.regime(z_rules.t>=.1)==4);
    row=struct('seed',seed,'in_box_fraction',in_box, ...
        'b3_detection_frames',det,'rules_retreat_rate',rr, ...
        'b3',off_row(z_b3),'no_conf',off_row(z_no),'rules',off_row(z_rules));
    rows(seed+1)=row;
    % Honest in-box property: the coupled load bias keeps most single-dimension
    % frames inside the data-derived box, so the range rule rarely retreats.
    assert(row.in_box_fraction>=.90,sprintf('Off-manifold frame %.3f is outside the empirical box.',row.in_box_fraction));
    assert(row.rules_retreat_rate<.10,'The range rule retreated inside its data-derived support box.');
    % The joint manifold detector must detect the coupled load bias at a far
    % higher frame rate than the single-dimension range rule.
    assert(row.b3_detection_frames>.80,'The manifold detector missed the coupled load bias.');
    assert(row.b3_detection_frames>max(5*row.rules_retreat_rate,.50),'Manifold detection did not beat the range rule.');
    assert(row.b3.low_confidence_ticks>0,'The manifold detector never abstained on the off-manifold fixture.');
    assert(row.b3.breach==0 && row.b3.max_abs_state<1,'Off-manifold B3 breached the state envelope.');
    assert(row.no_conf.low_confidence_ticks==0,'The no-confidence ablation still abstained.');
end
for j=1:numel(nom)
    assert(nom(j).low_confidence_ticks<=100 && nom(j).r4_ticks<=400, ...
        'Nominal control produced a false confidence veto.');
end
A.seeds=rows;
end

function rows=nominal_control(P)
% Independent nominal runs under the audit detector: no false abstention.
rows=repmat(struct('seed',0,'low_confidence_ticks',0,'r4_ticks',0),1,2);
for j=1:2
    fx=struct('branches',3,'seed',j-1,'duration',20,'scenario',0,'ood',false, ...
        'amplitude',.46,'mode','dynamic','skip_first_frame',true,'conf_persist',5);
    z=sim_core_engine(P,fx); a=z.b3.log; q=z.t>=.1;
    rows(j)=struct('seed',j-1,'low_confidence_ticks',sum(a(q,3)==3), ...
        'r4_ticks',sum(a(q,4)==4));
end
end

function r=off_row(z)
% Python _off_row(): tracking, abstention and fast-residual evidence.
a=z.b3.log;
r=struct('rmse',sqrt(mean((a(:,1)-z.r).^2)),'peak_error',max(abs(a(:,1)-z.r)), ...
    'low_confidence_ticks',sum(a(:,3)==3),'r4_ticks',sum(a(:,4)==4), ...
    'max_residual',max(a(:,7)),'max_abs_state',max(abs(a(:,1))), ...
    'breach',sum(abs(a(:,1))>1));
end

function R=contact_memory_audit(P,name)
% Python reverse_context() / retreat_context(): 10 s of S3 wall contact followed
% by S1 tracking.  'reverse' withdraws the wall (20 s fixture); 'retreat' keeps
% the wall in place while the reference returns to zero (30 s fixture), so only
% physical retreat can release the remembered contact.
if strcmp(name,'reverse')
    R=contact_memory_stats(P,name,20,18);
    assert(R.s3_peak_N<20,'Reverse S3->S1 peak contact exceeded the 20 N budget.');
    assert(R.release_s<0.35,'The compliant bank did not release within 0.35 s of wall withdrawal.');
    assert(R.last_r3_rate==0,'Late S1 tracking still routed the compliant bank.');
else
    R=contact_memory_stats(P,name,30,27);
    assert(R.release_s<0.50,'The contact memory did not release within 0.50 s of physical retreat.');
    assert(R.last_r3_rate==0,'Late S1 tracking still routed the compliant bank after retreat.');
    assert(R.s1_rmse<0.05,'The retreated S1 segment did not track the interior reference.');
end
end

function R=contact_memory_stats(P,name,duration,tail_start)
z=sim_core_engine(P,struct('branches',3,'duration',duration,'seed',5, ...
    'scenario',0,'ood',false,'contact_memory',name));
a=z.b3.log; t=z.t; q2=z.stage==0; tail=t>=tail_start;
r3=find(q2 & a(:,4)==3);
R=struct('s1_r3_rate',mean(a(q2,4)==3),'last_r3_rate',mean(a(tail,4)==3), ...
    's1_rmse',sqrt(mean((a(q2,1)-z.r(q2)).^2)),'s3_peak_N',0,'release_s',0);
if isempty(r3), R.release_s=0; else, R.release_s=t(r3(end))-10; end
if strcmp(name,'reverse'), R.s3_peak_N=max(a(z.stage==2,5)); end
end

function [reg,conf]=classify_feat(feat,clf)
% Mirror of python_bench classify(feat, protos, gamma).
d=sqrt(sum((clf.prototypes-feat).^2,2)); [ds,order]=sort(d);
reg=order(1); conf=(ds(2)-ds(1))/(ds(2)+ds(1)+1e-9)*exp(-clf.gamma*ds(1));
conf=min(max(conf,0),1);
end

function q=linear_quantile(x,p)
% Mirror of numpy.quantile(method='linear'): same virtual position and weight.
x=sort(x);
pos=(numel(x)-1)*p; lo=floor(pos); hi=min(lo+1,numel(x)-1);
q=x(lo+1)+(pos-lo)*(x(hi+1)-x(lo+1));
end

function l=conformal_level(a,n)
% Finite-sample split-conformal order: min(1, ceil((N+1)*a)/N).
l=min(1,ceil((n+1)*a)/n);
end

function draw_showdown(z,folder)
f=figure('Visible','off','Color','w','Position',[80 80 1450 1250]);
tiledlayout(f,5,1,'TileSpacing','compact','Padding','compact');
colors=[.15 .4 .7;.85 .33 .1;.1 .55 .35;.55 .35 .65;.75 .55 .15];
labels={'B1 FF + PID','B2 conditional RBF','B3 layered core','B1 + C99 gate','B2 + C99 gate'};
keep=1:5:numel(z.t);
nexttile; hold on;
for b=1:5, plot(z.t(keep),z.(sprintf('b%d',b)).x(keep),'Color',colors(b,:),'LineWidth',.8); end
plot(z.t(keep),z.r(keep),'k--','LineWidth',.8); yline(1,':r'); yline(-1,':r');
ylabel('State [rad]'); title('Four-stage physical stress test');
legend(labels{:},'Reference','Location','northoutside','Orientation','horizontal','AutoUpdate','off');
nexttile; hold on;
for b=1:5, plot(z.t(keep),z.(sprintf('b%d',b)).u(keep),'Color',colors(b,:),'LineWidth',.45); end
ylabel('Command [N m]');
nexttile; hold on;
for b=1:5, plot(z.t,z.(sprintf('b%d',b)).contact_force,'Color',colors(b,:),'LineWidth',.6); end
yline(20,':r','Contact budget'); ylabel('Contact force [N]');
nexttile; plot(z.t,z.b3.residual,'Color',colors(3,:),'LineWidth',.8); hold on; yline(1,':r');
ylabel('Normalized residual'); ylim([0 4]);
nexttile; stairs(z.t,z.b3.regime,'Color',colors(3,:),'LineWidth',1.1); hold on;
stairs(z.t,z.b3.gate,'Color',[.55 .35 .65],'LineWidth',.7); ylabel('Discrete state'); xlabel('Time [s]');
legend('Regime','Gate','Location','northwest','AutoUpdate','off'); yticks(0:6);
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
