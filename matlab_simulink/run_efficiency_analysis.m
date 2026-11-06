% SPDX-License-Identifier: MIT
% Copyright (c) 2026 Jiawei Li
% Source: https://github.com/lijiawei255/ai-layered-control
% License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
% Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
function E = run_efficiency_analysis(P, opts)
%RUN_EFFICIENCY_ANALYSIS Action labels and constrained cross-condition tuning.
if nargin<1, P=load_domain_profile(); end
if ischar(P) || isstring(P), P=load_domain_profile(char(P)); end
if nargin<2, opts=struct(); end
if ~isfield(opts,'make_figure'), opts.make_figure=true; end
if ~isfield(opts,'assets_dir'), opts.assets_dir=fullfile(fileparts(mfilename('fullpath')),'..','assets'); end
if isempty(P.classifier), P.classifier=calibrate_coarse_classifier(P); end
if isempty(P.policy), P.policy=train_continuous_policy(P); end
E.samples=[64 256 1024 4096 16000]; E.centers=zeros(1,5);
E.nominal=zeros(1,5); E.frozen=zeros(1,5); E.retrained=zeros(1,5);
for j=1:numel(E.samples)
    n=E.samples(j); nc=min(512,max(8,floor(n/8))); E.centers(j)=nc;
    Q=P; Q.policy=train_continuous_policy(P,n,nc);
    common=struct('branches',2,'duration',6,'seed',303,'ood',false);
    nominal=common; nominal.scenario=0; nominal.drift=0;
    shifted=common; shifted.scenario=1; shifted.drift=.35;
    a=sim_core_engine(Q,nominal); b=sim_core_engine(Q,shifted);
    Q.policy=train_continuous_policy(P,n,nc,.35); c=sim_core_engine(Q,shifted);
    E.nominal(j)=a.metrics.rmse(2,1); E.frozen(j)=b.metrics.rmse(2,2);
    E.retrained(j)=c.metrics.rmse(2,2);
end
% The four mismatches of the Python tuning_study: two loaded-stage drift
% levels, an inertia/damping cross mismatch and a reversed
% inertia/damping/friction mismatch, the latter two without drift.
conds=struct('drift',{.35,.5,0,0},'j_scale',{1,1,1.4,.75}, ...
    'b_scale',{1,1,.7,.75},'f_scale',{1,1,1,.75}, ...
    'label',{'+35% drift','+50% drift','cross J+40% b-30%','reverse J/b/fc/fs-25%'});
for j=1:numel(conds)
    fx=conds(j); started=tic;
    [candidates,knobs]=search(P,fx,3);
    % The B1 counterfactual is graded through the SAME 96 B C99 safety gate as
    % B3: branch 4 runs the B1 law as the shell's feedforward, so the equal
    % six-candidate budgets are compared on one safety floor.
    [b1_candidates,~]=search(P,fx,4);
    b1_valid=cellfun(@(a) a.valid,b1_candidates);
    b1_valid_candidates=sum(b1_valid);
    b1_costs=cellfun(@(a) a.score,b1_candidates); [~,best_b1]=min(b1_costs);
    % Figure 2 reports the lowest-cost B1 candidate across the full search grid;
    % candidate feasibility is reported separately.
    b1_knobs=b1_candidates{best_b1}.knobs;
    valid=struct('scenario',1,'duration',6,'seed',811,'drift',fx.drift, ...
        'ood',false,'amplitude',.43, ...
        'j_scale',fx.j_scale,'b_scale',fx.b_scale,'f_scale',fx.f_scale);
    vb=valid; vb.branches=[3 4]; before=sim_core_engine(P,vb);
    v3=valid; v3.branches=3; v3.tuning=knobs; after=sim_core_engine(P,v3);
    Q=P; Q.baseline=P.baseline.*b1_knobs([1 2 1]);
    v4=valid; v4.branches=4; b1_best=sim_core_engine(Q,v4);
    mission=sim_core_engine(P,struct('branches',3,'seed',811,'drift',fx.drift, ...
        'tuning',knobs,'j_scale',fx.j_scale,'b_scale',fx.b_scale,'f_scale',fx.f_scale));
    b1_mission=sim_core_engine(Q,struct('branches',4,'seed',811,'drift',fx.drift, ...
        'j_scale',fx.j_scale,'b_scale',fx.b_scale,'f_scale',fx.f_scale));
    chosen=find(cellfun(@(a) isequal(a.knobs,knobs),candidates),1);
    E.tuning(j)=struct('drift',fx.drift,'label',fx.label,'j_scale',fx.j_scale, ...
        'b_scale',fx.b_scale,'f_scale',fx.f_scale,'knobs',knobs, ...
        'b1_knobs',b1_knobs,'b1_feasible',b1_valid_candidates>0, ...
        'b1_valid_candidates',b1_valid_candidates, ...
        'before_rmse',before.metrics.rmse(3,2),'after_rmse',after.metrics.rmse(3,2), ...
        'b1_rmse',before.metrics.rmse(4,2),'b1_best_rmse',b1_best.metrics.rmse(4,2), ...
        'before_tv',before.metrics.tv(3,2),'after_tv',after.metrics.tv(3,2), ...
        'before_breach',before.metrics.violations(3,2),'after_breach',after.metrics.violations(3,2), ...
        'mission',mission.metrics,'mission_constraints',constraints(mission,3), ...
        'b1_best_mission',b1_mission.metrics,'b1_best_constraints',constraints(b1_mission,4), ...
        'candidates',{candidates},'b1_candidates',{b1_candidates}, ...
        'search_cost_before',candidates{2}.score,'search_cost_after',candidates{chosen}.score, ...
        'search_seconds',300,'paired_validation_seconds',12,'mission_validation_seconds',40, ...
        'compute_seconds',toc(started));
end
p=P.policy; E.model_scalars=numel(p.mu)+numel(p.sd)+numel(p.centers)+numel(p.w)+1;
E.model_fp64_bytes=8*E.model_scalars; E.model_fp32_bytes=4*E.model_scalars;
% Storage categories are reported separately: trained model, supervisor,
% and the per-axis deterministic core ABI (LayeredState_t / LayeredConfig_t /
% FieldTuning_t).
E.classifier_scalars=15; E.state_bytes=96; E.config_bytes=140; E.tuning_bytes=12;
% Ten B2 features: nine kinematic/context features plus the clamped error
% integral in column 10 (normalised by the 0.5 integrator limit).
feature=zeros(1,10); checksum=0; started=tic;
for j=1:10000
    z=(feature-p.mu)./p.sd; phi=exp(-sum((p.centers-z).^2,2)/(2*p.width^2));
    checksum=checksum+p.w(1)+z*p.w(2:11)+phi'*p.w(12:end);
end
E.matlab_action_us=toc(started)*100; assert(isfinite(checksum));
assert(all(isfinite([E.tuning.after_rmse])) && all([E.tuning.after_breach]==0),'Unsafe field-tuning validation.');
if opts.make_figure, draw_efficiency(E,opts.assets_dir); end
end

function [candidates,knobs]=search(P,fx,branch)
grid=[.75 1;1 1;1.25 1;2 1;1.25 .5;1.25 .75];
knobs=[1 1 1]; best=Inf; candidates=cell(1,6);
for k=1:6
    trial=[grid(k,:) 1];
    a=evaluate(P,fx,branch,trial); candidates{k}=a;
    if a.valid && a.score<best, best=a.score; knobs=trial; end
end
end

function a=evaluate(P,fx,branch,knobs)
% branch 3 grades B3; branch 4 grades the same B1 law through the C99 gate
% (mode 'b1g'), where the searched knob pair scales the B1 baseline gains
% exactly like the Python tuning_study baseline tuple.
opts=struct('branches',branch,'drift',fx.drift,'seed',7,'tuning',knobs, ...
    'j_scale',fx.j_scale,'b_scale',fx.b_scale,'f_scale',fx.f_scale);
if branch==4, P.baseline=P.baseline.*knobs([1 2 1]); end
z=sim_core_engine(P,opts); b=z.(sprintf('b%d',branch)); q=z.stage==1;
c=constraints(z,branch); score=sqrt(mean((z.r(q)-b.log(q,11)).^2));
% Second, independently wall-phased contact probe: a candidate that only clears
% the 20 N contact budget for one wall phase is not an accepted tuning.  The
% two-seed validity is part of the commissioning record (Python tuning_study).
copts=struct('branches',branch,'drift',fx.drift,'seed',811,'scenario',2, ...
    'duration',10,'ood',false,'amplitude',.5,'tuning',knobs, ...
    'j_scale',fx.j_scale,'b_scale',fx.b_scale,'f_scale',fx.f_scale);
zc=sim_core_engine(P,copts); cc=contact_constraints(zc,branch);
a=struct('knobs',knobs,'score',score,'valid',c.valid && cc.valid, ...
    'constraints',c,'contact_constraints',cc, ...
    'metrics',z.metrics,'contact_metrics',zc.metrics);
end

function c=contact_constraints(z,branch)
% Joint physical constraints on the second contact probe (scenario 2, 10 s):
% no envelope breach, 20 N contact budget, low veto occupancy and a bounded R4
% dwell, mirroring the Python candidate validity on the contact fixture.
b=z.(sprintf('b%d',branch)); q=z.stage==2;
veto=mean(b.gate(q)~=0); r4=sum(b.regime(q)==4);
peak=z.metrics.peak_contact(branch,3); breach=z.metrics.violations(branch,3);
valid=breach==0 && peak<20 && veto<=.05 && r4<=1000 && all(isfinite(b.u));
c=struct('peak_contact',peak,'veto_fraction',veto,'r4_ticks',r4, ...
    'violations',breach,'valid',valid);
end

function c=constraints(z,branch)
% The same joint constraints for B3 and the gated B1 counterfactual: no state
% envelope breach, S1 nominal tracking, contact force, and veto / R4 occupancy.
b=z.(sprintf('b%d',branch)); veto=zeros(1,3); fallback=veto;
for s=0:2
    q=z.stage==s; veto(s+1)=mean(b.gate(q)~=0); fallback(s+1)=mean(b.regime(q)==4);
end
peak=z.metrics.peak_contact(branch,1:3); breaches=sum(z.metrics.violations(branch,:));
valid=all(peak<20) && all(veto<=.05) && all(fallback<=.10) && breaches==0 && ...
    z.metrics.rmse(branch,1)<.01 && all(isfinite(b.u));
c=struct('peak_contact',peak,'veto_fraction',veto,'fallback_fraction',fallback, ...
    'violations',breaches,'s1_rmse',z.metrics.rmse(branch,1),'valid',valid);
end

function draw_efficiency(E,folder)
f=figure('Visible','off','Color','w','Position',[100 100 1500 900]);
tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
nexttile; semilogx(E.samples,E.nominal,'o-','LineWidth',1.6); hold on;
semilogx(E.samples,E.frozen,'s-','LineWidth',1.6); semilogx(E.samples,E.retrained,'^-','LineWidth',1.6);
set(gca,'YScale','log'); grid on; xlabel('Action labels'); ylabel('Closed-loop RMSE [rad]');
title('A  Conditional RBF sample budget'); legend('Nominal unit','Frozen +35% unit drift','Retrained +35% unit drift','Location','best');
nexttile; labels={E.tuning.label};
scores=[[E.tuning.before_rmse]' [E.tuning.after_rmse]' [E.tuning.b1_rmse]' [E.tuning.b1_best_rmse]']; bar(scores);
set(gca,'XTickLabel',labels); xlabel('Device mismatch condition');
ylabel('S2 validation RMSE [rad]'); ylim([0 1.6*max(scores,[],'all')]); grid on;
title('B  Equal six-candidate search budget');
legend('B3 factory','B3 feasible tuning','B1+C99 gate factory','B1+C99 S2-best (lowest cost)','Location','northwest');
nexttile; force=zeros(numel(E.tuning),2);
for j=1:numel(E.tuning), force(j,:)=[E.tuning(j).mission.peak_contact(3,3),E.tuning(j).b1_best_mission.peak_contact(4,3)]; end
hb=bar(force); hold on; yline(20,':r','Contact budget','HandleVisibility','off'); grid on;
set(gca,'XTickLabel',labels); xlabel('Device mismatch condition');
ylabel('S3 peak contact force [N]');
% Use a 220 N minimum axis ceiling and 10% headroom above peaks over 200 N.
% Label bars above 100 N with their contact-force values.
ylim([0 220]);
peak=max(force(:));
if peak>200, ylim([0 1.1*peak]); end
for b=1:numel(hb)
    xe=hb(b).XEndPoints; ye=hb(b).YEndPoints;
    for j=1:numel(xe)
        if ye(j)>100
            text(xe(j),ye(j)+4,sprintf('%.1f N',ye(j)),'HorizontalAlignment','center', ...
                'VerticalAlignment','bottom','FontSize',8,'Color','k');
        end
    end
end
title('C  Independent full-mission validation');
legend('B3 feasible tuning','B1+C99 lowest-cost','Location','northwest');
nexttile; bar([E.model_fp32_bytes,4*E.classifier_scalars,E.state_bytes,E.config_bytes,E.tuning_bytes]);
set(gca,'YScale','log','XTickLabel',{'RBF FP32','Supervisor FP32','Core state','Core config','Knobs'},'XTickLabelRotation',25);
ylabel('Stored bytes (separate categories)'); grid on; title('D  Model storage and per-axis RAM');
sgtitle('Action learning and constrained deterministic adaptation','FontWeight','bold');
exportgraphics(f,fullfile(folder,'efficiency_and_generalization.png'),'Resolution',240); close(f);
end
