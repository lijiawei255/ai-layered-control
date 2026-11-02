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
for j=1:2
    drift=[.35 .5]; drift=drift(j); started=tic;
    [candidates,knobs]=search(P,drift,3);
    [b1_candidates,b1_knobs]=search(P,drift,1);
    b1_costs=cellfun(@(a) a.score,b1_candidates); [~,best_b1]=min(b1_costs);
    b1_best_knobs=b1_candidates{best_b1}.knobs;
    valid=struct('branches',[1 3],'scenario',1,'duration',6,'seed',811, ...
        'drift',drift,'ood',false,'amplitude',.43);
    before=sim_core_engine(P,valid); valid.branches=3; valid.tuning=knobs; after=sim_core_engine(P,valid);
    Q=P; Q.baseline=P.baseline.*b1_best_knobs([1 2 1]); valid.branches=1;
    b1_best=sim_core_engine(Q,valid);
    mission=sim_core_engine(P,struct('branches',3,'seed',811,'drift',drift,'tuning',knobs));
    b1_mission=sim_core_engine(Q,struct('branches',1,'seed',811,'drift',drift));
    E.tuning(j)=struct('drift',drift,'knobs',knobs,'b1_deployment_knobs',b1_knobs, ...
        'b1_best_knobs',b1_best_knobs,'b1_best_feasible',b1_candidates{best_b1}.valid, ...
        'before_rmse',before.metrics.rmse(3,2),'after_rmse',after.metrics.rmse(3,2), ...
        'b1_rmse',before.metrics.rmse(1,2),'b1_best_rmse',b1_best.metrics.rmse(1,2), ...
        'before_tv',before.metrics.tv(3,2),'after_tv',after.metrics.tv(3,2), ...
        'before_breach',before.metrics.violations(3,2),'after_breach',after.metrics.violations(3,2), ...
        'mission',mission.metrics,'mission_constraints',constraints(mission,3), ...
        'b1_best_mission',b1_mission.metrics,'b1_best_constraints',constraints(b1_mission,1), ...
        'candidates',{candidates},'b1_candidates',{b1_candidates}, ...
        'search_seconds',240,'paired_validation_seconds',12,'mission_validation_seconds',40, ...
        'compute_seconds',toc(started));
end
p=P.policy; E.model_scalars=numel(p.mu)+numel(p.sd)+numel(p.centers)+numel(p.w)+1;
E.model_fp64_bytes=8*E.model_scalars; E.model_fp32_bytes=4*E.model_scalars;
E.classifier_scalars=15; E.state_bytes=88; E.config_bytes=132; E.tuning_bytes=12;
feature=zeros(1,9); checksum=0; started=tic;
for j=1:10000
    z=(feature-p.mu)./p.sd; phi=exp(-sum((p.centers-z).^2,2)/(2*p.width^2));
    checksum=checksum+p.w(1)+z*p.w(2:10)+phi'*p.w(11:end);
end
E.matlab_action_us=toc(started)*100; assert(isfinite(checksum));
assert(all(isfinite([E.tuning.after_rmse])) && all([E.tuning.after_breach]==0),'Unsafe field-tuning validation.');
if opts.make_figure, draw_efficiency(E,opts.assets_dir); end
end

function [candidates,knobs]=search(P,drift,branch)
gains=[.75 1 1.25 2]; damping=[.5 .75]; knobs=[1 1 1]; best=Inf;
candidates=cell(1,6); anchor=knobs(1);
for k=1:6
    if k<=4, trial=[gains(k) 1 1]; else, trial=[anchor damping(k-4) 1]; end
    a=evaluate(P,drift,branch,trial); candidates{k}=a;
    if a.valid && a.score<best, best=a.score; knobs=trial; end
    if k==4, anchor=knobs(1); end
end
end

function a=evaluate(P,drift,branch,knobs)
opts=struct('branches',branch,'drift',drift,'seed',7,'tuning',knobs);
if branch==1, P.baseline=P.baseline.*knobs([1 2 1]); end
z=sim_core_engine(P,opts); b=z.(sprintf('b%d',branch)); q=z.stage==1;
c=constraints(z,branch); score=sqrt(mean((z.r(q)-b.log(q,11)).^2));
a=struct('knobs',knobs,'score',score,'valid',c.valid,'constraints',c,'metrics',z.metrics);
end

function c=constraints(z,branch)
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
nexttile; scores=[[E.tuning.before_rmse]' [E.tuning.after_rmse]' [E.tuning.b1_rmse]' [E.tuning.b1_best_rmse]']; bar(scores);
set(gca,'XTickLabel',{'+35% unit drift','+50% unit drift'}); xlabel('Unit Drift: Inertia + Stiction');
ylabel('S2 validation RMSE [rad]'); ylim([0 1.6*max(scores,[],'all')]); grid on;
title('B  Equal six-candidate search budget');
legend('B3 factory','B3 feasible tuning','B1 factory','B1 S2-best (infeasible)','Location','northwest');
nexttile; force=zeros(2,2);
for j=1:2, force(j,:)=[E.tuning(j).mission.peak_contact(3,3),E.tuning(j).b1_best_mission.peak_contact(1,3)]; end
bar(force); hold on; yline(20,':r','Contact budget','HandleVisibility','off'); grid on;
set(gca,'XTickLabel',{'+35% unit drift','+50% unit drift'}); xlabel('Unit Drift: Inertia + Stiction');
ylabel('S3 peak contact force [N]'); ylim([0 100]); title('C  Independent full-mission validation');
legend('B3 feasible tuning','B1 S2-best (infeasible)','Location','northwest');
nexttile; bar([E.model_fp32_bytes,4*E.classifier_scalars,E.state_bytes,E.config_bytes,E.tuning_bytes]);
set(gca,'YScale','log','XTickLabel',{'RBF FP32','Supervisor FP32','Core state','Core config','Knobs'},'XTickLabelRotation',25);
ylabel('Stored bytes (separate categories)'); grid on; title('D  Model storage and per-axis RAM');
sgtitle('Action learning and constrained deterministic adaptation','FontWeight','bold');
exportgraphics(f,fullfile(folder,'efficiency_and_generalization.png'),'Resolution',240); close(f);
end
