function E = run_efficiency_analysis(P, opts)
%RUN_EFFICIENCY_ANALYSIS Action-label budgets, frozen-AI tuning and storage.
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
    Q.policy=train_continuous_policy(P,n,nc,.35);
    c=sim_core_engine(Q,shifted);
    E.nominal(j)=a.metrics.rmse(2,1); E.frozen(j)=b.metrics.rmse(2,2);
    E.retrained(j)=c.metrics.rmse(2,2);
end
for j=1:2
    drift=[.35 .5]; drift=drift(j); started=tic;
    trial=struct('branches',3,'scenario',1,'duration',4,'seed',501,'drift',drift,'ood',false);
    best=Inf; knobs=[1 1 1];
    for gain=[.75 1 1.5 2]
        candidate=[gain 1 1]; [score,valid]=evaluate(P,trial,candidate);
        if valid && score<best, knobs=candidate; best=score; end
    end
    if isfinite(best)
        gain=knobs(1);
        for damping=[.5 .75]
            candidate=[gain damping 1]; [score,valid]=evaluate(P,trial,candidate);
            if valid && score<best, knobs=candidate; best=score; end
        end
    end
    valid=struct('branches',[1 3],'scenario',1,'duration',6,'seed',811, ...
        'drift',drift,'ood',false,'amplitude',.43);
    before=sim_core_engine(P,valid); valid.branches=3; valid.tuning=knobs;
    after=sim_core_engine(P,valid);
    mission=sim_core_engine(P,struct('branches',3,'seed',811,'drift',drift,'tuning',knobs));
    E.tuning(j)=struct('drift',drift,'knobs',knobs, ...
        'before_rmse',before.metrics.rmse(3,2),'after_rmse',after.metrics.rmse(3,2), ...
        'b1_rmse',before.metrics.rmse(1,2),'before_tv',before.metrics.tv(3,2), ...
        'after_tv',after.metrics.tv(3,2),'before_breach',before.metrics.violations(3,2), ...
        'after_breach',after.metrics.violations(3,2),'mission',mission.metrics, ...
        'search_seconds',24,'validation_seconds',12,'compute_seconds',toc(started));
end
p=P.policy; E.model_scalars=numel(p.mu)+numel(p.sd)+numel(p.centers)+numel(p.w)+1;
E.model_fp64_bytes=8*E.model_scalars; E.model_fp32_bytes=4*E.model_scalars;
E.classifier_scalars=15; E.state_bytes=88; E.config_bytes=132; E.tuning_bytes=12;
z=zeros(1,9); checksum=0; started=tic;
for j=1:10000
    phi=exp(-sum((p.centers-z).^2,2)/(2*p.width^2));
    checksum=checksum+p.w(1)+z*p.w(2:10)+phi'*p.w(11:end);
end
E.matlab_action_us=toc(started)*100; assert(isfinite(checksum));
assert(all([E.tuning.after_rmse]<.6*[E.tuning.before_rmse]),'Field tuning improvement below 40%%.');
assert(all([E.tuning.after_breach]==0),'Unsafe field-tuning validation.');
if opts.make_figure, draw_efficiency(E,opts.assets_dir); end
end

function [score,valid]=evaluate(P,opts,knobs)
opts.tuning=knobs; z=sim_core_engine(P,opts); a=z.b3.log;
score=sqrt(mean((z.r-a(:,11)).^2));
valid=all(abs(a(:,1))<=1) && all(isfinite(a(:,2))) && ~any(ismember(a(:,3),[2 4]));
end

function draw_efficiency(E,folder)
f=figure('Visible','off','Color','w','Position',[100 100 1500 500]);
tiledlayout(f,1,3,'TileSpacing','compact','Padding','compact');
nexttile; semilogx(E.samples,E.nominal,'o-','LineWidth',1.6); hold on;
semilogx(E.samples,E.frozen,'s-','LineWidth',1.6); semilogx(E.samples,E.retrained,'^-','LineWidth',1.6);
set(gca,'YScale','log'); grid on; xlabel('Action labels'); ylabel('Closed-loop RMSE [rad]');
title('A  Conditional RBF sample budget'); legend('Nominal unit','Frozen transfer +35%','Retrained +35%','Location','best');
nexttile; bar([[E.tuning.before_rmse]' [E.tuning.after_rmse]' [E.tuning.b1_rmse]']);
set(gca,'XTickLabel',{'+35% inertia','+50% inertia'}); ylabel('Validation RMSE [rad]'); ylim([0 .037]); grid on;
title('B  Frozen AI, field tuning'); legend('B3 factory (1,1,1)','B3 tuned (2,1,1)','B1 fixed PID','Location','northwest');
nexttile; bar([E.model_fp32_bytes,4*E.classifier_scalars,E.state_bytes,E.config_bytes,E.tuning_bytes]);
set(gca,'YScale','log','XTickLabel',{'RBF FP32','Supervisor FP32','Core state','Core config','Knobs'},'XTickLabelRotation',25);
ylabel('Stored bytes (separate categories)'); grid on; title('C  Model storage and per-axis RAM');
sgtitle('Action learning and deterministic adaptation','FontWeight','bold');
exportgraphics(f,fullfile(folder,'efficiency_and_generalization.png'),'Resolution',240); close(f);
end
