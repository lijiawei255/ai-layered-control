% SPDX-License-Identifier: MIT
% Copyright (c) 2026 Jiawei Li
% Source: https://github.com/lijiawei255/ai-layered-control
% License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
% Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
function [out, diagnostics] = sim_core_engine(P, opts)
%SIM_CORE_ENGINE Three controllers on a shared four-stage physical fixture.
if nargin < 1, P = load_domain_profile(); end
if ischar(P) || isstring(P), P = load_domain_profile(char(P)); end
if nargin < 2, opts = struct(); end
defaults = struct('branches',[1 2 3],'mode','dynamic','fixed_regime',[], ...
    'seed',7,'drift',.35,'scenario',[],'duration',40,'substeps',2, ...
    'ood',true,'jitter',true,'spikes',true,'bias',true,'wallphase',.4, ...
    'noise_scale',1,'s3_noise',.006,'fault_f',[12 24],'zeta',.012,'wallk',300,'wallx',.43, ...
    'tuning',[1 1 1],'ood_gate',true,'conf_gate',true, ...
    'auto_calibrate',true,'log_core',false,'amplitude',[],'probe_spikes',true, ...
    'corrupt_slow_at',Inf,'smoothing',false,'seq_mode',false);
keys = fieldnames(defaults);
for j=1:numel(keys)
    if ~isfield(opts,keys{j}), opts.(keys{j})=defaults.(keys{j}); end
end
if isempty(P.classifier) && opts.auto_calibrate
    P.classifier = calibrate_coarse_classifier(P);
end
if any(opts.branches==2) && isempty(P.policy)
    P.policy = train_continuous_policy(P);
end
c = make_context(P,opts);
out = struct('context',c,'profile',P,'t',c.t,'r',c.r,'stage',c.stage, ...
    'ref',struct('r',c.r,'rd',c.rd,'rdd',c.rdd),'b1',[],'b2',[],'b3',[]);
for b=opts.branches
    mode = opts.mode;
    if b==1, mode='b1'; elseif b==2, mode='b2'; end
    out.(sprintf('b%d',b)) = run_branch(P,c,opts,mode);
end
out.config = P.cfg;
if ~isempty(P.classifier)
    out.config.min_ai_confidence=P.classifier.trip;
    out.config.confidence_clear_ratio=P.classifier.clear;
end
M=struct('rmse',nan(3,4),'tv',nan(3,4),'violations',nan(3,4), ...
    'peak_contact',nan(3,4),'maxabs',nan(3,4));
for b=opts.branches
    z=out.(sprintf('b%d',b)).log;
    for s=unique(c.stage)'
        q=c.stage==s;
        M.rmse(b,s+1)=sqrt(mean((z(q,1)-c.r(q)).^2));
        M.tv(b,s+1)=sum(abs(diff(z(q,2))));
        M.violations(b,s+1)=sum(abs(z(q,1))>1);
        M.peak_contact(b,s+1)=max(z(q,5));
        M.maxabs(b,s+1)=max(abs(z(q,1)));
    end
end
out.metrics=M;
if nargout>1, diagnostics=physical_probes(P); end
end

function c = make_context(P,o)
dt=P.dt; n=round(o.duration/dt); t=(0:n-1)'*dt;
if isempty(o.scenario), stage=min(floor(t/10),3); else, stage=repmat(o.scenario,n,1); end
r=zeros(n,1); rd=r; rdd=r;
for s=0:3
    mask=stage==s; tr=t(mask);
    if isempty(o.scenario), tr=tr-10*s; amp=.5; else, amp=.46; end
    if ~isempty(o.amplitude), amp=o.amplitude; end
    [r(mask),rd(mask),rdd(mask)]=reference(s,tr,amp);
end
drift=(stage>=1)*o.drift; p=zeros(n,5);
p(:,1)=P.nominal.J*(1+drift); p(:,2)=P.nominal.b*(1+.3*drift);
p(:,3)=P.nominal.k; p(:,4)=P.nominal.fc; p(:,5)=P.nominal.fs;
p(stage==1,4)=1.2*(1+4.5*o.drift);
p(stage==1,5)=2.5*(1+4.3*o.drift);
ran=reshape(lcg(o.seed+11,6*n),n,6)';
white=(ran(1,:)+ran(2,:)+ran(3,:)-1.5)'*2;
sigma=repmat(.0015,n,1); sigma(stage==2)=o.s3_noise;
noise=white.*sigma*o.noise_scale;
onset=30.037; wallstart=20; freqstart=30;
if ~isempty(o.scenario), onset=.037; wallstart=0; freqstart=0; end
fault=(stage==3)&o.ood&(t>=onset);
sfault=double(fault)*.025*o.bias;
spike=zeros(n,1); hit=fault&(ran(4,:)'<.004);
sg=ones(n,1); sg(ran(5,:)'<=.5)=-1;
spike(hit)=.18*sg(hit);
first=find(fault,1); if ~isempty(first), spike(first)=.18; end
if ~o.spikes, spike(:)=0; end
ct=max(t-wallstart,0); approach=.24*exp(-ct/.25);
wall=o.wallx+.025*sin(2*pi*.73*t+o.wallphase)+approach;
wv=.025*2*pi*.73*cos(2*pi*.73*t+o.wallphase)-approach/.25;
wc=double(stage==2)*o.wallk;
emit=(0:100:n-1)'; nf=numel(emit); rn=reshape(lcg(o.seed+37,2*nf),nf,2)';
delays=repmat(.02,nf,1); edge=(stage(emit+1)>=2)&o.jitter;
delays(edge)=.06+.06*rn(1,edge)';
drops=edge&(rn(2,:)'<.08); arrivals=cell(n,1);
for i=1:nf
    k=emit(i)+round(delays(i)/dt)+1;
    if k<=n && ~drops(i), arrivals{k}=[arrivals{k},i]; end
end
freq=o.fault_f(1)+diff(o.fault_f)*min(max((t-freqstart)/10,0),1);
c=struct('t',t,'stage',stage,'r',r,'rd',rd,'rdd',rdd,'p',p, ...
    'noise',noise,'sfault',sfault,'spike',spike,'wall',wall,'wv',wv, ...
    'wc',wc,'arrivals',{arrivals},'stamp',emit*dt,'fault',fault, ...
    'freq',freq,'zeta',o.zeta,'substeps',o.substeps);
end

function a=lcg(seed,n)
a=zeros(n,1); state=uint32(seed);
for k=1:n
    state=uint32(bitand(uint64(1664525)*uint64(state)+uint64(1013904223),uint64(4294967295)));
    a(k)=(double(state)+.5)/4294967296;
end
end

function [r,v,a]=reference(stage,t,amp)
if stage==0
    w=pi; r=amp*cos(w*t); v=-amp*w*sin(w*t); a=-amp*w*w*cos(w*t); return;
end
if stage==1, move=.9; dwell=.9; phase=.8; else, move=1.; dwell=1.5; phase=0.; end
u=mod(t+phase,2*(move+dwell)); r=zeros(size(t)); v=r; a=r;
r(u<dwell)=amp; r(u>=dwell+move & u<2*dwell+move)=-amp;
for side=1:2
    if side==1, q=u>=dwell & u<dwell+move; tau=(u(q)-dwell)/move; sg=-1;
    else, q=u>=2*dwell+move; tau=(u(q)-2*dwell-move)/move; sg=1; end
    pp=tau.*tau.*tau.*(10+tau.*(-15+6*tau));
    vv=tau.*tau.*(30-60*tau+30*tau.*tau);
    aa=tau.*(60-180*tau+120*tau.*tau);
    r(q)=-sg*amp+sg*2*amp*pp; v(q)=sg*2*amp*vv/move; a(q)=sg*2*amp*aa/(move*move);
end
end

function [x,v,stuck]=advance(x,v,u,p,dt,force)
drive=u-p(2)*v-p(3)*x-force; stuck=0;
if abs(v)<.003 && abs(drive)<=p(5), v=0.; stuck=1; return; end
direction=v; if abs(v)<=1e-10, direction=drive; end
f=p(4); if direction<0, f=-f; end
vn=v+dt*(drive-f)/p(1);
if v*vn<0 && abs(drive)<=p(5), vn=0.; end
x=x+dt*vn; v=vn;
end

function f=friction(v,p)
f=0.; if abs(v)>1e-10, f=sign(v)*p(4); end
end

function pr=probe_init(x)
% Contact memory: cy is the position of the last real unilateral blockage, free
% flags a surface the state has since passed unhindered, and retreat_s times a
% sustained retreat behind that remembered position.
pr=struct('xh',x,'vh',0.,'lp',0.,'buf',[],'yf',x,'vf',0., ...
    'me',0.,'mu',0.,'hf',0.,'sp',0.,'power',0.,'load_energy',0.,'contact',0., ...
    'slowenergy',0.,'veto',0.,'y1',x,'y2',x,'yold',x,'yolder',x, ...
    'a1',0.,'a2',0.,'alp',0.,'ae',0.,'apbuf',[],'cy',1.,'free',0,'retreat_s',0);
end

function [pr,ym,yd,res]=probe_step(pr,y,u,r,p,dt,spike_gate)
pr.buf=[pr.buf,y]; if numel(pr.buf)>5, pr.buf=pr.buf(2:end); end
sorted=sort(pr.buf); med=sorted(floor(numel(sorted)/2)+1);
pr.sp=abs(y-med)/.12; ym=y; if pr.sp>1, ym=med; end
[xp,vp]=advance(pr.xh,pr.vh,u,p,dt,0.);
innov=ym-xp;
pr.lp=pr.lp+dt/.10*(innov-pr.lp);
pr.slowenergy=pr.slowenergy+dt/.35*(pr.lp*pr.lp-pr.slowenergy);
aa=dt/(dt+1/(2*pi*25));
old_y2=pr.y2;
pr.y1=pr.y1+aa*(med-pr.y1); pr.y2=pr.y2+aa*(pr.y1-pr.y2);
actual=(pr.y2-2*pr.yold+pr.yolder)/(dt*dt); pr.yolder=pr.yold; pr.yold=pr.y2;
predicted=(u-p(2)*pr.vf-p(3)*ym-friction(pr.vf,p))/p(1);
pr.apbuf=[pr.apbuf,predicted]; if numel(pr.apbuf)>3, pr.apbuf=pr.apbuf(2:end); end
pr.a1=pr.a1+aa*(pr.apbuf(1)-pr.a1); pr.a2=pr.a2+aa*(pr.a1-pr.a2);
ar=actual-pr.a2; pr.alp=pr.alp+dt/(dt+1/(2*pi*5))*(ar-pr.alp);
v25=(pr.y2-old_y2)/dt;
pr.power=pr.power+dt/.05*(p(1)*pr.alp*v25-pr.power);
pr.load_energy=pr.load_energy+dt/.05*((p(1)*pr.alp)^2-pr.load_energy);
% Charge only on a real unilateral blockage (inside the contact band and the
% low-frequency effort/acceleration product showing resistance), so a free
% oscillation cannot fake contact.  Release stays at 8 s while the state is
% against the surface and drops to 0.15 s once the state passes the remembered
% contact position unhindered, or after 3 s of sustained retreat behind it: the
% wall is gone either way.
ratio=pr.power/(pr.load_energy+.25);
blocked=ym>.40 && ratio>.03 && pr.power>.10;
evidence=0.; if blocked, evidence=pr.power-.10; end
if max(ym,r)<pr.cy-.08
    pr.retreat_s=pr.retreat_s+dt;
else
    pr.retreat_s=0.;
end
if blocked
    pr.cy=ym; pr.free=0;
elseif ym>pr.cy+.04 || pr.retreat_s>3.0
    pr.free=1;
end
tau=8.; if pr.free, tau=.15; end
pr.contact=max([pr.contact*(1-dt/tau),evidence,0]);
pr.ae=pr.ae+dt/.008*((ar-pr.alp)^2-pr.ae); pr.hf=sqrt(max(pr.ae,0))/500;
pr.xh=xp+dt*40*innov; pr.vh=vp+dt*400*innov;
vd=(ym-pr.yf)/.002; pr.yf=pr.yf+dt/.002*(ym-pr.yf); pr.vf=pr.vf+dt/.002*(vd-pr.vf);
pr.me=pr.me+dt/.25*((r-ym)^2-pr.me); pr.mu=pr.mu+dt/.25*(u*u-pr.mu);
sp=pr.sp; if ~spike_gate, sp=0; end
pr.veto=max([sp,pr.hf,pr.veto*(1-dt)]);
yd=pr.vf; res=pr.veto;
end

function [reg,conf]=classify(feat,clf)
d=sqrt(sum((clf.prototypes-feat).^2,2)); [ds,order]=sort(d);
reg=order(1); conf=(ds(2)-ds(1))/(ds(2)+ds(1)+1e-9)*exp(-clf.gamma*ds(1));
conf=min(max(conf,0),1);
end

function B=run_branch(P,c,o,mode)
n=numel(c.t); dt=P.dt; nom=[P.nominal.J,P.nominal.b,P.nominal.k,P.nominal.fc,P.nominal.fs];
x=c.r(1); v=0.; xm=x; vm=0.; wasflex=false; u0=0.; integ=0.; target=0.; pr=probe_init(x);
ctx=zeros(1,4); frames=zeros(ceil(n/100),4); last_stamp=0.; cfg=P.cfg;
if ~isempty(P.classifier)
    cfg.min_ai_confidence=P.classifier.trip; cfg.confidence_clear_ratio=P.classifier.clear;
end
ckeys=fieldnames(cfg); for j=1:numel(ckeys), cfg.(ckeys{j})=single(cfg.(ckeys{j})); end
cfg.min_dwell_ticks=uint32(P.cfg.min_dwell_ticks);
cfg.ood_recovery_cooldown_ticks=uint32(P.cfg.ood_recovery_cooldown_ticks);
cfg.max_hold_ticks=uint32(P.cfg.max_hold_ticks);
cfg.max_fault_vetoes_before_latch=uint32(P.cfg.max_fault_vetoes_before_latch);
tuning=single(o.tuning); core=core_init(); ai=1; conf=1.; calls=0;
logs=zeros(n,14); macro=zeros(ceil(n/100),5);
age=zeros(n,1);
% A sequenced-lease trace publishes a monotonic outer_seq and a zero
% transported age, so the core has to age the frame on its own clock.  The
% producer then stalls (no committed frame, constant sequence) long enough for
% the local lease to age past the timeout and exhaust the native HOLD budget,
% a short fallback-to-zero window exercises the corruption path of a live
% sequence, and the producer finally resumes with a new number.
seq=uint32(0); seq_drop=false(n,1); seq_stall=false(n,1); ood_burst=false(n,1);
if o.seq_mode
    s_stall=min(n,max(1,round(.3*n))); seq_stall(s_stall:min(n,s_stall+619))=true;
    s_drop=min(n,max(1,round(.4*n))); seq_drop(s_drop:min(n,s_drop+19))=true;
    % Deliberate physics-residual OOD episodes after the lease has recovered:
    % three rising edges exercise the OOD veto and the cumulative severe-fault
    % latch (GATE_LATCHED_SAFE) that no purely nominal closed loop can reach.
    centers=[round(.52*n),round(.56*n),round(.60*n)];
    for s_center=centers
        s_ood=min(n,max(1,s_center)); ood_burst(s_ood:min(n,s_ood+4))=true;
    end
end
CI=zeros(n,(9+double(o.seq_mode))*double(o.log_core));
CO=zeros(n,7*double(o.log_core));
CT=zeros(n,3*double(o.log_core)); integrators_finite=true;
for k=1:n
    r=c.r(k); rd=c.rd(k); rdd=c.rdd(k);
    [pr,ym,yd,res]=probe_step(pr,x+c.noise(k)+c.sfault(k)+c.spike(k),u0,r,nom,dt,o.probe_spikes);
    if mod(k-1,100)==0
        mi=floor((k-1)/100)+1;
        q=sqrt(max([pr.me,pr.mu,pr.slowenergy],0));
        frames(mi,:)=[q./(q+P.feature_scales).*[1,1,.5],1.4*min(pr.contact/.01,1)];
        macro(mi,:)=[c.t(k),frames(mi,:)];
    end
    if ~(o.seq_mode && seq_stall(k))
        for mi=c.arrivals{k}
            if c.stamp(mi)>=last_stamp
                last_stamp=c.stamp(mi); ctx=frames(mi,:);
                % One committed slow frame, one sequence number.
                if o.seq_mode, seq=seq+uint32(1); end
                if ~isempty(P.classifier), [ai,conf]=classify(ctx,P.classifier); end
            end
        end
    end
    age(k)=c.t(k)-last_stamp;
    e=r-ym; ed=rd-yd;
    uff=nom(1)*rdd+nom(2)*rd+nom(3)*r+friction(rd,nom);
    gate=0; reg=0;
    if strcmp(mode,'b2')
        feat=[e/.1,ed,r/.5,rd/2,rdd/6,ctx];
        z=(feat-P.policy.mu)./P.policy.sd;
        phi=exp(-sum((P.policy.centers-z).^2,2)/(2*P.policy.width^2));
        raw=P.policy.w(1)+z*P.policy.w(2:10)+phi'*P.policy.w(11:end); calls=calls+1;
        u=clamp(clamp(raw,-20,20),u0-2,u0+2);
    elseif strcmp(mode,'b1')
        kp=P.baseline(1); kd=P.baseline(2); ki=P.baseline(3);
        raw=kp*e+kd*ed+ki*integ+uff;
        if o.smoothing && k>1, target=target+P.cfg.blend_alpha*(raw-target);
        else, target=raw; end
        u=clamp(clamp(target,-20,20),u0-2,u0+2);
        if ki~=0, integ=clamp(integ+dt*e+(u-raw)*dt/(ki*.1),-.5,.5); end
    else
        if ~isempty(o.fixed_regime), ai=o.fixed_regime; conf=1.; end
        if strcmp(mode,'rules')
            if ctx(4)>.65, ai=3; elseif ctx(3)>.25 || ctx(2)>.55, ai=2; else, ai=1; end
            conf=1.;
        end
        ci=conf; if ~o.conf_gate, ci=1.; end
        mai=ai;
        % In sequenced-lease mode the local lease owns the timeout, so the
        % adapter keeps publishing the last decision during a producer stall.
        if ~o.seq_mode && age(k)>P.cfg.outer_timeout_s, mai=4; ci=0.; end
        if c.t(k)>=o.corrupt_slow_at, ci=NaN; end
        residual=res; if ~o.ood_gate, residual=0.; end
        if ood_burst(k), residual=single(2.0); end
        age_in=age(k);
        if o.seq_mode, age_in=single(0); end
        input=single([mai,ci,age_in,residual,ym,yd,r,rd,uff]);
        if o.seq_mode
            if seq_drop(k), input(10)=single(0); else, input(10)=single(seq); end
        end
        tune=tuning(min(k,size(tuning,1)),:);
        [core,result]=core_step(core,cfg,tune,input);
        integrators_finite=integrators_finite && all(isfinite(core.integ));
        u=result(1); gate=result(3); reg=result(4);
        if o.log_core, CI(k,:)=double(input); CO(k,:)=double(result); CT(k,:)=double(tune); end
    end
    sx=x; sxm=xm; p=c.p(k,:); peak=0.; stuck=0;
    if c.fault(k) && ~wasflex, xm=x; vm=v; wasflex=true; end
    for j=1:c.substeps
        h=dt/c.substeps; delta=x-c.wall(k); contact=0.;
        if delta>0 && c.wc(k)>0, contact=max(0,c.wc(k)*delta+.8*(v-c.wv(k))); end
        peak=max(peak,contact/.25);
        if c.fault(k)
            [x,v,xm,vm,stuck]=flex_advance(x,v,xm,vm,u,p,h,c.freq(k),c.zeta,contact);
        else
            [x,v,stuck]=advance(x,v,u,p,h,contact);
        end
    end
    logs(k,:)=[sx,u,gate,reg,peak,stuck,res,conf,pr.hf,ai,ym,sxm,yd,integ]; u0=u;
end
B=struct('log',logs,'macro',macro,'calls',calls,'core_in',CI,'core_out',CO, ...
    'core_tuning',CT,'integrators_finite',integrators_finite, ...
    'age',age, ...
    'x',logs(:,1),'u',logs(:,2),'gate',logs(:,3),'regime',logs(:,4), ...
    'residual',logs(:,7),'confidence',logs(:,8),'contact_force',logs(:,5));
end

function [x,v,xm,vm,stuck]=flex_advance(x,v,xm,vm,u,p,dt,freq,zeta,contact)
jm=p(1)*.2; jl=p(1)*.8; reduced=jm*jl/(jm+jl); w=2*pi*freq;
kc=reduced*w*w; dc=2*zeta*w*reduced; spring=kc*(xm-x)+dc*(vm-v);
vm=vm+dt*(u-.1*vm-spring)/jm; xm=xm+dt*vm;
pl=p; pl(1)=jl; [x,v,stuck]=advance(x,v,spring,pl,dt,contact);
end

function D=physical_probes(P)
nom=[P.nominal.J,P.nominal.b,P.nominal.k,P.nominal.fc,P.nominal.fs];
D.counter_bounds=counter_bounds(P.cfg);
sequence=lcg(18,1000); D.lcg_last=sequence(end);
D.sensor_hz=[12 18 37 83 137]; D.sensor_detect_ms=nan(1,5);
for j=1:5
    pr=probe_init(0);
    for k=1:200, pr=probe_step(pr,0,0,0,nom,P.dt,false); end
    for k=0:999
        y=.2*sin(2*pi*D.sensor_hz(j)*k*P.dt);
        [pr,~,~,res]=probe_step(pr,y,0,0,nom,P.dt,false);
        if res>1, D.sensor_detect_ms(j)=1000*k*P.dt; break; end
    end
end
D.free_hz=[12 18 24]; D.free_energy_ratio=zeros(1,3); D.free_peak_ratio=zeros(1,3);
p=nom; p(1)=p(1)*1.35; p(2)=p(2)*(1+.3*.35); h=P.dt/2;
jm=.2*p(1); jl=.8*p(1); reduced=jm*jl/(jm+jl);
for j=1:3
    freq=D.free_hz(j); kc=reduced*(2*pi*freq)^2;
    x=0; v=0; xm=.05; vm=0; initial=.5*kc*xm^2; peak=initial;
    for k=1:6000
        [x,v,xm,vm]=flex_advance(x,v,xm,vm,0,p,h,freq,.012,0);
        energy=.5*(jm*vm^2+jl*v^2+kc*(xm-x)^2+p(3)*x^2);
        peak=max(peak,energy);
    end
    D.free_energy_ratio(j)=energy/initial; D.free_peak_ratio(j)=peak/initial;
end
end

function passed=counter_bounds(c)
keys=fieldnames(c);
ints={'min_dwell_ticks','ood_recovery_cooldown_ticks','max_hold_ticks','max_fault_vetoes_before_latch'};
for j=1:numel(keys)
    if ~any(strcmp(keys{j},ints)), c.(keys{j})=single(c.(keys{j})); end
end
for j=1:numel(ints), c.(ints{j})=uint32(c.(ints{j})); end
maximum=intmax('uint32'); dwell0=c.min_dwell_ticks; c.min_dwell_ticks=maximum;
% The cumulative latch would otherwise mask the saturation boundaries.
c.max_fault_vetoes_before_latch=uint32(0);
t=single([1 1 1]); input=single([1 1 0 0 0 0 0 0 0]); base=core_init();
counts={'step_count','safety_veto_count','ood_veto_count','low_confidence_veto_count', ...
    'latency_hold_count','corrupted_frame_count'};
for j=1:numel(counts), base.(counts{j})=maximum-uint32(1); end
base.dwell=maximum-uint32(1); st=base;
for j=1:2, st=core_step(st,c,t,input); end
assert(st.dwell==maximum && st.step_count==maximum,'Normal counters wrapped.');
request=input; request(1)=2; st=base; st.dwell=maximum-uint32(2);
[st,out]=core_step(st,c,t,request);
assert(out(4)==1 && st.dwell==maximum-uint32(1),'Maximum dwell switched too early.');
[st,out]=core_step(st,c,t,request);
assert(out(4)==2 && st.dwell==0,'Maximum dwell did not accept its boundary tick.');
st=base; st.dwell=maximum; [~,out]=core_step(st,c,t,request);
assert(out(4)==2,'Saturated dwell blocked a normal switch.');
for threshold=0:1
    q=c; q.min_dwell_ticks=uint32(threshold); [~,out]=core_step(core_init(),q,t,request);
    assert(out(4)==2,'Zero/one-tick dwell delayed a normal switch.');
end
frames=repmat(input,7,1); frames(1,5)=2; frames(2,4)=2; frames(3,2)=0;
frames(4,3)=c.outer_timeout_s+c.dt_s/2; frames(5,2)=NaN; frames(6,5)=NaN;
targets=counts([2 3 4 5 6 6 6]);
for j=1:size(frames,1)
    st=base; q=c; if j==7, q.dt_s=single(NaN); end
    for k=1:2
        st.safety=false; st.ood=false; st.conf=false;
        st=core_step(st,q,t,frames(j,:));
    end
    assert(st.(targets{j})==maximum && st.step_count==maximum,'Diagnostic counters wrapped.');
end
% L1 slow-frame lease: a live outer_seq ages the slow frame on the fast clock,
% a changed number opens a fresh lease, and a sequence that falls back to zero
% is a mailbox anomaly that requests closed-loop R4.
q=c; q.min_dwell_ticks=dwell0; st=core_init(); lease=single([1 1 0 0 0 0 0 0 0 7]);
for k=1:400, [st,out]=core_step(st,q,t,lease); end
assert(st.outer_age_ticks==uint32(399) && st.last_outer_seq==uint32(7),'Sequence lease did not age locally.');
assert(out(3)==1 && st.hold_ticks>uint32(0) && st.hold_ticks<uint32(100),'Leased age did not reach the bounded hold.');
st=core_step(st,q,t,single([1 1 0 0 0 0 0 0 0 8]));
assert(st.outer_age_ticks==uint32(0) && st.last_outer_seq==uint32(8),'A new sequence number did not reset the lease.');
[st,out]=core_step(st,q,t,single([1 1 .005 0 0 0 0 0 0 0]));
assert(out(3)==3 && out(4)==4 && st.conf,'A sequence falling back to zero did not request closed-loop R4.');
assert(out(7)==1 && st.corrupted_frame_count==uint32(1) && ...
    bitand(st.fault,uint32(16))~=0,'A sequence falling back to zero was not audited as corruption.');
% Bounded native HOLD: a stale heartbeat runs out its budget and forces a
% de-energised zero without arming the permanent latch.
st=core_init(); stale=single([1 1 .5 0 0 0 0 0 0]);
for k=1:100, [st,out]=core_step(st,c,t,stale); assert(out(3)==1,'Unexpected HOLD escalation.'); end
assert(st.latency_hold_count==uint32(1),'A native HOLD was counted more than once.');
[st,out]=core_step(st,c,t,stale);
assert(out(3)==5 && out(4)==4 && out(1)==0 && st.hold_ticks==uint32(101),'HOLD budget did not force SAFE_ZERO.');
assert(bitand(st.fault,uint32(32))~=0 && ~st.permanent,'HOLD timeout audit bit or permanent latch is wrong.');
assert(st.latency_hold_count==uint32(1),'The HOLD escalation changed the episode count.');
% A constant age just past the timeout enters one native HOLD episode that
% runs for every stepped tick.
st=core_init(); edge=single([1 1 c.outer_timeout_s+c.dt_s/2 0 0 0 0 0 0]);
for k=1:5, [st,out]=core_step(st,c,t,edge); assert(out(3)==1,'A constant stale frame left the native HOLD.'); end
assert(st.latency_hold_count==uint32(1) && st.hold_ticks==uint32(5),'A constant stale age was counted per tick.');
% One isolated stale tick is still one episode, even when the age overshoots
% the timeout by many periods.
st=core_init(); jump=single([1 1 c.outer_timeout_s+10*c.dt_s 0 0 0 0 0 0]);
[st,out]=core_step(st,c,t,jump);
assert(out(3)==1 && st.latency_hold_count==uint32(1) && st.hold_ticks==uint32(1),'An age jump was not counted as one HOLD episode.');
% A corrupt configuration holds for its fixed 100-tick budget (the slew limit
% is untrusted, so the command is de-energised in one tick) and an unfinishable
% UINT32_MAX budget is rejected as corrupt from the first tick.
q=c; q.max_fault_vetoes_before_latch=uint32(3); q.kp(1)=single(NaN); st=core_init();
for k=1:100, [st,out]=core_step(st,q,t,input); assert(out(3)==1 && out(7)==1,'Corrupt configuration did not hold.'); end
[st,out]=core_step(st,q,t,input);
assert(out(3)==5 && out(1)==0 && st.hold_ticks==uint32(101) && st.permanent,'Corrupt-configuration budget failed.');
assert(bitand(st.fault,uint32(32))~=0 && bitand(st.fault,uint32(64))~=0,'Corrupt-configuration audit bits are missing.');
q=c; q.max_hold_ticks=intmax('uint32'); [st,out]=core_step(core_init(),q,t,input);
assert(out(3)==1 && out(7)==1 && st.hold_ticks==uint32(1),'UINT32_MAX hold budget was not rejected.');
% Cumulative severe-fault latch: three safety/OOD episodes latch
% GATE_LATCHED_SAFE.  The acknowledgement keeps the audit history and restores
% the full episode quota, then the 300-tick cooldown gates the AI re-arm.
q=c; q.min_dwell_ticks=dwell0; q.max_fault_vetoes_before_latch=uint32(3); st=core_init();
for episode=1:3
    for k=1:5, st=core_step(st,q,t,single([1 1 .005 3 0 0 0 0 0])); end
    for k=1:5, st=core_step(st,q,t,single([1 1 .005 .1 0 0 0 0 0])); end
    assert(st.latch_episode_count==uint32(episode) && st.ood_veto_count==uint32(episode),'Episode accounting drifted.');
end
[st,out]=core_step(st,q,t,single([1 1 .005 .1 0 0 0 0 0]));
assert(out(3)==6 && out(4)==4 && st.permanent,'Cumulative fault latch did not engage.');
history=[st.fault,st.step_count,st.ood_veto_count];
[st,released]=core_ack(st,q);
assert(released==1 && ~st.permanent && st.latch_episode_count==uint32(0) && st.hold_ticks==uint32(0),'Acknowledgement did not re-arm the latch.');
assert(isequal([st.fault,st.step_count,st.ood_veto_count],history),'Acknowledgement cleared audit history.');
assert(st.recovery==q.ood_recovery_cooldown_ticks,'Acknowledgement did not re-arm the recovery cooldown.');
for k=1:q.ood_recovery_cooldown_ticks+1, [st,out]=core_step(st,q,t,single([1 1 .005 0 0 0 0 0 0])); end
assert(out(3)==0 && out(4)==1,'Acknowledged core did not return to the nominal regime.');
passed=true;
end

function st=core_init()
% Mirror of Layered_Init(): NOMINAL regime, no latches, no fault bits, zero
% command/target/integrators, control not started, dwell and lease clocks zero.
st=struct('u_applied',single(0),'u_target',single(0),'integ',zeros(4,1,'single'), ...
    'blend',single(0),'started',false,'regime',1,'ood',false,'safety',false, ...
    'conf',false,'permanent',false,'dwell',uint32(0),'recovery',uint32(0), ...
    'fault',uint32(0),'step_count',uint32(0),'safety_veto_count',uint32(0), ...
    'ood_veto_count',uint32(0),'low_confidence_veto_count',uint32(0), ...
    'latency_hold_count',uint32(0),'corrupted_frame_count',uint32(0), ...
    'hold_ticks',uint32(0),'last_outer_seq',uint32(0),'outer_age_ticks',uint32(0), ...
    'latch_episode_count',uint32(0));
end

function [st,released]=core_ack(st,c)
% Mirror of Layered_AcknowledgeFaultLatch(): clear the permanent latch and the
% episode quota, re-arm the recovery cooldown.  Audit history (fault bits,
% step count, veto counters) is never cleared here.
if config_corrupt(c), released=0; return; end
st.permanent=false; st.ood=false; st.safety=false; st.conf=false;
st.hold_ticks=uint32(0); st.latch_episode_count=uint32(0);
st.recovery=uint32(c.ood_recovery_cooldown_ticks);
released=1;
end

function bad=config_corrupt(c)
% Mirror of lg_config_is_corrupt(): same finite and range rules, including the
% two fail-safe fields (a zero or UINT32_MAX hold budget can never time out).
fields={'dt_s','outer_timeout_s','max_physics_residual','residual_clear_ratio', ...
 'min_ai_confidence','confidence_clear_ratio','state_safety_limit','safety_trip_ratio', ...
 'safety_clear_ratio','safety_predict_s','u_max','u_slew_max_per_s','blend_alpha','integ_limit','windup_tau_s'};
values=zeros(16+numel(fields),1,'single');
values(1:16)=[c.kp;c.kd;c.ki;c.ff_scale];
for j=1:numel(fields), values(16+j)=c.(fields{j}); end
hold_budget=uint32(c.max_hold_ticks);
bad=any(~isfinite(values)) || any(abs(values)>single(3e38));
bad=bad || c.dt_s<=0 || c.outer_timeout_s<=0 || c.max_physics_residual<=0 || ...
    c.state_safety_limit<=0 || c.u_max<=0 || c.u_slew_max_per_s<=0 || ...
    c.blend_alpha<=0 || c.blend_alpha>1 || c.integ_limit<0 || c.windup_tau_s<0 || ...
    c.safety_predict_s<0 || hold_budget==0 || hold_budget==intmax('uint32') || ...
    c.residual_clear_ratio<0 || c.residual_clear_ratio>1 || ...
    c.min_ai_confidence<0 || c.min_ai_confidence>1 || ...
    c.confidence_clear_ratio<c.min_ai_confidence || c.confidence_clear_ratio>1 || ...
    c.safety_trip_ratio<=0 || c.safety_trip_ratio>1 || c.safety_clear_ratio<0 || ...
    c.safety_clear_ratio>c.safety_trip_ratio;
end

function [st,out]=core_step(st,c,t,in)
% Mirror of Layered_Step().  in carries the nine LayeredInput_t floats; an
% optional tenth element is the slow-frame sequence number (0 = legacy
% transport-age mode).  Every host value is quantized to binary32 at the point
% the C core rounds it so the closed-loop C replay matches bit for bit.
seq=uint32(0); if numel(in)>=10, seq=uint32(in(10)); end
in=single(in(1:9));
config_bad=config_corrupt(c);
hold_budget=uint32(c.max_hold_ticks);
veto_threshold=uint32(c.max_fault_vetoes_before_latch);
dt=abs(c.dt_s); du=abs(c.u_slew_max_per_s)*dt;
u_max=abs(c.u_max); integ_limit=abs(c.integ_limit);
fast_bad=any(~isfinite(in(4:9))) || any(abs(in(4:9))>single(3e38)) || in(4)<0;
mailbox_bad=any(~isfinite(in(1:3))) || any(abs(in(1:3))>single(3e38)) || ...
    in(1)<1 || in(1)>4 || in(2)<0 || in(2)>1 || in(3)<0;
% ---- 0. split input firewall and bounded native HOLD --------------------
if fast_bad || config_bad
    st.fault=bitor(st.fault,uint32(16));
    st.corrupted_frame_count=st.corrupted_frame_count+uint32(1);
    st.recovery=uint32(c.ood_recovery_cooldown_ticks);
    st.step_count=st.step_count+uint32(1);
    % A rejected configuration has no trustworthy hold budget: use the fixed
    % fail-safe budget.  hold_ticks always advances so a broken sensor or a
    % broken configuration can never hold the last command forever.
    fw_budget=hold_budget; if config_bad, fw_budget=uint32(100); end
    st.hold_ticks=st.hold_ticks+uint32(1);
    if st.hold_ticks>fw_budget
        st.fault=bitor(st.fault,bitor(uint32(32),uint32(64)));
        st.permanent=true; st.u_target=single(0); st.integ(:)=single(0);
        st.blend=single(0); st.regime=4;
        if config_bad
            % The slew limit belongs to the rejected configuration.
            st.u_applied=single(0);
        else
            st.u_applied=clamp(single(0),st.u_applied-du,st.u_applied+du);
        end
        out=[double(st.u_applied),0,5,4,double(st.blend),double(st.fault),1];
        return;
    end
    out=[double(st.u_applied),double(st.u_target),1,st.regime,double(st.blend),double(st.fault),1];
    return;
end
% ---- 0b. tuning firewall and slow mailbox ------------------------------
tuning_bad=any(~isfinite(t)) || any(abs(t)>single(3e38)) || ...
    any(t<single(.1)) || t(1)>4 || t(2)>4 || t(3)>1;
if tuning_bad, t=single([1 1 1]); end
outer_bad=mailbox_bad || tuning_bad;
if outer_bad
    st.fault=bitor(st.fault,uint32(16));
    st.corrupted_frame_count=st.corrupted_frame_count+uint32(1);
end
snap=~st.started;
conf_before=st.conf; safety_before=st.safety; ood_before=st.ood;
% ---- 0c. L1 slow-frame lease -------------------------------------------
age_eff=in(3); seq_bad=false;
if seq~=0
    if st.last_outer_seq==0 || seq~=st.last_outer_seq
        st.last_outer_seq=seq; st.outer_age_ticks=uint32(0);
    else
        st.outer_age_ticks=st.outer_age_ticks+uint32(1);
    end
    age_eff=single(st.outer_age_ticks)*dt;
elseif st.last_outer_seq~=0
    seq_bad=true;
end
% A live sequence that falls back to zero is a mailbox anomaly of its own, not
% a fresh frame: record the corruption exactly once on this tick (a mailbox
% that was already corrupt has been counted above).
if seq_bad && ~outer_bad
    outer_bad=true;
    st.corrupted_frame_count=st.corrupted_frame_count+uint32(1);
    st.fault=bitor(st.fault,uint32(16));
end
% ---- 1. hard safety envelope -------------------------------------------
limit=abs(c.state_safety_limit*t(3));
xa=abs(in(5)); pred=xa+abs(c.safety_predict_s)*abs(in(6));
if st.safety
    clear_level=c.safety_clear_ratio*limit;
    if xa<clear_level && pred<clear_level, st.safety=false; end
elseif xa>c.safety_trip_ratio*limit || pred>limit
    st.safety=true; st.fault=bitor(st.fault,uint32(8));
    st.safety_veto_count=st.safety_veto_count+uint32(1);
    st.latch_episode_count=st.latch_episode_count+uint32(1);
end
% ---- 2. physics-residual OOD veto --------------------------------------
ra=abs(in(4));
if st.ood
    if ra<c.residual_clear_ratio*c.max_physics_residual, st.ood=false; end
elseif ra>c.max_physics_residual
    st.ood=true; st.fault=bitor(st.fault,uint32(2));
    st.ood_veto_count=st.ood_veto_count+uint32(1);
    st.latch_episode_count=st.latch_episode_count+uint32(1);
end
% ---- 3. confidence / explicit abstention -------------------------------
mailbox=1+double(in(1)>=1.5)+double(in(1)>=2.5)+double(in(1)>=3.5);
if outer_bad, mailbox=4; end
if outer_bad || seq_bad || in(2)<c.min_ai_confidence || in(1)>=3.5
    st.conf=true; st.fault=bitor(st.fault,uint32(4));
elseif in(2)>=c.confidence_clear_ratio && in(1)<3.5
    st.conf=false;
end
conf_veto=st.conf;
if conf_veto && ~conf_before
    st.low_confidence_veto_count=st.low_confidence_veto_count+uint32(1);
end
% ---- 3b. cumulative severe-fault latch ---------------------------------
if double(veto_threshold)>0 && ...
        ((st.safety && ~safety_before) || (st.ood && ~ood_before)) && ...
        st.latch_episode_count>=veto_threshold
    st.permanent=true; st.fault=bitor(st.fault,uint32(64));
end
% ---- 4. heartbeat watchdog ---------------------------------------------
stale=isfinite(age_eff) && abs(age_eff)<=single(3e38) && age_eff>c.outer_timeout_s;
if stale
    st.fault=bitor(st.fault,uint32(1));
    st.recovery=uint32(c.ood_recovery_cooldown_ticks);
end
% ---- 5. decision priority ----------------------------------------------
veto_active=st.safety || st.ood || conf_veto;
old=st.regime;
if st.safety
    gate=4; reg=4;
elseif st.ood
    gate=2; reg=4;
elseif conf_veto
    gate=3; reg=4;
elseif stale
    gate=1; reg=old;
else
    gate=0;
    if st.recovery>0
        st.recovery=st.recovery-uint32(1); reg=4;
    elseif mailbox==old
        reg=old;
    elseif st.dwell+uint32(1)>=c.min_dwell_ticks
        reg=mailbox;
    else
        reg=old;
    end
end
if veto_active
    st.recovery=uint32(c.ood_recovery_cooldown_ticks);
end
% The cumulative latch outranks every other decision.
if st.permanent
    gate=6; reg=4;
end
% ---- 5b. bounded native HOLD -------------------------------------------
if gate==1
    % Diagnostics only: one count per HOLD episode, on the entry tick of a
    % native hold, never one per held tick or per overshooting age window.
    if st.hold_ticks==uint32(0)
        st.latency_hold_count=st.latency_hold_count+uint32(1);
    end
    st.hold_ticks=st.hold_ticks+uint32(1);
    if st.hold_ticks>hold_budget, gate=5; end
else
    st.hold_ticks=uint32(0);
end
if gate==5
    st.fault=bitor(st.fault,uint32(32));
    st.u_target=single(0); st.integ(:)=single(0); st.blend=single(0);
    st.regime=4; reg=4;
    st.u_applied=clamp(single(0),st.u_applied-du,st.u_applied+du);
end
% ---- 6. controller bank, tracking anti-windup, bumpless transfer -------
do_control=(gate~=1) && (gate~=5);
if do_control
    e=in(7)-in(5); ed=in(8)-in(6);
    bank=zeros(4,1,'single');
    bank(1:3)=(c.kp(1:3)*t(1))*e+(c.kd(1:3)*t(2))*ed+ ...
        (c.ki(1:3)*t(1)).*st.integ(1:3)+c.ff_scale(1:3)*in(9);
    bank(4)=c.kp(4)*(-in(5))+c.kd(4)*(-in(6))+c.ki(4)*st.integ(4);
    idx=reg; if idx<1, idx=1; elseif idx>4, idx=4; end
    u_active=bank(idx);
    if snap
        % Very first control step: take the target as-is so the command never
        % has to ramp up from zero.
        st.u_target=u_active; st.blend=single(1);
    elseif (veto_active || st.permanent) && reg==4
        % Emergency R4 entry bypasses the slow blend filter.
        st.u_target=u_active; st.blend=single(1);
    else
        if reg~=old, st.blend=single(0); end
        st.u_target=st.u_target+c.blend_alpha*(u_active-st.u_target);
        st.blend=clamp(st.blend+c.blend_alpha,single(0),single(1));
    end
    if isnan(st.u_target), st.u_target=single(0); end
    command=clamp(st.u_target,-u_max,u_max);
    command=clamp(command,st.u_applied-du,st.u_applied+du);
    for j=1:4
        if j==4, ki=c.ki(4); err=-in(5);
        else, ki=c.ki(j)*t(1); err=e; end
        next=st.integ(j)+err*dt;
        if ki~=0 && c.windup_tau_s>0
            next=next+(command-bank(j))*dt/(ki*c.windup_tau_s);
        end
        st.integ(j)=clamp(next,-integ_limit,integ_limit);
    end
    st.u_applied=command; st.started=true;
    out=[double(command),double(st.u_target),gate,reg,double(st.blend),double(st.fault),double(outer_bad)];
else
    out=[double(st.u_applied),double(st.u_target),gate,reg,double(st.blend),double(st.fault),double(outer_bad)];
end
% ---- 7. dwell bookkeeping and step counter -----------------------------
if reg~=old
    st.dwell=uint32(0);
else
    st.dwell=st.dwell+uint32(1);
end
st.regime=reg;
st.step_count=st.step_count+uint32(1);
end

function y=clamp(x,lo,hi)
if isnan(x), x=0; end
y=min(max(x,lo),hi);
end
