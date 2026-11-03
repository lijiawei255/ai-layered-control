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
    'corrupt_slow_at',Inf,'smoothing',false);
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
pr=struct('xh',x,'vh',0.,'lp',0.,'buf',[],'yf',x,'vf',0., ...
    'me',0.,'mu',0.,'hf',0.,'sp',0.,'power',0.,'load_energy',0.,'contact',0., ...
    'slowenergy',0.,'veto',0.,'y1',x,'y2',x,'yold',x,'yolder',x, ...
    'a1',0.,'a2',0.,'alp',0.,'ae',0.,'apbuf',[]);
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
evidence=0; if pr.power/(pr.load_energy+.25)>.03, evidence=pr.power-.10; end
pr.contact=max([pr.contact*(1-dt/8),evidence,0]);
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
tuning=single(o.tuning); core=core_init(); ai=1; conf=1.; calls=0;
logs=zeros(n,14); macro=zeros(ceil(n/100),5);
age=zeros(n,1);
CI=zeros(n,9*double(o.log_core)); CO=zeros(n,7*double(o.log_core));
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
    for mi=c.arrivals{k}
        if c.stamp(mi)>=last_stamp
            last_stamp=c.stamp(mi); ctx=frames(mi,:);
            if ~isempty(P.classifier), [ai,conf]=classify(ctx,P.classifier); end
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
        mai=ai; if age(k)>P.cfg.outer_timeout_s, mai=4; ci=0.; end
        if c.t(k)>=o.corrupt_slow_at, ci=NaN; end
        residual=res; if ~o.ood_gate, residual=0.; end
        input=single([mai,ci,age(k),residual,ym,yd,r,rd,uff]);
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
keys=fieldnames(c); for j=1:numel(keys), c.(keys{j})=single(c.(keys{j})); end
maximum=intmax('uint32'); c.min_dwell_ticks=maximum;
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
passed=true;
end

function st=core_init()
st=struct('u_applied',single(0),'u_target',single(0),'integ',zeros(4,1,'single'), ...
    'blend',single(0),'started',false,'regime',1,'ood',false,'safety',false, ...
    'conf',false,'dwell',uint32(0),'recovery',uint32(0),'fault',uint32(0), ...
    'step_count',uint32(0),'safety_veto_count',uint32(0),'ood_veto_count',uint32(0), ...
    'low_confidence_veto_count',uint32(0),'latency_hold_count',uint32(0),'corrupted_frame_count',uint32(0));
end

function [st,out]=core_step(st,c,t,in)
bad=any(~isfinite(in(4:9))) || any(abs(in(4:9))>single(3e38)) || in(4)<0;
mailbox_bad=any(~isfinite(in(1:3))) || any(abs(in(1:3))>single(3e38)) || ...
    in(1)<1 || in(1)>4 || in(2)<0 || in(2)>1 || in(3)<0;
fields={'dt_s','outer_timeout_s','max_physics_residual','residual_clear_ratio', ...
 'min_ai_confidence','confidence_clear_ratio','state_safety_limit','safety_trip_ratio', ...
 'safety_clear_ratio','safety_predict_s','u_max','u_slew_max_per_s','blend_alpha','integ_limit','windup_tau_s'};
values=[c.kp;c.kd;c.ki;c.ff_scale;zeros(numel(fields),1,'single')];
for j=1:numel(fields), values(16+j)=c.(fields{j}); end
bad=bad || any(~isfinite(values)) || any(abs(values)>single(3e38));
bad=bad || c.dt_s<=0 || c.outer_timeout_s<=0 || c.max_physics_residual<=0 || ...
    c.state_safety_limit<=0 || c.u_max<=0 || c.u_slew_max_per_s<=0 || ...
    c.blend_alpha<=0 || c.blend_alpha>1 || c.integ_limit<0 || c.windup_tau_s<0 || ...
    c.safety_predict_s<0 || c.residual_clear_ratio<0 || c.residual_clear_ratio>1 || ...
    c.min_ai_confidence<0 || c.min_ai_confidence>1 || ...
    c.confidence_clear_ratio<c.min_ai_confidence || c.confidence_clear_ratio>1 || ...
    c.safety_trip_ratio<=0 || c.safety_trip_ratio>1 || c.safety_clear_ratio<0 || ...
    c.safety_clear_ratio>c.safety_trip_ratio;
if bad
    st.fault=bitor(st.fault,uint32(16)); st.recovery=uint32(c.ood_recovery_cooldown_ticks);
    st.corrupted_frame_count=st.corrupted_frame_count+uint32(1);
    st.step_count=st.step_count+uint32(1);
    out=[double(st.u_applied),double(st.u_target),1,st.regime,double(st.blend),double(st.fault),1]; return;
end
tuning_bad=any(~isfinite(t)) || any(abs(t)>single(3e38)) || ...
    any(t<single(.1)) || t(1)>4 || t(2)>4 || t(3)>1;
if tuning_bad, t=single([1 1 1]); end
outer_bad=mailbox_bad || tuning_bad;
if outer_bad
    st.fault=bitor(st.fault,uint32(16));
    st.corrupted_frame_count=st.corrupted_frame_count+uint32(1);
end
dt=abs(c.dt_s); du=abs(c.u_slew_max_per_s)*dt;
limit=abs(c.state_safety_limit*t(3)); xa=abs(in(5)); pred=xa+abs(c.safety_predict_s)*abs(in(6));
if st.safety
    if xa<c.safety_clear_ratio*limit && pred<c.safety_clear_ratio*limit, st.safety=false; end
elseif xa>c.safety_trip_ratio*limit || pred>limit
    st.safety=true; st.fault=bitor(st.fault,uint32(8));
    st.safety_veto_count=st.safety_veto_count+uint32(1);
end
if st.ood
    if abs(in(4))<c.residual_clear_ratio*c.max_physics_residual, st.ood=false; end
elseif abs(in(4))>c.max_physics_residual
    st.ood=true; st.fault=bitor(st.fault,uint32(2));
    st.ood_veto_count=st.ood_veto_count+uint32(1);
end
mailbox=1+(in(1)>=1.5)+(in(1)>=2.5)+(in(1)>=3.5);
if outer_bad, mailbox=4; end
conf_before=st.conf;
if outer_bad || in(2)<c.min_ai_confidence || in(1)>=3.5
    st.conf=true; st.fault=bitor(st.fault,uint32(4));
elseif in(2)>=c.confidence_clear_ratio && in(1)<3.5
    st.conf=false;
end
if st.conf && ~conf_before, st.low_confidence_veto_count=st.low_confidence_veto_count+uint32(1); end
stale=isfinite(in(3)) && abs(in(3))<=single(3e38) && in(3)>c.outer_timeout_s;
if stale
    st.fault=bitor(st.fault,uint32(1)); st.recovery=uint32(c.ood_recovery_cooldown_ticks);
    if in(3)-c.outer_timeout_s<=dt, st.latency_hold_count=st.latency_hold_count+uint32(1); end
end
old=st.regime;
if st.safety, gate=4; reg=4;
elseif st.ood, gate=2; reg=4;
elseif st.conf, gate=3; reg=4;
elseif stale, gate=1; reg=old;
else
    gate=0;
    if st.recovery>0
        st.recovery=st.recovery-1; reg=4;
    elseif mailbox==old
        reg=old;
    elseif st.dwell+uint32(1)>=c.min_dwell_ticks
        reg=double(mailbox);
    else
        reg=old;
    end
end
if st.safety || st.ood || st.conf, st.recovery=uint32(c.ood_recovery_cooldown_ticks); end
if gate~=1
    e=in(7)-in(5); ed=in(8)-in(6);
    bank=zeros(4,1,'single');
    bank(1:3)=(c.kp(1:3)*t(1))*e+(c.kd(1:3)*t(2))*ed+ ...
        (c.ki(1:3)*t(1)).*st.integ(1:3)+c.ff_scale(1:3)*in(9);
    bank(4)=c.kp(4)*(-in(5))+c.kd(4)*(-in(6))+c.ki(4)*st.integ(4);
    if ~st.started, st.u_target=bank(reg); st.blend=single(1);
    else
        if reg~=old, st.blend=single(0); end
        st.u_target=st.u_target+c.blend_alpha*(bank(reg)-st.u_target);
        st.blend=clamp(st.blend+c.blend_alpha,single(0),single(1));
    end
    if isnan(st.u_target), st.u_target=single(0); end
    command=clamp(st.u_target,-abs(c.u_max),abs(c.u_max));
    command=clamp(command,st.u_applied-du,st.u_applied+du);
    for j=1:4
        if j==4, ki=c.ki(4); error=-in(5);
        else, ki=c.ki(j)*t(1); error=e; end
        next=st.integ(j)+error*dt;
        if ki~=0 && c.windup_tau_s>0
            next=next+(command-bank(j))*dt/(ki*c.windup_tau_s);
        end
        st.integ(j)=clamp(next,-abs(c.integ_limit),abs(c.integ_limit));
    end
    st.u_applied=command; st.started=true;
end
if reg~=old, st.dwell=uint32(0); else, st.dwell=st.dwell+uint32(1); end
st.regime=reg;
st.step_count=st.step_count+uint32(1);
out=[double(st.u_applied),double(st.u_target),gate,reg,double(st.blend),double(st.fault),double(outer_bad)];
end

function y=clamp(x,lo,hi)
if isnan(x), x=0; end
y=min(max(x,lo),hi);
end
