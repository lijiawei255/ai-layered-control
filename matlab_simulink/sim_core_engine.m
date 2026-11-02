function [out, diagnostics] = sim_core_engine(P, opts)
%SIM_CORE_ENGINE Three controllers on a shared four-stage physical fixture.
if nargin < 1, P = load_domain_profile(); end
if ischar(P) || isstring(P), P = load_domain_profile(char(P)); end
if nargin < 2, opts = struct(); end
defaults = struct('branches',[1 2 3],'mode','dynamic','fixed_regime',[], ...
    'seed',7,'drift',.35,'scenario',[],'duration',40,'substeps',2, ...
    'ood',true,'jitter',true,'spikes',true,'bias',true,'wallphase',.4, ...
    'noise_scale',1,'fault_f',[12 24],'zeta',.012,'wallk',300,'wallx',.43, ...
    'tuning',[1 1 1],'ood_gate',true,'conf_gate',true,'ffsafe',0, ...
    'auto_calibrate',true,'log_core',false,'amplitude',[],'probe_spikes',true);
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
sigma=repmat(.0015,n,1); sigma(stage==2)=.015;
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
    'me',0.,'mu',0.,'hf',0.,'sp',0.,'rough',.0017^2,'noisef',x, ...
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
pr.y1=pr.y1+aa*(med-pr.y1); pr.y2=pr.y2+aa*(pr.y1-pr.y2);
actual=(pr.y2-2*pr.yold+pr.yolder)/(dt*dt); pr.yolder=pr.yold; pr.yold=pr.y2;
predicted=(u-p(2)*pr.vf-p(3)*ym-friction(pr.vf,p))/p(1);
pr.apbuf=[pr.apbuf,predicted]; if numel(pr.apbuf)>3, pr.apbuf=pr.apbuf(2:end); end
pr.a1=pr.a1+aa*(pr.apbuf(1)-pr.a1); pr.a2=pr.a2+aa*(pr.a1-pr.a2);
ar=actual-pr.a2; pr.alp=pr.alp+dt/(dt+1/(2*pi*5))*(ar-pr.alp);
pr.ae=pr.ae+dt/.008*((ar-pr.alp)^2-pr.ae); pr.hf=sqrt(max(pr.ae,0))/500;
pr.xh=xp+dt*40*innov; pr.vh=vp+dt*400*innov;
vd=(ym-pr.yf)/.002; pr.yf=pr.yf+dt/.002*(ym-pr.yf); pr.vf=pr.vf+dt/.002*(vd-pr.vf);
pr.noisef=pr.noisef+dt/.003*(ym-pr.noisef);
pr.rough=pr.rough+dt/.15*((ym-pr.noisef)^2-pr.rough);
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
x=c.r(1); v=0.; xm=x; vm=0.; wasflex=false; u0=0.; integ=0.; pr=probe_init(x);
ctx=zeros(1,4); frames=zeros(ceil(n/100),4); last_stamp=0.; cfg=P.cfg; cfg.ff_scale(4)=o.ffsafe;
if ~isempty(P.classifier)
    cfg.min_ai_confidence=P.classifier.trip; cfg.confidence_clear_ratio=P.classifier.clear;
end
ckeys=fieldnames(cfg); for j=1:numel(ckeys), cfg.(ckeys{j})=single(cfg.(ckeys{j})); end
tune=single(o.tuning); core=core_init(); ai=1; conf=1.; calls=0;
logs=zeros(n,14); macro=zeros(ceil(n/100),5);
age=zeros(n,1);
CI=zeros(n,9*double(o.log_core)); CO=zeros(n,6*double(o.log_core));
for k=1:n
    r=c.r(k); rd=c.rd(k); rdd=c.rdd(k);
    [pr,ym,yd,res]=probe_step(pr,x+c.noise(k)+c.sfault(k)+c.spike(k),u0,r,nom,dt,o.probe_spikes);
    if mod(k-1,100)==0
        mi=floor((k-1)/100)+1;
        q=sqrt(max([pr.me,pr.mu,pr.slowenergy,pr.rough],0));
        frames(mi,:)=q./(q+P.feature_scales).*[1,1,.5,1.4];
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
        u=clamp(clamp(raw,-20,20),u0-2,u0+2);
        if ki~=0, integ=clamp(integ+dt*e+(u-raw)*dt/(ki*.1),-.5,.5); end
    else
        if ~isempty(o.fixed_regime), ai=o.fixed_regime; conf=1.; end
        if strcmp(mode,'rules')
            if ctx(4)>.65, ai=3; elseif ctx(3)>.25 || ctx(2)>.55, ai=2; else, ai=1; end
            conf=1.;
        end
        ci=conf; if ~o.conf_gate, ci=1.; end
        mai=ai; if age(k)>P.cfg.outer_timeout_s, mai=4; ci=0.; end
        residual=res; if ~o.ood_gate, residual=0.; end
        input=single([mai,ci,age(k),residual,ym,yd,r,rd,uff]);
        [core,result]=core_step(core,cfg,tune,input);
        u=result(1); gate=result(3); reg=result(4);
        if o.log_core, CI(k,:)=double(input); CO(k,:)=double(result); end
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

function st=core_init()
st=struct('u_applied',single(0),'u_target',single(0),'integ',zeros(4,1,'single'), ...
    'blend',single(0),'started',false,'regime',1,'ood',false,'safety',false, ...
    'conf',false,'dwell',0,'recovery',0,'fault',uint32(0));
end

function [st,out]=core_step(st,c,t,in)
bad=any(~isfinite(in)) || any(abs(in)>single(3e38)) || in(1)<1 || in(1)>4 || ...
    in(2)<0 || in(2)>1 || in(3)<0 || in(4)<0;
fields={'dt_s','outer_timeout_s','max_physics_residual','residual_clear_ratio', ...
 'min_ai_confidence','confidence_clear_ratio','state_safety_limit','safety_trip_ratio', ...
 'safety_clear_ratio','safety_predict_s','u_max','u_slew_max_per_s','blend_alpha','integ_limit','windup_tau_s'};
values=[t(:);c.kp;c.kd;c.ki;c.ff_scale;zeros(numel(fields),1,'single')];
for j=1:numel(fields), values(19+j)=c.(fields{j}); end
bad=bad || any(~isfinite(values)) || any(abs(values)>single(3e38));
if bad
    st.fault=bitor(st.fault,uint32(16)); st.recovery=double(c.ood_recovery_cooldown_ticks);
    out=[double(st.u_applied),double(st.u_target),1,st.regime,double(st.blend),double(st.fault)]; return;
end
dt=abs(c.dt_s); du=abs(c.u_slew_max_per_s)*dt;
limit=abs(c.state_safety_limit*t(3)); xa=abs(in(5)); pred=xa+abs(c.safety_predict_s)*abs(in(6));
if st.safety
    if xa<c.safety_clear_ratio*limit && pred<c.safety_clear_ratio*limit, st.safety=false; end
elseif xa>c.safety_trip_ratio*limit || pred>limit
    st.safety=true; st.fault=bitor(st.fault,uint32(8));
end
if st.ood
    if abs(in(4))<c.residual_clear_ratio*c.max_physics_residual, st.ood=false; end
elseif abs(in(4))>c.max_physics_residual
    st.ood=true; st.fault=bitor(st.fault,uint32(2));
end
mailbox=1+(in(1)>=1.5)+(in(1)>=2.5)+(in(1)>=3.5);
if in(2)<c.min_ai_confidence || in(1)>=3.5
    st.conf=true; st.fault=bitor(st.fault,uint32(4));
elseif in(2)>=c.confidence_clear_ratio && in(1)<3.5
    st.conf=false;
end
stale=in(3)>c.outer_timeout_s;
if stale, st.fault=bitor(st.fault,uint32(1)); st.recovery=double(c.ood_recovery_cooldown_ticks); end
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
    elseif st.dwell+1>=c.min_dwell_ticks
        reg=double(mailbox);
    else
        reg=old;
    end
end
if st.safety || st.ood || st.conf, st.recovery=double(c.ood_recovery_cooldown_ticks); end
if gate~=1
    e=in(7)-in(5); ed=in(8)-in(6);
    bank=(c.kp*t(1))*e+(c.kd*t(2))*ed+(c.ki*t(1)).*st.integ+c.ff_scale*in(9);
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
        ki=c.ki(j)*t(1); next=st.integ(j)+e*dt;
        if ki~=0 && c.windup_tau_s>0
            next=next+(command-bank(j))*dt/(ki*c.windup_tau_s);
        end
        st.integ(j)=clamp(next,-abs(c.integ_limit),abs(c.integ_limit));
    end
    st.u_applied=command; st.started=true;
end
if reg~=old, st.dwell=0; else, st.dwell=st.dwell+1; end
st.regime=reg;
out=[double(st.u_applied),double(st.u_target),gate,reg,double(st.blend),double(st.fault)];
end

function y=clamp(x,lo,hi)
if isnan(x), x=0; end
y=min(max(x,lo),hi);
end
