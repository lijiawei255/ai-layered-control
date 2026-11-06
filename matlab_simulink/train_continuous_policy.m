% SPDX-License-Identifier: MIT
% Copyright (c) 2026 Jiawei Li
% Source: https://github.com/lijiawei255/ai-layered-control
% License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
% Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
function policy = train_continuous_policy(P, nSamples, nCenters, unit)
%TRAIN_CONTINUOUS_POLICY Conditional 1 kHz action head with 10 Hz context.
% Mirrors the Python train_policy(): ten input features combine tracking
% kinematics, slow context and the clamped error integral in column 10,
% normalized by the 0.5 integrator limit; training consumes 10*nSamples draws
% from the shared 32-bit LCG.
if nargin<1, P=load_domain_profile(); end
if nargin<2, nSamples=16000; end
if nargin<3, nCenters=512; end
if nargin<4, unit=0; end
state=uint32(933); values=zeros(10*nSamples,1);
for j=1:numel(values)
    state=uint32(bitand(uint64(1664525)*uint64(state)+uint64(1013904223),uint64(4294967295)));
    values(j)=(double(state)+.5)/4294967296;
end
X=reshape(values,10,nSamples)'*2-1;
index=(0:nSamples-1)'; near=mod(index,4)~=0;
for kind=0:1
    q=near & mod(index,2)==kind;
    [r,rd,rdd]=training_reference(kind,(X(q,5)+1)*5);
    X(q,1)=X(q,1)*.06;
    X(q,2)=15*X(q,1)+.4*X(q,2);
    X(q,3:5)=[r/.5,rd/2,rdd/6];
end
X(:,6:9)=(X(:,6:9)+1)/2; X(:,9)=X(:,9)*1.4;
X(near,6:8)=X(near,6:8).*[.03,.10,.06]; X(near,9)=0;
% Column 10 is the clamped error-integral history state, normalised by the 0.5
% integrator limit so the training support covers the full [-integ_limit,
% +integ_limit] range; its value stays in [-1,1] straight from the LCG.
h=min(max((X(:,9)-.65)/.35,0),1);
kp=60*(1-h)+8*h; kd=1.2*(1-h)+.8*h; p=P.nominal;
ff=p.J*(1+unit)*6*X(:,5)+p.b*2*X(:,4)+p.k*.5*X(:,3)+p.fc*sign(X(:,4));
% The training target carries the B1 baseline integral gain (BASE[3] = 300 on
% the Python side, P.baseline(3) here) times the normalised integral feature.
Y=kp.*(.1*X(:,1))+kd.*X(:,2)+P.baseline(3)*.5*X(:,10)+ff;
if unit~=0
    Y=Y+(1.2*(1+4.5*unit)-p.fc)*sign(2*X(:,4)-X(:,2));
end
mu=mean(X,1); sd=max(std(X,1,1),.05); Z=(X-mu)./sd;
nCenters=min(nCenters,nSamples); indices=ones(nCenters,1);
dist=sum((Z-Z(1,:)).^2,2);
for j=2:nCenters
    [~,indices(j)]=max(dist);
    dist=min(dist,sum((Z-Z(indices(j),:)).^2,2));
end
centers=Z(indices,:); d=zeros(nCenters);
for j=1:nCenters, d(:,j)=sum((centers-centers(j,:)).^2,2); end
d(1:nCenters+1:end)=Inf;
width=2*sqrt(median(min(d,[],2)));
radial=exp(-max(sum(Z.^2,2)+sum(centers.^2,2)'-2*Z*centers',0)/(2*width^2));
phi=[ones(nSamples,1),Z,radial];
w=(phi'*phi+1e-7*nSamples*eye(size(phi,2)))\(phi'*Y);
policy=struct('mu',mu,'sd',sd,'centers',centers,'width',width,'w',w);
end

function [r,v,a]=training_reference(kind,t)
if kind==0
    r=.5*cos(pi*t); v=-.5*pi*sin(pi*t); a=-.5*pi*pi*cos(pi*t); return;
end
move=.9; dwell=.9; u=mod(t+.8,2*(move+dwell));
r=zeros(size(t)); v=r; a=r;
r(u<dwell)=.5; r(u>=dwell+move & u<2*dwell+move)=-.5;
for side=1:2
    if side==1, q=u>=dwell & u<dwell+move; tau=(u(q)-dwell)/move; sg=-1;
    else, q=u>=2*dwell+move; tau=(u(q)-2*dwell-move)/move; sg=1; end
    pp=tau.*tau.*tau.*(10+tau.*(-15+6*tau));
    vv=tau.*tau.*(30-60*tau+30*tau.*tau);
    aa=tau.*(60-180*tau+120*tau.*tau);
    r(q)=-sg*.5+sg*pp; v(q)=sg*vv/move; a(q)=sg*aa/(move*move);
end
end
