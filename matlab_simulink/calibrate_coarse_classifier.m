% SPDX-License-Identifier: MIT
% Copyright (c) 2026 Jiawei Li
% Source: https://github.com/lijiawei255/ai-layered-control
% License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
% Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
function classifier = calibrate_coarse_classifier(P, nPerClass)
%CALIBRATE_COARSE_CLASSIFIER Fit three prototypes from labelled macro windows.
if nargin<1, P=load_domain_profile(); end
if nargin<2, nPerClass=12; end
samples=zeros(3*nPerClass,4);
for s=0:2
    opts=struct('branches',3,'scenario',s,'duration',10,'seed',101, ...
        'drift',0,'ood',false,'fixed_regime',s+1,'auto_calibrate',false);
    z=sim_core_engine(P,opts);
    windows=z.b3.macro(11:end,2:end);
    ix=floor(linspace(0,size(windows,1)-1,nPerClass))+1;
    samples(s*nPerClass+(1:nPerClass),:)=windows(ix,:);
end
labels=repelem((1:3)',nPerClass);
prototypes=zeros(3,4);
for j=1:3, prototypes(j,:)=mean(samples(labels==j,:),1); end
d=zeros(size(samples,1),3);
for j=1:3, d(:,j)=sqrt(sum((samples-prototypes(j,:)).^2,2)); end
d=sort(d,2);
gamma=1/max(linear_quantile(d(:,1),.95),1e-6);
score=(d(:,2)-d(:,1))./(d(:,2)+d(:,1)+1e-9).*exp(-gamma*d(:,1));
% The Python supervisor returns twice the helper's 0.15x quantile trip; the
% clear level is 1.25x the trip.  The caps only bound pathological scores and
% stay inactive for the calibrated benchmark values.
trip=2.0*0.15*linear_quantile(score,.01);
trip=min(trip,0.25);
clear=1.25*trip;
clear=min(clear,0.35);
classifier=struct('prototypes',prototypes,'gamma',gamma,'trip',trip,'clear',clear);
end

function q=linear_quantile(x,p)
% Same formula and index arithmetic as numpy.quantile(method='linear'): the
% virtual position is (n-1)*p without an extra +1/-1 round trip, so the
% interpolation weight matches the Python supervisor bit for bit.
x=sort(x);
pos=(numel(x)-1)*p; lo=floor(pos); hi=min(lo+1,numel(x)-1);
q=x(lo+1)+(pos-lo)*(x(hi+1)-x(lo+1));
end
