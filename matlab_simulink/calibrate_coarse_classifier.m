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
trip=.15*linear_quantile(score,.01);
classifier=struct('prototypes',prototypes,'gamma',gamma,'trip',trip,'clear',1.25*trip);
end

function q=linear_quantile(x,p)
x=sort(x); pos=1+(numel(x)-1)*p; lo=floor(pos); hi=ceil(pos);
q=x(lo)+(pos-lo)*(x(hi)-x(lo));
end
