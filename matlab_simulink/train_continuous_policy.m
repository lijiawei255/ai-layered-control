function policy = train_continuous_policy(P, nSamples, nCenters, unit)
%TRAIN_CONTINUOUS_POLICY Conditional 1 kHz action head with 10 Hz context.
if nargin<1, P=load_domain_profile(); end
if nargin<2, nSamples=16000; end
if nargin<3, nCenters=512; end
if nargin<4, unit=0; end
state=uint32(933); values=zeros(9*nSamples,1);
for j=1:numel(values)
    state=uint32(bitand(uint64(1664525)*uint64(state)+uint64(1013904223),uint64(4294967295)));
    values(j)=(double(state)+.5)/4294967296;
end
X=reshape(values,9,nSamples)'*2-1;
X(:,6:9)=(X(:,6:9)+1)/2; X(:,9)=X(:,9)*1.4;
h=min(max((X(:,9)-.65)/.35,0),1);
kp=60*(1-h)+8*h; kd=1.2*(1-h)+.8*h; p=P.nominal;
ff=p.J*(1+unit)*6*X(:,5)+p.b*2*X(:,4)+p.k*.5*X(:,3)+p.fc*sign(X(:,4));
Y=kp.*(.1*X(:,1))+kd.*X(:,2)+ff;
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
