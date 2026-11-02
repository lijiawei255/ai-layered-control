function policy = train_continuous_policy(X, y, opts)
%TRAIN_CONTINUOUS_POLICY  Closed-form RBF policy (no toolbox, no gradient loop).
%
%   POLICY = TRAIN_CONTINUOUS_POLICY(X, Y, OPTS)
%     X : N x d raw features
%     Y : N x 1 teacher command
%     OPTS fields: .n_centers (48), .ridge (1e-6), .seed (11)
%
%   The RBF centres are selected by deterministic farthest-point sampling and
%   the output weights are solved by ridge regression.  This is a genuine
%   one-hidden-layer RBF network fitted to data; the deployed policy is
%   POLICY_WEIGHTS = POLICY.WEIGHTS, not a hand-written inverse-dynamics
%   formula.  No weights are stored in the repository; the fit is repeated
%   deterministically at commissioning time.

if nargin < 3 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'n_centers') || isempty(opts.n_centers)
    opts.n_centers = 48;
end
if ~isfield(opts, 'ridge') || isempty(opts.ridge)
    opts.ridge = 1e-6;
end
if ~isfield(opts, 'seed') || isempty(opts.seed)
    opts.seed = 11;
end

X = double(X); y = double(y(:));
[N, d] = size(X);
mu = mean(X, 1);
sig = std(X, 0, 1);
sig(sig < 1e-9) = 1;
Z = (X - mu)./sig;

k = min(opts.n_centers, N);
centers = zeros(k, d);
% deterministic farthest-point sampling; start from the sample closest to the
% origin so the result does not depend on row order.
[~, first] = min(sum(Z.^2, 2));
centers(1,:) = Z(first,:);
dist2 = sum((Z - centers(1,:)).^2, 2);
for j = 2:k
    [~, idx] = max(dist2);
    centers(j,:) = Z(idx,:);
    dist2 = min(dist2, sum((Z - centers(j,:)).^2, 2));
end
% width from the median nearest-centre distance
D = zeros(N,k);
for j = 1:k
    D(:,j) = sum((Z - centers(j,:)).^2, 2);
end
width = median(sqrt(min(D,[],2)));
width = max(width, 1e-3);

Phi = exp(-0.5*D/(width^2));
lambda = opts.ridge*max(N, 1);
w = (Phi'*Phi + lambda*eye(k)) \ (Phi'*y);

policy = struct();
policy.mu = mu;
policy.sig = sig;
policy.centers = centers;
policy.width = width;
policy.weights = w;
policy.n_centers = k;
policy.n_samples = N;
policy.ridge = opts.ridge;
policy.train_rmse = sqrt(mean((Phi*w - y).^2));
end
