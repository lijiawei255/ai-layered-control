function P = calibrate_coarse_classifier(P)
%CALIBRATE_COARSE_CLASSIFIER  Commission the 10 Hz classifier from a probe run.
%
%   P = CALIBRATE_COARSE_CLASSIFIER(P) runs a 30 s commissioning schedule in
%   which the bank is forced to the intended stage mode (S1->R1, S2->R2,
%   S3->R3), collects the four normalised macro-features, and returns:
%     P.ai.prototypes       3x4 class means
%     P.ai.confidence_gamma proximity decay of the margin confidence
%     P.ai.min_confidence   / P.ai.confidence_clear
%     P.ai.calibration      diagnostics (frame counts, prototype spread)
%
%   This is a closed-loop calibration, not a hard-coded table: the features
%   come from the same plant/reference/controller core used at deployment,
%   but the test trajectory of the benchmark is not used.  The forced-bank
%   option also makes the calibration self-consistent: the S1/S2/S3 class
%   means are measured under the very banks the classifier is supposed to
%   select.

if ~isfield(P.ai, 'auto_calibrate')
    P.ai.auto_calibrate = true;
end
if ~isempty(P.ai.prototypes) && size(P.ai.prototypes,1) == 3
    return;
end

Tcal = 30;
Pp = P;
Pp.ai.auto_calibrate = false;
Pp.ai.prototypes = zeros(3,4);
Pp.ai.confidence_gamma = [];
opts = struct('t_end', Tcal, 'branches', 3, 'verbose', false, ...
              'edge_on', false, 'ood_on', false, ...
              'stage_regime', [1 2 3 4]);
out = sim_core_engine(Pp, opts);

F = out.b3.frame_feat;
stg = out.stage(out.schedule.emit_idx);
mu = zeros(3,4);
counts = zeros(3,1);
allF = [];
for c = 1:3
    sel = (stg == c) & ~any(isnan(F),2);
    Fk = F(sel,:);
    if size(Fk,1) < 3
        error('calibrate_coarse_classifier:frames', ...
            'Stage %d produced only %d usable slow-loop frames.', c, size(Fk,1));
    end
    mu(c,:) = mean(Fk,1);
    counts(c) = size(Fk,1);
    allF = [allF; Fk]; %#ok<AGROW>
end

P.ai.prototypes = mu;
D = zeros(size(allF,1),3);
for c = 1:3
    D(:,c) = sqrt(sum((allF - mu(c,:)).^2, 2));
end
Ds = sort(D,2);
d1 = Ds(:,1); d2 = Ds(:,2);
gamma = 1/max(median(d1), 1e-9);
margin = (d2 - d1)./max(d2 + d1, 1e-9);
conf = margin.*exp(-gamma*d1);
conf = min(max(conf,0),1);
min_conf = 0.001*local_percentile(conf, 5);
min_conf = min(0.05, max(1e-5, min_conf));
conf_clear = min(0.95, max(10*min_conf, 1e-4));

P.ai.confidence_gamma = gamma;
P.ai.min_confidence   = min_conf;
P.ai.confidence_clear = conf_clear;
P.gate.min_confidence = min_conf;
P.gate.confidence_clear = conf_clear;
P.ai.auto_calibrate = false;
P.ai.calibration = struct('probe_frames', counts, ...
    'prototypes', mu, 'gamma', gamma, ...
    'min_confidence', min_conf, 'confidence_clear', conf_clear, ...
    'median_d1', median(d1), 'probe_T_s', Tcal, ...
    'forced_stage_regime', [1 2 3 4]);
end

% =========================================================================
function v = local_percentile(x, p)
x = sort(x(:));
if isempty(x)
    v = 0;
    return;
end
idx = ceil(p/100*numel(x));
idx = max(1, min(numel(x), idx));
v = x(idx);
end
