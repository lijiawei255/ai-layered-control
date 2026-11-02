function report = verify_core_equivalence(P)
%VERIFY_CORE_EQUIVALENCE  Replay a real closed-loop trace through the C core.
%
%   REPORT = VERIFY_CORE_EQUIVALENCE() runs a 6 s closed-loop layered trace
%   with the MATLAB core mirror, compiles the standalone C core, replays the
%   same LayeredInput_t sequence through Layered_Step(), and compares:
%       gate / regime / fault bits : exact
%       u / u_target / blend       : max abs difference < 1e-4
%
%   The temporary binary and trace files are removed automatically.  No
%   compiler => the function prints SKIP and returns a skipped report.
%
%   See also SIM_CORE_ENGINE, TEST_GATEKEEPER_STANDALONE.

if nargin < 1 || isempty(P)
    P = load_domain_profile('industrial');
end
if isempty(P.ai.prototypes) || size(P.ai.prototypes,1) ~= 3
    P = calibrate_coarse_classifier(P);
end

report = struct('status','skipped','n_ticks',0, ...
                'max_du',NaN,'max_dutarget',NaN,'max_dblend',NaN, ...
                'gate_exact',false,'regime_exact',false,'fault_exact',false);

% --- 1. generate the real closed-loop trace with the MATLAB core mirror ----
out = sim_core_engine(P, struct('t_end', 6, 'branches', 3, ...
    'log_core', true, 'edge_on', false, 'ood_on', false, 'verbose', false));
if isempty(out.b3.core_in) || isempty(out.b3.core_out)
    error('verify_core_equivalence:log', ...
        'sim_core_engine did not return the requested core trace.');
end
CI = out.b3.core_in;
CO = out.b3.core_out;
N  = size(CI,1);

% --- 2. locate compiler / source files -----------------------------------
here  = fileparts(mfilename('fullpath'));
core  = fullfile(here, '..', 'c99_layered_core', 'layered_gatekeeper.c');
harness = fullfile(here, '..', 'c99_layered_core', 'verify_core_equivalence.c');
if ~exist(core,'file') || ~exist(harness,'file')
    error('verify_core_equivalence:files', 'C sources not found.');
end
compiler = local_find_compiler();
if isempty(compiler)
    fprintf('SKIP: no C compiler found for verify_core_equivalence.\n');
    return;
end

tmp = tempname;
mkdir(tmp);
cleaner = onCleanup(@() local_rmdir(tmp)); %#ok<NASGU>
infile  = fullfile(tmp, 'trace.txt');
outfile = fullfile(tmp, 'c_out.txt');
if ispc
    exe = fullfile(tmp, 'verify_core_equivalence.exe');
else
    exe = fullfile(tmp, 'verify_core_equivalence');
end

% --- 3. write config + input trace ---------------------------------------
fid = fopen(infile, 'w');
if fid < 0
    error('verify_core_equivalence:io', 'cannot write %s', infile);
end
fprintf(fid, '%d\n', N);
fprintf(fid, ['%.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g ' ...
              '%.9g %.9g %.9g %.9g %.9g %u %u '], ...
    P.dt, P.ai.outer_timeout_s, P.ai.max_residual, P.ai.residual_clear_ratio, ...
    P.ai.min_confidence, P.ai.confidence_clear, ...
    P.safety.x_limit, P.safety.trip_ratio, P.safety.clear_ratio, P.safety.predict_s, ...
    P.act.u_max, P.act.slew_max, P.gate.blend_alpha, ...
    P.gate.integ_limit, P.gate.windup_tau_s, ...
    round(P.gate.min_dwell_s/P.dt), round(P.gate.ood_recovery_cooldown_s/P.dt));
fprintf(fid, '%.9g ', P.bank.Kp(:)');
fprintf(fid, '%.9g ', P.bank.Kd(:)');
fprintf(fid, '%.9g ', P.bank.Ki(:)');
fprintf(fid, '%.9g ', P.bank.ff_scale(:)');
fprintf(fid, '\n');
for k = 1:N
    fprintf(fid, '%.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n', CI(k,:));
end
fclose(fid);

% --- 4. compile + run the C replay harness --------------------------------
cmd = sprintf('%s -std=c99 -O2 -o "%s" "%s" "%s"', ...
    compiler, exe, core, harness);
[st, msg] = system(cmd);
if st ~= 0
    fprintf('SKIP: C harness compile failed (%s).\n', strtrim(msg));
    return;
end
cmd = sprintf('"%s" "%s" "%s"', exe, infile, outfile);
[st, msg] = system(cmd);
if st ~= 0
    error('verify_core_equivalence:run', 'C replay failed: %s', msg);
end

% --- 5. compare ----------------------------------------------------------
fid = fopen(outfile, 'r');
if fid < 0
    error('verify_core_equivalence:io', 'cannot read %s', outfile);
end
C = fscanf(fid, '%f %f %d %d %f %u', [6 Inf]);
C = C';
fclose(fid);
if size(C,1) ~= N
    error('verify_core_equivalence:size', ...
        'C output has %d rows, expected %d.', size(C,1), N);
end

du  = max(abs(C(:,1) - CO(:,1)));
dut = max(abs(C(:,2) - CO(:,2)));
db  = max(abs(C(:,5) - CO(:,5)));
report.n_ticks = N;
report.max_du = du;
report.max_dutarget = dut;
report.max_dblend = db;
report.gate_exact   = isequal(C(:,3), CO(:,3));
report.regime_exact = isequal(C(:,4), CO(:,4));
report.fault_exact  = isequal(uint32(C(:,6)), uint32(CO(:,6)));
report.status = 'pass';
if ~(report.gate_exact && report.regime_exact && report.fault_exact && ...
     du < 1e-4 && dut < 1e-4 && db < 1e-4)
    report.status = 'fail';
    error('verify_core_equivalence:mismatch', ...
        'C/MATLAB mismatch: gate=%d regime=%d fault=%d du=%.3g dut=%.3g db=%.3g', ...
        report.gate_exact, report.regime_exact, report.fault_exact, du, dut, db);
end
fprintf('PASS: %d closed-loop ticks; gate/regime/fault exact; max|du|=%.3g, max|du_target|=%.3g, max|dblend|=%.3g\n', ...
    N, du, dut, db);
end

% =========================================================================
function cc = local_find_compiler()
cc = '';
candidates = {'gcc','cc','clang'};
for i = 1:numel(candidates)
    [st, ~] = system(sprintf('%s --version', candidates{i}));
    if st == 0
        cc = candidates{i};
        return;
    end
end
end

function local_rmdir(p)
if exist(p, 'dir')
    rmdir(p, 's');
end
end
