% SPDX-License-Identifier: MIT
% Copyright (c) 2026 Jiawei Li
% Source: https://github.com/lijiawei255/ai-layered-control
% License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
% Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
function report = verify_core_equivalence(P, opts)
%VERIFY_CORE_EQUIVALENCE Replay 40,000 identical input frames through C99.
if nargin<1, P=load_domain_profile(); end
if nargin<2, opts=struct(); end
caller_opts=opts;
here=fileparts(mfilename('fullpath'));
core=fullfile(here,'..','c99_layered_core','layered_gatekeeper.c');
harness=fullfile(here,'..','c99_layered_core','verify_core_equivalence.c');
compiler=''; candidates={'gcc','cc','clang'};
configured=getenv('CC'); if ~isempty(configured), candidates=[{configured},candidates]; end
for j=1:numel(candidates)
    [status,~]=system(sprintf('"%s" --version',candidates{j}));
    if status==0, compiler=candidates{j}; break; end
end
assert(~isempty(compiler),'verify_core_equivalence:compiler', ...
    'No C99 compiler found. Set CC or put gcc, cc or clang on PATH.');
if isempty(P.classifier), P.classifier=calibrate_coarse_classifier(P); end
tmp=tempname; mkdir(tmp); cleaner=onCleanup(@() rmdir(tmp,'s'));
infile=fullfile(tmp,'trace.txt'); outfile=fullfile(tmp,'output.txt');
exe=fullfile(tmp,'replay'); if ispc, exe=[exe,'.exe']; end
[status,msg]=system(sprintf('"%s" -std=c99 -O2 -o "%s" "%s" "%s"',compiler,exe,core,harness));
assert(status==0,'verify_core_equivalence:compile','C replay compilation failed: %s',msg);
% Primary replay uses transport-age mailbox timing.  The trace header carries
% max_hold_ticks and the fault-latch budget from the commissioned configuration.
opts.branches=3; opts.log_core=true; out=sim_core_engine(P,opts);
report=replay_group(out,infile,outfile,exe);
report.status='pass';
assert(report.gate_exact && report.regime_exact && report.fault_exact && report.corrupted_exact && ...
    report.integrators_finite && ...
    max([report.max_du,report.max_dutarget,report.max_dblend])<1e-7, ...
    'verify_core_equivalence:mismatch', ...
    'C/MATLAB mismatch: gate=%d regime=%d fault=%d corrupted=%d du=%.3g dut=%.3g db=%.3g', ...
    report.gate_exact,report.regime_exact,report.fault_exact,report.corrupted_exact, ...
    report.max_du,report.max_dutarget,report.max_dblend);
fprintf('PASS: %d closed-loop ticks; gate/regime/fault/corrupted exact; max|du|=%.3g\n', ...
    report.n_ticks,report.max_du);
% Sequenced-lease replay: the B3 loop publishes a monotonic outer_seq and a
% zero transported age so the core ages the frame on its own clock, plus a
% short fallback-to-zero window that must be audited as corruption.
include_lease=isempty(fieldnames(caller_opts)) || ...
    (isfield(caller_opts,'include_lease') && logical(caller_opts.include_lease));
if include_lease
    lease_opts=struct('seq_mode',true,'duration',10,'branches',3,'log_core',true);
    lease_out=sim_core_engine(P,lease_opts);
    L=replay_group(lease_out,infile,outfile,exe);
    assert(L.gate_exact && L.regime_exact && L.fault_exact && L.corrupted_exact && ...
        L.integrators_finite && ...
        max([L.max_du,L.max_dutarget,L.max_dblend])<1e-7, ...
        'verify_core_equivalence:lease-mismatch', ...
        'Lease C/MATLAB mismatch: gate=%d regime=%d fault=%d corrupted=%d du=%.3g dut=%.3g db=%.3g', ...
        L.gate_exact,L.regime_exact,L.fault_exact,L.corrupted_exact, ...
        L.max_du,L.max_dutarget,L.max_dblend);
    assert(L.corrupted_ticks>0,'verify_core_equivalence:lease-coverage', ...
        'The sequence fallback window did not exercise the corruption path.');
    assert(all(ismember([1 5 6],L.gates)),'verify_core_equivalence:lease-gates', ...
        'The lease replay did not cover HOLD, SAFE_ZERO and the latched-safe gate: %s', ...
        mat2str(L.gates));
    report.lease_n_ticks=L.n_ticks;
    report.lease_max_du=L.max_du;
    report.lease_max_dutarget=L.max_dutarget;
    report.lease_max_dblend=L.max_dblend;
    report.lease_gate_exact=L.gate_exact;
    report.lease_regime_exact=L.regime_exact;
    report.lease_fault_exact=L.fault_exact;
    report.lease_corrupted_exact=L.corrupted_exact;
    report.lease_integrators_finite=L.integrators_finite;
    report.lease_corrupted_ticks=L.corrupted_ticks;
    report.lease_gates=L.gates;
    fprintf(['PASS: %d lease-replayed ticks; gate/regime/fault/corrupted exact; ' ...
        'max|du|=%.3g; audited corruption ticks=%d; gates=%s\n'], ...
        L.n_ticks,L.max_du,L.corrupted_ticks,mat2str(L.gates));
end
end

function M=replay_group(out,infile,outfile,exe)
% Write one trace, run the C replay and compare the seven logged outputs.
CI=out.b3.core_in; CO=out.b3.core_out; N=size(CI,1); cfg=out.config;
use_seq=size(CI,2)>9;
fields={'dt_s','outer_timeout_s','max_physics_residual','residual_clear_ratio', ...
 'min_ai_confidence','confidence_clear_ratio','state_safety_limit','safety_trip_ratio', ...
 'safety_clear_ratio','safety_predict_s','u_max','u_slew_max_per_s','blend_alpha', ...
 'integ_limit','windup_tau_s'};
fid=fopen(infile,'w'); fprintf(fid,'%d\n',N);
for j=1:numel(fields), fprintf(fid,'%.9g ',double(single(cfg.(fields{j})))); end
fprintf(fid,'%u %u ',uint32(cfg.min_dwell_ticks),uint32(cfg.ood_recovery_cooldown_ticks));
fprintf(fid,'%.9g ',double(single([cfg.kp;cfg.kd;cfg.ki;cfg.ff_scale])));
fprintf(fid,'%u %u %u\n',uint32(cfg.max_hold_ticks), ...
    uint32(cfg.max_fault_vetoes_before_latch),uint32(use_seq));
trace=[out.b3.core_tuning,CI];
vals=single(trace(:,1:12));
bits=reshape(typecast(vals(:),'uint32'),N,12);
if use_seq
    seqcol=uint32(CI(:,10));
    fprintf(fid,'%u %u %u %u %u %u %u %u %u %u %u %u %u\n',[bits,seqcol]');
else
    fprintf(fid,'%u %u %u %u %u %u %u %u %u %u %u %u\n',bits');
end
fclose(fid);
[status,msg]=system(sprintf('"%s" "%s" "%s"',exe,infile,outfile));
assert(status==0,'verify_core_equivalence:run','C replay failed: %s',msg);
fid=fopen(outfile,'r'); C=fscanf(fid,'%f',[7 Inf])'; fclose(fid);
assert(isequal(size(C),size(CO)),'C replay output shape differs.');
M=struct('n_ticks',N,'max_du',max(abs(C(:,1)-CO(:,1))), ...
    'max_dutarget',max(abs(C(:,2)-CO(:,2))), ...
    'max_dblend',max(abs(C(:,5)-CO(:,5))), ...
    'gate_exact',isequal(C(:,3),CO(:,3)), ...
    'regime_exact',isequal(C(:,4),CO(:,4)), ...
    'fault_exact',isequal(C(:,6),CO(:,6)), ...
    'corrupted_exact',isequal(C(:,7),CO(:,7)), ...
    'corrupted_ticks',sum(C(:,7)~=0), ...
    'gates',unique(CO(:,3))', ...
    'integrators_finite',out.b3.integrators_finite);
end
