function report = verify_core_equivalence(P, opts)
%VERIFY_CORE_EQUIVALENCE Replay 40,000 identical input frames through C99.
if nargin<1, P=load_domain_profile(); end
if nargin<2, opts=struct(); end
if isempty(P.classifier), P.classifier=calibrate_coarse_classifier(P); end
here=fileparts(mfilename('fullpath'));
core=fullfile(here,'..','c99_layered_core','layered_gatekeeper.c');
harness=fullfile(here,'..','c99_layered_core','verify_core_equivalence.c');
compiler=''; candidates={'gcc','cc','clang'};
configured=getenv('CC'); if ~isempty(configured), candidates=[{configured},candidates]; end
for j=1:numel(candidates)
    [status,~]=system(sprintf('"%s" --version',candidates{j}));
    if status==0, compiler=candidates{j}; break; end
end
if isempty(compiler)
    report=struct('status','skipped'); fprintf('SKIP: no C99 compiler found.\n'); return;
end
opts.branches=3; opts.log_core=true; out=sim_core_engine(P,opts);
CI=out.b3.core_in; CO=out.b3.core_out; N=size(CI,1); cfg=out.config;
tmp=tempname; mkdir(tmp); cleaner=onCleanup(@() rmdir(tmp,'s'));
infile=fullfile(tmp,'trace.txt'); outfile=fullfile(tmp,'output.txt');
exe=fullfile(tmp,'replay'); if ispc, exe=[exe,'.exe']; end
fid=fopen(infile,'w'); fprintf(fid,'%d\n',N);
fields={'dt_s','outer_timeout_s','max_physics_residual','residual_clear_ratio', ...
 'min_ai_confidence','confidence_clear_ratio','state_safety_limit','safety_trip_ratio', ...
 'safety_clear_ratio','safety_predict_s','u_max','u_slew_max_per_s','blend_alpha', ...
 'integ_limit','windup_tau_s','min_dwell_ticks','ood_recovery_cooldown_ticks'};
for j=1:numel(fields), fprintf(fid,'%.9g ',double(single(cfg.(fields{j})))); end
fprintf(fid,'%.9g ',double(single([cfg.kp;cfg.kd;cfg.ki;cfg.ff_scale])));
trace=[out.b3.core_tuning,CI];
bits=reshape(typecast(single(trace(:)),'uint32'),size(trace));
fprintf(fid,'\n');
fprintf(fid,'%u %u %u %u %u %u %u %u %u %u %u %u\n',bits'); fclose(fid);
[status,msg]=system(sprintf('"%s" -std=c99 -O2 -o "%s" "%s" "%s"',compiler,exe,core,harness));
assert(status==0,'verify_core_equivalence:compile','C replay compilation failed: %s',msg);
[status,msg]=system(sprintf('"%s" "%s" "%s"',exe,infile,outfile));
assert(status==0,'verify_core_equivalence:run','C replay failed: %s',msg);
fid=fopen(outfile,'r'); C=fscanf(fid,'%f',[7 Inf])'; fclose(fid);
assert(isequal(size(C),size(CO)),'C replay output shape differs.');
report=struct('status','pass','n_ticks',N, ...
    'max_du',max(abs(C(:,1)-CO(:,1))),'max_dutarget',max(abs(C(:,2)-CO(:,2))), ...
    'max_dblend',max(abs(C(:,5)-CO(:,5))),'gate_exact',isequal(C(:,3),CO(:,3)), ...
    'regime_exact',isequal(C(:,4),CO(:,4)),'fault_exact',isequal(C(:,6),CO(:,6)), ...
    'corrupted_exact',isequal(C(:,7),CO(:,7)), ...
    'integrators_finite',out.b3.integrators_finite);
assert(report.gate_exact && report.regime_exact && report.fault_exact && report.corrupted_exact && ...
    report.integrators_finite && ...
    max([report.max_du,report.max_dutarget,report.max_dblend])<1e-4, ...
    'verify_core_equivalence:mismatch', ...
    'C/MATLAB mismatch: gate=%d regime=%d fault=%d corrupted=%d du=%.3g dut=%.3g db=%.3g', ...
    report.gate_exact,report.regime_exact,report.fault_exact,report.corrupted_exact, ...
    report.max_du,report.max_dutarget,report.max_dblend);
fprintf('PASS: %d closed-loop ticks; gate/regime/fault/corrupted exact; max|du|=%.3g\n',N,report.max_du);
end
