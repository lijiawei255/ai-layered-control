function [mdl, status] = build_simulink_model(out_path, opts)
%BUILD_SIMULINK_MODEL  Programmatically build ai_layered_control_bench.slx
%
%   MDL = BUILD_SIMULINK_MODEL() writes the model next to this file and
%   returns its name.  MDL = BUILD_SIMULINK_MODEL(OUT_PATH) writes it there.
%   [MDL,STATUS] = ... also returns 'built', 'skipped_dirty' or 'unavailable'.
%
%   A loaded model with unsaved edits is NEVER closed or overwritten unless
%   opts.force is true.  Without Simulink the numeric benchmark remains fully
%   usable; the caller can skip the model step.
%
%   The model is generated with BUILT-IN Simulink blocks only (no MATLAB
%   Function blocks, no Stateflow, no toolbox blocks, no MEX compiler), so it
%   opens and runs on any Simulink installation:
%
%     three parallel branches on the same plant and reference:
%       Branch 1 - fixed PID (nominal baseline)
%       Branch 2 - continuous-policy sketch with a 100 ms transport delay and
%                  a 10 Hz zero-order hold emulating edge inference latency
%                  (the trained RBF baseline is quantitative code in
%                  sim_core_engine.m, not a block in this visualization)
%       Branch 3 - layered architecture: 1 kHz PID with a hard safety
%                  saturation (the deterministic core)
%
%   The quantitative benchmark lives in sim_core_engine.m; this model is the
%   inspectable, non-binary artefact of the same architecture and the .slx
%   counterpart of the plain-text reference implementation.
%
%   See also SIM_CORE_ENGINE, RUN_ALL_BENCHMARKS.

if nargin < 1 || isempty(out_path)
    here = fileparts(mfilename('fullpath'));
    out_path = fullfile(here, 'ai_layered_control_bench.slx');
end
if nargin < 2 || isempty(opts)
    opts = struct();
end
force = isfield(opts, 'force') && opts.force;

model = 'ai_layered_control_bench';
if ~license('test', 'Simulink') || exist('sim', 'file') == 0
    mdl = model;
    status = 'unavailable';
    return;
end
if bdIsLoaded(model)
    dirty = strcmp(get_param(model, 'Dirty'), 'on');
    if dirty && ~force
        mdl = model;
        status = 'skipped_dirty';
        return;
    end
    close_system(model, 0);
end
if exist(out_path, 'file')
    delete(out_path);
end

new_system(model);
set_param(model, 'Solver', 'ode1', 'FixedStep', '1e-3', 'StopTime', '20');

P = load_domain_profile('industrial');
J = P.plant.J; b = P.plant.b; k = P.plant.k;
fc = P.plant.fc; fs = P.plant.fs; vs = P.plant.vs; fv = P.plant.fvisc;
Kp = P.ctrl.Kp; Kd = P.ctrl.Kd; Ki = P.ctrl.Ki;
u_max = P.act.u_max;

add_block('simulink/Sources/Sine Wave', [model '/Reference'], ...
    'Amplitude', num2str(P.ref.amp), ...
    'Frequency', num2str(2*pi*P.ref.f1_hz), ...
    'Position', [30 180 70 220]);

names = {'FixedPID', 'EndToEndAI', 'Layered'};
y0 = [80 280 480];
for br = 1:3
    yb = y0(br);
    tag = names{br};

    add_block('simulink/Math Operations/Sum', [model '/' tag '_err'], ...
        'Inputs', '+-', 'Position', [130 yb 150 yb+20]);
    add_block('simulink/Math Operations/Gain', [model '/' tag '_Kp'], ...
        'Gain', num2str(Kp), 'Position', [180 yb-10 210 yb+10]);
    add_block('simulink/Continuous/Derivative', [model '/' tag '_d'], ...
        'Position', [140 yb+40 165 yb+60]);
    add_block('simulink/Math Operations/Gain', [model '/' tag '_Kd'], ...
        'Gain', num2str(Kd), 'Position', [180 yb+40 210 yb+60]);
    add_block('simulink/Continuous/Integrator', [model '/' tag '_i'], ...
        'Position', [140 yb+75 165 yb+95]);
    add_block('simulink/Math Operations/Gain', [model '/' tag '_Ki'], ...
        'Gain', num2str(Ki), 'Position', [180 yb+75 210 yb+95]);
    add_block('simulink/Math Operations/Sum', [model '/' tag '_sum'], ...
        'Inputs', '+++', 'Position', [245 yb+20 265 yb+60]);
    add_block('simulink/Discontinuities/Saturation', [model '/' tag '_sat'], ...
        'UpperLimit', num2str(u_max), 'LowerLimit', num2str(-u_max), ...
        'Position', [400 yb+20 430 yb+60]);

    plant = sprintf('Plant%d', br);
    add_block('built-in/Subsystem', [model '/' plant], ...
        'Position', [490 yb 580 yb+70]);
    build_plant([model '/' plant], J, b, k, fc, fs, vs, fv);

    % ---- wiring ---------------------------------------------------------
    add_line(model, 'Reference/1', [tag '_err/1'], 'autorouting', 'on');
    add_line(model, [tag '_err/1'], [tag '_Kp/1'], 'autorouting', 'on');
    add_line(model, [tag '_err/1'], [tag '_d/1'], 'autorouting', 'on');
    add_line(model, [tag '_err/1'], [tag '_i/1'], 'autorouting', 'on');
    add_line(model, [tag '_d/1'], [tag '_Kd/1'], 'autorouting', 'on');
    add_line(model, [tag '_i/1'], [tag '_Ki/1'], 'autorouting', 'on');
    add_line(model, [tag '_Kp/1'], [tag '_sum/1'], 'autorouting', 'on');
    add_line(model, [tag '_Kd/1'], [tag '_sum/2'], 'autorouting', 'on');
    add_line(model, [tag '_Ki/1'], [tag '_sum/3'], 'autorouting', 'on');
    if br == 2
        add_block('simulink/Continuous/Transport Delay', [model '/' tag '_delay'], ...
            'DelayTime', '0.10', 'Position', [285 yb+20 320 yb+60]);
        add_block('simulink/Discrete/Zero-Order Hold', [model '/' tag '_zoh'], ...
            'SampleTime', '0.1', 'Position', [340 yb+20 375 yb+60]);
        add_line(model, [tag '_sum/1'], [tag '_delay/1'], 'autorouting', 'on');
        add_line(model, [tag '_delay/1'], [tag '_zoh/1'], 'autorouting', 'on');
        add_line(model, [tag '_zoh/1'], [tag '_sat/1'], 'autorouting', 'on');
    else
        add_line(model, [tag '_sum/1'], [tag '_sat/1'], 'autorouting', 'on');
    end
    add_line(model, [tag '_sat/1'], [plant '/1'], 'autorouting', 'on');
    add_line(model, [plant '/1'], [tag '_err/2'], 'autorouting', 'on');
end

% ------------------------------------------------------------------------
% Comparison scopes
% ------------------------------------------------------------------------
add_block('simulink/Signal Routing/Mux', [model '/Mux_x'], 'Inputs', '3', ...
    'Position', [700 260 705 420]);
add_block('simulink/Sinks/Scope', [model '/Scope_states'], ...
    'Position', [740 320 780 380]);
add_block('simulink/Signal Routing/Mux', [model '/Mux_u'], 'Inputs', '3', ...
    'Position', [700 500 705 620]);
add_block('simulink/Sinks/Scope', [model '/Scope_commands'], ...
    'Position', [740 530 780 590]);
for br = 1:3
    tag = names{br};
    add_line(model, sprintf('Plant%d/1', br), ['Mux_x/' num2str(br)], 'autorouting', 'on');
    add_line(model, [tag '_sat/1'], ['Mux_u/' num2str(br)], 'autorouting', 'on');
end
add_line(model, 'Mux_x/1', 'Scope_states/1', 'autorouting', 'on');
add_line(model, 'Mux_u/1', 'Scope_commands/1', 'autorouting', 'on');

% ------------------------------------------------------------------------
% Stamp the generated model with the project name instead of the local account name
% ------------------------------------------------------------------------
try
    set_param(model, 'Creator', 'ai-layered-control', ...
                     'LastModifiedBy', 'ai-layered-control', ...
                     'ModifiedByFormat', 'ai-layered-control');
catch
    try
        set_param(model, 'Creator', 'ai-layered-control', ...
                         'LastModifiedBy', 'ai-layered-control');
    catch
        % keep the installation default rather than failing the build
    end
end

save_system(model, out_path);
if bdIsLoaded(model)
    close_system(model, 0);
end

% Simulink records the saving account in the OPC package core properties
% (dc:creator / cp:lastModifiedBy) regardless of set_param.  Rewrite those two
% fields inside the .slx (it is a plain zip container) so the committed model
% carries no local personal data.
local_sanitize_slx(out_path);

mdl = model;
status = 'built';
end

% =========================================================================
%  Strip local user names out of the .slx OPC core properties
% =========================================================================
function local_sanitize_slx(path)
tmp = tempname;
mkdir(tmp);
cleaner = onCleanup(@() local_rmdir(tmp));
unzip(path, tmp);
cp = fullfile(tmp, 'metadata', 'coreProperties.xml');
if exist(cp, 'file')
    txt = fileread(cp);
    txt = regexprep(txt, '<dc:creator>[^<]*</dc:creator>', ...
                         '<dc:creator>ai-layered-control</dc:creator>');
    txt = regexprep(txt, '<cp:lastModifiedBy>[^<]*</cp:lastModifiedBy>', ...
                         '<cp:lastModifiedBy>ai-layered-control</cp:lastModifiedBy>');
    fid = fopen(cp, 'w');
    fwrite(fid, txt);
    fclose(fid);
end

% Rebuild the package into a fresh file first, then replace the original, so a
% failed re-zip can never destroy the model.
d = dir(fullfile(tmp, '**', '*'));
keep = ~[d.isdir];
d = d(keep);
rel = cell(1, numel(d));
prefix = [tmp filesep];
for i = 1:numel(d)
    full = fullfile(d(i).folder, d(i).name);
    rel{i} = strrep(full, prefix, '');
end
newzip = [tempname '.zip'];
zip(newzip, rel, tmp);
movefile(newzip, path, 'f');
end

function local_rmdir(p)
if exist(p, 'dir')
    rmdir(p, 's');
end
end

% =========================================================================
%  Plant subsystem: J*xdd + b*xd + k*x + friction(xd) = u
% =========================================================================
function build_plant(path, J, b, k, fc, fs, vs, fv)
veps = max(vs*0.02, 1e-9);
expr = ['(' num2str(fv) ')*u + (' num2str(fc) ')*tanh(u/' num2str(veps) ') ' ...
        '+ (' num2str(fs) '-' num2str(fc) ')*exp(-abs(u)/' num2str(vs) ...
        ')*tanh(u/' num2str(veps) ')'];

add_block('simulink/Sources/In1', [path '/u'], 'Position', [20 100 50 120]);
add_block('simulink/Math Operations/Sum', [path '/sum'], 'Inputs', '+-+-', ...
    'Position', [100 90 120 130]);
add_block('simulink/Math Operations/Gain', [path '/invJ'], ...
    'Gain', num2str(1/J), 'Position', [150 100 180 120]);
add_block('simulink/Continuous/Integrator', [path '/xd'], ...
    'Position', [210 100 240 120]);
add_block('simulink/Continuous/Integrator', [path '/x'], ...
    'Position', [280 100 310 120]);
add_block('simulink/Sinks/Out1', [path '/x_out'], 'Position', [350 100 380 120]);
add_block('simulink/Math Operations/Gain', [path '/gain_b'], ...
    'Gain', num2str(b), 'Position', [180 150 210 170]);
add_block('simulink/Math Operations/Gain', [path '/gain_k'], ...
    'Gain', num2str(k), 'Position', [230 195 260 215]);
add_block('simulink/User-Defined Functions/Fcn', [path '/friction'], ...
    'Expr', expr, 'Position', [150 250 210 290]);

add_line(path, 'u/1', 'sum/1', 'autorouting', 'on');
add_line(path, 'sum/1', 'invJ/1', 'autorouting', 'on');
add_line(path, 'invJ/1', 'xd/1', 'autorouting', 'on');
add_line(path, 'xd/1', 'x/1', 'autorouting', 'on');
add_line(path, 'x/1', 'x_out/1', 'autorouting', 'on');
add_line(path, 'xd/1', 'gain_b/1', 'autorouting', 'on');
add_line(path, 'gain_b/1', 'sum/2', 'autorouting', 'on');
add_line(path, 'x/1', 'gain_k/1', 'autorouting', 'on');
add_line(path, 'gain_k/1', 'sum/3', 'autorouting', 'on');
add_line(path, 'xd/1', 'friction/1', 'autorouting', 'on');
add_line(path, 'friction/1', 'sum/4', 'autorouting', 'on');
end
