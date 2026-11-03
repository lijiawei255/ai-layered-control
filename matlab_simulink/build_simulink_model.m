% SPDX-License-Identifier: MIT
% Copyright (c) 2026 Jiawei Li
% Source: https://github.com/lijiawei255/ai-layered-control
% License: https://github.com/lijiawei255/ai-layered-control/blob/main/LICENSE
% Citation (optional): https://github.com/lijiawei255/ai-layered-control/blob/main/CITATION.cff
function [mdl, status] = build_simulink_model(out_path)
%BUILD_SIMULINK_MODEL Built-in-block illustration with explicit rate boundaries.
% The numeric engine owns the trained RBF, Karnopp plant and nine-gate core.
if nargin < 1
    out_path=fullfile(fileparts(mfilename('fullpath')),'ai_layered_control_bench.slx');
end
mdl='ai_layered_control_bench';
if ~license('test','Simulink'), status='unavailable'; return; end
if bdIsLoaded(mdl)
    assert(strcmp(get_param(mdl,'Dirty'),'off'),'Save open model edits before rebuilding.');
    close_system(mdl,0);
end
new_system(mdl);
set_param(mdl,'Solver','ode1','FixedStep','0.001','StopTime','40', ...
    'Creator','ai-layered-control');
note=Simulink.Annotation(mdl,'Rate/interface sketch. Quantitative RBF policy and C99 gate semantics: sim_core_engine.m');
note.Position=[25 5];
refs={'Reference','ReferenceRate','ReferenceAccel'};
amplitudes={'0.5','0.5*pi','-0.5*pi*pi'}; phases={'0','pi/2','0'};
for j=1:3
    add_block('simulink/Sources/Sine Wave',[mdl '/' refs{j}], ...
        'Amplitude',amplitudes{j},'Frequency','pi','Phase',phases{j}, ...
        'SampleTime','0.001','Position',[25 25+55*j 65 55+55*j]);
end
add_block('simulink/Signal Routing/Mux',[mdl '/ReferenceVector'],'Inputs','3','Position',[95 80 100 220]);
for j=1:3, add_line(mdl,[refs{j} '/1'],['ReferenceVector/' num2str(j)]); end
add_block('simulink/User-Defined Functions/Fcn',[mdl '/NominalFF'], ...
    'Expr','0.3*u(1)+0.06*u(2)+0.01*u(3)','Position',[130 105 240 140]);
add_line(mdl,'ReferenceVector/1','NominalFF/1');
tags={'B1_FF_PID','B2_ConditionalAction','B3_DeterministicBank'};
for br=1:3
    y=270+(br-1)*240; tag=tags{br};
    add_block('simulink/Math Operations/Sum',[mdl '/' tag '_Error'],'Inputs','+-','Position',[100 y 120 y+20]);
    add_block('simulink/Continuous/Derivative',[mdl '/' tag '_Rate'],'Position',[100 y+40 130 y+65]);
    add_block('simulink/Math Operations/Sum',[mdl '/' tag '_RateError'],'Inputs','+-','Position',[160 y+40 180 y+65]);
    add_block('simulink/Continuous/Transfer Fcn',[mdl '/' tag '_Plant'], ...
        'Numerator','[1]','Denominator','[0.01 0.06 0.3]','Position',[820 y 915 y+45]);
    add_line(mdl,'Reference/1',[tag '_Error/1']);
    add_line(mdl,[tag '_Plant/1'],[tag '_Error/2']);
    add_line(mdl,[tag '_Plant/1'],[tag '_Rate/1']);
    add_line(mdl,'ReferenceRate/1',[tag '_RateError/1']);
    add_line(mdl,[tag '_Rate/1'],[tag '_RateError/2']);
    if br==1
        add_block('simulink/Continuous/Integrator',[mdl '/' tag '_Integral'],'Position',[190 y+95 220 y+120]);
        add_block('simulink/Signal Routing/Mux',[mdl '/' tag '_Inputs'],'Inputs','4','Position',[280 y 285 y+130]);
        signals={[tag '_Error'],[tag '_RateError'],[tag '_Integral'],'NominalFF'};
        add_line(mdl,[tag '_Error/1'],[tag '_Integral/1']);
        expr='110*u(1)+1.5*u(2)+300*u(3)+u(4)';
    else
        add_block('simulink/User-Defined Functions/Fcn',[mdl '/' tag '_Features'], ...
            'Expr','abs(u)/(abs(u)+0.05)','Position',[190 y+105 285 y+130]);
        add_block('simulink/Discrete/Zero-Order Hold',[mdl '/' tag '_Context10Hz'], ...
            'SampleTime','0.1','Position',[310 y+100 380 y+135]);
        add_block('simulink/Continuous/Transport Delay',[mdl '/' tag '_ContextDelay'], ...
            'DelayTime','0.1','Position',[410 y+100 480 y+135]);
        add_line(mdl,[tag '_Error/1'],[tag '_Features/1']);
        add_line(mdl,[tag '_Features/1'],[tag '_Context10Hz/1']);
        add_line(mdl,[tag '_Context10Hz/1'],[tag '_ContextDelay/1']);
        add_block('simulink/Signal Routing/Mux',[mdl '/' tag '_Inputs'],'Inputs','5','Position',[510 y 515 y+130]);
        signals={[tag '_Error'],[tag '_RateError'],'NominalFF',[tag '_ContextDelay'],[tag '_Plant']};
        if br==2
            expr='(60-52*u(4))*u(1)+(1.2-0.4*u(4))*u(2)+u(3)';
        else
            set_param([mdl '/' tag '_Inputs'],'Inputs','6'); signals{6}=[tag '_Rate'];
            expr='(abs(u(5))<0.9)*((60+50*(u(4)>0.5))*u(1)+1.2*u(2)+u(3))-(abs(u(5))>=0.9)*(2*u(5)+0.35*u(6))';
        end
    end
    for j=1:numel(signals), add_line(mdl,[signals{j} '/1'],[tag '_Inputs/' num2str(j)]); end
    add_block('simulink/User-Defined Functions/Fcn',[mdl '/' tag '_Control1kHz'], ...
        'Expr',expr,'Position',[550 y 675 y+45]);
    add_line(mdl,[tag '_Inputs/1'],[tag '_Control1kHz/1']);
    source=[tag '_Control1kHz'];
    if br==3
        add_block('simulink/Discrete/Discrete Transfer Fcn',[mdl '/' tag '_Blend'], ...
            'Numerator','[0.3]','Denominator','[1 -0.7]','SampleTime','0.001','Position',[600 y+65 665 y+90]);
        add_line(mdl,[source '/1'],[tag '_Blend/1']); source=[tag '_Blend'];
    end
    add_block('simulink/Discontinuities/Saturation',[mdl '/' tag '_Magnitude'], ...
        'UpperLimit','20','LowerLimit','-20','Position',[690 y 735 y+45]);
    add_block('simulink/Discontinuities/Rate Limiter',[mdl '/' tag '_HardwareSlew'], ...
        'RisingSlewLimit','2000','FallingSlewLimit','-2000','Position',[755 y 800 y+45]);
    add_line(mdl,[source '/1'],[tag '_Magnitude/1']);
    add_line(mdl,[tag '_Magnitude/1'],[tag '_HardwareSlew/1']);
    add_line(mdl,[tag '_HardwareSlew/1'],[tag '_Plant/1']);
    add_block('simulink/Sinks/Scope',[mdl '/' tag '_State'],'Position',[960 y 1005 y+45]);
    add_line(mdl,[tag '_Plant/1'],[tag '_State/1']);
end
set_param(mdl,'SimulationCommand','update');
save_system(mdl,out_path);
close_system(mdl,0);
status='built';
end
