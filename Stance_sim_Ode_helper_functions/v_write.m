function [data, func_param]=v_write(n,data,simoutput,robot, IC, sim_param,motor_parameter,state, func_param)
% write the voltage and time vectors to the workspace for both stance and
% flight

%% Write td variable to phi0


if state==0 ||state== 1
    assignin('caller','t_con',motor_parameter.stanceV(:,1))
    assignin('caller','v_con',motor_parameter.stanceV(:,2))
end

if state== 2
    assignin('caller','t_con',motor_parameter.flightV(:,1))
    assignin('caller','v_con',motor_parameter.flightV(:,2))
end
