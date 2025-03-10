function [data, func_param]=con_td_write(n,data,simoutput,robot, IC, sim_param,motor_parameter,state, func_param)
% Allows you to change the td point while using the flight conservative
% model, right now still keeps td the same but can be independent from
% stance ic, could be changed in the future

%% Write td variable to phi0


if state==2
    tempIC=evalin('caller','fIC');
    tempIC(5)=-func_param.td;
    assignin('caller','fIC',tempIC)
end
