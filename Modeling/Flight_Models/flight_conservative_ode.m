function xdot=flight_conservative_ode(t,x,IC,robot,simp, motor_parameter,func_param,mvar,V,type)
%Basic flight model for ODE45
%Inputs: 
%       - t [time] for ODE45
%       - x states [x, x', y, y',phi]
%       - IC 1x5 vector of initial conditions
%       - Robot param, not actually used
%       - simp, simulation param structure
%       - Motor_parameter, motor_param structure, not used here
%       - func_param, function_param, not used here


xdot(1,:) =x(2,:); %x'
xdot(2,:) =0;      %x"

xdot(3,:) =x(4,:); %y'
xdot(4,:) =-simp.g;%y"

xdot(5,:)=0;
end