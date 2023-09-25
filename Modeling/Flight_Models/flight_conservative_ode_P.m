function xdot=flight_conservative_ode_P(t,x,IC,robot,simp, motor_parameter,func_param,mvar,V,type)
%Basic flight model for ODE45 in polar form
%Inputs: 
%       - t [time] for ODE45
%       - x states [x, x', y, y',phi]
%       - IC 1x5 vector of initial conditions
%       - Robot param, not actually used
%       - simp, simulation param structure
%       - Motor_parameter, motor_param structure, not used here
%       - func_param, function_param, not used here


xdot(1,:) =x(2,:); %r'
xdot(2,:) =x(1,:).*x(4,:).^2-simp.g.*sin(x(3,:)); %r"

xdot(3,:) =x(4,:); %theta'
xdot(4,:) =(-2*x(2,:).*x(4,:))./x(1,:)-(simp.g.*cos(x(3,:)))./x(1,:);%theta"

xdot(5,:)=0;
end