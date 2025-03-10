function xdot=flight_motor_ode_con_v(t,x,IC,robot,simp, motor,func_param,mvar,V,type)
%Inputs: 
%       - t [time] for ODE45
%       - x states [x, x', y, y',phi]
%       - IC 1x5 vector of initial conditions
%       - Robot param, not actually used
%       - simp, simulation param structure
%       - Motor_parameter, motor_param structure, not used here
%       - func_param, function_param, not used here

%% Initial Setup
V=V(1);
Ra=mvar(1);
La=mvar(2);
kt=mvar(3);
kb=mvar(4);
J=mvar(5);
C=mvar(6);
R=mvar(7);

g=simp.g;

%% Rec Cords
xdot(1,:) =x(2,:); %x'
xdot(2,:) =0;      %x"

xdot(3,:) =x(4,:); %y'
xdot(4,:) =-g;%y"


%% Polar Cords and Current

xdot(5,:)=-x(6,:)/R; %phi
xdot(6,:)=kt/J*x(7,:)-C/J*x(6,:); %dphi [Omega]
xdot(7,:)=V'./La-Ra/La*x(7,:)-kb/La*x(6,:); %current
end