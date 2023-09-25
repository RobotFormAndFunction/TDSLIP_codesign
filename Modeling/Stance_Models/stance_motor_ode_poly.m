function xdot=stance_motor_ode_poly(t,x,IC,robot,simp, motor,func_param,mvar,V,type)
% Inputs:
%
%   IC, a vector of initial conditions in the following form
%       - [zeta0, dzeta0, phi0, dphi0, curr0] with the following units
%       - [[m], [m/s], [rad], [rad/sec]]
%
%   Robot_Param class
%
%   simulation_param class
%
% motor_parameter
%
% func_param anything that needs to be passed into func, can be any data
%       type func needs to be set up to interpret
%
% mvar, motor variables in a vector form, speeds up computation [Ra, La,
% kt, kb, J, C, R];
%
% V voltage vector of polynomial powers that describe the voltage profile
%
% type, voltage interpolation type pulled from motor_param
%
%  Outputs are [r, r', theta, theta', current]



%V=interpn(V(:,1),V(:,2),t,type);
volts=zeros(length(t),1);
for i=1:length(t)
    for j=1:length(V)
        volts(i)=volts(i)+V(j)*t(i)^(j-1);
    end
end

Ra=mvar(1);
La=mvar(2);
kt=mvar(3);
kb=mvar(4);
J=mvar(5);
C=mvar(6);
R=mvar(7);

[m, l0, k0, bl, ~, ~]=robot.unpack;
g=simp.g;

x1 = x(1,:); %r
x2 = x(2,:); %r'
x3 = x(3,:); %theta
x4 = x(4,:); %theta'
x5 = x(5,:); %current

xdot(1,:) =x2;
xdot(2,:) =x1.*x4.^2-g*sin(x3)-k0/m*(x1-l0)- bl/m*x2;
xdot(3,:) =x4;
xdot(4,:) =(1+(R^2*J)./(m*x1.^2)).^(-1).*(-(2*x2.*x4)./x1 ...
    -g*cos(x3)./x1-(R^2*C*x4)./(m*x1.^2)+(kt*x5*R)./(m*x1.^2));
xdot(5,:) =volts'./La-Ra*x5./La-R*kb*x4./La;
end