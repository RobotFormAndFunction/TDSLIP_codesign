function xdot=stance_model(t,x,IC,robot,simp, motor_parameter,func_param,mvar,V,type)
[m, l0, k0, bl, ~, ~]=robot.unpack;
g=simp.g;

x1 = x(1,:); %r
x2 = x(2,:); %r'
x3 = x(3,:); %theta
x4 = x(4,:); %theta'

xdot(1,:) =x2;
xdot(2,:) =x1.*x4.^2-g*sin(x3)-k0/m*(x1-l0)- bl/m*x2;

xdot(3,:) =x4;
xdot(4,:) =-(2*x2.*x4)./x1-g./x1.*cos(x3);
end