function [sens,IC]=gait_sens(robot_param, IC,sim_param, motor,IC_solved,voltage_solved)
% GAIT_SENS calculates the gait sensitivity norm of the system as defined
% by Hobbelen's paper "A Disturbance Rejection Measure for Limit Cycle
% Walkers: The Gait Snesitivity Norm"
%
% Note it is not recommended to use the non motor system with damping as
% the damping guarentees an unstable system without energy being added back
% in.  Runs all models for 4 cycles.
%
% Gait sensitivity metrics are changes in x and y velocities as y velocity
% changes correspond directly to step height and step clearance.  X
% velocity corresponds to step distance.
%
%
% Inputs:
%   Robot_Param class
%
%   IC, a vector of initial conditions in the following form
%       - [zeta0, dzeta0, phi0, dphi0] with the following units
%       - [[m], [m/s], [rad], [rad/sec]]
%
%   simulation_param class
%
% motor_parameter [optional] a class only needed if a motor is being used
% in the simulation to skip enter a 0.  If motor is used you must supply
% the correct models to use.  This is incase one wants to look at only
% motor during stance with a conservative flight or only during flight
%
% IC_solved: enter a 1 if IC conditions were found based on the
% conservative system that return a stable gait.  If [0] it will remove
% motor and any damping and solve for a set of IC's.  Uses the Td angle
% given by IC(3)
%
% Voltage solved:
%
% Returns:
%       - sens: sensitivities of the system in vector form []
%               entry 1: sensitivity to 5% pertabation in x vel
%               entry 2: sensitivity to 2% pertabation in y vel + step
%                       height of 1/20th the leg length.  Wanted more but becomes
%                       to unstable
%               entry 3: sensitivity to 3 deg td error
%               entry 4: sensitivity to 5% uncertainity in k
%               entry 5: sensitivity to 5% uncertainity in b
%               entry 6: [optional] sensitivity 5% uncertainty in voltage.
%                        a random pertabation is applied to each voltage
%                        step
%       - IC: entered IC if IC_solved = 1 or calculated IC if IC_solved=0

% Determine if motor was given or not
if ~exist('motor','var') || isa(motor, 'double')
    %Selects the correct stance,flight models
    sim_param_temp=sim_param;
    sim_param_temp.stancemodel=@stance_model;
    sim_param_temp.flightmodel=@flight_conservative_ode;
    motor=0;
    
else
    
    %Selects the correct stance,flight models
    sim_param_temp=sim_param;
    sim_param_temp.stancemodel=@stance_motor_ode_poly;
    sim_param_temp.flightmodel=@flight_motor_ode_con_v;
    
%     motor_temp=motor;
%     motor_temp.flightV(:,2)=motor_temp.flightV(:,2).*(1+.05*randi([-50, 50], 100,1)/50);
%     motor_temp.stanceV(:,2)=motor_temp.stanceV(:,2).*(1+.05*randi([-50, 50], 100,1)/50);
    
end


%solves for IC's if they have not yet been solved for
if IC_solved ~= 1
    
    %uses conservative model
    sim_param_temp1=sim_param;
    sim_param_temp1.stancemodel=@stance_model;
    sim_param_temp1.flightmodel=@flight_conservative_ode;
    
    robot=robot_param;
    robot.bl=0;
    
    lb=[-10 ,0.0001];
    ub=[-0.001, 100];
    
    %opt values are [dzeta0, dphi0]
    [x]=fmincon(@SLIPopt,[-0.4, 1.5],[],[],[],[],lb,ub,[],[], ...
        robot,sim_param_temp1,IC(3),3);
    IC=[robot_param.l0, x(1), IC(3), x(2)];
    
    
end

%Create all the distubances

%X,Y velocity
[x,dx,y,dy]=p2r(IC);

yvel=dy;
xvel=dx*1.05;
dzeta0 = (x*xvel+y*yvel)/sqrt(x^2+y^2);
dphi0  = -(x*yvel-y*xvel)/(x^2+y^2);

dely=sqrt(2*sim_param.g*robot_param.l0/20);
yvel=dy*1.02+dely; %add 2% uncertainity due to sensor noise then due to sttep
xvel=dx;
dzeta01 = (x*xvel+y*yvel)/sqrt(x^2+y^2);
dphi01  = -(x*yvel-y*xvel)/(x^2+y^2);

IC1=[robot_param.l0, dzeta0, IC(3), dphi0];
IC2=[robot_param.l0, dzeta01, IC(3), dphi01];

%Variability in K
robot1=robot_param;
robot1.k0=robot1.k0*1.05;

%Variability in damping
robot2=robot_param;
robot2.bl=robot2.bl*1.05;

% td uncert
td_pert.err=3;

%Create baseline
data=Stance_sim_Ode(robot_param,IC,sim_param_temp,4,motor);
% data1=Stance_sim_Ode(robot_param,IC1,sim_param_temp,4,motor); %xvel disturbance
data2=Stance_sim_Ode(robot_param,IC2,sim_param_temp,4,motor); %yvel disturbance
data3=Stance_sim_Ode(robot_param,IC,sim_param_temp,4,motor,@td_pertabation,td_pert); %td uncertainity
% data4=Stance_sim_Ode(robot1,IC,sim_param_temp,4,motor); %k uncertainity
% data5=Stance_sim_Ode(robot2,IC,sim_param_temp,4,motor); %damping uncertainity

% sens1=calc_gait(data.stanceIC,data1.stanceIC,0.05); %xvel disturbance
sens2=calc_gait(data.stanceIC,data2.stanceIC,abs((yvel-dy)/dy)); %yvel disturbance
sens3=calc_gait(data.stanceIC,data3.stanceIC,deg2rad(td_pert.err)/IC(3)); %td uncertainity
% sens4=calc_gait(data.stanceIC,data4.stanceIC,0.05); %k uncertainity
% sens5=calc_gait(data.stanceIC,data5.stanceIC,0.05); %damping uncertainity

% sens=[sens1 sens2 sens3 sens4 sens5];
sens=[0 sens2 sens3 0 0];

% graphing(data, sim_param,3,10,[data.rec_states(:,1), data.rec_states(:,4)],  ["r", "b--", "k"])
% graphing(data1,sim_param,3,11,[data1.rec_states(:,1),data1.rec_states(:,4)], ["r", "b--", "k"])
% graphing(data2,sim_param,3,12,[data2.rec_states(:,1),data2.rec_states(:,4)], ["r", "b--", "k"])
% graphing(data3,sim_param,3,13,[data3.rec_states(:,1),data3.rec_states(:,4)], ["r", "b--", "k"])
% graphing(data4,sim_param,3,14,[data4.rec_states(:,1),data4.rec_states(:,4)], ["r", "b--", "k"])
% graphing(data5,sim_param,3,15,[data5.rec_states(:,1),data5.rec_states(:,4)], ["r", "b--", "k"])
% 

% if motor~=0
%     data6=Stance_sim_Ode(robot2,IC,sim_param_temp,4,motor); %damping uncertainity
%     sens6=calc_gait(data.stanceIC,data6.stanceIC,0.05); %damping uncertainity
%     sens=[sens, sens6];
% end



end


function err=calc_gait(un_IC, dis_IC, errmag)
%un_IC and dis_IC should be stance_IC values of undesturbed and disturbed,
%errmag is the magnitude of the error

a=min([length(un_IC(:,1)), length(dis_IC(:,1))]);

g=0;
for i=2:a
    
    [~,dx,~,dy]=p2r(un_IC(i,:));
    [~,dx1,~,dy1]=p2r(dis_IC(i,:));
    
    g=g+(dx1-dx)^2+(dy1-dy)^2;
    
end

err=1/errmag*sqrt(g);

end
