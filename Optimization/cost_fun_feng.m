function [fval, data, flightV]=cost_fun_feng(robot_param, IC,sim_param, motor_parameter)
% COST_FUN_FENG is the cost function feng is using.  
%
% For a system without a motor assumes no energy loss during flight and 
% simply runs a single cycle of stance and reports difference in total energy.
% A change in energy will only occur if leg damping is non zero.  While
% this may not be a perfect representation of how much energy is needed it
% should be a close enough estimate.
%
% If a motor is present it runs a single cycle of stance and finds the
% change in total energy.  Then for the flight phase it uses the initial y
% velocity calculates the time to touchdown based on that initial y vel.
% Assuming a symetric stance cycle the voltage can be calulated thats
% needed to complete that delta theta in the given time.  The voltage can
% then be used to find the current from which power can be found.
%
% Energy is in Joules
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
% motor_parameter a class only needed if a motor is being used in the simulation
%
% Returns:
%       - data structure as specified by Stance_sim_Ode
%       - data.polar_states       vector of polar states [phi, phi', phi", z, z', z"]
%                                   during flight all states are returned
%                                   as 0


if exist('motor_parameter','var') & ~isa(motor_parameter, 'double')


    sim_param.stancemodel=@stance_motor_ode_poly;
    sim_param.flightmodel=@flight_motor_ode_con_v;
%     data=Stance_sim_Ode(robot_param, IC,sim_param, 1,motor_parameter);
% 
%     dx_rec = data.rec_states(end, 2);
%     dy_rec = data.rec_states(end, 5);
% 
%     if dx_rec>0 && dy_rec<0
%         constraint_flag = -1;
%     else
%         constraint_flag = 1;
%     end
% 
% %     col.N=11; % #of points you want to divide collocation into less is faster but wouldn't go lower than 9 or 11
% % col.mmax=18;
% % col.FC=[IC(1) -IC(2) pi-IC(3) IC(4)];
% % col.FC=[IC(1) 0.7 pi/2 3];
% % col.FC=[IC(1) 0 pi/2 0];
% % col.FC_type=2;
% % col.tf=0.12;
% % col.flightV_calc=1; %tells it to calculate the flight voltage
% % data=Stance_sim_Ode(robot_param, IC, sim_param,1,motor_parameter,@SLIP_col_Max_Y_Ode,col);
% 
%     %Find energy from flight
%     %[~,~,~,dy]=p2r(IC);
%     %period=2*dy/sim_param.g;
%     %deltaphi=2*pi-2*IC(3);
%     %v=motor_parameter.v_for_rot_in_time_T(period, deltaphi, IC(4));
%     %I=motor_parameter.ss_current(v,0);
%     %P=v*I*period;
%     %eng_diff=P+eng_diff_stance;
% 
%     %Calculate the power used by the motor
%     p=data.voltage.*data.curr;
%     t=data.tvec;
%     z=0;
% 
%     for i=1:length(p)-1
% 
%         % z=z+ (p(i)+p(i+1))/2*(t(i+1)-t(i)); %trapazodial rule
%          z=z+ (p(i)^2+p(i+1)^2)/2*(t(i+1)-t(i));
% 
% 
%     end
% 
%     %max sure x is going in the forward direction and y is increasing at
%     %the end of stance
% 
%     [~,dx,~,dy]=p2r(data.stanceIC(end,:));
%     dy=-dy;
% 
%     if dx<0
%         dx=dx*1000;
%     end
% 
%     if dy<0
%         dy=dy*1000;
%     end
% 
%     %index=find(data.switch==1);
%     %xval=data.rec_states(index(1),:);
% 
%     %f=z/motor(1)-2*dx-10*dy; %maximize x' y'
% 
%     % if dy<0 && dx>0
%     %     constraint_flag = -1;
%     % else
%     %     constraint_flag = 1;
%     % end
% 
%     % fval=z/motor_parameter.Ra-2*dx-10*dy; %maximize x' y'
%     index=find(data.switch==1);
%     % fval=z/motor_parameter.Ra + (data.rec_states(1,2)-data.rec_states(end,2))^2 + (data.rec_states(1,5)-data.rec_states(end,5))^2 + 10 * (data.stanceIC(1,3)-data.stanceIC(2,3))^2;
%     % fval=z/motor_parameter.Ra + (data.rec_states(1,2)-data.rec_states(index(1),2))^2 + (data.rec_states(1,5)-(-data.rec_states(index(1),5)))^2 + (data.stanceIC(1,3)-data.stanceIC(2,3))^2;
% 
%     fval=5*(data.rec_states(1,2)-data.rec_states(index(1),2))^2 + 5*(data.rec_states(1,5)-(-data.rec_states(index(1),5)))^2 + (data.stanceIC(1,3)-data.stanceIC(2,3))^2;

    data=Stance_sim_Ode(robot_param, IC,sim_param, 0.5,motor_parameter);
    %Find flight voltage
    dy=data.rec_states(end,5); %grab y speed
    period=2*abs(dy)/sim_param.g;   %calculate flight time
    omega0=data.polar_states(end,2);
    deltaphi=pi+IC(3)+pi-data.polar_states(end,1);
    flightV=motor_parameter.v_for_rot_in_time_T(period, deltaphi, omega0);
    % for a 0.5, 1.5 etc number of steps need to convert the second
        % part from - to +
    % fval=(data.rec_states(1,2)-data.rec_states(end,2))^2+(data.rec_states(1,5)+data.rec_states(end,5))^2;
    fval = (1-data.rec_states(end,2)/data.rec_states(1,2))^2 + (1-data.rec_states(end,5)/abs(data.rec_states(1,5)))^2;


else
    sim_param.stancemodel=@stance_model;
    sim_param.flightmodel=@flight_conservative_ode;
    data=Stance_sim_Ode(robot_param, IC,sim_param, 0.5);
    
    fval=data.Etot(end)-data.Etot(1);
    flightV = ["Check this statement"];
     
end
end

