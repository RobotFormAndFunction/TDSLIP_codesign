function [obj, C_eq, C_ineq] = objfun_Min_EnergyLoss_MDPSO(vars)

    persistent counter;

    if isempty(counter)
        counter = 0;
    end
    
    % Increment the counter
    counter = counter + 1;
    
    % Print the counter value
    fprintf('This is the %d-th time running obj function.\n', counter);

    C_eq = [];
    C_ineq = zeros(1,16);

    x = vars;
    [motor_label, robot_base_mass, E_idx, R, b, h, dmp, IC2, IC3, IC4, sa, sb, sc, sd, se, sf, sg] = deal(x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11), x(12), x(13), x(14), x(15), x(16), x(17));
    E_list = 10*10^3:100000:130*10^9;  % set a step size for E
    E = E_list(E_idx);
    
    robot=Robot_Param;
    sim_param=simulation_param;
    sim_param.stancemodel=@stance_motor_ode_poly;
    sim_param.flightmodel=@flight_motor_ode_con_v;

    mass_per_batt = 0.003;
    vol_per_batt = 3.7;

    motor_size = 0; motor_index = 0; motor_gear = 0;

    % assign motor_size, motor_index, motor_gear
    if motor_label<=20
        motor_size = 1;
        motor_index = ceil(motor_label/5);
        motor_gear = mod(motor_label,5)+1;

    elseif motor_label>20 && motor_label<=68
        motor_size = 2;
        motor_index = ceil((motor_label-20)/8);
        motor_gear = mod((motor_label-20),8)+1;

    elseif motor_label>68
        motor_size = 3;
        motor_index = ceil((motor_label-68)/5);
        motor_gear = mod((motor_label-68),5)+1;
    end

    [motor, motor_mass, num_batt] = select_motor(motor_size, motor_index, motor_gear);
    mass_batt = num_batt*mass_per_batt;
    mass_addition = 2*(2*motor_mass + 0.005 + num_batt*mass_batt);
    robot.mass = robot_base_mass + mass_addition;
    robot.bl=dmp;

    I = b*h^3/12;
    robot.k0 = 2*E*I/(R^3*pi);
    robot.l0 = 2*R;

    motor.connect_torque=1;
    motor.connect_voltage=1;

    IC = [2*R, IC2, IC3, IC4];

    sim_param.g=9.81;
    sim_param.terrainvar=0;

    stanceV_var = [sa, sb, sc, sd, se, sf, sg];
    motor.stanceV = stanceV_var;

    % call cost function
    [e,data1, flightV] = cost_fun_feng(robot,IC,sim_param, motor);
    save('input_gait_sens.mat', 'robot', 'IC', 'sim_param','x');
    [a,b]=gait_sens(robot, IC, sim_param, motor, 1, 1);

    stanceV = data1.voltage;
    motor.flightV = flightV;

    %     minimizing 2nd and 3rd of a; using these as constraints
    entry2 = a(2);
    entry3 = a(3);

    entry2_threshold = 30;
    entry3_threshold = 30;

    C_ineq(1) = (max(abs(stanceV)) - num_batt * vol_per_batt) / (num_batt * vol_per_batt);      % V <= power supply
    C_ineq(2) = (max(abs(flightV)) - num_batt * vol_per_batt) / (num_batt * vol_per_batt);
    C_ineq(3) = (entry2 - entry2_threshold) / entry2_threshold;                                 % sensitivity
    C_ineq(4) = (entry3 - entry3_threshold) / entry3_threshold;
    C_ineq(5) = (-min(data1.rec_states(:,4)) + 0.00000001) / robot.l0;                           % y > 0     (robot leg length as scaler)
    C_ineq(6) = (-data1.rec_states(end,2) + 0.00000001) / abs(data1.rec_states(1,2));           % end x vel > 0
    C_ineq(7) = (-data1.rec_states(end,5) + 0.00000001) / abs(data1.rec_states(1,5));           % end y vel > 0

    C_ineq(8) = (-data1.rec_states(end,1) + 0.00000001) / robot.l0;                           % x > 0     (robot leg length as scaler)
    
    data_len = length(data1.rec_states(:,4));
    midpoint = round(data_len/2);
    C_ineq(9) = (data1.rec_states(midpoint,4) - data1.rec_states(1,4)) / data1.rec_states(1,4);            % y(mid) < y(1)
    C_ineq(10) = (data1.rec_states(midpoint,4) - data1.rec_states(end,4)) / data1.rec_states(end,4);        % y(mid) < y(end)
    
    x_vel_ref = 0.05;
    max_v = max([stanceV;flightV]);
    C_ineq(11) = (-mean(data1.rec_states(:, 2)) + x_vel_ref) / x_vel_ref;                     % average x vel > ref
    C_ineq(12) = ( motor.ss_speed(robot.mass*IC(1), max_v) - IC(4) ) / IC(4);       % motor.ss_speed(robot.mass*IC(1), max_v)<IC(4)
    C_ineq(13) = ( data1.rec_states(end, 5) - 2 ) / 2;                              % end y vel < 1 
    C_ineq(14) = (atan2(data1.rec_states(1,5),data1.rec_states(1,2)) - (-0.8)) / 0.8 ;          % inital vel angle -1.5~-0.8
    C_ineq(15) = (atan2(data1.rec_states(1,5),data1.rec_states(1,2)) + (-1.5)) / 1.5 ;
    C_ineq(16) =  (data1.KE(1)/data1.PeGravity(1) - 2.5) / 2.5;                     % data.KE(1)/data.PeGravity(1)<2.5
    obj = e;
end