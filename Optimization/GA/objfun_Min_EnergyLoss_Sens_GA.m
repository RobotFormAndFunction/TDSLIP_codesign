function [obj] = objfun_Min_EnergyLoss_Sens_GA(vars)

    x = vars;
    [motor_label, robot_base_mass, E_idx, R, b, h, dmp, IC2, IC3, IC4, sa, sb, sc, sd, se, sf, sg] = deal(x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11), x(12), x(13), x(14), x(15), x(16), x(17));
    E_list = 10*10^3:100000:130*10^9;  % Gives E a fixed step size
    E = E_list(E_idx);
    
    robot=Robot_Param;
    sim_param=simulation_param;
    sim_param.stancemodel=@stance_motor_ode_poly;
    sim_param.flightmodel=@flight_motor_ode_con_v;

    mass_per_batt = 0.003;
%     vol_per_batt = 3.7;

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

    
    obj(1) = a(2);  % sensitivities
    obj(2) = a(3);
    obj(3) = e;     % symetricity

    fprintf('obj1: %.7f, obj2: %.7f, obj3: %.7f \n', obj)

end