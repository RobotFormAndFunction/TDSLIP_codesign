%clc; clear;
close all
%% Set Filepaths

load("GA_fin_lab.mat")

%% Plot with new plotting functions
% ct = table2array(results.XAtMinObjective); %categorical table
% st = string(ct);
% x = str2double(st);
% 
% % graph_opt_outputs(x, 1, 5)
% 


% Assign opt vars
% all_pos = {pop.Position};
% pick one for simulation
% x = all_pos{1};

% ct = table2array(results.XTrace(944,:));
% load("workspace815.mat")

% BO
% ct = table2array(results.XAtMinObjective); %categorical table
% st = string(ct);
% x = str2double(st);
% [motor_label, robot_base_mass, E, R, b, h, dmp, IC2, IC3, IC4, flightV_ratio, sa, sb, sc, sd, se, sf, sg] = deal(x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11), x(12), x(13), x(14), x(15), x(16), x(17), x(18));

% MDPSO
load("ICRAF_convhist.mat")
x = globalbestlog(end).var;
[sim_param_m,motor_m,robot_m,IC_m]=form_sims(x);

%GA
load("GA_fin_lab.mat")
x1=x(33,:);
[sim_param_g,motor_g,robot_g,IC_g]=form_sims(x1);
x1=x(14,:);
[sim_param_g2,motor_g2,robot_g2,IC_g2]=form_sims(x1);
x1=x(21,:);
[sim_param_g3,motor_g3,robot_g3,IC_g3]=form_sims(x1);


data1=Stance_sim_Ode(robot_m,IC_m,sim_param_m,2.5,motor_m);
data2=Stance_sim_Ode(robot_g,IC_g,sim_param_g,2.5,motor_g);
data3=Stance_sim_Ode(robot_g2,IC_g2,sim_param_g2,2.5,motor_g2);
data4=Stance_sim_Ode(robot_g3,IC_g3,sim_param_g3,2.5,motor_g3);
%% Simulation
% [e,data1]=energy_loss(robot,IC,sim_param)
% [e,data1] = cost_fun_feng(robot,IC,sim_param, motor)
% [a,b]=gait_sens(robot, IC, sim_param, motor, 1, 1)

% %% Plotting Objetives
% objs = results.ObjectiveTrace;
% feas = results.FeasibilityTrace;

% Get switching points
index1=find(data1.switch==1);
index2=find(data2.switch==1);
index3=find(data3.switch==1);
index4=find(data4.switch==1);

index1=[1;index1];
index2=[1;index2];
index3=[1;index3];
index4=[1;index4];
ty=1;

figure(31)
%subplot(2,1,1)
hold on
plot(data1.tvec,data1.voltage, 'Color',"#C32026",'LineWidth',1.5)
plot(data2.tvec,data2.voltage, 'Color',"#092B9C",'LineWidth',1.5)
plot(data3.tvec,data3.voltage, 'Color',"#238A31",'LineWidth',1.5)
plot(data4.tvec,data4.voltage, 'Color',"#ff8426",'LineWidth',1.5)

font_s = 18;
xl = xlabel("Time [sec]");
yl = ylabel("Voltage [V]");
set(xl, 'FontSize', font_s)
set(yl, 'FontSize', font_s)
ax = gca; % Get the current axes handle
set(ax, 'FontSize', font_s); % Adjust 14 to your desired font size
lgd = legend('MDPSO', 'NSGA \ast', 'NSGA \nabla', 'NSGA O');

fontsize(lgd,14,'points')

hold off


figure(40)
%subplot(2,1,1)
hold on
plot(data1.tvec(index1([2,4,6])),[0,0,0], 'Color',"#C32026",'LineStyle','none','Marker','*','LineWidth',1.5)
plot(data2.tvec(index2([2,4,6])),[0,0,0], 'Color',"#092B9C",'LineStyle','none','Marker','*','LineWidth',1.5)
plot(data3.tvec(index3([2,4,6])),[0,0,0], 'Color',"#238A31",'LineStyle','none','Marker','*','LineWidth',1.5)
plot(data4.tvec(index4([2,4,6])),[0,0,0], 'Color',"#ff8426",'LineStyle','none','Marker','*','LineWidth',1.5)

font_s = 18;
axis([0 0.45 -0.01 0.01])
xl = xlabel("Time [sec]");
set(xl, 'FontSize', font_s)
set(yl, 'FontSize', font_s)
ax = gca; % Get the current axes handle
set(ax, 'FontSize', font_s); % Adjust 14 to your desired font size
lgd = legend('MDPSO', 'NSGA \ast', 'NSGA \nabla', 'NSGA O');

fontsize(lgd,14,'points')

hold off

figure(41)
%subplot(2,1,1)
hold on
plot(data1.tvec(1:index1(2)),data1.voltage(1:index1(2)), 'Color',"#C32026",'LineWidth',1.5)
plot(data2.tvec(1:index2(2)),data2.voltage(1:index2(2)), 'Color',"#092B9C",'LineWidth',1.5)
plot(data3.tvec(1:index3(2)),data3.voltage(1:index3(2)), 'Color',"#238A31",'LineWidth',1.5)
plot(data4.tvec(1:index4(2)),data4.voltage(1:index4(2)), 'Color',"#ff8426",'LineWidth',1.5)

font_s = 18;
xl = xlabel("Time [sec]");
yl = ylabel("Voltage [V]");
set(xl, 'FontSize', font_s)
set(yl, 'FontSize', font_s)
ax = gca; % Get the current axes handle
set(ax, 'FontSize', font_s); % Adjust 14 to your desired font size
lgd = legend('MDPSO', 'NSGA \ast', 'NSGA \nabla', 'NSGA O');

fontsize(lgd,14,'points')

hold off

% subplot(2,1,2)
% hold on
% plot(data1.tvec, data1.curr,'Color',"#C32026",'LineWidth',1.5)
% plot(data2.tvec, data2.curr,'Color',"#092B9C",'LineWidth',1.5)
% plot(data3.tvec, data3.curr,'Color',"#238A31",'LineWidth',1.5)
% plot(data4.tvec, data4.curr,'Color',"#ff8426",'LineWidth',1.5)
% 
% font_s = 18;
% xl = xlabel("Time [sec]");
% yl = ylabel("Current [A]");
% set(xl, 'FontSize', font_s)
% set(yl, 'FontSize', font_s)
% ax = gca; % Get the current axes handle
% set(ax, 'FontSize', font_s); % Adjust 14 to your desired font size
% hold off

figure(32)
ty=1;
hold on
for i=1:length(index1)-1
        if ty ==1
            ty=2;
            plot(data1.rec_states(index1(i):index1(i+1),1), ...
            data1.rec_states(index1(i):index1(i+1),4), 'Color',"#C32026",'LineWidth',1.5)
            
            if i<length(index2)+1
            plot(data2.rec_states(index2(i):index2(i+1),1), ...
            data2.rec_states(index2(i):index2(i+1),4), 'Color',"#092B9C",'LineWidth',1.5)
            end

            if i<length(index3)+1
            plot(data3.rec_states(index3(i):index3(i+1),1), ...
            data3.rec_states(index3(i):index3(i+1),4), 'Color',"#238A31",'LineWidth',1.5)
            end

            if i<length(index4)+1
            plot(data4.rec_states(index4(i):index4(i+1),1), ...
            data4.rec_states(index4(i):index4(i+1),4), 'Color',"#ff8426",'LineWidth',1.5)
            end
        else
            ty=1;
            plot(data1.rec_states(index1(i):index1(i+1),1), ...
            data1.rec_states(index1(i):index1(i+1),4), 'Color',"#E19093",'LineStyle','--','LineWidth',1.5)

            if i<length(index2)+1
            plot(data2.rec_states(index2(i):index2(i+1),1), ...
            data2.rec_states(index2(i):index2(i+1),4), 'Color',"#2DA0ED",'LineStyle','--','LineWidth',1.5)
            end

            if i<length(index3)+1
            plot(data3.rec_states(index3(i):index3(i+1),1), ...
            data3.rec_states(index3(i):index3(i+1),4), 'Color',"#ACC674",'LineStyle','--','LineWidth',1.5)
            end

            if i<length(index4)+1
            plot(data4.rec_states(index4(i):index4(i+1),1), ...
            data4.rec_states(index4(i):index4(i+1),4), 'Color',"#FFBE8C",'LineStyle','--','LineWidth',1.5)
            end
        end
end

font_s = 18;
xl = xlabel("X pos [m]");
axis([-0.05, 0.3, 0, 0.07])
yl = ylabel("Y pos [m]");
set(xl, 'FontSize', font_s)
set(yl, 'FontSize', font_s)
ax = gca; % Get the current axes handle
set(ax, 'FontSize', font_s); % Adjust 14 to your desired font size
lgd = legend('MDPSO', 'NSGA \ast', 'NSGA \nabla', ...
    'NSGA O');%'MDPSO Flight', 'NSGA Flight \ast','NSGA Flight \nabla');
fontsize(lgd,14,'points')

%% make animation
tend=max([data1.tvec(end),data2.tvec(end),data3.tvec(end),data4.tvec(end)]);

tvec=linspace(0,tend,1001);


p1=movie_points(data1,index1,tvec);

figure(106)
for i=1:length(tvec)
    plot(data1.rec_states(:,1),data1.rec_states(:,4),'k--')
    hold on 
    plot(p1(i,1),p1(i,3),'r*')
    plot([p1(i,1);p1(i,2)],[p1(i,3);p1(i,4)],'k')
    hold off
    axis([-0.05,0.25,0,0.12])
    tx=['T=' num2str(tvec(i),3)  ' [s]'];
    text(0.15,0.08,tx)
    pause(0.0005)
end

figure(105)
hold on
for i=1:length(tvec)
    subplot(4,1,1)
    plot(data1.rec_states(:,1),data1.rec_states(:,4),'k--')
    plot(p1(i,1),p1(i,3),'r*')
    %plot([p1(i,1);p1(i,2)],[p1(i,3);p1(i,4)],'k')
    axis([-0.05,0.25,0,0.12])

    subplot(4,1,2)

    plot(data2.rec_states(:,1),data2.rec_states(:,4),'k--')
    axis([-0.05,0.25,0,0.12])

    subplot(4,1,3)

    plot(data2.rec_states(:,1),data2.rec_states(:,4),'k--')
    axis([-0.05,0.25,0,0.12])

    subplot(4,1,4)

    plot(data2.rec_states(:,1),data2.rec_states(:,4),'k--')
    axis([-0.05,0.25,0,0.12])


end

hold off

%% Functions

function [dat]=movie_points(data,index,tvec)
    dat=zeros(length(tvec),4);
    xoffset=data.rec_states(index,1);
    index=index(2:end);

    l0=data.stanceIC(1);

    for i=1:length(data.stanceIC(:,1))
        xoffset(2*i-1)=cos(data.stanceIC(i,3))*l0+xoffset(2*i-1);
    end

    ty=1;
    count=1;

    for i=1:length(tvec)
        tcur=tvec(i);
        if tcur<data.tvec(end)
            k=find(abs(data.tvec-tcur)==min(abs(data.tvec-tcur)));

            dat(i,1)=data.rec_states(k,1);
            dat(i,3)=data.rec_states(k,4);

            if ty==1

                dat(i,2)=xoffset(count);

                if tvec(i+1)>data.tvec(index(count))
                    ty=2;
                    count=count+1;
                end

            else

                theta=data.polar_states(k,1);
          
                dat(i,2)=dat(i,1)+l0*cos(theta);
                dat(i,4)=dat(i,3)+l0*sin(theta);

                % need to find the x and y position interpolate theta and
                % add

                if tvec(i+1)>data.tvec(index(count))
                    ty=1;
                    count=count+1;
                end

            end
            

        else
            dat(i,:)=dat(i-1,:);
        end

    end


end

function [sim_param,motor,robot,IC]=form_sims(x)
[motor_label, robot_base_mass, E_idx, R, b, h, dmp, IC2, IC3, IC4, sa, sb, sc, sd, se, sf, sg] = deal(x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11), x(12), x(13), x(14), x(15), x(16), x(17));
E_list = 10*10^3:100000:130*10^9;
E = E_list(E_idx);

labels = {"motor_label", "robot_base_mass", "E_idx", "R", "b", "h", "dmp", "IC2", "IC3", "IC4", "sa", "sb", "sc", "sd", "se", "sf", "sg"};
labels = labels(:);
x = x(:);
T = table(labels, x);
T.Properties.VariableNames = {'Label', 'Value'};
%disp(T)


[sim_param, robot, ~] = make_sim_param('simulation parameters.txt');
% mass_per_batt = 0.048;
mass_per_batt = 0.003;
vol_per_batt = 3.7;

motor_size = 0; motor_index = 0; motor_gear = 0;

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

fprintf('%f, %f, %f \n',[motor_size,motor_index,motor_gear])

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


% t_arr = linspace(0,1.01,100)';
% stanceV = t_arr.^6*sa + t_arr.^5*sb + t_arr.^4*sc + t_arr.^3*sd + t_arr.^2*se + t_arr.^1*sf + sg;
% stanceV = [sa, sb, sc, sd, se, sf, sg];
% % motor.stanceV = [t_arr,stanceV];
% motor.stanceV = stanceV;
stanceV = [sa, sb, sc, sd, se, sf, sg];
motor.stanceV = stanceV;

% [e,data]=energy_loss(robot,IC,sim_param);
[e,data] = cost_fun_feng(robot,IC,sim_param, motor);
% save('input_gait_sens.mat', 'robot', 'IC', 'sim_param','x');

%[a,b]=gait_sens(robot, IC, sim_param, motor, 1, 1);


%Calculate the flight voltage
dy=data.rec_states(end,5); %grab y speed
period=2*abs(dy)/sim_param.g;   %calculate flight time
omega0=data.polar_states(end,2);
deltaphi=pi+IC(3)+pi-data.polar_states(end,1);
motor.flightV=motor.v_for_rot_in_time_T(period, deltaphi, omega0);

%Run simulation for plotting
sim_param.stancemodel=@stance_motor_ode_poly;
sim_param.flightmodel=@flight_motor_ode_con_v;

end