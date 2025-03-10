function [sim_param, robot, motor]=make_sim_param(textfile)
%Creates simulation, robot, and motor params.  Prevents the user from
%having to define everything at the top of each script.  That way if one
%parameter changes i.e. system mass can change it in one central location.
%Uses default values unless a text file is provided in which case it can
%read in text file.  I.e. you have 2 different robot models then the models
%can be defined in text files.  Copy and paste the following into a text
%file:
% Robot:
% Mass, 1.2
% l0, 0.17
% k0, 1699
% b, 5
% Xoffset, 0
% Yoffset, 0
% ------------------------------------------
% Simulation:
% tend, 1
% tsteps, 1.0000e-04
% tstepf, 1.0000e-04
% terrainvar, 0
% g, 9.8000
% tervec, 100
% yVar, 10
% filepath, filepathName
% stancemodel, StanceModel
% flightmodel, FlightModel
% ------------------------------------------
% Motor:
% Ra, 0.1350
% La, 1.6600e-05
% kt, 0.0098
% kb, 0.0098
% J, 1.8300e-06
% mech_c, 1.6000e-06
% omega0, 0
% curr0, 0.2000
% gearR, 33.0625
% gearJ, 8.0000e-08
% connect_torque, 0
% connect_voltage, 0
% omega_drive, 0

% Initialize Var
robot=Robot_Param;
sim_param=simulation_param;
motor=motor_param;


if ~exist('textfile','var')
    %Robot parameters
    robot.mass = 1.2; %mass in kilograms
    robot.l0   = 0.17; %leg length in meters
    robot.k0   = 1699; % leg stiffness
    robot.bl   = 5;
    
    %Simulation Parameters
    sim_param.tsteps      = 0.0001;
    sim_param.tstepf      = 0.0001;
    sim_param.stancemodel = "stance";
    sim_param.flightmodel = "flight_conservative";
    
    %Motor Parameters
    %Specs for 309755
    motor.Ra     = 0.135;      %armature resistance
    motor.La     = 0.0166e-3;       %armature inductance
    motor.kt     = 9.8e-3; %Nm/A motor torque constant
    motor.kb     = 9.8e-3;  %V/rad/sec back emf constant 974 V/rpm
    motor.J      = 1.83e-6;    %inetia of motor shaft kgm^2
    motor.mech_c = 1.6e-6;       %rotational damping
    motor.curr0  = 0.2;
    motor.gearR  = 529/16; %PN 166163
    motor.gearJ  = 8e-8;
    motor.omega_drive=0;
    
else
    fileID=fopen(textfile);
    
    %Robot parameters
    p=textscan(fileID,'%s',1);
    r=textscan(fileID,'%s %f',6,'Delimiter',',');
    
    robot.mass    = r{2}(1); %mass in kilograms
    robot.l0      = r{2}(2); %leg length in meters
    robot.k0      = r{2}(3); % leg stiffness
    robot.bl      = r{2}(4);
    robot.xoffset = r{2}(5);
    robot.yoffset = r{2}(6);
    
    %Simulation Parameters
    
    p=textscan(fileID,'%s',2);
    s=textscan(fileID,'%s %f',7,'Delimiter',',');
    
    sim_param.tend       = s{2}(1);
    sim_param.tsteps     = s{2}(2);
    sim_param.tstepf     = s{2}(3);
    sim_param.terrainvar = s{2}(4);
    sim_param.g          = s{2}(5);
    sim_param.tervec     = s{2}(6);
    sim_param.yVar       = s{2}(7);
    
    s=textscan(fileID,'%s %q',3,'Delimiter',',');
    sim_param.filepath    = convertCharsToStrings(s{2}(1));
    sim_param.stancemodel = convertCharsToStrings(s{2}(2));
    sim_param.flightmodel = convertCharsToStrings(s{2}(3));  
    
    p=textscan(fileID,'%s',2);
    s=textscan(fileID,'%s %f',13,'Delimiter',',');
    
    %Motor Parameters
    motor.Ra     = s{2}(1); %armature resistance
    motor.La     = s{2}(2); %armature inductance
    motor.kt     = s{2}(3); %Nm/A motor torque constant
    motor.kb     = s{2}(4); %V/rad/sec back emf constant 974 V/rpm
    motor.J      = s{2}(5); %inetia of motor shaft kgm^2
    motor.mech_c = s{2}(6); %rotational damping
    motor.omega0 = s{2}(7);
    motor.curr0  = s{2}(8);
    motor.gearR  = s{2}(9); 
    motor.gearJ  = s{2}(10);
    motor.connect_torque  = s{2}(11);
    motor.connect_voltage = s{2}(12);
    motor.omega_drive     = s{2}(13);
    
end
end