clc; clear;close all;
%% Set Filepaths

folder = fileparts(which('stance.slx'));
addpath(genpath(folder));

%% Set Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
total_idx_motor = 4*5 + 6*8 + 10*5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
E_list = 10*10^3:100000:130*10^9;   % Set up step size
E_idx = 1:length(E_list);


varlim = [1, total_idx_motor;
           0.01, 1;     % robot base mass
           1, max(E_idx);     % E
           0.001, 0.03;     % R
           0.0005, 0.01;     % b
           0.0005, 0.01;     % h
           0.0005, 0.01;     % dmp
           -10, -0.1;     % IC2
           0.45, 1.4835;     % IC3
           5, 100;     % IC4
           -2, 2;     % sa
           -2, 2;     % sb
           -2, 2;     % sc
           -2, 2;     % sd
           -2, 2;     % se
           -2, 2;     % sf
           -2, 2;];   % sg

nvars = size(varlim,1);
lb = varlim(:,1);
ub = varlim(:,2);
intcon = [1,3];

fun = @objfun_Min_EnergyLoss_Sens_GA;
nonlcon = @constraintsGA;

options = optimoptions('gamultiobj', 'UseParallel', true, 'CrossoverFraction', 0.8, 'MaxGenerations', 300, 'PopulationSize', 170, 'Display', 'iter');

[x,fval,exitflag,output,population,scores] = gamultiobj(fun,nvars,[],[],[],[],lb,ub,nonlcon,intcon,options);
