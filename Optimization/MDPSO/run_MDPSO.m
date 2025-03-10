clc; clear;close all;
%% Set Filepaths
folder = fileparts(which('stance.slx'));
% Add that folder plus all subfolders to the path.
addpath(genpath(folder));

%% MDPSO

% set parameters
Nv = 17;
Nvc = 15;
Nvd = 2;
Nobj = 1;
Nc = 16;
Ncie = 16;
Nce = 0;



%% MDPSO

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
total_idx_motor = 4*5 + 6*8 + 10*5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
E_list = 10*10^3:100000:130*10^9;
E_idx = 1:length(E_list);

func_param = [Nv,Nvc,Nvd,Nobj,Nc,Ncie,Nce];
vartype = [2,0;     % motor index
           0,0;     % robot base mass
           2,0;     % E
           0,0;     % R
           0,0;     % b
           0,0;     % h
           0,0;     % dmp
           0,0;     % IC2
           0,0;     % IC3
           0,0;     % IC4
           0,0;     % sa
           0,0;     % sb
           0,0;     % sc
           0,0;     % sd
           0,0;     % se
           0,0;     % sf
           0,0;];   % sg

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
dvvec = [];
max_min = 1;

algo_inputs = MDPSO_input_param(func_param,'Popsize',90,'InitpopGen','Sobols','Itermax',1000);
algo_control = MDPSO_control_param(func_param);
run_options = MDPSO_run_options('plotconv','ON','useparallel','ON','header','ICRAFIN');



%% Runnning MDPSO
optim = MDPSO('objfun_Min_EnergyLoss_MDPSO', func_param, max_min, vartype, varlim, dvvec, algo_inputs, algo_control, run_options);


