function [motor, mass, num_batt]=select_motor(size, index, gear)
% SELECT_MOTOR creates a motor class structure based on different index
% posititions
%
% Inputs:
%           - size: enter a 1 for 6mm, 2 for 8mm, or 3 for 10mm
%
%           - index: select which motor you want within that size.  The
%           only different here is they have different voltage ratings the
%           6mm has 4 options, 8mm 6 options and 10mm 10 options
%
%           - gear: selects a different gearing option the 6mm has 5
%           options, 8mm has 8, and the 10mm has 5
%
% Ouputs:
%           - motor: Filled out motor parameter class
%
%           - Total mass of the combination
%
%           - How many 3.7v batteries would be needed
% 
%           - mpn: motor PN
%
%           - gpn: gearbox pn
motor=motor_param;

switch size
    case 1
        %PN, Ra, La, kt/kb, J, no load speed [rpm]
        motor_p=[386780 386781 38782 386783;
            2.58 8.92 20.8 37.2;
            2.3e-5 9.1e-5 2.04e-4 3.63e-4;
            7.2e-4 1.44e-3 2.16e-3 2.88e-3;
            1.5e-9 1.67e-9 1.63e-9 1.62e-9;
            18500 18600 18600 18600];
        
        weight=2.3;
        
        V=[1.5 3 4.5 6];
        
        %the rows are: PN, weight, J, gear reduction
        gear_param=[472919, 472920, 472921, 472229, 472922
            1.7 2.1 2.5 2.9 3.3;
            1e-10 1e-10 1e-10 1e-10 1e-10;
            27/7 729/49 19683/343 531441/2401 14348907/16807]; 
        
        
    case 2
       %PN, Ra, La, kt/kb, J, no load speed [rpm] 
        motor_p=[462207 463219 463220 463221 463222 463223;
            4.13 12.3 29 38.5 54.5 92.2;
            3e-5 9e-5 2.06e-4 2.57e-4 4e-4 6.06e-4;
            1.59e-3 2.74e-3 4.15e-3 4.63e-3 5.77e-3 7.11e-3;
            3.83e-9 3.83e-9 3.75e-9 3.58e-9 3.87e-9 3.55e-9;
            13900 14200 13300 14300 14400 15600];
        
        weight=4;
        
        V=[2.4 4.2 6 7.2 9 12];
        
        gear_param=[46899 468998 474124 468997 272127 468996 474129 468995;
            2.6 3.2 3.2 3.8 3.8 4.4 4.4 5.0;
            4e-10 4e-10 4e-10 4e-10 4e-10 4e-10 4e-10 4e-10;
            4 16 36 64 216 256 1296 1024];
        
        
    case 3
        %PN, Ra, La, kt/kb, J, no load speed [rpm]
        motor_p=[118382 118383 118384 118385 118386 118387 118388 118389 118390 118391;
            5.55 8 12.7 15.2 20.6 25.8 36.4 47.9 72.9 114;
            4.6e-5 7.2e-5 1.12e-4 1.36e-4 1.84e-4 2.4e-4 3.25e-4 3.98e-4 6.05e-4 9.2e-4;
            2.14e-3 2.67e-3 3.34e-3 3.67e-3 4.27e-3 4.88e-3 5.68e-3 6.28e-3 7.75e-3 9.55e-3;
            6.6e-9 7.11e-9 7.04e-9 7.06e-9 7.06e-9 7.26e-9 7.06e-9 6.66e-9 6.66e-9 6.54e-9;
            13000 11100 9930 1130 13000 11400 11400 10600 10700 11600];
        
        weight=7;
        
        V=[2.4 3 3.6 4.5 6 6 7.2 7.2 9 12];
        
        gear_param=[218415, 218416, 218417, 218418, 218419
            6.7 7.2 7.7 8.2 8.7;
            5e-10 5e-10 5e-10 5e-10 5e-10;
            4 16 64 256 1024]; %the first rows are: PN, weight, J, gear reduction
        
        
        
    
end

%motor
motor.Ra = motor_p(2,index);
motor.La = motor_p(3,index);
motor.kt = motor_p(4,index);
motor.kb = motor_p(4,index);
motor.J  = motor_p(5,index);
omega=motor_p(6,index)/30*pi; %convert max rpm to rad/sec
motor.mech_c=(V(index)*motor.kt-omega*motor.kt^2)/(omega*motor.Ra);


% gearbox
motor.gearJ=gear_param(3,gear);
motor.gearR=gear_param(4,gear);

%mass
mass=(gear_param(2,gear)+weight)/1000; %total mass in kg

%PN
motor.PN=motor_p(1,index);
motor.GearPN=gear_param(1,gear);

num_batt=ceil(V(index)/3.7);



end