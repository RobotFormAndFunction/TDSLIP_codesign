Welcome to the world of legged locomotion 

Stance_sim runs the simulations and handles all the data parsing and formatting,
 switching between flight and stance, and handling the initial conditions when 
 switching models.

The heading for this function is fairly complete and calling help stance_sim will 
display all the information (true for most of my functions and classes).

This can run both controlled (with motor) and uncontrolled models.

Can adjust the number of iterations with iter.  Using 0.5 will only run the stance phase
or 1.5 would be two stances and one flight.

Func, and func_param variables allow you to pass it an anonymous function that performs 
additional computations that are specific to your simulation.  This anonymous function
 will be passed set variables and it knows what part of the simulation you’re at.
   See SLIP_col_May_Y for an example on how this works.


Stance_sim_Ode requires that the robot_parameter, simulation_parameter and optionally
the motor_parameter be passed to it.  These can be found in the classes folder.

motor_param 
	Properties of the motor.  Has a ton of functions for motor analysis built in
	 such as steady state speed, stall current, and other common performance metrics.
Robot_param
	A class that holds all properties of the physical robot i.e. mass, spring, etc.
Simulation_param 
	Contains variables relative to the simulation i.e. evaluation time step size,
	 what models you want to run and variable relative to rough terrain.

If you'd like to use the same robot parameters across multiple scripts without having to 
redifine them every time you can use the function make_sim_param which pulls this data
from a text file.

make_sim_param 
	This makes the motor, robot, and simulation param from a text which contains
	 all the properties.

An example text file is:
	Simulation_parameters.txt


An example of how this can be run can be found in evaluation_opt_sim.

Evaluation_opt_sim was used to plot the results from the optimization which are 
included as matlab workspaces: GA_fin_lab.mat (NSGA results) and 
ICRAF_convhist.mat (MDPSO) results

------------------------------------------------------------------------------------------

gait_sens performs the sensitivity analysis of the system

------------------------------------------------------------------------------------------

Select_motor will generate a motor_param from the 118 different motor options.

------------------------------------------------------------------------------------------
A number of simulation models are available.  

Stance:
stance_model - nonconservative SLIP model no motor 

stance_motor_ode -TD SLIP assumes voltage is provided in motor parameter class as
				  a two column matrix stanceV[t, v] and performs interpolation to find v
				  at time t

stance_motor_ode_poly - TD-SLIP model where voltage is a function of a polynomial.
						The polynomial values are in motor.stanceV as: 
						[a_1*t^n + a_2*t^(n-1)... + a_(n-1)*t+a_n].  This was done for
						performance reasons where the polynomial evaluation is faster than
						interpolation.
						
						
Flight:
flight_conservative_ode - nonconservative SLIP model no motor 

flight_conservative_ode_p - nonconservative SLIP model no motor all calculations are done
							in polar form.  Developed to try and chase small errors and 
							see if it would improve numerical stability (it didn't)

flight_motor_ode -  motor controls rotational speed of leg during flight.  Motor voltage
					is interpolated from motor.flightV[t,v].
					
flight_motor_con_v - assumes a constant voltage during flight motor.flightV=constant. 
					 Makes computations faster.

------------------------------------------------------------------------------------------					 

v_write and con_td_write are supplemental functions for Stance_sim_Ode that perform
						 additional calculations. If you want a purely conservative model
						 con_td_write allows the td angle to be kept constant stance to 
						 stance. v_write writes the polynomial constants to the workspace.
						 
------------------------------------------------------------------------------------------

The functions used for the optimization are outlined here.  MDPSO is a proprietary 
function and thus not shared here.  GA was run using matlab's built in algorithm.

Set up the constraints and cost function for the genetic algorithm
constraintsGA
objfun_Min_EnergyLoss_Sens_GA

set up everything for MDPSO
optimization_energy_dr_mdpso

The two algorithms were run using:
run_GA
run_MDPSO
------------------------------------------------------------------------------------------

I got tired of writing lots of lines of code every time I wanted to graph something and
kept having to graph the same things, so I made a graphing function called graphing.m. 
The main reason for doing this is the time vectors aren’t outputted thinking about it
now not sure why I’m not because it would be fairly easy.  The header explains how it 
works pretty well but having some examples is nice.  Here are all the calls I usually
make with it.

Data is the structure outputted by stance_sim

System Energy

graphing(data,sim_param,1,fignum)

Center of Mass

graphing(data,sim_param,3,fignum,[data.rec_states(:,1),data.rec_states(:,4)], ["r", "b--", "k"])

Motor Current and Voltage

labels={["Control Voltage", "Time [sec]","Voltage [V]"];
    ["Motor Current", "Time [sec]", "Current [Amps]"]};
graphing(data, sim_param, 4, fignum+1, {[data.voltage],[data.curr]},[],labels)

All the velocity position data in rectangular cords , Position, Velocity, Acceleration
 
label={["Position vs. Time","Time [sec]","Pos [m]","X", "Y"];
    ["Velocity vs. Time","Time [sec]","Vel [m/s]","X", "Y"];
    ["Acceleration vs. Time","Time [sec]","Accel [m/s^2]","X", "Y"]};
 
graph_dat={[data.rec_states(:,1), data.rec_states(:,4)],[data.rec_states(:,2), data.rec_states(:,5)],...
    [data.rec_states(:,3), data.rec_states(:,6)]};
graphing(data, sim_param, 4, fignum+2, graph_dat,[],label)
 
All the velocity position data in polar cords , Position, Velocity, Acceleration

label={["\theta, z vs. Time","Time [sec]","Pos [m]","\theta", "z"];
    ["theta', z' vs. Time","Time [sec]","Vel ","theta'", "z'"];
    ["\theta'', z' vs. Time","Time [sec]","Accel [m/s^2]","\theta''", "z''"]};
 
graph_dat={[data.polar_states(:,1), data.polar_states(:,4)],[data.polar_states(:,2), data.polar_states(:,5)],...
    [data.polar_states(:,3), data.polar_states(:,6)]};
graphing(data, sim_param, 4, fignum+3, graph_dat,[],label)

------------------------------------------------------------------------------------------
						 
						