# TDSLIP_codesign

To use a fixed touchdown angle during optimization change:

	sim_param.flightmodel to @flight_conservative_ode

Then add the following to func_param
	
	func_param.F_con=2; %enable flight fixed touchdown flight control
	func_param.td_noise = [mu,sigma]; %noise is added as a gussian distribution
					  % mu is the offset and sigma is noise in 
					  % radians	
