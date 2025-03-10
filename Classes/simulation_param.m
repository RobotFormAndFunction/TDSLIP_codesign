classdef simulation_param
% Creates a Robot_Param Class
%   sim_param a structure with the following fields
%       - tend, type double, maximum number of seconds to run each simulation
%         for default 1 sec
%       - tsteps, type double, time step for stance simulation, default 0.0001s
%       - tstepf, type double, time step for flight model, default 0.0001s
%       - stancemodel, type string, name of stance model
%       - flightmodel, type string, name of flight model
%       - filepath, type string, optional field if filepath for sim is in a
%         different folder
%       - g, type double, gravity constant, defaults to 9.8 if not specified
%       - terrainvar, variable to set is model will have variable
%         terrrain, set 1 for varaible terrain, 0 for constant y offest
%         defined by robot_param class
%       - tervec, vector of terrain values.  Can be used so multiple
%         simulations can be run with the same terrain.  Does not check that
%         length is correct for number of iterations.  Defaults to value of
%         100, replace with vector to use.
%       - yVar terrain variation is a Gaussian distribution with mean of 0
%         and sigma value of l0/yVar defaults to 10 
%       - opt set to 1 is trying to run optimizations on initial conditions
%         it prevents the code from breaking out if the stance starts
%         getting wonky
%
% Public Methods:
%   Unpack:
%       returns [tend, tsteps, tstepf, stancemodel, flightmodel, g, terrainvar, tervec]
%       will add any needed filepaths

    properties
        tend=1
        tsteps=0.0001
        tstepf=0.0001
        stancemodel=""
        flightmodel=""
        terrainvar=0
        g=9.8
        filepath=""
        tervec=100;
        yVar=10;
        opt=0;
    end
    methods
        function [tend, tsteps, tstepf, stancemodel, flightmodel, g, terrainvar, tervec]=unpack(obj)
            tend=obj.tend;
            tsteps=obj.tsteps;
            tstepf=obj.tstepf;
            stancemodel=obj.stancemodel;
            flightmodel=obj.flightmodel;
            g=obj.g;
            terrainvar=obj.terrainvar;
            tervec=obj.tervec;
            
            if obj.filepath~=""
                addpath(obj.filepath)
            end
            
        end

    end
end

