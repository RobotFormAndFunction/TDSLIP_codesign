classdef Robot_Param
% Creates a Robot_Param Class
%   robot_param is a class with the following feilds:
%       - mass, mass of robot in [kg]
%       - l0, leg length [m]
%       - k0, spring stiffness [n/m]
%       - bl, leg damping if not provided this will default to zero
%       - xoffset, set if initial leg end effector not 0,0
%       - yoffset, set if initial leg end effector not 0,0
%         All values default to 0 if not specified
%
% Public Methods:
%   Unpack:
%       returns [mass, l0, k0, bl, xoffset, yoffset]

    properties
        mass = 0
        l0 = 0
        k0 = 0
        bl = 0
        xoffset = 0
        yoffset = 0
    end
    methods
        function [mass, l0, k0, bl, xoffset, yoffset]=unpack(obj)
            mass=obj.mass;
            l0=obj.l0;
            k0=obj.k0;
            bl=obj.bl;
            xoffset=obj.xoffset;
            yoffset=obj.yoffset;
            
        end

    end
end

