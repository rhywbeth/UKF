classdef ACSim
    properties
       pos
       vel
       vel_std
    end
    methods
        function obj = ACSim(pos, vel, vel_std)
        obj.pos = pos;
        obj.vel = vel;
        obj.vel_std = vel_std;
        end
        
        function pos = update(obj, dt)
        % Compute and returns next position. Incorporates
        % random variation in velocity.
        dx = obj.vel*dt+(randn()*obj.vel_std)*dt;
        obj.pos = obj.pos+dx;
        pos = obj.pos;
        end  
    end
end