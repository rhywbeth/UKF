function [slant_range, elevation_angle, z1, z2] = h_vel(x)
% The measurement function (h_vel) returns four values:
% - the slant range 
% - elevation angle 
% Both will be computed as before.
% - the velocity in x 
% - the velocity in y
% They are provided by the state estimate.
radar_pos = [0 0];
dx = x(1) - radar_pos(1);
dz = x(3) - radar_pos(2);
slant_range = sqrt(dx^2 + dz^2);
elevation_angle = atan2(dz, dx) ;
z1 = x(2);
z2 = x(4);
end