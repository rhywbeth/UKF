function [slant_range, elevation_angle] = h_radar(x)
% Measurement function we need to convert the position and velocity 
% of the aircraft into the elevation angle and range from the radar station
% Range is computed with the Pythagorean theorem, 
% The elevation angle ? is the arctangent of y/x
radar_pos = [0, 0]; % is that fixed for a single simulation? 
dx = x(1) - radar_pos(1);
dy = x(3) - radar_pos(2);
slant_range = sqrt(dx^2 + dy^2);
elevation_angle = atan2(dy, dx);
end