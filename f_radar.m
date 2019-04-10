function d = f_radar(x, dt)
% State transition function for a constant velocity aircraft with state 
% vector [x, velocity, altitude]
F = [1, dt, 0;0, 1, 0;0, 0, 1];
d = F*x';
end