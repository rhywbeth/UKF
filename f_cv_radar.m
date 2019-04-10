function d = f_cv_radar(x, dt)
% State transition function for a constant velocity aircraft
F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
d = F*x';
end