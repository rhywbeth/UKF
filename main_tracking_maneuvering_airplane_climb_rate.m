clear()

% The tracking an airplane case with additional parameter of the climb rate: 
%  - The filter will track an airplane using again radar as the sensor. We 
%    track an airplane again only in two dimensions: one dimension on the ground 
%    and second dimension the altitude of the aircraft.
%  - Slant distance is the straight line distance from the radar to the object. 
%    Bearing is computed using the directive gain of the antenna.
%  - Number of state variables for the filter is 4 and number of the measurement 
%    inputs still is 2. 
%    x = [distance; velocity; atlitude; climb rate]
%    z = [slant distance; elevation angle]
%  - The process noise matrix Q is modeled with variance of 0.1. And the 
%    measurement noise matrix R is modeled with range standard deviation 
%    of 5 meters and elevation angle standard deviation of 0.5 degrees. 
%  - Merwe Scaled Sigma Points are used for generating the sigma points. 
%  - The state transition function for a constant velocity aircraft (f_cv_radar):
%    x_est = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1]*[distance; velocity; atlitude; climb rate]
%  - The measurement function (h_radar) convert the position and velocity 
%    of the aircraft(ac_pos) into the elevation angle and range from the 
%    radar station (radar_pos = [0, 0]). 
%  - Range is computed with the Pythagorean theorem:
%    range = ?((x_airplane-x_radar )^2+(y_airplane-y_radar )^2)
%  - The elevation angle is the arctangent of y/x:
%    ? = (tan)^(-1)*(y_airplane-y_radar)/(x_airplane-x_radar)


dt = 3;         % seconds between readings
range_std = 5;  % meters
elevation_angle_std = degtorad(0.5);
R_std = [range_std, elevation_angle_std];

ac_pos = [0, 1000];   % 1000 m diraectly above the radar station 
ac_vel = [100, 0];    % 100 m/s
radar_pos = [0, 0];
h_radar.radar_pos = radar_pos;

% Dimensions
dim_x = 4; %[distance,velocity,altitude,climb_rate]
dim_z = 2;
% MerweScaledSigmaPoints(n, alpha, beta, kappa)
sp = MerweScaledSigmaPoints(dim_x, 0.1, 2, -1);

% Unscented Kalman Filter
ukf = UnscentedKalmanFilter(dim_x, dim_z, dt, sp);

ukf.Q(1:2,1:2) = Q_discrete_white_noise(2, dt, 0.1, 1, true);
ukf.Q(3:4,3:4) = Q_discrete_white_noise(2, dt, 0.1, 1, true);

ukf.R = diag(R_std.^2);
ukf.x = [0, 90, 1100, 0];
ukf.P = diag([300^2, 3^2, 150^2, 3^2]);

pos = [0, 0];
radar = RadarStation(pos, range_std, elevation_angle_std);
ac = ACSim(ac_pos, [100, 0], 0.02);

time = 360/dt+1;
tim = 0:3:360;
xs = [0 0 0];
ys = [0];
for t = 1:time
    if t >= 30
        ac.vel(2) = 300/60;
    end
    ac.pos = ac.update(dt);
    [r1,r2] = radar.noisy_reading(ac.pos);
    ys(t,1) = ac.pos(2);
    ukf = ukf.predict(dt);
    ukf = ukf.update([r1,r2],ukf.R);
    xs(t,1:4) = ukf.x;
end

plot_radar(xs, tim');
figure;
plot_altitude(xs, tim', ys)
sprintf('Actual altitude : %d',ac.pos(2))
sprintf('UKF altitude : %d',xs(time,3))

