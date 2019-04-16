clear()

% The tracking an airplane case with additional parameter of the climb rate 
% and with much higher standard deviation for range. The Doppler system velocity 
% readings are added (sensor fusion):
%  - Number of state variables for the filter is 4 and number of the measurement 
%    inputs also is 4. Time between steps equals 3 second.
%    x = [distance; velocity; atlitude; climb rate]
%    z = [slant distance; elevation angle; vx; vz]
%  - The process noise matrix Q is modeled with variance of 0.1. And the 
%    measurement noise matrix R is modeled with range standard deviation 
%    of 500 meters and elevation angle standard deviation of 0.5 degrees, 
%    the Doppler system velocity standard deviation of 2 m/s. 
%  - Merwe Scaled Sigma Points are used for generating the sigma points. 
%  - The state transition function for a constant velocity aircraft (f_cv_radar):
%    x_est =[1 dt 0 0;0 1 0 0;0 0 1 dt;0 0 0 1][distance; velocity; atlitude; climb rate]
%  - The measurement contains four values so the measurement function (h_vel) 
%    also needs to return four values. The velocity in x and y direction are 
%    measured by the Doppler system and included into the measurements. 
%    The slant range and elevation angle will be computed as before, and 
%    we do not need to compute the velocity in x and y as they are provided 
%    by the state estimate.

dt = 3;         % 12 seconds between readings
range_std = 500;  % meters
elevation_angle_std = degtorad(0.5);
vel_std = 2;   % m/s
R_std = [range_std, elevation_angle_std, vel_std, vel_std];

ac_pos = [0, 1000];   % 1000 m diraectly above the radar station 
ac_vel = [100, 0];    % 100 m/s
radar_pos = [0, 0];
h_radar.radar_pos = radar_pos;

% Dimensions
dim_x = 4; %[distance, velocity, altitude, climb_rate]
dim_z = 4; %[slant_range, elevation_angle, x_dot, y_dot]

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
    % simulate the doppler velocity reading
    vx = ac.vel(1) + randn*vel_std;
    vy = ac.vel(2) + randn*vel_std;
    ys(t,1) = ac.pos(2);
    ukf = ukf.predict(dt);
    ukf = ukf.update([r1,r2,vx,vy],ukf.R);
    xs(t,1:4) = ukf.x;
end

%figure;
plot_radar(xs, tim');
plot_altitude(xs, tim', ys)
sprintf('Velocity std : %.2d m/s',std(xs(:,2)))
sprintf('Altitude std : %.2d m',std(xs(:,3)))
sprintf('Climb rate std : %.2d m/s',std(xs(:,4)))

