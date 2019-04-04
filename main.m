clear()

% dim_x : Number of state variables for the filter.
% dim_z : Number of of measurement inputs.
% dt : Time between steps in seconds.

dim_x = 2;
dim_z = 1;
dt = 1;

% Create sigma points
% JulierSigmaPoints(n, kappa)
% sp = JulierSigmaPoints(dim_x, 1);
% MerweScaledSigmaPoints(n, alpha, beta, kappa)
sp = MerweScaledSigmaPoints(dim_x, 0.1, 2, -1);

% Unscented Kalman Filter
ukf = UnscentedKalmanFilter(2, 1, 1, sp);
ukf.P = ukf.P * 10;
ukf.R = ukf.R * 0.5;
ukf.Q = Q_discrete_white_noise(2, 1, 0.03, 1, true);
zs = [];
xs = [];
for i = 1:100
    z = sin(0.1*i) + randn*0.1;
    ukf = predict(ukf, 0);
    ukf = update(ukf,z,ukf.R);
    xs(i) = (ukf.x(1));
    zs(i) = (z);
end

% Plot results
figure(1)
plot(xs)
hold on;
plot(zs,'x');
grid on;
