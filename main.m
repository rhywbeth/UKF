clear()

% Mean and covariance
meann = [0,0];
p = [32, 15; 15, 40];

% Compute linearized mean
mean_fx = f_nonlinear_xy(meann(1),meann(2));

% Generate random points
s = mvnrnd(meann,p,10000);
xs = s(:,1);
ys = s(:,2);

% Calculate difference in mean:
fs = f_nonlinear_xy(xs, ys);
fxs = fs(:,1);
fys = fs(:,2);
computed_mean_x = mean(fxs);
computed_mean_y = mean(fys);
n1 = computed_mean_x - mean_fx(1);
n2 = computed_mean_y - mean_fx(2);
sprintf('Difference in mean x=%d, y=%d',n1,n2)

% Create sigma points
sp = JulierSigmaPoints(2, 1);

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