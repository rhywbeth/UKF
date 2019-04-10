function plot_radar(xs, t)
figure(1)
plot(t, xs(:, 1)/1000);
xlim([0,t(end)]);
xlabel('Time[sec]');
ylabel('Position[km]')
grid on;

figure(2)
plot(t, xs(:, 2));
xlim([0,t(end)]);
xlabel('Time[sec]');
ylabel('Velocity[km/s]');
grid on;

figure(3)
plot(t, xs(:,3));
xlim([0,t(end)]);
xlabel('Time[sec]');
ylabel('Altitude[m]');
grid on;
end
    
    