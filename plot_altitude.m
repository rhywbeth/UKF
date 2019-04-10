function plot_altitude(xs, t, track)
 plot(t, xs(:,3));
 hold on;
 plot(t, track);
 xlim([0,t(end)]);
 xlabel('Time[sec]');
 ylabel('Altitude[m]');
 grid on;
end