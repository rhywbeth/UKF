function xout = fx(x, dt)
    xout = zeros('like',x);
    % nonlinear function
    xout(1) = x(2)*dt + x(1);
    xout(2) = x(2);
end
