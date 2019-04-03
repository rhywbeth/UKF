function arr = f_nonlinear_xy(x,y)
% Compute linearized mean
if size(x) == 1
    arr = [x+y, 0.1*x*x + y*y];
else
    arr = [x+y, 0.1*x.^2 + y.^2];
end
end