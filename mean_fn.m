function x = mean_fn(sigmas, Wm)
x = zeros(3);
sum_sin = 0;
sum_cos = 0;
for i = 1:(length(sigmas))
    s = sigmas(i);
    x(1) = x(1) + s(1) * Wm(i);
    x(2) = x(2) + s(2) * Wm(i);
    sum_sin = sum_sin + sin(s(3)) * Wm(i);
    sum_cos = sum_cos + cos(s(3)) * Wm(i);
    x(3) = atan2(sum_sin, sum_cos);
end
end

                
                