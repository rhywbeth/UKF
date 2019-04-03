function yy = outer(x,y)
% Calculates outer product 
m = length(x);
n = length(y);
yy = zeros(m,n);
for i = 1:m
    for j = 1:n
        yy(i,j) = x(i)*y(j);
    end
end
end