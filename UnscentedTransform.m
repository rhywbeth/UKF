function [x, P] = UnscentedTransform(sigmas, Wm, Wc, noise_cov,mean_f, residual_f)

% Computes unscented transform of a set of sigma points and weights.
% returns the mean and covariance in a tuple.
% This works in conjunction with the UnscentedKalmanFilter class.

% Parameters 
% sigmas :  2D array of sigma points
% Wm : Weights for the mean.
% Wc : Weights for the covariance.
% noise_cov : Noise matrix added to the final computed covariance matrix.
% mean_f : mean of the provided sigma points and weights.
% residual_f : the residual (difference) between x and y
% x : Mean of the sigma points after passing through the transform.
% P : Covariance of the sigma points after passing throgh the transform.

[kmax, n] = size(sigmas);
P = eye(n);

if isempty(mean_f)
    % New mean of the sigmas and weights.
    for i = 1:n
        x(i) = dot(Wm, sigmas(:,i));
    end
else
    x = mean_fn(sigmas, Wm);
end

% New covariance is the sum of the outer product of the residuals and weights
if isempty(residual_f)
    for i = 1:kmax
        y(i,:) = sigmas(i,:) - x;
    end
    for i = 1:n
        dota(:,i) = (Wc.*y(:,i)')';
        % zsumowany iloczyn y i dota
        P(i,:) = sum(gmultiply(y,dota(:,i)));
    end
else
    P = zeros(n, n);
    for k = 1:kmax
        y = residual(sigmas(k), x);
        P = P + Wc(k) * outer(y, y);
    end
end

if ~isempty(noise_cov)
        P = P + noise_cov;
end

end