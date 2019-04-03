classdef JulierSigmaPoints
% Generates sigma points and weights according to Simon J. Julier and 
% Jeffery K. Uhlmann's original paper[Julier, Simon J.; Uhlmann, Jeffrey 
% "A New Extension of the Kalman Filter to Nonlinear Systems". Proc. 
% SPIE 3068, Signal Processing, Sensor Fusion, and Target Recognition VI, 
% 182 (July 28, 1997)]. It parametizes the sigma points using kappa.

% Properties 
% n : Dimensionality of the state.
% kappa : Scaling factor that can reduce high order errors. 
% Wm : Weight for each sigma point for the mean.
% Wc : Weight for each sigma point for the covariance.
   properties
      n
      kappa
      Wm
      Wc
   end
   methods
       function obj = JulierSigmaPoints(n, kappa)
           % Class constructor
            obj.n = n;
            obj.kappa = kappa;
            obj = compute_weights(obj);
       end
       
       function ns = num_sigmas(obj)
            % Number of sigma points for each variable in the state x.
            ns = 2*obj.n + 1;
       end
       
       function obj = compute_weights(obj)
            % Computes the weights for the unscented Kalman filter.
            n = obj.n;
            k = obj.kappa;
            Wm = ones(2*n+1);
            Wm = Wm(1,:);
            obj.Wm = Wm*(0.5/(n+k));
            obj.Wm(1) = k/(n+k);
            obj.Wc = obj.Wm;
       end
       
       function sigmas = sigma_points(obj, x, P)
           if obj.n ~= size(x)
                error('expected size(x) %d, but size is %d',obj.n, size(x));
           end
           n = obj.n;
           n = length(x);
           n = n(1);
           if isscalar(P)
                P = eye(n) * P;
           end
           sigmas = zeros(2*n+1, n);
           
           % Returns lower triangular matrix.
           U = chol((n + obj.kappa)*P);
           
           % Gives you 5 sigma points, this amount is not mandatory, you
           % can have 3 or 7 or more, first is always the middle point and
           % the remaining must be an even number.
           sigmas(1,:) = x(1,:);
           for k = 1:n
               sigmas(k+1,:) = minus(x(1,:),-U(k,:));
               sigmas(n+k+1,:) = minus(x(1,:),U(k,:));
           end
           
       end
       
   end
end
















