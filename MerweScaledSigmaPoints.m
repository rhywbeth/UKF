classdef MerweScaledSigmaPoints
% Generates sigma points and weights according to Van der Merwe's 2004
% dissertation for the Unscented Kalman Filter. It parametrizes the sigma
% points using alpha, beta, kappa terms, and is the version seen in most
% publications.

% Properties 
% n : Dimensionality of the state.
% alpha : The spread of the sigma points around the mean. 
% beta : Incorporates prior knowledge of the distribution of the mean.
% kappa : Secondary scaling parameter that can reduce high order errors. 
% Wm : Weight for each sigma point for the mean.
% Wc : Weight for each sigma point for the covariance.
   properties
      n
      alpha
      beta
      kappa
      Wm
      Wc
   end
   methods
       function obj = MerweScaledSigmaPoints(n, alpha, beta, kappa)
           % Class constructor
            obj.n = n;
            obj.alpha = alpha;
            obj.beta = beta;
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
            lambda = obj.alpha^2*(n+k)-n;
            c = 0.5/(n+lambda);
            
            Wc = ones(2*n+1);
            Wm = ones(2*n+1);
            Wm = Wm(1,:);
            Wc = Wc(1,:);
            obj.Wc = Wc*c;
            obj.Wm = Wm*c;
            obj.Wc(1) = lambda/(n+lambda)+(1-obj.alpha^2+obj.beta);
            obj.Wm(1) = lambda/(n+lambda);
            
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
 
           lambda = obj.alpha^2*(n+obj.kappa)-n;
           U = sqrt((lambda + n)*P);

           sigmas = zeros(2*n+1, n);
           sigmas(1,:) = x(1,:);
           for k = 1:n
               sigmas(k+1,:) = minus(x(1,:),-U(k,:));
               sigmas(n+k+1,:) = minus(x(1,:),U(k,:));
           end
           
           
           
       end
       
   end
end
















