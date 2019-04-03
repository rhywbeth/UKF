classdef UnscentedKalmanFilter
% Implements the Scaled Unscented Kalman filter (UKF) as defined by Simon 
% Julier, using the formulation provided by Wan and Merle. This filter 
% scales the sigma points to avoid strong nonlinearities.

% Properties 
% x : State estimate vector
% P : Covariance estimate matrix
% x_prior : Prior (predicted) state estimate.
% P_prior : Prior (predicted) state covariance matrix.
% x_post : Posterior (updated) state estimate.
% P_post : Posterior (updated) state covariance matrix.
% Q : process noise matrix
% R : measurement noise matrix
% Wm : Weight for each sigma point for the mean.
% Wc : Weight for each sigma point for the covariance.
% K : Kalman gain
% y : innovation residual
% z : Last measurement used in update().
   properties
      x 
      P 
      x_prior 
      P_prior 
      x_post
      P_post
      Q 
      R
      d_dim_x 
      d_dim_z 
      points_fn 
      d_dt 
      d_num_sigmas 
      x_mean 
      z_mean 
      Wm
      Wc
      residual_x
      residual_z
      sigmas_f
      sigmas_h
      K
      y
      z
      S
      SI
   end
   methods
       function obj = UnscentedKalmanFilter(dim_x, dim_z, dt, points)%, sqrt_fn, x_mean_fn, z_mean_fn, residual_x, residual_z)
            % Class constructor
            % dim_x : Number of state variables for the filter.
            % dim_z : Number of of measurement inputs.
            % dt : Time between steps in seconds.
            obj.x = zeros(dim_x);
            obj.P = eye(dim_x);
            obj.x_prior = obj.x; 
            obj.P_prior = obj.P; 
            obj.Q = eye(dim_x);
            obj.R = eye(dim_z);
            obj.d_dim_x = dim_x;
            obj.d_dim_z = dim_z;
            obj.points_fn = points;
            obj.d_dt = dt;
            obj.d_num_sigmas = points.num_sigmas(); 
            x_mean_fn = [];
            z_mean_fn = [];
            obj.x_mean = x_mean_fn;
            obj.z_mean = z_mean_fn;
            
            % Weights for the means and covariances.
            obj.Wm = points.Wm;
            obj.Wc = points.Wc;
            
            % Sigma points transformed through f(x) and h(x)
            obj.sigmas_f = zeros(obj.d_num_sigmas, obj.d_dim_x);
            obj.sigmas_h = zeros(obj.d_num_sigmas, obj.d_dim_z);

            obj.K = zeros(dim_x, dim_z);   % Kalman gain
            obj.y = zeros(dim_z);          % residual
            obj.z = ([]*dim_z)';           % measurement
            obj.S = zeros(dim_z, dim_z);   % system uncertainty
            obj.SI = zeros(dim_z, dim_z);  % inverse system uncertainty

            % these will always be a copy of x,P after predict() is called
            obj.x_prior = obj.x; 
            obj.P_prior = obj.P; 

            % these will always be a copy of x,P after update() is called
            obj.x_post = obj.x; 
            obj.P_post = obj.P; 
       end
       
      function obj = predict(obj, dt)
          % Performs the predict step of the UKF. On return, obj.x and
          % obj.P contain the predicted state (x) and covariance (P).
          % Important: this MUST be called before update() is called for 
          % the first time.
          
          if dt == 0
             dt = obj.d_dt;
          end
          % Calculate sigma points for given mean and covariance
          obj = compute_process_sigmas(obj, dt);
          % and pass sigmas through the unscented transform to compute prior
          [obj.x, obj.P] = UnscentedTransform(obj.sigmas_f, obj.Wm, obj.Wc, obj.Q, obj.x_mean, obj.residual_x);
          % Save prior
          obj.x_prior = obj.x;
          obj.P_prior = obj.P;   
      end
      
      function obj = update(obj, z, R)
          % Update the UKF with the given measurements. On return, obj.x
          % and obj.P contain the new mean and covariance of the filter.  
          if z == 0
              obj.z = ([]*obj.d_dim_z)';
              obj.x_post = obj.x;
              obj.P_post = obj.P;
              return
          end

          if isempty(R)
            R = obj.R;
          elseif isscalar(R)
            R = eye(obj.d_dim_z)*R;
          end
          
        % Pass prior sigmas through h(x) to get measurement sigmas
        % sigmas_h = [];
        for s = 1:(size(obj.sigmas_h))
            obj.sigmas_h(s,:) = hx(obj.sigmas_f(s,:));
        end
        
        % mean and covariance of prediction passed through unscented transform
        [zp, obj.S] = UnscentedTransform(obj.sigmas_h, obj.Wm, obj.Wc, R, obj.z_mean, obj.residual_z);
        obj.SI = inv(obj.S);

        % compute cross variance of the state and the measurements
        Pxz = cross_variance(obj, obj.x, zp, obj.sigmas_f, obj.sigmas_h);
        
        obj.K = Pxz*obj.SI;          % Kalman gain
        obj.y = residual_x(z, zp);   % residual
        
        % update Gaussian state estimate (x, P)
        obj.x = obj.x + (obj.K*obj.y)';
        obj.P = obj.P - (obj.K*(obj.S*(obj.K)'));
        
        % save measurement and posterior state
        obj.z = z;
        obj.x_post = obj.x;
        obj.P_post = obj.P;
        
      end
      
      function obj = compute_process_sigmas(obj, dt) 
          % Calculate sigma points for given mean and covariance
          sigmas = obj.points_fn.sigma_points(obj.x, obj.P);
          for i = 1:(size(sigmas))
              obj.sigmas_f(i,:) = fx(sigmas(i,:), dt);
          end
      end
      
      function Pxz = cross_variance(obj, x, z, sigmas_f, sigmas_h)
          % Compute cross variance of the state `x` and measurement `z`
          Pxz = zeros(size(sigmas_f,2), size(sigmas_h,2));
          N = size(sigmas_f,1);
          for i = 1:N
              dx = residual_x(sigmas_f(i,:), x);
              dz = residual_x(sigmas_h(i,:), z);
              Pxz = Pxz + obj.Wc(i) * outer(dx, dz);
          end
      end
      
   end
end





