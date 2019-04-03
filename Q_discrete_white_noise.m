function Qdwn = Q_discrete_white_noise(dim, dt, var, block_size, order_by_dim)
% Returns the Q matrix for the Discrete Constant White Noise Model.
% Q is computed as G*G^T*variance, where G is the process noise per time step. 

% Parameters:
% dim : dimension for Q, the final dimension is (dim x dim), may be either 2,3 or 4 
% dt : time step
% var : variance in the noise
% block_size : if state variable contains more than one dimension, such as
%              a 3d constant velocity model [x x' y y' z z']^T, then Q must 
%              be a block diagonal matrix.
% order_by_dim : defines ordering of variables in the state vector.
%               `True`   [x x' x'' y y' y'']  
%               `False`  [x y z x' y' z' x'' y'' z'']      
% sigma : the variance in the noise

if (dim ~= 2) && (dim ~= 3) && (dim ~= 4)
       error('dim must be between 2 and 4');
end

if dim == 2
        Q = [0.25*dt^4, 0.5*dt^3;
              0.5*dt^3,    dt^2];
elseif dim == 3;
        Q = [0.25*dt^4, 0.5*dt^3, 0.5*dt^2;
              0.5*dt^3,     dt^2,       dt;
              0.5*dt^2,       dt,       1];
elseif dim == 4
        Q = [(dt^6)/36, (dt^5)/12, (dt^4)/6, (dt^3)/6;
             (dt^5)/12, (dt^4)/4,  (dt^3)/2, (dt^2)/2;
             (dt^4)/6,  (dt^3)/2,   dt^2,     dt;
             (dt^3)/6,  (dt^2)/2 ,  dt,        1.0];
end

% var - multipy every element in matrix
% block size - multiply the size of the matrix 

if order_by_dim == 1
    Q1 = repmat({Q}, 1, block_size);
    Qdwn = blkdiag(Q1{:})*var;
else 
    error('order_by_dim must true, what can you do');
end

end