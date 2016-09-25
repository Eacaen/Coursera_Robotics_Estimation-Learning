function [ predictx, predicty ,  state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y
 
    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end
     %% Place parameters like covarainces, etc. here:
      dt = t - previous_t;
     R = eye(4)*0.00001;

          R = [dt*dt/4  0    dt/2 0 ;
               0    dt*dt/4  0  dt/2;
               dt/2     0    1   0;
               0     dt/2     0  1];
%            
     Q = eye(2)*0.01;
     

     A = [ 1 0 dt 0;
              0 1 0 dt;
              0 0 1 0;
              0 0 0 1];
     C = [ 1 0  0 0;
              0 1 0 0];
%               0 0 1 0;
%               0 0 0 1];
     I = eye(4);
     % Z= C * state
     %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
%     
    z_t = [x,y]';
    
    state = state' ;
    
    state = A * state;
    param.P = A * param.P * A' + R;
    K = param.P * C' * inv( C * param.P * C' + Q);
  %  Z= C * state;
    state = state + K * ( z_t - C * state);
    param.P = ( I - K * C) * param.P;
    
    state = state' ;
    
 
    % Predict 330ms into the future
    predictx = state(1) + state(3) * 0.33;
    predicty = state(2) + state(4) * 0.33;
    % State is a four dimensional element
  
end
