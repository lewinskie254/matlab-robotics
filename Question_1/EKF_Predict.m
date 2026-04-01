function [x_pred, P_pred] = EKF_Predict(x_est, P_est, v, om, dt, Q)
    phi = x_est(3);
    x_pred = x_est;
    
    % State Prediction (Eq 1.1)
    x_pred(1) = x_est(1) + dt * v * cos(phi);
    x_pred(2) = x_est(2) + dt * v * sin(phi);
    % Manual wrapToPi
    new_theta = phi + dt * om;
    x_pred(3) = atan2(sin(new_theta), cos(new_theta)); 
    
    % Jacobians (Eq 1.4)
    F = [1, 0, -dt*v*sin(phi); 
         0, 1,  dt*v*cos(phi); 
         0, 0,  1];
    Gamma = [dt*cos(phi), 0; 
             dt*sin(phi), 0; 
             0,           dt];
    
    % Covariance Prediction (Eq 1.3)
    P_pred = F * P_est * F' + Gamma * Q * Gamma';
end