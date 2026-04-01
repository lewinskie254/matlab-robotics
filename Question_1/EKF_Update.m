function [x_new, P_new, inn, S] = EKF_Update(x_pred, P_pred, z, landmark, R)
    lx = landmark(1); ly = landmark(2);
    rx = x_pred(1); ry = x_pred(2);
    
    % Geometry (Eq 1.5)
    dx = lx - rx; dy = ly - ry;
    d2 = dx^2 + dy^2; d = sqrt(d2);
    
    % Expected measurement z_hat
    bearing_pred = atan2(dy, dx) - x_pred(3);
    z_hat = [d; atan2(sin(bearing_pred), cos(bearing_pred))];
    
    % Innovation (Residual)
    inn = z - z_hat;
    inn(2) = atan2(sin(inn(2)), cos(inn(2)));
    
    % Jacobian H (Simplified with d=0)
    H = [-dx/d, -dy/d, 0; 
          dy/d2, -dx/d2, -1];
    
    % EKF Update Equations
    S = H * P_pred * H' + R;
    K = (P_pred * H') / S;
    
    x_new = x_pred + K * inn;
    x_new(3) = atan2(sin(x_new(3)), cos(x_new(3)));
    P_new = (eye(3) - K * H) * P_pred;
end