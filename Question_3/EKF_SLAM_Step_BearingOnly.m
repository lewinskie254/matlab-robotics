function [X, P] = EKF_SLAM_Step_BearingOnly(X, P, vel, z, fov, dt, Q, R_bearing)
    num_L = (length(X) - 3) / 2;
    
    %% 1. Prediction (Same as 3.4)
    phi = X(3); v = vel(1); om = vel(2);
    X(1) = X(1) + dt * v * cos(phi + om*dt);
    X(2) = X(2) + dt * v * sin(phi + om*dt);
    X(3) = X(3) + dt * om;
    
    G = eye(length(X));
    G(1:3, 1:3) = [1, 0, -dt*v*sin(phi + om*dt); 0, 1, dt*v*cos(phi + om*dt); 0, 0, 1];
    W = [dt*cos(phi + om*dt), 0; dt*sin(phi + om*dt), 0; 0, dt];
    
    Q_aug = zeros(length(X));
    Q_aug(1:3, 1:3) = W * Q * W';
    P = G * P * G' + Q_aug;

    %% 2. Update (Bearing-Only)
    for l = 1:num_L
        if fov(l)
            idx = 3 + 2*l - 1;
            dx = X(idx) - X(1);
            dy = X(idx+1) - X(2);
            d2 = dx^2 + dy^2;
            
            % Predicted Measurement is ONLY the angle
            z_hat = atan2(dy, dx);
            
            % Innovation (Scalar)
            % Ensure z is just the bearing part: z(2, l)
            inn = z(2,l) - z_hat;
            inn = atan2(sin(inn), cos(inn)); % Angle Wrap
            
            % Jacobian H (1 row only)
            H = zeros(1, length(X));
            H(1, 1:3) = [dy/d2, -dx/d2, -1];      % Robot part
            H(1, idx:idx+1) = [-dy/d2, dx/d2];   % Landmark part
            
            % Kalman Update
            S = H * P * H' + R_bearing; % R_bearing is 0.04
            K = P * H' / S;
            
            X = X + K * inn;
            P = (eye(length(X)) - K * H) * P;
        end
    end
end