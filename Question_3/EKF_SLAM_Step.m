function [X, P] = EKF_SLAM_Step(X, P, vel, z, fov, dt, Q, R)
    num_L = (length(X) - 3) / 2;

    %% 1. Prediction (Robot Motion)
    phi = X(3);
    v = vel(1); 
    om = vel(2);

    

    % Update State (Task 2.1 Kinematics)
    X(1) = X(1) + dt * v * cos(phi + om*dt);
    X(2) = X(2) + dt * v * sin(phi + om*dt);
    X(3) = X(3) + dt * om;

    

    % Jacobian of motion w.r.t State (G)
    G = eye(length(X));
    G(1:3, 1:3) = [1, 0, -dt*v*sin(phi + om*dt);
                   0, 1,  dt*v*cos(phi + om*dt);
                   0, 0,  1];

               

    % Jacobian of motion w.r.t Control (W) - Maps 2x2 noise to 3x3 state
    W = [dt*cos(phi + om*dt), 0;
         dt*sin(phi + om*dt), 0;
         0,                  dt];

    

    % Process noise (Augmented)
    Q_aug = zeros(length(X));
    Q_aug(1:3, 1:3) = W * Q * W'; 

    

    % Update Covariance
    P = G * P * G' + Q_aug;



    %% 2. Update (Measurement Model)
    for l = 1:num_L
        if fov(l)
            idx = 3 + 2*l - 1;
            dx = X(idx) - X(1);
            dy = X(idx+1) - X(2);
            d2 = dx^2 + dy^2;
            d = sqrt(d2);

            

            % Predicted Measurement (Range, Bearing)
            z_hat = [d; atan2(dy, dx)];

            

            % Innovation with Angle Wrapping
            inn = z(:,l) - z_hat;
            inn(2) = atan2(sin(inn(2)), cos(inn(2)));

            

            % Jacobian H for Range/Bearing
            H = zeros(2, length(X));
            H(:, 1:3) = [-dx/d, -dy/d, 0; 
                          dy/d2, -dx/d2, -1];
            H(:, idx:idx+1) = [dx/d, dy/d; 
                              -dy/d2, dx/d2];


            % Kalman Gain and Update
            S = H * P * H' + R;
            K = P * H' / S;
            X = X + K * inn;
            P = (eye(length(X)) - K * H) * P;

        end
    end
end