function [X, P] = EKF_SLAM_Step_DataAssoc(X, P, vel, z, fov, dt, Q, R)
    % Task 3.3: EKF SLAM with Nearest Neighbor Data Association
    num_L = (length(X) - 3) / 2;
    
    %% 1. Prediction (Robot Motion Model)
    phi = X(3); v = vel(1); om = vel(2);
    
    % Update State
    X(1) = X(1) + dt * v * cos(phi + om*dt);
    X(2) = X(2) + dt * v * sin(phi + om*dt);
    X(3) = X(3) + dt * om;
    
    % Motion Jacobians
    G = eye(length(X));
    G(1:3, 1:3) = [1, 0, -dt*v*sin(phi + om*dt); 
                   0, 1,  dt*v*cos(phi + om*dt); 
                   0, 0,  1];
               
    W = [dt*cos(phi + om*dt), 0; dt*sin(phi + om*dt), 0; 0, dt];
    
    % Covariance Prediction
    Q_aug = zeros(length(X));
    Q_aug(1:3, 1:3) = W * Q * W';
    P = G * P * G' + Q_aug;

    %% 2. Data Association (Nearest Neighbor)
    % Find measurements currently in the sensor's Field of View
    active_meas_indices = find(fov); 
    
    for i = active_meas_indices
        zi = z(:, i); % Range and Bearing measurement
        
        % Step A: Project measurement to Global Coordinates
        rx = X(1); ry = X(2); rphi = X(3);
        m_x = rx + zi(1) * cos(rphi + zi(2));
        m_y = ry + zi(1) * sin(rphi + zi(2));
        
        % Step B: Search for the Nearest Neighbor in the Map
        best_dist = inf;
        best_l = 0;
        
        for l = 1:num_L
            idx = 3 + 2*l - 1;
            lx = X(idx); ly = X(idx+1);
            
            % Distance between sensed point and mapped landmark l
            dist = sqrt((m_x - lx)^2 + (m_y - ly)^2);
            
            if dist < best_dist
                best_dist = dist;
                best_l = l;
            end
        end
        
        %% --- VALIDATION GATE ---
        % If the nearest landmark is too far, it's likely a misidentification.
        % We set best_l to 0 to skip the update and prevent the "bonkers" jump.
        gate_threshold = 2.5; % Meters
        if best_dist > gate_threshold
            best_l = 0; 
        end
        
        %% 3. EKF Update (only if a valid match was found)
        if best_l > 0
            idx = 3 + 2*best_l - 1;
            dx = X(idx) - X(1);
            dy = X(idx+1) - X(2);
            d2 = dx^2 + dy^2;
            d = sqrt(d2);
            
            % Innovation
            z_hat = [d; atan2(dy, dx)];
            inn = zi - z_hat;
            inn(2) = atan2(sin(inn(2)), cos(inn(2))); % Angle Wrap
            
            % Measurement Jacobian H
            H = zeros(2, length(X));
            H(:, 1:3) = [-dx/d, -dy/d, 0; 
                          dy/d2, -dx/d2, -1];
            H(:, idx:idx+1) = [dx/d, dy/d; 
                              -dy/d2, dx/d2];
            
            % Kalman Gain and State/Covariance Update
            S = H * P * H' + R;
            K = P * H' / S;
            
            X = X + K * inn;
            P = (eye(length(X)) - K * H) * P;
        end
    end
end