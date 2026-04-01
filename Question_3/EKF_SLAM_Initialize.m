function [X, P] = EKF_SLAM_Initialize(initial_robot, initial_landmarks, P0)
    num_L = size(initial_landmarks, 2);
    
    % State: [x; y; theta; L1x; L1y; ... LNx; LNy]
    X = zeros(3 + 2*num_L, 1);
    X(1:3) = initial_robot;
    
    for l = 1:num_L
        X(3 + 2*l - 1 : 3 + 2*l) = initial_landmarks(:,l);
    end
    
    % Covariance Matrix
    P = zeros(3 + 2*num_L);
    P(1:3, 1:3) = diag([0.1, 0.1, 0.01]); % Robot uncertainty
    for l = 1:num_L
        idx = 3 + 2*l - 1;
        P(idx:idx+1, idx:idx+1) = P0; % Landmark uncertainty
    end
end