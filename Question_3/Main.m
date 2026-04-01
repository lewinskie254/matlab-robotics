clc; clear all; close all; warning off;

% 1. Simulation Settings
rng(2025); 
fusion_flag = 0;
task_mode = 3.2;

% This script defines 'timesteps', 'dt', 'num_landmarks', 'P0', etc.
Initialisation_scenario; 

% Verify timesteps exists to prevent the error
if ~exist('timesteps', 'var')
    error('Variable "timesteps" was not defined in Initialisation_scenario.m');
end

% Initialise Augmented State [Robot; Landmarks] for Task 3.4
% State Vector Size: (3 + 2 * num_landmarks) x 1
[X1, P1] = EKF_SLAM_Initialize(initial_robot1, initial_landmarks, P0);
[X2, P2] = EKF_SLAM_Initialize(initial_robot2, initial_landmarks, P0);

% Initialise recording matrices
pos_robot1 = zeros(3, timesteps);
pos_robot2 = zeros(3, timesteps);
pos_landmark1 = zeros(2, num_landmarks, timesteps);
pos_landmark2 = zeros(2, num_landmarks, timesteps);

% Covariance recording for error analysis
cov_landmark1 = zeros(2, 2, num_landmarks, timesteps);
cov_landmark2 = zeros(2, 2, num_landmarks, timesteps);

% Record Initial States
pos_robot1(:,1) = X1(1:3);
pos_robot2(:,1) = X2(1:3);

% 2. Simulation Loop
for t = 2:timesteps    
    if task_mode == 3.4
        % --- Task 3.4: Perfect Range/Bearing ---
        [X1, P1] = EKF_SLAM_Step(X1, P1, vel_cmd1(:,t-1), ...
                                meas_landmark1(:,:,t), index_fov(1,:,t), ...
                                dt, Q, R);
        [X2, P2] = EKF_SLAM_Step(X2, P2, vel_cmd2(:,t-1), ...
                                meas_landmark2(:,:,t), index_fov(2,:,t), ...
                                dt, Q, R);

    elseif task_mode == 3.3
        % --- Task 3.3: Data Association ---
        [X1, P1] = EKF_SLAM_Step_DataAssoc(X1, P1, vel_cmd1(:,t-1), ...
                                          meas_landmark1(:,:,t), index_fov(1,:,t), ...
                                          dt, Q, R);
        [X2, P2] = EKF_SLAM_Step_DataAssoc(X2, P2, vel_cmd2(:,t-1), ...
                                          meas_landmark2(:,:,t), index_fov(2,:,t), ...
                                          dt, Q, R);

    else % Task 3.2
        % --- Task 3.2: Bearing-Only ---
        [X1, P1] = EKF_SLAM_Step_BearingOnly(X1, P1, vel_cmd1(:,t-1), ...
                                             meas_landmark1(:,:,t), index_fov(1,:,t), ...
                                             dt, Q, R(2,2));
        [X2, P2] = EKF_SLAM_Step_BearingOnly(X2, P2, vel_cmd2(:,t-1), ...
                                             meas_landmark2(:,:,t), index_fov(2,:,t), ...
                                             dt, Q, R(2,2));
    end


    % --- Save Data ---
    pos_robot1(:,t) = X1(1:3);
    pos_robot2(:,t) = X2(1:3);
    
    for l = 1:num_landmarks
        idx = 3 + 2*l - 1;
        pos_landmark1(:,l,t) = X1(idx:idx+1);
        pos_landmark2(:,l,t) = X2(idx:idx+1);
        cov_landmark1(:,:,l,t) = P1(idx:idx+1, idx:idx+1);
        cov_landmark2(:,:,l,t) = P2(idx:idx+1, idx:idx+1);
    end

    % Dummy particles for Plot_animation compatibility
    pos_particles1 = repmat(X1(1:2), 1, 100); 
    pos_particles2 = repmat(X2(1:2), 1, 100);
    
    Plot_animation;
    drawnow;
end

Plot_analysis;