clc; clear; close all;
load('my_input.mat'); load('my_measurements.mat');

% SET TASK: 2 = Single Landmark, 3 = Multi-Landmark
task_mode = 3; 

% 1. Initialization (Parameters from Eq 1.1 & 1.5)
dt = 0.1; 
x_est = [0.5; 1; 0]; % Robot initial coordinates
P_est = diag([0.1, 0.1, 0.01]);
Q = diag([0.007^2, 0.0075^2]); %
R = diag([0.02^2, 0.15^2]);    %

% 2. Pre-allocate Memory
num_steps = length(t);
state_hist = zeros(num_steps, 3);
inn_hist = nan(num_steps, 2); 
S_bounds = nan(num_steps, 2);

% 3. EKF Loop
for k = 1:num_steps
    % --- PREDICT ---
    [x_est, P_est] = EKF_Predict(x_est, P_est, v(k), om(k), dt, Q);
    
    % --- UPDATE ---
    if task_mode == 2
        % Question 2: Single landmark
        [x_est, P_est, inn, S] = EKF_Update(x_est, P_est, [r(k,1); b(k,1)], l(1,:), R);
        inn_hist(k,:) = inn'; S_bounds(k,:) = 3*sqrt(diag(S))';
    else
        % Question 3: Multi-landmark sequential fusion
        for i = 1:size(l,1)
            [x_est, P_est, inn, S] = EKF_Update(x_est, P_est, [r(k,i); b(k,i)], l(i,:), R);
            if i == 1 % Monitor Landmark 1 for innovation plots
                inn_hist(k,:) = inn'; S_bounds(k,:) = 3*sqrt(diag(S))';
            end
        end
    end
    state_hist(k,:) = x_est'; 
end

% 4. Visualizing Results for Report
figure('Name', 'Trajectory Map', 'Color', 'w'); hold on; grid on; axis equal;
plot(state_hist(:,1), state_hist(:,2), 'b', 'LineWidth', 1.5, 'DisplayName', 'EKF Path');
scatter(l(:,1), l(:,2), 100, 'r^', 'filled', 'DisplayName', 'Landmarks');
xlabel('X (m)'); ylabel('Y (m)'); 
title(['Task ', num2str(task_mode), ' Trajectory']); legend('Location', 'best');

% Innovation Analysis (Essential for Task 4 "Compare and Analyse")
figure('Name', 'Innovation Analysis', 'Color', 'w');
subplot(2,1,1); hold on; grid on;
plot(t, inn_hist(:,1), 'g'); plot(t, S_bounds(:,1), 'r--', t, -S_bounds(:,1), 'r--');
ylabel('Range Inn. (m)'); title('Innovation Analysis with 3\sigma Bounds');
subplot(2,1,2); hold on; grid on;
plot(t, rad2deg(inn_hist(:,2)), 'b'); plot(t, rad2deg(S_bounds(:,2)), 'r--', t, -rad2deg(S_bounds(:,2)), 'r--');
ylabel('Bearing Inn. (deg)'); xlabel('Time (s)');