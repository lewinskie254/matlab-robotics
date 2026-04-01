global dt num_landmarks
global Q R Rm P0 num_particles

%% 1. Simulation parameters
dt = 0.5;
timesteps = 100;
time = (1:timesteps)*dt;

% Landmarks setup
block = 10;         % size of the road blocks
width = 3;          % width of roads
num_landmarks = 50; % number of landmarks
real_landmarks = -25+50*rand(2,num_landmarks); 

% Ensure landmarks are located outside the roads
for l = 1:num_landmarks     
    if mod(real_landmarks(1,l),block)<.5*width || mod(real_landmarks(1,l),block)>block-.5*width
        real_landmarks(1,l) = real_landmarks(1,l)+width;
    end
    if mod(real_landmarks(2,l),block)<.5*width || mod(real_landmarks(2,l),block)>block-.5*width
        real_landmarks(2,l) = real_landmarks(2,l)+width;
    end
end

%% 2. Robot Trajectory & Velocity (Fixed: Restored real_vel)
real_robot1 = zeros(3,timesteps);
real_robot2 = zeros(3,timesteps);
real_robot1(:,1) = [-20; 20; 0]; % x; y; psi
real_robot2(:,1) = [-10; 10; 0]; 

% Define the velocity profile
timeinterv = timesteps/4;
V = 30/(timeinterv*dt);                         % Linear velocity (m/s)
psi_dot_turn = -pi/2/(dt*5);                    % Angular velocity (rad/s)
real_vel = [V;0]*ones(1,timesteps);             % Default: moving straight

% Add turns to the trajectory
real_vel(:,  timeinterv-1:  timeinterv+3) = [V/1.3; psi_dot_turn]*ones(1,5);
real_vel(:,2*timeinterv-1:2*timeinterv+3) = [V/1.3; psi_dot_turn]*ones(1,5);
real_vel(:,3*timeinterv-1:3*timeinterv+3) = [V/1.3; psi_dot_turn]*ones(1,5);

%% 3. Noise Settings (Updated for Task 3.1)
% Process noise (velocity and heading rate)
process_variance = [0.1^2; 0.01^2]; 

% Task 3.1: Measurement noise for Range (2.0) and Bearing (0.04)
measurement_variance = [2; 0.04]; 

% Sensor range
max_read_distance = 10;

%% 4. Create Measurements & Noisy Commands
% Control input with process noise
vel_cmd1 = real_vel - sqrt(diag(process_variance))*randn(2,timesteps);
vel_cmd2 = real_vel - sqrt(diag(process_variance))*randn(2,timesteps);

% Initialize measurement and visibility arrays
meas_landmark1 = zeros(2,num_landmarks,timesteps);
meas_landmark2 = zeros(2,num_landmarks,timesteps);
index_seen = zeros(3,num_landmarks,timesteps); 
index_fov = zeros(3,num_landmarks,timesteps);  

% Generate the ground truth trajectory and simulated measurements
for t = 2:timesteps
    % Move the real robots
    real_robot1(:,t) = Propagation(real_robot1(:,t-1), real_vel(:,t-1), zeros(2,1));
    real_robot2(:,t) = Propagation(real_robot2(:,t-1), real_vel(:,t-1), zeros(2,1));
    
    for l = 1:num_landmarks
        % Check if landmark is within sensor range
        dist_to_L1 = sqrt(sum((real_robot1(1:2,t) - real_landmarks(:,l)).^2));
        if dist_to_L1 < max_read_distance
            % Store Range and Bearing with noise
            noise = [sqrt(measurement_variance(1)); sqrt(measurement_variance(2))].*randn(2,1);
            meas_landmark1(:,l,t) = Measurement(real_robot1(:,t), real_landmarks(:,l)) + noise;
            index_seen(1,l,t:end) = 1;
            index_fov(1,l,t) = 1;
        end
        
        dist_to_L2 = sqrt(sum((real_robot2(1:2,t) - real_landmarks(:,l)).^2));
        if dist_to_L2 < max_read_distance
            noise = [sqrt(measurement_variance(1)); sqrt(measurement_variance(2))].*randn(2,1);
            meas_landmark2(:,l,t) = Measurement(real_robot2(:,t), real_landmarks(:,l)) + noise;
            index_seen(2,l,t:end) = 1;
            index_fov(2,l,t) = 1;
        end
    end
end

% Compute shared visibility for Fusion
index_seen(3,:,:) = index_seen(1,:,:) .* index_seen(2,:,:);
index_fov(3,:,:) = index_fov(1,:,:) .* index_fov(2,:,:);

%% 5. Filter Parameters
Q = diag(process_variance); 
R = diag(measurement_variance); 
P0 = diag([1, 1]); 
num_particles = 100;

% Initial estimates (adding some initial error)
initial_landmarks = real_landmarks + [0.5;0.5].*randn(2,num_landmarks); 
initial_robot1 = real_robot1(:,1) + [0.5;0.5;0].*randn(3,1); 
initial_robot2 = real_robot2(:,1) + [0.5;0.5;0].*randn(3,1);