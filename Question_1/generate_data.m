%% Updated Simulation to Match Assignment Figure
T = 0.1;
t = 0:T:10; 

% 1. Landmarks (Scattered around the area, similar to Figure 1)
% These positions cover the "box" the robot is driving in
l = [ 150, 100;   % Landmark 1
      350, 100;   % Landmark 2
      450, 300;   % Landmark 3
      300, 450;   % Landmark 4
      100, 400;   % Landmark 5
      200, 250 ]; % Landmark 6
numL = size(l, 1);

% 2. Inputs (Linear and Angular Velocity)
v = 5.0 + 0.007 * randn(size(t));  % Increased speed to match your 500m scale
om = 0.05 + 0.0075 * randn(size(t)); 
save('my_input.mat', 'v', 'om', 't');

% 3. Measurements (r and b)
% Note: In a real simulation, r and b should be calculated from 
% the Ground Truth distance to these 'l' coordinates.
r = zeros(length(t), numL);
b = zeros(length(t), numL);

for i = 1:numL
    % Simple noisy simulation
    r(:, i) = 150 + 20 * randn(length(t), 1); 
    b(:, i) = 0.1 + 0.15 * randn(length(t), 1);
end

save('my_measurements.mat', 'r', 'b', 'l');