clc; clear; close all;

% --- TASK 2.1: Dubins Path Planning ---
fprintf('Running Task 2.1: Dubins Path Planning...\n');
waypoints_dubins = [
    0,   10,   0;    % Initial Point
    60,  60,  45;    % Waypoint 1
    80, 120,  30;    % Waypoint 2
    150, 70, -90;    % Waypoint 3
    100, 30,-120;    % Waypoint 4
    50,   0,-180     % Waypoint 5
];
[T_dubins] = run_dubins_planner(waypoints_dubins, 5, 5);
disp(T_dubins);

% --- TASK 2.2 (Part A): Carrot-Chasing Simulation ---
fprintf('\nRunning Task 2.2: Carrot-Chasing Path Following...\n');
W_carrot = [ 150, 100; 350, 100; 450, 300; 300, 450; 100, 400; 200, 250 ];
run_carrot_chasing(W_carrot, 5, 10, 0.1);

% --- TASK 2.2 (Part B): RRT Path Planning ---
fprintf('\nRunning Task 2.2: RRT Navigation...\n');
% Define Obstacles from Assignment Map
obstacles{1} = [200,200; 300,500; 460,300];           
obstacles{2} = [310,750; 480,650; 500,550; 340,600];  
obstacles{3} = [630,300; 550,450; 750,350; 700,250];  
obstacles{4} = [600,700; 800,780; 750,600; 600,600];  

start_pos = [50, 50];    
goal_pos = [600, 550];   
run_rrt_planner(start_pos, goal_pos, obstacles);