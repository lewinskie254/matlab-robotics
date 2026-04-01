function meas = Measurement(pos_robot, pos_landmarks)
    % Extracts robot position
    x = pos_robot(1);
    y = pos_robot(2);

    % Extracts landmark positions
    x_L = pos_landmarks(1,:);
    y_L = pos_landmarks(2,:);

    % Relative differences
    dx = x_L - x;
    dy = y_L - y;

    % Range and Bearing Measurement Equation
    range = sqrt(dx.^2 + dy.^2);
    bearing = atan2(dy, dx);
    
    % Return as a 2xN matrix
    meas = [range; bearing];
end