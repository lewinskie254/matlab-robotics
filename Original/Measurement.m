function meas = Measurement(pos_robot, pos_landmarks)
x = pos_robot(1);
y = pos_robot(2);

x_L = pos_landmarks(1,:);
y_L = pos_landmarks(2,:);

% Measurement equation
meas = [x_L-x;y_L-y];
%meas = pos_landmarks-pos_robot(1:2);
%meas=pos_landmarks(1,:)-...
end
