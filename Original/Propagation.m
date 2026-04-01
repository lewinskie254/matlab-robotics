function newpos = Propagation(pos_robot, vel_cmd, w)
global dt
% Position
x = pos_robot(1);
y = pos_robot(2);
psi = pos_robot(3);

% Velocity command
speed      = vel_cmd(1) + w(1);
psi_dot    = vel_cmd(2) + w(2);

% Propagation equation %% Alterado
newpos(1,1) = x + dt*(speed*cos(psi+psi_dot*dt)) ;
newpos(2,1) = y + dt*(speed*sin(psi+psi_dot*dt));
newpos(3,1) = psi + dt*psi_dot;

end
