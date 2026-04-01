function [particles]= SLAM_PF(particles,vel_cmd,z,index_fov)
global num_landmarks num_particles Q R
%% Predict
for p = 1:num_particles
    particles(p).position = Propagation(particles(p).position,vel_cmd,sqrt(Q)*randn(2,1));
end

%% Update
doResample = false;
for l = 1:num_landmarks
    if index_fov(l) % If within field of view
        doResample = true;        
        for p = 1:num_particles           
            % Predicted range/bearing based on current particle's estimate
            z_p = Measurement(particles(p).position, particles(p).landmarks(l).pos);
            
            % Calculate Residual
            residual = z(:,l) - z_p;
            
            % IMPORTANT: Angle Wrapping for Bearing
            residual(2) = atan2(sin(residual(2)), cos(residual(2)));

            % Calculate the Jacobian H for Range and Bearing
            dx = particles(p).landmarks(l).pos(1) - particles(p).position(1);
            dy = particles(p).landmarks(l).pos(2) - particles(p).position(2);
            d2 = dx^2 + dy^2;
            d = sqrt(d2);
            
            % H = dh/dx_L (Derivative of measurement w.r.t landmark position)
            H = [dx/d, dy/d; 
                -dy/d2, dx/d2]; 

            % EKF Update for the Landmark Map
            P = particles(p).landmarks(l).P;
            S = H*P*H' + R;
            K = P*H'/S;
            
            x__L = particles(p).landmarks(l).pos;
            particles(p).landmarks(l).pos = x__L + K*(residual);
            particles(p).landmarks(l).P = (eye(2) - K*H)*P;

            % Update particle weight (Likelihood)
            expoente = -0.5*(residual')*inv(S)*residual;
            denominator = 2*pi*det(S);
            particles(p).w = particles(p).w * exp(expoente)/sqrt(denominator);
        end
    end
end

% Resample all particles based on their weights
if doResample
    particles = Resample(particles);
end
end