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
            z_p = Measurement(particles(p).position,particles(p).landmarks(l).pos);
            residual = z(:,l) - z_p;

            % Calculate the Kalman gain
            H = [1 0;0 1]; %%alterado
            P = particles(p).landmarks(l).P;
            S = H*P*H' + R;
            K = P*H'/S;
            x__L= particles(p).landmarks(l).pos; %% adicionado

            % Update EKF %% muito mais rápido e tá sendo usado para poupar
            % tempo computacional. Se só usasse o PF, ao invés de uma
            % matriz 2x2, seria uma partícula num_part*num_part e isso pode
            % ser mto despendioso
            particles(p).landmarks(l).pos = x__L + K*(residual);
            particles(p).landmarks(l).P = ((eye(size(K*H)))-K*H)*P;

            % Update particle filter %
            expoente = -0.5*(residual')*inv(S)*residual;
            denominator = 2*pi*det(S);
            particles(p).w = particles(p).w*exp(expoente)/sqrt(denominator);
        end
    end
end

% Resample all particles based on their weights
if doResample
    particles = Resample(particles);
end



