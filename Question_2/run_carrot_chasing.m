function run_carrot_chasing(W, v_a, s, dt)
    num_wp = size(W, 1);
    state = [W(1,1); W(1,2); 0]; % [x; y; psi]
    path_history = state(1:2)';
    curr_wp_idx = 1;
    
    % Data for Cost Calculation
    total_sq_error = 0;
    count = 0;
    
    fig = figure('Name', 'Task 2.2 - Carrot Chasing'); 
    set(fig, 'Color', 'w'); % Force figure background white
    hold on; grid on; axis equal;
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k'); % Force axes white/black text
    
    plot(W(:,1), W(:,2), 'r^', 'MarkerFaceColor', 'r', 'DisplayName', 'Landmarks');
    plot(W(:,1), W(:,2), 'k--', 'DisplayName', 'Desired Path');
    h_ugv = plot(state(1), state(2), 'bo', 'MarkerFaceColor', 'b');
    
    for t = 0:dt:100
        if curr_wp_idx >= num_wp, break; end
        W_start = W(curr_wp_idx, :)'; W_end = W(curr_wp_idx+1, :)';
        
        path_vec = W_end - W_start;
        unit_path = path_vec / norm(path_vec);
        vec_to_ugv = state(1:2) - W_start;
        projection_dist = dot(vec_to_ugv, unit_path);
        
        % Carrot Point Calculation
        p_carrot = W_start + min(projection_dist + s, norm(path_vec)) * unit_path;
        
        % Cross-Track Error Calculation for Cost
        dist_to_path = abs(det([unit_path, vec_to_ugv])) / norm(unit_path);
        total_sq_error = total_sq_error + dist_to_path^2;
        count = count + 1;
        
        if projection_dist > norm(path_vec) - 2
            curr_wp_idx = curr_wp_idx + 1; continue;
        end
        
        % Control Law implementation
        psi_d = atan2(p_carrot(2) - state(2), p_carrot(1) - state(1));
        kp = 2.5; 
        angle_err = atan2(sin(psi_d - state(3)), cos(psi_d - state(3)));
        omega = kp * angle_err; 
        
        % Update State
        state(1) = state(1) + v_a * cos(state(3)) * dt;
        state(2) = state(2) + v_a * sin(state(3)) * dt;
        state(3) = state(3) + omega * dt; 
        
        path_history = [path_history; state(1:2)'];
        set(h_ugv, 'XData', state(1), 'YData', state(2));
        drawnow limitrate;
    end
    plot(path_history(:,1), path_history(:,2), 'b', 'LineWidth', 1.5);
    
    rmse = sqrt(total_sq_error / count);
    fprintf('Simulation Comparison Cost (RMSE): %.4f\n', rmse);
end