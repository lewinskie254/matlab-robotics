function T = run_dubins_planner(data, v_a, R)
    num_pts = size(data, 1);
    waypoints_table = cell(num_pts-1, 6);
    
    figure('Name', 'Task 2.1 - Dubins Path'); hold on; grid on; axis equal;
    
    for i = 1:num_pts-1
        x_s = data(i,1); y_s = data(i,2); h_s = deg2rad(data(i,3));
        x_e = data(i+1,1); y_e = data(i+1,2); h_e = deg2rad(data(i+1,3));
        
        % Calculate Circle Centers
        C_ex = [x_s + R*sin(h_s), y_s - R*cos(h_s)];
        C_en = [x_e + R*sin(h_e), y_e - R*cos(h_e)];
        
        phi_ex = rad2deg(atan2(y_s - C_ex(2), x_s - C_ex(1)));
        phi_en = rad2deg(atan2(y_e - C_en(2), x_e - C_en(1)));
        
        waypoints_table{i,1} = sprintf('WP%d to WP%d', i-1, i);
        waypoints_table{i,2} = sprintf('(%.1f, %.1f)', C_ex(1), C_ex(2));
        waypoints_table{i,3} = phi_ex;
        waypoints_table{i,4} = phi_en;
        waypoints_table{i,5} = sprintf('(%.1f, %.1f)', x_s, y_s);
        waypoints_table{i,6} = sprintf('(%.1f, %.1f)', x_e, y_e);
        
        plot(x_s, y_s, 'ro', 'MarkerFaceColor', 'r');
        quiver(x_s, y_s, cos(h_s)*5, sin(h_s)*5, 'k', 'LineWidth', 1.5);
        line([x_s, x_e], [y_s, y_e], 'Color', 'b', 'LineStyle', '--');
    end
    T = cell2table(waypoints_table, 'VariableNames', ...
        {'Segment', 'Circle_Centres', 'Phi_ex_deg', 'Phi_en_deg', 'Exit_Loc', 'Entry_Loc'});
end