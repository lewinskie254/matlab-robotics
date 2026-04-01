function run_rrt_planner(start_pt, goal_pt, obs)
    x_lim = [0, 1000]; y_lim = [0, 900];
    step_size = 30; max_nodes = 3000; goal_thresh = 35;
    nodes = [start_pt, 0]; 
    
    fig = figure('Name', 'Task 2.2 - RRT'); 
    set(fig, 'Color', 'w'); % Force White
    hold on; grid on; axis([x_lim y_lim]);
    set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k');
    
    for k = 1:length(obs)
        fill(obs{k}(:,1), obs{k}(:,2), [0.7 0.7 0.7], 'FaceAlpha', 0.5); % Lighter gray for visibility
    end
    
    for i = 1:max_nodes
        q_rand = [rand*x_lim(2), rand*y_lim(2)];
        [~, idx] = min(sqrt((nodes(:,1)-q_rand(1)).^2 + (nodes(:,2)-q_rand(2)).^2));
        q_near = nodes(idx, 1:2);
        
        theta = atan2(q_rand(2)-q_near(2), q_rand(1)-q_near(1));
        q_new = q_near + [step_size*cos(theta), step_size*sin(theta)];
        
        % Collision checking along segment
        collision = false;
        for k = 1:length(obs)
            for f = 0:0.1:1
                check_pt = q_near + f * (q_new - q_near);
                if inpolygon(check_pt(1), check_pt(2), obs{k}(:,1), obs{k}(:,2))
                    collision = true; break;
                end
            end
            if collision, break; end
        end
        
        if ~collision
            nodes = [nodes; q_new, idx];
            line([q_near(1), q_new(1)], [q_near(2), q_new(2)], 'Color', [0.4 0.4 0.4]); % Darker gray lines
            if norm(q_new - goal_pt) < goal_thresh
                nodes = [nodes; goal_pt, size(nodes,1)];
                draw_final_path(nodes); 
                return;
            end
        end
        if mod(i,500)==0, drawnow; end
    end
end

function draw_final_path(nodes)
    curr = size(nodes, 1);
    while curr > 1
        parent = nodes(curr, 3);
        line([nodes(curr,1), nodes(parent,1)], [nodes(curr,2), nodes(parent,2)], 'Color', 'b', 'LineWidth', 2);
        curr = parent;
    end
end