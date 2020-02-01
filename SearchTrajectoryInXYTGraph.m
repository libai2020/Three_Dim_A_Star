function [x, y, theta, fitness] = SearchTrajectoryInXYTGraph(start_ind, goal_ind)
global xyt_graph_search_ vehicle_TPBV_ environment_scale_
[ind_vec, fitness] = SearchViaAStar(start_ind, goal_ind);
x = environment_scale_.environment_x_min + (ind_vec(:,1)' - 1) .* xyt_graph_search_.resolution_x;
y = environment_scale_.environment_y_min + (ind_vec(:,2)' - 1) .* xyt_graph_search_.resolution_y;
dx = vehicle_TPBV_.x0 - x(1); dy = vehicle_TPBV_.y0 - y(1);
x = x + dx; y = y + dy;
x(end) = vehicle_TPBV_.xtf; y(1) = vehicle_TPBV_.y0;
theta = zeros(1,length(x));
theta(1) = vehicle_TPBV_.theta0;
for ii = 2 : length(x)
    dy = y(ii) - y(ii-1); dx = x(ii) - x(ii-1);
    theta(ii) = atan2(dy, dx);
    while (theta(ii) - theta(ii-1) > pi + 0.001)
        theta(ii) = theta(ii) - 2 * pi;
    end
    while (theta(ii) - theta(ii-1) < -pi - 0.001)
        theta(ii) = theta(ii) + 2 * pi;
    end
end
end

function [ind_vec, fitness] = SearchViaAStar(start_ind, goal_ind)
global xyt_graph_search_
grid_space_3D = cell(xyt_graph_search_.num_nodes_x, xyt_graph_search_.num_nodes_y, xyt_graph_search_.num_nodes_t);
% Information of each element in each node:
%  Dim # |  Variable
%  1-3      index of current node
%  4        f
%  5        g
%  6        h
%  7        is_in_openlist
%  8        is_in_closedlist
%  9-11     index of parent node
%  12-14    parent node's expansion vector
init_node(1:3) = start_ind;
init_node(5) = 0;
init_node(6) = sum(abs(start_ind - goal_ind));
init_node(4) = xyt_graph_search_.multiplier_H_for_A_star * init_node(6);
init_node(7) = 1;
init_node(8) = 0;
init_node(9:11) = [start_ind(1) start_ind(2) -999];
init_node(12:14) = [0 0 0];
openlist_ = init_node;
grid_space_3D{init_node(1), init_node(2), init_node(3)} = init_node;
cur_best_node = init_node;
expansion_pattern = [
    1  1  1;
    1  0  1;
    1 -1  1;
    0  1  1;
    0  0  1;
    0 -1  1;
    -1  1  1;
    -1  0  1;
    -1 -1  1
    ];
length_type_1 = sqrt(1^2 + 1^2 + xyt_graph_search_.weight_for_time^2);
length_type_2 = sqrt(1^2 + xyt_graph_search_.weight_for_time^2);
length_type_3 = abs(xyt_graph_search_.weight_for_time);
expansion_length = [
    length_type_1;
    length_type_2;
    length_type_1;
    length_type_2;
    length_type_3;
    length_type_2;
    length_type_1;
    length_type_2;
    length_type_1
    ];
completeness_flag = 0;

iter = 0;
while ((~isempty(openlist_))&&(iter <= xyt_graph_search_.max_iter)&&(~completeness_flag))
    iter = iter + 1;
    cur_node_order = find(openlist_(:,4) == min(openlist_(:,4))); cur_node_order = cur_node_order(end);
    cur_node = openlist_(cur_node_order, :);
    cur_ind = cur_node(1:3);
    cur_g = cur_node(5);
    cur_operation = cur_node(12:14);
    % Remove cur_node from open list and close it
    openlist_(cur_node_order, :) = [];
    grid_space_3D{cur_ind(1), cur_ind(2), cur_ind(3)}(7) = 0;
    grid_space_3D{cur_ind(1), cur_ind(2), cur_ind(3)}(8) = 1;
    for ii = 1 : size(expansion_pattern,1)
        child_node_ind = cur_ind + expansion_pattern(ii,:);
        if ((child_node_ind(1) < 1)||(child_node_ind(2) < 1)||...
                (child_node_ind(3) < 1)||(child_node_ind(1) > xyt_graph_search_.num_nodes_x)||...
                (child_node_ind(2) > xyt_graph_search_.num_nodes_y)||...
                (child_node_ind(3) > xyt_graph_search_.num_nodes_t))
            continue;
        end
        % If the child node has been explored ever before, and then if the
        % child has been within the closed list, abandon it and continue.
        if ((~isempty(grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)}))...
                &&(grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)}(8) == 1))
            continue;
        end
        child_g = cur_g + expansion_length(ii,1) + 0.25 * sum(abs(expansion_pattern(ii,1:2) - cur_operation(1:2)));
        child_h = sum(abs(child_node_ind - goal_ind));
        child_f = child_g + xyt_graph_search_.multiplier_H_for_A_star * child_h;
        child_node_prepare = [child_node_ind, child_f, child_g, child_h, 1, 0, cur_ind, expansion_pattern(ii,:)];
        % If the child node has been explored ever before (but not closed yet)
        if (~isempty(grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)}))
            % The child must be in the open list now, then check if its
            % recorded parent deserves to be switched as our cur_node.
            if (grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)}(5) > child_g + 0.01)
                child_node_order1 = find(openlist_(:,1) == child_node_ind(1));
                child_node_order2 = find(openlist_(child_node_order1,2) == child_node_ind(2));
                child_node_order3 = find(openlist_(child_node_order1(child_node_order2),3) == child_node_ind(3));
                openlist_(child_node_order1(child_node_order2(child_node_order3)), :) = [];
                grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)} = child_node_prepare;
                openlist_ = [openlist_; child_node_prepare];
            end
        else % Child node has never been explored before
            % If the child node is collison free
            if (IsNodeValid(child_node_ind, cur_ind))
                openlist_ = [openlist_; child_node_prepare];
                grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)} = child_node_prepare;
                if (~any(child_node_ind - goal_ind))
                    cur_best_node = child_node_ind;
                    fitness = child_g;
                    completeness_flag = 1;
                    break;
                end
                if (child_h < cur_best_node(6))
                    cur_best_node = child_node_prepare;
                end
            else % If the child node involves collisons
                child_node_prepare(8) = 1;
                child_node_prepare(7) = 0;
                grid_space_3D{child_node_ind(1), child_node_ind(2), child_node_ind(3)} = child_node_prepare;
            end
        end
    end
end
ind_vec = cur_best_node(1:3);
parent_ind = grid_space_3D{cur_best_node(1), cur_best_node(2), cur_best_node(3)}(9:11);
while (parent_ind(3) ~= -999)
    ind_vec = [parent_ind; ind_vec];
    parent_ind = grid_space_3D{parent_ind(1), parent_ind(2), parent_ind(3)}(9:11);
end
if (~completeness_flag)
    fitness = 10000000 + cur_best_node(6);
end
end

function is_valid = IsNodeValid(child_node_ind, cur_ind)
global xyt_graph_search_ environment_scale_ obstacle_vertexes_ dynamic_obs
x = environment_scale_.environment_x_min + xyt_graph_search_.resolution_x * (child_node_ind(1) - 1);
y = environment_scale_.environment_y_min + xyt_graph_search_.resolution_y * (child_node_ind(2) - 1);
dx = (child_node_ind(1) - cur_ind(1)) * xyt_graph_search_.resolution_x;
dy = (child_node_ind(2) - cur_ind(2)) * xyt_graph_search_.resolution_y;
theta = atan2(dy, dx);
is_valid = 0;
for ii = 1 : size(obstacle_vertexes_,2)
    if (IsVehicleCollidingWithMovingObstacle(x, y, theta, obstacle_vertexes_{1,ii}))
        return;
    end
end
for ii = 1 : size(dynamic_obs,2)
    if (IsVehicleCollidingWithMovingObstacle(x, y, theta, dynamic_obs{child_node_ind(3),ii}))
        return;
    end
end
is_valid = 1;
end

function is_collided = IsVehicleCollidingWithMovingObstacle(x, y, theta, V)
is_collided = 0;
if (min(hypot(V.x - x, V.y - y)) > 10)
    return;
end
Vcar = CreateVehiclePolygonFull(x, y, theta);
if (any(inpolygon(Vcar.x, Vcar.y, V.x, V.y)))
    is_collided = 1;
    return;
end
if (any(inpolygon(V.x, V.y, Vcar.x, Vcar.y)))
    is_collided = 1;
    return;
end
end

function Vcar = CreateVehiclePolygonFull(x, y, theta)
global vehicle_geometrics_
cos_theta = cos(theta);
sin_theta = sin(theta);
vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5;
AX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta - vehicle_half_width * sin_theta;
BX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta + vehicle_half_width * sin_theta;
CX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta;
DX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta;
AY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta + vehicle_half_width * cos_theta;
BY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta - vehicle_half_width * cos_theta;
CY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta;
DY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta;
Vcar.x = [AX, BX, CX, DX, AX];
Vcar.y = [AY, BY, CY, DY, AY];
end