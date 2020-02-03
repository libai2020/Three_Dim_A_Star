%  MATLAB Source Codes for the book "Cooperative Decision and Planning for
%  Connected and Automated Vehicles" published by Mechanical Industry Press
%  in 2020.
% 《智能网联汽车协同决策与规划技术》书籍配套代码
%  Copyright (C) 2020 Bai Li
%  2020.01.31
% ==============================================================================
function obstacle_cell = GenerateDynamicObstacles(layer)
load DynObs
global xyt_graph_search_
Nobs = size(dynamic_obs,2);
obstacle_cell = cell(layer, Nobs);
for ii = 1 : Nobs
    dx = (dynamic_obs{end,ii}.x(1) - dynamic_obs{1,ii}.x(1)) / layer;
    dy = (dynamic_obs{end,ii}.y(1) - dynamic_obs{1,ii}.y(1)) / layer;
    for jj = 1 : xyt_graph_search_.num_nodes_t
        elem.x = dynamic_obs{1,ii}.x + dx * (jj - 1);
        elem.y = dynamic_obs{1,ii}.y + dy * (jj - 1);
        obstacle_cell{jj, ii} = elem;
    end
end
end