%  MATLAB Source Codes for the book "Cooperative Dedcision and Planning for
%  Connected and Automated Vehicles" published by Mechanical Industry Press
%  in 2020.
% 《智能网联汽车协同决策与规划技术》书籍配套代码
%  Copyright (C) 2020 Bai Li
%  2020.02.01
% ==============================================================================
%  第二章. 2.4.5小节. X-Y-T图与A星搜索算法实现一次性的轨迹决策、间接呈现结果
% ==============================================================================
%  备注：
%  1. 结果精度不高，且没有考虑运动学方程约束，直接将其用于泊车问题的可能性不大，
%     往往需要衔接优化方案.
%  2. Figure 2需要读者自行旋转后，才会看起来像立体图.
% ==============================================================================
close all
clc

% % 参数设置
global vehicle_geometrics_ % 车辆轮廓几何尺寸
vehicle_geometrics_.vehicle_wheelbase = 2.8;
vehicle_geometrics_.vehicle_front_hang = 0.96;
vehicle_geometrics_.vehicle_rear_hang = 0.929;
vehicle_geometrics_.vehicle_width = 1.942;
vehicle_geometrics_.vehicle_length = vehicle_geometrics_.vehicle_wheelbase + vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_rear_hang;
vehicle_geometrics_.radius = hypot(0.25 * vehicle_geometrics_.vehicle_length, 0.5 * vehicle_geometrics_.vehicle_width);
global environment_scale_ % 车辆所在环境范围
environment_scale_.environment_x_min = -20;
environment_scale_.environment_x_max = 20;
environment_scale_.environment_y_min = -20;
environment_scale_.environment_y_max = 20;
environment_scale_.x_scale = environment_scale_.environment_x_max - environment_scale_.environment_x_min;
environment_scale_.y_scale = environment_scale_.environment_y_max - environment_scale_.environment_y_min;
% % 用于X-Y-T图搜索的A星算法涉及的参数
global xyt_graph_search_
xyt_graph_search_.num_nodes_x = 200;
xyt_graph_search_.num_nodes_y = 200;
xyt_graph_search_.num_nodes_t = 210;
xyt_graph_search_.multiplier_H_for_A_star = 10.0;
xyt_graph_search_.weight_for_time = 3.0;
xyt_graph_search_.max_iter = 5000;
xyt_graph_search_.max_t = 40;
xyt_graph_search_.resolution_t = xyt_graph_search_.max_t / (xyt_graph_search_.num_nodes_t - 1);
xyt_graph_search_.resolution_x = environment_scale_.x_scale / (xyt_graph_search_.num_nodes_x - 1);
xyt_graph_search_.resolution_y = environment_scale_.y_scale / (xyt_graph_search_.num_nodes_y - 1);
% % 导入既定算例以及静止障碍物分布情况
global vehicle_TPBV_ obstacle_vertexes_ dynamic_obs
load TaskSetup.mat
dynamic_obs = GenerateDynamicObstacles(xyt_graph_search_.num_nodes_t);
% % X-Y-T图搜索
start_ind = ConvertConfigToIndex(vehicle_TPBV_.x0, vehicle_TPBV_.y0, 0);
goal_ind = ConvertConfigToIndex(vehicle_TPBV_.xtf, vehicle_TPBV_.ytf, xyt_graph_search_.max_t);
tic
[x, y, theta, fitness] = SearchTrajectoryInXYTGraph(start_ind, goal_ind);
disp(['CPU time elapsed for A* search: ',num2str(toc), ' sec.'])
% % 直观呈现结果
DemonstrateDynamicResult(x, y, theta);