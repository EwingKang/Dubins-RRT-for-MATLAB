%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INTRODUCTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is the final phase of the RRT path planning project.
% This program can generates a RRT with dubins curve without interference
% with obstacles.
% Please refer to "RRT.m" for more information about RRT.
% See "dubins_curve.m" and "dubins_core" for more info about Dubin's curve.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ALGORITHM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The program is a modified RRT:
%   1. Generate a random 2D pose (x, y with an angle) as a vertex
%   2. Find the closest vertex (shortest dubins path) from the existing 
%      vertices list. 
%   3. Check for collision violation. Note the border of the map does not 
%      have collision detection
%   4. Append (add) the new vertex to the vertices list
%   5. Append (add) the newly generated dubins path in step 2 to the edges 
%      list
%   6. Repeat the process
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PROGRAM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% User Congerations: Please refer to the User Configuraion Area in the code
%                    section.
% Output:
%   <>Figure of the RRT_obstacles result
%   <>vertecies: An array of Vertecies' pose (coordinate and heading), 
%     sorted by the generated order. vertecies(1) is the original point
%   <>edges: List of generated dubins parameters, including starting pose  
%            (x, y, th) and dubins parameters
%   <>edges.param: the Dubins parameter connecting both points
%   <>ind_nearest(i): index of the nearest vertex in the vertices list.
%                     Vertecies(ind_nearest(i)) is the nearest pose to the 
%                     vertecies(i+1).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Notes %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ** The final segment count will most likely be less then the amount of
%    iterations because 
%    a) random edges are not be connected due to violation of collision.
%    b) it is possible to find any Dubins cruve.
% ** Please consider numercial errors see more info at <RRT.m> 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Ewing Kang
% Date: 2018.7.19
% contact: f039281310 [at] yahoo.com.tw
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2018 Ewing Kang                                           %
% Released under GPLv3 license                                            %
% Dubin's path generator is a MATLAB re-written from Andrew Walker's work,%
% which was originally distributed under MIT license in C language        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
main_fig_name = 'RRT with dubins curve and obstacle detection';
close(findobj('type','figure','name',main_fig_name));

%%%%%%%%%%%%%%%%%%%% User Configuration Area %%%%%%%%%%%%%%%%%%%%
% define the map
map.height = 20;
map.width = 20;
map.center = [0, 0];
map.offset = map.center - [map.width, map.height]./2; % bottom-left corner

% starting vertex
origin = [1,2, 20*pi/180];

% define the obstacles using polygones, should be a cell stack of arrays.
poly = { [-4,-4; -1.5, -4; 0, -2.5; -0.5, -1; -3, 0],...
         [0,3; 3,3; 3, 6; 4, 6; 1.5, 8; -1, 6; 0, 6] };

% RRT parameters
iteration = 1000;
th_range = 2*pi;    % do not change without knowledge about Dubins
th_center = 0;
th_offset = 0;

% dubins parameters
turning_rad = 0.5;
exp = 0.2;
%%%%%%%%%%%%%%%%%%%% End of configuration area %%%%%%%%%%%%%%%%%%%%
% main program starts here
poly = poly_verify(poly, map);
if ~iscell(poly)
    error("Ending due to incorrect obsticle polygon setting");
    return;
end

% prelocation of data
edges.x = zeros(iteration,2);
edges.y = zeros(iteration,2);
edges.th = zeros(iteration,2);
edges.param(iteration).p_init = [0, 0, 0];      % the initial configuration
edges.param(iteration).seg_param = [0, 0, 0];   % the lengths of the three segments
edges.param(iteration).r = turning_rad;         % model forward velocity / model angular velocity turning radius
edges.param(iteration).type = -1;               % path type. one of LSL, LSR, ... 
edges.param(iteration).flag = 0;

% initial conditions, DO NOT CHANGE
vertecies = origin;
vert_count = 1;
ind_nearest = zeros(iteration,1);
edge_count = 0;

% figure('name', 'RRT growing'); % originally for real-time animation
tic;
for i=1:iteration
    % random point generation
    x_rand = map.width*rand() + map.offset(1);
    y_rand = map.height*rand() + map.offset(2);
    th_rand = th_range*rand() + th_offset;
    
    % check if (x,y) is available or not
    if chk_xy_available([x_rand,y_rand], map, poly) == 0
        continue;
    end
        
  
    % connect to nearest point
    [ind_nearest(i),param_nearest] = dubins_searchn(vertecies, [x_rand, y_rand, th_rand], turning_rad);
    
    % check availablility, see dubins_core.m for more info
    if( param_nearest.flag < 0)
        continue;  % goto next loop, reset i doesn't work in MATLAB
    elseif( chk_dubins_collision(param_nearest, map, poly, exp)==0 )
        %%%%%%%%%%%%%%%%% edit line %%%%%%%%%%%%%%%%
        % append the newly generated point and edge to the existing list
        vertecies(vert_count+1,:) = [x_rand, y_rand, th_rand];
        vert_count = vert_count + 1;

        edges.x(edge_count+1,:) = [vertecies(ind_nearest(i),1), x_rand];
        edges.y(edge_count+1,:) = [vertecies(ind_nearest(i),2), y_rand];
        edges.th(edge_count+1,:) = [vertecies(ind_nearest(i),3), th_rand];
        edges.param(edge_count+1) = param_nearest;
        edge_count = edge_count + 1;
    end
end
toc;
clear i x_rand y_rand th_rand param_nearest 

figure('name', main_fig_name);
% plot map
boundary = [map.offset; map.offset+[map.width 0]; map.offset+[map.width, map.height]; map.offset+[0 map.height]; map.offset];
plot(boundary(:,1),boundary(:,2),'--r'); hold on;
% plot obstacles
plot_obstacle_poly(gca, poly);
% plot path
plot_RRT_Dubins(gca, vertecies, edges, vert_count);
% set size
xlim([map.offset(1)-1,map.offset(1)+map.width+1]);
ylim([map.offset(2)-1,map.offset(2)+map.height+1]);