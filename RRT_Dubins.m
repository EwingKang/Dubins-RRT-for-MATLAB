%%%%% INTRODUCTION %%%%%
% RRT, the Rapidly-Exploring Random Trees is a ramdomized method of 
% exploring within dimensions. Please see "RRT.m" for more information about
% RRT.
% In this file, the paths connecting each points, or the "edges" to the
% "vertices" are replaced with Dubins curve. Please see more information
% about Dubins curve in "dubins_core.m"
% This is a very common practice when dealing with dynamic system. And even
% more specifically for non-holonomic system, such as cars, airplanes or
% ships. Replacements of Dubins curve means the branches growing out of 
% a vertex is always smooth in the sense tengency. This makes the path 
% generated more feassibly then the original straight line connection style
% if taking dynamics into consideration. 
%
%%%%% ALGORITHM %%%%%
% The basic method is the same with RRT. But instead of a simple random 
% 2-D coordinates, we now have to consider a third dimension -- the angle.
% 
% The method itself is almost identical to the 4-step loop of RRT:
%   1. generate a random point with an angle
%   2. find the closest vertex from the existing list. The "closest is
%      judged by length of dubins path, not euclidean distance
%   3. connect the new vertex to the closest existing vertex. The "edge"
%      now consists of Dubins parameter
%   4. append the newly generated vertex and edge to the known list
%
%%%%% PROGRAM %%%%%
% All the parameters are commented throughly and setup within the progrm, 
% please see main program below.
% The rectangle area to grow RRT is the same, you may see "RRT.M".
% The starting point will be a proper Dubins point, i.e. x-y coord. and an
% angle. You'll also have to define the turning radius of the Dubins path.
%
% Output:
%   <>Figure of the RRT_Dubins result
%   <>vertecies: An array of Vertecies' coordinates and radius(heading), 
%     sorted by the generated order
%   <>edges: starting and ending of each edges, note edges(i) corresponds
%     to vertecies(i+1), because vertecies(1) is the original point
%   <>edges.param: the Dubins parameter connecting both points
%   <>ind_nearest(i): index to the nearest point. 
%     vertecies(ind_nearest(i)(i)) is the nearest to the vertecies(i+1)
%
%%%%% IMPORTANT %%%%%
% ***The boundary will not limit the path from leaving it. I haven't implement
% the code to check so. To make sure you have everthing inside, you can set
% program boundary to be (actual_boundry - turning_radius*2).
% ***Note that the actual segment count might be less then the iteration
% times. Sometimes, it is possible to have two points that is impossible to
% interconnect with Dubins. When this happened, that attempt will be
% discarded. 
% ***Currently, this program is not very efficient especially in the 
% searching survey. There are many means of data structure method to
% improve the efficiency, maybe up to 5 time faster. However it costs a lot
% of effert and probably will make the code harder to read. Plus, it is
% somewhat unrealistic to optimize a MATLAB prototype code. Thus, the code
% is presented as it is.
% ***Please consider numercial errors see more info at <RRT.m> 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Ewing Kang
% Date: 2016.2.28
% contact: f039281310 [at] yahoo.com.tw
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2016, Ewing Kang                                                 % 
%                                                                                %
% Permission is hereby granted, free of charge, to any person obtaining a copy   %
% of this software and associated documentation files (the "Software"), to deal  %
% in the Software without restriction, including without limitation the rights   %
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      %
% copies of the Software, and to permit persons to whom the Software is          %  
% furnished to do so, subject to the following conditions:                       %
%                                                                                %
% The above copyright notice and this permission notice shall be included in     %
% all copies or substantial portions of the Software.                            %
%                                                                                %
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     %
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       %
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    %
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         %
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  %
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN      %
% THE SOFTWARE.                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close(findobj('type','figure','name','RRT w/ Dubins curve'));
close(findobj('type','figure','name','RRT growing'));
clear;

% Planning area initialization
height = 10;
width = 10;
center = [0, 0];
% Initial condition of [x, y, direction]
origin = [1,2, 20*pi/180];
turning_rad = 0.5;      % dubins turning raduis
% Define iterations count
iteration = 100;

offset = center - [width, height]./2;
th_range = 2*pi;    % do not change without knowledge about Dubins
th_center = 0;
th_offset = 0;

% prelocation of data
edges.x = zeros(iteration,2);
edges.y = zeros(iteration,2);
edges.th = zeros(iteration,2);
edges.param(iteration).p_init = [0, 0, 0];      % the initial configuration
edges.param(iteration).SEG_param = [0, 0, 0];   % the lengths of the three segments
edges.param(iteration).r = turning_rad;         % model forward velocity / model angular velocity turning radius
edges.param(iteration).type = -1;               % path type. one of LSL, LSR, ... 
edges.param(iteration).STATUS = 0;

vertecies = origin;
vert_count = 1;
ind_nearest = zeros(iteration,1);
edge_count = 0;

% figure('name', 'RRT growing'); % originally for real-time animation
tic;
for i=1:iteration
    % random point generation
    x_rand = width*rand() + offset(1);
    y_rand = height*rand() + offset(2);
    th_rand = th_range*rand() + th_offset;
    
    % connect to nearest point
    [ind_nearest(i),param_nearest] = dubins_searchn(vertecies, [x_rand, y_rand, th_rand], turning_rad);
    % edge_rand = [x_rand, y_rand, th_rand ; vertecies(ind_nearest(i),:)];
    
    % check availablility, see dubins_core.m for more info
    if( param_nearest.STATUS < 0)
        % goto next loop
        %i = i-1; %doesn't work under MATLAB
    else
        % append the newly generated point and edge to the existing list
        vertecies(vert_count+1,:) = [x_rand, y_rand, th_rand];
        vert_count = vert_count + 1;

        edges.x(edge_count+1,:) = [vertecies(ind_nearest(i),1), x_rand];
        edges.y(edge_count+1,:) = [vertecies(ind_nearest(i),2), y_rand];
        edges.th(edge_count+1,:) = [vertecies(ind_nearest(i),3), th_rand];
        edges.param(edge_count+1) = param_nearest;
        edge_count = edge_count + 1;
    end
    
    % plot animation here is undoable probably due to MATLAB running
    % optimization...
    % scatter(x_rand, y_rand, 10,'filled'); hold on;
    % plot([vertecies(ind_nearest(i),1); x_rand], [vertecies(ind_nearest(i),2); y_rand]); hold on;
end
toc;
clear i x_rand y_rand edge_rand th_rand param_nearest

figure('name', 'RRT w/ Dubins curve');
plot_RRT_Dubins(gca, vertecies, edges, vert_count);

% plot bounderies
boundary = [offset; offset+[width 0]; offset+[width height]; offset+[0 height]; offset];
plot(boundary(:,1),boundary(:,2),'--r');