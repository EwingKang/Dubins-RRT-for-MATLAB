%%%%% INTRODUCTION %%%%%
% Please see "RRT.m" for more information about RRT.
% This is the third phase of the RRT path planning project.
% In this program, in addition to a simple RRT, some "forbidden zone" is
% defined. This is to simulate "obstacles" when performing path planning.
% This program deomonstrate the ability to get a path plan around obstacles 
% in very short amount of time.
%
%%%%% ALGORITHM %%%%%
% The program is almost identical to the RRT:
%   1. generate a random point with an angle
%   2. find the closest vertex from the existing list.
%   3. connect the new vertex to the closest existing vertex IF the vertex
%      and edge doesn't intersect with prohibited zone.
%   4. append the newly generated vertex and edge to the known list
%
% The check of forbidden area is done by checking line intersection. For 
% more information, please see <chk_collision.m>
%
%%%%% PROGRAM %%%%%
% All the parameters are commented throughly and setup within the progrm, 
% please see main program below.
% Defines:
%   The rectangle area to grow RRT is the same, you may see "RRT.M".
%   The starting x-y coordinate.
%   The forbidden zone, defined by polygon coordinates
%
% Output:
%   <>Figure of the RRT_obstacles result
%   <>vertecies: An array of Vertecies' coordinates and radius(heading), 
%     sorted by the generated order
%   <>edges: starting and ending of each edges, note edges(i) corresponds
%     to vertecies(i+1), because vertecies(1) is the original point
%   <>edges.param: the Dubins parameter connecting both points
%   <>ind_nearest(i)(i): index to the nearest point. 
%     vertecies(ind_nearest(i)(i)(i)) is the nearest to the vertecies(i+1)
%
%%%% IMPORTANT %%%%%
% ***The actual segment count might be less then the iteration
% times. Sometimes, it is possible to have two points that is impossible to
% interconnect with Dubins. When this happened, that attempt will be
% discarded.
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
close(findobj('type','figure','name','RRT basic'));
close(findobj('type','figure','name','RRT growing'));

% define the map
height = 20;
width = 20;
center = [0, 0];

% starting vertex
origin = [1,0];
iterations = 1000;

% define the obstacles using polygones, should be a cell stack of arrays.
poly = { [-4,-4; -1.5, -4; 0, -2.5; -0.5, -1; -3, 0],...
         [0,3; 3,3; 3, 6; 4, 6; 1.5, 8; -1, 6; 0, 6] };
     
     
offset = center - [width, height]./2;
vertecies = origin;
vert_count = 1;
edges.x = zeros(iterations,2);
edges.y = zeros(iterations,2);
ind_nearest = zeros(iterations,1);
edge_count = 0;

% figure('name', 'RRT growing'); % originally for real-time animation
tic;
for i=1:iterations
    % random point generation
    x_rand = width*rand() + offset(1);
    y_rand = height*rand() + offset(2);
    
    % connect to nearest point
    ind_nearest(i) = dsearchn(vertecies, [x_rand, y_rand]);
    edge_rand = [x_rand, y_rand ; vertecies(ind_nearest(i),:)];
    
    % check availablility
    if chk_collision(edge_rand,poly) == 0
        
        % connect and add to list
        vertecies(vert_count+1,:) = [x_rand, y_rand];
        vert_count = vert_count + 1;
        edges.x(edge_count+1,:) = [vertecies(ind_nearest(i),1), x_rand];
        edges.y(edge_count+1,:) = [vertecies(ind_nearest(i),2), y_rand];
        edge_count = edge_count + 1;
    else
        %i = i - 1;         % doesn't work under MATLAB
    end
    
    % plot animation here is undoable probably due to MATLAB running
    % optimization...
    % scatter(x_rand, y_rand, 10,'filled'); hold on;
    % plot([vertecies(ind_nearest(i),1); x_rand], [vertecies(ind_nearest(i),2); y_rand]); hold on;
end
toc;
clear i x_rand y_rand edge_rand

figure('name', 'RRT basic');
scatter(origin(1), origin(2), 45, '*','r','LineWidth',1); hold on;
scatter(vertecies(:,1), vertecies(:,2), 10,linspace(1,10,length(vertecies(:,1))),'filled'); hold on;
plot(edges.x', edges.y'); hold on;

RRT_obstacles_plot_poly(gca, poly);