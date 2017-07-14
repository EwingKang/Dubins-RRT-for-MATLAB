%%%%% INTRODUCTION %%%%%
% RRT, the Rapidly-Exploring Random Trees is a ramdomized method of 
% exploring within dimensions. This method can effectively generate a path
% to reach any point within certain limited steps due to its random
% characteristics. 
% This method is proprosed by LaValle, Steven M. in 
% October 1998, in his technical report to Computer Science Department of
% Iowa State University as "Rapidly-exploring random trees: A new tool for 
% path planning"
% Today, multiple variation of RRT method is widely applied with path
% planning problem among UAVs for ground based, aerial, and marinetime
% vehicles.
%
%%%% REFERENCES %%%%%
% <>Steven M. LaValle "Rapidly-Exploring Random Trees: A New Tool for Path 
% Planning" 1998, tech. rpt C.S.Dept, Iowa State U
% <>Sertac Karaman and Emilio Frazzoli "Sampling-based algorithms for 
% optimal motion planning",2011, The International Journal of Robotics Research
% <>Yoshiaki Kuwata, Gaston A. Fiore, Justin Teo, Emilio Frazzoli, and 
% Jonathan P. How, "Motion Planning for Urban Driving using RRT"
% <>https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
%
%%%%% ALGORITHM %%%%%
% For the sake of simplicity, I will discuss the algorithm only with 2-D
% planes. The problem is, given a starting point and limited boundre, how 
% do we reach everypoint within the area systematically?
% The method itself is very simple, only repeative iteration of 
% are 4 steps. 
%   1. generate a random point, i.e, a "vertex"
%   2. find the closest vertex from the existing list
%   3. connect the new vertex to the closest existing vertex, so called "edge"
%   4. append the newly generated vertex and edge to the known list
% As the iteration goes, it looks like a tree consists of edges is growing
% within the boundry and thus named so.
%
%%%%% PROGRAM %%%%%
% All the parameters are commented throughly and setup within the progrm, 
% please see main program below.
% Basically, you need to define a rectangle area for program to grow RRT.
% It is done by defining the height, width, and center of the area. Also,
% you need to define the starting point where the RRT starts to grow. Noted
% that this starting point is not limited to within the area, but the
% second generated vertex will bring the first edge right back inside the
% boundry anyway.
%
% Output: 
%   <>Figure of the RRT result
%   <>vertecies: An array of Vertecies' coordinates, 
%     sorted by the generated order
%   <>edges: starting and ending of each edges, note edges(i) corresponds
%     to vertecies(i+1), because vertecies(1) is the original point
%   <>ind_nearest: index to the nearest point. vertecies(ind_nearest(i))
%     is the closetest vertex to vertecies(i+1)
%
% NOTICE:
% please consider numercial error when defining boundries and area. There
% are still limitations with MATLAB. Generally I can garantee anything
% within order of plus minus 4.
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

% define the area and boundry
height = 10;
width = 10;
center = [0, 0];

% define the starting point and iterations
origin = [9,6];
iterations = 100;

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
    % edge_rand = [x_rand, y_rand ; vertecies(ind_nearest(i),:)];
    
    % check availablility
    
    % connect and add to list
    vertecies(vert_count+1,:) = [x_rand, y_rand];
    vert_count = vert_count + 1;
    edges.x(edge_count+1,:) = [vertecies(ind_nearest(i),1), x_rand];
    edges.y(edge_count+1,:) = [vertecies(ind_nearest(i),2), y_rand];
    edge_count = edge_count + 1;
    
    % plot animation here is undoable probably due to MATLAB running
    % optimization...
    % scatter(x_rand, y_rand, 10,'filled'); hold on;
    % plot([vertecies(ind_nearest,1); x_rand], [vertecies(ind_nearest,2); y_rand]); hold on;

end
toc;
clear i x_rand y_rand edge_rand

figure('name', 'RRT basic');
scatter(origin(1), origin(2), 45, '*','r','LineWidth',1); hold on;
scatter(vertecies(:,1), vertecies(:,2), 10,linspace(1,10,length(vertecies(:,1))),'filled'); hold on;
plot(edges.x', edges.y');
