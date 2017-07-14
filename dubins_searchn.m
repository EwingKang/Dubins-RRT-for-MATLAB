%%%%% INFORMATION %%%%%
% This function is needed by <RRT_Dubins.m> to search the "nearest" vertex
% to the newly generated one in the sense of Dubins.
% The method is simply exhausive search, connect the vertex to everyone in
% the existing list. The length of Dubins can be calculated easily by its
% parameters. This shortest one is the one we want. 
% Noted that this is a very inefficient program by the exhausive search.I 
% use the term "cost" here because there are data structure/optimization 
% means to greatly improve the calculation speed. However, it costs a lot
% of effert and probably will make the code harder to read. Regarding a
% MATLAB prototype code I think it's more important to achieve
% functional-complete.
%
%%%%% PROGRAM %%%%%
% Input:
%   <>vertecies: target list of vertices
%   <>rand: initial vertex
%   <>rad turning raduis of the Dubins
%
% Output:
%   <>ind_clostest: index to the nearest vertex within the list
%   <>param_clostest: Dubins parameter from the nearest vertex to the
%     initial vertex
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
function [ind_clostest,param_clostest] = dubins_searchn(vertecies, rand, rad)
total = size(vertecies,1);

% find parameter for the first one in the list
param = dubins_core(vertecies(1, :), rand, rad);
cost = dubins_length(param);
param_clostest = param;
ind_clostest = 1;

for i = 2:total
    param = dubins_core(vertecies(i, :), rand, rad);
    
    % if the new cost is lower, record the lower one
    if(dubins_length(param) < cost)
        cost = dubins_length(param);
        ind_clostest = i;
        param_clostest = param;
    end
end

end

% calculate length of the Dubins
function length = dubins_length(param)
    length = param.SEG_param(1) + param.SEG_param(2) + param.SEG_param(3);
    length = length * param.r;
end