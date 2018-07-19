%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INFORMATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Func: This function is needed by <RRT_obstacles.m> to check if an edge 
%       collide with the obstacles polygon or not.
% Input
%   <>line: generated edge
%   <>poly: cell stack defining polygon edges. Each cell contains a n-by-2
%     array that defines the corner of the polygon
% Output
%   <>true/false: collide / or not
%
% Method
%   The examination is done by checking wheather the edge has intersection
%   with the polynomial or not.
%   Googling keywords: "line intersection check"
% References
%   https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-
%                                                   line-segments-intersect
%   http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Ewing Kang
% Date: 2016.2.28
% contact: f039281310 [at] yahoo.com.tw
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2016  Ewing Kang          %
% Released under GPLv3 license            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function result = chk_collision(line, poly)
% plotting
% plot_chk_collision(line, poly);

result = 0;

% poly should be a cell array, cound how many polygon we need to check
shape_count = size(poly, 2);

for i = 1:shape_count
    crnt_poly = poly{i};
    poly_size = size(crnt_poly);  % the ith perticular segments
    
    % polygon should consists of at least 3 points
    %{
    if poly_size(1) <= 2
        result = -2;
        return;                 % error, incorrect polymer definition
    end
    

    % check line segement return to initial point.
    % If not, add initial point as ending point to encircle the polygon
    if any(crnt_poly(poly_size(1), :) ~= crnt_poly(1, :))
        poly_size(1) = poly_size(1)+1;
        crnt_poly(poly_size(1), :) = crnt_poly(1, :);    
    end
    %}
    seg_count = poly_size(1) - 1;   % segment is one count lesser then vertex
    
    dA = line(2,:) - line(1,:);                                     % dimension should be [1, 2]    
    dB = crnt_poly(2:poly_size(1), :) - crnt_poly(1:poly_size(1)-1, :);       % dimension should be [seg_count, 2]
    dA1B1 = repmat(line(1,:), [seg_count, 1]) - crnt_poly(1:seg_count, :);% dimension should be [seg_count, 2]
    denominator = dB(:, 2) .* dA(1) - dB(:, 1) .*dA(2);
    if all(denominator == 0)         % all lines are parrel, which is not possible
        result = 0;
        return;
    end
    ua = dB(:, 1) .* dA1B1(:, 2) - dB(:, 2) .* dA1B1(:, 1);
    ub = dA1B1(:, 2) .* dA(1) - dA1B1(:, 1) .* dA(2);
    ua = ua ./ denominator;
    ub = ub ./ denominator;
    if ( all( ((ua<0)|(ua>1))|((0>ub)|(ub>1)) ) )
        result = 0;     % no intersection
    else
        result = 1;
        return;
    end
end

end
