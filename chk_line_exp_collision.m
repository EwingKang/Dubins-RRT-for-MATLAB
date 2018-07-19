%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Func: Check collision between polies and line with width expansion
%   Output: 1 if there's a collision; 0 if not
%   Input: line: 2 rows of coordinates [x1,y1 ; x2,y2]
%          poly: cell array with enclosed polygon of obstcles
%          exp: expansion distance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2018.07.19 Ewing Kang       %
% Distributed under GPLv3 license           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function collide = chk_line_exp_collision(line, poly, exp)
per_vec = line(2,:)-line(1,:);
per_vec(1, [1 2]) = per_vec(1, [2 1]);
per_vec(2) = -per_vec(2);
per_vec = per_vec./norm(per_vec);

left_line = line + repmat(per_vec.*exp, [2, 1]);
right_line = line - repmat(per_vec.*exp, [2, 1]);

% poly should be a cell array, cound how many polygon we need to check
shape_count = size(poly, 2);

for i = 1:shape_count
    crnt_poly = poly{i};
    seg_count = size(crnt_poly, 1)-1;  % segment is one count lesser then vertex
    
    if ( chk_line_collision(left_line, crnt_poly, seg_count) ||...
         chk_line_collision(right_line, crnt_poly, seg_count) ||...
         chk_line_collision(line, crnt_poly, seg_count)             )
            collide = 1;
            return;
    end
end
collide = 0;
return;
end

%{
    Func: Check collision between a line and a poly (line array)
          Returns 1 if there's collision; 0 if not
%}
function collide = chk_line_collision(line, crnt_poly, seg_count)

dA = line(2,:) - line(1,:);                                     % dimension should be [1, 2]    
dB = crnt_poly(2:seg_count+1, :) - crnt_poly(1:seg_count, :);       % dimension should be [seg_count, 2]
dA1B1 = repmat(line(1,:), [seg_count, 1]) - crnt_poly(1:seg_count, :);% dimension should be [seg_count, 2]
denominator = dB(:, 2) .* dA(1) - dB(:, 1) .*dA(2);
if all(denominator == 0)         % all lines are parrel, which is not possible
    collide = 0;
    return;
end
ua = dB(:, 1) .* dA1B1(:, 2) - dB(:, 2) .* dA1B1(:, 1);
ub = dA1B1(:, 2) .* dA(1) - dA1B1(:, 1) .* dA(2);
ua = ua ./ denominator;
ub = ub ./ denominator;
if ( all( ((ua<0)|(ua>1))|((0>ub)|(ub>1)) ) )
    collide = 0;     % no intersection
else
    collide = 1;
    return;
end

end