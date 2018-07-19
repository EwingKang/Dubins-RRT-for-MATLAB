function result = chk_xy_available(point, map, poly)
% https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
% First, check no matching points
for i = 1:size(poly, 2)
    if (ismember(point, poly{i}, 'rows'))
        result = 0;
        return;
    end
end


% Create a straight line to the right edge of the map plus one
edge = [point(1), point(2) ; (map.center(1) + map.width/2)+1,point(2) ];
collide_count = chk_collision_count(edge,poly);

if(size(collide_count,2) > 1)
    % colinear, overlapping, check if point is on line or not
    left_pt = min([ poly{collide_count(2)}(collide_count(3),1), ...
                    poly{collide_count(2)}(collide_count(3)+1,1) ]);
    right_pt = max([ poly{collide_count(2)}(collide_count(3),1);
                     poly{collide_count(2)}(collide_count(3)+1,1) ]);
    if(left_pt<=point(1) && point(1)<=right_pt)
        result = 0; % point in between, not available
        return;
    end
end

if rem(collide_count,2) ~= 0
    result = 0;     % inside
    return;
end

% Create a straight line to the "left" edge of the map plus one
edge = [point(1), point(2) ; (map.center(1) - map.width/2)-1,point(2) ];
collide_count = chk_collision_count(edge,poly);
if rem(collide_count,2) ~= 0
    result = 0;     % inside
    return;
end

result = 1;
return;
end