%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Func: Check collision between polies and a arc (segment of a circle)
%  Out:  Returns 1 if there's a collision; 0 if not
%  Input: arc: struct:[x, y, ang_init, ang_end, r]
%              (center xy, start-end angle of the arc, and arc radius)
%              arc from init to end is going counter-cloclwise
%         poly: cell array with enclosed polygon of obstcles
%         exp: expansion distance
%  Reference:
%    http://doswa.com/2009/07/13/circle-segment-intersectioncollision.html
%    https://codereview.stackexchange.com/questions/86421/line-segment-to-
%                                               circle-collision-algorithm
%    https://math.stackexchange.com/questions/1316803/algorithm-to-find-a-
%                           line-segment-is-passing-through-a-circle-or-not
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2018.07.19 by Ewing Kang    %
% Distributed under GPLv3 license           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function collide = chk_arc_poly_collision(arc ,poly, exp)
arc = normalize_arc(arc);

for i = 1:size(poly, 2)
    crnt_poly = poly{i};
    seg_count = size(crnt_poly,1)-1;  % how many segment there is
    vec_seg = crnt_poly(2:seg_count+1, :) - crnt_poly(1:seg_count, :); % R^[seg_count, 2]
    vec_center = repmat([arc.x, arc.y], [seg_count, 1]) - crnt_poly(1:seg_count, :);
    seg_len = row_vecnorm(vec_seg);
    
    % A-C dot A-B / |A-B|
    ext_ratio = vec_seg(:,1).*vec_center(:,1) + vec_seg(:,2).*vec_center(:,2);
    ext_ratio = ext_ratio ./ seg_len ./ seg_len;  % extension ratio to nearest

    % projection point is at the back of point A, set point to A
    back_ind = ext_ratio > 0;
    nearest_pt_ratio = ext_ratio .* back_ind; % so minimum will be zero
    back_ind = ~back_ind;
    % projection point is beyond B, set point to B
    forth_ind = ext_ratio < 1;
    nearest_pt_ratio = nearest_pt_ratio .* forth_ind;   % set matching to 0
    forth_ind = ~forth_ind;
    nearest_pt_ratio = nearest_pt_ratio + forth_ind;    % set 1 to those matching
    
    % nearest point on line seg
    nearest_xy = nearest_pt_ratio .* vec_seg + crnt_poly(1:seg_count,:);
    
    % find nearest distance to the arc center
    dist = row_vecnorm( repmat([arc.x, arc.y], [seg_count, 1]) - nearest_xy );
    
    % find those have nearest point within outer circle
    near_ind = find( dist < (arc.r+exp) );
    if(isempty(near_ind))
        % all segments is outside of the outer ring
        continue;
    end
    
    for j = 1:size(near_ind,1)
        crnt_ind = near_ind(j,1);
        if max( [norm(crnt_poly(crnt_ind,:)-[arc.x, arc.y]),...
                 norm(crnt_poly(crnt_ind+1,:)-[arc.x, arc.y])  ]) < (arc.r-exp)
            % if all points are within the inner limit
            continue;
        end
        % check outer ring
        if(check_arc_angle_overlap( arc.r+exp, arc, crnt_ind, ...
                                    dist, vec_seg, vec_center, ...
                                    seg_len, ext_ratio, crnt_poly ) )

            collide = 1;
            return;
        end
        % check inner ring
        if( dist(near_ind) < (arc.r-exp) )
            if(check_arc_angle_overlap( arc.r-exp, arc, crnt_ind, ...
                                        dist, vec_seg, vec_center, ...
                                        seg_len, ext_ratio, crnt_poly ) )
                collide = 1;
                return;
            end
        end
    end
end
collide = 0;     % no collide at all
return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Func: Check if the point of intersection of a line and a circle is 
%         within the arc range or not
%   Out: 1 if collision; 0 if not
%   Input: everything
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function collide = check_arc_angle_overlap(r, arc, crnt_ind, dist, ...
                                        vec_seg, vec_center, seg_len, ext_ratio, poly)
p = sqrt(r^2-dist(crnt_ind)^2) / seg_len(crnt_ind);
two_point_flag = 0;

%%%% find the intersection coordinate %%%%
if ( ext_ratio(crnt_ind) > 0 )
    pt_a = poly(crnt_ind, :) + (ext_ratio(crnt_ind) - p)*vec_seg(crnt_ind, :);
    if( ext_ratio(crnt_ind) + p < 1)    % the line intersects the circle at 2 point
        pt_b = poly(crnt_ind, :) + (ext_ratio(crnt_ind) + p)*vec_seg(crnt_ind, :);
        two_point_flag = 1;
    end
elseif ( ext_ratio(crnt_ind) < 0 )
    pt_a = poly(crnt_ind, :) + (ext_ratio(crnt_ind) + p)*vec_seg(crnt_ind, :);
else % ext_ratio(crnt_ind) == 0  inline and perpendicular
    center_leng = norm( vec_center(crnt_ind,:) );
    if(center_leng <= r)
        p = sqrt( r^2 - center_leng^2 );
        pt_a = poly(crnt_ind, :) + p*vec_seg(crnt_ind, :);
    else
        collide = 0; % perpendicular and too far
        return;
    end
end

%%%% check if angles overlap or not %%%
vec_a = pt_a - [arc.x, arc.y];
ang_a = atan2(vec_a(2), vec_a(1));      % polar angle from epicenter
if( ((arc.ang_init<ang_a)&&(ang_a<arc.ang_end)) ||...
    ((arc.ang_init< (ang_a+2*pi()))&&((ang_a+2*pi())<arc.ang_end))  )
    collide = 1;
    return;
end
if(two_point_flag)
    vec_b = pt_b - [arc.x, arc.y];
    ang_b = atan2(vec_b(2), vec_b(1));
    if( ((arc.ang_init<ang_b)&&(ang_b<arc.ang_end)) ||...
        ((arc.ang_init<(ang_b+2*pi()))&&((ang_b+2*pi())<arc.ang_end)))
        collide = 1;
        return;
    end
end
collide = 0;
return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Func: normalize angle so the end is greater than the begining
%   Input: arc struct with ang_init and ang_end field
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function arc = normalize_arc(arc)
while(arc.ang_init) < 0
    arc.ang_init = arc.ang_init+2*pi();
end
while(arc.ang_end < arc.ang_init)
    arc.ang_end = arc.ang_end+2*pi();
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Func: Calculate length of stacked row vectors
%   Out: N-by-1 array of length
%   Input: N-by-m row vectors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function n = row_vecnorm(x)
% Norm of a row vector
    n = sqrt(sum(x.^2, 2));
end