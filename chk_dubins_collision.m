%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Func: Check collitions between a dubins curve and polygons
%  Out:  Returns 1 if there's a collision along the path; 0 if not
%  Input: param: dubins parameter (see dubins_core.m)
%         map: map struct
%         poly: cell array with enclosed polygon of obstcles
%         exp: expansion distance
%  Note:
%    Parameter struct list
%      param.p_init = p1;              % Initial point
%      param.seg_param = [0, 0, 0];    % lengths of three segments
%      param.r = r;                    % turning radius
%      param.type = -1;                % path type. one of LSL, LSR, ... 
%      param.flag = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2018.07.19 by Ewing Kang    %
% Distributed under GPLv3 license           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function collide = chk_dubins_collision(param, map, poly, exp)
    %%%%%%%%%%%%%%% DEFINE %%%%%%%%%%%%%%%
    % The three segment types a path can be made up of
    L_SEG = 1;
    S_SEG = 2;
    R_SEG = 3;
    % The segment types for each of the 6 dubin's path types
    DIRDATA = [ L_SEG, S_SEG, L_SEG ;...    % param.type = 1
                L_SEG, S_SEG, R_SEG ;...    % param.type = 2
                R_SEG, S_SEG, L_SEG ;...    % param.type = 3
                R_SEG, S_SEG, R_SEG ;...    % param.type = 4
                R_SEG, L_SEG, R_SEG ;...    % param.type = 5
                L_SEG, R_SEG, L_SEG ];      % param.type = 6
    %%%%%%%%%%%%% END DEFINE %%%%%%%%%%%%%
    last_end = param.p_init;
    arc.x = 0;
    arc.y = 0;
    arc.ang_init = 0;
    arc.ang_end = 0;
    arc.r = param.r;
    
    for seg_i = 1:3
        seg_type = DIRDATA(param.type, seg_i);
        this_end = dubins_get_xyt(param.seg_param(seg_i), last_end, seg_type, param.r);
        if(chk_map_range(this_end, map, exp) == 0)
            collide = 1;     % collide!
            return;
        end
        
        if(seg_type == L_SEG)
            arc.x = last_end(1) - param.r*sin( last_end(3) );
            arc.y = last_end(2) + param.r*cos( last_end(3) );
            arc.ang_init = last_end(3) - pi/2;
            arc.ang_end = last_end(3)+param.seg_param(seg_i) - pi/2;
            collide = chk_arc_poly_collision(arc, poly, exp);
        elseif(seg_type == R_SEG)
            arc.x = last_end(1) + param.r*sin( last_end(3) );
            arc.y = last_end(2) - param.r*cos( last_end(3) );
            arc.ang_init = (last_end(3)-param.seg_param(seg_i)) + pi/2;
            arc.ang_end = last_end(3) + pi/2;
            collide = chk_arc_poly_collision(arc, poly, exp);
        elseif(seg_type == S_SEG)
            collide = chk_line_exp_collision([last_end(1:2);this_end(1:2)], poly, exp);
        else
            error("error segment type");
            return;
        end
        
        if(collide)
            return;
        end
        last_end = this_end;    % next segment
    end
    collide = 0;    % no collide
    return;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Func: Returns the end point coordinates (x,y,theta) of a segment
%  Input: seg_param: Dubins param = normalized segment length
%         seg_init: initial points (x,y,theta), 
%         seg_type: segment type (L_SEG, S_SEG, R_SEG)
%         r: turning radius
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function seg_end = dubins_get_xyt(seg_param, seg_init, seg_type, r)
    L_SEG = 1;
    S_SEG = 2;
    R_SEG = 3;
    if( seg_type == L_SEG ) 
        seg_end(1) = seg_init(1) + r*( sin(seg_init(3)+seg_param) - sin(seg_init(3)) );
        seg_end(2) = seg_init(2) - r*( cos(seg_init(3)+seg_param) - cos(seg_init(3)) );
        seg_end(3) = seg_init(3) + seg_param;
    elseif( seg_type == R_SEG )
        seg_end(1) = seg_init(1) - r*( sin(seg_init(3)-seg_param) - sin(seg_init(3)) );
        seg_end(2) = seg_init(2) + r*( cos(seg_init(3)-seg_param) - cos(seg_init(3)) );
        seg_end(3) = seg_init(3) - seg_param;
    elseif( seg_type == S_SEG ) 
        seg_end(1) = seg_init(1) + cos(seg_init(3)) * seg_param * r;
        seg_end(2) = seg_init(2) + sin(seg_init(3)) * seg_param * r;
        seg_end(3) = seg_init(3);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Func: 1 if point is inside the map; 0 if not
%  Input: point: [x,y] coordinate
%         map: map struct
%         exp: expansion distance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function in_range = chk_map_range(point, map, exp)
if( ( point(1) < (map.center(1) - map.width/2 + exp) ) ||...
    ( point(1) > (map.center(1) + map.width/2 - exp) ) ||...
    ( point(2) < (map.center(2) - map.height/2 + exp) ) ||...
    ( point(2) > (map.center(2) + map.height/2 - exp) )      )
    in_range = 0;   %false
else
    in_range = 1;
end
return;
end