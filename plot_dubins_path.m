function plot_dubins_path(gca, param)

segments = 100;
path = dubins_path_draw(param, segments);
plot(gca, path(1,:), path(2,:)); hold on    
grid on;
axis equal;

end

function path = dubins_path_draw(param,segments)
    %%%%%%%%%%%%%%%%%%%%%%%%% DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % The three segment types a path can be made up of
    L_SEG = 1;
    S_SEG = 2;
    R_SEG = 3;

    % The segment types for each of the Path types
    DIRDATA = [ L_SEG, S_SEG, L_SEG ;...
                L_SEG, S_SEG, R_SEG ;...
                R_SEG, S_SEG, L_SEG ;...
                R_SEG, S_SEG, R_SEG ;...
                R_SEG, L_SEG, R_SEG ;...
                L_SEG, R_SEG, L_SEG ]; 
    %%%%%%%%%%%%%%%%%%%%%%%%% END DEFINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Generate the target configuration
    types = DIRDATA(param.type, :);
    param1 = param.seg_param(1);
    param2 = param.seg_param(2);
    param3 = param.seg_param(3);
    r = param.r;
    
    curve_cut = sum(param.seg_param(1:3))/segments;
    
    t1 = 0:curve_cut:param1;
    if( types(1) == L_SEG )
        seg1 = [r* sin(param.p_init(3)+t1) ; r* -cos(param.p_init(3)+t1) ];
        seg1 = [seg1(1,:) - r* sin(param.p_init(3)) + param.p_init(1) ;...
                seg1(2,:) + r* cos(param.p_init(3)) + param.p_init(2)      ];
    elseif( types(1) == R_SEG )
        seg1 = [r* sin(-param.p_init(3)+t1) ; r* cos(-param.p_init(3)+t1) ];
        seg1 = [seg1(1,:) - r* sin(-param.p_init(3)) + param.p_init(1) ;...
                seg1(2,:) - r* cos(-param.p_init(3)) + param.p_init(2)      ];
    end
    
    
    mid_pt1 = dubins_segment( param1, param.p_init, types(1), r);
    
    t2 = 0:curve_cut:param2;
    if( types(2) == S_SEG )
        seg2 = [r*t2* cos(mid_pt1(3)) ; r*t2*sin(mid_pt1(3))];
        seg2 = [seg2(1,:) + mid_pt1(1) ;...
                seg2(2,:) + mid_pt1(2)     ];
    elseif(types(2) == L_SEG)
        seg2 = [r* sin(mid_pt1(3)+t2) ; r* -cos(mid_pt1(3)+t2) ];
        seg2 = [seg2(1,:) - r* sin(mid_pt1(3)) + mid_pt1(1) ;...
                seg2(2,:) + r* cos(mid_pt1(3)) + mid_pt1(2)      ];
    elseif(types(2) == R_SEG)
        seg2 = [r* sin(-mid_pt1(3) + t2) ; r* cos(-mid_pt1(3) + t2) ];
        seg2 = [seg2(1,:) - r* sin(-mid_pt1(3)) + mid_pt1(1) ;...
                seg2(2,:) - r* cos(-mid_pt1(3)) + mid_pt1(2)      ];
    end
    
    mid_pt2 = dubins_segment( param2, mid_pt1,  types(2) , r);
    
    t3 =  0:curve_cut:param3;
    if( types(3) == L_SEG )
        seg3 = [r* sin(mid_pt2(3)+t3) ; r* -cos(mid_pt2(3)+t3) ];
        seg3 = [seg3(1,:) - r* sin(mid_pt2(3)) + mid_pt2(1) ;...
                seg3(2,:) + r* cos(mid_pt2(3)) + mid_pt2(2)      ];
    elseif( types(3) == R_SEG )
        seg3 = [r* sin(-mid_pt2(3)+t3) ; r* cos(-mid_pt2(3)+t3) ];
        seg3 = [seg3(1,:) - r* sin(-mid_pt2(3)) + mid_pt2(1) ;...
                seg3(2,:) - r* cos(-mid_pt2(3)) + mid_pt2(2)      ];
    end

        
    path = [seg1, mid_pt1(1:2)', seg2, mid_pt2(1:2)', seg3];
    

end

%{
 returns the parameter of certain location according to an inititalpoint,
 segment type, and its corresponding parameter
%}
function seg_end = dubins_segment(seg_param, seg_init, seg_type, r)
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