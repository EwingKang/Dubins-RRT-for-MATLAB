function plot_obstacle_poly(ax, poly, color)
shape_count = size(poly, 2);
for i = 1:shape_count
    crnt_poly = poly{i};
    poly_size = size(crnt_poly);  % the ith perticular segments
    
    % check dimension is 2
    if poly_size(2) ~= 2
        return;                 % error, incorrect polymer definition
    end

    % check line segement return to start
    if any(crnt_poly(poly_size(1), :) ~= crnt_poly(1, :))
        poly_size(1) = poly_size(1)+1;
        crnt_poly(poly_size(1), :) = crnt_poly(1, :);    
    end
    %seg_count = poly_size(1) - 1;
    if( nargin == 2 )
        plot(ax, crnt_poly(:,1), crnt_poly(:,2)); hold on;
    else
        plot(ax, crnt_poly(:,1), crnt_poly(:,2), color); hold on;
    end
end
end