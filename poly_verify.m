function result = poly_verify(poly, map)
poly_count = size(poly, 2);
result = poly;
left_b = map.center(1) - map.width/2;
right_b = map.center(1) + map.width/2;
top_b = map.center(2) + map.height/2;
bottom_b = map.center(2) - map.height/2;
for i = 1:poly_count
    crnt_poly = poly{i};
    poly_size = size(crnt_poly);  % the ith perticular segments
    
    % polygon should consists of at least 3 points
    if poly_size(1) <= 2
        result = -1;
        error("Polygon should have at least 3 points");
        return;                 % error, incorrect polymer definition
    end
    
    % polygon should not exceed map border
    if any( (crnt_poly(:,1)<left_b) | (crnt_poly(:,1)>right_b) |...
            (crnt_poly(:,2)<bottom_b) | (crnt_poly(:,2)>top_b)       )
        result = -2;
        error("Polygon should not be outside of the map");
        return;                 % error, incorrect polymer definition
    end

    % polygon should be a closed loop
    % If not, add initial point as ending point to encircle the polygon
    if any(crnt_poly(poly_size(1), :) ~= crnt_poly(1, :))
        poly_size(1) = poly_size(1)+1;
        crnt_poly(poly_size(1), :) = crnt_poly(1, :);    
    end
    result{i} = crnt_poly;
end

end