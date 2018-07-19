function int_count = chk_collision_count(line, poly)
% plotting
% plot_chk_collision(line, poly);
int_count = 0;

% poly should be a cell array, cound how many polygon we need to check
for i = 1:size(poly, 2)
    crnt_poly = poly{i};
    poly_size = size(crnt_poly);  % the ith perticular segments

    seg_count = poly_size(1) - 1;   % segment is one count lesser then vertex
    
    dA = line(2,:) - line(1,:);                                     % dimension should be [1, 2]    
    dB = crnt_poly(2:poly_size(1), :) - crnt_poly(1:poly_size(1)-1, :);       % dimension should be [seg_count, 2]
    dA1B1 = repmat(line(1,:), [seg_count, 1]) - crnt_poly(1:seg_count, :);% dimension should be [seg_count, 2]
    denominator = dB(:, 2) .* dA(1) - dB(:, 1) .*dA(2);  % dA cross dB
    
    parallel_ind = find(abs(denominator) < 0.000001);       % two lines are very close to parallel

    ua = dB(:, 1) .* dA1B1(:, 2) - dB(:, 2) .* dA1B1(:, 1); % dAB cross dB
    ub = dA1B1(:, 2) .* dA(1) - dA1B1(:, 1) .* dA(2);       % dAB cross dA
   
    for j = 1:size(parallel_ind,1)
        if(abs( ua(parallel_ind(j)) ) >0.000001)  
            % parallel, do nothing
        else                        % collinear
            if( (min(line(:,1)) <= crnt_poly(parallel_ind(j),1)) && (max(line(:,1)) >= crnt_poly(parallel_ind(j),1))...
                || (min(line(:,1)) <= crnt_poly(parallel_ind(j)+1,1)) && (max(line(:,1)) >= crnt_poly(parallel_ind(j)+1,1)))
                % overlapping segment
                int_count = [1000001, i, j];
                return;
            else % non-overlapping
            end
        end
        % make sure it won't count as having a intersection
        denominator(parallel_ind(j)) = min( [abs(ua(parallel_ind(j))),abs(ub(parallel_ind(j)))] )/2;
    end

    ua = ua ./ denominator;
    ub = ub ./ denominator;
    cond = ~( ((ua<0)|(ua>1))|((0>ub)|(ub>1)) );
    int_count = int_count + sum(cond);

end

end
