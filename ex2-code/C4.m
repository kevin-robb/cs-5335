% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
    % find coords of the nearest configuration in grid to q_start.
    i_cur = round(q_start / (2*pi) * size(q_grid,2));
    path = i_cur';
    % make list of modifiers for choosing neighbors ahead of time.
    modifiers = [-1,-1,-1,0,0,1,1,1;-1,0,1,-1,1,-1,0,1];
    % loop through distances to generate greedy path to goal.
    while ~(distances(i_cur(1),i_cur(2)) == 2)
        % i.e., until our current cell is the goal.
        % check distance values of all neighboring cells.
        i_min_nbr = [100;100];
        min_dist = 10000;
        for m = 1:size(modifiers,2)
            % use modulo to allow path to wrap around the cspace.
            % do a weird hacky thing to prevent zeros messing us up.
%             i_nbr = i_cur + [r;c];
            i_nbr = mod(i_cur + modifiers(:,m) - [1;1], size(q_grid,2)) + [1;1];
            % minimize dist, and don't choose occluded neighbors.
            if (distances(i_nbr(1),i_nbr(2)) < min_dist) && (distances(i_nbr(1),i_nbr(2)) ~= 1)
                % found new best neighbor to go to.
                i_min_nbr = i_nbr;
                min_dist = distances(i_min_nbr(1),i_min_nbr(2));
            end
        end
        % add the best neighbor to the path and change our current cell to it.
        i_cur = i_min_nbr;
        path = [path; i_cur'];
    end
end
