% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    % instantiate with all obstacle cells = 1, and all other cells = 0.
    % i.e., start with cspace.
    distances = cspace;
    % find coords of the nearest configuration in grid to q_goal.
    i_goal = round(q_goal / (2*pi) * size(q_grid,2));
    % set value of goal cell.
    distances(i_goal) = 2;
    % make list of modifiers for choosing neighbors ahead of time.
    modifiers = [-1,-1,-1,0,0,1,1,1;-1,0,1,-1,1,-1,0,1];
    % expand out from the goal, computing distances.
    cells_to_expand = i_goal;
    while ~isempty(cells_to_expand)
        i_cur_cell = cells_to_expand(:,1);
        % add all adjacent unoccupied cells to the queue.
        for m = 1:size(modifiers,2)
            % use modulo to allow neighbors to wrap around the cspace.
            % do a weird hacky thing to prevent zeros messing us up.
            i_new_cell = mod(i_cur_cell + modifiers(:,m) - [1;1], size(q_grid,2)) + [1;1];
            % NOTE this makes the plot look different from the figure in
            % the assignment pdf, but wrapping should be possible so I'll
            % keep it this way. The plots match if I use a try/catch
            % instead to prevent wrapping.
            if distances(i_new_cell(1),i_new_cell(2)) == 0
                % increment distance and assign to neighboring free cells.
                incr_dist = 1;
%                 % if this cell is right next to occluded space, penalize
%                 % the distance further.
%                 for m = 1:size(modifiers,2)
%                     i_nbr = mod(i_new_cell + modifiers(:,m) - [1;1], size(q_grid,2)) + [1;1];
%                     if distances(i_nbr(1),i_nbr(2)) == 1
%                         incr_dist = incr_dist + 1;
%                         break
%                     end
%                 end
                distances(i_new_cell(1),i_new_cell(2)) = distances(i_cur_cell(1),i_cur_cell(2)) + incr_dist;
                % add these newly explored cells to the queue.
                cells_to_expand = [cells_to_expand, i_new_cell];
            end
        end
        % remove expanded cell from queue.
        cells_to_expand(:,1) = [];
    end
end