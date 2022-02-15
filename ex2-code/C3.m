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
    % find coords of the nearest configuration in grid to goal.
    i_goal = round(q_goal / (2*pi) * size(q_grid,2));
    % set value of goal cell.
    distances(i_goal) = 2;
    % expand out from the goal, computing distances.
    cells_to_expand = i_goal;
    while ~isempty(cells_to_expand)
        i_cur_cell = cells_to_expand(:,1);
        % add all adjacent unoccupied cells to the queue.
        for r = -1:1
        for c = -1:1
            try % use a try-catch since the indices may go out of range.
            i_new_cell = i_cur_cell + [r;c];
%             if i_new_cell(1)
            if distances(i_new_cell(1),i_new_cell(2)) == 0
                % increment distance and assign to neighboring free cells.
                distances(i_new_cell(1),i_new_cell(2)) = distances(i_cur_cell(1),i_cur_cell(2)) + 1;
                % add these newly explored cells to the queue.
                cells_to_expand = [cells_to_expand, i_new_cell];
            end
            catch % don't do anything, just pass.
            end
        end
        end
        % remove expanded cell from queue.
        cells_to_expand(:,1) = [];
    end
end