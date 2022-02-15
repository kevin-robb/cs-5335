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
    cells_to_expand = [i_goal];
    while ~isempty(cells_to_expand)
        % add all adjacent unoccupied cells to the queue.
        for r = -1:1
        for c = -1:1
            if distances(cells_to_expand(1) + [r;c]) == 0
                % increment distance and assign to neighboring free cells.
                distances(cells_to_expand(1) + [r;c]) = distances(cells_to_expand(1)) + 1;
                % add these newly explored cells to the queue.
                cells_to_expand = [cells_to_expand, cells_to_expand(1) + [r;c]];
            end
            % remove expanded cell from queue.
            cells_to_expand(:,1) = [];
        end
        end
    end
end


% % % testing
% poly1 = polyshape([0 0 0.4 0.4],[1 0 0 1]);
% poly2 = polyshape([0.75 1.25 1.25 0.75],[0.25 0.25 0.75 0.75]);
% plot(poly1)
% hold on
% plot(poly2)
% polyout = intersect(poly1,poly2)
% polyout.Vertices