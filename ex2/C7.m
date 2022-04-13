% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = C7(cspace)
    % create a new configuration space by padding all obstacles by one grid
    % cell, including diagonal neighbors, in order to help reduce swept
    % volume collisions.
    padded_cspace = cspace;
    % list of modifiers for choosing neighbors.
    pad_dist = 1;
    modifiers = [];
    for r=-pad_dist:pad_dist
    for c=-pad_dist:pad_dist
        modifiers = [modifiers, [r;c]];
    end
    end
    
    for r = 1:size(cspace,1)
    for c = 1:size(cspace,2)
        % for each occluded cell, mark its neighbors as occluded too.
%         if cspace(r,c) == 1
            for m = 1:size(modifiers,2)
                % use modulo to allow path to wrap around the cspace.
                % do a weird hacky thing to prevent zeros messing us up.
                i_nbr = mod([r;c] + modifiers(:,m) - [1;1], size(cspace,2)) + [1;1];
                if cspace(i_nbr(1),i_nbr(2)) == 1
                    padded_cspace(r,c) = 1;
                    break
                end
            end
%         end
    end
    end
end