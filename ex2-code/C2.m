% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)
    % instantiate the config space with the desired dimensions.
    cspace = q_grid + q_grid';
    for q1 in q_grid
        for q2 in q_grid
            % get polyshapes for arm in this configuration.
            [poly1, poly2, pivot1, pivot2] = q2poly(robot, [q1; q2]);
            % check for any collisions with this configuration.
            collision = intersect([poly1, poly2, obstacles]);
            % update the grid based on the result.
            if isempty(collision)
                cspace()=0; % TODO change loops to use indexes so I can access the right cell to update
            else
                cspace()=1;
            end
        end
    end
end