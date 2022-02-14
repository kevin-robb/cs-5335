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
    % check each cell for collision and assign it 0 or 1.
    for i1 = 1:size(q_grid,2)
        for i2 = 1:size(q_grid,2)
            % get config params.
            q1 = q_grid(i1); q2 = q_grid(i2);
            % get polyshapes for arm in this configuration.
            [poly1, poly2, ~, ~] = q2poly(robot, [q1; q2]);
            % check for any collisions with this configuration.
            in_collision = false;
            % (poly1 and poly2 are allowed to collide with each other.)
            for i_ob = 1:size(obstacles,2)
                if in_collision
                    break
                end
                obs_poly = obstacles(i_ob);
                col1 = isempty(intersect(poly1, obs_poly).Vertices);
                col2 = isempty(intersect(poly2, obs_poly).Vertices);
                in_collision = in_collision || col1 || col2;
            end
            % update the grid based on the result.
            if in_collision
                cspace(i1,i2)=1;
            else
                cspace(i1,i2)=0;
            end
        end
    end
end




% % testing
% poly1 = polyshape([0 0 0.4 0.4],[1 0 0 1]);
% poly2 = polyshape([0.75 1.25 1.25 0.75],[0.25 0.25 0.75 0.75]);
% plot(poly1)
% hold on
% plot(poly2)
% polyout = intersect(poly1,poly2)
% polyout.Vertices

