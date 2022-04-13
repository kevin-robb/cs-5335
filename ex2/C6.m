% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)
    num_collisions = 0;
    % check every pair of frames in the path for a collision.
    for i = 2:size(q_path,1)
        % get the space taken by the arm during this sweep.
        [poly11, poly12, ~, ~] = q2poly(robot, q_path(i-1,:)');
        [poly21, poly22, ~, ~] = q2poly(robot, q_path(i,:)');
        arm1_space = convhull(union(poly11,poly21)); %lower arm
        arm2_space = convhull(union(poly12,poly22)); %upper arm
        arm_space = union(arm1_space,arm2_space);
        % check for any collisions with this configuration.
        in_collision = false;
        % (poly1 and poly2 are allowed to collide with each other.)
        for i_ob = 1:size(obstacles,2)
            in_collision = ~isempty(intersect(arm_space, obstacles(i_ob)).Vertices);
            if in_collision
                % stop the loop if a collision is found.
                break
            end
        end
        if in_collision
            num_collisions = num_collisions + 1;
            % plot the links before and after the collision.
%             C1(robot,q_path(i-1,:)')
%             C1(robot,q_path(i,:)')
            % plot the violating swept volume.
            plot(arm1_space, 'FaceColor', 'r');
            plot(arm2_space, 'FaceColor', 'b');
        end
    end
end