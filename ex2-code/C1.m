% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to plot the robot at

function C1(robot, q)
    % use the q2poly function to get plotting parameters.
    [poly1, poly2, pivot1, pivot2] = q2poly(robot, q);
    % Plot the links.
    plot(poly1, 'FaceColor', 'r');
    plot(poly2, 'FaceColor', 'b');
    % Plot the pivot points.
    plot(pivot1(1), pivot1(2), 'k.', 'MarkerSize', 10);
    plot(pivot2(1), pivot2(2), 'k.', 'MarkerSize', 10);
end