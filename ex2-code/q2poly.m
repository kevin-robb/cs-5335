% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)
    % calculate positions of pivot points.
    pivot1 = robot.pivot1; % always the same.
    pivot2 = pivot1 + [robot.pivot2(1)*cos(q(1)); robot.pivot2(1)*sin(q(1))];
    % create transformation matrices for link params.
    link1 = robot.link1; link2 = robot.link2;
    R1 = rot2(q(1)); R2 = rot2(q(2));
    for p = 1:4
        link1(:,p) = R1 * link1(:,p);
        link2(:,p) = R2 * R1 * link2(:,p);
    end
    % shift link params by the pivot position.
    link1 = link1 + pivot1;
    link2 = link2 + pivot2;
    % create polyshapes based on link coords.
    poly1 = polyshape(link1(1,:),link1(2,:));
    poly2 = polyshape(link2(1,:),link2(2,:));
end