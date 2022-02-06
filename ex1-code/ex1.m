% DO NOT MODIFY THIS FILE!
% input: questionNum -> Integer between 0 and 5 that denotes question
%                       number to run.
function ex1(questionNum)

    if nargin < 1
        error('Error: please enter a question number as a parameter');
    end

    % Set up robot and initial joint configuration
    [f1, f2] = createRobot();
    qn = [0 -pi/4 0 pi/2 0 pi/2 0];
    q1Init = [qn -1 1];
    q2Init = [qn 1 -1];
    spherePos = [0.65; 0.0; -0.75];

    % ========== Question 0 ==========
    if questionNum == 0
        close all;
        figure;
        q = Q0();
        f1.plot([q -1 1], 'jointdiam', 1);
        hold on;
        drawSphere(spherePos, 0.1);
    end
    
    % ========== Question 1 ==========
    if questionNum == 1
        % Draw arm and sphere (target)
        close all;
        figure;
        f1.plot(q1Init, 'jointdiam', 1);
        hold on;
        drawSphere(spherePos, 0.1);

        % TODO: Implement this function
        qTarget = Q1(f1, spherePos);

        % Interpolate and plot joint trajectory from q1Init to qTarget
        t = [0:0.05:2]';
        traj = jtraj(q1Init, qTarget, t);
        f1.plot(traj);
    end

    % ========== Question 2 ==========
    if questionNum == 2
        % Draw arm and sphere (target)
        close all;
        figure;
        f1.plot(q1Init,'jointdiam', 1);
        hold on;
        drawSphere(spherePos, 0.1);

        % TODO: Implement this function
        qTarget = Q2(f1, q1Init, spherePos);
        
        % Interpolate and plot joint trajectory from q1Init to qTarget
        t = [0:0.05:2]';
        traj = jtraj(q1Init, qTarget, t);
        f1.plot(traj);
    end

    % ========== Question 3 ==========
    if questionNum == 3
        % Draw arm and sphere (target)
        close all;
        figure;
        f1.plot(q1Init, 'jointdiam', 1);
        hold on;
        drawSphere(spherePos, 0.1);
        
        % TODO: Implement this function
        traj = Q3(f1, q1Init, spherePos, 0.05, 0.01);
        f1.plot(traj);
    end   
    
    % ========== Question 4 ==========
    if questionNum == 4
        % Calculate circle to trace out
        nTheta = 15;
        radius = 0.3;
        theta = linspace(0, 2*pi, nTheta);
        circle = repmat(spherePos, 1, nTheta) + radius * [cos(theta); sin(theta); zeros(1, nTheta)];

        % Get initial arm pose at the start of the circle
        qInit = Q1(f1, circle(:,1));

        % Draw circle and arm
        close all;
        figure;
        plot3(circle(1,:), circle(2,:), circle(3,:))
        hold on;
        f1.plot(qInit, 'jointdiam', 1);
        
        % TODO: Implement this function
        traj = Q4(f1, qInit, circle, 0.01);
        f1.plot(traj);
    end    
    
    % ========== Question 5 ==========
    if questionNum == 5
        % Draw arm, second finger (f2), and sphere
        close all;
        figure;
        f1.plot(q1Init, 'jointdiam', 1);
        hold on;
        f2.plot(q2Init, 'jointdiam', 1);
        drawSphere(spherePos, 0.1);

        % TODO: Implement this function
        qInit = [q1Init(1:9) q2Init(8:9)];
        
        qTarget = Q5(f1, f2, qInit, spherePos + [0; -0.1; -0.0], spherePos + [0; 0.1; 0.0]);

        % Interpolate and plot joint trajectory from qInit to qTarget
        t = [0:0.05:2]';
        traj = jtraj(qInit, qTarget, t);
        for q = traj'
            f1.plot(q(1:9)', 'jointdiam', 1);
            f2.plot([q(1:7)' q(10:11)'], 'jointdiam', 1);
        end
    end
end


% ========== Helper functions ==========

% Create two 7-DOF arms with 4 links (2 arm links and 2 finger links)
function [f1, f2] = createRobot()
    L(1) = Link([0 0 0 pi/2]);
    L(2) = Link([0 0 0 -pi/2]);
    L(3) = Link([0 0.4318 0 -pi/2]);
    L(4) = Link([0 0 0 1.571]);
    L(5) = Link([0 0.4318 0 pi/2]);
    L(6) = Link([0 0 0 -pi/2]);
    L(7) = Link([0 0 0 0]);
    L(8) = Link([0 0 0.2 0]);
    L(9) = Link([0 0 0.2 0]);

    f1 = SerialLink(L, 'name', 'f1');
    f2 = SerialLink(L, 'name', 'f2');
end

% Draw a sphere with specified center position and radius
function drawSphere(position,diameter)
    [X, Y, Z] = sphere;
    X = X * diameter;
    Y = Y * diameter;
    Z = Z * diameter;
    X = X + position(1);
    Y = Y + position(2);
    Z = Z + position(3);
    surf(X, Y, Z);
end