% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q2(f, qInit, posGoal)
    K = 100; %number of iteration steps
    stepsize = 0.05;
    q = qInit;
    for k = 1:K
        % fwd kin with current choice of joint angles (extract position).
        posCurrent = f.fkine(q).t;
        dx = posGoal - posCurrent;
        % only want part of jacobian about position.
        J = f.jacob0(q, 'trans');
        dq = stepsize * pinv(J) * dx;
        q = q + dq';
    end
end