% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector describing goal position
%        epsilon -> scalar denoting how close end effector must be before
%                   loop terminates
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory.

function traj = Q3(f, qInit, posGoal, epsilon, velocity)
    traj = qInit;
    q = qInit;
    posCurrent = f.fkine(q).t;
    % run until satisfactorily close, instead of a set # of iterations.
    while norm(posGoal - posCurrent) > epsilon
        % fwd kin with current choice of joint angles.
        posCurrent = f.fkine(q).t;
        % same as Q2 but replace step size by velocity.
        dq = pinv(f.jacob0(q, 'trans')) * (posGoal - posCurrent);
        % ensure dq is normalized to length 1, then multiply by velocity.
        dq = velocity * (dq/norm(dq));
        q = q + dq';
        % append next move onto the trajectory.
        traj = [traj; q];
    end
end