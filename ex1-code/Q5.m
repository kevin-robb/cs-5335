% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First seven joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)

function q = Q5(f1, f2, qInit, f1Target, f2Target)
    % f1 and f2 share joints 1:7.
    % thus q = [q1Init(1:9) q2Init(8:9)].
    % so our approach must include the same movements for joints 1:7.
    K = 100;
    stepsize = 0.05;
    q = qInit;
    for k = 1:K
        % fwd kin for both arms.
        pos1 = f1.fkine(q(1:9)).t;
        pos2 = f2.fkine([q(1:7), q(10:11)]).t;
        dx1 = f1Target-pos1;
        dx2 = f2Target-pos2;
        dq1 = stepsize * pinv(f1.jacob0(q(1:9), 'trans')) * dx1;
        dq2 = pinv(f2.jacob0([q(1:7), q(10:11)], 'trans')) * dx2;
        % convert back to single q representation. Ensure q(1:7) change
        % together, keeping the two arms aligned.
        dq1 = [dq1;0;0];
        dq2 = [dq2(1:7);0;0;dq2(8:9)];
        q = q + dq1' + dq2';
    end
end