% input: f -> an 9-joint robot encoded as a SerialLink class
%        position -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q1(f, position)
    % the position must be converted to a homogenous matrix, T.
    %T = transl(position(1),position(2),position(3));
    % since we care only about pos and not orientation, we will use a mask.
    %q = f.ikine(T,'m',[1 1 1 0 0 0]);
    
    % the above code works fine on its own, but gives errors when
    % the setup for Q4 runs in ex1. since I can't change that code, i'll
    % just use Q2 to solve this problem and avoid ikine errors later.
    qInit = [0 -pi/4 0 pi/2 0 pi/2 0 0 0]; %arbitrary initial pose
    q = Q2(f, qInit, position);
end