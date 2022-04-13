% input: f -> an 9-joint robot encoded as a SerialLink class
%        position -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q1(f, position)
    % the position must be converted to a homogenous matrix, T.
    T = transl(position(1),position(2),position(3));
    % since we care only about pos and not orientation, we will use a mask.
    q = f.ikine(T,'mask',[1 1 1 0 0 0]);
end