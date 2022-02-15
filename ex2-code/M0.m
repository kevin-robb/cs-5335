% Output: q -> 1x4 vector of joint angles
%           - rotation of whole arm around base z axis
%           - joint 1 angle from straight up
%           - rotation around arm (not allowed to be != 0)
%           - joint 2 angle from 90deg

function q = M0()
    q = [0 0 0 0];
%     q = [0 -pi/4 0 -pi/4];
end