
% work for ex1 written Q3.
% a
T_10 = [1 0 0 0; 0 1 0 1; 0 0 1 1; 0 0 0 1];
T_20 = [1 0 0 -0.5; 0 1 0 1.5; 0 0 1 1; 0 0 0 1];
T_30 = [0 1 0 -0.5; 1 0 0 1.5; 0 0 -1 3; 0 0 0 1];
% b
T_23 = inv(T_30)*T_20;
% c
% new position of cube in camera frame.
P_23 = [0.2; -0.3; 2; 1];
% want position of cube in base frame.
P_20 = T_30 * P_23;
% d
% cube is moved to [-0.1, 0.1, 0]^T relative to {1}.
T_21 = [1 0 0 -0.1; 0 1 0 0.1; 0 0 1 0; 0 0 0 1];
% want pos of cube in camera frame.
T_23 = inv(T_30) * T_10 * T_21





