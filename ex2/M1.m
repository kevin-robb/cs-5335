% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
    % the rand(m,n) function generates an m x n matrix of uniformly
    % distributed random values in the range (0,1). We can transform each
    % column to align with its corresponding joint angle range.
    range = q_max - q_min;
    qs = rand(num_samples, 4) .* range + q_min;
end


%     % Check if configuration is within joint limits / in collision
%     in_bounds = (all(q >= q_min) && all(q <= q_max));