% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        circle -> 3xn matrix of Cartesian positions that describe the
%                  circle
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory. The end effector should trace a circle in the
%                 workspace, as specified by the input argument circle.
%                 (orientation is to be ignored)

function traj = Q4(f, qInit, circle, velocity)
    disp Q4_called
    circle
    traj = qInit;
    % we will use Q3 to get a trajectory to each 
    % successive point at the desired velocity.
    for k = 1:size(circle,2)
        k
        % current q will be last row of most recent trajectory.
        q = traj(end,:);
        % goal position will be next point on circle.
        next_pos = circle(:,k);
        traj = [traj; Q3(f,q, next_pos, 0.05, velocity)];
    end
    % result is concatenation of all these (const vel) trajectories
    % that trace out the given path (which need not even be a circle!).
end