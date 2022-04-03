% define constraints.
x_start = [2; 2]; x_goal = [4; 0];
T = 20; % # of timesteps.
u = zeros(2*T,1); % 2Tx1. format: [x1,x2,..,xT,y1,y2,...,yT].
% need to make matrices Aeq and Beq s.t. Aeq * u = Beq for valid trajectories.
Aeq = [ones(1,T), zeros(1,T);
       zeros(1,T), ones(1,T)]; % 2x2T.
Beq = x_goal - x_start; % 2x1. overall displacement.

% make shorthand for the cost function.
cost_fun = @(x) cost_fn_a(x);
% perform traj optimization.
[traj,traj_cost] = fmincon(cost_fun, u, [], [], Aeq, Beq);

% plot the traj.
traj_x = [x_start(1)]; traj_y = [x_start(2)];
for i = 1:T
    traj_x = [traj_x, traj_x(end)+traj(i)];
    traj_y = [traj_y, traj_y(end)+traj(i+T)];
%     traj_points = [traj_points; [traj(i), traj(i+T)]];
end
plot(traj_x, traj_y)
% show start and end
hold on
title("Optimized Trajectory")
plot(x_start(1),x_start(2),'r*')
text(x_start(1),x_start(2),'\leftarrow Start')
plot(x_goal(1),x_goal(2),'r*')
text(x_goal(1),x_goal(2),'\leftarrow Goal')


% Cost function
% - x = 2Tx1 matrix of control commands.
function cost = cost_fn_a(x)
    % T = total number of timesteps.
    T = round(size(x) / 2);
    % compute the cost of this trajectory.
    % cost = square of each component summed.
    cost = sum(x .* x);
end