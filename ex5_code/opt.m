% Code for OPT-a and OPT-b.
%%%%%%%%%%%%%%%%%%%%%%%% CONSTRAINTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_start = [2; 2]; x_goal = [4; 0];
T = 20; % # of timesteps.
u = zeros(2*T,1); % 2Tx1. format: [x1,x2,..,xT,y1,y2,...,yT].
% need to make matrices Aeq and Beq s.t. Aeq * u = Beq for valid trajectories.
Aeq = [ones(1,T), zeros(1,T);
       zeros(1,T), ones(1,T)]; % 2x2T.
Beq = x_goal - x_start; % 2x1. overall displacement.
% constraints for max of a single control.
% A * u < B. Have to check for pos and neg.
max_control = 0.5;
A = [eye(2*T); -eye(2*T)];
B = zeros(4*T,1) + max_control;

%%%%%%%%%%%%%%%%%%%% CHOOSE COST FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%
% cost_fun = @(x) cost_fn_a(x);
% cost_fun = @(x) cost_fn_dist_to_goal(x,x_start,x_goal);
cost_fun = @(x) cost_fn_inv_dist_to_mid(x,x_start,x_goal);
% cost_fun = @(x) cost_fn_dist_to_pt(x,x_start,x_goal);

%%%%%%%%%%%%%%% RUN TRAJECTORY OPTIMIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%
[traj,traj_cost] = fmincon(cost_fun, u, A, B, Aeq, Beq);

%%%%%%%%%%%%%%%%%%%%% PLOT TRAJECTORY %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
traj_x = zeros(T+1,1); traj_y = zeros(T+1,1);
traj_x(1) = x_start(1); traj_y(1) = x_start(2);
for i = 1:T
    traj_x(i+1) = traj_x(i) + traj(i);
    traj_y(i+1) = traj_y(i) + traj(i+T);
end
plot(traj_x, traj_y, '-o')
% show start and end
hold on
title(strcat("Optimized Trajectory. Cost=",num2str(traj_cost)))
plot(x_start(1),x_start(2),'r*')
text(x_start(1),x_start(2),'\leftarrow Start')
plot(x_goal(1),x_goal(2),'r*')
text(x_goal(1),x_goal(2),'\leftarrow Goal')


%%%%%%%%%%%%%%%%%%%%%% COST FUNCTION DEFNS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OPT-a sum of squares.
% @param x = 2Tx1 matrix of control commands.
function cost = cost_fn_a(x)
    % compute the cost of this trajectory.
    % cost = square of each component summed.
    cost = sum(x .* x);
end

% OPT-b dist to goal.
function cost = cost_fn_dist_to_goal(x,x0,xT)
    % T = total number of timesteps.
    T = round(size(x, 1) / 2);
    % compute the cost of this trajectory.
    % cost = dist to goal.
    cost = sum(x .* x); % always have sum of squares as base.
    x_t = x0; % track the position.
    for i = 1:T
        x_t = x_t + [x(i); x(i+T)];
        cost = cost + norm(xT - x_t);
    end
end

% OPT-b inv dist to midpt.
function cost = cost_fn_inv_dist_to_mid(x,x0,xT)
    % T = total number of timesteps.
    T = round(size(x, 1) / 2);
    midpt = (xT+x0)./2;
    % compute the cost of this trajectory.
    % cost = inv dist to midpt.
    cost = sum(x .* x); % always have sum of squares as base.
    x_t = x0;
    for i = 1:T
        x_t = x_t + [x(i); x(i+T)];
        cost = cost + 1/norm(midpt - x_t);
    end
end

% OPT-b dist to pt.
function cost = cost_fn_dist_to_pt(x,x0,xT)
    % T = total number of timesteps.
    T = round(size(x, 1) / 2);
    pt = [x0(1); xT(1)];
    % compute the cost of this trajectory.
    % cost = inv dist to midpt.
    cost = sum(x .* x); % always have sum of squares as base.
    x_t = x0;
    for i = 1:T
        x_t = x_t + [x(i); x(i+T)];
        cost = cost + norm(pt - x_t);
    end
end



