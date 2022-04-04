% Code for OPT-c, the light-dark domain.
%%%%%%%%%%%%%%%%%%%%%%%% CONSTRAINTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_start = [2; 2; 5]; x_goal = [4; 0; 0.01];
T = 20; % # of timesteps.
x = zeros(3*T,1); % 3Tx1. format: [x1,x2,..,xT,y1,y2,...,yT,v1,...,vT].
% need to make matrices Aeq and Beq s.t. Aeq * u = Beq for valid trajectories.
% i.e., start and end should match x_start, x_goal.
Aeq = zeros(6,3*T); % select out the points we want to check.
Aeq(1,1) = 1; Aeq(2,T+1) = 1; Aeq(3,2*T+1) = 1;
Aeq(4,T) = 1; Aeq(5,2*T) = 1; Aeq(6,3*T) = 1;
Beq = [x_start; x_goal]; % 6x1.
% ensure variance is positive and greater than epsilon.
variance_epsilon = 0.0001; % minimum allowable variance.
A = -eye(3*T); A = A(2*T+1:3*T, 1:3*T); % Tx3T.
B = zeros(T,1) - variance_epsilon; % Tx1.
% set nonlinear constraint function.
max_control = 0.5; % constraint for max of a single control.
process_variance = 0.01; % variance increase in prediction step.
nl_con_fn = @(x) nl_con_c(x, max_control, process_variance);

%%%%%%%%%%%%%%%%%%%% CHOOSE COST FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%
cost_fun = @(x) cost_fn_c(x);

%%%%%%%%%%%%%%% RUN TRAJECTORY OPTIMIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%
options=optimset('MaxIter',100000,'MaxFunEvals',100000);
[traj,traj_cost] = fmincon(cost_fun, x, A, B, Aeq, Beq, [], [], nl_con_fn, options);
% Aeq * traj
% [traj(1:T), traj(T+1:2*T), traj(2*T+1:3*T)]

%%%%%%%%%%%%%%%%%%%%% PLOT TRAJECTORY %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot(traj(1:T), traj(T+1:2*T), '-o', 'MarkerSize', 3)
% plot markers separately with size dependent on variance.
hold on
for i = 1:T
    plot(traj(i), traj(T+i), 'ro', 'MarkerSize', 100*traj(2*T+i))
end
% show start and end points.
% hold on
title(strcat("Optimized Trajectory. T=",num2str(T),". Cost=",num2str(traj_cost)))
plot(x_start(1),x_start(2),'r*')
text(x_start(1),x_start(2),'\leftarrow Start')
plot(x_goal(1),x_goal(2),'r*')
text(x_goal(1),x_goal(2),'\leftarrow Goal')


%%%%%%%%%%%%%%%%%%%%%% COST FUNCTION DEFN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OPT-c light-dark domain.
% @param x = 3Tx1 matrix of control commands + variances.
function cost = cost_fn_c(x)
    T = round(size(x,1) / 3);
    % compute the cost of this trajectory.
    % cost = square of each component summed.
    diffs = [x(2:T) - x(1:T-1); x(T+2:2*T) - x(T+1:2*T-1)];
    cost = sum(diffs .* diffs);
end

%%%%%%%%%%%%%%%%%% NONLINEAR CONSTRAINT FUNCTION %%%%%%%%%%%%%%%%%%%%%%%%%
% OPT-c light-dark domain.
% @param x = 3Tx1 matrix of positions + variances.
function [c,ceq] = nl_con_c(x, max_control, process_variance)
    % create cons s.t. c <= 0, ceq = 0.
    T = round(size(x,1) / 3);
    % each position must be no more than max_control from the prev.
    c = abs([x(2:T) - x(1:T-1); x(T+2:2*T) - x(T+1:2*T-1)]) - max_control;
    % variances update as:
    w = @(x_x) 1/2 * (5 - x_x)^2 + 0.1;
    v = @(x_x, v) ((v + process_variance) * w(x_x)) / ((v + process_variance) + w(x_x));
    ceq = zeros(T-1,1);
    for i = 1:T-1
        % ensure the variance at the next state matches the prediction.
        ceq(i) = v(x(i), x(2*T+i)) - x(2*T+i+1);
    end
end


