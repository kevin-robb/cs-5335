% define constraints.
x_start = [2, 2]; x_goal = [4, 0];
T = 20; u = zeros(T,2);
% need to make matrices Aeq and Beq s.t. 
% Aeq * u = Beq for valid trajectories.
Aeq = zeros(1,T*2) + 1; % all ones.
Beq = [norm(x_goal - x_start)]; % overall displacement.

% make sure the initial traj is valid.
% u(1,:) = Beq;

% set additional options.
op = optimset('fmincon');
% op.algorithm = 'active-set';

% perform traj optimization.
fmincon(@cost_fn_a, u, [], [], Aeq, Beq) %[],[],[],op




% Cost function
% - x = Tx2 matrix of control commands.
function cost = cost_fn_a(x) %x0,xT,
    % T = total number of timesteps.
    T = size(x,1);
%     R = [1, 0; 0, 1];
    % compute the cost of this trajectory.
    cost = 0;
    for i = 1:T
        cost = cost + x(i,:) * x(i,:)';
    end
end