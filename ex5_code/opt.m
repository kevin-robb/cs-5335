% define constraints.
x_start = [2; 2]; x_goal = [4; 0];
T = 20; 
% u = zeros(T,2); % Tx2
u = ones(2*T,1); % 2Tx1
% need to make matrices Aeq and Beq s.t. Aeq * u = Beq for valid trajectories.
% Aeq = zeros(1,T) + 1; % 1xT of all ones.
Aeq = [ones(1,T), zeros(1,T);
       zeros(1,T), ones(1,T)]; % 2x2T.
Beq = x_goal - x_start; % 1x2 of overall displacement.

% make sure the initial traj is valid.
% u(1,:) = Beq;

% % set additional options.
% op = optimset('fmincon');
% % op.algorithm = 'active-set';

% make shorthand for the cost function.
cost_fun = @(x) cost_fn_a(x);

% perform traj optimization.
fmincon(cost_fun, u, [], [], Aeq, Beq) %[],[],[],op




% Cost function
% - x = Tx2 matrix of control commands.
function cost = cost_fn_a(x)
    % T = total number of timesteps.
    T = round(size(x) / 2);
%     R = [1, 0; 0, 1];
    % compute the cost of this trajectory.
    % cost = square of each component summed.
    cost = sum(x .* x);
%     cost = 0;
%     for i = 1:T
%         x(i).^2
%         cost = cost + x(i).^2 + x(i+T).^2
%     end
end