%ch7 examples
% create an instance of a Puma 560 arm.
mdl_puma560
% display its DH param table.
p560
% some premade configurations are created too:
% - qz = zero angle
% - qr = ready, the arm is straight and vertical
% - qs = stretch, the arm is straight and horizontal
% - qn = nominal, the arm is in a dextrous working pose
% plot the arm in its zero configuration
p560.plot(qz)
%p560.plot3d(qz) %fancier plot for some arm types

% forward kinematics are computed as before
TE = p560.fkine(qz);

% 'end effector' position is technically inside the wrist.
% we can define a tool transform from this to the actual tool tip.
p560.tool = SE3(0, 0, 0.2);
% pose of tool tip ('tool center point', TCP) is now
p560.fkine(qz);
% 'base' pos is technically inside shoulder.
% it actually has a 30-inch tall pedestal, so we can shift the origin down.
p560.base = SE3(0, 0, 30*0.0254);
% now with both base and TCP accounted for,
p560.fkine(qz);
% we can even do the following to make the arm hanging from the ceiling:
%p560.base = SE3(0,0,3) * SE3.Rx(pi);

% we can make joint angle time series (trajectories),
% where each row rep joint coords at diff timestep, and cols rep joints.
% discussed on page 204, using jtraj function.




