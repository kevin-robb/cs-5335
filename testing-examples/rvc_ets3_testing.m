%ch7 textbook examples
import ETS3.* %3d arm
L1 = 0; L2 = -0.2337; L3 = 0.4318; L4 = 0.0203; L5 = 0.0837; L6 = 0.4318;
E3 = Tz(L1) * Rz('q1') * Ry('q2') * Ty(L2) * Tz(L3) * Ry('q3') * Tx(L4) * Ty(L5) * Tz(L6) * Rz('q4') * Ry('q5') * Rz('q6');
E3.fkine([0 0 0 0 0 0]);
%E3.teach

% DH-params are introduced on page 197.
% create revolute joint and link
L = Revolute('a', 1);
L.A(0.5); %get homogenous transform of this link given theta.
L.type; %returns 'R' since link is revolute.
L.a; %returns DH param 'a'.
L.offset = 0.5; %offset is added to joint variable before computing trafo.
L.A(0);

% we can connect Link objects in series using the SerialLink class.
robot = SerialLink( [ Revolute('a', 1) Revolute('a', 1) ], 'name', 'my robot');
% this is the same as our demo ETS2 arm, but embedded in SE(3).
robot.fkine([30 40], 'deg');

% there are many arm models created with SerialLink. see them with:
%models
% we can edit params of any model with
%robot.edit















