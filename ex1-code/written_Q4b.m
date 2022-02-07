% question 4-c on the written part of ex1.
syms th1 th2 th3 L1 L2
% create the arm in 4-b.
% L1 = 1; L2 = 1;
% J1 = Revolute('d', L1, 'a', 0, 'alpha', pi/2);
% J2 = Revolute('d', 0, 'a', L2, 'alpha', pi/2);
% T_23 = [0 0 1 0; 1 0 0 0; 0 1 0 0; 0 0 0 1];
% J3 = Prismatic('a', 0, 'alpha', 0, 'theta', pi/2, 'qlim', 5);
% arm_b = SerialLink([J1 J2 J3], 'name', 'arm_b');
% arm_b.teach([-pi/4 pi/4 1])
%arm_b.plot([-pi/4 pi/4 2])
%fk = arm_a.fkine([th1 th2 th3]);
%simplify(fk);


% my result from doing it manually:
t1=[1,0,0,th3;0,1,0,0;0,0,1,0;0,0,0,1];
%t2=[cos(th3),-sin(th3),0,0;sin(th3),cos(th3),0,0;0,0,1,0;0,0,0,1];
t3=[1,0,0,L2;0,1,0,0;0,0,1,0;0,0,0,1];
t4=[cos(th2),-sin(th2),0,0;sin(th2),cos(th2),0,0;0,0,1,0;0,0,0,1];
t5=[cos(th1),0,sin(th1),0;0,1,0,0;-sin(th1),0,cos(th1),0;0,0,0,1];
t6=[0,0,1,0;1,0,0,0;0,1,0,L1;0,0,0,1];
simplify(t6*t5*t4*t3*t1)

% make the arm using ETS3
import ETS3.*
L1 = 1; L2 = 1; th3 = 1;
% first two joints
E3 = Tz(L1) * Rz('q1') * Ry(pi/2) * Rx(pi) * Rz(pi/2) * Rz('q2') * Tx(L2);
% prismatic joint
E3 = E3 * Ry(pi/2) * Rz(pi/2) * Tz(th3);
%E3.teach()
fk = E3.fkine([th1 th2])
%simplify(fk)





