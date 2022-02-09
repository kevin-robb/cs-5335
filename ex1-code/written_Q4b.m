% question 4-c on the written part of ex1.
syms th1 th2 th3 L1 L2
% create the arm in 4-b using ETS3.
import ETS3.*
%L1 = 1; L2 = 1; th3 = 1;
% first two joints
E3 = Rz(pi/2) * Tz(L1) * Rz('q1') * Ry(pi/2) * Rx(pi) * Rz(pi/2) * Rz('q2') * Tx(L2);
% prismatic joint
E3 = E3 * Ry(pi/2) * Rz(pi/2) * Tz(th3);
%E3.teach()
simplify(E3.fkine([th1 th2]))


% my result from doing it manually:
t1=[0,0,1,0;1,0,0,0;0,1,0,L1;0,0,0,1];
t2=[cos(th1),0,sin(th1),0;0,1,0,0;-sin(th1),0,cos(th1),0;0,0,0,1];
t3=[cos(th2),-sin(th2),0,0;sin(th2),cos(th2),0,0;0,0,1,0;0,0,0,1];
t4=[1,0,0,L2;0,1,0,0;0,0,1,0;0,0,0,1];
t5=[0,0,1,0;1,0,0,0;0,1,0,0;0,0,0,1];
t6=[1,0,0,0;0,1,0,0;0,0,1,th3;0,0,0,1];
simplify(t1*t2*t3*t4*t5*t6)



