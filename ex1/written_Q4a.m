% question 4-c on the written part of ex1.
syms th1 th2 th3 L1 L2 L3
% create the arm in 4-a.
%L1 = 1; L2 = 1; L3 = 1;
L(1) = Revolute('d', L1, 'a', 0, 'alpha', pi/2);
L(2) = Revolute('d', 0, 'a', L2, 'alpha', 0);
L(3) = Revolute('d', 0, 'a', L3, 'alpha', 0);
arm_a = SerialLink(L, 'name', 'arm_a');
%arm_a
%arm_a.plot([pi/4 0 pi/4])
fk = arm_a.fkine([th1 th2 th3]);
simplify(fk);
% for some reason this gives huge answers with factors of 81129638414606686663546605165575/162259276829213363391578010288128
% that it won't simplify on its own even though it equals 1/2. thus I have
% to do it manually.
fk_a = [0.5*cos(th1+th2+th3)+0.5*cos(th2-th1+th3), -0.5*sin(th1+th2+th3)+0.5*sin(th2-th1+th3), sin(th1), 0.5*L3*cos(th2-th1+th3)+0.5*L2*cos(th1+th2)+0.5*L2*cos(th1-th2)+0.5*L3*cos(th1+th2+th3);
        0.5*sin(th1+th2+th3)-0.5*sin(th2-th1+th3), 0.5*cos(th1+th2+th3)-0.5*cos(th2-th1+th3), -1*cos(th1), 0.5*L2*sin(th1+th2)-0.5*L3*sin(th2-th1+th3)+0.5*L2*sin(th1-th2)+0.5*L3*sin(th1+th2+th3);
        sin(th2+th3), cos(th2+th3), 0, L1 + L3*sin(th2+th3) + L2*sin(th2);
        0,0,0,1];
simplify(fk_a)

% my result from doing it manually:
t1=[1,0,0,L3;0,1,0,0;0,0,1,0;0,0,0,1];
t2=[cos(th3),-sin(th3),0,0;sin(th3),cos(th3),0,0;0,0,1,0;0,0,0,1];
t3=[1,0,0,L2;0,1,0,0;0,0,1,0;0,0,0,1];
t4=[cos(th2),-sin(th2),0,0;sin(th2),cos(th2),0,0;0,0,1,0;0,0,0,1];
t5=[cos(th1),0,sin(th1),0;0,1,0,0;-sin(th1),0,cos(th1),0;0,0,0,1];
t6=[0,0,1,0;1,0,0,0;0,1,0,L1;0,0,0,1];
t=t1*t2*t3*t4*t5*t6;
simplify(t6*t5*t4*t3*t2*t1)

% try with regular links. params = [theta,d,a,alpha]
% L(1) = Link([0 L1 0 pi/2]);
% L(2) = Link([0 0 L2 0]);
% L(3) = Link([0 0 L2 0]);
% arm_b = SerialLink(L, 'name', 'arm_b');
% fk_b = arm_b.fkine([theta1 theta2 theta3]);