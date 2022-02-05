%ch7 textbook examples
import ETS2.* %2d robot arm
a1 = 1; %length of this arm segment
E = Rz('q1') * Tx(a1); %1R arm
E.fkine(30, 'deg'); %forward kinematics with given joint angle.
%E.teach %interactive gui of arm

a1 = 1; a2 = 1;
E = Rz('q1') * Tx(a1) * Rz('q2') * Tx(a2); %2R arm
E.fkine( [30, 40], 'deg'); % forward kin from joint angle vector.
%E.plot( [30, 40], 'deg') %non-interactive gui of arm
E.structure; %will print RR since it is 2R arm

% new examples, page 205
E = Rz('q1') * Tx(a1) * Rz('q2') * Tx(a2);
syms q1 q2 real
TE = E.fkine( [q1, q2] );
% represent desired end-eff pos
syms x y real
e1 =  x == TE.t(1);
e2 =  y == TE.t(2);
% we now have 2 scalar eqns we can solve simultaneously.
[s1,s2] = solve( [e1 e2], [q1 q2] );
% there is not a unique solution.
length(s2); %indicates there are 2 solns for q2.
s2(1); %one solution, pairs with s1(1).
% we can use ikine_sym to generate symbolic inv kin solns for some arms.

% numerical solution. try to minimize error of forward kinematics.
pstar = [0.6; 0.7];
q = fminsearch( @(q) norm( E.fkine(q).t - pstar ), [0 0] );
% 1st arg is error function, fwd kin position (t) - desired position.
% 2nd arg is initial guess for joint coords.
E.fkine(q).print %shows our calculated q does give correct position.







