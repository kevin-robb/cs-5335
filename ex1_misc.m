%ch2 textbook examples
syms theta phi psi
% logm of a rot2 by theta = skew symmetric matrix with w=theta.
% inverse is true, so the following 2 statements are equiv.
R = rot2(0.3);
R = expm(skew(0.3));

% homogenous transformation composed of a translation and a rotation.
T1 = transl2(1,2) * trot2(30, 'deg');
% plot this in world frame.
plotvol([0,5,0,5])
trplot2(T1,'frame','1','color','b') %'b'=blue
% plot a point
plot_point([3;2], 'label', 'P', 'solid', 'ko');
% can convert column vectors between euclidean and homogenous (adding the
% 1) with the helper functions e2h() and h2e().

% 3d rotation matrices about each axis:
Rx = rotx(pi/2);
Ry = roty(theta);
Rz = rotz(theta);
% plot this. can also animate it
trplot(Rx)
tranimate(Rx)
% euler angles are ZYZ rotations, so the following are equivalent.
gamma = [phi, theta, psi];
R = rotz(phi)*roty(theta)*rotz(psi);
R = eul2r(phi,theta,psi);
% the inverse of this gives euler angles from a rot matrix.
tr2eul(R) %always returns positive angles (not unique mapping)

% bc of conventions for robot gripper, we use ZYX seq.
% we can get this roll-pitch-yaw with
R = rpy2r(roll, pitch, yaw);
gamma = tr2rpy(R);
% gui to experiment with euler angles
tripleangle
% we can instead define the pose of the hand with orientation and approach
% vectors (normal vec can be calc from these). knowing a and o,
a = [1 0 0]';
o = [0 1 0]';
R = oa2r(o,a);
% any rotation can be defined by an axis and angle.
[theta, v] = tr2angvec(R);
% can also use vex(logm(R)), or
[theta,axis] = trlog(R);

% we can get eigenvalues and vectors
[x,e] = eig(R);

% quaternion stuff. can use constructor to convert something such as a
% rotation matrix to a unit quaternion.
q = UnitQuaternion( rpy2tr(0.1,0.2,0.3) );
% can be converted to orthonormal rot matrix by
r = q.R;
% rotate a vector by a quaternion with mult *
vect = q*[1 0 0]';

% homogenous transforms in 3d
T = transl(1, 0, 0) * trotx(pi/2) * transl(0, 1, 0);









