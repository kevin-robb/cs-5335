% pick 2 pts and their surface normal vectors.
p1 = [1;0;1]; n1 = p1 / norm(p1);
p2 = [1;0;-1]; n2 = p2 / norm(p2);
true_ctr = [2;3;4];
p1 = p1 + true_ctr; p2 = p2 + true_ctr;

% use surface norms of pts to find central axis unit vector.
axis = cross(n1, n2); axis = axis / norm(axis);
% project pts/vectors onto plane orthogonal to axis.
% plane will include p1 and use axis as normal vector.
% dist of a point x to the plane is:
dist_to_plane = @(x) dot(axis, x) - dot(axis, p1);
% project p2 onto plane.
p2_proj = p2 - dist_to_plane(p2) * axis;
% find projections of n1 and n2 on plane.
n1_proj = n1 - dist_to_plane(p1 + n1) * axis;
n2_proj = n2 - dist_to_plane(p2_proj + n2) * axis;
% create coord system on plane.
x_plane = n1_proj / norm(n1_proj);
y_plane = cross(x_plane, axis);
y_plane = y_plane / norm(y_plane);
% make change of basis matrix.
chg_basis = [x_plane'; y_plane'];
% change all coords to plane frame.
% pts relative to p1.
p1_plane = chg_basis * (p1 - p1);
p2_plane = chg_basis * (p2_proj - p1);
n1_plane = chg_basis * n1_proj;
n2_plane = chg_basis * n2_proj;
% compute intersection of lines in this frame.
c_x = (n1_plane(2)*n2_plane(1)*p1_plane(1) - n2_plane(2)*n1_plane(1)*p2_plane(1) - n1_plane(1)*n2_plane(1)*(p1_plane(2)-p2_plane(2))) / (n1_plane(2)*n2_plane(1) - n2_plane(2)*n1_plane(1));
c_y = n1_plane(2)/n1_plane(1)*(c_x-p1_plane(1)) + p1_plane(2);
% transform center pt back to 3d coords.
ctr = chg_basis' * [c_x; c_y] + p1;
