function T_0_sphere = calc_T_0_sphere(theta,phi)
%Calculate rotation with sphere coordinates
%   e_x -> e_theta, e_y -> e_phi, e_z -> e_r
R_0_sphere=[cos(theta)*cos(phi),-sin(phi),sin(theta)*cos(phi); ...
cos(theta)*sin(phi),cos(phi),sin(theta)*sin(phi); ...
-sin(theta),0,cos(theta)];
T_0_sphere=eye(4);
T_0_sphere(1:3,1:3)=R_0_sphere;
end