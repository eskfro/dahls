function [xd_dot] = ref_model(xd, psi_r)
% [xd_dot] = psi_desired_tf(xd, psi_r)
% state vector xd = (psi, r, a)

xd_dot = zeros(3, 1);

r_max = 10;
a_max = 10;
w = 0.03;       % w_ref
w = 0.08;
z = 1;          % zeta

sat_ad = sat(xd(3), a_max);
sat_rd = sat(xd(2), r_max);

xd_dot(1) = sat_rd;
xd_dot(2) = sat_ad;
xd_dot(3) = -(2*z+1)*w*sat_ad - (2*z+1)*w^2*sat_rd + w^3*ssa(psi_r - xd(1));

end