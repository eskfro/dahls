function [x_post, P_post, x_pred, P_pred] = KF(x_pred, P_pred, Ad, Bd, Cd, Ed, Qd, Rd, psi_meas, delta)

% Prediction
x_pred = Ad * x_pred + Bd * delta;
P_pred = Ad * P_pred * Ad' + Ed * Qd * Ed';

% Correction
psi_pred = x_pred(1);
v = -ssa(psi_pred - psi_meas);
K = P_pred * Cd' * invQR(Cd * P_pred * Cd' + Rd);
IKC = eye(3) - K * Cd;
x_post = x_pred + K * v;
P_post = IKC*P_pred*IKC' + K*Rd*K';  

end