function [n_c] = closed_loop_speed_control(U_ref, e_u, e_int_u)

t = 0.05;       % Same as t_thr in ship.m
Xu = -898300;   % Calulated in main.m - somehow the same as Xudot ...
Dia = 3.3;
KT = 0.6367;
d_u = -Xu;

w = 0.05;
z = 0.8;
m = 17.0677e6;

% Controller params
kp = 2*z*w*m - d_u;
ki = m*w^2;

% Desired thrust for the get speed U_ref
T = U_ref * Xu / (t-1);

% add the PI controller terms to thrust vector
T = T - ( kp * e_u + ki * e_int_u );

% Invert the thrust function to get the desired revolution command
n_c = sign(T) * sqrt( abs(T) / (1025 * Dia^4 * KT) );

 
end