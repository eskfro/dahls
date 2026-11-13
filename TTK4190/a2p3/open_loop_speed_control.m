function [n_c] = open_loop_speed_control(U_ref)

t = 0.05;       % Same as t_thr in ship.m
Xu = -898300;   % Calulated in main.m - somehow the same as Xudot ...
Dia = 3.3;
KT = 0.6367;

% Desired thrust for the get speed U_ref
T = U_ref * Xu / (t-1);

% Invert the thrust function to get the desired revolution command
n_c = sign(T) * sqrt( abs(T) / (1025 * Dia^4 * KT) );

end