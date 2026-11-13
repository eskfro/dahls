function [chi_d, ye, ye_int] = guidance(x, y, wp_1, wp_2, lookahead, use_ilos, ye_int_, params)

% hint: chi_d = pi_h - atan(Kp * y_e^p)
% params: [Ki, kappa, h]

D = lookahead;
Kp = 1/D;
Ki = params(1);
kappa = params(2);
h = params(3);

x1 = wp_1(1); y1 = wp_1(2);
x2 = wp_2(1); y2 = wp_2(2); 
pi_h = atan2(y2 - y1, x2 - x1); % path angle between two waypoints

% cross track error
y_e = -(x - x1)*sin(pi_h) + (y - y1)*cos(pi_h); 

% cross track error integral dynamics
ye_int = ye_int_ + h * (D * y_e) / (D^2 + (y_e + kappa * ye_int_)^2);

corr = Kp * y_e;
if use_ilos, corr = corr + Ki * ye_int; end
chi_d = pi_h - atan(corr);
ye = y_e;


end