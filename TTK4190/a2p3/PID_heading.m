function [delta_c] = PID_heading(e_psi, e_r, e_int, params)

kp = params(1);
ki = params(2);
kd = params(3);

delta_c = kp * e_psi + kd * e_r + ki * e_int;

end
