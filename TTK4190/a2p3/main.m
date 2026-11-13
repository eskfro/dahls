% Project in TTK4190 Guidance, Navigation and Control of Vehicles 
%
% Author:           Eskil Aarset Fromholtz
% Study program:    Cybernetics and Robotics

addpath(genpath('flypath3d_v3'))
clc; clear; close all;

% helpers
d2r = pi/180;
r2d = 180/pi;
g = 9.81;

% Rigid-body mass (ship.m lines 51-54)
m  = 17.0677e6;
Iz = 2.1732e10;
xg = -3.7;
L  = 161;

% Added mass (lines 78-83)
Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;

% Damping (lines 121-124, renamed so they don't clash with Nomoto T1/T2)
T_sway = 20; T_yaw = 10;
Yv = -(m - Yvdot)/T_sway;
Nr = -(Iz - Nrdot)/T_yaw;

% Rudder coefficients (lines 59, 92-107, span renamed from b)
rho    = 1025;
b_rud  = 2;
AR     = 8;
lambda = b_rud^2 / AR;
CN     = 6.13*lambda / (lambda + 2.25);
aH     = 0.75;
xH     = -0.4 * L;
xR     = -0.5 * L;
Y_delta = 0.25 * (1 + aH) * rho * AR * CN;
N_delta = 0.25 * (xR + aH*xH) * rho * AR * CN;
delta_max_ = 40 * d2r;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initial references
psi_ref = -110 * d2r;  % desired yaw angle (rad)
U_ref   = 9;           % desired surge speed (m/s)
chi_ref = psi_ref;

% initial states
rudder_saturated_ = 0;
e_int = 0; Qm = 0; e_int_u = 0; u_d_ = 0;
eta_0 = [0 0 psi_ref]';
nu_0  = [0 0 0]';
delta_0 = 0;
n_0 = 0;
xd_ = [eta_0(3) 0 0]';
ye_int_ = 0;
x_pred = [0 0 0]'; x_post = x_pred; % not both are needed Im debugging
P_pred = 100 * eye(3); P_post = P_pred;

% State vector
x = [nu_0' eta_0' delta_0 n_0 Qm]'; 
xdot = zeros([9, 1]);

% Nomoto model for heading
u_lin_ = U_ref;
N_ = [(m-Yvdot)/T_sway              (m - Xudot)*u_lin_
      (Xudot - Yvdot)*u_lin_        (Iz - Nrdot)/T_yaw + (m*xg - Yrdot)*u_lin_];

M_RB_ = [m m*xg; m*xg Iz];
M_A_ = -[Yvdot Yrdot; Nvdot Nrdot];
M_ = M_RB_ + M_A_;
b_ = u_lin_^2*[-Y_delta; -N_delta];
M_inv_ = invQR(M_);

% A B C 
A = -M_inv_ * N_; B = M_inv_ * b_; C = [0 1]; D = 0;
[num_, den_] = ss2tf(A, B, C, D);
G_ = tf(num_, den_)

% Heading controller params
% K and T from the first order Nomoto Model for 
% Pole placement algorithm (algo 15.1, Fossen)
wb_ = 0.01;
zeta_ = 1;
wn_ = wb_ / sqrt(1 - 2*zeta_^2 + sqrt(4*zeta_^4 - 4*zeta_^2 + 2));
%T_ = 169.5;
%K_ = 0.00749;
K_ = num_(3) / den_(3)
T_ = den_(2) / den_(3) - num_(2) / num_(3)

m_ = T_/K_; d_ = 1/K_; k_ = 0;

kp_ = m_ * wn_^2 - k_
ki_ = wn_ * kp_ / 10
ki_= 0;
kd_ = 2 * zeta_ * wn_ * m_ - d_
kp_ = 7; kd_ = 110;

% Propeller control
Ja_ = 0;
PD_ = 1.5;
AEAO_ = 0.65;
num_blades_ = 4;
[KT_, KQ_] = wageningen(Ja_, PD_, AEAO_, num_blades_)

% Ocean currents
V_c = 1;
B_Vc = 45 * d2r;

% Wind
rho_air = 1.247;
c_y = 0.95; c_n = 0.15;
V_w =  10;
B_Vw = 135 * d2r;
A_Lw = 10 * L;

% Printing
T1 = 20;
Xu = -(m-Xudot)/T1;

%% LOAD WAYPOINTS FROM DATABASE

WP_struct = load("WP.mat");
WP_ = WP_struct.WP;
num_wp_ = size(WP_, 2);
wp_lookahead_ = 8 * L;
k_wp_ = 1; % wp index pointer
R_wp_ = wp_lookahead_;
Ki_ = 0.003; kappa_ = 5;

%% TIMESTEP

h = 0.1;
h_pos = 0.2; % f_gps = 5 Hz
t_pos = 0;


%% KALMAN FILTER - HEADING

% Nomoto model cont.
A_kf = [0 1 0; 0 -1/T_ -K_/T_; 0 0 0]; B_kf = [0 K_/T_ 0]'; 
C_kf = [1 0 0]; D_kf = 0; E_kf = [0 0; 1 0; 0 1];

% First order discretization
A_d = eye(3) + h*A_kf; B_d = h*B_kf; C_d = C_kf; E_d = h*E_kf;

% Observability check
Ob_kf = obsv(A_d,C_d); rank_Ob_kf = rank(Ob_kf);

% Noise
k = 1e-9;
Q_c = k * [1 0; 0 1]; Q_d = Q_c / h;
R_d = 1e-4;

%% NOISE INJECTION

sigma_psi = 0.5 * d2r; sigma_r = 0.1 * d2r;

%% ESKF

x_esk = zeros([15,1]);
dx_esk = zeros([15,1]);
P_esk = 5 * eye(15);
Q_esk = diag([0.1 0.1 0 0.001 0.001 0 0 0 0.1 0 0 0.001]);
k_esk = 1e3;
R_esk = k_esk * diag([0.1 0.1 0.1 0.1 0.1 0.1 1 1 1 0.01]);
mu_ = 63.44 * d2r;
T_acc_ = 80;
T_gyro_ = 80;

% Sensor noise
sigma_acc_ = 0.001;
sigma_gyro_ = 0.0000175;
sigma_gps_pos_ = 0.025;
sigma_gps_vel_ = 0.02;
sigma_compass_psi_ = deg2rad(0.5);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% sim params
T_final = 6000;
visualizer_speed_ = 250;

% ============================= Selectors ================================

task_1a = 1;    % ocean currents
task_1c = 1;    % wind
task_2d = 0;    % change degree psi_r at t=1000
open_loop_speed_control_ = 0;
use_wp_tracking = 1;        % navigate using wp
show_visualization_ = 0;    % vessel visualization
crab_compensation_ = 0;     % a2p3-2c
use_ilos_ = 1;              % a2p3-2d
inject_noise_ = 1;          % a2p3-4
use_kf_ = 0;                % a2p3-4
use_eskf_ = 1;

if ~task_1a,  print_error('OCEAN CURRENTS DISABLED', 0); end
if use_kf_ && use_eskf_, print_error('BOTH KF AND ESKF ARE CHOSEN', 1); end

t = 0:h:T_final;                
nTimeSteps = length(t);      

simdata = zeros(nTimeSteps, 13);
simdata_2 = zeros(nTimeSteps, 20); 

% ================================ LOOP ==================================

for i = 1:nTimeSteps
    % variable renaming
    t_ = t(i);
    eta_ = x(4:6); nu_ = x(1:3); delta_ = x(7);
    x_ = eta_(1); y_ = eta_(2); psi_ = eta_(3);
    u_ = nu_(1); v_ = nu_(2); r_ = nu_(3);
    x_meas = x_; y_meas = y_;
    u_meas = u_; v_meas = v_;
    psi_meas = psi_; r_meas = r_;
    u_dot = xdot(1); v_dot = xdot(2);

    % ===== OCEAN CURRENT =====
    if task_1a, uc = V_c * cos(B_Vc - psi_); vc = V_c * sin(B_Vc - psi_);
    else, uc = 0; vc = 0; end
    nu_c = [ uc vc 0 ]'; u_rc_ = u_ - uc; v_rc_ = v_ - vc;
    simdata_2(i, 1) = u_rc_;
    simdata_2(i, 2) = v_rc_;

    % ===== WIND =====
    if t_ >= 200 && task_1c
        u_w_ = V_w * cos(B_Vw - psi_); v_w_ = V_w * sin(B_Vw - psi_);
        u_rw_ = u_ - u_w_; v_rw_ = v_ - v_w_;
        gamma_wr_ = -atan2(v_rw_, u_rw_);
        V_rw_ = sqrt(u_rw_^2 + v_rw_^2);
        CY_ = c_y * sin(gamma_wr_);
        CN_ = c_n * sin(2 * gamma_wr_);
        k_w_ = 0.5 * rho_air * V_rw_^2;
        Ywind = k_w_ * CY_ * A_Lw;
        Nwind = k_w_ * CN_ * A_Lw * L;
        tau_wind = [0 Ywind Nwind]';
    else
        tau_wind = [0 0 0]';
    end

    % ===== CRAB AND SIDESLIP =====     
    B_c_ = atan2(v_, u_);       % crab
    B_ = atan2(v_rc_, u_rc_);   % sideslip
    chi_ = psi_ + B_c_;         % course
    simdata_2(i, 4) = B_c_; simdata_2(i, 5) = B_; simdata_2(i, 6) = chi_;

    % ===== NOISE INJECTION =====

    if inject_noise_
        psi_meas = psi_ + normrnd(0, sigma_psi);
        r_meas = r_ + normrnd(0, sigma_r);
        x_meas = x_ + normrnd(0, sigma_gps_pos_);
        y_meas = y_ + normrnd(0, sigma_gps_pos_);
        u_meas = u_ + normrnd(0, sigma_gps_vel_);
        v_meas = v_ + normrnd(0, sigma_gps_vel_);
    end
    r_est = r_meas; psi_est = psi_meas;
    x_est = x_meas; y_est = y_meas;
    u_est = u_meas; v_est = v_meas;
    b_est = 0;
    
    % Save measurements
    simdata_2(i, 8) = psi_meas; simdata_2(i, 9) = r_meas;
    simdata_2(i, 13) = x_meas; simdata_2(i, 14) = y_meas;

    % ===== KALMAN FILTER HEADING =====

    if use_kf_
        [x_post, P_post, x_pred, P_pred] = KF(x_post, P_post, ...
            A_d, B_d, C_d, E_d, Q_d, R_d, psi_meas, delta_);
        
        psi_est = x_post(1); r_est = x_post(2); b_est = x_post(3);
    end

        
    % ===== ESKF =====

    if use_eskf_

        a_ = [u_dot - r_ * v_, v_dot + r_ * u_, -gravity(mu_)]';
        w_ = [0 0 r_]';
        a_meas_ = a_ + normrnd(0, sigma_acc_, size(a_));
        w_meas_ = w_ + normrnd(0, sigma_gyro_, size(w_));
        imu_meas_ = [a_meas_; w_meas_];
        rcv_pos_meas = t_ > t_pos;
    
        if rcv_pos_meas
            y_pos = [x_meas; y_meas; 0];
            y_vel =  Rzyx(0,0,psi_) * [u_meas; v_meas; 0];
            [x_esk, P_esk] = ins_euler(x_esk, P_esk, mu_, h, Q_esk, R_esk, ...
                T_acc_, T_gyro_, imu_meas_, psi_meas, y_pos, y_vel);
            t_pos = t_pos + h_pos;
        else
            [x_esk, P_esk] = ins_euler(x_esk, P_esk, mu_, h, Q_esk, R_esk, ...
                T_acc_, T_gyro_, imu_meas_, psi_meas);
        end
    
         psi_est = x_esk(12); r_est = w_meas_(3) - x_esk(15);
         x_est = x_esk(1); y_est = x_esk(2);

    end

    simdata_2(i, 10) = psi_est; simdata_2(i, 11) = r_est; simdata_2(i, 12) = b_est;
    simdata_2(i, 15) = x_est; simdata_2(i, 16) = y_est;

    % ===== WP GUIDANCE =====

    if k_wp_ < (num_wp_ - 1)
        k_wp_ = wp_selector(x_est, y_est, k_wp_, WP_, R_wp_);
    end

    % This got broken when i added noise:
    % if rudder_saturated_, ye_int_ = 0; end

    [chi_ref, y_e_, ye_int_] = guidance(x_est, y_est, WP_(:,k_wp_), WP_(:,k_wp_+1) ...
        , wp_lookahead_, use_ilos_, ye_int_, [Ki_, kappa_, h]);

    simdata_2(i, 3) = y_e_; simdata_2(i, 7) = chi_ref;

    % ===== HEADING REFERENCE MODEL =====

    if t_>=1000 && task_2d, psi_ref = (-180) * d2r; end

    if use_wp_tracking
        if crab_compensation_, psi_ref = chi_ref - B_c_;
        else, psi_ref = chi_ref; end 
    end

    xd_dot_ = ref_model(xd_, psi_ref);
    xd_ = xd_ + xd_dot_ * h;
    psi_d = xd_(1);
    r_d = xd_(2);

    % ===== HEADING PID CONTROLLER =====

    e_psi = ssa(psi_d - psi_est);
    e_r = r_d - r_est;
    if rudder_saturated_, e_int = 0; else, e_int = e_int + e_psi * h; end
    
    delta_c = PID_heading(e_psi, e_r, e_int, [kp_, ki_, kd_]);
    if abs(delta_c) > delta_max_, rudder_saturated_ = 1; else, rudder_saturated_ = 0; end

    % ===== SURGE SPEED CONTROLLER =====
    
    T_u_ = 50; % Time constant for (u_d/U_ref)(s) 1st. order dynamics 
    u_d_ = u_d_ + h * (U_ref - u_d_) / T_u_;
    e_u = u_ - u_d_; e_int_u = e_int_u + e_u * h;
    if open_loop_speed_control_, n_c = open_loop_speed_control(U_ref);
    else, n_c = closed_loop_speed_control(u_d_, e_u, e_int_u); end

    % ===== SOLVER =====

    % ship dynamics
    u = [delta_c n_c]';
    [xdot,tau_total] = ship(x,u,nu_c,tau_wind);
    
    % store simulation data in a table (for testing)
    simdata(i,:) = [x(1:3)' x(4:6)' x(7) x(8) u(1) u(2) u_d_ psi_d r_d];     
 
    % Euler integration
    % x = euler2(xdot,x,h); 
 
    x = rk4(@ship,h,x,u,nu_c,tau_wind);

    % --- Progress Update Logic ---
    if mod(i, floor(nTimeSteps / 10)) == 0 % Print an update every 10%
        progress = (i/nTimeSteps) * 100;
        fprintf('  %d%% complete\n', round(progress));
    elseif i == 1
        disp("  Simulation in progress:")
        fprintf('  %d%% complete\n', 0);
    end

end
simdata = simdata(1:i,:);
t = t(1:i);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u           = simdata(:,1);                 % m/s
v           = simdata(:,2);                 % m/s
r           = simdata(:,3);                 % rad/s
r_deg       = (180/pi) * r;                 % deg/s
x           = simdata(:,4);                 % m
y           = simdata(:,5);                 % m
psi         = simdata(:,6);                 % rad
psi_deg     = (180/pi) * psi;               % deg
delta_deg   = (180/pi) * simdata(:,7);      % deg
n           = (30/pi) * simdata(:,8);       % rpm
delta_c_deg = (180/pi) * simdata(:,9);      % deg
n_c         = (30/pi) * simdata(:,10);      % rpm
u_d         = simdata(:,11);                % m/s
psi_d       = simdata(:,12);                % rad
psi_d_deg   = (180/pi) * psi_d;             % deg
r_d         =  simdata(:,13);               % rad/s
r_d_deg     = (180/pi) * r_d;               % deg/s

ur_ = simdata_2(:, 1);
vr_ = simdata_2(:, 2);
y_e_ = simdata_2(:, 3);
B_c_ = simdata_2(:, 4);
B_ = simdata_2(:, 5);
chi_ = simdata_2(:, 6);
chi_ref = simdata_2(:, 7);
psi_meas = simdata_2(:, 8);
r_meas = simdata_2(:, 9);
psi_est = simdata_2(:, 10);
r_est = simdata_2(:, 11);
b_est = simdata_2(:, 12);
x_meas = simdata_2(:, 13);
y_meas = simdata_2(:, 14);
x_est = simdata_2(:, 15);
y_est = simdata_2(:, 16);

%% PLOTS

figure(3)
figure(gcf)
subplot(311)
plot(y,x,'linewidth',2); axis('equal')
title('North-East positions'); xlabel('(m)'); ylabel('(m)'); 
subplot(312)
plot(t,psi_deg,t,psi_d_deg,'linewidth',2);
title('Actual and desired yaw angle'); xlabel('Time (s)');  ylabel('Angle (deg)'); 
legend('actual yaw','desired yaw')
subplot(313)
plot(t,r_deg,t,r_d_deg,'linewidth',2);
title('Actual and desired yaw rates'); xlabel('Time (s)');  ylabel('Angle rate (deg/s)'); 
legend('actual yaw rate','desired yaw rate')

figure(2)
figure(gcf)
subplot(311)
plot(t,u,t,u_d,'linewidth',2);
yline(0, 'linewidth', 2);
title('Actual and desired surge velocity'); xlabel('Time (s)'); ylabel('Velocity (m/s)');
legend('actual surge','desired surge')
subplot(312)
plot(t,n,t,n_c,'linewidth',2);
yline(0, 'linewidth', 2);
title('Actual and commanded propeller speed'); xlabel('Time (s)'); ylabel('Motor speed (RPM)');
legend('actual RPM','commanded RPM')
subplot(313)
plot(t,delta_deg,t,delta_c_deg,'linewidth',2);
yline(0, 'linewidth', 2);
title('Actual and commanded rudder angle'); xlabel('Time (s)'); ylabel('Angle (deg)');
ylim([-100,100]);
legend('actual rudder angle','commanded rudder angle')

%% WP PATHPLOTTER

pathplotter(x, y);

%% CRAB AND SIDESLIP PLOTS

figure(10)
figure(gcf)
plot(t, B_c_ * r2d, 'linewidth', 2, 'color', 'r');
hold on
plot(t, B_ * r2d, 'linewidth', 2, 'color', 'b');
if task_1a, title('Crab and sideslip | with ocean currents'); 
else, title('Crab and sideslip | without ocean currents'); end
xlabel('Time(s)'); ylabel('Angle (deg)');
grid on
legend('B_c', 'B');

%% CROSS TRACK ERROR PLOT

figure(11)
plot(t, y_e_, 'linewidth', 2);
yline(0);
grid on
title('WP cross track error');
xlabel('Time (s)');
ylabel('Distance (m)');

%% COURSE AND HEADING PLOT

t_start_ = 0;     % skip start-up, angles are large at low speed

figure(12)
subplot(211)
plot(t, chi_ * r2d, t, chi_ref * r2d, t, psi_deg, 'linewidth', 2);
yline(0, 'linewidth', 2);
xlim([t_start_ t(end)]);
grid on
title('Course and heading');
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('\chi', '\chi_d', '\psi');

subplot(212)
plot(t, B_c_ * r2d, t, B_ * r2d, 'linewidth', 2);
yline(0, 'linewidth', 2);
xlim([t_start_ t(end)]);
grid on
title('Crab and sideslip');
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('\beta_c', '\beta');

%% PLOT COURSE MEASUREMENTS

t_begin = 0; t_end = t(end);

figure(13)
subplot(311);
plot(t, psi_meas * r2d, 'linewidth', 2);
hold on
grid on
plot(t, psi_deg, 'linewidth',2);
plot(t, psi_est * r2d, 'linewidth', 2);
yline(0);
xlim([t_begin, t_end]);
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('psi meas', 'psi', 'psi est');
title('Yaw');

subplot(312);
plot(t, r_meas * r2d, 'linewidth', 2);
hold on
grid on
plot(t, r_deg, 'linewidth',2);
plot(t, r_est * r2d, 'linewidth', 2);
yline(0);
xlim([t_begin,t_end]);
ylim([-1.5, 1.5]);
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('r meas', 'r', 'r est');
title('Yaw rate');

subplot(313);
plot(t, b_est * r2d, 'linewidth', 2);
yline(0);
hold on
grid on
xlim([t_begin,t_end]);
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('bias estimate');
title('Rudder bias');

%% ESKF PLOTS

psi_err_deg_ = mod(psi_est - psi + pi, 2*pi) - pi;     % wrapped to [-pi, pi)
psi_err_deg_ = psi_err_deg_ * r2d;
r_err_deg_ = (r_est - r) * r2d;

figure(14)
subplot(311)
plot(t, psi_d_deg, t, psi_deg, t, psi_est * r2d, 'linewidth', 2);
grid on
title('Yaw angle: desired, true and ESKF estimate');
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('\psi_d', '\psi', '\psi_{est}');

subplot(312)
plot(t, r_d_deg, t, r_deg, t, r_est * r2d, 'linewidth', 2);
grid on
title('Yaw rate: desired, true and ESKF estimate');
xlabel('Time (s)'); ylabel('Rate (deg/s)');
legend('r_d', 'r', 'r_{est}');

subplot(313)
plot(t, psi_err_deg_, t, r_err_deg_, 'linewidth', 2);
yline(0, 'HandleVisibility', 'off');
grid on
title('Estimation error');
xlabel('Time (s)'); ylabel('Error (deg, deg/s)');
legend('\psi_{est} - \psi', 'r_{est} - r');

%% POSITION ESTIMATION PLOTS

figure(15)
subplot(211)
plot(y, x, y_est, x_est, '--', 'linewidth', 2); axis('equal')
grid on
title('North-East position: true and estimated');
xlabel('East (m)'); ylabel('North (m)');
legend('True', 'Estimated');

subplot(212)
plot(t, x_est - x, t, y_est - y, 'linewidth', 2);
yline(0, 'HandleVisibility', 'off');
grid on
title('Position estimation error');
xlabel('Time (s)'); ylabel('Error (m)');
legend('North error', 'East error');

figure(16)
plot(t, x_meas - x, '.', t, x_est - x, 'linewidth', 2);
xlim([1000 1060]);
grid on
title('North position: GNSS measurement error vs. ESKF estimate error');
xlabel('Time (s)'); ylabel('Error (m)');
legend('GNSS measurement', 'ESKF estimate');

%% GNSS MEASUREMENTS AND ESTIMATED POSITION

gnss_idx_ = 2:2:length(t);                  % GNSS steps at 5 Hz (t = 0.1, 0.3, ...)
gnss_win_ = gnss_idx_(t(gnss_idx_) >= 1000 & t(gnss_idx_) <= 1100);
est_win_  = find(t >= 1000 & t <= 1100);

figure(17)
subplot(121)
plot(y, x, 'k', 'linewidth', 2); hold on
plot(y_est, x_est, '--', 'linewidth', 2);
plot(y_meas(gnss_idx_(1:50:end)), x_meas(gnss_idx_(1:50:end)), '.', 'markersize', 10);
hold off
axis('equal'); grid on
title('Route: true, estimated and GNSS');
xlabel('East (m)'); ylabel('North (m)');
legend('True', 'ESKF estimate', 'GNSS (every 10 s)');

subplot(122)
plot(y_meas(gnss_win_) - y(gnss_win_), x_meas(gnss_win_) - x(gnss_win_), '.', 'markersize', 10); hold on
plot(y_est(est_win_) - y(est_win_), x_est(est_win_) - x(est_win_), 'linewidth', 2);
plot(0, 0, 'k+', 'markersize', 12, 'linewidth', 2);
hold off
axis('equal'); grid on
title('Position error around true position (t = 1000-1100 s)');
xlabel('East error (m)'); ylabel('North error (m)');
legend('GNSS measurement', 'ESKF estimate', 'True position');


%% Create objects for 3-D visualization 

if show_visualization_
    % Since we only simulate 3-DOF we need to construct zero arrays for the 
    % excluded dimensions, including height, roll and pitch
    z = zeros(length(x),1);
    phi = zeros(length(psi),1);
    theta = zeros(length(psi),1);
    
    % create object 1: ship (ship1.mat)
    new_object('flypath3d_v3/ship1.mat',[x,y,z,phi,theta,psi],...
    'model','royalNavy2.mat','scale',(max(max(abs(x)),max(abs(y)))/1000),...
    'edge',[0 0 0],'face',[0 0 0],'alpha',1,...
    'path','on','pathcolor',[.89 .0 .27],'pathwidth',2);
    
    % Plot trajectories 
    flypath('flypath3d_v3/ship1.mat',...
    'animate','on','step',visualizer_speed_,...
    'axis','on','axiscolor',[0 0 0],'color',[1 1 1],...
    'font','Georgia','fontsize',12,...
    'view',[-25 35],'window',[900 900],...
    'xlim', [min(y)-0.1*max(abs(y)),max(y)+0.1*max(abs(y))],... 
    'ylim', [min(x)-0.1*max(abs(x)),max(x)+0.1*max(abs(x))], ...
    'zlim', [-max(max(abs(x)),max(abs(y)))/100,max(max(abs(x)),max(abs(y)))/20]); 
end