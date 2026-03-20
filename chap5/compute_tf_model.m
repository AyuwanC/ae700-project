% compute_tf_model.m
% Computes transfer function coefficients for the X-15 from trim conditions
% and the linearised model (Beard & McLain, Section 5.4)
%
% Must be run AFTER compute_trim.m has produced x_trim and u_trim.
% Produces the variables that compute_autopilot_gains.m needs.

%% -----------------------------------------------------------------------
%  Trim values
% -----------------------------------------------------------------------
Va_trim    = x_trim(4);          % trim airspeed (body x, approx Va for small alpha)
alpha_trim = atan2(x_trim(6), x_trim(4));  % trim angle of attack
theta_trim = x_trim(8);          % trim pitch angle
phi_trim   = x_trim(7);          % trim roll  angle (≈0 for level flight)

delta_e_trim = u_trim(1);
delta_t_trim = u_trim(4);

%% -----------------------------------------------------------------------
%  Intermediate inertia / coefficient groupings
% -----------------------------------------------------------------------
% Cp coefficients  (roll axis, Beard & McLain Eq 5.6)
C_p_0         = MAV.Gamma3 * MAV.C_ell_0       + MAV.Gamma4 * MAV.C_n_0;
C_p_beta      = MAV.Gamma3 * MAV.C_ell_beta     + MAV.Gamma4 * MAV.C_n_beta;
C_p_p         = MAV.Gamma3 * MAV.C_ell_p        + MAV.Gamma4 * MAV.C_n_p;
C_p_r         = MAV.Gamma3 * MAV.C_ell_r        + MAV.Gamma4 * MAV.C_n_r;
C_p_delta_a   = MAV.Gamma3 * MAV.C_ell_delta_a  + MAV.Gamma4 * MAV.C_n_delta_a;
C_p_delta_r   = MAV.Gamma3 * MAV.C_ell_delta_r  + MAV.Gamma4 * MAV.C_n_delta_r;

% Cr coefficients  (yaw axis)
C_r_0         = MAV.Gamma4 * MAV.C_ell_0       + MAV.Gamma8 * MAV.C_n_0;
C_r_beta      = MAV.Gamma4 * MAV.C_ell_beta     + MAV.Gamma8 * MAV.C_n_beta;
C_r_p         = MAV.Gamma4 * MAV.C_ell_p        + MAV.Gamma8 * MAV.C_n_p;
C_r_r         = MAV.Gamma4 * MAV.C_ell_r        + MAV.Gamma8 * MAV.C_n_r;
C_r_delta_a   = MAV.Gamma4 * MAV.C_ell_delta_a  + MAV.Gamma8 * MAV.C_n_delta_a;
C_r_delta_r   = MAV.Gamma4 * MAV.C_ell_delta_r  + MAV.Gamma8 * MAV.C_n_delta_r;

%% -----------------------------------------------------------------------
%  Longitudinal transfer function coefficients  (Beard & McLain Sec 5.4)
% -----------------------------------------------------------------------
q_bar_trim = 0.5 * MAV.rho * Va_trim^2;

% --- Pitch rate / elevator  T_theta_delta_e = a_theta3 / (s^2 + a_theta1*s + a_theta2) ---
a_theta1 = - q_bar_trim * MAV.S_wing * MAV.c * MAV.C_m_q ...
             * MAV.c / (2 * MAV.Jy * Va_trim);
a_theta2 = - q_bar_trim * MAV.S_wing * MAV.c * MAV.C_m_alpha / MAV.Jy;
a_theta3 =   q_bar_trim * MAV.S_wing * MAV.c * MAV.C_m_delta_e / MAV.Jy;

% --- Airspeed / throttle  T_Va_delta_t = a_V2 / (s + a_V1) ---
% a_V1 captures aerodynamic drag damping on Va
a_V1 = (MAV.rho * Va_trim * MAV.S_wing / MAV.mass) * ...
       (MAV.C_D_0 + MAV.C_D_alpha * alpha_trim + MAV.C_D_delta_e * delta_e_trim) ...
       + MAV.rho * Va_trim * MAV.S_prop * MAV.k_motor^2 * delta_t_trim^2 / MAV.mass;
% a_V2 captures thrust sensitivity to throttle
a_V2 = MAV.k_motor / MAV.mass;   % rocket: thrust directly proportional to throttle
% a_V3 captures pitch effect on airspeed
a_V3 =  MAV.gravity * cos(theta_trim - alpha_trim);

% --- Altitude / pitch  T_h_theta ---
%   already defined as Va_trim / s

%% -----------------------------------------------------------------------
%  Lateral transfer function coefficients  (Beard & McLain Sec 5.4)
% -----------------------------------------------------------------------
% --- Roll / aileron  T_phi_delta_a = a_phi2 / (s^2 + a_phi1*s) ---
a_phi1 = -0.5 * MAV.rho * Va_trim^2 * MAV.S_wing * MAV.b ...
          * C_p_p * (MAV.b / (2 * Va_trim));
a_phi2 =  0.5 * MAV.rho * Va_trim^2 * MAV.S_wing * MAV.b * C_p_delta_a;

% --- Sideslip / rudder  T_v_delta_r = a_beta2 / (s + a_beta1) ---
a_beta1 = -MAV.rho * Va_trim * MAV.S_wing * MAV.C_Y_beta / (2 * MAV.mass);
a_beta2 =  MAV.rho * Va_trim * MAV.S_wing * MAV.C_Y_delta_r / (2 * MAV.mass);

%% -----------------------------------------------------------------------
%  Define transfer functions  (same names the seniors used)
% -----------------------------------------------------------------------
T_phi_delta_a   = tf([a_phi2],          [1, a_phi1, 0]);
T_chi_phi       = tf([MAV.gravity / Va_trim], [1, 0]);
T_theta_delta_e = tf(a_theta3,          [1, a_theta1, a_theta2]);
T_h_theta       = tf([Va_trim],         [1, 0]);
T_h_Va          = tf([theta_trim],      [1, 0]);
T_Va_delta_t    = tf([a_V2],            [1, a_V1]);
T_Va_theta      = tf([-a_V3],           [1, a_V1]);
T_v_delta_r     = tf([a_beta2],         [1, a_beta1]);

%% -----------------------------------------------------------------------
%  Print summary for verification
% -----------------------------------------------------------------------
fprintf('\n=== Transfer Function Coefficients (X-15 @ Va=%.1f m/s) ===\n', Va_trim);
fprintf('Longitudinal:\n');
fprintf('  a_theta1 = %8.4f  a_theta2 = %8.4f  a_theta3 = %8.4f\n', a_theta1, a_theta2, a_theta3);
fprintf('  a_V1     = %8.4f  a_V2     = %8.4f  a_V3     = %8.4f\n', a_V1, a_V2, a_V3);
fprintf('Lateral:\n');
fprintf('  a_phi1   = %8.4f  a_phi2   = %8.4f\n', a_phi1, a_phi2);
fprintf('  a_beta1  = %8.4f  a_beta2  = %8.4f\n', a_beta1, a_beta2);
fprintf('  Va_trim  = %8.4f  theta_trim = %8.4f rad\n', Va_trim, theta_trim);