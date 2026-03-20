% aerosonde_parameters.m
% Aircraft: North American X-15
% All field names match the senior's forces_moments.m and mav_dynamics.m exactly.

addpath('../tools');

% -------------------------------------------------------------------------
% Initial conditions
% -------------------------------------------------------------------------
MAV.pn0    = 0;
MAV.pe0    = 0;
MAV.pd0    = -1000;
MAV.u0     = 100;
MAV.v0     = 0;
MAV.w0     = 0;
MAV.phi0   = 0;
MAV.theta0 = 0;
MAV.psi0   = 0;

% Quaternion initial conditions (required by mav_dynamics.m)
e          = Euler2Quaternion(MAV.phi0, MAV.theta0, MAV.psi0);
MAV.e0     = e(1);
MAV.e1     = e(2);
MAV.e2     = e(3);
MAV.e3     = e(4);

MAV.p0     = 0;
MAV.q0     = 0;
MAV.r0     = 0;

% -------------------------------------------------------------------------
% Physical parameters  (x15.dat, converted to SI)
% -------------------------------------------------------------------------
MAV.gravity = 9.81;
MAV.mass    = (31240 / 32.174) * 14.5939;   % 14170.2 kg
MAV.Jx      = 3650  * 1.35582;              % 4948.7  kg-m^2
MAV.Jy      = 80000 * 1.35582;              % 108465.6 kg-m^2
MAV.Jz      = 82000 * 1.35582;              % 111177.2 kg-m^2
MAV.Jxz     = 590   * 1.35582;              % 799.9   kg-m^2

MAV.S_wing  = 200.0 * 0.092903;             % 18.58 m^2
MAV.b       = 22.36 * 0.3048;               % 6.814 m
MAV.c       = 10.27 * 0.3048;               % 3.130 m
MAV.S_prop  = 0.0;                           % no propeller
MAV.rho     = 1.225;
MAV.e       = 0.9;                           % Oswald efficiency (assumed)
MAV.AR      = MAV.b^2 / MAV.S_wing;

% -------------------------------------------------------------------------
% Gamma constants  (Beard & McLain p.36)
% -------------------------------------------------------------------------
MAV.Gamma  = MAV.Jx*MAV.Jz - MAV.Jxz^2;
MAV.Gamma1 = (MAV.Jxz*(MAV.Jx - MAV.Jy + MAV.Jz)) / MAV.Gamma;
MAV.Gamma2 = (MAV.Jz*(MAV.Jz - MAV.Jy) + MAV.Jxz^2) / MAV.Gamma;
MAV.Gamma3 = MAV.Jz / MAV.Gamma;
MAV.Gamma4 = MAV.Jxz / MAV.Gamma;
MAV.Gamma5 = (MAV.Jz - MAV.Jx) / MAV.Jy;
MAV.Gamma6 = MAV.Jxz / MAV.Jy;
MAV.Gamma7 = (MAV.Jx*(MAV.Jx - MAV.Jy) + MAV.Jxz^2) / MAV.Gamma;
MAV.Gamma8 = MAV.Jx / MAV.Gamma;

% -------------------------------------------------------------------------
% Aerodynamic coefficients  — field names MUST match forces_moments.m
% -------------------------------------------------------------------------

% Lift
MAV.C_L_0         = 0.0;
MAV.C_L_alpha     = 3.5;
MAV.C_L_q         = 0.0;
MAV.C_L_delta_e   = 0.54;

% Drag
MAV.C_D_0         = 0.095;
MAV.C_D_alpha     = 0.6;
MAV.C_D_q         = 0.0;
MAV.C_D_delta_e   = 0.0;
MAV.C_D_p         = 0.0;

% Pitching moment
MAV.C_m_0         = 0.0;
MAV.C_m_alpha     = -1.2;
MAV.C_m_q         = -6.2;
MAV.C_m_delta_e   = -0.9;

% Stall model (kept from aerosonde — shape parameters, not aircraft-specific)
MAV.M             = 50;
MAV.alpha0        = 0.47;
MAV.epsilon       = 0.16;

% Side force
MAV.C_Y_0         = 0.0;
MAV.C_Y_beta      = -1.4;
MAV.C_Y_p         = 0.0;
MAV.C_Y_r         = 0.0;
MAV.C_Y_delta_a   = 0.05;
MAV.C_Y_delta_r   = 0.45;

% Rolling moment
MAV.C_ell_0       = 0.0;
MAV.C_ell_beta    = -0.01;
MAV.C_ell_p       = -0.35;
MAV.C_ell_r       = 0.04;
MAV.C_ell_delta_a = -0.06;
MAV.C_ell_delta_r = 0.012;

% Yawing moment
MAV.C_n_0         = 0.0;
MAV.C_n_beta      = 0.5;
MAV.C_n_p         = 0.0;
MAV.C_n_r         = -1.5;
MAV.C_n_delta_a   = -0.04;
MAV.C_n_delta_r   = -0.3;

% -------------------------------------------------------------------------
% Propulsion  (X-15: rocket engine, no propeller)
% -------------------------------------------------------------------------
MAV.k_motor   = 227200;   % max thrust [N]  (51090 lb * 4.44822)
MAV.k_T_p     = 0.0;
MAV.k_Omega   = 0.0;

% These are kept as zero/dummy so any leftover references don't crash
MAV.D_prop    = 0.0;
MAV.K_V       = 0.0;
MAV.KQ        = 0.0;
MAV.R_motor   = 0.0;
MAV.i0        = 0.0;
MAV.ncells    = 0;
MAV.V_max     = 0.0;
MAV.C_Q2      = 0.0;
MAV.C_Q1      = 0.0;
MAV.C_Q0      = 0.0;
MAV.C_T2      = 0.0;
MAV.C_T1      = 0.0;
MAV.C_T0      = 0.0;