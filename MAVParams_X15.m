% MAVParams_X15.m
% Aircraft parameters for the North American X-15
% Source: x15.dat (converted from imperial to SI units)
%
% Unit conversions used:
%   1 slug         = 14.5939 kg
%   1 slug-ft^2    = 1.35582 kg-m^2
%   1 ft^2         = 0.092903 m^2
%   1 ft           = 0.3048 m
%   1 lb (force)   = 4.44822 N
%   Weight -> mass: m = W_lb / 32.174 [slug] then * 14.5939 [kg]

%% --- Inertial properties ---
% Weight: 31240 lb  -> mass = 31240/32.174 slug * 14.5939 kg/slug
P.mass   = (31240 / 32.174) * 14.5939;   % [kg]  ≈ 14170 kg

% Moments / products of inertia (slug-ft^2 -> kg-m^2, factor = 1.35582)
P.Jx     = 3650  * 1.35582;   % [kg-m^2]  ≈  4950 kg-m^2
P.Jy     = 80000 * 1.35582;   % [kg-m^2]  ≈ 108466 kg-m^2
P.Jz     = 82000 * 1.35582;   % [kg-m^2]  ≈ 111177 kg-m^2
P.Jxz    = 590   * 1.35582;   % [kg-m^2]  ≈   800 kg-m^2

% Derived inertia constants (Beard & McLain, Table 3.1)
P.Gamma  = P.Jx * P.Jz - P.Jxz^2;
P.Gamma1 = P.Jxz * (P.Jx - P.Jy + P.Jz) / P.Gamma;
P.Gamma2 = (P.Jz * (P.Jz - P.Jy) + P.Jxz^2) / P.Gamma;
P.Gamma3 = P.Jz / P.Gamma;
P.Gamma4 = P.Jxz / P.Gamma;
P.Gamma5 = (P.Jz - P.Jx) / P.Jy;
P.Gamma6 = P.Jxz / P.Jy;
P.Gamma7 = ((P.Jx - P.Jy) * P.Jx + P.Jxz^2) / P.Gamma;
P.Gamma8 = P.Jx / P.Gamma;

%% --- Wing / reference geometry ---
P.S_wing = 200.0 * 0.092903;   % [m^2]   wing area  ≈ 18.58 m^2
P.b      = 22.36 * 0.3048;     % [m]     wingspan   ≈ 6.81 m
P.c      = 10.27 * 0.3048;     % [m]     mean chord ≈ 3.13 m

%% --- Aerodynamic coefficients (dimensionless, direct from data file) ---
% Lift
P.CL_alpha = 3.5;    % /rad
P.CL_de    = 0.54;   % /rad

% Drag
P.CDo      = 0.095;
P.CD_alpha = 0.6;    % /rad

% Pitching moment
P.Cm_alpha = -1.2;   % /rad
P.Cm_adot  = 0.0;    % /rad
P.Cm_q     = -6.2;   % /rad
P.Cm_de    = -0.9;   % /rad

% Side force
P.CY_beta  = -1.4;   % /rad
P.CY_da    =  0.05;  % /rad
P.CY_dr    =  0.45;  % /rad

% Rolling moment
P.Cl_beta  = -0.01;  % /rad
P.Cl_p     = -0.35;  % /rad
P.Cl_r     =  0.04;  % /rad
P.Cl_da    = -0.06;  % /rad
P.Cl_dr    =  0.012; % /rad

% Yawing moment
P.Cn_beta  =  0.5;   % /rad
P.Cn_p     =  0.0;   % /rad
P.Cn_r     = -1.5;   % /rad
P.Cn_da    = -0.04;  % /rad
P.Cn_dr    = -0.3;   % /rad

%% --- Control surface limits ---
P.de_max   =  35 * pi/180;   % elevator max deflection  [rad]
P.de_min   = -15 * pi/180;   % elevator min deflection  [rad]
P.da_max   =  15 * pi/180;   % aileron  max deflection  [rad]
P.da_min   = -15 * pi/180;
P.dr_max   =  20 * pi/180;   % rudder   max deflection  [rad]
P.dr_min   = -20 * pi/180;

%% --- Environment ---
P.gravity  = 9.81;            % [m/s^2]
P.rho      = 1.225;           % [kg/m^3]  sea-level ISA

%% --- Initial conditions ---
P.pn0      = 0;       % North position   [m]
P.pe0      = 0;       % East  position   [m]
P.pd0      = -1000;   % Down  position   [m]  (negative = altitude 1000 m)
P.u0       = 100;     % body-x velocity  [m/s]
P.v0       = 0;       % body-y velocity  [m/s]
P.w0       = 0;       % body-z velocity  [m/s]
P.phi0     = 0;       % roll  angle      [rad]
P.theta0   = 0;       % pitch angle      [rad]
P.psi0     = 0;       % yaw   angle      [rad]
P.p0       = 0;       % roll  rate       [rad/s]
P.q0       = 0;       % pitch rate       [rad/s]
P.r0       = 0;       % yaw   rate       [rad/s]

fprintf('MAVParams_X15: X-15 parameters loaded.\n');
fprintf('  Mass  = %.1f kg\n',   P.mass);
fprintf('  Jx    = %.1f kg-m^2\n', P.Jx);
fprintf('  Jy    = %.1f kg-m^2\n', P.Jy);
fprintf('  Jz    = %.1f kg-m^2\n', P.Jz);
fprintf('  Jxz   = %.1f kg-m^2\n', P.Jxz);
fprintf('  S     = %.2f m^2\n',  P.S_wing);
fprintf('  b     = %.2f m\n',    P.b);
fprintf('  c     = %.2f m\n',    P.c);
