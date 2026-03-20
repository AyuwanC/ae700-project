function [sys, x0, str, ts] = MAVDynamics(t, x, u, flag, P)
% MAVDynamics  –  6-DOF MAV equations of motion (Beard & McLain, Eqs 3.14-3.17)
%
% S-function interface:
%
%   Inputs  u(1..6) = [fx; fy; fz; ell; m; n]
%                      Forces (N) and moments (N-m) in the BODY frame.
%
%   States  x(1..12) = [pn; pe; pd; u_b; v_b; w_b; phi; theta; psi; p; q; r]
%                       positions [m], body velocities [m/s],
%                       Euler angles [rad], body angular rates [rad/s]
%
%   Output  sys = full state vector (12 x 1)
%
% Usage in Simulink:
%   Add an S-Function block, set the name to "MAVDynamics", and pass
%   the parameter struct P (from MAVParams_X15) as the S-function parameter.
%
% Reference:
%   Beard, R.W. & McLain, T.W., "Small Unmanned Aircraft", 2012,
%   Equations (3.14) through (3.17).

switch flag

  %% --- Initialisation ---
  case 0
    [sys, x0, str, ts] = mdlInitializeSizes(P);

  %% --- Derivatives ---
  case 1
    sys = mdlDerivatives(t, x, u, P);

  %% --- Outputs ---
  case 3
    sys = mdlOutputs(t, x, u);

  %% --- Unused flags ---
  case {2, 4, 9}
    sys = [];

  otherwise
    error(['Unhandled flag = ', num2str(flag)]);
end

end   % end of main S-function

% =========================================================================
%                         mdlInitializeSizes
% =========================================================================
function [sys, x0, str, ts] = mdlInitializeSizes(P)

sizes = simsizes;
sizes.NumContStates  = 12;   % continuous states
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;   % output = full state
sizes.NumInputs      = 6;    % [fx fy fz ell m n]
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
str = [];
ts  = [0, 0];   % continuous sample time

% Initial state vector from parameter struct
x0 = [...
    P.pn0;    % 1  north position  [m]
    P.pe0;    % 2  east  position  [m]
    P.pd0;    % 3  down  position  [m]
    P.u0;     % 4  body u velocity [m/s]
    P.v0;     % 5  body v velocity [m/s]
    P.w0;     % 6  body w velocity [m/s]
    P.phi0;   % 7  roll  angle     [rad]
    P.theta0; % 8  pitch angle     [rad]
    P.psi0;   % 9  yaw   angle     [rad]
    P.p0;     % 10 roll  rate      [rad/s]
    P.q0;     % 11 pitch rate      [rad/s]
    P.r0;     % 12 yaw   rate      [rad/s]
    ];
end

% =========================================================================
%                         mdlDerivatives
% =========================================================================
function sys = mdlDerivatives(~, x, uu, P)

% --- Unpack states ---
pn    = x(1);   %#ok<NASGU>  north position [m]
pe    = x(2);   %#ok<NASGU>  east  position [m]
pd    = x(3);   %#ok<NASGU>  down  position [m]
u_b   = x(4);   % body-frame x velocity [m/s]
v_b   = x(5);   % body-frame y velocity [m/s]
w_b   = x(6);   % body-frame z velocity [m/s]
phi   = x(7);   % roll  [rad]
theta = x(8);   % pitch [rad]
psi   = x(9);   % yaw   [rad]
p     = x(10);  % roll  rate [rad/s]
q     = x(11);  % pitch rate [rad/s]
r     = x(12);  % yaw   rate [rad/s]

% --- Unpack inputs (forces & moments in body frame) ---
fx  = uu(1);   % [N]
fy  = uu(2);   % [N]
fz  = uu(3);   % [N]
ell = uu(4);   % rolling  moment [N-m]
m   = uu(5);   % pitching moment [N-m]
n   = uu(6);   % yawing   moment [N-m]

% --- Trig shorthands ---
cphi = cos(phi);   sphi = sin(phi);
cth  = cos(theta); sth  = sin(theta); tth = tan(theta);
cpsi = cos(psi);   spsi = sin(psi);

% -----------------------------------------------------------------------
% Eq. 3.14  –  Position kinematics  (body -> NED)
%   [pn_dot; pe_dot; pd_dot] = R_v_b2i * [u; v; w]
% -----------------------------------------------------------------------
% Rotation matrix from body to inertial (NED):
%   R = Rz(psi) * Ry(theta) * Rx(phi)
R_bi = [...
    cth*cpsi,  sphi*sth*cpsi - cphi*spsi,  cphi*sth*cpsi + sphi*spsi;
    cth*spsi,  sphi*sth*spsi + cphi*cpsi,  cphi*sth*spsi - sphi*cpsi;
    -sth,      sphi*cth,                   cphi*cth                   ];

pos_dot = R_bi * [u_b; v_b; w_b];
pn_dot  = pos_dot(1);
pe_dot  = pos_dot(2);
pd_dot  = pos_dot(3);

% -----------------------------------------------------------------------
% Eq. 3.15  –  Attitude kinematics  (body rates -> Euler angle rates)
%   Note: singularity at theta = ±90 deg (not expected for most flight)
% -----------------------------------------------------------------------
% Kinematic matrix T such that [phi_dot; theta_dot; psi_dot] = T * [p; q; r]
T_euler = [...
    1,   sphi*tth,   cphi*tth;
    0,   cphi,      -sphi;
    0,   sphi/cth,   cphi/cth ];

euler_dot = T_euler * [p; q; r];
phi_dot   = euler_dot(1);
theta_dot = euler_dot(2);
psi_dot   = euler_dot(3);

% -----------------------------------------------------------------------
% Eq. 3.16  –  Translational (velocity) dynamics
%   Newton's 2nd law in body frame:
%   u_dot = r*v - q*w + fx/m
%   v_dot = p*w - r*u + fy/m
%   w_dot = q*u - p*v + fz/m
% -----------------------------------------------------------------------
u_dot = r*v_b - q*w_b + fx/P.mass;
v_dot = p*w_b - r*u_b + fy/P.mass;
w_dot = q*u_b - p*v_b + fz/P.mass;

% -----------------------------------------------------------------------
% Eq. 3.17  –  Rotational (angular rate) dynamics
%   Using Gamma constants (Table 3.1, Beard & McLain) that account for
%   the non-zero product of inertia Jxz:
%
%   p_dot = Gamma1*p*q - Gamma2*q*r + Gamma3*ell + Gamma4*n
%   q_dot = Gamma5*p*r - Gamma6*(p^2 - r^2) + m/Jy
%   r_dot = Gamma7*p*q - Gamma1*q*r + Gamma4*ell + Gamma8*n
% -----------------------------------------------------------------------
p_dot = P.Gamma1*p*q - P.Gamma2*q*r + P.Gamma3*ell + P.Gamma4*n;
q_dot = P.Gamma5*p*r - P.Gamma6*(p^2 - r^2)        + m/P.Jy;
r_dot = P.Gamma7*p*q - P.Gamma1*q*r + P.Gamma4*ell + P.Gamma8*n;

% --- Pack derivative vector ---
sys = [pn_dot; pe_dot; pd_dot;
       u_dot;  v_dot;  w_dot;
       phi_dot; theta_dot; psi_dot;
       p_dot;  q_dot;  r_dot];
end

% =========================================================================
%                         mdlOutputs
% =========================================================================
function sys = mdlOutputs(~, x, ~)
    sys = x;   % output the full state vector
end
