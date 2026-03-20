% forces_moments.m
%   Computes the forces and moments acting on the airframe.
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%
%   Aircraft: North American X-15
%   NOTE: X-15 is rocket-powered. Propulsion is modelled as a simple
%   constant-maximum-thrust engine scaled by throttle delta_t in [0,1].
%   There is no propeller, so S_prop, K_V, KQ etc. are not used.

function out = forces_moments(x, delta, wind, MAV)

    % --- Relabel states ---
    pn      = x(1);   %#ok<NASGU>
    pe      = x(2);   %#ok<NASGU>
    pd      = x(3);   %#ok<NASGU>
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);   %#ok<NASGU>
    p       = x(10);
    q       = x(11);
    r       = x(12);

    % --- Relabel control inputs ---
    delta_e = delta(1);   % elevator deflection  [rad]
    delta_a = delta(2);   % aileron  deflection  [rad]
    delta_r = delta(3);   % rudder   deflection  [rad]
    delta_t = delta(4);   % throttle             [0,1]

    % --- Relabel wind inputs ---
    w_ns  = wind(1);   % steady wind - North  [m/s]
    w_es  = wind(2);   % steady wind - East   [m/s]
    w_ds  = wind(3);   % steady wind - Down   [m/s]
    u_wg  = wind(4);   % gust along body x    [m/s]
    v_wg  = wind(5);   % gust along body y    [m/s]
    w_wg  = wind(6);   % gust along body z    [m/s]

    % --- Wind in NED (steady + gust, resolved into body frame by caller) ---
    w_n = w_ns + u_wg;
    w_e = w_es + v_wg;
    w_d = w_ds + w_wg;

    % --- Air-relative velocity components ---
    ur = u - w_n;
    vr = v - w_e;
    wr = w - w_d;

    % --- Airspeed, angle of attack, sideslip ---
    Va    = sqrt(ur^2 + vr^2 + wr^2);
    alpha = atan2(wr, ur);          % angle of attack  [rad]
    beta  = asin(vr / Va);          % sideslip angle   [rad]

    % ======================================================================
    % Aerodynamic forces (body frame)
    % Using linearised coefficient model (Beard & McLain Ch.4)
    % ======================================================================
    q_bar = 0.5 * MAV.rho * Va^2;   % dynamic pressure [Pa]

    % -- Lift and Drag (wind-frame, then rotated to body) --
    CL = MAV.C_L_0 + MAV.C_L_alpha * alpha ...
       + (MAV.C_L_q * MAV.c * q) / (2 * Va) ...
       + MAV.C_L_delta_e * delta_e;

    CD = MAV.C_D_0 + MAV.C_D_alpha * alpha ...
       + (MAV.C_D_q * MAV.c * q) / (2 * Va) ...
       + MAV.C_D_delta_e * delta_e;

    % Rotate wind-frame lift/drag into body-frame x and z
    x_aero =  q_bar * MAV.S_wing * (-CD * cos(alpha) + CL * sin(alpha));
    z_aero =  q_bar * MAV.S_wing * (-CD * sin(alpha) - CL * cos(alpha));

    % -- Side force --
    y_aero = q_bar * MAV.S_wing * ( ...
        MAV.C_Y_0 ...
      + MAV.C_Y_beta  * beta ...
      + (MAV.C_Y_p * MAV.b * p) / (2 * Va) ...
      + (MAV.C_Y_r * MAV.b * r) / (2 * Va) ...
      + MAV.C_Y_delta_a * delta_a ...
      + MAV.C_Y_delta_r * delta_r );

    % ======================================================================
    % Propulsion forces (X-15 rocket engine)
    %   F_prop = k_motor * delta_t  [N], along body x-axis only.
    %   No side/normal thrust component.
    %   No reactive propeller torque.
    % ======================================================================
    x_prop = MAV.k_motor * delta_t;
    y_prop = 0;
    z_prop = 0;

    % ======================================================================
    % Gravity (resolved into body frame via Euler angles)
    % ======================================================================
    x_grav = -MAV.mass * MAV.gravity * sin(theta);
    y_grav =  MAV.mass * MAV.gravity * cos(theta) * sin(phi);
    z_grav =  MAV.mass * MAV.gravity * cos(theta) * cos(phi);

    % ======================================================================
    % Total forces
    % ======================================================================
    Force(1) = x_grav + x_prop + x_aero;
    Force(2) = y_grav + y_prop + y_aero;
    Force(3) = z_grav + z_prop + z_aero;

    % ======================================================================
    % Aerodynamic moments (body frame)
    % ======================================================================
    % Rolling moment
    Torque(1) = q_bar * MAV.S_wing * MAV.b * ( ...
        MAV.C_ell_0 ...
      + MAV.C_ell_beta  * beta ...
      + (MAV.C_ell_p * MAV.b * p) / (2 * Va) ...
      + (MAV.C_ell_r * MAV.b * r) / (2 * Va) ...
      + MAV.C_ell_delta_a * delta_a ...
      + MAV.C_ell_delta_r * delta_r );

    % Pitching moment
    Torque(2) = q_bar * MAV.S_wing * MAV.c * ( ...
        MAV.C_m_0 ...
      + MAV.C_m_alpha * alpha ...
      + (MAV.C_m_q * MAV.c * q) / (2 * Va) ...
      + MAV.C_m_delta_e * delta_e );

    % Yawing moment
    Torque(3) = q_bar * MAV.S_wing * MAV.b * ( ...
        MAV.C_n_0 ...
      + MAV.C_n_beta  * beta ...
      + (MAV.C_n_p * MAV.b * p) / (2 * Va) ...
      + (MAV.C_n_r * MAV.b * r) / (2 * Va) ...
      + MAV.C_n_delta_a * delta_a ...
      + MAV.C_n_delta_r * delta_r );

    % No propeller reactive torque for rocket-powered X-15
    % (senior's code had: -ktp*komega^2*delta_t^2 on Torque(1) — not applicable)

    % ======================================================================
    % Output: [fx; fy; fz; ell; m; n; Va; alpha; beta; wn; we; wd]
    % ======================================================================
    out = [Force(1);  Force(2);  Force(3); ...
           Torque(1); Torque(2); Torque(3); ...
           Va; alpha; beta; ...
           w_n; w_e; w_d];
end