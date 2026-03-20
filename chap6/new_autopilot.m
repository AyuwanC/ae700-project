function y = autopilot(uu, AP)
% autopilot.m  —  AE700 X-15 Autopilot
% Implements lateral and longitudinal autopilots per Beard & McLain Ch.6
%
% Lateral loops:   course -> roll -> aileron,  sideslip -> rudder
% Longitudinal:    altitude -> pitch -> elevator  +  airspeed -> throttle
%                  airspeed -> pitch (used during climb/descend)

    % ---------- unpack inputs ----------
    NN = 0;
    h        = uu(3+NN);
    Va       = uu(4+NN);
    beta     = uu(6+NN);
    phi      = uu(7+NN);
    theta    = uu(8+NN);
    chi      = uu(9+NN);
    p        = uu(10+NN);
    q        = uu(11+NN);
    NN = NN + 19;
    Va_c     = uu(1+NN);
    h_c      = uu(2+NN);
    chi_c    = uu(3+NN);
    NN = NN + 3;
    t        = uu(1+NN);

    init = (t == 0);

    % =================================================================
    % LATERAL AUTOPILOT
    % =================================================================
    phi_c   = course_hold(chi_c, chi, init, AP);
    delta_a = roll_hold(phi_c, phi, p, init, AP);
    delta_r = sideslip_hold(beta, init, AP);

    % =================================================================
    % LONGITUDINAL AUTOPILOT  —  altitude state machine
    % =================================================================
    persistent alt_state;
    if init || isempty(alt_state)
        alt_state = 1;
    end

    if     h <= AP.altitude_take_off_zone,          alt_state = 1;
    elseif h <= h_c - AP.altitude_hold_zone,        alt_state = 2;
    elseif h >= h_c + AP.altitude_hold_zone,        alt_state = 3;
    else                                             alt_state = 4;
    end

    switch alt_state
        case 1   % take-off
            theta_c = AP.climb_angle;
            delta_t = 0.8;
        case 2   % climb  — airspeed held with pitch, throttle max
            theta_c = airspeed_with_pitch(Va_c, Va, init, AP);
            delta_t = 1.0;
        case 3   % descend — airspeed held with pitch, throttle idle
            theta_c = airspeed_with_pitch(Va_c, Va, init, AP);
            delta_t = 0.1;
        case 4   % altitude hold — pitch holds altitude, throttle holds airspeed
            theta_c = altitude_hold(h_c, h, init, AP);
            delta_t = airspeed_with_throttle(Va_c, Va, init, AP);
    end

    delta_e = pitch_hold(theta_c, theta, q, AP);
    delta_t = sat(delta_t, 1, 0);

    % =================================================================
    % OUTPUTS
    % =================================================================
    delta     = [delta_e; delta_a; delta_r; delta_t];
    x_command = [0; 0; h_c; Va_c; 0; 0; phi_c; theta_c; chi_c; 0; 0; 0];
    y = [delta; x_command];
end


%==================================================================
% LATERAL CONTROLLERS
%==================================================================

function phi_c = course_hold(chi_c, chi, init, AP)
    persistent integrator;
    if init || isempty(integrator), integrator = 0; end
    err        = chi_c - chi;
    integrator = integrator + AP.Ts * err;
    phi_c      = AP.course_kp * err + AP.course_ki * integrator;
    phi_c      = sat(phi_c, 40*pi/180, -40*pi/180);
end

function delta_a = roll_hold(phi_c, phi, p, init, AP)
    persistent integrator;
    if init || isempty(integrator), integrator = 0; end
    err        = phi_c - phi;
    integrator = integrator + AP.Ts * err;
    delta_a    = AP.roll_kp * err - AP.roll_kd * p + AP.roll_ki * integrator;
    delta_a    = sat(delta_a, 40*pi/180, -40*pi/180);
end

function delta_r = sideslip_hold(beta, init, AP)
    persistent integrator;
    if init || isempty(integrator), integrator = 0; end
    integrator = integrator + AP.Ts * (-beta);
    delta_r    = -AP.sideslip_kp * beta + AP.sideslip_ki * integrator;
    delta_r    = sat(delta_r, 40*pi/180, -40*pi/180);
end


%==================================================================
% LONGITUDINAL CONTROLLERS
%==================================================================

function delta_e = pitch_hold(theta_c, theta, q, AP)
    delta_e = AP.pitch_kp * (theta_c - theta) - AP.pitch_kd * q;
    delta_e = sat(delta_e, 40*pi/180, -40*pi/180);
end

function theta_c = altitude_hold(h_c, h, init, AP)
    persistent integrator;
    if init || isempty(integrator), integrator = 0; end
    err        = h_c - h;
    integrator = integrator + AP.Ts * err;
    theta_c    = AP.altitude_kp * err + AP.altitude_ki * integrator;
    theta_c    = sat(theta_c, 40*pi/180, -40*pi/180);
end

function delta_t = airspeed_with_throttle(Va_c, Va, init, AP)
    persistent integrator;
    if init || isempty(integrator), integrator = 0; end
    err        = Va_c - Va;
    integrator = integrator + AP.Ts * err;
    delta_t    = AP.airspeed_throttle_kp * err + AP.airspeed_throttle_ki * integrator;
    delta_t    = sat(delta_t, 1, 0);
end

function theta_c = airspeed_with_pitch(Va_c, Va, init, AP)
    persistent integrator;
    if init || isempty(integrator), integrator = 0; end
    err        = Va_c - Va;
    integrator = integrator + AP.Ts * err;
    % negative sign: nose down increases airspeed
    theta_c    = -(AP.airspeed_pitch_kp * err + AP.airspeed_pitch_ki * integrator);
    theta_c    = sat(theta_c, 40*pi/180, -40*pi/180);
end


%==================================================================
function out = sat(in, hi, lo)
    out = max(lo, min(hi, in));
end