% compute_autopilot_gains.m  —  AE700 X-15 Autopilot Gains
% All sqrt() calls protected against complex results.

AP.gravity = MAV.gravity;
AP.Va0     = Va_trim;
AP.Ts      = SIM.ts_simulation;
AP.sigma   = 0.05;
AP.altitude_take_off_zone = 30;
AP.altitude_hold_zone     = 15;
AP.climb_angle            = 20*pi/180;

%% LATERAL
delta_a_max = 40*pi/180;  e_phi_max = 15*pi/180;  zeta_phi = 0.707;
AP.roll_kp  = sign(a_phi2) * delta_a_max / e_phi_max;
wn_phi      = sqrt(max(abs(a_phi2) * abs(AP.roll_kp), 0.01));
AP.roll_kd  = (2*zeta_phi*wn_phi - a_phi1) / a_phi2;
AP.roll_ki  = 0.01;

wn_chi = real(wn_phi) / 6;
AP.course_kp = 2*1.0*wn_chi * Va_trim / MAV.gravity;
AP.course_ki = wn_chi^2     * Va_trim / MAV.gravity;

wn_beta = 3;
AP.sideslip_kp = (2*0.707*wn_beta - a_beta1) / a_beta2;
AP.sideslip_ki =  wn_beta^2 / a_beta2;

AP.yaw_damper_tau_r = 0.1;
AP.yaw_damper_kp    = 0.5;

%% LONGITUDINAL
delta_e_max = 40*pi/180;  e_theta_max = 20*pi/180;  zeta_theta = 0.707;
AP.pitch_kp  = sign(a_theta3) * delta_e_max / e_theta_max;
wn_sq        = a_theta2 + AP.pitch_kp * a_theta3;
if wn_sq <= 0,  AP.pitch_kp = -AP.pitch_kp;  wn_sq = a_theta2 + AP.pitch_kp*a_theta3;  end
wn_theta     = sqrt(max(wn_sq, 0.01));
AP.pitch_kd  = (2*zeta_theta*wn_theta - a_theta1) / a_theta3;
AP.K_theta_DC = a_theta3*AP.pitch_kp / max(abs(a_theta2 + a_theta3*AP.pitch_kp), 1e-6);
AP.K_theta_DC = sign(AP.K_theta_DC) * max(abs(AP.K_theta_DC), 1e-3);

wn_h = wn_theta / 12;
AP.altitude_kp   = 2*1.0*wn_h / (abs(AP.K_theta_DC)*Va_trim);
AP.altitude_ki   = wn_h^2     / (abs(AP.K_theta_DC)*Va_trim);
AP.altitude_zone = 10;

wn_Va = 0.3;
a_V2_use = a_V2;
if abs(a_V2) < 1e-6,  a_V2_use = MAV.k_motor/MAV.mass;  end
AP.airspeed_throttle_kp = (2*0.707*wn_Va - a_V1) / a_V2_use;
AP.airspeed_throttle_ki =  wn_Va^2 / a_V2_use;

wn_Va2 = wn_theta / 7;
AP.airspeed_pitch_kp = (2*0.707*wn_Va2 - a_V1) / a_V3;
AP.airspeed_pitch_ki =  wn_Va2^2 / a_V3;

%% Strip any accidental complex parts
fnames = fieldnames(AP);
for k = 1:length(fnames)
    v = AP.(fnames{k});
    if isnumeric(v) && ~isreal(v)
        warning('AP.%s was complex — taking real part.', fnames{k});
        AP.(fnames{k}) = real(v);
    end
end

fprintf('\n=== Autopilot Gains (X-15 @ Va=%.1f m/s) ===\n', Va_trim);
fprintf('  roll kp=%.4f kd=%.4f | pitch kp=%.4f kd=%.4f K_DC=%.4f\n',...
    AP.roll_kp, AP.roll_kd, AP.pitch_kp, AP.pitch_kd, AP.K_theta_DC);
fprintf('  alt  kp=%.6f ki=%.8f | Va/thr kp=%.4f ki=%.6f\n',...
    AP.altitude_kp, AP.altitude_ki, AP.airspeed_throttle_kp, AP.airspeed_throttle_ki);
fprintf('  Va/pitch kp=%.4f ki=%.6f\n', AP.airspeed_pitch_kp, AP.airspeed_pitch_ki);