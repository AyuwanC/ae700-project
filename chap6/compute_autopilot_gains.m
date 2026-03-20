addpath('../chap5')
load transfer_function_coef
addpath('../parameters')
simulation_parameters

% AP stands for autopilot
AP.gravity = 9.81;
AP.sigma = 1;
AP.Va0 = Va;
AP.Ts = ts_simulation;

%----------roll loop-------------
a_phi2 = 0.5*MAV.rho*Va_trim^2*MAV.S_wing*MAV.b*Cpda;

AP.roll_kp = 8;
AP.roll_kd = 2;
AP.roll_ki = 1;  % for disturbance rejection

%----------course loop-------------
AP.course_kp = 1;
AP.course_ki = 1;

%----------sideslip loop-------------
AP.sideslip_ki = 1;
AP.sideslip_kp = 1;

%----------yaw damper-------------
AP.yaw_damper_tau_r = 1;
AP.yaw_damper_kp = 1;

%----------pitch loop-------------
AP.pitch_kp = 1;
AP.pitch_kd = 1;
K_theta_DC = 1;

%----------altitude loop-------------
AP.altitude_kp = 1;
AP.altitude_ki = 1;
AP.altitude_zone = 500;

%---------airspeed hold using throttle---------------
AP.airspeed_throttle_kp = 1;
AP.airspeed_throttle_ki = 1;
