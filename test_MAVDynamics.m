% test_MAVDynamics.m
% Verification script for the X-15 6-DOF equations of motion.
%
% Tests from Project Section 2:
%   (ii)  Apply individual forces/moments and verify expected motion.
%   (iii) Verify Jxz coupling: zero Jxz => no roll/yaw coupling;
%         non-zero Jxz => coupling present.
%
% Run this script from the MATLAB command window.
% It does NOT require Simulink – it integrates the EOMs directly with ode45.

clear; clc; close all;
MAVParams_X15;   % load parameter struct P into workspace

fprintf('\n=== X-15 6-DOF EOM Verification ===\n\n');

% -------------------------------------------------------------------------
% Helper: right-hand side of state equations (same logic as S-function)
% -------------------------------------------------------------------------
function xdot = eom(~, x, fx, fy, fz, ell, m_moment, n_moment, P)
    u_b=x(4); v_b=x(5); w_b=x(6);
    phi=x(7); theta=x(8); psi=x(9); %#ok<NASGU>
    p=x(10); q=x(11); r=x(12);

    cphi=cos(phi); sphi=sin(phi);
    cth=cos(theta); sth=sin(theta); tth=tan(theta);
    cpsi=cos(psi); spsi=sin(psi);

    % Position kinematics
    R_bi=[cth*cpsi, sphi*sth*cpsi-cphi*spsi, cphi*sth*cpsi+sphi*spsi;
          cth*spsi, sphi*sth*spsi+cphi*cpsi, cphi*sth*spsi-sphi*cpsi;
         -sth,      sphi*cth,                cphi*cth];
    pd=R_bi*[u_b;v_b;w_b];

    % Attitude kinematics
    T=[1, sphi*tth, cphi*tth; 0, cphi, -sphi; 0, sphi/cth, cphi/cth];
    ed=T*[p;q;r];

    % Translational dynamics
    u_dot = r*v_b - q*w_b + fx/P.mass;
    v_dot = p*w_b - r*u_b + fy/P.mass;
    w_dot = q*u_b - p*v_b + fz/P.mass;

    % Rotational dynamics
    p_dot = P.Gamma1*p*q - P.Gamma2*q*r + P.Gamma3*ell + P.Gamma4*n_moment;
    q_dot = P.Gamma5*p*r - P.Gamma6*(p^2-r^2)          + m_moment/P.Jy;
    r_dot = P.Gamma7*p*q - P.Gamma1*q*r + P.Gamma4*ell + P.Gamma8*n_moment;

    xdot=[pd; u_dot; v_dot; w_dot; ed; p_dot; q_dot; r_dot];
end

% Initial state (at trim: level flight at 1000 m, 100 m/s)
x0 = [P.pn0; P.pe0; P.pd0; P.u0; P.v0; P.w0;
      P.phi0; P.theta0; P.psi0; P.p0; P.q0; P.r0];

tspan = [0, 5];   % 5-second simulation

% =========================================================================
% TEST 1: Pure fz force (downward body force -> pitch nose down)
% =========================================================================
fprintf('--- Test 1: Pure body-z force (fz = +50000 N) ---\n');
fprintf('  Expected: pitch-down motion (negative theta), nose pitches down\n');
fz_test = 50000;   % body z force [N] (positive = down in NED body frame)
f = @(t,x) eom(t, x, 0, 0, fz_test, 0, 0, 0, P);
[t1, X1] = ode45(f, tspan, x0);
fprintf('  theta at t=5s: %.3f deg  (should be negative for fz > 0)\n\n',...
        X1(end,8)*180/pi);

% =========================================================================
% TEST 2: Pure pitching moment m > 0 (nose-up)
% =========================================================================
fprintf('--- Test 2: Pure pitching moment (m = +5e6 N-m) ---\n');
fprintf('  Expected: nose-up pitch rate (positive q), theta increases\n');
f = @(t,x) eom(t, x, 0, 0, 0, 0, 5e6, 0, P);
[t2, X2] = ode45(f, tspan, x0);
fprintf('  q   at t=5s: %.3f rad/s  (should be positive)\n', X2(end,11));
fprintf('  theta at t=5s: %.3f deg  (should be positive)\n\n', X2(end,8)*180/pi);

% =========================================================================
% TEST 3: Pure rolling moment ell > 0 (right-roll)
% =========================================================================
fprintf('--- Test 3: Pure rolling moment (ell = +5e6 N-m), Jxz non-zero ---\n');
fprintf('  Expected: roll rate p > 0 AND yaw coupling r ≠ 0 (due to Jxz)\n');
f = @(t,x) eom(t, x, 0, 0, 0, 5e6, 0, 0, P);
[t3, X3] = ode45(f, tspan, x0);
fprintf('  p   at t=5s: %.4f rad/s\n', X3(end,10));
fprintf('  r   at t=5s: %.4f rad/s  (non-zero because Jxz = %.1f kg-m^2)\n\n',...
        X3(end,12), P.Jxz);

% =========================================================================
% TEST 4: Jxz = 0 check – roll/yaw decoupling
% =========================================================================
fprintf('--- Test 4: Jxz set to ZERO, rolling moment only ---\n');
fprintf('  Expected: p > 0 but r ≈ 0 (no coupling)\n');
P_decoupled = P;
P_decoupled.Jxz   = 0;
P_decoupled.Gamma  = P_decoupled.Jx * P_decoupled.Jz;
P_decoupled.Gamma1 = 0;
P_decoupled.Gamma2 = P_decoupled.Jz * (P_decoupled.Jz - P_decoupled.Jy) / P_decoupled.Gamma;
P_decoupled.Gamma3 = P_decoupled.Jz / P_decoupled.Gamma;
P_decoupled.Gamma4 = 0;
P_decoupled.Gamma5 = (P_decoupled.Jz - P_decoupled.Jx) / P_decoupled.Jy;
P_decoupled.Gamma6 = 0;
P_decoupled.Gamma7 = (P_decoupled.Jx - P_decoupled.Jy) * P_decoupled.Jx / P_decoupled.Gamma;
P_decoupled.Gamma8 = P_decoupled.Jx / P_decoupled.Gamma;

f = @(t,x) eom(t, x, 0, 0, 0, 5e6, 0, 0, P_decoupled);
[t4, X4] = ode45(f, tspan, x0);
fprintf('  p   at t=5s: %.4f rad/s  (non-zero, roll response)\n', X4(end,10));
fprintf('  r   at t=5s: %.6f rad/s  (should be ~0)\n\n', X4(end,12));

% =========================================================================
% TEST 5: Pure yawing moment n > 0 (Jxz = 0) – no roll coupling
% =========================================================================
fprintf('--- Test 5: Pure yawing moment (n = +5e6 N-m), Jxz = 0 ---\n');
fprintf('  Expected: r > 0 but p ≈ 0 (no coupling)\n');
f = @(t,x) eom(t, x, 0, 0, 0, 0, 0, 5e6, P_decoupled);
[t5, X5] = ode45(f, tspan, x0);
fprintf('  r   at t=5s: %.4f rad/s  (non-zero, yaw response)\n', X5(end,12));
fprintf('  p   at t=5s: %.6f rad/s  (should be ~0)\n\n', X5(end,10));

% =========================================================================
% PLOTS
% =========================================================================
figure('Name','Test 1 – fz force','NumberTitle','off');
subplot(2,1,1); plot(t1, X1(:,8)*180/pi,'b','LineWidth',1.5);
xlabel('Time [s]'); ylabel('\theta [deg]'); title('Test 1: Pure f_z force');
grid on;
subplot(2,1,2); plot(t1, X1(:,11),'r','LineWidth',1.5);
xlabel('Time [s]'); ylabel('q [rad/s]'); grid on;

figure('Name','Test 2 – pitching moment','NumberTitle','off');
subplot(2,1,1); plot(t2, X2(:,8)*180/pi,'b','LineWidth',1.5);
xlabel('Time [s]'); ylabel('\theta [deg]'); title('Test 2: Pure pitching moment');
grid on;
subplot(2,1,2); plot(t2, X2(:,11),'r','LineWidth',1.5);
xlabel('Time [s]'); ylabel('q [rad/s]'); grid on;

figure('Name','Test 3 vs 4 – Jxz coupling','NumberTitle','off');
subplot(2,1,1);
plot(t3, X3(:,10),'b','LineWidth',1.5); hold on;
plot(t4, X4(:,10),'b--','LineWidth',1.5);
legend('J_{xz} \neq 0','J_{xz} = 0');
xlabel('Time [s]'); ylabel('p [rad/s]');
title('Roll rate: effect of J_{xz} (rolling moment applied)'); grid on;

subplot(2,1,2);
plot(t3, X3(:,12),'r','LineWidth',1.5); hold on;
plot(t4, X4(:,12),'r--','LineWidth',1.5);
legend('J_{xz} \neq 0','J_{xz} = 0');
xlabel('Time [s]'); ylabel('r [rad/s]');
title('Yaw rate coupling due to J_{xz}'); grid on;

fprintf('All tests complete. Review the figures and printed values.\n');
fprintf('Key check: Test 3 r ≠ 0, Test 4 r ≈ 0  (verifies Jxz coupling).\n');
