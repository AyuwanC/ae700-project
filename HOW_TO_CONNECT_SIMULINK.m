% HOW_TO_CONNECT_SIMULINK.m
% Step-by-step guide for wiring MAVDynamics.m into Simulink
% and connecting it to the drawAircraft animation block.
%
% =========================================================================
% PREREQUISITES
% =========================================================================
% 1. All files in the same folder:
%      MAVParams_X15.m    – aircraft parameters (run this first)
%      MAVDynamics.m      – S-function (6-DOF EOM)
%      drawAircraft.m     – animation function (provided)
%      mavsim_chap2.slx   – template Simulink model (provided)
%
% 2. Before opening Simulink, run in MATLAB Command Window:
%      MAVParams_X15        % loads struct P into base workspace
%
% =========================================================================
% SIMULINK MODEL SETUP (mavsim_chap2.slx or new model)
% =========================================================================
%
% Block 1 – S-Function (6-DOF EOM)
%   • Library Browser -> User-Defined Functions -> S-Function
%   • S-function name:       MAVDynamics
%   • S-function parameters: P          <-- the struct from MAVParams_X15
%
%   Inputs  (6-element vector mux):
%     [fx; fy; fz; ell; m_moment; n_moment]
%
%   Outputs (12-element vector):
%     [pn; pe; pd; u; v; w; phi; theta; psi; p; q; r]
%
% Block 2 – Fcn / MATLAB Function block for gravity + test forces
%   For Part 2 testing, use Constant blocks to supply known forces/moments:
%     • A "Constant" block set to [0; 0; P.mass*9.81; 0; 0; 0] gives
%       gravity in body frame (level flight, no rotation assumed).
%     • Use a "Manual Switch" to toggle between gravity-only and
%       individual test inputs.
%
% Block 3 – Mux (12->13)
%   The drawAircraft function expects 13 inputs:
%     [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r, t]
%   Add a Clock block and mux its output as the 13th element.
%
% Block 4 – MATLAB Function (animation)
%   • Library Browser -> User-Defined Functions -> MATLAB Function
%   • Paste (or call) drawAircraft(uu) in the function body.
%   OR use the S-function approach already in mavsim_chap2.slx.
%
% =========================================================================
% WIRING DIAGRAM  (text representation)
% =========================================================================
%
%  [Constant: forces/moments]
%          |
%         [6x1 Mux]
%          |
%          v
%  [S-Function: MAVDynamics(P)]  ---> [12x1 state output]
%          |                                    |
%          |                           [Mux with Clock]
%          |                                    |
%          +---------------------> [drawAircraft animation block]
%
% =========================================================================
% SIMULATION SETTINGS
% =========================================================================
%   Solver:     ode45 (variable-step) or ode4 (fixed-step, e.g. dt=0.01 s)
%   Stop time:  10 s (for initial tests)
%
% =========================================================================
% VERIFICATION TESTS (Section 2, parts ii and iii)
% =========================================================================
%
% Test A – Pure pitching moment (q should grow, theta increases):
%   forces/moments = [0; 0; P.mass*9.81; 0; 5e6; 0]
%   Expect: nose pitches up, theta increases.
%
% Test B – Pure rolling moment with Jxz ≠ 0:
%   forces/moments = [0; 0; P.mass*9.81; 5e6; 0; 0]
%   Expect: roll (phi) increases AND yaw coupling (psi changes slightly).
%
% Test C – Pure rolling moment with Jxz = 0 (edit P.Jxz=0 and recompute Gammas):
%   Expect: roll only, NO yaw coupling.
%
% Test D – Order of rotation vs translation (Section 1, part iv):
%   In drawSpacecraftBody(), swap the two lines:
%     V = rotate(V', phi, theta, psi)';
%     V = translate(V', pn, pe, pd)';
%   -> translate THEN rotate: the aircraft spins around inertial origin
%      (wrong). Rotate THEN translate: correct body-fixed rotation.
%
% =========================================================================
% QUICK STANDALONE TEST (no Simulink needed)
% =========================================================================
%   Run:   test_MAVDynamics
%   This integrates the EOMs with ode45 directly and prints/plots results.

disp('Read this file for Simulink connection instructions.');
