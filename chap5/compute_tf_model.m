% x_trim is the trimmed state,
% u_trim is the trimmed input
      

%  need to find out values for these
C_p_p = 1;
C_p_delta_a = 1;
a_theta1 = 1;
a_theta2 = 1;
a_theta3 = 1;

a_V1 = 1;
a_V2 = 1;
a_V3 = 1;
% 
a_beta1 = 1;
a_beta2 = 1;
Va_trim = 1;
theta_trim = 1;


%Defining the parameters:
a_phi1 = -0.25*MAV.rho*(Va)*MAV.S_wing*MAV.b*C_p_p*MAV.b;
a_phi2 = 0.5*MAV.rho*(Va^2)*MAV.S_wing*MAV.b*C_p_delta_a;

% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([MAV.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([a_beta2],[1,a_beta1]);
