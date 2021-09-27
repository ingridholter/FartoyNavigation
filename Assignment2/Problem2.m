%% Autopilot for course hold using aileron and successive loop closure

a_phi1 = 2.87;
a_phi2= -0.65;

deg2rad = pi/180;
rad2deg= 180/pi;

g= 9.81;
V_g = 580;
zeta_phi =0.707;
omega_nphi = sqrt(abs(a_phi2)*2);

k_pphi = -2;
k_dphi = (2*zeta_phi*omega_nphi-a_phi1)/a_phi2;
k_iphi =-0.6;


W_chi = 10;
zeta_chi = 0.707;
omega_nchi = (1/W_chi)*omega_nphi;

k_pchi= (2*zeta_chi*omega_nchi*V_g)/g;

k_ichi = (omega_nchi^2*V_g)/g;

d= 1.5*deg2rad;

%% timeseries
h=30;
chi_ref = [0*ones(1,h) 10*ones(1,h) 15*ones(1,h) 20*ones(1,h) 5*ones(1,h) -10*ones(1,h)].*deg2rad;
%chi_ref = [0*ones(1,h) 10*ones(1,h) 10*ones(1,h) 10*ones(1,h) 0*ones(1,h) 0*ones(1,h)].*deg2rad;
T=h*6;
time_steps = [1:1:T]';
chi_c_ts = timeseries(chi_ref, time_steps);


%% root locus

k = [-3.22:0.05:0]; %-0.6 is nice

sys = tf([a_phi2],[1 (a_phi1+a_phi2*k_dphi) k_pphi*a_phi2 0]);
%rlocus(sys,k);


%% 2f

A = [-0.322 0.052 0.028 -1.12 0.002;
     0 0 1 -0.001 0;
     -10.6 0 -2.87 0.46 -0.65;
     6.87 0 -0.04 -0.32 -0.02;
     0 0 0 0 -7.5];

B= [0 0 0 0 7.5]';

C = [1 0 0 0 0;
    0 1 0 0 0;
    0 0 1 0 0;
    0 0 0 1 0];

D = zeros(4,1);

