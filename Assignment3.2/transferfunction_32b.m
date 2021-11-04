
m = 17.0677e6;          % mass (kg)
Iz = 2.1732e10;         % yaw moment of inertia (kg m^2)
xg = -3.7;              % CG x-ccordinate (m)
L = 161;                % length (m)
B = 21.8;               % beam (m)
T = 8.9;                % draft (m)
KT = 0.7;               % propeller coefficient (-)
Dia = 3.3;              % propeller diameter (m)
rho = 1025;             % density of water (m/s^3)

Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;
MA = -[ Xudot 0 0; 0 Yvdot Yrdot; 0 Nvdot Nrdot ];
T1 = 20; T2 = 20; T6 = 10;
Xu = -(m-Xudot)/T1;
Yv = -(m-Yvdot)/T2;
Nr = -(Iz-Nrdot)/T6;

% rigid-body mass matrix
MRB = [ m 0    0 
        0 m    m*xg
        0 m*xg Iz ];
Minv = inv(MRB + MA);




% ship parameters 
m = 17.0677e6;          % mass (kg)
Iz = 2.1732e10;         % yaw moment of inertia (kg m^2)
xg = -3.7;              % CG x-ccordinate (m)
L = 161;                % length (m)
B = 21.8;               % beam (m)
T = 8.9;                % draft (m)
KT = 0.7;               % propeller coefficient (-)
Dia = 3.3;              % propeller diameter (m)
rho = 1025;             % density of water (m/s^3)
u_d = 7;

% rudder limitations
delta_max  = 40 * pi/180;        % max rudder angle      (rad)
Ddelta_max = 5  * pi/180;        % max rudder derivative (rad/s)

% added mass matrix
Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;
MA = -[Yvdot Yrdot;Nvdot Nrdot ];

% rigid-body mass matrix
MRB = [m    m*xg
        m*xg Iz ];
Minv = inv(MRB + MA);


% rudder coefficients
b = 2;
AR = 8;
CB = 0.8;

lambda = b^2 / AR;
tR = 0.45 - 0.28*CB;
CN = 6.13*lambda / (lambda + 2.25);
aH = 0.75;
xH = -0.4 * L;
xR = -0.5 * L;

% input matrix
t_thr = 0.05;                                        % thrust deduction number
X_delta2 = 0.5 * (1 - tR) * rho * AR * CN;           % rudder coefficients (Section 9.5)
Y_delta = 0.25 * (1 + aH) * rho * AR * CN; 
N_delta = 0.25 * (xR + aH*xH) * rho * AR * CN;   

% state-dependent time-varying matrices
CRB_star = [0   m*u_d;
            0   m*xg*u_d];

CA_star = [0                        -Xudot*u_d;
           -Yvdot*u_d+Xudot*u_d     -Yrdot*u_d];

b_star = 2*u_d *[-Y_delta -N_delta]';
D = -[Yv 0;
    0  Nr];
N = CRB_star+ CA_star + D;

C = [0 1];
D = 0;

[num,den] = ss2tf(-Minv*N, Minv*b_star,C,D);
K = 6.146*10.^(-6) /0.0008203;
T1 = 6.89938600820852024;
T2 = 176.69198300312879537;
T3 = T1*T2/K*8.638*10.^(-5);
T = T1+T2-T3;



           
           
           
           
           