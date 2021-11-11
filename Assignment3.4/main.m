% Project in TTK4190 Guidance, Navigation and Control of Vehicles 
%
% Author:           My name
% Study program:    My study program

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('WP.mat');
h  = 0.1;    % sampling time [s]
Ns = 70000;    % no. of samples

xd = [0 0 0]';
psi_ref = -110*pi/180;
% psi_ref = zeros(Ns+1,1); % desired yaw angle (rad)
% psi_ref(1:Ns/2,1) = 10*pi/180;    
% psi_ref(Ns/2+1:end,1) = -20*pi/180;
U_ref   = 9;            % desired surge speed (m/s)
V_c = 1;                % current velocity (m/s)
beta_Vc = 45 * pi/180;  % current direction in NED (rad)

V_w = 10;               % wind velocity (m/s)
beta_Vw = 135*pi/180;   % wind direction in NED (rad)
rho_a = 1.247;          % air density (kg/m^3)
c_y = 0.95;
c_n = 0.15;
L = 161;                % Length of ship
A_Lw = 10*L;

% initial states
eta = [0 0 0]';
nu  = [0.1 0 0]';
delta = 0;
n = 0;
Q_m = 0;
x = [nu' eta' delta n Q_m]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simdata = zeros(Ns+1,14);       % table of simulation data
nu_r =zeros(Ns+1,2);            % vectors for u_r and v_r
for i=1:Ns+1
    
    t = (i-1) * h;              % time (s)
    
    % current disturbance
%     uc = V_c * cos(beta_Vc - x(6));
    uc = 0;
%     vc = V_c * sin(beta_Vc -x(6));
    vc = 0;
    nu_c = [ uc vc 0 ]';
    
    % wind disturbance
    Ywind = zeros(Ns+1,1);
    Ywind(2000:end) = c_y * sin(x(6)-beta_Vw-pi)*A_Lw;
    Nwind = zeros(Ns+1,1);
    Nwind(2000:end) = c_n * sin(2*(x(6)-beta_Vw-pi))*A_Lw;
    tau_wind = 0.5 * rho_a * V_w.^2 * [0 Ywind(i) Nwind(i)]';
    
    nu_r(i,1) = x(1)-uc;
    nu_r(i,2) = x(2)-vc;
    if i~=1
        psi_ref = guidance(x(4),x(5),WP);
    end
    
    
    % reference models
    omega_ref = 0.03;
    Ad = [0 1 0;
          0 0 1;
          -omega_ref^3 -3*omega_ref^2   -3*omega_ref];
    Bd = [0 0 omega_ref^3]';
    xd_dot = Ad*xd + Bd*psi_ref;
    
    psi_d = xd(1);
    r_d = xd(2);
    
    
    u_d = U_ref;
        
    % control law
    delta_c = PID_heading(x(6),psi_d,x(3), r_d,i,h); % rudder angle command (rad)
%     delta_c = 0.1;              
    
    m = 17.0677e6;
    Xudot = -8.9830e5;
    T1 = 20;
    Xu = -(m-Xudot)/T1;
    t_thr = 0.05;
    rho = 1025; 
    Dia = 3.3;
    KT = 0.6367;
    
    T_d = u_d * Xu /(t_thr - 1);
    n_d = sign(T_d) * sqrt(T_d / (rho * Dia^4 * KT));
    n_c = n_d;                  % propeller speed (rps)
    
    % ship dynamics
    u = [delta_c n_c]';
    [xdot,u] = ship(x,u,nu_c,tau_wind);
    
    % store simulation data in a table (for testing)
    simdata(i,:) = [t x(1:3)' x(4:6)' x(7) x(8) u(1) u(2) u_d psi_d r_d];
    

    % Euler integration
    x = euler2(xdot,x,h);
    xd = euler2(xd_dot,xd,h);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t       = simdata(:,1);                 % s
u       = simdata(:,2);                 % m/s
v       = simdata(:,3);                 % m/s
r       = (180/pi) * simdata(:,4);      % deg/s
x       = simdata(:,5);                 % m
y       = simdata(:,6);                 % m
psi     = (180/pi) * simdata(:,7);      % deg
delta   = (180/pi) * simdata(:,8);      % deg
n       = 60 * simdata(:,9);            % rpm
delta_c = (180/pi) * simdata(:,10);     % deg
n_c     = 60 * simdata(:,11);           % rpm
u_d     = simdata(:,12);                % m/s
psi_d   = (180/pi) * simdata(:,13);     % deg
r_d     = (180/pi) * simdata(:,14);     % deg/s
u_r     = nu_r(:,1);
v_r     = nu_r(:,2);
U_r     = sqrt(v_r(:).^2 + u_r(:).^2);
% beta_u    = atan2(v_r, U_r)*180/pi;
beta_c    = atan2(v_r, U_r)*180/pi;
% beta_crab_u = atan2(v,u)*180/pi;
beta_crab_c = atan2(v,u)*180/pi;


figure(1)
figure(gcf)
subplot(311)
plot(y,x,'linewidth',2); axis('equal')
title('North-East positions (m)'); xlabel('(m)'); ylabel('(m)'); 
subplot(312)
plot(t,psi,t,psi_d,'linewidth',2);
title('Actual and desired yaw angles (deg)'); xlabel('time (s)');
legend('Actual', 'Desired');
subplot(313)
plot(t,r,t,r_d,'linewidth',2);
title('Actual and desired yaw rates (deg/s)'); xlabel('time (s)');
legend('Actual', 'Desired');

figure(2)
figure(gcf)
subplot(311)
plot(t,u,t,u_d,'linewidth',2);
title('Actual and desired surge velocities (m/s)'); xlabel('time (s)');
legend('Actual', 'Desired');
subplot(312)
plot(t,n,t,n_c,'linewidth',2);
title('Actual and commanded propeller speed (rpm)'); xlabel('time (s)');
legend('Actual', 'Desired');
subplot(313)
plot(t,delta,t,delta_c,'linewidth',2);
title('Actual and commanded rudder angles (deg)'); xlabel('time (s)');
legend('Actual', 'Desired');

pathplotter(x,y);

% figure(4)
% subplot(211)
% plot(t,beta_u,t,beta_c, 'LineWidth',2);
% title('Beta values');
% xlabel('time (s)'); ylabel('Degrees');
% legend('Beta with current','Beta without current');
% 
% subplot(212)
% plot(t,beta_crab_u,t, beta_crab_c, 'LineWidth',2);
% title('Crab angle in degrees');
% xlabel('time (s)'); ylabel('Degrees');
% legend('Crab angle with current', 'Crab angle without current');
% % 
% figure(4)
% plot(t,u_r, 'LineWidth',2);
% title('u_r values');
% xlabel('time (s)'); ylabel('m/s');
% legend('u_r with current');

