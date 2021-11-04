% Project in TTK4190 Guidance, Navigation and Control of Vehicles 
%
% Author:           My name
% Study program:    My study program

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h  = 0.1;    % sampling time [s]
Ns = 10000;    % no. of samples

psi_ref = 10 * pi/180;  % desired yaw angle (rad)
U_ref   = 9;            % desired surge speed (m/s)

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

for i=1:Ns+1
    
    t = (i-1) * h;              % time (s)
    
    % current disturbance
    uc = 0;
    vc = 0;
    nu_c = [ uc vc 0 ]';
    
    % wind disturbance
    Ywind = 0;
    Nwind = 0;
    tau_wind = [0 Ywind Nwind]';
    
    % reference models
    psi_d = psi_ref;
    r_d = 0;
    u_d = U_ref;
        
    % control law
    delta_c = 0.1;              % rudder angle command (rad)
    
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
saveas(gcf,'figures/speed_control.eps','epsc');
