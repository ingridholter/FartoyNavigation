function [delta_r] = PID_heading(psi, psi_d, r, r_d,flag,h)
persistent integrator;
persistent psi_error_past;
if flag ==1
    psi_error_past = 0;
    integrator = 0;
    
end
omega_b= 0.06; 
zeta = 1;

K = -0.0049;
T = -99.47013;


omega_n = 1/(1-2*zeta^2+sqrt(4*zeta^4-4*zeta^2+2))*omega_b;
Kp = omega_n^2*T/K;
Kd = (2*zeta*omega_n*T -1)/K;
Ki = omega_n^3*T/(10*K);


psi_error = psi-psi_d;
r_error = r-r_d;

integrator = ssa(integrator + (h/2)*(psi_error+psi_error_past));
delta_r = -Kp * ssa(psi_error) - Kd * r_error - Ki * integrator;

psi_error_past = psi_error;



end