function [chi_d] = ILOS(x,y,WP,h)
persistent k;
persistent wp_x_past;
persistent wp_y_past;
persistent y_int;
if isempty(k)
    k = 1;
    wp_x_past = WP(1,1);
    wp_y_past = WP(2,1);
    y_int =0;
end

% Parameter
delta = 1000;    %m
Kp = 1/delta;
kappa = 1;
Ki = kappa*Kp;
R_switch = 200;
n = length(WP(1,:));


if (k<n)
    wp_x_next = WP(1,k+1);
    wp_y_next = WP(2,k+1);
else
    wp_x_next = WP(1,end);
    wp_y_next = WP(2,end);
end

pi_p = atan2( wp_y_next - wp_y_past, wp_x_next - wp_x_past);

x_long = (x - wp_x_past)*cos(pi_p) + (y - wp_y_past)*sin(pi_p);
y_cross = -(x - wp_x_past)*sin(pi_p) + (y - wp_y_past)*cos(pi_p);

d_wp = sqrt( (wp_x_next - wp_x_past)^2 + (wp_y_next - wp_y_past)^2);

if (abs(d_wp - x_long) < R_switch) && k < n
    k=k+1;
    wp_x_past = wp_x_next;
    wp_y_past = wp_y_next;
    
    disp(k);
end



chi_d = ssa(pi_p - atan(Kp * y_cross+Ki*y_int));
dy_int = delta * y_cross / (delta^2+(y_cross + kappa * y_int)^2);

% Euler integration

y_int = euler2(dy_int,y_int,h);%y_int + h * dy_int;

end 