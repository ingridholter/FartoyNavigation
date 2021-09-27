
%% plots for 2e

m = load('measurements_2e.mat');

t = m.measurements(1,:);
chi_ref = m.measurements(2,:);
chi = m.measurements(3,:);
delta_a = m.measurements(4,:);
delta_a_u = m.measurements(5,:);

figure(1)
plot(t,chi_ref,'b');
hold on
plot(t,chi,'r');
grid on;
legend('\chi','\chi_{ref}');
title('Course');
xlabel('time [s]'); 
ylabel('Course [deg]'); 
saveas(gcf,'./figures/chi.pdf')
hold off

figure(2)
plot(t,delta_a);
hold on
plot(t,delta_a_u);
grid on;
legend('\delta_a saturated','\delta_a');
title('Aileron deflection');
xlabel('time [s]'); 
ylabel('Aileron deflection [deg]'); 
saveas(gcf,'./figures/aileron.pdf')