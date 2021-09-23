
%% plots for 2e

m = load('measurements_2e.mat');

t = m.measurements(1,:);
chi_ref = m.measurements(2,:);
chi = m.measurements(3,:);
delta_a = m.measurements(4,:);

figure(1)
plot(t,chi_ref);
hold on
plot(t,chi);
grid on;
legend('\chi','\chi_{ref}');
title('Chi');
xlabel('time [s]'); 
ylabel('?'); 
saveas(gcf,'./figures/chi.pdf')
hold off

figure(2)
plot(t,delta_a);
grid on;
legend('delta');
title('Aileron');
xlabel('time [s]'); 
ylabel('?'); 
saveas(gcf,'./figures/aileron.pdf')