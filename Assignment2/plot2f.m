%% plot 2f


m = load('measurements_2f.mat');

t = m.measurements(1,:);
chi_ref = m.measurements(2,:);
chi = m.measurements(3,:);
delta_a = m.measurements(4,:);

figure(1)
plot(t,chi_ref,'b');
hold on
plot(t,chi,'r');
grid on;
legend('\chi','\chi_{ref}');
title('Course');
xlabel('time [s]'); 
ylabel('Course '); 
saveas(gcf,'./figures/2f_chi.pdf')
hold off

figure(2)
plot(t,delta_a);
grid on;
legend('\delta_a');
title('Aileron');
xlabel('time [s]'); 
ylabel(''); 
saveas(gcf,'./figures/2f_aileron.pdf')