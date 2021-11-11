figure()
subplot(211)
plot(t,beta_u,t,beta_c, 'LineWidth',2);
title('Beta values');
xlabel('time (s)'); ylabel('Degrees');
legend('Beta without current','Beta with current');

subplot(212)
plot(t,beta_crab_u,t, beta_crab_c, 'LineWidth',2);
title('Crab angle in degrees');
xlabel('time (s)'); ylabel('Degrees');
legend('Crab angle wihtout current', 'Crab angle with current');