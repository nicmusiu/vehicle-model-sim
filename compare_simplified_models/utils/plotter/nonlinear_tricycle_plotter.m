% Define the variable names
varNames = {'vx', 'vy', 'r', 'Delta', 'alpha_f', 'alpha_r',...
    'Fyf', 'Fyr', 'Fzf', 'Fzr', 'ay','M_diff'};
results.nonlinear_tricycle = array2table(nonlinear_tricycle_results', 'VariableNames', varNames);

results.nonlinear_tricycle.yaw = settings.YAW_0 + cumtrapz(settings.h,results.nonlinear_tricycle.r);
results.nonlinear_tricycle.X = settings.X_0 + cumtrapz(settings.h,vx.*cos(results.nonlinear_tricycle.yaw) - results.nonlinear_tricycle.vy.*sin(results.nonlinear_tricycle.yaw));
results.nonlinear_tricycle.Y = settings.Y_0 + cumtrapz(settings.h,vx.*sin(results.nonlinear_tricycle.yaw) + results.nonlinear_tricycle.vy.*cos(results.nonlinear_tricycle.yaw));

figure
plot(results.nonlinear_tricycle.X,results.nonlinear_tricycle.Y)  
legend('path'), axis equal
xlabel('X - m'), ylabel('Y - m'), title('Nonlinear tricycle model')

figure;
subplot(6,1,1)
title('Nonlinear tricycle model'), hold on,
plot(t_sim,rad2deg(Delta)*car.steering_ratio), legend('steering - °')
subplot(6,1,2)
plot(t_sim,rad2deg(results.nonlinear_tricycle.yaw)), legend('yaw - °')
subplot(6,1,3), hold on
plot(t_sim,rad2deg(results.nonlinear_tricycle.vy./vx)), legend('beta - °')
subplot(6,1,4)
plot(t_sim,results.nonlinear_tricycle.vy), legend('vy - m/s')
subplot(6,1,5), hold on
plot(t_sim,rad2deg(results.nonlinear_tricycle.r)), legend('r - °/s')
subplot(6,1,6), hold on
plot(t_sim,results.nonlinear_tricycle.M_diff), legend('M_{diff}- Nm')
xlabel('time - s')
c = get(gcf,'children');
linkaxes(c,'x');

clear nonlinear_tricycle_results;
