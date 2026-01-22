% Define the variable names
varNames = {'vx', 'vy', 'r', 'Delta', 'alpha_f', 'alpha_r',...
    'Fyf', 'Fyr', 'Fzf', 'Fzr', 'ay'};
results.nonlinear_single_track = array2table(nonlinear_single_track_results', 'VariableNames', varNames);

results.nonlinear_single_track.yaw = settings.YAW_0 + cumtrapz(settings.h,results.nonlinear_single_track.r);
results.nonlinear_single_track.X = settings.X_0 + cumtrapz(settings.h,vx.*cos(results.nonlinear_single_track.yaw) - results.nonlinear_single_track.vy.*sin(results.nonlinear_single_track.yaw));
results.nonlinear_single_track.Y = settings.Y_0 + cumtrapz(settings.h,vx.*sin(results.nonlinear_single_track.yaw) + results.nonlinear_single_track.vy.*cos(results.nonlinear_single_track.yaw));

figure
plot(results.nonlinear_single_track.X,results.nonlinear_single_track.Y)
legend('path'), axis equal
xlabel('X - m'), ylabel('Y - m'), title('Nonlinear bycicle model')

figure
subplot(5,1,1)
title('Nonlinear bycicle model'), hold on,
plot(t_sim,rad2deg(Delta)*car.steering_ratio), legend('steering - °')
subplot(5,1,2)
plot(t_sim,rad2deg(results.nonlinear_single_track.yaw)), legend('yaw - °')
subplot(5,1,3), hold on
plot(t_sim,rad2deg(results.nonlinear_single_track.vy./vx)), legend('beta - °')
subplot(5,1,4)
plot(t_sim,results.nonlinear_single_track.vy), legend('vy - m/s')
subplot(5,1,5), hold on
plot(t_sim,rad2deg(results.nonlinear_single_track.r)), legend('r - °/s')
xlabel('time - s')
c = get(gcf,'children');
linkaxes(c,'x');

clear nonlinear_single_track_results;
