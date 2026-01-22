% Define the variable names
varNames = {'vy', 'r', 'alpha_f', 'alpha_r', ...
    'Fyf', 'Fyr', 'Fzf', 'Fzr'}; ...
    % , 'A11', 'A12', 'A21', 'A22', 'B1', 'B2'};

results.linear_single_track = array2table(linear_single_track_result', 'VariableNames', varNames);

results.linear_single_track.yaw = settings.YAW_0 + cumtrapz(settings.h,results.linear_single_track.r);
results.linear_single_track.X = settings.X_0 + cumtrapz(settings.h,vx.*cos(results.linear_single_track.yaw) - results.linear_single_track.vy.*sin(results.linear_single_track.yaw));
results.linear_single_track.Y = settings.Y_0 + cumtrapz(settings.h,vx.*sin(results.linear_single_track.yaw) + results.linear_single_track.vy.*cos(results.linear_single_track.yaw));

figure
plot(results.linear_single_track.X,results.linear_single_track.Y)
legend('path'), axis equal
xlabel('X - m'), ylabel('Y - m'), title('linear bycicle model')

figure
subplot(5,1,1)
title('linear bycicle model'), hold on,
plot(t_sim,rad2deg(Delta)*car.steering_ratio), legend('steering - °')
subplot(5,1,2)
plot(t_sim,rad2deg(cumtrapz(settings.h,results.linear_single_track.r))), legend('yaw - °')
subplot(5,1,3), hold on
plot(t_sim,rad2deg(results.linear_single_track.vy./vx)), legend('beta - °')
subplot(5,1,4)
plot(t_sim,results.linear_single_track.vy), legend('vy - m/s')
subplot(5,1,5), hold on
plot(t_sim,rad2deg(results.linear_single_track.r)), legend('r - °/s')
xlabel('time - s')
c = get(gcf,'children');
linkaxes(c,'x');

clear linear_single_track_result;