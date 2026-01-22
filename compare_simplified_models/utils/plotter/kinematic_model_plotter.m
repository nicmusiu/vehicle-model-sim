
results.kinematic = table(kinematic_model_results(1,:)',...
                          kinematic_model_results(2,:)',...
                          kinematic_model_results(3,:)',...
                          kinematic_model_results(4,:)',...
                          kinematic_model_results(5,:)',...
                          kinematic_model_results(6,:)',...
                          kinematic_model_results(7,:)',...
    'VariableNames', {'X', 'Y', 'Yaw', 'Beta', 'Vx', 'Vy', 'r'});

% kin.R = [cos(settings.YAW_0), -sin(settings.YAW_0); sin(settings.YAW_0), cos(settings.YAW_0)];
% rotated_points = kin.R * [kinematic_model_results(1,:); kinematic_model_results(2,:)];
% kin.X_rotated = rotated_points(1, :) + settings.X_0;
% kin.Y_rotated = rotated_points(2, :) + settings.Y_0;

figure, hold on;
plot(results.kinematic.X,results.kinematic.Y), legend('path')
xlabel('X - m'), ylabel('Y - m'), title('kinematic model')
axis equal

figure
subplot(4,1,1)
title('kinematic model'), hold on,
plot(t_sim,rad2deg(Delta)*15), legend('steering - °')
subplot(4,1,2)
plot(t_sim,rad2deg(results.kinematic.Yaw)), legend('yaw - °')
subplot(4,1,3), hold on
plot(t_sim,vx), legend('vx - m/s')
subplot(4,1,4)
plot(t_sim,rad2deg(results.kinematic.r)), legend('r - °/s')
xlabel('time - s')
c = get(gcf,'children');
linkaxes(c,'x');

clear kinematic_model_results;
