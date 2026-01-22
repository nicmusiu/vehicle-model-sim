clear all, close all, clc

addpath params/
addpath(genpath('utils'))

%% LOAD MODEL PARAMs and SIM. SETTINGS
car = yaml.loadFile("model.yaml").params;
settings = jsondecode(fileread("settings.json"));

% Simulation parameters
t_sim = (0:settings.h:settings.duration)';

%% MODEL INPUT
SPEED = settings.vx; % [m/s]
DELTA_SPEED = 10.0; % [m/s]
STEER = settings.steer_angle; % deg

vx = linspace(SPEED-DELTA_SPEED,SPEED+DELTA_SPEED,length(t_sim))';
Delta = deg2rad(linspace(STEER,STEER,length(t_sim))')/car.steering_ratio; % = deg2rad(sin(2*pi*t_sim*0.4)*steering)/15;
ax = gradient(vx,settings.h);

%% KINEMATIC SINGLE-TRACK MODEL

if settings.use_kinematic_model
    clear x0, clear u;

    % Initialize state/input vectors
    x0 = [0; 0; 0]; % [X; Y; YAW]
    u = [vx Delta]';

    % function
    f = @(x,u) kinematic_model(x, u, car, settings);

    % simulation
    for i = 1:length(t_sim)
        loopStart = tic;
        x1 = rk4(f,x0,[u(:,i)],settings.h);
        x0 = x1(:,end);
        execution_time.kinematic_model(i) = toc(loopStart);
        [~,kinematic_model_results(:,i)] = kinematic_model(x0, u(:,i), car, settings); % Store variables
    end
    kinematic_model_plotter;

end
%% linear single-track model
if settings.use_dynamic_linear
    clear x0, clear u, clear loopStart;

    x0 = [0; 0]; % [vy; r]
    u = [vx Delta ax]';

    % function
    f = @(x,u) linear_single_track_model(x,u,car);

    % simulation
    for i = 1:length(t_sim)
        loopStart = tic;
        x1 = rk4(f,x0,[u(:,i)],settings.h);
        x0 = x1(:,end);
        execution_time.linear_single_track_model(i) = toc(loopStart);
        [~,linear_single_track_result(:,i)] = linear_single_track_model(x0,u(:,i),car); % Store variables
    end
    linear_single_track_plotter;

end
 
%% non-linear single-track
if settings.use_dynamic_non_linear
    clear x0, clear u, clear x1, clear loopStart;

    % set input variables
    input = ax;

    % Initialization
    x0 = [0; 0];
    u = [vx Delta input]';

    % function
    f = @(xk,u) nonlinear_single_track_model(xk,u,car);

    % simulation
    for i = 1:length(t_sim)
        loopStart = tic;
        x1 = rk4(f,x0,[u(:,i)],settings.h);
        x0 = x1;
        execution_time.nonlinear_single_track_model(i) = toc(loopStart);
        % Store variables
        [~, nonlinear_single_track_results(:,i)] = nonlinear_single_track_model(x1,u(:,i),car);
    end
    nonlinear_single_track_plotter;

end

%% three-wheels model
if settings.use_tricycle_model
clear x0, clear u, clear x1, clear loopStart;

% set input variables
input = ax;

% Initialization
x0 = [0; 0];
u = [vx Delta input]';
ff_sim = [0, 0];

% function
f = @(xk,u) nonlinear_tricycle_model(xk,u,car,ff_sim);

% simulation
for i = 1:length(t_sim)
    loopStart = tic;
    % integrate model - RK4
    x1 = rk4(f,x0,[u(:,i)],settings.h);
    x0 = x1;
    execution_time.nonlinear_tricycle_model(i) = toc(loopStart);
    % Store variables
    [~, nonlinear_tricycle_results(:,i)] = nonlinear_tricycle_model(x1,u(:,i),car,ff_sim);
    % include simulation output as input!
    ff_sim(1) = (nonlinear_tricycle_results(11,end)); % ay
    ff_sim(2) = (nonlinear_tricycle_results(12,end)); % M_diff
end
nonlinear_tricycle_plotter;

end

%% GLOBAL COMPARISON
clear x0, clear u, clear x1,clear ff_sim, clear varNames, clear u, clear x1;
clear i, clear loopStart;
close all;



figure, hold on, grid minor;

plot(execution_time.kinematic_model, 'LineWidth', 1.5);
plot(execution_time.linear_single_track_model, 'LineWidth', 1.5);
plot(execution_time.nonlinear_single_track_model, 'LineWidth', 1.5);
plot(execution_time.nonlinear_tricycle_model, 'LineWidth', 1.5);
legend('kinematic single-track','linear single-track', ...
       'nonlinear single-track','nonlinear tricycle')
xlabel('Iteration');
ylabel('Loop Time [s]');
title('Turnaround Time');
% yline(settings.h, 'r', 'LineWidth', 1.5, ...
%     'Label', 'ACTUAL TIMESTEP', 'LabelHorizontalAlignment','right');


figure, hold on, grid minor;

plot(results.kinematic.X,results.kinematic.Y)  
plot(results.linear_single_track.X,results.linear_single_track.Y)  
plot(results.nonlinear_single_track.X,results.nonlinear_single_track.Y)  
plot(results.nonlinear_tricycle.X,results.nonlinear_tricycle.Y)  
legend('kinematic single-track','linear single-track', ...
       'nonlinear single-track','nonlinear tricycle'), axis equal
xlabel('X - m'), ylabel('Y - m'), title('XY path comparison')





figure;

subplot(4,1,1)
plot(t_sim,rad2deg(Delta)*car.steering_ratio), legend('steering - (deg)')

subplot(4,1,2)
plot(t_sim,vx), legend('vx - m(/s)')

subplot(4,1,3), hold on
plot(t_sim,rad2deg(results.kinematic.Beta))
plot(t_sim,rad2deg(results.linear_single_track.vy./vx))
plot(t_sim,rad2deg(results.nonlinear_single_track.vy./vx))
plot(t_sim,rad2deg(results.nonlinear_tricycle.vy./vx))
legend('kinematic single-track','linear single-track', ...
       'nonlinear single-track','nonlinear tricycle')
ylabel('sideslip \beta - (deg)')

subplot(4,1,4), hold on
plot(t_sim,rad2deg(results.kinematic.r))
plot(t_sim,rad2deg(results.linear_single_track.r))
plot(t_sim,rad2deg(results.nonlinear_single_track.r))
plot(t_sim,rad2deg(results.nonlinear_tricycle.r))
legend('kinematic single-track','linear single-track', ...
       'nonlinear single-track','nonlinear tricycle')
ylabel('yaw rate r - (deg/s)')
xlabel('time - s')

c = get(gcf,'children');
linkaxes(c,'x');

clc;

%% EXPORT DATA

if settings.export_models_output
    save('results', 'results');
end

