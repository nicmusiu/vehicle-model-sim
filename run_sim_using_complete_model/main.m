% -------------------------------------------------------------------------
% main.m - simulate simplified tricycle model over time, including engine
% model and full vehicle input.
%
% Copyright (c) 2025 [Unimore Racing]
% All rights reserved.
% 
% Author: [nicola.musiu@unimore.it]
% Date: 2025-03-12
% -------------------------------------------------------------------------

clear all, close all, clc

%% SET SIMULATION
addpath(genpath('utils/'));
addpath(genpath('models/'));
addpath params;
settings = jsondecode(fileread("sim_settings.json"));
car = jsondecode(fileread("model.json"));
engine = jsondecode(fileread("engine_map.json")).engineLUT;

% Simulation setup
sim.time = (0:settings.timestep:settings.duration)';

%% INPUT 
u.steering = + deg2rad(100) / car.steer_ratio; % [deg]
u.Delta = linspace(u.steering,u.steering,length(sim.time)); % steering angle @ wheels
u.D = linspace(0.0,0.5,length(sim.time)); % throttle
u.B = linspace(0.0,0.0,length(sim.time)); % brake
u.gear = linspace(1,1,length(sim.time));  % gear

%% INITIALIZE MODEL
sim.x0 = [  settings.s_start;... s
            settings.n_start;... n
            settings.mu_start;... mu
            settings.vx_start;... vx
            settings.vy_start;... vy
            settings.r_start;... r
                    ];
sim.u = [u.Delta'...    steer
         u.D'...        throttle
         u.B'...        brake
         u.gear'...     gear
                    ]';
sim.yaw0 = 0.0;
sim.X0 = 0.0;
sim.Y0 = 0.0;

%% IMPORT MODEL
f = @(xk,u) sim_model(xk,u,car,engine,settings);

%% SIMULATE MODEL
for i = 1:length(sim.time)
    
    % Store Sim. Results
    [~, sim_results(:,i)] = sim_model(sim.x0, sim.u(:,i), car, engine, settings);
    
    % Integrate Model using RK4
    sim.x1 = rk4(f, sim.x0, [sim.u(:,i)], settings.timestep);
    
    % Set New Initial Condition
    sim.x0 = sim.x1; 
end

%% STORE RESULTS
store_results

%% PLOT RESULTS
plot_vehicle_simulation(sim, sim_results, settings, car);