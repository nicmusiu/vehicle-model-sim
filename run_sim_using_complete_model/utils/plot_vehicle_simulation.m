function plot_vehicle_simulation(sim, sim_results, params, car)

    set(0, 'DefaultLineLineWidth', 2.0);

    %% Plot Vehicle State
    if params.plot_vehicle_state
        state = {'s - (m)', 'n - (m)', 'mu - (rad)', 'vx - (m/s)', 'vy - (m/s)', 'r - (rad/s)'};
        titles = {'Progression (S)', 'Lateral Deviation (n)', 'Heading Error (mu)', ...
                  'Longitudinal Velocity (vx)', 'Lateral Velocity (vy)', 'Yaw Rate (r)'};

        figure;
        sgtitle('Vehicle State');
        for i = 1:length(state)
            subplot(ceil(length(state) / 2), 2, i); hold on; grid on;
            plot(sim.time, sim_results(i, :));
            xlabel('Simulation Time');
            ylabel(state{i});
            title(titles{i});
        end
        linkaxes(get(gcf, 'children'), 'x');
    end

    %% Plot Vehicle Input
    if params.plot_vehicle_input
        input = {'delta - (rad)', 'D - (0-1)', 'B - (0-1)', 'Gear (1-N)'};
        titles = {'Steering (delta)', 'Throttle (D)', 'Brake (B)', 'Gear'};

        figure;
        sgtitle('Vehicle Input');
        for i = 1:length(input)
            subplot(ceil(length(input) / 2), 2, i); hold on; grid on;
            plot(sim.time, sim.u(i, :));
            xlabel('Simulation Time');
            ylabel(input{i});
            title(titles{i});
        end
        linkaxes(get(gcf, 'children'), 'x');
    end

    %% Plot Path
    if params.plot_path
        figure; hold on; grid minor; axis equal;
        plot(sim.results.X, sim.results.Y);
        legend('path');
        xlabel('X - m'); ylabel('Y - m'); title('Vehicle trajectory in Cartesian Coordinates');
    end

        %% Plot tire model
    if params.plot_tire_model
        figure;
        subplot(1,2,1), pbaspect([1 1 1]);
        hold on; grid minor; title('Front axle')
        scatter(sim.results.alpha_f, sim.results.Fyf_comb./sim.results.Fzf,[],sim.results.Gyf);
        plotPacejka(car, true, 15); % front tires
        colorbar; clim([0 1]); ylabel(colorbar, 'combined slip');
        xlabel('slip angle - rad'); ylabel('Grip');
        subplot(1,2,2), pbaspect([1 1 1]);
        hold on; grid minor; title('Rear axle')
        scatter(sim.results.alpha_r, sim.results.Fyr_comb./sim.results.Fzr,[],sim.results.Gyr);
        plotPacejka(car, false, 15); % Rear tire
        colorbar; clim([0 1]); ylabel(colorbar, 'combined slip');
        xlabel('slip angle - rad'); ylabel('Grip');
    end

    %% Plot Vehicle Dynamics (Lateral and Longitudinal)
    if params.plot_vehicle_dynamics
        % Lateral Dynamics
        figure;
        sgtitle('Model Dynamics - Lateral');
        n = 4;

        subplot(n,2,1); hold on; grid on;
        title('Front Vertical Load (N)');
        plot(sim.time, sim.results.Fzf); xlabel('Simulation Time');

        subplot(n,2,2); hold on; grid on;
        title('Rear Vertical Load (N)');
        plot(sim.time, sim.results.Fzr); xlabel('Simulation Time');

        subplot(n,2,3); hold on; grid on;
        title('Lateral Force Front (N)');
        plot(sim.time, sim.results.Fyf);
        plot(sim.time, sim.results.Fyf_comb);
        legend('pure lateral', 'lateral combined');
        xlabel('Simulation Time');

        subplot(n,2,4); hold on; grid on;
        title('Lateral Force Rear (N)');
        plot(sim.time, sim.results.Fyr);
        plot(sim.time, sim.results.Fyr_comb);
        legend('pure lateral', 'lateral combined');
        xlabel('Simulation Time');

        subplot(n,2,5); hold on; grid on;
        title('Lateral Acceleration (m/s^2)');
        plot(sim.time, sim.results.ay); xlabel('Simulation Time');

        subplot(n,2,6); hold on; grid on;
        title('Understeer Degree (Deg)');
        plot(sim.time, rad2deg(sim.results.understeer)); xlabel('Simulation Time');

        subplot(n,2,7); hold on; grid on;
        title('Combined Slip Front');
        plot(sim.time, sim.results.Gyf); xlabel('Simulation Time');

        subplot(n,2,8); hold on; grid on;
        title('Combined Slip Rear');
        plot(sim.time, sim.results.Gyr); xlabel('Simulation Time');

        linkaxes(get(gcf, 'children'), 'x');

        % Longitudinal Dynamics
        figure;
        sgtitle('Model Dynamics - Longitudinal');
        n = 3;

        subplot(n,2,1); hold on; grid on;
        title('Longitudinal Force Front (N)');
        plot(sim.time, sim.results.Fxf); xlabel('Simulation Time');

        subplot(n,2,2); hold on; grid on;
        title('Longitudinal Force Rear (N)');
        plot(sim.time, sim.results.Fxr); xlabel('Simulation Time');

        subplot(n,2,3); hold on; grid on;
        title('Longitudinal Forces Rear (N)');
        plot(sim.time, sim.results.Fxrl);
        plot(sim.time, sim.results.Fxrr);
        legend('rear left', 'rear right');
        xlabel('Simulation Time');

        subplot(n,2,4); hold on; grid on;
        title('Yaw Momentum (Nm)');
        plot(sim.time, sim.results.M_yaw); xlabel('Simulation Time');

        subplot(n,2,5); hold on; grid on;
        title('Longitudinal Acceleration (m/s^2)');
        plot(sim.time, sim.results.ax); xlabel('Simulation Time');

        subplot(n,2,6); hold on; grid on;
        title('Engine Speed (rpm)');
        plot(sim.time, sim_results(25, :)); xlabel('Simulation Time');

        linkaxes(get(gcf, 'children'), 'x');
    end
end