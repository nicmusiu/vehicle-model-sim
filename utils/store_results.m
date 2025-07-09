
sim.results.s = sim_results(1,:);
sim.results.n = sim_results(2,:);
sim.results.mu = sim_results(3,:);
sim.results.vx = sim_results(4,:);
sim.results.vy = sim_results(5,:);
sim.results.r = sim_results(6,:);
sim.results.ax = gradient(sim.results.vx,settings.timestep);
sim.results.vx_dot = sim_results(7,:);

sim.results.ay = sim_results(4,:).* sim_results(6,:);
sim.results.Fyf = sim_results(13,:);
sim.results.Fyr = sim_results(14,:);
sim.results.Fzf = sim_results(15,:);
sim.results.Fzr = sim_results(16,:);
sim.results.Fyf_comb = sim_results(17,:);
sim.results.Fyr_comb = sim_results(18,:);
sim.results.Gyf = sim_results(19,:);
sim.results.Gyr = sim_results(20,:);
sim.results.Fxf = sim_results(21,:);
sim.results.Fxrl = sim_results(22,:);
sim.results.Fxrr = sim_results(23,:);
sim.results.M_yaw = sim_results(24,:);
sim.results.Fxr = sim.results.Fxrl + sim.results.Fxrr;

sim.results.alpha_f = sim_results(11,:);
sim.results.alpha_r = sim_results(12,:);
sim.results.understeer = abs(sim.results.alpha_f)-abs(sim.results.alpha_r);

sim.results.yaw = sim.yaw0 + cumtrapz(settings.timestep,sim.results.r);
sim.results.X = sim.X0 + cumtrapz(settings.timestep,sim.results.vx.*cos(sim.results.yaw) - sim.results.vy.*sin(sim.results.yaw));
sim.results.Y = sim.Y0 + cumtrapz(settings.timestep,sim.results.vx.*sin(sim.results.yaw) - sim.results.vy.*cos(sim.results.yaw));

sim.results.Fzrl = sim_results(26,:);
sim.results.Fzrr = sim_results(27,:);

% sim.results.X_ref = interp1(track.S, track.X, sim.results.s, 'linear', 'extrap');
% sim.results.Y_ref = interp1(track.S, track.Y, sim.results.s, 'linear', 'extrap');
% 
% % Calcola l'angolo di yaw della traiettoria (derivata della traiettoria)
% sim.results.theta_ref = atan2(gradient(sim.results.Y_ref), gradient(sim.results.X_ref));
% 
% % Converti in coordinate cartesiane
% sim.results.X = sim.results.X_ref - sim.results.n .* sin(sim.results.theta_ref);
% sim.results.Y = sim.results.Y_ref + sim.results.n .* cos(sim.results.theta_ref);
% sim.results.yaw = sim.results.theta_ref + sim.results.mu;