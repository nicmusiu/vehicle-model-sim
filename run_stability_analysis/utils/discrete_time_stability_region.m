function f = discrete_time_stability_region(polesd)
% plot the stablity region for discrete-time model
figure, axis equal, hold on, grid on, grid minor
title('Discrete-time model stability')
r = 1; c = [0 0];
rectangle('Position',[c-r 2*r 2*r],'Curvature',[1 1], 'EdgeColor','r','LineWidth',2)
plot(real(polesd), imag(polesd), 'x', 'LineWidth', 2, 'MarkerSize', 12);
xlabel('Real Part');
ylabel('Imaginary Part');
end
