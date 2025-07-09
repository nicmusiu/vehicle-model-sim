function [x, y] = plotPacejka(car, front, slip_range)

    if front
        B = car.By_f;
        C = car.Cy_f;
        D = car.Dy_f;
        E = car.Ey_f;
        Sv = car.Svy_f;
        Sh = car.Shy_f;
        tire_label = 'Front Tire';
    else
        B = car.By_r;
        C = car.Cy_r;
        D = car.Dy_r;
        E = car.Ey_r;
        Sv = car.Svy_r;
        Sh = car.Shy_r;
        tire_label = 'Rear Tire';
    end

    % Compute Pacejka curve
    x = linspace(deg2rad(-slip_range), deg2rad(slip_range), 100)';
    xh = x + Sh;
    y = Sv + D .* sin(C * atan((B .* xh) - E * ((B .* xh) - atan(B .* xh))));

    % Plot
    plot(x, y, 'LineWidth', 2);

    % Display parameters on plot
    txt = {
        ['B = ', num2str(B)]
        ['C = ', num2str(C)]
        ['D = ', num2str(D)]
        ['E = ', num2str(E)]
        ['Sv = ', num2str(Sv)]
        ['Sh = ', num2str(Sh)]
    };
    text(0.05, 0.5, txt, 'Units', 'normalized');
end
