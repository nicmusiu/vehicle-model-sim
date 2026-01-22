% engineLUT = jsondecode(fileread("engineLUT.json")).engineLUT;
% 
% figure
% plot(engineLUT.rpm,engineLUT.torque,LineWidth=1.5)
% legendStrings = "D = " + string(engineLUT.throttle*100);
% xlabel('rpm'), ylabel('torque'), legend(legendStrings)
% hold off
% 
% rpm_in = 4000;
% throttle_in = 0.55;
% torque = getEngineTorque(engineLUT, rpm_in, throttle_in);

function Torque = getEngineTorque(engineLUT, rpm, throttle)
    eta = 1.0;
    idle = min(engineLUT.rpm);
    revLimiter = max(engineLUT.rpm);
    engineLUT.torque = engineLUT.torque * eta;
    rpm_in = min(revLimiter, max(rpm, idle)); % set limiter to avoid NaN

    % Interpolate torque for given RPM across all throttle values
    torque_rpm = interp2(engineLUT.rpm, engineLUT.throttle, engineLUT.torque, ...
        rpm_in, engineLUT.throttle);
    % Ensure the throttle input is within valid bounds
    throttle = max(min(throttle, max(engineLUT.throttle)), min(engineLUT.throttle));

    % Interpolate torque based on given throttle
    Torque = interp1(engineLUT.throttle, torque_rpm, throttle);
    if isnan(Torque) % Final check for NaN values
        disp('NaN in the engine LUT - replaced with zero');
        Torque = 0;
    end
    % disp(torque_)
end