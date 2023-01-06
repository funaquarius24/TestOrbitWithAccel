
function dYdt = customODE_accel(t, Y, acceleration)
    
    if numel(acceleration) == 1 && ~acceleration == 0
        accel = [acceleration acceleration acceleration];
    else
        accel = acceleration;
    end

    
    dt = 60; % problem. Find another way to supply3
    
    mu = 3.986*10^5; % Earth's gravitational parameter [km^3/s^2]
    x = Y(1); % [km]
    y = Y(2); % [km]
    z = Y(3); % [km]
    vx = Y(4); % [km/s]
    vy = Y(5); % [km/s]
    vz = Y(6); % [km/s]
    if ~ accel == 0
        xddot = vx/dt + accel(1); % [km/s^2]
        yddot = vy/dt  + accel(2); % [km/s^2]
        zddot = vz/dt  + accel(3); % [km/s^2]
    else
        xddot = -mu/(x^2+y^2+z^2)^(3/2)*x; % [km/s^2]
        yddot = -mu/(x^2+y^2+z^2)^(3/2)*y; % [km/s^2]
        zddot = -mu/(x^2+y^2+z^2)^(3/2)*z; % [km/s^2]
    end
    
    dYdt = [vx;vy;vz;xddot;yddot;zddot]; % Y'
end


