% Creating Inputs for Numerical Integration
Y0 = [20000; 0; 0; 0; 2.9; 1.8]; % [x; y; z; vx; vy; vz] [km, km/s]
tspan = [0:60:(24*60*60)]; % One day [s]
options = odeset('RelTol', 1e-13); % Setting a tolerance% Numerical Integration
[t, Y] = ode113(@customODE, tspan, Y0, options);% Pulling Position Data from Output
x = Y(:, 1); % [km]
y = Y(:, 2); % [km]
z = Y(:, 3); % [km]

%% Debris data
numDebris = 10;

range = 7e6 + 1e5*randn(numDebris,1);
ecc = 0.015 + 0.005*randn(numDebris,1);
inc = 80 + 10*rand(numDebris,1);
lan = 360*rand(numDebris,1);
w = 360*rand(numDebris,1);
nu = 360*rand(numDebris,1);

% Convert to initial position and velocity
for i = 1:numDebris
    [r,v] = oe2rv(range(i),ecc(i),inc(i),lan(i),w(i),nu(i));
    data(i).InitialPosition = r; %#ok<SAGROW>
    data(i).InitialVelocity = v; %#ok<SAGROW>
    
    Y_d0 = [r*1.2; v];
    Y_d0 = Y_d0/1000; % convert to km
    
    [t, Y] = ode113(@customODE, tspan, Y_d0, options);% Pulling Position Data from Output
    data(i).x_d = Y(:, 1); %#ok<SAGROW> % [km] 
    data(i).y_d = Y(:, 2); %#ok<SAGROW> % [km] 
    data(i).z_d = Y(:, 3); %#ok<SAGROW> % [km] 
end


%% Creating Figure
figure; 
hold on
title('Two-Body Trajectory', 'Interpreter', 'Latex')
xlabel('x', 'Interpreter', 'Latex')
ylabel('y', 'Interpreter', 'Latex')
zlabel('z', 'Interpreter', 'Latex')
axis equal
grid minor
view(30, 30)% Creating/Plotting Spherical Earth
rm = 6378.14; % Radius of Earth [km]
[xEarth, yEarth, zEarth] = sphere(25);
surf(rm*xEarth,rm*yEarth,rm*zEarth, 'FaceColor', [0 0 1]);% Plotting Trajectory
plot3(x, y, z, 'k')
for i=1:numDebris
    plot3(data(i).x_d, data(i).y_d, data(i).z_d)
end
hold off


