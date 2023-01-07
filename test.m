clear;

% Creating Inputs for Numerical Integration
Y0 = [20000; 0; 0; 0; 2.9; 1.8]; % [x; y; z; vx; vy; vz] [km, km/s]
tspan = [0:60:(24*60*60)]; % One day [s]
options = odeset('RelTol', 1e-13); % Setting a tolerance% Numerical Integration
[t, Ys] = ode113(@customODE, tspan, Y0, options);% Pulling Position Data from Output
x = Ys(:, 1); % [km]
y = Ys(:, 2); % [km]
z = Ys(:, 3); % [km]

Sat.position = Ys(:, [1 2 3]);
Sat.velocity = Ys(:, [4 5 6]);
Sat.size = 0.5;
Sat.mass = 500;

%% Create Debris data
numDebris = 5;

range = 7e6 + 1e5*randn(numDebris,1);
ecc = 0.015 + 0.005*randn(numDebris,1);
inc = 80 + 10*rand(numDebris,1);
lan = 360*rand(numDebris,1);
w = 360*rand(numDebris,1);
nu = 360*rand(numDebris,1);

indexPos = 1;
tspan_n = length(tspan);
% Convert to initial position and velocity
for i = 1:numDebris
    [r,v] = oe2rv(range(i),ecc(i),inc(i),lan(i),w(i),nu(i));
    
    debrisData(i).InitialPosition = r; %#ok<SAGROW>
    debrisData(i).InitialVelocity = v; %#ok<SAGROW>
    
    Y_d0 = [r*1.2; v];
    Y_d0 = Y_d0/1000; % convert to km
    
    part = indexPos + (tspan_n - indexPos) * i/numDebris;
    ind_time = randi(part);
%     Y_d0(1) = x(ind_time);
    
    debrisData(i).size = randi([250, 300]); %#ok<SAGROW>
    
    debrisData(i).position = [x(ind_time) + randi(20), y(ind_time) + randi(20), z(ind_time) + randi(20)]; %#ok<SAGROW>
    debrisData(1).position = 1.0e+04 * [1.6833    0.6919    0.4289];
    debrisData(2).position = 1.0e+04 * [0.4732   -1.0876   -0.6742];
    time = tspan(ind_time);
    
    [t, Y] = ode113(@customODE, tspan, Y_d0, options);% Pulling Position Data from Output
    debrisData(i).x_d = Y(:, 1); %#ok<SAGROW> % [km] 
    debrisData(i).y_d = Y(:, 2); %#ok<SAGROW> % [km] 
    debrisData(i).z_d = Y(:, 3); %#ok<SAGROW> % [km] 
    
    diff = [Sat.position(:, 1) - debrisData(i).position(:, 1), ...
        Sat.position(:, 2) - debrisData(i).position(:, 2), Sat.position(:, 3) - debrisData(i).position(:, 3)];
    Sat.debris(i).distance = sqrt(diff(:, 1).^2 + diff(:, 2).^2 + diff(:, 3).^2);
end

%% Compute the adjusted orbits

look_ahead_time = 3;
odeClass = ClassODE(0, Y0);
found = 0;

% realSat.position = [x(1:contactTime) y(1:contactTime) z(1:contactTime)];
% realSat.velocity = [Y(1:contactTime, 4) Y(1:contactTime, 5) Y(1:contactTime, 6)];

realSat.position = [x(1) y(1) z(1)];
realSat.velocity = [Ys(1, 4) Ys(1, 5) Ys(1, 6)];
realSat.t(1) = t(1);

odeClass.setDebrisData(debrisData);
odeClass.setTrajectory(Ys, t);
odeClass.numDebris = numDebris;

dt = t(2) - t(1) ;
odeClass.setSatData(Sat.size, dt, Sat.mass);

for realTimeInd=2:length(t)
    dt = t(realTimeInd) - t(realTimeInd-1) ;
    [res, newState, time] = odeClass.stepImpl(dt);
    if odeClass.avoiding
        avoiding = 9;
    end
    if max(newState) > 1e8
        tre = 0;
    end
    if isnan(newState(2))
        sdr = 9;
    end
    realSat.position(end+1, :) = newState([1 2 3]);
    realSat.velocity(end+1, :) = newState([4 5 6]);
    
    
end


%% Creating Figure for visualization
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

testDistance = zeros(length(x), 1);

% Plotting the first iteration

pLine1 = plot3(realSat.position(1, 1), realSat.position(1, 2), realSat.position(1, 3), 'r');
pLine = plot3(x,y,z,'b');

pObject = scatter3(realSat.position(1, 1), realSat.position(1, 2), realSat.position(1, 3), 30, 'filled','b');
for i=1:numDebris
    debrisData(i).scatter = scatter3(debrisData(i).position(1), debrisData(i).position(2), debrisData(i).position(3), 30, 'filled');
end
scatter3(odeClass.test(1, 1), odeClass.test(1, 2), odeClass.test(1, 3), 30, 'filled','g')
scatter3(odeClass.test(2, 1), odeClass.test(2, 2), odeClass.test(2, 3), 30, 'filled','y')
% scatter3(odeClass.test(3, 1), odeClass.test(3, 2), odeClass.test(3, 3), 30, 'filled','b')
for t=1:length(realSat.position(:, 1))
    if mod(t, 100) == 0
        disp(t)
    end
    
%     if t == contactTime
%         disp(t)
%     end
    
    % Updating satellite trajectory
    pLine1.XData = realSat.position(1:t, 1);
    pLine1.YData = realSat.position(1:t, 2);
    pLine1.ZData = realSat.position(1:t, 3);
    
    % Updating the satellite point
    pObject.XData = realSat.position(t, 1); 
    pObject.YData = realSat.position(t, 2);
    pObject.ZData = realSat.position(t, 3);
    
    for i=1:numDebris
%         data(i).scatter.XData = data(i).x_d(t);
%         data(i).scatter.YData =  data(i).y_d(t);
%         data(i).scatter.ZData = data(i).z_d(t);
%     
        B = [debrisData(i).position(1), debrisData(i).position(2), debrisData(i).position(3)];
    end
    
    
    pause(0.001);
end

hold off


