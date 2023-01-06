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
Sat.size =0.5;

%% Create Debris data
numDebris = 1;

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
    
    data(i).InitialPosition = r; %#ok<SAGROW>
    data(i).InitialVelocity = v; %#ok<SAGROW>
    
    Y_d0 = [r*1.2; v];
    Y_d0 = Y_d0/1000; % convert to km
    
    part = indexPos + (tspan_n - indexPos) * i/numDebris;
    ind_time = randi(part);
%     Y_d0(1) = x(ind_time);
    
    data(i).size = randi([10, 300]);
    
    data(i).position = [x(ind_time) + randi(20), y(ind_time) + randi(20), z(ind_time) + randi(20)]; %#ok<SAGROW>
    time = tspan(ind_time);
    
    [t, Y] = ode113(@customODE, tspan, Y_d0, options);% Pulling Position Data from Output
    data(i).x_d = Y(:, 1); %#ok<SAGROW> % [km] 
    data(i).y_d = Y(:, 2); %#ok<SAGROW> % [km] 
    data(i).z_d = Y(:, 3); %#ok<SAGROW> % [km] 
    
    diff = [Sat.position(:, 1) - data(i).position(:, 1), ...
        Sat.position(:, 2) - data(i).position(:, 2), Sat.position(:, 3) - data(i).position(:, 3)];
    Sat.debris(i).distance = sqrt(diff(:, 1).^2 + diff(:, 2).^2 + diff(:, 3).^2);
end

%% Compute the adjusted orbits

look_ahead_time = 3;
odeClass = ClassODE(0, Y0);
contactTime = length(t);
found = 0;
for kk=1:length(t)
    if kk + 3 > length(t) break; end
    for deb=1:numDebris
        if Sat.debris(deb).distance(kk+3) < (Sat.size + data(deb).size)
            contactTime = kk;
            found = 1;
            break
            
        end
    end
    if found ==1
        break
    end
end

realSat.position = [x(1:contactTime) y(1:contactTime) z(1:contactTime)];
realSat.velocity = [Y(1:contactTime, 4) Y(1:contactTime, 5) Y(1:contactTime, 6)];

odeClass.time = contactTime;
px = realSat.position(end, 1);
py = realSat.position(end, 2);
pz = realSat.position(end, 3);
vx = realSat.velocity(end, 1);
vy = realSat.velocity(end, 2);
vz = realSat.velocity(end, 3);
odeClass.setState([px py pz vx vy vz]);
odeClass.time = contactTime;

for classT=contactTime:length(t)
    
    for jk=1:look_ahead_time
        [~, newState, retTime] = odeClass.stepImpl(60, 0.01);
        satLookAhead.position(jk, :) = newState([1 2 3]);
        satLookAhead.velocity(jk, :) = newState([4 5 6]);
        
    end
    realSat.position(end+1:end+look_ahead_time, :) = satLookAhead.position;
    realSat.velocity(end+1:end+look_ahead_time, :) = satLookAhead.velocity;
    
    for jk=1:look_ahead_time
        [~, newState, retTime] = odeClass.stepImpl(60, -0.1);
        satLookAhead.position(jk, :) = newState([1 2 3]);
        satLookAhead.velocity(jk, :) = newState([4 5 6]);
         
    end
    realSat.position(end+1:end+look_ahead_time, :) = satLookAhead.position;
    realSat.velocity(end+1:end+look_ahead_time, :) = satLookAhead.velocity;
    
    
    realSat.position = [realSat.position; Sat.position(contactTime+look_ahead_time:end, :)];
    realSat.velocity = [realSat.velocity; Sat.velocity(contactTime+look_ahead_time:end, :)];
%     realSat.position(contactTime+look_ahead_time*2 + 1, :) = Sat.position(contactTime+look_ahead_time:end, :);
%     realSat.velocity(contactTime+look_ahead_time*2 + 1, :) = Sat.velocity(contactTime+look_ahead_time:end, :);
    break
    
    
%     diff = [satLookAhead.position(:, 1) - data(i).position(:, 1), ...
%         satLookAhead.position(:, 2) - data(i).position(:, 2), satLookAhead.position(:, 3) - data(i).position(:, 3)];
%     Sat.debris(i).distance = sqrt(diff(:, 1).^2 + diff(:, 2).^2 + diff(:, 3).^2);
%     for deb=1:numDebris
%         if Sat.debris(deb).distance(classT+3) > (Sat.size + data(deb).size)
%             contactTime = kk;
%         end
%     end
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

pLine1 = plot3(realSat.position(:, 1), realSat.position(:, 2), realSat.position(:, 3), 'r');
pLine = plot3(x(1),y(1),z(1),'b');

pObject = scatter3(realSat.position(1, 1), realSat.position(1, 2), realSat.position(1, 3), 30, 'filled','b');
for i=1:numDebris
    data(i).scatter = scatter3(data(i).position(1), data(i).position(2), data(i).position(3), 30, 'filled');
end
for t=1:length(x)
    if mod(t, 100) == 0
        disp(t)
    end
    
    if t == contactTime
        disp(t)
    end
    
    % Updating satellite trajectory
    pLine.XData = x(1:t);
    pLine.YData = y(1:t);
    pLine.ZData = z(1:t);
    
    % Updating the satellite point
    pObject.XData = realSat.position(t, 1); 
    pObject.YData = realSat.position(t, 2);
    pObject.ZData = realSat.position(t, 3);
    
    for i=1:numDebris
%         data(i).scatter.XData = data(i).x_d(t);
%         data(i).scatter.YData =  data(i).y_d(t);
%         data(i).scatter.ZData = data(i).z_d(t);
%     
        B = [data(i).position(1), data(i).position(2), data(i).position(3)];
    end
    
    
    pause(0.001);
end

hold off


