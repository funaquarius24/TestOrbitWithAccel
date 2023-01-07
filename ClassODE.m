classdef ClassODE < dynamicprops
    properties
        MotionModelFcn;
        time;
        CurrentPosition
        CurrentVelocity
        CurrentAcceleration = zeros(1,3); % not part of the state vector
        look_ahead_time = 3;
        dt = 0;
        debrisData = [];
        traj = [];
        trajTime = [];
        numDebris = 1;
        satSize = 0.5;
        satMass = 500;
        defaultAccel = 0.01;
        
        avoiding = 0; % In the process of avoiding debris
        avoidAngle = 15; 
        avoidingContactLength = 0; % How much are we avoiding to? Should be 0 if not avoiding
        
        test = [];
        testAdded = 0;
    end
    
    
    methods
        function obj = ClassODE(mmfcn, initialState)
            obj.MotionModelFcn = mmfcn;
            obj.CurrentPosition = initialState([1 2 3]);
            obj.CurrentVelocity = initialState([4 5 6]);
            obj.time = 0;
        end
        
    end
    
    methods (Access = public)
        function [obj, newState, time] = stepImpl(obj, dt, varargin)
            % Use the motion model function to update Position and Velocity
            acceleration = 0;
            if nargin >= 3
                acceleration = varargin{1};
            end
            if nargin >= 4
                obj.setState(varargin{2})
            end
            %             dt = 1/obj.SampleRate;
            
            if dt == 0
                dt = obj.dt;
            end
            
            [willCollide, contactTime, contactDebris, tTest, YTest] = obj.checkCollision();
            
            if willCollide && ~obj.avoiding
                obj.avoiding = 1;
                
                initialPosition = YTest(1, [1 2 3])';
                finalPosition = YTest(contactTime(1), [1 2 3])';
                
                debrisPosition = obj.debrisData(contactDebris(1)).position';
                
                normValue = norm(finalPosition-initialPosition);
                if normValue == 0
                    arcCosValue = [0 0 0]';
                else
                    arcCosValue = (finalPosition-initialPosition) / normValue;
                end
                
                origMoveAngle = acos(arcCosValue);
                
                origMoveDistance = norm(finalPosition-initialPosition);
                
                normValue = norm(finalPosition-initialPosition);
                if normValue == 0
                    arcCosValue = [0 0 0]';
                else
                    arcCosValue = (debrisPosition-initialPosition) / normValue;
                end
                debrisAngle = acos(arcCosValue);
                
                origSpeed = (finalPosition - initialPosition) / (dt*contactTime);
                
                
                signs = sign(origMoveAngle-debrisAngle);
                
                ang = origMoveAngle + signs*obj.avoidAngle - 180;
                
                if ~isreal(ang)
                    ang = real(ang);
                end
                
                
                te = finalPosition-initialPosition;
                size3 = obj.debrisData(contactDebris(1)).size * [1 1 1]';
                rotaZYX = initialPosition + rotz(ang(3))*roty(ang(2))*rotx(ang(1))*te + size3;
                
%                 rotaZYX = rotaZYX;
                
                finalSpeed = (rotaZYX - initialPosition) / (dt*contactTime);
                
                acceleration = (finalSpeed - obj.CurrentVelocity) / (dt*contactTime);
                
                obj.CurrentAcceleration = acceleration;
                obj.avoidingContactLength = contactTime - 1;
                
                v = obj.CurrentVelocity + acceleration * dt;
                s = (v + obj.CurrentVelocity) * dt/2;
                
                obj.CurrentPosition = s + obj.CurrentPosition;
                obj.CurrentVelocity = v;
                    
                
                if ~obj.testAdded
                    obj.test = [obj.test; initialPosition'; finalPosition'];
                    obj.testAdded = 1;
                end
                
            elseif obj.avoiding
                
                if obj.avoidingContactLength > 0
                    obj.avoidingContactLength = obj.avoidingContactLength-1;
                    acceleration = obj.CurrentAcceleration;
                    
                    % s=ut+1/2at^2
                    % V = u+at
                    % S = (u + V)t/2
                    
                    v = obj.CurrentVelocity + acceleration * dt;
                    s = (v + obj.CurrentVelocity) * dt/2;
                    
                    obj.CurrentPosition = s + obj.CurrentPosition;
                    obj.CurrentVelocity = v;
                    
                else
                    obj.avoiding = 0; % obj.avoidingContactLength should not be less than 0
                    acceleration = 0;
                    
                    obj.trajTime = obj.time;
                    
                    currentPosition = obj.traj(obj.time/dt, [1 2 3]);
                    currentVelocity = obj.traj(obj.time/dt, [4 5 6]);
                    obj.CurrentPosition = currentPosition';
                    obj.CurrentVelocity = currentVelocity';
                    
                end
                
                
            else
                options = odeset('RelTol', 1e-13); % Setting a tolerance% Numerical Integration
                
                r = obj.CurrentPosition;
                v = obj.CurrentVelocity;
                
                state = [r,v];
                tmp = obj.time + dt/2;
                tspan = [obj.time obj.time+dt];
                [t, Y] = ode113(@customODE_accel, tspan, state, options, acceleration);% Pulling Position Data from Output
                %             newstate = obj.MotionModelFcn(dt,state);
                
                Position = Y(end, [1 2 3])';
                Velocity = Y(end, [4 5 6])';
                newState = [Position; Velocity];
                obj.CurrentPosition = Position;
                obj.CurrentVelocity = Velocity;
                
            end
            
            newState = [obj.CurrentPosition; obj.CurrentVelocity];
            time = obj.time + dt;
            obj.time = time;
            
        end
        
        function [willCollide, contactTime, contactDebris, t, Y] = checkCollision(obj)
            tspan = [obj.time:obj.dt:(obj.time + (obj.dt * obj.look_ahead_time))];
            [t, Y] = obj.runODE(tspan);
            satPos = Y(:, [1 2 3]);
            
            
            contactTime = [];
            contactDebris = [];
            willCollide = 0;
            
            for j=1:obj.numDebris
                debrisPosition = obj.debrisData(j).position; 
                
                difference = [satPos(:, 1) - debrisPosition(1), ...
                    satPos(:, 2) - debrisPosition(2), satPos(:, 3) - debrisPosition(3)];
                satDistance = sqrt(difference(:, 1).^2 + difference(:, 2).^2 + difference(:, 3).^2);
                
                ind = find(satDistance <= obj.satSize + obj.debrisData(j).size);
                
                if ~isempty(ind)
                    ind = ind(1);
                    contactTime(end+1) = ind; %#ok
                    contactDebris(end+1) = j; %#ok
                    willCollide = 1;
                end
                
            end
            
        end
        
        function obj = setState(obj, initialState)
            obj.CurrentPosition = initialState([1 2 3]);
            obj.CurrentVelocity = initialState([4 5 6]);
        end
        
        function obj = setTrajectory(obj, traj, trajTime)
            obj.traj = traj;
            obj.trajTime = trajTime;
        end
        
        function obj = setDebrisData(obj, debrisData)
            obj.debrisData = debrisData;
        end
        
        function obj = setSatData(obj, size, dt, mass)
            obj.satSize = size;
            obj.satMass = mass;
            obj.dt = dt;
        end
        
        function [t, Y] = runODE(obj, tspan)
            r = obj.CurrentPosition;
            v = obj.CurrentVelocity;
            
            options = odeset('RelTol', 1e-13); % Setting a tolerance% Numerical Integration
            
            state = [r,v];
            
            [t, Y] = ode113(@customODE, tspan, state, options); % Pulling Position Data from Output
            %             newstate = obj.MotionModelFcn(dt,state);
        end
        
        function [t, Y] = runODE_accel(obj, tspan, acceleration)
            r = obj.CurrentPosition;
            v = obj.CurrentVelocity;
            
            options = odeset('RelTol', 1e-13); % Setting a tolerance% Numerical Integration
            
            state = [r,v];
            
            [t, Y] = ode113(@customODE_accel, tspan, state, options, acceleration); % Pulling Position Data from Output
            %             newstate = obj.MotionModelFcn(dt,state);
        end
    end
    
    
end