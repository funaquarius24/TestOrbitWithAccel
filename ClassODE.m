classdef ClassODE < dynamicprops
    properties
        MotionModelFcn;
        time;
        CurrentPosition
        CurrentVelocity
        CurrentAcceleration = zeros(1,3); % not part of the state vector
        look_ahead_time = 3;
        debrisData = [];
        numDebris = 1;
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
            
            if nargin >= 3
                acceleration = varargin{1};
            end
            if nargin >= 4
                obj.setState(varargin{2})
            end
            %             dt = 1/obj.SampleRate;
            r = obj.CurrentPosition;
            v = obj.CurrentVelocity;
            
            options = odeset('RelTol', 1e-13); % Setting a tolerance% Numerical Integration
            
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
            time = obj.time + dt;
            obj.time = time;
            
        end
        
        function newState = avoidDebris(obj)
            odeClass = ClassODE(0, Y0);
            contactTime = length(t);
            found = 0;
            for kk=1:length(t)
                if kk + obj.look_ahead_time > length(t) 
                    break; end
                for deb=1:obj.numDebris
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
            px = realSat.position(end, 1); py = realSat.position(end, 2); pz = realSat.position(end, 3);
            vx = realSat.velocity(end, 1); vy = realSat.velocity(end, 2); vz = realSat.velocity(end, 3);
            odeClass.setState([px py pz vx vy vz]);
            odeClass.time = contactTime;
        end
        function obj = setState(obj, initialState)
            obj.CurrentPosition = initialState([1 2 3]);
            obj.CurrentVelocity = initialState([4 5 6]);
        end
    end
    
    
end