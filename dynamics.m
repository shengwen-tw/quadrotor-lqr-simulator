classdef dynamics
    properties
        ENABLE_DISTURBANCE = 0;
        
        dt;      %[sec]
        mass;    %[kg]
        
        g = 9.8; %gravitational acceleration
        
        J;       %inertia matrix [kg*m^2]
        
        x;       %initial position
        v;       %initial velocity
        a;       %initial acceleration
        W;       %intial angular velocity
        W_dot;   %initial angular acceleration
        
        R;       %attitude, direction cosine matrix
        R_det;
        
        d = zeros(6, 1); %disturbance
        sigma_f_w = 2;   %distribution of the force disturbance
        sigma_tau_w = 2; %distribution of the torque disturbance
        tau_c = 3.2;     %correlation time of the wind disturbance
        
        prv_angle;
        
        f;       %control force
        M;       %control moment
    end
    
    methods
        function f_next = integrator(obj, f_now, f_dot, dt)
            %euler method
            f_next = f_now + f_dot .* dt;
        end
        
        function ret_obj = update(obj)
            math = se3_math;
            
            %simulate external disturbance
            inv_cor_time = -1 / obj.tau_c;
            A_d = inv_cor_time .* eye(6, 6);
            noise = randn(6, 1);
            noise(1:3) = obj.sigma_f_w * noise(1:3);
            noise(4:6) = obj.sigma_tau_w * noise(4:6);
            if 1
                %disturbance as random noise:
                obj.d = noise;
            else
                %disturbance as ODE
                %d_dot = A_d * obj.d + noise;
                %obj.d = obj.integrator(obj.d, d_dot, obj.dt);
            end
            
            %calculate angular velocity by integrating angular acceleration
            obj.W = obj.integrator(obj.W, obj.W_dot, obj.dt);
            
            %calculate velocity by integrating acceleration
            obj.v = obj.integrator(obj.v, obj.a, obj.dt);
            
            %calculate rotation matrix by intergrating DCM differential equation
            %read "Direction Cosine Matrix IMU: Theory" for detailed explanation
            Wdt = [obj.W(1) * obj.dt; obj.W(2) * obj.dt; obj.W(3) * obj.dt];
            I = eye(3);
            dR = math.hat_map_3x3(Wdt) + I;
            %according to definition of SO(3), to rotate R by dR, the correct way
            %to do is multiplication rather than addition, since SO(3) is not closed
            %under addition
            obj.R = obj.R * dR;
            %maintain the orthonomal property of DCM
            obj.R = math.dcm_orthonormalize(obj.R);
            %to check the orthonormal propertiy of DCM, we can check the determinant of it
            %obj.R_det = det(obj.R);
            %disp(obj.R_det)
            
            %calculate the angle of principle rotation vector
            obj.prv_angle = math.get_prv_angle(obj.R);
            
            %calculate position by integrating velocity
            obj.x = obj.integrator(obj.x, obj.v, obj.dt);
            
            %calculate current accelration from force
            e3 = [0; 0; 1];
            if obj.ENABLE_DISTURBANCE ~= 0
                mv_dot = (obj.mass * obj.g * e3) - obj.f + obj.d(1:3);
            else
                mv_dot = (obj.mass * obj.g * e3) - obj.f;
            end
            obj.a = mv_dot / obj.mass;
            
            %calculate current aungular acceleration from moment
            JW = obj.J * obj.W;
            WJW = cross(obj.W, JW);
            if obj.ENABLE_DISTURBANCE ~= 0
                obj.W_dot = inv(obj.J) * (obj.M - WJW - obj.d(4:6));
            else
                obj.W_dot = inv(obj.J) * (obj.M - WJW);
            end
            
            ret_obj = obj;
        end
    end
end
