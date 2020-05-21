classdef se3_math
    methods
        function dcm = euler_to_dcm(obj, roll, pitch ,yaw)
            %R = Rz(psi)Ry(theta)Rx(phi)
            %read: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
            cos_phi = cos(double(roll));
            cos_theta = cos(pitch);
            cos_psi = cos(yaw);
            sin_phi = sin(roll);
            sin_theta = sin(pitch);
            sin_psi = sin(yaw);
            dcm11 = cos_theta * cos_psi;
            dcm12 = (-cos_phi * sin_psi) + (sin_phi * sin_theta * cos_psi);
            dcm13 = (sin_phi * sin_psi) + (cos_phi * sin_theta * cos_psi);
            dcm21 = cos_theta * sin_psi;
            dcm22 = (cos_phi * cos_psi) + (sin_phi * sin_theta * sin_psi);
            dcm23 = (-sin_phi * cos_psi) + (cos_phi * sin_theta * sin_psi);
            dcm31 = -sin_theta;
            dcm32 = sin_phi * cos_theta;
            dcm33 = cos_phi * cos_theta;
            dcm = [dcm11 dcm12 dcm13;
                dcm21 dcm22 dcm23;
                dcm31 dcm32 dcm33;];
        end
        
        function euler = dcm_to_euler(obj, R)
            euler = [atan2(R(3, 2), R(3, 3));
                asin(-R(3, 1));
                atan2(R(2, 1), R(1, 1))];
        end
        
        function dcm = quaternion_to_dcm(obj, q0, q1, q2, q3)
            %read: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;
            q1q2 = q1 * q2;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q3 = q1 * q3;
            q2q3 = q2 * q3;
            q0q1 = q0 * q1;
            
            dcm11 = 1.0 - 2.0 * (q2q2 + q3q3);
            dcm12 = 2.0 * (q1q2 - q0q3);
            dcm13 = 2.0 * (q0q2 + q1q3);
            dcm21 = 2.0 * (q1q2 + q0q3);
            dcm22 = 1.0 - 2.0 * (q1q1 + q3q3);
            dcm23 = 2.0 * (q2q3 - q0q1);
            dcm31 = 2.0 * (q1q3 - q0q2);
            dcm32 = 2.0 * (q0q1 + q2q3);
            dcm33 = 1.0 - 2.0 * (q1q1 + q2q2);
            
            dcm = [dcm11 dcm12 dcm13;
                dcm21 dcm22 dcm23;
                dcm31 dcm32 dcm33;];
        end
        
        function quat = dcm_to_quaternion(obj, R)
            q1 = sqrt(0.25 * (1 + R(1, 1) - R(2, 2) - R(3, 3)));
            div_4q1 = 1.0 / (4.0 * q1);
            q2 = (R(2, 1) + R(1, 2)) * div_4q1;
            q3 = (R(3, 1) + R(1, 3)) * div_4q1;
            q0 = (R(2, 3) - R(3, 2)) * -div_4q1;
            quat = [q0; q1; q2; q3];
        end
        
        function vec = vee_map_3x3(obj, mat)
            vec=[mat(3, 2);
                 mat(1, 3);
                 mat(2, 1)];
        end
        
        function mat = hat_map_3x3(obj, vec)
            mat=[0.0 -vec(3) +vec(2);
                 +vec(3) 0.0 -vec(1);
                 -vec(2) +vec(1) 0.0];
        end
        
        function R_orthonormal = dcm_orthonormalize(obj, R)
            %read "Direction Cosine Matrix IMU: Theory" for detailed explanation
            x = [R(1, 1) R(1, 2) R(1, 3)];
            y = [R(2, 1) R(2, 2) R(2, 3)];
            error = dot(x, y);
            x_orthogonal = x - (0.5 * error * y);
            y_orthogonal = y - (0.5 * error * x);
            z_orthogonal = cross(x_orthogonal, y_orthogonal);
            x_normalized = 0.5 * (3 - dot(x_orthogonal, x_orthogonal)) * x_orthogonal;
            y_normalized = 0.5 * (3 - dot(y_orthogonal, y_orthogonal)) * y_orthogonal;
            z_normalized = 0.5 * (3 - dot(z_orthogonal, z_orthogonal)) * z_orthogonal;
            R_orthonormal = [x_normalized; y_normalized; z_normalized];
        end
        
        function angle_rad = get_prv_angle(obj, R)
            angle_rad = acos(0.5 * (R(1, 1) + R(2, 2) + R(3, 3) - 1));
        end
    end
end
