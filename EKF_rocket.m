classdef EKF_rocket
    properties
        x;
        P;
        Ts;

        AccelerometerNoise = 2;
        GyroscopeNoise =  1e-5;

        % extra additive noise
        AccelerometerBiasNoise =  2e-4;
        GyroscopeBiasNoise = 1e-16;
        MagnetometerBiasNoise = 1e-10;
        GeomagneticVectorNoise = 1e-15;
        additiveNoise = 1e-8;

        scale_var = -1;
        ang_delta_bias_sigma = -1;
        vel_delta_bias_sigma = -1;

    end

    methods

        function obj = set_state(obj,x)
            obj.x = x;
        end

        function gps_vel_2d = gps_vel_2d_from_vel_course(obj,V_kmh,course_deg)
                V = V_kmh*(60*60/1000); % km/s to m/s
                course_rad = rad2deg(course_deg); % deg to rad
                gps_vel_2d = [V*cos(course_rad);V*sin(course_rad)]; % m/s
        end

        function obj = set_mag_vec_from_body_avg(obj,magB)
            rot_mat = obj.get_rotmat_body_2_inertial();
            obj.x(20:22) = (rot_mat*magB);
        end

        function rot_mat = get_rotmat_body_2_inertial(obj)
            q0 = obj.x(1);
            q1 = obj.x(2);
            q2 = obj.x(3);
            q3 = obj.x(4);

                rot_mat = zeros(3);
    rot_mat(1,1) = q0*q0+q1*q1-q2*q2-q3*q3;
    rot_mat(1,2) = 2*(q1*q2-q0*q3);
    rot_mat(1,3) = 2*(q1*q3+q0*q2);

    rot_mat(2,1) = 2*(q1*q2+q0*q3);
    rot_mat(2,2) = q0*q0-q1*q1+q2*q2-q3*q3;
    rot_mat(2,3) = 2*(q2*q3-q0*q1);
    
    rot_mat(3,1) = 2*(q1*q3-q0*q2);
    rot_mat(3,2) = 2*(q2*q3+q0*q1);
    rot_mat(3,3) = q0*q0-q1*q1-q2*q2+q3*q3;

            
        end

        function obj = set_process_cov(obj,P)
            obj.P = P;
        end

        function x = get_state(obj)
            x = obj.x;
        end

        function q = get_quat(obj)
            q = obj.x(1:4);
        end

        function p = get_posNED(obj)
            p = obj.x(5:7);
        end

        function v = get_velNED(obj)
            v = obj.x(8:10);
        end

        function euler = get_eulerZYX(obj)
            q = obj.x(1:4);
            euler = quat2eul(q','ZYX');
        end

        function P = get_process_cov(obj)
            P = obj.P;
        end

        function gps_pos = get_gps_local_pos(obj,lla,lla0)
            gps_pos = lla2ned(lla',lla0','flat');
        end

        function F = predict_jacobian(obj,ang_delta,vel_delta,dt)
            q0 = obj.x(1);
            q1 = obj.x(2);
            q2 = obj.x(3);
            q3 = obj.x(4);
            dax_b = obj.x(11);
            day_b = obj.x(12);
            daz_b = obj.x(13);
            dvx_b = obj.x(14);
            dvy_b = obj.x(15);
            dvz_b = obj.x(16);


            dax = ang_delta(1);
            day = ang_delta(2);
            daz = ang_delta(3);

            dvx = vel_delta(1);
            dvy = vel_delta(2);
            dvz = vel_delta(3);




            F = [...
                1,                                              dax_b/2 - dax/2,                                              day_b/2 - day/2,                                              daz_b/2 - daz/2, 0, 0, 0,  0,  0,  0,  q1/2,  q2/2,  q3/2,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0
                dax/2 - dax_b/2,                                                            1,                                              daz/2 - daz_b/2,                                              day_b/2 - day/2, 0, 0, 0,  0,  0,  0, -q0/2,  q3/2, -q2/2,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0
                day/2 - day_b/2,                                              daz_b/2 - daz/2,                                                            1,                                              dax/2 - dax_b/2, 0, 0, 0,  0,  0,  0, -q3/2, -q0/2,  q1/2,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0
                daz/2 - daz_b/2,                                              day/2 - day_b/2,                                              dax_b/2 - dax/2,                                                            1, 0, 0, 0,  0,  0,  0,  q2/2, -q1/2, -q0/2,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 1, 0, 0, dt,  0,  0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 1, 0,  0, dt,  0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 1,  0,  0, dt,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0
                2*q0*(dvx - dvx_b) - 2*q3*(dvy - dvy_b) + 2*q2*(dvz - dvz_b), 2*q1*(dvx - dvx_b) + 2*q2*(dvy - dvy_b) + 2*q3*(dvz - dvz_b), 2*q1*(dvy - dvy_b) - 2*q2*(dvx - dvx_b) + 2*q0*(dvz - dvz_b), 2*q1*(dvz - dvz_b) - 2*q0*(dvy - dvy_b) - 2*q3*(dvx - dvx_b), 0, 0, 0,  1,  0,  0,     0,     0,     0, - q0^2 - q1^2 + q2^2 + q3^2,           2*q0*q3 - 2*q1*q2,         - 2*q0*q2 - 2*q1*q3, 0, 0, 0, 0, 0, 0
                2*q3*(dvx - dvx_b) + 2*q0*(dvy - dvy_b) - 2*q1*(dvz - dvz_b), 2*q2*(dvx - dvx_b) - 2*q1*(dvy - dvy_b) - 2*q0*(dvz - dvz_b), 2*q1*(dvx - dvx_b) + 2*q2*(dvy - dvy_b) + 2*q3*(dvz - dvz_b), 2*q0*(dvx - dvx_b) - 2*q3*(dvy - dvy_b) + 2*q2*(dvz - dvz_b), 0, 0, 0,  0,  1,  0,     0,     0,     0,         - 2*q0*q3 - 2*q1*q2, - q0^2 + q1^2 - q2^2 + q3^2,           2*q0*q1 - 2*q2*q3, 0, 0, 0, 0, 0, 0
                2*q1*(dvy - dvy_b) - 2*q2*(dvx - dvx_b) + 2*q0*(dvz - dvz_b), 2*q3*(dvx - dvx_b) + 2*q0*(dvy - dvy_b) - 2*q1*(dvz - dvz_b), 2*q3*(dvy - dvy_b) - 2*q0*(dvx - dvx_b) - 2*q2*(dvz - dvz_b), 2*q1*(dvx - dvx_b) + 2*q2*(dvy - dvy_b) + 2*q3*(dvz - dvz_b), 0, 0, 0,  0,  0,  1,     0,     0,     0,           2*q0*q2 - 2*q1*q3,         - 2*q0*q1 - 2*q2*q3, - q0^2 + q1^2 + q2^2 - q3^2, 0, 0, 0, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 0,  0,  0,  0,     1,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 0,  0,  0,  0,     0,     1,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 0,  0,  0,  0,     0,     0,     1,                           0,                           0,                           0, 0, 0, 0, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 0,  0,  0,  0,     0,     0,     0,                           1,                           0,                           0, 0, 0, 0, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 0,  0,  0,  0,     0,     0,     0,                           0,                           1,                           0, 0, 0, 0, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 0,  0,  0,  0,     0,     0,     0,                           0,                           0,                           1, 0, 0, 0, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 0,  0,  0,  0,     0,     0,     0,                           0,                           0,                           0, 1, 0, 0, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 0,  0,  0,  0,     0,     0,     0,                           0,                           0,                           0, 0, 1, 0, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 0,  0,  0,  0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 1, 0, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 0,  0,  0,  0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 1, 0, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 0,  0,  0,  0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 1, 0
                0,                                                            0,                                                            0,                                                            0, 0, 0, 0,  0,  0,  0,     0,     0,     0,                           0,                           0,                           0, 0, 0, 0, 0, 0, 1];


        end

        function G = predict_process_noise(obj,w)
            daxCov = w(1);
            dayCov = w(2);
            dazCov = w(3);
            dvxCov = w(4);
            dvyCov = w(5);
            dvzCov = w(6);


            q0 = obj.x(1);
            q1 = obj.x(2);
            q2 = obj.x(3);
            q3 = obj.x(4);


            G = [ ...
                (daxCov*q1^2)/4 + (dayCov*q2^2)/4 + (dazCov*q3^2)/4, (dayCov*q2*q3)/4 - (daxCov*q0*q1)/4 - (dazCov*q2*q3)/4, (dazCov*q1*q3)/4 - (dayCov*q0*q2)/4 - (daxCov*q1*q3)/4, (daxCov*q1*q2)/4 - (dayCov*q1*q2)/4 - (dazCov*q0*q3)/4, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                (dayCov*q2*q3)/4 - (daxCov*q0*q1)/4 - (dazCov*q2*q3)/4,    (daxCov*q0^2)/4 + (dazCov*q2^2)/4 + (dayCov*q3^2)/4, (daxCov*q0*q3)/4 - (dayCov*q0*q3)/4 - (dazCov*q1*q2)/4, (dazCov*q0*q2)/4 - (dayCov*q1*q3)/4 - (daxCov*q0*q2)/4, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                (dazCov*q1*q3)/4 - (dayCov*q0*q2)/4 - (daxCov*q1*q3)/4, (daxCov*q0*q3)/4 - (dayCov*q0*q3)/4 - (dazCov*q1*q2)/4,    (dayCov*q0^2)/4 + (dazCov*q1^2)/4 + (daxCov*q3^2)/4, (dayCov*q0*q1)/4 - (daxCov*q2*q3)/4 - (dazCov*q0*q1)/4, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                (daxCov*q1*q2)/4 - (dayCov*q1*q2)/4 - (dazCov*q0*q3)/4, (dazCov*q0*q2)/4 - (dayCov*q1*q3)/4 - (daxCov*q0*q2)/4, (dayCov*q0*q1)/4 - (daxCov*q2*q3)/4 - (dazCov*q0*q1)/4,    (dazCov*q0^2)/4 + (dayCov*q1^2)/4 + (daxCov*q2^2)/4, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                               dvyCov*(2*q0*q3 - 2*q1*q2)^2 + dvzCov*(2*q0*q2 + 2*q1*q3)^2 + dvxCov*(q0^2 + q1^2 - q2^2 - q3^2)^2, dvxCov*(2*q0*q3 + 2*q1*q2)*(q0^2 + q1^2 - q2^2 - q3^2) - dvyCov*(2*q0*q3 - 2*q1*q2)*(q0^2 - q1^2 + q2^2 - q3^2) - dvzCov*(2*q0*q1 - 2*q2*q3)*(2*q0*q2 + 2*q1*q3), dvzCov*(2*q0*q2 + 2*q1*q3)*(q0^2 - q1^2 - q2^2 + q3^2) - dvxCov*(2*q0*q2 - 2*q1*q3)*(q0^2 + q1^2 - q2^2 - q3^2) - dvyCov*(2*q0*q1 + 2*q2*q3)*(2*q0*q3 - 2*q1*q2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0, dvxCov*(2*q0*q3 + 2*q1*q2)*(q0^2 + q1^2 - q2^2 - q3^2) - dvyCov*(2*q0*q3 - 2*q1*q2)*(q0^2 - q1^2 + q2^2 - q3^2) - dvzCov*(2*q0*q1 - 2*q2*q3)*(2*q0*q2 + 2*q1*q3),                                                               dvxCov*(2*q0*q3 + 2*q1*q2)^2 + dvzCov*(2*q0*q1 - 2*q2*q3)^2 + dvyCov*(q0^2 - q1^2 + q2^2 - q3^2)^2, dvyCov*(2*q0*q1 + 2*q2*q3)*(q0^2 - q1^2 + q2^2 - q3^2) - dvzCov*(2*q0*q1 - 2*q2*q3)*(q0^2 - q1^2 - q2^2 + q3^2) - dvxCov*(2*q0*q2 - 2*q1*q3)*(2*q0*q3 + 2*q1*q2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0, dvzCov*(2*q0*q2 + 2*q1*q3)*(q0^2 - q1^2 - q2^2 + q3^2) - dvxCov*(2*q0*q2 - 2*q1*q3)*(q0^2 + q1^2 - q2^2 - q3^2) - dvyCov*(2*q0*q1 + 2*q2*q3)*(2*q0*q3 - 2*q1*q2), dvyCov*(2*q0*q1 + 2*q2*q3)*(q0^2 - q1^2 + q2^2 - q3^2) - dvzCov*(2*q0*q1 - 2*q2*q3)*(q0^2 - q1^2 - q2^2 + q3^2) - dvxCov*(2*q0*q2 - 2*q1*q3)*(2*q0*q3 + 2*q1*q2),                                                               dvxCov*(2*q0*q2 - 2*q1*q3)^2 + dvyCov*(2*q0*q1 + 2*q2*q3)^2 + dvzCov*(q0^2 - q1^2 - q2^2 + q3^2)^2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                0,                                                      0,                                                      0,                                                      0, 0, 0, 0,                                                                                                                                                                0,                                                                                                                                                                0,                                                                                                                                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        end

        function x = predict_state(obj,ang_delta,vel_delta,dt)
            x = obj.x;
            q0 = x(1);
            q1 = x(2);
            q2 = x(3);
            q3 = x(4);
            pn = x(5);
            pe = x(6);
            pd = x(7);
            vn = x(8);
            ve = x(9);
            vd = x(10);
            dax_b = x(11);
            day_b = x(12);
            daz_b = x(13);
            dvx_b = x(14);
            dvy_b = x(15);
            dvz_b = x(16);
            magNavX = x(17);
            magNavY = x(18);
            magNavZ = x(19);
            magX = x(20);
            magY = x(21);
            magZ = x(22);

            gnavx = 0;
            gnavy = 0;
            gnavz = 9.81;

            dvx = vel_delta(1);
            dvy = vel_delta(2);
            dvz = vel_delta(3);

            x = [
                q0 % preallocate
                q1 % preallocate
                q2 % preallocate
                q3 % preallocate
                pn + dt*vn
                pe + dt*ve
                pd + dt*vd
                vn + dt*gnavx + (dvx - dvx_b)*(q0^2 + q1^2 - q2^2 - q3^2) - (dvy - dvy_b)*(2*q0*q3 - 2*q1*q2) + (dvz - dvz_b)*(2*q0*q2 + 2*q1*q3)
                ve + dt*gnavy + (dvy - dvy_b)*(q0^2 - q1^2 + q2^2 - q3^2) + (dvx - dvx_b)*(2*q0*q3 + 2*q1*q2) - (dvz - dvz_b)*(2*q0*q1 - 2*q2*q3)
                vd + dt*gnavz + (dvz - dvz_b)*(q0^2 - q1^2 - q2^2 + q3^2) - (dvx - dvx_b)*(2*q0*q2 - 2*q1*q3) + (dvy - dvy_b)*(2*q0*q1 + 2*q2*q3)
                dax_b
                day_b
                daz_b
                dvx_b
                dvy_b
                dvz_b
                magNavX
                magNavY
                magNavZ
                magX
                magY
                magZ];

            qinit = quaternion(q0,q1,q2,q3);
%             x(1:4) = compact(normalize(qinit * quaternion(ang_delta - [dax_b, day_b, daz_b], 'rotvec')));
              delta_q = [1;(ang_delta'-[dax_b; day_b; daz_b])/2];
              x(1:4) = obj.mult_quat([q0;q1;q2;q3],delta_q);
        end

        function qn = mult_quat(obj,q1,q2)
            qnew_0 = q1(1)*q2(1)     -q1(2)*q2(2) - q1(3)*q2(3)-q1(4)*q2(4);
            qnew_1 = q1(1)*q2(2)      +q2(1)*q1(2)+ q1(3)*q2(4)-q2(3)*q1(4);
            qnew_2 = q1(1)*q2(3)      +q2(1)*q1(3) - q1(2)*q2(4)+q2(2)*q1(4);
            qnew_3 = q1(1)*q2(4)+      q2(1)*q1(4) + q1(2)*q2(3)-q2(2)*q1(3);
            qn = [qnew_0;qnew_1;qnew_2;qnew_3];
        end

        function obj = predict_step(obj,accB,omegaB,Ts)
            % state prediction

            ang_delta= omegaB'*Ts;
            vel_delta = accB'*Ts;

            x_new = obj.predict_state(ang_delta,vel_delta,Ts);

            % covariance prediction

            [~,Qs,w] = obj.set_additive_noise(Ts);

            G = obj.predict_process_noise(w);
            F = obj.predict_jacobian(ang_delta,vel_delta,Ts);

            P_new = F*obj.P*(F')+G+Qs;

            x_new = obj.quaternion_normalisation(x_new);
            P_new = 0.5*(P_new+P_new');

            obj.x = x_new;
            obj.P = P_new;

        end

        function x_next = quaternion_normalisation(obj,x)
            x_next = x;
            x_next(1:4) = x(1:4)/norm(x(1:4));
        end

        function [x_new,P_new]  = update_step(obj,z,h_x,H,R)
            nx = size(obj.x,1);
            inov = z-h_x;
            S = H*obj.P*(H')+R;
            K = obj.P*(H')*inv(S);
            x_new = obj.x + K*inov;
            P_new = (eye(nx)-K*H)*obj.P;
        end

        function obj = update_step_gps_pos(obj,lla,lla0)

            z = lla2ned(lla',lla0','flat')';

            Rpos = 2.56;
            R_gps_pos = eye(3)*Rpos;

            H_gps = zeros(3,22);
            H_gps(:,5:7) = eye(3);

            h_x = H_gps*obj.x;

            [x_new,P_new]  = obj.update_step(z,h_x,H_gps,R_gps_pos);

            obj.x = x_new;
            obj.P = P_new;
        end

        function obj = update_step_baro(obj,z)

            R_baro = 0.1;

            H_baro = zeros(1,22);
            H_baro(1,7) = 1.0;

            h_x = H_baro*obj.x;

            [x_new,P_new]  = obj.update_step(z,h_x,H_baro,R_baro);

            obj.x = x_new;
            obj.P = P_new;
        end

        function obj = update_step_gps_vel(obj,z)

            Rvel = 0.01;

            R_gps_vel = eye(3)*Rvel;

            H_gps = zeros(3,22);
            H_gps(:,8:10) = eye(3);

            h_x = H_gps*obj.x;

            [x_new,P_new]  = obj.update_step(z,h_x,H_gps,R_gps_vel);
            obj.x = x_new;
            obj.P = P_new;
        end

                function obj = update_step_gps_vel_2d(obj,z)

            Rvel2d = 0.01;

            R_gps_vel2d = eye(2)*Rvel2d;

            H_gps2d = zeros(2,22);
            H_gps2d(:,8:9) = eye(2);

            h_x = H_gps2d*obj.x;

            [x_new,P_new]  = obj.update_step(z,h_x,H_gps2d,R_gps_vel2d);
            obj.x = x_new;
            obj.P = P_new;
        end

        function obj = update_step_attitude_acc_mag(obj,acc_b,mag_b)


            euler = obj.eulerZYX_from_acc_mag(acc_b,mag_b);
            z = obj.eulZYX2quat(euler);

            Ratt = 0.001;

            R_att = eye(4)*Ratt;

            H_att = zeros(4,22);
            H_att(:,1:4) = eye(4);

            h_x = H_att*obj.x;

            [x_new,P_new]  = obj.update_step(z,h_x,H_att,R_att);
            obj.x = x_new;
            obj.P = P_new;
        end

        function euler = eulerZYX_from_acc_mag(obj,acc_b,mag_b)

            euler = zeros(3,1);
            euler(3) = atan2(acc_b(2),-acc_b(3));
            euler(2) = atan2(-acc_b(1),acc_b(2)*sin(euler(3))-acc_b(3)*cos(euler(3)));
            euler(1) = atan2(mag_b(3)*sin(euler(3))-mag_b(2)*cos(euler(3)),mag_b(1)*cos(euler(3))+mag_b(2)*sin(euler(3))*sin(euler(2))+mag_b(3)*sin(euler(2))*cos(euler(3)));

        end

        function q = eulZYX2quat(obj,euler)
            q = (eul2quat(euler','ZYX'))';
        end
        

        function [obj] = update_step_gps_pos_vel(obj,lla,lla0,gpsVel)

            z_pos = lla2ned(lla',lla0','flat')';
            z = [z_pos;gpsVel];

            Rpos = 2.56;
            Rvel = 0.01;

            R_gps = diag([ones(1,3)*Rpos,ones(1,3)*Rvel]);

            H_gps = zeros(6,22);
            H_gps(:,5:10) = eye(6);

            h_x = H_gps*obj.x;

            [x_new,P_new]  = obj.update_step(obj.x,obj.P,z,h_x,H_gps,R_gps);

            obj.x = x_new;
            obj.P = P_new;
        end


        function H = mag_measurement_jacobian(obj)
            q0 = obj.x(1);
            q1 = obj.x(2);
            q2 = obj.x(3);
            q3 = obj.x(4);
            magNavX = obj.x(17);
            magNavY = obj.x(18);
            magNavZ = obj.x(19);

            H = [ ...
                2*magNavY*q3 - 2*magNavZ*q2 + 2*magNavX*q0, 2*magNavZ*q3 + 2*magNavY*q2 + 2*magNavX*q1, 2*magNavY*q1 - 2*magNavZ*q0 - 2*magNavX*q2, 2*magNavZ*q1 + 2*magNavY*q0 - 2*magNavX*q3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, q0^2 + q1^2 - q2^2 - q3^2,         2*q0*q3 + 2*q1*q2,         2*q1*q3 - 2*q0*q2, 1, 0, 0;
                2*magNavZ*q1 + 2*magNavY*q0 - 2*magNavX*q3, 2*magNavZ*q0 - 2*magNavY*q1 + 2*magNavX*q2, 2*magNavZ*q3 + 2*magNavY*q2 + 2*magNavX*q1, 2*magNavZ*q2 - 2*magNavY*q3 - 2*magNavX*q0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,         2*q1*q2 - 2*q0*q3, q0^2 - q1^2 + q2^2 - q3^2,         2*q0*q1 + 2*q2*q3, 0, 1, 0;
                2*magNavZ*q0 - 2*magNavY*q1 + 2*magNavX*q2, 2*magNavX*q3 - 2*magNavY*q0 - 2*magNavZ*q1, 2*magNavY*q3 - 2*magNavZ*q2 + 2*magNavX*q0, 2*magNavZ*q3 + 2*magNavY*q2 + 2*magNavX*q1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,         2*q0*q2 + 2*q1*q3,         2*q2*q3 - 2*q0*q1, q0^2 - q1^2 - q2^2 + q3^2, 0, 0, 1];


        end


        function z =  mag_measurement_model(obj)
            q0 = obj.x(1);
            q1 = obj.x(2);
            q2 = obj.x(3);
            q3 = obj.x(4);
            magNavX = obj.x(17);
            magNavY = obj.x(18);
            magNavZ = obj.x(19);
            magBiasX = obj.x(20);
            magBiasY = obj.x(21);
            magBiasZ = obj.x(22);

            mx = magBiasX + magNavX*(q0^2 + q1^2 - q2^2 - q3^2) - magNavZ*(2*q0*q2 - 2*q1*q3) + magNavY*(2*q0*q3 + 2*q1*q2);
            my = magBiasY + magNavY*(q0^2 - q1^2 + q2^2 - q3^2) + magNavZ*(2*q0*q1 + 2*q2*q3) - magNavX*(2*q0*q3 - 2*q1*q2);
            mz = magBiasZ + magNavZ*(q0^2 - q1^2 - q2^2 + q3^2) - magNavY*(2*q0*q1 - 2*q2*q3) + magNavX*(2*q0*q2 + 2*q1*q3);

            z = [mx my mz]';
        end

        function   [obj] = update_step_mag(obj,z)

            Rmag = 0.09;
            R_mag = eye(3)*Rmag;

            h_x = obj.mag_measurement_model();

            H_mag = obj.mag_measurement_jacobian();

            [x_new,P_new]  = obj.update_step(z,h_x,H_mag,R_mag);

            obj.x = x_new;
            obj.P = P_new;
        end

        function obj = EKF_rocket(x_init,init_process_cov)
            obj.x = x_init;
            obj.P = ones(22)*init_process_cov;
        end

        function [obj,Qs,w] = set_additive_noise(obj,Ts)
            Fs = 1/Ts;

            obj.scale_var = 0.5*(1./(Fs.^2));
            obj.ang_delta_bias_sigma = obj.scale_var .* obj.GyroscopeBiasNoise;
            obj.vel_delta_bias_sigma = obj.scale_var .* obj.AccelerometerBiasNoise;

            w = obj.scale_var.*[obj.GyroscopeNoise*ones(1,3), obj.AccelerometerNoise*ones(1,3)];

            Qs = diag([obj.additiveNoise.*ones(1,10), obj.ang_delta_bias_sigma*ones(1,3), obj.vel_delta_bias_sigma*ones(1,3),  obj.GeomagneticVectorNoise*ones(1,3), obj.MagnetometerBiasNoise*ones(1,3)]);

        end


    end

end








