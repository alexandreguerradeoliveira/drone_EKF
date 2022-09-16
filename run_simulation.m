clc,close all,clear all
% EKF code - Alexandre Guerra de Oliveira
% state variable: x = [ q pos_NED vel_NED ang_bias_body vel_bias_body magvec_NED mag_bias_body ]
% (22x1)

%use realtime plotter from "Pose Estimation From Asynchronous Sensors" from Matlab's exemple
realtime_plots = true;

% custom plots
plot_pos_vel = true;
plot_attitude = true;
plot_bias = true;
plot_magvec = true;

% sensors to use
use_gps = true;
use_mag = true;
fuse_once_gps_pos_vel = false;

%% Load data
% ld = load('CircularTrajectorySensorData.mat');

ld = load("drone_traj.mat");
%% EKF parameters
% sampling rate
Fs = ld.imu.SampleRate;
Fs_gps = ld.gps.SampleRate;
Fs_mag = ld.gps.SampleRate;

Ts = 1/Fs;

%% initial state

Nav = 100;
initstate = zeros(22,1);
initstate(1:4) = compact( meanrot(ld.trajData.Orientation(1:Nav))); 
initstate(5:7) = mean( ld.trajData.Position(1:Nav,:), 1);
initstate(8:10) = mean( ld.trajData.Velocity(1:Nav,:), 1);
initstate(11:13) =  ld.imu.Gyroscope.ConstantBias./Fs;
initstate(14:16) =  ld.imu.Accelerometer.ConstantBias./Fs;
initstate(17:19) =  ld.imu.MagneticField;
initstate(20:22) = ld.imu.Magnetometer.ConstantBias;


P0 = ones(22)*1e-9;


ekf = EKF_rocket(initstate,P0);


%% Plotting stuff (Thanks matlab)

if(realtime_plots)
    useErrScope = true; % Turn on the streaming error plot.
    usePoseView = true; % Turn on the 3D pose viewer.
    if usePoseView
        posescope = PoseViewerWithSwitches(...
            'XPositionLimits', [-30 30], ...
            'YPositionLimits', [-30, 30], ...
            'ZPositionLimits', [-10 10]);
    end
    f = gcf;

    if useErrScope
        errscope = HelperScrollingPlotter(...
            'NumInputs', 4, ...
            'TimeSpan', 10, ...
            'SampleRate', Fs, ...
            'YLabel', {'degrees', ...
            'meters', ...
            'meters', ...
            'meters'}, ...
            'Title', {'Quaternion Distance', ...
            'Position X Error', ...
            'Position Y Error', ...
            'Position Z Error'}, ...
            'YLimits', ...
            [ -1, 2
            -2, 2
            -2 2
            -2 2]);
    end

end


%% Simulation loop
x = initstate;
x_traj = zeros(size(ld.trajData.Position,1),22);
gps_traj = zeros(size(ld.trajData.Position,1),6);

if(~realtime_plots)
tic
end
% for k = 1:size(ld.accel,1)
for k = 1:size(ld.trajData.Position,1)


  [accel, gyro, mag] = ld.imu(ld.trajData.Acceleration(k,:), ld.trajData.AngularVelocity(k, :),ld.trajData.Orientation(k));
  [lla, gpsvel] = ld.gps( ld.trajData.Position(k,:), ld.trajData.Velocity(k,:) );
   accel = -accel;



    ekf = ekf.predict_step(accel',gyro',Ts);



    if((use_gps)&&(mod(k,fix(Fs/Fs_gps))==0))
        if(fuse_once_gps_pos_vel)
%             [x,P] = update_step_gps_pos_vel(x,P,lla',ld.gps.ReferenceLocation',gpsvel');
            ekf = ekf.update_step_gps_pos(lla',ld.gps.ReferenceLocation',gpsvel');
            
        else
            ekf = ekf.update_step_gps_pos(lla',ld.gps.ReferenceLocation');
            ekf = ekf.update_step_gps_vel(gpsvel');

        end
    end


    if(use_mag&&(mod(k,fix(Fs/Fs_mag))==0))
        ekf = ekf.update_step_mag(mag');

    end


        z_gps = lla2ned(lla,ld.gps.ReferenceLocation,'flat');

    x_traj(k,:) = ekf.x';

    gps_traj(k,:) = [z_gps,gpsvel];


    if(realtime_plots)

    q = ekf.x(1:4)';
    q = quaternion(q(1),q(2),q(3),q(4));
    p = ekf.x(5:7)';

    posescope(p, q, ld.trajData.Position(k,:), ld.trajData.Orientation(k));
    
    orientErr = rad2deg(dist(q, ld.trajData.Orientation(k) ));
    posErr = p - ld.trajData.Position(k,:);
    errscope(orientErr, posErr(1), posErr(2), posErr(3));

    end
end

if(~realtime_plots)
toc
end


%% Attitude conversion to euler angles
q_traj_ekf = x_traj(:,1:4);
eulzyx_ekf = rad2deg(quat2eul(q_traj_ekf,'ZYX'));

q_traj_truth = compact(ld.trajData.Orientation);
eulzyx_truth = rad2deg(quat2eul(q_traj_truth,'ZYX'));

%% Plots

% time vector
tt = (0:(size(ld.trajData.Position,1)-1))/Fs;

if(plot_pos_vel)

figure
% pos
subplot(6,1,1)
% plot(tt,gps_traj(:,1),'--')
plot(tt,x_traj(:,5))
hold on
plot(tt,ld.trajData.Position(:,1),'--')
title("Position x axis")

subplot(6,1,2)
plot(tt,x_traj(:,6))
hold on
plot(tt,ld.trajData.Position(:,2),'--')
title("Position y axis")

subplot(6,1,3)
hold on
plot(tt,x_traj(:,7))
plot(tt,ld.trajData.Position(:,3),'--')
title("Position z axis")

% Velocity 
subplot(6,1,4)

% plot(tt,gps_traj(:,4),'--')
hold on
plot(tt,x_traj(:,8))
plot(tt,ld.trajData.Velocity(:,1),'--')
% legend("GPS","EKF","Truth")
title("Velocity inertial x axis")

subplot(6,1,5)
% plot(tt,gps_traj(:,5),'--')
hold on
plot(tt,x_traj(:,9))
plot(tt,ld.trajData.Velocity(:,2),'--')
% legend("GPS","EKF","Truth")
title("Velocity inertial y axis")

subplot(6,1,6)
% plot(tt,gps_traj(:,6),'--')
hold on
plot(tt,x_traj(:,10))
plot(tt,ld.trajData.Velocity(:,3),'--')
% legend("GPS","EKF","Truth")
title("Velocity inertial z axis")

end

if(plot_attitude)

figure

% euler angles
subplot(3,1,1)
plot(tt,eulzyx_ekf(:,1))
hold on
plot(tt,eulzyx_truth(:,1),'--')
title("euler angles 1 (ZYX)")

subplot(3,1,2)
plot(tt,eulzyx_ekf(:,2))
hold on
plot(tt,eulzyx_truth(:,2),'--')
title("euler angles 2 (ZYX)")

subplot(3,1,3)
plot(tt,eulzyx_ekf(:,3))
hold on
plot(tt,eulzyx_truth(:,3),'--')
title("euler angles 3 (ZYX)")



end

if(plot_bias)

% bias plots
figure
% magnetometer bias
subplot(9,1,1)
plot(tt,x_traj(:,20))
title("Magnetometer bias x axis")

subplot(9,1,2)
plot(tt,x_traj(:,21))
title("Magnetometer bias y axis")

subplot(9,1,3)
plot(tt,x_traj(:,22))
title("Magnetometer bias z axis")

% Gyroscope bias 
subplot(9,1,4)
plot(tt,x_traj(:,11))
title("angle bias x axis")

subplot(9,1,5)
plot(tt,x_traj(:,12))
title("angle bias y axis")

subplot(9,1,6)
plot(tt,x_traj(:,13))
title("angle bias z axis")

% Accelerometer bias
subplot(9,1,7)
plot(tt,x_traj(:,14))
title("Accelerometer bias x axis")

subplot(9,1,8)
plot(tt,x_traj(:,15))
title("Accelerometer bias y axis")

subplot(9,1,9)
plot(tt,x_traj(:,16))
title("Accelerometer inertial z axis")

end

if(plot_magvec)

figure
subplot(3,1,1)
plot(tt,x_traj(:,17))
title("Magnetic vector inertial x axis")

subplot(3,1,2)
plot(tt,x_traj(:,18))
title("Magnetic vector inertial y axis")

subplot(3,1,3)
plot(tt,x_traj(:,19))
title("Magnetic vector inertial z axis")

end



