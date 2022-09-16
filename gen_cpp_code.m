%% Code for generating c++ EKF library
clc,close all,clear all
%% function inputs
x0 = zeros(22,1);
x1 = ones(22,1);

Ts = 100;

accel = zeros(3,1);
gyro = zeros(3,1);
lla = zeros(3,1);
lla0 = zeros(3,1);
eulZYX = zeros(3,1);
gpsvel = zeros(3,1);
gpsvel2d = zeros(2,1);
mag = zeros(3,1);
z_baro = 0;

P0 = ones(22)*1e-8;
init_process_cov = 1e-9;
Vkmh = 1;
course_deg = 0.01;




%% generate code in c++

cfg = coder.config('lib');
cfg.TargetLang = 'C++';
cfg.InlineBetweenUserFunctions = 'Readability';

codegen -config cfg foo -args {x0,x1,accel,gyro,lla,lla0,gpsvel,gpsvel2d,mag,Ts,P0,init_process_cov,z_baro,eulZYX,Vkmh,course_deg} -report