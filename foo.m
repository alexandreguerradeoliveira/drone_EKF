function [x_out,euler,p,v,gps_pos,euler_acc_mag,q_from_eul,gps_vel_calculated] = foo(x0,x1,accel,gyro,lla,lla0,gpsvel,gpsvel2d,mag,Ts,P0,init_process_cov,z_baro,eulZYX,Vkmh,course_deg)

ekf = EKF_rocket(x0,init_process_cov);

ekf = ekf.set_state(x1);

ekf = ekf.set_process_cov(P0);

ekf = ekf.predict_step(accel,gyro,Ts);

ekf = ekf.update_step_gps_pos(lla,lla0);

ekf = ekf.update_step_gps_vel(gpsvel);

ekf = ekf.update_step_gps_vel_2d(gpsvel2d);

ekf = ekf.update_step_attitude_acc_mag(accel,mag);

ekf = ekf.update_step_mag(mag);

ekf = ekf.update_step_baro(z_baro);

ekf = ekf.set_mag_vec_from_body_avg(mag);

euler = ekf.get_eulerZYX();

p = ekf.get_posNED();

v = ekf.get_velNED();

x_out = ekf.get_state();

gps_vel_calculated = ekf.gps_vel_2d_from_vel_course(Vkmh,course_deg);

euler_acc_mag = ekf.eulerZYX_from_acc_mag(accel,mag);

q_from_eul = ekf.eulZYX2quat(eulZYX);

gps_pos = ekf.get_gps_local_pos(lla,lla0);


end