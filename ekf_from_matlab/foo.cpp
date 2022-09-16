//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// foo.cpp
//
// Code generation for function 'foo'
//

// Include files
#include "foo.h"
#include "EKF_rocket.h"
#include "rt_nonfinite.h"

// Function Definitions
void foo(const double x0[22], const double x1[22], const double accel[3],
         const double gyro[3], const double lla[3], const double lla0[3],
         const double gpsvel[3], const double gpsvel2d[2], const double mag[3],
         double Ts, const double P0[484], double init_process_cov,
         double z_baro, const double eulZYX[3], double Vkmh, double course_deg,
         double x_out[22], double euler[3], double p[3], double v[3],
         double gps_pos[3], double euler_acc_mag[3], double q_from_eul[4],
         double gps_vel_calculated[2])
{
  EKF_rocket ekf;
  ekf.init(x0, init_process_cov);
  ekf.set_state(x1);
  ekf.set_process_cov(P0);
  ekf.predict_step(accel, gyro, Ts);
  ekf.update_step_gps_pos(lla, lla0);
  ekf.update_step_gps_vel(gpsvel);
  ekf.update_step_gps_vel_2d(gpsvel2d);
  ekf.update_step_attitude_acc_mag(accel, mag);
  ekf.update_step_mag(mag);
  ekf.update_step_baro(z_baro);
  ekf.set_mag_vec_from_body_avg(mag);
  ekf.get_eulerZYX(euler);
  ekf.get_posNED(p);
  ekf.get_velNED(v);
  ekf.get_state(x_out);
  EKF_rocket::gps_vel_2d_from_vel_course(Vkmh, course_deg, gps_vel_calculated);
  EKF_rocket::eulerZYX_from_acc_mag(accel, mag, euler_acc_mag);
  EKF_rocket::eulZYX2quat(eulZYX, q_from_eul);
  EKF_rocket::get_gps_local_pos(lla, lla0, gps_pos);
}

// End of code generation (foo.cpp)
