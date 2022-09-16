//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EKF_rocket.h
//
// Code generation for function 'EKF_rocket'
//

#ifndef EKF_ROCKET_H
#define EKF_ROCKET_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class EKF_rocket {
public:
  void init(const double x_init[22], double init_process_cov);
  void predict_state(const double ang_delta[3], const double vel_delta[3],
                     double dt, double b_x[22]) const;
  static void mult_quat(const double q1[4], const double q2[4], double qn[4]);
  void predict_process_noise(const double w[6], double G[484]) const;
  void predict_jacobian(const double ang_delta[3], const double vel_delta[3],
                        double dt, double F[484]) const;
  void update_step(const double z[3], const double h_x[3], const double H[66],
                   const double R[9], double x_new[22],
                   double P_new[484]) const;
  static void eulerZYX_from_acc_mag(const double acc_b[3],
                                    const double mag_b[3], double euler[3]);
  static void eulZYX2quat(const double euler[3], double q[4]);
  void mag_measurement_model(double z[3]) const;
  void mag_measurement_jacobian(double H[66]) const;
  void get_rotmat_body_2_inertial(double rot_mat[9]) const;
  void get_eulerZYX(double euler[3]) const;
  void get_posNED(double p[3]) const;
  void get_velNED(double v[3]) const;
  void get_state(double b_x[22]) const;
  static void gps_vel_2d_from_vel_course(double V_kmh, double course_deg,
                                         double gps_vel_2d[2]);
  static void get_gps_local_pos(const double lla[3], const double lla0[3],
                                double gps_pos[3]);
  void update_step(const double z[2], const double h_x[2], const double H[44],
                   double x_new[22], double P_new[484]) const;
  void b_update_step(const double z[4], const double h_x[4], const double H[88],
                     double x_new[22], double P_new[484]) const;
  void set_state(const double b_x[22]);
  void set_process_cov(const double b_P[484]);
  void predict_step(const double accB[3], const double omegaB[3], double Ts);
  void set_additive_noise(double Ts, double Qs[484], double w[6]);
  static void quaternion_normalisation(double b_x[22]);
  void update_step_gps_pos(const double lla[3], const double lla0[3]);
  void update_step_gps_vel(const double z[3]);
  void update_step_gps_vel_2d(const double z[2]);
  void update_step_attitude_acc_mag(const double acc_b[3],
                                    const double mag_b[3]);
  void update_step_mag(const double z[3]);
  void update_step_baro(double z);
  void set_mag_vec_from_body_avg(const double magB[3]);
  double x[22];
  double P[484];
  double AccelerometerNoise;
  double GyroscopeNoise;
  double AccelerometerBiasNoise;
  double GyroscopeBiasNoise;
  double MagnetometerBiasNoise;
  double GeomagneticVectorNoise;
  double additiveNoise;
  double scale_var;
  double ang_delta_bias_sigma;
  double vel_delta_bias_sigma;
};

#endif
// End of code generation (EKF_rocket.h)
