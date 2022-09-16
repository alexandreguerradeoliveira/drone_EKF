//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// foo.h
//
// Code generation for function 'foo'
//

#ifndef FOO_H
#define FOO_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void foo(const double x0[22], const double x1[22], const double accel[3],
                const double gyro[3], const double lla[3], const double lla0[3],
                const double gpsvel[3], const double gpsvel2d[2],
                const double mag[3], double Ts, const double P0[484],
                double init_process_cov, double z_baro, const double eulZYX[3],
                double Vkmh, double course_deg, double x_out[22],
                double euler[3], double p[3], double v[3], double gps_pos[3],
                double euler_acc_mag[3], double q_from_eul[4],
                double gps_vel_calculated[2]);

#endif
// End of code generation (foo.h)
