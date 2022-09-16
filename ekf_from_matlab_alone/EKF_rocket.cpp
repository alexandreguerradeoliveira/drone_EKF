//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EKF_rocket.cpp
//
// Code generation for function 'EKF_rocket'
//

// Include files
#include "EKF_rocket.h"
#include "foo_rtwutil.h"
#include "lla2ned.h"
#include "quat2eul.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Variable Definitions
static const signed char iv[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};

// Function Definitions
void EKF_rocket::b_update_step(const double z[4], const double h_x[4],
                               const double H[88], double x_new[22],
                               double P_new[484]) const
{
  static const double dv[16]{0.001, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0,
                             0.0,   0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.001};
  double c_I[484];
  double K[88];
  double b_H[88];
  double b[16];
  double b_x[16];
  double s;
  double smax;
  double z_idx_2;
  double z_idx_3;
  int b_tmp;
  int i;
  int i1;
  int jA;
  int kAcol;
  signed char b_I[484];
  signed char ipiv[4];
  signed char p[4];
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 22; i1++) {
      K[i1 + 22 * i] = H[i + (i1 << 2)];
    }
  }
  std::memset(&b[0], 0, 16U * sizeof(double));
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 22; i1++) {
      smax = 0.0;
      for (jA = 0; jA < 22; jA++) {
        smax += H[i + (jA << 2)] * P[jA + 22 * i1];
      }
      b_H[i + (i1 << 2)] = smax;
    }
    for (i1 = 0; i1 < 4; i1++) {
      smax = 0.0;
      for (jA = 0; jA < 22; jA++) {
        smax += b_H[i + (jA << 2)] * K[jA + 22 * i1];
      }
      jA = i + (i1 << 2);
      b_x[jA] = smax + dv[jA];
    }
    ipiv[i] = static_cast<signed char>(i + 1);
  }
  for (int j{0}; j < 3; j++) {
    int jp1j;
    int mmj_tmp;
    mmj_tmp = 2 - j;
    b_tmp = j * 5;
    jp1j = b_tmp + 2;
    jA = 4 - j;
    kAcol = 0;
    smax = std::abs(b_x[b_tmp]);
    for (int k{2}; k <= jA; k++) {
      s = std::abs(b_x[(b_tmp + k) - 1]);
      if (s > smax) {
        kAcol = k - 1;
        smax = s;
      }
    }
    if (b_x[b_tmp + kAcol] != 0.0) {
      if (kAcol != 0) {
        jA = j + kAcol;
        ipiv[j] = static_cast<signed char>(jA + 1);
        smax = b_x[j];
        b_x[j] = b_x[jA];
        b_x[jA] = smax;
        smax = b_x[j + 4];
        b_x[j + 4] = b_x[jA + 4];
        b_x[jA + 4] = smax;
        smax = b_x[j + 8];
        b_x[j + 8] = b_x[jA + 8];
        b_x[jA + 8] = smax;
        smax = b_x[j + 12];
        b_x[j + 12] = b_x[jA + 12];
        b_x[jA + 12] = smax;
      }
      i = (b_tmp - j) + 4;
      for (int b_i{jp1j}; b_i <= i; b_i++) {
        b_x[b_i - 1] /= b_x[b_tmp];
      }
    }
    jA = b_tmp;
    for (kAcol = 0; kAcol <= mmj_tmp; kAcol++) {
      smax = b_x[(b_tmp + (kAcol << 2)) + 4];
      if (smax != 0.0) {
        i = jA + 6;
        i1 = (jA - j) + 8;
        for (jp1j = i; jp1j <= i1; jp1j++) {
          b_x[jp1j - 1] += b_x[((b_tmp + jp1j) - jA) - 5] * -smax;
        }
      }
      jA += 4;
    }
  }
  p[0] = 1;
  p[1] = 2;
  p[2] = 3;
  p[3] = 4;
  if (ipiv[0] > 1) {
    jA = p[ipiv[0] - 1];
    p[ipiv[0] - 1] = 1;
    p[0] = static_cast<signed char>(jA);
  }
  if (ipiv[1] > 2) {
    jA = p[ipiv[1] - 1];
    p[ipiv[1] - 1] = p[1];
    p[1] = static_cast<signed char>(jA);
  }
  if (ipiv[2] > 3) {
    jA = p[ipiv[2] - 1];
    p[ipiv[2] - 1] = p[2];
    p[2] = static_cast<signed char>(jA);
  }
  for (int k{0}; k < 4; k++) {
    b_tmp = (p[k] - 1) << 2;
    b[k + b_tmp] = 1.0;
    for (int j{k + 1}; j < 5; j++) {
      i = (j + b_tmp) - 1;
      if (b[i] != 0.0) {
        i1 = j + 1;
        for (int b_i{i1}; b_i < 5; b_i++) {
          jA = (b_i + b_tmp) - 1;
          b[jA] -= b[i] * b_x[(b_i + ((j - 1) << 2)) - 1];
        }
      }
    }
  }
  for (int j{0}; j < 4; j++) {
    jA = j << 2;
    for (int k{3}; k >= 0; k--) {
      kAcol = k << 2;
      i = k + jA;
      smax = b[i];
      if (smax != 0.0) {
        b[i] = smax / b_x[k + kAcol];
        for (int b_i{0}; b_i < k; b_i++) {
          b_tmp = b_i + jA;
          b[b_tmp] -= b[i] * b_x[b_i + kAcol];
        }
      }
    }
  }
  for (i = 0; i < 22; i++) {
    for (i1 = 0; i1 < 4; i1++) {
      smax = 0.0;
      for (jA = 0; jA < 22; jA++) {
        smax += P[i + 22 * jA] * K[jA + 22 * i1];
      }
      b_H[i + 22 * i1] = smax;
    }
  }
  for (i = 0; i < 22; i++) {
    smax = b_H[i];
    s = b_H[i + 22];
    z_idx_2 = b_H[i + 44];
    z_idx_3 = b_H[i + 66];
    for (i1 = 0; i1 < 4; i1++) {
      jA = i1 << 2;
      K[i + 22 * i1] = ((smax * b[jA] + s * b[jA + 1]) + z_idx_2 * b[jA + 2]) +
                       z_idx_3 * b[jA + 3];
    }
  }
  smax = z[0] - h_x[0];
  s = z[1] - h_x[1];
  z_idx_2 = z[2] - h_x[2];
  z_idx_3 = z[3] - h_x[3];
  for (i = 0; i < 22; i++) {
    x_new[i] = x[i] + (((K[i] * smax + K[i + 22] * s) + K[i + 44] * z_idx_2) +
                       K[i + 66] * z_idx_3);
  }
  std::memset(&b_I[0], 0, 484U * sizeof(signed char));
  for (int k{0}; k < 22; k++) {
    b_I[k + 22 * k] = 1;
  }
  for (i = 0; i < 22; i++) {
    smax = K[i];
    s = K[i + 22];
    z_idx_2 = K[i + 44];
    z_idx_3 = K[i + 66];
    for (i1 = 0; i1 < 22; i1++) {
      jA = i1 << 2;
      kAcol = i + 22 * i1;
      c_I[kAcol] = static_cast<double>(b_I[kAcol]) -
                   (((smax * H[jA] + s * H[jA + 1]) + z_idx_2 * H[jA + 2]) +
                    z_idx_3 * H[jA + 3]);
    }
    for (i1 = 0; i1 < 22; i1++) {
      smax = 0.0;
      for (jA = 0; jA < 22; jA++) {
        smax += c_I[i + 22 * jA] * P[jA + 22 * i1];
      }
      P_new[i + 22 * i1] = smax;
    }
  }
}

void EKF_rocket::get_rotmat_body_2_inertial(double rot_mat[9]) const
{
  double b_rot_mat_tmp;
  double c_rot_mat_tmp;
  double d_rot_mat_tmp;
  double e_rot_mat_tmp;
  double f_rot_mat_tmp;
  double g_rot_mat_tmp;
  double h_rot_mat_tmp;
  double rot_mat_tmp;
  rot_mat_tmp = x[0] * x[0];
  b_rot_mat_tmp = x[1] * x[1];
  c_rot_mat_tmp = x[2] * x[2];
  d_rot_mat_tmp = x[3] * x[3];
  rot_mat[0] = ((rot_mat_tmp + b_rot_mat_tmp) - c_rot_mat_tmp) - d_rot_mat_tmp;
  e_rot_mat_tmp = x[1] * x[2];
  f_rot_mat_tmp = x[0] * x[3];
  rot_mat[3] = 2.0 * (e_rot_mat_tmp - f_rot_mat_tmp);
  g_rot_mat_tmp = x[1] * x[3];
  h_rot_mat_tmp = x[0] * x[2];
  rot_mat[6] = 2.0 * (g_rot_mat_tmp + h_rot_mat_tmp);
  rot_mat[1] = 2.0 * (e_rot_mat_tmp + f_rot_mat_tmp);
  rot_mat_tmp -= b_rot_mat_tmp;
  rot_mat[4] = (rot_mat_tmp + c_rot_mat_tmp) - d_rot_mat_tmp;
  b_rot_mat_tmp = x[2] * x[3];
  e_rot_mat_tmp = x[0] * x[1];
  rot_mat[7] = 2.0 * (b_rot_mat_tmp - e_rot_mat_tmp);
  rot_mat[2] = 2.0 * (g_rot_mat_tmp - h_rot_mat_tmp);
  rot_mat[5] = 2.0 * (b_rot_mat_tmp + e_rot_mat_tmp);
  rot_mat[8] = (rot_mat_tmp - c_rot_mat_tmp) + d_rot_mat_tmp;
}

void EKF_rocket::mag_measurement_jacobian(double H[66]) const
{
  double H_tmp;
  double H_tmp_tmp;
  double b_H_tmp;
  double b_H_tmp_tmp;
  double c_H_tmp;
  double c_H_tmp_tmp;
  double d_H_tmp;
  double e_H_tmp;
  double f_H_tmp;
  double g_H_tmp;
  double h_H_tmp;
  double i_H_tmp;
  double j_H_tmp;
  double k_H_tmp;
  double l_H_tmp;
  double m_H_tmp;
  double n_H_tmp;
  double o_H_tmp;
  double p_H_tmp;
  double q_H_tmp;
  H_tmp = 2.0 * x[18] * x[2];
  b_H_tmp = 2.0 * x[17] * x[3];
  c_H_tmp = 2.0 * x[16] * x[0];
  d_H_tmp = (b_H_tmp - H_tmp) + c_H_tmp;
  H[0] = d_H_tmp;
  e_H_tmp = (2.0 * x[18] * x[3] + 2.0 * x[17] * x[2]) + 2.0 * x[16] * x[1];
  H[3] = e_H_tmp;
  f_H_tmp = 2.0 * x[18] * x[0];
  g_H_tmp = 2.0 * x[17] * x[1];
  h_H_tmp = 2.0 * x[16] * x[2];
  H[6] = (g_H_tmp - f_H_tmp) - h_H_tmp;
  H_tmp_tmp = 2.0 * x[16] * x[3];
  b_H_tmp_tmp = 2.0 * x[17] * x[0];
  c_H_tmp_tmp = 2.0 * x[18] * x[1];
  i_H_tmp = (c_H_tmp_tmp + b_H_tmp_tmp) - H_tmp_tmp;
  H[9] = i_H_tmp;
  H[12] = 0.0;
  H[15] = 0.0;
  H[18] = 0.0;
  H[21] = 0.0;
  H[24] = 0.0;
  H[27] = 0.0;
  H[30] = 0.0;
  H[33] = 0.0;
  H[36] = 0.0;
  H[39] = 0.0;
  H[42] = 0.0;
  H[45] = 0.0;
  j_H_tmp = x[0] * x[0];
  k_H_tmp = x[1] * x[1];
  l_H_tmp = x[2] * x[2];
  m_H_tmp = x[3] * x[3];
  H[48] = ((j_H_tmp + k_H_tmp) - l_H_tmp) - m_H_tmp;
  n_H_tmp = 2.0 * x[1] * x[2];
  o_H_tmp = 2.0 * x[0] * x[3];
  H[51] = o_H_tmp + n_H_tmp;
  p_H_tmp = 2.0 * x[0] * x[2];
  q_H_tmp = 2.0 * x[1] * x[3];
  H[54] = q_H_tmp - p_H_tmp;
  H[57] = 1.0;
  H[60] = 0.0;
  H[63] = 0.0;
  H[1] = i_H_tmp;
  f_H_tmp = (f_H_tmp - g_H_tmp) + h_H_tmp;
  H[4] = f_H_tmp;
  H[7] = e_H_tmp;
  H[10] = (H_tmp - b_H_tmp) - c_H_tmp;
  H[13] = 0.0;
  H[16] = 0.0;
  H[19] = 0.0;
  H[22] = 0.0;
  H[25] = 0.0;
  H[28] = 0.0;
  H[31] = 0.0;
  H[34] = 0.0;
  H[37] = 0.0;
  H[40] = 0.0;
  H[43] = 0.0;
  H[46] = 0.0;
  H[49] = n_H_tmp - o_H_tmp;
  H_tmp = j_H_tmp - k_H_tmp;
  H[52] = (H_tmp + l_H_tmp) - m_H_tmp;
  b_H_tmp = 2.0 * x[2] * x[3];
  c_H_tmp = 2.0 * x[0] * x[1];
  H[55] = c_H_tmp + b_H_tmp;
  H[58] = 0.0;
  H[61] = 1.0;
  H[64] = 0.0;
  H[2] = f_H_tmp;
  H[5] = (H_tmp_tmp - b_H_tmp_tmp) - c_H_tmp_tmp;
  H[8] = d_H_tmp;
  H[11] = e_H_tmp;
  H[14] = 0.0;
  H[17] = 0.0;
  H[20] = 0.0;
  H[23] = 0.0;
  H[26] = 0.0;
  H[29] = 0.0;
  H[32] = 0.0;
  H[35] = 0.0;
  H[38] = 0.0;
  H[41] = 0.0;
  H[44] = 0.0;
  H[47] = 0.0;
  H[50] = p_H_tmp + q_H_tmp;
  H[53] = b_H_tmp - c_H_tmp;
  H[56] = (H_tmp - l_H_tmp) + m_H_tmp;
  H[59] = 0.0;
  H[62] = 0.0;
  H[65] = 1.0;
}

void EKF_rocket::mag_measurement_model(double z[3]) const
{
  double b_z_tmp;
  double c_z_tmp;
  double d_z_tmp;
  double e_z_tmp;
  double f_z_tmp;
  double g_z_tmp;
  double h_z_tmp;
  double i_z_tmp;
  double z_tmp;
  z_tmp = x[0] * x[0];
  b_z_tmp = x[1] * x[1];
  c_z_tmp = x[2] * x[2];
  d_z_tmp = x[3] * x[3];
  e_z_tmp = 2.0 * x[0] * x[3];
  f_z_tmp = 2.0 * x[1] * x[2];
  g_z_tmp = 2.0 * x[0] * x[2];
  h_z_tmp = 2.0 * x[1] * x[3];
  z[0] = ((x[19] + x[16] * (((z_tmp + b_z_tmp) - c_z_tmp) - d_z_tmp)) -
          x[18] * (g_z_tmp - h_z_tmp)) +
         x[17] * (e_z_tmp + f_z_tmp);
  z_tmp -= b_z_tmp;
  b_z_tmp = 2.0 * x[0] * x[1];
  i_z_tmp = 2.0 * x[2] * x[3];
  z[1] = ((x[20] + x[17] * ((z_tmp + c_z_tmp) - d_z_tmp)) +
          x[18] * (b_z_tmp + i_z_tmp)) -
         x[16] * (e_z_tmp - f_z_tmp);
  z[2] = ((x[21] + x[18] * ((z_tmp - c_z_tmp) + d_z_tmp)) -
          x[17] * (b_z_tmp - i_z_tmp)) +
         x[16] * (g_z_tmp + h_z_tmp);
}

void EKF_rocket::mult_quat(const double q1[4], const double q2[4], double qn[4])
{
  qn[0] = ((q1[0] - q1[1] * q2[1]) - q1[2] * q2[2]) - q1[3] * q2[3];
  qn[1] = ((q1[0] * q2[1] + q1[1]) + q1[2] * q2[3]) - q2[2] * q1[3];
  qn[2] = ((q1[0] * q2[2] + q1[2]) - q1[1] * q2[3]) + q2[1] * q1[3];
  qn[3] = ((q1[0] * q2[3] + q1[3]) + q1[1] * q2[2]) - q2[1] * q1[2];
}

void EKF_rocket::predict_jacobian(const double ang_delta[3],
                                  const double vel_delta[3], double dt,
                                  double F[484]) const
{
  static const signed char b_iv[22]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const signed char iv1[22]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const signed char iv10[22]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0};
  static const signed char iv11[22]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const signed char iv2[22]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static const signed char iv3[22]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0};
  static const signed char iv4[22]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
  static const signed char iv5[22]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0};
  static const signed char iv6[22]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0};
  static const signed char iv7[22]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0};
  static const signed char iv8[22]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
  static const signed char iv9[22]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0};
  double F_tmp;
  double F_tmp_tmp;
  double b_F_tmp;
  double b_F_tmp_tmp;
  double c_F_tmp;
  double c_F_tmp_tmp;
  double d_F_tmp;
  double e_F_tmp;
  double f_F_tmp;
  double g_F_tmp;
  double h_F_tmp;
  double i_F_tmp;
  double j_F_tmp;
  double k_F_tmp;
  double l_F_tmp;
  double m_F_tmp;
  double n_F_tmp;
  double o_F_tmp;
  F[0] = 1.0;
  F_tmp = x[10] / 2.0 - ang_delta[0] / 2.0;
  F[22] = F_tmp;
  b_F_tmp = x[11] / 2.0 - ang_delta[1] / 2.0;
  F[44] = b_F_tmp;
  c_F_tmp = x[12] / 2.0 - ang_delta[2] / 2.0;
  F[66] = c_F_tmp;
  F[88] = 0.0;
  F[110] = 0.0;
  F[132] = 0.0;
  F[154] = 0.0;
  F[176] = 0.0;
  F[198] = 0.0;
  F[220] = x[1] / 2.0;
  F[242] = x[2] / 2.0;
  F[264] = x[3] / 2.0;
  F[286] = 0.0;
  F[308] = 0.0;
  F[330] = 0.0;
  F[352] = 0.0;
  F[374] = 0.0;
  F[396] = 0.0;
  F[418] = 0.0;
  F[440] = 0.0;
  F[462] = 0.0;
  d_F_tmp = ang_delta[0] / 2.0 - x[10] / 2.0;
  F[1] = d_F_tmp;
  F[23] = 1.0;
  e_F_tmp = ang_delta[2] / 2.0 - x[12] / 2.0;
  F[45] = e_F_tmp;
  F[67] = b_F_tmp;
  F[89] = 0.0;
  F[111] = 0.0;
  F[133] = 0.0;
  F[155] = 0.0;
  F[177] = 0.0;
  F[199] = 0.0;
  b_F_tmp = -x[0] / 2.0;
  F[221] = b_F_tmp;
  F[243] = x[3] / 2.0;
  F[265] = -x[2] / 2.0;
  F[287] = 0.0;
  F[309] = 0.0;
  F[331] = 0.0;
  F[353] = 0.0;
  F[375] = 0.0;
  F[397] = 0.0;
  F[419] = 0.0;
  F[441] = 0.0;
  F[463] = 0.0;
  f_F_tmp = ang_delta[1] / 2.0 - x[11] / 2.0;
  F[2] = f_F_tmp;
  F[24] = c_F_tmp;
  F[46] = 1.0;
  F[68] = d_F_tmp;
  F[90] = 0.0;
  F[112] = 0.0;
  F[134] = 0.0;
  F[156] = 0.0;
  F[178] = 0.0;
  F[200] = 0.0;
  F[222] = -x[3] / 2.0;
  F[244] = b_F_tmp;
  F[266] = x[1] / 2.0;
  F[288] = 0.0;
  F[310] = 0.0;
  F[332] = 0.0;
  F[354] = 0.0;
  F[376] = 0.0;
  F[398] = 0.0;
  F[420] = 0.0;
  F[442] = 0.0;
  F[464] = 0.0;
  F[3] = e_F_tmp;
  F[25] = f_F_tmp;
  F[47] = F_tmp;
  F[69] = 1.0;
  F[91] = 0.0;
  F[113] = 0.0;
  F[135] = 0.0;
  F[157] = 0.0;
  F[179] = 0.0;
  F[201] = 0.0;
  F[223] = x[2] / 2.0;
  F[245] = -x[1] / 2.0;
  F[267] = b_F_tmp;
  F[289] = 0.0;
  F[311] = 0.0;
  F[333] = 0.0;
  F[355] = 0.0;
  F[377] = 0.0;
  F[399] = 0.0;
  F[421] = 0.0;
  F[443] = 0.0;
  F[465] = 0.0;
  F[4] = 0.0;
  F[26] = 0.0;
  F[48] = 0.0;
  F[70] = 0.0;
  F[92] = 1.0;
  F[114] = 0.0;
  F[136] = 0.0;
  F[158] = dt;
  F[180] = 0.0;
  F[202] = 0.0;
  F[224] = 0.0;
  F[246] = 0.0;
  F[268] = 0.0;
  F[290] = 0.0;
  F[312] = 0.0;
  F[334] = 0.0;
  F[356] = 0.0;
  F[378] = 0.0;
  F[400] = 0.0;
  F[422] = 0.0;
  F[444] = 0.0;
  F[466] = 0.0;
  F[5] = 0.0;
  F[27] = 0.0;
  F[49] = 0.0;
  F[71] = 0.0;
  F[93] = 0.0;
  F[115] = 1.0;
  F[137] = 0.0;
  F[159] = 0.0;
  F[181] = dt;
  F[203] = 0.0;
  F[225] = 0.0;
  F[247] = 0.0;
  F[269] = 0.0;
  F[291] = 0.0;
  F[313] = 0.0;
  F[335] = 0.0;
  F[357] = 0.0;
  F[379] = 0.0;
  F[401] = 0.0;
  F[423] = 0.0;
  F[445] = 0.0;
  F[467] = 0.0;
  F[6] = 0.0;
  F[28] = 0.0;
  F[50] = 0.0;
  F[72] = 0.0;
  F[94] = 0.0;
  F[116] = 0.0;
  F[138] = 1.0;
  F[160] = 0.0;
  F[182] = 0.0;
  F[204] = dt;
  F[226] = 0.0;
  F[248] = 0.0;
  F[270] = 0.0;
  F[292] = 0.0;
  F[314] = 0.0;
  F[336] = 0.0;
  F[358] = 0.0;
  F[380] = 0.0;
  F[402] = 0.0;
  F[424] = 0.0;
  F[446] = 0.0;
  F[468] = 0.0;
  F_tmp = vel_delta[0] - x[13];
  b_F_tmp = vel_delta[1] - x[14];
  c_F_tmp = vel_delta[2] - x[15];
  F_tmp_tmp = 2.0 * x[3] * b_F_tmp;
  b_F_tmp_tmp = 2.0 * x[0] * F_tmp;
  c_F_tmp_tmp = 2.0 * x[2] * c_F_tmp;
  d_F_tmp = (b_F_tmp_tmp - F_tmp_tmp) + c_F_tmp_tmp;
  F[7] = d_F_tmp;
  e_F_tmp = (2.0 * x[1] * F_tmp + 2.0 * x[2] * b_F_tmp) + 2.0 * x[3] * c_F_tmp;
  F[29] = e_F_tmp;
  f_F_tmp = 2.0 * x[2] * F_tmp;
  g_F_tmp = 2.0 * x[1] * b_F_tmp;
  h_F_tmp = 2.0 * x[0] * c_F_tmp;
  i_F_tmp = (g_F_tmp - f_F_tmp) + h_F_tmp;
  F[51] = i_F_tmp;
  F_tmp *= 2.0 * x[3];
  b_F_tmp *= 2.0 * x[0];
  c_F_tmp *= 2.0 * x[1];
  F[73] = (c_F_tmp - b_F_tmp) - F_tmp;
  F[95] = 0.0;
  F[117] = 0.0;
  F[139] = 0.0;
  F[161] = 1.0;
  F[183] = 0.0;
  F[205] = 0.0;
  F[227] = 0.0;
  F[249] = 0.0;
  F[271] = 0.0;
  j_F_tmp = -(x[0] * x[0]);
  k_F_tmp = x[1] * x[1];
  l_F_tmp = x[2] * x[2];
  m_F_tmp = x[3] * x[3];
  F[293] = ((j_F_tmp - k_F_tmp) + l_F_tmp) + m_F_tmp;
  n_F_tmp = 2.0 * x[1] * x[2];
  F[315] = 2.0 * x[0] * x[3] - n_F_tmp;
  o_F_tmp = 2.0 * x[1] * x[3];
  F[337] = -2.0 * x[0] * x[2] - o_F_tmp;
  F[359] = 0.0;
  F[381] = 0.0;
  F[403] = 0.0;
  F[425] = 0.0;
  F[447] = 0.0;
  F[469] = 0.0;
  F_tmp = (F_tmp + b_F_tmp) - c_F_tmp;
  F[8] = F_tmp;
  F[30] = (f_F_tmp - g_F_tmp) - h_F_tmp;
  F[52] = e_F_tmp;
  F[74] = d_F_tmp;
  F[96] = 0.0;
  F[118] = 0.0;
  F[140] = 0.0;
  F[162] = 0.0;
  F[184] = 1.0;
  F[206] = 0.0;
  F[228] = 0.0;
  F[250] = 0.0;
  F[272] = 0.0;
  F[294] = -2.0 * x[0] * x[3] - n_F_tmp;
  b_F_tmp = j_F_tmp + k_F_tmp;
  F[316] = (b_F_tmp - l_F_tmp) + m_F_tmp;
  c_F_tmp = 2.0 * x[2] * x[3];
  F[338] = 2.0 * x[0] * x[1] - c_F_tmp;
  F[360] = 0.0;
  F[382] = 0.0;
  F[404] = 0.0;
  F[426] = 0.0;
  F[448] = 0.0;
  F[470] = 0.0;
  F[9] = i_F_tmp;
  F[31] = F_tmp;
  F[53] = (F_tmp_tmp - b_F_tmp_tmp) - c_F_tmp_tmp;
  F[75] = e_F_tmp;
  F[97] = 0.0;
  F[119] = 0.0;
  F[141] = 0.0;
  F[163] = 0.0;
  F[185] = 0.0;
  F[207] = 1.0;
  F[229] = 0.0;
  F[251] = 0.0;
  F[273] = 0.0;
  F[295] = 2.0 * x[0] * x[2] - o_F_tmp;
  F[317] = -2.0 * x[0] * x[1] - c_F_tmp;
  F[339] = (b_F_tmp + l_F_tmp) - m_F_tmp;
  F[361] = 0.0;
  F[383] = 0.0;
  F[405] = 0.0;
  F[427] = 0.0;
  F[449] = 0.0;
  F[471] = 0.0;
  for (int i{0}; i < 22; i++) {
    F[22 * i + 10] = b_iv[i];
    F[22 * i + 11] = iv1[i];
    F[22 * i + 12] = iv2[i];
    F[22 * i + 13] = iv3[i];
    F[22 * i + 14] = iv4[i];
    F[22 * i + 15] = iv5[i];
    F[22 * i + 16] = iv6[i];
    F[22 * i + 17] = iv7[i];
    F[22 * i + 18] = iv8[i];
    F[22 * i + 19] = iv9[i];
    F[22 * i + 20] = iv10[i];
    F[22 * i + 21] = iv11[i];
  }
}

void EKF_rocket::predict_process_noise(const double w[6], double G[484]) const
{
  double G_tmp;
  double a_tmp;
  double b_G_tmp;
  double b_a_tmp;
  double c_G_tmp;
  double c_a_tmp;
  double d_G_tmp;
  double d_a_tmp;
  double e_G_tmp;
  double e_a_tmp;
  double f_G_tmp;
  double f_a_tmp;
  double g_a_tmp;
  double h_a_tmp;
  double i_a_tmp;
  double j_a_tmp;
  double k_a_tmp;
  double l_a_tmp;
  double m_a_tmp;
  double n_a_tmp;
  double o_a_tmp;
  a_tmp = 2.0 * x[0] * x[3];
  b_a_tmp = 2.0 * x[1] * x[2];
  c_a_tmp = a_tmp - b_a_tmp;
  d_a_tmp = 2.0 * x[0] * x[2];
  e_a_tmp = 2.0 * x[1] * x[3];
  f_a_tmp = d_a_tmp + e_a_tmp;
  g_a_tmp = x[0] * x[0];
  h_a_tmp = x[1] * x[1];
  i_a_tmp = x[2] * x[2];
  j_a_tmp = x[3] * x[3];
  k_a_tmp = ((g_a_tmp + h_a_tmp) - i_a_tmp) - j_a_tmp;
  a_tmp += b_a_tmp;
  b_a_tmp = 2.0 * x[0] * x[1];
  l_a_tmp = 2.0 * x[2] * x[3];
  m_a_tmp = b_a_tmp - l_a_tmp;
  n_a_tmp = g_a_tmp - h_a_tmp;
  o_a_tmp = (n_a_tmp + i_a_tmp) - j_a_tmp;
  d_a_tmp -= e_a_tmp;
  b_a_tmp += l_a_tmp;
  e_a_tmp = (n_a_tmp - i_a_tmp) + j_a_tmp;
  G[0] = (w[0] * h_a_tmp / 4.0 + w[1] * i_a_tmp / 4.0) + w[2] * j_a_tmp / 4.0;
  l_a_tmp = w[0] * x[0];
  n_a_tmp = (w[1] * x[2] * x[3] / 4.0 - l_a_tmp * x[1] / 4.0) -
            w[2] * x[2] * x[3] / 4.0;
  G[22] = n_a_tmp;
  G_tmp = w[0] * x[1];
  b_G_tmp = x[0] * w[1];
  c_G_tmp = x[1] * w[2];
  d_G_tmp = (c_G_tmp * x[3] / 4.0 - b_G_tmp * x[2] / 4.0) - G_tmp * x[3] / 4.0;
  G[44] = d_G_tmp;
  e_G_tmp = x[0] * w[2];
  f_G_tmp = w[1] * x[1];
  G_tmp = (G_tmp * x[2] / 4.0 - f_G_tmp * x[2] / 4.0) - e_G_tmp * x[3] / 4.0;
  G[66] = G_tmp;
  G[88] = 0.0;
  G[110] = 0.0;
  G[132] = 0.0;
  G[154] = 0.0;
  G[176] = 0.0;
  G[198] = 0.0;
  G[220] = 0.0;
  G[242] = 0.0;
  G[264] = 0.0;
  G[286] = 0.0;
  G[308] = 0.0;
  G[330] = 0.0;
  G[352] = 0.0;
  G[374] = 0.0;
  G[396] = 0.0;
  G[418] = 0.0;
  G[440] = 0.0;
  G[462] = 0.0;
  G[1] = n_a_tmp;
  G[23] = (w[0] * g_a_tmp / 4.0 + w[2] * i_a_tmp / 4.0) + w[1] * j_a_tmp / 4.0;
  n_a_tmp =
      (l_a_tmp * x[3] / 4.0 - b_G_tmp * x[3] / 4.0) - c_G_tmp * x[2] / 4.0;
  G[45] = n_a_tmp;
  c_G_tmp =
      (e_G_tmp * x[2] / 4.0 - f_G_tmp * x[3] / 4.0) - l_a_tmp * x[2] / 4.0;
  G[67] = c_G_tmp;
  G[89] = 0.0;
  G[111] = 0.0;
  G[133] = 0.0;
  G[155] = 0.0;
  G[177] = 0.0;
  G[199] = 0.0;
  G[221] = 0.0;
  G[243] = 0.0;
  G[265] = 0.0;
  G[287] = 0.0;
  G[309] = 0.0;
  G[331] = 0.0;
  G[353] = 0.0;
  G[375] = 0.0;
  G[397] = 0.0;
  G[419] = 0.0;
  G[441] = 0.0;
  G[463] = 0.0;
  G[2] = d_G_tmp;
  G[24] = n_a_tmp;
  G[46] = (w[1] * g_a_tmp / 4.0 + w[2] * h_a_tmp / 4.0) + w[0] * j_a_tmp / 4.0;
  n_a_tmp =
      (b_G_tmp * x[1] / 4.0 - w[0] * x[2] * x[3] / 4.0) - e_G_tmp * x[1] / 4.0;
  G[68] = n_a_tmp;
  G[90] = 0.0;
  G[112] = 0.0;
  G[134] = 0.0;
  G[156] = 0.0;
  G[178] = 0.0;
  G[200] = 0.0;
  G[222] = 0.0;
  G[244] = 0.0;
  G[266] = 0.0;
  G[288] = 0.0;
  G[310] = 0.0;
  G[332] = 0.0;
  G[354] = 0.0;
  G[376] = 0.0;
  G[398] = 0.0;
  G[420] = 0.0;
  G[442] = 0.0;
  G[464] = 0.0;
  G[3] = G_tmp;
  G[25] = c_G_tmp;
  G[47] = n_a_tmp;
  G[69] = (w[2] * g_a_tmp / 4.0 + w[1] * h_a_tmp / 4.0) + w[0] * i_a_tmp / 4.0;
  G[91] = 0.0;
  G[113] = 0.0;
  G[135] = 0.0;
  G[157] = 0.0;
  G[179] = 0.0;
  G[201] = 0.0;
  G[223] = 0.0;
  G[245] = 0.0;
  G[267] = 0.0;
  G[289] = 0.0;
  G[311] = 0.0;
  G[333] = 0.0;
  G[355] = 0.0;
  G[377] = 0.0;
  G[399] = 0.0;
  G[421] = 0.0;
  G[443] = 0.0;
  G[465] = 0.0;
  G[7] = 0.0;
  G[29] = 0.0;
  G[51] = 0.0;
  G[73] = 0.0;
  G[95] = 0.0;
  G[117] = 0.0;
  G[139] = 0.0;
  G[161] = (w[4] * (c_a_tmp * c_a_tmp) + w[5] * (f_a_tmp * f_a_tmp)) +
           w[3] * (k_a_tmp * k_a_tmp);
  l_a_tmp = w[5] * m_a_tmp;
  n_a_tmp =
      (w[3] * a_tmp * k_a_tmp - w[4] * c_a_tmp * o_a_tmp) - l_a_tmp * f_a_tmp;
  G[183] = n_a_tmp;
  G_tmp = w[4] * b_a_tmp;
  b_G_tmp = w[3] * d_a_tmp;
  c_G_tmp = (w[5] * f_a_tmp * e_a_tmp - b_G_tmp * k_a_tmp) - G_tmp * c_a_tmp;
  G[205] = c_G_tmp;
  G[227] = 0.0;
  G[249] = 0.0;
  G[271] = 0.0;
  G[293] = 0.0;
  G[315] = 0.0;
  G[337] = 0.0;
  G[359] = 0.0;
  G[381] = 0.0;
  G[403] = 0.0;
  G[425] = 0.0;
  G[447] = 0.0;
  G[469] = 0.0;
  G[8] = 0.0;
  G[30] = 0.0;
  G[52] = 0.0;
  G[74] = 0.0;
  G[96] = 0.0;
  G[118] = 0.0;
  G[140] = 0.0;
  G[162] = n_a_tmp;
  G[184] = (w[3] * (a_tmp * a_tmp) + w[5] * (m_a_tmp * m_a_tmp)) +
           w[4] * (o_a_tmp * o_a_tmp);
  n_a_tmp = (G_tmp * o_a_tmp - l_a_tmp * e_a_tmp) - b_G_tmp * a_tmp;
  G[206] = n_a_tmp;
  G[228] = 0.0;
  G[250] = 0.0;
  G[272] = 0.0;
  G[294] = 0.0;
  G[316] = 0.0;
  G[338] = 0.0;
  G[360] = 0.0;
  G[382] = 0.0;
  G[404] = 0.0;
  G[426] = 0.0;
  G[448] = 0.0;
  G[470] = 0.0;
  G[9] = 0.0;
  G[31] = 0.0;
  G[53] = 0.0;
  G[75] = 0.0;
  G[97] = 0.0;
  G[119] = 0.0;
  G[141] = 0.0;
  G[163] = c_G_tmp;
  G[185] = n_a_tmp;
  G[207] = (w[3] * (d_a_tmp * d_a_tmp) + w[4] * (b_a_tmp * b_a_tmp)) +
           w[5] * (e_a_tmp * e_a_tmp);
  G[229] = 0.0;
  G[251] = 0.0;
  G[273] = 0.0;
  G[295] = 0.0;
  G[317] = 0.0;
  G[339] = 0.0;
  G[361] = 0.0;
  G[383] = 0.0;
  G[405] = 0.0;
  G[427] = 0.0;
  G[449] = 0.0;
  G[471] = 0.0;
  for (int i{0}; i < 22; i++) {
    G[22 * i + 4] = 0.0;
    G[22 * i + 5] = 0.0;
    G[22 * i + 6] = 0.0;
    G[22 * i + 10] = 0.0;
    G[22 * i + 11] = 0.0;
    G[22 * i + 12] = 0.0;
    G[22 * i + 13] = 0.0;
    G[22 * i + 14] = 0.0;
    G[22 * i + 15] = 0.0;
    G[22 * i + 16] = 0.0;
    G[22 * i + 17] = 0.0;
    G[22 * i + 18] = 0.0;
    G[22 * i + 19] = 0.0;
    G[22 * i + 20] = 0.0;
    G[22 * i + 21] = 0.0;
  }
}

void EKF_rocket::predict_state(const double ang_delta[3],
                               const double vel_delta[3], double dt,
                               double b_x[22]) const
{
  double dv[4];
  double dv1[4];
  double obj[4];
  double b_x_tmp;
  double c_x_tmp;
  double d_x_tmp;
  double e_x_tmp;
  double f_x_tmp;
  double g_x_tmp;
  double h_x_tmp;
  double i_x_tmp;
  double j_x_tmp;
  double k_x_tmp;
  double l_x_tmp;
  double x_tmp;
  b_x[4] = x[4] + dt * x[7];
  b_x[5] = x[5] + dt * x[8];
  b_x[6] = x[6] + dt * x[9];
  x_tmp = x[0] * x[0];
  b_x_tmp = x[1] * x[1];
  c_x_tmp = x[2] * x[2];
  d_x_tmp = x[3] * x[3];
  e_x_tmp = vel_delta[1] - x[14];
  f_x_tmp = 2.0 * x[0] * x[3];
  g_x_tmp = 2.0 * x[1] * x[2];
  h_x_tmp = vel_delta[0] - x[13];
  i_x_tmp = vel_delta[2] - x[15];
  j_x_tmp = 2.0 * x[0] * x[2];
  k_x_tmp = 2.0 * x[1] * x[3];
  b_x[7] = (((x[7] + dt * 0.0) +
             h_x_tmp * (((x_tmp + b_x_tmp) - c_x_tmp) - d_x_tmp)) -
            e_x_tmp * (f_x_tmp - g_x_tmp)) +
           i_x_tmp * (j_x_tmp + k_x_tmp);
  x_tmp -= b_x_tmp;
  b_x_tmp = 2.0 * x[0] * x[1];
  l_x_tmp = 2.0 * x[2] * x[3];
  b_x[8] = (((x[8] + dt * 0.0) + e_x_tmp * ((x_tmp + c_x_tmp) - d_x_tmp)) +
            h_x_tmp * (f_x_tmp + g_x_tmp)) -
           i_x_tmp * (b_x_tmp - l_x_tmp);
  b_x[9] = (((x[9] + dt * 9.81) + i_x_tmp * ((x_tmp - c_x_tmp) + d_x_tmp)) -
            h_x_tmp * (j_x_tmp - k_x_tmp)) +
           e_x_tmp * (b_x_tmp + l_x_tmp);
  std::copy(&x[10], &x[22], &b_x[10]);
  //  preallocate
  //              x(1:4) = compact(normalize(qinit * quaternion(ang_delta -
  //              [dax_b, day_b, daz_b], 'rotvec')));
  obj[0] = x[0];
  obj[1] = x[1];
  obj[2] = x[2];
  obj[3] = x[3];
  dv[0] = 1.0;
  dv[1] = (ang_delta[0] - x[10]) / 2.0;
  dv[2] = (ang_delta[1] - x[11]) / 2.0;
  dv[3] = (ang_delta[2] - x[12]) / 2.0;
  EKF_rocket::mult_quat(obj, dv, dv1);
  b_x[0] = dv1[0];
  b_x[1] = dv1[1];
  b_x[2] = dv1[2];
  b_x[3] = dv1[3];
}

void EKF_rocket::quaternion_normalisation(double b_x[22])
{
  double absxk;
  double scale;
  double t;
  double y;
  scale = 3.3121686421112381E-170;
  absxk = std::abs(b_x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = std::abs(b_x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = std::abs(b_x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = std::abs(b_x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * std::sqrt(y);
  b_x[0] /= y;
  b_x[1] /= y;
  b_x[2] /= y;
  b_x[3] /= y;
}

void EKF_rocket::set_additive_noise(double Ts, double Qs[484], double w[6])
{
  static const double dv[6]{1.0E-5, 1.0E-5, 1.0E-5, 2.0, 2.0, 2.0};
  double v[22];
  double a;
  a = 1.0 / Ts;
  scale_var = 0.5 * (1.0 / (a * a));
  ang_delta_bias_sigma = scale_var * 1.0E-16;
  vel_delta_bias_sigma = scale_var * 0.0002;
  for (int j{0}; j < 6; j++) {
    w[j] = scale_var * dv[j];
  }
  for (int j{0}; j < 10; j++) {
    v[j] = 1.0E-8;
  }
  v[10] = ang_delta_bias_sigma;
  v[13] = vel_delta_bias_sigma;
  v[16] = 1.0E-15;
  v[19] = 1.0E-10;
  v[11] = ang_delta_bias_sigma;
  v[14] = vel_delta_bias_sigma;
  v[17] = 1.0E-15;
  v[20] = 1.0E-10;
  v[12] = ang_delta_bias_sigma;
  v[15] = vel_delta_bias_sigma;
  v[18] = 1.0E-15;
  v[21] = 1.0E-10;
  std::memset(&Qs[0], 0, 484U * sizeof(double));
  for (int j{0}; j < 22; j++) {
    Qs[j + 22 * j] = v[j];
  }
}

void EKF_rocket::update_step(const double z[2], const double h_x[2],
                             const double H[44], double x_new[22],
                             double P_new[484]) const
{
  static const double dv[4]{0.01, 0.0, 0.0, 0.01};
  double c_I[484];
  double K[44];
  double b_H[44];
  double S[4];
  double b_idx_0;
  double b_idx_1;
  double b_idx_2;
  double d;
  double r;
  double t;
  double z_idx_0;
  double z_idx_1;
  int K_tmp;
  int i2;
  signed char b_I[484];
  for (int i{0}; i < 2; i++) {
    for (int i1{0}; i1 < 22; i1++) {
      K_tmp = i + (i1 << 1);
      K[i1 + 22 * i] = H[K_tmp];
      r = 0.0;
      for (i2 = 0; i2 < 22; i2++) {
        r += H[i + (i2 << 1)] * P[i2 + 22 * i1];
      }
      b_H[K_tmp] = r;
    }
  }
  for (int i{0}; i < 2; i++) {
    for (int i1{0}; i1 < 2; i1++) {
      r = 0.0;
      for (i2 = 0; i2 < 22; i2++) {
        r += b_H[i + (i2 << 1)] * K[i2 + 22 * i1];
      }
      K_tmp = i + (i1 << 1);
      S[K_tmp] = r + dv[K_tmp];
    }
  }
  if (std::abs(S[1]) > std::abs(S[0])) {
    r = S[0] / S[1];
    t = 1.0 / (r * S[3] - S[2]);
    b_idx_0 = S[3] / S[1] * t;
    b_idx_1 = -t;
    b_idx_2 = -S[2] / S[1] * t;
    t *= r;
  } else {
    r = S[1] / S[0];
    t = 1.0 / (S[3] - r * S[2]);
    b_idx_0 = S[3] / S[0] * t;
    b_idx_1 = -r * t;
    b_idx_2 = -S[2] / S[0] * t;
  }
  for (int i{0}; i < 22; i++) {
    for (int i1{0}; i1 < 2; i1++) {
      r = 0.0;
      for (i2 = 0; i2 < 22; i2++) {
        r += P[i + 22 * i2] * K[i2 + 22 * i1];
      }
      b_H[i + 22 * i1] = r;
    }
  }
  z_idx_0 = z[0] - h_x[0];
  z_idx_1 = z[1] - h_x[1];
  for (int i{0}; i < 22; i++) {
    double d1;
    r = b_H[i + 22];
    d = b_H[i];
    d1 = d * b_idx_0 + r * b_idx_1;
    K[i] = d1;
    r = d * b_idx_2 + r * t;
    K[i + 22] = r;
    x_new[i] = x[i] + (d1 * z_idx_0 + r * z_idx_1);
  }
  std::memset(&b_I[0], 0, 484U * sizeof(signed char));
  for (K_tmp = 0; K_tmp < 22; K_tmp++) {
    b_I[K_tmp + 22 * K_tmp] = 1;
  }
  for (int i{0}; i < 22; i++) {
    r = K[i];
    d = K[i + 22];
    for (int i1{0}; i1 < 22; i1++) {
      i2 = i1 << 1;
      K_tmp = i + 22 * i1;
      c_I[K_tmp] =
          static_cast<double>(b_I[K_tmp]) - (r * H[i2] + d * H[i2 + 1]);
    }
    for (int i1{0}; i1 < 22; i1++) {
      r = 0.0;
      for (i2 = 0; i2 < 22; i2++) {
        r += c_I[i + 22 * i2] * P[i2 + 22 * i1];
      }
      P_new[i + 22 * i1] = r;
    }
  }
}

void EKF_rocket::update_step(const double z[3], const double h_x[3],
                             const double H[66], const double R[9],
                             double x_new[22], double P_new[484]) const
{
  double c_I[484];
  double K[66];
  double b_H[66];
  double S[9];
  double b_x[9];
  double absx11;
  double absx21;
  double absx31;
  int itmp;
  int p1;
  int p2;
  int p3;
  signed char b_I[484];
  for (p3 = 0; p3 < 3; p3++) {
    for (itmp = 0; itmp < 22; itmp++) {
      p1 = p3 + 3 * itmp;
      K[itmp + 22 * p3] = H[p1];
      absx11 = 0.0;
      for (p2 = 0; p2 < 22; p2++) {
        absx11 += H[p3 + 3 * p2] * P[p2 + 22 * itmp];
      }
      b_H[p1] = absx11;
    }
  }
  for (p3 = 0; p3 < 3; p3++) {
    for (itmp = 0; itmp < 3; itmp++) {
      absx11 = 0.0;
      for (p2 = 0; p2 < 22; p2++) {
        absx11 += b_H[p3 + 3 * p2] * K[p2 + 22 * itmp];
      }
      p1 = p3 + 3 * itmp;
      S[p1] = absx11 + R[p1];
    }
  }
  std::copy(&S[0], &S[9], &b_x[0]);
  p1 = 0;
  p2 = 3;
  p3 = 6;
  absx11 = std::abs(S[0]);
  absx21 = std::abs(S[1]);
  absx31 = std::abs(S[2]);
  if ((absx21 > absx11) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = S[1];
    b_x[1] = S[0];
    b_x[3] = S[4];
    b_x[4] = S[3];
    b_x[6] = S[7];
    b_x[7] = S[6];
  } else if (absx31 > absx11) {
    p1 = 6;
    p3 = 0;
    b_x[0] = S[2];
    b_x[2] = S[0];
    b_x[3] = S[5];
    b_x[5] = S[3];
    b_x[6] = S[8];
    b_x[8] = S[6];
  }
  b_x[1] /= b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= b_x[1] * b_x[3];
  b_x[5] -= b_x[2] * b_x[3];
  b_x[7] -= b_x[1] * b_x[6];
  b_x[8] -= b_x[2] * b_x[6];
  if (std::abs(b_x[5]) > std::abs(b_x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    absx11 = b_x[1];
    b_x[1] = b_x[2];
    b_x[2] = absx11;
    absx11 = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = absx11;
    absx11 = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = absx11;
  }
  b_x[5] /= b_x[4];
  b_x[8] -= b_x[5] * b_x[7];
  absx11 = (b_x[1] * b_x[5] - b_x[2]) / b_x[8];
  absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
  S[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
  S[p1 + 1] = absx21;
  S[p1 + 2] = absx11;
  absx11 = -b_x[5] / b_x[8];
  absx21 = (1.0 - b_x[7] * absx11) / b_x[4];
  S[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  S[p2 + 1] = absx21;
  S[p2 + 2] = absx11;
  absx11 = 1.0 / b_x[8];
  absx21 = -b_x[7] * absx11 / b_x[4];
  S[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
  S[p3 + 1] = absx21;
  S[p3 + 2] = absx11;
  for (p3 = 0; p3 < 22; p3++) {
    for (itmp = 0; itmp < 3; itmp++) {
      absx11 = 0.0;
      for (p2 = 0; p2 < 22; p2++) {
        absx11 += P[p3 + 22 * p2] * K[p2 + 22 * itmp];
      }
      b_H[p3 + 22 * itmp] = absx11;
    }
  }
  for (p3 = 0; p3 < 22; p3++) {
    absx11 = b_H[p3];
    absx21 = b_H[p3 + 22];
    absx31 = b_H[p3 + 44];
    for (itmp = 0; itmp < 3; itmp++) {
      K[p3 + 22 * itmp] = (absx11 * S[3 * itmp] + absx21 * S[3 * itmp + 1]) +
                          absx31 * S[3 * itmp + 2];
    }
  }
  absx11 = z[0] - h_x[0];
  absx21 = z[1] - h_x[1];
  absx31 = z[2] - h_x[2];
  for (p3 = 0; p3 < 22; p3++) {
    x_new[p3] =
        x[p3] + ((K[p3] * absx11 + K[p3 + 22] * absx21) + K[p3 + 44] * absx31);
  }
  std::memset(&b_I[0], 0, 484U * sizeof(signed char));
  for (p1 = 0; p1 < 22; p1++) {
    b_I[p1 + 22 * p1] = 1;
  }
  for (p3 = 0; p3 < 22; p3++) {
    absx11 = K[p3];
    absx21 = K[p3 + 22];
    absx31 = K[p3 + 44];
    for (itmp = 0; itmp < 22; itmp++) {
      p1 = p3 + 22 * itmp;
      c_I[p1] = static_cast<double>(b_I[p1]) -
                ((absx11 * H[3 * itmp] + absx21 * H[3 * itmp + 1]) +
                 absx31 * H[3 * itmp + 2]);
    }
    for (itmp = 0; itmp < 22; itmp++) {
      absx11 = 0.0;
      for (p2 = 0; p2 < 22; p2++) {
        absx11 += c_I[p3 + 22 * p2] * P[p2 + 22 * itmp];
      }
      P_new[p3 + 22 * itmp] = absx11;
    }
  }
}

void EKF_rocket::eulZYX2quat(const double euler[3], double q[4])
{
  double b_q_tmp;
  double c_idx_0;
  double c_idx_1;
  double c_idx_2;
  double d;
  double q_tmp;
  double s_idx_0;
  double s_idx_1;
  d = euler[0] / 2.0;
  c_idx_0 = std::cos(d);
  d = std::sin(d);
  s_idx_0 = d;
  d = euler[1] / 2.0;
  c_idx_1 = std::cos(d);
  d = std::sin(d);
  s_idx_1 = d;
  d = euler[2] / 2.0;
  c_idx_2 = std::cos(d);
  d = std::sin(d);
  q_tmp = c_idx_0 * c_idx_1;
  b_q_tmp = s_idx_0 * s_idx_1;
  q[0] = q_tmp * c_idx_2 + b_q_tmp * d;
  q[1] = q_tmp * d - b_q_tmp * c_idx_2;
  q_tmp = s_idx_0 * c_idx_1;
  b_q_tmp = c_idx_0 * s_idx_1;
  q[2] = b_q_tmp * c_idx_2 + q_tmp * d;
  q[3] = q_tmp * c_idx_2 - b_q_tmp * d;
}

void EKF_rocket::eulerZYX_from_acc_mag(const double acc_b[3],
                                       const double mag_b[3], double euler[3])
{
  double b_euler_tmp;
  double c_euler_tmp;
  double euler_tmp;
  euler[2] = rt_atan2d_snf(acc_b[1], -acc_b[2]);
  euler[1] = rt_atan2d_snf(-acc_b[0], acc_b[1] * std::sin(euler[2]) -
                                          acc_b[2] * std::cos(euler[2]));
  euler_tmp = std::cos(euler[2]);
  b_euler_tmp = std::sin(euler[2]);
  c_euler_tmp = std::sin(euler[1]);
  euler[0] = rt_atan2d_snf(
      mag_b[2] * b_euler_tmp - mag_b[1] * euler_tmp,
      (mag_b[0] * euler_tmp + mag_b[1] * b_euler_tmp * c_euler_tmp) +
          mag_b[2] * c_euler_tmp * euler_tmp);
}

void EKF_rocket::get_eulerZYX(double euler[3]) const
{
  double aSinInput;
  double b;
  double b_euler_tmp;
  double c_euler_tmp;
  double d_euler_tmp;
  double euler_tmp;
  double y_idx_0;
  double y_idx_1;
  double y_idx_2;
  int b_trueCount;
  int trueCount;
  signed char tmp_data;
  boolean_T mask1;
  boolean_T mask2;
  b = 1.0 /
      std::sqrt(((x[0] * x[0] + x[1] * x[1]) + x[2] * x[2]) + x[3] * x[3]);
  y_idx_0 = x[0] * b;
  y_idx_1 = x[1] * b;
  y_idx_2 = x[2] * b;
  b *= x[3];
  aSinInput = -2.0 * (y_idx_1 * b - y_idx_0 * y_idx_2);
  mask1 = (aSinInput >= 0.99999999999999778);
  mask2 = (aSinInput <= -0.99999999999999778);
  if (aSinInput >= 0.99999999999999778) {
    aSinInput = 1.0;
  }
  euler_tmp = y_idx_0 * y_idx_0;
  b_euler_tmp = y_idx_1 * y_idx_1;
  c_euler_tmp = y_idx_2 * y_idx_2;
  d_euler_tmp = b * b;
  euler[0] =
      rt_atan2d_snf(2.0 * (y_idx_1 * y_idx_2 + y_idx_0 * b),
                    ((euler_tmp + b_euler_tmp) - c_euler_tmp) - d_euler_tmp);
  euler[2] =
      rt_atan2d_snf(2.0 * (y_idx_2 * b + y_idx_0 * y_idx_1),
                    ((euler_tmp - b_euler_tmp) - c_euler_tmp) + d_euler_tmp);
  trueCount = 0;
  if (mask2) {
    aSinInput = -1.0;
  }
  if (mask1 || mask2) {
    trueCount = 1;
  }
  euler[1] = std::asin(aSinInput);
  for (int k{0}; k < trueCount; k++) {
    if (!std::isnan(aSinInput)) {
      if (aSinInput < 0.0) {
        aSinInput = -1.0;
      } else {
        aSinInput = (aSinInput > 0.0);
      }
    }
  }
  b_trueCount = 0;
  if (mask1 || mask2) {
    b_trueCount = 1;
  }
  for (int k{0}; k < b_trueCount; k++) {
    b = rt_atan2d_snf(y_idx_1, y_idx_0);
  }
  if (mask1 || mask2) {
    tmp_data = 1;
  }
  if (trueCount == b_trueCount) {
    if (trueCount - 1 >= 0) {
      euler[0] = -aSinInput * 2.0 * b;
    }
  } else {
    binary_expand_op(euler, (const signed char *)&tmp_data,
                     (const double *)&aSinInput, &trueCount, (const double *)&b,
                     &b_trueCount);
  }
  trueCount = 0;
  if (mask1 || mask2) {
    trueCount = 1;
  }
  if (trueCount - 1 >= 0) {
    euler[2] = 0.0;
  }
}

void EKF_rocket::get_gps_local_pos(const double lla[3], const double lla0[3],
                                   double gps_pos[3])
{
  coder::lla2ned(lla, lla0, gps_pos);
}

void EKF_rocket::get_posNED(double p[3]) const
{
  p[0] = x[4];
  p[1] = x[5];
  p[2] = x[6];
}

void EKF_rocket::get_state(double b_x[22]) const
{
  std::copy(&x[0], &x[22], &b_x[0]);
}

void EKF_rocket::get_velNED(double v[3]) const
{
  v[0] = x[7];
  v[1] = x[8];
  v[2] = x[9];
}

void EKF_rocket::gps_vel_2d_from_vel_course(double V_kmh, double course_deg,
                                            double gps_vel_2d[2])
{
  double V;
  double course_rad;
  V = V_kmh * 3.6;
  //  km/s to m/s
  course_rad = 57.295779513082323 * course_deg;
  //  deg to rad
  gps_vel_2d[0] = V * std::cos(course_rad);
  gps_vel_2d[1] = V * std::sin(course_rad);
  //  m/s
}

void EKF_rocket::init(const double x_init[22], double init_process_cov)
{
  AccelerometerNoise = 2.0;
  GyroscopeNoise = 1.0E-5;
  AccelerometerBiasNoise = 0.0002;
  GyroscopeBiasNoise = 1.0E-16;
  MagnetometerBiasNoise = 1.0E-10;
  GeomagneticVectorNoise = 1.0E-15;
  additiveNoise = 1.0E-8;
  std::copy(&x_init[0], &x_init[22], &x[0]);
  for (int i{0}; i < 484; i++) {
    P[i] = init_process_cov;
  }
}

void EKF_rocket::predict_step(const double accB[3], const double omegaB[3],
                              double Ts)
{
  EKF_rocket a__1;
  double F[484];
  double P_new[484];
  double Qs[484];
  double b_F[484];
  double dv[484];
  double w[6];
  double ang_delta[3];
  double vel_delta[3];
  int P_new_tmp;
  //  state prediction
  ang_delta[0] = omegaB[0] * Ts;
  vel_delta[0] = accB[0] * Ts;
  ang_delta[1] = omegaB[1] * Ts;
  vel_delta[1] = accB[1] * Ts;
  ang_delta[2] = omegaB[2] * Ts;
  vel_delta[2] = accB[2] * Ts;
  //  covariance prediction
  a__1 = *this;
  a__1.set_additive_noise(Ts, Qs, w);
  predict_jacobian(ang_delta, vel_delta, Ts, F);
  predict_process_noise(w, dv);
  for (int i{0}; i < 22; i++) {
    double d;
    for (int i1{0}; i1 < 22; i1++) {
      d = 0.0;
      for (P_new_tmp = 0; P_new_tmp < 22; P_new_tmp++) {
        d += F[i + 22 * P_new_tmp] * P[P_new_tmp + 22 * i1];
      }
      b_F[i + 22 * i1] = d;
    }
    for (int i1{0}; i1 < 22; i1++) {
      d = 0.0;
      for (P_new_tmp = 0; P_new_tmp < 22; P_new_tmp++) {
        d += b_F[i + 22 * P_new_tmp] * F[i1 + 22 * P_new_tmp];
      }
      P_new_tmp = i + 22 * i1;
      P_new[P_new_tmp] = (d + dv[P_new_tmp]) + Qs[P_new_tmp];
    }
  }
  a__1 = *this;
  a__1.predict_state(ang_delta, vel_delta, Ts, x);
  EKF_rocket::quaternion_normalisation(x);
  for (int i{0}; i < 22; i++) {
    for (int i1{0}; i1 < 22; i1++) {
      P_new_tmp = i1 + 22 * i;
      P[P_new_tmp] = 0.5 * (P_new[P_new_tmp] + P_new[i + 22 * i1]);
    }
  }
}

void EKF_rocket::set_mag_vec_from_body_avg(const double magB[3])
{
  double dv[9];
  double d;
  double d1;
  double d2;
  get_rotmat_body_2_inertial(dv);
  d = magB[0];
  d1 = magB[1];
  d2 = magB[2];
  for (int i{0}; i < 3; i++) {
    x[i + 19] = (dv[i] * d + dv[i + 3] * d1) + dv[i + 6] * d2;
  }
}

void EKF_rocket::set_process_cov(const double b_P[484])
{
  std::copy(&b_P[0], &b_P[484], &P[0]);
}

void EKF_rocket::set_state(const double b_x[22])
{
  std::copy(&b_x[0], &b_x[22], &x[0]);
}

void EKF_rocket::update_step_attitude_acc_mag(const double acc_b[3],
                                              const double mag_b[3])
{
  static const signed char b_iv[16]{1, 0, 0, 0, 0, 1, 0, 0,
                                    0, 0, 1, 0, 0, 0, 0, 1};
  EKF_rocket obj;
  double H_att[88];
  double b_H_att[4];
  int H_att_tmp;
  std::memset(&H_att[0], 0, 88U * sizeof(double));
  for (int i{0}; i < 4; i++) {
    H_att_tmp = i << 2;
    H_att[H_att_tmp] = b_iv[H_att_tmp];
    H_att[H_att_tmp + 1] = b_iv[H_att_tmp + 1];
    H_att[H_att_tmp + 2] = b_iv[H_att_tmp + 2];
    H_att[H_att_tmp + 3] = b_iv[H_att_tmp + 3];
  }
  for (int i{0}; i < 4; i++) {
    double d;
    d = 0.0;
    for (H_att_tmp = 0; H_att_tmp < 22; H_att_tmp++) {
      d += H_att[i + (H_att_tmp << 2)] * x[H_att_tmp];
    }
    b_H_att[i] = d;
  }
  double dv1[4];
  double dv[3];
  EKF_rocket::eulerZYX_from_acc_mag(acc_b, mag_b, dv);
  EKF_rocket::eulZYX2quat(dv, dv1);
  obj = *this;
  obj.b_update_step(dv1, b_H_att, H_att, x, P);
}

void EKF_rocket::update_step_baro(double z)
{
  static const signed char c_a[22]{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  double c_I[484];
  double d_I[484];
  double K[22];
  double a;
  double a_tmp;
  double b_a;
  double d;
  int k;
  signed char b_I[484];
  a = 0.0;
  b_a = 0.0;
  for (int i{0}; i < 22; i++) {
    a_tmp = c_a[i];
    a += a_tmp * x[i];
    d = 0.0;
    for (int i1{0}; i1 < 22; i1++) {
      d += static_cast<double>(c_a[i1]) * P[i1 + 22 * i];
    }
    b_a += d * a_tmp;
  }
  a_tmp = z - a;
  a = 1.0 / (b_a + 0.1);
  for (int i{0}; i < 22; i++) {
    d = 0.0;
    for (int i1{0}; i1 < 22; i1++) {
      d += P[i + 22 * i1] * static_cast<double>(c_a[i1]);
    }
    d *= a;
    K[i] = d;
    x[i] += d * a_tmp;
  }
  std::memset(&b_I[0], 0, 484U * sizeof(signed char));
  for (k = 0; k < 22; k++) {
    b_I[k + 22 * k] = 1;
  }
  for (int i{0}; i < 22; i++) {
    for (int i1{0}; i1 < 22; i1++) {
      k = i1 + 22 * i;
      c_I[k] =
          static_cast<double>(b_I[k]) - K[i1] * static_cast<double>(c_a[i]);
    }
  }
  for (int i{0}; i < 22; i++) {
    for (int i1{0}; i1 < 22; i1++) {
      d = 0.0;
      for (k = 0; k < 22; k++) {
        d += c_I[i + 22 * k] * P[k + 22 * i1];
      }
      d_I[i + 22 * i1] = d;
    }
  }
  std::copy(&d_I[0], &d_I[484], &P[0]);
}

void EKF_rocket::update_step_gps_pos(const double lla[3], const double lla0[3])
{
  static const double dv1[9]{2.56, 0.0, 0.0, 0.0, 2.56, 0.0, 0.0, 0.0, 2.56};
  EKF_rocket obj;
  double H_gps[66];
  double b_H_gps[3];
  int H_gps_tmp;
  std::memset(&H_gps[0], 0, 66U * sizeof(double));
  for (int i{0}; i < 3; i++) {
    H_gps_tmp = 3 * (i + 4);
    H_gps[H_gps_tmp] = iv[3 * i];
    H_gps[H_gps_tmp + 1] = iv[3 * i + 1];
    H_gps[H_gps_tmp + 2] = iv[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    double d;
    d = 0.0;
    for (H_gps_tmp = 0; H_gps_tmp < 22; H_gps_tmp++) {
      d += H_gps[i + 3 * H_gps_tmp] * x[H_gps_tmp];
    }
    b_H_gps[i] = d;
  }
  double dv[3];
  coder::lla2ned(lla, lla0, dv);
  obj = *this;
  obj.update_step(dv, b_H_gps, H_gps, dv1, x, P);
}

void EKF_rocket::update_step_gps_vel(const double z[3])
{
  static const double dv[9]{0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
  EKF_rocket obj;
  double H_gps[66];
  double b_H_gps[3];
  int H_gps_tmp;
  std::memset(&H_gps[0], 0, 66U * sizeof(double));
  for (int i{0}; i < 3; i++) {
    H_gps_tmp = 3 * (i + 7);
    H_gps[H_gps_tmp] = iv[3 * i];
    H_gps[H_gps_tmp + 1] = iv[3 * i + 1];
    H_gps[H_gps_tmp + 2] = iv[3 * i + 2];
  }
  for (int i{0}; i < 3; i++) {
    double d;
    d = 0.0;
    for (H_gps_tmp = 0; H_gps_tmp < 22; H_gps_tmp++) {
      d += H_gps[i + 3 * H_gps_tmp] * x[H_gps_tmp];
    }
    b_H_gps[i] = d;
  }
  obj = *this;
  obj.update_step(z, b_H_gps, H_gps, dv, x, P);
}

void EKF_rocket::update_step_gps_vel_2d(const double z[2])
{
  EKF_rocket obj;
  double H_gps2d[44];
  double b_H_gps2d[2];
  std::memset(&H_gps2d[0], 0, 44U * sizeof(double));
  H_gps2d[14] = 1.0;
  H_gps2d[15] = 0.0;
  H_gps2d[16] = 0.0;
  H_gps2d[17] = 1.0;
  for (int i{0}; i < 2; i++) {
    double d;
    d = 0.0;
    for (int i1{0}; i1 < 22; i1++) {
      d += H_gps2d[i + (i1 << 1)] * x[i1];
    }
    b_H_gps2d[i] = d;
  }
  obj = *this;
  obj.update_step(z, b_H_gps2d, H_gps2d, x, P);
}

void EKF_rocket::update_step_mag(const double z[3])
{
  static const double dv2[9]{0.09, 0.0, 0.0, 0.0, 0.09, 0.0, 0.0, 0.0, 0.09};
  EKF_rocket obj;
  double dv1[66];
  double dv[3];
  mag_measurement_model(dv);
  mag_measurement_jacobian(dv1);
  obj = *this;
  obj.update_step(z, dv, dv1, dv2, x, P);
}

// End of code generation (EKF_rocket.cpp)
