//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// lla2ned.cpp
//
// Code generation for function 'lla2ned'
//

// Include files
#include "lla2ned.h"
#include "foo_rtwutil.h"
#include "rt_nonfinite.h"
#include <cfloat>
#include <cmath>

// Function Declarations
static double rt_remd_snf(double u0, double u1);

// Function Definitions
static double rt_remd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1) || std::isinf(u0)) {
    y = rtNaN;
  } else if (std::isinf(u1)) {
    y = u0;
  } else if ((u1 != 0.0) && (u1 != std::trunc(u1))) {
    double q;
    q = std::abs(u0 / u1);
    if (!(std::abs(q - std::floor(q + 0.5)) > DBL_EPSILON * q)) {
      y = 0.0 * u0;
    } else {
      y = std::fmod(u0, u1);
    }
  } else {
    y = std::fmod(u0, u1);
  }
  return y;
}

namespace coder {
void lla2ned(const double lla[3], const double lla0[3], double xyzNED[3])
{
  double absx;
  double dLat;
  double dLon;
  double flat;
  double x;
  double x_tmp;
  int k;
  boolean_T b[3];
  boolean_T exitg1;
  boolean_T latp2;
  dLat = lla[0] - lla0[0];
  dLon = lla[1] - lla0[1];
  flat = std::abs(dLat);
  if (flat > 180.0) {
    if (std::isnan(dLat + 180.0) || std::isinf(dLat + 180.0)) {
      flat = rtNaN;
    } else if (dLat + 180.0 == 0.0) {
      flat = 0.0;
    } else {
      flat = std::fmod(dLat + 180.0, 360.0);
      if (flat == 0.0) {
        flat = 0.0;
      } else if (dLat + 180.0 < 0.0) {
        flat += 360.0;
      }
    }
    dLat = dLat * 0.0 + (flat - 180.0);
    flat = std::abs(dLat);
  }
  if (flat > 90.0) {
    flat = std::abs(dLat);
    latp2 = (flat > 90.0);
    dLon += 180.0;
    x = dLat * static_cast<double>(latp2);
    if (!std::isnan(x)) {
      if (x < 0.0) {
        x = -1.0;
      } else {
        x = (x > 0.0);
      }
    }
    dLat = dLat * static_cast<double>(!latp2) +
           x * (90.0 - (flat * static_cast<double>(latp2) - 90.0)) *
               static_cast<double>(latp2);
  }
  if ((dLon > 180.0) || (dLon < -180.0)) {
    flat = rt_remd_snf(dLon, 360.0);
    dLon = dLon * 0.0 + (flat - 360.0 * std::trunc(flat / 180.0));
  }
  if (std::isinf(lla0[0]) || std::isnan(lla0[0])) {
    x_tmp = rtNaN;
    x = rtNaN;
  } else {
    signed char n;
    flat = rt_remd_snf(lla0[0], 360.0);
    x = flat;
    absx = std::abs(flat);
    if (absx > 180.0) {
      if (flat > 0.0) {
        x = flat - 360.0;
      } else {
        x = flat + 360.0;
      }
      absx = std::abs(x);
    }
    if (absx <= 45.0) {
      x *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (x > 0.0) {
        x = 0.017453292519943295 * (x - 90.0);
        n = 1;
      } else {
        x = 0.017453292519943295 * (x + 90.0);
        n = -1;
      }
    } else if (x > 0.0) {
      x = 0.017453292519943295 * (x - 180.0);
      n = 2;
    } else {
      x = 0.017453292519943295 * (x + 180.0);
      n = -2;
    }
    if (n == 0) {
      x_tmp = std::sin(x);
    } else if (n == 1) {
      x_tmp = std::cos(x);
    } else if (n == -1) {
      x_tmp = -std::cos(x);
    } else {
      x_tmp = -std::sin(x);
    }
    absx = std::abs(flat);
    if (absx > 180.0) {
      if (flat > 0.0) {
        flat -= 360.0;
      } else {
        flat += 360.0;
      }
      absx = std::abs(flat);
    }
    if (absx <= 45.0) {
      flat *= 0.017453292519943295;
      n = 0;
    } else if (absx <= 135.0) {
      if (flat > 0.0) {
        flat = 0.017453292519943295 * (flat - 90.0);
        n = 1;
      } else {
        flat = 0.017453292519943295 * (flat + 90.0);
        n = -1;
      }
    } else if (flat > 0.0) {
      flat = 0.017453292519943295 * (flat - 180.0);
      n = 2;
    } else {
      flat = 0.017453292519943295 * (flat + 180.0);
      n = -2;
    }
    if (n == 0) {
      x = std::cos(flat);
    } else if (n == 1) {
      x = -std::sin(flat);
    } else if (n == -1) {
      x = std::sin(flat);
    } else {
      x = -std::cos(flat);
    }
  }
  flat = 1.0 - 0.0066943799901413165 * x_tmp * x_tmp;
  absx = 6.378137E+6 / std::sqrt(flat);
  xyzNED[0] = dLat / (57.295779513082323 *
                      rt_atan2d_snf(1.0, absx * (0.99330562000985867 / flat)));
  xyzNED[1] = dLon / (57.295779513082323 * rt_atan2d_snf(1.0, absx * x));
  xyzNED[2] = -lla[2] + lla0[2];
  b[0] = std::isnan(xyzNED[0]);
  b[1] = std::isnan(xyzNED[1]);
  b[2] = std::isnan(xyzNED[2]);
  latp2 = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (b[k]) {
      latp2 = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  flat = 0.0 / static_cast<double>(!latp2);
  xyzNED[0] += flat;
  xyzNED[1] += flat;
  xyzNED[2] += flat;
}

} // namespace coder

// End of code generation (lla2ned.cpp)
