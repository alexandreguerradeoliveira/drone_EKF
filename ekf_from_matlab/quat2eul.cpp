//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// quat2eul.cpp
//
// Code generation for function 'quat2eul'
//

// Include files
#include "quat2eul.h"
#include "rt_nonfinite.h"

// Function Definitions
void binary_expand_op(double in1[3], const signed char in2_data[],
                      const double in3_data[], const int *in3_size,
                      const double in4_data[], const int *in4_size)
{
  int loop_ub;
  if (*in4_size == 1) {
    loop_ub = *in3_size;
  } else {
    loop_ub = *in4_size;
  }
  for (int i{0}; i < loop_ub; i++) {
    in1[in2_data[0] - 1] = -in3_data[0] * 2.0 * in4_data[0];
  }
}

// End of code generation (quat2eul.cpp)
