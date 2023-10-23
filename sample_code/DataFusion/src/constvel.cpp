//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: constvel.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "constvel.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : coder::array<double, 2U> &in1
//                int in2
//                const coder::array<double, 2U> &in3
// Return Type  : void
//
void binary_expand_op(coder::array<double, 2U> &in1, int in2,
                      const coder::array<double, 2U> &in3)
{
  coder::array<double, 2U> b_in1;
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  i = in2 << 1;
  if (in3.size(1) == 1) {
    loop_ub = in1.size(1);
  } else {
    loop_ub = in3.size(1);
  }
  b_in1.set_size(1, loop_ub);
  stride_0_1 = (in1.size(1) != 1);
  stride_1_1 = (in3.size(1) != 1);
  for (int i1{0}; i1 < loop_ub; i1++) {
    b_in1[i1] = in1[(i + 6 * (i1 * stride_0_1)) - 1] + in3[i1 * stride_1_1];
  }
  loop_ub = b_in1.size(1);
  for (int i1{0}; i1 < loop_ub; i1++) {
    in1[(i + 6 * i1) - 1] = b_in1[i1];
  }
}

//
// Arguments    : coder::array<double, 2U> &in1
//                int in2
//                int in3
//                const coder::array<double, 2U> &in4
//                const coder::array<double, 2U> &in5
// Return Type  : void
//
void binary_expand_op_1(coder::array<double, 2U> &in1, int in2, int in3,
                        const coder::array<double, 2U> &in4,
                        const coder::array<double, 2U> &in5)
{
  coder::array<double, 2U> b_in1;
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  int stride_2_1;
  i = in2 << 1;
  if (in5.size(1) == 1) {
    if (in4.size(1) == 1) {
      loop_ub = in1.size(1);
    } else {
      loop_ub = in4.size(1);
    }
  } else {
    loop_ub = in5.size(1);
  }
  b_in1.set_size(1, loop_ub);
  stride_0_1 = (in1.size(1) != 1);
  stride_1_1 = (in4.size(1) != 1);
  stride_2_1 = (in5.size(1) != 1);
  for (int i1{0}; i1 < loop_ub; i1++) {
    b_in1[i1] = (in1[in3 + 6 * (i1 * stride_0_1)] + in4[i1 * stride_1_1]) +
                in5[i1 * stride_2_1];
  }
  loop_ub = b_in1.size(1);
  for (int i1{0}; i1 < loop_ub; i1++) {
    in1[(i + 6 * i1) - 2] = b_in1[i1];
  }
}

//
// File trailer for constvel.cpp
//
// [EOF]
//
