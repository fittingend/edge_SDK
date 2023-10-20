//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mldivide.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "mldivide.h"
#include "EDGE_fusion_function_231019_2222_data.h"
#include "rt_nonfinite.h"
#include "xzgetrf.h"

// Function Definitions
//
// Arguments    : const double A[36]
//                double Y[36]
// Return Type  : void
//
namespace coder {
void mldivide(const double A[36], double Y[36])
{
  double b_A[36];
  int ipiv[6];
  int Y_tmp;
  int i;
  int jBcol;
  int kAcol;
  int temp;
  for (i = 0; i < 36; i++) {
    Y[i] = iv[i];
    b_A[i] = A[i];
  }
  internal::reflapack::xzgetrf(b_A, ipiv);
  for (int b_i{0}; b_i < 5; b_i++) {
    i = ipiv[b_i];
    if (i != b_i + 1) {
      for (int j{0}; j < 6; j++) {
        jBcol = b_i + 6 * j;
        temp = static_cast<int>(Y[jBcol]);
        Y_tmp = (i + 6 * j) - 1;
        Y[jBcol] = Y[Y_tmp];
        Y[Y_tmp] = temp;
      }
    }
  }
  for (int j{0}; j < 6; j++) {
    jBcol = 6 * j;
    for (temp = 0; temp < 6; temp++) {
      kAcol = 6 * temp;
      i = temp + jBcol;
      if (Y[i] != 0.0) {
        int i1;
        i1 = temp + 2;
        for (int b_i{i1}; b_i < 7; b_i++) {
          Y_tmp = (b_i + jBcol) - 1;
          Y[Y_tmp] -= Y[i] * b_A[(b_i + kAcol) - 1];
        }
      }
    }
  }
  for (int j{0}; j < 6; j++) {
    jBcol = 6 * j;
    for (temp = 5; temp >= 0; temp--) {
      double d;
      kAcol = 6 * temp;
      i = temp + jBcol;
      d = Y[i];
      if (d != 0.0) {
        Y[i] = d / b_A[temp + kAcol];
        for (int b_i{0}; b_i < temp; b_i++) {
          Y_tmp = b_i + jBcol;
          Y[Y_tmp] -= Y[i] * b_A[b_i + kAcol];
        }
      }
    }
  }
}

} // namespace coder

//
// File trailer for mldivide.cpp
//
// [EOF]
//
