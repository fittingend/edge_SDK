//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mrdivide_helper.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include "xzgetrf.h"
#include <algorithm>

// Function Definitions
//
// Arguments    : double A[6]
//                const double B[36]
// Return Type  : void
//
namespace coder {
namespace internal {
void mrdiv(double A[6], const double B[36])
{
  double b_A[36];
  double temp;
  int ipiv[6];
  int i;
  int jAcol;
  std::copy(&B[0], &B[36], &b_A[0]);
  reflapack::xzgetrf(b_A, ipiv);
  for (int j{0}; j < 6; j++) {
    jAcol = 6 * j;
    for (int k{0}; k < j; k++) {
      temp = b_A[k + jAcol];
      if (temp != 0.0) {
        A[j] -= temp * A[k];
      }
    }
    A[j] *= 1.0 / b_A[j + jAcol];
  }
  for (int j{5}; j >= 0; j--) {
    jAcol = 6 * j - 1;
    i = j + 2;
    for (int k{i}; k < 7; k++) {
      temp = b_A[k + jAcol];
      if (temp != 0.0) {
        A[j] -= temp * A[k - 1];
      }
    }
  }
  for (int j{4}; j >= 0; j--) {
    i = ipiv[j];
    if (i != j + 1) {
      temp = A[j];
      A[j] = A[i - 1];
      A[i - 1] = temp;
    }
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for mrdivide_helper.cpp
//
// [EOF]
//
