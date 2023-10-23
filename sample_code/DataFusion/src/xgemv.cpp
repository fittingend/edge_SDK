//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "xgemv.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : int n
//                const double x[18]
//                double beta1
//                double y[36]
//                int iy0
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void xgemv(int n, const double x[18], double beta1, double y[36], int iy0)
{
  int iy;
  int iyend;
  iyend = iy0 + 5;
  if (beta1 != 1.0) {
    if (beta1 == 0.0) {
      if (iy0 <= iyend) {
        std::memset(&y[iy0 + -1], 0,
                    static_cast<unsigned int>((iyend - iy0) + 1) *
                        sizeof(double));
      }
    } else {
      for (iy = iy0; iy <= iyend; iy++) {
        y[iy - 1] *= beta1;
      }
    }
  }
  iyend = 12;
  iy = 6 * (n - 1) + 1;
  for (int iac{1}; iac <= iy; iac += 6) {
    int i;
    i = iac + 5;
    for (int ia{iac}; ia <= i; ia++) {
      int i1;
      i1 = ((iy0 + ia) - iac) - 1;
      y[i1] += y[ia - 1] * x[iyend];
    }
    iyend++;
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xgemv.cpp
//
// [EOF]
//
