//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzunghr.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "xzunghr.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include <cstring>

// Function Definitions
//
// Arguments    : int ilo
//                int ihi
//                double A[36]
//                const double tau[5]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
void xzunghr(int ilo, int ihi, double A[36], const double tau[5])
{
  int a;
  int i;
  int ia;
  int ia0;
  int itau;
  int nh;
  nh = ihi - ilo;
  a = ilo + 1;
  for (int j{ihi}; j >= a; j--) {
    ia = (j - 1) * 6;
    i = static_cast<unsigned char>(j - 1);
    std::memset(&A[ia], 0,
                static_cast<unsigned int>((i + ia) - ia) * sizeof(double));
    i = j + 1;
    for (int b_i{i}; b_i <= ihi; b_i++) {
      itau = ia + b_i;
      A[itau - 1] = A[itau - 7];
    }
    i = ihi + 1;
    if (i <= 6) {
      std::memset(&A[(i + ia) + -1], 0,
                  static_cast<unsigned int>(((ia - i) - ia) + 7) *
                      sizeof(double));
    }
  }
  i = static_cast<unsigned char>(ilo);
  for (int j{0}; j < i; j++) {
    ia = j * 6;
    for (int b_i{0}; b_i < 6; b_i++) {
      A[ia + b_i] = 0.0;
    }
    A[ia + j] = 1.0;
  }
  i = ihi + 1;
  for (int j{i}; j < 7; j++) {
    ia = (j - 1) * 6;
    for (int b_i{0}; b_i < 6; b_i++) {
      A[ia + b_i] = 0.0;
    }
    A[(ia + j) - 1] = 1.0;
  }
  ia0 = ilo + ilo * 6;
  if (nh >= 1) {
    double work[6];
    i = nh - 1;
    for (int j{nh}; j <= i; j++) {
      ia = ia0 + j * 6;
      std::memset(&A[ia], 0,
                  static_cast<unsigned int>(((i + ia) - ia) + 1) *
                      sizeof(double));
      A[ia + j] = 1.0;
    }
    itau = (ilo + nh) - 2;
    for (int b_i{0}; b_i < 6; b_i++) {
      work[b_i] = 0.0;
    }
    for (int b_i{nh}; b_i >= 1; b_i--) {
      ia = (ia0 + b_i) + (b_i - 1) * 6;
      if (b_i < nh) {
        A[ia - 1] = 1.0;
        i = nh - b_i;
        xzlarf(i + 1, i, ia, tau[itau], A, ia + 6, work);
        a = ia + 1;
        i = (ia + nh) - b_i;
        for (int j{a}; j <= i; j++) {
          A[j - 1] *= -tau[itau];
        }
      }
      A[ia - 1] = 1.0 - tau[itau];
      i = static_cast<unsigned char>(b_i - 1);
      for (int j{0}; j < i; j++) {
        A[(ia - j) - 2] = 0.0;
      }
      itau--;
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzunghr.cpp
//
// [EOF]
//
