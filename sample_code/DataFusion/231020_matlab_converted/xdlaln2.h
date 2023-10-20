//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdlaln2.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

#ifndef XDLALN2_H
#define XDLALN2_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
double xdlaln2(int na, int nw, double smin, const double A[36], int ia0,
               const double B[18], int ib0, double wr, double wi, double X[4],
               double &xnorm);

}
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xdlaln2.h
//
// [EOF]
//
