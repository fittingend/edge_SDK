//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzlascl.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

#ifndef XZLASCL_H
#define XZLASCL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
void b_xzlascl(double cfrom, double cto, int m, double A[5], int iA0);

void xzlascl(double cfrom, double cto, int m, double A[6], int iA0);

void xzlascl(double cfrom, double cto, double A[36]);

} // namespace reflapack
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xzlascl.h
//
// [EOF]
//
