//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdlahqr.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

#ifndef XDLAHQR_H
#define XDLAHQR_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
int xdlahqr(int ilo, int ihi, double h[36], int iloz, int ihiz, double z[36],
            double wr[6], double wi[6]);

}
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xdlahqr.h
//
// [EOF]
//
