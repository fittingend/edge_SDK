//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: gaussEKFilter.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

#ifndef GAUSSEKFILTER_H
#define GAUSSEKFILTER_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace fusion {
namespace internal {
class gaussEKFilter {
public:
  static void predict(const double x[6], double P[36], const double Q[9],
                      double varargin_1, double xk[6]);
};

} // namespace internal
} // namespace fusion
} // namespace coder

#endif
//
// File trailer for gaussEKFilter.h
//
// [EOF]
//
