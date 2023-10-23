//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: gaussEKFilter.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "gaussEKFilter.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double x[6]
//                double P[36]
//                const double Q[9]
//                double varargin_1
//                double xk[6]
// Return Type  : void
//
namespace coder {
namespace fusion {
namespace internal {
void gaussEKFilter::predict(const double x[6], double P[36], const double Q[9],
                            double varargin_1, double xk[6])
{
  double F[36];
  double b_F[36];
  double U[18];
  double b_U[18];
  double imvec[6];
  double z[6];
  double d;
  double xk_tmp;
  double xk_tmp_tmp;
  xk_tmp_tmp = 0.5 * (varargin_1 * varargin_1);
  xk_tmp = xk_tmp_tmp * 0.0;
  for (int i{0}; i < 6; i++) {
    xk[i] = x[i];
    z[i] = x[i];
  }
  xk[0] = (xk[0] + xk[1] * varargin_1) + xk_tmp;
  xk[1] += 0.0 * varargin_1;
  xk[2] = (xk[2] + xk[3] * varargin_1) + xk_tmp;
  xk[3] += 0.0 * varargin_1;
  xk[4] = (xk[4] + xk[5] * varargin_1) + xk_tmp;
  xk[5] += 0.0 * varargin_1;
  z[0] = (z[0] + z[1] * varargin_1) + xk_tmp;
  z[1] += 0.0 * varargin_1;
  z[2] = (z[2] + z[3] * varargin_1) + xk_tmp;
  z[3] += 0.0 * varargin_1;
  z[4] = (z[4] + z[5] * varargin_1) + xk_tmp;
  z[5] += 0.0 * varargin_1;
  for (int j{0}; j < 6; j++) {
    double epsilon;
    for (int i{0}; i < 6; i++) {
      imvec[i] = x[i];
    }
    d = x[j];
    epsilon =
        std::fmax(1.4901161193847656E-8, 1.4901161193847656E-8 * std::abs(d));
    imvec[j] = d + epsilon;
    imvec[0] = (imvec[0] + imvec[1] * varargin_1) + xk_tmp;
    imvec[1] += 0.0 * varargin_1;
    imvec[2] = (imvec[2] + imvec[3] * varargin_1) + xk_tmp;
    imvec[3] += 0.0 * varargin_1;
    imvec[4] = (imvec[4] + imvec[5] * varargin_1) + xk_tmp;
    imvec[5] += 0.0 * varargin_1;
    for (int b_i{0}; b_i < 6; b_i++) {
      F[b_i + 6 * j] = (imvec[b_i] - z[b_i]) / epsilon;
    }
  }
  for (int i{0}; i < 6; i++) {
    z[i] = x[i];
  }
  z[0] = (z[0] + z[1] * varargin_1) + xk_tmp;
  z[1] += 0.0 * varargin_1;
  z[2] = (z[2] + z[3] * varargin_1) + xk_tmp;
  z[3] += 0.0 * varargin_1;
  z[4] = (z[4] + z[5] * varargin_1) + xk_tmp;
  z[5] += 0.0 * varargin_1;
  for (int j{0}; j < 3; j++) {
    double specvec_f2[3];
    specvec_f2[0] = 0.0;
    specvec_f2[1] = 0.0;
    specvec_f2[2] = 0.0;
    specvec_f2[j] = 1.4901161193847656E-8;
    for (int i{0}; i < 6; i++) {
      imvec[i] = x[i];
    }
    imvec[0] = (imvec[0] + imvec[1] * varargin_1) + xk_tmp_tmp * specvec_f2[0];
    imvec[1] += specvec_f2[0] * varargin_1;
    imvec[2] = (imvec[2] + imvec[3] * varargin_1) + xk_tmp_tmp * specvec_f2[1];
    imvec[3] += specvec_f2[1] * varargin_1;
    imvec[4] = (imvec[4] + imvec[5] * varargin_1) + xk_tmp_tmp * specvec_f2[2];
    imvec[5] += specvec_f2[2] * varargin_1;
    for (int b_i{0}; b_i < 6; b_i++) {
      U[b_i + 6 * j] = (imvec[b_i] - z[b_i]) / 1.4901161193847656E-8;
    }
  }
  for (int b_i{0}; b_i < 6; b_i++) {
    for (int i{0}; i < 6; i++) {
      d = 0.0;
      for (int j{0}; j < 6; j++) {
        d += F[b_i + 6 * j] * P[j + 6 * i];
      }
      b_F[b_i + 6 * i] = d;
    }
    d = U[b_i];
    xk_tmp_tmp = U[b_i + 6];
    xk_tmp = U[b_i + 12];
    for (int i{0}; i < 3; i++) {
      b_U[b_i + 6 * i] =
          (d * Q[3 * i] + xk_tmp_tmp * Q[3 * i + 1]) + xk_tmp * Q[3 * i + 2];
    }
  }
  for (int b_i{0}; b_i < 6; b_i++) {
    for (int i{0}; i < 6; i++) {
      d = 0.0;
      for (int j{0}; j < 6; j++) {
        d += b_F[b_i + 6 * j] * F[i + 6 * j];
      }
      P[b_i + 6 * i] = d;
    }
  }
  for (int b_i{0}; b_i < 6; b_i++) {
    d = b_U[b_i];
    xk_tmp_tmp = b_U[b_i + 6];
    xk_tmp = b_U[b_i + 12];
    for (int i{0}; i < 6; i++) {
      F[b_i + 6 * i] = (d * U[i] + xk_tmp_tmp * U[i + 6]) + xk_tmp * U[i + 12];
    }
  }
  for (int b_i{0}; b_i < 36; b_i++) {
    P[b_i] += F[b_i];
  }
}

} // namespace internal
} // namespace fusion
} // namespace coder

//
// File trailer for gaussEKFilter.cpp
//
// [EOF]
//
