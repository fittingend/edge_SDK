//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzgebal.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "xzgebal.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double A[36]
//                int &ihi
//                double scale[6]
// Return Type  : int
//
namespace coder {
namespace internal {
namespace reflapack {
int xzgebal(double A[36], int &ihi, double scale[6])
{
  double b_scale;
  int b_i;
  int b_ix0_tmp;
  int exitg5;
  int i;
  int ilo;
  int ira;
  int ix0_tmp;
  int kend;
  boolean_T converged;
  boolean_T notdone;
  for (i = 0; i < 6; i++) {
    scale[i] = 1.0;
  }
  ilo = 1;
  ihi = 6;
  notdone = true;
  do {
    exitg5 = 0;
    if (notdone) {
      int exitg4;
      int j;
      notdone = false;
      j = ihi;
      do {
        exitg4 = 0;
        if (j > 0) {
          boolean_T exitg6;
          converged = false;
          i = 0;
          exitg6 = false;
          while ((!exitg6) && (i <= static_cast<unsigned char>(ihi) - 1)) {
            if ((i + 1 == j) || (!(A[(j + 6 * i) - 1] != 0.0))) {
              i++;
            } else {
              converged = true;
              exitg6 = true;
            }
          }
          if (converged) {
            j--;
          } else {
            scale[ihi - 1] = j;
            if (j != ihi) {
              ira = (j - 1) * 6;
              kend = (ihi - 1) * 6;
              b_i = static_cast<unsigned char>(ihi);
              for (int k{0}; k < b_i; k++) {
                b_ix0_tmp = ira + k;
                b_scale = A[b_ix0_tmp];
                ix0_tmp = kend + k;
                A[b_ix0_tmp] = A[ix0_tmp];
                A[ix0_tmp] = b_scale;
              }
              for (int k{0}; k < 6; k++) {
                b_ix0_tmp = (j + k * 6) - 1;
                b_scale = A[b_ix0_tmp];
                b_i = (ihi + k * 6) - 1;
                A[b_ix0_tmp] = A[b_i];
                A[b_i] = b_scale;
              }
            }
            exitg4 = 1;
          }
        } else {
          exitg4 = 2;
        }
      } while (exitg4 == 0);
      if (exitg4 == 1) {
        if (ihi == 1) {
          ilo = 1;
          ihi = 1;
          exitg5 = 1;
        } else {
          ihi--;
          notdone = true;
        }
      }
    } else {
      notdone = true;
      while (notdone) {
        int j;
        boolean_T exitg6;
        notdone = false;
        j = ilo;
        exitg6 = false;
        while ((!exitg6) && (j <= ihi)) {
          boolean_T exitg7;
          converged = false;
          i = ilo;
          exitg7 = false;
          while ((!exitg7) && (i <= ihi)) {
            if ((i == j) || (!(A[(i + 6 * (j - 1)) - 1] != 0.0))) {
              i++;
            } else {
              converged = true;
              exitg7 = true;
            }
          }
          if (converged) {
            j++;
          } else {
            scale[ilo - 1] = j;
            if (j != ilo) {
              ira = (j - 1) * 6;
              kend = (ilo - 1) * 6;
              b_i = static_cast<unsigned char>(ihi);
              for (int k{0}; k < b_i; k++) {
                b_ix0_tmp = ira + k;
                b_scale = A[b_ix0_tmp];
                ix0_tmp = kend + k;
                A[b_ix0_tmp] = A[ix0_tmp];
                A[ix0_tmp] = b_scale;
              }
              ira = (kend + j) - 1;
              kend = (kend + ilo) - 1;
              b_i = static_cast<unsigned char>(7 - ilo);
              for (int k{0}; k < b_i; k++) {
                b_ix0_tmp = ira + k * 6;
                b_scale = A[b_ix0_tmp];
                ix0_tmp = kend + k * 6;
                A[b_ix0_tmp] = A[ix0_tmp];
                A[ix0_tmp] = b_scale;
              }
            }
            ilo++;
            notdone = true;
            exitg6 = true;
          }
        }
      }
      converged = false;
      exitg5 = 2;
    }
  } while (exitg5 == 0);
  if (exitg5 != 1) {
    boolean_T exitg3;
    exitg3 = false;
    while ((!exitg3) && (!converged)) {
      int exitg2;
      converged = true;
      i = ilo - 1;
      do {
        exitg2 = 0;
        if (i + 1 <= ihi) {
          double absxk;
          double c;
          double ca;
          double r;
          double t;
          kend = (ihi - ilo) + 1;
          c = blas::xnrm2(kend, A, i * 6 + ilo);
          ix0_tmp = (ilo - 1) * 6 + i;
          r = 0.0;
          if (kend >= 1) {
            if (kend == 1) {
              r = std::abs(A[ix0_tmp]);
            } else {
              b_scale = 3.3121686421112381E-170;
              kend = (ix0_tmp + (kend - 1) * 6) + 1;
              for (int k{ix0_tmp + 1}; k <= kend; k += 6) {
                absxk = std::abs(A[k - 1]);
                if (absxk > b_scale) {
                  t = b_scale / absxk;
                  r = r * t * t + 1.0;
                  b_scale = absxk;
                } else {
                  t = absxk / b_scale;
                  r += t * t;
                }
              }
              r = b_scale * std::sqrt(r);
            }
          }
          b_ix0_tmp = i * 6;
          kend = 1;
          if (ihi > 1) {
            b_scale = std::abs(A[b_ix0_tmp]);
            for (int k{2}; k <= ihi; k++) {
              t = std::abs(A[(b_ix0_tmp + k) - 1]);
              if (t > b_scale) {
                kend = k;
                b_scale = t;
              }
            }
          }
          ca = std::abs(A[(kend + 6 * i) - 1]);
          kend = 7 - ilo;
          if (7 - ilo < 1) {
            ira = 0;
          } else {
            ira = 1;
            if (7 - ilo > 1) {
              b_scale = std::abs(A[ix0_tmp]);
              for (int k{2}; k <= kend; k++) {
                t = std::abs(A[ix0_tmp + (k - 1) * 6]);
                if (t > b_scale) {
                  ira = k;
                  b_scale = t;
                }
              }
            }
          }
          b_scale = std::abs(A[i + 6 * ((ira + ilo) - 2)]);
          if ((c == 0.0) || (r == 0.0)) {
            i++;
          } else {
            double f;
            int exitg1;
            absxk = r / 2.0;
            f = 1.0;
            t = c + r;
            do {
              exitg1 = 0;
              if ((c < absxk) &&
                  (std::fmax(f, std::fmax(c, ca)) < 4.9896007738368E+291) &&
                  (std::fmin(r, std::fmin(absxk, b_scale)) >
                   2.0041683600089728E-292)) {
                if (std::isnan(((((c + f) + ca) + r) + absxk) + b_scale)) {
                  exitg1 = 1;
                } else {
                  f *= 2.0;
                  c *= 2.0;
                  ca *= 2.0;
                  r /= 2.0;
                  absxk /= 2.0;
                  b_scale /= 2.0;
                }
              } else {
                absxk = c / 2.0;
                while ((absxk >= r) &&
                       (std::fmax(r, b_scale) < 4.9896007738368E+291) &&
                       (std::fmin(std::fmin(f, c), std::fmin(absxk, ca)) >
                        2.0041683600089728E-292)) {
                  f /= 2.0;
                  c /= 2.0;
                  absxk /= 2.0;
                  ca /= 2.0;
                  r *= 2.0;
                  b_scale *= 2.0;
                }
                if ((!(c + r >= 0.95 * t)) &&
                    ((!(f < 1.0)) || (!(scale[i] < 1.0)) ||
                     (!(f * scale[i] <= 1.0020841800044864E-292))) &&
                    ((!(f > 1.0)) || (!(scale[i] > 1.0)) ||
                     (!(scale[i] >= 9.9792015476736E+291 / f)))) {
                  b_scale = 1.0 / f;
                  scale[i] *= f;
                  kend = ix0_tmp + 1;
                  b_i = (ix0_tmp + 6 * (6 - ilo)) + 1;
                  for (int k{kend}; k <= b_i; k += 6) {
                    A[k - 1] *= b_scale;
                  }
                  b_i = b_ix0_tmp + ihi;
                  for (int k{b_ix0_tmp + 1}; k <= b_i; k++) {
                    A[k - 1] *= f;
                  }
                  converged = false;
                }
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = 2;
            } else {
              i++;
            }
          }
        } else {
          exitg2 = 1;
        }
      } while (exitg2 == 0);
      if (exitg2 != 1) {
        exitg3 = true;
      }
    }
  }
  return ilo;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzgebal.cpp
//
// [EOF]
//
