//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdtrevc3.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "xdtrevc3.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdlaln2.h"
#include "xgemv.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double T[36]
//                double vr[36]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
void xdtrevc3(const double T[36], double vr[36])
{
  double work[18];
  double x[4];
  double emax;
  int ip;
  int iyend;
  int j;
  std::memset(&work[0], 0, 18U * sizeof(double));
  x[0] = 0.0;
  x[1] = 0.0;
  x[2] = 0.0;
  x[3] = 0.0;
  work[0] = 0.0;
  for (j = 0; j < 5; j++) {
    work[j + 1] = 0.0;
    for (iyend = 0; iyend <= j; iyend++) {
      work[j + 1] += std::abs(T[iyend + 6 * (j + 1)]);
    }
  }
  ip = 0;
  for (int ki{5}; ki >= 0; ki--) {
    if (ip == -1) {
      ip = 1;
    } else {
      double smin;
      double wi;
      double wr_tmp;
      if ((ki + 1 == 1) || (T[ki + 6 * (ki - 1)] == 0.0)) {
        ip = 0;
      } else {
        ip = -1;
      }
      iyend = ki + 6 * ki;
      wr_tmp = T[iyend];
      wi = 0.0;
      if (ip != 0) {
        wi = std::sqrt(std::abs(T[ki + 6 * (ki - 1)])) *
             std::sqrt(std::abs(T[iyend - 1]));
      }
      smin = std::fmax(2.2204460492503131E-16 * (std::abs(wr_tmp) + wi),
                       6.0125050800269183E-292);
      if (ip == 0) {
        double scale;
        int i;
        work[ki + 12] = 1.0;
        for (int k{0}; k < ki; k++) {
          work[k + 12] = -T[k + 6 * ki];
        }
        j = ki - 1;
        int exitg1;
        do {
          exitg1 = 0;
          if (j + 1 >= 1) {
            boolean_T guard1;
            guard1 = false;
            if (j + 1 == 1) {
              guard1 = true;
            } else {
              int i1;
              i = 6 * (j - 1);
              i1 = j + i;
              if (T[i1] == 0.0) {
                guard1 = true;
              } else {
                scale = xdlaln2(2, 1, smin, T, i1, work, j + 12, wr_tmp, 0.0, x,
                                emax);
                if ((emax > 1.0) && (std::fmax(work[j - 1], work[j]) >
                                     1.6632002579455995E+291 / emax)) {
                  x[0] /= emax;
                  x[1] /= emax;
                  scale /= emax;
                }
                if (scale != 1.0) {
                  i1 = ki + 13;
                  for (int k{13}; k <= i1; k++) {
                    work[k - 1] *= scale;
                  }
                }
                work[j + 11] = x[0];
                work[j + 12] = x[1];
                blas::xaxpy(j - 1, -x[0], T, i + 1, work);
                blas::xaxpy(j - 1, -x[1], T, j * 6 + 1, work);
                j -= 2;
              }
            }
            if (guard1) {
              scale = xdlaln2(1, 1, smin, T, (j * 6 + j) + 1, work, j + 13,
                              wr_tmp, 0.0, x, emax);
              if ((emax > 1.0) && (work[j] > 1.6632002579455995E+291 / emax)) {
                x[0] /= emax;
                scale /= emax;
              }
              if (scale != 1.0) {
                i = ki + 13;
                for (int k{13}; k <= i; k++) {
                  work[k - 1] *= scale;
                }
              }
              work[j + 12] = x[0];
              blas::xaxpy(j, -x[0], T, j * 6 + 1, work);
              j--;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        if (ki + 1 > 1) {
          blas::xgemv(ki, work, work[ki + 12], vr, ki * 6 + 1);
        }
        j = ki * 6;
        iyend = -1;
        emax = std::abs(vr[j]);
        for (int k{0}; k < 5; k++) {
          scale = std::abs(vr[(j + k) + 1]);
          if (scale > emax) {
            iyend = k;
            emax = scale;
          }
        }
        emax = 1.0 / std::abs(vr[(iyend + 6 * ki) + 1]);
        i = j + 6;
        for (int k{j + 1}; k <= i; k++) {
          vr[k - 1] *= emax;
        }
      } else {
        double scale;
        int i;
        int i1;
        int ix;
        int ix0;
        emax = T[iyend - 1];
        ix0 = 6 * (ki - 1);
        scale = T[ki + ix0];
        if (std::abs(emax) >= std::abs(scale)) {
          work[ki + 5] = 1.0;
          work[ki + 12] = wi / emax;
        } else {
          work[ki + 5] = -wi / scale;
          work[ki + 12] = 1.0;
        }
        work[ki + 6] = 0.0;
        work[ki + 11] = 0.0;
        for (int k{0}; k <= ki - 2; k++) {
          work[k + 6] = -work[ki + 5] * T[k + ix0];
          work[k + 12] = -work[ki + 12] * T[k + 6 * ki];
        }
        j = ki - 2;
        int exitg1;
        do {
          exitg1 = 0;
          if (j + 1 >= 1) {
            boolean_T guard1;
            guard1 = false;
            if (j + 1 == 1) {
              guard1 = true;
            } else {
              i = 6 * (j - 1);
              i1 = j + i;
              if (T[i1] == 0.0) {
                guard1 = true;
              } else {
                scale = xdlaln2(2, 2, smin, T, i1, work, j + 6, wr_tmp, wi, x,
                                emax);
                if ((emax > 1.0) && (std::fmax(work[j - 1], work[j]) >
                                     1.6632002579455995E+291 / emax)) {
                  emax = 1.0 / emax;
                  x[0] *= emax;
                  x[2] *= emax;
                  x[1] *= emax;
                  x[3] *= emax;
                  scale *= emax;
                }
                if (scale != 1.0) {
                  i1 = ki + 7;
                  for (int k{7}; k <= i1; k++) {
                    work[k - 1] *= scale;
                  }
                  i1 = ki + 13;
                  for (int k{13}; k <= i1; k++) {
                    work[k - 1] *= scale;
                  }
                }
                work[j + 5] = x[0];
                work[j + 6] = x[1];
                work[j + 11] = x[2];
                work[j + 12] = x[3];
                if ((j - 1 >= 1) && (!(-x[0] == 0.0))) {
                  i1 = j - 2;
                  for (int k{0}; k <= i1; k++) {
                    work[k + 6] += -x[0] * T[i + k];
                  }
                }
                if ((j - 1 >= 1) && (!(-x[1] == 0.0))) {
                  ix = j * 6;
                  i1 = j - 2;
                  for (int k{0}; k <= i1; k++) {
                    work[k + 6] += -x[1] * T[ix + k];
                  }
                }
                blas::xaxpy(j - 1, -x[2], T, i + 1, work);
                blas::xaxpy(j - 1, -x[3], T, j * 6 + 1, work);
                j -= 2;
              }
            }
            if (guard1) {
              scale = xdlaln2(1, 2, smin, T, (j * 6 + j) + 1, work, j + 7,
                              wr_tmp, wi, x, emax);
              if ((emax > 1.0) && (work[j] > 1.6632002579455995E+291 / emax)) {
                x[0] /= emax;
                x[2] /= emax;
                scale /= emax;
              }
              if (scale != 1.0) {
                i = ki + 7;
                for (int k{7}; k <= i; k++) {
                  work[k - 1] *= scale;
                }
                i = ki + 13;
                for (int k{13}; k <= i; k++) {
                  work[k - 1] *= scale;
                }
              }
              work[j + 6] = x[0];
              work[j + 12] = x[2];
              if ((j >= 1) && (!(-x[0] == 0.0))) {
                ix = j * 6;
                i = j - 1;
                for (int k{0}; k <= i; k++) {
                  work[k + 6] += -x[0] * T[ix + k];
                }
              }
              blas::xaxpy(j, -x[2], T, j * 6 + 1, work);
              j--;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        if (ki + 1 > 2) {
          iyend = ix0 + 6;
          emax = work[ki + 5];
          if (emax != 1.0) {
            if (emax == 0.0) {
              if (ix0 + 1 <= iyend) {
                std::memset(&vr[ix0], 0,
                            static_cast<unsigned int>(iyend - ix0) *
                                sizeof(double));
              }
            } else {
              for (j = ix0 + 1; j <= iyend; j++) {
                vr[j - 1] *= emax;
              }
            }
          }
          ix = 6;
          i = 6 * (ki - 2) + 1;
          for (j = 1; j <= i; j += 6) {
            i1 = j + 5;
            for (int k{j}; k <= i1; k++) {
              iyend = (ix0 + k) - j;
              vr[iyend] += vr[k - 1] * work[ix];
            }
            ix++;
          }
          blas::xgemv(ki - 1, work, work[ki + 12], vr, ki * 6 + 1);
        } else {
          i = ix0 + 6;
          for (int k{ix0 + 1}; k <= i; k++) {
            vr[k - 1] *= work[6];
          }
          iyend = ki * 6;
          i = iyend + 6;
          for (int k{iyend + 1}; k <= i; k++) {
            vr[k - 1] *= work[ki + 12];
          }
        }
        emax = 0.0;
        for (int k{0}; k < 6; k++) {
          emax =
              std::fmax(emax, std::abs(vr[k + ix0]) + std::abs(vr[k + 6 * ki]));
        }
        emax = 1.0 / emax;
        i = ix0 + 6;
        for (int k{ix0 + 1}; k <= i; k++) {
          vr[k - 1] *= emax;
        }
        ix0 = ki * 6;
        i = ix0 + 6;
        for (int k{ix0 + 1}; k <= i; k++) {
          vr[k - 1] *= emax;
        }
      }
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xdtrevc3.cpp
//
// [EOF]
//
