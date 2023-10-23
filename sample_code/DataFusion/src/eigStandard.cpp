//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eigStandard.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "eigStandard.h"
#include "EDGE_fusion_function_231019_2222_rtwutil.h"
#include "rt_nonfinite.h"
#include "xdlahqr.h"
#include "xdtrevc3.h"
#include "xnrm2.h"
#include "xzgebal.h"
#include "xzgehrd.h"
#include "xzlascl.h"
#include "xzunghr.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double A[36]
//                creal_T V[36]
//                creal_T D[36]
// Return Type  : void
//
namespace coder {
void eigStandard(const double A[36], creal_T V[36], creal_T D[36])
{
  creal_T W[6];
  double b_A[36];
  double vr[36];
  double absxk;
  double anrm;
  int ihi;
  int k;
  boolean_T exitg1;
  std::copy(&A[0], &A[36], &b_A[0]);
  anrm = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 36)) {
    absxk = std::abs(A[k]);
    if (std::isnan(absxk)) {
      anrm = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }
      k++;
    }
  }
  if (std::isinf(anrm) || std::isnan(anrm)) {
    for (int i{0}; i < 6; i++) {
      W[i].re = rtNaN;
      W[i].im = 0.0;
    }
    for (int b_i{0}; b_i < 36; b_i++) {
      V[b_i].re = rtNaN;
      V[b_i].im = 0.0;
    }
  } else {
    double scale[6];
    double wi[6];
    double wr[6];
    double tau[5];
    double cscale;
    int ilo;
    int info;
    boolean_T scalea;
    cscale = anrm;
    scalea = false;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      scalea = true;
      cscale = 6.7178761075670888E-139;
      internal::reflapack::xzlascl(anrm, cscale, b_A);
    } else if (anrm > 1.4885657073574029E+138) {
      scalea = true;
      cscale = 1.4885657073574029E+138;
      internal::reflapack::xzlascl(anrm, cscale, b_A);
    }
    ilo = internal::reflapack::xzgebal(b_A, ihi, scale);
    internal::reflapack::xzgehrd(b_A, ilo, ihi, tau);
    std::copy(&b_A[0], &b_A[36], &vr[0]);
    internal::reflapack::xzunghr(ilo, ihi, vr, tau);
    info = internal::reflapack::xdlahqr(ilo, ihi, b_A, ilo, ihi, vr, wr, wi);
    if (info == 0) {
      double f1;
      int b_i;
      int temp_tmp;
      int vr_tmp;
      internal::reflapack::xdtrevc3(b_A, vr);
      if (ilo != ihi) {
        for (int i{ilo}; i <= ihi; i++) {
          b_i = i + 30;
          for (k = i; k <= b_i; k += 6) {
            vr[k - 1] *= scale[i - 1];
          }
        }
      }
      b_i = ilo - 1;
      for (int i{b_i}; i >= 1; i--) {
        f1 = scale[i - 1];
        if (static_cast<int>(f1) != i) {
          for (k = 0; k < 6; k++) {
            temp_tmp = (i + k * 6) - 1;
            absxk = vr[temp_tmp];
            vr_tmp = (static_cast<int>(f1) + k * 6) - 1;
            vr[temp_tmp] = vr[vr_tmp];
            vr[vr_tmp] = absxk;
          }
        }
      }
      b_i = ihi + 1;
      for (int i{b_i}; i < 7; i++) {
        f1 = scale[i - 1];
        if (static_cast<int>(f1) != i) {
          for (k = 0; k < 6; k++) {
            temp_tmp = (i + k * 6) - 1;
            absxk = vr[temp_tmp];
            vr_tmp = (static_cast<int>(f1) + k * 6) - 1;
            vr[temp_tmp] = vr[vr_tmp];
            vr[vr_tmp] = absxk;
          }
        }
      }
      for (int i{0}; i < 6; i++) {
        f1 = wi[i];
        if (!(f1 < 0.0)) {
          if ((i + 1 != 6) && (f1 > 0.0)) {
            double c;
            double f1_tmp;
            double g1_tmp;
            double s;
            int ix0_tmp;
            int scl_tmp;
            scl_tmp = (i + 1) * 6;
            absxk =
                1.0 / rt_hypotd_snf(internal::blas::xnrm2(6, vr, i * 6 + 1),
                                    internal::blas::xnrm2(6, vr, scl_tmp + 1));
            ix0_tmp = i * 6;
            b_i = ix0_tmp + 6;
            for (k = ix0_tmp + 1; k <= b_i; k++) {
              vr[k - 1] *= absxk;
            }
            b_i = scl_tmp + 6;
            for (k = scl_tmp + 1; k <= b_i; k++) {
              vr[k - 1] *= absxk;
            }
            for (vr_tmp = 0; vr_tmp < 6; vr_tmp++) {
              f1 = vr[vr_tmp + 6 * i];
              absxk = vr[vr_tmp + scl_tmp];
              scale[vr_tmp] = f1 * f1 + absxk * absxk;
            }
            k = 0;
            absxk = std::abs(scale[0]);
            for (ihi = 0; ihi < 5; ihi++) {
              s = std::abs(scale[ihi + 1]);
              if (s > absxk) {
                k = ihi + 1;
                absxk = s;
              }
            }
            f1_tmp = vr[k + 6 * i];
            f1 = std::abs(f1_tmp);
            ihi = k + scl_tmp;
            g1_tmp = vr[ihi];
            absxk = std::abs(g1_tmp);
            if (g1_tmp == 0.0) {
              c = 1.0;
              s = 0.0;
            } else if (f1_tmp == 0.0) {
              c = 0.0;
              if (g1_tmp >= 0.0) {
                s = 1.0;
              } else {
                s = -1.0;
              }
            } else if ((f1 > 1.4916681462400413E-154) &&
                       (f1 < 4.7403759540545887E+153) &&
                       (absxk > 1.4916681462400413E-154) &&
                       (absxk < 4.7403759540545887E+153)) {
              double d;
              d = std::sqrt(f1_tmp * f1_tmp + g1_tmp * g1_tmp);
              c = f1 / d;
              if (!(f1_tmp >= 0.0)) {
                d = -d;
              }
              s = g1_tmp / d;
            } else {
              double d;
              absxk = std::fmin(
                  4.49423283715579E+307,
                  std::fmax(2.2250738585072014E-308, std::fmax(f1, absxk)));
              s = f1_tmp / absxk;
              absxk = g1_tmp / absxk;
              d = std::sqrt(s * s + absxk * absxk);
              c = std::abs(s) / d;
              if (!(f1_tmp >= 0.0)) {
                d = -d;
              }
              s = absxk / d;
            }
            for (k = 0; k < 6; k++) {
              temp_tmp = scl_tmp + k;
              absxk = vr[temp_tmp];
              vr_tmp = ix0_tmp + k;
              f1 = vr[vr_tmp];
              vr[temp_tmp] = c * absxk - s * f1;
              vr[vr_tmp] = c * f1 + s * absxk;
            }
            vr[ihi] = 0.0;
          } else {
            absxk = 1.0 / internal::blas::xnrm2(6, vr, i * 6 + 1);
            ihi = i * 6;
            b_i = ihi + 6;
            for (k = ihi + 1; k <= b_i; k++) {
              vr[k - 1] *= absxk;
            }
          }
        }
      }
      for (b_i = 0; b_i < 36; b_i++) {
        V[b_i].re = vr[b_i];
        V[b_i].im = 0.0;
      }
      for (vr_tmp = 0; vr_tmp < 5; vr_tmp++) {
        if ((wi[vr_tmp] > 0.0) && (wi[vr_tmp + 1] < 0.0)) {
          for (int i{0}; i < 6; i++) {
            ihi = i + 6 * vr_tmp;
            temp_tmp = i + 6 * (vr_tmp + 1);
            absxk = V[temp_tmp].re;
            V[ihi].im = absxk;
            V[temp_tmp].re = V[ihi].re;
            V[temp_tmp].im = -absxk;
          }
        }
      }
    } else {
      for (int b_i{0}; b_i < 36; b_i++) {
        V[b_i].re = rtNaN;
        V[b_i].im = 0.0;
      }
    }
    if (scalea) {
      internal::reflapack::xzlascl(cscale, anrm, 6 - info, wr, info + 1);
      internal::reflapack::xzlascl(cscale, anrm, 6 - info, wi, info + 1);
      if (info != 0) {
        internal::reflapack::xzlascl(cscale, anrm, ilo - 1, wr, 1);
        internal::reflapack::xzlascl(cscale, anrm, ilo - 1, wi, 1);
      }
    }
    if (info != 0) {
      for (int i{ilo}; i <= info; i++) {
        wr[i - 1] = rtNaN;
        wi[i - 1] = 0.0;
      }
    }
    for (int i{0}; i < 6; i++) {
      W[i].re = wr[i];
      W[i].im = wi[i];
    }
  }
  std::memset(&D[0], 0, 36U * sizeof(creal_T));
  for (k = 0; k < 6; k++) {
    D[k + 6 * k] = W[k];
  }
}

} // namespace coder

//
// File trailer for eigStandard.cpp
//
// [EOF]
//
