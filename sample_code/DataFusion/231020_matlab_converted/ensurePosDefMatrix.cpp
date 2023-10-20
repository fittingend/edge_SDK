//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ensurePosDefMatrix.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "ensurePosDefMatrix.h"
#include "eigStandard.h"
#include "rt_nonfinite.h"
#include "xdlahqr.h"
#include "xzgehrd.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "xzlascl.h"
#include "xzsteqr.h"
#include "xzunghr.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : double P[36]
// Return Type  : void
//
namespace coder {
namespace fusion {
namespace internal {
void ensurePosDefMatrix(double P[36])
{
  creal_T D[36];
  creal_T V[36];
  creal_T b_V[36];
  double A[36];
  double b_D[36];
  double work[6];
  double absx;
  double anrm;
  double taui;
  double temp1;
  double temp2;
  int i;
  int i1;
  int i2;
  int iaii;
  int k;
  int sgn;
  boolean_T iscale;
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      iaii = i1 + 6 * i;
      A[iaii] = (P[iaii] + P[i + 6 * i1]) / 2.0;
    }
  }
  iscale = true;
  for (k = 0; k < 36; k++) {
    if (iscale) {
      absx = A[k];
      if (std::isinf(absx) || std::isnan(absx)) {
        iscale = false;
      }
    } else {
      iscale = false;
    }
  }
  if (!iscale) {
    for (i = 0; i < 36; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
      D[i].re = 0.0;
      D[i].im = 0.0;
    }
    for (k = 0; k < 6; k++) {
      i = k + 6 * k;
      D[i].re = rtNaN;
      D[i].im = 0.0;
    }
  } else {
    int b_i;
    int exitg1;
    boolean_T exitg2;
    iscale = true;
    k = 0;
    exitg2 = false;
    while ((!exitg2) && (k < 6)) {
      b_i = 0;
      do {
        exitg1 = 0;
        if (b_i <= k) {
          if (!(A[b_i + 6 * k] == A[k + 6 * b_i])) {
            iscale = false;
            exitg1 = 1;
          } else {
            b_i++;
          }
        } else {
          k++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
    if (iscale) {
      double a__4[6];
      anrm = 0.0;
      k = 0;
      exitg2 = false;
      while ((!exitg2) && (k < 6)) {
        b_i = 0;
        do {
          exitg1 = 0;
          if (b_i <= k) {
            absx = std::abs(A[b_i + 6 * k]);
            if (std::isnan(absx)) {
              anrm = rtNaN;
              exitg1 = 1;
            } else {
              if (absx > anrm) {
                anrm = absx;
              }
              b_i++;
            }
          } else {
            k++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (std::isinf(anrm) || std::isnan(anrm)) {
        for (b_i = 0; b_i < 6; b_i++) {
          a__4[b_i] = rtNaN;
        }
        for (i = 0; i < 36; i++) {
          A[i] = rtNaN;
        }
      } else {
        double e[5];
        double tau[5];
        iscale = false;
        if ((anrm > 0.0) && (anrm < 1.0010415475915505E-146)) {
          iscale = true;
          anrm = 1.0010415475915505E-146 / anrm;
          ::coder::internal::reflapack::xzlascl(1.0, anrm, A);
        } else if (anrm > 9.9895953610111751E+145) {
          iscale = true;
          anrm = 9.9895953610111751E+145 / anrm;
          ::coder::internal::reflapack::xzlascl(1.0, anrm, A);
        }
        for (b_i = 0; b_i < 5; b_i++) {
          int e_tmp_tmp;
          e_tmp_tmp = b_i + 6 * b_i;
          e[b_i] = A[e_tmp_tmp + 1];
          sgn = b_i + 3;
          if (sgn > 6) {
            sgn = 6;
          }
          taui = ::coder::internal::reflapack::xzlarfg(5 - b_i, e[b_i], A,
                                                       b_i * 6 + sgn);
          if (taui != 0.0) {
            int tau_tmp;
            A[e_tmp_tmp + 1] = 1.0;
            for (sgn = b_i + 1; sgn < 6; sgn++) {
              tau[sgn - 1] = 0.0;
            }
            i = 4 - b_i;
            i1 = 5 - b_i;
            for (int jj{0}; jj <= i; jj++) {
              iaii = b_i + jj;
              temp1 = taui * A[(iaii + 6 * b_i) + 1];
              temp2 = 0.0;
              tau_tmp = 6 * (iaii + 1);
              tau[iaii] += temp1 * A[(iaii + tau_tmp) + 1];
              i2 = jj + 2;
              for (int ii{i2}; ii <= i1; ii++) {
                sgn = b_i + ii;
                absx = A[sgn + tau_tmp];
                tau[sgn - 1] += temp1 * absx;
                temp2 += absx * A[sgn + 6 * b_i];
              }
              tau[iaii] += taui * temp2;
            }
            temp2 = 0.0;
            for (k = 0; k <= i; k++) {
              temp2 += tau[b_i + k] * A[(e_tmp_tmp + k) + 1];
            }
            temp2 *= -0.5 * taui;
            if (!(temp2 == 0.0)) {
              for (k = 0; k <= i; k++) {
                tau_tmp = b_i + k;
                tau[tau_tmp] += temp2 * A[(e_tmp_tmp + k) + 1];
              }
            }
            for (int jj{0}; jj <= i; jj++) {
              iaii = b_i + jj;
              temp1 = A[(iaii + 6 * b_i) + 1];
              absx = tau[iaii];
              temp2 = absx * temp1;
              k = 6 * (iaii + 1);
              iaii = (iaii + k) + 1;
              A[iaii] = (A[iaii] - temp2) - temp2;
              i2 = jj + 2;
              for (int ii{i2}; ii <= i1; ii++) {
                iaii = b_i + ii;
                sgn = iaii + k;
                A[sgn] =
                    (A[sgn] - tau[iaii - 1] * temp1) - A[iaii + 6 * b_i] * absx;
              }
            }
          }
          A[e_tmp_tmp + 1] = e[b_i];
          a__4[b_i] = A[e_tmp_tmp];
          tau[b_i] = taui;
        }
        a__4[5] = A[35];
        for (k = 4; k >= 0; k--) {
          iaii = 6 * (k + 1);
          A[iaii] = 0.0;
          i = k + 3;
          for (b_i = i; b_i < 7; b_i++) {
            A[(b_i + iaii) - 1] = A[(b_i + 6 * k) - 1];
          }
        }
        A[0] = 1.0;
        for (b_i = 0; b_i < 5; b_i++) {
          A[b_i + 1] = 0.0;
        }
        for (b_i = 0; b_i < 6; b_i++) {
          work[b_i] = 0.0;
        }
        for (b_i = 4; b_i >= 0; b_i--) {
          iaii = (b_i + b_i * 6) + 7;
          if (b_i + 1 < 5) {
            A[iaii] = 1.0;
            ::coder::internal::reflapack::xzlarf(5 - b_i, 4 - b_i, iaii + 1,
                                                 tau[b_i], A, iaii + 7, work);
            sgn = iaii + 2;
            i = (iaii - b_i) + 5;
            for (k = sgn; k <= i; k++) {
              A[k - 1] *= -tau[b_i];
            }
          }
          A[iaii] = 1.0 - tau[b_i];
          for (k = 0; k < b_i; k++) {
            A[(iaii - k) - 1] = 0.0;
          }
        }
        sgn = ::coder::internal::reflapack::xzsteqr(a__4, e, A);
        if (sgn != 0) {
          for (b_i = 0; b_i < 6; b_i++) {
            a__4[b_i] = rtNaN;
          }
          for (i = 0; i < 36; i++) {
            A[i] = rtNaN;
          }
        } else if (iscale) {
          temp2 = 1.0 / anrm;
          for (k = 0; k < 6; k++) {
            a__4[k] *= temp2;
          }
        }
      }
      std::memset(&D[0], 0, 36U * sizeof(creal_T));
      for (b_i = 0; b_i < 6; b_i++) {
        i = b_i + 6 * b_i;
        D[i].re = a__4[b_i];
        D[i].im = 0.0;
      }
      for (i = 0; i < 36; i++) {
        V[i].re = A[i];
        V[i].im = 0.0;
      }
    } else {
      iscale = true;
      k = 0;
      exitg2 = false;
      while ((!exitg2) && (k < 6)) {
        b_i = 0;
        do {
          exitg1 = 0;
          if (b_i <= k) {
            if (!(A[b_i + 6 * k] == -A[k + 6 * b_i])) {
              iscale = false;
              exitg1 = 1;
            } else {
              b_i++;
            }
          } else {
            k++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (iscale) {
        double a__4[6];
        double tau[5];
        ::coder::internal::reflapack::xzgehrd(A, 1, 6, tau);
        std::copy(&A[0], &A[36], &b_D[0]);
        ::coder::internal::reflapack::xzunghr(1, 6, b_D, tau);
        sgn = ::coder::internal::reflapack::xdlahqr(1, 6, A, 1, 6, b_D, a__4,
                                                    work);
        std::memset(&D[0], 0, 36U * sizeof(creal_T));
        i = static_cast<unsigned char>(sgn);
        for (b_i = 0; b_i < i; b_i++) {
          i1 = b_i + 6 * b_i;
          D[i1].re = rtNaN;
          D[i1].im = 0.0;
        }
        i = sgn + 1;
        for (b_i = i; b_i < 7; b_i++) {
          i1 = (b_i + 6 * (b_i - 1)) - 1;
          D[i1].re = 0.0;
          D[i1].im = work[b_i - 1];
        }
        if (sgn == 0) {
          for (i = 0; i < 36; i++) {
            V[i].re = b_D[i];
            V[i].im = 0.0;
          }
          k = 1;
          do {
            exitg1 = 0;
            if (k <= 6) {
              if (k != 6) {
                i = 6 * (k - 1);
                absx = A[k + i];
                if (absx != 0.0) {
                  if (absx < 0.0) {
                    sgn = 1;
                  } else {
                    sgn = -1;
                  }
                  for (b_i = 0; b_i < 6; b_i++) {
                    i1 = b_i + i;
                    absx = V[i1].re;
                    i2 = b_i + 6 * k;
                    temp2 = static_cast<double>(sgn) * V[i2].re;
                    if (temp2 == 0.0) {
                      V[i1].re = absx / 1.4142135623730951;
                      V[i1].im = 0.0;
                    } else if (absx == 0.0) {
                      V[i1].re = 0.0;
                      V[i1].im = temp2 / 1.4142135623730951;
                    } else {
                      V[i1].re = absx / 1.4142135623730951;
                      V[i1].im = temp2 / 1.4142135623730951;
                    }
                    V[i2].re = V[i1].re;
                    V[i2].im = -V[i1].im;
                  }
                  k += 2;
                } else {
                  k++;
                }
              } else {
                k++;
              }
            } else {
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        } else {
          for (i = 0; i < 36; i++) {
            V[i].re = rtNaN;
            V[i].im = 0.0;
          }
        }
      } else {
        eigStandard(A, V, D);
      }
    }
  }
  for (k = 0; k < 6; k++) {
    work[k] = std::fmax(D[k + 6 * k].re, 2.2204460492503131E-16);
  }
  std::memset(&b_D[0], 0, 36U * sizeof(double));
  for (k = 0; k < 6; k++) {
    b_D[k + 6 * k] = work[k];
  }
  for (i = 0; i < 36; i++) {
    D[i].re = b_D[i];
    D[i].im = 0.0;
  }
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      absx = 0.0;
      temp2 = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        double V_re_tmp;
        sgn = i + 6 * i2;
        temp1 = V[sgn].re;
        iaii = i2 + 6 * i1;
        taui = D[iaii].im;
        anrm = V[sgn].im;
        V_re_tmp = D[iaii].re;
        absx += temp1 * V_re_tmp - anrm * taui;
        temp2 += temp1 * taui + anrm * V_re_tmp;
      }
      i2 = i + 6 * i1;
      b_V[i2].re = absx;
      b_V[i2].im = temp2;
    }
    for (i1 = 0; i1 < 6; i1++) {
      absx = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        sgn = i1 + 6 * i2;
        iaii = i + 6 * i2;
        absx += b_V[iaii].re * V[sgn].re - b_V[iaii].im * -V[sgn].im;
      }
      P[i + 6 * i1] = absx;
    }
  }
}

} // namespace internal
} // namespace fusion
} // namespace coder

//
// File trailer for ensurePosDefMatrix.cpp
//
// [EOF]
//
