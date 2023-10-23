//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qrsolve.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "qrsolve.h"
#include "EDGE_fusion_function_231019_2222_rtwutil.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const array<double, 2U> &A
//                const array<double, 1U> &B
//                array<double, 1U> &Y
// Return Type  : void
//
namespace coder {
namespace internal {
void qrsolve(const array<double, 2U> &A, const array<double, 1U> &B,
             array<double, 1U> &Y)
{
  array<double, 2U> b_A;
  array<double, 1U> tau;
  array<double, 1U> vn1;
  array<double, 1U> vn2;
  array<double, 1U> work;
  array<int, 2U> jpvt;
  double smax;
  int b_i;
  int coltop;
  int i;
  int ix;
  int k;
  int m;
  int ma;
  int n;
  int pvt;
  int u1;
  b_A.set_size(A.size(0), A.size(1));
  i = A.size(0) * A.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    b_A[b_i] = A[b_i];
  }
  m = A.size(0);
  n = A.size(1);
  i = A.size(0);
  u1 = A.size(1);
  if (i <= u1) {
    u1 = i;
  }
  tau.set_size(u1);
  for (b_i = 0; b_i < u1; b_i++) {
    tau[b_i] = 0.0;
  }
  jpvt.set_size(1, A.size(1));
  i = A.size(1);
  ma = A.size(0);
  work.set_size(A.size(1));
  vn1.set_size(A.size(1));
  vn2.set_size(A.size(1));
  for (k = 0; k < i; k++) {
    jpvt[k] = k + 1;
    work[k] = 0.0;
    smax = blas::xnrm2(m, A, k * ma + 1);
    vn1[k] = smax;
    vn2[k] = smax;
  }
  for (int c_i{0}; c_i < u1; c_i++) {
    double s;
    double temp2;
    int ii;
    int ip1;
    int mmi;
    int nmi;
    ip1 = c_i + 2;
    coltop = c_i * ma;
    ii = coltop + c_i;
    nmi = n - c_i;
    mmi = m - c_i;
    if (nmi < 1) {
      i = -1;
    } else {
      i = 0;
      if (nmi > 1) {
        smax = std::abs(vn1[c_i]);
        for (k = 2; k <= nmi; k++) {
          s = std::abs(vn1[(c_i + k) - 1]);
          if (s > smax) {
            i = k - 1;
            smax = s;
          }
        }
      }
    }
    pvt = c_i + i;
    if (pvt + 1 != c_i + 1) {
      ix = pvt * ma;
      for (k = 0; k < m; k++) {
        i = ix + k;
        smax = b_A[i];
        b_i = coltop + k;
        b_A[i] = b_A[b_i];
        b_A[b_i] = smax;
      }
      ix = jpvt[pvt];
      jpvt[pvt] = jpvt[c_i];
      jpvt[c_i] = ix;
      vn1[pvt] = vn1[c_i];
      vn2[pvt] = vn2[c_i];
    }
    if (c_i + 1 < m) {
      temp2 = b_A[ii];
      coltop = ii + 2;
      tau[c_i] = 0.0;
      if (mmi > 0) {
        smax = blas::xnrm2(mmi - 1, b_A, ii + 2);
        if (smax != 0.0) {
          s = rt_hypotd_snf(b_A[ii], smax);
          if (b_A[ii] >= 0.0) {
            s = -s;
          }
          if (std::abs(s) < 1.0020841800044864E-292) {
            ix = 0;
            b_i = ii + mmi;
            do {
              ix++;
              for (k = coltop; k <= b_i; k++) {
                b_A[k - 1] = 9.9792015476736E+291 * b_A[k - 1];
              }
              s *= 9.9792015476736E+291;
              temp2 *= 9.9792015476736E+291;
            } while ((std::abs(s) < 1.0020841800044864E-292) && (ix < 20));
            s = rt_hypotd_snf(temp2, blas::xnrm2(mmi - 1, b_A, ii + 2));
            if (temp2 >= 0.0) {
              s = -s;
            }
            tau[c_i] = (s - temp2) / s;
            smax = 1.0 / (temp2 - s);
            for (k = coltop; k <= b_i; k++) {
              b_A[k - 1] = smax * b_A[k - 1];
            }
            for (k = 0; k < ix; k++) {
              s *= 1.0020841800044864E-292;
            }
            temp2 = s;
          } else {
            tau[c_i] = (s - b_A[ii]) / s;
            smax = 1.0 / (b_A[ii] - s);
            b_i = ii + mmi;
            for (k = coltop; k <= b_i; k++) {
              b_A[k - 1] = smax * b_A[k - 1];
            }
            temp2 = s;
          }
        }
      }
      b_A[ii] = temp2;
    } else {
      tau[c_i] = 0.0;
    }
    if (c_i + 1 < n) {
      int jA;
      int lastv;
      temp2 = b_A[ii];
      b_A[ii] = 1.0;
      jA = (ii + ma) + 1;
      if (tau[c_i] != 0.0) {
        boolean_T exitg2;
        lastv = mmi - 1;
        i = (ii + mmi) - 1;
        while ((lastv + 1 > 0) && (b_A[i] == 0.0)) {
          lastv--;
          i--;
        }
        ix = nmi - 2;
        exitg2 = false;
        while ((!exitg2) && (ix + 1 > 0)) {
          int exitg1;
          coltop = jA + ix * ma;
          k = coltop;
          do {
            exitg1 = 0;
            if (k <= coltop + lastv) {
              if (b_A[k - 1] != 0.0) {
                exitg1 = 1;
              } else {
                k++;
              }
            } else {
              ix--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = -1;
        ix = -1;
      }
      if (lastv + 1 > 0) {
        if (ix + 1 != 0) {
          for (coltop = 0; coltop <= ix; coltop++) {
            work[coltop] = 0.0;
          }
          coltop = 0;
          b_i = jA + ma * ix;
          for (pvt = jA; ma < 0 ? pvt >= b_i : pvt <= b_i; pvt += ma) {
            smax = 0.0;
            i = pvt + lastv;
            for (k = pvt; k <= i; k++) {
              smax += b_A[k - 1] * b_A[(ii + k) - pvt];
            }
            work[coltop] = work[coltop] + smax;
            coltop++;
          }
        }
        if (!(-tau[c_i] == 0.0)) {
          for (pvt = 0; pvt <= ix; pvt++) {
            if (work[pvt] != 0.0) {
              smax = work[pvt] * -tau[c_i];
              b_i = lastv + jA;
              for (i = jA; i <= b_i; i++) {
                b_A[i - 1] = b_A[i - 1] + b_A[(ii + i) - jA] * smax;
              }
            }
            jA += ma;
          }
        }
      }
      b_A[ii] = temp2;
    }
    for (pvt = ip1; pvt <= n; pvt++) {
      i = c_i + (pvt - 1) * ma;
      smax = vn1[pvt - 1];
      if (smax != 0.0) {
        s = std::abs(b_A[i]) / smax;
        s = 1.0 - s * s;
        if (s < 0.0) {
          s = 0.0;
        }
        temp2 = smax / vn2[pvt - 1];
        temp2 = s * (temp2 * temp2);
        if (temp2 <= 1.4901161193847656E-8) {
          if (c_i + 1 < m) {
            smax = blas::xnrm2(mmi - 1, b_A, i + 2);
            vn1[pvt - 1] = smax;
            vn2[pvt - 1] = smax;
          } else {
            vn1[pvt - 1] = 0.0;
            vn2[pvt - 1] = 0.0;
          }
        } else {
          vn1[pvt - 1] = smax * std::sqrt(s);
        }
      }
    }
  }
  coltop = 0;
  if (b_A.size(0) < b_A.size(1)) {
    i = b_A.size(0);
    ix = b_A.size(1);
  } else {
    i = b_A.size(1);
    ix = b_A.size(0);
  }
  smax = std::fmin(1.4901161193847656E-8,
                   2.2204460492503131E-15 * static_cast<double>(ix)) *
         std::abs(b_A[0]);
  while ((coltop < i) &&
         (!(std::abs(b_A[coltop + b_A.size(0) * coltop]) <= smax))) {
    coltop++;
  }
  work.set_size(B.size(0));
  i = B.size(0);
  for (b_i = 0; b_i < i; b_i++) {
    work[b_i] = B[b_i];
  }
  Y.set_size(b_A.size(1));
  i = b_A.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    Y[b_i] = 0.0;
  }
  m = b_A.size(0);
  for (pvt = 0; pvt < u1; pvt++) {
    if (tau[pvt] != 0.0) {
      smax = work[pvt];
      b_i = pvt + 2;
      for (int c_i{b_i}; c_i <= m; c_i++) {
        smax += b_A[(c_i + b_A.size(0) * pvt) - 1] * work[c_i - 1];
      }
      smax *= tau[pvt];
      if (smax != 0.0) {
        work[pvt] = work[pvt] - smax;
        for (int c_i{b_i}; c_i <= m; c_i++) {
          work[c_i - 1] =
              work[c_i - 1] - b_A[(c_i + b_A.size(0) * pvt) - 1] * smax;
        }
      }
    }
  }
  for (int c_i{0}; c_i < coltop; c_i++) {
    Y[jpvt[c_i] - 1] = work[c_i];
  }
  for (pvt = coltop; pvt >= 1; pvt--) {
    b_i = jpvt[pvt - 1];
    Y[b_i - 1] = Y[b_i - 1] / b_A[(pvt + b_A.size(0) * (pvt - 1)) - 1];
    for (int c_i{0}; c_i <= pvt - 2; c_i++) {
      Y[jpvt[c_i] - 1] =
          Y[jpvt[c_i] - 1] - Y[b_i - 1] * b_A[c_i + b_A.size(0) * (pvt - 1)];
    }
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for qrsolve.cpp
//
// [EOF]
//
