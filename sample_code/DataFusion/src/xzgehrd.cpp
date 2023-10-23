//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzgehrd.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "xzgehrd.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <cstring>

// Function Definitions
//
// Arguments    : double a[36]
//                int ilo
//                int ihi
//                double tau[5]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
void xzgehrd(double a[36], int ilo, int ihi, double tau[5])
{
  double work[6];
  if ((ihi - ilo) + 1 > 1) {
    int i;
    i = static_cast<unsigned char>(ilo - 1);
    if (i - 1 >= 0) {
      std::memset(&tau[0], 0, static_cast<unsigned int>(i) * sizeof(double));
    }
    for (int b_i{ihi}; b_i < 6; b_i++) {
      tau[b_i - 1] = 0.0;
    }
    for (int b_i{0}; b_i < 6; b_i++) {
      work[b_i] = 0.0;
    }
    i = ihi - 1;
    for (int b_i{ilo}; b_i <= i; b_i++) {
      double alpha1;
      double d;
      int alpha1_tmp_tmp;
      int c_i;
      int i1;
      int ia;
      int ic0;
      int in;
      int lastc;
      int lastv;
      int n;
      c_i = (b_i - 1) * 6;
      in = b_i * 6;
      alpha1_tmp_tmp = b_i + c_i;
      alpha1 = a[alpha1_tmp_tmp];
      n = ihi - b_i;
      if (b_i + 2 <= 6) {
        i1 = b_i + 1;
      } else {
        i1 = 5;
      }
      d = xzlarfg(n, alpha1, a, (i1 + c_i) + 1);
      tau[b_i - 1] = d;
      a[alpha1_tmp_tmp] = 1.0;
      ic0 = in + 1;
      if (d != 0.0) {
        boolean_T exitg2;
        lastv = n;
        c_i = alpha1_tmp_tmp + n;
        while ((lastv > 0) && (a[c_i - 1] == 0.0)) {
          lastv--;
          c_i--;
        }
        lastc = ihi;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          int exitg1;
          c_i = in + lastc;
          ia = c_i;
          do {
            exitg1 = 0;
            if (ia <= c_i + (lastv - 1) * 6) {
              if (a[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia += 6;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        int i2;
        int jA;
        if (lastc != 0) {
          i1 = static_cast<unsigned char>(lastc);
          std::memset(&work[0], 0,
                      static_cast<unsigned int>(i1) * sizeof(double));
          c_i = alpha1_tmp_tmp;
          i1 = (in + 6 * (lastv - 1)) + 1;
          for (int iac{ic0}; iac <= i1; iac += 6) {
            i2 = (iac + lastc) - 1;
            for (ia = iac; ia <= i2; ia++) {
              jA = ia - iac;
              work[jA] += a[ia - 1] * a[c_i];
            }
            c_i++;
          }
        }
        d = -tau[b_i - 1];
        if (!(d == 0.0)) {
          jA = in;
          i1 = static_cast<unsigned char>(lastv);
          for (int iac{0}; iac < i1; iac++) {
            double temp;
            temp = a[alpha1_tmp_tmp + iac];
            if (temp != 0.0) {
              temp *= d;
              i2 = jA + 1;
              c_i = lastc + jA;
              for (ic0 = i2; ic0 <= c_i; ic0++) {
                a[ic0 - 1] += work[(ic0 - jA) - 1] * temp;
              }
            }
            jA += 6;
          }
        }
      }
      xzlarf(n, 6 - b_i, alpha1_tmp_tmp + 1, tau[b_i - 1], a, (b_i + in) + 1,
             work);
      a[alpha1_tmp_tmp] = alpha1;
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzgehrd.cpp
//
// [EOF]
//
