//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdlahqr.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "xdlahqr.h"
#include "rt_nonfinite.h"
#include "xdlanv2.h"
#include "xzlarfg.h"
#include <cmath>

// Function Definitions
//
// Arguments    : int ilo
//                int ihi
//                double h[36]
//                int iloz
//                int ihiz
//                double z[36]
//                double wr[6]
//                double wi[6]
// Return Type  : int
//
namespace coder {
namespace internal {
namespace reflapack {
int xdlahqr(int ilo, int ihi, double h[36], int iloz, int ihiz, double z[36],
            double wr[6], double wi[6])
{
  double aa;
  double h22;
  double rt1r;
  double s;
  int b_i;
  int i;
  int info;
  info = 0;
  i = static_cast<unsigned char>(ilo - 1);
  for (b_i = 0; b_i < i; b_i++) {
    wr[b_i] = h[b_i + 6 * b_i];
    wi[b_i] = 0.0;
  }
  i = ihi + 1;
  for (b_i = i; b_i < 7; b_i++) {
    wr[b_i - 1] = h[(b_i + 6 * (b_i - 1)) - 1];
    wi[b_i - 1] = 0.0;
  }
  if (ilo == ihi) {
    wr[ilo - 1] = h[(ilo + 6 * (ilo - 1)) - 1];
    wi[ilo - 1] = 0.0;
  } else {
    double smlnum;
    int i1;
    int kdefl;
    int nr;
    int nz;
    boolean_T exitg1;
    i = ihi - 3;
    for (nr = ilo; nr <= i; nr++) {
      i1 = nr + 6 * (nr - 1);
      h[i1 + 1] = 0.0;
      h[i1 + 2] = 0.0;
    }
    if (ilo <= ihi - 2) {
      h[(ihi + 6 * (ihi - 3)) - 1] = 0.0;
    }
    nz = (ihiz - iloz) + 1;
    smlnum = 2.2250738585072014E-308 *
             (static_cast<double>((ihi - ilo) + 1) / 2.2204460492503131E-16);
    kdefl = 0;
    b_i = ihi - 1;
    exitg1 = false;
    while ((!exitg1) && (b_i + 1 >= ilo)) {
      double d;
      double h21;
      double rt2r;
      double s_tmp_tmp;
      double tst;
      int b_k;
      int its;
      int ix;
      int k;
      int l;
      int temp_tmp_tmp;
      boolean_T converged;
      boolean_T exitg2;
      l = ilo;
      converged = false;
      its = 0;
      exitg2 = false;
      while ((!exitg2) && (its < 301)) {
        double tr;
        boolean_T exitg3;
        k = b_i;
        exitg3 = false;
        while ((!exitg3) && (k + 1 > l)) {
          i = k + 6 * (k - 1);
          d = std::abs(h[i]);
          if (d <= smlnum) {
            exitg3 = true;
          } else {
            ix = k + 6 * k;
            h21 = h[ix];
            tr = std::abs(h21);
            aa = h[i - 1];
            tst = std::abs(aa) + tr;
            if (tst == 0.0) {
              if (k - 1 >= ilo) {
                tst = std::abs(h[(k + 6 * (k - 2)) - 1]);
              }
              if (k + 2 <= ihi) {
                tst += std::abs(h[ix + 1]);
              }
            }
            if (d <= 2.2204460492503131E-16 * tst) {
              h22 = std::abs(h[ix - 1]);
              h21 = std::abs(aa - h21);
              aa = std::fmax(tr, h21);
              tst = std::fmin(tr, h21);
              s = aa + tst;
              if (std::fmin(d, h22) * (std::fmax(d, h22) / s) <=
                  std::fmax(smlnum,
                            2.2204460492503131E-16 * (tst * (aa / s)))) {
                exitg3 = true;
              } else {
                k--;
              }
            } else {
              k--;
            }
          }
        }
        l = k + 1;
        if (k + 1 > ilo) {
          h[k + 6 * (k - 1)] = 0.0;
        }
        if (k + 1 >= b_i) {
          converged = true;
          exitg2 = true;
        } else {
          double v[3];
          int m;
          kdefl++;
          if (kdefl - kdefl / 20 * 20 == 0) {
            s = std::abs(h[b_i + 6 * (b_i - 1)]) +
                std::abs(h[(b_i + 6 * (b_i - 2)) - 1]);
            tst = 0.75 * s + h[b_i + 6 * b_i];
            aa = -0.4375 * s;
            h21 = s;
            h22 = tst;
          } else if (kdefl - kdefl / 10 * 10 == 0) {
            ix = k + 6 * k;
            s = std::abs(h[ix + 1]) + std::abs(h[(k + 6 * (k + 1)) + 2]);
            tst = 0.75 * s + h[ix];
            aa = -0.4375 * s;
            h21 = s;
            h22 = tst;
          } else {
            ix = b_i + 6 * (b_i - 1);
            tst = h[ix - 1];
            h21 = h[ix];
            ix = b_i + 6 * b_i;
            aa = h[ix - 1];
            h22 = h[ix];
          }
          s = ((std::abs(tst) + std::abs(aa)) + std::abs(h21)) + std::abs(h22);
          if (s == 0.0) {
            rt1r = 0.0;
            tr = 0.0;
            rt2r = 0.0;
            h22 = 0.0;
          } else {
            tst /= s;
            h21 /= s;
            aa /= s;
            h22 /= s;
            tr = (tst + h22) / 2.0;
            tst = (tst - tr) * (h22 - tr) - aa * h21;
            h21 = std::sqrt(std::abs(tst));
            if (tst >= 0.0) {
              rt1r = tr * s;
              rt2r = rt1r;
              tr = h21 * s;
              h22 = -tr;
            } else {
              rt1r = tr + h21;
              rt2r = tr - h21;
              if (std::abs(rt1r - h22) <= std::abs(rt2r - h22)) {
                rt1r *= s;
                rt2r = rt1r;
              } else {
                rt2r *= s;
                rt1r = rt2r;
              }
              tr = 0.0;
              h22 = 0.0;
            }
          }
          m = b_i - 1;
          exitg3 = false;
          while ((!exitg3) && (m >= k + 1)) {
            ix = m + 6 * (m - 1);
            tst = h[ix];
            s_tmp_tmp = h[ix - 1];
            h21 = s_tmp_tmp - rt2r;
            s = (std::abs(h21) + std::abs(h22)) + std::abs(tst);
            aa = tst / s;
            ix = m + 6 * m;
            v[0] = (aa * h[ix - 1] + h21 * (h21 / s)) - tr * (h22 / s);
            tst = h[ix];
            v[1] = aa * (((s_tmp_tmp + tst) - rt1r) - rt2r);
            v[2] = aa * h[ix + 1];
            s = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
            v[0] /= s;
            v[1] /= s;
            v[2] /= s;
            if (m == k + 1) {
              exitg3 = true;
            } else {
              i = m + 6 * (m - 2);
              if (std::abs(h[i - 1]) * (std::abs(v[1]) + std::abs(v[2])) <=
                  2.2204460492503131E-16 * std::abs(v[0]) *
                      ((std::abs(h[i - 2]) + std::abs(s_tmp_tmp)) +
                       std::abs(tst))) {
                exitg3 = true;
              } else {
                m--;
              }
            }
          }
          for (int c_k{m}; c_k <= b_i; c_k++) {
            ix = (b_i - c_k) + 2;
            if (ix >= 3) {
              nr = 3;
            } else {
              nr = ix;
            }
            if (c_k > m) {
              ix = ((c_k - 2) * 6 + c_k) - 1;
              for (b_k = 0; b_k < nr; b_k++) {
                v[b_k] = h[ix + b_k];
              }
            }
            tst = v[0];
            tr = xzlarfg(nr, tst, v);
            if (c_k > m) {
              i = c_k + 6 * (c_k - 2);
              h[i - 1] = tst;
              h[i] = 0.0;
              if (c_k < b_i) {
                h[i + 1] = 0.0;
              }
            } else if (m > k + 1) {
              i = (c_k + 6 * (c_k - 2)) - 1;
              h[i] *= 1.0 - tr;
            }
            d = v[1];
            tst = tr * v[1];
            if (nr == 3) {
              s_tmp_tmp = v[2];
              aa = tr * v[2];
              for (nr = c_k; nr < 7; nr++) {
                i = c_k + 6 * (nr - 1);
                rt2r = h[i - 1];
                rt1r = h[i];
                s = h[i + 1];
                h21 = (rt2r + d * rt1r) + s_tmp_tmp * s;
                rt2r -= h21 * tr;
                h[i - 1] = rt2r;
                rt1r -= h21 * tst;
                h[i] = rt1r;
                s -= h21 * aa;
                h[i + 1] = s;
              }
              if (c_k + 3 <= b_i + 1) {
                i = c_k;
              } else {
                i = b_i - 2;
              }
              i = static_cast<unsigned char>(i + 3);
              for (nr = 0; nr < i; nr++) {
                i1 = nr + 6 * (c_k - 1);
                rt2r = h[i1];
                ix = nr + 6 * c_k;
                rt1r = h[ix];
                temp_tmp_tmp = nr + 6 * (c_k + 1);
                s = h[temp_tmp_tmp];
                h21 = (rt2r + d * rt1r) + s_tmp_tmp * s;
                rt2r -= h21 * tr;
                h[i1] = rt2r;
                rt1r -= h21 * tst;
                h[ix] = rt1r;
                s -= h21 * aa;
                h[temp_tmp_tmp] = s;
              }
              for (nr = iloz; nr <= ihiz; nr++) {
                i = (nr + 6 * (c_k - 1)) - 1;
                rt2r = z[i];
                i1 = (nr + 6 * c_k) - 1;
                rt1r = z[i1];
                ix = (nr + 6 * (c_k + 1)) - 1;
                s = z[ix];
                h21 = (rt2r + d * rt1r) + s_tmp_tmp * s;
                rt2r -= h21 * tr;
                z[i] = rt2r;
                rt1r -= h21 * tst;
                z[i1] = rt1r;
                s -= h21 * aa;
                z[ix] = s;
              }
            } else if (nr == 2) {
              for (nr = c_k; nr < 7; nr++) {
                i = c_k + 6 * (nr - 1);
                s_tmp_tmp = h[i - 1];
                rt2r = h[i];
                h21 = s_tmp_tmp + d * rt2r;
                s_tmp_tmp -= h21 * tr;
                h[i - 1] = s_tmp_tmp;
                rt2r -= h21 * tst;
                h[i] = rt2r;
              }
              i = static_cast<unsigned char>(b_i + 1);
              for (nr = 0; nr < i; nr++) {
                i1 = nr + 6 * (c_k - 1);
                s_tmp_tmp = h[i1];
                ix = nr + 6 * c_k;
                rt2r = h[ix];
                h21 = s_tmp_tmp + d * rt2r;
                s_tmp_tmp -= h21 * tr;
                h[i1] = s_tmp_tmp;
                rt2r -= h21 * tst;
                h[ix] = rt2r;
              }
              for (nr = iloz; nr <= ihiz; nr++) {
                i = (nr + 6 * (c_k - 1)) - 1;
                s_tmp_tmp = z[i];
                i1 = (nr + 6 * c_k) - 1;
                rt2r = z[i1];
                h21 = s_tmp_tmp + d * rt2r;
                s_tmp_tmp -= h21 * tr;
                z[i] = s_tmp_tmp;
                rt2r -= h21 * tst;
                z[i1] = rt2r;
              }
            }
          }
          its++;
        }
      }
      if (!converged) {
        info = b_i + 1;
        exitg1 = true;
      } else {
        if (l == b_i + 1) {
          wr[b_i] = h[b_i + 6 * b_i];
          wi[b_i] = 0.0;
        } else if (l == b_i) {
          i = b_i + 6 * b_i;
          d = h[i - 1];
          i1 = 6 * (b_i - 1);
          ix = b_i + i1;
          s_tmp_tmp = h[ix];
          rt2r = h[i];
          wr[b_i - 1] = xdlanv2(h[ix - 1], d, s_tmp_tmp, rt2r, wi[b_i - 1],
                                rt1r, s, aa, h22);
          wr[b_i] = rt1r;
          wi[b_i] = s;
          h[i - 1] = d;
          h[ix] = s_tmp_tmp;
          h[i] = rt2r;
          if (b_i + 1 < 6) {
            ix = (b_i + 1) * 6 + b_i;
            i = static_cast<unsigned char>(5 - b_i);
            for (k = 0; k < i; k++) {
              nr = ix + k * 6;
              tst = h[nr];
              h21 = h[nr - 1];
              h[nr] = aa * tst - h22 * h21;
              h[nr - 1] = aa * h21 + h22 * tst;
            }
          }
          if (b_i - 1 >= 1) {
            nr = b_i * 6;
            i = static_cast<unsigned char>(b_i - 1);
            for (k = 0; k < i; k++) {
              b_k = nr + k;
              tst = h[b_k];
              temp_tmp_tmp = i1 + k;
              h21 = h[temp_tmp_tmp];
              h[b_k] = aa * tst - h22 * h21;
              h[temp_tmp_tmp] = aa * h21 + h22 * tst;
            }
          }
          if (nz >= 1) {
            ix = (i1 + iloz) - 1;
            nr = (b_i * 6 + iloz) - 1;
            i = static_cast<unsigned char>(nz);
            for (k = 0; k < i; k++) {
              b_k = nr + k;
              tst = z[b_k];
              temp_tmp_tmp = ix + k;
              h21 = z[temp_tmp_tmp];
              z[b_k] = aa * tst - h22 * h21;
              z[temp_tmp_tmp] = aa * h21 + h22 * tst;
            }
          }
        }
        kdefl = 0;
        b_i = l - 2;
      }
    }
    for (nr = 0; nr < 4; nr++) {
      for (b_i = nr + 3; b_i < 7; b_i++) {
        h[(b_i + 6 * nr) - 1] = 0.0;
      }
    }
  }
  return info;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xdlahqr.cpp
//
// [EOF]
//
