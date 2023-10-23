//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzsteqr.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "xzsteqr.h"
#include "EDGE_fusion_function_231019_2222_rtwutil.h"
#include "rt_nonfinite.h"
#include "xdlaev2.h"
#include "xzlartg.h"
#include "xzlascl.h"
#include <cmath>
#include <cstring>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
static void b_rotateRight(int n, double z[36], int iz0, const double cs[10],
                          int ic0, int is0);

static void rotateRight(int n, double z[36], int iz0, const double cs[10],
                        int ic0, int is0);

} // namespace reflapack
} // namespace internal
} // namespace coder

// Function Definitions
//
// Arguments    : int n
//                double z[36]
//                int iz0
//                const double cs[10]
//                int ic0
//                int is0
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
static void b_rotateRight(int n, double z[36], int iz0, const double cs[10],
                          int ic0, int is0)
{
  for (int j{0}; j <= n - 2; j++) {
    double ctemp;
    double stemp;
    int offsetj;
    int offsetjp1;
    ctemp = cs[(ic0 + j) - 1];
    stemp = cs[(is0 + j) - 1];
    offsetj = (j * 6 + iz0) - 2;
    offsetjp1 = ((j + 1) * 6 + iz0) - 2;
    if ((ctemp != 1.0) || (stemp != 0.0)) {
      for (int i{0}; i < 6; i++) {
        double temp;
        int b_i;
        int temp_tmp;
        temp_tmp = (offsetjp1 + i) + 1;
        temp = z[temp_tmp];
        b_i = (offsetj + i) + 1;
        z[temp_tmp] = ctemp * temp - stemp * z[b_i];
        z[b_i] = stemp * temp + ctemp * z[b_i];
      }
    }
  }
}

//
// Arguments    : int n
//                double z[36]
//                int iz0
//                const double cs[10]
//                int ic0
//                int is0
// Return Type  : void
//
static void rotateRight(int n, double z[36], int iz0, const double cs[10],
                        int ic0, int is0)
{
  int i;
  i = n - 1;
  for (int j{i}; j >= 1; j--) {
    double ctemp;
    double stemp;
    int offsetj;
    int offsetjp1;
    ctemp = cs[(ic0 + j) - 2];
    stemp = cs[(is0 + j) - 2];
    offsetj = ((j - 1) * 6 + iz0) - 2;
    offsetjp1 = (j * 6 + iz0) - 2;
    if ((ctemp != 1.0) || (stemp != 0.0)) {
      for (int b_i{0}; b_i < 6; b_i++) {
        double temp;
        int i1;
        int temp_tmp;
        temp_tmp = (offsetjp1 + b_i) + 1;
        temp = z[temp_tmp];
        i1 = (offsetj + b_i) + 1;
        z[temp_tmp] = ctemp * temp - stemp * z[i1];
        z[i1] = stemp * temp + ctemp * z[i1];
      }
    }
  }
}

//
// Arguments    : double d[6]
//                double e[5]
//                double z[36]
// Return Type  : int
//
int xzsteqr(double d[6], double e[5], double z[36])
{
  double work[10];
  double c;
  double r;
  double s;
  double tst;
  int info;
  int jtot;
  int l1;
  info = 0;
  std::memset(&work[0], 0, 10U * sizeof(double));
  jtot = 0;
  l1 = 1;
  int exitg1;
  do {
    exitg1 = 0;
    if (l1 > 6) {
      for (l1 = 0; l1 < 5; l1++) {
        double p;
        int iscale;
        int m;
        m = l1;
        p = d[l1];
        for (iscale = l1 + 2; iscale < 7; iscale++) {
          c = d[iscale - 1];
          if (c < p) {
            m = iscale - 1;
            p = c;
          }
        }
        if (m != l1) {
          int iy;
          d[m] = d[l1];
          d[l1] = p;
          iscale = l1 * 6;
          iy = m * 6;
          for (m = 0; m < 6; m++) {
            int b_i;
            jtot = iscale + m;
            tst = z[jtot];
            b_i = iy + m;
            z[jtot] = z[b_i];
            z[b_i] = tst;
          }
        }
      }
      exitg1 = 1;
    } else {
      int l;
      int lend;
      int lendsv;
      int lsv;
      int m;
      boolean_T exitg2;
      if (l1 > 1) {
        e[l1 - 2] = 0.0;
      }
      m = l1;
      exitg2 = false;
      while ((!exitg2) && (m < 6)) {
        tst = std::abs(e[m - 1]);
        if (tst == 0.0) {
          exitg2 = true;
        } else if (tst <= std::sqrt(std::abs(d[m - 1])) *
                              std::sqrt(std::abs(d[m])) *
                              2.2204460492503131E-16) {
          e[m - 1] = 0.0;
          exitg2 = true;
        } else {
          m++;
        }
      }
      l = l1 - 1;
      lsv = l1;
      lend = m;
      lendsv = m;
      l1 = m + 1;
      if (m != l + 1) {
        double anorm;
        int i;
        int iscale;
        int iy;
        iy = m - l;
        if (iy <= 0) {
          anorm = 0.0;
        } else {
          anorm = std::abs(d[(l + iy) - 1]);
          i = 0;
          exitg2 = false;
          while ((!exitg2) && (i <= iy - 2)) {
            iscale = l + i;
            tst = std::abs(d[iscale]);
            if (std::isnan(tst)) {
              anorm = rtNaN;
              exitg2 = true;
            } else {
              if (tst > anorm) {
                anorm = tst;
              }
              tst = std::abs(e[iscale]);
              if (std::isnan(tst)) {
                anorm = rtNaN;
                exitg2 = true;
              } else {
                if (tst > anorm) {
                  anorm = tst;
                }
                i++;
              }
            }
          }
        }
        iscale = 0;
        if (!(anorm == 0.0)) {
          if (std::isinf(anorm) || std::isnan(anorm)) {
            for (i = 0; i < 6; i++) {
              d[i] = rtNaN;
            }
            for (int b_i{0}; b_i < 36; b_i++) {
              z[b_i] = rtNaN;
            }
            exitg1 = 1;
          } else {
            int b_i;
            if (anorm > 2.2346346549904327E+153) {
              iscale = 1;
              xzlascl(anorm, 2.2346346549904327E+153, iy, d, l + 1);
              b_xzlascl(anorm, 2.2346346549904327E+153, iy - 1, e, l + 1);
            } else if (anorm < 3.02546243347603E-123) {
              iscale = 2;
              xzlascl(anorm, 3.02546243347603E-123, iy, d, l + 1);
              b_xzlascl(anorm, 3.02546243347603E-123, iy - 1, e, l + 1);
            }
            if (std::abs(d[m - 1]) < std::abs(d[l])) {
              lend = lsv;
              l = m - 1;
            }
            if (lend > l + 1) {
              int exitg4;
              do {
                exitg4 = 0;
                if (l + 1 != lend) {
                  m = l + 1;
                  exitg2 = false;
                  while ((!exitg2) && (m < lend)) {
                    tst = std::abs(e[m - 1]);
                    if (tst * tst <= 4.9303806576313238E-32 *
                                             std::abs(d[m - 1]) *
                                             std::abs(d[m]) +
                                         2.2250738585072014E-308) {
                      exitg2 = true;
                    } else {
                      m++;
                    }
                  }
                } else {
                  m = lend;
                }
                if (m < lend) {
                  e[m - 1] = 0.0;
                }
                if (m == l + 1) {
                  l++;
                  if (l + 1 > lend) {
                    exitg4 = 1;
                  }
                } else if (m == l + 2) {
                  d[l] = xdlaev2(d[l], e[l], d[l + 1], c, work[l], tst);
                  d[l + 1] = c;
                  work[l + 5] = tst;
                  rotateRight(2, z, l * 6 + 1, work, l + 1, l + 6);
                  e[l] = 0.0;
                  l += 2;
                  if (l + 1 > lend) {
                    exitg4 = 1;
                  }
                } else if (jtot == 180) {
                  exitg4 = 1;
                } else {
                  double g;
                  double p;
                  jtot++;
                  g = (d[l + 1] - d[l]) / (2.0 * e[l]);
                  c = rt_hypotd_snf(g, 1.0);
                  if (!(g >= 0.0)) {
                    c = -c;
                  }
                  g = (d[m - 1] - d[l]) + e[l] / (g + c);
                  tst = 1.0;
                  c = 1.0;
                  p = 0.0;
                  b_i = m - 1;
                  for (i = b_i; i >= l + 1; i--) {
                    double b;
                    double b_tmp;
                    b_tmp = e[i - 1];
                    b = c * b_tmp;
                    c = xzlartg(g, tst * b_tmp, s, r);
                    tst = s;
                    if (i != m - 1) {
                      e[i] = r;
                    }
                    g = d[i] - p;
                    r = (d[i - 1] - g) * s + 2.0 * c * b;
                    p = s * r;
                    d[i] = g + p;
                    g = c * r - b;
                    work[i - 1] = c;
                    work[i + 4] = -s;
                  }
                  rotateRight(m - l, z, l * 6 + 1, work, l + 1, l + 6);
                  d[l] -= p;
                  e[l] = g;
                }
              } while (exitg4 == 0);
            } else {
              int exitg3;
              do {
                exitg3 = 0;
                if (l + 1 != lend) {
                  m = l + 1;
                  exitg2 = false;
                  while ((!exitg2) && (m > lend)) {
                    tst = std::abs(e[m - 2]);
                    if (tst * tst <= 4.9303806576313238E-32 *
                                             std::abs(d[m - 1]) *
                                             std::abs(d[m - 2]) +
                                         2.2250738585072014E-308) {
                      exitg2 = true;
                    } else {
                      m--;
                    }
                  }
                } else {
                  m = lend;
                }
                if (m > lend) {
                  e[m - 2] = 0.0;
                }
                if (m == l + 1) {
                  l--;
                  if (l + 1 < lend) {
                    exitg3 = 1;
                  }
                } else if (m == l) {
                  d[l - 1] =
                      xdlaev2(d[l - 1], e[l - 1], d[l], c, work[m - 1], tst);
                  d[l] = c;
                  work[m + 4] = tst;
                  b_rotateRight(2, z, (l - 1) * 6 + 1, work, m, m + 5);
                  e[l - 1] = 0.0;
                  l -= 2;
                  if (l + 1 < lend) {
                    exitg3 = 1;
                  }
                } else if (jtot == 180) {
                  exitg3 = 1;
                } else {
                  double g;
                  double p;
                  jtot++;
                  tst = e[l - 1];
                  g = (d[l - 1] - d[l]) / (2.0 * tst);
                  c = rt_hypotd_snf(g, 1.0);
                  if (!(g >= 0.0)) {
                    c = -c;
                  }
                  g = (d[m - 1] - d[l]) + tst / (g + c);
                  tst = 1.0;
                  c = 1.0;
                  p = 0.0;
                  for (i = m; i <= l; i++) {
                    double b;
                    double b_tmp;
                    b_tmp = e[i - 1];
                    b = c * b_tmp;
                    c = xzlartg(g, tst * b_tmp, s, r);
                    tst = s;
                    if (i != m) {
                      e[i - 2] = r;
                    }
                    g = d[i - 1] - p;
                    r = (d[i] - g) * s + 2.0 * c * b;
                    p = s * r;
                    d[i - 1] = g + p;
                    g = c * r - b;
                    work[i - 1] = c;
                    work[i + 4] = s;
                  }
                  b_rotateRight((l - m) + 2, z, (m - 1) * 6 + 1, work, m,
                                m + 5);
                  d[l] -= p;
                  e[l - 1] = g;
                }
              } while (exitg3 == 0);
            }
            if (iscale == 1) {
              b_i = lendsv - lsv;
              xzlascl(2.2346346549904327E+153, anorm, b_i + 1, d, lsv);
              b_xzlascl(2.2346346549904327E+153, anorm, b_i, e, lsv);
            } else if (iscale == 2) {
              b_i = lendsv - lsv;
              xzlascl(3.02546243347603E-123, anorm, b_i + 1, d, lsv);
              b_xzlascl(3.02546243347603E-123, anorm, b_i, e, lsv);
            }
            if (jtot >= 180) {
              for (i = 0; i < 5; i++) {
                if (e[i] != 0.0) {
                  info++;
                }
              }
              exitg1 = 1;
            }
          }
        }
      }
    }
  } while (exitg1 == 0);
  return info;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzsteqr.cpp
//
// [EOF]
//
