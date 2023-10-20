//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sort.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "sort.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : array<unsigned int, 1U> &x
// Return Type  : void
//
namespace coder {
namespace internal {
void sort(array<unsigned int, 1U> &x)
{
  array<int, 1U> iidx;
  array<int, 1U> iwork;
  array<unsigned int, 1U> vwork;
  array<unsigned int, 1U> xwork;
  int dim;
  int i;
  int vlen;
  int vstride;
  dim = 0;
  if (x.size(0) != 1) {
    dim = -1;
  }
  if (dim + 2 <= 1) {
    i = x.size(0);
  } else {
    i = 1;
  }
  vlen = i - 1;
  vwork.set_size(i);
  vstride = 1;
  for (int k{0}; k <= dim; k++) {
    vstride *= x.size(0);
  }
  for (int b_i{0}; b_i < 1; b_i++) {
    for (int j{0}; j < vstride; j++) {
      for (int k{0}; k <= vlen; k++) {
        vwork[k] = x[j + k * vstride];
      }
      dim = vwork.size(0);
      iidx.set_size(vwork.size(0));
      for (i = 0; i < dim; i++) {
        iidx[i] = 0;
      }
      dim = vwork.size(0);
      if (vwork.size(0) != 0) {
        int idx4[4];
        unsigned int x4[4];
        int c_i;
        int i1;
        int i2;
        int i3;
        int i4;
        int offset;
        x4[0] = 0U;
        idx4[0] = 0;
        x4[1] = 0U;
        idx4[1] = 0;
        x4[2] = 0U;
        idx4[2] = 0;
        x4[3] = 0U;
        idx4[3] = 0;
        iwork.set_size(vwork.size(0));
        xwork.set_size(vwork.size(0));
        for (i = 0; i < dim; i++) {
          iwork[i] = 0;
          xwork[i] = 0U;
        }
        dim = vwork.size(0) >> 2;
        for (int b_j{0}; b_j < dim; b_j++) {
          unsigned int b_x4_tmp;
          unsigned int c_x4_tmp;
          unsigned int x4_tmp;
          c_i = b_j << 2;
          idx4[0] = c_i + 1;
          idx4[1] = c_i + 2;
          idx4[2] = c_i + 3;
          idx4[3] = c_i + 4;
          x4[0] = vwork[c_i];
          x4_tmp = vwork[c_i + 1];
          x4[1] = x4_tmp;
          b_x4_tmp = vwork[c_i + 2];
          x4[2] = b_x4_tmp;
          c_x4_tmp = vwork[c_i + 3];
          x4[3] = c_x4_tmp;
          if (vwork[c_i] <= x4_tmp) {
            i1 = 1;
            i2 = 2;
          } else {
            i1 = 2;
            i2 = 1;
          }
          if (b_x4_tmp <= c_x4_tmp) {
            i3 = 3;
            i4 = 4;
          } else {
            i3 = 4;
            i4 = 3;
          }
          x4_tmp = x4[i3 - 1];
          b_x4_tmp = x4[i1 - 1];
          if (b_x4_tmp <= x4_tmp) {
            b_x4_tmp = x4[i2 - 1];
            if (b_x4_tmp <= x4_tmp) {
              i = i1;
              offset = i2;
              i1 = i3;
              i2 = i4;
            } else if (b_x4_tmp <= x4[i4 - 1]) {
              i = i1;
              offset = i3;
              i1 = i2;
              i2 = i4;
            } else {
              i = i1;
              offset = i3;
              i1 = i4;
            }
          } else {
            x4_tmp = x4[i4 - 1];
            if (b_x4_tmp <= x4_tmp) {
              if (x4[i2 - 1] <= x4_tmp) {
                i = i3;
                offset = i1;
                i1 = i2;
                i2 = i4;
              } else {
                i = i3;
                offset = i1;
                i1 = i4;
              }
            } else {
              i = i3;
              offset = i4;
            }
          }
          iidx[c_i] = idx4[i - 1];
          iidx[c_i + 1] = idx4[offset - 1];
          iidx[c_i + 2] = idx4[i1 - 1];
          iidx[c_i + 3] = idx4[i2 - 1];
          vwork[c_i] = x4[i - 1];
          vwork[c_i + 1] = x4[offset - 1];
          vwork[c_i + 2] = x4[i1 - 1];
          vwork[c_i + 3] = x4[i2 - 1];
        }
        c_i = dim << 2;
        i1 = vwork.size(0) - c_i;
        if (i1 > 0) {
          signed char perm[4];
          for (int k{0}; k < i1; k++) {
            dim = c_i + k;
            idx4[k] = dim + 1;
            x4[k] = vwork[dim];
          }
          perm[1] = 0;
          perm[2] = 0;
          perm[3] = 0;
          if (i1 == 1) {
            perm[0] = 1;
          } else if (i1 == 2) {
            if (x4[0] <= x4[1]) {
              perm[0] = 1;
              perm[1] = 2;
            } else {
              perm[0] = 2;
              perm[1] = 1;
            }
          } else if (x4[0] <= x4[1]) {
            if (x4[1] <= x4[2]) {
              perm[0] = 1;
              perm[1] = 2;
              perm[2] = 3;
            } else if (x4[0] <= x4[2]) {
              perm[0] = 1;
              perm[1] = 3;
              perm[2] = 2;
            } else {
              perm[0] = 3;
              perm[1] = 1;
              perm[2] = 2;
            }
          } else if (x4[0] <= x4[2]) {
            perm[0] = 2;
            perm[1] = 1;
            perm[2] = 3;
          } else if (x4[1] <= x4[2]) {
            perm[0] = 2;
            perm[1] = 3;
            perm[2] = 1;
          } else {
            perm[0] = 3;
            perm[1] = 2;
            perm[2] = 1;
          }
          for (int k{0}; k < i1; k++) {
            i2 = c_i + k;
            i = perm[k];
            iidx[i2] = idx4[i - 1];
            vwork[i2] = x4[i - 1];
          }
        }
        dim = 2;
        if (vwork.size(0) > 1) {
          if (vwork.size(0) >= 256) {
            i4 = vwork.size(0) >> 8;
            for (int b{0}; b < i4; b++) {
              int b_iwork[256];
              unsigned int b_xwork[256];
              offset = (b << 8) - 1;
              for (int b_b{0}; b_b < 6; b_b++) {
                int bLen;
                int bLen2;
                bLen = 1 << (b_b + 2);
                bLen2 = bLen << 1;
                i = 256 >> (b_b + 3);
                for (int k{0}; k < i; k++) {
                  i1 = (offset + k * bLen2) + 1;
                  for (int b_j{0}; b_j < bLen2; b_j++) {
                    dim = i1 + b_j;
                    b_iwork[b_j] = iidx[dim];
                    b_xwork[b_j] = vwork[dim];
                  }
                  i3 = 0;
                  c_i = bLen;
                  dim = i1 - 1;
                  int exitg1;
                  do {
                    exitg1 = 0;
                    dim++;
                    if (b_xwork[i3] <= b_xwork[c_i]) {
                      iidx[dim] = b_iwork[i3];
                      vwork[dim] = b_xwork[i3];
                      if (i3 + 1 < bLen) {
                        i3++;
                      } else {
                        exitg1 = 1;
                      }
                    } else {
                      iidx[dim] = b_iwork[c_i];
                      vwork[dim] = b_xwork[c_i];
                      if (c_i + 1 < bLen2) {
                        c_i++;
                      } else {
                        dim -= i3;
                        for (int b_j{i3 + 1}; b_j <= bLen; b_j++) {
                          i2 = dim + b_j;
                          iidx[i2] = b_iwork[b_j - 1];
                          vwork[i2] = b_xwork[b_j - 1];
                        }
                        exitg1 = 1;
                      }
                    }
                  } while (exitg1 == 0);
                }
              }
            }
            dim = i4 << 8;
            c_i = vwork.size(0) - dim;
            if (c_i > 0) {
              merge_block(iidx, vwork, dim, c_i, 2, iwork, xwork);
            }
            dim = 8;
          }
          merge_block(iidx, vwork, 0, vwork.size(0), dim, iwork, xwork);
        }
      }
      for (int k{0}; k <= vlen; k++) {
        x[j + k * vstride] = vwork[k];
      }
    }
  }
}

//
// Arguments    : array<double, 2U> &x
//                array<int, 2U> &idx
// Return Type  : void
//
void sort(array<double, 2U> &x, array<int, 2U> &idx)
{
  array<double, 1U> xwork;
  array<int, 1U> iwork;
  int i;
  int ib;
  idx.set_size(1, x.size(1));
  ib = x.size(1);
  for (i = 0; i < ib; i++) {
    idx[i] = 0;
  }
  if (x.size(1) != 0) {
    double x4[4];
    int idx4[4];
    int bLen;
    int bLen2;
    int i1;
    int i2;
    int i3;
    int i4;
    int idx_tmp;
    int n;
    int wOffset_tmp;
    n = x.size(1);
    x4[0] = 0.0;
    idx4[0] = 0;
    x4[1] = 0.0;
    idx4[1] = 0;
    x4[2] = 0.0;
    idx4[2] = 0;
    x4[3] = 0.0;
    idx4[3] = 0;
    iwork.set_size(x.size(1));
    ib = x.size(1);
    xwork.set_size(x.size(1));
    for (i = 0; i < ib; i++) {
      iwork[i] = 0;
      xwork[i] = 0.0;
    }
    bLen2 = 0;
    ib = 0;
    for (int k{0}; k < n; k++) {
      if (std::isnan(x[k])) {
        idx_tmp = (n - bLen2) - 1;
        idx[idx_tmp] = k + 1;
        xwork[idx_tmp] = x[k];
        bLen2++;
      } else {
        ib++;
        idx4[ib - 1] = k + 1;
        x4[ib - 1] = x[k];
        if (ib == 4) {
          double d;
          double d1;
          ib = k - bLen2;
          if (x4[0] <= x4[1]) {
            i1 = 1;
            i2 = 2;
          } else {
            i1 = 2;
            i2 = 1;
          }
          if (x4[2] <= x4[3]) {
            i3 = 3;
            i4 = 4;
          } else {
            i3 = 4;
            i4 = 3;
          }
          d = x4[i3 - 1];
          d1 = x4[i1 - 1];
          if (d1 <= d) {
            d1 = x4[i2 - 1];
            if (d1 <= d) {
              i = i1;
              bLen = i2;
              i1 = i3;
              i2 = i4;
            } else if (d1 <= x4[i4 - 1]) {
              i = i1;
              bLen = i3;
              i1 = i2;
              i2 = i4;
            } else {
              i = i1;
              bLen = i3;
              i1 = i4;
            }
          } else {
            d = x4[i4 - 1];
            if (d1 <= d) {
              if (x4[i2 - 1] <= d) {
                i = i3;
                bLen = i1;
                i1 = i2;
                i2 = i4;
              } else {
                i = i3;
                bLen = i1;
                i1 = i4;
              }
            } else {
              i = i3;
              bLen = i4;
            }
          }
          idx[ib - 3] = idx4[i - 1];
          idx[ib - 2] = idx4[bLen - 1];
          idx[ib - 1] = idx4[i1 - 1];
          idx[ib] = idx4[i2 - 1];
          x[ib - 3] = x4[i - 1];
          x[ib - 2] = x4[bLen - 1];
          x[ib - 1] = x4[i1 - 1];
          x[ib] = x4[i2 - 1];
          ib = 0;
        }
      }
    }
    wOffset_tmp = x.size(1) - bLen2;
    if (ib > 0) {
      signed char perm[4];
      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      if (ib == 1) {
        perm[0] = 1;
      } else if (ib == 2) {
        if (x4[0] <= x4[1]) {
          perm[0] = 1;
          perm[1] = 2;
        } else {
          perm[0] = 2;
          perm[1] = 1;
        }
      } else if (x4[0] <= x4[1]) {
        if (x4[1] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 2;
          perm[2] = 3;
        } else if (x4[0] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 3;
          perm[2] = 2;
        } else {
          perm[0] = 3;
          perm[1] = 1;
          perm[2] = 2;
        }
      } else if (x4[0] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 1;
        perm[2] = 3;
      } else if (x4[1] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 3;
        perm[2] = 1;
      } else {
        perm[0] = 3;
        perm[1] = 2;
        perm[2] = 1;
      }
      i = static_cast<unsigned char>(ib);
      for (int k{0}; k < i; k++) {
        idx_tmp = (wOffset_tmp - ib) + k;
        bLen = perm[k];
        idx[idx_tmp] = idx4[bLen - 1];
        x[idx_tmp] = x4[bLen - 1];
      }
    }
    i1 = bLen2 >> 1;
    for (int k{0}; k < i1; k++) {
      ib = wOffset_tmp + k;
      i2 = idx[ib];
      idx_tmp = (n - k) - 1;
      idx[ib] = idx[idx_tmp];
      idx[idx_tmp] = i2;
      x[ib] = xwork[idx_tmp];
      x[idx_tmp] = xwork[ib];
    }
    if ((bLen2 & 1) != 0) {
      i = wOffset_tmp + i1;
      x[i] = xwork[i];
    }
    ib = 2;
    if (wOffset_tmp > 1) {
      if (x.size(1) >= 256) {
        n = wOffset_tmp >> 8;
        if (n > 0) {
          for (int b{0}; b < n; b++) {
            double b_xwork[256];
            int b_iwork[256];
            i4 = (b << 8) - 1;
            for (int b_b{0}; b_b < 6; b_b++) {
              bLen = 1 << (b_b + 2);
              bLen2 = bLen << 1;
              i = 256 >> (b_b + 3);
              for (int k{0}; k < i; k++) {
                i2 = (i4 + k * bLen2) + 1;
                for (i1 = 0; i1 < bLen2; i1++) {
                  ib = i2 + i1;
                  b_iwork[i1] = idx[ib];
                  b_xwork[i1] = x[ib];
                }
                i3 = 0;
                i1 = bLen;
                ib = i2 - 1;
                int exitg1;
                do {
                  exitg1 = 0;
                  ib++;
                  if (b_xwork[i3] <= b_xwork[i1]) {
                    idx[ib] = b_iwork[i3];
                    x[ib] = b_xwork[i3];
                    if (i3 + 1 < bLen) {
                      i3++;
                    } else {
                      exitg1 = 1;
                    }
                  } else {
                    idx[ib] = b_iwork[i1];
                    x[ib] = b_xwork[i1];
                    if (i1 + 1 < bLen2) {
                      i1++;
                    } else {
                      ib -= i3;
                      for (i1 = i3 + 1; i1 <= bLen; i1++) {
                        idx_tmp = ib + i1;
                        idx[idx_tmp] = b_iwork[i1 - 1];
                        x[idx_tmp] = b_xwork[i1 - 1];
                      }
                      exitg1 = 1;
                    }
                  }
                } while (exitg1 == 0);
              }
            }
          }
          ib = n << 8;
          i1 = wOffset_tmp - ib;
          if (i1 > 0) {
            merge_block(idx, x, ib, i1, 2, iwork, xwork);
          }
          ib = 8;
        }
      }
      merge_block(idx, x, 0, wOffset_tmp, ib, iwork, xwork);
    }
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for sort.cpp
//
// [EOF]
//
