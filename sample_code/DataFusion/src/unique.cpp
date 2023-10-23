//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: unique.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "unique.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const array<unsigned int, 1U> &a
//                array<unsigned int, 1U> &b
// Return Type  : void
//
namespace coder {
void unique_vector(const array<unsigned int, 1U> &a, array<unsigned int, 1U> &b)
{
  array<int, 1U> idx;
  array<int, 1U> iwork;
  int b_i;
  int i;
  int k;
  int n;
  int na;
  int qEnd;
  na = a.size(0);
  n = a.size(0) + 1;
  idx.set_size(a.size(0));
  i = a.size(0);
  for (b_i = 0; b_i < i; b_i++) {
    idx[b_i] = 0;
  }
  if (a.size(0) != 0) {
    iwork.set_size(a.size(0));
    b_i = a.size(0) - 1;
    for (k = 1; k <= b_i; k += 2) {
      if (a[k - 1] <= a[k]) {
        idx[k - 1] = k;
        idx[k] = k + 1;
      } else {
        idx[k - 1] = k + 1;
        idx[k] = k;
      }
    }
    if ((a.size(0) & 1) != 0) {
      idx[a.size(0) - 1] = a.size(0);
    }
    i = 2;
    while (i < n - 1) {
      int i2;
      int j;
      i2 = i << 1;
      j = 1;
      for (int pEnd{i + 1}; pEnd < n; pEnd = qEnd + i) {
        int kEnd;
        int p;
        int q;
        p = j;
        q = pEnd;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }
        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          int i1;
          b_i = idx[p - 1];
          i1 = idx[q - 1];
          if (a[b_i - 1] <= a[i1 - 1]) {
            iwork[k] = b_i;
            p++;
            if (p == pEnd) {
              while (q < qEnd) {
                k++;
                iwork[k] = idx[q - 1];
                q++;
              }
            }
          } else {
            iwork[k] = i1;
            q++;
            if (q == qEnd) {
              while (p < pEnd) {
                k++;
                iwork[k] = idx[p - 1];
                p++;
              }
            }
          }
          k++;
        }
        for (k = 0; k < kEnd; k++) {
          idx[(j + k) - 1] = iwork[k];
        }
        j = qEnd;
      }
      i = i2;
    }
  }
  b.set_size(a.size(0));
  for (k = 0; k < na; k++) {
    b[k] = a[idx[k] - 1];
  }
  i = 0;
  k = 1;
  while (k <= na) {
    unsigned int x;
    x = b[k - 1];
    do {
      k++;
    } while (!((k > na) || (b[k - 1] != x)));
    i++;
    b[i - 1] = x;
  }
  if (i < 1) {
    i = 0;
  }
  b.set_size(i);
}

//
// Arguments    : const array<double, 2U> &a
//                array<double, 2U> &b
// Return Type  : void
//
void unique_vector(const array<double, 2U> &a, array<double, 2U> &b)
{
  array<int, 2U> idx;
  array<int, 1U> iwork;
  double x;
  int b_i;
  int i;
  int i2;
  int j;
  int k;
  int n;
  int na;
  int nb;
  int pEnd;
  int qEnd;
  boolean_T exitg1;
  na = a.size(1);
  n = a.size(1) + 1;
  idx.set_size(1, a.size(1));
  i = a.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    idx[b_i] = 0;
  }
  if (a.size(1) != 0) {
    iwork.set_size(a.size(1));
    b_i = a.size(1) - 1;
    for (k = 1; k <= b_i; k += 2) {
      x = a[k];
      if ((a[k - 1] <= x) || std::isnan(x)) {
        idx[k - 1] = k;
        idx[k] = k + 1;
      } else {
        idx[k - 1] = k + 1;
        idx[k] = k;
      }
    }
    if ((a.size(1) & 1) != 0) {
      idx[a.size(1) - 1] = a.size(1);
    }
    i = 2;
    while (i < n - 1) {
      i2 = i << 1;
      j = 1;
      for (pEnd = i + 1; pEnd < n; pEnd = qEnd + i) {
        int kEnd;
        int q;
        nb = j;
        q = pEnd - 1;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }
        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          x = a[idx[q] - 1];
          b_i = idx[nb - 1];
          if ((a[b_i - 1] <= x) || std::isnan(x)) {
            iwork[k] = b_i;
            nb++;
            if (nb == pEnd) {
              while (q + 1 < qEnd) {
                k++;
                iwork[k] = idx[q];
                q++;
              }
            }
          } else {
            iwork[k] = idx[q];
            q++;
            if (q + 1 == qEnd) {
              while (nb < pEnd) {
                k++;
                iwork[k] = idx[nb - 1];
                nb++;
              }
            }
          }
          k++;
        }
        for (k = 0; k < kEnd; k++) {
          idx[(j + k) - 1] = iwork[k];
        }
        j = qEnd;
      }
      i = i2;
    }
  }
  b.set_size(1, a.size(1));
  for (k = 0; k < na; k++) {
    b[k] = a[idx[k] - 1];
  }
  k = 0;
  while ((k + 1 <= na) && std::isinf(b[k]) && (b[k] < 0.0)) {
    k++;
  }
  i2 = k;
  k = a.size(1);
  while ((k >= 1) && std::isnan(b[k - 1])) {
    k--;
  }
  pEnd = a.size(1) - k;
  exitg1 = false;
  while ((!exitg1) && (k >= 1)) {
    x = b[k - 1];
    if (std::isinf(x) && (x > 0.0)) {
      k--;
    } else {
      exitg1 = true;
    }
  }
  i = (a.size(1) - k) - pEnd;
  nb = -1;
  if (i2 > 0) {
    nb = 0;
  }
  while (i2 + 1 <= k) {
    x = b[i2];
    do {
      i2++;
    } while (!((i2 + 1 > k) || (b[i2] != x)));
    nb++;
    b[nb] = x;
  }
  if (i > 0) {
    nb++;
    b[nb] = b[k];
  }
  i2 = k + i;
  for (j = 0; j < pEnd; j++) {
    b[(nb + j) + 1] = b[i2 + j];
  }
  if (pEnd - 1 >= 0) {
    nb += pEnd;
  }
  if (nb + 1 < 1) {
    b_i = 0;
  } else {
    b_i = nb + 1;
  }
  b.set_size(b.size(0), b_i);
}

} // namespace coder

//
// File trailer for unique.cpp
//
// [EOF]
//
