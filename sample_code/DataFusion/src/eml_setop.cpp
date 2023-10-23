//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eml_setop.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "eml_setop.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : const array<unsigned int, 1U> &a
//                const array<unsigned int, 1U> &b
//                array<unsigned int, 1U> &c
//                array<int, 1U> &ia
//                array<int, 1U> &ib
// Return Type  : void
//
namespace coder {
void do_vectors(const array<unsigned int, 1U> &a,
                const array<unsigned int, 1U> &b, array<unsigned int, 1U> &c,
                array<int, 1U> &ia, array<int, 1U> &ib)
{
  int iafirst;
  int ialast;
  int ibfirst;
  int iblast;
  int nc;
  int ncmax;
  nc = a.size(0);
  ncmax = b.size(0);
  if (nc <= ncmax) {
    ncmax = nc;
  }
  c.set_size(ncmax);
  ia.set_size(ncmax);
  ib.set_size(ncmax);
  nc = 0;
  iafirst = 0;
  ialast = 1;
  ibfirst = 0;
  iblast = 1;
  while ((ialast <= a.size(0)) && (iblast <= b.size(0))) {
    unsigned int ak;
    int b_ialast;
    int b_iblast;
    unsigned int bk;
    b_ialast = ialast;
    ak = a[ialast - 1];
    while ((b_ialast < a.size(0)) && (a[b_ialast] == ak)) {
      b_ialast++;
    }
    ialast = b_ialast;
    b_iblast = iblast;
    bk = b[iblast - 1];
    while ((b_iblast < b.size(0)) && (b[b_iblast] == bk)) {
      b_iblast++;
    }
    iblast = b_iblast;
    if (ak == bk) {
      nc++;
      c[nc - 1] = ak;
      ia[nc - 1] = iafirst + 1;
      ib[nc - 1] = ibfirst + 1;
      ialast = b_ialast + 1;
      iafirst = b_ialast;
      iblast = b_iblast + 1;
      ibfirst = b_iblast;
    } else if (ak < bk) {
      ialast = b_ialast + 1;
      iafirst = b_ialast;
    } else {
      iblast = b_iblast + 1;
      ibfirst = b_iblast;
    }
  }
  if (ncmax > 0) {
    if (nc < 1) {
      nc = 0;
    }
    ia.set_size(nc);
    ib.set_size(nc);
    c.set_size(nc);
  }
}

} // namespace coder

//
// File trailer for eml_setop.cpp
//
// [EOF]
//
