//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Fuserxcov.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "Fuserxcov.h"
#include "ensurePosDefMatrix.h"
#include "gaussEKFilter.h"
#include "mldivide.h"
#include "objectTrack.h"
#include "qrsolve.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "unique.h"
#include "xzgetrf.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : objectTrack &centralTrack
//                const array<objectTrack, 2U> &sourceTracks
//                const array<double, 1U> &inAssigned
// Return Type  : void
//
namespace coder {
namespace fusion {
namespace internal {
void Fuserxcov::fuse(objectTrack &centralTrack,
                     const array<objectTrack, 2U> &sourceTracks,
                     const array<double, 1U> &inAssigned) const
{
  array<double, 3U> allCovars;
  array<double, 2U> A;
  array<double, 2U> CovMatrix;
  array<double, 2U> allStates;
  array<double, 2U> allTimes;
  array<double, 2U> b_A;
  array<double, 2U> uniqueTimes;
  array<double, 1U> B;
  array<double, 1U> b_B;
  array<int, 2U> b_I;
  array<int, 2U> ii;
  array<unsigned int, 2U> tracksAtThisTime;
  array<boolean_T, 2U> x;
  double b_centralTrack[36];
  double trackTime;
  int exponent;
  int i;
  int ibtile;
  int nx;
  allTimes.set_size(1, inAssigned.size(0));
  nx = inAssigned.size(0);
  for (i = 0; i < nx; i++) {
    allTimes[i] = sourceTracks[static_cast<int>(inAssigned[0]) - 1].pUpdateTime;
  }
  i = inAssigned.size(0);
  for (ibtile = 0; ibtile <= i - 2; ibtile++) {
    allTimes[ibtile + 1] =
        sourceTracks[static_cast<int>(inAssigned[ibtile + 1]) - 1].pUpdateTime;
  }
  ::coder::internal::sort(allTimes, ii);
  b_I.set_size(1, ii.size(1));
  nx = ii.size(1);
  for (i = 0; i < nx; i++) {
    b_I[i] = ii[i];
  }
  unique_vector(allTimes, uniqueTimes);
  trackTime = centralTrack.pUpdateTime;
  i = uniqueTimes.size(1);
  for (int j{0}; j < i; j++) {
    double initialFusedState[6];
    double s;
    int b_k;
    int i1;
    int i2;
    int ibmat;
    int idx;
    int jtilecol;
    int k;
    boolean_T exitg1;
    s = uniqueTimes[j];
    gaussEKFilter::predict(centralTrack.pState, centralTrack.pStateCovariance,
                           ProcessNoise, s - trackTime, initialFusedState);
    nx = allTimes.size(1);
    x.set_size(1, allTimes.size(1));
    for (i1 = 0; i1 < nx; i1++) {
      x[i1] = (s == allTimes[i1]);
    }
    nx = x.size(1);
    idx = 0;
    ii.set_size(1, x.size(1));
    ibtile = 0;
    exitg1 = false;
    while ((!exitg1) && (ibtile <= nx - 1)) {
      if (x[ibtile]) {
        idx++;
        ii[idx - 1] = ibtile + 1;
        if (idx >= nx) {
          exitg1 = true;
        } else {
          ibtile++;
        }
      } else {
        ibtile++;
      }
    }
    if (x.size(1) == 1) {
      if (idx == 0) {
        ii.set_size(1, 0);
      }
    } else {
      if (idx < 1) {
        idx = 0;
      }
      ii.set_size(ii.size(0), idx);
    }
    tracksAtThisTime.set_size(1, ii.size(1));
    nx = ii.size(1);
    for (i1 = 0; i1 < nx; i1++) {
      tracksAtThisTime[i1] = static_cast<unsigned int>(ii[i1]);
    }
    i1 = static_cast<int>(static_cast<unsigned int>(tracksAtThisTime.size(1)) +
                          1U);
    allStates.set_size(
        6, static_cast<int>(
               static_cast<unsigned int>(tracksAtThisTime.size(1)) + 1U));
    allCovars.set_size(
        6, 6,
        static_cast<int>(static_cast<unsigned int>(tracksAtThisTime.size(1)) +
                         1U));
    for (jtilecol = 0; jtilecol < i1; jtilecol++) {
      nx = jtilecol * 6;
      ibtile = jtilecol * 36 - 1;
      for (k = 0; k < 6; k++) {
        allStates[nx + k] = initialFusedState[k];
        idx = k * 6;
        ibmat = ibtile + k * 6;
        for (b_k = 0; b_k < 6; b_k++) {
          allCovars[(ibmat + b_k) + 1] =
              centralTrack.pStateCovariance[idx + b_k];
        }
      }
    }
    i1 = tracksAtThisTime.size(1);
    for (k = 0; k < i1; k++) {
      nx = static_cast<int>(
               inAssigned[b_I[static_cast<int>(tracksAtThisTime[k]) - 1] - 1]) -
           1;
      for (i2 = 0; i2 < 6; i2++) {
        allStates[i2 + 6 * (k + 1)] = sourceTracks[nx].pState[i2];
        for (idx = 0; idx < 6; idx++) {
          allCovars[(idx + 6 * i2) + 36 * (k + 1)] =
              sourceTracks[nx].pStateCovariance[idx + 6 * i2];
        }
      }
    }
    i1 = allCovars.size(2);
    for (ibtile = 0; ibtile < i1; ibtile++) {
      for (k = 0; k < 6; k++) {
        centralTrack.pState[k] = std::abs(allCovars[(k + 6 * k) + 36 * ibtile]);
      }
      if (!std::isnan(centralTrack.pState[0])) {
        idx = 1;
      } else {
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k < 7)) {
          if (!std::isnan(centralTrack.pState[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }
      if (idx == 0) {
        trackTime = centralTrack.pState[0];
      } else {
        trackTime = centralTrack.pState[idx - 1];
        i2 = idx + 1;
        for (k = i2; k < 7; k++) {
          s = centralTrack.pState[k - 1];
          if (trackTime < s) {
            trackTime = s;
          }
        }
      }
      if ((!std::isinf(trackTime)) && (!std::isnan(trackTime)) &&
          (!(trackTime < 4.4501477170144028E-308))) {
        std::frexp(trackTime, &exponent);
      }
    }
    if (allCovars.size(2) == 1) {
      for (i1 = 0; i1 < 6; i1++) {
        centralTrack.pState[i1] = allStates[i1];
        for (i2 = 0; i2 < 6; i2++) {
          centralTrack.pStateCovariance[i2 + 6 * i1] = allCovars[i2 + 6 * i1];
        }
      }
    } else {
      CovMatrix.set_size(1, allCovars.size(2));
      i1 = allCovars.size(2);
      for (ibtile = 0; ibtile < i1; ibtile++) {
        int ipiv[6];
        boolean_T isodd;
        for (i2 = 0; i2 < 6; i2++) {
          for (idx = 0; idx < 6; idx++) {
            centralTrack.pStateCovariance[idx + 6 * i2] =
                allCovars[(idx + 6 * i2) + 36 * ibtile];
          }
        }
        ::coder::internal::reflapack::xzgetrf(centralTrack.pStateCovariance,
                                              ipiv);
        trackTime = centralTrack.pStateCovariance[0];
        isodd = false;
        for (k = 0; k < 5; k++) {
          trackTime *= centralTrack.pStateCovariance[(k + 6 * (k + 1)) + 1];
          if (ipiv[k] > k + 1) {
            isodd = !isodd;
          }
        }
        if (isodd) {
          trackTime = -trackTime;
        }
        CovMatrix[ibtile] = trackTime;
      }
      A.set_size(allCovars.size(2) - 1, allCovars.size(2));
      nx = (allCovars.size(2) - 1) * allCovars.size(2);
      for (i1 = 0; i1 < nx; i1++) {
        A[i1] = 0.0;
      }
      i1 = allCovars.size(2);
      for (ibtile = 0; ibtile <= i1 - 2; ibtile++) {
        A[ibtile + A.size(0) * ibtile] = CovMatrix[ibtile];
        A[ibtile + A.size(0) * (ibtile + 1)] = -CovMatrix[ibtile + 1];
      }
      b_A.set_size(A.size(0) + 1, A.size(1));
      nx = A.size(1);
      for (i1 = 0; i1 < nx; i1++) {
        ibtile = A.size(0);
        for (i2 = 0; i2 < ibtile; i2++) {
          b_A[i2 + b_A.size(0) * i1] = A[i2 + A.size(0) * i1];
        }
      }
      nx = A.size(1);
      for (i1 = 0; i1 < nx; i1++) {
        b_A[A.size(0) + b_A.size(0) * i1] = 1.0;
      }
      B.set_size(allCovars.size(2));
      nx = allCovars.size(2);
      for (i1 = 0; i1 <= nx - 2; i1++) {
        B[i1] = 0.0;
      }
      B[allCovars.size(2) - 1] = 1.0;
      if (b_A.size(0) == b_A.size(1)) {
        int LDA;
        int n;
        jtilecol = b_A.size(0);
        n = b_A.size(1);
        if (jtilecol <= n) {
          n = jtilecol;
        }
        jtilecol = B.size(0);
        if (jtilecol <= n) {
          n = jtilecol;
        }
        LDA = b_A.size(0);
        ii.set_size(1, n);
        ii[0] = 1;
        nx = 1;
        for (k = 2; k <= n; k++) {
          nx++;
          ii[k - 1] = nx;
        }
        jtilecol = n - 1;
        if (jtilecol > n) {
          jtilecol = n;
        }
        for (int b_j{0}; b_j < jtilecol; b_j++) {
          int b_tmp;
          b_k = n - b_j;
          b_tmp = b_j * (LDA + 1);
          ibmat = b_tmp + 2;
          if (b_k < 1) {
            nx = -1;
          } else {
            nx = 0;
            if (b_k > 1) {
              trackTime = std::abs(b_A[b_tmp]);
              for (k = 2; k <= b_k; k++) {
                s = std::abs(b_A[(b_tmp + k) - 1]);
                if (s > trackTime) {
                  nx = k - 1;
                  trackTime = s;
                }
              }
            }
          }
          if (b_A[b_tmp + nx] != 0.0) {
            if (nx != 0) {
              idx = b_j + nx;
              ii[b_j] = idx + 1;
              for (k = 0; k < n; k++) {
                nx = k * LDA;
                ibtile = b_j + nx;
                trackTime = b_A[ibtile];
                i1 = idx + nx;
                b_A[ibtile] = b_A[i1];
                b_A[i1] = trackTime;
              }
            }
            i1 = b_tmp + b_k;
            for (ibtile = ibmat; ibtile <= i1; ibtile++) {
              b_A[ibtile - 1] = b_A[ibtile - 1] / b_A[b_tmp];
            }
          }
          nx = b_tmp + LDA;
          idx = nx;
          for (ibtile = 0; ibtile <= b_k - 2; ibtile++) {
            trackTime = b_A[nx + ibtile * LDA];
            if (trackTime != 0.0) {
              i1 = idx + 2;
              i2 = b_k + idx;
              for (ibmat = i1; ibmat <= i2; ibmat++) {
                b_A[ibmat - 1] = b_A[ibmat - 1] +
                                 b_A[((b_tmp + ibmat) - idx) - 1] * -trackTime;
              }
            }
            idx += LDA;
          }
        }
        LDA = b_A.size(0);
        for (ibtile = 0; ibtile <= n - 2; ibtile++) {
          i1 = ii[ibtile];
          if (i1 != ibtile + 1) {
            nx = static_cast<int>(B[ibtile]);
            B[ibtile] = B[i1 - 1];
            B[i1 - 1] = nx;
          }
        }
        for (k = 0; k < n; k++) {
          nx = LDA * k;
          if (B[k] != 0.0) {
            i1 = k + 2;
            for (ibtile = i1; ibtile <= n; ibtile++) {
              B[ibtile - 1] = B[ibtile - 1] - B[k] * b_A[(ibtile + nx) - 1];
            }
          }
        }
        for (k = n; k >= 1; k--) {
          nx = LDA * (k - 1);
          s = B[k - 1];
          if (s != 0.0) {
            B[k - 1] = s / b_A[(k + nx) - 1];
            for (ibtile = 0; ibtile <= k - 2; ibtile++) {
              B[ibtile] = B[ibtile] - B[k - 1] * b_A[ibtile + nx];
            }
          }
        }
      } else {
        b_B.set_size(B.size(0));
        nx = B.size(0) - 1;
        for (i1 = 0; i1 <= nx; i1++) {
          b_B[i1] = B[i1];
        }
        ::coder::internal::qrsolve(b_A, b_B, B);
      }
      std::memset(&centralTrack.pStateCovariance[0], 0, 36U * sizeof(double));
      i1 = allCovars.size(2);
      for (ibtile = 0; ibtile < i1; ibtile++) {
        mldivide(&allCovars[36 * ibtile], b_centralTrack);
        for (i2 = 0; i2 < 36; i2++) {
          centralTrack.pStateCovariance[i2] += B[ibtile] * b_centralTrack[i2];
        }
      }
      std::copy(&centralTrack.pStateCovariance[0],
                &centralTrack.pStateCovariance[36], &b_centralTrack[0]);
      mldivide(b_centralTrack, centralTrack.pStateCovariance);
      for (ibtile = 0; ibtile < 6; ibtile++) {
        initialFusedState[ibtile] = 0.0;
      }
      i1 = allCovars.size(2);
      for (ibtile = 0; ibtile < i1; ibtile++) {
        mldivide(&allCovars[36 * ibtile], b_centralTrack);
        for (i2 = 0; i2 < 6; i2++) {
          s = 0.0;
          for (idx = 0; idx < 6; idx++) {
            s += B[ibtile] * b_centralTrack[i2 + 6 * idx] *
                 allStates[idx + 6 * ibtile];
          }
          initialFusedState[i2] += s;
        }
      }
      for (i1 = 0; i1 < 6; i1++) {
        centralTrack.pState[i1] = 0.0;
        for (i2 = 0; i2 < 6; i2++) {
          centralTrack.pState[i1] +=
              centralTrack.pStateCovariance[i1 + 6 * i2] *
              initialFusedState[i2];
        }
      }
    }
    trackTime = uniqueTimes[j];
  }
  ensurePosDefMatrix(centralTrack.pStateCovariance);
  centralTrack.pUpdateTime = trackTime;
}

} // namespace internal
} // namespace fusion
} // namespace coder

//
// File trailer for Fuserxcov.cpp
//
// [EOF]
//
