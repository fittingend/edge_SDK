//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: matchpairs.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "matchpairs.h"
#include "minPriorityQueue.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const array<double, 2U> &A
//                array<int, 1U> &m1
//                array<int, 1U> &m2
// Return Type  : void
//
namespace coder {
void perfectMatching(const array<double, 2U> &A, array<int, 1U> &m1,
                     array<int, 1U> &m2)
{
  matlab::internal::coder::minPriorityQueue b_queue;
  matlab::internal::coder::minPriorityQueue queue;
  array<double, 1U> a__2;
  array<double, 1U> distancesR;
  array<double, 1U> ex;
  array<double, 1U> minIndices;
  array<double, 1U> pairWeightR;
  array<int, 1U> idx;
  array<unsigned char, 1U> colorsR;
  int c;
  int i;
  int n_tmp;
  n_tmp = A.size(0) - 1;
  m1.set_size(A.size(0));
  c = A.size(0);
  m2.set_size(A.size(0));
  a__2.set_size(A.size(0));
  for (i = 0; i < c; i++) {
    m1[i] = 0;
    m2[i] = 0;
    a__2[i] = rtInf;
  }
  if (n_tmp + 1 != 0) {
    double edge_weight_shifted;
    int b_i;
    int n;
    boolean_T guard1;
    boolean_T p;
    n = A.size(1);
    ex.set_size(A.size(0));
    idx.set_size(A.size(0));
    c = A.size(0);
    for (i = 0; i < c; i++) {
      idx[i] = 1;
    }
    if (A.size(0) >= 1) {
      for (b_i = 0; b_i <= n_tmp; b_i++) {
        ex[b_i] = A[b_i];
      }
      for (c = 2; c <= n; c++) {
        for (b_i = 0; b_i <= n_tmp; b_i++) {
          edge_weight_shifted = A[b_i + A.size(0) * (c - 1)];
          if (std::isnan(edge_weight_shifted)) {
            p = false;
          } else if (std::isnan(ex[b_i])) {
            p = true;
          } else {
            p = (ex[b_i] > edge_weight_shifted);
          }
          if (p) {
            ex[b_i] = edge_weight_shifted;
            idx[b_i] = c;
          }
        }
      }
    }
    minIndices.set_size(idx.size(0));
    c = idx.size(0);
    p = true;
    for (n = 0; n < c; n++) {
      minIndices[n] = idx[n];
      if ((!p) || (std::isinf(ex[n]) || std::isnan(ex[n]))) {
        p = false;
      }
    }
    guard1 = false;
    if (!p) {
      guard1 = true;
    } else {
      int colStart;
      boolean_T exitg1;
      pairWeightR.set_size(n_tmp + 1);
      for (n = 0; n <= n_tmp; n++) {
        pairWeightR[n] = 0.0;
        i = static_cast<int>(minIndices[n]);
        if (m1[i - 1] == 0) {
          m2[n] = i;
          m1[i - 1] = n + 1;
          pairWeightR[n] = ex[n];
        }
        for (c = 0; c <= n_tmp; c++) {
          edge_weight_shifted = A[c + A.size(0) * n] - ex[c];
          if (edge_weight_shifted < a__2[n]) {
            a__2[n] = edge_weight_shifted;
          }
        }
      }
      queue.heap.set_size(n_tmp + 1);
      queue.indexToHeap.set_size(n_tmp + 1);
      queue.len = 0;
      colStart = 0;
      exitg1 = false;
      while ((!exitg1) && (colStart <= n_tmp)) {
        if (m1[colStart] != 0) {
          colStart++;
        } else {
          double last_weight_sap;
          double lsap;
          int clast;
          int exitg2;
          int rlast;
          boolean_T guard2;
          b_queue = queue;
          minIndices.set_size(n_tmp + 1);
          idx.set_size(n_tmp + 1);
          distancesR.set_size(n_tmp + 1);
          colorsR.set_size(n_tmp + 1);
          for (i = 0; i <= n_tmp; i++) {
            minIndices[i] = 0.0;
            idx[i] = 0;
            distancesR[i] = rtInf;
            colorsR[i] = 0U;
          }
          idx[colStart] = 0;
          b_queue.len = 0;
          edge_weight_shifted = 0.0;
          lsap = rtInf;
          rlast = -1;
          clast = -1;
          last_weight_sap = rtInf;
          c = colStart;
          guard2 = false;
          int exitg3;
          do {
            double edge_weight;
            exitg3 = 0;
            n = 0;
            do {
              exitg2 = 0;
              if (n <= n_tmp) {
                if (colorsR[n] == 2) {
                  n++;
                } else {
                  double dnew;
                  edge_weight = A[n + A.size(0) * c];
                  dnew =
                      edge_weight_shifted + ((edge_weight - ex[n]) - a__2[c]);
                  if (dnew < lsap) {
                    if (m2[n] == 0) {
                      lsap = dnew;
                      rlast = n;
                      clast = c;
                      last_weight_sap = edge_weight;
                    } else if (dnew < distancesR[n]) {
                      distancesR[n] = dnew;
                      idx[m2[n] - 1] = c + 1;
                      minIndices[n] = edge_weight;
                      if (colorsR[n] == 0) {
                        b_queue.len++;
                        b_queue.heap[b_queue.len - 1] = n + 1;
                        b_queue.indexToHeap[n] = b_queue.len;
                        b_queue.percUp(b_queue.len, distancesR);
                        colorsR[n] = 1U;
                      } else {
                        b_queue.percUp(b_queue.indexToHeap[n], distancesR);
                      }
                    }
                    n++;
                  } else if (((dnew >= -1.7976931348623157E+308) &&
                              (dnew <= 1.7976931348623157E+308)) ||
                             (!(edge_weight >= -1.7976931348623157E+308)) ||
                             (!(edge_weight <= 1.7976931348623157E+308))) {
                    n++;
                  } else {
                    p = false;
                    exitg2 = 2;
                  }
                }
              } else {
                exitg2 = 1;
              }
            } while (exitg2 == 0);
            if (exitg2 == 1) {
              if (b_queue.len == 0) {
                guard2 = true;
                exitg3 = 1;
              } else {
                c = b_queue.heap[0] - 1;
                i = b_queue.len - 1;
                b_queue.heap[0] = b_queue.heap[b_queue.len - 1];
                b_queue.indexToHeap[b_queue.heap[0] - 1] = 1;
                b_queue.len--;
                b_i = 0;
                int exitg4;
                int ichild;
                do {
                  exitg4 = 0;
                  ichild = (b_i + 1) << 1;
                  if (ichild <= i) {
                    if (ichild + 1 > i) {
                      ichild--;
                    } else {
                      n = b_queue.heap[ichild - 1];
                      edge_weight_shifted = distancesR[n - 1];
                      edge_weight = distancesR[b_queue.heap[ichild] - 1];
                      if ((edge_weight_shifted < edge_weight) ||
                          ((edge_weight_shifted == edge_weight) &&
                           (n <= b_queue.heap[ichild]))) {
                        ichild--;
                      }
                    }
                    edge_weight_shifted = distancesR[b_queue.heap[b_i] - 1];
                    edge_weight = distancesR[b_queue.heap[ichild] - 1];
                    if ((edge_weight_shifted < edge_weight) ||
                        ((edge_weight_shifted == edge_weight) &&
                         (b_queue.heap[b_i] <= b_queue.heap[ichild]))) {
                      exitg4 = 1;
                    } else {
                      n = b_queue.heap[b_i];
                      b_queue.heap[b_i] = b_queue.heap[ichild];
                      b_queue.heap[ichild] = n;
                      n = b_queue.indexToHeap[b_queue.heap[b_i] - 1];
                      b_queue.indexToHeap[b_queue.heap[b_i] - 1] =
                          b_queue.indexToHeap[b_queue.heap[ichild] - 1];
                      b_queue.indexToHeap[b_queue.heap[ichild] - 1] = n;
                      b_i = ichild;
                    }
                  } else {
                    exitg4 = 1;
                  }
                } while (exitg4 == 0);
                edge_weight_shifted = distancesR[c];
                if (lsap <= distancesR[c]) {
                  guard2 = true;
                  exitg3 = 1;
                } else {
                  colorsR[c] = 2U;
                  c = m2[c] - 1;
                  guard2 = false;
                }
              }
            } else {
              exitg3 = 1;
            }
          } while (exitg3 == 0);
          if (guard2) {
            p = (lsap < rtInf);
            if (p) {
              c = rlast + 1;
              do {
                exitg2 = 0;
                n = m1[clast];
                m2[c - 1] = clast + 1;
                m1[clast] = c;
                pairWeightR[c - 1] = last_weight_sap;
                if (idx[clast] == 0) {
                  exitg2 = 1;
                } else {
                  c = n;
                  clast = idx[clast] - 1;
                  last_weight_sap = minIndices[n - 1];
                }
              } while (exitg2 == 0);
              for (n = 0; n <= n_tmp; n++) {
                if (colorsR[n] == 2) {
                  ex[n] = (ex[n] - lsap) + distancesR[n];
                }
              }
              for (c = 0; c <= n_tmp; c++) {
                if (m1[c] != 0) {
                  a__2[c] = pairWeightR[m1[c] - 1] - ex[m1[c] - 1];
                }
              }
            }
          }
          if (!p) {
            guard1 = true;
            exitg1 = true;
          } else {
            colStart++;
          }
        }
      }
    }
    if (guard1) {
      m1.set_size(0);
      m2.set_size(0);
    }
  }
}

} // namespace coder

//
// File trailer for matchpairs.cpp
//
// [EOF]
//
