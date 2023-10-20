//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: minPriorityQueue.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "minPriorityQueue.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : int i
//                const array<double, 1U> &dist
// Return Type  : void
//
namespace coder {
namespace matlab {
namespace internal {
namespace coder {
void minPriorityQueue::percUp(int i, const array<double, 1U> &dist)
{
  int iparent;
  boolean_T exitg1;
  iparent = i / 2 - 1;
  exitg1 = false;
  while ((!exitg1) && (iparent + 1 > 0)) {
    double d;
    double d1;
    int obj_idx_1;
    obj_idx_1 = heap[i - 1];
    d = dist[obj_idx_1 - 1];
    d1 = dist[heap[iparent] - 1];
    if ((d < d1) || ((d == d1) && (obj_idx_1 <= heap[iparent]))) {
      obj_idx_1 = heap[i - 1];
      heap[i - 1] = heap[iparent];
      heap[iparent] = obj_idx_1;
      obj_idx_1 = indexToHeap[heap[i - 1] - 1];
      indexToHeap[heap[i - 1] - 1] = indexToHeap[heap[iparent] - 1];
      indexToHeap[heap[iparent] - 1] = obj_idx_1;
      i = iparent + 1;
      iparent = (iparent + 1) / 2 - 1;
    } else {
      exitg1 = true;
    }
  }
}

} // namespace coder
} // namespace internal
} // namespace matlab
} // namespace coder

//
// File trailer for minPriorityQueue.cpp
//
// [EOF]
//
