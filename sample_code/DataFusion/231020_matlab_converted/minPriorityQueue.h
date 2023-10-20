//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: minPriorityQueue.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

#ifndef MINPRIORITYQUEUE_H
#define MINPRIORITYQUEUE_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace matlab {
namespace internal {
namespace coder {
class minPriorityQueue {
public:
  void percUp(int i, const array<double, 1U> &dist);
  array<int, 1U> heap;
  array<int, 1U> indexToHeap;
  int len;
};

} // namespace coder
} // namespace internal
} // namespace matlab
} // namespace coder

#endif
//
// File trailer for minPriorityQueue.h
//
// [EOF]
//
