//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Fuserxcov.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

#ifndef FUSERXCOV_H
#define FUSERXCOV_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class objectTrack;

}

// Type Definitions
namespace coder {
namespace fusion {
namespace internal {
class Fuserxcov {
public:
  void fuse(objectTrack &centralTrack,
            const array<objectTrack, 2U> &sourceTracks,
            const array<double, 1U> &inAssigned) const;
  double ProcessNoise[9];
};

} // namespace internal
} // namespace fusion
} // namespace coder

#endif
//
// File trailer for Fuserxcov.h
//
// [EOF]
//
