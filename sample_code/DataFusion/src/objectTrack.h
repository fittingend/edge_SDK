//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: objectTrack.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

#ifndef OBJECTTRACK_H
#define OBJECTTRACK_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
class objectTrack {
public:
  unsigned int TrackID;
  unsigned int BranchID;
  unsigned int SourceIndex;
  unsigned int Age;
  double ObjectClassID;
  double ObjectClassProbabilities;
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
  double pState[6];
  double pStateCovariance[36];
  double pUpdateTime;
  array<boolean_T, 2U> pTrackLogicState;
};

} // namespace coder

#endif
//
// File trailer for objectTrack.h
//
// [EOF]
//
