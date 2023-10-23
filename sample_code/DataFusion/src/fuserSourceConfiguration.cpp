//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fuserSourceConfiguration.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "fuserSourceConfiguration.h"
#include "rt_nonfinite.h"

// Function Definitions
//
// Arguments    : fuserSourceConfiguration &iobj_0
// Return Type  : fuserSourceConfiguration *
//
namespace coder {
fuserSourceConfiguration *
fuserSourceConfiguration::clone(fuserSourceConfiguration &iobj_0) const
{
  fuserSourceConfiguration *clonedObj;
  iobj_0.pIsTransformToCentralValid = false;
  iobj_0.pIsTransformToLocalValid = false;
  clonedObj = &iobj_0;
  iobj_0.SourceIndex = SourceIndex;
  iobj_0.IsInternalSource = IsInternalSource;
  iobj_0.IsInitializingCentralTracks = IsInitializingCentralTracks;
  return clonedObj;
}

} // namespace coder

//
// File trailer for fuserSourceConfiguration.cpp
//
// [EOF]
//
