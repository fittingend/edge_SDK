//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fuserSourceConfiguration.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

#ifndef FUSERSOURCECONFIGURATION_H
#define FUSERSOURCECONFIGURATION_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
class fuserSourceConfiguration {
public:
  fuserSourceConfiguration *clone(fuserSourceConfiguration &iobj_0) const;
  double SourceIndex;
  boolean_T IsInternalSource;
  boolean_T IsInitializingCentralTracks;
  boolean_T pIsTransformToCentralValid;
  boolean_T pIsTransformToLocalValid;
};

} // namespace coder

#endif
//
// File trailer for fuserSourceConfiguration.h
//
// [EOF]
//
