//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: EDGE_fusion_function_231019_2222_internal_types.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

#ifndef EDGE_FUSION_FUNCTION_231019_2222_INTERNAL_TYPES_H
#define EDGE_FUSION_FUNCTION_231019_2222_INTERNAL_TYPES_H

// Include Files
#include "EDGE_fusion_function_231019_2222_types.h"
#include "rtwtypes.h"

// Type Definitions
struct struct_T {
  unsigned int TrackID;
  unsigned int BranchID;
  unsigned int SourceIndex;
  double UpdateTime;
  unsigned int Age;
  double State[6];
  double StateCovariance[36];
  double ObjectClassID;
  double ObjectClassProbabilities;
  char TrackLogic[7];
  boolean_T TrackLogicState;
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
};

#endif
//
// File trailer for EDGE_fusion_function_231019_2222_internal_types.h
//
// [EOF]
//
