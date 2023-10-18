//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: EDGE_fusion_function_231017_2135_types.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 17-Oct-2023 22:24:52
//

#ifndef EDGE_FUSION_FUNCTION_231017_2135_TYPES_H
#define EDGE_FUSION_FUNCTION_231017_2135_TYPES_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include "coder_bounded_array.h"

// Type Definitions
struct struct2_T {
  double timestamp;
  double obstacle_class;
  double cuboid_x;
  double cuboid_y;
  double cuboid_z;
  double heading_angle;
  double covariance_matrix[9];
  double Position_x;
  double Position_y;
  double Position_z;
  double Velocity_x;
  double Velocity_y;
  double Velocity_z;
};

struct struct1_T {
  struct2_T obstacle[3];
  double road_z;
  double vehicle_class;
  double Position_lat;
  double Position_long;
  double Position_Height;
  double Yaw;
  double Roll;
  double Pitch;
  double Velocity_long;
  double Velocity_lat;
  double Velocity_ang;
};

struct struct0_T {
  struct1_T vehicle[5];
};

struct struct4_T {
  coder::bounded_array<unsigned int, 100U, 2U> TrackIDsAtStepBeginning;
  coder::array<double, 2U> CostMatrix;
  coder::array<unsigned int, 2U> Assignments;
  coder::array<unsigned int, 1U> UnassignedCentralTracks;
  coder::array<unsigned int, 1U> UnassignedLocalTracks;
  coder::array<unsigned int, 1U> NonInitializingLocalTracks;
  coder::bounded_array<unsigned int, 100U, 2U> InitializedCentralTrackIDs;
  coder::array<unsigned int, 2U> UpdatedCentralTrackIDs;
  coder::bounded_array<unsigned int, 100U, 2U> DeletedTrackIDs;
  coder::bounded_array<unsigned int, 100U, 2U> TrackIDsAtStepEnd;
};

struct struct3_T {
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
  coder::array<boolean_T, 2U> TrackLogicState;
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
};

#endif
//
// File trailer for EDGE_fusion_function_231017_2135_types.h
//
// [EOF]
//
