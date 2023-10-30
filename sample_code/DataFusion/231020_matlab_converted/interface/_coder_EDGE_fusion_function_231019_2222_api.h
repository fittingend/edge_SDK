//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_EDGE_fusion_function_231019_2222_api.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

#ifndef _CODER_EDGE_FUSION_FUNCTION_231019_2222_API_H
#define _CODER_EDGE_FUSION_FUNCTION_231019_2222_API_H

// Include Files
#include "coder_array_mex.h"
#include "coder_bounded_array.h"
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
struct HubObstacleData {
  real_T timestamp;
  real_T obstacle_class;
  real_T cuboid_x;
  real_T cuboid_y;
  real_T cuboid_z;
  real_T heading_angle;
  real_T covariance_matrix[9];
  real_T Position_x;
  real_T Position_y;
  real_T Position_z;
  real_T Velocity_x;
  real_T Velocity_y;
  real_T Velocity_z;
};

struct HubVehicleData {
  HubObstacleData obstacle[3];
  real_T road_z;
  real_T vehicle_class;
  real_T Position_lat;
  real_T Position_long;
  real_T Position_Height;
  real_T Yaw;
  real_T Roll;
  real_T Pitch;
  real_T Velocity_long;
  real_T Velocity_lat;
  real_T Velocity_ang;
};

struct HubData {
  HubVehicleData vehicle[5];
};

struct struct4_T {
  coder::bounded_array<uint32_T, 100U, 2U> TrackIDsAtStepBeginning;
  coder::array<real_T, 2U> CostMatrix;
  coder::array<uint32_T, 2U> Assignments;
  coder::array<uint32_T, 1U> UnassignedCentralTracks;
  coder::array<uint32_T, 1U> UnassignedLocalTracks;
  coder::array<uint32_T, 1U> NonInitializingLocalTracks;
  coder::bounded_array<uint32_T, 100U, 2U> InitializedCentralTrackIDs;
  coder::array<uint32_T, 2U> UpdatedCentralTrackIDs;
  coder::bounded_array<uint32_T, 100U, 2U> DeletedTrackIDs;
  coder::bounded_array<uint32_T, 100U, 2U> TrackIDsAtStepEnd;
};

struct struct3_T {
  uint32_T TrackID;
  uint32_T BranchID;
  uint32_T SourceIndex;
  real_T UpdateTime;
  uint32_T Age;
  real_T State[6];
  real_T StateCovariance[36];
  real_T ObjectClassID;
  real_T ObjectClassProbabilities;
  char_T TrackLogic[7];
  coder::array<boolean_T, 2U> TrackLogicState;
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
};

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void EDGE_fusion_function_231019_2222(HubData *hub_data_Object,
                                      struct3_T confirmedTracks_data[],
                                      int32_T confirmedTracks_size[1],
                                      struct3_T tentativeTracks_data[],
                                      int32_T tentativeTracks_size[1],
                                      coder::array<struct3_T, 1U> *allTracks,
                                      struct4_T *analysisInformation);

void EDGE_fusion_function_231019_2222_api(const mxArray *prhs, int32_T nlhs,
                                          const mxArray *plhs[4]);

void EDGE_fusion_function_231019_2222_atexit();

void EDGE_fusion_function_231019_2222_initialize();

void EDGE_fusion_function_231019_2222_terminate();

void EDGE_fusion_function_231019_2222_xil_shutdown();

void EDGE_fusion_function_231019_2222_xil_terminate();

#endif
//
// File trailer for _coder_EDGE_fusion_function_231019_2222_api.h
//
// [EOF]
//
