//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: EDGE_fusion_function_231019_2222.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "EDGE_fusion_function_231019_2222.h"
#include "EDGE_fusion_function_231019_2222_internal_types.h"
#include "EDGE_fusion_function_231019_2222_types.h"
#include "fuserSourceConfiguration.h"
#include "rt_nonfinite.h"
#include "trackFuser.h"
#include "coder_array.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cstring>

// Function Definitions
//
// function [confirmedTracks,tentativeTracks,allTracks,analysisInformation] =
// EDGE_fusion_function_231019_2222(hub_data_Object)
//
// UNTITLED9 이 함수의 요약 설명 위치
//    자세한 설명 위치
//
// Arguments    : const struct0_T *hub_data_Object
//                struct3_T confirmedTracks_data[]
//                int confirmedTracks_size[1]
//                struct3_T tentativeTracks_data[]
//                int tentativeTracks_size[1]
//                coder::array<struct3_T, 1U> &allTracks
//                struct4_T *analysisInformation
// Return Type  : void
//
void EDGE_fusion_function_231019_2222(const struct0_T *hub_data_Object,
                                      struct3_T confirmedTracks_data[],
                                      int confirmedTracks_size[1],
                                      struct3_T tentativeTracks_data[],
                                      int tentativeTracks_size[1],
                                      coder::array<struct3_T, 1U> &allTracks,
                                      struct4_T *analysisInformation)
{
  static const struct_T r{
      1U,                             // TrackID
      0U,                             // BranchID
      1U,                             // SourceIndex
      0.0,                            // UpdateTime
      1U,                             // Age
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, // State
      {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, // StateCovariance
      0.0,                                                // ObjectClassID
      1.0,                                 // ObjectClassProbabilities
      {'H', 'i', 's', 't', 'o', 'r', 'y'}, // TrackLogic
      true,                                // TrackLogicState
      true,                                // IsConfirmed
      false,                               // IsCoasted
      true                                 // IsSelfReported
  };
  static const signed char stateCov[36]{5, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0,
                                        0, 0, 5, 0, 0, 0, 0, 0, 0, 5, 0, 0,
                                        0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 5};
  static const signed char b_iv[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const char logicType[7]{'H', 'i', 's', 't', 'o', 'r', 'y'};
  coder::fuserSourceConfiguration lobj_1[5];
  coder::fuserSourceConfiguration *sourceconfig_ego[5];
  coder::fuserSourceConfiguration *b_obj;
  coder::fuserSourceConfiguration *obj;
  coder::trackFuser fuser;
  coder::array<struct_T, 2U> track_ego;
  struct_T expl_temp;
  double ids[20];
  double d;
  int b_track_ego;
  boolean_T flag;
  fuser.matlabCodegenIsDeleted = true;
  // 'EDGE_fusion_function_231019_2222:4' maxobs=20;
  //  차량 1대당 최대 인식 장애물 갯수
  // 'EDGE_fusion_function_231019_2222:5' numveh=5;
  //  차량 댓수
  //  Preallocation
  // 'EDGE_fusion_function_231019_2222:7' track_ego=toStruct(objectTrack());
  track_ego.set_size(1, 1);
  track_ego[0] = r;
  // 'EDGE_fusion_function_231019_2222:8' temp_track=toStruct(objectTrack());
  // 'EDGE_fusion_function_231019_2222:11'
  // state=struct('obs_state',[0;0;0;0;0;0],'obs_time',0,'obs_class',0);
  // 'EDGE_fusion_function_231019_2222:12'
  // target_vehicle=struct('state',struct('obs_state',[0;0;0;0;0;0],'obs_time',0,'obs_class',0));
  // 'EDGE_fusion_function_231019_2222:15' numobs=zeros(numveh,1);
  // 'EDGE_fusion_function_231019_2222:18' sourceconfig_ego=cell(1,numveh);
  // 'EDGE_fusion_function_231019_2222:19' for i= 1 : numveh
  //  차량별 tracking source configuration
  // 'EDGE_fusion_function_231019_2222:20'
  // sourceconfig_ego{i}=fuserSourceConfiguration(i);
  obj = &lobj_1[0];
  obj->pIsTransformToCentralValid = false;
  obj->pIsTransformToLocalValid = false;
  obj->SourceIndex = 1.0;
  obj->IsInternalSource = true;
  obj->IsInitializingCentralTracks = true;
  sourceconfig_ego[0] = obj;
  //  차량별 tracking source configuration
  // 'EDGE_fusion_function_231019_2222:20'
  // sourceconfig_ego{i}=fuserSourceConfiguration(i);
  b_obj = &lobj_1[1];
  b_obj->pIsTransformToCentralValid = false;
  b_obj->pIsTransformToLocalValid = false;
  b_obj->SourceIndex = 2.0;
  b_obj->IsInternalSource = true;
  b_obj->IsInitializingCentralTracks = true;
  sourceconfig_ego[1] = b_obj;
  //  차량별 tracking source configuration
  // 'EDGE_fusion_function_231019_2222:20'
  // sourceconfig_ego{i}=fuserSourceConfiguration(i);
  b_obj = &lobj_1[2];
  b_obj->pIsTransformToCentralValid = false;
  b_obj->pIsTransformToLocalValid = false;
  b_obj->SourceIndex = 3.0;
  b_obj->IsInternalSource = true;
  b_obj->IsInitializingCentralTracks = true;
  sourceconfig_ego[2] = b_obj;
  //  차량별 tracking source configuration
  // 'EDGE_fusion_function_231019_2222:20'
  // sourceconfig_ego{i}=fuserSourceConfiguration(i);
  b_obj = &lobj_1[3];
  b_obj->pIsTransformToCentralValid = false;
  b_obj->pIsTransformToLocalValid = false;
  b_obj->SourceIndex = 4.0;
  b_obj->IsInternalSource = true;
  b_obj->IsInitializingCentralTracks = true;
  sourceconfig_ego[3] = b_obj;
  //  차량별 tracking source configuration
  // 'EDGE_fusion_function_231019_2222:20'
  // sourceconfig_ego{i}=fuserSourceConfiguration(i);
  b_obj = &lobj_1[4];
  b_obj->pIsTransformToCentralValid = false;
  b_obj->pIsTransformToLocalValid = false;
  b_obj->SourceIndex = 5.0;
  b_obj->IsInternalSource = true;
  b_obj->IsInitializingCentralTracks = true;
  sourceconfig_ego[4] = b_obj;
  // 'EDGE_fusion_function_231019_2222:23' fuser =
  // trackFuser('SourceConfigurations',sourceconfig_ego,...  % 인지 융합 함수
  // configuration 'EDGE_fusion_function_231019_2222:24'
  // 'HasAdditiveProcessNoise',false,... 'EDGE_fusion_function_231019_2222:25'
  // 'AssignmentThreshold',[5 5],... 'EDGE_fusion_function_231019_2222:26'
  // 'ConfirmationThreshold',[5 5],... 'EDGE_fusion_function_231019_2222:27'
  // 'DeletionThreshold',[5 5],... 'EDGE_fusion_function_231019_2222:28'
  // 'Assignment','MatchPairs','StateFusion','Intersection');
  for (b_track_ego = 0; b_track_ego < 9; b_track_ego++) {
    fuser.ProcessNoise[b_track_ego] = b_iv[b_track_ego];
  }
  fuser.isInitialized = 0;
  b_obj = &fuser._pobj1[0];
  flag = (fuser.isInitialized == 1);
  if (flag) {
    fuser.TunablePropsChanged = true;
  }
  fuser.pSourceConfigurations[0] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[0]);
  fuser.pSourceConfigurations[1] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[1]);
  fuser.pSourceConfigurations[2] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[2]);
  fuser.pSourceConfigurations[3] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[3]);
  fuser.pSourceConfigurations[4] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[4]);
  fuser.pSourceConfigurations[5] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[5]);
  fuser.pSourceConfigurations[6] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[6]);
  fuser.pSourceConfigurations[7] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[7]);
  fuser.pSourceConfigurations[8] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[8]);
  fuser.pSourceConfigurations[9] = obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[9]);
  fuser.pSourceConfigurations[10] =
      obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[10]);
  fuser.pSourceConfigurations[11] =
      obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[11]);
  fuser.pSourceConfigurations[12] =
      obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[12]);
  fuser.pSourceConfigurations[13] =
      obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[13]);
  fuser.pSourceConfigurations[14] =
      obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[14]);
  fuser.pSourceConfigurations[15] =
      obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[15]);
  fuser.pSourceConfigurations[16] =
      obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[16]);
  fuser.pSourceConfigurations[17] =
      obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[17]);
  fuser.pSourceConfigurations[18] =
      obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[18]);
  fuser.pSourceConfigurations[19] =
      obj->clone((&(&(&(&b_obj[0])[0])[0])[0])[19]);
  for (int i{0}; i < 5; i++) {
    fuser.pSourceConfigurations[i] = sourceconfig_ego[i];
  }
  fuser.pNumUsedConfigs = 5.0;
  std::memset(&ids[0], 0, 20U * sizeof(double));
  d = fuser.pNumUsedConfigs;
  b_track_ego = static_cast<int>(d);
  for (int i{0}; i < b_track_ego; i++) {
    ids[i] = fuser.pSourceConfigurations[i]->SourceIndex;
  }
  std::copy(&ids[0], &ids[20], &fuser.pSourceConfigIDs[0]);
  for (b_track_ego = 0; b_track_ego < 20; b_track_ego++) {
    fuser.pIsValidSource[b_track_ego] = false;
  }
  flag = (fuser.isInitialized == 1);
  if (flag) {
    fuser.TunablePropsChanged = true;
  }
  for (int i{0}; i < 5; i++) {
    fuser.pSourceConfigurations[i] = sourceconfig_ego[i];
  }
  fuser.pNumUsedConfigs = 5.0;
  std::memset(&ids[0], 0, 20U * sizeof(double));
  d = fuser.pNumUsedConfigs;
  b_track_ego = static_cast<int>(d);
  for (int i{0}; i < b_track_ego; i++) {
    ids[i] = fuser.pSourceConfigurations[i]->SourceIndex;
  }
  for (b_track_ego = 0; b_track_ego < 20; b_track_ego++) {
    fuser.pSourceConfigIDs[b_track_ego] = ids[b_track_ego];
    fuser.pIsValidSource[b_track_ego] = false;
  }
  fuser.matlabCodegenIsDeleted = false;
  //   % 인지 융합 함수 configuration
  // 'EDGE_fusion_function_231019_2222:30' for i = 1 : numveh
  // 'EDGE_fusion_function_231019_2222:34' idx=numobs>0;
  //  장애물 인지한 차량 인덱스
  // 'EDGE_fusion_function_231019_2222:35' numobs=numobs(idx);
  //  Obstacle state-space structure
  // 'EDGE_fusion_function_231019_2222:40' for i = 1 : numveh
  expl_temp.BranchID = 0U;
  expl_temp.Age = 1U;
  expl_temp.ObjectClassProbabilities = 1.0;
  expl_temp.TrackLogicState = true;
  expl_temp.IsConfirmed = true;
  expl_temp.IsCoasted = false;
  expl_temp.IsSelfReported = true;
  for (b_track_ego = 0; b_track_ego < 7; b_track_ego++) {
    expl_temp.TrackLogic[b_track_ego] = logicType[b_track_ego];
  }
  for (b_track_ego = 0; b_track_ego < 36; b_track_ego++) {
    expl_temp.StateCovariance[b_track_ego] = stateCov[b_track_ego];
  }
  for (int i{0}; i < 5; i++) {
    // 'EDGE_fusion_function_231019_2222:44' for j = 1 : numobs(i,1)
    expl_temp.SourceIndex = static_cast<unsigned int>(i + 1);
    // 'EDGE_fusion_function_231019_2222:48' obs_state =
    // [hub_data_Object.vehicle(i).obstacle(j).Position_x
    // 'EDGE_fusion_function_231019_2222:49'
    // hub_data_Object.vehicle(i).obstacle(j).Velocity_x
    // 'EDGE_fusion_function_231019_2222:50'
    // hub_data_Object.vehicle(i).obstacle(j).Position_y
    // 'EDGE_fusion_function_231019_2222:51'
    // hub_data_Object.vehicle(i).obstacle(j).Velocity_y
    // 'EDGE_fusion_function_231019_2222:52'
    // hub_data_Object.vehicle(i).obstacle(j).Position_z
    // 'EDGE_fusion_function_231019_2222:53'
    // hub_data_Object.vehicle(i).obstacle(j).Velocity_z];
    // 'EDGE_fusion_function_231019_2222:55'
    // temp_track=toStruct(objectTrack('State',obs_state, ...
    // 'EDGE_fusion_function_231019_2222:56' 'TrackID',(i-1)*maxobs+j, ...
    // 'EDGE_fusion_function_231019_2222:57' 'SourceIndex',i, ...
    // 'EDGE_fusion_function_231019_2222:58' 'StateCovariance',5*eye(6), ...
    // 'EDGE_fusion_function_231019_2222:59'
    // 'IsConfirmed',true,'UpdateTime',hub_data_Object.vehicle(i).obstacle(j).timestamp,
    // ... 'EDGE_fusion_function_231019_2222:60'
    // 'ObjectClassID',hub_data_Object.vehicle(i).obstacle(j).obstacle_class));
    expl_temp.State[0] = hub_data_Object->vehicle[i].obstacle[0].Position_x;
    expl_temp.State[1] = hub_data_Object->vehicle[i].obstacle[0].Velocity_x;
    expl_temp.State[2] = hub_data_Object->vehicle[i].obstacle[0].Position_y;
    expl_temp.State[3] = hub_data_Object->vehicle[i].obstacle[0].Velocity_y;
    expl_temp.State[4] = hub_data_Object->vehicle[i].obstacle[0].Position_z;
    expl_temp.State[5] = hub_data_Object->vehicle[i].obstacle[0].Velocity_z;
    // 'EDGE_fusion_function_231019_2222:61' track_ego(end+1)=temp_track;
    b_track_ego = track_ego.size(1) + 1;
    track_ego.set_size(1, track_ego.size(1) + 1);
    expl_temp.TrackID = static_cast<unsigned int>(i * 20 + 1);
    expl_temp.UpdateTime = hub_data_Object->vehicle[i].obstacle[0].timestamp;
    expl_temp.ObjectClassID =
        hub_data_Object->vehicle[i].obstacle[0].obstacle_class;
    track_ego[b_track_ego - 1] = expl_temp;
    // 'EDGE_fusion_function_231019_2222:48' obs_state =
    // [hub_data_Object.vehicle(i).obstacle(j).Position_x
    // 'EDGE_fusion_function_231019_2222:49'
    // hub_data_Object.vehicle(i).obstacle(j).Velocity_x
    // 'EDGE_fusion_function_231019_2222:50'
    // hub_data_Object.vehicle(i).obstacle(j).Position_y
    // 'EDGE_fusion_function_231019_2222:51'
    // hub_data_Object.vehicle(i).obstacle(j).Velocity_y
    // 'EDGE_fusion_function_231019_2222:52'
    // hub_data_Object.vehicle(i).obstacle(j).Position_z
    // 'EDGE_fusion_function_231019_2222:53'
    // hub_data_Object.vehicle(i).obstacle(j).Velocity_z];
    // 'EDGE_fusion_function_231019_2222:55'
    // temp_track=toStruct(objectTrack('State',obs_state, ...
    // 'EDGE_fusion_function_231019_2222:56' 'TrackID',(i-1)*maxobs+j, ...
    // 'EDGE_fusion_function_231019_2222:57' 'SourceIndex',i, ...
    // 'EDGE_fusion_function_231019_2222:58' 'StateCovariance',5*eye(6), ...
    // 'EDGE_fusion_function_231019_2222:59'
    // 'IsConfirmed',true,'UpdateTime',hub_data_Object.vehicle(i).obstacle(j).timestamp,
    // ... 'EDGE_fusion_function_231019_2222:60'
    // 'ObjectClassID',hub_data_Object.vehicle(i).obstacle(j).obstacle_class));
    expl_temp.State[0] = hub_data_Object->vehicle[i].obstacle[1].Position_x;
    expl_temp.State[1] = hub_data_Object->vehicle[i].obstacle[1].Velocity_x;
    expl_temp.State[2] = hub_data_Object->vehicle[i].obstacle[1].Position_y;
    expl_temp.State[3] = hub_data_Object->vehicle[i].obstacle[1].Velocity_y;
    expl_temp.State[4] = hub_data_Object->vehicle[i].obstacle[1].Position_z;
    expl_temp.State[5] = hub_data_Object->vehicle[i].obstacle[1].Velocity_z;
    // 'EDGE_fusion_function_231019_2222:61' track_ego(end+1)=temp_track;
    b_track_ego = track_ego.size(1) + 1;
    track_ego.set_size(1, track_ego.size(1) + 1);
    expl_temp.TrackID = static_cast<unsigned int>(i * 20 + 2);
    expl_temp.UpdateTime = hub_data_Object->vehicle[i].obstacle[1].timestamp;
    expl_temp.ObjectClassID =
        hub_data_Object->vehicle[i].obstacle[1].obstacle_class;
    track_ego[b_track_ego - 1] = expl_temp;
    // 'EDGE_fusion_function_231019_2222:48' obs_state =
    // [hub_data_Object.vehicle(i).obstacle(j).Position_x
    // 'EDGE_fusion_function_231019_2222:49'
    // hub_data_Object.vehicle(i).obstacle(j).Velocity_x
    // 'EDGE_fusion_function_231019_2222:50'
    // hub_data_Object.vehicle(i).obstacle(j).Position_y
    // 'EDGE_fusion_function_231019_2222:51'
    // hub_data_Object.vehicle(i).obstacle(j).Velocity_y
    // 'EDGE_fusion_function_231019_2222:52'
    // hub_data_Object.vehicle(i).obstacle(j).Position_z
    // 'EDGE_fusion_function_231019_2222:53'
    // hub_data_Object.vehicle(i).obstacle(j).Velocity_z];
    // 'EDGE_fusion_function_231019_2222:55'
    // temp_track=toStruct(objectTrack('State',obs_state, ...
    // 'EDGE_fusion_function_231019_2222:56' 'TrackID',(i-1)*maxobs+j, ...
    // 'EDGE_fusion_function_231019_2222:57' 'SourceIndex',i, ...
    // 'EDGE_fusion_function_231019_2222:58' 'StateCovariance',5*eye(6), ...
    // 'EDGE_fusion_function_231019_2222:59'
    // 'IsConfirmed',true,'UpdateTime',hub_data_Object.vehicle(i).obstacle(j).timestamp,
    // ... 'EDGE_fusion_function_231019_2222:60'
    // 'ObjectClassID',hub_data_Object.vehicle(i).obstacle(j).obstacle_class));
    expl_temp.State[0] = hub_data_Object->vehicle[i].obstacle[2].Position_x;
    expl_temp.State[1] = hub_data_Object->vehicle[i].obstacle[2].Velocity_x;
    expl_temp.State[2] = hub_data_Object->vehicle[i].obstacle[2].Position_y;
    expl_temp.State[3] = hub_data_Object->vehicle[i].obstacle[2].Velocity_y;
    expl_temp.State[4] = hub_data_Object->vehicle[i].obstacle[2].Position_z;
    expl_temp.State[5] = hub_data_Object->vehicle[i].obstacle[2].Velocity_z;
    // 'EDGE_fusion_function_231019_2222:61' track_ego(end+1)=temp_track;
    b_track_ego = track_ego.size(1) + 1;
    track_ego.set_size(1, track_ego.size(1) + 1);
    expl_temp.TrackID = static_cast<unsigned int>(i * 20 + 3);
    expl_temp.UpdateTime = hub_data_Object->vehicle[i].obstacle[2].timestamp;
    expl_temp.ObjectClassID =
        hub_data_Object->vehicle[i].obstacle[2].obstacle_class;
    track_ego[b_track_ego - 1] = expl_temp;
  }
  // 'EDGE_fusion_function_231019_2222:67'
  // [confirmedTracks,tentativeTracks,allTracks,analysisInformation]=fuser(track_ego,hub_data_Object.vehicle(1).obstacle(1).timestamp);
  confirmedTracks_size[0] = fuser.step(
      track_ego, hub_data_Object->vehicle[0].obstacle[0].timestamp,
      confirmedTracks_data, tentativeTracks_data, allTracks,
      analysisInformation->TrackIDsAtStepBeginning.data,
      analysisInformation->TrackIDsAtStepBeginning.size,
      analysisInformation->CostMatrix, analysisInformation->Assignments,
      analysisInformation->UnassignedCentralTracks,
      analysisInformation->UnassignedLocalTracks,
      analysisInformation->NonInitializingLocalTracks,
      analysisInformation->InitializedCentralTrackIDs.data,
      analysisInformation->InitializedCentralTrackIDs.size,
      analysisInformation->UpdatedCentralTrackIDs,
      analysisInformation->DeletedTrackIDs.data,
      analysisInformation->DeletedTrackIDs.size,
      analysisInformation->TrackIDsAtStepEnd.data,
      analysisInformation->TrackIDsAtStepEnd.size, tentativeTracks_size[0]);
}

//
// File trailer for EDGE_fusion_function_231019_2222.cpp
//
// [EOF]
//
