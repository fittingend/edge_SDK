//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: trackFuser.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

#ifndef TRACKFUSER_H
#define TRACKFUSER_H

// Include Files
#include "AssignerGNN.h"
#include "Fuserxcov.h"
#include "fuserSourceConfiguration.h"
#include "objectTrack.h"
#include "rtwtypes.h"
#include "trackHistoryLogic.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct struct_T;

struct struct3_T;

// Type Definitions
struct cell_wrap_3 {
  unsigned int f1[8];
};

namespace coder {
class trackFuser {
public:
  int step(const array<struct_T, 2U> &varargin_1, double varargin_2,
           struct3_T varargout_1_data[], struct3_T varargout_2_data[],
           array<struct3_T, 1U> &varargout_3,
           unsigned int varargout_4_TrackIDsAtStepBeginning_data[],
           int varargout_4_TrackIDsAtStepBeginning_size[2],
           array<double, 2U> &varargout_4_CostMatrix,
           array<unsigned int, 2U> &varargout_4_Assignments,
           array<unsigned int, 1U> &varargout_4_UnassignedCentralTracks,
           array<unsigned int, 1U> &varargout_4_UnassignedLocalTracks,
           array<unsigned int, 1U> &varargout_4_NonInitializingLocalTracks,
           unsigned int varargout_4_InitializedCentralTrackIDs_data[],
           int varargout_4_InitializedCentralTrackIDs_size[2],
           array<unsigned int, 2U> &varargout_4_UpdatedCentralTrackIDs,
           unsigned int varargout_4_DeletedTrackIDs_data[],
           int varargout_4_DeletedTrackIDs_size[2],
           unsigned int varargout_4_TrackIDsAtStepEnd_data[],
           int varargout_4_TrackIDsAtStepEnd_size[2], int &varargout_2_size);
  void matlabCodegenDestructor();
  void releaseWrapper();
  ~trackFuser();
  trackFuser();

protected:
  void setupImpl(const array<struct_T, 2U> &tracks);
  void resetImpl();
  int stepImpl(const array<struct_T, 2U> &localTracks, double tFusion,
               struct3_T confTracks_data[], struct3_T tentTracks_data[],
               array<struct3_T, 1U> &allTracks,
               unsigned int info_TrackIDsAtStepBeginning_data[],
               int info_TrackIDsAtStepBeginning_size[2],
               array<double, 2U> &info_CostMatrix,
               array<unsigned int, 2U> &info_Assignments,
               array<unsigned int, 1U> &info_UnassignedCentralTracks,
               array<unsigned int, 1U> &info_UnassignedLocalTracks,
               array<unsigned int, 1U> &info_NonInitializingLocalTracks,
               unsigned int info_InitializedCentralTrackIDs_data[],
               int info_InitializedCentralTrackIDs_size[2],
               array<unsigned int, 2U> &info_UpdatedCentralTrackIDs,
               unsigned int info_DeletedTrackIDs_data[],
               int info_DeletedTrackIDs_size[2],
               unsigned int info_TrackIDsAtStepEnd_data[],
               int info_TrackIDsAtStepEnd_size[2], int &tentTracks_size);
  void assign(const array<double, 2U> &costMatrix,
              array<unsigned int, 2U> &overallAssignments,
              array<unsigned int, 1U> &overallUnassignedCentralTracks,
              array<unsigned int, 1U> &overallUnassignedLocalTracks);
  void initializeCentralTracks(const array<struct_T, 2U> &localTracks,
                               array<unsigned int, 1U> &unassignedlocalTracks);
  static void ensureTrack(unsigned int in_TrackID, unsigned int in_SourceIndex,
                          double in_UpdateTime, const double in_State[6],
                          const double in_StateCovariance[36],
                          double in_ObjectClassID, const char in_TrackLogic[7],
                          objectTrack *out);
  void fuseAssigned(const array<struct_T, 2U> &localTracks,
                    const array<unsigned int, 2U> &assignments);
  boolean_T
  getSelfReporting(const array<struct_T, 2U> &localTracks,
                   const array<unsigned int, 1U> &assignedlocalTracks);
  void b_fuseAssigned(const array<struct_T, 2U> &localTracks,
                      const array<unsigned int, 2U> &assignments,
                      array<boolean_T, 2U> &updated);
  void coastUnassigned(const array<unsigned int, 1U> &unassignedTracks,
                       const array<double, 2U> &notUpdated,
                       boolean_T toDelete[100]);
  void distance(const array<struct_T, 2U> &localTracks,
                array<double, 2U> &costMatrix);

public:
  boolean_T matlabCodegenIsDeleted;
  int isInitialized;
  boolean_T TunablePropsChanged;
  double ProcessNoise[9];
  objectTrack pTracksList[100];
  trackHistoryLogic *pTrackLogics[100];
  double pSourceConfigIDs[20];
  fuserSourceConfiguration *pSourceConfigurations[20];
  double pNumUsedConfigs;
  boolean_T pIsValidSource[20];
  matlabshared::tracking::internal::fusion::AssignerGNN cAssigner;
  trackHistoryLogic _pobj0[100];
  fuserSourceConfiguration _pobj1[20];

protected:
  double pNumLiveTracks;
  unsigned int pTrackIDs[100];
  boolean_T pConfirmedTracks[100];
  array<boolean_T, 2U> pUsedConfigIDs;
  double pLastTimeStamp;
  unsigned int pLastTrackID;

private:
  boolean_T isSetupComplete;
  cell_wrap_3 inputVarSize[2];
  fusion::internal::Fuserxcov cFuser;
};

} // namespace coder

#endif
//
// File trailer for trackFuser.h
//
// [EOF]
//
