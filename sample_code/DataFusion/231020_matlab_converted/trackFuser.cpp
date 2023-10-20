//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: trackFuser.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "trackFuser.h"
#include "AssignerGNN.h"
#include "EDGE_fusion_function_231019_2222_data.h"
#include "EDGE_fusion_function_231019_2222_internal_types.h"
#include "EDGE_fusion_function_231019_2222_types.h"
#include "Fuserxcov.h"
#include "constvel.h"
#include "eml_setop.h"
#include "find.h"
#include "fuserSourceConfiguration.h"
#include "gaussEKFilter.h"
#include "matchpairs.h"
#include "mrdivide_helper.h"
#include "objectTrack.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "trackHistoryLogic.h"
#include "unique.h"
#include "xzgetrf.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Type Definitions
struct b_struct_T {
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
  boolean_T TrackLogicState[5];
  boolean_T IsConfirmed;
  boolean_T IsCoasted;
  boolean_T IsSelfReported;
};

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : const array<double, 2U> &costMatrix
//                array<unsigned int, 2U> &overallAssignments
//                array<unsigned int, 1U> &overallUnassignedCentralTracks
//                array<unsigned int, 1U> &overallUnassignedLocalTracks
// Return Type  : void
//
namespace coder {
void trackFuser::assign(const array<double, 2U> &costMatrix,
                        array<unsigned int, 2U> &overallAssignments,
                        array<unsigned int, 1U> &overallUnassignedCentralTracks,
                        array<unsigned int, 1U> &overallUnassignedLocalTracks)
{
  array<double, 2U> b_paddedCost;
  array<double, 2U> paddedCost;
  array<double, 2U> y;
  array<unsigned int, 2U> assignments;
  array<int, 2U> matchings;
  array<int, 1U> b_r;
  array<int, 1U> colToRow;
  array<int, 1U> r1;
  array<int, 1U> rowToCol;
  array<int, 1U> sourceIndex;
  array<unsigned int, 1U> unassignedCentralTracks;
  array<int, 1U> unassignedCols;
  array<int, 1U> unassignedRows;
  array<unsigned int, 1U> unassignedlocalTracks;
  array<boolean_T, 1U> obj;
  double lastAssigned;
  double lastUnassigned;
  double numCentralTrks;
  int eint;
  int i;
  int i1;
  int i2;
  int ix;
  int j;
  int n;
  int nz;
  int r;
  signed char tmp_data[20];
  boolean_T usedSources[20];
  numCentralTrks = pNumLiveTracks;
  for (i = 0; i < 20; i++) {
    usedSources[i] = false;
  }
  j = pUsedConfigIDs.size(0);
  i2 = 1;
  for (n = 0; n < 20; n++) {
    boolean_T exitg1;
    r = (i2 + j) - 1;
    ix = i2;
    i2 += pUsedConfigIDs.size(0);
    exitg1 = false;
    while ((!exitg1) && (ix <= r)) {
      if (pUsedConfigIDs[ix - 1]) {
        usedSources[n] = true;
        exitg1 = true;
      } else {
        ix++;
      }
    }
  }
  nz = usedSources[0];
  for (i2 = 0; i2 < 19; i2++) {
    nz += usedSources[i2 + 1];
  }
  i = static_cast<int>(numCentralTrks * static_cast<double>(nz));
  overallAssignments.set_size(i, 2);
  j = i << 1;
  for (i = 0; i < j; i++) {
    overallAssignments[i] = 0U;
  }
  overallUnassignedLocalTracks.set_size(costMatrix.size(1));
  j = costMatrix.size(1);
  for (i = 0; i < j; i++) {
    overallUnassignedLocalTracks[i] = 0U;
  }
  if (std::isnan(numCentralTrks)) {
    y.set_size(1, 1);
    y[0] = rtNaN;
  } else if (numCentralTrks < 1.0) {
    y.set_size(1, 0);
  } else {
    y.set_size(1, static_cast<int>(numCentralTrks - 1.0) + 1);
    j = static_cast<int>(numCentralTrks - 1.0);
    for (i = 0; i <= j; i++) {
      y[i] = static_cast<double>(i) + 1.0;
    }
  }
  overallUnassignedCentralTracks.set_size(y.size(1));
  j = y.size(1);
  for (i = 0; i < j; i++) {
    unsigned int u;
    numCentralTrks = y[i];
    if (numCentralTrks < 4.294967296E+9) {
      u = static_cast<unsigned int>(numCentralTrks);
    } else {
      u = 0U;
    }
    overallUnassignedCentralTracks[i] = u;
  }
  lastAssigned = 0.0;
  lastUnassigned = 0.0;
  for (int s{0}; s < nz; s++) {
    j = 0;
    for (n = 0; n < 20; n++) {
      if (usedSources[n]) {
        tmp_data[j] = static_cast<signed char>(n);
        j++;
      }
    }
    i = tmp_data[s];
    j = pUsedConfigIDs.size(0);
    obj.set_size(j);
    for (i1 = 0; i1 < j; i1++) {
      obj[i1] = pUsedConfigIDs[i1 + pUsedConfigIDs.size(0) * i];
    }
    eml_find(obj, rowToCol);
    sourceIndex.set_size(rowToCol.size(0));
    j = rowToCol.size(0);
    for (i = 0; i < j; i++) {
      sourceIndex[i] = rowToCol[i];
    }
    if (cAssigner.isInitialized != 1) {
      cAssigner.isInitialized = 1;
      cAssigner.isSetupComplete = true;
    }
    i = costMatrix.size(0);
    n = sourceIndex.size(0) - 1;
    i2 = costMatrix.size(0) + sourceIndex.size(0);
    paddedCost.set_size(i2, i2);
    ix = i2 * i2;
    for (i1 = 0; i1 < ix; i1++) {
      paddedCost[i1] = rtInf;
    }
    for (j = 0; j <= n; j++) {
      for (r = 0; r < i; r++) {
        paddedCost[r + paddedCost.size(0) * j] =
            costMatrix[r + costMatrix.size(0) * (sourceIndex[j] - 1)];
      }
    }
    for (j = 0; j < i; j++) {
      for (r = 0; r <= n; r++) {
        paddedCost[(i + r) + paddedCost.size(0) * ((n + j) + 1)] =
            costMatrix[j + costMatrix.size(0) * (sourceIndex[r] - 1)];
      }
    }
    for (r = 0; r < i; r++) {
      paddedCost[r + paddedCost.size(0) * ((n + r) + 1)] = 5.0;
    }
    for (j = 0; j <= n; j++) {
      paddedCost[(i + j) + paddedCost.size(0) * j] = 5.0;
    }
    perfectMatching(paddedCost, colToRow, rowToCol);
    if ((colToRow.size(0) == 0) && (i2 > 0)) {
      double maxVal;
      maxVal = 0.0;
      for (i2 = 0; i2 < ix; i2++) {
        numCentralTrks = std::abs(paddedCost[i2]);
        if ((numCentralTrks > maxVal) && (!std::isinf(numCentralTrks))) {
          maxVal = numCentralTrks;
        }
      }
      if (!std::isinf(maxVal)) {
        numCentralTrks = std::frexp(maxVal, &eint);
        maxVal = eint;
        if (numCentralTrks == 0.5) {
          maxVal = static_cast<double>(eint) - 1.0;
        }
      }
      numCentralTrks = rt_powd_snf(2.0, -maxVal);
      b_paddedCost.set_size(paddedCost.size(0), paddedCost.size(1));
      for (i1 = 0; i1 < ix; i1++) {
        b_paddedCost[i1] = paddedCost[i1] * numCentralTrks;
      }
      perfectMatching(b_paddedCost, colToRow, rowToCol);
    }
    i2 = 0;
    for (j = 0; j <= n; j++) {
      i1 = colToRow[j];
      r = colToRow[j];
      if ((colToRow[j] <= i) &&
          (costMatrix[(colToRow[j] +
                       costMatrix.size(0) * (sourceIndex[j] - 1)) -
                      1] == 5.0)) {
        ix = i + n;
        i1 = ix + 2;
        colToRow[j] = ix + 2;
        rowToCol[r - 1] = ix + 2;
      }
      if (i1 <= i) {
        i2++;
      }
    }
    matchings.set_size(i2, 2);
    j = -1;
    for (i2 = 0; i2 <= n; i2++) {
      if (colToRow[i2] <= i) {
        j++;
        matchings[j] = colToRow[i2];
        matchings[j + matchings.size(0)] = i2 + 1;
      }
    }
    i1 = costMatrix.size(0);
    unassignedRows.set_size(costMatrix.size(0));
    i2 = 0;
    for (r = 0; r < i1; r++) {
      if (rowToCol[r] > n + 1) {
        i2++;
        unassignedRows[i2 - 1] = r + 1;
      }
    }
    if (i2 < 1) {
      i2 = 0;
    }
    unassignedRows.set_size(i2);
    i1 = sourceIndex.size(0);
    unassignedCols.set_size(sourceIndex.size(0));
    i2 = 0;
    for (r = 0; r < i1; r++) {
      if (colToRow[r] > i) {
        i2++;
        unassignedCols[i2 - 1] = r + 1;
      }
    }
    if (i2 < 1) {
      i2 = 0;
    }
    unassignedCols.set_size(i2);
    assignments.set_size(matchings.size(0), 2);
    j = matchings.size(0) << 1;
    for (i = 0; i < j; i++) {
      i1 = matchings[i];
      if (i1 < 0) {
        i1 = 0;
      }
      assignments[i] = static_cast<unsigned int>(i1);
    }
    unassignedCentralTracks.set_size(unassignedRows.size(0));
    j = unassignedRows.size(0);
    for (i = 0; i < j; i++) {
      unassignedCentralTracks[i] = static_cast<unsigned int>(unassignedRows[i]);
    }
    unassignedlocalTracks.set_size(unassignedCols.size(0));
    j = unassignedCols.size(0);
    for (i = 0; i < j; i++) {
      unassignedlocalTracks[i] = static_cast<unsigned int>(unassignedCols[i]);
    }
    if (assignments.size(0) != 0) {
      numCentralTrks = lastAssigned + static_cast<double>(assignments.size(0));
      if (lastAssigned + 1.0 > numCentralTrks) {
        i = 1;
      } else {
        i = static_cast<int>(lastAssigned + 1.0);
      }
      j = assignments.size(0);
      for (i1 = 0; i1 < j; i1++) {
        overallAssignments[(i + i1) - 1] = assignments[i1];
      }
      if (lastAssigned + 1.0 > numCentralTrks) {
        i = 1;
      } else {
        i = static_cast<int>(lastAssigned + 1.0);
      }
      j = assignments.size(0);
      for (i1 = 0; i1 < j; i1++) {
        overallAssignments[((i + i1) + overallAssignments.size(0)) - 1] =
            static_cast<unsigned int>(
                sourceIndex[static_cast<int>(
                                assignments[i1 + assignments.size(0)]) -
                            1]);
      }
      lastAssigned += static_cast<double>(assignments.size(0));
    }
    if (unassignedlocalTracks.size(0) != 0) {
      if (lastUnassigned + 1.0 >
          lastUnassigned + static_cast<double>(unassignedlocalTracks.size(0))) {
        i = 1;
      } else {
        i = static_cast<int>(lastUnassigned + 1.0);
      }
      j = unassignedlocalTracks.size(0);
      for (i1 = 0; i1 < j; i1++) {
        overallUnassignedLocalTracks[(i + i1) - 1] = static_cast<unsigned int>(
            sourceIndex[static_cast<int>(unassignedlocalTracks[i1]) - 1]);
      }
      lastUnassigned += static_cast<double>(unassignedlocalTracks.size(0));
    }
    if (unassignedCentralTracks.size(0) != 0) {
      internal::sort(unassignedCentralTracks);
      unassignedlocalTracks.set_size(overallUnassignedCentralTracks.size(0));
      j = overallUnassignedCentralTracks.size(0) - 1;
      for (i = 0; i <= j; i++) {
        unassignedlocalTracks[i] = overallUnassignedCentralTracks[i];
      }
      do_vectors(unassignedlocalTracks, unassignedCentralTracks,
                 overallUnassignedCentralTracks, rowToCol, unassignedCols);
    } else {
      overallUnassignedCentralTracks.set_size(0);
    }
  }
  i2 = overallUnassignedLocalTracks.size(0) - 1;
  j = 0;
  for (n = 0; n <= i2; n++) {
    if (static_cast<int>(overallUnassignedLocalTracks[n]) > 0) {
      j++;
    }
  }
  b_r.set_size(j);
  j = 0;
  for (n = 0; n <= i2; n++) {
    if (static_cast<int>(overallUnassignedLocalTracks[n]) > 0) {
      b_r[j] = n;
      j++;
    }
  }
  unassignedCentralTracks.set_size(b_r.size(0));
  j = b_r.size(0);
  for (i = 0; i < j; i++) {
    unassignedCentralTracks[i] = overallUnassignedLocalTracks[b_r[i]];
  }
  unique_vector(unassignedCentralTracks, overallUnassignedLocalTracks);
  obj.set_size(overallAssignments.size(0));
  j = overallAssignments.size(0);
  for (i = 0; i < j; i++) {
    obj[i] = (static_cast<int>(overallAssignments[i]) > 0);
  }
  i2 = obj.size(0) - 1;
  j = 0;
  for (n = 0; n <= i2; n++) {
    if (obj[n]) {
      j++;
    }
  }
  r1.set_size(j);
  j = 0;
  for (n = 0; n <= i2; n++) {
    if (obj[n]) {
      r1[j] = n;
      j++;
    }
  }
  assignments.set_size(r1.size(0), 2);
  j = r1.size(0);
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < j; i1++) {
      assignments[i1 + assignments.size(0) * i] =
          overallAssignments[r1[i1] + overallAssignments.size(0) * i];
    }
  }
  overallAssignments.set_size(assignments.size(0), 2);
  j = assignments.size(0) << 1;
  for (i = 0; i < j; i++) {
    overallAssignments[i] = assignments[i];
  }
}

//
// Arguments    : const array<struct_T, 2U> &localTracks
//                const array<unsigned int, 2U> &assignments
//                array<boolean_T, 2U> &updated
// Return Type  : void
//
void trackFuser::b_fuseAssigned(const array<struct_T, 2U> &localTracks,
                                const array<unsigned int, 2U> &assignments,
                                array<boolean_T, 2U> &updated)
{
  fuserSourceConfiguration *thisConfig;
  objectTrack obj;
  trackHistoryLogic *b_obj;
  array<objectTrack, 2U> transformedTracks;
  array<double, 1U> b_assignedSourceInds;
  array<int, 1U> assignedSourceInds;
  array<unsigned int, 1U> b_assignments;
  array<int, 1U> r;
  array<int, 1U> r1;
  array<unsigned int, 1U> uniqueAssigned;
  array<boolean_T, 1U> inAssigned;
  array<boolean_T, 1U> toFuse;
  int c_i;
  int i;
  int k;
  int ntilecols;
  int nz;
  boolean_T exitg1;
  obj = pTracksList[0];
  transformedTracks.set_size(1, assignments.size(0));
  if (assignments.size(0) != 0) {
    ntilecols = assignments.size(0);
    for (nz = 0; nz < ntilecols; nz++) {
      transformedTracks[nz] = obj;
    }
  }
  i = assignments.size(0);
  for (int b_i{0}; b_i < i; b_i++) {
    struct_T expl_temp;
    boolean_T inKnownIDs[20];
    expl_temp =
        localTracks[static_cast<int>(assignments[b_i + assignments.size(0)]) -
                    1];
    for (k = 0; k < 20; k++) {
      inKnownIDs[k] = (expl_temp.SourceIndex == pSourceConfigIDs[k]);
    }
    c_i = 0;
    exitg1 = false;
    while ((!exitg1) && (c_i < 20)) {
      if (inKnownIDs[c_i]) {
        thisConfig = pSourceConfigurations[c_i];
        exitg1 = true;
      } else {
        c_i++;
      }
    }
    if (!thisConfig->pIsTransformToCentralValid) {
      thisConfig->pIsTransformToCentralValid = true;
    }
    trackFuser::ensureTrack(expl_temp.TrackID, expl_temp.SourceIndex,
                            expl_temp.UpdateTime, expl_temp.State,
                            expl_temp.StateCovariance, expl_temp.ObjectClassID,
                            expl_temp.TrackLogic, &transformedTracks[b_i]);
  }
  b_assignments.set_size(assignments.size(0));
  ntilecols = assignments.size(0);
  for (i = 0; i < ntilecols; i++) {
    b_assignments[i] = assignments[i];
  }
  unique_vector(b_assignments, uniqueAssigned);
  updated.set_size(1, uniqueAssigned.size(0));
  ntilecols = uniqueAssigned.size(0);
  for (i = 0; i < ntilecols; i++) {
    updated[i] = false;
  }
  i = uniqueAssigned.size(0);
  for (int b_i{0}; b_i < i; b_i++) {
    boolean_T y;
    ntilecols = assignments.size(0);
    inAssigned.set_size(assignments.size(0));
    for (k = 0; k < ntilecols; k++) {
      inAssigned[k] = (uniqueAssigned[b_i] == assignments[k]);
    }
    eml_find(inAssigned, assignedSourceInds);
    toFuse.set_size(assignedSourceInds.size(0));
    k = assignedSourceInds.size(0);
    for (c_i = 0; c_i < k; c_i++) {
      if ((transformedTracks[assignedSourceInds[c_i] - 1].IsConfirmed &&
           (!transformedTracks[assignedSourceInds[c_i] - 1].IsCoasted)) ||
          transformedTracks[assignedSourceInds[c_i] - 1].IsSelfReported) {
        toFuse[c_i] = true;
      } else {
        toFuse[c_i] = false;
      }
    }
    y = false;
    ntilecols = 1;
    exitg1 = false;
    while ((!exitg1) && (ntilecols <= toFuse.size(0))) {
      if (toFuse[ntilecols - 1]) {
        y = true;
        exitg1 = true;
      } else {
        ntilecols++;
      }
    }
    if (y) {
      double id;
      int obj_tmp_tmp;
      unsigned int u;
      ntilecols = toFuse.size(0);
      if (toFuse.size(0) == 0) {
        nz = 0;
      } else {
        nz = toFuse[0];
        for (k = 2; k <= ntilecols; k++) {
          nz += toFuse[k - 1];
        }
      }
      obj_tmp_tmp = static_cast<int>(uniqueAssigned[b_i]) - 1;
      obj = pTracksList[obj_tmp_tmp];
      id = static_cast<double>(pTracksList[obj_tmp_tmp].Age) +
           static_cast<double>(nz);
      if (id < 4.294967296E+9) {
        if (id >= 0.0) {
          u = static_cast<unsigned int>(id);
        } else {
          u = 0U;
        }
      } else {
        u = MAX_uint32_T;
      }
      obj.Age = u;
      pTracksList[obj_tmp_tmp] = obj;
      nz = inAssigned.size(0) - 1;
      ntilecols = 0;
      for (c_i = 0; c_i <= nz; c_i++) {
        if (inAssigned[c_i]) {
          ntilecols++;
        }
      }
      r.set_size(ntilecols);
      ntilecols = 0;
      for (c_i = 0; c_i <= nz; c_i++) {
        if (inAssigned[c_i]) {
          r[ntilecols] = c_i;
          ntilecols++;
        }
      }
      id = pTracksList[obj_tmp_tmp].ObjectClassID;
      ntilecols = 0;
      while ((id == 0.0) && (ntilecols < r.size(0))) {
        ntilecols++;
        id = localTracks[static_cast<int>(assignments[r[ntilecols - 1] +
                                                      assignments.size(0)]) -
                         1]
                 .ObjectClassID;
      }
      obj = pTracksList[obj_tmp_tmp];
      obj.ObjectClassID = id;
      pTracksList[obj_tmp_tmp] = obj;
      obj = pTracksList[obj_tmp_tmp];
      nz = toFuse.size(0) - 1;
      ntilecols = 0;
      for (c_i = 0; c_i <= nz; c_i++) {
        if (toFuse[c_i]) {
          ntilecols++;
        }
      }
      r1.set_size(ntilecols);
      ntilecols = 0;
      for (c_i = 0; c_i <= nz; c_i++) {
        if (toFuse[c_i]) {
          r1[ntilecols] = c_i;
          ntilecols++;
        }
      }
      ntilecols = r1.size(0);
      b_assignedSourceInds.set_size(r1.size(0));
      for (k = 0; k < ntilecols; k++) {
        b_assignedSourceInds[k] = assignedSourceInds[r1[k]];
      }
      cFuser.fuse(obj, transformedTracks, b_assignedSourceInds);
      pTracksList[obj_tmp_tmp] = obj;
      ntilecols = toFuse.size(0);
      if (toFuse.size(0) == 0) {
        nz = 0;
      } else {
        nz = toFuse[0];
        for (k = 2; k <= ntilecols; k++) {
          nz += toFuse[k - 1];
        }
      }
      for (ntilecols = 0; ntilecols < nz; ntilecols++) {
        boolean_T bv[50];
        b_obj = pTrackLogics[obj_tmp_tmp];
        bv[0] = true;
        for (k = 0; k < 49; k++) {
          bv[k + 1] = b_obj->pRecentHistory[k];
        }
        for (k = 0; k < 50; k++) {
          b_obj->pRecentHistory[k] = bv[k];
        }
        b_obj->pIsFirstUpdate = false;
      }
      if (pTracksList[obj_tmp_tmp].IsConfirmed) {
        y = true;
      } else {
        b_obj = pTrackLogics[obj_tmp_tmp];
        if (b_obj->pIsFirstUpdate) {
          y = false;
        } else {
          boolean_T x[5];
          for (k = 0; k < 5; k++) {
            x[k] = b_obj->pRecentHistory[k];
          }
          y = ((((x[0] + x[1]) + x[2]) + x[3]) + x[4] >= 5);
        }
        if (y || (pTracksList[obj_tmp_tmp].ObjectClassID > 0.0)) {
          y = true;
        } else {
          y = false;
        }
      }
      obj = pTracksList[obj_tmp_tmp];
      obj.IsConfirmed = y;
      pTracksList[obj_tmp_tmp] = obj;
      pConfirmedTracks[obj_tmp_tmp] = pTracksList[obj_tmp_tmp].IsConfirmed;
      obj = pTracksList[obj_tmp_tmp];
      obj.IsCoasted = false;
      pTracksList[obj_tmp_tmp] = obj;
      obj = pTracksList[obj_tmp_tmp];
      ntilecols = r.size(0);
      b_assignments.set_size(r.size(0));
      for (k = 0; k < ntilecols; k++) {
        b_assignments[k] = assignments[r[k] + assignments.size(0)];
      }
      obj.IsSelfReported = getSelfReporting(localTracks, b_assignments);
      pTracksList[obj_tmp_tmp] = obj;
      obj = pTracksList[obj_tmp_tmp];
      pTracksList[obj_tmp_tmp] = obj;
      obj = pTracksList[obj_tmp_tmp];
      pTracksList[obj_tmp_tmp] = obj;
      updated[b_i] = true;
    }
  }
}

//
// Arguments    : const array<unsigned int, 1U> &unassignedTracks
//                const array<double, 2U> &notUpdated
//                boolean_T toDelete[100]
// Return Type  : void
//
void trackFuser::coastUnassigned(
    const array<unsigned int, 1U> &unassignedTracks,
    const array<double, 2U> &notUpdated, boolean_T toDelete[100])
{
  objectTrack temp;
  trackHistoryLogic *tempLogic;
  array<unsigned int, 2U> a;
  array<unsigned int, 2U> b_unassignedTracks;
  array<int, 2U> idx;
  array<int, 1U> iwork;
  array<boolean_T, 2U> c_x;
  double d;
  int b_i;
  int i;
  int i1;
  int j;
  int k;
  int n;
  int na;
  int nb;
  int nz;
  int p;
  int pEnd;
  int qEnd;
  unsigned int u;
  unsigned int x;
  signed char ii_data[100];
  boolean_T exitg1;
  a.set_size(1, unassignedTracks.size(0) + notUpdated.size(1));
  pEnd = unassignedTracks.size(0);
  for (i = 0; i < pEnd; i++) {
    a[i] = unassignedTracks[i];
  }
  pEnd = notUpdated.size(1);
  for (i = 0; i < pEnd; i++) {
    d = std::round(notUpdated[i]);
    if (d < 4.294967296E+9) {
      if (d >= 0.0) {
        u = static_cast<unsigned int>(d);
      } else {
        u = 0U;
      }
    } else if (d >= 4.294967296E+9) {
      u = MAX_uint32_T;
    } else {
      u = 0U;
    }
    a[i + unassignedTracks.size(0)] = u;
  }
  na = a.size(1);
  n = a.size(1) + 1;
  idx.set_size(1, a.size(1));
  pEnd = a.size(1);
  for (i = 0; i < pEnd; i++) {
    idx[i] = 0;
  }
  if (a.size(1) != 0) {
    iwork.set_size(a.size(1));
    i = a.size(1) - 1;
    for (k = 1; k <= i; k += 2) {
      if (a[k - 1] <= a[k]) {
        idx[k - 1] = k;
        idx[k] = k + 1;
      } else {
        idx[k - 1] = k + 1;
        idx[k] = k;
      }
    }
    if ((a.size(1) & 1) != 0) {
      idx[a.size(1) - 1] = a.size(1);
    }
    b_i = 2;
    while (b_i < n - 1) {
      nb = b_i << 1;
      j = 1;
      for (pEnd = b_i + 1; pEnd < n; pEnd = qEnd + b_i) {
        int kEnd;
        p = j;
        nz = pEnd;
        qEnd = j + nb;
        if (qEnd > n) {
          qEnd = n;
        }
        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          i = idx[p - 1];
          i1 = idx[nz - 1];
          if (a[i - 1] <= a[i1 - 1]) {
            iwork[k] = i;
            p++;
            if (p == pEnd) {
              while (nz < qEnd) {
                k++;
                iwork[k] = idx[nz - 1];
                nz++;
              }
            }
          } else {
            iwork[k] = i1;
            nz++;
            if (nz == qEnd) {
              while (p < pEnd) {
                k++;
                iwork[k] = idx[p - 1];
                p++;
              }
            }
          }
          k++;
        }
        for (k = 0; k < kEnd; k++) {
          idx[(j + k) - 1] = iwork[k];
        }
        j = qEnd;
      }
      b_i = nb;
    }
  }
  b_unassignedTracks.set_size(1, a.size(1));
  for (k = 0; k < na; k++) {
    b_unassignedTracks[k] = a[idx[k] - 1];
  }
  nb = 0;
  k = 1;
  while (k <= na) {
    x = b_unassignedTracks[k - 1];
    do {
      k++;
    } while (!((k > na) || (b_unassignedTracks[k - 1] != x)));
    nb++;
    b_unassignedTracks[nb - 1] = x;
  }
  if (nb < 1) {
    nb = 0;
  }
  b_unassignedTracks.set_size(b_unassignedTracks.size(0), nb);
  std::memset(&toDelete[0], 0, 100U * sizeof(boolean_T));
  i = b_unassignedTracks.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    unsigned int qY;
    boolean_T bv[50];
    boolean_T tentativeTrack;
    u = b_unassignedTracks[b_i];
    temp = pTracksList[static_cast<int>(u) - 1];
    x = pTracksList[static_cast<int>(u) - 1].Age;
    qY = x + 1U;
    if (x + 1U < x) {
      qY = MAX_uint32_T;
    }
    temp.Age = qY;
    pTracksList[static_cast<int>(u) - 1] = temp;
    temp = pTracksList[static_cast<int>(u) - 1];
    temp.IsCoasted = true;
    pTracksList[static_cast<int>(u) - 1] = temp;
    tempLogic = pTrackLogics[static_cast<int>(u) - 1];
    bv[0] = false;
    for (i1 = 0; i1 < 49; i1++) {
      bv[i1 + 1] = tempLogic->pRecentHistory[i1];
    }
    for (i1 = 0; i1 < 50; i1++) {
      tempLogic->pRecentHistory[i1] = bv[i1];
    }
    tempLogic->pIsFirstUpdate = false;
    tempLogic = pTrackLogics[static_cast<int>(u) - 1];
    tentativeTrack = !pTracksList[static_cast<int>(u) - 1].IsConfirmed;
    x = pTracksList[static_cast<int>(u) - 1].Age;
    if (tempLogic->pIsFirstUpdate) {
      toDelete[static_cast<int>(u) - 1] = false;
    } else if (!tentativeTrack) {
      if (x > 5U) {
        boolean_T b_x[5];
        for (i1 = 0; i1 < 5; i1++) {
          b_x[i1] = !tempLogic->pRecentHistory[i1];
        }
        toDelete[static_cast<int>(u) - 1] =
            ((((b_x[0] + b_x[1]) + b_x[2]) + b_x[3]) + b_x[4] >= 5);
      } else {
        if (x < 1U) {
          pEnd = 0;
        } else {
          pEnd = static_cast<int>(x);
        }
        c_x.set_size(1, pEnd);
        for (i1 = 0; i1 < pEnd; i1++) {
          c_x[i1] = !tempLogic->pRecentHistory[i1];
        }
        nb = c_x.size(1);
        if (c_x.size(1) == 0) {
          nz = 0;
        } else {
          nz = c_x[0];
          for (k = 2; k <= nb; k++) {
            nz += c_x[k - 1];
          }
        }
        if (nz < 0) {
          nz = 0;
        }
        toDelete[static_cast<int>(u) - 1] =
            (static_cast<unsigned int>(nz) >= 5U);
      }
    } else {
      boolean_T b_x[5];
      for (i1 = 0; i1 < 5; i1++) {
        b_x[i1] = tempLogic->pRecentHistory[i1];
      }
      qY = 5U - x;
      if (5U - x > 5U) {
        qY = 0U;
      }
      toDelete[static_cast<int>(u) - 1] =
          (5U - static_cast<unsigned int>(
                    (((b_x[0] + b_x[1]) + b_x[2]) + b_x[3]) + b_x[4]) >
           qY);
    }
    if (toDelete[static_cast<int>(u) - 1]) {
      tempLogic = pTrackLogics[static_cast<int>(u) - 1];
      for (i1 = 0; i1 < 50; i1++) {
        tempLogic->pRecentHistory[i1] = false;
      }
      tempLogic->pIsFirstUpdate = true;
    }
  }
  nz = toDelete[0] - 1;
  for (k = 0; k < 99; k++) {
    nz += toDelete[k + 1];
  }
  nb = 0;
  pEnd = 0;
  exitg1 = false;
  while ((!exitg1) && (pEnd < 100)) {
    if (toDelete[pEnd]) {
      nb++;
      ii_data[nb - 1] = static_cast<signed char>(pEnd + 1);
      if (nb >= 100) {
        exitg1 = true;
      } else {
        pEnd++;
      }
    } else {
      pEnd++;
    }
  }
  for (b_i = 0; b_i <= nz; b_i++) {
    unsigned int b_tmp_data[101];
    signed char i2;
    boolean_T tmp_data[101];
    nb = nz - b_i;
    i2 = ii_data[nb];
    temp = pTracksList[i2 - 1];
    tempLogic = pTrackLogics[i2 - 1];
    d = pNumLiveTracks;
    i = static_cast<int>(d + (1.0 - (static_cast<double>(i2) + 1.0)));
    for (j = 0; j < i; j++) {
      x = (static_cast<unsigned int>(i2) + static_cast<unsigned int>(j)) + 1U;
      pTracksList[static_cast<int>(static_cast<double>(x) - 1.0) - 1] =
          pTracksList[static_cast<int>(x) - 1];
      pTrackLogics[static_cast<int>(static_cast<double>(x) - 1.0) - 1] =
          pTrackLogics[static_cast<int>(x) - 1];
    }
    pTracksList[static_cast<int>(pNumLiveTracks) - 1] = temp;
    pTrackLogics[static_cast<int>(pNumLiveTracks) - 1] = tempLogic;
    pNumLiveTracks--;
    i = ii_data[nb] + 1;
    if (i > 100) {
      i1 = 0;
      p = 0;
    } else {
      i1 = i2;
      p = 100;
    }
    pEnd = p - i1;
    nb = pEnd + 1;
    for (p = 0; p < pEnd; p++) {
      tmp_data[p] = pConfirmedTracks[i1 + p];
    }
    tmp_data[pEnd] = false;
    for (i1 = 0; i1 < nb; i1++) {
      pConfirmedTracks[(i2 + i1) - 1] = tmp_data[i1];
    }
    if (i > 100) {
      i = 0;
      i1 = 0;
    } else {
      i = i2;
      i1 = 100;
    }
    pEnd = i1 - i;
    nb = pEnd + 1;
    for (i1 = 0; i1 < pEnd; i1++) {
      b_tmp_data[i1] = pTrackIDs[i + i1];
    }
    b_tmp_data[pEnd] = 0U;
    for (i = 0; i < nb; i++) {
      pTrackIDs[(i2 + i) - 1] = b_tmp_data[i];
    }
  }
}

//
// Arguments    : const array<struct_T, 2U> &localTracks
//                array<double, 2U> &costMatrix
// Return Type  : void
//
void trackFuser::distance(const array<struct_T, 2U> &localTracks,
                          array<double, 2U> &costMatrix)
{
  fuserSourceConfiguration *sourceConfig;
  array<objectTrack, 2U> centralTracks;
  array<double, 3U> S;
  array<double, 3U> allCovars;
  array<double, 3U> centralCovars;
  array<double, 3U> transformedCovars;
  array<double, 2U> allStates;
  array<double, 2U> c;
  array<double, 2U> centralStates;
  array<double, 2U> e;
  array<double, 2U> oneColumnCost;
  array<double, 2U> r;
  array<double, 2U> transformedStates;
  array<int, 2U> r1;
  array<unsigned int, 2U> trackClasses;
  double F[36];
  double numCentralTracks;
  double z_tmp;
  int i;
  int i1;
  int loop_ub_tmp;
  numCentralTracks = pNumLiveTracks;
  i = static_cast<int>(numCentralTracks);
  costMatrix.set_size(i, localTracks.size(1));
  loop_ub_tmp = static_cast<int>(numCentralTracks) * localTracks.size(1);
  for (i1 = 0; i1 < loop_ub_tmp; i1++) {
    costMatrix[i1] = rtInf;
  }
  if (!(pNumLiveTracks == 0.0)) {
    objectTrack obj;
    double d;
    int b_csz_idx_1;
    int b_loop_ub_tmp;
    int csz_idx_1;
    int i2;
    int ibmat;
    int ibtile;
    int jtilecol;
    int loop_ub;
    obj = pTracksList[0];
    centralTracks.set_size(1, i);
    if (static_cast<int>(numCentralTracks) != 0) {
      for (jtilecol = 0; jtilecol < i; jtilecol++) {
        centralTracks[jtilecol] = obj;
      }
    }
    trackClasses.set_size(1, i);
    for (int b_i{0}; b_i < i; b_i++) {
      unsigned int u;
      centralTracks[b_i] = pTracksList[b_i];
      d = std::round(pTracksList[b_i].ObjectClassID);
      if (d < 4.294967296E+9) {
        if (d >= 0.0) {
          u = static_cast<unsigned int>(d);
        } else {
          u = 0U;
        }
      } else if (d >= 4.294967296E+9) {
        u = MAX_uint32_T;
      } else {
        u = 0U;
      }
      trackClasses[b_i] = u;
    }
    centralStates.set_size(6, centralTracks.size(1));
    centralCovars.set_size(6, 6, centralTracks.size(1));
    i1 = centralTracks.size(1);
    for (int b_i{0}; b_i < i1; b_i++) {
      for (i2 = 0; i2 < 6; i2++) {
        centralStates[i2 + 6 * b_i] = centralTracks[b_i].pState[i2];
        for (ibmat = 0; ibmat < 6; ibmat++) {
          centralCovars[(ibmat + 6 * i2) + 36 * b_i] =
              centralTracks[b_i].pStateCovariance[ibmat + 6 * i2];
        }
      }
    }
    obj = pTracksList[0];
    i1 = localTracks.size(1);
    costMatrix.set_size(i, localTracks.size(1));
    b_loop_ub_tmp = 36 * centralCovars.size(2);
    loop_ub = 6 * centralStates.size(1);
    csz_idx_1 = centralStates.size(1);
    b_csz_idx_1 = centralStates.size(1);
    i2 = centralStates.size(1);
    for (int b_i{0}; b_i < i1; b_i++) {
      double Q[9];
      double imvec[6];
      double a_tmp;
      double b_imvec;
      double varargin_1;
      int acoef;
      int iacol;
      boolean_T inKnownIDs[20];
      boolean_T exitg1;
      for (ibmat = 0; ibmat < 9; ibmat++) {
        Q[ibmat] = ProcessNoise[ibmat];
      }
      varargin_1 = localTracks[b_i].UpdateTime - obj.pUpdateTime;
      S.set_size(6, 6, centralCovars.size(2));
      for (ibmat = 0; ibmat < b_loop_ub_tmp; ibmat++) {
        S[ibmat] = centralCovars[ibmat];
      }
      e.set_size(6, centralStates.size(1));
      for (ibmat = 0; ibmat < loop_ub; ibmat++) {
        e[ibmat] = centralStates[ibmat];
      }
      a_tmp = 0.5 * (varargin_1 * varargin_1);
      for (iacol = 0; iacol < 3; iacol++) {
        ibmat = (iacol + 1) << 1;
        oneColumnCost.set_size(1, centralStates.size(1));
        if (csz_idx_1 != 0) {
          ibtile = csz_idx_1 - 1;
          for (int k{0}; k <= ibtile; k++) {
            oneColumnCost[k] = a_tmp * 0.0;
          }
        }
        c.set_size(1, e.size(1));
        if (e.size(1) != 0) {
          acoef = (e.size(1) != 1);
          ibtile = e.size(1) - 1;
          for (int k{0}; k <= ibtile; k++) {
            c[k] = e[(ibmat + 6 * (acoef * k)) - 1] * varargin_1;
          }
        }
        acoef = e.size(1);
        if (e.size(1) == 1) {
          jtilecol = c.size(1);
        } else {
          jtilecol = e.size(1);
        }
        if ((e.size(1) == c.size(1)) && (jtilecol == oneColumnCost.size(1))) {
          c.set_size(1, e.size(1));
          for (ibtile = 0; ibtile < acoef; ibtile++) {
            c[ibtile] = (e[(ibmat + 6 * ibtile) - 2] + c[ibtile]) +
                        oneColumnCost[ibtile];
          }
          acoef = c.size(1);
          for (ibtile = 0; ibtile < acoef; ibtile++) {
            e[(ibmat + 6 * ibtile) - 2] = c[ibtile];
          }
        } else {
          binary_expand_op_1(e, iacol + 1, ibmat - 2, c, oneColumnCost);
        }
        c.set_size(1, centralStates.size(1));
        if (b_csz_idx_1 != 0) {
          ibtile = b_csz_idx_1 - 1;
          for (int k{0}; k <= ibtile; k++) {
            c[k] = 0.0 * varargin_1;
          }
        }
        acoef = e.size(1);
        if (e.size(1) == c.size(1)) {
          c.set_size(1, e.size(1));
          for (ibtile = 0; ibtile < acoef; ibtile++) {
            c[ibtile] = e[(ibmat + 6 * ibtile) - 1] + c[ibtile];
          }
          acoef = c.size(1);
          for (ibtile = 0; ibtile < acoef; ibtile++) {
            e[(ibmat + 6 * ibtile) - 1] = c[ibtile];
          }
        } else {
          binary_expand_op(e, iacol + 1, c);
        }
      }
      if (i2 - 1 >= 0) {
        z_tmp = a_tmp * 0.0;
      }
      for (iacol = 0; iacol < i2; iacol++) {
        double b_F[36];
        double c_F[36];
        double U[18];
        double b_U[18];
        double z[6];
        for (ibmat = 0; ibmat < 6; ibmat++) {
          z[ibmat] = centralStates[ibmat + 6 * iacol];
        }
        z[0] = (z[0] + z[1] * varargin_1) + z_tmp;
        z[1] += 0.0 * varargin_1;
        z[2] = (z[2] + z[3] * varargin_1) + z_tmp;
        z[3] += 0.0 * varargin_1;
        z[4] = (z[4] + z[5] * varargin_1) + z_tmp;
        z[5] += 0.0 * varargin_1;
        for (acoef = 0; acoef < 6; acoef++) {
          for (ibmat = 0; ibmat < 6; ibmat++) {
            imvec[ibmat] = centralStates[ibmat + 6 * iacol];
          }
          d = centralStates[acoef + 6 * iacol];
          numCentralTracks = std::fmax(1.4901161193847656E-8,
                                       1.4901161193847656E-8 * std::abs(d));
          imvec[acoef] = d + numCentralTracks;
          imvec[0] = (imvec[0] + imvec[1] * varargin_1) + z_tmp;
          imvec[1] += 0.0 * varargin_1;
          imvec[2] = (imvec[2] + imvec[3] * varargin_1) + z_tmp;
          imvec[3] += 0.0 * varargin_1;
          imvec[4] = (imvec[4] + imvec[5] * varargin_1) + z_tmp;
          imvec[5] += 0.0 * varargin_1;
          for (ibmat = 0; ibmat < 6; ibmat++) {
            F[ibmat + 6 * acoef] = (imvec[ibmat] - z[ibmat]) / numCentralTracks;
          }
        }
        for (ibmat = 0; ibmat < 6; ibmat++) {
          z[ibmat] = centralStates[ibmat + 6 * iacol];
        }
        z[0] = (z[0] + z[1] * varargin_1) + z_tmp;
        z[1] += 0.0 * varargin_1;
        z[2] = (z[2] + z[3] * varargin_1) + z_tmp;
        z[3] += 0.0 * varargin_1;
        z[4] = (z[4] + z[5] * varargin_1) + z_tmp;
        z[5] += 0.0 * varargin_1;
        for (acoef = 0; acoef < 3; acoef++) {
          double specvec_f2[3];
          specvec_f2[0] = 0.0;
          specvec_f2[1] = 0.0;
          specvec_f2[2] = 0.0;
          specvec_f2[acoef] = 1.4901161193847656E-8;
          for (ibmat = 0; ibmat < 6; ibmat++) {
            imvec[ibmat] = centralStates[ibmat + 6 * iacol];
          }
          imvec[0] = (imvec[0] + imvec[1] * varargin_1) + a_tmp * specvec_f2[0];
          imvec[1] += specvec_f2[0] * varargin_1;
          imvec[2] = (imvec[2] + imvec[3] * varargin_1) + a_tmp * specvec_f2[1];
          imvec[3] += specvec_f2[1] * varargin_1;
          imvec[4] = (imvec[4] + imvec[5] * varargin_1) + a_tmp * specvec_f2[2];
          imvec[5] += specvec_f2[2] * varargin_1;
          for (ibmat = 0; ibmat < 6; ibmat++) {
            U[ibmat + 6 * acoef] =
                (imvec[ibmat] - z[ibmat]) / 1.4901161193847656E-8;
          }
        }
        for (ibmat = 0; ibmat < 6; ibmat++) {
          for (ibtile = 0; ibtile < 6; ibtile++) {
            d = 0.0;
            for (acoef = 0; acoef < 6; acoef++) {
              d += F[ibmat + 6 * acoef] * S[(acoef + 6 * ibtile) + 36 * iacol];
            }
            b_F[ibmat + 6 * ibtile] = d;
          }
          d = U[ibmat];
          numCentralTracks = U[ibmat + 6];
          b_imvec = U[ibmat + 12];
          for (ibtile = 0; ibtile < 3; ibtile++) {
            b_U[ibmat + 6 * ibtile] =
                (d * Q[3 * ibtile] + numCentralTracks * Q[3 * ibtile + 1]) +
                b_imvec * Q[3 * ibtile + 2];
          }
          for (ibtile = 0; ibtile < 6; ibtile++) {
            d = 0.0;
            for (acoef = 0; acoef < 6; acoef++) {
              d += b_F[ibmat + 6 * acoef] * F[ibtile + 6 * acoef];
            }
            c_F[ibmat + 6 * ibtile] = d;
          }
        }
        for (ibmat = 0; ibmat < 6; ibmat++) {
          d = b_U[ibmat];
          numCentralTracks = b_U[ibmat + 6];
          b_imvec = b_U[ibmat + 12];
          for (ibtile = 0; ibtile < 6; ibtile++) {
            F[ibmat + 6 * ibtile] =
                (d * U[ibtile] + numCentralTracks * U[ibtile + 6]) +
                b_imvec * U[ibtile + 12];
          }
        }
        for (ibmat = 0; ibmat < 6; ibmat++) {
          for (ibtile = 0; ibtile < 6; ibtile++) {
            acoef = ibtile + 6 * ibmat;
            S[(ibtile + 6 * ibmat) + 36 * iacol] = c_F[acoef] + F[acoef];
          }
        }
      }
      for (ibmat = 0; ibmat < 20; ibmat++) {
        inKnownIDs[ibmat] =
            (localTracks[b_i].SourceIndex == pSourceConfigIDs[ibmat]);
      }
      iacol = 0;
      exitg1 = false;
      while ((!exitg1) && (iacol < 20)) {
        if (inKnownIDs[iacol]) {
          sourceConfig = pSourceConfigurations[iacol];
          exitg1 = true;
        } else {
          iacol++;
        }
      }
      transformedStates.set_size(6, i);
      transformedCovars.set_size(6, 6, i);
      for (jtilecol = 0; jtilecol < i; jtilecol++) {
        acoef = jtilecol * 6;
        ibtile = jtilecol * 36 - 1;
        for (int k{0}; k < 6; k++) {
          transformedStates[acoef + k] = localTracks[b_i].State[k];
          iacol = k * 6;
          ibmat = ibtile + k * 6;
          for (int b_k{0}; b_k < 6; b_k++) {
            transformedCovars[(ibmat + b_k) + 1] =
                localTracks[b_i].StateCovariance[iacol + b_k];
          }
        }
      }
      for (acoef = 0; acoef < i; acoef++) {
        if (!sourceConfig->pIsTransformToLocalValid) {
          sourceConfig->pIsTransformToLocalValid = true;
        }
        for (ibmat = 0; ibmat < 6; ibmat++) {
          transformedStates[ibmat + 6 * acoef] = e[ibmat + 6 * acoef];
          for (ibtile = 0; ibtile < 6; ibtile++) {
            transformedCovars[(ibtile + 6 * ibmat) + 36 * acoef] =
                S[(ibtile + 6 * ibmat) + 36 * acoef];
          }
        }
      }
      allStates.set_size(6, transformedStates.size(1) + 1);
      for (ibmat = 0; ibmat < 6; ibmat++) {
        allStates[ibmat] = localTracks[b_i].State[ibmat];
      }
      acoef = transformedStates.size(1);
      for (ibmat = 0; ibmat < acoef; ibmat++) {
        for (ibtile = 0; ibtile < 6; ibtile++) {
          allStates[ibtile + 6 * (ibmat + 1)] =
              transformedStates[ibtile + 6 * ibmat];
        }
      }
      allCovars.set_size(6, 6, transformedCovars.size(2) + 1);
      for (acoef = 0; acoef < 36; acoef++) {
        allCovars[acoef] = localTracks[b_i].StateCovariance[acoef];
      }
      for (acoef = 0; acoef < b_loop_ub_tmp; acoef++) {
        allCovars[acoef + 36] = transformedCovars[acoef];
      }
      oneColumnCost.set_size(1, allStates.size(1));
      acoef = allStates.size(1);
      for (ibmat = 0; ibmat < acoef; ibmat++) {
        oneColumnCost[ibmat] = 0.0;
      }
      e.set_size(6, allStates.size(1));
      ibtile = (allStates.size(1) != 1);
      ibmat = allStates.size(1) - 1;
      for (int k{0}; k <= ibmat; k++) {
        acoef = ibtile * k;
        for (int b_k{0}; b_k < 6; b_k++) {
          e[b_k + 6 * k] = allStates[b_k] - allStates[b_k + 6 * acoef];
        }
      }
      S.set_size(6, 6, allCovars.size(2));
      for (int k{0}; k <= ibmat; k++) {
        acoef = ibtile * k;
        for (int b_k{0}; b_k < 6; b_k++) {
          for (iacol = 0; iacol < 6; iacol++) {
            S[(iacol + 6 * b_k) + 36 * k] =
                allCovars[iacol + 6 * b_k] +
                allCovars[(iacol + 6 * b_k) + 36 * acoef];
          }
        }
      }
      r.set_size(1, allStates.size(1));
      ibmat = allStates.size(1);
      for (iacol = 0; iacol < ibmat; iacol++) {
        int ipiv[6];
        boolean_T isodd;
        for (ibtile = 0; ibtile < 6; ibtile++) {
          for (acoef = 0; acoef < 6; acoef++) {
            F[acoef + 6 * ibtile] = S[(acoef + 6 * ibtile) + 36 * iacol];
          }
        }
        internal::reflapack::xzgetrf(F, ipiv);
        numCentralTracks = F[0];
        isodd = false;
        for (int k{0}; k < 5; k++) {
          numCentralTracks *= F[(k + 6 * (k + 1)) + 1];
          if (ipiv[k] > k + 1) {
            isodd = !isodd;
          }
        }
        if (isodd) {
          numCentralTracks = -numCentralTracks;
        }
        for (ibtile = 0; ibtile < 6; ibtile++) {
          imvec[ibtile] = e[ibtile + 6 * iacol];
        }
        std::copy(&(*(double(*)[36]) & S[36 * iacol])[0],
                  &(*(double(*)[36]) & S[36 * iacol])[36], &F[0]);
        internal::mrdiv(imvec, F);
        b_imvec = 0.0;
        for (ibtile = 0; ibtile < 6; ibtile++) {
          b_imvec += imvec[ibtile] * e[ibtile + 6 * iacol];
        }
        r[iacol] = b_imvec + std::log(numCentralTracks);
      }
      acoef = r.size(1);
      for (ibmat = 0; ibmat < acoef; ibmat++) {
        oneColumnCost[ibmat] = r[ibmat];
      }
      ibmat = (oneColumnCost.size(1) >= 2);
      acoef = costMatrix.size(0);
      for (ibtile = 0; ibtile < acoef; ibtile++) {
        costMatrix[ibtile + costMatrix.size(0) * b_i] =
            oneColumnCost[ibmat + ibtile];
      }
      if (localTracks[b_i].ObjectClassID != 0.0) {
        acoef = trackClasses.size(1);
        oneColumnCost.set_size(1, trackClasses.size(1));
        for (ibmat = 0; ibmat < acoef; ibmat++) {
          oneColumnCost[ibmat] = trackClasses[ibmat];
        }
        ibtile = oneColumnCost.size(1) - 1;
        acoef = 0;
        for (iacol = 0; iacol <= ibtile; iacol++) {
          d = oneColumnCost[iacol];
          if ((d != localTracks[b_i].ObjectClassID) && (d != 0.0)) {
            acoef++;
          }
        }
        r1.set_size(1, acoef);
        acoef = 0;
        for (iacol = 0; iacol <= ibtile; iacol++) {
          d = oneColumnCost[iacol];
          if ((d != localTracks[b_i].ObjectClassID) && (d != 0.0)) {
            r1[acoef] = iacol;
            acoef++;
          }
        }
        acoef = r1.size(1);
        for (ibmat = 0; ibmat < acoef; ibmat++) {
          costMatrix[r1[ibmat] + costMatrix.size(0) * b_i] = rtInf;
        }
      }
    }
    ibtile = loop_ub_tmp - 1;
    for (int b_i{0}; b_i <= ibtile; b_i++) {
      if (costMatrix[b_i] > 5.0) {
        costMatrix[b_i] = rtInf;
      }
    }
  }
}

//
// Arguments    : unsigned int in_TrackID
//                unsigned int in_SourceIndex
//                double in_UpdateTime
//                const double in_State[6]
//                const double in_StateCovariance[36]
//                double in_ObjectClassID
//                const char in_TrackLogic[7]
//                objectTrack *out
// Return Type  : void
//
void trackFuser::ensureTrack(unsigned int in_TrackID,
                             unsigned int in_SourceIndex, double in_UpdateTime,
                             const double in_State[6],
                             const double in_StateCovariance[36],
                             double in_ObjectClassID,
                             const char in_TrackLogic[7], objectTrack *out)
{
  static const char cv[128]{
      '\x00', '\x01', '\x02', '\x03', '\x04', '\x05', '\x06', '\a',   '\b',
      '\t',   '\n',   '\v',   '\f',   '\r',   '\x0e', '\x0f', '\x10', '\x11',
      '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19', '\x1a',
      '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',    '!',    '\"',   '#',
      '$',    '%',    '&',    '\'',   '(',    ')',    '*',    '+',    ',',
      '-',    '.',    '/',    '0',    '1',    '2',    '3',    '4',    '5',
      '6',    '7',    '8',    '9',    ':',    ';',    '<',    '=',    '>',
      '?',    '@',    'a',    'b',    'c',    'd',    'e',    'f',    'g',
      'h',    'i',    'j',    'k',    'l',    'm',    'n',    'o',    'p',
      'q',    'r',    's',    't',    'u',    'v',    'w',    'x',    'y',
      'z',    '[',    '\\',   ']',    '^',    '_',    '`',    'a',    'b',
      'c',    'd',    'e',    'f',    'g',    'h',    'i',    'j',    'k',
      'l',    'm',    'n',    'o',    'p',    'q',    'r',    's',    't',
      'u',    'v',    'w',    'x',    'y',    'z',    '{',    '|',    '}',
      '~',    '\x7f'};
  static const char cv1[7]{'H', 'i', 's', 't', 'o', 'r', 'y'};
  int i;
  boolean_T b_bool;
  out->pTrackLogicState.set_size(1, 0);
  out->TrackID = in_TrackID;
  out->SourceIndex = in_SourceIndex;
  out->BranchID = 0U;
  out->pUpdateTime = in_UpdateTime;
  out->Age = 1U;
  for (i = 0; i < 6; i++) {
    out->pState[i] = in_State[i];
  }
  std::copy(&in_StateCovariance[0], &in_StateCovariance[36],
            &out->pStateCovariance[0]);
  out->IsConfirmed = true;
  out->IsCoasted = false;
  out->IsSelfReported = true;
  out->ObjectClassProbabilities = 1.0;
  out->ObjectClassID = in_ObjectClassID;
  b_bool = false;
  i = 0;
  int exitg1;
  do {
    exitg1 = 0;
    if (i < 7) {
      if (cv[static_cast<int>(in_TrackLogic[i])] !=
          cv[static_cast<int>(cv1[i])]) {
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      b_bool = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  if (b_bool) {
    out->pTrackLogicState.set_size(1, 1);
    out->pTrackLogicState[0] = true;
  }
}

//
// Arguments    : const array<struct_T, 2U> &localTracks
//                const array<unsigned int, 2U> &assignments
// Return Type  : void
//
void trackFuser::fuseAssigned(const array<struct_T, 2U> &localTracks,
                              const array<unsigned int, 2U> &assignments)
{
  fuserSourceConfiguration *thisConfig;
  objectTrack obj;
  trackHistoryLogic *b_obj;
  array<objectTrack, 2U> transformedTracks;
  array<double, 1U> b_assignedSourceInds;
  array<int, 1U> assignedSourceInds;
  array<unsigned int, 1U> b_assignments;
  array<int, 1U> r;
  array<int, 1U> r1;
  array<unsigned int, 1U> uniqueAssigned;
  array<boolean_T, 1U> inAssigned;
  array<boolean_T, 1U> toFuse;
  int c_i;
  int i;
  int k;
  int ntilecols;
  int nz;
  boolean_T exitg1;
  obj = pTracksList[0];
  transformedTracks.set_size(1, assignments.size(0));
  if (assignments.size(0) != 0) {
    ntilecols = assignments.size(0);
    for (nz = 0; nz < ntilecols; nz++) {
      transformedTracks[nz] = obj;
    }
  }
  i = assignments.size(0);
  for (int b_i{0}; b_i < i; b_i++) {
    struct_T expl_temp;
    boolean_T inKnownIDs[20];
    expl_temp =
        localTracks[static_cast<int>(assignments[b_i + assignments.size(0)]) -
                    1];
    for (k = 0; k < 20; k++) {
      inKnownIDs[k] = (expl_temp.SourceIndex == pSourceConfigIDs[k]);
    }
    c_i = 0;
    exitg1 = false;
    while ((!exitg1) && (c_i < 20)) {
      if (inKnownIDs[c_i]) {
        thisConfig = pSourceConfigurations[c_i];
        exitg1 = true;
      } else {
        c_i++;
      }
    }
    if (!thisConfig->pIsTransformToCentralValid) {
      thisConfig->pIsTransformToCentralValid = true;
    }
    trackFuser::ensureTrack(expl_temp.TrackID, expl_temp.SourceIndex,
                            expl_temp.UpdateTime, expl_temp.State,
                            expl_temp.StateCovariance, expl_temp.ObjectClassID,
                            expl_temp.TrackLogic, &transformedTracks[b_i]);
  }
  b_assignments.set_size(assignments.size(0));
  ntilecols = assignments.size(0);
  for (i = 0; i < ntilecols; i++) {
    b_assignments[i] = assignments[i];
  }
  unique_vector(b_assignments, uniqueAssigned);
  i = uniqueAssigned.size(0);
  for (int b_i{0}; b_i < i; b_i++) {
    boolean_T y;
    ntilecols = assignments.size(0);
    inAssigned.set_size(assignments.size(0));
    for (k = 0; k < ntilecols; k++) {
      inAssigned[k] = (uniqueAssigned[b_i] == assignments[k]);
    }
    eml_find(inAssigned, assignedSourceInds);
    toFuse.set_size(assignedSourceInds.size(0));
    k = assignedSourceInds.size(0);
    for (c_i = 0; c_i < k; c_i++) {
      if ((transformedTracks[assignedSourceInds[c_i] - 1].IsConfirmed &&
           (!transformedTracks[assignedSourceInds[c_i] - 1].IsCoasted)) ||
          transformedTracks[assignedSourceInds[c_i] - 1].IsSelfReported) {
        toFuse[c_i] = true;
      } else {
        toFuse[c_i] = false;
      }
    }
    y = false;
    ntilecols = 1;
    exitg1 = false;
    while ((!exitg1) && (ntilecols <= toFuse.size(0))) {
      if (toFuse[ntilecols - 1]) {
        y = true;
        exitg1 = true;
      } else {
        ntilecols++;
      }
    }
    if (y) {
      double id;
      int obj_tmp_tmp;
      unsigned int u;
      ntilecols = toFuse.size(0);
      if (toFuse.size(0) == 0) {
        nz = 0;
      } else {
        nz = toFuse[0];
        for (k = 2; k <= ntilecols; k++) {
          nz += toFuse[k - 1];
        }
      }
      obj_tmp_tmp = static_cast<int>(uniqueAssigned[b_i]) - 1;
      obj = pTracksList[obj_tmp_tmp];
      id = static_cast<double>(pTracksList[obj_tmp_tmp].Age) +
           static_cast<double>(nz);
      if (id < 4.294967296E+9) {
        if (id >= 0.0) {
          u = static_cast<unsigned int>(id);
        } else {
          u = 0U;
        }
      } else {
        u = MAX_uint32_T;
      }
      obj.Age = u;
      pTracksList[obj_tmp_tmp] = obj;
      nz = inAssigned.size(0) - 1;
      ntilecols = 0;
      for (c_i = 0; c_i <= nz; c_i++) {
        if (inAssigned[c_i]) {
          ntilecols++;
        }
      }
      r.set_size(ntilecols);
      ntilecols = 0;
      for (c_i = 0; c_i <= nz; c_i++) {
        if (inAssigned[c_i]) {
          r[ntilecols] = c_i;
          ntilecols++;
        }
      }
      id = pTracksList[obj_tmp_tmp].ObjectClassID;
      ntilecols = 0;
      while ((id == 0.0) && (ntilecols < r.size(0))) {
        ntilecols++;
        id = localTracks[static_cast<int>(assignments[r[ntilecols - 1] +
                                                      assignments.size(0)]) -
                         1]
                 .ObjectClassID;
      }
      obj = pTracksList[obj_tmp_tmp];
      obj.ObjectClassID = id;
      pTracksList[obj_tmp_tmp] = obj;
      obj = pTracksList[obj_tmp_tmp];
      nz = toFuse.size(0) - 1;
      ntilecols = 0;
      for (c_i = 0; c_i <= nz; c_i++) {
        if (toFuse[c_i]) {
          ntilecols++;
        }
      }
      r1.set_size(ntilecols);
      ntilecols = 0;
      for (c_i = 0; c_i <= nz; c_i++) {
        if (toFuse[c_i]) {
          r1[ntilecols] = c_i;
          ntilecols++;
        }
      }
      ntilecols = r1.size(0);
      b_assignedSourceInds.set_size(r1.size(0));
      for (k = 0; k < ntilecols; k++) {
        b_assignedSourceInds[k] = assignedSourceInds[r1[k]];
      }
      cFuser.fuse(obj, transformedTracks, b_assignedSourceInds);
      pTracksList[obj_tmp_tmp] = obj;
      ntilecols = toFuse.size(0);
      if (toFuse.size(0) == 0) {
        nz = 0;
      } else {
        nz = toFuse[0];
        for (k = 2; k <= ntilecols; k++) {
          nz += toFuse[k - 1];
        }
      }
      for (ntilecols = 0; ntilecols < nz; ntilecols++) {
        boolean_T bv[50];
        b_obj = pTrackLogics[obj_tmp_tmp];
        bv[0] = true;
        for (k = 0; k < 49; k++) {
          bv[k + 1] = b_obj->pRecentHistory[k];
        }
        for (k = 0; k < 50; k++) {
          b_obj->pRecentHistory[k] = bv[k];
        }
        b_obj->pIsFirstUpdate = false;
      }
      if (pTracksList[obj_tmp_tmp].IsConfirmed) {
        y = true;
      } else {
        b_obj = pTrackLogics[obj_tmp_tmp];
        if (b_obj->pIsFirstUpdate) {
          y = false;
        } else {
          boolean_T x[5];
          for (k = 0; k < 5; k++) {
            x[k] = b_obj->pRecentHistory[k];
          }
          y = ((((x[0] + x[1]) + x[2]) + x[3]) + x[4] >= 5);
        }
        if (y || (pTracksList[obj_tmp_tmp].ObjectClassID > 0.0)) {
          y = true;
        } else {
          y = false;
        }
      }
      obj = pTracksList[obj_tmp_tmp];
      obj.IsConfirmed = y;
      pTracksList[obj_tmp_tmp] = obj;
      pConfirmedTracks[obj_tmp_tmp] = pTracksList[obj_tmp_tmp].IsConfirmed;
      obj = pTracksList[obj_tmp_tmp];
      obj.IsCoasted = false;
      pTracksList[obj_tmp_tmp] = obj;
      obj = pTracksList[obj_tmp_tmp];
      ntilecols = r.size(0);
      b_assignments.set_size(r.size(0));
      for (k = 0; k < ntilecols; k++) {
        b_assignments[k] = assignments[r[k] + assignments.size(0)];
      }
      obj.IsSelfReported = getSelfReporting(localTracks, b_assignments);
      pTracksList[obj_tmp_tmp] = obj;
      obj = pTracksList[obj_tmp_tmp];
      pTracksList[obj_tmp_tmp] = obj;
      obj = pTracksList[obj_tmp_tmp];
      pTracksList[obj_tmp_tmp] = obj;
    }
  }
}

//
// Arguments    : const array<struct_T, 2U> &localTracks
//                const array<unsigned int, 1U> &assignedlocalTracks
// Return Type  : boolean_T
//
boolean_T
trackFuser::getSelfReporting(const array<struct_T, 2U> &localTracks,
                             const array<unsigned int, 1U> &assignedlocalTracks)
{
  fuserSourceConfiguration *thisSource;
  int i;
  boolean_T tf;
  tf = false;
  i = 0;
  while ((!tf) && (i < assignedlocalTracks.size(0))) {
    int b_i;
    boolean_T inKnownIDs[20];
    boolean_T exitg1;
    i++;
    for (b_i = 0; b_i < 20; b_i++) {
      inKnownIDs[b_i] =
          (localTracks[static_cast<int>(assignedlocalTracks[i - 1]) - 1]
               .SourceIndex == pSourceConfigIDs[b_i]);
    }
    b_i = 0;
    exitg1 = false;
    while ((!exitg1) && (b_i < 20)) {
      if (inKnownIDs[b_i]) {
        thisSource = pSourceConfigurations[b_i];
        exitg1 = true;
      } else {
        b_i++;
      }
    }
    tf = thisSource->IsInternalSource;
  }
  return tf;
}

//
// Arguments    : const array<struct_T, 2U> &localTracks
//                array<unsigned int, 1U> &unassignedlocalTracks
// Return Type  : void
//
void trackFuser::initializeCentralTracks(
    const array<struct_T, 2U> &localTracks,
    array<unsigned int, 1U> &unassignedlocalTracks)
{
  fuserSourceConfiguration *b_thisSource;
  fuserSourceConfiguration *thisConfig;
  fuserSourceConfiguration *thisSource;
  objectTrack centralTrack;
  trackHistoryLogic *obj;
  array<double, 2U> costMatrix;
  array<double, 2U> r1;
  array<unsigned int, 2U> a;
  array<unsigned int, 1U> allSourceInds;
  array<unsigned int, 1U> checkedUnassigned;
  array<int, 1U> r;
  array<boolean_T, 2U> assignedToNewTrack;
  double S[72];
  double P[36];
  int i;
  int j;
  unsigned int u;
  boolean_T exitg1;
  allSourceInds.set_size(unassignedlocalTracks.size(0));
  j = unassignedlocalTracks.size(0);
  for (i = 0; i < j; i++) {
    allSourceInds[i] =
        localTracks[static_cast<int>(unassignedlocalTracks[i]) - 1].SourceIndex;
  }
  exitg1 = false;
  while ((!exitg1) &&
         ((unassignedlocalTracks.size(0) > 0) && (pNumLiveTracks < 100.0))) {
    double x[6];
    double c_i;
    int b_i;
    unsigned int centralTrack_tmp;
    int end_tmp;
    int nz;
    int partialTrueCount;
    unsigned int q0;
    unsigned int qY;
    int trueCount;
    char cv[7];
    boolean_T bv[50];
    boolean_T inKnownIDs[20];
    boolean_T exitg2;
    boolean_T isodd;
    boolean_T tf;
    for (b_i = 0; b_i < 20; b_i++) {
      inKnownIDs[b_i] =
          (localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1]
               .SourceIndex == pSourceConfigIDs[b_i]);
    }
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i < 20)) {
      if (inKnownIDs[i]) {
        thisConfig = pSourceConfigurations[i];
        exitg2 = true;
      } else {
        i++;
      }
    }
    if (!thisConfig->pIsTransformToCentralValid) {
      thisConfig->pIsTransformToCentralValid = true;
    }
    nz = static_cast<int>(unassignedlocalTracks[0]) - 1;
    centralTrack_tmp =
        localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1].SourceIndex;
    for (int i1{0}; i1 < 6; i1++) {
      x[i1] =
          localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1].State[i1];
    }
    std::copy(&localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1]
                   .StateCovariance[0],
              &localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1]
                   .StateCovariance[36],
              &P[0]);
    for (int i2{0}; i2 < 7; i2++) {
      cv[i2] = localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1]
                   .TrackLogic[i2];
    }
    trackFuser::ensureTrack(
        localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1].TrackID,
        centralTrack_tmp,
        localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1].UpdateTime,
        x, P,
        localTracks[static_cast<int>(unassignedlocalTracks[0]) - 1]
            .ObjectClassID,
        cv, &centralTrack);
    pNumLiveTracks++;
    q0 = pLastTrackID;
    qY = q0 + 1U;
    if (q0 + 1U < q0) {
      qY = MAX_uint32_T;
    }
    pLastTrackID = qY;
    centralTrack.TrackID = pLastTrackID;
    centralTrack.SourceIndex = 1U;
    centralTrack.Age = 1U;
    pTracksList[static_cast<int>(pNumLiveTracks) - 1] = centralTrack;
    tf = false;
    i = 0;
    while ((!tf) && (i < 1)) {
      i = 1;
      for (b_i = 0; b_i < 20; b_i++) {
        inKnownIDs[b_i] = (centralTrack_tmp == pSourceConfigIDs[b_i]);
      }
      partialTrueCount = 0;
      exitg2 = false;
      while ((!exitg2) && (partialTrueCount < 20)) {
        if (inKnownIDs[partialTrueCount]) {
          thisSource = pSourceConfigurations[partialTrueCount];
          exitg2 = true;
        } else {
          partialTrueCount++;
        }
      }
      tf = thisSource->IsInternalSource;
    }
    centralTrack.IsSelfReported = tf;
    c_i = pNumLiveTracks;
    c_i = pTracksList[static_cast<int>(c_i) - 1].ObjectClassID;
    j = 0;
    while ((c_i == 0.0) && (j < 1)) {
      j = 1;
      c_i = localTracks[nz].ObjectClassID;
    }
    centralTrack.ObjectClassID = c_i;
    obj = pTrackLogics[static_cast<int>(pNumLiveTracks) - 1];
    bv[0] = true;
    for (b_i = 0; b_i < 49; b_i++) {
      bv[b_i + 1] = obj->pRecentHistory[b_i];
    }
    for (b_i = 0; b_i < 50; b_i++) {
      obj->pRecentHistory[b_i] = bv[b_i];
    }
    obj->pIsFirstUpdate = false;
    obj = pTrackLogics[static_cast<int>(pNumLiveTracks) - 1];
    if (obj->pIsFirstUpdate) {
      isodd = false;
    } else {
      boolean_T b_x[5];
      for (b_i = 0; b_i < 5; b_i++) {
        b_x[b_i] = obj->pRecentHistory[b_i];
      }
      isodd = ((((b_x[0] + b_x[1]) + b_x[2]) + b_x[3]) + b_x[4] >= 5);
    }
    if (isodd || (c_i > 0.0)) {
      isodd = true;
    } else {
      isodd = false;
    }
    centralTrack.IsConfirmed = isodd;
    pTrackIDs[static_cast<int>(pNumLiveTracks) - 1] = pLastTrackID;
    pTracksList[static_cast<int>(pNumLiveTracks) - 1] = centralTrack;
    pConfirmedTracks[static_cast<int>(pNumLiveTracks) - 1] = isodd;
    costMatrix.set_size(1, unassignedlocalTracks.size(0));
    j = unassignedlocalTracks.size(0);
    for (b_i = 0; b_i < j; b_i++) {
      costMatrix[b_i] = 0.0;
    }
    end_tmp = allSourceInds.size(0) - 1;
    for (i = 0; i <= end_tmp; i++) {
      if (allSourceInds[i] == localTracks[nz].SourceIndex) {
        costMatrix[i] = rtInf;
      }
    }
    trueCount = 0;
    for (i = 0; i <= end_tmp; i++) {
      if (allSourceInds[i] != localTracks[nz].SourceIndex) {
        trueCount++;
      }
    }
    r.set_size(trueCount);
    partialTrueCount = 0;
    for (i = 0; i <= end_tmp; i++) {
      if (allSourceInds[i] != localTracks[nz].SourceIndex) {
        r[partialTrueCount] = i;
        partialTrueCount++;
      }
    }
    if (r.size(0) == 0) {
      r1.set_size(1, 0);
    } else {
      r1.set_size(1, r.size(0));
      j = r.size(0);
      for (b_i = 0; b_i < j; b_i++) {
        r1[b_i] = 0.0;
      }
      b_i = r.size(0);
      for (i = 0; i < b_i; i++) {
        struct_T expl_temp;
        double allCovars[72];
        double allStates[12];
        double e[12];
        double allCosts[2];
        j = static_cast<int>(unassignedlocalTracks[r[i]]) - 1;
        expl_temp = localTracks[j];
        std::copy(&centralTrack.pStateCovariance[0],
                  &centralTrack.pStateCovariance[36], &P[0]);
        fusion::internal::gaussEKFilter::predict(
            centralTrack.pState, P, ProcessNoise,
            localTracks[static_cast<int>(unassignedlocalTracks[r[i]]) - 1]
                    .UpdateTime -
                centralTrack.pUpdateTime,
            x);
        for (trueCount = 0; trueCount < 20; trueCount++) {
          inKnownIDs[trueCount] =
              (localTracks[j].SourceIndex == pSourceConfigIDs[trueCount]);
        }
        partialTrueCount = 0;
        exitg2 = false;
        while ((!exitg2) && (partialTrueCount < 20)) {
          if (inKnownIDs[partialTrueCount]) {
            b_thisSource = pSourceConfigurations[partialTrueCount];
            exitg2 = true;
          } else {
            partialTrueCount++;
          }
        }
        if (!b_thisSource->pIsTransformToLocalValid) {
          b_thisSource->pIsTransformToLocalValid = true;
        }
        for (j = 0; j < 6; j++) {
          allStates[j] = x[j];
          allStates[j + 6] = expl_temp.State[j];
        }
        for (j = 0; j < 36; j++) {
          allCovars[j] = P[j];
          allCovars[j + 36] = expl_temp.StateCovariance[j];
        }
        for (end_tmp = 0; end_tmp < 2; end_tmp++) {
          for (nz = 0; nz < 6; nz++) {
            j = nz + 6 * end_tmp;
            e[j] = allStates[nz] - allStates[j];
            for (partialTrueCount = 0; partialTrueCount < 6;
                 partialTrueCount++) {
              j = partialTrueCount + 6 * nz;
              trueCount = j + 36 * end_tmp;
              S[trueCount] = allCovars[j] + allCovars[trueCount];
            }
          }
        }
        for (partialTrueCount = 0; partialTrueCount < 2; partialTrueCount++) {
          double c_x;
          int ipiv[6];
          std::copy(
              &S[partialTrueCount * 36],
              &S[static_cast<int>(
                  static_cast<unsigned int>(partialTrueCount * 36) + 36U)],
              &P[0]);
          internal::reflapack::xzgetrf(P, ipiv);
          c_i = P[0];
          isodd = false;
          for (end_tmp = 0; end_tmp < 5; end_tmp++) {
            c_i *= P[(end_tmp + 6 * (end_tmp + 1)) + 1];
            if (ipiv[end_tmp] > end_tmp + 1) {
              isodd = !isodd;
            }
          }
          if (isodd) {
            c_i = -c_i;
          }
          for (trueCount = 0; trueCount < 6; trueCount++) {
            x[trueCount] = e[trueCount + 6 * partialTrueCount];
          }
          internal::mrdiv(x, &S[36 * partialTrueCount]);
          c_x = 0.0;
          for (trueCount = 0; trueCount < 6; trueCount++) {
            c_x += x[trueCount] * e[trueCount + 6 * partialTrueCount];
          }
          allCosts[partialTrueCount] = c_x + std::log(c_i);
        }
        r1[i] = allCosts[1];
      }
    }
    j = r.size(0) - 1;
    for (b_i = 0; b_i <= j; b_i++) {
      costMatrix[r[b_i]] = r1[b_i];
    }
    checkedUnassigned.set_size(unassignedlocalTracks.size(0));
    j = unassignedlocalTracks.size(0);
    assignedToNewTrack.set_size(1, costMatrix.size(1));
    for (b_i = 0; b_i < j; b_i++) {
      checkedUnassigned[b_i] = unassignedlocalTracks[b_i];
      assignedToNewTrack[b_i] = (costMatrix[b_i] < 5.0);
    }
    isodd = false;
    j = 1;
    exitg2 = false;
    while ((!exitg2) && (j <= assignedToNewTrack.size(1))) {
      if (assignedToNewTrack[j - 1]) {
        isodd = true;
        exitg2 = true;
      } else {
        j++;
      }
    }
    if (isodd) {
      j = assignedToNewTrack.size(1);
      nz = assignedToNewTrack[0];
      for (end_tmp = 2; end_tmp <= j; end_tmp++) {
        nz += assignedToNewTrack[end_tmp - 1];
      }
      c_i = pNumLiveTracks;
      end_tmp = assignedToNewTrack.size(1) - 1;
      trueCount = 0;
      for (i = 0; i <= end_tmp; i++) {
        if (assignedToNewTrack[i]) {
          trueCount++;
        }
      }
      checkedUnassigned.set_size(trueCount);
      partialTrueCount = 0;
      for (i = 0; i <= end_tmp; i++) {
        if (assignedToNewTrack[i]) {
          checkedUnassigned[partialTrueCount] = unassignedlocalTracks[i];
          partialTrueCount++;
        }
      }
      a.set_size(nz, 2);
      if (nz - 1 >= 0) {
        c_i = std::round(c_i);
        if (c_i < 4.294967296E+9) {
          if (c_i >= 0.0) {
            u = static_cast<unsigned int>(c_i);
          } else {
            u = 0U;
          }
        } else if (c_i >= 4.294967296E+9) {
          u = MAX_uint32_T;
        } else {
          u = 0U;
        }
      }
      for (b_i = 0; b_i < nz; b_i++) {
        a[b_i] = u;
      }
      j = checkedUnassigned.size(0);
      for (b_i = 0; b_i < j; b_i++) {
        a[b_i + a.size(0)] = checkedUnassigned[b_i];
      }
      fuseAssigned(localTracks, a);
      trueCount = 0;
      for (i = 0; i <= end_tmp; i++) {
        if (!assignedToNewTrack[i]) {
          trueCount++;
        }
      }
      checkedUnassigned.set_size(trueCount);
      partialTrueCount = 0;
      for (i = 0; i <= end_tmp; i++) {
        if (!assignedToNewTrack[i]) {
          checkedUnassigned[partialTrueCount] = unassignedlocalTracks[i];
          partialTrueCount++;
        }
      }
      trueCount = 0;
      partialTrueCount = 0;
      for (i = 0; i <= end_tmp; i++) {
        if (!assignedToNewTrack[i]) {
          j = trueCount + 1;
          trueCount++;
          allSourceInds[partialTrueCount] = allSourceInds[i];
          partialTrueCount = j;
        }
      }
      allSourceInds.set_size(trueCount);
      if (tf ||
          pTracksList[static_cast<int>(pNumLiveTracks) - 1].IsSelfReported) {
        isodd = true;
      } else {
        isodd = false;
      }
      b_i = static_cast<int>(pNumLiveTracks) - 1;
      centralTrack = pTracksList[b_i];
      centralTrack.IsSelfReported = isodd;
      pTracksList[b_i] = centralTrack;
    }
    j = checkedUnassigned.size(0);
    for (end_tmp = 0; end_tmp <= j - 2; end_tmp++) {
      checkedUnassigned[end_tmp] = checkedUnassigned[end_tmp + 1];
    }
    if (checkedUnassigned.size(0) - 1 < 1) {
      b_i = 0;
    } else {
      b_i = checkedUnassigned.size(0) - 1;
    }
    checkedUnassigned.set_size(b_i);
    j = allSourceInds.size(0);
    for (end_tmp = 0; end_tmp <= j - 2; end_tmp++) {
      allSourceInds[end_tmp] = allSourceInds[end_tmp + 1];
    }
    if (allSourceInds.size(0) - 1 < 1) {
      b_i = 0;
    } else {
      b_i = allSourceInds.size(0) - 1;
    }
    allSourceInds.set_size(b_i);
    if (allSourceInds.size(0) != 0) {
      unassignedlocalTracks.set_size(checkedUnassigned.size(0));
      j = checkedUnassigned.size(0);
      for (b_i = 0; b_i < j; b_i++) {
        unassignedlocalTracks[b_i] = checkedUnassigned[b_i];
      }
    } else {
      exitg1 = true;
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
void trackFuser::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
    if (isInitialized == 1) {
      isInitialized = 2;
      releaseWrapper();
    }
  }
}

//
// Arguments    : void
// Return Type  : void
//
void trackFuser::releaseWrapper()
{
  trackHistoryLogic *obj;
  if (isSetupComplete) {
    pNumLiveTracks = 0.0;
    for (int i{0}; i < 100; i++) {
      pTrackIDs[i] = 0U;
    }
    for (int i{0}; i < 100; i++) {
      pConfirmedTracks[i] = false;
    }
    if (cAssigner.isInitialized == 1) {
      cAssigner.isInitialized = 2;
    }
    obj = pTrackLogics[0];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[1];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[2];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[3];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[4];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[5];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[6];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[7];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[8];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[9];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[10];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[11];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[12];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[13];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[14];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[15];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[16];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[17];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[18];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[19];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[20];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[21];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[22];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[23];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[24];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[25];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[26];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[27];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[28];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[29];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[30];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[31];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[32];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[33];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[34];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[35];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[36];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[37];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[38];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[39];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[40];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[41];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[42];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[43];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[44];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[45];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[46];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[47];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[48];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[49];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[50];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[51];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[52];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[53];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[54];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[55];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[56];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[57];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[58];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[59];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[60];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[61];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[62];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[63];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[64];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[65];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[66];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[67];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[68];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[69];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[70];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[71];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[72];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[73];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[74];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[75];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[76];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[77];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[78];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[79];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[80];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[81];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[82];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[83];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[84];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[85];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[86];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[87];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[88];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[89];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[90];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[91];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[92];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[93];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[94];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[95];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[96];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[97];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[98];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    obj = pTrackLogics[99];
    for (int i{0}; i < 50; i++) {
      obj->pRecentHistory[i] = false;
    }
    obj->pIsFirstUpdate = true;
    pLastTrackID = 0U;
  }
}

//
// Arguments    : void
// Return Type  : void
//
void trackFuser::resetImpl()
{
  trackHistoryLogic *obj;
  pNumLiveTracks = 0.0;
  pLastTimeStamp = -2.2204460492503131E-16;
  for (int i{0}; i < 100; i++) {
    pTrackIDs[i] = 0U;
  }
  for (int i{0}; i < 100; i++) {
    pConfirmedTracks[i] = false;
  }
  for (int i{0}; i < 20; i++) {
    pIsValidSource[i] = false;
  }
  obj = pTrackLogics[0];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[1];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[2];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[3];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[4];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[5];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[6];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[7];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[8];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[9];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[10];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[11];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[12];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[13];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[14];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[15];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[16];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[17];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[18];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[19];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[20];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[21];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[22];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[23];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[24];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[25];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[26];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[27];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[28];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[29];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[30];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[31];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[32];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[33];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[34];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[35];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[36];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[37];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[38];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[39];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[40];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[41];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[42];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[43];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[44];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[45];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[46];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[47];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[48];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[49];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[50];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[51];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[52];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[53];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[54];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[55];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[56];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[57];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[58];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[59];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[60];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[61];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[62];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[63];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[64];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[65];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[66];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[67];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[68];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[69];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[70];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[71];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[72];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[73];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[74];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[75];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[76];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[77];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[78];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[79];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[80];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[81];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[82];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[83];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[84];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[85];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[86];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[87];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[88];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[89];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[90];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[91];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[92];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[93];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[94];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[95];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[96];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[97];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[98];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  obj = pTrackLogics[99];
  for (int i{0}; i < 50; i++) {
    obj->pRecentHistory[i] = false;
  }
  obj->pIsFirstUpdate = true;
  pLastTrackID = 0U;
}

//
// Arguments    : const array<struct_T, 2U> &tracks
// Return Type  : void
//
void trackFuser::setupImpl(const array<struct_T, 2U> &tracks)
{
  fuserSourceConfiguration *source;
  fuserSourceConfiguration *sourceConfig;
  objectTrack b[100];
  objectTrack centralTrack;
  int i;
  boolean_T inKnownIDs[20];
  boolean_T exitg1;
  for (i = 0; i < 20; i++) {
    inKnownIDs[i] = (tracks[0].SourceIndex == pSourceConfigIDs[i]);
  }
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 20)) {
    if (inKnownIDs[i]) {
      sourceConfig = pSourceConfigurations[i];
      exitg1 = true;
    } else {
      i++;
    }
  }
  if (!sourceConfig->pIsTransformToCentralValid) {
    sourceConfig->pIsTransformToCentralValid = true;
  }
  for (i = 0; i < 20; i++) {
    inKnownIDs[i] = (tracks[0].SourceIndex == pSourceConfigIDs[i]);
  }
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 20)) {
    if (inKnownIDs[i]) {
      source = pSourceConfigurations[i];
      exitg1 = true;
    } else {
      i++;
    }
  }
  if (!source->pIsTransformToCentralValid) {
    source->pIsTransformToCentralValid = true;
  }
  centralTrack.TrackID = MAX_uint32_T;
  centralTrack.BranchID = MAX_uint32_T;
  centralTrack.SourceIndex = 1U;
  centralTrack.pUpdateTime = 0.0;
  centralTrack.Age = MAX_uint32_T;
  for (i = 0; i < 6; i++) {
    centralTrack.pState[i] = 0.0;
  }
  for (i = 0; i < 36; i++) {
    centralTrack.pStateCovariance[i] = iv[i];
  }
  centralTrack.IsConfirmed = false;
  centralTrack.IsCoasted = false;
  centralTrack.IsSelfReported = false;
  centralTrack.ObjectClassProbabilities = 1.0;
  centralTrack.ObjectClassID = 0.0;
  centralTrack.pTrackLogicState.set_size(1, 5);
  for (i = 0; i < 5; i++) {
    centralTrack.pTrackLogicState[i] = false;
  }
  for (i = 0; i < 100; i++) {
    b[i] = centralTrack;
  }
  for (i = 0; i < 100; i++) {
    pTracksList[i] = b[i];
  }
  pUsedConfigIDs.set_size(1, 20);
  for (i = 0; i < 20; i++) {
    pUsedConfigIDs[i] = false;
  }
  pUsedConfigIDs.set_size(2, 20);
  for (i = 0; i < 40; i++) {
    pUsedConfigIDs[i] = false;
  }
  pUsedConfigIDs.set_size(1, 20);
  for (i = 0; i < 20; i++) {
    pUsedConfigIDs[i] = false;
  }
}

//
// Arguments    : const array<struct_T, 2U> &localTracks
//                double tFusion
//                struct3_T confTracks_data[]
//                struct3_T tentTracks_data[]
//                array<struct3_T, 1U> &allTracks
//                unsigned int info_TrackIDsAtStepBeginning_data[]
//                int info_TrackIDsAtStepBeginning_size[2]
//                array<double, 2U> &info_CostMatrix
//                array<unsigned int, 2U> &info_Assignments
//                array<unsigned int, 1U> &info_UnassignedCentralTracks
//                array<unsigned int, 1U> &info_UnassignedLocalTracks
//                array<unsigned int, 1U> &info_NonInitializingLocalTracks
//                unsigned int info_InitializedCentralTrackIDs_data[]
//                int info_InitializedCentralTrackIDs_size[2]
//                array<unsigned int, 2U> &info_UpdatedCentralTrackIDs
//                unsigned int info_DeletedTrackIDs_data[]
//                int info_DeletedTrackIDs_size[2]
//                unsigned int info_TrackIDsAtStepEnd_data[]
//                int info_TrackIDsAtStepEnd_size[2]
//                int &tentTracks_size
// Return Type  : int
//
int trackFuser::stepImpl(
    const array<struct_T, 2U> &localTracks, double tFusion,
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
    unsigned int info_DeletedTrackIDs_data[], int info_DeletedTrackIDs_size[2],
    unsigned int info_TrackIDsAtStepEnd_data[],
    int info_TrackIDsAtStepEnd_size[2], int &tentTracks_size)
{
  static const char logicType[7]{'H', 'i', 's', 't', 'o', 'r', 'y'};
  fuserSourceConfiguration *source;
  fuserSourceConfiguration *thisSource;
  objectTrack obj;
  objectTrack track;
  trackHistoryLogic *b_obj;
  array<b_struct_T, 1U> allStructs;
  array<double, 2U> b_times;
  array<double, 2U> times;
  array<unsigned int, 2U> assigned;
  array<int, 2U> r1;
  array<unsigned int, 1U> assignedTrIDs;
  array<unsigned int, 1U> b_info_UnassignedLocalTracks;
  array<int, 1U> r;
  array<unsigned int, 1U> unassignedCentral;
  array<boolean_T, 2U> updated;
  array<boolean_T, 1U> isInitializing;
  b_struct_T oneStruct;
  double d;
  double prevNumLive;
  int b_end_tmp;
  int confTracks_size;
  int end_tmp;
  int i;
  int loop_ub;
  int loop_ub_tmp;
  int partialTrueCount;
  int trueCount;
  signed char b_tmp_data[100];
  signed char c_tmp_data[100];
  signed char d_tmp_data[100];
  signed char tmp_data[20];
  boolean_T deleted[100];
  boolean_T inKnownIDs[20];
  boolean_T exitg1;
  prevNumLive = pNumLiveTracks;
  if (std::isnan(prevNumLive)) {
    times.set_size(1, 1);
    times[0] = rtNaN;
  } else if (prevNumLive < 1.0) {
    times.set_size(1, 0);
  } else {
    times.set_size(1, static_cast<int>(prevNumLive - 1.0) + 1);
    loop_ub = static_cast<int>(prevNumLive - 1.0);
    for (i = 0; i <= loop_ub; i++) {
      times[i] = static_cast<double>(i) + 1.0;
    }
  }
  if (prevNumLive < 1.0) {
    loop_ub_tmp = 0;
  } else {
    loop_ub_tmp = static_cast<int>(prevNumLive);
  }
  info_TrackIDsAtStepBeginning_size[0] = 1;
  info_TrackIDsAtStepBeginning_size[1] = loop_ub_tmp;
  for (i = 0; i < loop_ub_tmp; i++) {
    info_TrackIDsAtStepBeginning_data[i] = pTrackIDs[i];
  }
  pUsedConfigIDs.set_size(localTracks.size(1), 20);
  loop_ub = localTracks.size(1) * 20;
  for (i = 0; i < loop_ub; i++) {
    pUsedConfigIDs[i] = false;
  }
  i = localTracks.size(1);
  for (int b_i{0}; b_i < i; b_i++) {
    boolean_T y;
    for (end_tmp = 0; end_tmp < 20; end_tmp++) {
      inKnownIDs[end_tmp] =
          (localTracks[b_i].SourceIndex == pSourceConfigIDs[end_tmp]);
    }
    trueCount = 0;
    partialTrueCount = 0;
    for (loop_ub = 0; loop_ub < 20; loop_ub++) {
      if (inKnownIDs[loop_ub]) {
        trueCount++;
        tmp_data[partialTrueCount] = static_cast<signed char>(loop_ub);
        partialTrueCount++;
      }
    }
    for (end_tmp = 0; end_tmp < trueCount; end_tmp++) {
      pUsedConfigIDs[b_i + pUsedConfigIDs.size(0) * tmp_data[end_tmp]] = true;
    }
    for (end_tmp = 0; end_tmp < trueCount; end_tmp++) {
      inKnownIDs[end_tmp] = !pIsValidSource[tmp_data[end_tmp]];
    }
    y = (trueCount != 0);
    if (y) {
      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub <= trueCount - 1)) {
        if (!inKnownIDs[loop_ub]) {
          y = false;
          exitg1 = true;
        } else {
          loop_ub++;
        }
      }
    }
    if (y) {
      for (end_tmp = 0; end_tmp < 20; end_tmp++) {
        inKnownIDs[end_tmp] =
            (localTracks[b_i].SourceIndex == pSourceConfigIDs[end_tmp]);
      }
      loop_ub = 0;
      exitg1 = false;
      while ((!exitg1) && (loop_ub < 20)) {
        if (inKnownIDs[loop_ub]) {
          source = pSourceConfigurations[loop_ub];
          exitg1 = true;
        } else {
          loop_ub++;
        }
      }
      if (!source->pIsTransformToCentralValid) {
        source->pIsTransformToCentralValid = true;
      }
      for (end_tmp = 0; end_tmp < trueCount; end_tmp++) {
        pIsValidSource[tmp_data[end_tmp]] = true;
      }
    }
  }
  distance(localTracks, info_CostMatrix);
  assign(info_CostMatrix, assigned, unassignedCentral,
         info_UnassignedLocalTracks);
  assignedTrIDs.set_size(assigned.size(0));
  loop_ub = assigned.size(0);
  for (i = 0; i < loop_ub; i++) {
    assignedTrIDs[i] = pTrackIDs[static_cast<int>(assigned[i]) - 1];
  }
  info_UnassignedCentralTracks.set_size(unassignedCentral.size(0));
  loop_ub = unassignedCentral.size(0);
  for (i = 0; i < loop_ub; i++) {
    info_UnassignedCentralTracks[i] =
        pTrackIDs[static_cast<int>(unassignedCentral[i]) - 1];
  }
  isInitializing.set_size(info_UnassignedLocalTracks.size(0));
  i = info_UnassignedLocalTracks.size(0);
  for (int b_i{0}; b_i < i; b_i++) {
    for (end_tmp = 0; end_tmp < 20; end_tmp++) {
      inKnownIDs[end_tmp] =
          (localTracks[static_cast<int>(info_UnassignedLocalTracks[b_i]) - 1]
               .SourceIndex == pSourceConfigIDs[end_tmp]);
    }
    loop_ub = 0;
    exitg1 = false;
    while ((!exitg1) && (loop_ub < 20)) {
      if (inKnownIDs[loop_ub]) {
        thisSource = pSourceConfigurations[loop_ub];
        exitg1 = true;
      } else {
        loop_ub++;
      }
    }
    isInitializing[b_i] = thisSource->IsInitializingCentralTracks;
  }
  b_end_tmp = isInitializing.size(0) - 1;
  trueCount = 0;
  for (int b_i{0}; b_i <= b_end_tmp; b_i++) {
    if (isInitializing[b_i]) {
      trueCount++;
    }
  }
  r.set_size(trueCount);
  partialTrueCount = 0;
  for (int b_i{0}; b_i <= b_end_tmp; b_i++) {
    if (isInitializing[b_i]) {
      r[partialTrueCount] = b_i;
      partialTrueCount++;
    }
  }
  b_info_UnassignedLocalTracks.set_size(r.size(0));
  loop_ub = r.size(0);
  for (i = 0; i < loop_ub; i++) {
    b_info_UnassignedLocalTracks[i] = info_UnassignedLocalTracks[r[i]];
  }
  initializeCentralTracks(localTracks, b_info_UnassignedLocalTracks);
  d = pNumLiveTracks;
  if (prevNumLive + 1.0 > d) {
    i = 0;
    end_tmp = 0;
  } else {
    i = static_cast<int>(prevNumLive + 1.0) - 1;
    end_tmp = static_cast<int>(d);
  }
  info_InitializedCentralTrackIDs_size[0] = 1;
  loop_ub = end_tmp - i;
  info_InitializedCentralTrackIDs_size[1] = loop_ub;
  for (end_tmp = 0; end_tmp < loop_ub; end_tmp++) {
    info_InitializedCentralTrackIDs_data[end_tmp] = pTrackIDs[i + end_tmp];
  }
  b_fuseAssigned(localTracks, assigned, updated);
  end_tmp = updated.size(1) - 1;
  trueCount = 0;
  for (int b_i{0}; b_i <= end_tmp; b_i++) {
    if (!updated[b_i]) {
      trueCount++;
    }
  }
  r1.set_size(1, trueCount);
  partialTrueCount = 0;
  for (int b_i{0}; b_i <= end_tmp; b_i++) {
    if (!updated[b_i]) {
      r1[partialTrueCount] = b_i;
      partialTrueCount++;
    }
  }
  b_times.set_size(1, r1.size(1));
  loop_ub = r1.size(1);
  for (i = 0; i < loop_ub; i++) {
    b_times[i] = times[r1[i]];
  }
  coastUnassigned(unassignedCentral, b_times, deleted);
  d = pNumLiveTracks;
  i = static_cast<int>(d);
  for (int b_i{0}; b_i < i; b_i++) {
    track = pTracksList[b_i];
    prevNumLive = tFusion - track.pUpdateTime;
    if (prevNumLive > 0.0) {
      double x[6];
      track = pTracksList[b_i];
      obj = pTracksList[b_i];
      fusion::internal::gaussEKFilter::predict(
          track.pState, obj.pStateCovariance, ProcessNoise, prevNumLive, x);
      track = pTracksList[b_i];
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        track.pState[loop_ub] = x[loop_ub];
      }
      std::copy(&obj.pStateCovariance[0], &obj.pStateCovariance[36],
                &track.pStateCovariance[0]);
      track.pUpdateTime = tFusion;
      pTracksList[b_i] = track;
    }
  }
  pLastTimeStamp = tFusion;
  d = pNumLiveTracks;
  if (d < 1.0) {
    loop_ub = 0;
  } else {
    loop_ub = static_cast<int>(d);
  }
  info_TrackIDsAtStepEnd_size[0] = 1;
  info_TrackIDsAtStepEnd_size[1] = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    info_TrackIDsAtStepEnd_data[i] = pTrackIDs[i];
  }
  if (info_UnassignedLocalTracks.size(0) == 0) {
    info_NonInitializingLocalTracks.set_size(0);
  } else {
    trueCount = 0;
    for (int b_i{0}; b_i <= b_end_tmp; b_i++) {
      if (!isInitializing[b_i]) {
        trueCount++;
      }
    }
    info_NonInitializingLocalTracks.set_size(trueCount);
    partialTrueCount = 0;
    for (int b_i{0}; b_i <= b_end_tmp; b_i++) {
      if (!isInitializing[b_i]) {
        info_NonInitializingLocalTracks[partialTrueCount] =
            info_UnassignedLocalTracks[b_i];
        partialTrueCount++;
      }
    }
  }
  info_Assignments.set_size(assignedTrIDs.size(0), 2);
  loop_ub = assignedTrIDs.size(0);
  for (i = 0; i < loop_ub; i++) {
    info_Assignments[i] = assignedTrIDs[i];
    info_Assignments[i + info_Assignments.size(0)] =
        assigned[i + assigned.size(0)];
  }
  trueCount = 0;
  for (int b_i{0}; b_i <= end_tmp; b_i++) {
    if (updated[b_i]) {
      trueCount++;
    }
  }
  info_UpdatedCentralTrackIDs.set_size(1, trueCount);
  partialTrueCount = 0;
  for (int b_i{0}; b_i <= end_tmp; b_i++) {
    if (updated[b_i]) {
      info_UpdatedCentralTrackIDs[partialTrueCount] =
          info_TrackIDsAtStepBeginning_data[b_i];
      partialTrueCount++;
    }
  }
  loop_ub = loop_ub_tmp - 1;
  trueCount = 0;
  partialTrueCount = 0;
  for (int b_i{0}; b_i <= loop_ub; b_i++) {
    if (deleted[b_i]) {
      trueCount++;
      b_tmp_data[partialTrueCount] = static_cast<signed char>(b_i);
      partialTrueCount++;
    }
  }
  info_DeletedTrackIDs_size[0] = 1;
  info_DeletedTrackIDs_size[1] = trueCount;
  for (i = 0; i < trueCount; i++) {
    info_DeletedTrackIDs_data[i] =
        info_TrackIDsAtStepBeginning_data[b_tmp_data[i]];
  }
  prevNumLive = pNumLiveTracks;
  if (std::isnan(prevNumLive)) {
    times.set_size(1, 1);
    times[0] = rtNaN;
  } else if (prevNumLive < 1.0) {
    times.set_size(1, 0);
  } else {
    times.set_size(1, static_cast<int>(prevNumLive - 1.0) + 1);
    loop_ub = static_cast<int>(prevNumLive - 1.0);
    for (i = 0; i <= loop_ub; i++) {
      times[i] = static_cast<double>(i) + 1.0;
    }
  }
  if (times.size(1) > 0) {
    loop_ub = static_cast<int>(times[0]) - 1;
    track = pTracksList[loop_ub];
    oneStruct.TrackID = track.TrackID;
    oneStruct.BranchID = track.BranchID;
    oneStruct.SourceIndex = track.SourceIndex;
    oneStruct.UpdateTime = track.pUpdateTime;
    oneStruct.Age = track.Age;
    for (int b_i{0}; b_i < 6; b_i++) {
      oneStruct.State[b_i] = track.pState[b_i];
    }
    std::copy(&track.pStateCovariance[0], &track.pStateCovariance[36],
              &oneStruct.StateCovariance[0]);
    oneStruct.ObjectClassID = track.ObjectClassID;
    oneStruct.ObjectClassProbabilities = track.ObjectClassProbabilities;
    for (i = 0; i < 7; i++) {
      oneStruct.TrackLogic[i] = logicType[i];
    }
    b_obj = pTrackLogics[loop_ub];
    for (i = 0; i < 5; i++) {
      oneStruct.TrackLogicState[i] = b_obj->pRecentHistory[i];
    }
    oneStruct.IsConfirmed = track.IsConfirmed;
    oneStruct.IsCoasted = track.IsCoasted;
    oneStruct.IsSelfReported = track.IsSelfReported;
    allStructs.set_size(times.size(1));
    loop_ub = times.size(1);
    for (i = 0; i < loop_ub; i++) {
      allStructs[i] = oneStruct;
    }
    i = times.size(1);
    for (int b_i{0}; b_i <= i - 2; b_i++) {
      d = times[b_i + 1];
      allStructs[b_i + 1].TrackID =
          pTracksList[static_cast<int>(d) - 1].TrackID;
      allStructs[b_i + 1].BranchID =
          pTracksList[static_cast<int>(d) - 1].BranchID;
      allStructs[b_i + 1].Age = pTracksList[static_cast<int>(d) - 1].Age;
      track = pTracksList[static_cast<int>(d) - 1];
      for (end_tmp = 0; end_tmp < 6; end_tmp++) {
        allStructs[b_i + 1].State[end_tmp] = track.pState[end_tmp];
      }
      track = pTracksList[static_cast<int>(d) - 1];
      for (end_tmp = 0; end_tmp < 36; end_tmp++) {
        allStructs[b_i + 1].StateCovariance[end_tmp] =
            track.pStateCovariance[end_tmp];
      }
      allStructs[b_i + 1].ObjectClassID =
          pTracksList[static_cast<int>(d) - 1].ObjectClassID;
      b_obj = pTrackLogics[static_cast<int>(d) - 1];
      for (end_tmp = 0; end_tmp < 5; end_tmp++) {
        allStructs[b_i + 1].TrackLogicState[end_tmp] =
            b_obj->pRecentHistory[end_tmp];
      }
      allStructs[b_i + 1].IsConfirmed =
          pTracksList[static_cast<int>(d) - 1].IsConfirmed;
      allStructs[b_i + 1].IsCoasted =
          pTracksList[static_cast<int>(d) - 1].IsCoasted;
      allStructs[b_i + 1].IsSelfReported =
          pTracksList[static_cast<int>(d) - 1].IsSelfReported;
    }
    allTracks.set_size(allStructs.size(0));
    i = allStructs.size(0);
    for (end_tmp = 0; end_tmp < i; end_tmp++) {
      allTracks[end_tmp].TrackID = allStructs[end_tmp].TrackID;
      allTracks[end_tmp].BranchID = allStructs[end_tmp].BranchID;
      allTracks[end_tmp].SourceIndex = allStructs[end_tmp].SourceIndex;
      allTracks[end_tmp].UpdateTime = allStructs[end_tmp].UpdateTime;
      allTracks[end_tmp].Age = allStructs[end_tmp].Age;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        allTracks[end_tmp].State[loop_ub] = allStructs[end_tmp].State[loop_ub];
      }
      for (loop_ub = 0; loop_ub < 36; loop_ub++) {
        allTracks[end_tmp].StateCovariance[loop_ub] =
            allStructs[end_tmp].StateCovariance[loop_ub];
      }
      allTracks[end_tmp].ObjectClassID = allStructs[end_tmp].ObjectClassID;
      allTracks[end_tmp].ObjectClassProbabilities =
          allStructs[end_tmp].ObjectClassProbabilities;
      for (loop_ub = 0; loop_ub < 7; loop_ub++) {
        allTracks[end_tmp].TrackLogic[loop_ub] =
            allStructs[end_tmp].TrackLogic[loop_ub];
      }
      allTracks[end_tmp].TrackLogicState.set_size(1, 5);
      for (loop_ub = 0; loop_ub < 5; loop_ub++) {
        allTracks[end_tmp].TrackLogicState[loop_ub] =
            allStructs[end_tmp].TrackLogicState[loop_ub];
      }
      allTracks[end_tmp].IsConfirmed = allStructs[end_tmp].IsConfirmed;
      allTracks[end_tmp].IsCoasted = allStructs[end_tmp].IsCoasted;
      allTracks[end_tmp].IsSelfReported = allStructs[end_tmp].IsSelfReported;
    }
  } else {
    allTracks.set_size(0);
  }
  confTracks_size = 0;
  for (int b_i{0}; b_i < 100; b_i++) {
    if (pConfirmedTracks[b_i]) {
      confTracks_size++;
    }
  }
  partialTrueCount = 0;
  for (int b_i{0}; b_i < 100; b_i++) {
    if (pConfirmedTracks[b_i]) {
      c_tmp_data[partialTrueCount] = static_cast<signed char>(b_i);
      partialTrueCount++;
    }
  }
  for (i = 0; i < confTracks_size; i++) {
    confTracks_data[i] = allTracks[static_cast<int>(c_tmp_data[i])];
  }
  d = pNumLiveTracks;
  if (d < 1.0) {
    i = 0;
  } else {
    i = static_cast<int>(d);
  }
  loop_ub = i - 1;
  tentTracks_size = 0;
  for (int b_i{0}; b_i <= loop_ub; b_i++) {
    if (!pConfirmedTracks[b_i]) {
      tentTracks_size++;
    }
  }
  partialTrueCount = 0;
  for (int b_i{0}; b_i <= loop_ub; b_i++) {
    if (!pConfirmedTracks[b_i]) {
      d_tmp_data[partialTrueCount] = static_cast<signed char>(b_i);
      partialTrueCount++;
    }
  }
  for (i = 0; i < tentTracks_size; i++) {
    tentTracks_data[i] = allTracks[static_cast<int>(d_tmp_data[i])];
  }
  return confTracks_size;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
} // namespace coder
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// Arguments    : void
// Return Type  : trackFuser
//
namespace coder {
trackFuser::trackFuser()
{
  matlabCodegenIsDeleted = true;
}

//
// Arguments    : void
// Return Type  : void
//
trackFuser::~trackFuser()
{
  matlabCodegenDestructor();
}

//
// Arguments    : const array<struct_T, 2U> &varargin_1
//                double varargin_2
//                struct3_T varargout_1_data[]
//                struct3_T varargout_2_data[]
//                array<struct3_T, 1U> &varargout_3
//                unsigned int varargout_4_TrackIDsAtStepBeginning_data[]
//                int varargout_4_TrackIDsAtStepBeginning_size[2]
//                array<double, 2U> &varargout_4_CostMatrix
//                array<unsigned int, 2U> &varargout_4_Assignments
//                array<unsigned int, 1U> &varargout_4_UnassignedCentralTracks
//                array<unsigned int, 1U> &varargout_4_UnassignedLocalTracks
//                array<unsigned int, 1U>
//                &varargout_4_NonInitializingLocalTracks unsigned int
//                varargout_4_InitializedCentralTrackIDs_data[] int
//                varargout_4_InitializedCentralTrackIDs_size[2] array<unsigned
//                int, 2U> &varargout_4_UpdatedCentralTrackIDs unsigned int
//                varargout_4_DeletedTrackIDs_data[] int
//                varargout_4_DeletedTrackIDs_size[2] unsigned int
//                varargout_4_TrackIDsAtStepEnd_data[] int
//                varargout_4_TrackIDsAtStepEnd_size[2] int &varargout_2_size
// Return Type  : int
//
int trackFuser::step(
    const array<struct_T, 2U> &varargin_1, double varargin_2,
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
    int varargout_4_TrackIDsAtStepEnd_size[2], int &varargout_2_size)
{
  unsigned int inSize[8];
  int k;
  boolean_T exitg1;
  if (isInitialized != 1) {
    cell_wrap_3 varSizes[2];
    double Q[9];
    isSetupComplete = false;
    isInitialized = 1;
    varSizes[0].f1[0] = 1U;
    varSizes[0].f1[1] = static_cast<unsigned int>(varargin_1.size(1));
    for (k = 0; k < 6; k++) {
      varSizes[0].f1[k + 2] = 1U;
    }
    for (k = 0; k < 8; k++) {
      varSizes[1].f1[k] = 1U;
    }
    inputVarSize[0] = varSizes[0];
    inputVarSize[1] = varSizes[1];
    setupImpl(varargin_1);
    cAssigner.isInitialized = 0;
    for (k = 0; k < 9; k++) {
      Q[k] = ProcessNoise[k];
    }
    for (k = 0; k < 9; k++) {
      cFuser.ProcessNoise[k] = Q[k];
    }
    for (k = 0; k < 50; k++) {
      _pobj0[0].pRecentHistory[k] = false;
    }
    pTrackLogics[0] = &_pobj0[0];
    for (k = 0; k < 50; k++) {
      _pobj0[1].pRecentHistory[k] = false;
    }
    pTrackLogics[1] = &_pobj0[1];
    for (k = 0; k < 50; k++) {
      _pobj0[2].pRecentHistory[k] = false;
    }
    pTrackLogics[2] = &_pobj0[2];
    for (k = 0; k < 50; k++) {
      _pobj0[3].pRecentHistory[k] = false;
    }
    pTrackLogics[3] = &_pobj0[3];
    for (k = 0; k < 50; k++) {
      _pobj0[4].pRecentHistory[k] = false;
    }
    pTrackLogics[4] = &_pobj0[4];
    for (k = 0; k < 50; k++) {
      _pobj0[5].pRecentHistory[k] = false;
    }
    pTrackLogics[5] = &_pobj0[5];
    for (k = 0; k < 50; k++) {
      _pobj0[6].pRecentHistory[k] = false;
    }
    pTrackLogics[6] = &_pobj0[6];
    for (k = 0; k < 50; k++) {
      _pobj0[7].pRecentHistory[k] = false;
    }
    pTrackLogics[7] = &_pobj0[7];
    for (k = 0; k < 50; k++) {
      _pobj0[8].pRecentHistory[k] = false;
    }
    pTrackLogics[8] = &_pobj0[8];
    for (k = 0; k < 50; k++) {
      _pobj0[9].pRecentHistory[k] = false;
    }
    pTrackLogics[9] = &_pobj0[9];
    for (k = 0; k < 50; k++) {
      _pobj0[10].pRecentHistory[k] = false;
    }
    pTrackLogics[10] = &_pobj0[10];
    for (k = 0; k < 50; k++) {
      _pobj0[11].pRecentHistory[k] = false;
    }
    pTrackLogics[11] = &_pobj0[11];
    for (k = 0; k < 50; k++) {
      _pobj0[12].pRecentHistory[k] = false;
    }
    pTrackLogics[12] = &_pobj0[12];
    for (k = 0; k < 50; k++) {
      _pobj0[13].pRecentHistory[k] = false;
    }
    pTrackLogics[13] = &_pobj0[13];
    for (k = 0; k < 50; k++) {
      _pobj0[14].pRecentHistory[k] = false;
    }
    pTrackLogics[14] = &_pobj0[14];
    for (k = 0; k < 50; k++) {
      _pobj0[15].pRecentHistory[k] = false;
    }
    pTrackLogics[15] = &_pobj0[15];
    for (k = 0; k < 50; k++) {
      _pobj0[16].pRecentHistory[k] = false;
    }
    pTrackLogics[16] = &_pobj0[16];
    for (k = 0; k < 50; k++) {
      _pobj0[17].pRecentHistory[k] = false;
    }
    pTrackLogics[17] = &_pobj0[17];
    for (k = 0; k < 50; k++) {
      _pobj0[18].pRecentHistory[k] = false;
    }
    pTrackLogics[18] = &_pobj0[18];
    for (k = 0; k < 50; k++) {
      _pobj0[19].pRecentHistory[k] = false;
    }
    pTrackLogics[19] = &_pobj0[19];
    for (k = 0; k < 50; k++) {
      _pobj0[20].pRecentHistory[k] = false;
    }
    pTrackLogics[20] = &_pobj0[20];
    for (k = 0; k < 50; k++) {
      _pobj0[21].pRecentHistory[k] = false;
    }
    pTrackLogics[21] = &_pobj0[21];
    for (k = 0; k < 50; k++) {
      _pobj0[22].pRecentHistory[k] = false;
    }
    pTrackLogics[22] = &_pobj0[22];
    for (k = 0; k < 50; k++) {
      _pobj0[23].pRecentHistory[k] = false;
    }
    pTrackLogics[23] = &_pobj0[23];
    for (k = 0; k < 50; k++) {
      _pobj0[24].pRecentHistory[k] = false;
    }
    pTrackLogics[24] = &_pobj0[24];
    for (k = 0; k < 50; k++) {
      _pobj0[25].pRecentHistory[k] = false;
    }
    pTrackLogics[25] = &_pobj0[25];
    for (k = 0; k < 50; k++) {
      _pobj0[26].pRecentHistory[k] = false;
    }
    pTrackLogics[26] = &_pobj0[26];
    for (k = 0; k < 50; k++) {
      _pobj0[27].pRecentHistory[k] = false;
    }
    pTrackLogics[27] = &_pobj0[27];
    for (k = 0; k < 50; k++) {
      _pobj0[28].pRecentHistory[k] = false;
    }
    pTrackLogics[28] = &_pobj0[28];
    for (k = 0; k < 50; k++) {
      _pobj0[29].pRecentHistory[k] = false;
    }
    pTrackLogics[29] = &_pobj0[29];
    for (k = 0; k < 50; k++) {
      _pobj0[30].pRecentHistory[k] = false;
    }
    pTrackLogics[30] = &_pobj0[30];
    for (k = 0; k < 50; k++) {
      _pobj0[31].pRecentHistory[k] = false;
    }
    pTrackLogics[31] = &_pobj0[31];
    for (k = 0; k < 50; k++) {
      _pobj0[32].pRecentHistory[k] = false;
    }
    pTrackLogics[32] = &_pobj0[32];
    for (k = 0; k < 50; k++) {
      _pobj0[33].pRecentHistory[k] = false;
    }
    pTrackLogics[33] = &_pobj0[33];
    for (k = 0; k < 50; k++) {
      _pobj0[34].pRecentHistory[k] = false;
    }
    pTrackLogics[34] = &_pobj0[34];
    for (k = 0; k < 50; k++) {
      _pobj0[35].pRecentHistory[k] = false;
    }
    pTrackLogics[35] = &_pobj0[35];
    for (k = 0; k < 50; k++) {
      _pobj0[36].pRecentHistory[k] = false;
    }
    pTrackLogics[36] = &_pobj0[36];
    for (k = 0; k < 50; k++) {
      _pobj0[37].pRecentHistory[k] = false;
    }
    pTrackLogics[37] = &_pobj0[37];
    for (k = 0; k < 50; k++) {
      _pobj0[38].pRecentHistory[k] = false;
    }
    pTrackLogics[38] = &_pobj0[38];
    for (k = 0; k < 50; k++) {
      _pobj0[39].pRecentHistory[k] = false;
    }
    pTrackLogics[39] = &_pobj0[39];
    for (k = 0; k < 50; k++) {
      _pobj0[40].pRecentHistory[k] = false;
    }
    pTrackLogics[40] = &_pobj0[40];
    for (k = 0; k < 50; k++) {
      _pobj0[41].pRecentHistory[k] = false;
    }
    pTrackLogics[41] = &_pobj0[41];
    for (k = 0; k < 50; k++) {
      _pobj0[42].pRecentHistory[k] = false;
    }
    pTrackLogics[42] = &_pobj0[42];
    for (k = 0; k < 50; k++) {
      _pobj0[43].pRecentHistory[k] = false;
    }
    pTrackLogics[43] = &_pobj0[43];
    for (k = 0; k < 50; k++) {
      _pobj0[44].pRecentHistory[k] = false;
    }
    pTrackLogics[44] = &_pobj0[44];
    for (k = 0; k < 50; k++) {
      _pobj0[45].pRecentHistory[k] = false;
    }
    pTrackLogics[45] = &_pobj0[45];
    for (k = 0; k < 50; k++) {
      _pobj0[46].pRecentHistory[k] = false;
    }
    pTrackLogics[46] = &_pobj0[46];
    for (k = 0; k < 50; k++) {
      _pobj0[47].pRecentHistory[k] = false;
    }
    pTrackLogics[47] = &_pobj0[47];
    for (k = 0; k < 50; k++) {
      _pobj0[48].pRecentHistory[k] = false;
    }
    pTrackLogics[48] = &_pobj0[48];
    for (k = 0; k < 50; k++) {
      _pobj0[49].pRecentHistory[k] = false;
    }
    pTrackLogics[49] = &_pobj0[49];
    for (k = 0; k < 50; k++) {
      _pobj0[50].pRecentHistory[k] = false;
    }
    pTrackLogics[50] = &_pobj0[50];
    for (k = 0; k < 50; k++) {
      _pobj0[51].pRecentHistory[k] = false;
    }
    pTrackLogics[51] = &_pobj0[51];
    for (k = 0; k < 50; k++) {
      _pobj0[52].pRecentHistory[k] = false;
    }
    pTrackLogics[52] = &_pobj0[52];
    for (k = 0; k < 50; k++) {
      _pobj0[53].pRecentHistory[k] = false;
    }
    pTrackLogics[53] = &_pobj0[53];
    for (k = 0; k < 50; k++) {
      _pobj0[54].pRecentHistory[k] = false;
    }
    pTrackLogics[54] = &_pobj0[54];
    for (k = 0; k < 50; k++) {
      _pobj0[55].pRecentHistory[k] = false;
    }
    pTrackLogics[55] = &_pobj0[55];
    for (k = 0; k < 50; k++) {
      _pobj0[56].pRecentHistory[k] = false;
    }
    pTrackLogics[56] = &_pobj0[56];
    for (k = 0; k < 50; k++) {
      _pobj0[57].pRecentHistory[k] = false;
    }
    pTrackLogics[57] = &_pobj0[57];
    for (k = 0; k < 50; k++) {
      _pobj0[58].pRecentHistory[k] = false;
    }
    pTrackLogics[58] = &_pobj0[58];
    for (k = 0; k < 50; k++) {
      _pobj0[59].pRecentHistory[k] = false;
    }
    pTrackLogics[59] = &_pobj0[59];
    for (k = 0; k < 50; k++) {
      _pobj0[60].pRecentHistory[k] = false;
    }
    pTrackLogics[60] = &_pobj0[60];
    for (k = 0; k < 50; k++) {
      _pobj0[61].pRecentHistory[k] = false;
    }
    pTrackLogics[61] = &_pobj0[61];
    for (k = 0; k < 50; k++) {
      _pobj0[62].pRecentHistory[k] = false;
    }
    pTrackLogics[62] = &_pobj0[62];
    for (k = 0; k < 50; k++) {
      _pobj0[63].pRecentHistory[k] = false;
    }
    pTrackLogics[63] = &_pobj0[63];
    for (k = 0; k < 50; k++) {
      _pobj0[64].pRecentHistory[k] = false;
    }
    pTrackLogics[64] = &_pobj0[64];
    for (k = 0; k < 50; k++) {
      _pobj0[65].pRecentHistory[k] = false;
    }
    pTrackLogics[65] = &_pobj0[65];
    for (k = 0; k < 50; k++) {
      _pobj0[66].pRecentHistory[k] = false;
    }
    pTrackLogics[66] = &_pobj0[66];
    for (k = 0; k < 50; k++) {
      _pobj0[67].pRecentHistory[k] = false;
    }
    pTrackLogics[67] = &_pobj0[67];
    for (k = 0; k < 50; k++) {
      _pobj0[68].pRecentHistory[k] = false;
    }
    pTrackLogics[68] = &_pobj0[68];
    for (k = 0; k < 50; k++) {
      _pobj0[69].pRecentHistory[k] = false;
    }
    pTrackLogics[69] = &_pobj0[69];
    for (k = 0; k < 50; k++) {
      _pobj0[70].pRecentHistory[k] = false;
    }
    pTrackLogics[70] = &_pobj0[70];
    for (k = 0; k < 50; k++) {
      _pobj0[71].pRecentHistory[k] = false;
    }
    pTrackLogics[71] = &_pobj0[71];
    for (k = 0; k < 50; k++) {
      _pobj0[72].pRecentHistory[k] = false;
    }
    pTrackLogics[72] = &_pobj0[72];
    for (k = 0; k < 50; k++) {
      _pobj0[73].pRecentHistory[k] = false;
    }
    pTrackLogics[73] = &_pobj0[73];
    for (k = 0; k < 50; k++) {
      _pobj0[74].pRecentHistory[k] = false;
    }
    pTrackLogics[74] = &_pobj0[74];
    for (k = 0; k < 50; k++) {
      _pobj0[75].pRecentHistory[k] = false;
    }
    pTrackLogics[75] = &_pobj0[75];
    for (k = 0; k < 50; k++) {
      _pobj0[76].pRecentHistory[k] = false;
    }
    pTrackLogics[76] = &_pobj0[76];
    for (k = 0; k < 50; k++) {
      _pobj0[77].pRecentHistory[k] = false;
    }
    pTrackLogics[77] = &_pobj0[77];
    for (k = 0; k < 50; k++) {
      _pobj0[78].pRecentHistory[k] = false;
    }
    pTrackLogics[78] = &_pobj0[78];
    for (k = 0; k < 50; k++) {
      _pobj0[79].pRecentHistory[k] = false;
    }
    pTrackLogics[79] = &_pobj0[79];
    for (k = 0; k < 50; k++) {
      _pobj0[80].pRecentHistory[k] = false;
    }
    pTrackLogics[80] = &_pobj0[80];
    for (k = 0; k < 50; k++) {
      _pobj0[81].pRecentHistory[k] = false;
    }
    pTrackLogics[81] = &_pobj0[81];
    for (k = 0; k < 50; k++) {
      _pobj0[82].pRecentHistory[k] = false;
    }
    pTrackLogics[82] = &_pobj0[82];
    for (k = 0; k < 50; k++) {
      _pobj0[83].pRecentHistory[k] = false;
    }
    pTrackLogics[83] = &_pobj0[83];
    for (k = 0; k < 50; k++) {
      _pobj0[84].pRecentHistory[k] = false;
    }
    pTrackLogics[84] = &_pobj0[84];
    for (k = 0; k < 50; k++) {
      _pobj0[85].pRecentHistory[k] = false;
    }
    pTrackLogics[85] = &_pobj0[85];
    for (k = 0; k < 50; k++) {
      _pobj0[86].pRecentHistory[k] = false;
    }
    pTrackLogics[86] = &_pobj0[86];
    for (k = 0; k < 50; k++) {
      _pobj0[87].pRecentHistory[k] = false;
    }
    pTrackLogics[87] = &_pobj0[87];
    for (k = 0; k < 50; k++) {
      _pobj0[88].pRecentHistory[k] = false;
    }
    pTrackLogics[88] = &_pobj0[88];
    for (k = 0; k < 50; k++) {
      _pobj0[89].pRecentHistory[k] = false;
    }
    pTrackLogics[89] = &_pobj0[89];
    for (k = 0; k < 50; k++) {
      _pobj0[90].pRecentHistory[k] = false;
    }
    pTrackLogics[90] = &_pobj0[90];
    for (k = 0; k < 50; k++) {
      _pobj0[91].pRecentHistory[k] = false;
    }
    pTrackLogics[91] = &_pobj0[91];
    for (k = 0; k < 50; k++) {
      _pobj0[92].pRecentHistory[k] = false;
    }
    pTrackLogics[92] = &_pobj0[92];
    for (k = 0; k < 50; k++) {
      _pobj0[93].pRecentHistory[k] = false;
    }
    pTrackLogics[93] = &_pobj0[93];
    for (k = 0; k < 50; k++) {
      _pobj0[94].pRecentHistory[k] = false;
    }
    pTrackLogics[94] = &_pobj0[94];
    for (k = 0; k < 50; k++) {
      _pobj0[95].pRecentHistory[k] = false;
    }
    pTrackLogics[95] = &_pobj0[95];
    for (k = 0; k < 50; k++) {
      _pobj0[96].pRecentHistory[k] = false;
    }
    pTrackLogics[96] = &_pobj0[96];
    for (k = 0; k < 50; k++) {
      _pobj0[97].pRecentHistory[k] = false;
    }
    pTrackLogics[97] = &_pobj0[97];
    for (k = 0; k < 50; k++) {
      _pobj0[98].pRecentHistory[k] = false;
    }
    pTrackLogics[98] = &_pobj0[98];
    for (k = 0; k < 50; k++) {
      _pobj0[99].pRecentHistory[k] = false;
    }
    pTrackLogics[99] = &_pobj0[99];
    isSetupComplete = true;
    TunablePropsChanged = false;
    resetImpl();
  }
  if (TunablePropsChanged) {
    TunablePropsChanged = false;
  }
  inSize[0] = 1U;
  inSize[1] = static_cast<unsigned int>(varargin_1.size(1));
  for (k = 0; k < 6; k++) {
    inSize[k + 2] = 1U;
  }
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 8)) {
    if (inputVarSize[0].f1[k] != inSize[k]) {
      for (k = 0; k < 8; k++) {
        inputVarSize[0].f1[k] = inSize[k];
      }
      exitg1 = true;
    } else {
      k++;
    }
  }
  return stepImpl(
      varargin_1, varargin_2, varargout_1_data, varargout_2_data, varargout_3,
      varargout_4_TrackIDsAtStepBeginning_data,
      varargout_4_TrackIDsAtStepBeginning_size, varargout_4_CostMatrix,
      varargout_4_Assignments, varargout_4_UnassignedCentralTracks,
      varargout_4_UnassignedLocalTracks, varargout_4_NonInitializingLocalTracks,
      varargout_4_InitializedCentralTrackIDs_data,
      varargout_4_InitializedCentralTrackIDs_size,
      varargout_4_UpdatedCentralTrackIDs, varargout_4_DeletedTrackIDs_data,
      varargout_4_DeletedTrackIDs_size, varargout_4_TrackIDsAtStepEnd_data,
      varargout_4_TrackIDsAtStepEnd_size, varargout_2_size);
}

} // namespace coder

//
// File trailer for trackFuser.cpp
//
// [EOF]
//
