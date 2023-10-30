//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_EDGE_fusion_function_231019_2222_api.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "_coder_EDGE_fusion_function_231019_2222_api.h"
#include "_coder_EDGE_fusion_function_231019_2222_mex.h"
#include "coder_array_mex.h"
#include "coder_bounded_array.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131643U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "EDGE_fusion_function_231019_2222",                   // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

static const char_T *sv[16]{"TrackID",
                            "BranchID",
                            "SourceIndex",
                            "UpdateTime",
                            "Age",
                            "State",
                            "StateCovariance",
                            "StateParameters",
                            "ObjectClassID",
                            "ObjectClassProbabilities",
                            "TrackLogic",
                            "TrackLogicState",
                            "IsConfirmed",
                            "IsCoasted",
                            "IsSelfReported",
                            "ObjectAttributes"};

// Function Declarations
static void b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[9]);

static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, HubData &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, HubData &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             HubVehicleData y[5]);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId,
                             HubObstacleData y[3]);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[9]);

static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static const mxArray *emlrt_marshallOut(const struct4_T &u);

static const mxArray *emlrt_marshallOut(const uint32_T u_data[],
                                        const int32_T u_size[2]);

static const mxArray *emlrt_marshallOut(const emlrtStack &sp,
                                        const struct3_T u_data[],
                                        const int32_T &u_size);

static const mxArray *emlrt_marshallOut(const emlrtStack &sp,
                                        const coder::array<struct3_T, 1U> &u);

static const mxArray *emlrt_marshallOut(const real_T u[36]);

// Function Definitions
//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
//                real_T ret[9]
// Return Type  : void
//
static void b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[9])
{
  static const int32_T dims[2]{3, 3};
  real_T(*r)[9];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[9])emlrtMxGetData(src);
  for (int32_T i{0}; i < 9; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T
//
static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *b_nullptr
//                const char_T *identifier
//                HubData &y
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, HubData &y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId, y);
  emlrtDestroyArray(&b_nullptr);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
//                HubData &y
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, HubData &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames{"vehicle"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 1,
                         (const char_T **)&fieldNames, 0U, (const void *)&dims);
  thisId.fIdentifier = "vehicle";
  emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0, "vehicle")),
      &thisId, y.vehicle);
  emlrtDestroyArray(&u);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
//                HubVehicleData y[5]
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, HubVehicleData y[5])
{
  static const int32_T dims[2]{1, 5};
  static const char_T *fieldNames[12]{
      "obstacle",      "road_z",          "vehicle_class", "Position_lat",
      "Position_long", "Position_Height", "Yaw",           "Roll",
      "Pitch",         "Velocity_long",   "Velocity_lat",  "Velocity_ang"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 12,
                         (const char_T **)&fieldNames[0], 2U,
                         (const void *)&dims[0]);
  for (int32_T i{0}; i < 5; i++) {
    thisId.fIdentifier = "obstacle";
    emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, i, 0,
                                                    "obstacle")),
                     &thisId, y[i].obstacle);
    thisId.fIdentifier = "road_z";
    y[i].road_z = emlrt_marshallIn(
        sp,
        emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, i, 1, "road_z")),
        &thisId);
    thisId.fIdentifier = "vehicle_class";
    y[i].vehicle_class =
        emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        i, 2, "vehicle_class")),
                         &thisId);
    thisId.fIdentifier = "Position_lat";
    y[i].Position_lat =
        emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        i, 3, "Position_lat")),
                         &thisId);
    thisId.fIdentifier = "Position_long";
    y[i].Position_long =
        emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        i, 4, "Position_long")),
                         &thisId);
    thisId.fIdentifier = "Position_Height";
    y[i].Position_Height =
        emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, i, 5, "Position_Height")),
                         &thisId);
    thisId.fIdentifier = "Yaw";
    y[i].Yaw = emlrt_marshallIn(
        sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, i, 6, "Yaw")),
        &thisId);
    thisId.fIdentifier = "Roll";
    y[i].Roll = emlrt_marshallIn(
        sp,
        emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, i, 7, "Roll")),
        &thisId);
    thisId.fIdentifier = "Pitch";
    y[i].Pitch = emlrt_marshallIn(
        sp,
        emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, i, 8, "Pitch")),
        &thisId);
    thisId.fIdentifier = "Velocity_long";
    y[i].Velocity_long =
        emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        i, 9, "Velocity_long")),
                         &thisId);
    thisId.fIdentifier = "Velocity_lat";
    y[i].Velocity_lat =
        emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        i, 10, "Velocity_lat")),
                         &thisId);
    thisId.fIdentifier = "Velocity_ang";
    y[i].Velocity_ang =
        emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u,
                                                        i, 11, "Velocity_ang")),
                         &thisId);
  }
  emlrtDestroyArray(&u);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
//                HubObstacleData y[3]
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, HubObstacleData y[3])
{
  static const int32_T dims[2]{1, 3};
  static const char_T *fieldNames[13]{
      "timestamp",  "obstacle_class", "cuboid_x",          "cuboid_y",
      "cuboid_z",   "heading_angle",  "covariance_matrix", "Position_x",
      "Position_y", "Position_z",     "Velocity_x",        "Velocity_y",
      "Velocity_z"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 13,
                         (const char_T **)&fieldNames[0], 2U,
                         (const void *)&dims[0]);
  thisId.fIdentifier = "timestamp";
  y[0].timestamp = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0, "timestamp")),
      &thisId);
  thisId.fIdentifier = "obstacle_class";
  y[0].obstacle_class =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      1, "obstacle_class")),
                       &thisId);
  thisId.fIdentifier = "cuboid_x";
  y[0].cuboid_x = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2, "cuboid_x")),
      &thisId);
  thisId.fIdentifier = "cuboid_y";
  y[0].cuboid_y = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3, "cuboid_y")),
      &thisId);
  thisId.fIdentifier = "cuboid_z";
  y[0].cuboid_z = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4, "cuboid_z")),
      &thisId);
  thisId.fIdentifier = "heading_angle";
  y[0].heading_angle =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      5, "heading_angle")),
                       &thisId);
  thisId.fIdentifier = "covariance_matrix";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 6,
                                                  "covariance_matrix")),
                   &thisId, y[0].covariance_matrix);
  thisId.fIdentifier = "Position_x";
  y[0].Position_x =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      7, "Position_x")),
                       &thisId);
  thisId.fIdentifier = "Position_y";
  y[0].Position_y =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      8, "Position_y")),
                       &thisId);
  thisId.fIdentifier = "Position_z";
  y[0].Position_z =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      9, "Position_z")),
                       &thisId);
  thisId.fIdentifier = "Velocity_x";
  y[0].Velocity_x =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      10, "Velocity_x")),
                       &thisId);
  thisId.fIdentifier = "Velocity_y";
  y[0].Velocity_y =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      11, "Velocity_y")),
                       &thisId);
  thisId.fIdentifier = "Velocity_z";
  y[0].Velocity_z =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0,
                                                      12, "Velocity_z")),
                       &thisId);
  thisId.fIdentifier = "timestamp";
  y[1].timestamp = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1, 0, "timestamp")),
      &thisId);
  thisId.fIdentifier = "obstacle_class";
  y[1].obstacle_class =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1,
                                                      1, "obstacle_class")),
                       &thisId);
  thisId.fIdentifier = "cuboid_x";
  y[1].cuboid_x = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1, 2, "cuboid_x")),
      &thisId);
  thisId.fIdentifier = "cuboid_y";
  y[1].cuboid_y = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1, 3, "cuboid_y")),
      &thisId);
  thisId.fIdentifier = "cuboid_z";
  y[1].cuboid_z = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1, 4, "cuboid_z")),
      &thisId);
  thisId.fIdentifier = "heading_angle";
  y[1].heading_angle =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1,
                                                      5, "heading_angle")),
                       &thisId);
  thisId.fIdentifier = "covariance_matrix";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1, 6,
                                                  "covariance_matrix")),
                   &thisId, y[1].covariance_matrix);
  thisId.fIdentifier = "Position_x";
  y[1].Position_x =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1,
                                                      7, "Position_x")),
                       &thisId);
  thisId.fIdentifier = "Position_y";
  y[1].Position_y =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1,
                                                      8, "Position_y")),
                       &thisId);
  thisId.fIdentifier = "Position_z";
  y[1].Position_z =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1,
                                                      9, "Position_z")),
                       &thisId);
  thisId.fIdentifier = "Velocity_x";
  y[1].Velocity_x =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1,
                                                      10, "Velocity_x")),
                       &thisId);
  thisId.fIdentifier = "Velocity_y";
  y[1].Velocity_y =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1,
                                                      11, "Velocity_y")),
                       &thisId);
  thisId.fIdentifier = "Velocity_z";
  y[1].Velocity_z =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 1,
                                                      12, "Velocity_z")),
                       &thisId);
  thisId.fIdentifier = "timestamp";
  y[2].timestamp = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2, 0, "timestamp")),
      &thisId);
  thisId.fIdentifier = "obstacle_class";
  y[2].obstacle_class =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2,
                                                      1, "obstacle_class")),
                       &thisId);
  thisId.fIdentifier = "cuboid_x";
  y[2].cuboid_x = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2, 2, "cuboid_x")),
      &thisId);
  thisId.fIdentifier = "cuboid_y";
  y[2].cuboid_y = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2, 3, "cuboid_y")),
      &thisId);
  thisId.fIdentifier = "cuboid_z";
  y[2].cuboid_z = emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2, 4, "cuboid_z")),
      &thisId);
  thisId.fIdentifier = "heading_angle";
  y[2].heading_angle =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2,
                                                      5, "heading_angle")),
                       &thisId);
  thisId.fIdentifier = "covariance_matrix";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2, 6,
                                                  "covariance_matrix")),
                   &thisId, y[2].covariance_matrix);
  thisId.fIdentifier = "Position_x";
  y[2].Position_x =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2,
                                                      7, "Position_x")),
                       &thisId);
  thisId.fIdentifier = "Position_y";
  y[2].Position_y =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2,
                                                      8, "Position_y")),
                       &thisId);
  thisId.fIdentifier = "Position_z";
  y[2].Position_z =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2,
                                                      9, "Position_z")),
                       &thisId);
  thisId.fIdentifier = "Velocity_x";
  y[2].Velocity_x =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2,
                                                      10, "Velocity_x")),
                       &thisId);
  thisId.fIdentifier = "Velocity_y";
  y[2].Velocity_y =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2,
                                                      11, "Velocity_y")),
                       &thisId);
  thisId.fIdentifier = "Velocity_z";
  y[2].Velocity_z =
      emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 2,
                                                      12, "Velocity_z")),
                       &thisId);
  emlrtDestroyArray(&u);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
//                real_T y[9]
// Return Type  : void
//
static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[9])
{
  b_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T
//
static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = b_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const struct4_T &u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const struct4_T &u)
{
  static const char_T *b_sv[10]{"TrackIDsAtStepBeginning",
                                "CostMatrix",
                                "Assignments",
                                "UnassignedCentralTracks",
                                "UnassignedLocalTracks",
                                "NonInitializingLocalTracks",
                                "InitializedCentralTrackIDs",
                                "UpdatedCentralTrackIDs",
                                "DeletedTrackIDs",
                                "TrackIDsAtStepEnd"};
  const coder::array<uint32_T, 1U> *b_u;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T iv[2];
  int32_T i;
  uint32_T *b_pData;
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 10, (const char_T **)&b_sv[0]));
  emlrtSetFieldR2017b(y, 0, "TrackIDsAtStepBeginning",
                      emlrt_marshallOut(u.TrackIDsAtStepBeginning.data,
                                        u.TrackIDsAtStepBeginning.size),
                      0);
  b_y = nullptr;
  iv[0] = u.CostMatrix.size(0);
  iv[1] = u.CostMatrix.size(1);
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (int32_T b_i{0}; b_i < u.CostMatrix.size(1); b_i++) {
    for (int32_T c_i{0}; c_i < u.CostMatrix.size(0); c_i++) {
      pData[i] = u.CostMatrix[c_i + u.CostMatrix.size(0) * b_i];
      i++;
    }
  }
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "CostMatrix", b_y, 1);
  c_y = nullptr;
  iv[0] = u.Assignments.size(0);
  iv[1] = 2;
  m = emlrtCreateNumericArray(2, &iv[0], mxUINT32_CLASS, mxREAL);
  b_pData = static_cast<uint32_T *>(emlrtMxGetData(m));
  i = 0;
  for (int32_T b_i{0}; b_i < u.Assignments.size(0); b_i++) {
    b_pData[i] = u.Assignments[b_i];
    i++;
  }
  for (int32_T b_i{0}; b_i < u.Assignments.size(0); b_i++) {
    b_pData[i] = u.Assignments[b_i + u.Assignments.size(0)];
    i++;
  }
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, "Assignments", c_y, 2);
  b_u = &u.UnassignedCentralTracks;
  d_y = nullptr;
  m = emlrtCreateNumericArray(
      1, ((coder::array<uint32_T, 1U> *)&u.UnassignedCentralTracks)->size(),
      mxUINT32_CLASS, mxREAL);
  b_pData = static_cast<uint32_T *>(emlrtMxGetData(m));
  i = 0;
  for (int32_T b_i{0}; b_i < b_u->size(0); b_i++) {
    b_pData[i] = (*b_u)[b_i];
    i++;
  }
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, "UnassignedCentralTracks", d_y, 3);
  b_u = &u.UnassignedLocalTracks;
  e_y = nullptr;
  m = emlrtCreateNumericArray(
      1, ((coder::array<uint32_T, 1U> *)&u.UnassignedLocalTracks)->size(),
      mxUINT32_CLASS, mxREAL);
  b_pData = static_cast<uint32_T *>(emlrtMxGetData(m));
  i = 0;
  for (int32_T b_i{0}; b_i < b_u->size(0); b_i++) {
    b_pData[i] = (*b_u)[b_i];
    i++;
  }
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, "UnassignedLocalTracks", e_y, 4);
  b_u = &u.NonInitializingLocalTracks;
  f_y = nullptr;
  m = emlrtCreateNumericArray(
      1, ((coder::array<uint32_T, 1U> *)&u.NonInitializingLocalTracks)->size(),
      mxUINT32_CLASS, mxREAL);
  b_pData = static_cast<uint32_T *>(emlrtMxGetData(m));
  i = 0;
  for (int32_T b_i{0}; b_i < b_u->size(0); b_i++) {
    b_pData[i] = (*b_u)[b_i];
    i++;
  }
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, "NonInitializingLocalTracks", f_y, 5);
  emlrtSetFieldR2017b(y, 0, "InitializedCentralTrackIDs",
                      emlrt_marshallOut(u.InitializedCentralTrackIDs.data,
                                        u.InitializedCentralTrackIDs.size),
                      6);
  g_y = nullptr;
  iv[0] = 1;
  iv[1] = u.UpdatedCentralTrackIDs.size(1);
  m = emlrtCreateNumericArray(2, &iv[0], mxUINT32_CLASS, mxREAL);
  b_pData = static_cast<uint32_T *>(emlrtMxGetData(m));
  i = 0;
  for (int32_T b_i{0}; b_i < u.UpdatedCentralTrackIDs.size(1); b_i++) {
    b_pData[i] = u.UpdatedCentralTrackIDs[b_i];
    i++;
  }
  emlrtAssign(&g_y, m);
  emlrtSetFieldR2017b(y, 0, "UpdatedCentralTrackIDs", g_y, 7);
  emlrtSetFieldR2017b(
      y, 0, "DeletedTrackIDs",
      emlrt_marshallOut(u.DeletedTrackIDs.data, u.DeletedTrackIDs.size), 8);
  emlrtSetFieldR2017b(
      y, 0, "TrackIDsAtStepEnd",
      emlrt_marshallOut(u.TrackIDsAtStepEnd.data, u.TrackIDsAtStepEnd.size), 9);
  return y;
}

//
// Arguments    : const uint32_T u_data[]
//                const int32_T u_size[2]
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const uint32_T u_data[],
                                        const int32_T u_size[2])
{
  const mxArray *m;
  const mxArray *y;
  int32_T iv[2];
  int32_T i;
  uint32_T *pData;
  y = nullptr;
  iv[0] = 1;
  iv[1] = u_size[1];
  m = emlrtCreateNumericArray(2, &iv[0], mxUINT32_CLASS, mxREAL);
  pData = static_cast<uint32_T *>(emlrtMxGetData(m));
  i = 0;
  for (int32_T b_i{0}; b_i < u_size[1]; b_i++) {
    pData[i] = u_data[b_i];
    i++;
  }
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const struct3_T u_data[]
//                const int32_T &u_size
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const emlrtStack &sp,
                                        const struct3_T u_data[],
                                        const int32_T &u_size)
{
  static const int32_T iv[2]{1, 7};
  static const int32_T b_i{6};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *i_y;
  const mxArray *j_y;
  const mxArray *k_y;
  const mxArray *l_y;
  const mxArray *m;
  const mxArray *m_y;
  const mxArray *n_y;
  const mxArray *o_y;
  const mxArray *p_y;
  const mxArray *y;
  real_T *pData;
  int32_T iv1[2];
  int32_T i;
  y = nullptr;
  emlrtAssign(&y,
              emlrtCreateStructArray(1, &u_size, 16, (const char_T **)&sv[0]));
  emlrtCreateField(y, "TrackID");
  emlrtCreateField(y, "BranchID");
  emlrtCreateField(y, "SourceIndex");
  emlrtCreateField(y, "UpdateTime");
  emlrtCreateField(y, "Age");
  emlrtCreateField(y, "State");
  emlrtCreateField(y, "StateCovariance");
  emlrtCreateField(y, "StateParameters");
  emlrtCreateField(y, "ObjectClassID");
  emlrtCreateField(y, "ObjectClassProbabilities");
  emlrtCreateField(y, "TrackLogic");
  emlrtCreateField(y, "TrackLogicState");
  emlrtCreateField(y, "IsConfirmed");
  emlrtCreateField(y, "IsCoasted");
  emlrtCreateField(y, "IsSelfReported");
  emlrtCreateField(y, "ObjectAttributes");
  i = 0;
  for (int32_T b_j0{0}; b_j0 < u_size; b_j0++) {
    b_y = nullptr;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *static_cast<uint32_T *>(emlrtMxGetData(m)) = u_data[b_j0].TrackID;
    emlrtAssign(&b_y, m);
    emlrtSetFieldR2017b(y, i, "TrackID", b_y, 0);
    c_y = nullptr;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *static_cast<uint32_T *>(emlrtMxGetData(m)) = u_data[b_j0].BranchID;
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(y, i, "BranchID", c_y, 1);
    d_y = nullptr;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *static_cast<uint32_T *>(emlrtMxGetData(m)) = u_data[b_j0].SourceIndex;
    emlrtAssign(&d_y, m);
    emlrtSetFieldR2017b(y, i, "SourceIndex", d_y, 2);
    e_y = nullptr;
    m = emlrtCreateDoubleScalar(u_data[b_j0].UpdateTime);
    emlrtAssign(&e_y, m);
    emlrtSetFieldR2017b(y, i, "UpdateTime", e_y, 3);
    f_y = nullptr;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *static_cast<uint32_T *>(emlrtMxGetData(m)) = u_data[b_j0].Age;
    emlrtAssign(&f_y, m);
    emlrtSetFieldR2017b(y, i, "Age", f_y, 4);
    g_y = nullptr;
    m = emlrtCreateNumericArray(1, (const void *)&b_i, mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    for (int32_T c_i{0}; c_i < 6; c_i++) {
      pData[c_i] = u_data[b_j0].State[c_i];
    }
    emlrtAssign(&g_y, m);
    emlrtSetFieldR2017b(y, i, "State", g_y, 5);
    emlrtSetFieldR2017b(y, i, "StateCovariance",
                        emlrt_marshallOut(u_data[b_j0].StateCovariance), 6);
    h_y = nullptr;
    emlrtAssign(&h_y, emlrtCreateStructMatrix(1, 1, 0, nullptr));
    emlrtSetFieldR2017b(y, i, "StateParameters", h_y, 7);
    i_y = nullptr;
    m = emlrtCreateDoubleScalar(u_data[b_j0].ObjectClassID);
    emlrtAssign(&i_y, m);
    emlrtSetFieldR2017b(y, i, "ObjectClassID", i_y, 8);
    j_y = nullptr;
    m = emlrtCreateDoubleScalar(u_data[b_j0].ObjectClassProbabilities);
    emlrtAssign(&j_y, m);
    emlrtSetFieldR2017b(y, i, "ObjectClassProbabilities", j_y, 9);
    k_y = nullptr;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a((emlrtConstCTX)&sp, 7, m,
                             &u_data[b_j0].TrackLogic[0]);
    emlrtAssign(&k_y, m);
    emlrtSetFieldR2017b(y, i, "TrackLogic", k_y, 10);
    l_y = nullptr;
    iv1[0] = 1;
    iv1[1] = u_data[b_j0].TrackLogicState.size(1);
    m = emlrtCreateLogicalArray(2, &iv1[0]);
    emlrtInitLogicalArray(u_data[b_j0].TrackLogicState.size(1), m,
                          &u_data[b_j0].TrackLogicState[0]);
    emlrtAssign(&l_y, m);
    emlrtSetFieldR2017b(y, i, "TrackLogicState", l_y, 11);
    m_y = nullptr;
    m = emlrtCreateLogicalScalar(u_data[b_j0].IsConfirmed);
    emlrtAssign(&m_y, m);
    emlrtSetFieldR2017b(y, i, "IsConfirmed", m_y, 12);
    n_y = nullptr;
    m = emlrtCreateLogicalScalar(u_data[b_j0].IsCoasted);
    emlrtAssign(&n_y, m);
    emlrtSetFieldR2017b(y, i, "IsCoasted", n_y, 13);
    o_y = nullptr;
    m = emlrtCreateLogicalScalar(u_data[b_j0].IsSelfReported);
    emlrtAssign(&o_y, m);
    emlrtSetFieldR2017b(y, i, "IsSelfReported", o_y, 14);
    p_y = nullptr;
    emlrtAssign(&p_y, emlrtCreateStructMatrix(1, 1, 0, nullptr));
    emlrtSetFieldR2017b(y, i, "ObjectAttributes", p_y, 15);
    i++;
  }
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const coder::array<struct3_T, 1U> &u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const emlrtStack &sp,
                                        const coder::array<struct3_T, 1U> &u)
{
  static const int32_T iv[2]{1, 7};
  static const int32_T b_i{6};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *i_y;
  const mxArray *j_y;
  const mxArray *k_y;
  const mxArray *l_y;
  const mxArray *m;
  const mxArray *m_y;
  const mxArray *n_y;
  const mxArray *o_y;
  const mxArray *p_y;
  const mxArray *y;
  real_T *pData;
  int32_T iv1[2];
  int32_T i;
  y = nullptr;
  emlrtAssign(
      &y, emlrtCreateStructArray(1, ((coder::array<struct3_T, 1U> *)&u)->size(),
                                 16, (const char_T **)&sv[0]));
  emlrtCreateField(y, "TrackID");
  emlrtCreateField(y, "BranchID");
  emlrtCreateField(y, "SourceIndex");
  emlrtCreateField(y, "UpdateTime");
  emlrtCreateField(y, "Age");
  emlrtCreateField(y, "State");
  emlrtCreateField(y, "StateCovariance");
  emlrtCreateField(y, "StateParameters");
  emlrtCreateField(y, "ObjectClassID");
  emlrtCreateField(y, "ObjectClassProbabilities");
  emlrtCreateField(y, "TrackLogic");
  emlrtCreateField(y, "TrackLogicState");
  emlrtCreateField(y, "IsConfirmed");
  emlrtCreateField(y, "IsCoasted");
  emlrtCreateField(y, "IsSelfReported");
  emlrtCreateField(y, "ObjectAttributes");
  i = 0;
  for (int32_T b_j0{0}; b_j0 < u.size(0); b_j0++) {
    b_y = nullptr;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *static_cast<uint32_T *>(emlrtMxGetData(m)) = u[b_j0].TrackID;
    emlrtAssign(&b_y, m);
    emlrtSetFieldR2017b(y, i, "TrackID", b_y, 0);
    c_y = nullptr;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *static_cast<uint32_T *>(emlrtMxGetData(m)) = u[b_j0].BranchID;
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(y, i, "BranchID", c_y, 1);
    d_y = nullptr;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *static_cast<uint32_T *>(emlrtMxGetData(m)) = u[b_j0].SourceIndex;
    emlrtAssign(&d_y, m);
    emlrtSetFieldR2017b(y, i, "SourceIndex", d_y, 2);
    e_y = nullptr;
    m = emlrtCreateDoubleScalar(u[b_j0].UpdateTime);
    emlrtAssign(&e_y, m);
    emlrtSetFieldR2017b(y, i, "UpdateTime", e_y, 3);
    f_y = nullptr;
    m = emlrtCreateNumericMatrix(1, 1, mxUINT32_CLASS, mxREAL);
    *static_cast<uint32_T *>(emlrtMxGetData(m)) = u[b_j0].Age;
    emlrtAssign(&f_y, m);
    emlrtSetFieldR2017b(y, i, "Age", f_y, 4);
    g_y = nullptr;
    m = emlrtCreateNumericArray(1, (const void *)&b_i, mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    for (int32_T c_i{0}; c_i < 6; c_i++) {
      pData[c_i] = u[b_j0].State[c_i];
    }
    emlrtAssign(&g_y, m);
    emlrtSetFieldR2017b(y, i, "State", g_y, 5);
    emlrtSetFieldR2017b(y, i, "StateCovariance",
                        emlrt_marshallOut(u[b_j0].StateCovariance), 6);
    h_y = nullptr;
    emlrtAssign(&h_y, emlrtCreateStructMatrix(1, 1, 0, nullptr));
    emlrtSetFieldR2017b(y, i, "StateParameters", h_y, 7);
    i_y = nullptr;
    m = emlrtCreateDoubleScalar(u[b_j0].ObjectClassID);
    emlrtAssign(&i_y, m);
    emlrtSetFieldR2017b(y, i, "ObjectClassID", i_y, 8);
    j_y = nullptr;
    m = emlrtCreateDoubleScalar(u[b_j0].ObjectClassProbabilities);
    emlrtAssign(&j_y, m);
    emlrtSetFieldR2017b(y, i, "ObjectClassProbabilities", j_y, 9);
    k_y = nullptr;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a((emlrtConstCTX)&sp, 7, m, &u[b_j0].TrackLogic[0]);
    emlrtAssign(&k_y, m);
    emlrtSetFieldR2017b(y, i, "TrackLogic", k_y, 10);
    l_y = nullptr;
    iv1[0] = 1;
    iv1[1] = u[b_j0].TrackLogicState.size(1);
    m = emlrtCreateLogicalArray(2, &iv1[0]);
    emlrtInitLogicalArray(u[b_j0].TrackLogicState.size(1), m,
                          &u[b_j0].TrackLogicState[0]);
    emlrtAssign(&l_y, m);
    emlrtSetFieldR2017b(y, i, "TrackLogicState", l_y, 11);
    m_y = nullptr;
    m = emlrtCreateLogicalScalar(u[b_j0].IsConfirmed);
    emlrtAssign(&m_y, m);
    emlrtSetFieldR2017b(y, i, "IsConfirmed", m_y, 12);
    n_y = nullptr;
    m = emlrtCreateLogicalScalar(u[b_j0].IsCoasted);
    emlrtAssign(&n_y, m);
    emlrtSetFieldR2017b(y, i, "IsCoasted", n_y, 13);
    o_y = nullptr;
    m = emlrtCreateLogicalScalar(u[b_j0].IsSelfReported);
    emlrtAssign(&o_y, m);
    emlrtSetFieldR2017b(y, i, "IsSelfReported", o_y, 14);
    p_y = nullptr;
    emlrtAssign(&p_y, emlrtCreateStructMatrix(1, 1, 0, nullptr));
    emlrtSetFieldR2017b(y, i, "ObjectAttributes", p_y, 15);
    i++;
  }
  return y;
}

//
// Arguments    : const real_T u[36]
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u[36])
{
  static const int32_T iv[2]{6, 6};
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T i;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (int32_T b_i{0}; b_i < 6; b_i++) {
    for (int32_T c_i{0}; c_i < 6; c_i++) {
      pData[i + c_i] = u[c_i + 6 * b_i];
    }
    i += 6;
  }
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const mxArray *prhs
//                int32_T nlhs
//                const mxArray *plhs[4]
// Return Type  : void
//
void EDGE_fusion_function_231019_2222_api(const mxArray *prhs, int32_T nlhs,
                                          const mxArray *plhs[4])
{
  coder::array<struct3_T, 1U> allTracks;
  coder::bounded_array<struct3_T, 100U, 1U> confirmedTracks;
  coder::bounded_array<struct3_T, 100U, 1U> tentativeTracks;
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  HubData hub_data_Object;
  struct4_T analysisInformation;
  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  // Marshall function inputs
  emlrt_marshallIn(st, emlrtAliasP(prhs), "hub_data_Object", hub_data_Object);
  // Invoke the target function
  EDGE_fusion_function_231019_2222(&hub_data_Object, confirmedTracks.data,
                                   confirmedTracks.size, tentativeTracks.data,
                                   tentativeTracks.size, allTracks,
                                   &analysisInformation);
  // Marshall function outputs
  plhs[0] =
      emlrt_marshallOut(st, confirmedTracks.data, confirmedTracks.size[0]);
  if (nlhs > 1) {
    plhs[1] =
        emlrt_marshallOut(st, tentativeTracks.data, tentativeTracks.size[0]);
  }
  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(st, allTracks);
  }
  if (nlhs > 3) {
    plhs[3] = emlrt_marshallOut(analysisInformation);
  }
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

//
// Arguments    : void
// Return Type  : void
//
void EDGE_fusion_function_231019_2222_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  EDGE_fusion_function_231019_2222_xil_terminate();
  EDGE_fusion_function_231019_2222_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void EDGE_fusion_function_231019_2222_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void EDGE_fusion_function_231019_2222_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// File trailer for _coder_EDGE_fusion_function_231019_2222_api.cpp
//
// [EOF]
//
