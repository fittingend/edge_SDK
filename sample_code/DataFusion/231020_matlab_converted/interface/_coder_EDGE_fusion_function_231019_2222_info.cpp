//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_EDGE_fusion_function_231019_2222_info.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "_coder_EDGE_fusion_function_231019_2222_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

// Function Declarations
static const mxArray *emlrtMexFcnResolvedFunctionsInfo();

// Function Definitions
//
// Arguments    : void
// Return Type  : const mxArray *
//
static const mxArray *emlrtMexFcnResolvedFunctionsInfo()
{
  const mxArray *nameCaptureInfo;
  const char_T *data[4]{
      "789cc553bb4ec330147550412c2d99f882ce9d6047442a0a8212a9a1426a909a386ee3d6"
      "8eabc445e9c6c2cc2f30f245f02b8c280fe72559094594b35c1f1df9"
      "9ee36b1b28faad0200e88114b3a3b47633ae66f5005451d715491538049dca3ea1bf6615"
      "329fa388a7c4b729ca77ba8c62dff6b9b95d2310a0909127e426ca1c",
      "1364628ac665328a191d96a49cc452bcd63c0457e30d05811716094999e4f378939cb7d3"
      "721e179279a8357daa6b8f7d6a73623b0163bc6f71c688c3222bf4ec"
      "00b9160f6cb8c2fec29a6f42cc7c821d8b394b04b9190b032af2ae7f99b757e3f5bc424f"
      "f2dcb00586c9bd14ef6747ff3a64fe02c2ef7d473fd15f6bf013fa54",
      "1ffdf47eaa231ad04aee9924d749cbdcb27fd605c7495d7e7e28fbf47b365ebef6e927f0"
      "5f7e91a45fdb77772af1536bbacbeeefce1c4a0c783e712faf09dc3e"
      "4cae86450ea3c1a7290790f0bfeeff0df20074c7",
      ""};
  nameCaptureInfo = nullptr;
  emlrtNameCaptureMxArrayR2016a(&data[0], 1656U, &nameCaptureInfo);
  return nameCaptureInfo;
}

//
// Arguments    : void
// Return Type  : mxArray *
//
mxArray *emlrtMexFcnProperties()
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9]{"Version",
                                 "ResolvedFunctions",
                                 "Checksum",
                                 "EntryPoints",
                                 "CoverageInfo",
                                 "IsPolymorphic",
                                 "PropertyList",
                                 "UUID",
                                 "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8]{
      "Name",     "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "FullPath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 1);
  emlrtSetField(xEntryPoints, 0, "Name",
                emlrtMxCreateString("EDGE_fusion_function_231019_2222"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("D:\\BACKUP\\backup_230803\\data_arrange\\EDGE_"
                          "fusion_function_231019_2222.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739178.93211805553));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("23.2.0.2365128 (R2023b)"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)emlrtMexFcnResolvedFunctionsInfo());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("Z7SsGYacnNs5mQXnlmqABF"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

//
// File trailer for _coder_EDGE_fusion_function_231019_2222_info.cpp
//
// [EOF]
//
