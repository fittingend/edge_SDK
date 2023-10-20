//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_EDGE_fusion_function_231019_2222_mex.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

// Include Files
#include "_coder_EDGE_fusion_function_231019_2222_mex.h"
#include "_coder_EDGE_fusion_function_231019_2222_api.h"

// Function Definitions
//
// Arguments    : int32_T nlhs
//                mxArray *plhs[]
//                int32_T nrhs
//                const mxArray *prhs[]
// Return Type  : void
//
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&EDGE_fusion_function_231019_2222_atexit);
  // Module initialization.
  EDGE_fusion_function_231019_2222_initialize();
  // Dispatch the entry-point.
  unsafe_EDGE_fusion_function_231019_2222_mexFunction(nlhs, plhs, nrhs, prhs);
  // Module termination.
  EDGE_fusion_function_231019_2222_terminate();
}

//
// Arguments    : void
// Return Type  : emlrtCTX
//
emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr, "windows-949", true);
  return emlrtRootTLSGlobal;
}

//
// Arguments    : int32_T nlhs
//                mxArray *plhs[4]
//                int32_T nrhs
//                const mxArray *prhs[1]
// Return Type  : void
//
void unsafe_EDGE_fusion_function_231019_2222_mexFunction(int32_T nlhs,
                                                         mxArray *plhs[4],
                                                         int32_T nrhs,
                                                         const mxArray *prhs[1])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *outputs[4];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 1, 4,
                        32, "EDGE_fusion_function_231019_2222");
  }
  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 32,
                        "EDGE_fusion_function_231019_2222");
  }
  // Call the function.
  EDGE_fusion_function_231019_2222_api(prhs[0], nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

//
// File trailer for _coder_EDGE_fusion_function_231019_2222_mex.cpp
//
// [EOF]
//
