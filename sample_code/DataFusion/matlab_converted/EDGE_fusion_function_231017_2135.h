//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: EDGE_fusion_function_231017_2135.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 17-Oct-2023 22:24:52
//

#ifndef EDGE_FUSION_FUNCTION_231017_2135_H
#define EDGE_FUSION_FUNCTION_231017_2135_H

// Include Files
#include "EDGE_fusion_function_231017_2135_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void EDGE_fusion_function_231017_2135(
    const HubData *hub_data_Object, struct3_T confirmedTracks_data[],
    int confirmedTracks_size[1], struct3_T tentativeTracks_data[],
    int tentativeTracks_size[1], coder::array<struct3_T, 1U> &allTracks,
    struct4_T *analysisInformation);

extern void EDGE_fusion_function_231017_2135_initialize();

extern void EDGE_fusion_function_231017_2135_terminate();

#endif
//
// File trailer for EDGE_fusion_function_231017_2135.h
//
// [EOF]
//
