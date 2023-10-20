//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 19-Oct-2023 22:29:03
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include Files
#include "main.h"
#include "EDGE_fusion_function_231019_2222.h"
#include "EDGE_fusion_function_231019_2222_terminate.h"
#include "EDGE_fusion_function_231019_2222_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include "coder_bounded_array.h"

// Function Declarations
static void argInit_1x3_struct2_T(struct2_T result[3]);

static void argInit_1x5_struct1_T(struct1_T result[5]);

static void argInit_3x3_real_T(double result[9]);

static double argInit_real_T();

static void argInit_struct0_T(struct0_T &result);

static void argInit_struct1_T(struct1_T &result);

static void argInit_struct2_T(struct2_T &result);

// Function Definitions
//
// Arguments    : struct2_T result[3]
// Return Type  : void
//
static void argInit_1x3_struct2_T(struct2_T result[3])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 3; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    argInit_struct2_T(result[idx1]);
  }
}

//
// Arguments    : struct1_T result[5]
// Return Type  : void
//
static void argInit_1x5_struct1_T(struct1_T result[5])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 5; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    argInit_struct1_T(result[idx1]);
  }
}

//
// Arguments    : double result[9]
// Return Type  : void
//
static void argInit_3x3_real_T(double result[9])
{
  // Loop over the array to initialize each element.
  for (int i{0}; i < 9; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[i] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : struct0_T &result
// Return Type  : void
//
static void argInit_struct0_T(struct0_T &result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_1x5_struct1_T(result.vehicle);
}

//
// Arguments    : struct1_T &result
// Return Type  : void
//
static void argInit_struct1_T(struct1_T &result)
{
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.vehicle_class = result_tmp;
  result.Position_lat = result_tmp;
  result.Position_long = result_tmp;
  result.Position_Height = result_tmp;
  result.Yaw = result_tmp;
  result.Roll = result_tmp;
  result.Pitch = result_tmp;
  result.Velocity_long = result_tmp;
  result.Velocity_lat = result_tmp;
  result.Velocity_ang = result_tmp;
  argInit_1x3_struct2_T(result.obstacle);
  result.road_z = result_tmp;
}

//
// Arguments    : struct2_T &result
// Return Type  : void
//
static void argInit_struct2_T(struct2_T &result)
{
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.obstacle_class = result_tmp;
  result.cuboid_x = result_tmp;
  result.cuboid_y = result_tmp;
  result.cuboid_z = result_tmp;
  result.heading_angle = result_tmp;
  result.Position_x = result_tmp;
  result.Position_y = result_tmp;
  result.Position_z = result_tmp;
  result.Velocity_x = result_tmp;
  result.Velocity_y = result_tmp;
  result.Velocity_z = result_tmp;
  result.timestamp = result_tmp;
  argInit_3x3_real_T(result.covariance_matrix);
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_EDGE_fusion_function_231019_2222();
  // Terminate the application.
  // You do not need to do this more than one time.
  EDGE_fusion_function_231019_2222_terminate();
  return 0;
}

//
// Arguments    : void
// Return Type  : void
//
void main_EDGE_fusion_function_231019_2222()
{
  coder::array<struct3_T, 1U> allTracks;
  coder::bounded_array<struct3_T, 100U, 1U> confirmedTracks;
  coder::bounded_array<struct3_T, 100U, 1U> tentativeTracks;
  struct0_T r;
  struct4_T analysisInformation;
  // Initialize function 'EDGE_fusion_function_231019_2222' input arguments.
  // Initialize function input argument 'hub_data_Object'.
  // Call the entry-point 'EDGE_fusion_function_231019_2222'.
  argInit_struct0_T(r);
  EDGE_fusion_function_231019_2222(
      &r, confirmedTracks.data, confirmedTracks.size, tentativeTracks.data,
      tentativeTracks.size, allTracks, &analysisInformation);
}

//
// File trailer for main.cpp
//
// [EOF]
//
