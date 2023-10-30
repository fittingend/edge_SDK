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
static void argInit_1x3_HubObstacleData(HubObstacleData result[3]);

static void argInit_1x5_HubVehicleData(HubVehicleData result[5]);

static void argInit_3x3_real_T(double result[9]);

static double argInit_real_T();

static void argInit_HubData(HubData &result);

static void argInit_HubVehicleData(HubVehicleData &result);

static void argInit_HubObstacleData(HubObstacleData &result);

// Function Definitions
//
// Arguments    : HubObstacleData result[3]
// Return Type  : void
//
static void argInit_1x3_HubObstacleData(HubObstacleData result[3])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 3; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    argInit_HubObstacleData(result[idx1]);
  }
}

//
// Arguments    : HubVehicleData result[5]
// Return Type  : void
//
static void argInit_1x5_HubVehicleData(HubVehicleData result[5])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 5; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    argInit_HubVehicleData(result[idx1]);
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
// Arguments    : HubData &result
// Return Type  : void
//
static void argInit_HubData(HubData &result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_1x5_HubVehicleData(result.vehicle);
}

//
// Arguments    : HubVehicleData &result
// Return Type  : void
//
static void argInit_HubVehicleData(HubVehicleData &result)
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
  argInit_1x3_HubObstacleData(result.obstacle);
  result.road_z = result_tmp;
}

//
// Arguments    : HubObstacleData &result
// Return Type  : void
//
static void argInit_HubObstacleData(HubObstacleData &result)
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
// int main(int, char **)
// {
//   // The initialize function is being called automatically from your entry-point
//   // function. So, a call to initialize is not included here. Invoke the
//   // entry-point functions.
//   // You can call entry-point functions multiple times.
//   matlab_fusion();
//   // Terminate the application.
//   // You do not need to do this more than one time.
//   EDGE_fusion_function_231019_2222_terminate();
//   return 0;
// }

//
// Arguments    : void
// Return Type  : void
//
void matlab_fusion(HubData hub_data_Object)
{
  coder::array<struct3_T, 1U> allTracks;
  coder::bounded_array<struct3_T, 100U, 1U> confirmedTracks;
  coder::bounded_array<struct3_T, 100U, 1U> tentativeTracks;
  struct4_T analysisInformation;
  argInit_HubData(hub_data_Object); // TODO:뭐하는건지 확인필요
  EDGE_fusion_function_231019_2222(&hub_data_Object, confirmedTracks.data, confirmedTracks.size, tentativeTracks.data,
      tentativeTracks.size, allTracks, &analysisInformation);
}

//
// File trailer for main.cpp
//
// [EOF]
//
