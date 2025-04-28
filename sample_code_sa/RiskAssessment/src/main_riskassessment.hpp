#ifndef MAIN_RISKASSESSMENT_HPP
#define MAIN_RISKASSESSMENT_HPP
#include <thread>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <csignal>
#include <stdio.h>
#include <unordered_set>
#include <mutex>
#include <cmath>
#inclued <vector>

#include <ara/com/e2exf/status_handler.h>
#include <ara/exec/execution_client.h>

#include <ara/log/logger.h>
#include "logger.h"
#include "ara/core/initialization.h"

#include "risk_assessment_provider.h"
#include "build_path_subscriber.h"
#include "build_path_test_subscriber.h"
#include "map_data_subscriber.h"

#define map_n 2000
#define map_m 1000
#define MAP_TILT_ANGLE -86
#define STOP_VALUE 10 // 1초간 정지시 정지 장애물로 판단
#define INVALID_RETURN_VALUE 99999 
#define M_TO_10CM_PRECISION 10 
#define MAP_ANGLE -86
#define WHEEL_DIAMETER_M 0.71

mutex mtx_map;
mutex mtx_rass;
mutex mtx_path;

std::vector<adcm::map_2dListVector> map_2d;
obstacleListVector obstacle_list_temp;
adcm::vehicleListStruct ego_vehicle_temp, sub_vehicle_1_temp, sub_vehicle_2_temp, sub_vehicle_3_temp, sub_vehicle_4_temp;
doubleVector position_x;
doubleVector position_y;
obstacleListVector obstacle_pedes_initial;   // 시나리오 5 용
obstacleListVector obstacle_vehicle_initial; // 시나리오 6 용


//함수선언
void clear_and_free_memory(std::vector<int>& vec); 

using namespace std;
typedef struct
{
    double x, y;
} Point2D; 

enum ObstacleClass
{
    NO_OBSTACLE,
    VEHICLE_LARGE,
    VEHICLE_SMALL,
    PEDESTRIAN,
    STRUCTURE
};

enum VehicleClass
{
    EGO_VEHICLE = 0xF0,
    SUB_VEHICLE_1 = 0x00,
    SUB_VEHICLE_2 = 0x01,
    SUB_VEHICLE_3 = 0x02,
    SUB_VEHICLE_4 = 0x03,
    NO_VEHICLE = 0x04
};

enum HazardClass
{
    NO_HAZARD,
    SCENARIO_1,
    SCENARIO_2,
    SCENARIO_3,
    SCENARIO_4,
    SCENARIO_5,
    SCENARIO_6,
    SCENARIO_7,
    SCENARIO_8
};

namespace
{
    // Atomic flag for exit after SIGTERM caught
    std::atomic_bool continueExecution{true};
    std::atomic_uint gReceivedEvent_count_map_data{0};
    std::atomic_uint gReceivedEvent_count_build_path{0};
    std::atomic_uint gReceivedEvent_count_build_path_test{0};
    std::atomic_uint gMainthread_Loopcount{0};

    void SigTermHandler(int signal)
    {
        if(signal == SIGTERM) {
            // set atomic exit flag
            continueExecution = false;
        }
    }

    bool RegisterSigTermHandler()
    {
        struct sigaction sa;
        sa.sa_handler = SigTermHandler;
        sa.sa_flags = 0;
        sigemptyset(&sa.sa_mask);

        // register signal handler
        if(sigaction(SIGTERM, &sa, NULL) == -1) {
            // Could not register a SIGTERM signal handler
            return false;
        }

        return true;
    }
}

#endif