// --------------------------------------------------------------------------
// |              _    _ _______     .----.      _____         _____        |
// |         /\  | |  | |__   __|  .  ____ .    / ____|  /\   |  __ \       |
// |        /  \ | |  | |  | |    .  / __ \ .  | (___   /  \  | |__) |      |
// |       / /\ \| |  | |  | |   .  / / / / v   \___ \ / /\ \ |  _  /       |
// |      / /__\ \ |__| |  | |   . / /_/ /  .   ____) / /__\ \| | \ \       |
// |     /________\____/   |_|   ^ \____/  .   |_____/________\_|  \_\      |
// |                              . _ _  .                                  |
// --------------------------------------------------------------------------
//
// All Rights Reserved.
// Any use of this source code is subject to a license agreement with the
// AUTOSAR development cooperation.
// More information is available at www.autosar.org.
//
// Disclaimer
//
// This work (specification and/or software implementation) and the material
// contained in it, as released by AUTOSAR, is for the purpose of information
// only. AUTOSAR and the companies that have contributed to it shall not be
// liable for any use of the work.
//
// The material contained in this work is protected by copyright and other
// types of intellectual property rights. The commercial exploitation of the
// material contained in this work requires a license to such intellectual
// property rights.
//
// This work may be utilized or reproduced without any modification, in any
// form or by any means, for informational purposes only. For any other
// purpose, no part of the work may be utilized or reproduced, in any form
// or by any means, without permission in writing to the publisher.
//
// The work has been developed for automotive applications only. It has
// neither been developed, nor tested for non-automotive applications.
//
// The word AUTOSAR and the AUTOSAR logo are registered trademarks.
// --------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////
// This is just a test main to test the communication API
// The different components radar, video, tester, ... are used in one
// application. This could be also different applications but we
// currently have no mechanism implemeted for inter-process-communication
// between applications. We also have no execution environment in use here
// I.e. this code as nothing to do with the communication or execution API
///////////////////////////////////////////////////////////////////////

#include <thread>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <csignal>
#include <stdio.h>
#include <random>

#include "main_datafusion.hpp"

#include <ara/com/e2exf/status_handler.h>
#include <ara/exec/execution_client.h>

#include <ara/log/logger.h>
#include "logger.h"
#include "ara/core/initialization.h"

#include "map_data_provider.h"

#include "hub_data_subscriber.h"
#include "work_information_subscriber.h"

VehicleData main_vehicle_temp;
VehicleData sub1_vehicle_temp;
VehicleData sub2_vehicle_temp;

std::vector<ObstacleData> obstacle_list_temp;
long ContourX[map_m][2];
bool once = 1;
// 차량 크기(work_information data)
std::uint16_t main_vehicle_size_length;
std::uint16_t main_vehicle_size_width;
std::vector<VehicleSizeData> sub_vehicle_size;
std::vector<BoundaryData> work_boundary;
double min_x, min_y, max_x, max_y;

bool checkRange(VehicleData vehicle)
{
    bool range_OK = false;

    if (vehicle.position_x > 0 && vehicle.position_x < map_n &&
        vehicle.position_y > 0 && vehicle.position_y < map_m &&
        abs(vehicle.velocity_x) < 500 && abs(vehicle.velocity_y) < 500)
    {
        range_OK = true;
    }

    return range_OK;
}

void checkRange(Point2D &point)
{

    if (point.x > map_n)
    {
        point.x = map_n;
    }
    if (point.y > map_m)
    {
        point.y = map_m;
    }
}
// map배열의 한 점이 실제 다각형 map내부에 있는지 확인

bool isValid(int x, int y)
{
    int cross = 0;
    std::vector<BoundaryData> p = work_boundary;

    for (int i = 0; i < p.size(); i++)
    {
        int j = (i + 1) % p.size();
        if ((p[i].y > y) != (p[j].y > y))
        {
            double meetX = (p[j].x - p[i].x) * (y - p[i].y) / (p[j].y - p[i].y) + p[i].x;
            if (x < meetX)
                cross++;
        }
    }
    return cross % 2 > 0;
}

void globalToLocalcoordinate(VehicleData &vehicle)
{
    // 시뮬레이션의 global 좌표계를 작업환경 XY 기반의 local 좌표계로 변환하는 함수
    double alpha = 537.92;
    double beta = -416.58;
    double theta = vehicle.yaw * M_PI / 180;
    double velocity_ang = vehicle.velocity_ang;
    double position_x = vehicle.position_long;
    double position_y = vehicle.position_lat;
    double velocity_x = vehicle.velocity_long;
    double velocity_y = vehicle.velocity_lat;

    vehicle.position_x = (cos(theta) * (position_x - alpha) + sin(theta) * (position_y - beta)) * M_TO_10CM_PRECISION;
    vehicle.position_y = (-sin(theta) * (position_x - alpha) + cos(theta) * (position_y - beta)) * M_TO_10CM_PRECISION;
    vehicle.velocity_x = (velocity_ang * (-sin(theta) * (position_x - alpha) + (cos(theta) * (position_y - beta)))) + (velocity_x * cos(theta)) + (velocity_y * sin(theta));
    vehicle.velocity_y = (velocity_ang * (-cos(theta) * (position_x - alpha) - (sin(theta) * (position_y - beta)))) + (velocity_x * -sin(theta)) + (velocity_y * cos(theta));

    vehicle.yaw = vehicle.yaw - theta;
    adcm::Log::Info() << "차량 globalToLocalcoordinate 좌표변환 before (" << position_x << " , " << position_y << " , " << velocity_x << " , " << velocity_y << ")";
    adcm::Log::Info() << "차량 globalToLocalcoordinate 좌표변환 after(" << vehicle.position_x << " , " << vehicle.position_y << " , " << vehicle.velocity_x << " , " << vehicle.velocity_y << ")";
    // adcm::Log::Info() << "차량 globalToLocalcoordinate timestamp: " << vehicle.timestamp;
}
void globalToLocalcoordinate(std::vector<ObstacleData> &obstacle_list, VehicleData main_vehicle)
{
    // 시뮬레이션의 global 좌표계를 작업환경 XY 기반의 local 좌표계로 변환하는 함수
    double alpha = 537.92;
    double beta = -416.58;
    double velocity_ang = main_vehicle.velocity_ang;
    double theta = main_vehicle.yaw * M_PI / 180;

    for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
    {
        double position_x = iter->fused_position_x;
        double position_y = iter->fused_position_y;
        double velocity_x = iter->fused_velocity_x;
        double velocity_y = iter->fused_velocity_y;

        iter->fused_position_x = (cos(theta) * (position_x - alpha) + sin(theta) * (position_y - beta)) * M_TO_10CM_PRECISION;
        iter->fused_position_y = (-sin(theta) * (position_x - alpha) + cos(theta) * (position_y - beta)) * M_TO_10CM_PRECISION;

        iter->fused_velocity_x = (velocity_ang * (-sin(theta) * (position_x - alpha) + (cos(theta) * (position_y - beta)))) + (velocity_x * cos(theta)) + (velocity_y * sin(theta));

        iter->fused_velocity_y = (velocity_ang * (-cos(theta) * (position_x - alpha) - (sin(theta) * (position_y - beta)))) + (velocity_x * -sin(theta)) + (velocity_y * cos(theta));

        iter->fused_heading_angle = iter->fused_heading_angle - main_vehicle.yaw;

        adcm::Log::Info() << "장애물 globalToLocalcoordinate 좌표변환 (" << iter->fused_position_x << " , " << iter->fused_position_y << " , " << iter->fused_velocity_x << " , " << iter->fused_velocity_y << ")";
        // adcm::Log::Info() << "해당 timestamp: " << iter->timestamp;
    }
}
void relativeToGlobalcoordinate(std::vector<ObstacleData> &obstacle_list, VehicleData main_vehicle)
{
    double theta = main_vehicle.yaw * M_PI / 180;
    double velocity_ang = main_vehicle.velocity_ang;

    for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
    {
        adcm::Log::Info() << "장애물 relativeToGlobal 좌표변환 전 (" << iter->fused_position_x << " , " << iter->fused_position_y << " , " << iter->fused_velocity_x << " , " << iter->fused_velocity_y << ")";

        double obstacle_position_x = iter->fused_position_x;
        double obstacle_position_y = iter->fused_position_y;
        double obstacle_velocity_x = iter->fused_velocity_x;
        double obstacle_velocity_y = iter->fused_velocity_y;

        iter->fused_position_x = main_vehicle.position_long + (obstacle_position_x)*cos(theta) - (obstacle_position_y)*sin(theta);
        iter->fused_position_y = main_vehicle.position_lat + (obstacle_position_x)*sin(theta) - (obstacle_position_y)*cos(theta);

        iter->fused_velocity_x = main_vehicle.velocity_long + (obstacle_velocity_x * cos(theta)) - (velocity_ang * obstacle_position_x * sin(theta)) - ((obstacle_velocity_y)*sin(theta)) - (velocity_ang * obstacle_position_y * cos(theta));

        iter->fused_velocity_y = main_vehicle.velocity_lat + (obstacle_velocity_x * sin(theta)) + (velocity_ang * obstacle_position_x * cos(theta)) + (obstacle_velocity_y * cos(theta)) - (velocity_ang * obstacle_position_y * sin(theta));

        iter->fused_heading_angle = main_vehicle.yaw + iter->fused_heading_angle;

        adcm::Log::Info() << "장애물 relativeToGlobal 좌표변환 (" << iter->fused_position_x << " , " << iter->fused_position_y << " , " << iter->fused_velocity_x << " , " << iter->fused_velocity_y << ")";
    }
}
void ScanLine(long x1, long y1, long x2, long y2, long min_y, long max_y)
{
    long sx, sy, dx1, dy1, dx2, dy2, x, y, m, n, k, cnt;

    sx = x2 - x1;
    sy = y2 - y1;

    if (sx > 0)
        dx1 = 1;
    else if (sx < 0)
        dx1 = -1;
    else
        dx1 = 0;

    if (sy > 0)
        dy1 = 1;
    else if (sy < 0)
        dy1 = -1;
    else
        dy1 = 0;

    m = ABS(sx);
    n = ABS(sy);
    dx2 = dx1;
    dy2 = 0;

    if (m < n)
    {
        m = ABS(sy);
        n = ABS(sx);
        dx2 = 0;
        dy2 = dy1;
    }

    x = x1;
    y = y1;
    cnt = m + 1;
    k = n / 2;

    while (cnt--)
    {
        if ((y >= min_y) && (y < max_y + 1))
        {
            if (x < ContourX[y][0])
                ContourX[y][0] = x;
            if (x > ContourX[y][1])
                ContourX[y][1] = x;
        }

        k += n;
        if (k < m)
        {
            x += dx2;
            y += dy2;
        }
        else
        {
            k -= m;
            x += dx1;
            y += dy1;
        }
    }
}

void generateRoadZValue(VehicleData target_vehicle, std::vector<adcm::map_2dListVector> &map_2d_test)
{
// 현재 차량의 position_x position_y 중심으로 좌우전방 5m 를 스캔해서 road_z 값을 1로 지정
#define SCANNING_RANGE 30
    adcm::Log::Info() << "vehicle class " << target_vehicle.vehicle_class << " generateRoadZValue";

    int scanned_range_LL_x = floor(target_vehicle.position_x - SUB_VEHICLE_SIZE_X / 2) - SCANNING_RANGE;
    int scanned_range_LL_y = floor(target_vehicle.position_y - SUB_VEHICLE_SIZE_Y / 2) - SCANNING_RANGE;

    int scanned_range_RU_x = floor(target_vehicle.position_x + SUB_VEHICLE_SIZE_X / 2) + SCANNING_RANGE;
    int scanned_range_RU_y = floor(target_vehicle.position_y + SUB_VEHICLE_SIZE_Y / 2) + SCANNING_RANGE;
    adcm::Log::Info() << "x는 " << scanned_range_LL_x << " ~ " << scanned_range_RU_x + 1 << "까지";
    adcm::Log::Info() << "y는 " << scanned_range_LL_y << " ~ " << scanned_range_RU_y + 1 << "까지";
    for (int i = scanned_range_LL_x; i < (scanned_range_RU_x + 1); i++)
    {
        for (int j = scanned_range_LL_y; j < (scanned_range_RU_y + 1); j++)
        {
            if (i >= 0 && j >= 0 && i < map_n && j < map_m)
            {
                map_2d_test[i][j].road_z = 1;
                // adcm::Log::Info() << "main vehicle generateRoadZValue[" << i << "]["<< j << "]:" << map_2d_test[i][j].road_z;
            }
        }
    }
    adcm::Log::Info() << "generateRoadZValue finish";
}

void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, VehicleData &vehicle, std::vector<adcm::map_2dListVector> &map_2d_test)
{
    long arr_x[] = {p0.x, p1.x, p2.x, p3.x};
    long arr_y[] = {p0.y, p1.y, p2.y, p3.y};
    // find max x&y and min x&y of the rectangle
    int n = sizeof(arr_y) / sizeof(arr_y[0]);
    // Implemented inbuilt function to sort array
    std::sort(arr_x, arr_x + n);
    std::sort(arr_y, arr_y + n);
    long min_x = arr_x[0];
    long max_x = arr_x[n - 1];
    long min_y = arr_y[0];
    long max_y = arr_y[n - 1];

    Point2D index;
    for (index.y = min_y; index.y < max_y + 1; index.y++)
    {
        ContourX[index.y][0] = LONG_MAX; // min X
        ContourX[index.y][1] = LONG_MIN; // max X
    }

    ScanLine(p0.x, p0.y, p1.x, p1.y, min_y, max_y);
    ScanLine(p1.x, p1.y, p2.x, p2.y, min_y, max_y);
    ScanLine(p2.x, p2.y, p3.x, p3.y, min_y, max_y);
    ScanLine(p3.x, p3.y, p0.x, p0.y, min_y, max_y);

    for (index.y = min_y; index.y < max_y + 1; index.y++)
    {
        if (ContourX[index.y][1] >= ContourX[index.y][0])
        {
            index.x = ContourX[index.y][0];
            long len = 1 + ContourX[index.y][1] - ContourX[index.y][0];

            // Can draw a horizontal line instead of individual pixels here
            while (len--)
            {
                // occupied
                vehicle.map_2d_location.push_back(index);
                index.x++;
                /*int x = index.x;
                int y = index.y;
                map_2d_test[x][y].road_z= 1;*/
                // 코드 실행 안됨...우선은 1 을 넣음으로써 scanned 완료된 map
                // adcm::Log::Info() << "x value is " << index.x << " y value is " << index.y;
            }
        }
    }
    adcm::Log::Info() << "Vehicle class " << vehicle.vehicle_class << " generateOccupancyIndex";
}
void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, std::vector<ObstacleData>::iterator iter)
{
    long arr_x[] = {p0.x, p1.x, p2.x, p3.x};
    long arr_y[] = {p0.y, p1.y, p2.y, p3.y};
    // find max x&y and min x&y of the rectangle
    int n = sizeof(arr_y) / sizeof(arr_y[0]);
    // Implemented inbuilt function to sort array
    std::sort(arr_x, arr_x + n);
    std::sort(arr_y, arr_y + n);
    long min_x = arr_x[0];
    long max_x = arr_x[n - 1];
    long min_y = arr_y[0];
    long max_y = arr_y[n - 1];

    Point2D index;
    for (index.y = min_y; index.y < max_y + 1; index.y++)
    {
        // adcm::Log::Info() << "min:" << ContourX[index.y][0];
        // adcm::Log::Info() << "max:" << ContourX[index.y][1];
        ContourX[index.y][0] = LONG_MAX; // min X
        ContourX[index.y][1] = LONG_MIN; // max X
    }

    ScanLine(p0.x, p0.y, p1.x, p1.y, min_y, max_y);
    ScanLine(p1.x, p1.y, p2.x, p2.y, min_y, max_y);
    ScanLine(p2.x, p2.y, p3.x, p3.y, min_y, max_y);
    ScanLine(p3.x, p3.y, p0.x, p0.y, min_y, max_y);

    for (index.y = min_y; index.y < max_y + 1; index.y++)
    {
        if (ContourX[index.y][1] >= ContourX[index.y][0])
        {
            index.x = ContourX[index.y][0];
            long len = 1 + ContourX[index.y][1] - ContourX[index.y][0];

            // Can draw a horizontal line instead of individual pixels here
            while (len--)
            {
                // occupied
                iter->map_2d_location.push_back(index);
                index.x++;
                // adcm::Log::Info() << "x value is " << index.x << " y value is " << index.y;
            }
        }
    }
}
void find4VerticesVehicle(VehicleData &target_vehicle, std::vector<adcm::map_2dListVector> &map_2d_test)
{
    Point2D LU, RU, RL, LL;
    double theta = target_vehicle.yaw * M_PI / 180;

    if (target_vehicle.vehicle_class == EGO_VEHICLE)
    {
// 특장차일 경우
#define VEHICLE_SIZE_X MAIN_VEHICLE_SIZE_X
#define VEHICLE_SIZE_Y MAIN_VEHICLE_SIZE_Y
        // #define VEHICLE_SIZE_X
        // #define VEHICLE_SIZE_Y
    }
    else
    {
#define VEHICLE_SIZE_X SUB_VEHICLE_SIZE_X
#define VEHICLE_SIZE_Y SUB_VEHICLE_SIZE_Y
        // #define VEHICLE_SIZE_X
        // #define VEHICLE_SIZE_Y
    }
    // step1. 4 꼭지점을 각각 찾는다
    LU.x = target_vehicle.position_x + (cos(theta) * VEHICLE_SIZE_X / 2 - sin(theta) * VEHICLE_SIZE_Y / 2);
    LU.y = target_vehicle.position_y + (sin(theta) * VEHICLE_SIZE_X / 2 + cos(theta) * VEHICLE_SIZE_Y / 2);

    RU.x = target_vehicle.position_x + (cos(theta) * VEHICLE_SIZE_X / 2 - sin(theta) * VEHICLE_SIZE_Y / 2);
    RU.y = target_vehicle.position_y - (sin(theta) * VEHICLE_SIZE_X / 2 + cos(theta) * VEHICLE_SIZE_Y / 2);

    RL.x = target_vehicle.position_x - (cos(theta) * VEHICLE_SIZE_X / 2 - sin(theta) * VEHICLE_SIZE_Y / 2);
    RL.y = target_vehicle.position_y - (sin(theta) * VEHICLE_SIZE_X / 2 + cos(theta) * VEHICLE_SIZE_Y / 2);

    LL.x = target_vehicle.position_x - (cos(theta) * VEHICLE_SIZE_X / 2 - sin(theta) * VEHICLE_SIZE_Y / 2);
    LL.y = target_vehicle.position_y + (sin(theta) * VEHICLE_SIZE_X / 2 + cos(theta) * VEHICLE_SIZE_Y / 2);

    adcm::Log::Info() << "find4VerticesVehicle: LU.x is" << LU.x;
    adcm::Log::Info() << "find4VerticesVehicle: LU.y is" << LU.y;

    adcm::Log::Info() << "find4VerticesVehicle: RU.x is" << RU.x;
    adcm::Log::Info() << "find4VerticesVehicle: RU.y is" << RU.y;

    adcm::Log::Info() << "find4VerticesVehicle: RL.x is" << RL.x;
    adcm::Log::Info() << "find4VerticesVehicle: RL.y is" << RL.y;

    adcm::Log::Info() << "find4VerticesVehicle: LL.x is" << LL.x;
    adcm::Log::Info() << "find4VerticesVehicle: LL.y is" << LL.y;

    checkRange(LU);
    checkRange(RU);
    checkRange(RL);
    checkRange(LL);

    generateRoadZValue(target_vehicle, map_2d_test);
    generateOccupancyIndex(LU, RU, RL, LL, target_vehicle, map_2d_test);
}
void find4VerticesObstacle(std::vector<ObstacleData> &obstacle_list_filtered)
{
    // 4 vertices 를 찾고싶은 해당 obstacle
    for (auto iter = obstacle_list_filtered.begin(); iter < obstacle_list_filtered.end(); iter++)
    {
        Point2D LU, RU, RL, LL;
        double obstacle_size_x = iter->fused_cuboid_x;
        double obstacle_size_y = iter->fused_cuboid_y;
        double obstacle_position_x = iter->fused_position_x;
        double obstacle_position_y = iter->fused_position_y;
        double theta = iter->fused_heading_angle * M_PI / 180;

        LU.x = obstacle_position_x + (cos(theta) * (obstacle_size_x / 2) - sin(theta) * (obstacle_size_y / 2));
        LU.y = obstacle_position_y + (sin(theta) * (obstacle_size_x / 2) + cos(theta) * (obstacle_size_y / 2));

        RU.x = obstacle_position_x + (cos(theta) * (obstacle_size_x / 2) - sin(theta) * (obstacle_size_y / 2));
        RU.y = obstacle_position_y - (sin(theta) * (obstacle_size_x / 2) + cos(theta) * (obstacle_size_y / 2));

        RL.x = obstacle_position_x - (cos(theta) * (obstacle_size_x / 2) - sin(theta) * (obstacle_size_y / 2));
        RL.y = obstacle_position_y - (sin(theta) * (obstacle_size_x / 2) + cos(theta) * (obstacle_size_y / 2));

        LL.x = obstacle_position_x - (cos(theta) * (obstacle_size_x / 2) - sin(theta) * (obstacle_size_y / 2));
        LL.y = obstacle_position_y + (sin(theta) * (obstacle_size_x / 2) + cos(theta) * (obstacle_size_y / 2));

        adcm::Log::Info() << "find4VerticesObstacle: LU.x is" << LU.x;
        adcm::Log::Info() << "find4VerticesObstacle: LU.y is" << LU.y;

        adcm::Log::Info() << "find4VerticesObstacle: RU.x is" << RU.x;
        adcm::Log::Info() << "find4VerticesObstacle: RU.y is" << RU.y;

        adcm::Log::Info() << "find4VerticesObstacle: RL.x is" << RL.x;
        adcm::Log::Info() << "find4VerticesObstacle: RL.y is" << RL.y;

        adcm::Log::Info() << "find4VerticesObstacle: LL.x is" << LL.x;
        adcm::Log::Info() << "find4VerticesObstacle: LL.y is" << LL.y;

        checkRange(LU);
        checkRange(RU);
        checkRange(RL);
        checkRange(LL);

        generateOccupancyIndex(LU, RU, RL, LL, *(&iter));
    }
}

namespace
{

    // Atomic flag for exit after SIGTERM caught
    std::atomic_bool continueExecution{true};
    std::atomic_uint gReceivedEvent_count_hub_data{0};
    std::atomic_uint gMainthread_Loopcount{0};

    void SigTermHandler(int signal)
    {
        if (signal == SIGTERM)
        {
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
        if (sigaction(SIGTERM, &sa, NULL) == -1)
        {
            // Could not register a SIGTERM signal handler
            return false;
        }

        return true;
    }

} // namespace

// hubData 수신
void ThreadReceiveHudData()
{
    adcm::Log::Info() << "SDK release_240524_interface v1.7";
    // adcm::MapData_Provider mapData_provider;
    adcm::HubData_Subscriber hubData_subscriber;
    INFO("DataFusion .init()");
    // mapData_provider.init("DataFusion/DataFusion/PPort_map_data");
    hubData_subscriber.init("DataFusion/DataFusion/RPort_hub_data");
    INFO("ThreadReceiveHudData start...");

    while (continueExecution)
    {
        gMainthread_Loopcount++;
        VERBOSE("[DataFusion] Application loop");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool hubData_rxEvent = hubData_subscriber.waitEvent(0); // wait event

        if (hubData_rxEvent)
        {
            adcm::Log::Verbose() << "[EVENT] DataFusion Hub Data received";

            while (!hubData_subscriber.isEventQueueEmpty())
            {
                auto data = hubData_subscriber.getEvent();
                gReceivedEvent_count_hub_data++;

                // 수신된 데이터 handling 위한 추가 코드
                switch (data->vehicle_class)
                {
                case EGO_VEHICLE: // 특장차가 보낸 인지데이터

                    main_vehicle_temp.vehicle_class = EGO_VEHICLE;
                    main_vehicle_temp.timestamp = data->timestamp;
                    // main_vehicle_temp.road_z = data->road_z; //vector assignment to fix?
                    main_vehicle_temp.position_lat = data->position_lat;
                    main_vehicle_temp.position_long = data->position_long;
                    main_vehicle_temp.position_height = data->position_height;
                    main_vehicle_temp.yaw = data->yaw;
                    main_vehicle_temp.roll = data->roll;
                    main_vehicle_temp.pitch = data->pitch;
                    main_vehicle_temp.velocity_long = data->velocity_long;
                    main_vehicle_temp.velocity_lat = data->velocity_lat;
                    main_vehicle_temp.velocity_ang = data->velocity_ang;

                    obstacle_list_temp.clear();
                    for (int i = 0; i < data->obstacle.size(); i++)
                    {
                        ObstacleData obstacle_to_push;
                        obstacle_to_push.obstacle_class = data->obstacle[i].obstacle_class;
                        obstacle_to_push.timestamp = data->timestamp;
                        obstacle_to_push.fused_cuboid_x = data->obstacle[i].cuboid_x;
                        obstacle_to_push.fused_cuboid_y = data->obstacle[i].cuboid_y;
                        obstacle_to_push.fused_cuboid_z = data->obstacle[i].cuboid_z;
                        obstacle_to_push.fused_heading_angle = data->obstacle[i].heading_angle;
                        obstacle_to_push.fused_position_x = data->obstacle[i].position_x;
                        obstacle_to_push.fused_position_y = data->obstacle[i].position_y;
                        obstacle_to_push.fused_position_z = data->obstacle[i].position_z;
                        obstacle_to_push.fused_velocity_x = data->obstacle[i].velocity_x;
                        obstacle_to_push.fused_velocity_y = data->obstacle[i].velocity_y;
                        obstacle_to_push.fused_velocity_z = data->obstacle[i].velocity_z;
                        obstacle_list_temp.push_back(obstacle_to_push);
                    }

                    adcm::Log::Info() << "main vehicle data received";
                    break;

                case SUB_VEHICLE_1: // 보조차1이 보낸 인지데이터
                    sub1_vehicle_temp.vehicle_class = SUB_VEHICLE_1;
                    sub1_vehicle_temp.timestamp = data->timestamp;
                    // sub1_vehicle_temp.road_z = data->road_z; //vector assignment to fix?
                    sub1_vehicle_temp.position_lat = data->position_lat;
                    sub1_vehicle_temp.position_long = data->position_long;
                    sub1_vehicle_temp.position_height = data->position_height;
                    sub1_vehicle_temp.yaw = data->yaw;
                    sub1_vehicle_temp.roll = data->roll;
                    sub1_vehicle_temp.pitch = data->pitch;
                    sub1_vehicle_temp.velocity_long = data->velocity_long;
                    sub1_vehicle_temp.velocity_lat = data->velocity_lat;
                    sub1_vehicle_temp.velocity_ang = data->velocity_ang;
                    adcm::Log::Info() << "sub vehicle1 data received";
                    break;

                case SUB_VEHICLE_2: // 보조차2가 보낸 인지데이터
                    sub2_vehicle_temp.vehicle_class = SUB_VEHICLE_2;
                    sub2_vehicle_temp.timestamp = data->timestamp;
                    // sub2_vehicle_temp.road_z = data->road_z; //vector assignment to fix?
                    sub2_vehicle_temp.position_lat = data->position_lat;
                    sub2_vehicle_temp.position_long = data->position_long;
                    sub2_vehicle_temp.position_height = data->position_height;
                    sub2_vehicle_temp.yaw = data->yaw;
                    sub2_vehicle_temp.roll = data->roll;
                    sub2_vehicle_temp.pitch = data->pitch;
                    sub2_vehicle_temp.velocity_long = data->velocity_long;
                    sub2_vehicle_temp.velocity_lat = data->velocity_lat;
                    sub2_vehicle_temp.velocity_ang = data->velocity_ang;
                    adcm::Log::Info() << "sub vehicle2 data received";
                    break;

                default:
                    adcm::Log::Info() << "data received but belongs to no vehicle hence discarded";
                    break;
                }
                adcm::Log::Info() << "=============KATECH: handling of received data DONE==============";
            }
        }
        if (continueExecution != true)
        {
            adcm::Log::Info() << "continueExection : " << continueExecution << " > 종료";
        }
    }
}

/* work_information 수신
void ThreadReceiveWorkInfo()
{
    adcm::Log::Info() << "DataFusion ThreadAct2";
    adcm::WorkInformation_Subscriber workInformation_subscriber;
    workInformation_subscriber.init("DataFusion/DataFusion/RPort_work_information");
    INFO("ThreadReceiveWorkInfo start...");

    while (continueExecution)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool workInfo_rxEvent = workInformation_subscriber.waitEvent(0);

        if (workInfo_rxEvent)
        {
            adcm::Log::Verbose() << "[EVENT] DataFusion Work Information received";

            while (!workInformation_subscriber.isEventQueueEmpty())
            {
                auto data = workInformation_subscriber.getEvent();

                main_vehicle_size_length = data->main_vehicle.length;
                main_vehicle_size_width = data->main_vehicle.width;
                for (int i = 0; i < data->sub_vehicle.size(); i++)
                {
                    VehicleSizeData vehicle_to_push;
                    vehicle_to_push.length = data->sub_vehicle[i].length;
                    vehicle_to_push.width = data->sub_vehicle[i].width;
                    sub_vehicle_size.push_back(vehicle_to_push);
                }
                for (int i = 0; i < data->working_area_boundary.size(); i++)
                {
                    BoundaryData boundary_to_push;
                    boundary_to_push.x = data->working_area_boundary.x;
                    boundary_to_push.y = data->working_area_boundary.y;
                    work_boundary.push_back(boundary_to_push);
                }
            }
        }

        // 좌표계 변환 전 map 범위 둘러싸는 사각형 좌표
        min_x = work_boundary[0].x;
        min_y = work_boundary[0].y;
        max_x = work_boundary[0].x;
        max_y = work_boundary[0].y;

        for (int i = 1; i < work_boundary.size(); i++)
        {
            min_x = work_boundary[i].x < min_x ? work_boundary[i].x : min_x;
            min_y = work_boundary[i].y < min_y ? work_boundary[i].y : min_y;
            max_x = work_boundary[i].x > max_x ? work_boundary[i].x : max_x;
            max_y = work_boundary[i].y > max_y ? work_boundary[i].y : max_y;
        }

        for (int i = 0; i < work_boundary.size(); i++)
        {
            work_boundary[i].x -= min_x;
            work_boundary[j].y -= min_y;
        }

    }
}*/

void ThreadKatech()
{
    //==============1.전역변수인 MapData 생성 =================
    adcm::map_data_Objects mapData;
    // 한번 생성후 관제에서 인지데이터를 받을때마다 (100ms) 마다 업데이트
    adcm::Log::Info() << "mapData created for the first time";

    ::adcm::map_2dListStruct map_2dStruct_init;
    map_2dStruct_init.obstacle_id = NO_OBSTACLE;
    map_2dStruct_init.road_z = 0;
    map_2dStruct_init.vehicle_class = NO_VEHICLE; // 시뮬레이션 데이터 설정때문에 부득이 NO_VEHICLE =5 로 바꿈

    std::vector<adcm::map_2dListVector> map_2d_test(map_n, adcm::map_2dListVector(map_m, map_2dStruct_init));

    /* 좌표계 변환 전 map
    std::vector<adcm::map_2dListVector> map_2d_test(max_x-min_x, adcm::map_2dListVector(max_y-min_y, map_2dStruct_init));
    for (int i = 0; i < max_x - min_x; i++)
    {
        for (int j = 0; j < max_y - min_y; j++)
        {
            map_2d_test[i][j].isvalid=isValid(i,j);
        }
    }
    */
    adcm::Log::Info() << "mapData 2d info initialized";

    //=============mapData provider 여기다 해보기==================
    adcm::MapData_Provider mapData_provider;
    mapData_provider.init("DataFusion/DataFusion/PPort_map_data");

    VehicleData main_vehicle;
    VehicleData sub1_vehicle;
    VehicleData sub2_vehicle;
    std::vector<ObstacleData> obstacle_list;

    while (continueExecution)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        adcm::Log::Info() << "==============KATECH modified code start==========";
        main_vehicle = main_vehicle_temp;
        sub1_vehicle = sub1_vehicle_temp;
        sub2_vehicle = sub2_vehicle_temp;
        obstacle_list = obstacle_list_temp;

        adcm::map_2dListVector map_2dListVector;
        adcm::map_2dListStruct map_2dStruct;

        adcm::Log::Info() << "mapData obstacle list size is at start is" << mapData.obstacle_list.size();

        //==============1. obstacle 로 인지된 sub 차량 제거 (OK)=================
        for (auto iter = obstacle_list.begin(); iter != obstacle_list.end();)
        {
            if ((abs(iter->fused_cuboid_x - SUB_VEHICLE_SIZE_X / 10)) < 0.1 && (abs(iter->fused_cuboid_y - SUB_VEHICLE_SIZE_Y / 10)) < 0.1)
            {
                iter = obstacle_list.erase(iter);
            }
            else
                iter++;
        }
        adcm::Log::Info() << "obstacle without sub-vehicle size is " << obstacle_list.size();

        //==============2. obstacle ID assignment =================
        for (auto iter1 = mapData.obstacle_list.begin(); iter1 != mapData.obstacle_list.end(); iter1++)
        {
            adcm::Log::Info() << "previous obstacle saved in the mapData!" << iter1->obstacle_id;
        }
        std::vector<ObstacleData> obstacle_list_filtered;
        obstacle_list_filtered.clear();

        if (mapData.obstacle_list.empty())
        {
            // 최초 obstacle ID assignment 진행. random하게 assign 한다
            // 트래킹하는 알고리즘 필요 *추후 보완
            int i = 1;
            for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
            {
                iter->obstacle_id = iter->timestamp + i;
                i++;
                // 타임스탬프 값으로 장애물 id assign
                adcm::Log::Info() << "obstacle assigned is " << iter->obstacle_id;
                obstacle_list_filtered.push_back(*iter);
            }
        }
        else // 이미 obstacle list 존재하는 경우는 obstacle id 비교가 필요하다
        {
            for (auto iter = mapData.obstacle_list.begin(); iter != mapData.obstacle_list.end(); iter++)
            {
                for (auto iter1 = obstacle_list.begin(); iter1 != obstacle_list.end();)
                {
                    // 사이즈 오차 10cm 이내면 동일 장애물이라 본다
                    // if ((ABS(iter->fused_cuboid_x - iter1->fused_cuboid_x)< 0.001) && (ABS(iter->fused_cuboid_y - iter1->fused_cuboid_y)< 0.001) && (ABS(iter->fused_cuboid_z - iter1->fused_cuboid_z)<0.001))
                    if ((iter->fused_cuboid_x == iter1->fused_cuboid_x) && (iter->fused_cuboid_y == iter1->fused_cuboid_y) && (iter->fused_cuboid_z == iter1->fused_cuboid_z))
                    {
                        adcm::Log::Info() << "cuboid x: " << iter->fused_cuboid_x << "cuboid y:" << iter->fused_cuboid_y << "obstacle id" << iter->obstacle_id;
                        adcm::Log::Info() << "cuboid x: " << iter1->fused_cuboid_x << "cuboid y:" << iter1->fused_cuboid_y;
                        adcm::Log::Info() << "Identical obstacle detected : " << iter->obstacle_id;
                        iter1->obstacle_id = iter->obstacle_id;
                        obstacle_list_filtered.push_back(*iter1);
                        iter1 = obstacle_list.erase(iter1);
                    }
                    else // 동일하지 않은 장애물에는 temp value 인 99 으로 id를 우선 지정
                    {
                        iter1->obstacle_id = INVALID_VALUE;
                        adcm::Log::Info() << " New obstacle appeared OR tracking failed";
                        ++iter1;
                    }
                }
            }

            // 새로운 객체들에 대한 id 지정 (INVALID_VALUE 를 실제 아이디로 지정)
            srand(time(NULL));
            unsigned short rand_id = (unsigned short)rand();
            for (auto iter1 = obstacle_list.begin(); iter1 != obstacle_list.end(); iter1++)
            {
                bool idcheck = false;
                // TO DO:id 중복여부 duplicate 체크 해야함 *추후 수정
                // 추가: id 중복 없을 시 idcheck->true, 중복체크 완료 후 rand_id 값으로 id 지정
                while (!idcheck)
                {
                    idcheck = true;
                    for (auto iter2 = obstacle_list_filtered.begin(); iter2 != obstacle_list_filtered.end(); iter2++)
                    {
                        if (iter2->obstacle_id == rand_id)
                        {
                            idcheck = false;
                            rand_id++;
                            break;
                        }
                    }
                }
                // 타임스탬프 값으로 장애물 id assign
                iter1->obstacle_id = rand_id;
                adcm::Log::Info() << "obstacle newly assigned is " << iter1->obstacle_id;
                obstacle_list_filtered.push_back(*iter1);
                rand_id++;
            }
        }
        obstacle_list.clear();

        for (auto iter1 = obstacle_list_filtered.begin(); iter1 != obstacle_list_filtered.end(); iter1++)
        {
            adcm::Log::Info() << "obstacle filtered are: " << iter1->obstacle_id;
        }

        //==============3. 장애물 좌표계 변환=================
        relativeToGlobalcoordinate(obstacle_list_filtered, main_vehicle);
        globalToLocalcoordinate(obstacle_list_filtered, main_vehicle);

        //==============4. 차량 좌표계 변환==================================
        // 차량마다 받은 글로벌 좌표를 작업공간 local 좌표계로 변환
        globalToLocalcoordinate(main_vehicle);
        globalToLocalcoordinate(sub1_vehicle);
        globalToLocalcoordinate(sub2_vehicle);

        bool a = checkRange(main_vehicle);
        bool b = checkRange(sub1_vehicle);
        bool c = checkRange(sub2_vehicle);

        if (a && b && c)
        { // execute only if all true!
            //==============5. 0.1 m/s 미만인 경우 장애물 정지 상태 판정 및 stop_count 값 assign =================
            for (auto iter = obstacle_list_filtered.begin(); iter != obstacle_list_filtered.end(); iter++)
            {
                if ((abs(iter->fused_velocity_x)) < 0.1 && (abs(iter->fused_velocity_y)) < 0.1)
                {
                    iter->stop_count = 1; // 해당 시각 물체 정지상태
                    adcm::Log::Info() << "obstacle stopped " << iter->stop_count;
                }
                else
                {
                    iter->stop_count = 0;
                    adcm::Log::Info() << "obstacle not stopped " << iter->stop_count;
                }
            }

            // 이 전에 장애물의 stop status 를 카운트 하는 카운터 값 변동
            if (!mapData.obstacle_list.empty())
            {
                for (auto iter = mapData.obstacle_list.begin(); iter != mapData.obstacle_list.end(); iter++)
                { // 최초 mapData 아무것도 없으면 for loop 들어가지 않음

                    for (auto iter1 = obstacle_list_filtered.begin(); iter1 != obstacle_list_filtered.end(); iter1++)
                    {
                        if (iter->obstacle_id == iter1->obstacle_id)
                        { // 동일 장애물
                            if (iter1->stop_count == 1)
                            {
                                iter1->stop_count = iter->stop_count + iter1->stop_count;
                                adcm::Log::Info() << "stop count updated to " << iter1->stop_count << "for obstacle " << iter1->obstacle_id;

                                // stop_count 업데이트
                            }
                            // 한번이라도 stop_count가 0 이 라면 카운트 리셋
                        }
                    }
                }
            }
            mapData.obstacle_list.clear();

            //==============7. 장애물과 차량의 occupancy 계산해 map_2d_location 값 업데이트 ========

            if (!obstacle_list_filtered.empty())
            {
                find4VerticesObstacle(obstacle_list_filtered);
            }

            if (main_vehicle.vehicle_class == EGO_VEHICLE && main_vehicle.timestamp != 0)
            {
                // main vehicle 존재하므로 해당 function execution
                find4VerticesVehicle(main_vehicle, map_2d_test);
            }

            if (sub1_vehicle.vehicle_class == SUB_VEHICLE_1)
            {
                // sub1_vehicle 존재하므로 해당 function execution
                find4VerticesVehicle(sub1_vehicle, map_2d_test);
            }

            if (sub2_vehicle.vehicle_class == SUB_VEHICLE_2)
            {
                // sub2_vehicle 존재하므로 해당 function execution
                find4VerticesVehicle(sub2_vehicle, map_2d_test);
            }

            //==============8. 현재까지의 데이터를 adcm mapData 형식으로 재구성해서 업데이트 ================
            //================ adcm mapData 내 obstacle list 업데이트 ===============================

            adcm::obstacleListStruct obstacle1;
            adcm::obstacleListStruct obstacle2;
            adcm::obstacleListStruct obstacle3;
            adcm::obstacleListStruct obstacle4; // 우선 장애물 최대 4개만 있다고 가정하고 진행

            adcm::vehicleListStruct main_vehicle_final;
            adcm::vehicleListStruct sub1_vehicle_final;
            adcm::vehicleListStruct sub2_vehicle_final;

            int count = 1;
            int max_count = obstacle_list_filtered.size() + 1;
            adcm::Log::Info() << "장애물 mapdata 반영 예정 개수 : " << max_count - 1;
            for (auto iter = obstacle_list_filtered.begin(); iter != obstacle_list_filtered.end(); iter++)
            {
                switch (count)
                {
                case 1:
                    adcm::Log::Info() << "obstacle 1 start pushing";
                    obstacle1.obstacle_id = iter->obstacle_id;
                    obstacle1.obstacle_class = iter->obstacle_class;
                    obstacle1.timestamp = iter->timestamp;
                    obstacle1.map_2d_location.clear();
                    for (auto iter1 = iter->map_2d_location.begin(); iter1 < iter->map_2d_location.end(); iter1++)
                    {
                        adcm::map2dIndex index_to_push;
                        index_to_push.x = iter1->x;
                        index_to_push.y = iter1->y;
                        map_2d_test[index_to_push.x][index_to_push.y].obstacle_id = iter->obstacle_id;
                        // adcm::Log::Info() << "occupancy index pair of obstacle 1 is " << index_to_push.x << " , " << index_to_push.y;
                        // adcm::Log::Info() << "occupancy index of obstacle id " << iter->obstacle_id;
                        // adcm::Log::Info() << "map_2d_test[" << index_to_push.x << "][" <<index_to_push.y << "] = " <<  map_2d_test[index_to_push.x][index_to_push.y].obstacle_id;
                        obstacle1.map_2d_location.push_back(index_to_push);
                    }
                    obstacle1.stop_count = iter->stop_count;
                    obstacle1.fused_cuboid_x = iter->fused_cuboid_x;
                    obstacle1.fused_cuboid_y = iter->fused_cuboid_y;
                    obstacle1.fused_cuboid_z = iter->fused_cuboid_z;
                    obstacle1.fused_heading_angle = iter->fused_heading_angle;
                    obstacle1.fused_position_x = iter->fused_position_x;
                    obstacle1.fused_position_y = iter->fused_position_y;
                    obstacle1.fused_position_z = iter->fused_position_z;
                    obstacle1.fused_velocity_x = iter->fused_velocity_x;
                    obstacle1.fused_velocity_y = iter->fused_velocity_y;
                    obstacle1.fused_velocity_z = iter->fused_velocity_z;

                    mapData.obstacle_list.push_back(obstacle1);
                    adcm::Log::Info() << "obstacle 1 is pushed to the mapData";
                    break;

                case 2:
                    adcm::Log::Info() << "obstacle 2 start pushing";
                    obstacle2.obstacle_id = iter->obstacle_id;
                    obstacle2.obstacle_class = iter->obstacle_class;
                    obstacle2.timestamp = iter->timestamp;
                    obstacle2.map_2d_location.clear();
                    adcm::Log::Info() << "check1";
                    for (auto iter1 = iter->map_2d_location.begin(); iter1 < iter->map_2d_location.end(); iter1++)
                    {
                        adcm::map2dIndex index_to_push;
                        index_to_push.x = iter1->x;
                        index_to_push.y = iter1->y;
                        map_2d_test[index_to_push.x][index_to_push.y].obstacle_id = iter->obstacle_id;
                        adcm::Log::Info() << "obstacle id : " << iter->obstacle_id;
                        adcm::Log::Info() << "obstacle2 map_2d_location : (" << index_to_push.x << ", " << index_to_push.y << ")";
                        // adcm::Log::Info() << "occupancy index pair of obstacle 2 is " << index_to_push.x << " , " << index_to_push.y;
                        // adcm::Log::Info() << "occupancy index of obstacle id " << iter->obstacle_id;
                        obstacle2.map_2d_location.push_back(index_to_push);
                    }
                    adcm::Log::Info() << "check2";
                    obstacle2.stop_count = iter->stop_count;
                    obstacle2.fused_cuboid_x = iter->fused_cuboid_x;
                    obstacle2.fused_cuboid_y = iter->fused_cuboid_y;
                    obstacle2.fused_cuboid_z = iter->fused_cuboid_z;
                    obstacle2.fused_heading_angle = iter->fused_heading_angle;
                    obstacle2.fused_position_x = iter->fused_position_x;
                    obstacle2.fused_position_y = iter->fused_position_y;
                    obstacle2.fused_position_z = iter->fused_position_z;
                    obstacle2.fused_velocity_x = iter->fused_velocity_x;
                    obstacle2.fused_velocity_y = iter->fused_velocity_y;
                    obstacle2.fused_velocity_z = iter->fused_velocity_z;
                    mapData.obstacle_list.push_back(obstacle2);
                    adcm::Log::Info() << "obstacle 2 is pushed to the mapData";
                    break;

                case 3:
                    adcm::Log::Info() << "obstacle 3 start pushing";
                    obstacle3.obstacle_id = iter->obstacle_id;
                    obstacle3.obstacle_class = iter->obstacle_class;
                    obstacle3.timestamp = iter->timestamp;
                    obstacle3.map_2d_location.clear();
                    for (auto iter1 = iter->map_2d_location.begin(); iter1 < iter->map_2d_location.end(); iter1++)
                    {
                        adcm::map2dIndex index_to_push;
                        index_to_push.x = iter1->x;
                        index_to_push.y = iter1->y;
                        map_2d_test[index_to_push.x][index_to_push.y].obstacle_id = iter->obstacle_id;
                        obstacle3.map_2d_location.push_back(index_to_push);
                    }
                    obstacle3.stop_count = iter->stop_count;
                    obstacle3.fused_cuboid_x = iter->fused_cuboid_x;
                    obstacle3.fused_cuboid_y = iter->fused_cuboid_y;
                    obstacle3.fused_cuboid_z = iter->fused_cuboid_z;
                    obstacle3.fused_heading_angle = iter->fused_heading_angle;
                    obstacle3.fused_position_x = iter->fused_position_x;
                    obstacle3.fused_position_y = iter->fused_position_y;
                    obstacle3.fused_position_z = iter->fused_position_z;
                    obstacle3.fused_velocity_x = iter->fused_velocity_x;
                    obstacle3.fused_velocity_y = iter->fused_velocity_y;
                    obstacle3.fused_velocity_z = iter->fused_velocity_z;
                    mapData.obstacle_list.push_back(obstacle3);
                    adcm::Log::Info() << "obstacle 3 is pushed to the mapData";
                    break;

                case 4:
                    adcm::Log::Info() << "obstacle 4 start pushing";
                    obstacle4.obstacle_id = iter->obstacle_id;
                    obstacle4.obstacle_class = iter->obstacle_class;
                    obstacle4.timestamp = iter->timestamp;
                    obstacle4.map_2d_location.clear();
                    for (auto iter1 = iter->map_2d_location.begin(); iter1 < iter->map_2d_location.end(); iter1++)
                    {
                        adcm::map2dIndex index_to_push;
                        index_to_push.x = iter1->x;
                        index_to_push.y = iter1->y;
                        map_2d_test[index_to_push.x][index_to_push.y].obstacle_id = iter->obstacle_id;
                        mapData.map_2d[iter1->x][iter1->y].obstacle_id = iter->obstacle_id;
                        obstacle4.map_2d_location.push_back(index_to_push);
                    }
                    obstacle4.stop_count = iter->stop_count;
                    obstacle4.fused_cuboid_x = iter->fused_cuboid_x;
                    obstacle4.fused_cuboid_y = iter->fused_cuboid_y;
                    obstacle4.fused_cuboid_z = iter->fused_cuboid_z;
                    obstacle4.fused_heading_angle = iter->fused_heading_angle;
                    obstacle4.fused_position_x = iter->fused_position_x;
                    obstacle4.fused_position_y = iter->fused_position_y;
                    obstacle4.fused_position_z = iter->fused_position_z;
                    obstacle4.fused_velocity_x = iter->fused_velocity_x;
                    obstacle4.fused_velocity_y = iter->fused_velocity_y;
                    obstacle4.fused_velocity_z = iter->fused_velocity_z;
                    adcm::Log::Info() << "obstacle 4 is pushed to the mapData";

                default:
                    adcm::Log::Info() << "drop obstacle";
                    break;
                }
                if (count < max_count)
                {
                    count++;
                }
                else
                    break;
            }
            adcm::Log::Info() << "mapData obstacle list size is " << mapData.obstacle_list.size();

            //================ adcm mapData 내 vehicle list 업데이트 ===============================

            mapData.vehicle_list.clear();

            if (main_vehicle.vehicle_class == EGO_VEHICLE && main_vehicle.timestamp != 0)
            {
                main_vehicle_final.vehicle_class = main_vehicle.vehicle_class;
                main_vehicle_final.timestamp = main_vehicle.timestamp;
                main_vehicle_final.map_2d_location.clear();
                adcm::Log::Info() << "main_vehicle push to mapData (x:" << main_vehicle.map_2d_location.begin()->x << " ~ " << main_vehicle.map_2d_location.end()->x << " )";
                adcm::Log::Info() << "main_vehicle push to mapData (y:" << main_vehicle.map_2d_location.begin()->y << " ~ " << main_vehicle.map_2d_location.end()->y << " )";
                for (auto iter1 = main_vehicle.map_2d_location.begin(); iter1 < main_vehicle.map_2d_location.end(); iter1++)
                {
                    adcm::map2dIndex index_to_push;
                    index_to_push.x = iter1->x;
                    index_to_push.y = iter1->y;
                    map_2d_test[index_to_push.x][index_to_push.y].vehicle_class = main_vehicle_final.vehicle_class;
                    map_2d_test[index_to_push.x][index_to_push.y].road_z = 1;
                    // adcm::Log::Info() << "map_2d_test[" << index_to_push.x << "][" <<index_to_push.y << "] = " <<  map_2d_test[index_to_push.x][index_to_push.y].vehicle_class;
                    // adcm::Log::Info() << "map_2d_test[" << index_to_push.x << "][" <<index_to_push.y << "] = " <<  map_2d_test[index_to_push.x][index_to_push.y].road_z;
                    main_vehicle_final.map_2d_location.push_back(index_to_push);
                }

                main_vehicle_final.position_long = main_vehicle.position_long;
                main_vehicle_final.position_lat = main_vehicle.position_lat;
                main_vehicle_final.position_height = main_vehicle.position_height;
                main_vehicle_final.position_x = main_vehicle.position_x;
                main_vehicle_final.position_y = main_vehicle.position_y;
                main_vehicle_final.position_z = main_vehicle.position_z;
                main_vehicle_final.yaw = main_vehicle.yaw;
                main_vehicle_final.roll = main_vehicle.roll;
                main_vehicle_final.velocity_long = main_vehicle.velocity_long;
                main_vehicle_final.velocity_lat = main_vehicle.velocity_lat;
                main_vehicle_final.velocity_x = main_vehicle.velocity_x;
                main_vehicle_final.velocity_y = main_vehicle.velocity_y;
                main_vehicle_final.velocity_ang = main_vehicle.velocity_ang;
                mapData.vehicle_list.push_back(main_vehicle_final);
                INFO("main_vehicle_final pushed to mapData");
            }

            if (sub1_vehicle.vehicle_class == SUB_VEHICLE_1)
            // 테스트용 sub1 값이 있을때만 아래 수행
            {
                adcm::Log::Info() << "sub1_vehicle push to mapData (x:" << sub1_vehicle.map_2d_location.begin()->x << " ~ " << sub1_vehicle.map_2d_location.end()->x << " )";
                adcm::Log::Info() << "sub1_vehicle push to mapData (y:" << sub1_vehicle.map_2d_location.begin()->y << " ~ " << sub1_vehicle.map_2d_location.end()->y << " )";
                sub1_vehicle_final.vehicle_class = sub1_vehicle.vehicle_class;
                sub1_vehicle_final.timestamp = sub1_vehicle.timestamp;
                sub1_vehicle_final.map_2d_location.clear();
                for (auto iter1 = sub1_vehicle.map_2d_location.begin(); iter1 < sub1_vehicle.map_2d_location.end(); iter1++)
                {
                    adcm::map2dIndex index_to_push;
                    index_to_push.x = iter1->x;
                    index_to_push.y = iter1->y;
                    map_2d_test[index_to_push.x][index_to_push.y].vehicle_class = sub1_vehicle_final.vehicle_class;
                    map_2d_test[index_to_push.x][index_to_push.y].road_z = 1;
                    // adcm::Log::Info() << "map_2d_test[" << index_to_push.x << "][" <<index_to_push.y << "] = " <<  map_2d_test[index_to_push.x][index_to_push.y].vehicle_class;
                    sub1_vehicle_final.map_2d_location.push_back(index_to_push);
                }
                sub1_vehicle_final.position_long = sub1_vehicle.position_long;
                sub1_vehicle_final.position_lat = sub1_vehicle.position_lat;
                sub1_vehicle_final.position_height = sub1_vehicle.position_height;
                sub1_vehicle_final.position_x = sub1_vehicle.position_x;
                sub1_vehicle_final.position_y = sub1_vehicle.position_y;
                sub1_vehicle_final.position_z = sub1_vehicle.position_z;
                sub1_vehicle_final.yaw = sub1_vehicle.yaw;
                sub1_vehicle_final.roll = sub1_vehicle.roll;
                sub1_vehicle_final.velocity_long = sub1_vehicle.velocity_long;
                sub1_vehicle_final.velocity_lat = sub1_vehicle.velocity_lat;
                sub1_vehicle_final.velocity_x = sub1_vehicle.velocity_x;
                sub1_vehicle_final.velocity_y = sub1_vehicle.velocity_y;
                sub1_vehicle_final.velocity_ang = sub1_vehicle.velocity_ang;

                mapData.vehicle_list.push_back(sub1_vehicle_final);
                INFO("sub1_vehicle_final pushed to mapData");
            }

            if (sub2_vehicle.vehicle_class == SUB_VEHICLE_2)
            {
                sub2_vehicle_final.vehicle_class = sub2_vehicle.vehicle_class;
                sub2_vehicle_final.timestamp = sub2_vehicle.timestamp;
                sub2_vehicle_final.map_2d_location.clear();
                adcm::Log::Info() << "sub2_vehicle push to mapData (x:" << sub2_vehicle.map_2d_location.begin()->x << " ~ " << sub2_vehicle.map_2d_location.end()->x << " )";
                adcm::Log::Info() << "sub2_vehicle push to mapData (y:" << sub2_vehicle.map_2d_location.begin()->y << " ~ " << sub2_vehicle.map_2d_location.end()->y << " )";
                for (auto iter1 = sub2_vehicle.map_2d_location.begin(); iter1 < sub2_vehicle.map_2d_location.end(); iter1++)
                {
                    adcm::map2dIndex index_to_push;
                    index_to_push.x = iter1->x;
                    index_to_push.y = iter1->y;
                    map_2d_test[index_to_push.x][index_to_push.y].vehicle_class = sub2_vehicle_final.vehicle_class;
                    map_2d_test[index_to_push.x][index_to_push.y].road_z = 1;
                    // adcm::Log::Info() << "map_2d_test[" << index_to_push.x << "][" <<index_to_push.y << "] = " <<  map_2d_test[index_to_push.x][index_to_push.y].vehicle_class;
                    sub2_vehicle_final.map_2d_location.push_back(index_to_push);
                }
                sub2_vehicle_final.position_long = sub2_vehicle.position_long;
                sub2_vehicle_final.position_lat = sub2_vehicle.position_lat;
                sub2_vehicle_final.position_height = sub2_vehicle.position_height;
                sub2_vehicle_final.position_x = sub2_vehicle.position_x;
                sub2_vehicle_final.position_y = sub2_vehicle.position_y;
                sub2_vehicle_final.position_z = sub2_vehicle.position_z;
                sub2_vehicle_final.yaw = sub2_vehicle.yaw;
                sub2_vehicle_final.roll = sub2_vehicle.roll;
                sub2_vehicle_final.velocity_long = sub2_vehicle.velocity_long;
                sub2_vehicle_final.velocity_lat = sub2_vehicle.velocity_lat;
                sub2_vehicle_final.velocity_x = sub2_vehicle.velocity_x;
                sub2_vehicle_final.velocity_y = sub2_vehicle.velocity_y;
                sub2_vehicle_final.velocity_ang = sub2_vehicle.velocity_ang;

                mapData.vehicle_list.push_back(sub2_vehicle_final);
                INFO("sub2_vehicle_final pushed to mapData");
            }
            //==============mapData.2d 가 obstacle 리스트와 vehicle 리스트를 정보를 가지도록 assignment============
            /*if(once)
            {
                for (int i =29; i < 100; i++)
                {
                    for (int j=45; j <55; j++)
                    {
                        map_2d_test[i][j].road_z = 1;
                    }
                }
            once = 0;
            }*/

            mapData.map_2d.clear();
            for (int i = 0; i < map_n; ++i)
            {
                map_2dListVector.clear();
                for (int j = 0; j < map_m; ++j)
                {
                    map_2dStruct.obstacle_id = map_2d_test[i][j].obstacle_id;
                    map_2d_test[i][j].obstacle_id = NO_OBSTACLE;
                    // obstacle_id 초기화
                    map_2dStruct.vehicle_class = map_2d_test[i][j].vehicle_class;
                    map_2d_test[i][j].vehicle_class = NO_VEHICLE;
                    // vehicle_class 초기화
                    map_2dStruct.road_z = map_2d_test[i][j].road_z;
                    // road_z 정보는 계속 가져간다
                    map_2dListVector.push_back(map_2dStruct);
                }
                mapData.map_2d.push_back(map_2dListVector);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            adcm::Log::Info() << "DATA FUSION DONE";
            INFO("map_2d pushed to mapData");
            mapData_provider.send(mapData);
            INFO("mapData sent to RASS");
        }
        else
        {
            adcm::Log::Info() << "Invalid input data - no map data sent";
            // mapData_provider.send(mapData);
        }

        /*
        int i = 30;
        int j = 50;
        adcm::Log::Info() << "[" << i << "]" << "[" << j << "]" << "obstacle_id : " << mapData.map_2d[i][j].obstacle_id;
        adcm::Log::Info() << "[" << i << "]" << "[" << j << "]" << "vehicle_class : " << mapData.map_2d[i][j].vehicle_class;
        adcm::Log::Info() << "[" << i << "]" << "[" << j << "]" << "road_z : " << mapData.map_2d[i][j].road_z;
        */
    }
}
void ThreadMonitor()
{
    while (continueExecution)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if (gMainthread_Loopcount == 0)
        {
            adcm::Log::Error() << "Main thread Timeout!!!";
        }
        else
        {
            gMainthread_Loopcount = 0;

            if (gReceivedEvent_count_hub_data != 0)
            {
                adcm::Log::Info() << "hub_data Received count = " << gReceivedEvent_count_hub_data;
                gReceivedEvent_count_hub_data = 0;
            }
            else
            {
                adcm::Log::Info() << "hub_data event timeout!!!";
            }
        }
    }
}

int main(int argc, char *argv[])
{
    std::vector<std::thread> thread_list;
    UNUSED(argc);
    UNUSED(argv);

    if (!ara::core::Initialize())
    {
        // No interaction with ARA is possible here since initialization failed
        return EXIT_FAILURE;
    }

    ara::exec::ExecutionClient exec_client;
    exec_client.ReportExecutionState(ara::exec::ExecutionState::kRunning);

    if (!RegisterSigTermHandler())
    {
        adcm::Log::Error() << "Unable to register signal handler";
    }

#ifndef R19_11_1
    adcm::Log::Info() << "DataFusion: configure e2e protection";
    bool success = ara::com::e2exf::StatusHandler::Configure("./etc/e2e_dataid_mapping.json",
                                                             ara::com::e2exf::ConfigurationFormat::JSON,
                                                             "./etc/e2e_statemachines.json",
                                                             ara::com::e2exf::ConfigurationFormat::JSON);
    adcm::Log::Info() << "DataFusion: e2e configuration " << (success ? "succeeded" : "failed");
#endif
    adcm::Log::Info() << "Ok, let's produce some DataFusion data...";

    thread_list.push_back(std::thread(ThreadReceiveHudData));
    thread_list.push_back(std::thread(ThreadMonitor));
    thread_list.push_back(std::thread(ThreadKatech));
    // thread_list.push_back(std::thread(DataFusion::ThreadReceiveWorkInfo));
    adcm::Log::Info() << "Thread join";
    for (int i = 0; i < static_cast<int>(thread_list.size()); i++)
    {
        thread_list[i].join();
    }

    adcm::Log::Info() << "done.";

    if (!ara::core::Deinitialize())
    {
        // No interaction with ARA is possible here since some ARA resources can be destroyed already
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}