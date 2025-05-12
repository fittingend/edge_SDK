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
#include <unordered_set>
#include <mutex>

#include <ara/com/e2exf/status_handler.h>
#include <ara/exec/execution_client.h>

#include <ara/log/logger.h>
#include "logger.h"
#include "ara/core/initialization.h"

#include "risk_assessment_provider.h"
#include "build_path_subscriber.h"
#include "build_path_test_subscriber.h"
#include "map_data_subscriber.h"
#include "main_riskassessment.hpp"
#define WHEEL_DIAMETER_M 0.71
// 추후에 수정 필요 특장차 휠의 지름 (미터단위) - 28인치로 우선 설정
//================1. global variables=========================
std::vector<adcm::map_2dListVector> map_2d;
std::mutex mtx;
obstacleListVector obstacle_list_temp;
adcm::vehicleListStruct ego_vehicle_temp, sub_vehicle_1_temp, sub_vehicle_2_temp, sub_vehicle_3_temp, sub_vehicle_4_temp;
doubleVector position_x;
doubleVector position_y;
obstacleListVector obstacle_pedes_initial;   // 시나리오 5 용
obstacleListVector obstacle_vehicle_initial; // 시나리오 6 용
//=================================================================

// Atomic flag for exit after SIGTERM caught
std::atomic_bool continueExecution{true};
std::atomic_uint gReceivedEvent_count_map_data{0};
std::atomic_uint gReceivedEvent_count_build_path{0};
std::atomic_uint gReceivedEvent_count_build_path_test{0};
std::atomic_uint gMainthread_Loopcount{0};

//============== 2. 함수 definition =================
// #define NATS_TEST
#ifdef NATS_TEST
#include "./NATS_IMPLEMENTATION/NatsConnManager.h"
#define HMI_SERVER_URL "https://nats.beyless.com"
#include <sstream>
#include <fstream> // Required for file handling
#include <filesystem>

bool firstTime = true;
natsStatus s = NATS_OK;
std::vector<const char *> subject = {"test1.*", "test2.*"};
std::shared_ptr<adcm::etc::NatsConnManager> natsManager;

void asyncCb(natsConnection *nc, natsSubscription *sub, natsStatus err, void *closure)
{
    std::cout << "Async error: " << err << " - " << natsStatus_GetText(err) << std::endl;
    natsManager->NatsSubscriptionGetDropped(sub, (int64_t *)&natsManager->dropped);
}

void onMsg(natsConnection *nc, natsSubscription *sub, natsMsg *msg, void *closure)
{
    const char *subject = NULL;

    // 뮤텍스를 사용하여 공유 변수 접근 보호
    std::lock_guard<std::mutex> lock(mtx);
    subject = natsMsg_GetSubject(msg);

    std::cout << "Received msg: [" << subject << " : " << natsMsg_GetDataLength(msg) << "]" << natsMsg_GetData(msg) << std::endl;
    natsManager->NatsMsgDestroy(msg);
}

void saveToJsonFile(const std::string &key, const std::string &value, int &fileCount)
{
    std::ostringstream fileNameStream;
    fileNameStream << key << "_" << fileCount << ".json"; // Construct the file name
    std::string fileName = fileNameStream.str();
    std::ofstream outFile(fileName);
    // Open file for writing with the generated name
    if (outFile.is_open())
    {
        outFile << value; // Write the JSON string to the file
        outFile.close();  // Close the file after writing
        adcm::Log::Info() << "JSON data stored in " << fileName;
    }
    else
    {
        adcm::Log::Error() << "Failed to open file " << fileName << " for writing!";
    }

    // Increment the file count for the next call
    ++fileCount;
}
std::string convertRiskAssessmentToJsonString(const adcm::risk_assessment_Objects &riskAssessment)
{
    // test purpose

    uint64_t timestamp_map = 111;
    uint64_t timestamp_risk = riskAssessment.timestamp;
    std::string model_id = "test_id_v1";

    std::ostringstream oss; // Use a string stream for easier manipulation
    oss << "{\n";           // Start the JSON object

    // Convert riskAssessmentList to JSON
    oss << "    \"riskAssessmentList\": [\n";
    for (size_t i = 0; i < riskAssessment.riskAssessmentList.size(); ++i)
    {
        const auto &item = riskAssessment.riskAssessmentList[i];
        oss << "        {\n"
            << "            \"obstacle_id\": " << item.obstacle_id << ",\n"
            << "            \"wgs84_xy_start\": [\n";

        // Convert wgs84_xy_start to JSON
        for (size_t j = 0; j < item.wgs84_xy_start.size(); ++j)
        {
            const auto &start = item.wgs84_xy_start[j];
            oss << "                {\n"
                << "                    \"x\": " << start.x << ",\n"
                << "                    \"y\": " << start.y << "\n"
                << "                }";

            if (j < item.wgs84_xy_start.size() - 1)
            {
                oss << ",";
            }
            oss << "\n"; // Newline for readability
        }

        oss << "            ],\n" // Close wgs84_xy_start array
            << "            \"wgs84_xy_end\": [\n";

        // Convert wgs84_xy_end to JSON
        for (size_t j = 0; j < item.wgs84_xy_end.size(); ++j)
        {
            const auto &end = item.wgs84_xy_end[j];
            oss << "                {\n"
                << "                    \"x\": " << end.x << ",\n"
                << "                    \"y\": " << end.y << "\n"
                << "                }";

            if (j < item.wgs84_xy_end.size() - 1)
            {
                oss << ",";
            }
            oss << "\n"; // Newline for readability
        }

        oss << "            ],\n"                                                               // Close wgs84_xy_end array
            << "            \"hazard_class\": " << static_cast<int>(item.hazard_class) << ",\n" // hazard_class
            << "            \"isHarzard\": " << (item.isHarzard ? "true" : "false") << ",\n"    // isHazard
            << "            \"confidence\": " << item.confidence << ",\n"                       // confidence
            << "            \"timestamp_map\": " << timestamp_map << ",\n"                      // timestamp_map
            << "            \"timestamp_risk\": " << timestamp_risk << ",\n"                    // timestamp_risk
            << "            \"model_id\": \"" << model_id << "\"\n";                            // model_id

        oss << "        }";

        if (i < riskAssessment.riskAssessmentList.size() - 1)
        {
            oss << ",";
        }
        oss << "\n"; // Newline for readability
    }
    oss << "    ]\n"; // Close riskAssessmentList array
    oss << "}";       // Close the main JSON object

    return oss.str(); // Return the constructed JSON string
}

void NatsSend(const adcm::risk_assessment_Objects &riskAssessment)
{
    static int risk_count = 0; // Static variable to track file number

    if (firstTime == true)
    {
        adcm::Log::Info() << "NATS first time setup!";
        natsManager = std::make_shared<adcm::etc::NatsConnManager>(HMI_SERVER_URL, subject, onMsg, asyncCb, adcm::etc::NatsConnManager::Mode::Default);
        s = natsManager->NatsExecute();
        firstTime = false;
    }
    if (s == NATS_OK)
    {
        const char *pubSubject = "test2.JSON";
        natsManager->ClearJsonData();
        std::string riskToStr = convertRiskAssessmentToJsonString(riskAssessment);
        natsManager->addJsonData("riskAssessment", riskToStr);
        natsManager->NatsPublishJson(pubSubject);
        adcm::Log::Info() << "NatsPublishJson";
        saveToJsonFile("riskAssessment", riskToStr, risk_count);
    }
    else
    {
        std::cout << "Nats Connection error" << std::endl;
        try
        {
            natsManager = std::make_shared<adcm::etc::NatsConnManager>(HMI_SERVER_URL, subject, onMsg, asyncCb, adcm::etc::NatsConnManager::Mode::Default);
            s = natsManager->NatsExecute();
        }
        catch (std::exception e)
        {
            std::cout << "Nats reConnection error" << std::endl;
        }
    }
}
#endif
template <typename T>
void clear_and_free_memory(std::vector<T> &vec)
{
    // 벡터의 내용을 모두 삭제
    vec.clear();
    // 벡터가 할당한 메모리 공간을 최소화
    vec.shrink_to_fit();
}

void GPStoUTM(double lat, double lon, double &utmX, double &utmY)
{
    INFO("[RiskAssessment] GPStoUTM");
    // WGS84 Parameters
    const double WGS84_A = 6378137.0;    // Major semiaxis [m]
    const double WGS84_E = 0.0818191908; // First Eccentricity
    // Constants
    const double k0 = 0.9996;
    const double eSquared = WGS84_E * WGS84_E;
    const double ePrimeSquared = eSquared / (1 - eSquared);
    const double RADIANS_PER_DEGREE = M_PI / 180.0;
    int zone = 52;
    // Calculate the Central Meridian of the Zone
    double lonOrigin = (zone - 1) * 6 - 180 + 3;

    // Convert lat/lon to radians
    double latRad = lat * RADIANS_PER_DEGREE;
    double lonRad = lon * RADIANS_PER_DEGREE;
    double lonOriginRad = lonOrigin * RADIANS_PER_DEGREE;

    // Calculate UTM coordinates
    double N = WGS84_A / sqrt(1 - eSquared * sin(latRad) * sin(latRad));
    double T = tan(latRad) * tan(latRad);
    double C = ePrimeSquared * cos(latRad) * cos(latRad);
    double A = cos(latRad) * (lonRad - lonOriginRad);

    double M = WGS84_A * ((1 - eSquared / 4 - 3 * pow(eSquared, 2) / 64 - 5 * pow(eSquared, 3) / 256) * latRad - (3 * eSquared / 8 + 3 * pow(eSquared, 2) / 32 + 45 * pow(eSquared, 3) / 1024) * sin(2 * latRad) + (15 * pow(eSquared, 2) / 256 + 45 * pow(eSquared, 3) / 1024) * sin(4 * latRad) - (35 * pow(eSquared, 3) / 3072) * sin(6 * latRad));
    utmX = (k0 * N * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ePrimeSquared) * A * A * A * A * A / 120) + 500000.0);
    utmY = (k0 * (M + N * tan(latRad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * ePrimeSquared) * A * A * A * A * A * A / 720)));

    if (lat < 0)
    {
        utmY += 10000000.0; // 10000000 meter offset for southern hemisphere
    }
}

bool isRouteValid(routeVector &route)
{
    for (int count = 0; count < route.size(); count++)
    {
        if (!(33 < route[count].latitude && route[count].latitude < 39 &&
              124 < route[count].longitude && route[count].longitude < 132))
        {
            return false; // Return false immediately if any condition is not met
        }
    }
    return true; // Return true if all values meet the condition
}

void gpsToMapcoordinate(routeVector &route)
{
    INFO("[RiskAssessment] gpsToMapcoordinate");

    // wps84기반 gps(global)좌표계를 작업환경 XY 기반의 Map 좌표계로 변환
    // 시뮬레이터 map 기준 원점(0,0) global좌표
    double mapOrigin_x = 453.088714;
    double mapOrigin_y = 507.550078;
    // 시뮬레이터 기준점 utm좌표
    double origin_x = 278296.968;
    double origin_y = 3980466.846;
    double angle_radians = -MAP_ANGLE * M_PI / 180.0;

    for (int count = 0; count < route.size(); count++)
    {
        double utm_x, utm_y; // 해당 경로의 utm x,y 좌표
        GPStoUTM(route[count].latitude, route[count].longitude, utm_x, utm_y);
        utm_x -= origin_x;
        utm_y -= origin_y;
        adcm::Log::Info() << "utm_x:" << utm_x;
        adcm::Log::Info() << "utm_y:" << utm_y;

        position_x[count] = static_cast<int>((utm_x * cos(angle_radians) - utm_y * sin(angle_radians) - mapOrigin_x) * M_TO_10CM_PRECISION);
        position_y[count] = static_cast<int>((utm_x * sin(angle_radians) + utm_y * cos(angle_radians) - mapOrigin_y) * M_TO_10CM_PRECISION);

        adcm::Log::Info() << "경로생성 값 gpsToMapcoordinate 좌표변환 before (" << route[count].latitude << " , " << route[count].longitude << ")";
        adcm::Log::Info() << "경로생성 값 gpsToMapcoordinate 좌표변환 after (" << position_x[count] << " , " << position_y[count] << ")";
    }
}

double getDistance(adcm::obstacleListStruct obstacle1, adcm::obstacleListStruct obstacle2)
{
    return sqrt(pow(obstacle1.fused_position_x - obstacle2.fused_position_x, 2) + pow(obstacle1.fused_position_y - obstacle2.fused_position_y, 2));
}
double getDistance(adcm::obstacleListStruct obstacle, adcm::vehicleListStruct vehicle)
{
    return sqrt(pow(obstacle.fused_position_x - vehicle.position_x, 2) + pow(obstacle.fused_position_y - vehicle.position_y, 2));
}
double getMagnitude(Point2D point)
{
    return sqrt(point.x * point.x + point.y * point.y);
}
double getTTC(const adcm::obstacleListStruct &obstacle, const adcm::vehicleListStruct &vehicle)
{
    // Calculate relative position and velocity of the obstacle with respect to the vehicle
    double relative_position_x = obstacle.fused_position_x - vehicle.position_x;
    double relative_position_y = obstacle.fused_position_y - vehicle.position_y;
    double relative_velocity_x = obstacle.fused_velocity_x - vehicle.velocity_x;
    double relative_velocity_y = obstacle.fused_velocity_y - vehicle.velocity_y;

    // Calculate determinant (cross product) for relative motion
    double c = (relative_velocity_x * relative_position_y) - (relative_velocity_y * relative_position_x);

    // Prevent division by zero when relative_velocity_y is zero
    if (relative_velocity_y == 0 || relative_velocity_x == 0)
    {
        return INVALID_RETURN_VALUE; // No meaningful TTC can be calculated
    }

    // Calculate TTC using relative motion
    double ttc = ((c / (2 * relative_velocity_y)) - relative_position_x) / relative_velocity_x;

    // Return TTC if valid, otherwise return a predefined invalid value
    return (ttc > 0) ? ttc : INVALID_RETURN_VALUE;
}
double getDistance_LinearTrajectory(const adcm::obstacleListStruct &obstacle)
{
    double distance = 0;

    if (position_x.size() < 2 || position_y.size() < 2 || position_x.size() != position_y.size())
    {
        adcm::Log::Error() << "Invalid trajectory data.";
        return INVALID_RETURN_VALUE;
    }

    for (int count = 0; count < position_x.size(); count++)
    {
        double x_start = position_x[count];
        double x_end = position_x[count + 1];
        double y_start = position_y[count];
        double y_end = position_y[count + 1];

        // Vectors: start-to-end and start-to-obstacle
        Point2D start_to_end_vector = {x_end - x_start, y_end - y_start};
        Point2D start_to_obs_vector = {obstacle.fused_position_x - x_start, obstacle.fused_position_y - y_start};

        // Calculate angle
        double angle = atan2(start_to_end_vector.y, start_to_end_vector.x) - atan2(start_to_obs_vector.y, start_to_obs_vector.x);
        double cosine_angle = cos(angle);

        // Check if obstacle is between the start and end points
        double start_to_end_dot_start_to_obs = cosine_angle * getMagnitude(start_to_end_vector) * getMagnitude(start_to_obs_vector);
        double magnitude_start_to_end = getMagnitude(start_to_end_vector);

        if (start_to_end_dot_start_to_obs > 0 && start_to_end_dot_start_to_obs < pow(magnitude_start_to_end, 2))
        {
            // Calculate perpendicular distance
            double magnitude_start_to_obs = getMagnitude(start_to_obs_vector);
            double distance = magnitude_start_to_obs * sqrt(1 - pow(start_to_end_dot_start_to_obs / (magnitude_start_to_end * magnitude_start_to_obs), 2));

            adcm::Log::Info() << "Obstacle " << obstacle.obstacle_id << " is within trajectory segment " << count
                              << ". Distance: " << distance;
            return distance;
        }
    }
    adcm::Log::Info() << "Obstacle " << obstacle.obstacle_id << " is not within any trajectory segment.";
    return INVALID_RETURN_VALUE;
}
double getLinearApprox(const adcm::obstacleListStruct &obstacle, const adcm::vehicleListStruct &vehicle)
{
    // Initialize minimum distance to a very large value
    double min_distance_ego_obs = INVALID_RETURN_VALUE;

    // Loop to calculate positions at intervals and find the minimum distance
    for (double k = 0.0; k <= 5.0; k += 0.5)
    {
        // Compute positions at time `k`
        double vehicle_future_x = vehicle.position_x + vehicle.velocity_x * k;
        double vehicle_future_y = vehicle.position_y + vehicle.velocity_y * k;
        double obstacle_future_x = obstacle.fused_position_x + obstacle.fused_velocity_x * k;
        double obstacle_future_y = obstacle.fused_position_y + obstacle.fused_velocity_y * k;

        // Calculate the distance between future positions
        double temp_distance = sqrt(pow(obstacle_future_x - vehicle_future_x, 2) +
                                    pow(obstacle_future_y - vehicle_future_y, 2));

        // Update minimum distance if a smaller value is found
        if (temp_distance < min_distance_ego_obs)
        {
            min_distance_ego_obs = temp_distance;
        }
    }

    // Return the smallest distance found
    return min_distance_ego_obs;
}
void symmDiff(const obstacleListVector &vec1, const obstacleListVector &vec2, obstacleListVector &output, int n, int m)
{
    // Traverse both arrays simultaneously.
    int i = 0, j = 0;
    while (i < n && j < m)
    {
        // Print smaller element and move
        // ahead in array with smaller element
        if (vec1[i].obstacle_id < vec2[j].obstacle_id)
        {
            output.push_back(vec1[i]);
            i++;
        }
        else if (vec2[j].obstacle_id < vec1[i].obstacle_id)
        {
            output.push_back(vec2[j]);
            j++;
        }
        // If both elements same, move ahead
        // in both arrays.
        else
        {
            i++;
            j++;
        }
    }
    while (i < n)
    {
        output.push_back(vec1[i]);
        i++;
    }
    while (j < m)
    {
        output.push_back(vec2[j]);
        j++;
    }
}
void detectUnscannedPath(adcm::risk_assessment_Objects &riskAssessment)
{
    bool breakFlag; // 지정된 전역경로 (x1,y1) 과 (x2, y2) 사이 하나라도
    for (int count = 0; count < position_x.size() - 1; count++)
    {
        int x_start = floor(position_x[count]);
        int x_end = floor(position_x[count + 1]);
        int y_start = floor(position_y[count]);
        int y_end = floor(position_y[count + 1]);

        // Bresenham's line algorithm
        const bool steep = (fabs(y_end - y_start) > fabs(x_end - x_start));
        if (steep)
        {
            std::swap(x_start, y_start);
            std::swap(x_end, y_end);
        }

        if (x_start > x_end)
        {
            std::swap(x_start, x_end);
            std::swap(y_start, y_end);
        }
        const float dx = x_end - x_start;
        const float dy = fabs(y_end - y_start);
        float error = dx / 2.0f;
        const int ystep = (y_start < y_end) ? 1 : -1;
        int y = (int)y_start;
        const int maxX = x_end;

        for (int x = x_start; x <= maxX; x++)
        {
            if (steep)
            {
                if (map_2d[y][x].road_z != 1) // 스캔되지 않은 map 의 index - 노면정보 X
                {
                    adcm::riskAssessmentStruct riskAssessment7;
                    riskAssessment7.wgs84_xy_start.clear();
                    riskAssessment7.wgs84_xy_end.clear();
                    adcm::globalPathPosition unscanned_start_path, unscanned_end_path;
                    unscanned_start_path.x = position_x[count];
                    unscanned_start_path.y = position_y[count];
                    unscanned_end_path.x = position_x[count + 1];
                    unscanned_end_path.y = position_y[count + 1];
                    riskAssessment7.wgs84_xy_start.push_back(unscanned_start_path);
                    riskAssessment7.wgs84_xy_end.push_back(unscanned_end_path);
                    riskAssessment7.hazard_class = SCENARIO_7;
                    riskAssessment7.isHarzard = true;
                    adcm::Log::Info() << "Risk assessment generated for #7 is X: " << position_x[count] << " Y: " << position_y[count] << " with flag 1 ";
                    riskAssessment.riskAssessmentList.push_back(riskAssessment7);
                    break; // 한번만 들어가도 for loop break
                }
            }

            else if (map_2d[x][y].road_z != 1) // 스캔되지 않은 map 의 index - 노면정보 X
            {
                adcm::riskAssessmentStruct riskAssessment7;
                // adcm::Log::Info() << "detectUnscannedPath test else";
                adcm::Log::Info() << "map_2d[" << x << "][" << y << "] is 0";
                riskAssessment7.wgs84_xy_start.clear();
                riskAssessment7.wgs84_xy_end.clear();
                adcm::globalPathPosition unscanned_start_path, unscanned_end_path;
                unscanned_start_path.x = position_x[count];
                unscanned_start_path.y = position_y[count];
                unscanned_end_path.x = position_x[count + 1];
                unscanned_end_path.y = position_y[count + 1];
                riskAssessment7.wgs84_xy_start.push_back(unscanned_start_path);
                riskAssessment7.wgs84_xy_end.push_back(unscanned_end_path);
                riskAssessment7.hazard_class = SCENARIO_7;
                riskAssessment7.isHarzard = true;
                adcm::Log::Info() << "Risk assessment generated for #7 is X: " << riskAssessment7.wgs84_xy_start[0].x << " Y: " << riskAssessment7.wgs84_xy_start[0].y << " with flag" << riskAssessment7.isHarzard;
                riskAssessment.riskAssessmentList.push_back(riskAssessment7);
                break;
            }
            /*if (map_2d[x][y].road_z ==1)
            {
                adcm::Log::Info() << "map_2d[" << x << "][" <<x << "] is 1";
                adcm::Log::Info() << "scanned! safe to go";
            }*/

            error -= dy;
            if (error < 0)
            {
                y += ystep;
                error += dx;
            }
        }
    }
}

void calculateShiftedLines(int &x_start, int &x_end, int &y_start, int &y_end, int shift, double &original_m, double &original_c, double &up_c, double &down_c, bool &isVertical, double &x_up, double &x_down)
{
    // Calculate the slope (gradient) of the original line

    double dx = x_end - x_start;
    double dy = y_end - y_start;

    // Check if the line is vertical
    if (dx == 0)
    {
        isVertical = true;
        original_m = INFINITY; // Undefined slope
        original_c = INFINITY; // No y-intercept
        x_up = x_start + shift;
        x_down = x_start - shift;
        return;
    }
    isVertical = false;
    original_m = dy / dx;                        // Slope of the original line
    original_c = y_start - original_m * x_start; // Intercept of the original line

    // Adjust intercepts for vertical shifts
    up_c = original_c + shift;
    down_c = original_c - shift;
}

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

void ThreadReceiveMapData()
{
    adcm::Log::Info() << "RiskAssessment ThreadReceiveMapData";
    adcm::MapData_Subscriber mapData_subscriber;
    mapData_subscriber.init("RiskAssessment/RiskAssessment/RPort_map_data");
    INFO("Thread ThreadReceiveMapData start...");

    while (continueExecution)
    {
        gMainthread_Loopcount++;
        VERBOSE("[RiskAssessment] Application loop");
        bool mapData_rxEvent = mapData_subscriber.waitEvent(100); // wait event

        if (mapData_rxEvent)
        {
            adcm::Log::Info() << "[EVENT] RiskAssessment Map Data received";

            while (!mapData_subscriber.isEventQueueEmpty())
            {
                auto data = mapData_subscriber.getEvent();
                gReceivedEvent_count_map_data++;

                auto timestamp = data->timestamp;
                map_2d = data->map_2d;
                auto obstacle_list = data->obstacle_list;
                auto vehicle_list = data->vehicle_list;

                adcm::Log::Verbose() << "timestamp : " << timestamp;

                if (!map_2d.empty())
                {
                    adcm::Log::Verbose() << "=== map_2d ===";
                    adcm::Log::Info() << "size of map_2d received: " << map_2d.size() * map_2d[0].size();
                }
                else
                {
                    adcm::Log::Verbose() << "map_2d Vector empty!!! ";
                }

                if (!obstacle_list.empty())
                {
                    adcm::Log::Verbose() << "=== obstacle_list ===";
                    adcm::Log::Info() << "size of obstacle list received: " << obstacle_list.size();
                    for (auto itr = obstacle_list.begin(); itr != obstacle_list.end(); ++itr)
                    {
                        adcm::Log::Verbose() << "obstacle_id : " << itr->obstacle_id;
                        adcm::Log::Verbose() << "obstacle_class : " << itr->obstacle_class;
                        adcm::Log::Verbose() << "timestamp : " << itr->timestamp;

                        obstacle_list_temp = obstacle_list; // assign to gloabal variable
                    }
                }
                else
                {
                    adcm::Log::Verbose() << "obstacle_list Vector empty!!! ";
                }

                if (!vehicle_list.empty())
                {
                    adcm::Log::Verbose() << "=== vehicle_list ===";
                    adcm::Log::Info() << "size of vehicle list received: " << vehicle_list.size();
                    for (auto iter = vehicle_list.begin(); iter != vehicle_list.end(); iter++)
                    {
                        switch (iter->vehicle_class)
                        {
                        case EGO_VEHICLE:
                            ego_vehicle_temp = *iter;
                            break;
                        case SUB_VEHICLE_1:
                            sub_vehicle_1_temp = *iter;
                            break;
                        case SUB_VEHICLE_2:
                            sub_vehicle_2_temp = *iter;
                            break;
                        case SUB_VEHICLE_3:
                            sub_vehicle_3_temp = *iter;
                            break;
                        case SUB_VEHICLE_4:
                            sub_vehicle_4_temp = *iter;
                            break;
                        default:
                            break;
                        }
                    }
                }
                else
                {
                    adcm::Log::Verbose() << "vehicle_list Vector empty!!! ";
                }
            }
        }
    }
}
void ThreadReceiveBuildPathTest()
{
    adcm::Log::Info() << "RiskAssessment ThreadReceiveBuildPathTest";
    adcm::BuildPathTest_Subscriber buildPathTest_subscriber;
    buildPathTest_subscriber.init("RiskAssessment/RiskAssessment/RPort_build_path_test");
    INFO("Thread ThreadReceiveBuildPathTest start...");

    while (continueExecution)
    {
        gMainthread_Loopcount++;
        VERBOSE("[RiskAssessment] Application loop");
        bool buildPathTest_rxEvent = buildPathTest_subscriber.waitEvent(100); // wait event

        if (buildPathTest_rxEvent)
        {
            adcm::Log::Info() << "[EVENT] RiskAssessment Build Path Test received";

            while (!buildPathTest_subscriber.isEventQueueEmpty())
            {
                auto data = buildPathTest_subscriber.getEvent();
                gReceivedEvent_count_build_path_test++;

                auto size = data->size;
                auto utm_x = data->utm_x;
                auto utm_y = data->utm_y;

                adcm::Log::Verbose() << "size : " << size;

                if (!utm_x.empty())
                {
                    adcm::Log::Verbose() << "=== utm_x ===";
                    for (auto itr = utm_x.begin(); itr != utm_x.end(); ++itr)
                    {
                        adcm::Log::Verbose() << *itr;
                    }
                }
                else
                {
                    adcm::Log::Verbose() << "utm_x Vector empty!!! ";
                }

                if (!utm_y.empty())
                {
                    adcm::Log::Verbose() << "=== utm_y ===";
                    for (auto itr = utm_y.begin(); itr != utm_y.end(); ++itr)
                    {
                        adcm::Log::Verbose() << *itr;
                    }
                }
                else
                {
                    adcm::Log::Verbose() << "utm_y Vector empty!!! ";
                }
            }
        }
    }
}
void ThreadReceiveBuildPath()
{
    adcm::Log::Info() << "RiskAssessment ThreadReceiveBuildPath";
    adcm::BuildPath_Subscriber buildPath_subscriber;
    buildPath_subscriber.init("RiskAssessment/RiskAssessment/RPort_build_path");
    INFO("Thread ThreadReceiveBuildPath start...");

    while (continueExecution)
    {
        gMainthread_Loopcount++;
        VERBOSE("[RiskAssessment] Application loop");
        bool buildPath_rxEvent = buildPath_subscriber.waitEvent(100); // wait event

        if (buildPath_rxEvent)
        {
            adcm::Log::Info() << "[EVENT] RiskAssessment Build Path received";

            while (!buildPath_subscriber.isEventQueueEmpty())
            {
                auto data = buildPath_subscriber.getEvent();
                gReceivedEvent_count_build_path++;

                auto Path = data->Path;

                if (!Path.empty())
                {
                    adcm::Log::Verbose() << "=== Path ===";
                    for (auto itr = Path.begin(); itr != Path.end(); ++itr)
                    {
                        adcm::Log::Verbose() << "result : " << itr->result;
                        adcm::Log::Verbose() << "timestamp : " << itr->timestamp;
                        adcm::Log::Verbose() << "mapdata_timestamp : " << itr->mapdata_timestamp;
                        adcm::Log::Verbose() << "riskassessment_timestamp : " << itr->riskassessment_timestamp;
                        adcm::Log::Verbose() << "t0 : " << itr->t0;
                        adcm::Log::Verbose() << "vehicle_class : " << itr->vehicle_class;
                        adcm::Log::Verbose() << "move_type : " << itr->move_type;
                        adcm::Log::Verbose() << "job_type : " << itr->job_type;
                        adcm::Log::Verbose() << "size : " << itr->size;

                        auto route = itr->route;
                        if (!route.empty())
                        {
                            if (itr->vehicle_class == EGO_VEHICLE)
                            {
                                if (isRouteValid(route))
                                {
                                    INFO("특장차의 경로가 valid 함 => 좌표전환 진행");
                                    mtx.lock();
                                    position_x.clear();
                                    position_y.clear();
                                    // 새로운 경로를 받으면 예전 변환값이 담긴 position_x 와 position_y 초기화
                                    gpsToMapcoordinate(route);
                                    mtx.unlock();
                                }
                                else
                                {
                                    INFO("특장차의 경로가 invalid 함 => do nothing");
                                }
                            }

                            adcm::Log::Verbose() << "=== route ===";
                            for (auto itr = route.begin(); itr != route.end(); ++itr)
                            {
                                adcm::Log::Verbose() << "route.latitude : " << itr->latitude;
                                adcm::Log::Verbose() << "route.longitude : " << itr->longitude;
                                adcm::Log::Verbose() << "route.delta_t : " << itr->delta_t;
                            }
                        }
                        else
                        {
                            adcm::Log::Info() << "route Vector empty!!! ";
                        }
                    }
                }
            }
        }
    }
}

void ThreadKatech()
{
    adcm::Log::Info() << "RiskAssessment ThreadKatech";
    adcm::RiskAssessment_Provider riskAssessment_provider;
    riskAssessment_provider.init("RiskAssessment/RiskAssessment/PPort_risk_assessment");
    INFO("Thread ThreadKatech start...");

    //================전역변수 생성==================
    obstacleListVector obstacle_list;
    adcm::vehicleListStruct ego_vehicle, sub_vehicle_1, sub_vehicle_2, sub_vehicle_3, sub_vehicle_4;
    adcm::risk_assessment_Objects riskAssessment;
    riskAssessment.riskAssessmentList.clear();

    while (continueExecution)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        obstacle_list = obstacle_list_temp;
        ego_vehicle = ego_vehicle_temp;
        sub_vehicle_1 = sub_vehicle_1_temp;
        sub_vehicle_2 = sub_vehicle_2_temp;

        if (map_2d.size() != 0)
        {
            {
                //=====시나리오 #1. 주행중 전역경로 근방 이동가능한 정지 장애물이 존재하는 위험 환경=====
                adcm::Log::Info() << "=============KATECH: scenario 1 START==============";
                obstacleListVector obstacle_stop;
                double distance_scenario_1;
                double confidence_scenario_1;

                //=========i) 정지상태 판정: 정해진 duration STOP_VALUE * 0.1s 만큼 정지해 있을경우
                for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
                {
                    if ((iter->stop_count > STOP_VALUE || iter->stop_count == STOP_VALUE) && (iter->obstacle_class != STRUCTURE) && (iter->obstacle_class != PEDESTRIAN))
                    {
                        adcm::Log::Info() << "시나리오1-i) 장애물 정지 상태 감지";
                        obstacle_stop.push_back(*iter);
                    }
                }

                //=========ii) 특장차로부터의 거리 판정: 30m 거리 이내인 경우
                for (auto iter = obstacle_stop.begin(); iter != obstacle_stop.end();)
                {
                    // adcm::Log::Info() << "시나리오1-ii)";
                    double distance_ego_obs = getDistance(*iter, ego_vehicle);
                    adcm::Log::Info() << "distance_ego_obs: " << distance_ego_obs;
                    if (distance_ego_obs > 300 || distance_ego_obs == 300)
                    {
                        adcm::Log::Info() << "시나리오1-ii) 특장차와 장애물 거리가 30m 이상이라 해당사항 없음";
                        iter = obstacle_stop.erase(iter); // 30m 이상인 경우 해당 장애물 삭제
                    }
                    else
                        ++iter;
                }

                //=========iii) 장애물과 전역경로간 거리 추정
                for (auto iter = obstacle_stop.begin(); iter != obstacle_stop.end(); iter++)
                {
                    distance_scenario_1 = getDistance_LinearTrajectory(*iter);
                    adcm::riskAssessmentStruct riskAssessment1;
                    confidence_scenario_1 = 200 / getDistance(*iter, ego_vehicle) * 0.7;
                    riskAssessment1.obstacle_id = iter->obstacle_id;
                    riskAssessment1.hazard_class = SCENARIO_1;
                    riskAssessment1.confidence = confidence_scenario_1;
                    adcm::Log::Info() << "Risk assessment generated for #1: " << iter->obstacle_id << "with confidence:  " << confidence_scenario_1;
                    riskAssessment.riskAssessmentList.push_back(riskAssessment1);
                }
                adcm::Log::Info() << "scenario 1 DONE";
                clear_and_free_memory(obstacle_stop);
            }

            {
                //=====시나리오 #2. 주행중 사각영역 존재 환경 판단=====
                adcm::Log::Info() << "=============KATECH: scenario 2 START==============";
                obstacleListVector obstacle_static_stop;
                double distance_scenario_2;
                double confidence_scenario_2;

                //=========i) 정지상태 판정: obstacle_class 가 정지한 동적객체 + 정적객체이고 1m 이상인 경우
                for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
                {
                    if ((iter->stop_count > STOP_VALUE || iter->stop_count == STOP_VALUE) && (iter->obstacle_class != STRUCTURE) && (iter->obstacle_class != PEDESTRIAN))
                    {
                        adcm::Log::Info() << "시나리오2-i) 동적 장애물 정지 상태 감지: " << iter->obstacle_id;
                        obstacle_static_stop.push_back(*iter);
                    }

                    else if (iter->obstacle_class == STRUCTURE && iter->fused_cuboid_z > 1)
                    {
                        adcm::Log::Info() << "시나리오2-i) 정적 장애물이 높이 1m 초과: " << iter->obstacle_id;
                        obstacle_static_stop.push_back(*iter);
                    }
                }

                //=========ii) 특장차로부터의 거리 판정: 40m 거리 이내인 경우
                for (auto iter = obstacle_static_stop.begin(); iter != obstacle_static_stop.end();)
                {
                    double distance_ego_obs = getDistance(*iter, ego_vehicle);
                    if (distance_ego_obs > 400 || distance_ego_obs == 400)
                    {
                        iter = obstacle_static_stop.erase(iter); // 40m 이상인 경우 해당 장애물 삭제
                    }
                    else
                        ++iter;
                }

                //==========iii) 장애물과 전역경로간 거리 추정
                for (auto iter = obstacle_static_stop.begin(); iter != obstacle_static_stop.end(); iter++)
                {
                    distance_scenario_2 = getDistance_LinearTrajectory(*iter);
                    //                if (distance_scenario_2 < 10)
                    if (!distance_scenario_2)
                    {
                        // adcm::Log::Info() << "시나리오2-iii) 장애물과 전역경로간 거리가 10m 이내 판별 불가 " << iter->obstacle_id;
                    }
                    else
                    {
                        adcm::Log::Info() << "시나리오2-iii) 장애물과 전역경로간 거리가 10m 이내 " << iter->obstacle_id;
                        adcm::riskAssessmentStruct riskAssessment2;
                        confidence_scenario_2 = 200 / getDistance(*iter, ego_vehicle) * 0.7;
                        riskAssessment2.obstacle_id = iter->obstacle_id;
                        riskAssessment2.hazard_class = SCENARIO_2;
                        riskAssessment2.confidence = confidence_scenario_2;
                        adcm::Log::Info() << "Risk assessment generated for #2: " << iter->obstacle_id << "with confidence:  " << confidence_scenario_2;
                        riskAssessment.riskAssessmentList.push_back(riskAssessment2);
                    }
                }
                adcm::Log::Info() << "scenario 2 DONE";
                clear_and_free_memory(obstacle_static_stop);
            }

            {
                //=====시나리오 #3. 주행 중 경로 주변 동적 장애물 통행 환경 판단=====
                adcm::Log::Info() << "=============KATECH: scenario 3 START==============";

                obstacleListVector obstacle_near_10_30;
                constexpr double SAFE_DISTANCE = 300;
                constexpr double TTC_THRESHOLD = 100;

                //=========i) 특장차로부터의 거리 판정: 10 < obs < 30m 거리 이내인 경우
                for (const auto &obstacle : obstacle_list)
                {
                    double distance_ego_obs = getDistance(obstacle, ego_vehicle);
                    if (distance_ego_obs >= TTC_THRESHOLD && distance_ego_obs <= SAFE_DISTANCE &&
                        obstacle.obstacle_class != STRUCTURE)
                    {
                        adcm::Log::Info() << "시나리오3-i) 10~30m 사이의 동적객체 추출: " << obstacle.obstacle_id;
                        obstacle_near_10_30.push_back(obstacle);
                    }
                }

                //=========ii) TTC와 5초 후 안전영역 진입 여부 확인 후 컨피던스 계산
                for (const auto &obstacle : obstacle_near_10_30)
                {
                    double ttc = getTTC(obstacle, ego_vehicle);
                    double dist_ego_obs_linear_approx = getLinearApprox(obstacle, ego_vehicle);

                    if (ttc == INVALID_RETURN_VALUE || dist_ego_obs_linear_approx == 0)
                    {
                        continue; // 유효하지 않은 경우 건너뜀
                    }

                    double ttc_confidence = (ttc > 0) ? (5 / ttc * 0.7) : 0.0;
                    double area_confidence = (dist_ego_obs_linear_approx > 0)
                                                 ? (SAFE_DISTANCE / dist_ego_obs_linear_approx * 0.7)
                                                 : 0.0;

                    if (ttc_confidence > 0 || area_confidence > 0)
                    {
                        double confidence_scenario_3 = std::max(ttc_confidence, area_confidence);

                        adcm::riskAssessmentStruct riskAssessment3{
                            .obstacle_id = obstacle.obstacle_id,
                            .hazard_class = SCENARIO_3,
                            .confidence = confidence_scenario_3};

                        adcm::Log::Info() << "Risk assessment generated for #3: "
                                          << obstacle.obstacle_id
                                          << " with confidence: " << confidence_scenario_3;

                        riskAssessment.riskAssessmentList.push_back(riskAssessment3);
                    }
                }
                adcm::Log::Info() << "scenario 3 DONE";
                clear_and_free_memory(obstacle_near_10_30);
            }

            {
                //=====시나리오 #4. 작업 중 경로 주변 동적 장애물 통행 환경=====
                adcm::Log::Info() << "=============KATECH: scenario 4 START==============";
                obstacleListVector obstacle_near_20_40;
                double confidence_scenario_4;
                // double dist_ego_obs_linear_approx = INVALID_RETURN_VALUE;
                double dist_ego_obs_linear_approx;

                //=========i) 20~40m 내 동적객체 추출
                for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
                {
                    double distance_ego_obs = getDistance(*iter, ego_vehicle);
                    if (distance_ego_obs > 200 && distance_ego_obs < 400 && iter->obstacle_class != STRUCTURE)
                    {
                        adcm::Log::Info() << "시나리오4-i) 20~40m 사이의 동적객체 추출: " << iter->obstacle_id;
                        obstacle_near_20_40.push_back(*iter);
                    }
                }
                //=========ii) 5초후 경로가 40m 이내인 경우 안전영역 진입했음으로 컨피던스값 계산해 넣어줌

                for (auto iter = obstacle_near_20_40.begin(); iter != obstacle_near_20_40.end(); iter++)
                {
                    dist_ego_obs_linear_approx = getLinearApprox(*iter, ego_vehicle);
                    adcm::Log::Info() << "시나리오4-ii) 5초 이내 최소 거리값: " << dist_ego_obs_linear_approx;

                    //                if (dist_ego_obs_linear_approx < 40)
                    //                {
                    confidence_scenario_4 = 400 / dist_ego_obs_linear_approx * 0.7;
                    adcm::riskAssessmentStruct riskAssessment4;
                    riskAssessment4.obstacle_id = iter->obstacle_id;
                    riskAssessment4.hazard_class = SCENARIO_4;
                    riskAssessment4.confidence = confidence_scenario_4;
                    riskAssessment.riskAssessmentList.push_back(riskAssessment4);
                    adcm::Log::Info() << "Risk assessment generated for #4: " << iter->obstacle_id << "with confidence:  " << confidence_scenario_4;
                    adcm::Log::Info() << "시나리오4-iii) 최소 거리값이 40 이내이므로 시나리오 4 해당!! " << iter->obstacle_id << " , " << confidence_scenario_4;
                    //                }
                }
                adcm::Log::Info() << "scenario 4 DONE";
                clear_and_free_memory(obstacle_near_20_40);
            }

            {
                //=====시나리오 #5. 주행 경로상 장애물 통행량이 과다한 환경(사람)=====
                adcm::Log::Info() << "=============KATECH: scenario 5 START==============";

                obstacleListVector obstacle_pedes_40_50;
                obstacleListVector obstacle_pedes_repeated;
                obstacleListVector obstacle_pedes_new;
                uint64_t ori_timestamp_max = 0, new_timestamp_min = INVALID_RETURN_VALUE, timestamp_diff;
                double distance, max_distance = 0, confidence_scenario_5;

                //=========i) 40 < pedestrian < 50 인 보행자만 추출
                for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
                {
                    double distance_ego_obs = getDistance(*iter, ego_vehicle);
                    // distance_ego_obs > 40
                    if (distance_ego_obs < 500 && iter->obstacle_class == PEDESTRIAN)
                    {
                        adcm::Log::Info() << "시나리오5-i) 40~50m 사이의 보행자 추출: " << iter->obstacle_id;
                        obstacle_pedes_40_50.push_back(*iter);
                    }
                }
                //=========ii) 주행경로 반경 10m 이내 필터링
                obstacle_pedes_40_50.erase(
                    std::remove_if(obstacle_pedes_40_50.begin(), obstacle_pedes_40_50.end(), [&](const auto &iter)
                                   {
                        double distance = getDistance_LinearTrajectory(iter);
                        if (distance > 300 || distance == INVALID_RETURN_VALUE) {
                            adcm::Log::Info() << "시나리오5-ii) 주행경로 반경 10m 초과하므로 해당 장애물 삭제: " << iter.obstacle_id;
                            return true; // 삭제 조건
                        }
                        return false; }),
                    obstacle_pedes_40_50.end());
                if (obstacle_pedes_initial.empty())
                { // 최초 리스트 생성시 -> 여기서 해당시나리오 종료
                    adcm::Log::Info() << "시나리오5 최초 실행하므로 장애물 리스트만 생성하고 다음 loop 에 분석을 이어서 한다";
                    obstacle_pedes_initial.assign(obstacle_pedes_40_50.begin(), obstacle_pedes_40_50.end());
                }
                else
                {
                    // 해당 시나리오 계속 체크
                    adcm::Log::Info() << "시나리오5 n 번째 실행중... 위험 분석 진행";
                    //=========iii) 신규 객체 생성 빈도 측정

                    int n = obstacle_pedes_initial.size();
                    int m = obstacle_pedes_40_50.size();
                    adcm::Log::Info() << "size n" << n;
                    adcm::Log::Info() << "size m" << m;

                    symmDiff(obstacle_pedes_initial, obstacle_pedes_40_50, obstacle_pedes_new, n, m);

                    if (obstacle_pedes_new.empty())
                    {
                        adcm::Log::Info() << "새로운 객체 등장X. 시나리오 5 종료";
                    }
                    else
                    {
                        adcm::Log::Info() << "새로운 객체 등장. New obstacle detected! 시나리오 5 계속";
                        // 기존 보행자 타임스탬프 갱신
                        for (const auto &iter : obstacle_pedes_initial)
                            ori_timestamp_max = std::max(ori_timestamp_max, iter.timestamp);

                        // 신규 보행자 타임스탬프 갱신
                        for (const auto &iter : obstacle_pedes_new)
                            new_timestamp_min = std::min(new_timestamp_min, iter.timestamp);

                        timestamp_diff = new_timestamp_min - ori_timestamp_max;

                        // 현재 시뮬레이션에서는 timetampe 1 이 10ms 이므로 이므로 10s=10000ms=>1000 값 이내면 신규객체 등장으로 인정
                        //=========iv) 객체 출현시간 10s 이내일때 객체간 최대거리 계산
                        if (timestamp_diff < 2000)
                        {
                            // adcm::Log::Info() << "10초 이내 신규객체 발견";
                            for (const auto &iter : obstacle_pedes_initial)
                            {
                                for (const auto &iter1 : obstacle_pedes_new)
                                {
                                    distance = getDistance(iter, iter1);
                                    max_distance = std::max(max_distance, distance);
                                }
                            }
                            adcm::Log::Info() << "객체간 최대거리: " << max_distance;

                            //=========iv) 객체 출현시간 10s 이내일때 객체간 최대거리 계산
                            // 최대 거리 < 400m 인 경우만 통행과다환경으로 판단
                            if (max_distance < 400)
                            {
                                // 30m 내외로 통행과다환경 판단
                                for (const auto &iter : obstacle_pedes_new)
                                {
                                    distance = getDistance(iter, ego_vehicle);
                                    adcm::Log::Info() << "ego 와의 거리: " << distance;

                                    confidence_scenario_5 = 300 / distance * 0.7;
                                    adcm::riskAssessmentStruct riskAssessment5;
                                    riskAssessment5.obstacle_id = iter.obstacle_id;
                                    riskAssessment5.hazard_class = SCENARIO_5;
                                    riskAssessment5.confidence = confidence_scenario_5;
                                    riskAssessment.riskAssessmentList.push_back(riskAssessment5);

                                    adcm::Log::Info() << "Risk assessment generated for #5: " << iter.obstacle_id << " with confidence: " << confidence_scenario_5;
                                }
                            }
                        }
                    }
                }

                adcm::Log::Info() << "scenario 5 DONE";

                // 메모리 해제
                clear_and_free_memory(obstacle_pedes_40_50);
                clear_and_free_memory(obstacle_pedes_new);
            }

            {
                //=====시나리오 #6. 주행 경로상 통행량이 과다한 환경(차량)=====
                adcm::Log::Info() << "=============KATECH: scenario 6 START==============";

                obstacleListVector obstacle_vehicle_50_60;
                obstacleListVector obstacle_vehicle_new;
                uint64_t ori_timestamp_max = 0, new_timestamp_min = INVALID_RETURN_VALUE, timestamp_diff;
                double distance, max_distance = 0, confidence_scenario_6;

                //=========i) 50~60m 사이의 차량 추출
                for (const auto &iter : obstacle_list)
                {
                    double distance_ego_obs = getDistance(iter, ego_vehicle);
                    if (distance_ego_obs > 500 && distance_ego_obs < 600 &&
                        (iter.obstacle_class == VEHICLE_LARGE || iter.obstacle_class == VEHICLE_SMALL))
                    {
                        adcm::Log::Info() << "시나리오6-i) 50~60m 사이의 차량 추출: " << iter.obstacle_id;
                        obstacle_vehicle_50_60.push_back(iter);
                    }
                }

                //=========ii) 주행경로 반경 15m 이내인지 계산
                obstacle_vehicle_50_60.erase(
                    std::remove_if(obstacle_vehicle_50_60.begin(), obstacle_vehicle_50_60.end(),
                                   [&](const auto &iter)
                                   {
                                       distance = getDistance_LinearTrajectory(iter);
                                       if (distance > 150 || distance == INVALID_RETURN_VALUE)
                                       {
                                           adcm::Log::Info() << "시나리오6-ii) 주행경로 반경 15m 초과 장애물 삭제: " << iter.obstacle_id;
                                           return true;
                                       }
                                       return false;
                                   }),
                    obstacle_vehicle_50_60.end());

                // 최초 실행시 장애물 리스트만 생성
                if (obstacle_vehicle_initial.empty())
                {
                    adcm::Log::Info() << "시나리오6 최초 실행하므로 장애물 리스트만 생성하고 다음 loop 에 분석을 이어서 한다";
                    obstacle_vehicle_initial.assign(obstacle_vehicle_50_60.begin(), obstacle_vehicle_50_60.end());
                }
                else
                {
                    // 신규 객체 추출 (set을 사용하여 중복 제거)
                    std::unordered_set<int> initial_obstacle_ids;
                    for (const auto &iter : obstacle_vehicle_initial)
                        initial_obstacle_ids.insert(iter.obstacle_id);

                    for (const auto &iter : obstacle_vehicle_50_60)
                    {
                        if (initial_obstacle_ids.find(iter.obstacle_id) == initial_obstacle_ids.end())
                            obstacle_vehicle_new.push_back(iter);
                    }

                    if (obstacle_vehicle_new.empty())
                    {
                        adcm::Log::Info() << "NO new obstacle. #6 terminates here";
                    }
                    else
                    {
                        // 타임스탬프 최대값 및 최소값 계산
                        for (const auto &iter : obstacle_vehicle_initial)
                        {
                            ori_timestamp_max = std::max(ori_timestamp_max, iter.timestamp);
                            adcm::Log::Info() << "ori obstacle : " << iter.obstacle_id;
                        }

                        for (const auto &iter : obstacle_vehicle_new)
                        {
                            new_timestamp_min = std::min(new_timestamp_min, iter.timestamp);
                            adcm::Log::Info() << "new obstacle : " << iter.obstacle_id;
                        }

                        timestamp_diff = new_timestamp_min - ori_timestamp_max;

                        // 객체 출현시간 10초 이내일 때 객체 간 최대 거리 계산
                        if (timestamp_diff < 1500)
                        {
                            for (const auto &iter : obstacle_vehicle_initial)
                            {
                                for (const auto &iter1 : obstacle_vehicle_new)
                                {
                                    distance = getDistance(iter, iter1);
                                    max_distance = std::max(max_distance, distance);
                                }
                            }

                            adcm::Log::Info() << "객체간 최대거리: " << max_distance;

                            if (max_distance < 600)
                            {
                                // 통행과다환경 지정하고 환경 내 ego와 40m 내외인 장애물에 대해 confidence 계산
                                for (const auto &iter : obstacle_vehicle_new)
                                {
                                    distance = getDistance(iter, ego_vehicle);
                                    if (distance < 40)
                                    {
                                        confidence_scenario_6 = 400 / distance * 0.7;
                                        adcm::riskAssessmentStruct riskAssessment6;
                                        riskAssessment6.obstacle_id = iter.obstacle_id;
                                        riskAssessment6.hazard_class = SCENARIO_6;
                                        riskAssessment6.confidence = confidence_scenario_6;
                                        riskAssessment.riskAssessmentList.push_back(riskAssessment6);
                                        adcm::Log::Info() << "Risk assessment generated for #6: " << iter.obstacle_id << " with confidence: " << confidence_scenario_6;
                                    }
                                }
                            }
                        }
                    }
                }

                adcm::Log::Info() << "scenario 6 DONE";

                // 벡터 메모리 해제
                clear_and_free_memory(obstacle_vehicle_50_60);
                clear_and_free_memory(obstacle_vehicle_new);
            }

            {
                if (riskAssessment.riskAssessmentList.size() == 0)
                {
                    adcm::Log::Info() << "no riskAssessment data before scenario 7!";
                }
                //=====시나리오 #7. 미개척 지역 주행환경======
                adcm::Log::Info() << "=============KATECH: scenario 7 START==============";
                adcm::riskAssessmentStruct riskAssessment7;

                if (position_x.size() != 0 && map_2d.size() != 0)
                {
                    detectUnscannedPath(riskAssessment);
                }
                adcm::Log::Info() << "scenario 7 DONE";
            }

            {
                if (riskAssessment.riskAssessmentList.size() == 0)
                {
                    adcm::Log::Info() << "no riskAssessment data before scenario 8!";
                }
                //=====시나리오 #8. 주행가능 영역 너비/높이 변화: 전역경로 및 작업지점 근방의 노면 상태가 불균일 할 경우======
                adcm::Log::Info() << "=============KATECH: scenario 8 START==============";
                adcm::riskAssessmentStruct riskAssessment8;
                int shift = 50; // 전역경로와 작업지점의 근방 5m 스캔
                double original_m, original_c, up_c, down_c;
                double x_up, x_down;
                bool isVertical;
                double vehicle_road_z = 0; // TO DO: 특장차가 위치한 시작 노면정보를 읽어 오도록 수정 필요
                int risk_count = 0;
                Point2D p1, p2, p3, p4;

                if (position_x.size() != 0 && map_2d.size() != 0)
                {
                    for (int count = 0; count < position_x.size() - 1; count++)
                    {
                        int x_start = floor(position_x[count]);
                        int x_end = floor(position_x[count + 1]);
                        int y_start = floor(position_y[count]);
                        int y_end = floor(position_y[count + 1]);

                        calculateShiftedLines(x_start, x_end, y_start, y_end, shift, original_m, original_c, up_c, down_c, isVertical, x_up, x_down);
                        // Print the line equations
                        if (isVertical)
                        {
                            adcm::Log::Info() << "Original line equation: x = " << x_start << "\n";
                            adcm::Log::Info() << "Line shifted right: x = " << x_up << "\n";
                            adcm::Log::Info() << "Line shifted left: x = " << x_down << "\n";

                            p1 = {x_down, y_start}; // First line start point
                            p2 = {x_down, y_end};   // First line end point
                            p3 = {x_up, y_start};   // Second line start point
                            p4 = {x_up, y_end};     // Second line end point
                        }
                        else
                        {
                            adcm::Log::Info() << "Original line equation: y = " << original_m << "x + " << original_c << "\n";
                            adcm::Log::Info() << "Line shifted up: y = " << original_m << "x + " << up_c << "\n";
                            adcm::Log::Info() << "Line shifted down: y = " << original_m << "x + " << down_c << "\n";

                            // Points of the parallelogram (calculated for x = 0 and x = 10)
                            p1 = {x_start, original_m * x_start + up_c};   // First line start point
                            p2 = {x_end, original_m * x_end + up_c};       // First line end point
                            p3 = {x_start, original_m * x_start + down_c}; // Second line start point
                            p4 = {x_end, original_m * x_end + down_c};     // Second line end point
                        }

                        int minX = std::min({p1.x, p2.x, p3.x, p4.x});
                        int maxX = std::max({p1.x, p2.x, p3.x, p4.x});
                        int minY = std::min({p1.y, p2.y, p3.y, p4.y});
                        int maxY = std::max({p1.y, p2.y, p3.y, p4.y});

                        for (int x = minX; x <= maxX; ++x)
                        {
                            for (int y = minY; y <= maxY; ++y)
                            {
                                if (std::abs(map_2d[x][y].road_z - vehicle_road_z) > WHEEL_DIAMETER_M / 2)
                                {
                                    // TO DO: road_z 값이 없는 경우도 고려해야 함 => 무시할 것인가 COUNT 를 올릴 것인가?
                                    risk_count++;
                                }

                                if (risk_count == 20)
                                {
                                    adcm::Log::Info() << "Road cave-in detected!";
                                    adcm::riskAssessmentStruct riskAssessment8;
                                    riskAssessment8.wgs84_xy_start.clear();
                                    riskAssessment8.wgs84_xy_end.clear();
                                    adcm::globalPathPosition uneven_start_path, uneven_end_path;
                                    uneven_start_path.x = position_x[count];
                                    uneven_start_path.y = position_y[count];
                                    uneven_end_path.x = position_x[count + 1];
                                    uneven_end_path.y = position_y[count + 1];
                                    riskAssessment8.wgs84_xy_start.push_back(uneven_start_path);
                                    riskAssessment8.wgs84_xy_end.push_back(uneven_end_path);
                                    riskAssessment8.hazard_class = SCENARIO_8;
                                    riskAssessment8.isHarzard = true;
                                    adcm::Log::Info() << "Risk assessment generated for #8 is X: " << position_x[count] << " Y: " << position_y[count] << " with flag 1 ";
                                    riskAssessment.riskAssessmentList.push_back(riskAssessment8);
                                    break; // y 루프 종료
                                }
                            }

                            if (risk_count == 20)
                            {
                                risk_count = 0;
                                break; // x 루프도 종료
                            }
                        }
                    }
                    adcm::Log::Info() << "scenario 8 DONE";
                }
            }
            adcm::Log::Info() << "build riskAssessment data - size " << riskAssessment.riskAssessmentList.size();
            adcm::Log::Info() << "========================obstacle and vehicle info===========================";
            for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
            {
                adcm::Log::Info() << "obstacle ID:" << iter->obstacle_id << " obstacle XY position: " << iter->fused_position_x << "," << iter->fused_position_y << " obstacle XY velocity: " << iter->fused_velocity_x << "," << iter->fused_velocity_y;
            }

            adcm::Log::Info() << "ego-vehicle XY position: " << ego_vehicle.position_x << "," << ego_vehicle.position_y << " ego-vehicle XY velocity: " << ego_vehicle.velocity_x << "," << ego_vehicle.velocity_y;
            adcm::Log::Info() << "sub-vehicle-1 XY position: " << sub_vehicle_1.position_x << "," << sub_vehicle_1.position_y << " sub-vehicle_1 XY velocity: " << sub_vehicle_1.velocity_x << "," << sub_vehicle_1.velocity_y;
            adcm::Log::Info() << "sub-vehicle-2 XY position: " << sub_vehicle_2.position_x << "," << sub_vehicle_2.position_y << " sub-vehicle_2 XY velocity: " << sub_vehicle_2.velocity_x << "," << sub_vehicle_2.velocity_y;
            ;
            adcm::Log::Info() << "========================RiskAssessment Output===========================";
            for (auto iter = riskAssessment.riskAssessmentList.begin(); iter < riskAssessment.riskAssessmentList.end(); iter++)
            {
                if (iter->hazard_class != SCENARIO_7)
                {
                    adcm::Log::Info() << "riskAssessment - obstacle id:" << iter->obstacle_id << " hazard class:" << iter->hazard_class << " confidence:" << iter->confidence;
                }
                else
                {
                    adcm::Log::Info() << "riskAssessment - unscanned utm XY value:" << iter->wgs84_xy_start[0].x << "," << iter->wgs84_xy_start[0].y << " hazard class:" << iter->hazard_class << " isHazard:" << iter->isHarzard;
                }
            }
            adcm::Log::Info() << "===============================================================";
            if (riskAssessment.riskAssessmentList.size() != 0)
            {
                // risktAssessment object 의 생성시간 추가
                auto now = std::chrono::system_clock::now();
                auto riskAssessment_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
                adcm::Log::Info() << "Current timestamp in milliseconds: " << riskAssessment_timestamp;
                riskAssessment.timestamp = riskAssessment_timestamp;
                adcm::Log::Info() << "riskAssessment send!";
                riskAssessment_provider.send(riskAssessment);
#ifdef NATS
                NatsSend(riskAssessment);
#endif
            }
            else
            {
                // NatsSend(riskAssessment); //테스트 용도로 계속 송신하게 한다
                adcm::Log::Info() << "riskAssessment size is 0, do NOT send";
            }
            riskAssessment.riskAssessmentList.clear();
        }
    }
}

void ThreadMonitor()
{
    while (continueExecution)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        if (gMainthread_Loopcount == 0)
        {
            adcm::Log::Error() << "Main thread Timeout!!!";
        }
        else
        {
            gMainthread_Loopcount = 0;

            if (gReceivedEvent_count_map_data != 0)
            {
                adcm::Log::Info() << "map_data Received count = " << gReceivedEvent_count_map_data;
                gReceivedEvent_count_map_data = 0;
            }
            else
            {
                adcm::Log::Info() << "map_data event timeout!!!";
            }

            if (gReceivedEvent_count_build_path != 0)
            {
                adcm::Log::Info() << "build_path Received count = " << gReceivedEvent_count_build_path;
                gReceivedEvent_count_build_path = 0;
            }
            else
            {
                adcm::Log::Info() << "build_path event timeout!!!";
            }

            if (gReceivedEvent_count_build_path_test != 0)
            {
                adcm::Log::Info() << "build_path_test Received count = " << gReceivedEvent_count_build_path_test;
                gReceivedEvent_count_build_path_test = 0;
            }
            else
            {
                adcm::Log::Info() << "build_path_test event timeout!!!";
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
    adcm::Log::Info() << "RiskAssessment: configure e2e protection";
    bool success = ara::com::e2exf::StatusHandler::Configure("./etc/e2e_dataid_mapping.json",
                                                             ara::com::e2exf::ConfigurationFormat::JSON,
                                                             "./etc/e2e_statemachines.json",
                                                             ara::com::e2exf::ConfigurationFormat::JSON);
    adcm::Log::Info() << "RiskAssessment: e2e configuration " << (success ? "succeeded" : "failed");
#endif
    adcm::Log::Info() << "Ok, let's produce some RiskAssessment data...";
    adcm::Log::Info() << "SDK release_250314_interface v2.1";
    adcm::Log::Info() << "RiskAssessment Build " << BUILD_TIMESTAMP;
#ifdef NATS
    // Code to execute if NATS is defined
    adcm::Log::Info() << "NATS ON";
#else
    // Code to execute if NATS is not defined
    adcm::Log::Info() << "NATS OFF";
#endif
    thread_list.push_back(std::thread(ThreadReceiveMapData));
    thread_list.push_back(std::thread(ThreadReceiveBuildPathTest));
    thread_list.push_back(std::thread(ThreadReceiveBuildPath));
    thread_list.push_back(std::thread(ThreadMonitor));
    thread_list.push_back(std::thread(ThreadKatech));

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
