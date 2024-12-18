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

#include <array>
#include <vector>
#include <algorithm>

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
#include <queue>
#include <condition_variable>

#define MAX_BATCH_SIZE 5
#define DEMO

std::vector<adcm::map_2dListVector> map_2d;

obstacleListVector obstacle_list_temp;
adcm::vehicleListStruct ego_vehicle_temp, sub_vehicle_1_temp, sub_vehicle_2_temp, sub_vehicle_3_temp, sub_vehicle_4_temp;
doubleVector utm_x;
doubleVector utm_y;
obstacleListVector obstacle_pedes_initial;   // 시나리오 5 용
obstacleListVector obstacle_vehicle_initial; // 시나리오 6 용

//============== 2. 함수 definition =================
#define NATS_TEST
#ifdef NATS_TEST
#include "./NATS_IMPLEMENTATION/NatsConnManager.h"
#define HMI_SERVER_URL  "https://nats.beyless.com"
#include <sstream>
#include <fstream>  // Required for file handling
#include <filesystem>

std::mutex mtx; // 뮤텍스 선언
bool firstTime = true;
natsStatus s = NATS_OK;
std::vector<const char*> subject = {"test1.*", "test2.*"};
std::shared_ptr<adcm::etc::NatsConnManager> natsManager;

void asyncCb(natsConnection* nc, natsSubscription* sub, natsStatus err, void* closure)
{
    std::cout << "Async error: " << err << " - " << natsStatus_GetText(err) << std::endl;
    natsManager->NatsSubscriptionGetDropped(sub, (int64_t*) &natsManager->dropped);
}

void onMsg(natsConnection* nc, natsSubscription* sub, natsMsg* msg, void* closure)
{
    const char* subject = NULL;
    
    // 뮤텍스를 사용하여 공유 변수 접근 보호
    std::lock_guard<std::mutex> lock(mtx);
    subject = natsMsg_GetSubject(msg);

    std::cout << "Received msg: [" << subject << " : " << natsMsg_GetDataLength(msg) << "]" << natsMsg_GetData(msg) << std::endl;
    natsManager->NatsMsgDestroy(msg);
}

void saveToJsonFile(const std::string& key, const std::string& value, int& fileCount)
{
    std::ostringstream fileNameStream;
    fileNameStream << key <<"_" << fileCount << ".json";  // Construct the file name
    std::string fileName = fileNameStream.str();
    std::ofstream outFile(fileName);
    // Open file for writing with the generated name
    if (outFile.is_open())
    {
        outFile << value;  // Write the JSON string to the file
        outFile.close();          // Close the file after writing
        adcm::Log::Info() << "JSON data stored in " << fileName;
    }
    else
    {
        adcm::Log::Error() << "Failed to open file " << fileName << " for writing!";
    }

    // Increment the file count for the next call
    ++fileCount;
}
std::string convertRiskAssessmentToJsonString(const adcm::risk_assessment_Objects& riskAssessment) {
    //test purpose

    uint64_t timestamp_map = 111;
    uint64_t timestamp_risk = riskAssessment.timestamp;
    std::string model_id = "test_id_v1";

    std::ostringstream oss;  // Use a string stream for easier manipulation
    oss << "{\n";  // Start the JSON object

    // Convert riskAssessmentList to JSON
    oss << "    \"riskAssessmentList\": [\n";
    for (size_t i = 0; i < riskAssessment.riskAssessmentList.size(); ++i) {
        const auto& item = riskAssessment.riskAssessmentList[i];
        oss << "        {\n"
            << "            \"obstacle_id\": " << item.obstacle_id << ",\n"
            << "            \"wgs84_xy_start\": [\n";

        // Convert wgs84_xy_start to JSON
        for (size_t j = 0; j < item.wgs84_xy_start.size(); ++j) {
            const auto& start = item.wgs84_xy_start[j];
            oss << "                {\n"
                << "                    \"x\": " << start.x << ",\n"
                << "                    \"y\": " << start.y << "\n"
                << "                }";

            if (j < item.wgs84_xy_start.size() - 1) {
                oss << ",";
            }
            oss << "\n";  // Newline for readability
        }

        oss << "            ],\n"  // Close wgs84_xy_start array
            << "            \"wgs84_xy_end\": [\n";

        // Convert wgs84_xy_end to JSON
        for (size_t j = 0; j < item.wgs84_xy_end.size(); ++j) {
            const auto& end = item.wgs84_xy_end[j];
            oss << "                {\n"
                << "                    \"x\": " << end.x << ",\n"
                << "                    \"y\": " << end.y << "\n"
                << "                }";

            if (j < item.wgs84_xy_end.size() - 1) {
                oss << ",";
            }
            oss << "\n";  // Newline for readability
        }

        oss << "            ],\n"  // Close wgs84_xy_end array
            << "            \"hazard_class\": " << static_cast<int>(item.hazard_class) << ",\n"  // hazard_class
            << "            \"isHarzard\": " << (item.isHarzard ? "true" : "false") << ",\n"  // isHazard
            << "            \"confidence\": " << item.confidence << ",\n"  // confidence
            << "            \"timestamp_map\": " << timestamp_map << ",\n"  // timestamp_map
            << "            \"timestamp_risk\": " << timestamp_risk << ",\n"  // timestamp_risk
            << "            \"model_id\": \"" << model_id << "\"\n";  // model_id

        oss << "        }";

        if (i < riskAssessment.riskAssessmentList.size() - 1) {
            oss << ",";
        }
        oss << "\n";  // Newline for readability
    }
    oss << "    ]\n";  // Close riskAssessmentList array
    oss << "}";  // Close the main JSON object

    return oss.str();  // Return the constructed JSON string
}

void NatsSend(const adcm::risk_assessment_Objects& riskAssessment)
{
    static int risk_count = 0;  // Static variable to track file number

    if (firstTime == true)
    {
        adcm::Log::Info() << "NATS first time setup!";
        natsManager = std::make_shared<adcm::etc::NatsConnManager>(HMI_SERVER_URL, subject, onMsg, asyncCb, adcm::etc::NatsConnManager::Mode::Default);
        s = natsManager->NatsExecute();
        firstTime = false;
    }
    if(s == NATS_OK)
    {
        const char* pubSubject = "test2.JSON";
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
        try{
        natsManager = std::make_shared<adcm::etc::NatsConnManager>(HMI_SERVER_URL,subject, onMsg, asyncCb, adcm::etc::NatsConnManager::Mode::Default);
        s = natsManager->NatsExecute();
        }catch(std::exception e)
        {
            std::cout << "Nats reConnection error" << std::endl;
        }
    }  
}
#endif
void symmDiff(obstacleListVector vec1, obstacleListVector vec2, obstacleListVector &output, int n, int m)
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
void globalToLocalcoordinate(doubleVector &utm_x, doubleVector &utm_y)
{
    // 시뮬레이션의 global 좌표계를 작업환경 XY 기반의 local 좌표계로 변환하는 함수
    double alpha = 537.92;
    double beta = -416.58;
    double theta = MAP_TILT_ANGLE * M_PI / 180;

    for (int count = 0; count < utm_x.size(); count++)
    {
        double old_x = utm_x[count];
        double old_y = utm_y[count];

        adcm::Log::Info() << "utm globalToLocalcoordinate 좌표변환 before :" << utm_x[count] << " , " << utm_y[count];
        utm_x[count] = (cos(theta) * (old_x - alpha) + sin(theta) * (old_y - beta)) * M_TO_10CM_PRECISION;
        utm_y[count] = (-sin(theta) * (old_x - alpha) + cos(theta) * (old_y - beta)) * M_TO_10CM_PRECISION;

        adcm::Log::Info() << "utm globalToLocalcoordinate 좌표변환 after :" << utm_x[count] << " , " << utm_y[count];
    }
}
double getMagnitude(Point2D a)
{
    return sqrt(a.x ^ 2 + a.y ^ 2);
}

double getDistance(adcm::obstacleListStruct obstacle, adcm::vehicleListStruct vehicle)
{
    return sqrt(pow(obstacle.fused_position_x - vehicle.position_x, 2) + pow(obstacle.fused_position_y - vehicle.position_y, 2));
}

double getDistance(adcm::obstacleListStruct obstacle1, adcm::obstacleListStruct obstacle2)
{
    return sqrt(pow(obstacle1.fused_position_x - obstacle2.fused_position_x, 2) + pow(obstacle1.fused_position_y - obstacle2.fused_position_y, 2));
}

float getDistance_LinearTrajectory(adcm::obstacleListStruct obstacle, doubleVector utm_x, doubleVector utm_y)
{
    float distance = 0;
    for (int count = 0; count < utm_x.size(); count++)
    {
        float x_start = utm_x[count];
        float x_end = utm_x[count + 1];
        float y_start = utm_y[count];
        float y_end = utm_y[count + 1];

        Point2D start_to_end_vector;
        start_to_end_vector.x = x_end - x_start;
        start_to_end_vector.y = y_end - y_start;

        Point2D start_to_obs_vector;
        start_to_obs_vector.x = obstacle.fused_position_x - x_start;
        start_to_obs_vector.y = obstacle.fused_position_y - y_start;

        double angle = atan2(start_to_end_vector.y, start_to_end_vector.x) - atan2(start_to_obs_vector.y, start_to_obs_vector.x);
        double angle_to_degree = angle * 180 / M_PI;
        // adcm::Log::Info() << " 장애물과 경로사이 각도" << angle_to_degree;

        double start_to_end_vector_DOT_start_to_obs_vector = cos(angle) * getMagnitude(start_to_end_vector) * getMagnitude(start_to_obs_vector);
        if (start_to_end_vector_DOT_start_to_obs_vector > 0 && start_to_end_vector_DOT_start_to_obs_vector < pow(getMagnitude(start_to_end_vector), 2))
        // 장애물 위치가 시작점과 끝점 사이일때
        // 장애물과 전역경로간의 거리를 구한다
        {
            distance = getMagnitude(start_to_obs_vector) * sqrt(1 - pow(start_to_end_vector_DOT_start_to_obs_vector / (getMagnitude(start_to_end_vector) * getMagnitude(start_to_obs_vector)), 2));
            adcm::Log::Info() << "condition met! 장애물과 경로간 거리는 " << distance;
            break;
            // for loop 에서 나가서 distance 를 return
        }
        else
        {
            distance = INVALID_RETURN_VALUE;
            // 계속 for loop 반복
        }
    }
    return distance;
}
float getLinearApprox(::adcm::obstacleListStruct obstacle, ::adcm::vehicleListStruct vehicle)
{
    // 5초 동안의 장애물과 차량의 거리중 가장 짧은 거리를 return
    float min_distance_ego_obs = INVALID_RETURN_VALUE; // random big number
    for (float k = 0; k < 5.1; k = k + 0.5)
    {
        vehicle.position_x = vehicle.position_x + vehicle.velocity_x * k;
        vehicle.position_y = vehicle.position_y + vehicle.velocity_y * k;
        obstacle.fused_position_x = obstacle.fused_position_x + obstacle.fused_velocity_x * k;
        obstacle.fused_position_y = obstacle.fused_position_y + obstacle.fused_velocity_y * k;

        float temp = getDistance(obstacle, vehicle);
        if (min_distance_ego_obs > temp)
        {
            min_distance_ego_obs = temp;
        }
    }
    return min_distance_ego_obs;
}
float getTTC(::adcm::obstacleListStruct obstacle, ::adcm::vehicleListStruct vehicle)
{
    float c, ttc;
    ::adcm::obstacleListStruct obstacle_relative_to_vehicle;
    obstacle_relative_to_vehicle.fused_position_x = obstacle.fused_position_x - vehicle.position_x;
    obstacle_relative_to_vehicle.fused_position_y = obstacle.fused_position_y - vehicle.position_y;
    obstacle_relative_to_vehicle.fused_velocity_x = obstacle.fused_velocity_x - vehicle.velocity_x;
    obstacle_relative_to_vehicle.fused_velocity_y = obstacle.fused_velocity_y - vehicle.velocity_y;

    c = (obstacle_relative_to_vehicle.fused_velocity_x * obstacle_relative_to_vehicle.fused_position_y) - (obstacle_relative_to_vehicle.fused_velocity_y * obstacle_relative_to_vehicle.fused_position_x);
    ttc = ((c / (2 * obstacle_relative_to_vehicle.fused_velocity_y)) - obstacle_relative_to_vehicle.fused_position_x) / obstacle_relative_to_vehicle.fused_velocity_x;

    if (ttc > 0)
    {
        return ttc;
    }
    else
        return INVALID_RETURN_VALUE; // return random big number
}
void drawline(doubleVector utm_x, doubleVector utm_y, std::vector<adcm::map_2dListVector> &map_2d, adcm::risk_assessment_Objects &riskAssessment)
{
    bool breakFlag; // 지정된 전역경로 (x1,y1) 과 (x2, y2) 사이 하나라도
    for (int count = 0; count < utm_x.size() - 1; count++)
    {
        //        int i = 30;
        //        int j = 50;
        //        adcm::Log::Info() << "[" << i << "]" << "[" << j << "]" << "road_z : " << map_2d[i][j].road_z;
        //        adcm::Log::Info() << "drawline test 1: "<< utm_x.size();
        int x_start = floor(utm_x[count]);
        int x_end = floor(utm_x[count + 1]);
        int y_start = floor(utm_y[count]);
        int y_end = floor(utm_y[count + 1]);

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
                    unscanned_start_path.x = utm_x[count];
                    unscanned_start_path.y = utm_y[count];
                    unscanned_end_path.x = utm_x[count + 1];
                    unscanned_end_path.y = utm_y[count + 1];
                    riskAssessment7.wgs84_xy_start.push_back(unscanned_start_path);
                    riskAssessment7.wgs84_xy_end.push_back(unscanned_end_path);
                    riskAssessment7.hazard_class = SCENARIO_7;
                    riskAssessment7.isHarzard = true;
                    adcm::Log::Info() << "Risk assessment generated for #7 is X: " << utm_x[count] << " Y: " << utm_y[count] << " with flag 1 ";
                    riskAssessment.riskAssessmentList.push_back(riskAssessment7);
                    break; // 한번만 들어가도 for loop break
                }
                /*else if (map_2d[y][x].road_z == 1 )
                {
                    adcm::Log::Info() << "map_2d[" << y << "][" << x << "] is 1";
                    adcm::Log::Info() << "scanned! safe to go";
                }*/
            }

            else if (map_2d[x][y].road_z != 1) // 스캔되지 않은 map 의 index - 노면정보 X
            {
                adcm::riskAssessmentStruct riskAssessment7;
                // adcm::Log::Info() << "drawline test else";
                adcm::Log::Info() << "map_2d[" << x << "][" << y << "] is 0";
                riskAssessment7.wgs84_xy_start.clear();
                riskAssessment7.wgs84_xy_end.clear();
                adcm::globalPathPosition unscanned_start_path, unscanned_end_path;
                unscanned_start_path.x = utm_x[count];
                unscanned_start_path.y = utm_y[count];
                unscanned_end_path.x = utm_x[count + 1];
                unscanned_end_path.y = utm_y[count + 1];
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

namespace
{

    // Atomic flag for exit after SIGTERM caught
    std::atomic_bool continueExecution{true};
    std::atomic_uint gReceivedEvent_count_map_data{0};
    std::atomic_uint gReceivedEvent_count_build_path{0};
    std::atomic_uint gReceivedEvent_count_build_path_test{0};
    std::atomic_uint gMainthread_Loopcount{0};

    std::mutex riskAssDataQueueMutex;
    std::condition_variable riskAssDataQueueCondition;
    std::queue<adcm::risk_assessment_Objects> risk_assessmentDataQueue;

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

void GetRiskAssDataFromQueue(std::vector<adcm::risk_assessment_Objects> &dataBatch)
{
    std::unique_lock<std::mutex> lock(riskAssDataQueueMutex);

    riskAssDataQueueCondition.wait(lock, []
                                   { return !risk_assessmentDataQueue.empty(); });

    const size_t batchSize = std::min(risk_assessmentDataQueue.size(), static_cast<size_t>(MAX_BATCH_SIZE));

    for (size_t i = 0; i < batchSize; ++i)
    {
        dataBatch.push_back(risk_assessmentDataQueue.front());
        risk_assessmentDataQueue.pop();
    }
}
void ThreadAct1()
{
    adcm::Log::Info() << "SDK release_241008_interface v1.9";
    adcm::Log::Info() << "RiskAssessment ThreadAct1";
    INFO("RiskAssessment .init()");
    // adcm::RiskAssessment_Provider riskAssessment_provider;
    adcm::MapData_Subscriber mapData_subscriber;
    adcm::BuildPathTest_Subscriber buildPathTest_subscriber;
    adcm::BuildPath_Subscriber buildPath_subscriber;
    // riskAssessment_provider.init("RiskAssessment/RiskAssessment/PPort_risk_assessment");
    buildPath_subscriber.init("RiskAssessment/RiskAssessment/RPort_build_path");
    buildPathTest_subscriber.init("RiskAssessment/RiskAssessment/RPort_build_path_test");
    mapData_subscriber.init("RiskAssessment/RiskAssessment/RPort_map_data");
    INFO("After RiskAssessment .init()");
    INFO("Thread loop start...");

    //==========================변수생성=======================
    adcm::Log::Info() << "=============KATECH: variables created==============";
    /*adcm::map_2dListStruct map_2dStruct_init;

    map_2dStruct_init.obstacle_id = NO_OBSTACLE;
    map_2dStruct_init.road_z = 0;
    map_2dStruct_init.vehicle_class = NO_VEHICLE;
    std::vector<adcm::map_2dListVector> map_2d (map_n, adcm::map_2dListVector(map_m, map_2dStruct_init));
    adcm::Log::Info() << "mapData 2d info initialized";
*/
    obstacleListVector obstacle_list;
    vehicleListVector vehicle_list;

    while (continueExecution)
    {
        gMainthread_Loopcount++;
        VERBOSE("[RiskAssessment] Application loop");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool mapData_rxEvent = mapData_subscriber.waitEvent(0);             // wait event
        bool buildPathTest_rxEvent = buildPathTest_subscriber.waitEvent(0); // wait event
        bool buildPath_rxEvent = buildPath_subscriber.waitEvent(0);         // wait event

        if (mapData_rxEvent)
        {
            adcm::Log::Verbose() << "[EVENT] RiskAssessment Map Data received";
            adcm::Log::Info() << "=============KATECH: DAFU 값 가져오기==============";
            while (!mapData_subscriber.isEventQueueEmpty())
            {
                auto data = mapData_subscriber.getEvent();
                gReceivedEvent_count_map_data++;

                map_2d = data->map_2d;
                obstacle_list_temp = data->obstacle_list;
                auto vehicle_list = data->vehicle_list;
                adcm::Log::Info() << "size of map_2d received: " << map_2d.size() * map_m;
                adcm::Log::Info() << "size of obstacle list received: " << obstacle_list_temp.size();
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
                    }
                }
            }
        }
        if (buildPathTest_rxEvent)
        {
            adcm::Log::Info() << "Build Path Test 수신";
            adcm::Log::Verbose() << "[EVENT] RiskAssessment Build Path Test received";
            while (!buildPathTest_subscriber.isEventQueueEmpty())
            {
                auto data = buildPathTest_subscriber.getEvent();
                gReceivedEvent_count_build_path_test++;

                auto size = data->size;
                utm_x = data->utm_x;
                utm_y = data->utm_y;
                globalToLocalcoordinate(utm_x, utm_y);

                adcm::Log::Info() << "size : " << size;

                if (!utm_x.empty())
                {
                    adcm::Log::Info() << "=== utm_x Rx Text event===";
                    for (auto itr = utm_x.begin(); itr != utm_x.end(); ++itr)
                    {
                        adcm::Log::Info() << *itr;
                    }
                }
                else
                {
                    adcm::Log::Info() << "utm_x Vector empty!!! ";
                }

                if (!utm_y.empty())
                {
                    adcm::Log::Info() << "=== utm_y ===";
                    for (auto itr = utm_y.begin(); itr != utm_y.end(); ++itr)
                    {
                        adcm::Log::Info() << *itr;
                    }
                }
                else
                {
                    adcm::Log::Info() << "utm_y Vector empty!!! ";
                }
            }
        }

        if (buildPath_rxEvent)
        {
            // adcm::Log::Info() << "Build Path 수신";
            adcm::Log::Verbose() << "[EVENT] RiskAssessment Build Path received";
            // if (buildPath_subscriber.isEventQueueEmpty())
            // {
            //     adcm::Log::Info() << "Build Path 비어있음";
            //     buildPath_subscriber.deQueue();
            // }
            while (!buildPath_subscriber.isEventQueueEmpty())
            {
                auto data = buildPath_subscriber.getEvent();
            }
        }
    }
}

void ThreadKatech()
{
    adcm::RiskAssessment_Provider riskAssessment_provider;
    riskAssessment_provider.init("RiskAssessment/RiskAssessment/PPort_risk_assessment");

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
        adcm::Log::Info() << "map_2d size (katech):" << map_2d.size() * map_m;

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
                    float distance_ego_obs = getDistance(*iter, ego_vehicle);
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
                    distance_scenario_1 = getDistance_LinearTrajectory(*iter, utm_x, utm_y);
                    //                if (distance_scenario_1 < 10)
                    //                {
                    adcm::riskAssessmentStruct riskAssessment1;
                    confidence_scenario_1 = 200 / getDistance(*iter, ego_vehicle) * 0.7;
                    riskAssessment1.obstacle_id = iter->obstacle_id;
                    riskAssessment1.hazard_class = SCENARIO_1;
                    riskAssessment1.confidence = confidence_scenario_1;
                    adcm::Log::Info() << "Risk assessment generated for #1: " << iter->obstacle_id << "with confidence:  " << confidence_scenario_1;
                    riskAssessment.riskAssessmentList.push_back(riskAssessment1);
                    //                }
                }
                adcm::Log::Info() << "scenario 1 DONE";
                obstacleListVector().swap(obstacle_stop); // free memory
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
                    float distance_ego_obs = getDistance(*iter, ego_vehicle);
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
                    distance_scenario_2 = getDistance_LinearTrajectory(*iter, utm_x, utm_y);
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
                obstacleListVector().swap(obstacle_static_stop); // free memory
            }

            {
                //=====시나리오 #3. 주행중 경로 주변 동적 장애물 통행 환경 판단=====
                adcm::Log::Info() << "=============KATECH: scenario 3 START==============";
                obstacleListVector obstacle_near_10_30;
                double ttc;
                double dist_ego_obs_linear_approx;
                double ttc_confidence, area_confidence, confidence_scenario_3;
#define SAFE_DISTANCE 300
#define TTC_THRESHOLD 100

                //=========i) 특장차로부터의 거리 판정: 10<obs<30m 거리 이내인 경우
                for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
                {
                    double distance_ego_obs = getDistance(*iter, ego_vehicle);
                    if ((distance_ego_obs > TTC_THRESHOLD || distance_ego_obs == TTC_THRESHOLD) && (distance_ego_obs < SAFE_DISTANCE || distance_ego_obs == SAFE_DISTANCE) &&
                        iter->obstacle_class != STRUCTURE)
                    {
                        adcm::Log::Info() << "시나리오3-i) 10~30m 사이의 동적객체 추출: " << iter->obstacle_id;
                        obstacle_near_10_30.push_back(*iter);
                    }
                }

                //=========ii) TTC 와 5초후 안전영역 진입여부 확인 후 컨피던스 계산
                for (auto iter = obstacle_near_10_30.begin(); iter != obstacle_near_10_30.end(); iter++)
                {
                    ttc = getTTC(*iter, ego_vehicle);
                    dist_ego_obs_linear_approx = getLinearApprox(*iter, ego_vehicle);

                    //                if (ttc != INVALID_RETURN_VALUE && ttc < TTC_THRESHOLD)
                    if (ttc != INVALID_RETURN_VALUE)
                    {
                        // adcm::Log::Info() << "TTC 계산: " << ttc;
                        ttc_confidence = 5 / ttc * 0.7;
                    }
                    //                if (dist_ego_obs_linear_approx < SAFE_DISTANCE)
                    //                {
                    adcm::Log::Info() << "5초 후 경로: " << iter->obstacle_id << " , " << dist_ego_obs_linear_approx;
                    area_confidence = SAFE_DISTANCE / dist_ego_obs_linear_approx * 0.7;
                    //                }

                    if (ttc_confidence != 0 || area_confidence != 0)
                    {
                        if (ttc_confidence > area_confidence)
                        {
                            confidence_scenario_3 = ttc_confidence;
                        }
                        else
                            confidence_scenario_3 = area_confidence; // 큰 값을 취한다

                        adcm::riskAssessmentStruct riskAssessment3;
                        riskAssessment3.obstacle_id = iter->obstacle_id;
                        riskAssessment3.hazard_class = SCENARIO_3;
                        riskAssessment3.confidence = confidence_scenario_3;
                        adcm::Log::Info() << "Risk assessment generated for #3: " << iter->obstacle_id << "with confidence:  " << confidence_scenario_3;
                        riskAssessment.riskAssessmentList.push_back(riskAssessment3);
                    }
                }

                adcm::Log::Info() << "scenario 3 DONE";
                obstacleListVector().swap(obstacle_near_10_30); // free memory
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
                obstacleListVector().swap(obstacle_near_20_40); // free memory
            }

            {
                //=====시나리오 #5. 주행 경로상 장애물 통행량이 과다한 환경(사람)=====
                adcm::Log::Info() << "=============KATECH: scenario 5 START==============";

                obstacleListVector obstacle_pedes_40_50;
                // obstacleListVector obstacle_pedes_initial;
                obstacleListVector obstacle_pedes_repeated;
                obstacleListVector obstacle_pedes_new;
                double ori_timestamp_max = 0, new_timestamp_min = INVALID_RETURN_VALUE, distance, timestamp_diff, max_distance = 0, min_distance = INVALID_RETURN_VALUE, confidence_scenario_5;

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
                //=========ii) 주행경로 반경 10 m 이내인지 계산
                for (auto iter = obstacle_pedes_40_50.begin(); iter != obstacle_pedes_40_50.end();)
                {
                    distance = getDistance_LinearTrajectory(*iter, utm_x, utm_y);
                    if (distance > 300 || distance == INVALID_RETURN_VALUE)
                    {
                        // 10m 이상이거나 invalid 값을 지닌 사람들은 삭제
                        adcm::Log::Info() << "시나리오5-ii) 주행경로 반경 10m 초과하므로 해당 장애물 삭제: " << iter->obstacle_id;

                        iter = obstacle_pedes_40_50.erase(iter);
                    }
                    else
                        ++iter;
                }

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
                        for (auto iter = obstacle_pedes_initial.begin(); iter != obstacle_pedes_initial.end(); iter++)
                        {
                            if (ori_timestamp_max < iter->timestamp)
                                ori_timestamp_max = iter->timestamp;
                            adcm::Log::Info() << "ori obstacle : " << iter->obstacle_id;
                        }
                        for (auto iter = obstacle_pedes_new.begin(); iter != obstacle_pedes_new.end(); iter++)
                        {
                            if (new_timestamp_min > iter->timestamp)
                                new_timestamp_min = iter->timestamp;
                            adcm::Log::Info() << "new obstacle : " << iter->obstacle_id;
                        }

                        timestamp_diff = new_timestamp_min - ori_timestamp_max;
                        // adcm::Log::Info() << "timestamp ori : " << ori_timestamp_max;
                        // adcm::Log::Info() << "timestamp new : " << new_timestamp_min;
                        // adcm::Log::Info() << "timestamp 차이: " << timestamp_diff;

                        // 현재 시뮬레이션에서는 timetampe 1 이 10ms 이므로 이므로 10s=10000ms=>1000 값 이내면 신규객체 등장으로 인정
                        //=========iv) 객체 출현시간 10s 이내일때 객체간 최대거리 계산
                        if (timestamp_diff < 2000)
                        {
                            // adcm::Log::Info() << "10초 이내 신규객체 발견";
                            for (auto iter = obstacle_pedes_initial.begin(); iter != obstacle_pedes_initial.end(); iter++)
                            {
                                for (auto iter1 = obstacle_pedes_new.begin(); iter1 != obstacle_pedes_new.end(); iter1++)
                                {
                                    distance = getDistance(*iter, *iter1);
                                    adcm::Log::Info() << "distance between " << iter->obstacle_id << " , " << iter1->obstacle_id;
                                    if (max_distance < distance)
                                        max_distance = distance;
                                }
                            }
                            adcm::Log::Info() << "객체간 최대거리: " << max_distance;

                            //=========iv) 객체 출현시간 10s 이내일때 객체간 최대거리 계산
                            if (max_distance < 400)
                            {
                                //=========v) 통행과다환경 지정하고 환경 내 ego 와 30m 내외인 장애물에 대해서 confidence 값 계산
                                for (auto iter = obstacle_pedes_new.begin(); iter != obstacle_pedes_new.end(); iter++)
                                {
                                    distance = getDistance(*iter, ego_vehicle);
                                    adcm::Log::Info() << "ego 와의 거리: " << distance;

                                    //                                if (distance < 40)
                                    //                                {
                                    adcm::Log::Info() << "통행과다환경 지정!!!: " << iter->obstacle_id;
                                    confidence_scenario_5 = 300 / distance * 0.7;
                                    adcm::riskAssessmentStruct riskAssessment5;
                                    riskAssessment5.obstacle_id = iter->obstacle_id;
                                    riskAssessment5.hazard_class = SCENARIO_5;
                                    riskAssessment5.confidence = confidence_scenario_5;
                                    riskAssessment.riskAssessmentList.push_back(riskAssessment5);
                                    adcm::Log::Info() << "Risk assessment generated for #5: " << iter->obstacle_id << "with confidence:  " << confidence_scenario_5;

                                    //                                }
                                }
                            }
                        }
                    }
                }
                adcm::Log::Info() << "scenario 5 DONE";
                obstacleListVector().swap(obstacle_pedes_40_50); // free memory
                // obstacleListVector().swap(obstacle_pedes_repeated);  //free memory
                obstacleListVector().swap(obstacle_pedes_new); // free memory
            }

            {
                //=====시나리오 #6. 주행 경로상 통행량이 과다한 환경(차량)=====
                adcm::Log::Info() << "=============KATECH: scenario 6 START==============";
                obstacleListVector obstacle_vehicle_50_60;
                obstacleListVector obstacle_vehicle_repeated;
                obstacleListVector obstacle_vehicle_new;
                double ori_timestamp_max = 0, new_timestamp_min = INVALID_RETURN_VALUE, distance, timestamp_diff, max_distance = 0, min_distance = INVALID_RETURN_VALUE, confidence_scenario_6;

                //=========i) 50 < vehicle < 60 인 동적객체만 추출
                for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
                {
                    float distance_ego_obs = getDistance(*iter, ego_vehicle);
                    adcm::Log::Info() << "차량과 장애물 거리: " << distance_ego_obs;

                    if (distance_ego_obs > 500 && distance_ego_obs < 600 && (iter->obstacle_class == VEHICLE_LARGE || iter->obstacle_class == VEHICLE_SMALL))
                    {
                        adcm::Log::Info() << "시나리오6-i) 50~60m 사이의 차량 추출: " << iter->obstacle_id;

                        obstacle_vehicle_50_60.push_back(*iter);
                    }
                }
                //=========ii) 주행경로 반경 15m 이내인지 계산

                for (auto iter = obstacle_vehicle_50_60.begin(); iter != obstacle_vehicle_50_60.end();)
                {
                    distance = getDistance_LinearTrajectory(*iter, utm_x, utm_y);
                    if (distance > 150 || distance == 150 || distance == INVALID_RETURN_VALUE)
                    {
                        // 15m 이상이거나 invalid 값을 지닌 사람들은 삭제
                        adcm::Log::Info() << "시나리오6-ii) 주행경로 반경 15 이상인 장애물 삭제: " << iter->obstacle_id;

                        iter = obstacle_vehicle_50_60.erase(iter);
                    }
                    else
                        ++iter;
                }

                if (obstacle_vehicle_initial.empty())
                { // 최초 리스트 생성시 -> 여기서 해당시나리오 종료
                    adcm::Log::Info() << "시나리오6 최초 실행하므로 장애물 리스트만 생성하고 다음 loop 에 분석을 이어서 한다";
                    obstacle_vehicle_initial.assign(obstacle_vehicle_50_60.begin(), obstacle_vehicle_50_60.end());
                }

#ifdef DEMO
                // 시뮬레이션 데이터의 한계로 데모용 추가 코드 작성
                else
                {
                    // obstacle_vehicle_new 만들기
                    for (auto iter = obstacle_vehicle_initial.begin(); iter != obstacle_vehicle_initial.end(); iter++)
                    {
                        for (auto iter1 = obstacle_vehicle_50_60.begin(); iter1 != obstacle_vehicle_50_60.end(); iter1++)
                        {
                            if (iter->obstacle_id != iter1->obstacle_id)
                            {
                                obstacle_vehicle_new.push_back(*iter1);
                            }
                        }
                    }

#endif
#ifndef DEMO
                    else
                    {
                        // 해당 시나리오 계속 체크
                        //=========iii) 신규 객체 생성 빈도 측정
                        for (auto iter = obstacle_vehicle_initial.begin(); iter != obstacle_vehicle_initial.end(); iter++)
                        {
                            for (auto iter1 = obstacle_vehicle_50_60.begin(); iter1 != obstacle_vehicle_50_60.end(); iter1++)
                            {
                                if (iter->obstacle_id == iter1->obstacle_id)
                                {
                                    // 동일한 객체인 경우는 original timestamp 가지고 있는 previous 리스트에서 가지고 옴
                                    obstacle_vehicle_repeated.push_back(*iter);
                                    adcm::Log::Info() << "시나리오6-iii)시나리오 시작시 최초로 인지되고 현재 계속 인지되는 장애물: " << iter->obstacle_id;
                                }
                            }
                        }

                        // obstacle_vehicle_repeated 만들기
                        for (auto iter = obstacle_vehicle_50_60.begin(); iter != obstacle_vehicle_50_60.end(); iter++)
                        {
                            for (auto iter1 = obstacle_vehicle_repeated.begin(); iter1 != obstacle_vehicle_repeated.end(); iter1++)
                            {
                                if (iter->obstacle_id != iter1->obstacle_id)
                                {
                                    obstacle_vehicle_new.push_back(*iter);
                                }
                            }
                        }
#endif
                        // sort(obstacle_vehicle_new.begin(), obstacle_vehicle_new.end());
                        // obstacle_vehicle_new.erase( unique( obstacle_vehicle_new.begin(), obstacle_vehicle_new.end() ), obstacle_vehicle_new.end() );

                        if (obstacle_vehicle_new.empty())
                        {
                            adcm::Log::Info() << "NO new obstacle. #6 terminates here";
                        }
                        else
                        {
                            // adcm::Log::Info() << "새로운 객체 등장. 시나리오 6 계속 GOOD!";

                            for (auto iter = obstacle_vehicle_initial.begin(); iter != obstacle_vehicle_initial.end(); iter++)
                            {
                                if (ori_timestamp_max < iter->timestamp)
                                    ori_timestamp_max = iter->timestamp;
                                adcm::Log::Info() << "ori obstacle : " << iter->obstacle_id;
                            }
                            for (auto iter = obstacle_vehicle_new.begin(); iter != obstacle_vehicle_new.end(); iter++)
                            {
                                if (new_timestamp_min > iter->timestamp)
                                    new_timestamp_min = iter->timestamp;
                                adcm::Log::Info() << "new obstacle : " << iter->obstacle_id;
                            }

                            timestamp_diff = new_timestamp_min - ori_timestamp_max;
                            // adcm::Log::Info() << "timestamp ori : " << ori_timestamp_max;
                            // adcm::Log::Info() << "timestamp new : " << new_timestamp_min;
                            // adcm::Log::Info() << "timestamp diff: " << timestamp_diff;

                            // 현재 시뮬레이션에서는 timetampe 1 이 10ms 이므로 이므로 10s=10000ms=>1000 값 이내면 신규객체 등장으로 인정
                            //=========iv) 객체 출현시간 10s 이내일때 객체간 최대거리 계산
                            if (timestamp_diff < 1500)
                            {
#ifdef DEMO
                                for (auto iter = obstacle_vehicle_initial.begin(); iter != obstacle_vehicle_initial.end(); iter++)
                                {
                                    for (auto iter1 = obstacle_vehicle_new.begin(); iter1 != obstacle_vehicle_new.end(); iter1++)
                                    {
                                        distance = getDistance(*iter, *iter1);
                                        // adcm::Log::Info() << "distance between " << iter->obstacle_id << " , " << iter1->obstacle_id;
                                        if (max_distance < distance)
                                            max_distance = distance;
                                    }
                                }

#endif
#ifndef DEMO
                                //=========iv) 객체간 최대거리 < 30m? 측정
                                for (auto iter = obstacle_vehicle_50_60.begin(); iter != obstacle_vehicle_50_60.end(); iter++)
                                {
                                    for (auto iter1 = obstacle_vehicle_50_60.begin(); iter1 != obstacle_vehicle_50_60.end(); iter1++)
                                    {
                                        if (iter->obstacle_id != iter1->obstacle_id)
                                        {
                                            distance = getDistance(*iter, *iter1);
                                            adcm::Log::Info() << "distance between " << iter->obstacle_id << " , " << iter1->obstacle_id;
                                            if (max_distance < distance)
                                                max_distance = distance;
                                        }
                                    }
                                }
#endif
                                // adcm::Log::Info() << "객체간 최대거리: " << max_distance;

                                if (max_distance < 600)
                                {
                                    //=========v) 통행과다환경 지정하고 환경 내 ego 와 40m 내외인 장애물에 대해서 confidence 값 계산
#ifdef DEMO
                                    for (auto iter = obstacle_vehicle_new.begin(); iter != obstacle_vehicle_new.end(); iter++)
#endif
#ifndef DEMO
                                        for (auto iter = obstacle_vehicle_50_60.begin(); iter != obstacle_vehicle_50_60.end(); iter++)
#endif
                                        {
                                            distance = getDistance(*iter, ego_vehicle);
#ifndef DEMO
                                            if (distance < 40)
                                            {
#endif
                                                // adcm::Log::Info() << "통행과다환경 지정!!! GOOD: " << iter->obstacle_id;
                                                confidence_scenario_6 = 400 / distance * 0.7;
                                                adcm::riskAssessmentStruct riskAssessment6;
                                                riskAssessment6.obstacle_id = iter->obstacle_id;
                                                riskAssessment6.hazard_class = SCENARIO_6;
                                                riskAssessment6.confidence = confidence_scenario_6;
                                                riskAssessment.riskAssessmentList.push_back(riskAssessment6);
                                                adcm::Log::Info() << "Risk assessment generated for #6: " << iter->obstacle_id << "with confidence:  " << confidence_scenario_6;
#ifndef DEMO
                                            }
#endif
                                        }
                                }
                            }
                        }
                    }
                    adcm::Log::Info() << "scenario 6 DONE";
                    obstacleListVector().swap(obstacle_vehicle_50_60);    // free memory
                    obstacleListVector().swap(obstacle_vehicle_repeated); // free memory
                    obstacleListVector().swap(obstacle_vehicle_new);      // free memory
                }

                {
                    if (riskAssessment.riskAssessmentList.size() == 0)
                    {
                        adcm::Log::Info() << "no riskAssessment data before scenario 7!";
                    }
                    //=====시나리오 #7. 미개척 지역 주행환경======
                    adcm::Log::Info() << "=============KATECH: scenario 7 START==============";
                    // adcm::Log::Info() << "size of map_2d received (before drawline): "<<map_2d.size();
                    adcm::riskAssessmentStruct riskAssessment7;

                    if (utm_x.size() != 0 && map_2d.size() != 0)
                    {
                        drawline(utm_x, utm_y, map_2d, riskAssessment);
                    }
                    adcm::Log::Info() << "scenario 7 DONE";
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
                adcm::Log::Info() << "==================================================================";
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
                    //risktAssessment object 의 생성시간 추가
                    auto now = std::chrono::system_clock::now();
                    auto riskAssessment_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
                    adcm::Log::Info() << "Current timestamp in milliseconds: " << riskAssessment_timestamp;
                    riskAssessment.timestamp = riskAssessment_timestamp;
                    adcm::Log::Info() << "riskAssessment send!";
                    riskAssessment_provider.send(riskAssessment);

                    NatsSend(riskAssessment);
                }
                else
                {
                    //NatsSend(riskAssessment); //테스트 용도로 계속 송신하게 한다
                    adcm::Log::Info() << "riskAssessment size is 0, doesn't send";

                }
                riskAssessment.riskAssessmentList.clear();
            }

            // risk_assessmentDataQueue.push(riskAssessment);
            // riskAssDataQueueCondition.notify_one();
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

    /*void ThreadTxManager()
    {
        adcm::RiskAssessment_Provider riskAssessment_provider;
        riskAssessment_provider.init("RiskAssessment/RiskAssessment/PPort_risk_assessment");

        while(continueExecution)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::vector<adcm::risk_assessment_Objects> dataBatch;
            GetRiskAssDataFromQueue(dataBatch);

            for (auto& data : dataBatch)
            {
                adcm::Log::Info() << "Send data...";
                riskAssessment_provider.send(data);
            }
        }
    }
    */
    int main(int argc, char *argv[])
    {
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
        std::thread act1(ThreadAct1);
        std::thread monitor(ThreadMonitor);
        std::thread katechThread(ThreadKatech);
        //  	std::thread txMgrThread(ThreadTxManager);

        // build_path Method Call Test
        adcm::Log::Info() << "Thread join";
        act1.join();
        monitor.join();
        katechThread.join();
        //    txMgrThread.join();
        adcm::Log::Info() << "done.";

        if (!ara::core::Deinitialize())
        {
            // No interaction with ARA is possible here since some ARA resources can be destroyed already
            return EXIT_FAILURE;
        }

        return EXIT_SUCCESS;
    }
