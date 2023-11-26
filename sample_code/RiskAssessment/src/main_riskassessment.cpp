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

#include <ara/com/e2exf/status_handler.h>
#include <ara/exec/execution_client.h>

#include <ara/log/logger.h>
#include "logger.h"
#include "ara/core/initialization.h"

#include "risk_assessment_provider.h"
#include "map_data_subscriber.h"

#include "main_riskassessment.hpp"


    
namespace
{

// Atomic flag for exit after SIGTERM caught
std::atomic_bool continueExecution{true};
std::atomic_uint gReceivedEvent_count_map_data{0};
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


//==============1. MapData 생성 =================
//TO DO: Datafusion 에서 제대로 받아오면 삭제
//현재 테스트용
MapData map_data;


std::vector<ObstacleData> obstacle_pedes_previous;
std::vector<ObstacleData> obstacle_vehicle_previous;

//=============================================

//============== 함수 definition =================

double getDistance(ObstacleData obstacle, VehicleData vehicle)
{
    return sqrt(pow(obstacle.fused_position_x - vehicle.position_x, 2) + pow(obstacle.fused_position_y - vehicle.position_y, 2));
}
double getDistance(ObstacleData obstacle1, ObstacleData obstacle2)
{
    return sqrt(pow(obstacle1.fused_position_x - obstacle2.fused_position_x, 2) + pow(obstacle1.fused_position_y - obstacle2.fused_position_y, 2));
}
double getDistance(ObstacleData obstacle, double a_x, double a_y)
{
    return sqrt(pow(obstacle.fused_position_x - a_x, 2) + pow(obstacle.fused_position_y - a_y, 2));
}
double getDistance(double a_x, double a_y, double b_x, double b_y)
{
    return sqrt(pow(a_x - b_x, 2) + pow(a_y - b_y, 2));
}

double cot(float x)
{
    return (1 / tan(x));
}

float getTTC(ObstacleData obstacle, VehicleData vehicle)
{
    int ttc_difference = 5; //to adjust
    float ttc = 0;
    float x_collision, y_collision;
    x_collision = ((obstacle.fused_position_y - vehicle.position_y) - (obstacle.fused_position_x * tan(obstacle.fused_heading_angle) - vehicle.position_x * tan(vehicle.yaw))) / (tan(vehicle.yaw) - tan(obstacle.fused_heading_angle));
    y_collision = ((obstacle.fused_position_x - vehicle.position_x) - (obstacle.fused_position_y * cot(obstacle.fused_heading_angle) - vehicle.position_y * cot(vehicle.yaw))) / (cot(vehicle.yaw) - cot(obstacle.fused_heading_angle));
    
    if (x_collision >0 && x_collision < map_n && y_collision > 0 && y_collision < map_m) //작업환경 내에서 충돌이 일어날때 (infinity check)
    {
        float time_vehicle_x_collision = abs((x_collision - vehicle.position_x) / vehicle.velocity_x);
        float time_obstacle_x_collision = abs((x_collision - obstacle.fused_position_x) / obstacle.fused_velocity_x);
        if (abs(time_vehicle_x_collision - time_obstacle_x_collision) < ttc_difference)
        {
            if (time_vehicle_x_collision < time_obstacle_x_collision)
                return time_vehicle_x_collision;
            else
                return time_obstacle_x_collision;
        }
    }
    else
        return INVALID_RETURN_VALUE; //return random big number
}

float getMagnitude(Point2D a)
{
    return sqrt(a.x^2 + a.y^2);
}
float getLinearApprox(ObstacleData obstacle, VehicleData vehicle)
{
    float min_distance_ego_obs=INVALID_RETURN_VALUE; //random big number
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
//double getTTC(ObstacleData obstacle, VehicleData vehicle)
//{
//     double ttc=0;
//     double distance=0;
//     float t = 0.1;
//     double obstacle_final_pos_x, obstacle_final_pos_y, vehicle_final_pos_x, vehicle_final_pos_y;
    
//     for (t; t < 10; t=t+0.05)
//     {
//         obstacle_final_pos_x = obstacle.fused_position_x + obstacle.fused_velocity_x*t;
//         obstacle_final_pos_y = obstacle.fused_position_y + obstacle.fused_velocity_y*t;
//         vehicle_final_pos_x = vehicle.position_long + vehicle.velocity_long*t;
//         vehicle_final_pos_y = vehicle.position_lat + vehicle.velocity_lat*t;
//         distance=getDistance(obstacle_final_pos_x,obstacle_final_pos_y,vehicle_final_pos_x,vehicle_final_pos_y);

//         if (distance < COLLISION_DISTANCE)
//         {    //현재 상대 거리가 3m 인 경우 충돌이 일어났다고 가정-이 시간을 ttc로 정의
//             break; 
//         }
//     }
//     ttc = t;
// //현재 ttc가 10초가 넘어가면 그냥 10초로 정의한다 
//     return ttc;
//}  

float getDistance_LinearTrajectory(ObstacleData obstacle, VehicleData ego_vehicle, build_path_Objects path, std::vector<RiskAssessment>& risk_assessment)
{
    float confidence;
    for (int count = 0; count < path.utm_x.size(); count++)
    {
        float x_start = path.utm_x[count];
        float x_end = path.utm_x[count + 1];
        float y_start = path.utm_y[count];
        float y_end = path.utm_y[count + 1];

        Point2D start_to_end_vector;
        start_to_end_vector.x = x_end - x_start;
        start_to_end_vector.y = y_end - y_start;

        Point2D start_to_obs_vector;
        start_to_obs_vector.x = obstacle.fused_position_x - x_start;
        start_to_obs_vector.y = obstacle.fused_position_y - y_start;

        double angle = atan2(start_to_end_vector.y, start_to_end_vector.x) - atan2(start_to_obs_vector.y,start_to_obs_vector.x);
        double start_to_end_vector_DOT_start_to_obs_vector = cos(angle) * getMagnitude(start_to_end_vector) * getMagnitude(start_to_obs_vector);
        if (start_to_end_vector_DOT_start_to_obs_vector > 0 && start_to_end_vector_DOT_start_to_obs_vector < pow(getMagnitude(start_to_end_vector), 2))
            //장애물 위치가 시작점과 끝점 사이일때
        {
            double distance = getMagnitude(start_to_obs_vector) * sqrt(1 - pow(start_to_end_vector_DOT_start_to_obs_vector / (getMagnitude(start_to_end_vector) * getMagnitude(start_to_obs_vector)), 2));
            if (distance < 10)
            {
                confidence = 20 / getDistance(obstacle, ego_vehicle) * 0.7;
                risk_assessment.push_back({ obstacle.obstacle_id, SCENARIO_1, 1, confidence});
            }
        }
    }   
}

ObstacleData searchLinearpath(std::vector<ObstacleData> obstacle_list, float x1, float y1, float x2, float y2)
{
    //목적: 두 coordinate 사이를 연결해 직선을 만들고 map_n 등분해 
    // map_n 등분된 직선 위 x-y coordinate 과 모든 장애물간의 거리를 구해서 10m 이내를 찾는다
    float coeff = (y2 - y1) / (x2 - x1);
    float constant = y1 - x1 * coeff;
    int divide = 10; //임의의 수로 수정 가능 - 두 포인트를 10등분 해 장애물 찾기를 진행 

    for (float new_x = x1; new_x < x2; new_x = new_x + (x2 - x1) / divide)
    {
        float new_y = new_x * coeff + constant;
        for (auto iter = obstacle_list.begin(); iter != obstacle_list.end();iter++)
        {
            if (getDistance(*iter, new_x, new_y) <= 10)
            {
                //장애물이 전역경로와 10m 내에 위치 
                return (*iter); //해당 장애물을 리턴
            }
            else continue;

        }

    }
}

float getDistance_LinearTrajectory(ObstacleData current_obstacle, build_path_Objects path)
{
    for (int count = 0; count < path.utm_x.size(); count++)
    {
        float x_start = path.utm_x[count];
        float x_end = path.utm_x[count + 1];
        float y_start = path.utm_y[count];
        float y_end = path.utm_y[count + 1];

        Point2D start_to_end_vector;
        start_to_end_vector.x = x_end - x_start;
        start_to_end_vector.y = y_end - y_start;

        Point2D start_to_obs_vector;
        start_to_obs_vector.x = current_obstacle.fused_position_x - x_start;
        start_to_obs_vector.y = current_obstacle.fused_position_y - y_start;

        double angle = atan2(start_to_end_vector.y, start_to_end_vector.x) - atan2(start_to_obs_vector.y, start_to_obs_vector.x);
        double start_to_end_vector_DOT_start_to_obs_vector = cos(angle) * getMagnitude(start_to_end_vector) * getMagnitude(start_to_obs_vector);
        if (start_to_end_vector_DOT_start_to_obs_vector > 0 && start_to_end_vector_DOT_start_to_obs_vector < pow(getMagnitude(start_to_end_vector), 2))
            //장애물 위치가 시작점과 끝점 사이일때
            //장애물과 전역경로간의 거리를 구한다
        {
            float distance = getMagnitude(start_to_obs_vector) * sqrt(1 - pow(start_to_end_vector_DOT_start_to_obs_vector / (getMagnitude(start_to_end_vector) * getMagnitude(start_to_obs_vector)), 2));
            return distance;
        }
        else
        {
            float distance = INVALID_RETURN_VALUE;
            return distance;
        }

    }
}


void drawline(build_path_Objects path, std::vector<RiskAssessment>& risk_assessment)
{
    bool breakFlag; //지정된 전역경로 (x1,y1) 과 (x2, y2) 사이 하나라도 
    for (int count = 0; count < path.utm_x.size() - 1; count++)
    {
        float x_start = path.utm_x[count];
        float x_end = path.utm_x[count + 1];
        float y_start = path.utm_y[count];
        float y_end = path.utm_y[count + 1];

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
        const int maxX = (int)x_end;

        for (int x = (int)x_start; x <= maxX; x++)
        {
            if (steep)
            {
                if (map_data.map_2d[y][x].road_z == 0)
                {
                    risk_assessment.push_back({ std::make_pair(path.utm_x[count], path.utm_y[count]), std::make_pair(path.utm_x[count + 1], path.utm_y[count + 1]), SCENARIO_7, 1 });
                    break; //한번만 들어가도 for loop break
                }
            }
            else if (map_data.map_2d[x][y].road_z == 0)
            {
                risk_assessment.push_back({ std::make_pair(path.utm_x[count], path.utm_y[count]), std::make_pair(path.utm_x[count + 1], path.utm_y[count + 1]), SCENARIO_7, 1 });
                break;
            }

            error -= dy;
            if (error < 0)
            {
                y += ystep;
                error += dx;
            }
        }
    }
}

//===============================

}  // namespace


void ThreadAct1()
{
    adcm::Log::Info() << "RiskAssessment ThreadAct1";
    INFO("RiskAssessment .init()");
    adcm::RiskAssessment_Provider riskAssessment_provider;
    adcm::MapData_Subscriber mapData_subscriber;
    riskAssessment_provider.init("RiskAssessment/RiskAssessment/PPort_risk_assessment");
    mapData_subscriber.init("RiskAssessment/RiskAssessment/RPort_map_data");
    INFO("After RiskAssessment .init()");
    std::random_device m_rd;
    std::default_random_engine m_rand_eng(m_rd());
    std::uniform_real_distribution<double> m_ud_10000_10000(-10000, 10000);
    std::uniform_real_distribution<float> m_ud_100_100(-100, 100);
    std::uniform_int_distribution<std::uint32_t> m_ud_0_10000(0, 10000);
    std::uniform_int_distribution<std::uint8_t> m_ud_0_4(0, 4);
    INFO("Thread loop start...");

    while (continueExecution) {
        gMainthread_Loopcount++;
        VERBOSE("[RiskAssessment] Application loop");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool mapData_rxEvent = mapData_subscriber.waitEvent(0); // wait event

        if(mapData_rxEvent) {
            adcm::Log::Verbose() << "[EVENT] DataFusion Map Data received";

            while(!mapData_subscriber.isEventQueueEmpty()) {
                auto data = mapData_subscriber.getEvent();
                gReceivedEvent_count_map_data++;

                auto obstacle = data->obstacle;
                auto environment = data->environment;
                auto vehicle = data->vehicle;

                adcm::Log::Verbose() << "obstacle.Time_stamp : "<< obstacle.Time_stamp;
                adcm::Log::Info() << "Data received (dummy timestamp): "  << obstacle.Time_stamp;
                // adcm::Log::Verbose() << "obstacle.fused_index : "<< obstacle.fused_index;
                // adcm::Log::Verbose() << "obstacle.fused_cuboid_x : "<< obstacle.fused_cuboid_x;
                // adcm::Log::Verbose() << "obstacle.fused_cuboid_y : "<< obstacle.fused_cuboid_y;
                // adcm::Log::Verbose() << "obstacle.fused_cuboid_z : "<< obstacle.fused_cuboid_z;
                // adcm::Log::Verbose() << "obstacle.fused_heading_angle : "<< obstacle.fused_heading_angle;
                // adcm::Log::Verbose() << "obstacle.fused_Position_x : "<< obstacle.fused_Position_x;
                // adcm::Log::Verbose() << "obstacle.fused_Position_y : "<< obstacle.fused_Position_y;
                // adcm::Log::Verbose() << "obstacle.fused_Position_z : "<< obstacle.fused_Position_z;
                // adcm::Log::Verbose() << "obstacle.fused_velocity_x : "<< obstacle.fused_velocity_x;
                // adcm::Log::Verbose() << "obstacle.fused_velocity_y : "<< obstacle.fused_velocity_y;
                // adcm::Log::Verbose() << "obstacle.fused_velocity_z : "<< obstacle.fused_velocity_z;

                auto road_z = environment.road_z;

                // adcm::Log::Info() << "vehicle.Vehicle_id : "<< vehicle.Vehicle_id;
                // adcm::Log::Info() << "vehicle.Position_lat : "<< vehicle.Position_lat;
                // adcm::Log::Info() << "vehicle.Position_long : "<< vehicle.Position_long;
                // adcm::Log::Info() << "vehicle.Position_Height : "<< vehicle.Position_Height;
                // adcm::Log::Info() << "vehicle.Yaw : "<< vehicle.Yaw;
                // adcm::Log::Info() << "vehicle.Roll : "<< vehicle.Roll;
                // adcm::Log::Info() << "vehicle.Pitch : "<< vehicle.Pitch;
                // adcm::Log::Info() << "vehicle.Velocity_long : "<< vehicle.Velocity_long;
                // adcm::Log::Info() << "vehicle.Velocity_lat : "<< vehicle.Velocity_lat;
                // adcm::Log::Info() << "vehicle.Velocity_ang : "<< vehicle.Velocity_ang;
            }

            {
                adcm::risk_assessment_Objects riskAssessment;

                //================1. obstacle list 확인================
                std::vector<RiskAssessment> risk_assessment;

                //테스트용 코드
                //실제로는 obstacle list 는 Datafusion 에서 받음
                ObstacleData current_obstacle;
                current_obstacle.obstacle_id = 142;
                current_obstacle.action_class = REMOVE_BLIND_SPOT;
                current_obstacle.fused_position_x = 20;
                current_obstacle.fused_position_y = 10;
                current_obstacle.fused_cuboid_x = 4;
                current_obstacle.fused_cuboid_y = 2;
                current_obstacle.fused_cuboid_z = 10;
                current_obstacle.stop_count = 10;
                map_data.obstacle_list.push_back(current_obstacle);

                VehicleData current_vehicle;
                current_vehicle.vehicle_class = EGO_VEHICLE;
                current_vehicle.position_x = 0;
                current_vehicle.position_y = 0;
                map_data.vehicle_list.push_back(current_vehicle);

                VehicleData ego_vehicle, sub_vehicle_1, sub_vehicle_2, sub_vehicle_3, sub_vehicle_4;

                for (auto iter = map_data.vehicle_list.begin(); iter != map_data.vehicle_list.end(); iter++)
                {
                    switch (iter->vehicle_class)
                    {
                    case EGO_VEHICLE:
                        ego_vehicle = *iter;
                        break;
                    case SUB_VEHICLE_1:
                        sub_vehicle_1 = *iter;
                        break;
                    case SUB_VEHICLE_2:
                        sub_vehicle_2 = *iter;
                        break;
                    case SUB_VEHICLE_3:
                        sub_vehicle_3 = *iter;
                        break;
                    case SUB_VEHICLE_4:
                        sub_vehicle_4 = *iter;
                        break;
                    }
                }

                //=====시나리오 #1. 주행중 전역경로 근방 이동가능한 정지 장애물이 존재하는 위험 환경=====
                //=========i) 정지상태 판정: 정해진 duration STOP_VALUE * 0.1s 만큼 정지해 있을경우
                std::vector<ObstacleData> obstacle_stop;
                for (auto iter = map_data.obstacle_list.begin(); iter != map_data.obstacle_list.end(); iter++)
                {
                    if (iter->stop_count > STOP_VALUE || iter->stop_count == STOP_VALUE && iter->obstacle_class != STRUCTURE)
                    {
                        obstacle_stop.push_back(*iter);
                    }
                }

                //=========ii) 특장차로부터의 거리 판정: 30m 거리 이내인 경우
                for (auto iter = obstacle_stop.begin(); iter!= obstacle_stop.end(); iter++)
                {
                    float distance_ego_obs = getDistance(*iter, ego_vehicle);
                    if (distance_ego_obs > 30 || distance_ego_obs == 30) {
                        obstacle_stop.erase(iter); //30m 이상인 경우 해당 장애물 삭제 
                    }
                }

                //=========iii) 장애물과 전역경로간 거리 추정
                build_path_Objects path;
                double confidence = 0;

                path.utm_x.insert(path.utm_x.end(), { 0, 5, 56, 94, 150 });
                path.utm_y.insert(path.utm_y.end(), { 0, 20, 53, 88, 230 });

                for (int count = 0; count < path.utm_x.size(); count++)
                {
                    float x_start = path.utm_x[count];
                    float x_end = path.utm_x[count + 1];
                    float y_start = path.utm_y[count];
                    float y_end = path.utm_y[count + 1];

                    Point2D start_to_end_vector;
                    start_to_end_vector.x = x_end - x_start;
                    start_to_end_vector.y = y_end - y_start;

                    for (auto iter = obstacle_stop.begin(); iter != obstacle_stop.end(); iter++)
                    {
                        Point2D start_to_obs_vector;
                        start_to_obs_vector.x = iter->fused_position_x - x_start;
                        start_to_obs_vector.y = iter->fused_position_y - y_start;

                        double angle = atan2(start_to_end_vector.y, start_to_end_vector.x) - atan2(start_to_obs_vector.y,start_to_obs_vector.x);
                        double start_to_end_vector_DOT_start_to_obs_vector = cos(angle) * getMagnitude(start_to_end_vector) * getMagnitude(start_to_obs_vector);
                        if (start_to_end_vector_DOT_start_to_obs_vector > 0 && start_to_end_vector_DOT_start_to_obs_vector < pow(getMagnitude(start_to_end_vector), 2))
                            //장애물 위치가 시작점과 끝점 사이일때
                        {
                            double distance = getMagnitude(start_to_obs_vector) * sqrt(1 - pow(start_to_end_vector_DOT_start_to_obs_vector / (getMagnitude(start_to_end_vector) * getMagnitude(start_to_obs_vector)), 2));
                            if (distance < 10)
                            {
                                confidence = 20 / getDistance(*iter, ego_vehicle) * 0.7;
                                risk_assessment.push_back({iter->obstacle_id, SCENARIO_1, 1, confidence})
                            }
                        }
                    }
                }

                //=====시나리오 #2. 주행중 사각영역 존재 환경 판단=====
                //=========i) 정지상태 판정: 시나리오 #1에서 만든 obstacle_stop + obstacle_class 가 정적 객체이고 1m 이상인 경우도 포함
                std::vector<ObstacleData> obstacle_static_stop; 
                obstacle_static_stop.assign(obstacle_stop.begin(), obstacle_stop.end());

                for (auto iter = map_data.obstacle_list.begin(); iter != map_data.obstacle_list.end(); iter++)
                {
                    if (iter->obstacle_class == STRUCTURE && iter->fused_cuboid_z > 1)
                    { //obstacle_class 가 정적 객체이고 높이가 1m 이상인 경우
                        obstacle_static_stop.push_back(*iter);
                    }
                }

                //=========ii) 특장차로부터의 거리 판정: 40m 거리 이내인 경우
                for (auto iter = obstacle_static_stop.begin(); iter!= obstacle_static_stop.end(); iter++)
                {
                    float distance_ego_obs = getDistance(*iter, ego_vehicle);
                    if (distance_ego_obs > 40 || distance_ego_obs == 40) {
                        obstacle_static_stop.erase(iter); //30m 이상인 경우 해당 장애물 삭제 
                    }
                }

                //==========iii) 장애물과 전역경로간 거리 추정

                for (int count = 0; count < path.utm_x.size(); count++)
                {
                    float x_start = path.utm_x[count];
                    float x_end = path.utm_x[count + 1];
                    float y_start = path.utm_y[count];
                    float y_end = path.utm_y[count + 1];

                    Point2D start_to_end_vector;
                    start_to_end_vector.x = x_end - x_start;
                    start_to_end_vector.y = y_end - y_start;

                    for (auto iter = obstacle_static_stop.begin(); iter != obstacle_static_stop.end(); iter++)
                    {
                        Point2D start_to_obs_vector;
                        start_to_obs_vector.x = iter->fused_position_x - x_start;
                        start_to_obs_vector.y = iter->fused_position_y - y_start;

                        double angle = atan2(start_to_end_vector.y, start_to_end_vector.x) - atan2(start_to_obs_vector.y,start_to_obs_vector.x);
                        double start_to_end_vector_DOT_start_to_obs_vector = cos(angle) * getMagnitude(start_to_end_vector) * getMagnitude(start_to_obs_vector);
                        if (start_to_end_vector_DOT_start_to_obs_vector > 0 && start_to_end_vector_DOT_start_to_obs_vector < pow(getMagnitude(start_to_end_vector), 2))
                            //장애물 위치가 시작점과 끝점 사이일때
                        {
                            double distance = getMagnitude(start_to_obs_vector) * sqrt(1 - pow(start_to_end_vector_DOT_start_to_obs_vector / (getMagnitude(start_to_end_vector) * getMagnitude(start_to_obs_vector)), 2));
                            if (distance < 10)
                            {
                                confidence = 20 / getDistance(*iter, ego_vehicle) * 0.7;
                                risk_assessment.push_back({iter->obstacle_id, SCENARIO_2, 1, confidence})
                            }
                        }
                    }
                }


                std::vector<ObstacleData>().swap(obstacle_stop);  //free memory
                std::vector<ObstacleData>().swap(obstacle_static_stop);  //free memory            
            

            //=====시나리오 #3. 주행중 경로 주변 동적 장애물 통행 환경 판단=====
                //=========i) 특장차로부터의 거리 판정: 10<obs<30m 거리 이내인 경우
                std::vector<ObstacleData> obstacle_near;
                float ttc=99999;
                float dist_ego_obs_linear_approx;
                float ttc_confidence, area_confidence, final_confidence;

                for (auto iter = map_data.obstacle_list.begin(); iter != map_data.obstacle_list.end(); iter++)
                {
                    float distance_ego_obs = getDistance(*iter, ego_vehicle);
                    if ((distance_ego_obs > 10 || distance_ego_obs == 10) && (distance_ego_obs < 30 || distance_ego_obs == 30))
                    {
                        obstacle_near.push_back(*iter);
                    }
                }

                //=========ii) TTC 와 5초후 안전영역 진입여부 확인 후 컨피던스 계산
                for (auto iter = obstacle_near.begin(); iter != obstacle_near.end(); iter++)
                {
                    ttc = getTTC(*iter, ego_vehicle);
                    dist_ego_obs_linear_approx = getLinearApprox(*iter, ego_vehicle);
                    ttc_confidence = 5 / ttc * 0.7;
                    area_confidence = 30 / dist_ego_obs_linear_approx * 0.7;

                    //infinity check ->  isinf()
                    if (ttc_confidence > area_confidence)
                    {
                        final_confidence = ttc_confidence;
                    }
                    else final_confidence = area_confidence; // 큰 값을 취한다
                    
                    risk_assessment.push_back({iter->obstacle_id, SCENARIO_3, 1, final_confidence});
                }

                //=====시나리오 #4. 작업 중 경로 주변 동적 장애물 통행 환경=====
                //=========i) 20~40m 내 동적객체 추출
                std::vector<ObstacleData> obstacle_near_20_40;
                float confidence;

                for (auto iter = map_data.obstacle_list.begin(); iter != map_data.obstacle_list.end(); iter++)
                {
                    float distance_ego_obs = getDistance(*iter, ego_vehicle);
                    if (distance_ego_obs > 20 && distance_ego_obs < 40 && iter->obstacle_class != STRUCTURE)
                    {
                        obstacle_near_20_40.push_back(*iter);
                    }
                }
                //=========ii) 5초후 경로가 40m 이내인 경우 안전영역 진입했음으로 컨피던스값 계산해 넣어줌 

                for (auto iter = obstacle_near_20_40.begin(); iter != obstacle_near_20_40.end(); iter++)
                {
                    dist_ego_obs_linear_approx = getLinearApprox(*iter, ego_vehicle);
                    if (dist_ego_obs_linear_approx < 40)
                    {
                        confidence = 40 / dist_ego_obs_linear_approx * 0.7;
                        risk_assessment.push_back({ iter->obstacle_id, SCENARIO_4, 1, confidence});
                    }
                }

                
                //=====시나리오 #5. 주행 경로상 장애물 통행량이 과다한 환경(사람)=====
                //=========i) 40 < pedestrian < 50 인 보행자만 추출

                std::vector<ObstacleData> obstacle_pedes;
                std::vector<ObstacleData> obstacle_pedes_repeated;
                float distance,timestamp_diff, max_distance=0, min_distance =INVALID_RETURN_VALUE;
            

                for (auto iter = map_data.obstacle_list.begin(); iter != map_data.obstacle_list.end(); iter++)
                {
                    float distance_ego_obs = getDistance(*iter, ego_vehicle);
                    if (distance_ego_obs > 40 && distance_ego_obs < 50 && iter->obstacle_class == PEDESTRIAN)
                    {
                        obstacle_pedes.push_back(*iter);
                    }
                }
                //=========ii) 주행경로 반경 10 m 이내인지 계산 

                for (auto iter = obstacle_pedes.begin(); iter != obstacle_pedes.end(); iter++)
                {
                    distance = getDistance_LinearTrajectory(*iter, ego_vehicle, path);
                    if (distance > 10 || distance == 10 || distance == INVALID_RETURN_VALUE)
                    {
                        // 10m 이상이거나 invalid 값을 지닌 사람들은 삭제
                        obstacle_pedes.erase(iter);
                    }
                }
                if (obstacle_pedes_previous.empty())
                { //최초 리스트 생성시 -> 여기서 해당시나리오 종료
                    obstacle_pedes_previous.assign(obstacle_pedes.begin(), obstacle_pedes.end());
                }
                else
                {
                    // 해당 시나리오 계속 체크
                    //=========iii) 신규 객체 생성 빈도 측정
                    for (auto iter = obstacle_pedes.begin(); iter != obstacle_pedes.end(); iter++)
                    {
                        for (auto iter1 = obstacle_pedes_previous.begin(); iter1 != obstacle_pedes_previous.end(); iter1++)
                        {
                            if (iter1->obstacle_id == iter->obstacle_id)
                            {
                                //동일 장애물 발견 (else 아무것도 안함) 
                                //동일 장애물 timestamp 비교해 10s 이하만 keep 
                                timestamp_diff = iter->timestamp - iter1->timestamp;
                                if (timestamp_diff < 1000) // 1000ms 
                                {
                                    obstacle_pedes_repeated.push_back(*iter);
                                }
                            }
                        }
                    }
                    //업데이트 
                    obstacle_pedes_previous.assign(obstacle_pedes.begin(), obstacle_pedes.end());

                    //=========iv) 객체간 최대거리 < 20m? 측정

                    for (auto iter = obstacle_pedes_repeated.begin(); iter != obstacle_pedes_repeated.end(); iter++)
                    {
                        for (auto iter1 = obstacle_pedes_repeated.begin(); iter1 != obstacle_pedes_repeated.end(); iter1++)
                        {
                            if (iter->obstacle_id != iter1->obstacle_id)
                            {
                                distance = getDistance(*iter, *iter1);
                                if (max_distance < distance) max_distance = distance;
                            }
                        }
                    }

                    if (max_distance < 20)
                    {
                        //=========v) 통행과다환경 지정하고 환경 내 ego 와 30m 내외인 장애물에 대해서 confidence 값 계산
                        for (auto iter = obstacle_pedes_repeated.begin(); iter != obstacle_pedes_repeated.end(); iter++)
                        {
                            distance = getDistance(*iter, ego_vehicle);
                            if (distance < 30)
                            {
                                float confidence = 30 / distance * 0.7;
                                risk_assessment.push_back({ iter->obstacle_id, SCENARIO_5, 1, confidence });
                            }
                        }
                    }
                }
               
                
                //=====시나리오 #6. 주행 경로상 통행량이 과다한 환경(차량)=====
                //=========i) 50 < vehicle < 60 인 동적객체만 추출
                std::vector<ObstacleData> obstacle_vehicle;
                std::vector<ObstacleData> obstacle_vehicle_repeated;
                distance = 0;
                timestamp_diff = 0;
                max_distance = 0;

                for (auto iter = map_data.obstacle_list.begin(); iter != map_data.obstacle_list.end(); iter++)
                {
                    float distance_ego_obs = getDistance(*iter, ego_vehicle);
                    if (distance_ego_obs > 50 && distance_ego_obs < 60 && (iter->obstacle_class == VEHICLE_LARGE || iter->obstacle_class == VEHICLE_SMALL))
                    {
                        obstacle_vehicle.push_back(*iter);
                    }
                }
                //=========ii) 주행경로 반경 15 m 이내인지 계산 

                for (auto iter = obstacle_vehicle.begin(); iter != obstacle_vehicle.end(); iter++)
                {
                    distance = getDistance_LinearTrajectory(*iter, ego_vehicle, path);
                    if (distance > 15 || distance == 15 || distance == INVALID_RETURN_VALUE)
                    {
                        // 15m 이상이거나 invalid 값을 지닌 사람들은 삭제
                        obstacle_vehicle.erase(iter);
                    }
                }

                if (obstacle_vehicle_previous.empty())
                { //최초 리스트 생성시 -> 여기서 해당시나리오 종료
                    obstacle_vehicle_previous.assign(obstacle_vehicle.begin(), obstacle_vehicle.end());
                }
                else
                {
                    // 해당 시나리오 계속 체크
                    //=========iii) 신규 객체 생성 빈도 측정
                    for (auto iter = obstacle_vehicle.begin(); iter != obstacle_vehicle.end(); iter++)
                    {
                        for (auto iter1 = obstacle_vehicle_previous.begin(); iter1 != obstacle_vehicle_previous.end(); iter1++)
                        {
                            if (iter1->obstacle_id == iter->obstacle_id)
                            {
                                //동일 장애물 발견 (else 아무것도 안함) 
                                //동일 장애물 timestamp 비교해 10s 이하만 keep 
                                timestamp_diff = iter->timestamp - iter1->timestamp;
                                if (timestamp_diff < 1000) // 1000ms 
                                {
                                    obstacle_vehicle_repeated.push_back(*iter);
                                }
                            }

                        }
                    }
                    //업데이트 
                    obstacle_vehicle_previous.assign(obstacle_vehicle.begin(), obstacle_vehicle.end());

                    //=========iv) 객체간 최대거리 < 30m? 측정

                    for (auto iter = obstacle_vehicle_repeated.begin(); iter != obstacle_vehicle_repeated.end(); iter++)
                    {
                        for (auto iter1 = obstacle_vehicle_repeated.begin(); iter1 != obstacle_vehicle_repeated.end(); iter1++)
                        {
                            if (iter->obstacle_id != iter1->obstacle_id)
                            {
                                distance = getDistance(*iter, *iter1);
                                if (max_distance < distance) max_distance = distance;
                            }
                        }
                    }

                    if (max_distance < 30)
                    {
                        //=========v) 통행과다환경 지정하고 환경 내 ego 와 40m 내외인 장애물에 대해서 confidence 값 계산
                        for (auto iter = obstacle_vehicle_repeated.begin(); iter != obstacle_vehicle_repeated.end(); iter++)
                        {
                            distance = getDistance(*iter, ego_vehicle);
                            if (distance < 40)
                            {
                                float confidence = 40 / distance * 0.7;
                                risk_assessment.push_back({ iter->obstacle_id, SCENARIO_6, confidence });
                            }
                        }
                    }
                }


                //=====시나리오 #7. 미개척 지역 주행환경======

                drawline(path, risk_assessment);


                // //================전역경로 관련 위험 시나리오================

                // //특장차의 위치와 첫 x-y 경로 좌표 기준으로 직선을 그어 이차방적식 생성
                // //첫 x-y 좌표는 특장차의 위치인 시작점 (여기선 0,0)을 포함한다고 가정 함
                // for (int count = 0; count < path.utm_x.size ;  count++)
                // {
                //     searchLinearpath(map_data.obstacle_list, path.utm_x[count], path.utm_y[count], path.utm_x[count+1], path.utm_y[count+1]);

                // }

                

                // for (auto iter = map_data.obstacle_list.begin(); iter!= map_data.obstacle_list.end();iter++)
                // {
                //     switch(iter->action_class)
                //     {
                //         case REMOVE_BLIND_SPOT:
                //         {
                //             float final_confidence = 0;
                //             // risk_assessment.obstacle_id.push_back(current_obstacle.obstacle_id);
                //             // risk_assessment.hazard_class.push_back(BLIND_SPOT);
                //             // angle 포함 안시킨 경우
                //             double xy_distance = getDistance(*iter, ego_vehicle);
                //             final_confidence = STRUCTURE_DISTANCE / xy_distance * 0.7;
                //             if (final_confidence > CONFIDENCE_THRESHOLD || final_confidence == CONFIDENCE_THRESHOLD)
                //             {
                //                 risk_assessment.emplace_back(RiskAssessment(iter->obstacle_id, BLIND_SPOT, 1, final_confidence));
                //             }
                //             else
                //             {
                //                 risk_assessment.emplace_back(RiskAssessment(iter->obstacle_id, BLIND_SPOT, 0, final_confidence));
                //             }
                //             adcm::Log::Info() << "confident for blindspot is " << final_confidence;
                //             adcm::Log::Info() << "obstalce id is " << risk_assessment[0].obstacle_id;

                //         } break;

                //         case ALERT_OBSTACLE: 
                //         {
                //             //동적 장애물일 경우
                //             float final_confidence = 0;
                //             double xy_distance = getDistance(*iter, ego_vehicle);
                //             double xy_ttc = getTTC(*iter, ego_vehicle);
                //             double dist_confidence = PEDESTRIAN_DISTANCE/xy_distance*0.7; 
                //             double ttc_confidence =  PEDESTRIAN_TTC/xy_ttc*0.7;
                //             if (ttc_confidence > dist_confidence)
                //                 final_confidence = ttc_confidence;
                //             else    
                //                 final_confidence = dist_confidence;

                //             if (final_confidence > CONFIDENCE_THRESHOLD || final_confidence == CONFIDENCE_THRESHOLD)
                //                 risk_assessment.emplace_back(RiskAssessment(iter->obstacle_id, PEDESTRIAN_HAZARD, 1, final_confidence));
                //             else
                //                 risk_assessment.emplace_back(RiskAssessment(iter->obstacle_id, PEDESTRIAN_HAZARD, 0, final_confidence));
                //         } break;
                //     }
                // }

                //dummy output 
                riskAssessment.hazard_index.clear();
                riskAssessment.hazard_index.push_back("BLIND_SPOT");

                riskAssessment.confidence.clear();
                riskAssessment.confidence.push_back(0.9);
                // adcm::Log::Info() << "Data sent (hazard index):  " << riskAssessment.hazard_index[0];
                riskAssessment_provider.send(riskAssessment);
            }
        }
    }   
}


void ThreadMonitor()
{
    while(continueExecution) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if(gMainthread_Loopcount == 0) {
            adcm::Log::Error() << "Main thread Timeout!!!";

        } else {
            gMainthread_Loopcount = 0;

            if(gReceivedEvent_count_map_data != 0) {
                adcm::Log::Info() << "map_data Received count = " << gReceivedEvent_count_map_data;
                gReceivedEvent_count_map_data = 0;

            } else {
                adcm::Log::Info() << "map_data event timeout!!!";
            }
        }
    }
}


int main(int argc, char* argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    if(!ara::core::Initialize()) {
        // No interaction with ARA is possible here since initialization failed
        return EXIT_FAILURE;
    }

    ara::exec::ExecutionClient exec_client;
    exec_client.ReportExecutionState(ara::exec::ExecutionState::kRunning);

    if(!RegisterSigTermHandler()) {
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
    adcm::Log::Info() << "Thread join";
    act1.join();
    monitor.join();
    adcm::Log::Info() << "done.";

    if(!ara::core::Deinitialize()) {
        // No interaction with ARA is possible here since some ARA resources can be destroyed already
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
