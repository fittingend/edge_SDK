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


//============== 함수 definition =================

double getDistance(ObstacleData obstacle, VehicleData vehicle)
{
    return sqrt(pow(obstacle.fused_position_x -vehicle.position_lat,2)+ pow(obstacle.fused_position_y -vehicle.position_long, 2));
}
double getDistance(double a_x, double a_y, double b_x, double b_y)
{
    return sqrt(pow(a_x-b_x,2)+ pow(a_y-b_y, 2));

}
double getTTC(ObstacleData obstacle, VehicleData vehicle)
{
    double ttc=0;
    double distance=0;
    float t = 0.1;
    double obstacle_final_pos_x, obstacle_final_pos_y, vehicle_final_pos_x, vehicle_final_pos_y;
    
    for (t; t < 10; t=t+0.05)
    {
        obstacle_final_pos_x = obstacle.fused_position_x + obstacle.fused_velocity_x*t;
        obstacle_final_pos_y = obstacle.fused_position_y + obstacle.fused_velocity_y*t;
        vehicle_final_pos_x = vehicle.position_long + vehicle.velocity_long*t;
        vehicle_final_pos_y = vehicle.position_lat + vehicle.velocity_lat*t;
        distance=getDistance(obstacle_final_pos_x,obstacle_final_pos_y,vehicle_final_pos_x,vehicle_final_pos_y);

        if (distance < COLLISION_DISTANCE)
        {    //현재 상대 거리가 3m 인 경우 충돌이 일어났다고 가정-이 시간을 ttc로 정의
            break; 
        }
    }
    ttc = t;
//현재 ttc가 10초가 넘어가면 그냥 10초로 정의한다 
    return ttc;
}  

//===============================

}  // namespace


//==============1. MapData 생성 =================
//TO DO: Datafusion 에서 제대로 받아오면 삭제
//현재 테스트용
MapData map_data;
//=============================================

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
                map_data.obstacle_list.push_back(current_obstacle);
                
                VehicleData current_vehicle;
                current_vehicle.vehicle_class = EGO_VEHICLE;
                current_vehicle.position_lat = 5;
                current_vehicle.position_lat = 3;
                map_data.vehicle_list.push_back(current_vehicle);

                VehicleData ego_vehicle, sub_vehicle_1, sub_vehicle_2,sub_vehicle_3,sub_vehicle_4;

                for (auto iter = map_data.vehicle_list.begin(); iter!=map_data.vehicle_list.end(); iter++)
                {
                    switch(iter->vehicle_class)
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

                for (auto iter = map_data.obstacle_list.begin(); iter!= map_data.obstacle_list.end();iter++)
                {
                    switch(iter->action_class)
                    {
                        case REMOVE_BLIND_SPOT:
                        {
                            float final_confidence = 0;
                            // risk_assessment.obstacle_id.push_back(current_obstacle.obstacle_id);
                            // risk_assessment.hazard_class.push_back(BLIND_SPOT);
                            // angle 포함 안시킨 경우
                            double xy_distance = getDistance(*iter, ego_vehicle);
                            final_confidence = STRUCTURE_DISTANCE / xy_distance * 0.7;
                            if (final_confidence > CONFIDENCE_THRESHOLD || final_confidence == CONFIDENCE_THRESHOLD)
                            {
                                risk_assessment.emplace_back(RiskAssessment(iter->obstacle_id, BLIND_SPOT, 1, final_confidence));
                            }
                            else
                            {
                                risk_assessment.emplace_back(RiskAssessment(iter->obstacle_id, BLIND_SPOT, 0, final_confidence));
                            }
                            adcm::Log::Info() << "confident for blindspot is " << final_confidence;
                            adcm::Log::Info() << "obstalce id is " << risk_assessment[0].obstacle_id;

                        } break;

                        case ALERT_OBSTACLE: 
                        {
                            //동적 장애물일 경우
                            float final_confidence = 0;
                            double xy_distance = getDistance(*iter, ego_vehicle);
                            double xy_ttc = getTTC(*iter, ego_vehicle);
                            double dist_confidence = PEDESTRIAN_DISTANCE/xy_distance*0.7; 
                            double ttc_confidence =  PEDESTRIAN_TTC/xy_ttc*0.7;
                            if (ttc_confidence > dist_confidence)
                                final_confidence = ttc_confidence;
                            else    
                                final_confidence = dist_confidence;

                            if (final_confidence > CONFIDENCE_THRESHOLD || final_confidence == CONFIDENCE_THRESHOLD)
                                risk_assessment.emplace_back(RiskAssessment(iter->obstacle_id, PEDESTRIAN_HAZARD, 1, final_confidence));
                            else
                                risk_assessment.emplace_back(RiskAssessment(iter->obstacle_id, PEDESTRIAN_HAZARD, 0, final_confidence));
                        } break;
                    }
                }

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
