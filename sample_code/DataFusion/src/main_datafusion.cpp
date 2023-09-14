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

#include <set>
#include <vector>
#include <array>

#include <ara/com/e2exf/status_handler.h>
#include <ara/exec/execution_client.h>

#include <ara/log/logger.h>
#include "logger.h"
#include "ara/core/initialization.h"

#include "map_data_provider.h"
#include "build_path_subscriber.h"
#include "hub_data_subscriber.h"

#include "main_datafusion.hpp"

namespace
{

// Atomic flag for exit after SIGTERM caught
std::atomic_bool continueExecution{true};
std::atomic_uint gReceivedEvent_count_build_path{0};
std::atomic_uint gReceivedEvent_count_hub_data{0};
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


}  // namespace

void ThreadAct1()
{
    adcm::Log::Info() << "DataFusion ThreadAct1";
    adcm::MapData_Provider mapData_provider;
    adcm::BuildPath_Subscriber buildPath_subscriber;
    adcm::HubData_Subscriber hubData_subscriber;
    INFO("DataFusion .init()");

    mapData_provider.init("DataFusion/DataFusion/PPort_map_data");
    buildPath_subscriber.init("DataFusion/DataFusion/RPort_build_path");
    hubData_subscriber.init("DataFusion/DataFusion/RPort_hub_data");
    INFO("After DataFusion .init()");
    std::random_device m_rd;
    std::default_random_engine m_rand_eng(m_rd());
    std::uniform_real_distribution<double> m_ud_10000_10000(-10000, 10000);
    std::uniform_int_distribution<std::uint32_t> m_ud_0_10000(0, 10000);
    std::uniform_int_distribution<std::uint8_t> m_ud_0_4(0, 4);
    INFO("Thread loop start...");

    while (continueExecution) {
        gMainthread_Loopcount++;
        VERBOSE("[DataFusion] Application loop");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool buildPath_rxEvent = buildPath_subscriber.waitEvent(0); // wait event
        bool hubData_rxEvent = hubData_subscriber.waitEvent(0); // wait event

        if(buildPath_rxEvent) {
            adcm::Log::Verbose() << "[EVENT] DataFusion Build Path received";

            while(!buildPath_subscriber.isEventQueueEmpty()) {
                auto data = buildPath_subscriber.getEvent();
                gReceivedEvent_count_build_path++;

                auto Seq = data->Seq;
                auto group_id = data->group_id;
                auto vehicle_id = data->vehicle_id;
                auto mve_id = data->mve_id;
                auto mve_type = data->mve_type;
                auto sec = data->sec;
                auto nsec = data->nsec;
                auto frame_id = data->frame_id;
                auto size = data->size;
                auto utm_x = data->utm_x;
                auto utm_y = data->utm_y;

                adcm::Log::Verbose() << "Seq : " << Seq;
                adcm::Log::Verbose() << "group_id : " << group_id;
                adcm::Log::Verbose() << "vehicle_id : " << vehicle_id;
                adcm::Log::Verbose() << "mve_id : " << mve_id;
                adcm::Log::Verbose() << "mve_type : " << mve_type;
                adcm::Log::Verbose() << "sec : " << sec;
                adcm::Log::Verbose() << "nsec : " << nsec;
                adcm::Log::Verbose() << "frame_id : " << frame_id;
                adcm::Log::Verbose() << "size : " << size;
                
                if(!utm_x.empty()) {
                    adcm::Log::Verbose() << "=== utm_x ===";
                    for(auto itr = utm_x.begin(); itr != utm_x.end(); ++itr) {
                        adcm::Log::Verbose() << *itr;
                    }
                } else {
                    adcm::Log::Verbose() << "utm_x Vector empty!!! ";
                }

                if(!utm_y.empty()) {
                    adcm::Log::Verbose() << "=== utm_y ===";
                    for(auto itr = utm_y.begin(); itr != utm_y.end(); ++itr) {
                        adcm::Log::Verbose() << *itr;
                    }
                } else {
                    adcm::Log::Verbose() << "utm_y Vector empty!!! ";
                }
            }
        }

        if(hubData_rxEvent) {
            adcm::Log::Verbose() << "[EVENT] DataFusion Hub Data received";

            while(!hubData_subscriber.isEventQueueEmpty()) {
                auto data = hubData_subscriber.getEvent();
                gReceivedEvent_count_hub_data++;

                auto obstacle = data->obstacle;
                auto environment = data->environment;
                auto vehicle = data->vehicle;

                adcm::Log::Verbose() << "obstacle.Time_stamp : "<< obstacle.Time_stamp;
                adcm::Log::Verbose() << "obstacle.index : "<< obstacle.index;
                adcm::Log::Verbose() << "obstacle.cuboid_x : "<< obstacle.cuboid_x;
                adcm::Log::Verbose() << "obstacle.cuboid_y : "<< obstacle.cuboid_y;
                adcm::Log::Verbose() << "obstacle.cuboid_z : "<< obstacle.cuboid_z;
                adcm::Log::Verbose() << "obstacle.heading_angle : "<< obstacle.heading_angle;

                auto covariance_matrix = obstacle.covariance_matrix;

                if(!covariance_matrix.empty()) {
                    adcm::Log::Verbose() << "=== covariance_matrix ===";
                    for(auto itr = covariance_matrix.begin(); itr != covariance_matrix.end(); ++itr) {
                        adcm::Log::Verbose() << *itr;
                    }
                } else {
                    adcm::Log::Verbose() << "covariance_matrix Vector empty!!! ";
                }

                adcm::Log::Verbose() << "obstacle.Position_x : "<< obstacle.Position_x;
                adcm::Log::Verbose() << "obstacle.Position_y : "<< obstacle.Position_y;
                adcm::Log::Verbose() << "obstacle.Position_z : "<< obstacle.Position_z;
                adcm::Log::Verbose() << "obstacle.Velocity_x : "<< obstacle.Velocity_x;
                adcm::Log::Verbose() << "obstacle.Velocity_y : "<< obstacle.Velocity_y;
                adcm::Log::Verbose() << "obstacle.Velocity_z : "<< obstacle.Velocity_z;

                auto road_z = environment.road_z;

                if(!road_z.empty()) {
                    adcm::Log::Verbose() << "=== environment.road_z ===";
                    for(auto itr = road_z.begin(); itr != road_z.end(); ++itr) {
                        adcm::Log::Verbose() << *itr;
                    }
                } else {
                    adcm::Log::Verbose() << "environment.road_z Vector empty!!! ";
                }

                adcm::Log::Verbose() << "vehicle.Vehicle_id : "<< vehicle.Vehicle_id;
                adcm::Log::Verbose() << "vehicle.Position_lat : "<< vehicle.Position_lat;
                adcm::Log::Verbose() << "vehicle.Position_long : "<< vehicle.Position_long;
                adcm::Log::Verbose() << "vehicle.Position_Height : "<< vehicle.Position_Height;
                adcm::Log::Verbose() << "vehicle.Yaw : "<< vehicle.Yaw;
                adcm::Log::Verbose() << "vehicle.Roll : "<< vehicle.Roll;
                adcm::Log::Verbose() << "vehicle.Pitch : "<< vehicle.Pitch;
                adcm::Log::Verbose() << "vehicle.Velocity_long : "<< vehicle.Velocity_long;
                adcm::Log::Verbose() << "vehicle.Velocity_lat : "<< vehicle.Velocity_lat;
                adcm::Log::Verbose() << "vehicle.Velocity_ang : "<< vehicle.Velocity_ang;
            }
        }

        {
            adcm::map_data_Objects mapData;

            mapData.obstacle.Time_stamp = "Time_stamp";
            mapData.obstacle.fused_index = "fused_index";
            mapData.obstacle.fused_cuboid_x = m_ud_10000_10000(m_rand_eng);
            mapData.obstacle.fused_cuboid_y = m_ud_10000_10000(m_rand_eng);
            mapData.obstacle.fused_cuboid_z = m_ud_10000_10000(m_rand_eng);
            mapData.obstacle.fused_heading_angle = m_ud_10000_10000(m_rand_eng);
            mapData.obstacle.fused_Position_x = m_ud_10000_10000(m_rand_eng);
            mapData.obstacle.fused_Position_y = m_ud_10000_10000(m_rand_eng);
            mapData.obstacle.fused_Position_z = m_ud_10000_10000(m_rand_eng);
            mapData.obstacle.fused_velocity_x = m_ud_10000_10000(m_rand_eng);
            mapData.obstacle.fused_velocity_y = m_ud_10000_10000(m_rand_eng);
            mapData.obstacle.fused_velocity_z = m_ud_10000_10000(m_rand_eng);

            mapData.environment.road_z = m_ud_10000_10000(m_rand_eng);

            mapData.vehicle.Vehicle_id = m_ud_0_10000(m_rand_eng);
            mapData.vehicle.Position_lat = m_ud_10000_10000(m_rand_eng);
            mapData.vehicle.Position_long = m_ud_10000_10000(m_rand_eng);
            mapData.vehicle.Position_Height = m_ud_10000_10000(m_rand_eng);
            mapData.vehicle.Yaw = m_ud_10000_10000(m_rand_eng);
            mapData.vehicle.Roll = m_ud_10000_10000(m_rand_eng);
            mapData.vehicle.Pitch = m_ud_10000_10000(m_rand_eng);
            mapData.vehicle.Velocity_long = m_ud_10000_10000(m_rand_eng);
            mapData.vehicle.Velocity_lat = m_ud_10000_10000(m_rand_eng);
            mapData.vehicle.Velocity_ang = m_ud_10000_10000(m_rand_eng);

            mapData_provider.send(mapData);
#ifdef MAP_GENERATION


//어떤 셀에 해당하는 obstacle 데이터인지 식별
//which cell(s) does this obstacle belong to?

/* 
==============1.데이터 융합=================
메인/보조 차량에서 오는 데이터 융합
9월 워크샵때는 차량 1대만 고려하기 때문에 융합 부분은 구현 X 
받은 데이터 그대로 전달하면 OK 
*/

//==============2.map_2d 생성 =================
            map_data_type map_data; 

/*<실제코드>========================
            int fused_cuboid_x_start = static_cast<int> (mapData.obstacle.fused_Position_x - (mapData.obstacle.fused_cuboid_x/2));
            int fused_cuboid_x_end = static_cast<int> (mapData.obstacle.fused_Position_x + (mapData.obstacle.fused_cuboid_x/2));

            int fused_cuboid_y_start = static_cast<int> (mapData.obstacle.fused_Position_y - (mapData.obstacle.fused_cuboid_y/2));
            int fused_cuboid_y_end = static_cast<int> (mapData.obstacle.fused_Position_y + (mapData.obstacle.fused_cuboid_y/2));
*/ 
//<테스트용 코드 - obstacle 있는 cell 만 업데이트>=======================
// obstacle_type이 구조물이고 +  fused_cuboid_z 가 1m 이상(현재 임의로 지정)이여서 사각지대 가능성 발생 시나리오
            double fused_Position_x = 500; //50m
            double fused_Position_y = 1000; //100m
            double fused_cuboid_x = 200; //200m
            double fused_cuboid_y = 100; //임의지정 10m
            double fused_cuboid_z = 10;  //임의지정 1m
//10cm 가 1일때 
//1m 는 10 의 값을 지님 

            int fused_cuboid_x_start = static_cast<int> (fused_Position_x - (fused_cuboid_x/2));
            int fused_cuboid_x_end = static_cast<int> (fused_Position_x + (fused_cuboid_x/2));

            int fused_cuboid_y_start = static_cast<int> (fused_Position_y - (fused_cuboid_y/2));
            int fused_cuboid_y_end = static_cast<int> (fused_Position_y + (fused_cuboid_y/2));

//해당 map cell 에다 데이터 집어 넣기      
            int count = 0;
            adcm::Log::Info() << "i is from  " << fused_cuboid_y_start << " to " << fused_cuboid_y_end;
            adcm::Log::Info() << "j is from " <<  fused_cuboid_x_start << " to "<< fused_cuboid_x_end;
            for (int i=fused_cuboid_y_start; i<fused_cuboid_y_end+1; i++)
            {
                for (int j=fused_cuboid_x_start; j< fused_cuboid_x_end+1; j++)
                {
//                    map_data.map_2d[i][j].timestamp = ""; //이미 정의된 string 을 = 로 다시 지정 불가능. 베이리스 기다리기
                    map_data.map_2d[i][j].fused_index = {STRUCTURE};
                    map_data.map_2d[i][j].fused_cuboid_x = fused_cuboid_x; 
                    map_data.map_2d[i][j].fused_cuboid_y = fused_cuboid_y;
                    map_data.map_2d[i][j].fused_cuboid_z = fused_cuboid_z;
                    map_data.map_2d[i][j].fused_Position_x = fused_Position_x;
                    map_data.map_2d[i][j].fused_Position_x = fused_Position_y;
                    count++;
                    adcm::Log::Info() << "Map_2d is updated with "  << count << " times";
                    //등의 데이터 assignment 
                }
            }

//==============3.메인/보조차량 정보 업데이트=================

            map_data.vehicle_list[0].vehicle_id={EGO_VEHICLE};
 //....등등 1. 융합데이터에서 받은 정보 그대로 assign           
   



//            mapData_provider.send(map_data);//다른 모듈로 map_data 오브젝트 전달

//=============4.obstacle list 생성 =============
// 생성된 2d-map 을 기반으로 obstacle list 를 생성

            std::vector<obstacle_list_data_type> obstacle_list;
            std::vector<map_2d_data_type> unique_map;

            for (int i=0; i<n;i++)
            {
                for (int j=0; j<m; j++)
                {
                    if (map_data.map_2d[i][j].fused_index != NO_OBSTACLE) //obstacle 있는 지도 값인 경우에 unique_map 에 저장
                        unique_map.push_back(map_data.map_2d[i][j]); 
                }
            }

            //TO DO: obstacle_id 로 장애물 별 2d array 인덱스 값 구분해서 값도 저장해야함
            unique_map.erase(std::unique(unique_map.begin(), unique_map.end()), unique_map.end());
            //중복되는 값을 제거 

            for (auto iter = unique_map.begin(); iter!= unique_map.end();iter++)
            {
                switch(iter->fused_index){

                    case STRUCTURE: 
                        obstacle_list.emplace_back(obstacle_list_data_type(iter->obstacle_id, iter->fused_index, iter->timestamp, REMOVE_BLIND_SPOT,\
                        iter->fused_cuboid_x, iter->fused_cuboid_y, iter->fused_cuboid_z, iter->fused_heading_angle, iter->fused_Position_x,\
                        iter->fused_Position_y, iter->fused_Position_z, iter->fused_velocity_x, iter->fused_velocity_y, iter->fused_velocity_z));
                        break;

                    case PEDESTRIAN:
                        obstacle_list.emplace_back(obstacle_list_data_type(iter->obstacle_id, iter->fused_index, iter->timestamp, ALERT_OBSTACLE,\
                        iter->fused_cuboid_x, iter->fused_cuboid_y, iter->fused_cuboid_z, iter->fused_heading_angle, iter->fused_Position_x,\
                        iter->fused_Position_y, iter->fused_Position_z, iter->fused_velocity_x, iter->fused_velocity_y, iter->fused_velocity_z));
                        break;
                }
            }

//            xx_provider.send(obstacle_list);
//위험 판단 모듈로 리스트 전달


//TO DO:
//나중에 새로운 obstacle 정보가 수신 되었을때 지금 있는 obstacle 과 동일한 장애물이란걸 알아내는 알고리즘이 추가로 필요

#endif
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

            if(gReceivedEvent_count_build_path != 0) {
                adcm::Log::Info() << "build_path Received count = " << gReceivedEvent_count_build_path;
                gReceivedEvent_count_build_path = 0;

            } else {
                adcm::Log::Info() << "build_path event timeout!!!";
            }

            if(gReceivedEvent_count_hub_data != 0) {
                adcm::Log::Info() << "hub_data Received count = " << gReceivedEvent_count_hub_data;
                gReceivedEvent_count_hub_data = 0;

            } else {
                adcm::Log::Info() << "hub_data event timeout!!!";
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
    adcm::Log::Info() << "DataFusion: configure e2e protection";
    bool success = ara::com::e2exf::StatusHandler::Configure("./etc/e2e_dataid_mapping.json",
                   ara::com::e2exf::ConfigurationFormat::JSON,
                   "./etc/e2e_statemachines.json",
                   ara::com::e2exf::ConfigurationFormat::JSON);
    adcm::Log::Info() << "DataFusion: e2e configuration " << (success ? "succeeded" : "failed");
#endif
    adcm::Log::Info() << "Ok, let's produce some DataFusion data...";
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
