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


VehicleData main_vehicle_temp;
VehicleData sub1_vehicle_temp;
VehicleData sub2_vehicle_temp;
std::vector <ObstacleData> obstacle_list_temp;
void globalToLocalcoordinate (VehicleData& vehicle)
{
    //시뮬레이션의 global 좌표계를 작업환경 XY 기반의 local 좌표계로 변환하는 함수
    double alpha = 537.92;
    double beta = -416.58;
    double theta= vehicle.yaw * M_PI / 180; 
    double velocity_ang = vehicle.velocity_ang;
    double position_x = vehicle.position_long;
    double position_y = vehicle.position_lat; 
    double velocity_x = vehicle.velocity_long;
    double velocity_y = vehicle.velocity_lat;

    vehicle.position_x =  cos(theta)*(position_x - alpha) + sin(theta)*(position_y - beta);
    vehicle.position_y = -sin(theta)*(position_x - alpha) + cos(theta)*(position_y - beta);
    vehicle.velocity_x = (velocity_ang * (-sin(theta)*(position_x - alpha)+ (cos(theta)*(position_y - beta))))\
        + (velocity_x* cos(theta)) + (velocity_y * sin(theta));
    vehicle.velocity_y = (velocity_ang * (-cos(theta)*(position_x-alpha) -(sin(theta)*(position_y - beta))))\
        + (velocity_x* -sin(theta)) + (velocity_y* cos(theta)); 
    
    vehicle.yaw = vehicle.yaw - theta;
    adcm::Log::Info() << "차량 globalToLocalcoordinate 좌표변환 (" << vehicle.position_x <<" , " <<vehicle.position_y << " , " << vehicle.velocity_x << " , " <<vehicle.velocity_y <<")"; 
    adcm::Log::Info() << "차량 globalToLocalcoordinate timestamp: " << vehicle.timestamp; 

}
void globalToLocalcoordinate (std::vector <ObstacleData>& obstacle_list, VehicleData main_vehicle)
{
    //시뮬레이션의 global 좌표계를 작업환경 XY 기반의 local 좌표계로 변환하는 함수
    double alpha = 537.92;
    double beta = -416.58;
    double velocity_ang = main_vehicle.velocity_ang;
    double theta= main_vehicle.yaw * M_PI / 180; 

    for (auto iter=obstacle_list.begin(); iter!=obstacle_list.end(); iter++)
    {   
        double position_x = iter->fused_position_x;
        double position_y = iter->fused_position_y;
        double velocity_x = iter->fused_velocity_x;
        double velocity_y = iter->fused_velocity_y;

        iter->fused_position_x = cos(theta)*(position_x - alpha) + sin(theta)*(position_y - beta);
        iter->fused_position_y = -sin(theta)*(position_x - alpha) + cos(theta)*(position_y - beta);

        iter->fused_velocity_x = (velocity_ang * (-sin(theta)*(position_x - alpha)+ (cos(theta)*(position_y - beta))))\
        + (velocity_x* cos(theta)) + (velocity_y * sin(theta));

        iter->fused_velocity_y = (velocity_ang * (-cos(theta)*(position_x-alpha) -(sin(theta)*(position_y - beta))))\
        + (velocity_x* -sin(theta)) + (velocity_y* cos(theta)); 

        iter->fused_heading_angle = iter->fused_heading_angle - main_vehicle.yaw;

        adcm::Log::Info() << "장애물 globalToLocalcoordinate 좌표변환 (" << iter->fused_position_x <<" , " <<iter->fused_position_y << " , " << iter->fused_velocity_x << " , " << iter->fused_velocity_y << ")"; 
        adcm::Log::Info() << "해당 timestamp: " << iter->timestamp;
    }   
    
}
void relativeToGlobalcoordinate (std::vector <ObstacleData>& obstacle_list, VehicleData main_vehicle)
{
    double theta = main_vehicle.yaw * M_PI /180;
    double velocity_ang = main_vehicle.velocity_ang;

    for (auto iter= obstacle_list.begin(); iter!=obstacle_list.end(); iter++)
    {
        double obstacle_position_x = iter->fused_position_x;
        double obstacle_position_y = iter->fused_position_y;
        double obstacle_velocity_x = iter->fused_velocity_x;
        double obstacle_velocity_y = iter->fused_velocity_y;

        iter->fused_position_x = main_vehicle.position_long + (obstacle_position_x)*cos(theta) - (obstacle_position_y)*sin(theta);
        iter->fused_position_y = main_vehicle.position_lat + (obstacle_position_x)*sin(theta) - (obstacle_position_y)*cos(theta);

        iter->fused_velocity_x = main_vehicle.velocity_long +(obstacle_velocity_x*cos(theta)) - (velocity_ang*obstacle_position_x*sin(theta))\
        - ((obstacle_velocity_y)*sin(theta)) - (velocity_ang* obstacle_position_y * cos(theta));

        iter->fused_velocity_y = main_vehicle.velocity_lat + (obstacle_velocity_x *sin(theta)) + (velocity_ang*obstacle_position_x* cos(theta))\
        + (obstacle_velocity_y*cos(theta)) - (velocity_ang*obstacle_position_y*sin(theta));

        iter->fused_heading_angle = main_vehicle.yaw + iter->fused_heading_angle;

        adcm::Log::Info() << "장애물 relativeToGlobal 좌표변환 (" << iter->fused_position_x <<" , " <<iter->fused_position_y << " , " << iter->fused_velocity_x << " , " << iter->fused_velocity_y << ")"; 
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
    //adcm::MapData_Provider mapData_provider;
    adcm::HubData_Subscriber hubData_subscriber;
    INFO("DataFusion .init()");
    //mapData_provider.init("DataFusion/DataFusion/PPort_map_data");
    hubData_subscriber.init("DataFusion/DataFusion/RPort_hub_data");
    INFO("After DataFusion .init()");
    INFO("Thread loop start...");

    while (continueExecution) {
        gMainthread_Loopcount++;
        VERBOSE("[DataFusion] Application loop");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool hubData_rxEvent = hubData_subscriber.waitEvent(0); // wait event

        if(hubData_rxEvent) {
            adcm::Log::Verbose() << "[EVENT] DataFusion Hub Data received";

            while(!hubData_subscriber.isEventQueueEmpty()) {
                auto data = hubData_subscriber.getEvent();
                gReceivedEvent_count_hub_data++;

                 //수신된 데이터 handling 위한 추가 코드 
                switch (data->vehicle_class)
                {
                    case 0: //특장차가 보낸 인지데이터

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
                        for (int i=0; i < data->obstacle.size(); i++)
                        {
                            ObstacleData obstacle_to_push;
                            obstacle_to_push.obstacle_class = data->obstacle[i].obstacle_class;
                            obstacle_to_push.timestamp= data->timestamp;
                            obstacle_to_push.fused_cuboid_x= data->obstacle[i].cuboid_x;
                            obstacle_to_push.fused_cuboid_y= data->obstacle[i].cuboid_y;
                            obstacle_to_push.fused_cuboid_z= data->obstacle[i].cuboid_z;
                            obstacle_to_push.fused_heading_angle= data->obstacle[i].heading_angle;
                            obstacle_to_push.fused_position_x= data->obstacle[i].position_x;
                            obstacle_to_push.fused_position_y= data->obstacle[i].position_y;
                            obstacle_to_push.fused_position_z= data->obstacle[i].position_z;
                            obstacle_to_push.fused_velocity_x= data->obstacle[i].velocity_x*KM_TO_MS_CONVERSION;
                            obstacle_to_push.fused_velocity_y= data->obstacle[i].velocity_y*KM_TO_MS_CONVERSION;
                            obstacle_to_push.fused_velocity_z= data->obstacle[i].velocity_z*KM_TO_MS_CONVERSION;
                            obstacle_list_temp.push_back(obstacle_to_push);
                        }

                        adcm::Log::Info() << "main vehicle data received";
                        break;

                    case 1: //보조차1이 보낸 인지데이터
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

                    case 2: //보조차2가 보낸 인지데이터 
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
    }
}

void ThreadKatech()
{
    //==============1.전역변수인 MapData 생성 =================
    adcm::map_data_Objects mapData;
    //한번 생성후 관제에서 인지데이터를 받을때마다 (100ms) 마다 업데이트  
    adcm::Log::Info() << "mapData created for the first time";

    ::adcm::map_2dListStruct map_2dStruct_init;
    map_2dStruct_init.obstacle_id = 0; 
    map_2dStruct_init.road_z = 0;
    map_2dStruct_init.vehicle_class = 0;

    std::vector<adcm::map_2dListVector> map_2d_test (map_n, adcm::map_2dListVector(map_m, map_2dStruct_init));
    adcm::Log::Info() << "mapData 2d info initialized";

    VehicleData main_vehicle;
    VehicleData sub1_vehicle;
    VehicleData sub2_vehicle;
    std::vector <ObstacleData> obstacle_list;

    while(continueExecution)
    {
        adcm::Log::Info() << "==============KATECH modified code start==========";
        main_vehicle = main_vehicle_temp;
        sub1_vehicle = sub1_vehicle_temp;
        sub2_vehicle = sub2_vehicle_temp;
        obstacle_list = obstacle_list_temp;

        adcm::map_2dListVector map_2dListVector;
        adcm::map_2dListStruct map_2dStruct;
        adcm::Log::Info() << "mapData obstacle list size is at start is" <<mapData.obstacle_list.size();

        //==============1. obstacle 로 인지된 sub 차량 제거 (OK)=================
        for (auto iter = obstacle_list.begin(); iter != obstacle_list.end();)
        {
            if ((abs(iter->fused_cuboid_x - SUB_VEHICLE_SIZE_X)) < 0.1 && (abs(iter->fused_cuboid_y - SUB_VEHICLE_SIZE_Y)) < 0.1)
            {
                iter = obstacle_list.erase(iter);
            }
            else iter++;
        }
        adcm::Log::Info() <<  "obstacle without sub-vehicle size is " << obstacle_list.size();
        
       //==============2. obstacle ID assignment =================
        for (auto iter1 = mapData.obstacle_list.begin(); iter1 != mapData.obstacle_list.end(); iter1++)
        {
            adcm::Log::Info() <<  "previous obstacle saved in the mapData!" << iter1->obstacle_id;
        }
        std::vector <ObstacleData> obstacle_list_filtered;
        obstacle_list_filtered.clear();

        if (mapData.obstacle_list.empty()) 
        {
            //최초 obstacle ID assignment 진행. random하게 assign 한다
            //트래킹하는 알고리즘 필요 *추후 보완
            int i = 1;
            for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
            {
                iter->obstacle_id = iter->timestamp +i; 
                i++;
                //타임스탬프 값으로 장애물 id assign
                adcm::Log::Info() << "obstacle assigned is " << iter->obstacle_id;
                obstacle_list_filtered.push_back(*iter);
            }
        }
        else //이미 obstacle list 존재하는 경우는 obstacle id 비교가 필요하다
        {
            for (auto iter = mapData.obstacle_list.begin(); iter!=mapData.obstacle_list.end(); iter++)
            { 
                for (auto iter1 = obstacle_list.begin(); iter1 != obstacle_list.end();)
                {
                    //사이즈 오차 10cm 이내면 동일 장애물이라 본다
                    if ((ABS(iter->fused_cuboid_x - iter1->fused_cuboid_x)< 0.1) && (ABS(iter->fused_cuboid_y - iter1->fused_cuboid_y)< 0.1) && (ABS(iter->fused_cuboid_z - iter1->fused_cuboid_z)<0.1))
                    {
                        adcm::Log::Info() << "Identical obstacle detected : "<< iter->obstacle_id;
                        iter1->obstacle_id = iter->obstacle_id;
                        obstacle_list_filtered.push_back(*iter1);
                        iter1 = obstacle_list.erase(iter1);
                    }
                    else //동일하지 않은 장애물에는 temp value 인 99 으로 id를 우선 지정
                    {
                        iter1->obstacle_id = INVALID_VALUE;      
                        adcm::Log::Info() << " New obstacle appeared OR tracking failed ";
                        ++iter1;
                    }
                }
            }
            //새로운 객체들에 대한 id 지정 (INVALID_VALUE 를 실제 아이디로 지정)
            for (auto iter1 = obstacle_list.begin(); iter1 != obstacle_list.end(); iter1++)
            {
                srand(time(NULL));
                unsigned short rand_id = (unsigned short) rand();
                iter1->obstacle_id = rand_id;
                //TO DO:id 중복여부 duplicate 체크 해야함 *추후 수정
                obstacle_list_filtered.push_back(*iter1);
            }
        }
        obstacle_list.clear();

        for (auto iter1 = obstacle_list_filtered.begin(); iter1 != obstacle_list_filtered.end(); iter1++)
        {
            adcm::Log::Info() <<  "obstacle filtered are: " << iter1->obstacle_id;
        }

        //==============3. 장애물 좌표계 변환=================
        relativeToGlobalcoordinate(obstacle_list_filtered, main_vehicle);
        globalToLocalcoordinate (obstacle_list_filtered, main_vehicle);

        //==============4. 차량 좌표계 변환==================================
        //차량마다 받은 글로벌 좌표를 작업공간 local 좌표계로 변환

        globalToLocalcoordinate(main_vehicle); 
        globalToLocalcoordinate(sub1_vehicle);
        globalToLocalcoordinate(sub2_vehicle);
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
    std::thread katechThread(ThreadKatech);
    adcm::Log::Info() << "Thread join";
    act1.join();
    monitor.join();
    katechThread.join();
    adcm::Log::Info() << "done.";

    if(!ara::core::Deinitialize()) {
        // No interaction with ARA is possible here since some ARA resources can be destroyed already
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
