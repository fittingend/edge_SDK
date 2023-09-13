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

#include <ara/com/e2exf/status_handler.h>
#include <ara/exec/execution_client.h>

#include <ara/log/logger.h>
#include "logger.h"
#include "ara/core/initialization.h"

#include "risk_assessment_provider.h"
#include "map_data_subscriber.h"

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
                adcm::Log::Verbose() << "obstacle.fused_index : "<< obstacle.fused_index;
                adcm::Log::Verbose() << "obstacle.fused_cuboid_x : "<< obstacle.fused_cuboid_x;
                adcm::Log::Verbose() << "obstacle.fused_cuboid_y : "<< obstacle.fused_cuboid_y;
                adcm::Log::Verbose() << "obstacle.fused_cuboid_z : "<< obstacle.fused_cuboid_z;
                adcm::Log::Verbose() << "obstacle.fused_heading_angle : "<< obstacle.fused_heading_angle;
                adcm::Log::Verbose() << "obstacle.fused_Position_x : "<< obstacle.fused_Position_x;
                adcm::Log::Verbose() << "obstacle.fused_Position_y : "<< obstacle.fused_Position_y;
                adcm::Log::Verbose() << "obstacle.fused_Position_z : "<< obstacle.fused_Position_z;
                adcm::Log::Verbose() << "obstacle.fused_velocity_x : "<< obstacle.fused_velocity_x;
                adcm::Log::Verbose() << "obstacle.fused_velocity_y : "<< obstacle.fused_velocity_y;
                adcm::Log::Verbose() << "obstacle.fused_velocity_z : "<< obstacle.fused_velocity_z;

                adcm::Log::Verbose() << "environment.road_z : "<< environment.road_z;

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
            adcm::risk_assessment_Objects riskAssessment;

            riskAssessment.hazard_index.clear();
            riskAssessment.hazard_index.push_back("hazard_index_1");
            riskAssessment.hazard_index.push_back("hazard_index_2");
            riskAssessment.confidence.clear();
            riskAssessment.confidence.push_back(m_ud_100_100(m_rand_eng));
            riskAssessment.confidence.push_back(m_ud_100_100(m_rand_eng));

            riskAssessment_provider.send(riskAssessment);
            

/*1. obstacle table 확인해 obstacle (이미 알고있는 hazard) 
2. 2d_map 값 확인해서 현재 vehicle 의 위치와 비교
3. 상대속도 및 angle 계산해 충돌이 일정시간 이내에 일어날 경우 confidence 값 지정 

float collision_angle (ego_vehicle's angle, obstacle's angle)
{
    calcualte collision confidernce
    ...
    return collusion_angle_confidernce_value;
}

if (collusion_confidernce_value > certain_value)
그러면 collision speed function 불러서 충돌 시간 계산
float collision_time (ego_vehicle's speed, obstacle's speed)
{
    distance_between_ego_obstacle 구하고
    speed 비교해서 
    expected collision time 에 따라
    return collision_time_confidence_value;

}
두개 값 합산해 confidence 계산 


4. obstacle 종류에 따른 위험 요소 예) 사각지대 => 에 따른 후속 action 이 취해져야 함
if obstacle.index == "장애물"
if (obstacle.zvalue > certain value)
    bool obstacle.blindspot = TRUE;


==========사람이 등장! map_2d 및 obstacle table 값이 업데이트됨============

if(obstable_table_updated)

1. 사람 obstacle 추가된것을 확인 (새로운 hazard)
2. 위와 마찬가지로 상대속도 및 angle 계산해 충돌이 일정시간 이내에 일어날 경우 confidence 값 지정 
if (obstacle.index == "사람")
    bool obstacle.warning = TRUE;
3. 후속 action 을 결정할 수 있게 됨 




================= kimnh =================
> 보조차량 선행 -> map 제작
> Condition : 속도 30km/h, 정지 장애물, 사각지대, 사람
> map 실시간 업데이트

1. 초기 map 제작
1-1) 보조차량 2대가 목적지까지 직진 -> 정지 장애물 발견
    if(index == 4){ // 구조물 = 4
        boolean blindSpotFlag = FALSE;
        map_2d_location = int array[1000, 500] ?
        action_required = 2; // 접근해제 = 2
        fused_cuboid_x = 20;
        fused_Position_x = 1000; // cm 기준
        fused_Position_y = 500; // cm 기준
        fused_velocity_x = 0;
        fused_velocity_y = 0;
        fused_velocity_z = 0;
    }
    // 보조차량과 구조물 사이 거리가 2~5m 이내면 선회, 구조물과의 거리는 1~3m 유지
1-2) 정지 장애물 선회 후 목적지(1800, 500)까지 이동
1-3) 보조차량 2대 복귀
1-4) 초기 map 생성

2. 편대 출발 및 보조차량 사각지대 진입
2-1) 편대 출발
2-2) 정지 장애물과의 거리, 각도, 속도(?) 가 어떤 조건이 되면 편대 정지 -> confidence 증가
    if(distance between vehicle and obstacle < 1~3m || angle difference between vehicle and obstacle < degree){
        vehicle velocity = 0;
        confidence++;
    }
2-3) 보조차량 사각지대 진입 -> 사각지대 flag on
        blindSpotFlag = TRUE;
2-4) 보조차량 사각지대 제거 후 위치 고정 (사각지대와 거리는 1~3m) -> 사각지대 flag off -> confidence 감소
        blindSpotFlag = FALSE;
        confidence--;
        vehicle velocity = 0; // 보조차량 속도 0

3. 편대 다시 출발 및 사람 등장
3-1) 편대 출발 -> 사각지대에 있는 2번 보조차량과 합류
        vehicle velocity = 30;
3-2) 사람 등장 -> map, obstacle list 업데이트 -> 특장차와 사람 사이 거리가 20m 이내면 위험 -> confidence 증가
    if(distance between vehicle and pedestrian < 10~20m){
        index = 3; // 보행자 = 3
        confidence++;
        warningFlag = TRUE;
    }
3-3) 2번 보조차량 사람 앞으로 이동 및 경고 warning flag on, 나머지 차량 목적지까지 이동
        warningFlag = TRUE;
3-4) 사람이 안전한 곳으로 이동 (2번 보조차량 사람과 함께 이동) -> warning flag off, obstacle list 업데이트
        warningFlag = FALSE;

4. 사람 이동 및 편대 목적지까지 이동
4-1) 특장차 및 나머지 보조차량 편대 목적지까지 도착
4-2) 2번 보조차량은 사람이 안전구역에 간 것을 확인한 뒤 목적지에 있는 편대로 합류
        vehicle velocity = 30;

*/            
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
