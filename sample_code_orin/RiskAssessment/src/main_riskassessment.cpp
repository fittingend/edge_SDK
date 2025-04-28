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

#include "main_riskassessment.hpp"
#include "NATS/NatsHandler.hpp"

//define NATS

//전역변수
std::mutex mtx_map, mtx_rass, mtx_path, mtx_cv;
std::vector<adcm::map_2dListVector> map_2d;
//경로 path
doubleVector path_x;
doubleVector path_y;

obstacleListVector obstacle_list;
adcm::vehicleListStruct ego_vehicle, sub_vehicle_1, sub_vehicle_2, sub_vehicle_3, sub_vehicle_4;
adcm::risk_assessment_Objects riskAssessment;


bool isMapAvailable = false;
bool isPathAvailable = false;
std::condition_variable cv_mapData;

void ThreadReceiveMapData()
{
    adcm::MapData_Subscriber mapData_subscriber;
    mapData_subscriber.init("RiskAssessment/RiskAssessment/RPort_map_data");
    INFO("Thread ThreadReceiveMapData start...");

    while (continueExecution) {
        gMainthread_Loopcount++;
        VERBOSE("[RiskAssessment] Application loop");
        bool mapData_rxEvent = mapData_subscriber.waitEvent(100); // wait event

        if(mapData_rxEvent) {
            adcm::Log::Info() << "[EVENT] RiskAssessment Map Data received";

            while(!mapData_subscriber.isEventQueueEmpty()) {
                std::lock_guard<std::mutex> lock(mtx_map);
                auto data = mapData_subscriber.getEvent();
                gReceivedEvent_count_map_data++;

                //auto timestamp = data->timestamp;
                map_2d = data->map_2d;
                obstacle_list = data->obstacle_list; 
                //auto obstacle_list = data->obstacle_list;
                auto vehicle_list = data->vehicle_list;

                //adcm::Log::info() << "timestamp : " << timestamp;

                if(!map_2d.empty()) 
                {
                    adcm::Log::Info() << "size of map_2d received: " << map_2d.size() * map_2d[0].size();
                   } else {
                    adcm::Log::Info() << "map_2d Vector empty!!! ";
                }

                if(!obstacle_list.empty()) 
                {
                    adcm::Log::Info() << "size of obstacle list received: " << obstacle_list.size();
                    for(auto itr = obstacle_list.begin(); itr != obstacle_list.end(); ++itr) 
					{
                        adcm::Log::Info() << "obstacle_id : " << itr->obstacle_id;
                        adcm::Log::Info() << "obstacle_class : " << itr->obstacle_class;
                        adcm::Log::Info() << "timestamp : " << itr->timestamp;

                        //obstacle_list_temp = obstacle_list;//assign to gloabal variable 
                    }
                } else {
                    adcm::Log::Info() << "obstacle_list Vector empty!!! ";
                }

                if(!(vehicle_list.empty())) {
                    adcm::Log::Info() << "size of vehicle list received: " << vehicle_list.size();
                    for (auto iter = vehicle_list.begin(); iter != vehicle_list.end(); iter++)
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
                        default:
                            break;
                        }
                    }
                }
                else
                {
                    adcm::Log::Info() << "vehicle_list Vector empty!!! ";
                }

                {
                    std::lock_guard<std::mutex> lock(mtx_cv);
                    isMapAvailable = true;
                }
                cv_mapData.notify_one();  // 다른 스레드에게 알림
                adcm::Log::Info() << "다른 스레드에게 알림";

            }
        }
    }
}

void ThreadReceiveBuildPath()
{
    adcm::BuildPath_Subscriber buildPath_subscriber;
    buildPath_subscriber.init("RiskAssessment/RiskAssessment/RPort_build_path");
    INFO("Thread ThreadReceiveBuildPath start...");

    while (continueExecution) {
        gMainthread_Loopcount++;
        VERBOSE("[RiskAssessment] Application loop");
        bool buildPath_rxEvent = buildPath_subscriber.waitEvent(100); // wait event

        if(buildPath_rxEvent) {
            adcm::Log::Info() << "[EVENT] RiskAssessment Build Path received";

            while(!buildPath_subscriber.isEventQueueEmpty()) {
                auto data = buildPath_subscriber.getEvent();
                gReceivedEvent_count_build_path++;

                const auto& Path = data->Path;

                if (Path.empty()) {
                adcm::Log::Info() << "Received Path is empty.";
                continue;
                }

                for (const auto& pathItem : Path) {
                    const auto& route = pathItem.route;
                    if (pathItem.vehicle_class == EGO_VEHICLE)
                    {   
                        //if (isRouteValid(route))
                        //{
                            INFO("특장차의 경로 좌표변환 진행");
                            std::lock_guard<std::mutex> lock(mtx_path);
                            path_x.clear();
                            path_y.clear();
                            //새로운 경로를 받으면 예전 변환값이 담긴 path_x 와 path_y 초기화
                            gpsToMapcoordinate(route, path_x, path_y);
                            isPathAvailable = true;
                        //}
                        // else
                        // {
                        //     INFO("특장차의 경로가 invalid 함 => do nothing");
                        // }
                    }
                    for (const auto& point : route)
                    {
                        adcm::Log::Info() << "route.latitude : "<< point.latitude;
                        adcm::Log::Info() << "route.longitude : "<< point.longitude;
                        adcm::Log::Info() << "route.delta_t : "<< point.delta_t;
                    }
                }
            }
        }
    }
}

void ThreadRASS()
{
    adcm::Log::Info() << "Thread ThreadRASS start...";
    while (continueExecution)
    {
        std::unique_lock<std::mutex> lock(mtx_cv);
        cv_mapData.wait(lock, []{ return isMapAvailable && isPathAvailable;});  
        // 신호가 올 때까지 대기하다 map 데이터와 build path 수신하면 시작

        {
            std::lock_guard<std::mutex> mapLock(mtx_map);
            adcm::Log::Info() << "[PROCESS] Start processing received map data";
            riskAssessment.riskAssessmentList.clear();
            evaluateScenario1(obstacle_list, ego_vehicle, path_x, path_y, riskAssessment);
            evaluateScenario2(obstacle_list, ego_vehicle, path_x, path_y, riskAssessment);
            evaluateScenario3(obstacle_list, ego_vehicle, riskAssessment);
            evaluateScenario4(obstacle_list, ego_vehicle, riskAssessment);
            evaluateScenario5(obstacle_list, ego_vehicle, path_x, path_y, riskAssessment);
            evaluateScenario6(obstacle_list, ego_vehicle, path_x, path_y, riskAssessment);
            evaluateScenario7(path_x, path_y, map_2d, riskAssessment);
            evaluateScenario8(path_x, path_y, map_2d, riskAssessment);

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
/*            adcm::Log::Info() << "===============================================================";
            if (riskAssessment.riskAssessmentList.size() != 0)
            {
                //risktAssessment object 의 생성시간 추가
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
                //NatsSend(riskAssessment); //테스트 용도로 계속 송신하게 한다
                adcm::Log::Info() << "riskAssessment size is 0, do NOT send";
            }
            riskAssessment.riskAssessmentList.clear();
            */
    }
}

        
void ThreadSend()
{
    adcm::Log::Info() << "ThreadSend start...";

    int rassVer = 0; // 현재 위험판단이 몇 번째 값인지 확인
    adcm::RiskAssessment_Provider riskAssessment_provider;
    riskAssessment_provider.init("RiskAssessment/RiskAssessment/PPort_risk_assessment");
    // mutex, condition value 사용

    while (continueExecution)
    {
        {
            std::unique_lock<std::mutex> lock(mtx_rass);
            // rassReady.wait(lock, []
            //                { return sendEmptyMap == true ||
            //                        isRassAvail > 0; });
            auto startTime = std::chrono::high_resolution_clock::now();

            adcm::Log::Info() << ++rassVer << "번째 위험판단 값 전송 시작";
            if (riskAssessment.riskAssessmentList.size() != 0)
            {
                //risktAssessment object 의 생성시간 추가
                auto now = std::chrono::system_clock::now();
                auto riskAssessment_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
                adcm::Log::Info() << "Current timestamp in milliseconds: " << riskAssessment_timestamp;
                riskAssessment.timestamp = riskAssessment_timestamp;
                riskAssessment_provider.send(riskAssessment);
                adcm::Log::Info() << "riskAssessment sent!";

#ifdef NATS
                adcm::Log::Info() << "NATS 전송 시작";
                NatsSend(riskAssessment);
#endif
            }
            else
            {
                adcm::Log::Info() << "riskAssessment size is 0, do NOT send";
            }
            
            riskAssessment.riskAssessmentList.clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 대기시간
        }

        // {
        //     lock_guard<mutex> lock(mtx_send);
        //     send_wait = false;
        // }
        // mapSend.notify_one();
    }
}

void ThreadMonitor()
{
    while(continueExecution) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20000));

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

            if(gReceivedEvent_count_build_path != 0) {
                adcm::Log::Info() << "build_path Received count = " << gReceivedEvent_count_build_path;
                gReceivedEvent_count_build_path = 0;

            } else {
                adcm::Log::Info() << "build_path event timeout!!!";
            }
        }
    }
}


int main(int argc, char* argv[])
{
    // 자동 Build Time 생성
    time_t timer;
    struct tm *t;
    timer = time(NULL);
    t = localtime(&timer);

    std::string year = std::to_string(t->tm_year - 100);
    std::string mon = std::to_string(t->tm_mon + 1);
    std::string day = std::to_string(t->tm_mday);
    if (mon.length() == 1)
        mon.insert(0, "0");
    if (day.length() == 1)
        day.insert(0, "0");
    std::string b_day = year + mon + day;

    std::vector<std::thread> thread_list;
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
    adcm::Log::Info() << "SDK release_250321_interface v2.1, AGX Orin version";
    adcm::Log::Info() << "RiskAssessment Build " << b_day;
#ifdef NATS
    // Code to execute if NATS is defined
    adcm::Log::Info() << "NATS ON";
#else
    // Code to execute if NATS is not defined
    adcm::Log::Info() << "NATS OFF";
#endif
    thread_list.push_back(std::thread(ThreadReceiveMapData));
    thread_list.push_back(std::thread(ThreadReceiveBuildPath));
    thread_list.push_back(std::thread(ThreadMonitor));
    thread_list.push_back(std::thread(ThreadRASS));
    thread_list.push_back(std::thread(ThreadSend));


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
