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
/**
  * 쓰레드간 관계
[ThreadReceiveMapData]
    ↓ writes map + sets isMapAvailable
    ↓ (cv_mapData.notify_one)

[ThreadReceiveBuildPath]
    ↓ writes path + sets isPathAvailable
    (no notify, just sets flag)

[ThreadRASS]
    waits cv_mapData (조건: isMapAvailable && isPathAvailable)
    ↓ reads map/path, writes riskAssessment
    ↓ (cv_rass.notify_one)

[ThreadSend]
    waits cv_rass
    ↓ reads + sends riskAssessment
*/

// ==========================전역변수=================================
// Shared data
std::vector<adcm::map_2dListVector> map_2d;
doubleVector path_x, path_y;
obstacleListVector obstacle_list;
adcm::vehicleListStruct ego_vehicle, sub_vehicle_1, sub_vehicle_2, sub_vehicle_3, sub_vehicle_4;
adcm::risk_assessment_Objects riskAssessment;

// Mutexes
std::mutex mtx_map;   // map_2d, obstacle_list, vehicle structs
std::mutex mtx_path;  // path_x, path_y
std::mutex mtx_rass;  // riskAssessment
std::mutex mtx_cv;    // isMapAvailable, isPathAvailable flags

// Condition variables
std::condition_variable cv_mapData;  // ThreadRASS waits on this
std::condition_variable cv_rass;     // ThreadSend waits on this

// Shared flags
bool isMapAvailable = false;
bool isPathAvailable = false;

/**
 * @brief 맵 데이터 수신 쓰레드
 *
 * - map_2d, obstacle_list, vehicle_list 수신
 * - 수신 후 isMapAvailable = true 설정
 * - 조건변수 cv_mapData를 통해 ThreadRASS를 깨움
 */
void ThreadReceiveMapData()
{
    adcm::MapData_Subscriber mapData_subscriber;
    mapData_subscriber.init("RiskAssessment/RiskAssessment/RPort_map_data");
    INFO("Thread ThreadReceiveMapData start...");

    while (continueExecution) {
        gMainthread_Loopcount++;
        bool mapData_rxEvent = mapData_subscriber.waitEvent(10000); // wait event

        if(mapData_rxEvent) {
            adcm::Log::Info() << "[EVENT] RiskAssessment Map Data received";

            while(!mapData_subscriber.isEventQueueEmpty()) {
                std::lock_guard<std::mutex> lock(mtx_map);
                auto data = mapData_subscriber.getEvent();
                gReceivedEvent_count_map_data++;

                if (!data) {
                    adcm::Log::Info() << "[ERROR] getEvent returned null!";
                    continue;
                }

                adcm::Log::Info() << "[DEBUG] data->map_2d.size() = " << (data->map_2d.size())*(data->map_2d[0].size());

                // 안전성 체크 후 assign
                //map_2d.clear();  // <- 이건 메모리 누적 방지용으로는 OK
                map_2d = data->map_2d;
                obstacle_list = data->obstacle_list; 
                auto vehicle_list = data->vehicle_list;
                adcm::Log::Info() << "[DEBUG] map_2d assign 완료";

                if(!obstacle_list.empty())
                {
                    adcm::Log::Info() << "size of obstacle list received: " << obstacle_list.size();
                    // for(auto itr = obstacle_list.begin(); itr != obstacle_list.end(); ++itr) 
                    // {
                    //     adcm::Log::Info() << "obstacle_id : " << itr->obstacle_id;
                    //     adcm::Log::Info() << "obstacle_class : " << itr->obstacle_class;
                    //     adcm::Log::Info() << "timestamp : " << itr->timestamp;
                    // }
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
                    adcm::Log::Info() << "Map Available";

                }
                cv_mapData.notify_one();  // 다른 스레드에게 알림
                adcm::Log::Info() << "ThreadRass 에게 알림";
            
            }
        }
    }
}

/**
 * @brief 경로(Build Path) 수신 쓰레드
 *
 * - EGO_VEHICLE의 경로(route)를 수신하면 path_x, path_y 설정
 * - 최초 1회 수신 시 isPathAvailable = true 설정
 * - ThreadRASS는 isMapAvailable && isPathAvailable 조건 만족 시 실행됨
 * - 따로 notify는 하지 않음
 */
void ThreadReceiveBuildPath()
{
    adcm::BuildPath_Subscriber buildPath_subscriber;
    buildPath_subscriber.init("RiskAssessment/RiskAssessment/RPort_build_path");
    INFO("Thread ThreadReceiveBuildPath start...");

    while (continueExecution) {
        gMainthread_Loopcount++;
        VERBOSE("[RiskAssessment] Application loop");
        bool buildPath_rxEvent = buildPath_subscriber.waitEvent(1000); // wait event

        if(buildPath_rxEvent) {
            adcm::Log::Info() << "[EVENT] RiskAssessment Build Path received";

            while(!buildPath_subscriber.isEventQueueEmpty()) {
                auto data = buildPath_subscriber.getEvent();
                gReceivedEvent_count_build_path++;
                if (!data) {
                    adcm::Log::Info() << "[ERROR] getEvent returned null!";
                    continue;
                }
                const auto& Path = data->Path;
                for (const auto& pathItem : Path) {
                    const auto& route = pathItem.route;
                    if (pathItem.vehicle_class == VehicleClass::EGO_VEHICLE)
                    {   
                        std::lock_guard<std::mutex> lock(mtx_path);
                        path_x = doubleVector(route.size());
                        path_y = doubleVector(route.size());
                        //새로운 경로를 받으면 예전 변환값이 담긴 path_x 와 path_y 초기화
                        INFO("특장차의 경로 좌표변환 진행");
                        gpsToMapcoordinate(route, path_x, path_y);
                        
                        {
                            std::lock_guard<std::mutex> lock(mtx_cv);
                            isPathAvailable = true;  // 최초 1회 true면 ThreadRASS 조건 만족
                            INFO("Path Available");

                        }

                    }
                    for (const auto& point : route)
                    {
                        adcm::Log::Info() << "vehicle_class : "<< pathItem.vehicle_class;
                        adcm::Log::Info() << "route.latitude : "<< point.latitude;
                        adcm::Log::Info() << "route.longitude : "<< point.longitude;
                        adcm::Log::Info() << "route.delta_t : "<< point.delta_t;
                    }
                }
            }
        }
    }
}
/**
 * @brief 위험 판단 수행 쓰레드
 *
 * - cv_mapData 조건변수에 의해 map + path 수신 후 실행됨
 * - evaluateScenario1~8을 통해 riskAssessment.riskAssessmentList 생성
 * - 결과가 있을 경우 cv_rass.notify_one()으로 ThreadSend를 깨움
 */
void ThreadRASS()
{
    adcm::Log::Info() << "Thread ThreadRASS start...";

    while (continueExecution) {
        std::unique_lock<std::mutex> lock(mtx_cv);
        cv_mapData.wait(lock, [] {
            return !continueExecution || (isMapAvailable && isPathAvailable);
        });
        if (!continueExecution) break;
        lock.unlock();

        {
            std::lock_guard<std::mutex> lock_map(mtx_map);
            std::lock_guard<std::mutex> lock_path(mtx_path);
            std::lock_guard<std::mutex> lock_rass(mtx_rass);

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

        if (!riskAssessment.riskAssessmentList.empty())
        {
            // notify ThreadSend
            cv_rass.notify_one();  // 전송 쓰레드 깨움
        }
        isMapAvailable = false;
    }
}

/**
 * @brief 위험 판단 결과 전송 쓰레드
 *
 * - cv_rass 조건변수로 riskAssessment 생성 완료 알림을 받음
 * - SOME/IP 및 NATS로 전송 수행
 */
void ThreadSend()
{
    adcm::Log::Info() << "ThreadSend start...";

    int rassVer = 0; // 현재 위험판단이 몇 번째 값인지 확인
    adcm::RiskAssessment_Provider riskAssessment_provider;
    riskAssessment_provider.init("RiskAssessment/RiskAssessment/PPort_risk_assessment");
    // mutex, condition value 사용
  while (continueExecution)
    {
        // [1] 조건변수 대기: 위험판단 데이터 생성 알림 대기
        std::unique_lock<std::mutex> lock_rass_cv(mtx_rass);
        cv_rass.wait(lock_rass_cv, [] {
            return !riskAssessment.riskAssessmentList.empty() || !continueExecution;
        });

        if (!continueExecution) break;

        auto t_start_total = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::system_clock::now();
        auto riskAssessment_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        riskAssessment.timestamp = riskAssessment_timestamp;

        adcm::Log::Info() << "[" << ++rassVer << "차] 위험판단 전송 시작, timestamp: " << riskAssessment_timestamp;

        // [3] 전송 시작
        auto t_start_send = std::chrono::high_resolution_clock::now();
        riskAssessment_provider.send(riskAssessment);
        auto t_end_send = std::chrono::high_resolution_clock::now();
        adcm::Log::Info() << "→ 위험판단 전송 완료";

        auto send_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_send - t_start_send).count();
        adcm::Log::Info() << "→ 위험판단 전송 소요 시간: " << send_duration_ms << " ms";

    #ifdef NATS
        auto t_start_nats = std::chrono::high_resolution_clock::now();
        adcm::Log::Info() << "→ NATS 전송 시작";
        NatsSend(riskAssessment);
        auto t_end_nats = std::chrono::high_resolution_clock::now();
        adcm::Log::Info() << "→ NATS 전송 완료";

        auto nats_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_nats - t_start_nats).count();
        adcm::Log::Info() << "→ NATS 전송 소요 시간: " << nats_duration_ms << " ms";
    #endif

        auto t_end_total = std::chrono::high_resolution_clock::now();
        auto total_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_total - t_start_total).count();
        adcm::Log::Info() << "→ 전체 전송 처리 시간: " << total_duration_ms << " ms";

        // [4] 전송 완료 후 위험 판단 데이터 초기화
        riskAssessment.riskAssessmentList.clear();
        //std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
