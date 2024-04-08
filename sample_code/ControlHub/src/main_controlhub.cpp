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

#include "hub_data_provider.h"
#include "build_path_test_provider.h"
#include "work_information_provider.h"
#include "build_path_subscriber.h"

std::random_device m_rd;
std::default_random_engine m_rand_eng(m_rd());
std::uniform_real_distribution<double> m_ud_10000_10000(-10000, 10000);
std::uniform_real_distribution<float> m_ud_100_100(-100, 100);
std::uniform_int_distribution<std::uint32_t> m_ud_0_10000(0, 10000);
std::uniform_int_distribution<std::uint8_t> m_ud_0_8(0, 8);
std::uniform_int_distribution<std::uint16_t> m_ud_0_16(0, 16);
std::uniform_int_distribution<std::uint64_t> m_ud_0_64(0, 64);

namespace ControlHub
{
std::shared_ptr<adcm::BuildPath_Subscriber> buildPath_subscriber;

// Atomic flag for exit after SIGTERM caught
std::atomic_bool continueExecution{true};
std::atomic_uint gReceivedEvent_count_build_path{0};
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

void ThreadReceiveBuildPath()
{
    adcm::Log::Info() << "ControlHub ThreadReceiveBuildPath";

    while (continueExecution) {
        gMainthread_Loopcount++;
        VERBOSE("[ControlHub] ThreadReceiveBuildPath loop");
        bool buildPath_rxEvent = buildPath_subscriber->waitEvent(100); // wait event

        if(buildPath_rxEvent) {
            adcm::Log::Verbose() << "[EVENT] ControlHub Build Path received";

            while(!buildPath_subscriber->isEventQueueEmpty()) {
                auto data = buildPath_subscriber->getEvent();
                gReceivedEvent_count_build_path++;

                auto Path = data->Path;

                if(!Path.empty()) {
                    adcm::Log::Verbose() << "=== Path ===";
                    for(auto itr = Path.begin(); itr != Path.end(); ++itr) {
                        adcm::Log::Verbose() << "result : " << itr->result;
                        adcm::Log::Verbose() << "t0 : " << itr->t0;
                        adcm::Log::Verbose() << "vehicle_class : " << itr->vehicle_class;
                        adcm::Log::Verbose() << "move_type : " << itr->move_type;
                        adcm::Log::Verbose() << "job_type : " << itr->job_type;
                        adcm::Log::Verbose() << "size : " << itr->size;

                        auto route = itr->route;

                        if(!route.empty()) {
                            adcm::Log::Verbose() << "=== route ===";
                            for(auto itr = route.begin(); itr != route.end(); ++itr) {
                                adcm::Log::Verbose() << "route.x : "<< itr->x;
                                adcm::Log::Verbose() << "route.y : "<< itr->y;
                                adcm::Log::Verbose() << "route.delta_t : "<< itr->delta_t;
                            }
                        } else {
                            adcm::Log::Verbose() << "route Vector empty!!! ";
                        }
                    }
                } else {
                    adcm::Log::Verbose() << "Path Vector empty!!! ";
                }
            }
        }
    }
}

void ThreadSendHubData()
{
    adcm::Log::Info() << "ControlHub ThreadSendHubData";
    adcm::HubData_Provider hubData_provider;
    hubData_provider.init("ControlHub/ControlHub/PPort_hub_data");

    while (continueExecution) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        adcm::hub_data_Objects hubData;
        adcm::HubObstacleStruct hubObstacle;

        hubData.timestamp = m_ud_0_64(m_rand_eng);

        hubObstacle.obstacle_class = m_ud_0_8(m_rand_eng);
        hubObstacle.cuboid_x = m_ud_10000_10000(m_rand_eng);
        hubObstacle.cuboid_y = m_ud_10000_10000(m_rand_eng);
        hubObstacle.cuboid_z = m_ud_10000_10000(m_rand_eng);
        hubObstacle.heading_angle = m_ud_10000_10000(m_rand_eng);

        hubObstacle.covariance_matrix.clear();
        hubObstacle.covariance_matrix.push_back(m_ud_10000_10000(m_rand_eng));
        hubObstacle.covariance_matrix.push_back(m_ud_10000_10000(m_rand_eng));
        hubObstacle.covariance_matrix.push_back(m_ud_10000_10000(m_rand_eng));
        hubObstacle.covariance_matrix.push_back(m_ud_10000_10000(m_rand_eng));
        hubObstacle.covariance_matrix.push_back(m_ud_10000_10000(m_rand_eng));

        hubObstacle.position_x = m_ud_10000_10000(m_rand_eng);
        hubObstacle.position_y = m_ud_10000_10000(m_rand_eng);
        hubObstacle.position_z = m_ud_10000_10000(m_rand_eng);
        hubObstacle.velocity_x = m_ud_10000_10000(m_rand_eng);
        hubObstacle.velocity_y = m_ud_10000_10000(m_rand_eng);
        hubObstacle.velocity_z = m_ud_10000_10000(m_rand_eng);

        hubData.obstacle.clear();
        hubData.obstacle.push_back(hubObstacle);

        hubData.road_z.clear();
        hubData.road_z.push_back(m_ud_10000_10000(m_rand_eng));
        hubData.road_z.push_back(m_ud_10000_10000(m_rand_eng));
        hubData.road_z.push_back(m_ud_10000_10000(m_rand_eng));

        hubData.vehicle_class = m_ud_0_8(m_rand_eng);
        hubData.position_lat = m_ud_100_100(m_rand_eng);
        hubData.position_long = m_ud_100_100(m_rand_eng);
        hubData.position_height = m_ud_100_100(m_rand_eng);
        hubData.yaw = m_ud_100_100(m_rand_eng);
        hubData.roll = m_ud_100_100(m_rand_eng);
        hubData.pitch = m_ud_100_100(m_rand_eng);
        hubData.velocity_long = m_ud_100_100(m_rand_eng);
        hubData.velocity_lat = m_ud_100_100(m_rand_eng);
        hubData.velocity_ang = m_ud_100_100(m_rand_eng);

        hubData_provider.send(hubData);
    }
}

void ThreadSendBuildPathTest()
{
    adcm::Log::Info() << "ControlHub ThreadSendBuildPathTest";
    adcm::BuildPathTest_Provider buildPathTest_provider;
    buildPathTest_provider.init("ControlHub/ControlHub/PPort_build_path_test");

    while (continueExecution) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        adcm::build_path_test_Objects buildPathTest;

        buildPathTest.size = m_ud_0_16(m_rand_eng);
        buildPathTest.utm_x.clear();
        buildPathTest.utm_x.push_back(m_ud_10000_10000(m_rand_eng));
        buildPathTest.utm_x.push_back(m_ud_10000_10000(m_rand_eng));
        buildPathTest.utm_x.push_back(m_ud_10000_10000(m_rand_eng));
        buildPathTest.utm_y.clear();
        buildPathTest.utm_y.push_back(m_ud_10000_10000(m_rand_eng));
        buildPathTest.utm_y.push_back(m_ud_10000_10000(m_rand_eng));
        buildPathTest.utm_y.push_back(m_ud_10000_10000(m_rand_eng));

        buildPathTest_provider.send(buildPathTest);
    }
}

void ThreadSendBuildWorkInformation()
{
    adcm::Log::Info() << "ControlHub ThreadSendBuildWorkInformation";
    adcm::WorkInformation_Provider workInformation_provider;
    workInformation_provider.init("ControlHub/ControlHub/PPort_work_information");

    while (continueExecution) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        adcm::work_information_Objects workInformation;
        adcm::subVehicleStruct subVehicle;
        adcm::workingAreaBoundaryStruct workingAreaBoundary;

        workInformation.main_vehicle.length = m_ud_0_16(m_rand_eng);
        workInformation.main_vehicle.width = m_ud_0_16(m_rand_eng);

        workInformation.sub_vehicle.clear();
        for (int i = 0; i < 4; ++i){
            subVehicle.length = m_ud_0_16(m_rand_eng);
            subVehicle.width = m_ud_0_16(m_rand_eng);
            workInformation.sub_vehicle.push_back(subVehicle);
        }

        workInformation.working_area_boundary.clear();
        for (int i = 0; i < 10; ++i){
            workingAreaBoundary.x = m_ud_10000_10000(m_rand_eng);
            workingAreaBoundary.y = m_ud_10000_10000(m_rand_eng);
            workInformation.working_area_boundary.push_back(workingAreaBoundary);
        }

        workInformation_provider.send(workInformation);
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
        }
    }
}

void ThreadMethodCallTest()
{
    std::random_device m_rd;
    std::default_random_engine m_rand_eng(m_rd());
    std::uniform_real_distribution<double> m_ud_90_90(-90, 90);
    std::uniform_real_distribution<double> m_ud_180_180(-180, 180);
    std::uniform_int_distribution<std::uint8_t> m_ud_0_2(0, 2);

    double source_latitude = 0.0;
    double source_longitude = 0.0;
    double destination_latitude = 0.0;
    double destination_longitude = 0.0;
    std::uint8_t input_mve_type = 0;
    std::uint64_t deadLine = 0;

    while(continueExecution){
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        
        adcm::Log::Info() << "[ControlHub][buildPath_subscriber] BuildPath Method Call Test";

        source_latitude = m_ud_90_90(m_rand_eng);
        source_longitude = m_ud_180_180(m_rand_eng);
        destination_latitude = m_ud_90_90(m_rand_eng);
        destination_longitude = m_ud_180_180(m_rand_eng);
        input_mve_type = m_ud_0_2(m_rand_eng);
        deadLine = 1000;    //millisecond

        auto buildPathPtr = buildPath_subscriber->BuildPath(
            source_latitude, source_longitude, destination_latitude, destination_longitude, input_mve_type, deadLine);

        if(buildPathPtr != NULL){
            auto Path = buildPathPtr->Path;

            if(!Path.empty()) {
                adcm::Log::Verbose() << "=== Path ===";
                for(auto itr = Path.begin(); itr != Path.end(); ++itr) {
                    adcm::Log::Verbose() << "result : " << itr->result;
                    adcm::Log::Verbose() << "t0 : " << itr->t0;
                    adcm::Log::Verbose() << "vehicle_class : " << itr->vehicle_class;
                    adcm::Log::Verbose() << "move_type : " << itr->move_type;
                    adcm::Log::Verbose() << "job_type : " << itr->job_type;
                    adcm::Log::Verbose() << "size : " << itr->size;

                    auto route = itr->route;

                    if(!route.empty()) {
                        adcm::Log::Verbose() << "=== route ===";
                        for(auto itr = route.begin(); itr != route.end(); ++itr) {
                            adcm::Log::Verbose() << "route.x : "<< itr->x;
                            adcm::Log::Verbose() << "route.y : "<< itr->y;
                            adcm::Log::Verbose() << "route.delta_t : "<< itr->delta_t;
                        }
                    } else {
                        adcm::Log::Verbose() << "route Vector empty!!! ";
                    }
                }
            } else {
                adcm::Log::Verbose() << "Path Vector empty!!! ";
            }
        }else{
            adcm::Log::Error() << "buildPath is NULL...";
        }
    }
}

}  // namespace


int main(int argc, char* argv[])
{
    std::vector<std::thread> thread_list;
    UNUSED(argc);
    UNUSED(argv);

    if(!ara::core::Initialize()) {
        // No interaction with ARA is possible here since initialization failed
        return EXIT_FAILURE;
    }

    ara::exec::ExecutionClient exec_client;
    exec_client.ReportExecutionState(ara::exec::ExecutionState::kRunning);

    if(!ControlHub::RegisterSigTermHandler()) {
        adcm::Log::Error() << "Unable to register signal handler";
    }

#ifndef R19_11_1
    adcm::Log::Info() << "ControlHub: configure e2e protection";
    bool success = ara::com::e2exf::StatusHandler::Configure("./etc/e2e_dataid_mapping.json",
                   ara::com::e2exf::ConfigurationFormat::JSON,
                   "./etc/e2e_statemachines.json",
                   ara::com::e2exf::ConfigurationFormat::JSON);
    adcm::Log::Info() << "ControlHub: e2e configuration " << (success ? "succeeded" : "failed");
#endif
    adcm::Log::Info() << "Ok, let's produce some ControlHub data...";
    ControlHub::buildPath_subscriber = std::make_shared<adcm::BuildPath_Subscriber>();
    ControlHub::buildPath_subscriber->init("ControlHub/ControlHub/RPort_build_path");
    
    thread_list.push_back(std::thread(ControlHub::ThreadReceiveBuildPath));
    thread_list.push_back(std::thread(ControlHub::ThreadSendHubData));
    thread_list.push_back(std::thread(ControlHub::ThreadSendBuildPathTest));
    thread_list.push_back(std::thread(ControlHub::ThreadSendBuildWorkInformation));
    thread_list.push_back(std::thread(ControlHub::ThreadMonitor));
    thread_list.push_back(std::thread(ControlHub::ThreadMethodCallTest));

    adcm::Log::Info() << "Thread join";
    for(int i = 0; i < static_cast<int>(thread_list.size()); i++) {
        thread_list[i].join();
    }

    adcm::Log::Info() << "done.";

    if(!ara::core::Deinitialize()) {
        // No interaction with ARA is possible here since some ARA resources can be destroyed already
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
