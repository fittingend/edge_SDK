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
#include "build_path_subscriber.h"

std::shared_ptr<adcm::BuildPath_Subscriber> buildPath_subscriber;

namespace
{

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

}  // namespace

void ThreadAct1()
{
    adcm::Log::Info() << "ControlHub ThreadAct1";
    adcm::HubData_Provider hubData_provider;
    adcm::BuildPathTest_Provider buildPathTest_provider;
    INFO("ControlHub .init()");
    hubData_provider.init("ControlHub/ControlHub/PPort_hub_data");
    buildPathTest_provider.init("ControlHub/ControlHub/PPort_build_path_test");
    INFO("After ControlHub .init()");
    std::random_device m_rd;
    std::default_random_engine m_rand_eng(m_rd());
    std::uniform_real_distribution<double> m_ud_10000_10000(-10000, 10000);
    std::uniform_real_distribution<float> m_ud_100_100(-100, 100);
    std::uniform_int_distribution<std::uint32_t> m_ud_0_10000(0, 10000);
    std::uniform_int_distribution<std::uint8_t> m_ud_0_8(0, 8);
    std::uniform_int_distribution<std::uint16_t> m_ud_0_16(0, 16);
    std::uniform_int_distribution<std::uint64_t> m_ud_0_64(0, 64);
    INFO("Thread loop start...");

    while (continueExecution) {
        gMainthread_Loopcount++;
        VERBOSE("[ControlHub] Application loop");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool buildPath_rxEvent = buildPath_subscriber->waitEvent(0); // wait event

        if(buildPath_rxEvent) {
            adcm::Log::Verbose() << "[EVENT] ControlHub Build Path received";

            while(!buildPath_subscriber->isEventQueueEmpty()) {
                auto data = buildPath_subscriber->getEvent();
                gReceivedEvent_count_build_path++;

                auto Seq = data->Seq;
                auto vehicle_class = data->vehicle_class;
                auto mve_id = data->mve_id;
                auto mve_type = data->mve_type;
                auto sec = data->sec;
                auto nsec = data->nsec;
                auto frame_id = data->frame_id;
                auto size = data->size;
                auto utm_x = data->utm_x;
                auto utm_y = data->utm_y;

                adcm::Log::Verbose() << "Seq : " << Seq;
                adcm::Log::Verbose() << "vehicle_class : " << vehicle_class;
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

        {
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

        {
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

    String serviceFlag;
    double source_latitude = 0.0;
    double source_longitude = 0.0;
    double destination_latitude = 0.0;
    double destination_longitude = 0.0;
    std::uint8_t input_mve_type = 0;

    while(continueExecution){
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        
        serviceFlag = buildPath_subscriber->fieldGetter_ServiceFlag();

        if(serviceFlag == "On"){
            adcm::Log::Info() << "[ControlHub][buildPath_subscriber] BuildPath Method Call Test";

            source_latitude = m_ud_90_90(m_rand_eng);
            source_longitude = m_ud_180_180(m_rand_eng);
            destination_latitude = m_ud_90_90(m_rand_eng);
            destination_longitude = m_ud_180_180(m_rand_eng);
            input_mve_type = m_ud_0_2(m_rand_eng);

            buildPath_subscriber->BuildPath(source_latitude, source_longitude, destination_latitude, destination_longitude, input_mve_type);

            auto Path = buildPath_subscriber->getBuildPath();

            auto Seq = Path.Seq;
            auto vehicle_class = Path.vehicle_class;
            auto mve_id = Path.mve_id;
            auto mve_type = Path.mve_type;
            auto sec = Path.sec;
            auto nsec = Path.nsec;
            auto frame_id = Path.frame_id;
            auto size = Path.size;
            auto utm_x = Path.utm_x;
            auto utm_y = Path.utm_y;
            
            adcm::Log::Verbose() << "Seq : " << Seq;
            adcm::Log::Verbose() << "vehicle_class : " << vehicle_class;
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
                adcm::Log::Info() << "utm_x Vector empty!!! ";
            }

            if(!utm_y.empty()) {
                adcm::Log::Verbose() << "=== utm_y ===";
                for(auto itr = utm_y.begin(); itr != utm_y.end(); ++itr) {
                    adcm::Log::Verbose() << *itr;
                }
            } else {
                adcm::Log::Info() << "utm_y Vector empty!!! ";
            }

        }else{
            adcm::Log::Info() << "TaskManagerPathBuilder Process is not running...";
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
    adcm::Log::Info() << "ControlHub: configure e2e protection";
    bool success = ara::com::e2exf::StatusHandler::Configure("./etc/e2e_dataid_mapping.json",
                   ara::com::e2exf::ConfigurationFormat::JSON,
                   "./etc/e2e_statemachines.json",
                   ara::com::e2exf::ConfigurationFormat::JSON);
    adcm::Log::Info() << "ControlHub: e2e configuration " << (success ? "succeeded" : "failed");
#endif
    adcm::Log::Info() << "Ok, let's produce some ControlHub data...";
    buildPath_subscriber = std::make_shared<adcm::BuildPath_Subscriber>();
    buildPath_subscriber->init("ControlHub/ControlHub/RPort_build_path");
    
    std::thread act1(ThreadAct1);
    std::thread monitor(ThreadMonitor);
    // build_path Method Call Test
    std::thread methodCallTest(ThreadMethodCallTest);
    adcm::Log::Info() << "Thread join";
    act1.join();
    monitor.join();
    methodCallTest.join();
    adcm::Log::Info() << "done.";

    if(!ara::core::Deinitialize()) {
        // No interaction with ARA is possible here since some ARA resources can be destroyed already
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
