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

#include "risk_avoidance_provider.h"
#include "map_data_subscriber.h"
#include "risk_assessment_subscriber.h"
#include "build_path_subscriber.h"

namespace
{

// Atomic flag for exit after SIGTERM caught
std::atomic_bool continueExecution{true};
std::atomic_uint gReceivedEvent_count_build_path{0};
std::atomic_uint gReceivedEvent_count_map_data{0};
std::atomic_uint gReceivedEvent_count_risk_assessment{0};
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
    adcm::Log::Info() << "RiskAvoidance ThreadAct1";
    adcm::RiskAvoidance_Provider riskAvoidance_provider;
    adcm::BuildPath_Subscriber buildPath_subscriber;
    adcm::RiskAssessment_Subscriber riskAssessment_subscriber;
    adcm::MapData_Subscriber mapData_subscriber;
    INFO("RiskAvoidance .init()");
    riskAvoidance_provider.init("RiskAvoidance/RiskAvoidance/PPort_risk_avoidance");
    buildPath_subscriber.init("RiskAvoidance/RiskAvoidance/RPort_build_path");
    mapData_subscriber.init("RiskAvoidance/RiskAvoidance/RPort_map_data");
    riskAssessment_subscriber.init("RiskAvoidance/RiskAvoidance/RPort_risk_assessment");
    INFO("After RiskAvoidance .init()");
    std::random_device m_rd;
    std::default_random_engine m_rand_eng(m_rd());
    std::uniform_real_distribution<double> m_ud_10000_10000(-10000, 10000);
    std::uniform_int_distribution<std::uint32_t> m_ud_0_10000(0, 10000);
    std::uniform_int_distribution<std::uint8_t> m_ud_0_4(0, 4);
    INFO("Thread loop start...");

    while (continueExecution) {
        gMainthread_Loopcount++;
        VERBOSE("[RiskAvoidance] Application loop");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool buildPath_rxEvent = buildPath_subscriber.waitEvent(0); // wait event
        bool mapData_rxEvent = mapData_subscriber.waitEvent(0); // wait event
        bool riskAssessment_rxEvent = riskAssessment_subscriber.waitEvent(0); // wait event
        
        if(buildPath_rxEvent) {
            adcm::Log::Verbose() << "[EVENT] RiskAvoidance Build Path received";

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

        if(mapData_rxEvent) {
            adcm::Log::Verbose() << "[EVENT] RiskAvoidance Map Data received";

            while(!mapData_subscriber.isEventQueueEmpty()) {
                auto data = mapData_subscriber.getEvent();
                gReceivedEvent_count_map_data++;

                auto obstacle = data->obstacle;
                auto environment = data->environment;
                auto vehicle = data->vehicle;

                adcm::Log::Verbose() << "obstacle.Time_stamp : "<< obstacle.Time_stamp;
                adcm::Log::Verbose() << "obstacle.obstacle_class : "<< obstacle.obstacle_class;
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

        if(riskAssessment_rxEvent) {
            adcm::Log::Verbose() << "[EVENT] RiskAvoidance Risk Assessment received";

            while(!riskAssessment_subscriber.isEventQueueEmpty()) {
                auto data = riskAssessment_subscriber.getEvent();
                gReceivedEvent_count_risk_assessment++;

                auto hazard_index = data->hazard_index;
                auto confidence = data->confidence;

                if(!hazard_index.empty()) {
                    adcm::Log::Verbose() << "=== hazard_index ===";
                    for(auto itr = hazard_index.begin(); itr != hazard_index.end(); ++itr) {
                        adcm::Log::Verbose() << *itr;
                    }
                } else {
                    adcm::Log::Verbose() << "hazard_index Vector empty!!! ";
                }

                if(!confidence.empty()) {
                    adcm::Log::Verbose() << "=== confidence ===";
                    for(auto itr = confidence.begin(); itr != confidence.end(); ++itr) {
                        adcm::Log::Verbose() << *itr;
                    }
                } else {
                    adcm::Log::Verbose() << "confidence Vector empty!!! ";
                }
            }
        }

        {
            adcm::risk_avoidance_Objects riskAvoidance;

            riskAvoidance.robot_id = m_ud_0_10000(m_rand_eng);
            riskAvoidance.behavior_planning.obstacle_x = m_ud_10000_10000(m_rand_eng);
            riskAvoidance.behavior_planning.obstacle_y = m_ud_10000_10000(m_rand_eng);
            riskAvoidance.behavior_planning.robot_position_x = m_ud_10000_10000(m_rand_eng);
            riskAvoidance.behavior_planning.robot_position_y = m_ud_10000_10000(m_rand_eng);
            riskAvoidance.behavior_planning.robot_orientation_theta = m_ud_10000_10000(m_rand_eng);
            riskAvoidance.behavior_planning.safety_area = m_ud_10000_10000(m_rand_eng);

            riskAvoidance_provider.send(riskAvoidance);
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

            if(gReceivedEvent_count_map_data != 0) {
                adcm::Log::Info() << "map_data Received count = " << gReceivedEvent_count_map_data;
                gReceivedEvent_count_map_data = 0;

            } else {
                adcm::Log::Info() << "map_data event timeout!!!";
            }

            if(gReceivedEvent_count_risk_assessment != 0) {
                adcm::Log::Info() << "risk_assessment Received count = " << gReceivedEvent_count_risk_assessment;
                gReceivedEvent_count_risk_assessment = 0;

            } else {
                adcm::Log::Info() << "risk_assessment event timeout!!!";
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
    adcm::Log::Info() << "RiskAvoidance: configure e2e protection";
    bool success = ara::com::e2exf::StatusHandler::Configure("./etc/e2e_dataid_mapping.json",
                   ara::com::e2exf::ConfigurationFormat::JSON,
                   "./etc/e2e_statemachines.json",
                   ara::com::e2exf::ConfigurationFormat::JSON);
    adcm::Log::Info() << "RiskAvoidance: e2e configuration " << (success ? "succeeded" : "failed");
#endif
    adcm::Log::Info() << "Ok, let's produce some RiskAvoidance data...";
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
