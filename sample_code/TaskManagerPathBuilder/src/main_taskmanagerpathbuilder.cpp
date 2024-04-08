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

#include "build_path_provider.h"
#include "map_data_subscriber.h"
#include "risk_assessment_subscriber.h"
#include "work_information_subscriber.h"

std::random_device m_rd;
std::default_random_engine m_rand_eng(m_rd());
std::uniform_real_distribution<double> m_ud_10000_10000(-10000, 10000);
std::uniform_int_distribution<std::uint64_t> m_ud_0_64(0, 64);
std::uniform_int_distribution<std::uint32_t> m_ud_0_32(0, 32);
std::uniform_int_distribution<std::uint16_t> m_ud_0_16(0, 16);
std::uniform_int_distribution<std::uint8_t> m_ud_0_8(0, 8);


namespace TaskManagerPathBuilder
{

std::shared_ptr<adcm::BuildPath_Provider> buildPath_provider;

// Atomic flag for exit after SIGTERM caught
std::atomic_bool continueExecution{true};
std::atomic_uint gReceivedEvent_count_map_data{0};
std::atomic_uint gReceivedEvent_count_risk_assessment{0};
std::atomic_uint gReceivedEvent_count_work_information{0};
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

void setBuildPath(const double& source_latitude, const double& source_longitude, 
    const double& destination_latitude, const double& destination_longitude, const std::uint8_t& type)
{
    adcm::Log::Info() << "TaskManagerPathBuilder >> setBuildPath function callback";
    
    auto output = buildPath_provider->getPtrOutput();

    adcm::pathStruct ps;
    adcm::routeStruct route;

    ps.result = m_ud_0_8(m_rand_eng);
    ps.t0 = m_ud_0_64(m_rand_eng);
    ps.vehicle_class = m_ud_0_8(m_rand_eng);
    ps.move_type = m_ud_0_8(m_rand_eng);
    ps.job_type = m_ud_0_8(m_rand_eng);
    ps.size = m_ud_0_16(m_rand_eng);

    ps.route.clear();
    for (int i = 0; i < 3; ++i){
        route.x = m_ud_10000_10000(m_rand_eng);
        route.y = m_ud_10000_10000(m_rand_eng);
        route.delta_t =  m_ud_0_16(m_rand_eng);
        ps.route.push_back(route);
    }

    output->Object.Path.clear();
    output->Object.Path.push_back(ps);
}

void ThreadReceiveMapData()
{
    adcm::Log::Info() << "TaskManagerPathBuilder ThreadReceiveMapData";
    adcm::MapData_Subscriber mapData_subscriber;
    INFO("TaskManagerPathBuilder .init()");
    mapData_subscriber.init("TaskManagerPathBuilder/TaskManagerPathBuilder/RPort_map_data");
    INFO("Thread ThreadReceiveMapData start...");

    while (continueExecution) {
        gMainthread_Loopcount++;
        VERBOSE("[TaskManagerPathBuilder] ThreadReceiveMapData loop");
        bool mapData_rxEvent = mapData_subscriber.waitEvent(100); // wait event

        if(mapData_rxEvent) {
            adcm::Log::Verbose() << "[EVENT] TaskManagerPathBuilder Map Data received";

            while(!mapData_subscriber.isEventQueueEmpty()) {
                auto data = mapData_subscriber.getEvent();
                gReceivedEvent_count_map_data++;

                auto map_2d = data->map_2d;
                auto obstacle_list = data->obstacle_list;
                auto vehicle_list = data->vehicle_list;

                if(!map_2d.empty()) {
                    adcm::Log::Verbose() << "=== map_2d ===";
                    // sample case 1)
                    // for (int i = 0; i < map_2d.size(); ++i)
                    // {
                    //     adcm::Log::Verbose() << "===== map_2dList =====";
                    //     for(auto itr = map_2d[i].begin(); itr != map_2d[i].end(); ++itr) {
                    //         adcm::Log::Verbose() << "obstacle_id : " << itr->obstacle_id;
                    //         adcm::Log::Verbose() << "vehicle_class : " << itr->vehicle_class;
                    //         adcm::Log::Verbose() << "road_z : " << itr->road_z;
                    //     }
                    //     adcm::Log::Verbose() << "======================";
                    // }

                    // sample case 2)
                    int col = map_2d.size();
                    int row = map_2d[0].size();
                    for (int i = 0; i < col; ++i)
                    {
                        for (int j = 0; j < row; ++j)
                        {
                            adcm::Log::Verbose() << "[" << i << "]" << "[" << j << "]" << "obstacle_id : " << map_2d[i][j].obstacle_id;
                            adcm::Log::Verbose() << "[" << i << "]" << "[" << j << "]" << "vehicle_class : " << map_2d[i][j].vehicle_class;
                            adcm::Log::Verbose() << "[" << i << "]" << "[" << j << "]" << "road_z : " << map_2d[i][j].road_z;
                        }
                    }
                } else {
                    adcm::Log::Verbose() << "map_2d Vector empty!!! ";
                }

                if(!obstacle_list.empty()) {
                    adcm::Log::Verbose() << "=== obstacle_list ===";
                    for(auto itr = obstacle_list.begin(); itr != obstacle_list.end(); ++itr) {
                        adcm::Log::Verbose() << "obstacle_id : " << itr->obstacle_id;
                        adcm::Log::Verbose() << "obstacle_class : " << itr->obstacle_class;
                        adcm::Log::Verbose() << "timestamp : " << itr->timestamp;

                        auto map_2d_location = itr->map_2d_location;

                        if(!map_2d_location.empty()) {
                            adcm::Log::Verbose() << "=== map_2d_location ===";
                            for(auto itr = map_2d_location.begin(); itr != map_2d_location.end(); ++itr) {
                                adcm::Log::Verbose() << "x index : " << itr->x;
                                adcm::Log::Verbose() << "y index : " << itr->y;
                            }
                        } else {
                            adcm::Log::Verbose() << "obstacle_list map_2d_location Vector empty!!! ";
                        }

                        adcm::Log::Verbose() << "stop_count : " << itr->stop_count;
                        adcm::Log::Verbose() << "fused_cuboid_x : " << itr->fused_cuboid_x;
                        adcm::Log::Verbose() << "fused_cuboid_y : " << itr->fused_cuboid_y;
                        adcm::Log::Verbose() << "fused_cuboid_z : " << itr->fused_cuboid_z;
                        adcm::Log::Verbose() << "fused_heading_angle : " << itr->fused_heading_angle;
                        adcm::Log::Verbose() << "fused_position_x : " << itr->fused_position_x;
                        adcm::Log::Verbose() << "fused_position_y : " << itr->fused_position_y;
                        adcm::Log::Verbose() << "fused_position_z : " << itr->fused_position_z;
                        adcm::Log::Verbose() << "fused_velocity_x : " << itr->fused_velocity_x;
                        adcm::Log::Verbose() << "fused_velocity_y : " << itr->fused_velocity_y;
                        adcm::Log::Verbose() << "fused_velocity_z : " << itr->fused_velocity_z;
                    }
                } else {
                    adcm::Log::Verbose() << "obstacle_list Vector empty!!! ";
                }

                if(!vehicle_list.empty()) {
                    adcm::Log::Verbose() << "=== vehicle_list ===";
                    for(auto itr = vehicle_list.begin(); itr != vehicle_list.end(); ++itr) {
                        adcm::Log::Verbose() << "vehicle_class : " << itr->vehicle_class;
                        adcm::Log::Verbose() << "timestamp : " << itr->timestamp;
                        
                        auto map_2d_location = itr->map_2d_location;

                        if(!map_2d_location.empty()) {
                            adcm::Log::Verbose() << "=== map_2d_location ===";
                            for(auto itr = map_2d_location.begin(); itr != map_2d_location.end(); ++itr) {
                                adcm::Log::Verbose() << "x index : " << itr->x;
                                adcm::Log::Verbose() << "y index : " << itr->y;
                            }
                        } else {
                            adcm::Log::Verbose() << "vehicle_list map_2d_location Vector empty!!! ";
                        }

                        adcm::Log::Verbose() << "position_long : " << itr->position_long;
                        adcm::Log::Verbose() << "position_lat : " << itr->position_lat;
                        adcm::Log::Verbose() << "position_height : " << itr->position_height;
                        adcm::Log::Verbose() << "position_x : " << itr->position_x;
                        adcm::Log::Verbose() << "position_y : " << itr->position_y;
                        adcm::Log::Verbose() << "position_z : " << itr->position_z;
                        adcm::Log::Verbose() << "yaw : " << itr->yaw;
                        adcm::Log::Verbose() << "roll : " << itr->roll;
                        adcm::Log::Verbose() << "pitch : " << itr->pitch;
                        adcm::Log::Verbose() << "velocity_long : " << itr->velocity_long;
                        adcm::Log::Verbose() << "velocity_lat : " << itr->velocity_lat;
                        adcm::Log::Verbose() << "velocity_x : " << itr->velocity_x;
                        adcm::Log::Verbose() << "velocity_y : " << itr->velocity_y;
                        adcm::Log::Verbose() << "velocity_ang : " << itr->velocity_ang;
                    }
                } else {
                    adcm::Log::Verbose() << "vehicle_list Vector empty!!! ";
                }
            }
        }
    }
}

void ThreadReceiveRiskAssessment()
{
    adcm::Log::Info() << "TaskManagerPathBuilder ThreadReceiveRiskAssessment";
    adcm::RiskAssessment_Subscriber riskAssessment_subscriber;
    INFO("TaskManagerPathBuilder .init()");
    riskAssessment_subscriber.init("TaskManagerPathBuilder/TaskManagerPathBuilder/RPort_risk_assessment");
    INFO("Thread ThreadReceiveRiskAssessment start...");

    while (continueExecution) {
        gMainthread_Loopcount++;
        VERBOSE("[TaskManagerPathBuilder] Application loop");
        bool riskAssessment_rxEvent = riskAssessment_subscriber.waitEvent(100); // wait event

        if(riskAssessment_rxEvent) {
            adcm::Log::Verbose() << "[EVENT] TaskManagerPathBuilder Risk Assessment received";

            while(!riskAssessment_subscriber.isEventQueueEmpty()) {
                auto data = riskAssessment_subscriber.getEvent();
                gReceivedEvent_count_risk_assessment++;

                auto riskAssessmentList = data->riskAssessmentList;

                if(!riskAssessmentList.empty()) {
                    adcm::Log::Verbose() << "=== riskAssessmentList ===";
                    for(auto itr = riskAssessmentList.begin(); itr != riskAssessmentList.end(); ++itr) {
                        adcm::Log::Verbose() << "obstacle_id : " << itr->obstacle_id;
                        
                        auto wgs84_xy_start = itr->wgs84_xy_start;
                        auto wgs84_xy_end = itr->wgs84_xy_end;

                        if(!wgs84_xy_start.empty()) {
                            adcm::Log::Verbose() << "=== wgs84_xy_start ===";
                            for(auto itr = wgs84_xy_start.begin(); itr != wgs84_xy_start.end(); ++itr) {
                                adcm::Log::Verbose() << "start position x : " << itr->x;
                                adcm::Log::Verbose() << "start position y : " << itr->y;
                            }
                        } else {
                            adcm::Log::Verbose() << "wgs84_xy_start Vector empty!!! ";
                        }

                        if(!wgs84_xy_end.empty()) {
                            adcm::Log::Verbose() << "=== wgs84_xy_end ===";
                            for(auto itr = wgs84_xy_end.begin(); itr != wgs84_xy_end.end(); ++itr) {
                                adcm::Log::Verbose() << "end position x : " << itr->x;
                                adcm::Log::Verbose() << "end position y : " << itr->y;
                            }
                        } else {
                            adcm::Log::Verbose() << "wgs84_xy_end Vector empty!!! ";
                        }

                        adcm::Log::Verbose() << "hazard_class : " << itr->hazard_class;

                        if(itr->isHarzard)
                            adcm::Log::Verbose() << "isHarzard : true ";
                        else
                            adcm::Log::Verbose() << "isHarzard : false ";

                        adcm::Log::Verbose() << "confidence : " << itr->confidence;
                    }
                } else {
                    adcm::Log::Verbose() << "riskAssessmentList Vector empty!!! ";
                }
            }
        }
    }
}

void ThreadReceiveWorkInformation()
{
    adcm::Log::Info() << "TaskManagerPathBuilder ThreadReceiveWorkInformation";
    adcm::WorkInformation_Subscriber workInformation_subscriber;
    workInformation_subscriber.init("TaskManagerPathBuilder/TaskManagerPathBuilder/RPort_work_information");

    while (continueExecution) {
        gMainthread_Loopcount++;
        VERBOSE("[TaskManagerPathBuilder] ThreadReceiveWorkInformation loop");
        bool workInformation_rxEvent = workInformation_subscriber.waitEvent(100); // wait event

        if(workInformation_rxEvent) {
            adcm::Log::Verbose() << "[EVENT] TaskManagerPathBuilder Hub Data received";

            while(!workInformation_subscriber.isEventQueueEmpty()) {
                auto data = workInformation_subscriber.getEvent();
                gReceivedEvent_count_work_information++;

                auto main_vehicle = data->main_vehicle;
                auto sub_vehicle = data->sub_vehicle;
                auto working_area_boundary = data->working_area_boundary;
                
                adcm::Log::Verbose() << "main_vehicle.length : "<< main_vehicle.length;
                adcm::Log::Verbose() << "main_vehicle.width : "<< main_vehicle.width;

                if(!sub_vehicle.empty()) {
                    adcm::Log::Verbose() << "=== sub_vehicle ===";
                    for(auto itr = sub_vehicle.begin(); itr != sub_vehicle.end(); ++itr) {
                        adcm::Log::Verbose() << "sub_vehicle.length : "<< itr->length;
                        adcm::Log::Verbose() << "sub_vehicle.width : "<< itr->width;
                    }
                } else {
                    adcm::Log::Verbose() << "sub_vehicle Vector empty!!! ";
                }

                if(!working_area_boundary.empty()) {
                    adcm::Log::Verbose() << "=== working_area_boundary ===";
                    for(auto itr = working_area_boundary.begin(); itr != working_area_boundary.end(); ++itr) {
                        adcm::Log::Verbose() << "working_area_boundary.x : "<< itr->x;
                        adcm::Log::Verbose() << "working_area_boundary.y : "<< itr->y;
                    }
                } else {
                    adcm::Log::Verbose() << "working_area_boundary Vector empty!!! ";
                }
            }
        }
    }
}

void ThreadSendBuildPath()
{
    adcm::Log::Info() << "TaskManagerPathBuilder ThreadSendBuildPath";    
    buildPath_provider = std::make_shared<adcm::BuildPath_Provider>();
    buildPath_provider->init("TaskManagerPathBuilder/TaskManagerPathBuilder/PPort_build_path");
    buildPath_provider->setCallback(setBuildPath);
    INFO("Thread ThreadSendBuildPath start...");

    while (continueExecution) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        adcm::build_path_Objects buildPath;
        adcm::pathStruct ps;
        adcm::routeStruct route;

        ps.result = m_ud_0_8(m_rand_eng);
        ps.t0 = m_ud_0_64(m_rand_eng);
        ps.vehicle_class = m_ud_0_8(m_rand_eng);
        ps.move_type = m_ud_0_8(m_rand_eng);
        ps.job_type = m_ud_0_8(m_rand_eng);
        ps.size = m_ud_0_16(m_rand_eng);

        ps.route.clear();
        for (int i = 0; i < 5; ++i){
            route.x = m_ud_10000_10000(m_rand_eng);
            route.y = m_ud_10000_10000(m_rand_eng);
            route.delta_t =  m_ud_0_16(m_rand_eng);
            ps.route.push_back(route);
        }

        buildPath.Path.clear();
        buildPath.Path.push_back(ps);

        buildPath_provider->send(buildPath);
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

            if(gReceivedEvent_count_risk_assessment != 0) {
                adcm::Log::Info() << "risk_assessment Received count = " << gReceivedEvent_count_risk_assessment;
                gReceivedEvent_count_risk_assessment = 0;

            } else {
                adcm::Log::Info() << "risk_assessment event timeout!!!";
            }

            if(gReceivedEvent_count_work_information != 0) {
                adcm::Log::Info() << "work_information Received count = " << gReceivedEvent_count_work_information;
                gReceivedEvent_count_work_information = 0;

            } else {
                adcm::Log::Info() << "work_information event timeout!!!";
            }
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

    if(!TaskManagerPathBuilder::RegisterSigTermHandler()) {
        adcm::Log::Error() << "Unable to register signal handler";
    }

#ifndef R19_11_1
    adcm::Log::Info() << "TaskManagerPathBuilder: configure e2e protection";
    bool success = ara::com::e2exf::StatusHandler::Configure("./etc/e2e_dataid_mapping.json",
                   ara::com::e2exf::ConfigurationFormat::JSON,
                   "./etc/e2e_statemachines.json",
                   ara::com::e2exf::ConfigurationFormat::JSON);
    adcm::Log::Info() << "TaskManagerPathBuilder: e2e configuration " << (success ? "succeeded" : "failed");
#endif
    adcm::Log::Info() << "Ok, let's produce some TaskManagerPathBuilder data...";

    thread_list.push_back(std::thread(TaskManagerPathBuilder::ThreadReceiveMapData));
    thread_list.push_back(std::thread(TaskManagerPathBuilder::ThreadReceiveRiskAssessment));
    thread_list.push_back(std::thread(TaskManagerPathBuilder::ThreadReceiveWorkInformation));
    thread_list.push_back(std::thread(TaskManagerPathBuilder::ThreadSendBuildPath));
    thread_list.push_back(std::thread(TaskManagerPathBuilder::ThreadMonitor));

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
