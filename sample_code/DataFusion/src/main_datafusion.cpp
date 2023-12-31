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
#include <algorithm>
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
#include "coder_bounded_array.h"
#include "coder_array.h"
#include "EDGE_fusion_function_231019_2222_types.h"

#include "main.h"
namespace
{
long ContourX[map_m][2];

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

void ScanLine(long x1, long y1, long x2, long y2, long min_y, long max_y)
{
    long sx, sy, dx1, dy1, dx2, dy2, x, y, m, n, k, cnt;

    sx = x2 - x1;
    sy = y2 - y1;

    if (sx > 0) dx1 = 1;
    else if (sx < 0) dx1 = -1;
    else dx1 = 0;

    if (sy > 0) dy1 = 1;
    else if (sy < 0) dy1 = -1;
    else dy1 = 0;

    m = ABS(sx);
    n = ABS(sy);
    dx2 = dx1;
    dy2 = 0;

    if (m < n)
    {
        m = ABS(sy);
        n = ABS(sx);
        dx2 = 0;
        dy2 = dy1;
    }

    x = x1; y = y1;
    cnt = m + 1;
    k = n / 2;

    while (cnt--)
    {
        if ((y >= min_y) && (y < max_y+1))
        {
            if (x < ContourX[y][0]) ContourX[y][0] = x;
            if (x > ContourX[y][1]) ContourX[y][1] = x;
        }

        k += n;
        if (k < m)
        {
            x += dx2;
            y += dy2;
        }
        else
        {
            k -= m;
            x += dx1;
            y += dy1;
        }
    }
}

void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, std::vector<Out_HubVehicleData>::iterator iter)
{
    long arr_x[] = {p0.x, p1.x, p2.x, p3.x};
    long arr_y[] = {p0.y, p1.y, p2.y, p3.y};
    //find max x&y and min x&y of the rectangle 
    int n = sizeof(arr_y) / sizeof(arr_y[0]);
    // Implemented inbuilt function to sort array
    std::sort(arr_x, arr_x + n);
    std::sort(arr_y, arr_y + n);
    long min_x = arr_x[0];
    long max_x = arr_x[n - 1];
    long min_y = arr_y[0];
    long max_y = arr_y[n - 1];

    int y;
    for (y = min_y; y < max_y+1; y++)
    {
        ContourX[y][0] = LONG_MAX; // min X
        ContourX[y][1] = LONG_MIN; // max X
    }
    
    ScanLine(p0.x, p0.y, p1.x, p1.y, min_y, max_y);
    ScanLine(p1.x, p1.y, p2.x, p2.y, min_y, max_y);
    ScanLine(p2.x, p2.y, p3.x, p3.y, min_y, max_y);
    ScanLine(p3.x, p3.y, p0.x, p0.y, min_y, max_y);

    for (y = min_y; y < max_y+1; y++)
    {
        if (ContourX[y][1] >= ContourX[y][0])
        {
            long x = ContourX[y][0];
            long len = 1 + ContourX[y][1] - ContourX[y][0];

            // Can draw a horizontal line instead of individual pixels here
            while (len--)
            {
                //occupied
                iter->map_2d_location.push_back(std::make_pair(x,y));
                x++;
            }
        }
    }                          
}
void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, std::vector<Out_HubObstacleData>::iterator iter)
{
    long arr_x[] = {p0.x, p1.x, p2.x, p3.x};
    long arr_y[] = {p0.y, p1.y, p2.y, p3.y};
    //find max x&y and min x&y of the rectangle 
    int n = sizeof(arr_y) / sizeof(arr_y[0]);
    // Implemented inbuilt function to sort array
    std::sort(arr_x, arr_x + n);
    std::sort(arr_y, arr_y + n);
    long min_x = arr_x[0];
    long max_x = arr_x[n - 1];
    long min_y = arr_y[0];
    long max_y = arr_y[n - 1];

    int y;
    for (y = min_y; y < max_y+1; y++)
    {
        ContourX[y][0] = LONG_MAX; // min X
        ContourX[y][1] = LONG_MIN; // max X
    }
    
    ScanLine(p0.x, p0.y, p1.x, p1.y, min_y, max_y);
    ScanLine(p1.x, p1.y, p2.x, p2.y, min_y, max_y);
    ScanLine(p2.x, p2.y, p3.x, p3.y, min_y, max_y);
    ScanLine(p3.x, p3.y, p0.x, p0.y, min_y, max_y);

    for (y = min_y; y < max_y+1; y++)
    {
        if (ContourX[y][1] >= ContourX[y][0])
        {
            long x = ContourX[y][0];
            long len = 1 + ContourX[y][1] - ContourX[y][0];

            // Can draw a horizontal line instead of individual pixels here
            while (len--)
            {
                //occupied
                iter->map_2d_location.push_back(std::make_pair(x,y));
                x++;
            }
        }
    }                          
}


}  // namespace

//==============1.MapData 생성 =================

MapData map_data;

 
//=============================================

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

                    // mapData_provider.send(mapData);
 
//==============2.데이터 융합=================
// i)차량 정보 ii) 노면 정보 iii)장애물 정보를 융합
                    HubData hub_data_raw; //관제에서 데이터 수신 - 인풋 
 //                   FusionData hub_data = matlab_fusion(hub_data_raw);
                    FusionData fused_data_out; // matlab 모듈에서 나온 아웃풋

                    VehicleData main_vehicle={};
                    main_vehicle.vehicle_class = EGO_VEHICLE;
                    main_vehicle.position_x = 3;
                    main_vehicle.position_y = 1;
                    main_vehicle.road_z.push_back(11);

                    ObstacleData obstacle1={};

                    fused_data_out.vehicle_list.push_back(main_vehicle);
                    fused_data_out.obstacle_list.push_back(obstacle1);

//TODO 측위 좌표계 변환 - 현재 시뮬레이션 데이터 검증목표라 필요X

//==============2.1. 차량별 장애물의 속도 좌표계 절대값으로 변환=================
                    // for (auto iter = hub_data.vehicle.begin(); iter!=hub_data.vehicle.end(); iter++)
                    // {
                    //     for (auto iter1 = iter->obstacle.begin(); iter1!=iter->obstacle.end(); iter1++)
                    //     {
                    //         //장애물 속도 좌표계 변환 TODO 수식추가
                    //         iter1->velocity_x = iter1->velocity_x;
                    //         iter1->velocity_y = iter1->velocity_y;
                    //     }

                    //     //차량 속도 좌표계 변환 TODO 수식추가
                    //     iter->velocity_x = iter->velocity_long;
                    //     iter->velocity_y = iter->velocity_lat;
                    // }

// //==============2.2. 차량별 장애물의 위치 좌표계 절대값으로 변환=================
//                     for (auto iter = hub_data.vehicle.begin(); iter!=hub_data.vehicle.end(); iter++)
//                     {
//                         for (auto iter1 = iter->obstacle.begin(); iter1!=iter->obstacle.end(); iter1++)
//                         {
//                             //장애물 위치 좌표계 변환 TODO 수식추가
//                             iter1->position_x = iter1->position_x;
//                             iter1->position_y = iter1->position_y;
//                         }
//                         //차량 위치 좌표계 변환 TODO 수식추가
//                         iter->position_x = iter->position_long;
//                         iter->position_y = iter->position_lat;

//                     }

//==============2.3. 장애물 위치 보정 (엣지에서 처리 기준시점)=================
                    // m개의 장애물이 인지되어서 데이터가 옴
                    //엣지에서 처리를 시작한 시간을 기준시점 레퍼런스 타임으로 지정
                    std::time_t reference_time = std::time(0);

                    for (auto iter = fused_data_out.obstacle_list.begin(); iter!=fused_data_out.obstacle_list.end(); iter++)
                    {
                        iter->position_x = iter->position_x *(reference_time - iter->timestamp) * 100 * iter->velocity_x;
                        iter->position_y = iter->position_y *(reference_time - iter->timestamp) * 100 * iter->velocity_y;
                        //시차는 ms 단위라 100 곱하고 속도는 m/s 이라고 가정
                    }

//==============2.3. 0.1 m/s 미만인 경우 장애물 정지 상태 판정 및 map 데이터에 반영 =================
                    for (auto iter = fused_data_out.obstacle_list.begin(); iter!=fused_data_out.obstacle_list.end(); iter++)
                    {
                        if (iter->fused_position_x < 0.1 && iter -> fused_position_y < 0.1 && iter->fused_position_z < 0.1)
                        {
                            iter->stop_count = 1; //해당 시각 물체 정지상태
                        }
                        else 
                        {
                            iter->stop_count = 0;
                        }
                    }

//이 전에 장애물의 stop status 를 카운트 하는 카운터 값 변동
                    for (auto iter = map_data.obstacle_list.begin(); iter!=map_data.obstacle_list.end(); iter++)
                    {
                        for (auto iter1 = fused_data_out.obstacle_list.begin(); iter1 != fused_data_out.obstacle_list.end(); iter1++)
                        {
                            if (iter->obstacle_id == iter1->obstacle_id)
                            { //동일 장애물
                                if (iter1->stop_count == 1)
                                {
                                    iter->stop_count++;
                                }
                                //한번이라도 속도가 0 이 아니면 카운트 리셋
                                else iter->stop_count = 0;
                                break;
                            }
                        }
                    }
//==============2.4. 퓨전 후 데이터를 map data 형식으로 재구성해서 overwrite=================
// 맵 데이터 최초 생성시에는 동일한 장애물 id 가 없기에 해당사항 없음 => 단순이 벡터 assign  
                    map_data.vehicle_list.assign(fused_data_out.vehicle_list.begin(), fused_data_out.vehicle_list.end());
                    map_data.obstacle_list.assign(fused_data_out.obstacle_list.begin(), fused_data_out.obstacle_list.end());

//==============3.1. MapData에 노면데이터 추가 =================
                    //TO DO:차량이 작업공간을 정찰할 동안 해당 그리드에 맞는 노면데이터 업데이트 
                    //해당 차량의 위치 - 위경도 정보로 업데이트 그리드의 인덱스를 찾아내는 수식 만들어야 함 

                    for (auto iter = map_data.vehicle_list.begin(); iter != map_data.vehicle_list.end(); iter++)
                    {
                        for (auto iter1 = iter->road_z.begin(); iter1 != iter->road_z.end(); iter1++)
                        {
                            // TODO:차량의 위치에 기반해 주변 노면 정보 업데이트
                            // 현재 차량과 인지 노면데이터의 위치 관계가 불분명하므로 '차량의 위치 = 업데이트 되는 노면 위치' 라는 단순한 가정의 dummy code 로 대체 
                            int i = iter->position_x * 10;
                            int j = iter->position_y * 10;
                            map_data.map_2d[i][j].road_z= *iter1;
                            // 미터 단위의 position_x 를 10cm 그리드 셀 기준으로 하면 position_x *100 /10 = position_x*10 th grid cell
                        }
                    }

                    adcm::Log::Info() << "road info is " << map_data.map_2d[23][4142].road_z;
                    adcm::Log::Info() << "road info is " << map_data.map_2d[0][0].road_z;

//==============3.2. MapData에 메인/보조차량 정보 업데이트 ==============================
    //============== i) 차량의 2d 그리드 맵 인덱스 페어 찾아서 저장 ================
                    //우선 4 vertices 안다고 가정
                    //TODO: 중심점+rotation angle + cuboid_x/y 값으로 구하기 

                    for (auto iter = map_data.vehicle_list.begin(); iter != map_data.vehicle_list.end(); iter++)
                    {
                        Point2D p0, p1, p2, p3;

                        p0.x = 62;
                        p0.y = 113;

                        p1.x = 89;
                        p1.y = 79;

                        p2.x = 153;
                        p2.y = 115;

                        p3.x = 130;
                        p3.y = 150;

                        generateOccupancyIndex(p0, p1, p2, p3, *(&iter));
                    }

    //============== ii) 장애물의 2d 그리드 맵 인덱스 페어 찾아서 저장 ================                  

                    for (auto iter = hub_data.obstacle.begin(); iter!=hub_data.obstacle.end(); iter++)
                    {
                        Point2D p0, p1, p2, p3;

                        p0.x = 62;
                        p0.y = 113;

                        p1.x = 89;
                        p1.y = 79;

                        p2.x = 153;
                        p2.y = 115;

                        p3.x = 130;
                        p3.y = 150;
                        generateOccupancyIndex(p0, p1, p2, p3, *(&iter));
                    }
                    
    //============== iii) 2d MapData가 해당 차량과 장애물 ID 정보를 가지도록 설정 ================

                    //리스트를 iterate 하면서 index pair를 가져와서 MapData 의 index pair 가 해당 정보를 가지도록 설정
                    for (auto iter = map_data.vehicle_list.begin(); iter != map_data.vehicle_list.end(); iter++)
                    {
                        for (auto iter1 = iter->map_2d_location.begin(); iter1!= iter->map_2d_location.end(); iter1++)
                        {
                            //map_data.map_2d[iter1->first][iter1->second].vehicle_data = &(*iter);
                            map_data.map_2d[iter1->first][iter1->second].vehicle_class = iter->vehicle_class;
                        }
                    }

                    for (auto iter = map_data.obstacle_list.begin(); iter != map_data.obstacle_list.end(); iter++)
                    {
                        for (auto iter1 = iter->map_2d_location.begin(); iter1!= iter->map_2d_location.end(); iter1++)
                        {
                            map_data.map_2d[iter1->first][iter1->second].obstacle_id = iter->obstacle_id;
                        }
                    }


//==============4. 다른 모듈로 데이터 전달=======================================

                    // mapData_provider.send(map_data);
                    //다른 모듈로 map_data 오브젝트 전달

                    mapData_provider.send(mapData); //원본
                //    adcm::Log::Info() << "obstacle id saved in [9][20] is  " << map_data.map_2d[9][20].obstacle_data->obstacle_id;
                    adcm::Log::Info() << "size of MapData is  " << sizeof(map_data);
                //    adcm::Log::Info() << "starting address of map_2d is  " << &(map_data.map_2d[0][0].obstacle_data);
                //    adcm::Log::Info() << "ending address of map_2d is  " << &(map_data.map_2d[3999][4999].vehicle_data);
                }
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
