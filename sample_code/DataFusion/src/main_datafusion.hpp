#ifndef __MAIN_DATAFUSION_HPP__
#define __MAIN_DATAFUSION_HPP__
#include <thread>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <csignal>
#include <stdio.h>
#include <random>
#include <vector>

#include <ara/com/e2exf/status_handler.h>
#include <ara/exec/execution_client.h>

#include <ara/log/logger.h>
#include "logger.h"
#include "ara/core/initialization.h"

#include "map_data_provider.h"

#include "hub_data_subscriber.h"
#include "work_information_subscriber.h"
#include "id_manager.hpp"

#define map_n 2500
#define map_m 1500
#define ABS(x) ((x >= 0) ? x : -x)
#define INVALID_VALUE 999

#define M_TO_10CM_PRECISION 10.0
#define MAIN_VEHICLE_SIZE_X 10
#define MAIN_VEHICLE_SIZE_Y 10

#define SUB_VEHICLE_SIZE_X 21.92
#define SUB_VEHICLE_SIZE_Y 15.99
#define MAP_ANGLE -86
// 사이즈는 10cm 단위 기준

// #define KM_TO_MS_CONVERSION 5/18


///////////////////////////////////////////////////////////////////////
// 필드 목록
IDManager id_manager; // 장애물 ID 부여 및 반환

enum VehicleClass
{
    EGO_VEHICLE = 240,
    SUB_VEHICLE_1 = 0,
    SUB_VEHICLE_2 = 1,
    SUB_VEHICLE_3 = 2,
    SUB_VEHICLE_4 = 3,
    NO_VEHICLE = 4
};

struct Point2D
{
    long x, y;
};

struct ObstacleData
{
    std::uint16_t obstacle_id;
    std::uint8_t obstacle_class;
    std::uint64_t timestamp;
    //    std::vector<std::pair<unsigned short,unsigned short>> map_2d_location; //장애물이 위치한 2d 그리드 맵의 index 페어를 저장
    std::vector<Point2D> map_2d_location;
    std::uint8_t stop_count;
    double fused_cuboid_x;
    double fused_cuboid_y;
    double fused_cuboid_z;
    double fused_heading_angle;
    double fused_position_x;
    double fused_position_y;
    double fused_position_z;
    double fused_velocity_x;
    double fused_velocity_y;
    double fused_velocity_z;
};

struct VehicleSizeData
{
    std::uint16_t length;
    std::uint16_t width;
};

struct BoundaryData
{
    double x;
    double y;
};

struct VehicleData
{
    unsigned char vehicle_class;
    //    std::vector<std::pair<unsigned short,unsigned short>> map_2d_location; //해당 차량이 위치한 2d 그리드 맵의 index 페어를 저장
    //    std::vector<Point2D>
    std::vector<Point2D> map_2d_location;
    //    std::time_t timestamp;
    std::uint64_t timestamp;
    std::vector<double> road_z;
    //    double road_z[4];
    double position_long; // x equivalent
    double position_lat;  // y equivalent
    double position_height;
    double position_x; // new - to assign
    double position_y; // new - to assign
    double position_z;
    double yaw;
    double roll;
    double pitch;
    double velocity_long;
    double velocity_lat;
    double velocity_x; // new- to assign
    double velocity_y; // new- to assign
    double velocity_ang;
};

struct GridCellData
{
    unsigned short obstacle_id;
    VehicleClass vehicle_class;
    double road_z; // 2 bytes
}; // 패딩때문에 토탈 24 bytes

struct MapData
{
    GridCellData map_2d[map_n][map_m]; // 각각 24bytes
    std::vector<ObstacleData> obstacle_list;
    std::vector<VehicleData> vehicle_list;

}; // maptdata 사이즈는 24*4000*5000+24+24 = 480000048 byte = 약 450MB

struct FusionData
{
    std::vector<ObstacleData> obstacle_list;
    std::vector<VehicleData> vehicle_list;
};

enum ObstacleClass
{
    NO_OBSTACLE,
    VEHICLE_LARGE,
    VEHICLE_SMALL,
    PEDESTRIAN,
    STRUCTURE
};
/*
enum VehicleClass
{
    EGO_VEHICLE,
    SUB_VEHICLE_1,
    SUB_VEHICLE_2,
    SUB_VEHICLE_3,
    SUB_VEHICLE_4,
    NO_VEHICLE
};
*/

VehicleData main_vehicle_temp;
VehicleData sub1_vehicle_temp;
VehicleData sub2_vehicle_temp;

std::vector<ObstacleData> obstacle_list_temp;
long ContourX[map_m][2];
bool once = 1;
// 차량 크기(work_information data)

double utmOrigin_x, utmOrigin_y;

std::uint16_t main_vehicle_size_length;
std::uint16_t main_vehicle_size_width;
VehicleSizeData main_vehicle_size;
std::vector<VehicleSizeData> sub_vehicle_size;
std::vector<BoundaryData> work_boundary;
double min_a, min_b, max_a, max_b;




///////////////////////////////////////////////////////////////////////
// 함수 목록 
void GPStoUTM(double lat, double lon, double &utmX, double &utmY);

bool checkRange(VehicleData vehicle);
void checkRange(Point2D &point);
bool checkRange(int x, int y);

void gpsToMapcoordinate(VehicleData &vehicle);
void relativeToMapcoordinate(std::vector<ObstacleData> &obstacle_list, VehicleData vehicle);
void ScanLine(long x1, long y1, long x2, long y2, long min_y, long max_y);

void generateRoadZValue(VehicleData target_vehicle, std::vector<adcm::map_2dListVector> &map_2d_test);
void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, VehicleData &vehicle, std::vector<adcm::map_2dListVector> &map_2d_test);
void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, std::vector<ObstacleData>::iterator iter);

void find4VerticesVehicle(VehicleData &target_vehicle, std::vector<adcm::map_2dListVector> &map_2d_test);
void find4VerticesObstacle(std::vector<ObstacleData> &obstacle_list_filtered);

void ThreadReceiveHubData();
void ThreadReceiveWorkInfo();
void ThreadKatech();
void ThreadMonitor();

//리눅스 Sigterm 관리
namespace
{

    // Atomic flag for exit after SIGTERM caught
    std::atomic_bool continueExecution{true};
    std::atomic_uint gReceivedEvent_count_hub_data{0};
    std::atomic_uint gMainthread_Loopcount{0};

    void SigTermHandler(int signal)
    {
        if (signal == SIGTERM)
        {
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
        if (sigaction(SIGTERM, &sa, NULL) == -1)
        {
            // Could not register a SIGTERM signal handler
            return false;
        }

        return true;
    }

} // namespace


#endif