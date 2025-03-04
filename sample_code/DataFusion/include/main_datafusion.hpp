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
#include <algorithm>
#include <queue>

#include <ara/com/e2exf/status_handler.h>
#include <ara/exec/execution_client.h>

#include <ara/log/logger.h>
#include "logger.h"
#include "ara/core/initialization.h"

#include "map_data_provider.h"

#include "hub_data_subscriber.h"
#include "work_information_subscriber.h"
#include "edge_information_subscriber.h"
#include "id_manager.hpp"
#include "concurrentqueue.h"
#include "munkres/src/munkres.h"

#define map_n 2000
#define map_m 1000
#define ABS(x) ((x >= 0) ? x : -x)
#define INVALID_VALUE 999

#define M_TO_10CM_PRECISION 10.0
#define MAIN_VEHICLE_SIZE_X 10
#define MAIN_VEHICLE_SIZE_Y 10

#define SUB_VEHICLE_SIZE_X 21.92
#define SUB_VEHICLE_SIZE_Y 15.99
#define MAP_ANGLE -86

using namespace std;
// 사이즈는 10cm 단위 기준

// #define KM_TO_MS_CONVERSION 5/18

///////////////////////////////////////////////////////////////////////
// 필드 목록
IDManager id_manager; // 장애물 ID 부여 및 반환
bool ego = false;
bool sub1 = false;
bool sub2 = false;

mutex mtx_data;
mutex mtx_map;
condition_variable dataReady;
condition_variable mapReady;
std::uint8_t send_map = 0;

mutex mtx;

enum VehicleClass
{
    EGO_VEHICLE = 0xF0,
    SUB_VEHICLE_1 = 0x01,
    SUB_VEHICLE_2 = 0x02,
    SUB_VEHICLE_3 = 0x02,
    SUB_VEHICLE_4 = 0x03,
    NO_VEHICLE = 0x04
};

struct Point2D
{
    double x, y;
};

struct ObstacleData
{
    std::uint16_t obstacle_id;
    std::uint8_t obstacle_class;
    std::uint64_t timestamp = 0;
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
    double standard_deviation; // 오차의 표준편차(추가)
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
    std::vector<Point2D> map_2d_location;
    std::uint64_t timestamp;
    std::vector<int> road_z;
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
    std::uint16_t obstacle_id;
    VehicleClass vehicle_class;
    int road_z; // 2 bytes
}; // 패딩때문에 토탈 24 bytes

struct MapData
{
    GridCellData map_2d[map_n][map_m]; // 각각 24bytes
    std::vector<ObstacleData> obstacle_list;
    std::vector<VehicleData> vehicle_list;

}; // maptdata 사이즈는 24*4000*5000+24+24 = 480000048 byte = 약 450MB

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

struct FusionData
{
    VehicleData vehicle;
    std::vector<ObstacleData> obstacle_list;
};

moodycamel::ConcurrentQueue<FusionData> main_vehicle_queue;
moodycamel::ConcurrentQueue<FusionData> sub1_vehicle_queue;
moodycamel::ConcurrentQueue<FusionData> sub2_vehicle_queue;

queue<int> order;

// 허브데이터 수신 시 사용하는 임시 데이터
VehicleData main_vehicle_temp;
VehicleData sub1_vehicle_temp;
VehicleData sub2_vehicle_temp;
std::vector<ObstacleData> obstacle_list_temp;

// 맵데이터 생성 시 사용하는 데이터
FusionData main_vehicle_data;
FusionData sub1_vehicle_data;
FusionData sub2_vehicle_data;
std::vector<ObstacleData> obstacle_list_main;
std::vector<ObstacleData> obstacle_list_sub1;
std::vector<ObstacleData> obstacle_list_sub2;
VehicleData main_vehicle;
VehicleData sub1_vehicle;
VehicleData sub2_vehicle;
std::vector<VehicleData *> vehicles = {&main_vehicle, &sub1_vehicle, &sub2_vehicle};

// 이전 TimeStamp의 Obstacle_list
std::vector<ObstacleData> previous_obstacle_list;

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

// bool haveWorkInfo = false;
bool sendEmptyMap = false;
int receiveVer = 0;
adcm::map_data_Objects mapData;

///////////////////////////////////////////////////////////////////////
// 함수 목록
void GPStoUTM(double lat, double lon, double &utmX, double &utmY);

bool checkRange(const VehicleData &vehicle);
void checkRange(Point2D &point);
bool checkAllVehicleRange(const std::vector<VehicleData *> &vehicles);

void processVehicleData(moodycamel::ConcurrentQueue<FusionData> &vehicleQueue,
                        FusionData &vehicleData,
                        VehicleData &vehicle,
                        std::vector<ObstacleData> &obstacleList,
                        bool &vehicleFlag);
void gpsToMapcoordinate(VehicleData &vehicle);
void relativeToMapcoordinate(std::vector<ObstacleData> &obstacle_list, VehicleData vehicle);

void generateRoadZValue(VehicleData target_vehicle, std::vector<adcm::map_2dListVector> &map_2d_test);
void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, VehicleData &vehicle);
void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, std::vector<ObstacleData>::iterator iter);

void find4VerticesVehicle(VehicleData &target_vehicle);
void find4VerticesObstacle(std::vector<ObstacleData> &obstacle_list_filtered);

// 차량 데이터 저장
void fillVehicleData(VehicleData &vehicle_fill, const std::shared_ptr<adcm::hub_data_Objects> &data);

// 장애물 데이터 저장
void fillObstacleList(std::vector<ObstacleData> &obstacle_list_fill, const std::shared_ptr<adcm::hub_data_Objects> &data);

// 유클리디안 거리 계산
double euclideanDistance(const ObstacleData &a, const ObstacleData &b);

// 거리 행렬 생성
std::vector<std::vector<double>> createDistanceMatrix(const std::vector<ObstacleData> &listA, const std::vector<ObstacleData> &listB);

// 신뢰성 기반 융합 계산 함수
double calculateWeightedPosition(const std::vector<double> &positions, const std::vector<double> &variances);

// 헝가리안 알고리즘으로 매칭
std::vector<int> solveAssignment(const std::vector<std::vector<double>> &costMatrix);

// 메인차량, 서브차량 리스트에서 제외
void filterVehicleData(std::vector<ObstacleData> &obstacles);

// 장애물 데이터 병합
void processFusion(
    std::vector<ObstacleData> &presList,
    const std::vector<ObstacleData> &prevList,
    const std::vector<int> &assignment);
// void processFusion(std::vector<ObstacleData> &fusedList, const std::vector<ObstacleData> &listB, const std::vector<int> &assignment);

// 새 데이터에 대한 ID 관리 및 부여
void assignIDsForNewData(std::vector<ObstacleData> &resultFusionList,
                         const std::vector<ObstacleData> &currentFusionList,
                         const std::vector<int> &assignment);

// 메인 융합 함수
std::vector<ObstacleData> mergeAndCompareLists(
    const std::vector<ObstacleData> &previousFusionList,
    std::vector<ObstacleData> listMain,
    std::vector<ObstacleData> listSub1,
    std::vector<ObstacleData> listSub2,
    const VehicleData &mainVehicle,
    const VehicleData &sub1Vehicle,
    const VehicleData &sub2Vehicle);

// 장애물이 보조차량, 메인차량인지 확인
const double POSITION_TOLERANCE = 14.0;

void InitializeMapData(adcm::map_data_Objects& mapData);

void UpdateMapData(adcm::map_data_Objects& mapData, const std::vector<ObstacleData>& obstacle_list, const std::vector<VehicleData>& vehicles);

// VehicleData -> vehicleListStruct(맵데이터 호환)
adcm::vehicleListStruct ConvertToVehicleListStruct(const VehicleData &vehicle, std::vector<adcm::map_2dListVector> &map);

// ObstacleData -> obstacleListStruct(맵데이터 호환)
adcm::obstacleListStruct ConvertToObstacleListStruct(const ObstacleData &obstacle, std::vector<adcm::map_2dListVector> &map);

void ThreadReceiveHubData();
void ThreadReceiveWorkInfo();
void ThreadKatech();
void ThreadMonitor();

// 리눅스 Sigterm 관리
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