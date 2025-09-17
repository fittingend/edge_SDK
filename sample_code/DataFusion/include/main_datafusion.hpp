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
#include <string>
#include <algorithm>
#include <queue>
#include <unistd.h>
#include <limits.h>
#include <libgen.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>

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
#include "NatsConnManager.h"

#define INVALID_VALUE 999

#define M_TO_10CM_PRECISION 10.0
#define MAP_ANGLE -86

using namespace std;

// 사이즈는 10cm 단위 기준

///////////////////////////////////////////////////////////////////////
// 필드 목록

// 맵 x, y 방향 사이즈
std::uint16_t map_x = 2000;
std::uint16_t map_y = 1000;
std::uint8_t type = 0; // 시뮬레이션 = 0, 실증 = 1
// 맵(0,0)지점의 utm좌표
double origin_x = 0;
double origin_y = 0;
// 실증 상황 사각형 맵의 끝지점 x, y값
double min_utm_x, min_utm_y, max_utm_x, max_utm_y;

bool useNats = false;
bool saveJson = false;

IDManager id_manager; // 장애물 ID 부여 및 반환
bool ego = false;
bool sub1 = false;
bool sub2 = false;
bool workego = false;
bool worksub1 = false;
bool worksub2 = false;
bool get_workinfo = false;

mutex mtx_data;
mutex mtx_map_someip;
mutex mtx_map_nats;
mutex mtx_send;
condition_variable dataReady;
condition_variable someipReady;
condition_variable natsReady;
std::uint8_t send_map = 0;

queue<adcm::map_data_Objects> map_someip_queue;
queue<adcm::map_data_Objects> map_nats_queue;

mutex mtx;

int map_2d_size = 0;

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
    double lon;
    double lat;
};

struct VehicleData
{
    unsigned char vehicle_class;
    std::vector<Point2D> map_2d_location;
    std::uint64_t timestamp;
    std::vector<int> road_z;
    //    double road_z[4];
    double position_long; // wgs84좌표 -> x
    double position_lat;  // wgs84좌표 -> y
    double position_height;
    double position_x; // 맵상 x좌표
    double position_y; // 맵상 y좌표
    double position_z;
    double heading_angle;
    double velocity_long;
    double velocity_lat;
    double velocity_x; // 맵상 x방향 속도
    double velocity_y; // 맵상 y방향 속도
    double velocity_ang;
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

struct FusionData
{
    VehicleData vehicle;
    std::vector<ObstacleData> obstacle_list;
};

// moodycamel::ConcurrentQueue<FusionData> main_vehicle_queue;
// moodycamel::ConcurrentQueue<FusionData> sub1_vehicle_queue;
// moodycamel::ConcurrentQueue<FusionData> sub2_vehicle_queue;

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

// 차량 크기(work_information data)
VehicleSizeData main_vehicle_size;
std::vector<VehicleSizeData> sub_vehicle_size;
std::vector<BoundaryData> work_boundary;
double min_lon, min_lat, max_lon, max_lat;

bool sendEmptyMap = false;
int receiveVer = 0;
adcm::map_data_Objects mapData;

///////////////////////////////////////////////////////////////////////
// 함수 목록

// nats json 만드는 함수
Poco::JSON::Object::Ptr buildMapDataJson(const adcm::map_data_Objects &mapData);
void saveMapDataJsonFile(const std::string &filePrefix, Poco::JSON::Object::Ptr mapObj, int &fileCount);
void clearJsonDirectory(const std::string &dirPath);

// wgs84 -> utm 좌표변환
void GPStoUTM(double lat, double lon, double &utmX, double &utmY);

// 좌표 변환 후 맵 오버플로우 방지
bool checkRange(const VehicleData &vehicle);
void checkRange(Point2D &point);

bool checkAllVehicleRange(const std::vector<VehicleData *> &vehicles);

// 차량 데이터 처리
void processVehicleData(FusionData &vehicleData,
                        VehicleData &vehicle,
                        std::vector<ObstacleData> &obstacleList);
// 메인차량, 서브차량 리스트에서 제외
void filterVehicleData(std::vector<ObstacleData> &obstacles);
void gpsToMapcoordinate(VehicleData &vehicle);
void relativeToMapcoordinate(std::vector<ObstacleData> &obstacle_list, VehicleData vehicle);

// 차량주변 road_z 수정 -> road_index로 수정 예정
void generateRoadZValue(VehicleData target_vehicle, std::vector<adcm::map_2dListVector> &map_2d_test);

// 차량, 장애물의 꼭짓점 좌표 판단
void find4VerticesVehicle(VehicleData &target_vehicle);
void find4VerticesObstacle(std::vector<ObstacleData> &obstacle_list_filtered);

// map_2d_location 계산
void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, VehicleData &vehicle);
void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, std::vector<ObstacleData>::iterator iter);

// 삼각형 내부 포함 판단 함수 (레이캐스팅 대체)
bool isPointInTriangle(const Point2D &pt, const Point2D &v1, const Point2D &v2, const Point2D &v3);

// 차량 데이터 저장
void fillVehicleData(VehicleData &vehicle_fill, const std::shared_ptr<adcm::hub_data_Objects> &data);

// 장애물 데이터 저장
void fillObstacleList(std::vector<ObstacleData> &obstacle_list_fill, const std::shared_ptr<adcm::hub_data_Objects> &data);

// 장애물 리스트 처리 함수 //
// 유클리디안 거리 계산
double euclideanDistance(const ObstacleData &a, const ObstacleData &b);

// 거리 행렬 생성
std::vector<std::vector<double>> createDistanceMatrix(const std::vector<ObstacleData> &listA, const std::vector<ObstacleData> &listB);

// 신뢰성 기반 융합 계산
double calculateWeightedPosition(const std::vector<double> &positions, const std::vector<double> &variances);

// Munkres 알고리즘으로 매칭
std::vector<int> solveAssignment(const std::vector<std::vector<double>> &costMatrix);

// 장애물 데이터 융합
void processFusion(
    std::vector<ObstacleData> &presList,
    const std::vector<ObstacleData> &prevList,
    const std::vector<int> &assignment);
// void processFusion(std::vector<ObstacleData> &fusedList, const std::vector<ObstacleData> &listB, const std::vector<int> &assignment);

// 장애물 리스트 융합 및 이전 데이터와 비교
std::vector<ObstacleData> mergeAndCompareLists(
    const std::vector<ObstacleData> &previousFusionList,
    std::vector<ObstacleData> listMain,
    std::vector<ObstacleData> listSub1,
    std::vector<ObstacleData> listSub2,
    const VehicleData &mainVehicle,
    const VehicleData &sub1Vehicle,
    const VehicleData &sub2Vehicle);

void processWorkingAreaBoundary(const std::vector<BoundaryData> &work_boundary);

// stop_count 업데이트
void updateStopCount(std::vector<ObstacleData> &mergedList,
                     const std::vector<ObstacleData> &previousFusionList,
                     double threshold = 0.1);

// 차량 리스트, 장애물 리스트 맵데이터에 반영
void UpdateMapData(adcm::map_data_Objects &mapData, const std::vector<ObstacleData> &obstacle_list, const std::vector<VehicleData> &vehicles);

// road_z를 맵 좌표로 변환해 roadListStruct로 만드는 함수
std::vector<adcm::roadListStruct> ConvertRoadZToRoadList(const VehicleData &vehicle);

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