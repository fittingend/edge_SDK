#ifndef MAIN_RISKASSESSMENT_HPP
#define MAIN_RISKASSESSMENT_HPP

// ==== C++ 표준 라이브러리 헤더 ====
#include <thread>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <csignal>
#include <unordered_set>
#include <mutex>
#include <atomic>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <utility>

// ==== ARA & Logger 관련 헤더 ====
#include <ara/com/e2exf/status_handler.h>
#include <ara/exec/execution_client.h>
#include <ara/log/logger.h>
#include "logger.h"
#include "ara/core/initialization.h"

// ==== Project Specific Headers ====
#include "risk_assessment_provider.h"
#include "build_path_subscriber.h"
#include "map_data_subscriber.h"
#include "work_information_subscriber.h"

// ==== 상수 정의 ====
#define STOP_VALUE 30                // STOP_VALUE 1이 약 300ms 30 이상이면 10초정도 정지 시 정지 장애물로 판단
//#define INVALID_RETURN_VALUE 99999 
#define M_TO_10CM_PRECISION 10 
#define WHEEL_DIAMETER_M 0.71
#define MAP_ANGLE -86

// 맵 x, y 방향 사이즈
extern std::uint16_t map_x;
extern std::uint16_t map_y;
// 맵(0,0)지점의 utm좌표
extern double origin_x;
extern double origin_y;
extern std::uint8_t type;
// ==== 타입 정의 ====
struct Point2D {
    double x, y;
};

enum class ObstacleClass : uint8_t {
    NONE = 0,   // 없음

    // 건설기계류 (1 ~ 9)
    EXCAVATOR       = 1,  // 굴착기
    FORKLIFT        = 2,  // 지게차
    CONCRETE_MIXER  = 3,  // 콘크리트 믹서트럭
    CRANE           = 4,  // 기중기
    CONCRETE_PUMP   = 5,  // 콘크리트 펌프
    DUMP_TRUCK      = 6,  // 덤프트럭

    // 차량류 (10 ~ 19)
    SEDAN       = 10,  // 승용차
    SUV         = 11,  // SUV
    TRUCK       = 12,  // 트럭
    OTHER_VEH   = 13,  // 기타차량
    MOTORCYCLE  = 14,  // 오토바이
    BICYCLE     = 15,  // 자전거

    // 사람 (20 ~ 29)
    PEDESTRIAN  = 20,  // 일반인
    SAFETY_ROBOT = 21, // 안전유도로봇

    // 건축자재 (30 ~ 39)
    PIPE        = 30,  // 비계파이프
    CEMENT      = 31,  // 시멘트
    BRICK       = 32,  // 벽돌

    // 안전용품 (40 ~ 49)
    DRUM        = 40,  // 드럼통
    SHIELD      = 41,  // 가림막
    LARGE_CONE  = 42,  // 대형 라바콘
    SIGNBOARD   = 43   // 입간판
};

enum VehicleClass : uint8_t {
    EGO_VEHICLE    = 0xF0,
    SUB_VEHICLE_1  = 0x01,
    SUB_VEHICLE_2  = 0x02,
    SUB_VEHICLE_3  = 0x03,
    SUB_VEHICLE_4  = 0x04,
    NO_VEHICLE     = 0x05
};

enum HazardClass {
    NO_HAZARD,
    SCENARIO_1,
    SCENARIO_2,
    SCENARIO_3,
    SCENARIO_4,
    SCENARIO_5,
    SCENARIO_6,
    SCENARIO_7,
    SCENARIO_8,
    SCENARIO_9,
    SCENARIO_10
};

struct BoundaryData
{
    double lon;
    double lat;
};
// ==== 전역 변수 ====
//extern std::vector<adcm::map_2dListVector> map_2d;
//extern obstacleListVector obstacle_list_temp;
//extern adcm::adcm::vehicleListStruct ego_vehicle_temp, sub_vehicle_1_temp, sub_vehicle_2_temp, sub_vehicle_3_temp, sub_vehicle_4_temp;

// === RiskScenarios 함수 선언 ===
// 시나리오 1 ~ 6: 공통 파라미터
extern void evaluateScenario1(const obstacleListVector& obstacle_list,
                        const adcm::vehicleListStruct& ego_vehicle,
                        const std::vector<double>& path_x,
                        const std::vector<double>& path_y,
                        adcm::risk_assessment_Objects& riskAssessment);

void evaluateScenario2(const obstacleListVector& obstacle_list,
                        const adcm::vehicleListStruct& ego_vehicle,
                        const std::vector<double>& path_x,
                        const std::vector<double>& path_y,
                        adcm::risk_assessment_Objects& riskAssessment);

void evaluateScenario3(const obstacleListVector& obstacle_list,
                        const adcm::vehicleListStruct& ego_vehicle,
                        adcm::risk_assessment_Objects& riskAssessment);

void evaluateScenario4(const obstacleListVector& obstacle_list,
                        const adcm::vehicleListStruct& ego_vehicle,
                        adcm::risk_assessment_Objects& riskAssessment);

void evaluateScenario5(const obstacleListVector& obstacle_list,
                        const adcm::vehicleListStruct& ego_vehicle,
                        const std::vector<double>& path_x,
                        const std::vector<double>& path_y,
                        adcm::risk_assessment_Objects& riskAssessment);
void resetScenario5State();
void evaluateScenario6(const obstacleListVector& obstacle_list,
                        const adcm::vehicleListStruct& ego_vehicle,
                        const std::vector<double>& path_x,
                        const std::vector<double>& path_y,
                        adcm::risk_assessment_Objects& riskAssessment);
void resetScenario6State();

// 시나리오 7: 경로와 맵 정보 필요
void evaluateScenario7(const std::vector<double>& path_x,
                        const std::vector<double>& path_y,
                        const std::vector<adcm::map_2dListVector>& map_2d,
                        adcm::risk_assessment_Objects& riskAssessment);
// 시나리오 8: 경로와 맵 정보 필요
void evaluateScenario8(const std::vector<double>& path_x,
                        const std::vector<double>& path_y,
                        const std::vector<adcm::map_2dListVector>& map_2d,
                        adcm::risk_assessment_Objects& riskAssessment);

// ==== confidence 값을 clamp 해주는 clampValue 함수 선언 ====
template <typename T>
T clampValue(const T& value, const T& low, const T& high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}
// ==== utils 함수 선언 ====

bool extractNewObstacles(const obstacleListVector& vec_old,
                         const obstacleListVector& vec_new,
                         obstacleListVector& vec_output);
void GPStoUTM(double lat, double lon, double &utmX, double &utmY);
bool isRouteValid(routeVector& route);
void checkRange(Point2D &point);
void gpsToMapcoordinate(const routeVector& route, 
                        std::vector<double>& path_x, 
                        std::vector<double>& path_y);
double calculateDistance(const adcm::obstacleListStruct& obstacle1, const adcm::obstacleListStruct& obstacle2);
double calculateDistance(const adcm::obstacleListStruct& obstacle, const adcm::vehicleListStruct& vehicle);
double getMagnitude(Point2D point);
bool getTTC(const adcm::obstacleListStruct& obstacle, const adcm::vehicleListStruct& vehicle, double& ttc);
bool calculateMinDistanceToPath(const adcm::obstacleListStruct& obstacle,
                                const std::vector<double>& path_x, 
                                const std::vector<double>& path_y,
                                double& out_distance);
//bool calculateDistanceToPath(const adcm::obstacleListStruct& obstacle, const std::vector<double>& path_x, const std::vector<double>& path_y, double& out_distance);
bool calculateMinDistanceLinear(const adcm::obstacleListStruct& obstacle, const adcm::vehicleListStruct& vehicle, double& min_distance);
void symmDiff(const obstacleListVector& vec1, const obstacleListVector& vec2, obstacleListVector &output, int n, int m);
void detectUnscannedPath(const std::vector<adcm::map_2dListVector>& map_2d,
                         const std::vector<double>& path_x,
                         const std::vector<double>& path_y,
                         adcm::risk_assessment_Objects& riskAssessment);
void calculateShiftedLines(int &x_start, int &x_end, int &y_start, int &y_end, int shift,
                           double &original_m, double &original_c, double &up_c, double &down_c,
                           bool &isVertical, double &x_up, double &x_down);

void printObstacleList(obstacleListVector obstacle_list);
// 문자열 변환 함수 선언
const char* to_string(ObstacleClass cls);
// ==== 위험판단 시나리오 선언

void evaluateScenario1(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       adcm::risk_assessment_Objects& riskAssessment);


// ==== 템플릿 함수 ====
template <typename T>
void clear_and_free_memory(std::vector<T>& vec) {
    std::vector<T>().swap(vec);
}

// ==== 익명 네임스페이스 (파일 스코프) ====
namespace {
    std::atomic_bool continueExecution{true};
    std::atomic_uint gReceivedEvent_count_map_data{0};
    std::atomic_uint gReceivedEvent_count_build_path{0};
    std::atomic_uint gReceivedEvent_count_work_information{0};
    std::atomic_uint gMainthread_Loopcount{0};

    void SigTermHandler(int signal) {
        if (signal == SIGTERM) {
            continueExecution = false;
        }
    }

    bool RegisterSigTermHandler() {
        struct sigaction sa{};
        sa.sa_handler = SigTermHandler;
        sa.sa_flags = 0;
        sigemptyset(&sa.sa_mask);

        return sigaction(SIGTERM, &sa, nullptr) != -1;
    }
}

#endif // MAIN_RISKASSESSMENT_HPP
