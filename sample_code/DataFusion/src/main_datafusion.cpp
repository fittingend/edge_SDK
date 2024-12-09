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

#include "main_datafusion.hpp"

void GPStoUTM(double lat, double lon, double &utmX, double &utmY)
{
    // WGS84 Parameters
    const double WGS84_A = 6378137.0;    // Major semiaxis [m]
    const double WGS84_E = 0.0818191908; // First Eccentricity
    // Constants
    const double k0 = 0.9996;
    const double eSquared = WGS84_E * WGS84_E;
    const double ePrimeSquared = eSquared / (1 - eSquared);
    const double RADIANS_PER_DEGREE = M_PI / 180.0;
    int zone = 52;
    // Calculate the Central Meridian of the Zone
    double lonOrigin = (zone - 1) * 6 - 180 + 3;

    // Convert lat/lon to radians
    double latRad = lat * RADIANS_PER_DEGREE;
    double lonRad = lon * RADIANS_PER_DEGREE;
    double lonOriginRad = lonOrigin * RADIANS_PER_DEGREE;

    // Calculate UTM coordinates
    double N = WGS84_A / sqrt(1 - eSquared * sin(latRad) * sin(latRad));
    double T = tan(latRad) * tan(latRad);
    double C = ePrimeSquared * cos(latRad) * cos(latRad);
    double A = cos(latRad) * (lonRad - lonOriginRad);

    double M = WGS84_A * ((1 - eSquared / 4 - 3 * pow(eSquared, 2) / 64 - 5 * pow(eSquared, 3) / 256) * latRad - (3 * eSquared / 8 + 3 * pow(eSquared, 2) / 32 + 45 * pow(eSquared, 3) / 1024) * sin(2 * latRad) + (15 * pow(eSquared, 2) / 256 + 45 * pow(eSquared, 3) / 1024) * sin(4 * latRad) - (35 * pow(eSquared, 3) / 3072) * sin(6 * latRad));
    utmX = (k0 * N * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ePrimeSquared) * A * A * A * A * A / 120) + 500000.0);
    utmY = (k0 * (M + N * tan(latRad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * ePrimeSquared) * A * A * A * A * A * A / 720)));

    if (lat < 0)
    {
        utmY += 10000000.0; // 10000000 meter offset for southern hemisphere
    }
}

bool checkRange(VehicleData vehicle)
{
    bool range_OK = false;
    if (vehicle.timestamp == 0)
    {
        return range_OK = true;
    }
    if (vehicle.position_x > 0 && vehicle.position_x < map_n && vehicle.position_y > 0 && vehicle.position_y < map_m)
    {
        if (vehicle.timestamp)
            range_OK = true;
    }

    return range_OK;
}

void checkRange(Point2D &point)
{

    if (point.x > map_n)
    {
        point.x = map_n - 1;
    }
    if (point.y > map_m)
    {
        point.y = map_m - 1;
    }
}

//-------------------------boundary 맵 대상 코드-------------------------//
/*https://mail.katech.re.kr/mail/#
bool checkRange(int x, int y)
{
    int cross = 0;
    std::vector<BoundaryData> p = work_boundary;

    for (int i = 0; i < p.size(); i++)
    {
        int j = (i + 1) % p.size();
        if ((p[i].y > y) != (p[j].y > y))
        {
            double meetX = (p[j].x - p[i].x) * (y - p[i].y) / (p[j].y - p[i].y) + p[i].x;
            if (x < meetX)
                cross++;
        }
    }
    return cross % 2 > 0;
}
*/
/*
bool checkRange(VehicleData vehicle)
{
    double x = vehicle.position_x;
    double y = vehicle.position_y;
    int cross = 0;
    std::vector<BoundaryData> p = work_boundary;

    for (int i = 0; i < p.size(); i++)
    {
        int j = (i + 1) % p.size();
        if ((p[i].y > y) != (p[j].y > y))
        {
            double meetX = (p[j].x - p[i].x) * (y - p[i].y) / (p[j].y - p[i].y) + p[i].x;
            if (x < meetX)
                cross++;
        }
    }
    return cross % 2 > 0;
}

bool checkRange(Point2D point)
{
    long x = point.x;
    long y = point.y;
    int cross = 0;
    std::vector<BoundaryData> p = work_boundary;

    for (int i = 0; i < p.size(); i++)
    {
        int j = (i + 1) % p.size();
        if ((p[i].y > y) != (p[j].y > y))
        {
            double meetX = (p[j].x - p[i].x) * (y - p[i].y) / (p[j].y - p[i].y) + p[i].x;
            if (x < meetX)
                cross++;
        }
    }
    return cross % 2 > 0;
}
*/

void gpsToMapcoordinate(VehicleData &vehicle)
{
    // wps84기반 gps(global)좌표계를 작업환경 XY 기반의 Map 좌표계로 변환
    // 시뮬레이터 map 기준 원점(0,0) global좌표
    double mapOrigin_x = 453.088714;
    double mapOrigin_y = 507.550078;
    // 시뮬레이터 기준점 utm좌표
    double origin_x = 278296.968;
    double origin_y = 3980466.846;
    double angle_radians = -MAP_ANGLE * M_PI / 180.0;
    double velocity_ang = vehicle.velocity_ang;
    double position_x = vehicle.position_long;
    double position_y = vehicle.position_lat;
    double mapVehicle_theta = (vehicle.yaw + MAP_ANGLE) * M_PI / 180.0; // 시뮬레이터 상에서 차량이 바라보는 각도
    // 차량 utm 좌표로 변환
    double distance_x, distance_y; // 차량의 utm x,y 좌표
    GPStoUTM(position_y, position_x, distance_x, distance_y);
    distance_x -= origin_x;
    distance_y -= origin_y;
    vehicle.position_x = (distance_x * cos(angle_radians) - distance_y * sin(angle_radians) - mapOrigin_x) * M_TO_10CM_PRECISION;
    vehicle.position_y = (distance_x * sin(angle_radians) + distance_y * cos(angle_radians) - mapOrigin_y) * M_TO_10CM_PRECISION;
    // 속도 (각속도 보정 임시 제외)
    double velocity_x = vehicle.velocity_long;
    double velocity_y = vehicle.velocity_lat;
    vehicle.velocity_x = velocity_x * cos(angle_radians) - velocity_y * sin(angle_radians);
    vehicle.velocity_y = velocity_x * sin(angle_radians) + velocity_y * cos(angle_radians);
    // vehicle.velocity_x = (velocity_ang * (-sin(theta) * (position_x - alpha) + (cos(theta) * (position_y - beta)))) + (velocity_x * cos(theta)) + (velocity_y * sin(theta));
    // vehicle.velocity_y = (velocity_ang * (-cos(theta) * (position_x - alpha) - (sin(theta) * (position_y - beta)))) + (velocity_x * -sin(theta)) + (velocity_y * cos(theta));
    vehicle.yaw = -(vehicle.yaw + MAP_ANGLE - 90); // 맵에 맞춰 차량 각도 회전
    // adcm::Log::Info() << "차량" << vehicle.vehicle_class << "gpsToMapcoordinate 좌표변환 before (" << position_x << " , " << position_y << " , " << velocity_x << " , " << velocity_y << ")";
    // adcm::Log::Info() << "timestamp: " << vehicle.timestamp << " 차량" << vehicle.vehicle_class << "gpsToMapcoordinate 좌표변환 after (" << vehicle.position_x << " , " << vehicle.position_y << " , " << vehicle.yaw << ")";
}

void relativeToMapcoordinate(std::vector<ObstacleData> &obstacle_list, VehicleData vehicle)
{
    srand((unsigned int)time(NULL));
    double theta = vehicle.yaw * M_PI / 180.0;
    double velocity_ang = vehicle.velocity_ang;

    for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
    {
        // adcm::Log::Info() << "장애물 relativeToGlobal 좌표변환 before (" << iter->fused_position_x << " , " << iter->fused_position_y << " , " << iter->fused_velocity_x << " , " << iter->fused_velocity_y << ")";

        double obstacle_position_x = iter->fused_position_x;
        double obstacle_position_y = iter->fused_position_y;
        double obstacle_velocity_x = iter->fused_velocity_x; // 시뮬 로그 속도의 단위는 m/s인데, 결과 값의 단위는 미정
        double obstacle_velocity_y = iter->fused_velocity_y;

        // iter->fused_position_x = (obstacle_position_x * cos(angle_radians) - obstacle_position_y * sin(angle_radians) - mapOrigin_x) * M_TO_10CM_PRECISION;
        // iter->fused_position_y = (obstacle_position_x * sin(angle_radians) + obstacle_position_y * cos(angle_radians) - mapOrigin_y) * M_TO_10CM_PRECISION;
        // adcm::Log::Info() << "장애물 relativeToMap 좌표변환 before (" << iter->fused_position_x << " , " << iter->fused_position_y << ", " << iter->fused_velocity_x << " , " << iter->fused_velocity_y << ")";
        // adcm::Log::Info() << main_vehicle.yaw << "각 회전한 값 : (" << (obstacle_position_x)*cos(theta) - (obstacle_position_y)*sin(theta) << ", " << (obstacle_position_x)*sin(theta) + (obstacle_position_y)*cos(theta) << ")";

        iter->fused_position_x = vehicle.position_x + ((obstacle_position_x)*cos(theta) + (obstacle_position_y)*sin(theta)) * M_TO_10CM_PRECISION;
        iter->fused_position_y = vehicle.position_y + ((obstacle_position_x)*sin(theta) - (obstacle_position_y)*cos(theta)) * M_TO_10CM_PRECISION;

        iter->fused_velocity_x = obstacle_velocity_x + vehicle.velocity_x;
        iter->fused_velocity_y = obstacle_velocity_x + vehicle.velocity_y;

        iter->fused_heading_angle = vehicle.yaw + iter->fused_heading_angle;
        if (iter->fused_position_x < 0)
        {
            iter->fused_position_x = rand() % 3;
        }

        if (iter->fused_position_y < 0)
        {
            iter->fused_position_y = rand() % 3;
        }

        // adcm::Log::Info() << "장애물 relativeToMap 좌표변환 after (" << iter->fused_position_x << " , " << iter->fused_position_y << " , " << iter->fused_velocity_x << " , " << iter->fused_velocity_y << ")";
    }
}

/*
void generateRoadZValue(VehicleData target_vehicle, std::vector<adcm::map_2dListVector> &map_2d_test)
{
// 현재 차량의 position_x position_y 중심으로 좌우전방 5m 를 스캔해서 road_z 값을 1로 지정
#define SCANNING_RANGE 30
    // adcm::Log::Info() << "vehicle class " << target_vehicle.vehicle_class << " generateRoadZValue";

    int scanned_range_LL_x = floor(target_vehicle.position_x - SUB_VEHICLE_SIZE_X / 2) - SCANNING_RANGE;
    int scanned_range_LL_y = floor(target_vehicle.position_y - SUB_VEHICLE_SIZE_Y / 2) - SCANNING_RANGE;

    int scanned_range_RU_x = floor(target_vehicle.position_x + SUB_VEHICLE_SIZE_X / 2) + SCANNING_RANGE;
    int scanned_range_RU_y = floor(target_vehicle.position_y + SUB_VEHICLE_SIZE_Y / 2) + SCANNING_RANGE;
    // adcm::Log::Info() << "x는 " << scanned_range_LL_x << " ~ " << scanned_range_RU_x + 1 << "까지";
    // adcm::Log::Info() << "y는 " << scanned_range_LL_y << " ~ " << scanned_range_RU_y + 1 << "까지";
    for (int i = scanned_range_LL_x; i < (scanned_range_RU_x + 1); i++)
    {
        for (int j = scanned_range_LL_y; j < (scanned_range_RU_y + 1); j++)
        {
            if (i >= 0 && j >= 0 && i < map_n && j < map_m)
            {
                map_2d_test[i][j].road_z = 1;
                // adcm::Log::Info() << "main vehicle generateRoadZValue[" << i << "]["<< j << "]:" << map_2d_test[i][j].road_z;
            }
        }
    }
    // adcm::Log::Info() << "generateRoadZValue finish";
}
*/


void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, VehicleData &vehicle)
{
    // Collect all points in an array
    Point2D points[] = {p0, p1, p2, p3};

    // Find min and max values for x and y
    double min_x = points[0].x, max_x = points[0].x;
    double min_y = points[0].y, max_y = points[0].y;
    for (const auto& p : points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }

    Point2D index;
    //Ray-Casting algorithm 
    for (index.x = min_x; index.x <= max_x; ++index.x) {
        for (index.y = min_y; index.y <= max_y; ++index.y) {
            int cross = 0;
            for (int i = 0; i < 4; i++) {
                int j = (i + 1) % 4;
                if ((points[i].y > index.y) != (points[j].y > index.y)) {
                    double meetX = (points[j].x - points[i].x) * (index.y - points[i].y) / 
                                   (points[j].y - points[i].y) + points[i].x;
                    if (index.x < meetX) cross++;
                }
            }
            if (cross % 2 != 0) {
                vehicle.map_2d_location.push_back(index);
                //map_2d_test[index.x][index.y].vehicle_class = vehicle.vehicle_class;
                // TO DO: 현재는 물체가 있는 index 는 road_z 값 1로 설정 (아무것도 없으면 0)
                //map_2d_test[index.x][index.y].road_z = 1;
            }
        }
    }
}

void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, std::vector<ObstacleData>::iterator iter)
{
    if (iter == std::vector<ObstacleData>::iterator()) return; // Ensure valid iterator

    // Collect all points in an array
    Point2D points[] = {p0, p1, p2, p3};

    // Find min and max values for x and y
    double min_x = points[0].x, max_x = points[0].x;
    double min_y = points[0].y, max_y = points[0].y;
    for (const auto& p : points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }

    Point2D index;

    for (index.x = min_x; index.x <= max_x; ++index.x) {
        for (index.y = min_y; index.y <= max_y; ++index.y) {
            int cross = 0;
            for (int i = 0; i < 4; i++) {
                int j = (i + 1) % 4;
                if ((points[i].y > index.y) != (points[j].y > index.y)) {
                    double meetX = (points[j].x - points[i].x) * (index.y - points[i].y) / 
                                   (points[j].y - points[i].y) + points[i].x;
                    if (index.x < meetX) cross++;
                }
            }
            if (cross % 2 != 0) {
                iter->map_2d_location.push_back(index);
                //map_2d_test[index.x][index.y].obstacle_id = iter->obstacle_id;
                // TO DO: 현재는 물체가 있는 index 는 road_z 값 1로 설정 (아무것도 없으면 0)
                //map_2d_test[index.x][index.y].road_z = 1;
            }
        }
    }
}

/*
void generateOccupancyIndex_ori(Point2D p0, Point2D p1, Point2D p2, Point2D p3, std::vector<ObstacleData>::iterator iter)
{
    long arr_x[] = {p0.x, p1.x, p2.x, p3.x};
    long arr_y[] = {p0.y, p1.y, p2.y, p3.y};
    // find max x&y and min x&y of the rectangle
    int n = sizeof(arr_y) / sizeof(arr_y[0]);
    // Implemented inbuilt function to sort array
    std::sort(arr_x, arr_x + n);
    std::sort(arr_y, arr_y + n);
    long min_x = arr_x[0];
    long max_x = arr_x[n - 1];
    long min_y = arr_y[0];
    long max_y = arr_y[n - 1];

    Point2D p[] = {p0, p1, p2, p3};
    Point2D index;

    for (index.x = min_x; index.x < max_x; index.x++)
    {
        for (index.y = min_y; index.y < max_y; index.y++)
        {
            int cross = 0;
            for (int i = 0; i < 4; i++)
            {
                int j = (i + 1) % 4;
                if ((p[i].y > index.y) != (p[j].y > index.y))
                {
                    double meetX = (p[j].x - p[i].x) * (index.y - p[i].y) / (p[j].y - p[i].y) + p[i].x;
                    if (index.x < meetX)
                        cross++;
                }
            }
            if (cross % 2 > 0)
            {
                iter->map_2d_location.push_back(index);
            }
        }
    }
}
*/
void find4VerticesVehicle(VehicleData &target_vehicle)
{
    Point2D LU, RU, RL, LL;
    double half_x;
    double half_y;
    double theta = target_vehicle.yaw * M_PI / 180;

    if (target_vehicle.vehicle_class == EGO_VEHICLE)
    {
        half_x = MAIN_VEHICLE_SIZE_X / 2;
        half_y = MAIN_VEHICLE_SIZE_Y / 2;
    }
    
    else
    {
        half_x = SUB_VEHICLE_SIZE_X / 2;
        half_y = SUB_VEHICLE_SIZE_Y / 2;
    }
    // Top-left (LU)
    LU.x = target_vehicle.position_x + cos(theta) * (-half_x) - sin(theta) * (half_y);
    LU.y = target_vehicle.position_y + sin(theta) * (-half_x) + cos(theta) * (half_y);

    // Top-right (RU)
    RU.x = target_vehicle.position_x + cos(theta) * (half_x) - sin(theta) * (half_y);
    RU.y = target_vehicle.position_y + sin(theta) * (half_x) + cos(theta) * (half_y);

    // Bottom-right (RL)
    RL.x = target_vehicle.position_x + cos(theta) * (half_x) - sin(theta) * (-half_y);
    RL.y = target_vehicle.position_y + sin(theta) * (half_x) + cos(theta) * (-half_y);

    // Bottom-left (LL)
    LL.x = target_vehicle.position_x + cos(theta) * (-half_x) - sin(theta) * (-half_y);
    LL.y = target_vehicle.position_y + sin(theta) * (-half_x) + cos(theta) * (-half_y);

    checkRange(LU);
    checkRange(RU);
    checkRange(RL);
    checkRange(LL);

    //generateRoadZValue(target_vehicle, map_2d_test);
    //TO DO
    generateOccupancyIndex(LU, RU, RL, LL, target_vehicle);

}
/*
void find4VerticesVehicle(VehicleData &target_vehicle, std::vector<adcm::map_2dListVector> &map_2d_test)
{
    Point2D LU, RU, RL, LL;
    double theta = target_vehicle.yaw * M_PI / 180;

    if (target_vehicle.vehicle_class == EGO_VEHICLE)
    {
// 특장차일 경우
#define VEHICLE_SIZE_X MAIN_VEHICLE_SIZE_X
#define VEHICLE_SIZE_Y MAIN_VEHICLE_SIZE_Y
        // #define VEHICLE_SIZE_X main_vehicle_size_length
        // #define VEHICLE_SIZE_Y main_vehicle_size_width
    }
    else
    {

#define VEHICLE_SIZE_X SUB_VEHICLE_SIZE_X
#define VEHICLE_SIZE_Y SUB_VEHICLE_SIZE_Y
        // #define VEHICLE_SIZE_X sub_vehicle_size[target_vehicle.vehicle_class].length
        // #define VEHICLE_SIZE_Y sub_vehicle_size[target_vehicle.vehicle_class].width
    }
    // step1. 4 꼭지점을 각각 찾는다
    LU.x = target_vehicle.position_x + (cos(theta) * VEHICLE_SIZE_X / 2 - sin(theta) * VEHICLE_SIZE_Y / 2);
    LU.y = target_vehicle.position_y + (sin(theta) * VEHICLE_SIZE_X / 2 + cos(theta) * VEHICLE_SIZE_Y / 2);

    RU.x = target_vehicle.position_x + (cos(theta) * VEHICLE_SIZE_X / 2 - sin(theta) * VEHICLE_SIZE_Y / 2);
    RU.y = target_vehicle.position_y - (sin(theta) * VEHICLE_SIZE_X / 2 + cos(theta) * VEHICLE_SIZE_Y / 2);

    RL.x = target_vehicle.position_x - (cos(theta) * VEHICLE_SIZE_X / 2 - sin(theta) * VEHICLE_SIZE_Y / 2);
    RL.y = target_vehicle.position_y - (sin(theta) * VEHICLE_SIZE_X / 2 + cos(theta) * VEHICLE_SIZE_Y / 2);

    LL.x = target_vehicle.position_x - (cos(theta) * VEHICLE_SIZE_X / 2 - sin(theta) * VEHICLE_SIZE_Y / 2);
    LL.y = target_vehicle.position_y + (sin(theta) * VEHICLE_SIZE_X / 2 + cos(theta) * VEHICLE_SIZE_Y / 2);

    // adcm::Log::Info() << "find4VerticesVehicle: LU.x is" << LU.x;
    // adcm::Log::Info() << "find4VerticesVehicle: LU.y is" << LU.y;

    // adcm::Log::Info() << "find4VerticesVehicle: RU.x is" << RU.x;
    // adcm::Log::Info() << "find4VerticesVehicle: RU.y is" << RU.y;

    // adcm::Log::Info() << "find4VerticesVehicle: RL.x is" << RL.x;
    // adcm::Log::Info() << "find4VerticesVehicle: RL.y is" << RL.y;

    // adcm::Log::Info() << "find4VerticesVehicle: LL.x is" << LL.x;
    // adcm::Log::Info() << "find4VerticesVehicle: LL.y is" << LL.y;
    // 사각형 map안에 들어와있는지 확인 후 밖으로 나와있으면 범위 안으로 조정
    checkRange(LU);
    checkRange(RU);
    checkRange(RL);
    checkRange(LL);
    // 다각형 map안에 들어와있는지 확인 후 알림

    generateRoadZValue(target_vehicle, map_2d_test);
    generateOccupancyIndex(LU, RU, RL, LL, target_vehicle, map_2d_test);
}
*/
void find4VerticesObstacle(std::vector<ObstacleData> &obstacle_list_filtered)
{
    for (auto iter = obstacle_list_filtered.begin(); iter < obstacle_list_filtered.end(); iter++)
    {
        Point2D LU, RU, RL, LL;
        double half_x = iter->fused_cuboid_x / 2;
        double half_y = iter->fused_cuboid_y / 2;
        double obstacle_position_x = iter->fused_position_x;
        double obstacle_position_y = iter->fused_position_y;
        double theta = iter->fused_heading_angle * M_PI / 180;

        // Top-left (LU)
        LU.x = obstacle_position_x + cos(theta) * (-half_x) - sin(theta) * (half_y);
        LU.y = obstacle_position_y + sin(theta) * (-half_x) + cos(theta) * (half_y);

        // Top-right (RU)
        RU.x = obstacle_position_x + cos(theta) * (half_x) - sin(theta) * (half_y);
        RU.y = obstacle_position_y + sin(theta) * (half_x) + cos(theta) * (half_y);

        // Bottom-right (RL)
        RL.x = obstacle_position_x + cos(theta) * (half_x) - sin(theta) * (-half_y);
        RL.y = obstacle_position_y + sin(theta) * (half_x) + cos(theta) * (-half_y);

        // Bottom-left (LL)
        LL.x = obstacle_position_x + cos(theta) * (-half_x) - sin(theta) * (-half_y);
        LL.y = obstacle_position_y + sin(theta) * (-half_x) + cos(theta) * (-half_y);

        checkRange(LU);
        checkRange(RU);
        checkRange(RL);
        checkRange(LL);

        generateOccupancyIndex(LU, RU, RL, LL, *(&iter));
    }
}

/*
void find4VerticesObstacle_ori(std::vector<ObstacleData> &obstacle_list_filtered)
{
    // 4 vertices 를 찾고싶은 해당 obstacle
    for (auto iter = obstacle_list_filtered.begin(); iter < obstacle_list_filtered.end(); iter++)
    {
        Point2D LU, RU, RL, LL;
        double obstacle_size_x = iter->fused_cuboid_x;
        double obstacle_size_y = iter->fused_cuboid_y;
        double obstacle_position_x = iter->fused_position_x;
        double obstacle_position_y = iter->fused_position_y;
        double theta = iter->fused_heading_angle * M_PI / 180;

        LU.x = obstacle_position_x + (cos(theta) * (obstacle_size_x / 2) - sin(theta) * (obstacle_size_y / 2));
        LU.y = obstacle_position_y + (sin(theta) * (obstacle_size_x / 2) + cos(theta) * (obstacle_size_y / 2));

        RU.x = obstacle_position_x + (cos(theta) * (obstacle_size_x / 2) - sin(theta) * (obstacle_size_y / 2));
        RU.y = obstacle_position_y - (sin(theta) * (obstacle_size_x / 2) + cos(theta) * (obstacle_size_y / 2));

        RL.x = obstacle_position_x - (cos(theta) * (obstacle_size_x / 2) - sin(theta) * (obstacle_size_y / 2));
        RL.y = obstacle_position_y - (sin(theta) * (obstacle_size_x / 2) + cos(theta) * (obstacle_size_y / 2));

        LL.x = obstacle_position_x - (cos(theta) * (obstacle_size_x / 2) - sin(theta) * (obstacle_size_y / 2));
        LL.y = obstacle_position_y + (sin(theta) * (obstacle_size_x / 2) + cos(theta) * (obstacle_size_y / 2));

        // adcm::Log::Info() << "find4VerticesObstacle: LU.x is" << LU.x;
        // adcm::Log::Info() << "find4VerticesObstacle: LU.y is" << LU.y;

        // adcm::Log::Info() << "find4VerticesObstacle: RU.x is" << RU.x;
        // adcm::Log::Info() << "find4VerticesObstacle: RU.y is" << RU.y;

        // adcm::Log::Info() << "find4VerticesObstacle: RL.x is" << RL.x;
        // adcm::Log::Info() << "find4VerticesObstacle: RL.y is" << RL.y;

        // adcm::Log::Info() << "find4VerticesObstacle: LL.x is" << LL.x;
        // adcm::Log::Info() << "find4VerticesObstacle: LL.y is" << LL.y;

        checkRange(LU);
        checkRange(RU);
        checkRange(RL);
        checkRange(LL);

        generateOccupancyIndex(LU, RU, RL, LL, *(&iter));
    }
    adcm::Log::Info() << "장애물 꼭짓점 범위 확인완료";
}
*/
// hubData 수신
void ThreadReceiveHubData()
{
    adcm::Log::Info() << "SDK_release_240910_interface v1.8.4";
    adcm::Log::Info() << "DataFusion_release_241010";
    // adcm::MapData_Provider mapData_provider;
    adcm::HubData_Subscriber hubData_subscriber;
    // INFO("DataFusion .init()");
    // mapData_provider.init("DataFusion/DataFusion/PPort_map_data");
    hubData_subscriber.init("DataFusion/DataFusion/RPort_hub_data");
    INFO("ThreadReceiveHubData start...");

    while (continueExecution)
    {
        gMainthread_Loopcount++;
        VERBOSE("[DataFusion] Application loop");
        bool hubData_rxEvent = hubData_subscriber.waitEvent(100); // wait event
        // adcm::Log::Info() << "wait hubData" << gMainthread_Loopcount;

        if (hubData_rxEvent)
        {
            // adcm::Log::Verbose() << "[EVENT] DataFusion Hub Data received";

            while (!hubData_subscriber.isEventQueueEmpty())
            {
                auto data = hubData_subscriber.getEvent();
                gReceivedEvent_count_hub_data++;
                FusionData fusionData;
                // 수신된 데이터 handling 위한 추가 코드
                switch (data->vehicle_class)
                {
                case EGO_VEHICLE: // 특장차가 보낸 인지데이터

                    main_vehicle_temp.vehicle_class = EGO_VEHICLE;
                    main_vehicle_temp.timestamp = data->timestamp;
                    // main_vehicle_temp.road_z = data->road_z; //vector assignment to fix?
                    main_vehicle_temp.position_lat = data->position_lat;
                    main_vehicle_temp.position_long = data->position_long;
                    main_vehicle_temp.position_height = data->position_height;
                    main_vehicle_temp.yaw = data->yaw;
                    main_vehicle_temp.roll = data->roll;
                    main_vehicle_temp.pitch = data->pitch;
                    main_vehicle_temp.velocity_long = data->velocity_long;
                    main_vehicle_temp.velocity_lat = data->velocity_lat;
                    main_vehicle_temp.velocity_ang = data->velocity_ang;

                    obstacle_list_temp.clear();

                    for (int i = 0; i < data->obstacle.size(); i++)
                    {
                        ObstacleData obstacle_to_push;
                        obstacle_to_push.obstacle_class = data->obstacle[i].obstacle_class;
                        obstacle_to_push.timestamp = data->timestamp;
                        obstacle_to_push.fused_cuboid_x = data->obstacle[i].cuboid_x;
                        obstacle_to_push.fused_cuboid_y = data->obstacle[i].cuboid_y;
                        obstacle_to_push.fused_cuboid_z = data->obstacle[i].cuboid_z;
                        obstacle_to_push.fused_heading_angle = data->obstacle[i].heading_angle;
                        obstacle_to_push.fused_position_x = data->obstacle[i].position_x;
                        obstacle_to_push.fused_position_y = data->obstacle[i].position_y;
                        obstacle_to_push.fused_position_z = data->obstacle[i].position_z;
                        obstacle_to_push.fused_velocity_x = data->obstacle[i].velocity_x;
                        obstacle_to_push.fused_velocity_y = data->obstacle[i].velocity_y;
                        obstacle_to_push.fused_velocity_z = data->obstacle[i].velocity_z;
                        obstacle_list_temp.push_back(obstacle_to_push);
                        // adcm::Log::Info() << "메인 차량 기준 장애물 위치 : (" << obstacle_to_push.fused_position_x << ", " << obstacle_to_push.fused_position_y << ")";
                    }
                    
                    fusionData.vehicle = main_vehicle_temp;
                    fusionData.obstacle_list = obstacle_list_temp;
                    main_vehicle_queue.enqueue(fusionData);
                    break;

                case SUB_VEHICLE_1: // 보조차1이 보낸 인지데이터
                    sub1_vehicle_temp.vehicle_class = SUB_VEHICLE_1;
                    sub1_vehicle_temp.timestamp = data->timestamp;
                    // sub1_vehicle_temp.road_z = data->road_z; //vector assignment to fix?
                    sub1_vehicle_temp.position_lat = data->position_lat;
                    sub1_vehicle_temp.position_long = data->position_long;
                    sub1_vehicle_temp.position_height = data->position_height;
                    sub1_vehicle_temp.yaw = data->yaw;
                    sub1_vehicle_temp.roll = data->roll;
                    sub1_vehicle_temp.pitch = data->pitch;
                    sub1_vehicle_temp.velocity_long = data->velocity_long;
                    sub1_vehicle_temp.velocity_lat = data->velocity_lat;
                    sub1_vehicle_temp.velocity_ang = data->velocity_ang;

                    obstacle_list_temp.clear();

                    for (int i = 0; i < data->obstacle.size(); i++)
                    {
                        ObstacleData obstacle_to_push;
                        obstacle_to_push.obstacle_class = data->obstacle[i].obstacle_class;
                        obstacle_to_push.timestamp = data->timestamp;
                        obstacle_to_push.fused_cuboid_x = data->obstacle[i].cuboid_x;
                        obstacle_to_push.fused_cuboid_y = data->obstacle[i].cuboid_y;
                        obstacle_to_push.fused_cuboid_z = data->obstacle[i].cuboid_z;
                        obstacle_to_push.fused_heading_angle = data->obstacle[i].heading_angle;
                        obstacle_to_push.fused_position_x = data->obstacle[i].position_x;
                        obstacle_to_push.fused_position_y = data->obstacle[i].position_y;
                        obstacle_to_push.fused_position_z = data->obstacle[i].position_z;
                        obstacle_to_push.fused_velocity_x = data->obstacle[i].velocity_x;
                        obstacle_to_push.fused_velocity_y = data->obstacle[i].velocity_y;
                        obstacle_to_push.fused_velocity_z = data->obstacle[i].velocity_z;
                        obstacle_list_temp.push_back(obstacle_to_push);
                        // adcm::Log::Info() << "보조 차량1 기준 장애물 위치 : (" << obstacle_to_push.fused_position_x << ", " << obstacle_to_push.fused_position_y << ")";
                    }
                    
                    fusionData.vehicle = sub1_vehicle_temp;
                    fusionData.obstacle_list = obstacle_list_temp;
                    sub1_vehicle_queue.enqueue(fusionData);
                    break;

                case SUB_VEHICLE_2: // 보조차2가 보낸 인지데이터
                    sub2_vehicle_temp.vehicle_class = SUB_VEHICLE_2;
                    sub2_vehicle_temp.timestamp = data->timestamp;
                    // sub2_vehicle_temp.road_z = data->road_z; //vector assignment to fix?
                    sub2_vehicle_temp.position_lat = data->position_lat;
                    sub2_vehicle_temp.position_long = data->position_long;
                    sub2_vehicle_temp.position_height = data->position_height;
                    sub2_vehicle_temp.yaw = data->yaw;
                    sub2_vehicle_temp.roll = data->roll;
                    sub2_vehicle_temp.pitch = data->pitch;
                    sub2_vehicle_temp.velocity_long = data->velocity_long;
                    sub2_vehicle_temp.velocity_lat = data->velocity_lat;
                    sub2_vehicle_temp.velocity_ang = data->velocity_ang;

                    obstacle_list_temp.clear();

                    for (int i = 0; i < data->obstacle.size(); i++)
                    {
                        ObstacleData obstacle_to_push;
                        obstacle_to_push.obstacle_class = data->obstacle[i].obstacle_class;
                        obstacle_to_push.timestamp = data->timestamp;
                        obstacle_to_push.fused_cuboid_x = data->obstacle[i].cuboid_x;
                        obstacle_to_push.fused_cuboid_y = data->obstacle[i].cuboid_y;
                        obstacle_to_push.fused_cuboid_z = data->obstacle[i].cuboid_z;
                        obstacle_to_push.fused_heading_angle = data->obstacle[i].heading_angle;
                        obstacle_to_push.fused_position_x = data->obstacle[i].position_x;
                        obstacle_to_push.fused_position_y = data->obstacle[i].position_y;
                        obstacle_to_push.fused_position_z = data->obstacle[i].position_z;
                        obstacle_to_push.fused_velocity_x = data->obstacle[i].velocity_x;
                        obstacle_to_push.fused_velocity_y = data->obstacle[i].velocity_y;
                        obstacle_to_push.fused_velocity_z = data->obstacle[i].velocity_z;
                        obstacle_list_temp.push_back(obstacle_to_push);
                        // adcm::Log::Info() << "보조 차량2 기준 장애물 위치 : (" << obstacle_to_push.fused_position_x << ", " << obstacle_to_push.fused_position_y << ")";
                    }
                    
                    fusionData.vehicle = sub2_vehicle_temp;
                    fusionData.obstacle_list = obstacle_list_temp;
                    sub2_vehicle_queue.enqueue(fusionData);
                    break;
                default:
                    adcm::Log::Info() << "data received but belongs to no vehicle hence discarded";
                    break;
                }
                // adcm::Log::Info() << "=============KATECH: handling of received data DONE==============";
            }
        }
        if (continueExecution != true)
        {
            adcm::Log::Info() << "continueExection : " << continueExecution << " > 종료";
        }
    }
}
// work_information 수신
void ThreadReceiveWorkInfo()
{
    adcm::Log::Info() << "DataFusion ThreadReceiveWorkInfo";
    adcm::WorkInformation_Subscriber workInformation_subscriber;
    workInformation_subscriber.init("DataFusion/DataFusion/RPort_work_information");
    INFO("ThreadReceiveWorkInfo start...");

    while (continueExecution)
    {
        bool workInfo_rxEvent = workInformation_subscriber.waitEvent(10000);

        if (workInfo_rxEvent)
        {

            adcm::Log::Verbose() << "[EVENT] DataFusion Work Information received";
            adcm::Log::Info() << "DataFusion Work Information received";
            while (!workInformation_subscriber.isEventQueueEmpty())
            {
                auto data = workInformation_subscriber.getEvent();

                main_vehicle_size.length = data->main_vehicle.length;
                main_vehicle_size.width = data->main_vehicle.width;
                adcm::Log::Info() << "main vehicle size : (" << main_vehicle_size.length << ", " << main_vehicle_size.width << ")";
                for (int i = 0; i < data->sub_vehicle.size(); i++)
                {
                    VehicleSizeData vehicle_to_push;
                    vehicle_to_push.length = data->sub_vehicle[i].length;
                    vehicle_to_push.width = data->sub_vehicle[i].width;
                    sub_vehicle_size.push_back(vehicle_to_push);
                    adcm::Log::Info() << "sub vehicle " << i << " size : (" << vehicle_to_push.length << ", " << vehicle_to_push.width << ")";
                }
                for (int i = 0; i < data->working_area_boundary.size(); i++)
                {
                    BoundaryData boundary_to_push;
                    boundary_to_push.x = data->working_area_boundary[i].x;
                    boundary_to_push.y = data->working_area_boundary[i].y;
                    work_boundary.push_back(boundary_to_push);
                    adcm::Log::Info() << "boundary(" << i << ") : (" << work_boundary[i].x << ", " << work_boundary[i].y << ")";
                }
            }
            // 좌표계 변환 전 map 범위 둘러싸는 사각형 좌표

            min_a = work_boundary[0].x;
            min_b = work_boundary[0].y;

            max_a = work_boundary[0].x;
            max_b = work_boundary[0].y;

            for (int i = 1; i < work_boundary.size(); i++)
            {
                min_a = work_boundary[i].x < min_a ? work_boundary[i].x : min_a;
                min_b = work_boundary[i].y < min_b ? work_boundary[i].y : min_b;
                max_a = work_boundary[i].x > max_a ? work_boundary[i].x : max_a;
                max_b = work_boundary[i].y > max_b ? work_boundary[i].y : max_b;
            }
            adcm::Log::Info() << "map의 min 값: (" << min_a << ", " << min_b << "), max 값 : (" << max_a << ", " << max_b << ")";

            for (int i = 0; i < work_boundary.size(); i++)
            {
                work_boundary[i].x -= min_a;
                work_boundary[i].y -= min_b;
                adcm::Log::Info() << "변경한 boundary(" << i << ") : (" << work_boundary[i].x << ", " << work_boundary[i].y << ")";
            }
            sub_vehicle_size.clear();
            work_boundary.clear();
        }
    }
}

void ThreadKatech()
{
    //==============1.전역변수인 MapData 생성 =================
    adcm::map_data_Objects mapData;
    // 한번 생성후 관제에서 인지데이터를 받을때마다 (100ms) 마다 업데이트
    adcm::Log::Info() << "mapData created for the first time";
    ::adcm::map_2dListStruct map_2dStruct_init;
    map_2dStruct_init.obstacle_id = NO_OBSTACLE;
    map_2dStruct_init.road_z = 0;
    map_2dStruct_init.vehicle_class = NO_VEHICLE; // 시뮬레이션 데이터 설정때문에 부득이 NO_VEHICLE =5 로 바꿈
    bool sendEmptyMap = true;                     // 최초 실행 시 빈 맵 전송을 위한 변수
    int mapVer = 0; // 현재 맵이 몇 번째 맵인지 확인
    // 빈 맵 생성
    std::vector<adcm::map_2dListVector> map_2d_test(map_n, adcm::map_2dListVector(map_m, map_2dStruct_init));
    adcm::Log::Info() << "mapData 2d info initialized";

    adcm::MapData_Provider mapData_provider;
    mapData_provider.init("DataFusion/DataFusion/PPort_map_data");
    if (sendEmptyMap)
    {
        mapData.map_2d = map_2d_test;
        mapData_provider.send(mapData);
        adcm::Log::Info() << "Send empty map data";
        sendEmptyMap = false;
    }

    VehicleData main_vehicle;
    VehicleData sub1_vehicle;
    VehicleData sub2_vehicle;
    FusionData main_vehicle_data;
    FusionData sub1_vehicle_data;
    FusionData sub2_vehicle_data;
    std::vector<ObstacleData> obstacle_list;

    while (continueExecution)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        // 수신한 허브 데이터가 없으면 송신 X
        if(main_vehicle_queue.size_approx() == 0 && sub1_vehicle_queue.size_approx() == 0 && sub2_vehicle_queue.size_approx() == 0)
        {
            adcm::Log::Info() << "No Hub Data, sub2_vehicle_queue size(): " << sub2_vehicle_queue.size_approx();
            continue;
        }
        adcm::Log::Info() << "송신이 필요한 남은 허브 데이터 개수: " << sub2_vehicle_queue.size_approx();
        // else
        // {
        //     adcm::Log::Info() << "[업데이트 예정 허브 데이터] 메인: " << main_vehicle_temp.timestamp << ", 서브1: " << sub1_vehicle_temp.timestamp
        //                       << ", 서브2: " << sub2_vehicle_temp.timestamp;
        //     adcm::Log::Info() << "[현재 맵 데이터] " << mapUpdate;
        // }
        std::int8_t map_x = max_a - min_a;
        std::int8_t map_y = max_b - min_b;

        adcm::Log::Info() << "==============KATECH modified code start==========";

        // (임시) 데이터를 queue에서 꺼내면서 vehicle, obstacle_list 최신화
        if(main_vehicle_queue.try_dequeue(main_vehicle_data))
        {
            main_vehicle = main_vehicle_data.vehicle;
            obstacle_list = main_vehicle_data.obstacle_list;
        }
        if(sub1_vehicle_queue.try_dequeue(sub1_vehicle_data))
        {
            sub1_vehicle = sub1_vehicle_data.vehicle;
            obstacle_list = sub1_vehicle_data.obstacle_list;
        }
        if(sub2_vehicle_queue.try_dequeue(sub2_vehicle_data))
        {
            sub2_vehicle = sub2_vehicle_data.vehicle;
            obstacle_list = sub2_vehicle_data.obstacle_list;
        }
        // main_vehicle = main_vehicle_temp;
        // sub1_vehicle = sub1_vehicle_temp;
        // sub2_vehicle = sub2_vehicle_temp;
        // obstacle_list = obstacle_list_temp;

        adcm::map_2dListVector map_2dListVector;
        adcm::map_2dListStruct map_2dStruct;


        adcm::Log::Info() << "mapData obstacle list size is at start is" << mapData.obstacle_list.size();

        //==============1. obstacle 로 인지된 특장차 및 보조 차량 제거 =================

        adcm::Log::Info() << "특장차 및 보조차량 제거 전 obstacle 사이즈: " << obstacle_list.size();
        for (auto iter = obstacle_list.begin(); iter != obstacle_list.end();)
        {
            //if ((abs(iter->fused_cuboid_x - SUB_VEHICLE_SIZE_X / 10)) < 0.1 && (abs(iter->fused_cuboid_y - SUB_VEHICLE_SIZE_Y / 10)) < 0.1)
            if (iter->obstacle_class == 51)
            {
                adcm::Log::Info() << "보조차량 제거";
                iter = obstacle_list.erase(iter);
            }
            //if ((abs(iter->fused_cuboid_x - MAIN_VEHICLE_SIZE_X / 10)) < 0.1 && (abs(iter->fused_cuboid_y - MAIN_VEHICLE_SIZE_Y / 10)) < 0.1)
            else if(iter->obstacle_class == 50)
            {
                adcm::Log::Info() << "특장차 제거";
                iter = obstacle_list.erase(iter);
            }
            else
                iter++;
        }
        adcm::Log::Info() << "특장차 및 보조차량 제거 후 obstacle 사이즈: " << obstacle_list.size();

        //==============2. 차량 좌표계 변환==================================
        // 차량마다 받은 gps 좌표를 작업공간 map 좌표계로 변환
        gpsToMapcoordinate(main_vehicle);
        gpsToMapcoordinate(sub1_vehicle);
        gpsToMapcoordinate(sub2_vehicle);
        adcm::Log::Info() << "차량 측위 좌표계 변환 완료";


        //==============3. 장애물 좌표계 변환=================
        // 차량 기준 장애물 좌표를 작업공간 map 좌표계로 변환
        //relativeToMapcoordinate(obstacle_list_filtered, sub2_vehicle);
        relativeToMapcoordinate(obstacle_list, sub2_vehicle);
        bool isMainVehicleInRange = checkRange(main_vehicle);
        bool isSubVehicle1InRange = checkRange(sub1_vehicle);
        bool isSubVehicle2InRange = checkRange(sub2_vehicle);
        adcm::Log::Info() << "차량별 장애물 좌표계 변환 완료";

        //TO DO: 4. 차량별 장애물 fusion 여기서 필요


        //==============5. 장애물 ID 관리 =================

        for (auto iter1 = mapData.obstacle_list.begin(); iter1 != mapData.obstacle_list.end(); iter1++)
        {
            // adcm::Log::Info() << "previous obstacle saved in the mapData!" << iter1->obstacle_id;
        }
        std::vector<ObstacleData> obstacle_list_filtered;
        obstacle_list_filtered.clear();
        IDManager id_manager;

        if (mapData.obstacle_list.empty())
        {
            // 최초 obstacle ID assignment 진행
            // 트래킹하는 알고리즘 필요 *추후 보완
            int i = 1;
            for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
            {
                iter->obstacle_id = id_manager.allocID();
                obstacle_list_filtered.push_back(*iter);
            }
        }
        else // 이미 obstacle list 존재하는 경우는 obstacle id 비교가 필요하다
        {
            std::vector<int> removedObstacle;
            for (auto ori_iter = mapData.obstacle_list.begin(); ori_iter != mapData.obstacle_list.end(); ori_iter++)
            {
                bool identicalObstacleFound = false;
                for (auto new_iter = obstacle_list.begin(); new_iter != obstacle_list.end();)
                {
                    // 현재 정적장애물 기준 xyz 사이즈와 좌표위치가 10cm 오차 이내로 동일하면 동일 장애물로 associate 
                    // if ((ABS(iter->fused_cuboid_x - iter1->fused_cuboid_x)< 0.001) && (ABS(iter->fused_cuboid_y - iter1->fused_cuboid_y)< 0.001) && (ABS(iter->fused_cuboid_z - iter1->fused_cuboid_z)<0.001))
                    if ((ori_iter->fused_cuboid_x == new_iter->fused_cuboid_x) && (ori_iter->fused_cuboid_y == new_iter->fused_cuboid_y) && (ori_iter->fused_cuboid_z == new_iter->fused_cuboid_z)\
                    && (abs(ori_iter->fused_position_x) - (new_iter->fused_position_x) < 100) && (abs(ori_iter->fused_position_y - new_iter->fused_position_y) < 100) && (abs(ori_iter->fused_position_z - new_iter->fused_position_z) < 100)\
                    && (ori_iter->obstacle_class == new_iter->obstacle_class))
                    {
                        adcm::Log::Info() << "fused_position_x: " << ori_iter->fused_position_x << "fused_position_y:" << ori_iter->fused_position_y << "obstacle id" << ori_iter->obstacle_id;
                        adcm::Log::Info() << "new fused_position_x: " << new_iter->fused_position_x << "new fused_position_y:" << new_iter->fused_position_y;
                        adcm::Log::Info() << "Identical obstacle detected : " << ori_iter->obstacle_id;
                        new_iter->obstacle_id = ori_iter->obstacle_id;
                        obstacle_list_filtered.push_back(*new_iter);
                        new_iter = obstacle_list.erase(new_iter);
                        identicalObstacleFound = true;
                        break; //if 문 break
                        //동일한 장애물이 발견되면 obstacle_list 에서 obstacle_list_filtered 로 옮기고 obstacle list 에서는 삭제한다 
                    }
                    else
                    {
                        adcm::Log::Info() << "[ori]fused_position_x: " << ori_iter->fused_position_x << "fused_position_y:" << ori_iter->fused_position_y << "obstacle id" << ori_iter->obstacle_id;
                        adcm::Log::Info() << "[new]fused_position_x: " << new_iter->fused_position_x << "new fused_position_y:" << new_iter->fused_position_y;
                        adcm::Log::Info() << "[ori]fused_cuboid_x: " << ori_iter->fused_cuboid_x << "fused_cuboid_y:" << ori_iter->fused_cuboid_y << "fused_cuboid_z:" << ori_iter->fused_cuboid_z;
                        adcm::Log::Info() << "[new]fused_cuboid_x: " << new_iter->fused_cuboid_x << "fused_cuboid_y:" << new_iter->fused_cuboid_y << "fused_cuboid_z:" << new_iter->fused_cuboid_z;
                        adcm::Log::Info() << "[ori]obstacle_class: " << ori_iter->obstacle_class;
                        adcm::Log::Info() << "[new]obstacle_class: " << new_iter->obstacle_class;

                        adcm::Log::Info() << "Different obstacle detected : " << ori_iter->obstacle_id;
                        //동일하지 않은 장애물은 obstacle_list 에 계속 남긴다
                        ++new_iter;
                    }
                }
                if(identicalObstacleFound == false)
                {
                    //이전 장애물이 이번엔 발견되지 않았으므로 추후 id return 필요
                    removedObstacle.push_back(ori_iter->obstacle_id);
                }
            }

            // 새로운 객체들에 대한 id 지정
            for (auto new_iter = obstacle_list.begin(); new_iter != obstacle_list.end(); new_iter++)
            {
                new_iter->obstacle_id = id_manager.allocID();
                adcm::Log::Info() << "obstacle newly assigned is " << new_iter->obstacle_id;
                obstacle_list_filtered.push_back(*new_iter);
            }
            // 삭제된 객체에 대한 id 반납
            for (auto iter = removedObstacle.begin(); iter!=removedObstacle.end(); iter++)
            {
                id_manager.retID(*iter);
            }
        }
        obstacle_list.clear();

        for (auto filter_iter = obstacle_list_filtered.begin(); filter_iter != obstacle_list_filtered.end(); filter_iter++)
        {
            adcm::Log::Info() << "obstacle filtered are: " << filter_iter->obstacle_id;
        }
        adcm::Log::Info() << "장애물 ID allocation 완료";


        if (isMainVehicleInRange || (isSubVehicle1InRange && isSubVehicle2InRange))
        { // execute only if all true!
            //==============5. 0.1 m/s 미만인 경우 장애물 정지 상태 판정 및 stop_count 값 assign =================
            for (auto &obstacle : obstacle_list_filtered) {
                if (std::abs(obstacle.fused_velocity_x) < 0.1 && std::abs(obstacle.fused_velocity_y) < 0.1) {
                    obstacle.stop_count = 1; // Mark as stationary
                } else {
                    obstacle.stop_count = 0; // Not stationary
                }
            }

            // 이 전에 장애물의 stop status 를 카운트 하는 카운터 값 변동
            if (!mapData.obstacle_list.empty())
            {
                for (auto iter = mapData.obstacle_list.begin(); iter != mapData.obstacle_list.end(); iter++)
                { // 최초 mapData 아무것도 없으면 for loop 들어가지 않음

                    for (auto iter1 = obstacle_list_filtered.begin(); iter1 != obstacle_list_filtered.end(); iter1++)
                    {
                        if (iter->obstacle_id == iter1->obstacle_id)
                        { // 동일 장애물
                            if (iter1->stop_count == 1)
                            {
                                iter1->stop_count = iter->stop_count + iter1->stop_count;
                                // adcm::Log::Info() << "stop count updated to " << iter1->stop_count << "for obstacle " << iter1->obstacle_id;
                                //  stop_count 업데이트
                            }
                            // 한번이라도 stop_count가 0 이 라면 카운트 리셋
                        }
                    }
                }
            }
            mapData.obstacle_list.clear();

            adcm::Log::Info() << "stop count 변동 완료";
            //==============7. 장애물과 차량의 occupancy 계산해 map_2d_location 값 업데이트 ========

            if (!obstacle_list_filtered.empty())
            {
                find4VerticesObstacle(obstacle_list_filtered);
            }

            if (isMainVehicleInRange && main_vehicle.timestamp != 0)
            {
                // main vehicle 존재하므로 해당 function execution
                find4VerticesVehicle(main_vehicle);
            }

            if (isSubVehicle1InRange && sub1_vehicle.timestamp != 0)
            {
                // sub1_vehicle 존재하므로 해당 function execution
                find4VerticesVehicle(sub1_vehicle);
            }

            if (isSubVehicle2InRange && sub2_vehicle.timestamp != 0)
            {
                // sub2_vehicle 존재하므로 해당 function execution
                find4VerticesVehicle(sub2_vehicle);
            }

            //==============8. 현재까지의 데이터를 adcm mapData 형식으로 재구성해서 업데이트 ================
            //================ adcm mapData 내 obstacle list 업데이트 ===============================

            adcm::vehicleListStruct main_vehicle_final;
            adcm::vehicleListStruct sub1_vehicle_final;
            adcm::vehicleListStruct sub2_vehicle_final;
            adcm::Log::Info() << "장애물 mapdata 반영 예정 개수 : " << obstacle_list_filtered.size();
            for (auto iter = obstacle_list_filtered.begin(); iter != obstacle_list_filtered.end(); iter++)
            {
                adcm::obstacleListStruct obstacle_map; // 장애물 개수 무시
                // adcm::Log::Info() << "obstacle " << count << " start pushing";
                obstacle_map.obstacle_id = iter->obstacle_id;
                obstacle_map.obstacle_class = iter->obstacle_class;
                obstacle_map.timestamp = iter->timestamp;
                obstacle_map.map_2d_location.clear();
                for (auto iter1 = iter->map_2d_location.begin(); iter1 != iter->map_2d_location.end(); iter1++)
                {
                    adcm::map2dIndex index_to_push;
                    index_to_push.x = iter1->x;
                    index_to_push.y = iter1->y;
                    map_2d_test[index_to_push.x][index_to_push.y].obstacle_id = iter->obstacle_id;
                    // adcm::Log::Info() << "occupancy index pair of obstacle 1 is " << index_to_push.x << " , " << index_to_push.y;
                    // adcm::Log::Info() << "occupancy index of obstacle id " << iter->obstacle_id;
                    // adcm::Log::Info() << "map_2d_test[" << index_to_push.x << "][" <<index_to_push.y << "] = " <<  map_2d_test[index_to_push.x][index_to_push.y].obstacle_id;
                    obstacle_map.map_2d_location.push_back(index_to_push);
                }
                obstacle_map.stop_count = iter->stop_count;
                obstacle_map.fused_cuboid_x = iter->fused_cuboid_x;
                obstacle_map.fused_cuboid_y = iter->fused_cuboid_y;
                obstacle_map.fused_cuboid_z = iter->fused_cuboid_z;
                obstacle_map.fused_heading_angle = iter->fused_heading_angle;
                obstacle_map.fused_position_x = iter->fused_position_x;
                obstacle_map.fused_position_y = iter->fused_position_y;
                obstacle_map.fused_position_z = iter->fused_position_z;
                obstacle_map.fused_velocity_x = iter->fused_velocity_x;
                obstacle_map.fused_velocity_y = iter->fused_velocity_y;
                obstacle_map.fused_velocity_z = iter->fused_velocity_z;

                mapData.obstacle_list.push_back(obstacle_map);
            }
            adcm::Log::Info() << "mapData obstacle list size is " << mapData.obstacle_list.size();

            //================ adcm mapData 내 vehicle list 업데이트 ===============================

            mapData.vehicle_list.clear();

            if (isMainVehicleInRange && main_vehicle.timestamp != 0)
            {
                main_vehicle_final.vehicle_class = main_vehicle.vehicle_class;
                main_vehicle_final.timestamp = main_vehicle.timestamp;
                main_vehicle_final.map_2d_location.clear();
                // adcm::Log::Info() << "main_vehicle push to mapData (x:" << main_vehicle.map_2d_location[0].x << " ~ " << main_vehicle.map_2d_location[main_vehicle.map_2d_location.size() - 1].x << " )";
                // adcm::Log::Info() << "main_vehicle push to mapData (y:" << main_vehicle.map_2d_location[0].y << " ~ " << main_vehicle.map_2d_location[main_vehicle.map_2d_location.size() - 1].y << " )";
                for (auto iter1 = main_vehicle.map_2d_location.begin(); iter1 < main_vehicle.map_2d_location.end(); iter1++)
                {
                    adcm::map2dIndex index_to_push;
                    index_to_push.x = iter1->x;
                    index_to_push.y = iter1->y;
                    map_2d_test[index_to_push.x][index_to_push.y].vehicle_class = main_vehicle_final.vehicle_class;
                    map_2d_test[index_to_push.x][index_to_push.y].road_z = 1;
                    // adcm::Log::Info() << "(" << index_to_push.x << ", " << index_to_push.y << ") pushed to vehicle " << main_vehicle.vehicle_class;
                    // adcm::Log::Info() << "map_2d_test[" << index_to_push.x << "][" <<index_to_push.y << "] = " <<  map_2d_test[index_to_push.x][index_to_push.y].vehicle_class;
                    // adcm::Log::Info() << "map_2d_test[" << index_to_push.x << "][" <<index_to_push.y << "] = " <<  map_2d_test[index_to_push.x][index_to_push.y].road_z;
                    main_vehicle_final.map_2d_location.push_back(index_to_push);
                }

                main_vehicle_final.position_long = main_vehicle.position_long;
                main_vehicle_final.position_lat = main_vehicle.position_lat;
                main_vehicle_final.position_height = main_vehicle.position_height;
                main_vehicle_final.position_x = main_vehicle.position_x;
                main_vehicle_final.position_y = main_vehicle.position_y;
                main_vehicle_final.position_z = main_vehicle.position_z;
                main_vehicle_final.yaw = main_vehicle.yaw;
                main_vehicle_final.roll = main_vehicle.roll;
                main_vehicle_final.velocity_long = main_vehicle.velocity_long;
                main_vehicle_final.velocity_lat = main_vehicle.velocity_lat;
                main_vehicle_final.velocity_x = main_vehicle.velocity_x;
                main_vehicle_final.velocity_y = main_vehicle.velocity_y;
                main_vehicle_final.velocity_ang = main_vehicle.velocity_ang;
                mapData.vehicle_list.push_back(main_vehicle_final);
                // INFO("main_vehicle_final pushed to mapData");
            }

            if (isSubVehicle1InRange  && sub1_vehicle.timestamp != 0)
            // 테스트용 sub1 값이 있을때만 아래 수행
            {
                // adcm::Log::Info() << "sub1_vehicle push to mapData (x:" << sub1_vehicle.map_2d_location.begin()->x << " ~ " << sub1_vehicle.map_2d_location[sub1_vehicle.map_2d_location.size() - 1].x << " )";
                // adcm::Log::Info() << "sub1_vehicle push to mapData (y:" << sub1_vehicle.map_2d_location.begin()->y << " ~ " << sub1_vehicle.map_2d_location[sub1_vehicle.map_2d_location.size() - 1].y << " )";
                sub1_vehicle_final.vehicle_class = sub1_vehicle.vehicle_class;
                sub1_vehicle_final.timestamp = sub1_vehicle.timestamp;
                sub1_vehicle_final.map_2d_location.clear();
                for (auto iter1 = sub1_vehicle.map_2d_location.begin(); iter1 < sub1_vehicle.map_2d_location.end(); iter1++)
                {
                    adcm::map2dIndex index_to_push;
                    index_to_push.x = iter1->x;
                    index_to_push.y = iter1->y;
                    map_2d_test[index_to_push.x][index_to_push.y].vehicle_class = sub1_vehicle_final.vehicle_class;
                    map_2d_test[index_to_push.x][index_to_push.y].road_z = 1;
                    // adcm::Log::Info() << "map_2d_test[" << index_to_push.x << "][" <<index_to_push.y << "] = " <<  map_2d_test[index_to_push.x][index_to_push.y].vehicle_class;
                    sub1_vehicle_final.map_2d_location.push_back(index_to_push);
                }
                sub1_vehicle_final.position_long = sub1_vehicle.position_long;
                sub1_vehicle_final.position_lat = sub1_vehicle.position_lat;
                sub1_vehicle_final.position_height = sub1_vehicle.position_height;
                sub1_vehicle_final.position_x = sub1_vehicle.position_x;
                sub1_vehicle_final.position_y = sub1_vehicle.position_y;
                sub1_vehicle_final.position_z = sub1_vehicle.position_z;
                sub1_vehicle_final.yaw = sub1_vehicle.yaw;
                sub1_vehicle_final.roll = sub1_vehicle.roll;
                sub1_vehicle_final.velocity_long = sub1_vehicle.velocity_long;
                sub1_vehicle_final.velocity_lat = sub1_vehicle.velocity_lat;
                sub1_vehicle_final.velocity_x = sub1_vehicle.velocity_x;
                sub1_vehicle_final.velocity_y = sub1_vehicle.velocity_y;
                sub1_vehicle_final.velocity_ang = sub1_vehicle.velocity_ang;

                mapData.vehicle_list.push_back(sub1_vehicle_final);
                // INFO("sub1_vehicle_final pushed to mapData");
            }

            if (isSubVehicle2InRange  && sub2_vehicle.timestamp != 0)
            {
                sub2_vehicle_final.vehicle_class = sub2_vehicle.vehicle_class;
                sub2_vehicle_final.timestamp = sub2_vehicle.timestamp;
                sub2_vehicle_final.map_2d_location.clear();
                // adcm::Log::Info() << "sub2_vehicle push to mapData (x:" << sub2_vehicle.map_2d_location.begin()->x << " ~ " << sub2_vehicle.map_2d_location[sub2_vehicle.map_2d_location.size() - 1].x << " )";
                // adcm::Log::Info() << "sub2_vehicle push to mapData (y:" << sub2_vehicle.map_2d_location.begin()->y << " ~ " << sub2_vehicle.map_2d_location[sub2_vehicle.map_2d_location.size() - 1].y << " )";
                for (auto iter1 = sub2_vehicle.map_2d_location.begin(); iter1 < sub2_vehicle.map_2d_location.end(); iter1++)
                {
                    adcm::map2dIndex index_to_push;
                    index_to_push.x = iter1->x;
                    index_to_push.y = iter1->y;
                    map_2d_test[index_to_push.x][index_to_push.y].vehicle_class = sub2_vehicle_final.vehicle_class;
                    map_2d_test[index_to_push.x][index_to_push.y].road_z = 1;
                    // adcm::Log::Info() << "map_2d_test[" << index_to_push.x << "][" <<index_to_push.y << "] = " <<  map_2d_test[index_to_push.x][index_to_push.y].vehicle_class;
                    sub2_vehicle_final.map_2d_location.push_back(index_to_push);
                }
                sub2_vehicle_final.position_long = sub2_vehicle.position_long;
                sub2_vehicle_final.position_lat = sub2_vehicle.position_lat;
                sub2_vehicle_final.position_height = sub2_vehicle.position_height;
                sub2_vehicle_final.position_x = sub2_vehicle.position_x;
                sub2_vehicle_final.position_y = sub2_vehicle.position_y;
                sub2_vehicle_final.position_z = sub2_vehicle.position_z;
                sub2_vehicle_final.yaw = sub2_vehicle.yaw;
                sub2_vehicle_final.roll = sub2_vehicle.roll;
                sub2_vehicle_final.velocity_long = sub2_vehicle.velocity_long;
                sub2_vehicle_final.velocity_lat = sub2_vehicle.velocity_lat;
                sub2_vehicle_final.velocity_x = sub2_vehicle.velocity_x;
                sub2_vehicle_final.velocity_y = sub2_vehicle.velocity_y;
                sub2_vehicle_final.velocity_ang = sub2_vehicle.velocity_ang;

                mapData.vehicle_list.push_back(sub2_vehicle_final);
                // INFO("sub2_vehicle_final pushed to mapData");
            }

            mapData.map_2d.clear();
            for (int i = 0; i < map_n; ++i)
            {
                map_2dListVector.clear();
                for (int j = 0; j < map_m; ++j)
                {
                    // Copy and Reset Data for Each Grid Cell
                    map_2dStruct.obstacle_id = map_2d_test[i][j].obstacle_id;
                    map_2d_test[i][j].obstacle_id = NO_OBSTACLE;

                    map_2dStruct.vehicle_class = map_2d_test[i][j].vehicle_class;
                    map_2d_test[i][j].vehicle_class = NO_VEHICLE;

                    map_2dStruct.road_z = map_2d_test[i][j].road_z;
                    // road_z 정보는 reset 불필요
                    map_2dListVector.push_back(map_2dStruct);
                }
                mapData.map_2d.push_back(map_2dListVector);
            }
            adcm::Log::Info() << "DATA FUSION DONE";
            adcm::Log::Info() << "map_2d pushed to mapData";
            mapData_provider.send(mapData);
            mapVer++;
            adcm::Log::Info() << mapVer << "번째 허브 데이터 맵변환 후 전송 완료";
            adcm::Log::Info() << "mapData send";
        }
        else
        {
            adcm::Log::Info() << "Invalid input data - no map data sent";
            // mapData_provider.send(mapData);
        }
    }
}

void ThreadMonitor()
{
    while (continueExecution)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if (gMainthread_Loopcount == 0)
        {
            adcm::Log::Error() << "Main thread Timeout!!!";
        }
        else
        {
            gMainthread_Loopcount = 0;

            if (gReceivedEvent_count_hub_data != 0)
            {
                adcm::Log::Info() << "hub_data Received count = " << gReceivedEvent_count_hub_data;
                gReceivedEvent_count_hub_data = 0;
            }
            else
            {
                adcm::Log::Info() << "hub_data event timeout!!!";
            }
        }
    }
}

int main(int argc, char *argv[])
{
    std::vector<std::thread> thread_list;
    UNUSED(argc);
    UNUSED(argv);
    if (!ara::core::Initialize())
    {
        // No interaction with ARA is possible here since initialization failed
        return EXIT_FAILURE;
    }

    ara::exec::ExecutionClient exec_client;
    exec_client.ReportExecutionState(ara::exec::ExecutionState::kRunning);

    if (!RegisterSigTermHandler())
    {
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
    thread_list.push_back(std::thread(ThreadReceiveHubData));
    thread_list.push_back(std::thread(ThreadReceiveWorkInfo));
    thread_list.push_back(std::thread(ThreadMonitor));
    thread_list.push_back(std::thread(ThreadKatech));

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