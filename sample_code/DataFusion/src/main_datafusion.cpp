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
#include <sstream>
#include <fstream> // Required for file handling
#include <filesystem>
#include "main_datafusion.hpp"

#include <string>
#include <vector>

#define NATS
#ifdef NATS
#include "./NATS_IMPLEMENTATION/NatsConnManager.h"
#define HMI_SERVER_URL "https://nats.beyless.com"

bool firstTime = true;
natsStatus s = NATS_OK;
std::vector<const char *> subject = {"test1.*", "test2.*"};
std::shared_ptr<adcm::etc::NatsConnManager> natsManager;

void asyncCb(natsConnection *nc, natsSubscription *sub, natsStatus err, void *closure)
{
    std::cout << "Async error: " << err << " - " << natsStatus_GetText(err) << std::endl;
    natsManager->NatsSubscriptionGetDropped(sub, (int64_t *)&natsManager->dropped);
}

void onMsg(natsConnection *nc, natsSubscription *sub, natsMsg *msg, void *closure)
{
    const char *subject = NULL;

    // 뮤텍스를 사용하여 공유 변수 접근 보호
    std::lock_guard<std::mutex> lock(mtx);
    subject = natsMsg_GetSubject(msg);

    std::cout << "Received msg: [" << subject << " : " << natsMsg_GetDataLength(msg) << "]" << natsMsg_GetData(msg) << std::endl;
    // We should be using a mutex to protect those variables since
    // they are used from the subscription's delivery and the main
    // threads. For demo purposes, this is fine.

    // std::istringstream iss(subject);
    // std::vector<std::string> words;
    // std::string word;

    // while (std::getline(iss, word, '.'))
    // {
    //     words.push_back(word);
    // }
    // std::cout << words[0] << " Data Category : " << words[1] << std::endl;

    natsManager->NatsMsgDestroy(msg);
}

std::string convertMapDataToJsonString(const adcm::map_data_Objects &mapData)
{
    std::ostringstream oss; // Use a string stream for easier manipulation
    oss << "{\n";           // Start the JSON object

    // Convert map_2d to JSON
    oss << "    \"map_2d\": [\n";
    for (size_t i = 0; i < mapData.map_2d.size(); ++i)
    {
        oss << "        [\n"; // Start each row
        for (size_t j = 0; j < mapData.map_2d[i].size(); ++j)
        {
            const auto &item = mapData.map_2d[i][j];
            oss << "            {\n"
                << "                \"obstacle_id\": " << item.obstacle_id << ",\n"
                << "                \"vehicle_class\": " << static_cast<int>(item.vehicle_class) << ",\n"
                << "                \"road_z\": " << static_cast<int>(item.road_z) << "\n"
                << "            }";

            if (j < mapData.map_2d[i].size() - 1)
            {
                oss << ",";
            }
            oss << "\n"; // Newline for readability
        }
        oss << "        ]";

        if (i < mapData.map_2d.size() - 1)
        {
            oss << ",";
        }
        oss << "\n"; // Newline for readability
    }
    oss << "    ],\n";

    // Convert obstacle_list to JSON
    oss << "    \"obstacle_list\": [\n";
    for (size_t i = 0; i < mapData.obstacle_list.size(); ++i)
    {
        const auto &item = mapData.obstacle_list[i];
        oss << "        {\n"
            << "            \"obstacle_id\": " << item.obstacle_id << ",\n"
            << "            \"obstacle_class\": " << static_cast<int>(item.obstacle_class) << ",\n"
            << "            \"timestamp\": " << item.timestamp << ",\n"
            << "            \"stop_count\": " << static_cast<int>(item.stop_count) << ",\n"
            << "            \"fused_cuboid_x\": " << item.fused_cuboid_x << ",\n"
            << "            \"fused_cuboid_y\": " << item.fused_cuboid_y << ",\n"
            << "            \"fused_cuboid_z\": " << item.fused_cuboid_z << ",\n"
            << "            \"fused_heading_angle\": " << item.fused_heading_angle << ",\n"
            << "            \"fused_position_x\": " << item.fused_position_x << ",\n"
            << "            \"fused_position_y\": " << item.fused_position_y << ",\n"
            << "            \"fused_position_z\": " << item.fused_position_z << ",\n"
            << "            \"fused_velocity_x\": " << item.fused_velocity_x << ",\n"
            << "            \"fused_velocity_y\": " << item.fused_velocity_y << ",\n"
            << "            \"fused_velocity_z\": " << item.fused_velocity_z << ",\n"
            << "            \"map_2d_location\": [\n";

        for (size_t j = 0; j < item.map_2d_location.size(); ++j)
        {
            const auto &index = item.map_2d_location[j];
            oss << "                {\n"
                << "                    \"x\": " << index.x << ",\n"
                << "                    \"y\": " << index.y << "\n"
                << "                }";

            if (j < item.map_2d_location.size() - 1)
            {
                oss << ",";
            }
            oss << "\n"; // Newline for readability
        }

        oss << "            ]\n" // Close map_2d_location array
            << "        }";

        if (i < mapData.obstacle_list.size() - 1)
        {
            oss << ",";
        }
        oss << "\n"; // Newline for readability
    }
    oss << "    ],\n";

    // Convert vehicle_list to JSON
    oss << "    \"vehicle_list\": [\n";
    for (size_t i = 0; i < mapData.vehicle_list.size(); ++i)
    {
        const auto &item = mapData.vehicle_list[i];
        oss << "        {\n"
            << "            \"vehicle_class\": " << static_cast<int>(item.vehicle_class) << ",\n"
            << "            \"timestamp\": " << item.timestamp << ",\n"
            << "            \"position_long\": " << item.position_long << ",\n"
            << "            \"position_lat\": " << item.position_lat << ",\n"
            << "            \"position_height\": " << item.position_height << ",\n"
            << "            \"position_x\": " << item.position_x << ",\n"
            << "            \"position_y\": " << item.position_y << ",\n"
            << "            \"position_z\": " << item.position_z << ",\n"
            << "            \"yaw\": " << item.yaw << ",\n"
            << "            \"roll\": " << item.roll << ",\n"
            << "            \"pitch\": " << item.pitch << ",\n"
            << "            \"velocity_long\": " << item.velocity_long << ",\n"
            << "            \"velocity_lat\": " << item.velocity_lat << ",\n"
            << "            \"velocity_x\": " << item.velocity_x << ",\n"
            << "            \"velocity_y\": " << item.velocity_y << ",\n"
            << "            \"velocity_ang\": " << item.velocity_ang << ",\n"
            << "            \"map_2d_location\": [\n";

        for (size_t j = 0; j < item.map_2d_location.size(); ++j)
        {
            const auto &index = item.map_2d_location[j];
            oss << "                {\n"
                << "                    \"x\": " << index.x << ",\n"
                << "                    \"y\": " << index.y << "\n"
                << "                }";

            if (j < item.map_2d_location.size() - 1)
            {
                oss << ",";
            }
            oss << "\n"; // Newline for readability
        }

        oss << "            ]\n" // Close map_2d_location array
            << "        }";

        if (i < mapData.vehicle_list.size() - 1)
        {
            oss << ",";
        }
        oss << "\n"; // Newline for readability
    }
    oss << "    ]\n"; // Close vehicle_list

    oss << "}"; // Close the main JSON object

    return oss.str(); // Return the concatenated JSON string
}

void saveToJsonFile(const std::string &key, const std::string &value, int &fileCount)
{
    std::ostringstream fileNameStream;
    fileNameStream << key << "_" << fileCount << ".json"; // Construct the file name
    std::string fileName = fileNameStream.str();
    std::ofstream outFile(fileName);
    // Open file for writing with the generated name
    if (outFile.is_open())
    {
        outFile << value; // Write the JSON string to the file
        outFile.close();  // Close the file after writing
        adcm::Log::Info() << "JSON data stored in " << fileName;
    }
    else
    {
        adcm::Log::Error() << "Failed to open file " << fileName << " for writing!";
    }

    // Increment the file count for the next call
    ++fileCount;
}
void NatsStart()
{
    if (firstTime == true)
    {
        adcm::Log::Info() << "NATS first time setup!";
        natsManager = std::make_shared<adcm::etc::NatsConnManager>(HMI_SERVER_URL, subject, onMsg, asyncCb, adcm::etc::NatsConnManager::Mode::Default);
        s = natsManager->NatsExecute();
        firstTime = false;
    }
}
void NatsSend(const adcm::map_data_Objects &mapData)
{
    static int mapData_count = 0;
    if (s == NATS_OK)
    {
        const char *pubSubject = "mapdata.map";
        natsManager->ClearJsonData();
        adcm::Log::Info() << "NATS conversion start!";
        std::string mapDataStr = convertMapDataToJsonString(mapData);
        adcm::Log::Info() << "NATS conversion done!";
        natsManager->addJsonData("mapData", mapDataStr);
        adcm::Log::Info() << "NATS make Json Data!";
        natsManager->NatsPublishJson(pubSubject);
        adcm::Log::Info() << "NATS publish Json!"; // publish가 오래 걸림
        // saveToJsonFile("mapData", mapDataStr, mapData_count);
        // adcm::Log::Info() << "NATS save Json file!";
    }
    else
    {
        std::cout << "Nats Connection error" << std::endl;
        try
        {
            natsManager = std::make_shared<adcm::etc::NatsConnManager>(HMI_SERVER_URL, subject, onMsg, asyncCb, adcm::etc::NatsConnManager::Mode::Default);
            s = natsManager->NatsExecute();
        }
        catch (std::exception e)
        {
            std::cout << "Nats reConnection error" << std::endl;
        }
    }
}

#endif

void GPStoUTM(double lat, double lon, double &utmX, double &utmY)
{
    const double WGS84_A = 6378137.0;
    const double WGS84_E = 0.0818191908;
    const double k0 = 0.9996;
    const double eSquared = WGS84_E * WGS84_E;
    const double ePrimeSquared = eSquared / (1 - eSquared);
    const double RADIANS_PER_DEGREE = M_PI / 180.0;

    int zone = 52;
    double lonOrigin = (zone - 1) * 6 - 180 + 3;
    double lonOriginRad = lonOrigin * RADIANS_PER_DEGREE;

    double latRad = lat * RADIANS_PER_DEGREE;
    double lonRad = lon * RADIANS_PER_DEGREE;

    double sinLatRad = sin(latRad);
    double cosLatRad = cos(latRad);
    double tanLatRad = tan(latRad);

    double N = WGS84_A / sqrt(1 - eSquared * sinLatRad * sinLatRad);
    double T = tanLatRad * tanLatRad;
    double C = ePrimeSquared * cosLatRad * cosLatRad;
    double A = cosLatRad * (lonRad - lonOriginRad);

    double M = WGS84_A * ((1 - eSquared / 4 - 3 * eSquared * eSquared / 64 - 5 * eSquared * eSquared * eSquared / 256) * latRad - (3 * eSquared / 8 + 3 * eSquared * eSquared / 32 + 45 * eSquared * eSquared * eSquared / 1024) * sin(2 * latRad) + (15 * eSquared * eSquared / 256 + 45 * eSquared * eSquared * eSquared / 1024) * sin(4 * latRad) - (35 * eSquared * eSquared * eSquared / 3072) * sin(6 * latRad));

    utmX = (k0 * N * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ePrimeSquared) * pow(A, 5) / 120) + 500000.0);

    utmY = (k0 * (M + N * tanLatRad * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24 + (61 - 58 * T + T * T + 600 * C - 330 * ePrimeSquared) * pow(A, 6) / 720)));

    if (lat < 0)
    {
        utmY += 10000000.0;
    }
}

bool checkRange(const VehicleData &vehicle)
{
    if (vehicle.position_x >= 0 && vehicle.position_x < map_n &&
        vehicle.position_y >= 0 && vehicle.position_y < map_m)
        return true;
    return false;
}

void checkRange(Point2D &point)
{
    if (point.x < 0)
        point.x = 0;
    if (point.y < 0)
        point.y = 0;
    if (point.x > map_n)
        point.x = map_n - 1;
    if (point.y > map_m)
        point.y = map_m - 1;
}

bool checkAllVehicleRange(const std::vector<VehicleData *> &vehicles)
{
    for (const auto *vehicle : vehicles)
    {
        adcm::Log::Info() << vehicle->vehicle_class << "번 차량 위치: [" << vehicle->position_x << ", " << vehicle->position_y << "]";
        if (vehicle && !checkRange(*vehicle))
        {
            return false;
        }
    }
    return true;
}

/*
void processVehicleData(moodycamel::ConcurrentQueue<FusionData> &vehicleQueue,
                        FusionData &vehicleData,
                        VehicleData &vehicle,
                        std::vector<ObstacleData> &obstacleList)
{
    vehicleQueue.try_dequeue(vehicleData);
    vehicle = vehicleData.vehicle;
    obstacleList = vehicleData.obstacle_list;
    filterVehicleData(obstacleList);
    gpsToMapcoordinate(vehicle);
    relativeToMapcoordinate(obstacleList, vehicle);
}
*/

void processVehicleData(FusionData &vehicleData,
                        VehicleData &vehicle,
                        std::vector<ObstacleData> &obstacleList)
{
    vehicle = vehicleData.vehicle;
    obstacleList = vehicleData.obstacle_list;
    filterVehicleData(obstacleList);
    gpsToMapcoordinate(vehicle);
    relativeToMapcoordinate(obstacleList, vehicle);
}

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
    for (const auto &p : points)
    {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }

    Point2D index;
    // Ray-Casting algorithm
    for (index.x = min_x; index.x <= max_x; ++index.x)
    {
        for (index.y = min_y; index.y <= max_y; ++index.y)
        {
            int cross = 0;
            // 해당 사변형은 4변을 가지므로 i
            for (int i = 0; i < 4; i++)
            {
                // As the loop variable i iterates from 0 to 3, j will always represent the next vertex in the sequence
                int j = (i + 1) % 4;
                if ((points[i].y > index.y) != (points[j].y > index.y))
                {
                    // 교차점을 구한다
                    double meetX = (points[j].x - points[i].x) * (index.y - points[i].y) /
                                       (points[j].y - points[i].y) +
                                   points[i].x;
                    // 교차점 meetX 가 검증을 진행하는 인덱스의 x 좌표보다 크면 교차발생 cross++
                    if (index.x < meetX)
                        cross++;
                }
            }
            // 교차횟수 cross가 짝수이면 점은 외부,
            // 교차횟수 cross가 홀수이면 점은 내부에 있음
            if (cross % 2 != 0)
            {
                vehicle.map_2d_location.push_back(index);
                // map_2d_test[index.x][index.y].vehicle_class = vehicle.vehicle_class;
                //  TO DO: 현재는 물체가 있는 index 는 road_z 값 1로 설정 (아무것도 없으면 0)
                // mapData.map_2d[index.x][index.y].road_z = 1;
            }
        }
    }
}

void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, std::vector<ObstacleData>::iterator iter)
{
    if (iter == std::vector<ObstacleData>::iterator())
        return; // Ensure valid iterator

    // Collect all points in an array
    Point2D points[] = {p0, p1, p2, p3};

    // Find min and max values for x and y
    double min_x = points[0].x, max_x = points[0].x;
    double min_y = points[0].y, max_y = points[0].y;
    for (const auto &p : points)
    {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }

    Point2D index;

    for (index.x = min_x; index.x <= max_x; ++index.x)
    {
        for (index.y = min_y; index.y <= max_y; ++index.y)
        {
            int cross = 0;
            for (int i = 0; i < 4; i++)
            {
                int j = (i + 1) % 4;
                if ((points[i].y > index.y) != (points[j].y > index.y))
                {
                    double meetX = (points[j].x - points[i].x) * (index.y - points[i].y) /
                                       (points[j].y - points[i].y) +
                                   points[i].x;
                    if (index.x < meetX)
                        cross++;
                }
            }
            if (cross % 2 != 0)
            {
                iter->map_2d_location.push_back(index);
                // map_2d_test[index.x][index.y].obstacle_id = iter->obstacle_id;
                //  TO DO: 현재는 물체가 있는 index 는 road_z 값 1로 설정 (아무것도 없으면 0)
                // map_2d_test[index.x][index.y].road_z = 1;
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
    RU.x = target_vehicle.position_x + cos(theta) * (half_x)-sin(theta) * (half_y);
    RU.y = target_vehicle.position_y + sin(theta) * (half_x) + cos(theta) * (half_y);

    // Bottom-right (RL)
    RL.x = target_vehicle.position_x + cos(theta) * (half_x)-sin(theta) * (-half_y);
    RL.y = target_vehicle.position_y + sin(theta) * (half_x) + cos(theta) * (-half_y);

    // Bottom-left (LL)
    LL.x = target_vehicle.position_x + cos(theta) * (-half_x) - sin(theta) * (-half_y);
    LL.y = target_vehicle.position_y + sin(theta) * (-half_x) + cos(theta) * (-half_y);

    checkRange(LU);
    checkRange(RU);
    checkRange(RL);
    checkRange(LL);

    // generateRoadZValue(target_vehicle, map_2d_test);
    // TO DO
    generateOccupancyIndex(LU, RU, RL, LL, target_vehicle);
}

void find4VerticesObstacle(std::vector<ObstacleData> &obstacle_list_filtered)
{
    for (auto iter = obstacle_list_filtered.begin(); iter < obstacle_list_filtered.end(); iter++)
    {
        Point2D LU, RU, RL, LL;
        double half_x = iter->fused_cuboid_x / 2 * M_TO_10CM_PRECISION;
        double half_y = iter->fused_cuboid_y / 2 * M_TO_10CM_PRECISION;
        double obstacle_position_x = iter->fused_position_x;
        double obstacle_position_y = iter->fused_position_y;
        double theta = iter->fused_heading_angle * M_PI / 180;

        // Top-left (LU)
        LU.x = obstacle_position_x + cos(theta) * (-half_x) - sin(theta) * (half_y);
        LU.y = obstacle_position_y + sin(theta) * (-half_x) + cos(theta) * (half_y);

        // Top-right (RU)
        RU.x = obstacle_position_x + cos(theta) * (half_x)-sin(theta) * (half_y);
        RU.y = obstacle_position_y + sin(theta) * (half_x) + cos(theta) * (half_y);

        // Bottom-right (RL)
        RL.x = obstacle_position_x + cos(theta) * (half_x)-sin(theta) * (-half_y);
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

// 유클리디안 거리 계산
double euclideanDistance(const ObstacleData &a, const ObstacleData &b)
{
    return std::sqrt(std::pow(a.fused_position_x - b.fused_position_x, 2) +
                     std::pow(a.fused_position_y - b.fused_position_y, 2));
}

// 거리 행렬 생성 함수
std::vector<std::vector<double>> createDistanceMatrix(
    const std::vector<ObstacleData> &listA,
    const std::vector<ObstacleData> &listB)
{
    std::vector<std::vector<double>> distanceMatrix(listA.size(), std::vector<double>(listB.size()));
    for (size_t i = 0; i < listA.size(); ++i)
    {
        for (size_t j = 0; j < listB.size(); ++j)
        {
            distanceMatrix[i][j] = euclideanDistance(listA[i], listB[j]);
        }
    }
    return distanceMatrix;
}

// Munkres 알고리즘으로 매칭
std::vector<int> solveAssignment(const std::vector<std::vector<double>> &costMatrix)
{
    Matrix<double> matrix(costMatrix.size(), costMatrix[0].size());
    for (size_t i = 0; i < costMatrix.size(); ++i)
    {
        for (size_t j = 0; j < costMatrix[i].size(); ++j)
        {
            matrix(i, j) = costMatrix[i][j];
        }
    }
    Munkres<double> munkres;
    munkres.solve(matrix);
    std::vector<int> assignment(costMatrix.size(), -1);
    for (size_t i = 0; i < costMatrix.size(); ++i)
    {
        for (size_t j = 0; j < costMatrix[i].size(); ++j)
        {
            if (matrix(i, j) == 0)
            {
                assignment[i] = j;
            }
        }
    }
    return assignment;
}

// 신뢰성 기반 융합 계산 함수
double calculateWeightedPosition(const std::vector<double> &positions, const std::vector<double> &variances)
{
    double weightedSum = 0.0;
    double totalVarianceInverse = 0.0;
    for (size_t i = 0; i < positions.size(); ++i)
    {
        double varianceInverse = 1.0 / variances[i];
        weightedSum += positions[i] * varianceInverse;
        totalVarianceInverse += varianceInverse;
    }
    return (1.0 / totalVarianceInverse) * weightedSum;
}

// 장애물 리스트에서 특정 차량 데이터를 제외하는 함수
void filterVehicleData(std::vector<ObstacleData> &obstacles)
{
    for (auto iter = obstacles.begin(); iter != obstacles.end();)
    {
        // if ((abs(iter->fused_cuboid_x - SUB_VEHICLE_SIZE_X / 10)) < 0.1 && (abs(iter->fused_cuboid_y - SUB_VEHICLE_SIZE_Y / 10)) < 0.1)
        if (iter->obstacle_class == 51)
        {
            // adcm::Log::Info() << "보조차량 제거";
            iter = obstacles.erase(iter);
        }
        // if ((abs(iter->fused_cuboid_x - MAIN_VEHICLE_SIZE_X / 10)) < 0.1 && (abs(iter->fused_cuboid_y - MAIN_VEHICLE_SIZE_Y / 10)) < 0.1)
        else if (iter->obstacle_class == 50)
        {
            // adcm::Log::Info() << "특장차 제거";
            iter = obstacles.erase(iter);
        }
        else
            iter++;
    }

    return;
}

// 데이터 융합
void processFusion(
    std::vector<ObstacleData> &presList,
    const std::vector<ObstacleData> &prevList,
    const std::vector<int> &assignment)
{
    std::vector<ObstacleData> newList;

    // 1. 매칭된 장애물 처리
    for (size_t i = 0; i < assignment.size(); ++i)
    {
        int j = assignment[i];
        // 매칭된 장애물 newList에 추가
        if (j != -1)
        {
            presList[j].obstacle_id = prevList[i].obstacle_id;
            newList.push_back(presList[j]);
        }
    }

    // 2. prevList에서 매칭되지 않은 장애물 처리
    for (size_t i = 0; i < prevList.size(); ++i)
    {
        int j = assignment[i];
        // prevList에서 매칭되지 않은 항목을 newList에 추가
        if (j == -1)
            newList.push_back(prevList[i]);
    }

    // 3. presList에서 매칭되지 않은 장애물 처리
    for (size_t i = 0; i < presList.size(); ++i)
    {
        if (std::find(assignment.begin(), assignment.end(), i) == assignment.end())
        {
            // presList에서 매칭되지 않은 항목을 newList에 추가
            presList[i].obstacle_id = id_manager.allocID();
            newList.push_back(presList[i]);
        }
    }

    // 새로운 리스트로 갱신
    presList = newList;
}

// // 데이터 융합
// void processFusion(
//     std::vector<ObstacleData> &presList,
//     const std::vector<ObstacleData> &prevList,
//     const std::vector<int> &assignment)
// {
//     std::vector<ObstacleData> newList;
//     for (size_t i = 0; i < assignment.size(); ++i)
//     {
//         int j = assignment[i];
//         if (j != -1)
//         {
//             // const ObstacleData &recentData = (presList[i].timestamp > prevList[j].timestamp) ? presList[i] : prevList[j];
//             // // adcm::Log::Info() << "recentData 선정";
//             // ObstacleData fused;

//             // // 신뢰성 기반으로 위치 융합
//             // std::vector<double> positionsX = {presList[i].fused_position_x, prevList[j].fused_position_x};
//             // std::vector<double> positionsY = {presList[i].fused_position_y, prevList[j].fused_position_y};
//             // std::vector<double> positionsZ = {presList[i].fused_position_z, prevList[j].fused_position_z};

//             // std::vector<double> variances = {1.0 / presList[i].standard_deviation, 1.0 / prevList[j].standard_deviation};

//             // fused.fused_position_x = calculateWeightedPosition(positionsX, variances);
//             // fused.fused_position_y = calculateWeightedPosition(positionsY, variances);
//             // fused.fused_position_z = calculateWeightedPosition(positionsZ, variances);

//             // fused.fused_position_x = recentData.fused_position_x;
//             // fused.fused_position_y = recentData.fused_position_y;
//             // fused.fused_position_z = recentData.fused_position_z;

//             // // 나머지 정보는 최신 데이터로
//             // fused.obstacle_id = recentData.obstacle_id;
//             // fused.obstacle_class = recentData.obstacle_class;
//             // fused.map_2d_location = recentData.map_2d_location;
//             // fused.stop_count = recentData.stop_count;
//             // fused.fused_cuboid_x = recentData.fused_cuboid_x;
//             // fused.fused_cuboid_y = recentData.fused_cuboid_y;
//             // fused.fused_cuboid_z = recentData.fused_cuboid_z;
//             // fused.fused_heading_angle = recentData.fused_heading_angle;
//             // fused.fused_velocity_x = recentData.fused_velocity_x;
//             // fused.fused_velocity_y = recentData.fused_velocity_y;
//             // fused.fused_velocity_z = recentData.fused_velocity_z;
//             // fused.standard_deviation = recentData.standard_deviation;
//             // fused.timestamp = recentData.timestamp;

//             // newList.push_back(fused);

//             newList.push_back(presList[i]);
//         }
//     }

//     for (size_t j = 0; j < presList.size(); ++j)
//     {
//         // assignment에 존재하지 않는 j 값을 가지는 항목들을 추가
//         if (std::find(assignment.begin(), assignment.end(), j) == assignment.end())
//         {
//             newList.push_back(presList[j]);
//         }
//     }

//     presList = newList;
// }

// 새 데이터에 대한 ID 관리 및 부여
void assignIDsForNewData(
    std::vector<ObstacleData> &resultFusionList,
    const std::vector<ObstacleData> &currentFusionList,
    const std::vector<int> &assignment)
{
    // 기존 매칭된 인덱스 집합
    std::set<int> matchedIndices(assignment.begin(), assignment.end());
    for (size_t i = 0; i < currentFusionList.size(); ++i)
    {
        if (i >= assignment.size() || matchedIndices.find(i) == matchedIndices.end())
        {
            ObstacleData newObstacle = currentFusionList[i];
            newObstacle.obstacle_id = id_manager.allocID();
            resultFusionList.push_back(newObstacle);
        }
        else
        {
            resultFusionList.push_back(currentFusionList[i]);
        }
    }
}

// 장애물 리스트 융합 및 이전 데이터와 비교
std::vector<ObstacleData> mergeAndCompareLists(
    const std::vector<ObstacleData> &previousFusionList,
    std::vector<ObstacleData> listMain,
    std::vector<ObstacleData> listSub1,
    std::vector<ObstacleData> listSub2,
    const VehicleData &mainVehicle,
    const VehicleData &sub1Vehicle,
    const VehicleData &sub2Vehicle)
{
    std::vector<VehicleData> nonEmptyVehicles;
    std::vector<std::vector<ObstacleData>> nonEmptyLists;
    std::vector<ObstacleData> mergedList;

    if (mainVehicle.timestamp)
    {
        nonEmptyVehicles.push_back(mainVehicle);
        nonEmptyLists.push_back(listMain);
    }
    if (sub1Vehicle.timestamp)
    {
        nonEmptyVehicles.push_back(sub1Vehicle);
        nonEmptyLists.push_back(listSub1);
    }
    if (sub2Vehicle.timestamp)
    {
        nonEmptyVehicles.push_back(sub2Vehicle);
        nonEmptyLists.push_back(listSub2);
    }

    // adcm::Log::Info() << "융합: 빈 데이터 제외 완료: " << nonEmptyLists.size() << ", " << nonEmptyVehicles.size();
    // 융합할 리스트 필터링
    if (nonEmptyLists.size() == 1)
    {
        // 유일한 리스트 하나가 있을 경우 그대로 사용
        mergedList = nonEmptyLists[0];
    }

    else
    {
        // 둘 이상 리스트가 있을 때 융합 수행
        auto handleFusionForPair = [&](const std::vector<ObstacleData> &listA, const std::vector<ObstacleData> &listB)
        {
            std::vector<ObstacleData> fusionList = listA;
            if (!listA.empty() && !listB.empty())
            {
                auto distMatrix = createDistanceMatrix(listB, listA);
                auto assignment = solveAssignment(distMatrix);
                // for (int i = 0; i < assignment.size(); i++)
                //     adcm::Log::Info() << "assignment" << i << ": " << assignment[i];
                processFusion(fusionList, listB, assignment);
            }
            return fusionList;
        };

        // 처음 두 개 리스트 융합
        // for (auto first : nonEmptyLists[0])
        //     adcm::Log::Info() << first.obstacle_id << ": [" << first.fused_position_x << ", " << first.fused_position_y << "]";
        // for (auto second : nonEmptyLists[1])
        //     adcm::Log::Info() << second.obstacle_id << ": [" << second.fused_position_x << ", " << second.fused_position_y << "]";

        mergedList = handleFusionForPair(nonEmptyLists[0], nonEmptyLists[1]);
        // adcm::Log::Info() << "융합: 융합 1번 완료";

        // 세 번째 리스트가 있다면 그 결과와 함께 융합
        if (nonEmptyLists.size() > 2)
        {
            mergedList = handleFusionForPair(mergedList, nonEmptyLists[2]);
            // adcm::Log::Info() << "융합: 융합 2번 완료";
        }
    }

    if (mergedList.empty())
    {
        if (previousFusionList.empty())
            adcm::Log::Info() << "장애물 리스트 비어있음";
        else
            adcm::Log::Info() << "현재 TimeStamp 장애물 X, 이전 TimeStamp 장애물리스트 그대로 사용";
        return previousFusionList;
    }
    else
    {
        if (previousFusionList.empty())
        {
            for (auto &obstacle : mergedList)
                obstacle.obstacle_id = id_manager.allocID();
            adcm::Log::Info() << "새로운 장애물 리스트 생성: " << id_manager.getNum();
            return mergedList;
        }

        else
        {
            adcm::Log::Info() << "previousFusionList size: " << previousFusionList.size();
            adcm::Log::Info() << "mergedList size: " << mergedList.size();
            // adcm::Log::Info() << "융합: 이전 데이터와 융합하여 ID부여 시도";
            auto distMatrix = createDistanceMatrix(previousFusionList, mergedList);
            // adcm::Log::Info() << "융합: 거리배열 생성";
            auto assignment = solveAssignment(distMatrix);
            // adcm::Log::Info() << "융합: Munkres Algorithm 적용: " << assignment.size();
            // for (int i = 0; i < assignment.size(); i++)
            //     adcm::Log::Info() << "assignment" << i << ": " << assignment[i];

            processFusion(mergedList, previousFusionList, assignment);
            // adcm::Log::Info() << "융합: 이전 데이터와 융합 완료";
            // for (auto merge : mergedList)
            // {
            //     adcm::Log::Info() << "융합리스트 장애물id: " << merge.obstacle_id;
            // }
            // std::vector<ObstacleData> finalList;
            // assignIDsForNewData(finalList, mergedList, assignment);
            // adcm::Log::Info() << "융합: ID부여 완료: " << id_manager.getNum();
            return mergedList;
        }
    }
}

// VehicleData -> vehicleListStruct(맵데이터 호환)
adcm::vehicleListStruct ConvertToVehicleListStruct(const VehicleData &vehicle, std::vector<adcm::map_2dListVector> &map)
{
    adcm::vehicleListStruct vehicle_final;
    vehicle_final.vehicle_class = vehicle.vehicle_class;
    vehicle_final.timestamp = vehicle.timestamp;
    // adcm::Log::Info() << "차량 map_2d_location 사이즈: " << vehicle.map_2d_location.size();
    // map_2d_size += vehicle.map_2d_location.size() * sizeof(Point2D);
    for (const auto &point : vehicle.map_2d_location)
    {
        adcm::map2dIndex index_to_push = {point.x, point.y};
        auto &map_cell = map[index_to_push.x][index_to_push.y];
        map_cell.vehicle_class = vehicle_final.vehicle_class;
        map_cell.road_z = 1;
        vehicle_final.map_2d_location.push_back(index_to_push);
    }

    vehicle_final.position_long = vehicle.position_long;
    vehicle_final.position_lat = vehicle.position_lat;
    vehicle_final.position_height = vehicle.position_height;
    vehicle_final.position_x = vehicle.position_x;
    vehicle_final.position_y = vehicle.position_y;
    vehicle_final.position_z = vehicle.position_z;
    vehicle_final.yaw = vehicle.yaw;
    vehicle_final.roll = vehicle.roll;
    vehicle_final.pitch = vehicle.pitch;
    vehicle_final.velocity_long = vehicle.velocity_long;
    vehicle_final.velocity_lat = vehicle.velocity_lat;
    vehicle_final.velocity_x = vehicle.velocity_x;
    vehicle_final.velocity_y = vehicle.velocity_y;
    vehicle_final.velocity_ang = vehicle.velocity_ang;

    return vehicle_final;
}

// ObstacleData -> obstacleListStruct(맵데이터 호환)
adcm::obstacleListStruct ConvertToObstacleListStruct(const ObstacleData &obstacle, std::vector<adcm::map_2dListVector> &map)
{
    adcm::obstacleListStruct obstacle_map;
    obstacle_map.obstacle_id = obstacle.obstacle_id;
    obstacle_map.obstacle_class = obstacle.obstacle_class;
    obstacle_map.timestamp = obstacle.timestamp;
    // adcm::Log::Info() << "장애물 map_2d_location 사이즈: " << obstacle.map_2d_location.size();
    // map_2d_size += obstacle.map_2d_location.size() * sizeof(Point2D);
    for (const auto &point : obstacle.map_2d_location)
    {
        adcm::map2dIndex index_to_push = {point.x, point.y};
        map[index_to_push.x][index_to_push.y].obstacle_id = obstacle.obstacle_id;
        obstacle_map.map_2d_location.push_back(index_to_push);
    }

    obstacle_map.stop_count = obstacle.stop_count;
    obstacle_map.fused_cuboid_x = obstacle.fused_cuboid_x;
    obstacle_map.fused_cuboid_y = obstacle.fused_cuboid_y;
    obstacle_map.fused_cuboid_z = obstacle.fused_cuboid_z;
    obstacle_map.fused_heading_angle = obstacle.fused_heading_angle;
    obstacle_map.fused_position_x = obstacle.fused_position_x;
    obstacle_map.fused_position_y = obstacle.fused_position_y;
    obstacle_map.fused_position_z = obstacle.fused_position_z;
    obstacle_map.fused_velocity_x = obstacle.fused_velocity_x;
    obstacle_map.fused_velocity_y = obstacle.fused_velocity_y;
    obstacle_map.fused_velocity_z = obstacle.fused_velocity_z;

    return obstacle_map;
}

void UpdateMapData(adcm::map_data_Objects &mapData, const std::vector<ObstacleData> &obstacle_list, const std::vector<VehicleData *> &vehicles)
{
    mapData.obstacle_list.clear();
    mapData.vehicle_list.clear();
    for (const auto &obstacle : obstacle_list)
        mapData.obstacle_list.push_back(ConvertToObstacleListStruct(obstacle, mapData.map_2d));

    for (const auto vehicle : vehicles)
    {
        if (vehicle->timestamp != 0)
        {
            mapData.vehicle_list.push_back(ConvertToVehicleListStruct(*vehicle, mapData.map_2d));
        }
    }

    adcm::Log::Info() << "mapData 장애물 반영 완료 개수: " << mapData.obstacle_list.size();
    adcm::Log::Info() << "mapData 차량 반영 완료 개수: " << mapData.vehicle_list.size();
    // adcm::Log::Info() << "map_2d_location_size: " << map_2d_size;
    // adcm::Log::Info() << "map_2d size: " << mapData.map_2d.size() * mapData.map_2d[0].size() * sizeof(::adcm::map_2dListStruct);
    map_2d_size = 0;

    return;
}

// 차량 데이터 저장
void fillVehicleData(VehicleData &vehicle_fill, const std::shared_ptr<adcm::hub_data_Objects> &data)
{
    vehicle_fill.vehicle_class = data->vehicle_class;
    vehicle_fill.timestamp = data->timestamp;
    vehicle_fill.position_lat = data->position_lat;
    vehicle_fill.position_long = data->position_long;
    vehicle_fill.position_height = data->position_height;
    vehicle_fill.yaw = data->yaw;
    vehicle_fill.roll = data->roll;
    vehicle_fill.pitch = data->pitch;
    vehicle_fill.velocity_long = data->velocity_long;
    vehicle_fill.velocity_lat = data->velocity_lat;
    vehicle_fill.velocity_ang = data->velocity_ang;
    return;
}

// 장애물 데이터 저장
void fillObstacleList(std::vector<ObstacleData> &obstacle_list_fill, const std::shared_ptr<adcm::hub_data_Objects> &data)
{
    obstacle_list_fill.clear();
    for (const auto &obstacle : data->obstacle)
    {
        ObstacleData obstacle_to_push;
        obstacle_to_push.obstacle_class = obstacle.obstacle_class;
        obstacle_to_push.timestamp = data->timestamp;
        obstacle_to_push.fused_cuboid_x = obstacle.cuboid_x;
        obstacle_to_push.fused_cuboid_y = obstacle.cuboid_y;
        obstacle_to_push.fused_cuboid_z = obstacle.cuboid_z;
        obstacle_to_push.fused_heading_angle = obstacle.heading_angle;
        obstacle_to_push.fused_position_x = obstacle.position_x;
        obstacle_to_push.fused_position_y = obstacle.position_y;
        obstacle_to_push.fused_position_z = obstacle.position_z;
        obstacle_to_push.fused_velocity_x = obstacle.velocity_x;
        obstacle_to_push.fused_velocity_y = obstacle.velocity_y;
        obstacle_to_push.fused_velocity_z = obstacle.velocity_z;
        obstacle_list_fill.push_back(obstacle_to_push);
    }
    return;
}

// hubData 수신
void ThreadReceiveHubData()
{
    adcm::HubData_Subscriber hubData_subscriber;
    // INFO("DataFusion .init()");
    hubData_subscriber.init("DataFusion/DataFusion/RPort_hub_data");
    adcm::Log::Info() << "ThreadReceiveHubData start...";

    while (continueExecution)
    {
        gMainthread_Loopcount++;
        if (!hubData_subscriber.waitEvent(10000))
            continue;

        while (!hubData_subscriber.isEventQueueEmpty())
        {
            {
                lock_guard<mutex> lock(mtx_data);
                auto data = hubData_subscriber.getEvent();
                gReceivedEvent_count_hub_data++;

                // 수신된 데이터 handling 위한 추가 코드
                adcm::Log::Info() << "수신 데이터: " << data->vehicle_class;

                FusionData fusionData;
                fillVehicleData(fusionData.vehicle, data);
                fillObstacleList(fusionData.obstacle_list, data);

                switch (data->vehicle_class)
                {
                // <25.03.04> 큐 제거
                case EGO_VEHICLE:
                    // main_vehicle_queue.enqueue(fusionData);
                    main_vehicle_data = fusionData;
                    order.push(EGO_VEHICLE);
                    ego = true;
                    break;

                case SUB_VEHICLE_1:
                    // sub1_vehicle_queue.enqueue(fusionData);
                    sub1_vehicle_data = fusionData;
                    order.push(SUB_VEHICLE_1);
                    sub1 = true;
                    break;

                case SUB_VEHICLE_2:
                    // sub2_vehicle_queue.enqueue(fusionData);
                    sub2_vehicle_data = fusionData;
                    order.push(SUB_VEHICLE_2);
                    sub2 = true;
                    break;

                default:
                    adcm::Log::Verbose() << "Unknown vehicle class: " << data->vehicle_class;
                    continue; // 미확인 데이터는 처리하지 않고 다음으로 넘어감
                }
            }
            dataReady.notify_one();
            adcm::Log::Info() << ++receiveVer << "번째 허브 데이터 수신 완료";

            {
                unique_lock<mutex> lock(mtx_send);
                mapSend.wait(lock, []
                             { return !send_wait; });
            }
            // case 255: // 보조차1이 보낸 인지데이터
            //     data->vehicle_class = SUB_VEHICLE_1;
            //     fillVehicleData(sub1_vehicle_temp, data);
            //     fillObstacleList(obstacle_list_temp, data);
            //     fusionData.vehicle = sub1_vehicle_temp;
            //     fusionData.obstacle_list = obstacle_list_temp;
            //     sub1_vehicle_queue.enqueue(fusionData);
            //     order.push(SUB_VEHICLE_1);
            //     adcm::Log::Info() << ++receiveVer << "번째 허브 데이터 수신 완료";
            //     break;
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
    adcm::Log::Info() << "ThreadReceiveWorkInfo start...";

    while (continueExecution)
    {
        if (!workInformation_subscriber.waitEvent(10000))
            continue; // 이벤트가 없다면 루프 다시 실행

        adcm::Log::Info() << "DataFusion Work Information received";

        while (!workInformation_subscriber.isEventQueueEmpty())
        {
            auto data = workInformation_subscriber.getEvent();

            main_vehicle_size.length = data->main_vehicle.length;
            main_vehicle_size.width = data->main_vehicle.width;

            sub_vehicle_size.clear();
            for (const auto &sub_vehicle : data->sub_vehicle)
            {
                sub_vehicle_size.push_back({sub_vehicle.length, sub_vehicle.width});
            }

            work_boundary.clear();
            for (const auto &boundary : data->working_area_boundary)
            {
                work_boundary.push_back({boundary.x, boundary.y});
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
            }

            // 맵 범위 계산
            if (!work_boundary.empty())
            {
                min_a = max_a = work_boundary.front().x;
                min_b = max_b = work_boundary.front().y;
            }

            for (const auto &boundary : work_boundary)
            {
                min_a = std::min(min_a, boundary.x);
                min_b = std::min(min_b, boundary.y);
                max_a = std::max(max_a, boundary.x);
                max_b = std::max(max_b, boundary.y);
            }
            adcm::Log::Info() << "map의 min 값: (" << min_a << ", " << min_b << "), max 값 : (" << max_a << ", " << max_b << ")";

            for (auto &boundary : work_boundary)
            {
                boundary.x -= min_a;
                boundary.y -= min_b;
                adcm::Log::Info() << "변경한 boundary: (" << boundary.x << ", " << boundary.y << ")";
            }

            // haveWorkInfo = true;
            sendEmptyMap = true;
            mapReady.notify_one();
        }
    }
}

// edge_information 수신
void ThreadReceiveEdgeInfo()
{
    adcm::EdgeInformation_Subscriber edgeInformation_subscriber;
    edgeInformation_subscriber.init("DataFusion/DataFusion/RPort_edge_information");
    adcm::Log::Info() << "ThreadReceiveEdgeInfo start...";

    while (continueExecution)
    {
        if (!edgeInformation_subscriber.waitEvent(10000))
        {
            continue; // 이벤트가 없다면 루프 다시 실행
        }

        adcm::Log::Info() << "DataFusion Edge Information received";
        while (!edgeInformation_subscriber.isEventQueueEmpty())
        {
            auto data = edgeInformation_subscriber.getEvent();
            adcm::Log::Info() << "[EdgeInfo] EDGE System State: " << data->state;
        }
    }
}

void ThreadKatech()
{
    adcm::Log::Info() << "ThreadKatech start...";
    NatsStart();
    //==============1.전역변수인 MapData 생성 =================
    IDManager id_manager;
    adcm::Log::Info() << "mapData created for the first time";
    ::adcm::map_2dListStruct map_2dStruct_init;

    map_2dStruct_init.obstacle_id = NO_OBSTACLE;
    map_2dStruct_init.road_z = 0;
    map_2dStruct_init.vehicle_class = NO_VEHICLE; // 시뮬레이션 데이터 설정때문에 부득이 NO_VEHICLE로 바꿈

    // 빈 맵 생성
    std::vector<adcm::map_2dListVector> map_2d_init(map_n, adcm::map_2dListVector(map_m, map_2dStruct_init));
    std::vector<adcm::map_2dListVector> map_2d_test(1, adcm::map_2dListVector(1, map_2dStruct_init));

    mapData.map_2d = map_2d_init;

    std::vector<ObstacleData> obstacle_list;

    adcm::map_2dListVector map_2dListVector;

    while (continueExecution)
    {
        // 수신한 허브 데이터가 없으면 송신 X
        adcm::Log::Info() << "Wait Hub Data";
        {
            unique_lock<mutex> lock(mtx_data);
            dataReady.wait(lock, []
                           { return (ego == true && sub1 == true && sub2 == true); });

            // adcm::Log::Info() << "송신이 필요한 남은 허브 데이터 개수: " << main_vehicle_queue.size_approx() + sub1_vehicle_queue.size_approx() + sub2_vehicle_queue.size_approx();
            std::int8_t map_x = max_a - min_a;
            std::int8_t map_y = max_b - min_b;
            // auto startTime = std::chrono::high_resolution_clock::now();
            adcm::Log::Info() << "==============KATECH modified code start==========";

            adcm::Log::Info() << "KATECH: 이번 데이터 기준 차량: " << order.front();
            //==============1. Data Queue에서 차량 및 장애물 데이터 꺼내면서 위치 변환=================
            /*
            switch (order.front())
            {
            case EGO_VEHICLE:
                // processVehicleData(main_vehicle_queue, main_vehicle_data, main_vehicle, obstacle_list_main);
                // obstacle_list = obstacle_list_main;
                processVehicleData(main_vehicle_data, main_vehicle, obstacle_list_main);
                ego = false;
                break;
            case SUB_VEHICLE_1:
                // processVehicleData(sub1_vehicle_queue, sub1_vehicle_data, sub1_vehicle, obstacle_list_sub1);
                // obstacle_list = obstacle_list_sub1;
                processVehicleData(sub1_vehicle_data, sub1_vehicle, obstacle_list_sub1);
                sub1 = false;
                break;
            case SUB_VEHICLE_2:
                // processVehicleData(sub2_vehicle_queue, sub2_vehicle_data, sub2_vehicle, obstacle_list_sub2);
                // obstacle_list = obstacle_list_sub2;
                processVehicleData(sub2_vehicle_data, sub2_vehicle, obstacle_list_sub2);
                sub2 = false;
                break;
            }
            */
            processVehicleData(main_vehicle_data, main_vehicle, obstacle_list_main);
            processVehicleData(sub1_vehicle_data, sub1_vehicle, obstacle_list_sub1);
            processVehicleData(sub2_vehicle_data, sub2_vehicle, obstacle_list_sub2);
            ego = false;
            sub1 = false;
            sub2 = false;
            adcm::Log::Info() << "차량 및 장애물 좌표계 변환 완료";
        }

        // ==============2. 장애물 데이터 융합 / 3. 특장차 및 보조차량 제거 / 4. 장애물 ID 부여 =================
        obstacle_list = mergeAndCompareLists(previous_obstacle_list, obstacle_list_main, obstacle_list_sub1,
                                             obstacle_list_sub2, main_vehicle, sub1_vehicle, sub2_vehicle);

        order.pop();
        /*
        for (auto obstacle : obstacle_list)
        {
            adcm::Log::Info() << obstacle.obstacle_class << ": [" << obstacle.fused_position_x << ", " << obstacle.fused_position_y << "]";
        }

        adcm::Log::Info() << "장애물 리스트 융합 및 ID 부여 완료";
        adcm::Log::Info() << "장애물 리스트 사이즈: " << obstacle_list.size();
        */
        previous_obstacle_list = obstacle_list;

        // adcm::Log::Info() << "previous_obstacle_list: " << previous_obstacle_list.size();

        // adcm::Log::Info() << "mapData obstacle list size is at start is" << mapData.obstacle_list.size();

        //==============4. 장애물 ID 관리 =================
        /*
        for (auto iter1 = mapData.obstacle_list.begin(); iter1 != mapData.obstacle_list.end(); iter1++)
        {
            // adcm::Log::Info() << "previous obstacle saved in the mapData!" << iter1->obstacle_id;
        }
        std::vector<ObstacleData> obstacle_list_filtered;
        obstacle_list_filtered.clear();

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
                    if ((ori_iter->fused_cuboid_x == new_iter->fused_cuboid_x) && (ori_iter->fused_cuboid_y == new_iter->fused_cuboid_y) && (ori_iter->fused_cuboid_z == new_iter->fused_cuboid_z) && (abs(ori_iter->fused_position_x) - (new_iter->fused_position_x) < 100) && (abs(ori_iter->fused_position_y - new_iter->fused_position_y) < 100) && (abs(ori_iter->fused_position_z - new_iter->fused_position_z) < 100) && (ori_iter->obstacle_class == new_iter->obstacle_class))
                    {
                        adcm::Log::Info() << "fused_position_x: " << ori_iter->fused_position_x << "fused_position_y:" << ori_iter->fused_position_y << "obstacle id" << ori_iter->obstacle_id;
                        adcm::Log::Info() << "new fused_position_x: " << new_iter->fused_position_x << "new fused_position_y:" << new_iter->fused_position_y;
                        adcm::Log::Info() << "Identical obstacle detected : " << ori_iter->obstacle_id;
                        new_iter->obstacle_id = ori_iter->obstacle_id;
                        obstacle_list_filtered.push_back(*new_iter);
                        new_iter = obstacle_list.erase(new_iter);
                        identicalObstacleFound = true;
                        break; // if 문 break
                        // 동일한 장애물이 발견되면 obstacle_list 에서 obstacle_list_filtered 로 옮기고 obstacle list 에서는 삭제한다
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
        */

        // 차량이 맵 범위 내에 있는지 체크
        bool result = checkAllVehicleRange(vehicles);

        /*
        if (result)
        { // execute only if all true!
            //==============5. 0.1 m/s 미만인 경우 장애물 정지 상태 판정 및 stop_count 값 assign =================
            // 수정 예정//
            for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
            {
                if ((abs(iter->fused_velocity_x)) < 0.1 && (abs(iter->fused_velocity_y)) < 0.1)
                {
                    iter->stop_count = 1; // 해당 시각 물체 정지상태
                    // adcm::Log::Info() << "obstacle stopped " << iter->stop_count;
                }
                else
                {
                    iter->stop_count = 0;
                    // adcm::Log::Info() << "obstacle not stopped " << iter->stop_count;
                }
            }

            // 이 전에 장애물의 stop status 를 카운트 하는 카운터 값 변동
            if (!mapData.obstacle_list.empty())
            {
                for (auto iter = mapData.obstacle_list.begin(); iter != mapData.obstacle_list.end(); iter++)
                { // 최초 mapData 아무것도 없으면 for loop 들어가지 않음

                    for (auto iter1 = obstacle_list.begin(); iter1 != obstacle_list.end(); iter1++)
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
            */

        // adcm::Log::Info() << "stop count 변동 완료";
        //==============6. 장애물과 차량의 occupancy 계산해 map_2d_location 값 업데이트 ========

        if (!obstacle_list.empty())
            find4VerticesObstacle(obstacle_list);

        for (const auto &vehicle : vehicles)
        {
            if (vehicle->timestamp != 0)
                find4VerticesVehicle(*vehicle);
        }

        //==============7. 현재까지의 데이터를 adcm mapData 형식으로 재구성해서 업데이트 ================
        //================ adcm mapData 내 obstacle list 업데이트 ===============================

        // adcm::Log::Info() << "mapdata 장애물 반영 예정 개수: " << obstacle_list.size();

        // map_2d에서 map_2d_location이 존재하는 부분만 수정
        // 맵데이터 수정하며 lock걸기

        {
            lock_guard<mutex> map_lock(mtx_map);
            mapData.map_2d = map_2d_init;
            UpdateMapData(mapData, obstacle_list, vehicles);
            // mapData.map_2d = map_2d_test;
            send_map++;
        }

        adcm::Log::Info() << "mapdata 융합 완료";
        mapReady.notify_one();

        // for (int i = 0; i < map_n; ++i)
        // {
        //     map_2dListVector.clear();
        //     for (int j = 0; j < map_m; ++j)
        //     {
        //         map_2dStruct.obstacle_id = map_2d_test[i][j].obstacle_id;
        //         map_2d_test[i][j].obstacle_id = NO_OBSTACLE;

        //         // obstacle_id 초기화

        //         map_2dStruct.vehicle_class = map_2d_test[i][j].vehicle_class;
        //         map_2d_test[i][j].vehicle_class = NO_VEHICLE;

        //         // vehicle_class 초기화
        //         map_2dStruct.road_z = map_2d_test[i][j].road_z;
        //         // road_z 정보는 계속 가져간다
        //         map_2dListVector.push_back(map_2dStruct);
        //     }
        //     mapData.map_2d.push_back(map_2dListVector);
        // }

        // mapData.map_2d.push_back(map_2dListVector);

        // adcm::Log::Info() << "DATA FUSION DONE";
        // adcm::Log::Info() << "map_2d pushed to mapData";

        // mapData_provider.send(mapData);
        // auto endTime = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double, std::milli> duration = endTime - startTime;
        // adcm::Log::Info() << "mapdata 전송에 걸린 시간: " << duration.count() << " ms.";
        // mapData.map_2d.clear(); // json 데이터 경량화를 위해 map_2d 삭제
        // NatsSend(mapData);
        // endTime = std::chrono::high_resolution_clock::now();
        // duration = endTime - startTime;
        // adcm::Log::Info() << "mapdata + NATS 전송에 걸린 시간: " << duration.count() << " ms.";
        // mapData.map_2d = map_2d_init;                                // 맵 초기화
        // std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 대기시간
    }
}

void ThreadSend()
{
    adcm::Log::Info() << "ThreadSend start...";

    int mapVer = 0; // 현재 맵이 몇 번째 맵인지 확인

    adcm::MapData_Provider mapData_provider;
    mapData_provider.init("DataFusion/DataFusion/PPort_map_data");
    // mutex, condition value 사용

    while (continueExecution)
    {
        {
            unique_lock<mutex> lock(mtx_map);
            mapReady.wait(lock, []
                          { return sendEmptyMap == true ||
                                   send_map > 0; });
            if (sendEmptyMap)
            {
                mapData_provider.send(mapData);
                adcm::Log::Info() << "Send empty map data";
                sendEmptyMap = false;
                continue;
            }
            send_wait = true;
            adcm::Log::Info() << ++mapVer << "번째 mapdata 전송 시작";
            // 맵전송
            mapData_provider.send(mapData);
            mapData.map_2d.clear(); // json 데이터 경량화를 위해 map_2d 삭제
            NatsSend(mapData);
            adcm::Log::Info() << mapVer << "번째 mapdata 전송 완료";
            send_map = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        {
            lock_guard<mutex> lock(mtx_send);
            send_wait = false;
        }
        mapSend.notify_one();
    }
}

void ThreadMonitor()
{
    while (continueExecution)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20000));

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
    // 자동 Build Time 생성
    time_t timer;
    struct tm *t;
    timer = time(NULL);
    t = localtime(&timer);

    string year = to_string(t->tm_year - 100);
    string mon = to_string(t->tm_mon + 1);
    string day = to_string(t->tm_mday);
    if (mon.length() == 1)
        mon.insert(0, "0");
    if (day.length() == 1)
        day.insert(0, "0");
    string b_day = year + mon + day;

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
    adcm::Log::Info() << "SDK release_250102_interface v2.0";
    adcm::Log::Info() << "DataFusion Build " << b_day;
#ifdef NATS
    // Code to execute if NATS is defined
    adcm::Log::Info() << "NATS ON";
#else
    // Code to execute if NATS is not defined
    adcm::Log::Info() << "NATS OFF";
#endif
    thread_list.push_back(std::thread(ThreadReceiveHubData));
    thread_list.push_back(std::thread(ThreadReceiveWorkInfo));
    thread_list.push_back(std::thread(ThreadMonitor));
    thread_list.push_back(std::thread(ThreadKatech));
    thread_list.push_back(std::thread(ThreadReceiveEdgeInfo));
    thread_list.push_back(std::thread(ThreadSend));

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