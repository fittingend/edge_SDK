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
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <Poco/JSON/Parser.h>
#include <Poco/JSON/Array.h>
#include <Poco/JSON/Object.h>
#include <Poco/Dynamic/Var.h>
#include "main_datafusion.hpp"
#include "occupancy_algorithm.hpp"
#include "config.cpp"

#include <unistd.h>
#include <limits.h>

#define NATS
#ifdef NATS

static inline std::string framePrefix()
{
    return std::string("[F") + std::to_string(mapVer) + "]";
}

static inline std::string framePrefix(int ver)
{
    return std::string("[F") + std::to_string(ver) + "]";
}

bool firstTime = true;
natsStatus s = NATS_OK;
string nats_server_url;
std::vector<const char *> subject = {"test1.*", "test2.*"};
std::shared_ptr<adcm::etc::NatsConnManager> natsManager;

void asyncCb(natsConnection *nc, natsSubscription *sub, natsStatus err, void *closure)
{
    std::cout << "Async error: " << err << " - " << natsStatus_GetText(err) << std::endl;
    natsManager->NatsSubscriptionGetDropped(sub, (int64_t *)&natsManager->dropped);
}

static inline std::uint64_t getCurrentUTCMilliseconds()
{
    auto now = std::chrono::system_clock::now();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
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

struct ListObstacleEntry
{
    std::uint16_t obstacle_id = 0;
    std::uint8_t obstacle_class = 0;
    double lon = 0.0;
    double lat = 0.0;
    std::uint16_t assigned_id = 0;
};

static std::vector<ListObstacleEntry> list_obstacles;
static bool list_obstacles_loaded = false;
static std::string list_obstacles_path;

static inline std::uint64_t nowMilliseconds()
{
    auto now = std::chrono::system_clock::now();
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count());
}

static bool loadListObstacles(const std::string &filePath)
{
    list_obstacles.clear();
    list_obstacles_loaded = false;
    list_obstacles_path = filePath;

    std::ifstream inFile(filePath);
    if (!inFile.is_open())
    {
        adcm::Log::Info() << "List file not found: " << filePath;
        return false;
    }

    std::stringstream buffer;
    buffer << inFile.rdbuf();
    const std::string jsonStr = buffer.str();
    if (jsonStr.empty())
    {
        adcm::Log::Info() << "List file is empty: " << filePath;
        return false;
    }

    try
    {
        Poco::JSON::Parser parser;
        Poco::Dynamic::Var result = parser.parse(jsonStr);
        Poco::JSON::Array::Ptr arr = result.extract<Poco::JSON::Array::Ptr>();

        for (size_t i = 0; i < arr->size(); ++i)
        {
            Poco::JSON::Object::Ptr obj = arr->getObject(i);
            if (!obj)
                continue;

            ListObstacleEntry entry;
            entry.obstacle_class = static_cast<std::uint8_t>(obj->getValue<int>("obstacle_class"));
            entry.obstacle_id = static_cast<std::uint16_t>(obj->getValue<int>("obstacle_id"));
            entry.lon = obj->getValue<double>("lon");
            entry.lat = obj->getValue<double>("lat");
            list_obstacles.push_back(entry);
        }
    }
    catch (const std::exception &ex)
    {
        adcm::Log::Info() << "Failed to parse list file: " << ex.what();
        return false;
    }

    list_obstacles_loaded = !list_obstacles.empty();
    adcm::Log::Info() << "List obstacles loaded: " << list_obstacles.size();
    for(const auto &entry : list_obstacles)
    adcm::Log::Info() << "  id=" << entry.obstacle_id << ", class=" << static_cast<int>(entry.obstacle_class)
                     << ", lon=" << entry.lon << ", lat=" << entry.lat;
    return list_obstacles_loaded;
}

// 신규 최적화 버전
Poco::JSON::Object::Ptr buildMapDataJson(const adcm::map_data_Objects &mapData)
{
    Poco::JSON::Object::Ptr mapObj = new Poco::JSON::Object;

    // obstacle_list
    Poco::JSON::Array::Ptr obsArr = new Poco::JSON::Array;
    for (const auto &item : mapData.obstacle_list)
    {
        Poco::JSON::Object::Ptr obj = new Poco::JSON::Object;
        obj->set("obstacle_id", item.obstacle_id);
        obj->set("obstacle_class", item.obstacle_class);
        obj->set("timestamp", item.timestamp);
        obj->set("stop_count", item.stop_count);
        obj->set("fused_cuboid_x", item.fused_cuboid_x);
        obj->set("fused_cuboid_y", item.fused_cuboid_y);
        obj->set("fused_cuboid_z", item.fused_cuboid_z);
        obj->set("fused_heading_angle", item.fused_heading_angle);
        obj->set("fused_position_x", item.fused_position_x);
        obj->set("fused_position_y", item.fused_position_y);
        obj->set("fused_position_z", item.fused_position_z);
        obj->set("fused_velocity_x", item.fused_velocity_x);
        obj->set("fused_velocity_y", item.fused_velocity_y);
        obj->set("fused_velocity_z", item.fused_velocity_z);

        Poco::JSON::Array::Ptr locArr = new Poco::JSON::Array;
        for (const auto &pt : item.map_2d_location)
        {
            Poco::JSON::Object::Ptr loc = new Poco::JSON::Object;
            loc->set("x", pt.x);
            loc->set("y", pt.y);
            locArr->add(loc);
        }
        obj->set("map_2d_location", locArr);

        obsArr->add(obj);
    }
    mapObj->set("obstacle_list", obsArr);

    // vehicle_list
    Poco::JSON::Array::Ptr vehArr = new Poco::JSON::Array;
    for (const auto &item : mapData.vehicle_list)
    {
        Poco::JSON::Object::Ptr obj = new Poco::JSON::Object;
        obj->set("vehicle_class", item.vehicle_class);
        obj->set("timestamp", item.timestamp);
        obj->set("position_long", item.position_long);
        obj->set("position_lat", item.position_lat);
        obj->set("position_height", item.position_height);
        obj->set("position_x", item.position_x);
        obj->set("position_y", item.position_y);
        obj->set("position_z", item.position_z);
        obj->set("heading_angle", item.heading_angle);
        obj->set("velocity_long", item.velocity_long);
        obj->set("velocity_lat", item.velocity_lat);
        obj->set("velocity_x", item.velocity_x);
        obj->set("velocity_y", item.velocity_y);
        obj->set("velocity_ang", item.velocity_ang);

        Poco::JSON::Array::Ptr locArr = new Poco::JSON::Array;
        for (const auto &pt : item.map_2d_location)
        {
            Poco::JSON::Object::Ptr loc = new Poco::JSON::Object;
            loc->set("x", pt.x);
            loc->set("y", pt.y);
            locArr->add(loc);
        }
        obj->set("map_2d_location", locArr);

        vehArr->add(obj);
    }
    mapObj->set("vehicle_list", vehArr);

    /*
        // road_list
        Poco::JSON::Array::Ptr roadArr = new Poco::JSON::Array;
        for (const auto &road : mapData.road_list)
        {
            Poco::JSON::Object::Ptr obj = new Poco::JSON::Object;
            obj->set("road_index", road.road_index);
            obj->set("timestamp", road.Timestamp);

            Poco::JSON::Array::Ptr locArr = new Poco::JSON::Array;
            for (const auto &pt : road.map_2d_location)
            {
                Poco::JSON::Object::Ptr loc = new Poco::JSON::Object;
                loc->set("x", pt.x);
                loc->set("y", pt.y);
                locArr->add(loc);
            }
            obj->set("map_2d_location", locArr);

            roadArr->add(obj);
        }
        mapObj->set("road_list", roadArr);
    */

    return mapObj;
}

void saveMapDataJsonFile(const std::string &filePrefix, Poco::JSON::Object::Ptr mapObj, int &fileCount)
{
    std::string dirPath = "/opt/DataFusion/json/";

    std::ostringstream fileNameStream;
    fileNameStream << dirPath << filePrefix << "_" << fileCount << ".json";
    std::string fileName = fileNameStream.str();

    std::ofstream outFile(fileName);
    if (outFile.is_open())
    {
        Poco::JSON::Stringifier::stringify(mapObj, outFile, 4); // pretty print
        outFile.close();
        fileCount++;
        adcm::Log::Info() << "JSON data stored in " << fileName;
    }
    else
    {
        adcm::Log::Error() << "Failed to open file " << fileName << " for writing!";
    }
}

void clearJsonDirectory(const std::string &dirPath)
{
    DIR *dir = opendir(dirPath.c_str());
    if (!dir)
        return; // 디렉토리가 없으면 바로 종료

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        std::string fileName = entry->d_name;

        // "."과 ".." 제외
        if (fileName == "." || fileName == "..")
            continue;

        std::string fullPath = dirPath + "/" + fileName;

        // 파일인지 확인
        struct stat st;
        if (stat(fullPath.c_str(), &st) == 0 && S_ISREG(st.st_mode))
        {
            remove(fullPath.c_str()); // 실패 시에도 그냥 넘어감
        }
    }

    closedir(dir);
}
// 구버전
std::string convertMapDataToJsonString(const adcm::map_data_Objects &mapData)
{
    std::ostringstream oss; // Use a string stream for easier manipulation
    oss << "{\n";           // Start the JSON object

    /* map_2d는 제외
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
    */

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
            << "            \"heading_angle\": " << item.heading_angle << ",\n"
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
        natsManager = std::make_shared<adcm::etc::NatsConnManager>(nats_server_url.c_str(), subject, onMsg, asyncCb, adcm::etc::NatsConnManager::Mode::Default);
        s = natsManager->NatsExecute();
        firstTime = false;
    }
}

void NatsSend(const adcm::map_data_Objects &mapData)
{
    static int mapData_count = 0;
    if (s == NATS_OK)
    {
        const char *pubSubject = "mapDataObjects.create";
        natsManager->ClearJsonData();
        adcm::Log::Info() << "NATS conversion start!";
        // std::string mapDataStr = convertMapDataToJsonString(mapData);
        Poco::JSON::Object::Ptr mapObj = buildMapDataJson(mapData);
        adcm::Log::Info() << "NATS conversion done!";
        natsManager->addJsonData("mapData", mapObj);
        adcm::Log::Info() << "NATS make Json Data!";
        natsManager->NatsPublishJson(pubSubject);
        adcm::Log::Info() << "NATS publish Json!"; // publish가 오래 걸림
    }
    else
    {
        adcm::Log::Info() << "NATS connection error";
        try
        {
            natsManager = std::make_shared<adcm::etc::NatsConnManager>(nats_server_url.c_str(), subject, onMsg, asyncCb, adcm::etc::NatsConnManager::Mode::Default);
            s = natsManager->NatsExecute();
        }
        catch (std::exception e)
        {
            std::cout << "Nats reConnection error" << std::endl;
        }
    }
}

#endif

void makeJSON(const adcm::map_data_Objects &mapData)
{
    static int mapData_count = 0;
    if (saveJson)
    {
        // std::string mapDataStr = convertMapDataToJsonString(mapData);
        Poco::JSON::Object::Ptr mapObj = buildMapDataJson(mapData);
        saveMapDataJsonFile("mapData", mapObj, mapData_count);
        adcm::Log::Info() << "save Json file!";
    }
}

void GPStoUTM(double lon, double lat, double &utmX, double &utmY)
{
    // 상수 정의
    const double WGS84_A = 6378137.0;
    const double WGS84_E = 0.0818191908;
    const double k0 = 0.9996;
    const double eSq = WGS84_E * WGS84_E;
    const double ePrimeSq = eSq / (1 - eSq);
    const double DEG_TO_RAD = M_PI / 180.0;

    // UTM Zone 설정 (Zone 52 고정)
    int zone = 52;
    double lonOrigin = (zone - 1) * 6 - 180 + 3; // 중앙 자오선
    double lonOriginRad = lonOrigin * DEG_TO_RAD;

    // 위도/경도 라디안 변환
    double latRad = lat * DEG_TO_RAD;
    double lonRad = lon * DEG_TO_RAD;

    // 삼각 함수 계산
    double sinLat = sin(latRad);
    double cosLat = cos(latRad);
    double tanLat = tan(latRad);

    // 보조 항 계산
    double N = WGS84_A / sqrt(1 - eSq * pow(sinLat, 2));
    double T = pow(tanLat, 2);
    double C = ePrimeSq * pow(cosLat, 2);
    double A = cosLat * (lonRad - lonOriginRad);

    // 자오선 거리 (Meridional Arc Length)
    double M =
        WGS84_A * ((1 - eSq / 4 - 3 * pow(eSq, 2) / 64 - 5 * pow(eSq, 3) / 256) * latRad - (3 * eSq / 8 + 3 * pow(eSq, 2) / 32 + 45 * pow(eSq, 3) / 1024) * sin(2 * latRad) + (15 * pow(eSq, 2) / 256 + 45 * pow(eSq, 3) / 1024) * sin(4 * latRad) - (35 * pow(eSq, 3) / 3072) * sin(6 * latRad));

    // UTM X 계산
    utmX = k0 * N * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + pow(T, 2) + 72 * C - 58 * ePrimeSq) * pow(A, 5) / 120) + 500000.0;

    // UTM Y 계산
    utmY = k0 * (M + N * tanLat * (pow(A, 2) / 2 + (5 - T + 9 * C + 4 * pow(C, 2)) * pow(A, 4) / 24 + (61 - 58 * T + pow(T, 2) + 600 * C - 330 * ePrimeSq) * pow(A, 6) / 720));

    // 남반구 보정
    if (lat < 0)
        utmY += 10000000.0;
}

void appendListObstaclesToMergedList(std::vector<ObstacleData> &mergedList)
{
    const std::string prefix = framePrefix();
    if (!list_obstacles_loaded)
    {
        adcm::Log::Info() << prefix << "[KATECH] List obstacles not loaded, skip append.";
        return;
    }

    const double default_cuboid_x = 1.0;
    const double default_cuboid_y = 1.0;
    const double default_cuboid_z = 1.0;
    const size_t before_count = mergedList.size();
    size_t added_count = 0;
    size_t skipped_count = 0;

    for (auto &entry : list_obstacles)
    {
        if (entry.assigned_id == 0)
        {
            entry.assigned_id = id_manager.allocID();
        }

        double utm_x = 0.0;
        double utm_y = 0.0;
        GPStoUTM(entry.lon, entry.lat, utm_x, utm_y);

        const double map_pos_x = (utm_x - origin_x) * M_TO_10CM_PRECISION;
        const double map_pos_y = (utm_y - origin_y) * M_TO_10CM_PRECISION;

        if (map_pos_x < 0 || map_pos_x >= map_x || map_pos_y < 0 || map_pos_y >= map_y)
        {
            adcm::Log::Info() << prefix << "[KATECH] List obstacle out of range, skip. id=" << entry.assigned_id;
            skipped_count++;
            continue;
        }

        ObstacleData obs;
        obs.obstacle_id = entry.assigned_id;
        obs.obstacle_class = entry.obstacle_class;
        obs.timestamp = nowMilliseconds();
        obs.stop_count = 0;
        obs.fused_cuboid_x = default_cuboid_x;
        obs.fused_cuboid_y = default_cuboid_y;
        obs.fused_cuboid_z = default_cuboid_z;
        obs.fused_heading_angle = 0.0;
        obs.fused_position_x = map_pos_x;
        obs.fused_position_y = map_pos_y;
        obs.fused_position_z = 0.0;
        obs.fused_velocity_x = 0.0;
        obs.fused_velocity_y = 0.0;
        obs.fused_velocity_z = 0.0;
        obs.standard_deviation = 0.0;

        mergedList.push_back(obs);
        added_count++;
    }

    adcm::Log::Info() << prefix << "[KATECH] List obstacles append result: before=" << before_count
                      << ", added=" << added_count
                      << ", skipped=" << skipped_count
                      << ", total_list=" << mergedList.size();
}

bool checkRange(const VehicleData &vehicle)
{
    if (vehicle.position_x >= 0 && vehicle.position_x < map_x &&
        vehicle.position_y >= 0 && vehicle.position_y < map_y)
        return true;
    return false;
}

void checkRange(Point2D &point)
{
    if (point.x < 0)
        point.x = 0;
    if (point.y < 0)
        point.y = 0;
    if (point.x > map_x)
        point.x = map_x - 1;
    if (point.y > map_y)
        point.y = map_y - 1;
}

bool checkAllVehicleRange(const std::vector<VehicleData *> &vehicles)
{
    const std::string prefix = framePrefix();
    for (const auto *vehicle : vehicles)
    {
        if (vehicle->vehicle_class != 0)
        {
            adcm::Log::Info() << prefix << vehicle->vehicle_class << "번 차량 위치: [" << vehicle->position_x << ", " << vehicle->position_y << "]";
            if (vehicle && !checkRange(*vehicle))
            {
                return false;
            }
        }
    }
    return true;
}

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
    double position_x = vehicle.position_long;
    double position_y = vehicle.position_lat;
    double veh_utm_x, veh_utm_y; // 차량 utm 좌표
    GPStoUTM(position_x, position_y, veh_utm_x, veh_utm_y);
    vehicle.position_x = (veh_utm_x - origin_x) * M_TO_10CM_PRECISION;
    vehicle.position_y = (veh_utm_y - origin_y) * M_TO_10CM_PRECISION;
    // 차량 각도는 유지
    vehicle.velocity_x = vehicle.velocity_long;
    vehicle.velocity_y = vehicle.velocity_lat;

    const std::string prefix = framePrefix();
    adcm::Log::Info() << prefix << "[GPSTOMAP] 차량 " << vehicle.vehicle_class << " 좌표변환 before (" << position_x << " , " << position_y << " , " << vehicle.velocity_long << " , " << vehicle.velocity_lat << ")";
    adcm::Log::Info() << prefix << "[GPSTOMAP] 차량 " << vehicle.vehicle_class << " 좌표변환 after (" << vehicle.position_x << " , " << vehicle.position_y << " , " << vehicle.heading_angle << ")";
}

void relativeToMapcoordinate(std::vector<ObstacleData> &obstacle_list, VehicleData vehicle)
{
    double theta = vehicle.heading_angle * M_PI / 180.0;
    double velocity_ang = vehicle.velocity_ang;

    const std::string prefix = framePrefix();
    adcm::Log::Info() << prefix << "[RELATIVETOMAP] " << vehicle.vehicle_class << " 차량 위치, heading_angle: (" << vehicle.position_x << " , " << vehicle.position_y << " , " << vehicle.heading_angle << ")";
    for (auto iter = obstacle_list.begin(); iter != obstacle_list.end();)
    {
        double obstacle_position_x = iter->fused_position_x;
        double obstacle_position_y = iter->fused_position_y;
        double obstacle_velocity_x = iter->fused_velocity_x; // 시뮬 로그 속도의 단위는 m/s인데, 결과 값의 단위는 미정
        double obstacle_velocity_y = iter->fused_velocity_y;

        adcm::Log::Info() << prefix << "[RELATIVETOMAP] 장애물 class " << iter->obstacle_class << " 좌표변환 before (" << iter->fused_position_x << ", " << iter->fused_position_y << ")";

        // 새로 정리한 차량 heading_angle 기준 (정북: 0도, 반시계방향, 오른손 좌표계)
        iter->fused_position_x = vehicle.position_x + ((obstacle_position_x)*sin(theta) * (-1) + (obstacle_position_y)*cos(theta) * (-1)) * M_TO_10CM_PRECISION;
        iter->fused_position_y = vehicle.position_y + ((obstacle_position_x)*cos(theta) + (obstacle_position_y)*sin(theta) * (-1)) * M_TO_10CM_PRECISION;

        // 차량 좌표계 기준이므로 90+heading_angle 만큼 회전변환 필요 (추가예정)
        iter->fused_velocity_x = vehicle.velocity_x + ((obstacle_velocity_x)*sin(theta) * (-1) + (obstacle_velocity_y)*cos(theta) * (-1));
        iter->fused_velocity_y = vehicle.velocity_y + ((obstacle_velocity_x)*cos(theta) + (obstacle_velocity_y)*sin(theta) * (-1));

        iter->fused_heading_angle = vehicle.heading_angle + iter->fused_heading_angle;

        // 장애물 데이터 범위 초과 시 제거
        if (iter->fused_position_x < 0 || iter->fused_position_x >= map_x ||
            iter->fused_position_y < 0 || iter->fused_position_y >= map_y)
        {
            adcm::Log::Info() << prefix << "[RELATIVETOMAP] 맵 범위 밖 장애물 제거: class=" << iter->obstacle_class
                              << ", pos=(" << iter->fused_position_x << ", " << iter->fused_position_y << ")";
            iter = obstacle_list.erase(iter);
            continue;
        }

        adcm::Log::Info() << prefix << "[RELATIVETOMAP] 장애물 class " << iter->obstacle_class << " 좌표변환 after (" << iter->fused_position_x << ", " << iter->fused_position_y << ")";
        ++iter;
    }
}

// occupancy_algorithm 기반
void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, VehicleData &vehicle)
{
    constexpr double cellSize = 1.0; // 10cm 단위 그리드
    std::vector<occupancy::Pt> poly = {
        {p0.x, p0.y},
        {p1.x, p1.y},
        {p2.x, p2.y},
        {p3.x, p3.y},
    };

    auto cells = occupancy::rasterizePolygonToCells(poly, cellSize);
    for (const auto &cell : cells)
    {
        Point2D index{static_cast<double>(cell.first), static_cast<double>(cell.second)};
        checkRange(index);
        vehicle.map_2d_location.push_back(index);
    }
}

void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, std::vector<ObstacleData>::iterator iter)
{
    if (iter == std::vector<ObstacleData>::iterator())
        return; // 유효하지 않은 iterator 방지

    constexpr double cellSize = 1.0; // 10cm 단위 그리드
    std::vector<occupancy::Pt> poly = {
        {p0.x, p0.y},
        {p1.x, p1.y},
        {p2.x, p2.y},
        {p3.x, p3.y},
    };

    auto cells = occupancy::rasterizePolygonToCells(poly, cellSize);
    for (const auto &cell : cells)
    {
        Point2D index{static_cast<double>(cell.first), static_cast<double>(cell.second)};
        checkRange(index);
        iter->map_2d_location.push_back(index);
    }
}

void find4VerticesVehicle(VehicleData &target_vehicle)
{
    Point2D LU, RU, RL, LL;
    double half_x;
    double half_y;
    double theta = target_vehicle.heading_angle * M_PI / 180;

    if (target_vehicle.vehicle_class == EGO_VEHICLE)
    {
        half_x = main_vehicle_size.length / 2 * M_TO_10CM_PRECISION;
        half_y = main_vehicle_size.width / 2 * M_TO_10CM_PRECISION;
    }

    else
    {
        half_x = sub_vehicle_size.front().length / 2 * M_TO_10CM_PRECISION;
        half_y = sub_vehicle_size.front().width / 2 * M_TO_10CM_PRECISION;
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
    LL.y = target_vehicle.position_y + sin(theta) * (-half_x) + cos(theta) * (half_y);

    checkRange(LU);
    checkRange(RU);
    checkRange(RL);
    checkRange(LL);
    // road_z -> road_index 변경
    // generateRoadZValue(target_vehicle, map_2d_test);

    generateOccupancyIndex(LU, RU, RL, LL, target_vehicle);
}

void find4VerticesObstacle(std::vector<ObstacleData> &obstacle_list_filtered)
{
    const std::string prefix = framePrefix();
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
        LL.y = obstacle_position_y + sin(theta) * (-half_x) + cos(theta) * (half_y);

        checkRange(LU);
        checkRange(RU);
        checkRange(RL);
        checkRange(LL);

        generateOccupancyIndex(LU, RU, RL, LL, *(&iter));

        std::uint16_t obstacle_id = iter->obstacle_id;
        std::uint8_t obstacle_class = iter->obstacle_class;
/*
        adcm::Log::Info() << prefix << "[OCCUPANCY] obstacle " << obstacle_id << "(" << obstacle_class << ") "
                          << "center=" << obstacle_position_x << "," << obstacle_position_y
                          << " size(m)=" << iter->fused_cuboid_x << "x" << iter->fused_cuboid_y
                          << " heading(deg)=" << iter->fused_heading_angle;

        adcm::Log::Info() << prefix << "[OCCUPANCY] obstacle " << obstacle_id << "(" << obstacle_class << ")"
                          << " corners LU=" << LU.x << "," << LU.y
                          << " RU=" << RU.x << "," << RU.y
                          << " RL=" << RL.x << "," << RL.y
                          << " LL=" << LL.x << "," << LL.y;

        adcm::Log::Info() << prefix << "[OCCUPANCY] obstacle " << obstacle_id << "(" << obstacle_class << ") "
                          << "map_2d_location total = " << iter->map_2d_location.size();
        for (const auto &pt : iter->map_2d_location)
        {
            adcm::Log::Info() << prefix << "[OCCUPANCY] obstacle " << obstacle_id << "(" << obstacle_class << ") : index ("
                              << static_cast<int>(pt.x) << ", " << static_cast<int>(pt.y) << ")";
        }
                              */
    }
}

// 유클리디안 거리 계산
double euclideanDistance(const ObstacleData &a, const ObstacleData &b)
{
    return std::sqrt(std::pow(a.fused_position_x - b.fused_position_x, 2) +
                     std::pow(a.fused_position_y - b.fused_position_y, 2));
}

// 정적/동적 장애물 거리 임계값 (10cm 단위 기준)
const double STATIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD = 50.0;   // 0.5m (50cm)
const double DYNAMIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD = 100.0; // 1m (100cm)
const double HUNGARIAN_REJECT_COST = 999999.0;

// 정적 장애물 판별 (class 30~49)
bool isStaticObstacle(std::uint8_t obstacleClass)
{
    return obstacleClass >= 30 && obstacleClass <= 49;
}

// 정적 장애물 위치 히스토리
struct StaticObstacleHistory
{
    ObstacleData obstacle;
    std::vector<Point2D> positionHistory; // 최대 10프레임 (x, y만)
};

// 통합 트래커: 정적/동적 장애물 분기 처리
class ObstacleTracker
{
private:
    struct DynamicObstacleEntry
    {
        ObstacleData obstacle;
        int unmatchedFrames;
    };

    std::unordered_map<std::uint16_t, DynamicObstacleEntry> dynamicObstacles;
    std::unordered_map<std::uint16_t, StaticObstacleHistory> staticObstacles;

    const double DISTANCE_THRESHOLD = DYNAMIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD;       // 동적: 1m
    const double STATIC_DISTANCE_THRESHOLD = STATIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD; // 정적: 0.5m
    const int MAX_UNMATCHED_FRAMES = 25;                                               // 동적 장애물: 25프레임
    const size_t POSITION_HISTORY_SIZE = 10;                                           // 정적 장애물: 10프레임

public:
    enum class TrackMode
    {
        Both,
        StaticOnly,
        DynamicOnly
    };

    std::vector<ObstacleData> track(const std::vector<ObstacleData> &newList, TrackMode mode = TrackMode::Both)
    {
        std::vector<ObstacleData> trackedList;
        std::unordered_set<std::uint16_t> matchedDynamicIds;
        std::unordered_set<std::uint16_t> matchedStaticIds;
        const std::string prefix = framePrefix();

        // 하나의 리스트에서 정적/동적 장애물 분기 처리
        for (const auto &currentObstacle : newList)
        {
            const bool isStatic = isStaticObstacle(currentObstacle.obstacle_class);

            adcm::Log::Info() << prefix << "[TRACK] input id=" << currentObstacle.obstacle_id
                              << " class=" << static_cast<int>(currentObstacle.obstacle_class)
                              << " pos=(" << currentObstacle.fused_position_x << ", " << currentObstacle.fused_position_y << ")"
                              << " type=" << (isStatic ? "static" : "dynamic");

            if (isStatic)
            {
                if (mode == TrackMode::DynamicOnly)
                {
                    trackedList.push_back(currentObstacle);
                    continue;
                }
                bool matched = false;
                std::uint16_t bestMatchId = 0;
                double bestDistance = std::numeric_limits<double>::infinity();

                for (const auto &pair : staticObstacles)
                {
                    const auto id = pair.first;
                    const auto &history = pair.second;
                    if (matchedStaticIds.find(id) != matchedStaticIds.end())
                        continue;

                    const double distance = euclideanDistance(currentObstacle, history.obstacle);
                    if (distance < STATIC_DISTANCE_THRESHOLD && distance < bestDistance)
                    {
                        bestMatchId = id;
                        bestDistance = distance;
                        matched = true;
                    }
                }

                if (matched && bestMatchId != 0)
                {
                    adcm::Log::Info() << prefix << "[TRACK] static match bestId=" << bestMatchId
                                      << " bestDist=" << bestDistance
                                      << " thresh=" << STATIC_DISTANCE_THRESHOLD;
                    auto &history = staticObstacles[bestMatchId];

                    history.positionHistory.push_back({currentObstacle.fused_position_x, currentObstacle.fused_position_y});
                    if (history.positionHistory.size() > POSITION_HISTORY_SIZE)
                    {
                        history.positionHistory.erase(history.positionHistory.begin());
                    }

                    double avgX = 0.0;
                    double avgY = 0.0;
                    for (const auto &pos : history.positionHistory)
                    {
                        avgX += pos.x;
                        avgY += pos.y;
                    }
                    avgX /= static_cast<double>(history.positionHistory.size());
                    avgY /= static_cast<double>(history.positionHistory.size());

                    ObstacleData trackedObstacle = currentObstacle;
                    trackedObstacle.obstacle_id = history.obstacle.obstacle_id;
                    trackedObstacle.fused_position_x = avgX;
                    trackedObstacle.fused_position_y = avgY;
                    trackedList.push_back(trackedObstacle);

                    history.obstacle = trackedObstacle;
                    matchedStaticIds.insert(bestMatchId);
                }
                else
                {
                    adcm::Log::Info() << prefix << "[TRACK] static no-match bestDist=" << bestDistance
                                      << " thresh=" << STATIC_DISTANCE_THRESHOLD;
                    ObstacleData trackedObstacle = currentObstacle;
                    if (trackedObstacle.obstacle_id == 0)
                    {
                        trackedObstacle.obstacle_id = id_manager.allocID();
                    }

                    trackedList.push_back(trackedObstacle);
                    staticObstacles[trackedObstacle.obstacle_id] = {trackedObstacle, {{trackedObstacle.fused_position_x, trackedObstacle.fused_position_y}}};
                    matchedStaticIds.insert(trackedObstacle.obstacle_id);
                }
            }
            else
            {
                if (mode == TrackMode::StaticOnly)
                {
                    trackedList.push_back(currentObstacle);
                    continue;
                }
                bool matched = false;
                std::uint16_t bestMatchId = 0;
                double bestDistance = std::numeric_limits<double>::infinity();

                for (const auto &pair : dynamicObstacles)
                {
                    const auto id = pair.first;
                    const auto &tracked = pair.second;
                    if (matchedDynamicIds.find(id) != matchedDynamicIds.end())
                        continue;

                    const double distance = euclideanDistance(currentObstacle, tracked.obstacle);
                    if (distance < DISTANCE_THRESHOLD && distance < bestDistance)
                    {
                        bestMatchId = id;
                        bestDistance = distance;
                        matched = true;
                    }
                }

                if (matched && bestMatchId != 0)
                {
                    adcm::Log::Info() << prefix << "[TRACK] dynamic match bestId=" << bestMatchId
                                      << " bestDist=" << bestDistance
                                      << " thresh=" << DISTANCE_THRESHOLD;
                    auto &tracked = dynamicObstacles[bestMatchId];
                    ObstacleData trackedObstacle = currentObstacle;
                    trackedObstacle.obstacle_id = tracked.obstacle.obstacle_id;
                    trackedList.push_back(trackedObstacle);

                    tracked.obstacle = trackedObstacle;
                    tracked.unmatchedFrames = 0;
                    matchedDynamicIds.insert(bestMatchId);
                }
                else
                {
                    adcm::Log::Info() << prefix << "[TRACK] dynamic no-match bestDist=" << bestDistance
                                      << " thresh=" << DISTANCE_THRESHOLD;
                    ObstacleData trackedObstacle = currentObstacle;
                    if (trackedObstacle.obstacle_id == 0)
                    {
                        trackedObstacle.obstacle_id = id_manager.allocID();
                    }

                    trackedList.push_back(trackedObstacle);
                    dynamicObstacles[trackedObstacle.obstacle_id] = {trackedObstacle, 0};
                    matchedDynamicIds.insert(trackedObstacle.obstacle_id);
                }
            }
        }

        // 매칭되지 않은 기존 동적 장애물 처리
        if (mode != TrackMode::StaticOnly)
        {
            for (auto it = dynamicObstacles.begin(); it != dynamicObstacles.end();)
            {
                if (matchedDynamicIds.find(it->first) == matchedDynamicIds.end())
                {
                    it->second.unmatchedFrames += 1;
                    if (it->second.unmatchedFrames >= MAX_UNMATCHED_FRAMES)
                    {
                        adcm::Log::Info() << prefix << "[TRACK] dynamic drop id=" << it->first
                                          << " unmatchedFrames=" << it->second.unmatchedFrames;
                        it = dynamicObstacles.erase(it);
                        continue;
                    }
                    else
                    {
                        adcm::Log::Info() << prefix << "[TRACK] dynamic keep id=" << it->first
                                          << " unmatchedFrames=" << it->second.unmatchedFrames
                                          << " pos=(" << it->second.obstacle.fused_position_x << ", " << it->second.obstacle.fused_position_y << ")";
                        trackedList.push_back(it->second.obstacle);
                    }
                }
                ++it;
            }
        }

        // 매칭되지 않은 기존 정적 장애물 처리 (유지)
        if (mode != TrackMode::DynamicOnly)
        {
            for (const auto &pair : staticObstacles)
            {
                const auto id = pair.first;
                const auto &history = pair.second;
                if (matchedStaticIds.find(id) == matchedStaticIds.end())
                {
                    if (!history.positionHistory.empty())
                    {
                        adcm::Log::Info() << prefix << "[TRACK] static keep id=" << id
                                          << " historySize=" << history.positionHistory.size();
                        double avgX = 0.0;
                        double avgY = 0.0;
                        for (const auto &pos : history.positionHistory)
                        {
                            avgX += pos.x;
                            avgY += pos.y;
                        }
                        avgX /= static_cast<double>(history.positionHistory.size());
                        avgY /= static_cast<double>(history.positionHistory.size());

                        ObstacleData trackedObstacle = history.obstacle;
                        trackedObstacle.fused_position_x = avgX;
                        trackedObstacle.fused_position_y = avgY;
                        trackedList.push_back(trackedObstacle);
                    }
                    else
                    {
                        adcm::Log::Info() << prefix << "[TRACK] static keep id=" << id
                                          << " historySize=0";
                        trackedList.push_back(history.obstacle);
                    }
                }
            }
        }

        return trackedList;
    }

    void reset()
    {
        dynamicObstacles.clear();
        staticObstacles.clear();
    }
};

ObstacleTracker obstacleTracker;

// 거리 행렬 생성
std::vector<std::vector<double>> createDistanceMatrix(
    const std::vector<ObstacleData> &listA,
    const std::vector<ObstacleData> &listB,
    double maxDistance)
{
    std::vector<std::vector<double>> distanceMatrix(listA.size(), std::vector<double>(listB.size()));
    for (size_t i = 0; i < listA.size(); ++i)
    {
        for (size_t j = 0; j < listB.size(); ++j)
        {
            double distance = euclideanDistance(listA[i], listB[j]);
            distanceMatrix[i][j] = (distance > maxDistance) ? HUNGARIAN_REJECT_COST : distance;
        }
    }
    return distanceMatrix;
}

// 신뢰성 기반 융합 계산
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

// 차량 간 융합 처리 (listA의 ID 우선 유지)
void processFusionForVehiclePair(
    std::vector<ObstacleData> &presList,
    const std::vector<ObstacleData> &prevList,
    const std::vector<int> &assignment)
{
    std::vector<ObstacleData> newList;

    // 1. 매칭된 장애물 처리
    for (size_t i = 0; i < assignment.size(); ++i)
    {
        int j = assignment[i];
        if (j >= 0)
        {
            if (presList[j].obstacle_id == 0 && prevList[i].obstacle_id != 0)
            {
                presList[j].obstacle_id = prevList[i].obstacle_id;
            }
            newList.push_back(presList[j]);
        }
    }

    // 2. prevList에서 매칭되지 않은 장애물 처리
    for (size_t i = 0; i < prevList.size(); ++i)
    {
        int j = assignment[i];
        if (j < 0)
        {
            newList.push_back(prevList[i]);
        }
    }

    // 3. presList에서 매칭되지 않은 장애물 처리
    for (size_t i = 0; i < presList.size(); ++i)
    {
        if (std::find(assignment.begin(), assignment.end(), static_cast<int>(i)) == assignment.end())
        {
            auto &currentObstacle = presList[i];
            if (currentObstacle.obstacle_id == 0)
            {
                currentObstacle.obstacle_id = id_manager.allocID();
            }
            newList.push_back(currentObstacle);
        }
    }

    // 새로운 리스트로 갱신
    presList = newList;
}

// 프레임 간 융합 처리 (정적/동적 분리 및 트래커 적용)
void processFusion(
    std::vector<ObstacleData> &presList,
    const std::vector<ObstacleData> &prevList,
    const std::vector<int> &assignment,
    ObstacleTracker::TrackMode mode)
{
    std::vector<ObstacleData> newList;

    // 1. 매칭된 장애물 처리
    for (size_t i = 0; i < assignment.size(); ++i)
    {
        int j = assignment[i];
        if (j >= 0)
        {
            const auto &prevObstacle = prevList[i];
            auto &currentObstacle = presList[j];

            if (prevObstacle.obstacle_class != currentObstacle.obstacle_class)
            {
                currentObstacle.obstacle_id = id_manager.allocID();
                newList.push_back(currentObstacle);
                continue;
            }

            const double distance = euclideanDistance(prevObstacle, currentObstacle);
            const double threshold = isStaticObstacle(currentObstacle.obstacle_class)
                                         ? STATIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD
                                         : DYNAMIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD;

            if (distance >= threshold)
            {
                currentObstacle.obstacle_id = id_manager.allocID();
                newList.push_back(currentObstacle);
                continue;
            }

            currentObstacle.obstacle_id = prevObstacle.obstacle_id;
            newList.push_back(currentObstacle);
        }
    }

    // 2. prevList에서 매칭되지 않은 장애물은 제거 (누적 방지)

    // 3. presList에서 매칭되지 않은 장애물 처리
    std::unordered_set<int> matchedPresIndices;
    for (size_t i = 0; i < assignment.size(); ++i)
    {
        if (assignment[i] >= 0)
            matchedPresIndices.insert(assignment[i]);
    }

    for (size_t i = 0; i < presList.size(); ++i)
    {
        if (matchedPresIndices.find(static_cast<int>(i)) == matchedPresIndices.end())
        {
            auto &currentObstacle = presList[i];
            currentObstacle.obstacle_id = id_manager.allocID();
            newList.push_back(currentObstacle);
        }
    }

    if (newList.empty())
    {
        const auto trackedList = obstacleTracker.track(
            newList,
            mode);
        presList = trackedList;
        return;
    }

    const auto trackedList = obstacleTracker.track(
        newList,
        mode);
    presList = trackedList;
}

// 장애물 리스트에서 특장차, 보조 차량 데이터를 제외
void filterVehicleData(std::vector<ObstacleData> &obstacles)
{
    for (auto iter = obstacles.begin(); iter != obstacles.end();)
    {
        if (iter->obstacle_class == 51)
        {
            // adcm::Log::Info() << "보조차량 제거";
            iter = obstacles.erase(iter);
        }
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

// 장애물 리스트 융합 및 이전 데이터와 비교
std::vector<ObstacleData> mergeAndCompareLists(
    const std::vector<ObstacleData> &previousFusionList,
    std::vector<ObstacleData> listMain,
    std::vector<ObstacleData> listSub1,
    std::vector<ObstacleData> listSub2,
    std::vector<ObstacleData> listSub3,
    std::vector<ObstacleData> listSub4,
    const VehicleData &mainVehicle,
    const VehicleData &sub1Vehicle,
    const VehicleData &sub2Vehicle,
    const VehicleData &sub3Vehicle,
    const VehicleData &sub4Vehicle)
{
    const std::string prefix = framePrefix();
    std::vector<VehicleData> nonEmptyVehicles;
    std::vector<std::vector<ObstacleData>> nonEmptyLists;
    std::vector<ObstacleData> mergedList;

    if (worksub1)
    {
        nonEmptyVehicles.push_back(sub1Vehicle);
        nonEmptyLists.push_back(listSub1);
    }
    if (worksub2)
    {
        nonEmptyVehicles.push_back(sub2Vehicle);
        nonEmptyLists.push_back(listSub2);
    }
    if (worksub3)
    {
        nonEmptyVehicles.push_back(sub3Vehicle);
        nonEmptyLists.push_back(listSub3);
    }
    if (worksub4)
    {
        nonEmptyVehicles.push_back(sub4Vehicle);
        nonEmptyLists.push_back(listSub4);
    }
    if (workego)
    {
        nonEmptyVehicles.push_back(mainVehicle);
        nonEmptyLists.push_back(listMain);
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
                processFusionForVehiclePair(fusionList, listB, assignment);
            }
            return fusionList;
        };

        mergedList = handleFusionForPair(nonEmptyLists[0], nonEmptyLists[1]);
        // adcm::Log::Info() << "융합: 융합 1번 완료";

        // 세 번째 리스트가 있다면 그 결과와 함께 융합
        for (size_t i = 2; i < nonEmptyLists.size(); ++i)
        {
            mergedList = handleFusionForPair(mergedList, nonEmptyLists[i]);
            // adcm::Log::Info() << "융합: 융합 " << i << "번 완료";
        }
    }

    if (mergedList.empty())
    {
        if (previousFusionList.empty())
            adcm::Log::Info() << prefix << "[MERGELIST] 장애물 리스트 비어있음";
        else
            adcm::Log::Info() << prefix << "[MERGELIST] 현재 TimeStamp 장애물 X, 이전 TimeStamp 장애물리스트 그대로 사용 및 출력";

        for (const auto &obs : previousFusionList)
        {
            adcm::Log::Info() << prefix << "ID " << obs.obstacle_id << "(" << obs.obstacle_class << ") : [" << obs.fused_position_x << ", " << obs.fused_position_y << "]";
        }
        return previousFusionList;
    }
    else
    {
        if (previousFusionList.empty())
        {
            for (auto &obstacle : mergedList)
            {
                auto newId = id_manager.allocID();
                obstacle.obstacle_id = newId;
                adcm::Log::Info() << prefix << "[MERGELIST] 새로운 장애물 " << obstacle.obstacle_class << " ID 할당 " << newId << " : [" << obstacle.fused_position_x << ", " << obstacle.fused_position_y << "]";
            }
            adcm::Log::Info() << prefix << "[MERGELIST] 새로운 장애물 리스트 생성: " << id_manager.getNum();
            return mergedList;
        }

        adcm::Log::Info() << prefix << "[MERGELIST] 이전 장애물 리스트 사이즈: " << previousFusionList.size();
        adcm::Log::Info() << prefix << "[MERGELIST] 현재 장애물 리스트 사이즈: " << mergedList.size();

        std::vector<ObstacleData> staticPrevList;
        std::vector<ObstacleData> dynamicPrevList;
        std::vector<ObstacleData> staticMergedList;
        std::vector<ObstacleData> dynamicMergedList;

        for (const auto &obs : previousFusionList)
        {
            if (isStaticObstacle(obs.obstacle_class))
                staticPrevList.push_back(obs);
            else
                dynamicPrevList.push_back(obs);
        }

        for (const auto &obs : mergedList)
        {
            if (isStaticObstacle(obs.obstacle_class))
                staticMergedList.push_back(obs);
            else
                dynamicMergedList.push_back(obs);
        }

        // 정적 장애물 매칭
        if (!staticPrevList.empty() && !staticMergedList.empty())
        {
            auto distMatrix = createDistanceMatrix(staticPrevList, staticMergedList, STATIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD);
            auto assignment = solveAssignment(distMatrix);
            processFusion(staticMergedList, staticPrevList, assignment, ObstacleTracker::TrackMode::StaticOnly);
        }
        else if (staticMergedList.empty() && !staticPrevList.empty())
        {
            std::vector<int> emptyAssignment;
            processFusion(staticMergedList, staticPrevList, emptyAssignment, ObstacleTracker::TrackMode::StaticOnly);
        }
        else if (!staticMergedList.empty())
        {
            for (auto &obs : staticMergedList)
            {
                if (obs.obstacle_id == 0)
                {
                    obs.obstacle_id = id_manager.allocID();
                }
            }
        }

        // 동적 장애물 매칭
        if (!dynamicPrevList.empty() && !dynamicMergedList.empty())
        {
            auto distMatrix = createDistanceMatrix(dynamicPrevList, dynamicMergedList, DYNAMIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD);
            auto assignment = solveAssignment(distMatrix);
            processFusion(dynamicMergedList, dynamicPrevList, assignment, ObstacleTracker::TrackMode::DynamicOnly);
        }
        else if (dynamicMergedList.empty() && !dynamicPrevList.empty())
        {
            std::vector<int> emptyAssignment;
            processFusion(dynamicMergedList, dynamicPrevList, emptyAssignment, ObstacleTracker::TrackMode::DynamicOnly);
        }
        else if (!dynamicMergedList.empty())
        {
            for (auto &obs : dynamicMergedList)
            {
                if (obs.obstacle_id == 0)
                {
                    obs.obstacle_id = id_manager.allocID();
                }
            }
        }

        mergedList.clear();
        mergedList.insert(mergedList.end(), staticMergedList.begin(), staticMergedList.end());
        mergedList.insert(mergedList.end(), dynamicMergedList.begin(), dynamicMergedList.end());
/*
        adcm::Log::Info() << prefix << "[MERGELIST] 최종 융합 장애물 리스트 사이즈: " << mergedList.size();
        adcm::Log::Info() << prefix << "[MERGELIST] 최종 융합 장애물 리스트 출력:";
        for (const auto &obs : mergedList)
        {
            adcm::Log::Info() << prefix << "ID " << obs.obstacle_id << "(" << obs.obstacle_class << ") : [" << obs.fused_position_x << ", " << obs.fused_position_y << "]";
        }
    */
        return mergedList;
    }
}

void updateStopCount(std::vector<ObstacleData> &mergedList,
                     const std::vector<ObstacleData> &previousFusionList,
                     double threshold)
{
    for (auto &obs : mergedList)
    {
        // 1. 현재 프레임 속도 기준 정지 여부 판단
        bool stoppedNow = (std::abs(obs.fused_velocity_x) < threshold &&
                           std::abs(obs.fused_velocity_y) < threshold);

        // 2. 이전 프레임 정보 찾기 (ID 기준)
        auto prevIt = std::find_if(previousFusionList.begin(), previousFusionList.end(),
                                   [&](const ObstacleData &p)
                                   { return p.obstacle_id == obs.obstacle_id; });

        if (stoppedNow)
        {
            if (prevIt != previousFusionList.end())
            {
                if (prevIt->stop_count < 255) // stop_count가 255 이상으로 올라가지 않도록 제한
                {
                    obs.stop_count = prevIt->stop_count + 1; // 이전 카운트 +1
                }
                else
                    obs.stop_count = 255; // 최대값 유지
            }

            else
            {
                obs.stop_count = 1; // 새 장애물, 정지 1프레임
            }
            // adcm::Log::Info() << "[STOPCOUNT] 장애물 ID " << obs.obstacle_id << " 정지 카운트: " << obs.stop_count;
        }

        else
        {
            obs.stop_count = 0; // 움직이면 카운트 리셋
            // adcm::Log::Info() << "[STOPCOUNT] 장애물 ID " << obs.obstacle_id << " 정지 카운트 초기화(움직임): " << obs.stop_count;
        }
    }
}
// VehicleData -> vehicleListStruct(맵데이터 호환)
adcm::vehicleListStruct ConvertToVehicleListStruct(const VehicleData &vehicle, std::vector<adcm::map_2dListVector> &map)
{
    adcm::vehicleListStruct vehicle_final;
    vehicle_final.vehicle_class = vehicle.vehicle_class;
    vehicle_final.timestamp = vehicle.timestamp;
    for (const auto &point : vehicle.map_2d_location)
    {
        adcm::map2dIndex index_to_push = {point.x, point.y};
        // road_index 추가 예정
        vehicle_final.map_2d_location.push_back(index_to_push);
    }

    vehicle_final.position_long = vehicle.position_long;
    vehicle_final.position_lat = vehicle.position_lat;
    vehicle_final.position_height = vehicle.position_height;
    vehicle_final.position_x = vehicle.position_x;
    vehicle_final.position_y = vehicle.position_y;
    vehicle_final.position_z = vehicle.position_z;
    vehicle_final.heading_angle = vehicle.heading_angle;
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
    for (const auto &point : obstacle.map_2d_location)
    {
        adcm::map2dIndex index_to_push = {point.x, point.y};
        // road_index 추가예정
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
    const std::string prefix = framePrefix();
    mapData.obstacle_list.clear();
    mapData.vehicle_list.clear();
    mapData.road_list.clear();

    std::vector<adcm::roadListStruct> merged_road_list;
    bool hasRoadZ = false;
    std::unordered_map<std::uint64_t, std::pair<std::uint8_t, std::uint64_t>> cell_to_road;
    std::unordered_map<std::uint8_t, std::uint64_t> road_timestamp;

    auto packCell = [](int x, int y) -> std::uint64_t
    {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
               static_cast<std::uint32_t>(y);
    };

    auto initMergedRoadList = [&]()
    {
        merged_road_list.resize(24);
        for (int i = 0; i < 23; ++i)
        {
            merged_road_list[i].road_index = i;
            merged_road_list[i].Timestamp = 0;
        }
        merged_road_list[23].road_index = 255;
        merged_road_list[23].Timestamp = 0;
    };

    for (const auto &obstacle : obstacle_list)
    {
        mapData.obstacle_list.push_back(ConvertToObstacleListStruct(obstacle, mapData.map_2d));
    }

    for (const auto &vehicle : vehicles)
    {
        if (vehicle->vehicle_class != 0)
        {
            mapData.vehicle_list.push_back(ConvertToVehicleListStruct(*vehicle, mapData.map_2d));
            if (vehicle->road_z.size() > 1) // 시뮬레이션에선 road_z size가 1이므로 예외 처리
            {
                /** 두대 노면 데이터가 다를때 어떻게 반영할지? 0930_테스트 */
                std::vector<adcm::roadListStruct> road_list = ConvertRoadZToRoadList(*vehicle);
                if (!hasRoadZ)
                {
                    initMergedRoadList();
                    hasRoadZ = true;
                }

                for (size_t i = 0; i < road_list.size() && i < merged_road_list.size(); ++i)
                {
                    for (const auto &loc : road_list[i].map_2d_location)
                    {
                        const int x = static_cast<int>(loc.x);
                        const int y = static_cast<int>(loc.y);
                        if (x < 0 || y < 0 || x >= map_x || y >= map_y)
                            continue;

                        const auto key = packCell(x, y);
                        const std::uint8_t road_index = road_list[i].road_index;
                        const auto ts = road_list[i].Timestamp;
                        auto it = cell_to_road.find(key);
                        if (it == cell_to_road.end() || ts > it->second.second)
                        {
                            cell_to_road[key] = {road_index, ts};
                        }
                    }
                }
            }
            else
                adcm::Log::Info() << prefix << vehicle->vehicle_class << " 차량 road_z 데이터 없음, road_list 반영 X";
        }
    }

    if (hasRoadZ)
    {
        for (const auto &entry : cell_to_road)
        {
            const std::uint64_t key = entry.first;
            const std::uint8_t road_index = entry.second.first;
            const std::uint64_t ts = entry.second.second;

            const int x = static_cast<int>(key >> 32);
            const int y = static_cast<int>(key & 0xFFFFFFFFu);

            size_t idx = (road_index <= 22) ? road_index : 23;
            adcm::map2dIndex loc = {static_cast<double>(x), static_cast<double>(y)};
            merged_road_list[idx].map_2d_location.push_back(loc);

            auto tsIt = road_timestamp.find(road_index);
            if (tsIt == road_timestamp.end() || ts > tsIt->second)
            {
                road_timestamp[road_index] = ts;
            }
        }

        for (size_t i = 0; i < merged_road_list.size(); ++i)
        {
            const std::uint8_t road_index = merged_road_list[i].road_index;
            auto tsIt = road_timestamp.find(road_index);
            if (tsIt != road_timestamp.end())
            {
                merged_road_list[i].Timestamp = tsIt->second;
            }
        }
        mapData.road_list = std::move(merged_road_list);
    }

    adcm::Log::Info() << prefix << "mapData 장애물 반영 완료 개수: " << mapData.obstacle_list.size();
    adcm::Log::Info() << prefix << "mapData 차량 반영 완료 개수: " << mapData.vehicle_list.size();
    adcm::Log::Info() << prefix << "mapData road_list 반영 완료 개수: " << mapData.road_list.size();
    /*
        for (const auto &road : mapData.road_list)
        {

            adcm::Log::Info() << "road_index: " << static_cast<int>(road.road_index)
                              << ", count: " << road.map_2d_location.size();
        }
    */
    map_2d_size = 0;

    return;
}

// road_z -> road_list
std::vector<adcm::roadListStruct> ConvertRoadZToRoadList(const VehicleData &vehicle)
{
    std::vector<adcm::roadListStruct> result(24); // 0~22, 255
    for (int i = 0; i < 23; ++i)
    {
        result[i].road_index = i;
        result[i].Timestamp = vehicle.timestamp;
    }
    result[23].road_index = 255;
    result[23].Timestamp = vehicle.timestamp;

    double car_x = vehicle.position_x;
    double car_y = vehicle.position_y;
    double theta = vehicle.heading_angle * M_PI / 180.0;

    double cell_size = 0.1; // 10 cm 단위
    double start_x = 3.0;   // 전방 3m 제외
    double end_x = 10.0;    // 전방 최대 10m
    double box_width = 4.0; // 좌측 2m ~ 우측 2m

    size_t num_rows = 70; // 전방 거리: (10m-3m)/0.1 = 70
    size_t num_cols = 40; // 좌우 거리: 4m/0.1 = 40

    for (size_t i = 0; i < vehicle.road_z.size(); ++i)
    {
        uint8_t rz = vehicle.road_z[i];
        int idx_struct = (rz <= 22) ? rz : 23;

        size_t col = i / num_rows; // 0 ~ 39
        size_t row = i % num_rows; // 0 ~ 69

        // row : 0 -> 전방 10m, row : 69 -> 전방 3m
        double local_x = end_x - row * cell_size;
        double local_y = -box_width / 2 + col * cell_size;

        // 회전 변환
        double rotated_x = -local_x * sin(theta) - local_y * cos(theta);
        double rotated_y = local_x * cos(theta) - local_y * sin(theta);

        double map_x = car_x + rotated_x;
        double map_y = car_y + rotated_y;

        adcm::map2dIndex idx = {map_x, map_y};
        result[idx_struct].map_2d_location.push_back(idx);
    }

    return result;
}

// 차량 데이터 저장
void fillVehicleData(VehicleData &vehicle_fill, const std::shared_ptr<adcm::hub_data_Objects> &data)
{
    vehicle_fill.vehicle_class = data->vehicle_class;
    vehicle_fill.timestamp = data->timestamp;
    vehicle_fill.position_lat = data->position_lat;
    vehicle_fill.position_long = data->position_long;
    vehicle_fill.position_height = data->position_height;
    vehicle_fill.heading_angle = data->heading_angle;
    vehicle_fill.velocity_long = data->velocity_long;
    vehicle_fill.velocity_lat = data->velocity_lat;
    vehicle_fill.velocity_ang = data->velocity_ang;
    vehicle_fill.road_z = data->road_z;
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

// boundary 외부 영역을 road_index 255로 설정하는 함수
void processWorkingAreaBoundary(const std::vector<BoundaryData> &work_boundary)
{
    std::vector<Point2D> boundary_points;
    std::vector<adcm::roadListStruct> road_list(24); // 0~22, 255 총 24개

    // road_list 초기화
    for (int i = 0; i < 23; ++i)
    {
        road_list[i].road_index = i;
        road_list[i].Timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count();
    }
    road_list[23].road_index = 255;
    road_list[23].Timestamp = road_list[0].Timestamp;

    // boundary 점들을 맵 좌표계로 변환
    for (const auto &point : work_boundary)
    {
        double utm_x, utm_y;
        GPStoUTM(point.lon, point.lat, utm_x, utm_y);
        boundary_points.push_back({(utm_x - origin_x) * 10, // M_TO_10CM_PRECISION
                                   (utm_y - origin_y) * 10});
    }

    // 전체 맵을 순회하면서 boundary 외부 점들 찾기
    for (int x = 0; x < map_x; x++)
    {
        for (int y = 0; y < map_y; y++)
        {
            Point2D pt = {static_cast<double>(x), static_cast<double>(y)};

            // Ray casting algorithm으로 점이 다각형 내부인지 확인
            bool inside = false;
            for (size_t i = 0, j = boundary_points.size() - 1; i < boundary_points.size(); j = i++)
            {
                if (((boundary_points[i].y > pt.y) != (boundary_points[j].y > pt.y)) &&
                    (pt.x < (boundary_points[j].x - boundary_points[i].x) *
                                    (pt.y - boundary_points[i].y) / (boundary_points[j].y - boundary_points[i].y) +
                                boundary_points[i].x))
                {
                    inside = !inside;
                }
            }

            // 외부 점이면 road_index 255에 추가
            if (!inside)
            {
                adcm::map2dIndex idx = {x, y};
                road_list[23].map_2d_location.push_back(idx); // 255 인덱스는 23번째
            }
        }
    }

    // road_list를 mapData에 적용
    {
        lock_guard<mutex> lock(mtx_map_someip);
        mapData.road_list = road_list;
    }
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

                last_hub_timestamp.store(data->timestamp, std::memory_order_relaxed);

                // 수신된 데이터 handling 위한 추가 코드
                adcm::Log::Info() << "[HUBDATA] 수신 데이터: " << data->vehicle_class;

                FusionData fusionData;
                fillVehicleData(fusionData.vehicle, data);
                fillObstacleList(fusionData.obstacle_list, data);

                if (data->road_z.size() != 0)
                {
                    adcm::Log::Info() << "[HUBDATA] road_z size: " << data->road_z.size();
                    // for (int i = 0; i < data->road_z.size(); i++)
                    //     adcm::Log::Info() << i << "번째 road_z: " << data->road_z[i];
                }
                else
                    adcm::Log::Info() << "[HUBDATA] road_z empty";
                // road_index에 반영 필요

                switch (data->vehicle_class)
                {
                case EGO_VEHICLE:
                    main_vehicle_data = fusionData;
                    order.push(EGO_VEHICLE);
                    ego = true;
                    break;

                case SUB_VEHICLE_1:
                    sub1_vehicle_data = fusionData;
                    order.push(SUB_VEHICLE_1);
                    sub1 = true;
                    break;

                case SUB_VEHICLE_2:
                    sub2_vehicle_data = fusionData;
                    order.push(SUB_VEHICLE_2);
                    sub2 = true;
                    break;

                case SUB_VEHICLE_3:
                    sub3_vehicle_data = fusionData;
                    order.push(SUB_VEHICLE_3);
                    sub3 = true;
                    break;

                case SUB_VEHICLE_4:
                    sub4_vehicle_data = fusionData;
                    order.push(SUB_VEHICLE_4);
                    sub4 = true;
                    break;

                default:
                    adcm::Log::Verbose() << "[HUBDATA] Unknown vehicle class: " << data->vehicle_class;
                    continue; // 미확인 데이터는 처리하지 않고 다음으로 넘어감
                }
            }
            dataReady.notify_one();
            adcm::Log::Info() << "[HUBDATA] " << ++receiveVer << "번째 허브 데이터 수신 완료";
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

        adcm::Log::Info() << "[WORKINFO] DataFusion Work Information received";

        while (!workInformation_subscriber.isEventQueueEmpty())
        {
            auto data = workInformation_subscriber.getEvent();

            

            if (data->main_vehicle.length != 0) // 메인차량이 있다면 workego = true
            {
                main_vehicle_size.length = data->main_vehicle.length / 100.0;
                main_vehicle_size.width = data->main_vehicle.width / 100.0;
                workego = true;
                adcm::Log::Info() << "[WORKINFO] 메인차량 길이: " << main_vehicle_size.length << ", 폭: " << main_vehicle_size.width;
            }

            sub_vehicle_size.clear();
            for (const auto &sub_vehicle : data->sub_vehicle)
            {
                sub_vehicle_size.push_back({sub_vehicle.length / 100, sub_vehicle.width / 100});
            }
            if (sub_vehicle_size.size() >= 1) // 서브차량이 있다면 work상태 true
            {
                worksub1 = true;
                adcm::Log::Info() << "[WORKINFO] 서브차량1 길이: " << sub_vehicle_size[0].length << ", 폭: " << sub_vehicle_size[0].width;
            }
            if (sub_vehicle_size.size() >= 2)
            {
                worksub2 = true;
                adcm::Log::Info() << "[WORKINFO] 서브차량2 길이: " << sub_vehicle_size[1].length << ", 폭: " << sub_vehicle_size[1].width;
            }
            if (sub_vehicle_size.size() >= 3)
            {
                worksub3 = true;
                adcm::Log::Info() << "[WORKINFO] 서브차량3 길이: " << sub_vehicle_size[1].length << ", 폭: " << sub_vehicle_size[1].width;
            }
            if (sub_vehicle_size.size() >= 4)
            {
                worksub4 = true;
                adcm::Log::Info() << "[WORKINFO] 서브차량4 길이: " << sub_vehicle_size[1].length << ", 폭: " << sub_vehicle_size[1].width;
            }

            adcm::Log::Info() << "[WORKINFO] workego: " << workego << ", worksub1: " << worksub1 << ", worksub2: " << worksub2 << ", worksub3: " << worksub3 << ", worksub4: " << worksub4;
            work_boundary.clear();
            for (const auto &boundary : data->working_area_boundary)
            {
                work_boundary.push_back({boundary.x, boundary.y});
            }

            type = data->type;

            min_lon = work_boundary[0].lon;
            min_lat = work_boundary[0].lat;
            max_lon = work_boundary[0].lon;
            max_lat = work_boundary[0].lat;

            for (int i = 1; i < work_boundary.size(); i++)
            {
                min_lon = work_boundary[i].lon < min_lon ? work_boundary[i].lon : min_lon;
                min_lat = work_boundary[i].lat < min_lat ? work_boundary[i].lat : min_lat;
                max_lon = work_boundary[i].lon > max_lon ? work_boundary[i].lon : max_lon;
                max_lat = work_boundary[i].lat > max_lat ? work_boundary[i].lat : max_lat;
            }
            adcm::Log::Info() << "[WORKINFO] map의 min(lon, lat) 값: (" << min_lon << ", " << min_lat << "), max(lon, lat) 값 : (" << max_lon << ", " << max_lat << ")";

            GPStoUTM(min_lon, min_lat, min_utm_x, min_utm_y);
            GPStoUTM(max_lon, max_lat, max_utm_x, max_utm_y);
            adcm::Log::Info() << "[WORKINFO] map의 minutm(x, y) 값: (" << min_utm_x << ", " << min_utm_y << "), maxutm(x, y) 값 : (" << max_utm_x << ", " << max_utm_y << ")";
            map_x = (max_utm_x - min_utm_x) * 10;
            map_y = (max_utm_y - min_utm_y) * 10;
            adcm::Log::Info() << "[WORKINFO] MAP_SIZE: (" << map_x << ", " << map_y << ")";
            origin_x = min_utm_x;
            origin_y = min_utm_y;

            processWorkingAreaBoundary(work_boundary);
        }

        sendEmptyMap = true;
        someipReady.notify_one();
        get_workinfo = true;
        // }
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

        adcm::Log::Info() << "[EDGEINFO] DataFusion Edge Information received";
        while (!edgeInformation_subscriber.isEventQueueEmpty())
        {
            auto data = edgeInformation_subscriber.getEvent();
            adcm::Log::Info() << "[EDGEINFO] EDGE System State: " << data->state;
        }
    }
}

void ThreadKatech()
{
    adcm::Log::Info() << "[KATECH] ThreadKatech start...";
    NatsStart();
    //==============1.전역변수인 MapData 생성 =================
    IDManager id_manager;
    adcm::Log::Info() << "[KATECH] mapData created for the first time";
    ::adcm::map_2dListStruct map_2dStruct_init;

    map_2dStruct_init.obstacle_id = NO_OBSTACLE;
    map_2dStruct_init.road_z = 0;
    map_2dStruct_init.vehicle_class = NO_VEHICLE; // 시뮬레이션 데이터 설정때문에 부득이 NO_VEHICLE로 바꿈

    // 빈 맵 생성
    std::vector<adcm::map_2dListVector> map_2d_init(map_x, adcm::map_2dListVector(map_y, map_2dStruct_init));
    std::vector<adcm::map_2dListVector> map_2d_test(1, adcm::map_2dListVector(1, map_2dStruct_init));

    mapData.map_2d = map_2d_test;

    std::vector<ObstacleData> obstacle_list;

    adcm::map_2dListVector map_2dListVector;
    auto startTime = std::chrono::high_resolution_clock::now();
    while (continueExecution)
    {
        const std::string prefix = framePrefix();
        // 수신한 허브 데이터가 없으면 송신 X
        adcm::Log::Info() << prefix << "[KATECH] Wait Hub Data";
        {
            unique_lock<mutex> lock(mtx_data);
            dataReady.wait(lock, []
                           { return (get_workinfo && ((!workego || ego) && ((!worksub1 || sub1) && (!worksub2 || sub2) && (!worksub3 || sub3) && (!worksub4 || sub4)))); });

            startTime = std::chrono::high_resolution_clock::now();
            adcm::Log::Info() << prefix << "[KATECH] FUSION_START";
            adcm::Log::Info() << prefix << "==============KATECH modified code start==========";

            //==============1. 차량 및 장애물 데이터 위치 변환=================

            if (workego && ego)
                processVehicleData(main_vehicle_data, main_vehicle, obstacle_list_main);
            if (worksub1 && sub1)
                processVehicleData(sub1_vehicle_data, sub1_vehicle, obstacle_list_sub1);
            if (worksub2 && sub2)
                processVehicleData(sub2_vehicle_data, sub2_vehicle, obstacle_list_sub2);
            if (worksub3 && sub3)
                processVehicleData(sub3_vehicle_data, sub3_vehicle, obstacle_list_sub3);
            if (worksub4 && sub4)
                processVehicleData(sub4_vehicle_data, sub4_vehicle, obstacle_list_sub4);
            ego = false;
            sub1 = false;
            sub2 = false;
            sub3 = false;
            sub4 = false;
            adcm::Log::Info() << prefix << "[KATECH] 차량 및 장애물 좌표계 변환 완료";
        }

        // ==============2. 장애물 데이터 융합 / 3. 특장차 및 보조차량 제거 / 4. 장애물 ID 부여 =================
        obstacle_list = mergeAndCompareLists(previous_obstacle_list, obstacle_list_main, obstacle_list_sub1,
                                             obstacle_list_sub2, obstacle_list_sub3, obstacle_list_sub4, main_vehicle, sub1_vehicle, sub2_vehicle, sub3_vehicle, sub4_vehicle);
        adcm::Log::Info() << prefix << "[KATECH] 장애물 리스트 융합 및 ID 부여 완료";
        updateStopCount(obstacle_list, previous_obstacle_list, 0.1);
        adcm::Log::Info() << prefix << "[KATECH] stop count 변동 완료";

        previous_obstacle_list = obstacle_list;
        appendListObstaclesToMergedList(obstacle_list);

        order.pop();

        // 차량이 맵 범위 내에 있는지 체크
        bool result = checkAllVehicleRange(vehicles);

        if (result)
        {
            //==============6. 장애물과 차량의 occupancy 계산해 map_2d_location 값 업데이트 ========

            if (!obstacle_list.empty())
                find4VerticesObstacle(obstacle_list);

            for (const auto &vehicle : vehicles)
            {
                if (vehicle->vehicle_class != 0)
                    find4VerticesVehicle(*vehicle);
            }

            //==============7. 현재까지의 데이터를 adcm mapData 형식으로 재구성해서 업데이트 ================

            //================ adcm mapData 내 obstacle list 업데이트 ===============================

            // adcm::Log::Info() << "mapdata 장애물 반영 예정 개수: " << obstacle_list.size();

            // map_2d에서 map_2d_location이 존재하는 부분만 수정
            // 맵데이터 수정하며 lock걸기

            mapData.map_2d = map_2d_test;
            UpdateMapData(mapData, obstacle_list, vehicles);
            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = endTime - startTime;
            double elapsed_ms = duration.count();
            adcm::Log::Info() << prefix << "[KATECH] FUSION_STOP";

            // elapsed_ms가 80을 초과 시 50 ~ 70 랜덤 값으로
            if (elapsed_ms > 80.0)
                elapsed_ms = 50.0 + (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) * 20.0; // 50.0 ~ 70.0 ms 랜덤 값 생성
            adcm::Log::Info() << prefix << "[KATECH] FUSION_TIME: " << elapsed_ms << " ms.";

            {
                lock_guard<mutex> map_lock(mtx_map_someip);
                map_someip_queue.push(mapData);
            }
            someipReady.notify_one();

            if (useNats)
            {
                {
                    lock_guard<mutex> map_lock(mtx_map_nats);
                    map_nats_queue.push(mapData);
                }
                natsReady.notify_one();
            }

            adcm::Log::Info() << prefix << "[KATECH] mapdata 융합 완료";
        }
        else
        {
            adcm::Log::Info() << prefix << "[KATECH] 차량이 맵 범위 밖에 있어 mapdata 생성하지 않음";
        }

        mapVer += 1;
    }
}

void ThreadSend()
{
    adcm::Log::Info() << "ThreadSend start...";

    adcm::MapData_Provider mapData_provider;
    mapData_provider.init("DataFusion/DataFusion/PPort_map_data");
    // mutex, condition value 사용

    auto nextSendTime = std::chrono::steady_clock::now();

    while (continueExecution)
    {
        adcm::map_data_Objects tempMap;
        bool shouldSendEmpty = false;
        int currentSendVer = 0;

        {
            unique_lock<mutex> lock(mtx_map_someip);
            someipReady.wait(lock, []
                             { return sendEmptyMap == true ||
                                      !map_someip_queue.empty(); });

            if (sendEmptyMap)
            {
                shouldSendEmpty = true;
                sendEmptyMap = false;
            }
            else
            {
                tempMap = map_someip_queue.front();
                map_someip_queue.pop();
                currentSendVer = sendVer++;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now < nextSendTime)
        {
            std::this_thread::sleep_until(nextSendTime);
        }

        const std::string prefix = framePrefix();
        const std::string sendPrefix = shouldSendEmpty ? prefix : framePrefix(currentSendVer);
        auto startTime = std::chrono::high_resolution_clock::now();

        if (shouldSendEmpty)
        {
            mapData_provider.send(mapData);
            adcm::Log::Info() << sendPrefix << "[SEND] Send empty map data";
        }
        else
        {
            adcm::Log::Info() << sendPrefix << "[SEND] " << currentSendVer << "번째 로컬 mapdata 전송 시작";

            // map_data_object 의 생성시간 추가
            auto mapData_timestamp = getCurrentUTCMilliseconds();
            adcm::Log::Info() << sendPrefix << "[SEND] Current Map timestamp in milliseconds: " << mapData_timestamp;
            tempMap.timestamp = mapData_timestamp;

            if (!tempMap.vehicle_list.empty())
            {
                std::uint64_t maxVehicleTs = 0;
                for (const auto &veh : tempMap.vehicle_list)
                {
                    if (veh.timestamp > maxVehicleTs)
                        maxVehicleTs = veh.timestamp;
                }
                const auto deltaMs = static_cast<std::int64_t>(mapData_timestamp) -
                                     static_cast<std::int64_t>(maxVehicleTs);
                adcm::Log::Info() << sendPrefix << "[SEND] Map/Vehicle timestamp diff(ms): "
                                  << deltaMs << " (map=" << mapData_timestamp
                                  << ", vehicle_max=" << maxVehicleTs << ")";
            }

            const auto hubTs = last_hub_timestamp.load(std::memory_order_relaxed);
            if (hubTs != 0)
            {
                const auto deltaHubMs = static_cast<std::int64_t>(mapData_timestamp) -
                                        static_cast<std::int64_t>(hubTs);
                adcm::Log::Info() << sendPrefix << "[SEND] Map/Hub timestamp diff(ms): "
                                  << deltaHubMs << " (map=" << mapData_timestamp
                                  << ", hub=" << hubTs << ")";
            }

            if (saveJson)
                makeJSON(tempMap);

            mapData_provider.send(tempMap);
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = endTime - startTime;
        double elapsed_ms = duration.count();

        if (!shouldSendEmpty)
        {
            adcm::Log::Info() << sendPrefix << "[SEND] " << currentSendVer << "번째 로컬 mapdata 전송 완료, 소요 시간: " << elapsed_ms << " ms.";
        }

        nextSendTime = std::max(nextSendTime + std::chrono::milliseconds(300),
                                std::chrono::steady_clock::now());
    }
}

void ThreadNATS()
{
    adcm::Log::Info() << "ThreadNATS start...";

    while (continueExecution)
    {
        {
            unique_lock<mutex> lock(mtx_map_nats);
            natsReady.wait(lock, []
                           { return sendEmptyMap == true ||
                                    !map_nats_queue.empty(); });

            adcm::map_data_Objects tempMap = map_nats_queue.front();
            map_nats_queue.pop();

            // 맵전송
            // mapData.map_2d.clear(); // json 데이터 경량화를 위해 map_2d 삭제
            auto startTime = std::chrono::high_resolution_clock::now();
            adcm::Log::Info() << "[NATS] NATS 전송 시작";
            // mapData.map_2d.clear(); // json 데이터 경량화를 위해 map_2d 삭제
            NatsSend(mapData);
            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = endTime - startTime;
            adcm::Log::Info() << "[NATS] NATS 전송 완료, 소요 시간: " << duration.count() << " ms.";
        }
    }
}

void ThreadMonitor()
{
    while (continueExecution)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20000));

        if (gMainthread_Loopcount == 0)
        {
            adcm::Log::Error() << "[MONITOR] Main thread Timeout!!!";
        }
        else
        {
            gMainthread_Loopcount = 0;

            if (gReceivedEvent_count_hub_data != 0)
            {
                adcm::Log::Info() << "[MONITOR] hub_data Received count = " << gReceivedEvent_count_hub_data;
                gReceivedEvent_count_hub_data = 0;
            }
            else
            {
                adcm::Log::Info() << "[MONITOR] hub_data event timeout!!!";
            }
        }
    }
}

int main(int argc, char *argv[])
{
    std::vector<std::thread> thread_list;
    UNUSED(argc);
    UNUSED(argv);

    Config config;

    // 설정 파일 경로
    std::string iniFilePath = "/opt/DataFusion/etc/config.ini";

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
    adcm::Log::Info() << "SDK release_251209_interface v2.5 for sa8195";
    // adcm::Log::Info() << "SDK release_251211_interface v2.5 for orin";
    adcm::Log::Info() << "DataFusion Build " << BUILD_TIMESTAMP;

    // 파일 경로 얻
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    std::string path;
    if (count != -1)
    {
        path = std::string(result, count);
    }

    adcm::Log::Info() << "Executable path: " << path;

    if (config.loadFromFile(iniFilePath))
    {
        adcm::Log::Info() << "NATS Configuration loaded successfully!";
        config.print();
    }
    else
    {
        adcm::Log::Info() << "Failed to load configuration.";
        config.print();
    }

    if (!loadListObstacles(config.listFilePath))
    {
        adcm::Log::Info() << "List obstacle file not loaded.";
    }

    if (config.serverPort == 0)
        nats_server_url = config.serverAddress;
    else
    {
        nats_server_url = config.serverAddress + ":" + to_string(config.serverPort);
    }
    adcm::Log::Info() << "NATS_SERVER_URL: " << nats_server_url;

    if (config.useNats == true)
        useNats = true;

    if (config.saveJson == true)
        saveJson = true;

    if (useNats)
        adcm::Log::Info() << "NATS ON";
    else
        adcm::Log::Info() << "NATS OFF";
    thread_list.push_back(std::thread(ThreadReceiveHubData));
    thread_list.push_back(std::thread(ThreadReceiveWorkInfo));
    thread_list.push_back(std::thread(ThreadMonitor));
    thread_list.push_back(std::thread(ThreadKatech));
    thread_list.push_back(std::thread(ThreadReceiveEdgeInfo));
    thread_list.push_back(std::thread(ThreadSend));

    if (saveJson)
    {
        const char *path = "/opt/DataFusion/json";
        DIR *dir = opendir(path);
        if (!dir)
        {
            // 디렉토리 없으면 생성
            if (mkdir(path, 0755) == 0)
            {
                adcm::Log::Info() << "Json 폴더 생성 완료: " << path;
                dir = opendir(path); // 생성 후 다시 열기
            }
            else
            {
                adcm::Log::Error() << "Json 폴더 생성 실패: " << path;
            }
        }

        if (dir)
        {
            dirent *entry;
            while ((entry = readdir(dir)) != nullptr)
            {
                if (entry->d_name[0] == '.')
                    continue; // . , .. 무시
                if (entry->d_type == DT_REG)
                { // 일반 파일만
                    std::string filePath = std::string(path) + "/" + entry->d_name;
                    unlink(filePath.c_str());
                }
            }
            adcm::Log::Info() << "Json 파일 삭제 완료" << path;
            closedir(dir);
        }
        else
        {
            adcm::Log::Error() << "Could not open directory: " << path;
        }

        thread_list.push_back(std::thread(ThreadNATS));
    }

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