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
#include <numeric>
#include "main_datafusion.hpp"
#include "config.cpp"

#include <unistd.h>
#include <limits>
#include <limits.h>

#define NATS
#ifdef NATS

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

    natsManager->NatsMsgDestroy(msg);
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
            Poco::JSON::Object::Ptr point = new Poco::JSON::Object;
            point->set("x", pt.x);
            point->set("y", pt.y);
            locArr->add(point);
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
        vehArr->add(obj);
    }
    mapObj->set("vehicle_list", vehArr);

    return mapObj;
}

// 장애물 리스트 융합 (정적/동적 분리 처리)
std::vector<ObstacleData> mergeAndCompareListsDynamic(
    const std::vector<ObstacleData> &previousFusionList,
    const std::vector<ObstacleData> &currentFusionList)
{
    std::vector<ObstacleData> prevStatic, prevDynamic;
    std::vector<ObstacleData> curStatic, curDynamic;

    splitObstaclesByType(previousFusionList, prevStatic, prevDynamic);
    splitObstaclesByType(currentFusionList, curStatic, curDynamic);

    // 정적 장애물 매칭
    if (!curStatic.empty())
    {
        std::vector<int> assignment(curStatic.size(), -1);
        if (!prevStatic.empty())
        {
            auto distMatrix = createDistanceMatrix(curStatic, prevStatic, STATIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD);
            assignment = solveAssignment(distMatrix);
        }
        processFusion(curStatic, prevStatic, assignment);
    }

    // 동적 장애물 매칭
    if (!curDynamic.empty())
    {
        std::vector<int> assignment(curDynamic.size(), -1);
        if (!prevDynamic.empty())
        {
            auto distMatrix = createDistanceMatrix(curDynamic, prevDynamic, DYNAMIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD);
            assignment = solveAssignment(distMatrix);
        }
        processFusion(curDynamic, prevDynamic, assignment);
    }

    std::vector<ObstacleData> fused;
    fused.reserve(curStatic.size() + curDynamic.size());
    fused.insert(fused.end(), curStatic.begin(), curStatic.end());
    fused.insert(fused.end(), curDynamic.begin(), curDynamic.end());
    return fused;
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
    // 모든 차량의 장애물을 동등하게 수집 (순서 편향 제거)
    // Ego -> Sub1 -> Sub2 -> Sub3 -> Sub4 순서로 일관되게 처리
    std::vector<VehicleData> nonEmptyVehicles;
    std::vector<std::vector<ObstacleData>> nonEmptyLists;
    std::vector<ObstacleData> mergedList;

    // Ego 차량을 가장 먼저 추가 (우선순위: Ego > Sub1 > Sub2 > Sub3 > Sub4)
    if (workego)
    {
        nonEmptyVehicles.push_back(mainVehicle);
        nonEmptyLists.push_back(listMain);
    }
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

    if (nonEmptyLists.size() == 1)
    {
        mergedList = nonEmptyLists[0];
    }
    else if (nonEmptyLists.size() > 1)
    {
        // Step 0: 차량별 중복 제거 (같은 차량 내 동일 위치/클래스 중복을 하나로 통합)
        auto dedupeList = [&](const std::vector<ObstacleData> &inList) -> std::vector<ObstacleData> {
            if (inList.empty()) return inList;
            std::vector<bool> used(inList.size(), false);
            std::vector<ObstacleData> out;
            out.reserve(inList.size());

            for (size_t i = 0; i < inList.size(); ++i)
            {
                if (used[i]) continue;
                const auto &base = inList[i];
                // 정적/동적에 따라 더 엄격한 자체 중복 임계값 설정 (교차 임계의 절반)
                const double thr = isStaticObstacle(base)
                    ? std::min(STATIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD, CROSS_STATIC_MATCH_DISTANCE_THRESHOLD) / 2.0
                    : std::min(DYNAMIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD, CROSS_DYNAMIC_MATCH_DISTANCE_THRESHOLD) / 2.0;

                double sum_x = base.fused_position_x;
                double sum_y = base.fused_position_y;
                double sum_z = base.fused_position_z;
                double sum_vx = base.fused_velocity_x;
                double sum_vy = base.fused_velocity_y;
                size_t count = 1;
                used[i] = true;

                for (size_t j = i + 1; j < inList.size(); ++j)
                {
                    if (used[j]) continue;
                    const auto &cand = inList[j];
                    if (cand.obstacle_class != base.obstacle_class) continue;
                    double dist = std::sqrt(std::pow(base.fused_position_x - cand.fused_position_x, 2) +
                                            std::pow(base.fused_position_y - cand.fused_position_y, 2));
                    if (dist <= thr)
                    {
                        used[j] = true;
                        sum_x += cand.fused_position_x;
                        sum_y += cand.fused_position_y;
                        sum_z += cand.fused_position_z;
                        sum_vx += cand.fused_velocity_x;
                        sum_vy += cand.fused_velocity_y;
                        ++count;
                    }
                }

                ObstacleData fused = base;
                fused.fused_position_x = sum_x / count;
                fused.fused_position_y = sum_y / count;
                fused.fused_position_z = sum_z / count;
                fused.fused_velocity_x = sum_vx / count;
                fused.fused_velocity_y = sum_vy / count;
                // 동일 차량 내 관측값이므로 stop_count는 0 유지
                fused.stop_count = 0;
                out.push_back(fused);
            }

            return out;
        };

        for (size_t i = 0; i < nonEmptyLists.size(); ++i)
        {
            const std::size_t before = nonEmptyLists[i].size();
            nonEmptyLists[i] = dedupeList(nonEmptyLists[i]);
            const std::size_t after = nonEmptyLists[i].size();
            if (after < before)
            {
                adcm::Log::Info() << "per-vehicle dedupe: vehicleIdx=" << i << " size " << before << " → " << after
                                  << " (removed=" << (before - after) << ")";
            }
        }

        // Step 1: 모든 장애물을 하나의 후보 풀로 수집
        struct ObstacleWithSource {
            ObstacleData obs;
            int vehicleIdx;
            int localIdx;
        };
        std::vector<ObstacleWithSource> candidatePool;
        
        for (size_t i = 0; i < nonEmptyLists.size(); ++i)
        {
            for (size_t j = 0; j < nonEmptyLists[i].size(); ++j)
            {
                candidatePool.push_back({nonEmptyLists[i][j], static_cast<int>(i), static_cast<int>(j)});
            }
        }
        adcm::Log::Info() << "merge 후보 풀 크기: " << candidatePool.size();
        
        // Step 2: 전체 장애물 간 거리 행렬 생성 (Munkres 준비)
        size_t totalObs = candidatePool.size();
        std::vector<std::vector<double>> globalDistMatrix(totalObs, std::vector<double>(totalObs, HUNGARIAN_MAX_COST));
        
        // 차량 간 매칭: 클래스가 동일하고 거리가 임계값 이내일 때만 매칭 후보로 설정
        auto crossThreshold = [&](const ObstacleData &a, const ObstacleData &b) {
            bool aStatic = isStaticObstacle(a);
            bool bStatic = isStaticObstacle(b);
            if (aStatic && bStatic)
                return CROSS_STATIC_MATCH_DISTANCE_THRESHOLD;
            return CROSS_DYNAMIC_MATCH_DISTANCE_THRESHOLD;
        };

        for (size_t i = 0; i < totalObs; ++i)
        {
            for (size_t j = i + 1; j < totalObs; ++j)
            {
                // 같은 차량의 장애물끼리는 매칭 불가
                if (candidatePool[i].vehicleIdx == candidatePool[j].vehicleIdx)
                    continue;

                // 클래스 다르면 매칭하지 않음
                if (candidatePool[i].obs.obstacle_class != candidatePool[j].obs.obstacle_class)
                    continue;
                
                double dist = std::sqrt(
                    std::pow(candidatePool[i].obs.fused_position_x - candidatePool[j].obs.fused_position_x, 2) +
                    std::pow(candidatePool[i].obs.fused_position_y - candidatePool[j].obs.fused_position_y, 2)
                );

                double thr = crossThreshold(candidatePool[i].obs, candidatePool[j].obs);
                if (dist <= thr)
                {
                    globalDistMatrix[i][j] = dist;
                    globalDistMatrix[j][i] = dist;
                }
            }
        }
        
        // Step 3: Union-Find로 클러스터링 (거리 기반 그룹핑)
        std::vector<int> parent(totalObs);
        std::iota(parent.begin(), parent.end(), 0);
        
        std::function<int(int)> find = [&](int x) {
            return parent[x] == x ? x : parent[x] = find(parent[x]);
        };
        
        auto unite = [&](int x, int y) {
            x = find(x);
            y = find(y);
            if (x != y) parent[x] = y;
        };
        
        // 거리 임계값 이하인 장애물들을 같은 클러스터로 병합
        for (size_t i = 0; i < totalObs; ++i)
        {
            for (size_t j = i + 1; j < totalObs; ++j)
            {
                if (globalDistMatrix[i][j] < HUNGARIAN_MAX_COST)
                {
                    unite(i, j);
                }
            }
        }
        
        // Step 4: 클러스터별로 그룹화
        std::map<int, std::vector<int>> clusters;
        for (size_t i = 0; i < totalObs; ++i)
        {
            clusters[find(i)].push_back(i);
        }

        // 클러스터 요약 로그 (사이즈/다중 차량/클래스 불일치 비율)
        {
            std::size_t totalClusters = clusters.size();
            std::size_t size1 = 0, size2 = 0, size3plus = 0;
            std::size_t multiVehicleClusters = 0;
            std::size_t mixedClassClusters = 0;

            for (const auto &cluster_pair : clusters)
            {
                const std::vector<int> &indices = cluster_pair.second;
                if (indices.size() == 1) ++size1;
                else if (indices.size() == 2) ++size2;
                else ++size3plus;

                std::set<int> vehicleSet;
                std::set<int> classSet;
                for (int idx : indices)
                {
                    vehicleSet.insert(candidatePool[idx].vehicleIdx);
                    classSet.insert(candidatePool[idx].obs.obstacle_class);
                }
                if (vehicleSet.size() >= 2) ++multiVehicleClusters;
                if (classSet.size() >= 2) ++mixedClassClusters;
            }

            adcm::Log::Info() << "Cluster summary: total=" << totalClusters
                              << ", size1=" << size1
                              << ", size2=" << size2
                              << ", size3+=" << size3plus
                              << ", multiVehicle=" << multiVehicleClusters
                              << ", mixedClass=" << mixedClassClusters;
        }
        
        // Step 5: 각 클러스터 내에서 Munkres로 최적 매칭
        for (const auto &cluster_pair : clusters)
        {
            const std::vector<int> &indices = cluster_pair.second;
            
            if (indices.size() == 1)
            {
                // 단일 장애물 (매칭 없음)
                mergedList.push_back(candidatePool[indices[0]].obs);
            }
            else
            {
                // 클러스터 내 차량별로 분류
                std::map<int, std::vector<int>> vehicleGroups;
                for (int idx : indices)
                {
                    vehicleGroups[candidatePool[idx].vehicleIdx].push_back(idx);
                }
                
                // 차량이 1개만 있으면 매칭 불필요
                if (vehicleGroups.size() == 1)
                {
                    for (int idx : indices)
                    {
                        mergedList.push_back(candidatePool[idx].obs);
                    }
                }
                else
                {
                    adcm::Log::Info() << "cluster fused: size=" << indices.size() << " → 1";
                    // 2개 이상 차량: 클러스터 내 모든 장애물을 단일 객체로 융합
                    // (기존의 pairwise Munkres는 3-way 클러스터에서 2+1로 남겨 중복을 유발함)
                    const std::vector<int> &clusterIndices = indices;
                    ObstacleData fusedObs = candidatePool[clusterIndices[0]].obs;

                    double sum_x = 0, sum_y = 0, sum_z = 0;
                    double sum_vx = 0, sum_vy = 0;
                    for (int idxInCluster : clusterIndices)
                    {
                        const auto &obs = candidatePool[idxInCluster].obs;
                        sum_x += obs.fused_position_x;
                        sum_y += obs.fused_position_y;
                        sum_z += obs.fused_position_z;
                        sum_vx += obs.fused_velocity_x;
                        sum_vy += obs.fused_velocity_y;
                    }

                    const size_t count = clusterIndices.size();
                    fusedObs.fused_position_x = sum_x / count;
                    fusedObs.fused_position_y = sum_y / count;
                    fusedObs.fused_position_z = sum_z / count;
                    fusedObs.fused_velocity_x = sum_vx / count;
                    fusedObs.fused_velocity_y = sum_vy / count;

                    // stop_count는 관측된 프레임이므로 0으로 초기화, ID는 이후 processFusion에서 부여/보존
                    fusedObs.stop_count = 0;
                    mergedList.push_back(fusedObs);
                }
            }
        }
    }

    std::vector<ObstacleData> fusedWithIds;
    if (!mergedList.empty())
    {
        fusedWithIds = mergeAndCompareListsDynamic(previousFusionList, mergedList);
    }

    static ObstacleTracker obstacleTracker;
    auto tracked = obstacleTracker.update(fusedWithIds);
    return tracked;
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

#endif // NATS

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
    for (const auto *vehicle : vehicles)
    {
        if (vehicle->vehicle_class != 0)
        {
            adcm::Log::Info() << vehicle->vehicle_class << "번 차량 위치: [" << vehicle->position_x << ", " << vehicle->position_y << "]";
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
    // if (!type) // 시뮬레이션
    // {
    //     // wps84기반 gps(global)좌표계를 작업환경 XY 기반의 Map 좌표계로 변환
    //     // 시뮬레이터 map 기준 원점(0,0) global좌표
    //     double mapOrigin_x = 453.088714;
    //     double mapOrigin_y = 507.550078;
    //     // 시뮬레이터 기준점 utm좌표
    //     double ref_x = 278296.968;
    //     double ref_y = 3980466.846;
    //     double angle_radians = -MAP_ANGLE * M_PI / 180.0;
    //     double velocity_ang = vehicle.velocity_ang;
    //     double position_x = vehicle.position_long;
    //     double position_y = vehicle.position_lat;
    //     double mapVehicle_theta = (vehicle.heading_angle + MAP_ANGLE) * M_PI / 180.0; // 시뮬레이터 상에서 차량이 바라보는 각도
    //     // 차량 utm 좌표로 변환
    //     double distance_x, distance_y; // 차량의 utm x,y 좌표
    //     GPStoUTM(position_x, position_y, distance_x, distance_y);
    //     distance_x -= ref_x;
    //     distance_y -= ref_y;
    //     vehicle.position_x = (distance_x * cos(angle_radians) - distance_y * sin(angle_radians) - mapOrigin_x) * M_TO_10CM_PRECISION;
    //     vehicle.position_y = (distance_x * sin(angle_radians) + distance_y * cos(angle_radians) - mapOrigin_y) * M_TO_10CM_PRECISION;
    //     // 속도 (각속도 보정 임시 제외)
    //     double velocity_x = vehicle.velocity_long;
    //     double velocity_y = vehicle.velocity_lat;
    //     vehicle.velocity_x = velocity_x * cos(angle_radians) - velocity_y * sin(angle_radians);
    //     vehicle.velocity_y = velocity_x * sin(angle_radians) + velocity_y * cos(angle_radians);
    //     // vehicle.velocity_x = (velocity_ang * (-sin(theta) * (position_x - alpha) + (cos(theta) * (position_y - beta)))) + (velocity_x * cos(theta)) + (velocity_y * sin(theta));
    //     // vehicle.velocity_y = (velocity_ang * (-cos(theta) * (position_x - alpha) - (sin(theta) * (position_y - beta)))) + (velocity_x * -sin(theta)) + (velocity_y * cos(theta));
    //     vehicle.heading_angle = -(vehicle.heading_angle + MAP_ANGLE - 90); // 맵에 맞춰 차량 각도 회전
    //     // adcm::Log::Info() << "차량" << vehicle.vehicle_class << "gpsToMapcoordinate 좌표변환 before (" << position_x << " , " << position_y << " , " << velocity_x << " , " << velocity_y << ")";
    //     // adcm::Log::Info() << "timestamp: " << vehicle.timestamp << " 차량" << vehicle.vehicle_class << "gpsToMapcoordinate 좌표변환 after (" << vehicle.position_x << " , " << vehicle.position_y << " , " << vehicle.heading_angle << ")";
    // }

    // else // 실증
    // {
    double position_x = vehicle.position_long;
    double position_y = vehicle.position_lat;
    double veh_utm_x, veh_utm_y; // 차량 utm 좌표
    GPStoUTM(position_x, position_y, veh_utm_x, veh_utm_y);
    vehicle.position_x = (veh_utm_x - origin_x) * M_TO_10CM_PRECISION;
    vehicle.position_y = (veh_utm_y - origin_y) * M_TO_10CM_PRECISION;
    // 차량 각도는 유지
    vehicle.velocity_x = vehicle.velocity_long;
    vehicle.velocity_y = vehicle.velocity_lat;
    // vehicle.heading_angle = 90 - vehicle.heading_angle;
    // adcm::Log::Info() << "차량" << vehicle.vehicle_class << "gpsToMapcoordinate 좌표변환 before (" << position_x << " , " << position_y << " , " << velocity_x << " , " << velocity_y << ")";
    // adcm::Log::Info() << "timestamp: " << vehicle.timestamp << " 차량" << vehicle.vehicle_class << "gpsToMapcoordinate 좌표변환 after (" << vehicle.position_x << " , " << vehicle.position_y << " , " << vehicle.heading_angle << ")";
    // }
}

void relativeToMapcoordinate(std::vector<ObstacleData> &obstacle_list, VehicleData vehicle)
{
    double theta = vehicle.heading_angle * M_PI / 180.0;
    double velocity_ang = vehicle.velocity_ang;

    adcm::Log::Info() << vehicle.vehicle_class << " 차량 위치, heading_angle: (" << vehicle.position_x << " , " << vehicle.position_y << " , " << vehicle.heading_angle << ")";
    for (auto iter = obstacle_list.begin(); iter != obstacle_list.end(); iter++)
    {
        // adcm::Log::Info() << "장애물 relativeToGlobal 좌표변환 before (" << iter->fused_position_x << " , " << iter->fused_position_y << " , " << iter->fused_velocity_x << " , " << iter->fused_velocity_y << ")";

        double obstacle_position_x = iter->fused_position_x;
        double obstacle_position_y = iter->fused_position_y;
        double obstacle_velocity_x = iter->fused_velocity_x; // 시뮬 로그 속도의 단위는 m/s인데, 결과 값의 단위는 미정
        double obstacle_velocity_y = iter->fused_velocity_y;

        // 새로 정리한 차량 heading_angle 기준 (정북: 0도, 반시계방향, 오른손 좌표계)
        iter->fused_position_x = vehicle.position_x + ((obstacle_position_x)*sin(theta) * (-1) + (obstacle_position_y)*cos(theta) * (-1)) * M_TO_10CM_PRECISION;
        iter->fused_position_y = vehicle.position_y + ((obstacle_position_x)*cos(theta) + (obstacle_position_y)*sin(theta) * (-1)) * M_TO_10CM_PRECISION;

        // 차량 좌표계 기준이므로 90+heading_angle 만큼 회전변환 필요 (추가예정)
        iter->fused_velocity_x = vehicle.velocity_x + ((obstacle_velocity_x)*sin(theta) * (-1) + (obstacle_velocity_y)*cos(theta) * (-1));
        iter->fused_velocity_y = vehicle.velocity_y + ((obstacle_velocity_x)*cos(theta) + (obstacle_velocity_y)*sin(theta) * (-1));

        iter->fused_heading_angle = vehicle.heading_angle + iter->fused_heading_angle;

        // 장애물 데이터 오버플로우 방지
        if (iter->fused_position_x < 0)
            iter->fused_position_x = 0;
        else if (iter->fused_position_x >= map_x)
            iter->fused_position_x = map_x - 1;

        if (iter->fused_position_y < 0)
            iter->fused_position_y = 0;
        else if (iter->fused_position_y >= map_y)
            iter->fused_position_y = map_y - 1;

        // adcm::Log::Info() << iter->obstacle_class << " 장애물 relativeToMap 좌표변환 after (" << iter->fused_position_x << " , " << iter->fused_position_y << " , " << iter->fused_velocity_x << " , " << iter->fused_velocity_y << ")";
    }
}

// 레이캐스팅 대체 삼각형 내부포함 판단
bool isPointInTriangle(const Point2D &pt, const Point2D &v1, const Point2D &v2, const Point2D &v3)
{
    // 벡터 방식으로 barycentric 판별
    double dX = pt.x - v3.x;
    double dY = pt.y - v3.y;
    double dX21 = v2.x - v1.x;
    double dY21 = v2.y - v1.y;
    double dX31 = v3.x - v1.x;
    double dY31 = v3.y - v1.y;

    double denominator = dY21 * dX31 - dX21 * dY31;

    // 삼각형 넓이가 0인 경우 예외 처리
    if (denominator == 0)
        return false;

    double a = (dY21 * dX - dX21 * dY) / denominator;
    double b = (dY31 * dX - dX31 * dY) / -denominator;
    double c = 1.0 - a - b;

    return (a >= 0) && (b >= 0) && (c >= 0);
}

// 기존 레이캐스팅 기반
void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, VehicleData &vehicle)
{
    Point2D points[] = {p0, p1, p2, p3};

    // 4개 지점 좌표의 최소값 최댓값 계산
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
    // 레이캐스팅 알고리즘
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
                    // 교차점을 구한다
                    double meetX = (points[j].x - points[i].x) * (index.y - points[i].y) /
                                       (points[j].y - points[i].y) +
                                   points[i].x;
                    // 교차점 meetX 가 검증을 진행하는 인덱스의 x 좌표보다 크면 교차발생 cross++
                    if (index.x < meetX)
                        cross++;
                }
            }
            // 교차횟수 cross가 짝수이면 점은 외부, 홀수면 점은 내부에 있음
            if (cross % 2 != 0)
            {
                vehicle.map_2d_location.push_back(index);
                // road_z 코드 -> road_index로 수정 예정
                // mapData.map_2d[index.x][index.y].road_z = 1;
            }
        }
    }
}

/*
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
*/

/* barycentric 알고리즘 기반
void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, VehicleData &vehicle)
{
    Point2D points[] = {p0, p1, p2, p3};

    // AABB 계산 (정수로 내림/올림 처리)
    int min_x = static_cast<int>(std::floor(std::min({p0.x, p1.x, p2.x, p3.x})));
    int max_x = static_cast<int>(std::ceil(std::max({p0.x, p1.x, p2.x, p3.x})));
    int min_y = static_cast<int>(std::floor(std::min({p0.y, p1.y, p2.y, p3.y})));
    int max_y = static_cast<int>(std::ceil(std::max({p0.y, p1.y, p2.y, p3.y})));

    for (int x = min_x; x <= max_x; ++x)
    {
        for (int y = min_y; y <= max_y; ++y)
        {
            Point2D pt = {static_cast<double>(x), static_cast<double>(y)};
            // 삼각형 2개 중 하나라도 포함되면 vehicle 영역에 추가
            if (isPointInTriangle(pt, p0, p1, p2) ||
                isPointInTriangle(pt, p0, p2, p3))
            {
                vehicle.map_2d_location.push_back({x, y});
            }
        }
    }
}
*/

void generateOccupancyIndex(Point2D p0, Point2D p1, Point2D p2, Point2D p3, std::vector<ObstacleData>::iterator iter)
{
    if (iter == std::vector<ObstacleData>::iterator())
        return; // 유효하지 않은 iterator 방지

    Point2D points[] = {p0, p1, p2, p3};

    // AABB 계산
    int min_x = static_cast<int>(std::floor(std::min({p0.x, p1.x, p2.x, p3.x})));
    int max_x = static_cast<int>(std::ceil(std::max({p0.x, p1.x, p2.x, p3.x})));
    int min_y = static_cast<int>(std::floor(std::min({p0.y, p1.y, p2.y, p3.y})));
    int max_y = static_cast<int>(std::ceil(std::max({p0.y, p1.y, p2.y, p3.y})));

    for (int x = min_x; x <= max_x; ++x)
    {
        for (int y = min_y; y <= max_y; ++y)
        {
            Point2D pt = {static_cast<double>(x), static_cast<double>(y)};
            // 삼각형 2개 중 하나에 포함되면 내부
            if (isPointInTriangle(pt, p0, p1, p2) ||
                isPointInTriangle(pt, p0, p2, p3))
            {
                iter->map_2d_location.push_back({x, y});
                // map_2d_test[x][y].obstacle_id = iter->obstacle_id;
                // map_2d_test[x][y].road_z = 1; // TODO: 장애물 마킹
            }
        }
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
        half_x = main_vehicle_size.length / 2;
        half_y = main_vehicle_size.width / 2;
    }

    else
    {
        half_x = sub_vehicle_size.front().length / 2;
        half_y = sub_vehicle_size.front().width / 2;
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
    }
}

// 유클리디안 거리 계산
double euclideanDistance(const ObstacleData &a, const ObstacleData &b)
{
    return std::sqrt(std::pow(a.fused_position_x - b.fused_position_x, 2) +
                     std::pow(a.fused_position_y - b.fused_position_y, 2));
}

bool isStaticObstacle(const ObstacleData &obstacle)
{
    return obstacle.obstacle_class == 30 || obstacle.obstacle_class == 40;
}

static double getMatchThreshold(const ObstacleData &obstacle)
{
    return isStaticObstacle(obstacle) ? STATIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD : DYNAMIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD;
}

void splitObstaclesByType(const std::vector<ObstacleData> &input,
                          std::vector<ObstacleData> &statics,
                          std::vector<ObstacleData> &dynamics)
{
    statics.clear();
    dynamics.clear();
    for (const auto &obs : input)
    {
        if (isStaticObstacle(obs))
            statics.push_back(obs);
        else
            dynamics.push_back(obs);
    }
}

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
            double dist = euclideanDistance(listA[i], listB[j]);
            distanceMatrix[i][j] = (dist <= maxDistance) ? dist : HUNGARIAN_MAX_COST;
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

/**
 * @brief Temporal fusion: 이전 프레임 장애물과 현재 프레임 매칭
 * @param presList 현재 프레임 장애물 리스트 (입출력)
 * @param prevList 이전 프레임 장애물 리스트
 * @param assignment Munkres 알고리즘 매칭 결과
 */
void processFusion(
    std::vector<ObstacleData> &presList,
    const std::vector<ObstacleData> &prevList,
    const std::vector<int> &assignment)
{
    std::vector<ObstacleData> newList;
    newList.reserve(presList.size() + prevList.size());

    // 이전 프레임에서 이미 사용한 장애물 인덱스 추적
    std::vector<bool> prevUsed(prevList.size(), false);

    // 매칭 시도 헬퍼 람다
    auto tryAssignPrev = [&](size_t currentIdx, int prevIdx) -> bool {
        if (prevIdx < 0 || static_cast<size_t>(prevIdx) >= prevList.size() || prevUsed[prevIdx])
            return false;

        const auto &prevObstacle = prevList[prevIdx];
        auto &currentObstacle = presList[currentIdx];

        bool classMatched = prevObstacle.obstacle_class == currentObstacle.obstacle_class;
        double distance = euclideanDistance(prevObstacle, currentObstacle);
        double threshold = getMatchThreshold(prevObstacle);

        if (classMatched && distance <= threshold)
        {
            currentObstacle.obstacle_id = prevObstacle.obstacle_id;
            currentObstacle.stop_count = 0; // 이번 프레임에 관측됨
            prevUsed[prevIdx] = true;
            return true;
        }
        return false;
    };

    // === 1단계: Munkres 알고리즘 매칭 결과 적용 ===
    for (size_t i = 0; i < assignment.size(); ++i)
    {
        int j = assignment[i];
        if (j >= 0 && tryAssignPrev(i, j))
        {
            newList.push_back(presList[i]);
        }
    }

    // === 2단계: 미매칭 항목에 대한 보완 매칭 (거리 + 클래스 기반) ===
    for (size_t i = 0; i < presList.size(); ++i)
    {
        if (i < assignment.size() && assignment[i] >= 0 && prevUsed[assignment[i]])
        {
            continue; // 1단계에서 이미 매칭됨
        }

        auto &currentObstacle = presList[i];
        int bestIdx = -1;
        double bestDist = std::numeric_limits<double>::max();

        for (size_t k = 0; k < prevList.size(); ++k)
        {
            if (prevUsed[k])
                continue;
            if (prevList[k].obstacle_class != currentObstacle.obstacle_class)
                continue;

            double dist = euclideanDistance(prevList[k], currentObstacle);
            double threshold = getMatchThreshold(prevList[k]);
            if (dist <= threshold && dist < bestDist)
            {
                bestDist = dist;
                bestIdx = static_cast<int>(k);
            }
        }

        if (bestIdx >= 0 && tryAssignPrev(i, bestIdx))
        {
            newList.push_back(currentObstacle);
        }
        else
        {
            currentObstacle.obstacle_id = id_manager.allocID();
            currentObstacle.stop_count = 0;
            newList.push_back(currentObstacle);
        }
    }

    // === 3단계: 미관측 장애물 처리 (carry-over with stop_count 증가) ===
    std::size_t carriedCount = 0;
    std::size_t droppedCount = 0;
    
    for (size_t k = 0; k < prevList.size(); ++k)
    {
        if (prevUsed[k])
            continue; // 이미 매칭된 장애물은 제외

        ObstacleData unobservedObstacle = prevList[k];
        
        // stop_count 증가 (오버플로우 방지)
        if (unobservedObstacle.stop_count < std::numeric_limits<std::uint8_t>::max())
        {
            unobservedObstacle.stop_count++;
        }

        // 임계값 이하면 유지, 초과하면 제거
        if (unobservedObstacle.stop_count <= STOP_COUNT_REMOVE_THRESHOLD)
        {
            newList.push_back(unobservedObstacle);
            ++carriedCount;
        }
        else
        {
            id_manager.retID(unobservedObstacle.obstacle_id);
            ++droppedCount;
        }
    }

    // 통계 로그 출력
    if (carriedCount > 0 || droppedCount > 0)
    {
        adcm::Log::Info() << "⏳ Carry-over: kept=" << carriedCount << " dropped=" << droppedCount;
    }

    presList = newList;
}

/**
 * @brief 장애물 리스트에서 차량 클래스(특장차, 보조차량) 필터링
 * @param obstacles 장애물 리스트 (입출력)
 */
void filterVehicleData(std::vector<ObstacleData> &obstacles)
{
    const std::uint8_t SPECIAL_VEHICLE_CLASS = 50;
    const std::uint8_t AUXILIARY_VEHICLE_CLASS = 51;
    
    obstacles.erase(
        std::remove_if(obstacles.begin(), obstacles.end(),
            [](const ObstacleData& obs) {
                return obs.obstacle_class == 50 || obs.obstacle_class == 51;
            }),
        obstacles.end()
    );

    return;
}

/**
 * @brief Static 장애물 위치 스무딩 (이동 평균)
 * @param track 트랙 정보
 */
void ObstacleTracker::smoothStaticPosition(Track &track)
{
    if (track.history.positions.empty())
        return;

    // 히스토리 위치의 평균 계산
    double sumX = 0.0;
    double sumY = 0.0;
    for (const auto &pos : track.history.positions)
    {
        sumX += pos.x;
        sumY += pos.y;
    }

    const size_t historySize = track.history.positions.size();
    track.data.fused_position_x = sumX / historySize;
    track.data.fused_position_y = sumY / historySize;
}

/**
 * @brief Static 트랙 업데이트 및 pruning
 * @param detections 현재 프레임 detection 리스트
 * @param output 출력 장애물 리스트
 */
void ObstacleTracker::updateStaticTracks(const std::vector<ObstacleData> &detections, std::vector<ObstacleData> &output)
{
    std::vector<bool> matchedFlags(staticTracks_.size(), false);

    // === Detection과 기존 트랙 매칭 ===
    for (const auto &det : detections)
    {
        auto it = std::find_if(staticTracks_.begin(), staticTracks_.end(),
                               [&det](const Track &track) {
                                   return track.data.obstacle_id == det.obstacle_id;
                               });

        if (it != staticTracks_.end())
        {
            // 기존 트랙 업데이트
            it->data = det; // stop_count 포함하여 전체 데이터 업데이트
            it->unmatchedFrames = 0;
            
            // 위치 히스토리 업데이트
            it->history.positions.push_back({det.fused_position_x, det.fused_position_y});
            if (it->history.positions.size() > STATIC_OBSTACLE_HISTORY_WINDOW)
            {
                it->history.positions.pop_front();
            }
            
            matchedFlags[std::distance(staticTracks_.begin(), it)] = true;
        }
        else
        {
            // 새 트랙 생성
            Track newTrack;
            newTrack.data = det;
            newTrack.history.positions.push_back({det.fused_position_x, det.fused_position_y});
            staticTracks_.push_back(newTrack);
            matchedFlags.push_back(true);
        }
    }

    // === 미매칭 트랙 처리 및 Pruning ===
    std::vector<Track> survivingTracks;
    survivingTracks.reserve(staticTracks_.size());

    for (size_t idx = 0; idx < staticTracks_.size(); ++idx)
    {
        Track& track = staticTracks_[idx];
        
        // 미매칭 시 카운터 증가
        if (!matchedFlags[idx])
        {
            track.unmatchedFrames++;
            if (track.data.stop_count < std::numeric_limits<std::uint8_t>::max())
            {
                track.data.stop_count++;
            }
        }
        
        // Static 장애물 위치 스무딩 적용
        smoothStaticPosition(track);

        // Pruning 조건 체크 (두 임계값 모두 충족해야 유지)
        const bool withinUnmatchedLimit = track.unmatchedFrames <= STATIC_OBSTACLE_MAX_UNMATCHED_FRAMES;
        const bool withinStopCountLimit = track.data.stop_count <= STOP_COUNT_REMOVE_THRESHOLD;
        
        if (withinUnmatchedLimit && withinStopCountLimit)
        {
            survivingTracks.push_back(track);
            output.push_back(track.data);
        }
        else
        {
            // 트랙 제거 시 ID 반환
            id_manager.retID(track.data.obstacle_id);
        }
    }

    staticTracks_.swap(survivingTracks);
}

/**
 * @brief Dynamic 트랙 업데이트 및 pruning
 * @param detections 현재 프레임 detection 리스트
 * @param output 출력 장애물 리스트
 */
void ObstacleTracker::updateDynamicTracks(const std::vector<ObstacleData> &detections, std::vector<ObstacleData> &output)
{
    std::vector<bool> matchedFlags(dynamicTracks_.size(), false);

    // === Detection과 기존 트랙 매칭 ===
    for (const auto &det : detections)
    {
        auto it = std::find_if(dynamicTracks_.begin(), dynamicTracks_.end(),
                               [&det](const Track &track) {
                                   return track.data.obstacle_id == det.obstacle_id;
                               });

        if (it != dynamicTracks_.end())
        {
            // 기존 트랙 업데이트
            it->data = det; // stop_count 포함하여 전체 데이터 업데이트
            it->unmatchedFrames = 0;
            matchedFlags[std::distance(dynamicTracks_.begin(), it)] = true;
        }
        else
        {
            // 새 트랙 생성
            Track newTrack;
            newTrack.data = det;
            dynamicTracks_.push_back(newTrack);
            matchedFlags.push_back(true);
        }
    }

    // === 미매칭 트랙 처리 및 Pruning ===
    std::vector<Track> survivingTracks;
    survivingTracks.reserve(dynamicTracks_.size());

    for (size_t idx = 0; idx < dynamicTracks_.size(); ++idx)
    {
        Track& track = dynamicTracks_[idx];
        
        // 미매칭 시 카운터 증가
        if (!matchedFlags[idx])
        {
            track.unmatchedFrames++;
            if (track.data.stop_count < std::numeric_limits<std::uint8_t>::max())
            {
                track.data.stop_count++;
            }
        }

        // Pruning 조건 체크 (두 임계값 모두 충족해야 유지)
        const bool withinUnmatchedLimit = track.unmatchedFrames <= DYNAMIC_OBSTACLE_MAX_UNMATCHED_FRAMES;
        const bool withinStopCountLimit = track.data.stop_count <= STOP_COUNT_REMOVE_THRESHOLD;
        
        if (withinUnmatchedLimit && withinStopCountLimit)
        {
            survivingTracks.push_back(track);
            output.push_back(track.data);
        }
        else
        {
            // 트랙 제거 시 ID 반환
            id_manager.retID(track.data.obstacle_id);
        }
    }

    dynamicTracks_.swap(survivingTracks);
}

std::vector<ObstacleData> ObstacleTracker::update(const std::vector<ObstacleData> &detections)
{
    std::vector<ObstacleData> staticDetections;
    std::vector<ObstacleData> dynamicDetections;
    splitObstaclesByType(detections, staticDetections, dynamicDetections);

    std::vector<ObstacleData> output;
    output.reserve(staticDetections.size() + dynamicDetections.size() + staticTracks_.size() + dynamicTracks_.size());

    updateStaticTracks(staticDetections, output);
    updateDynamicTracks(dynamicDetections, output);

    adcm::Log::Info() << "Tracker update: detections=" << detections.size()
                      << " staticTracks=" << staticTracks_.size()
                      << " dynamicTracks=" << dynamicTracks_.size()
                      << " output=" << output.size();

    return output;
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
            adcm::Log::Info() << "장애물 ID " << obs.obstacle_id << " 정지 카운트: " << obs.stop_count;
        }

        else
        {
            obs.stop_count = 0; // 움직이면 카운트 리셋
            adcm::Log::Info() << "장애물 ID " << obs.obstacle_id << " 정지 카운트 초기화(움직임): " << obs.stop_count;
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
    // adcm::Log::Info() << "장애물 map_2d_location 사이즈: " << obstacle.map_2d_location.size();
    // map_2d_size += obstacle.map_2d_location.size() * sizeof(Point2D);
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
    // adcm::Log::Info() << "장애물 ID " << obstacle.obstacle_id << " 크기: [" << obstacle.fused_cuboid_x << ", " << obstacle.fused_cuboid_y << ", " << obstacle.fused_cuboid_z << "]";
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
    mapData.road_list.clear();
    
    for (const auto &obstacle : obstacle_list)
    {
        mapData.obstacle_list.push_back(ConvertToObstacleListStruct(obstacle, mapData.map_2d));
    }
    adcm::obstacleListStruct obstacle_fense;
    obstacle_fense.obstacle_id = 111111;
    obstacle_fense.obstacle_class = 99; // 가상 장애물
    obstacle_fense.timestamp = obstacle_list.empty() ? 0 : obstacle_list.front().timestamp;
    obstacle_fense.stop_count = 0;
    obstacle_fense.fused_cuboid_x = 1;
    obstacle_fense.fused_cuboid_y = 1;
    obstacle_fense.fused_cuboid_z = 1;
    obstacle_fense.fused_heading_angle = 0;
    obstacle_fense.fused_position_x = 0;
    obstacle_fense.fused_position_y = 0;
    obstacle_fense.fused_position_z = 0;
    obstacle_fense.fused_velocity_x = 0;
    obstacle_fense.fused_velocity_y = 0;
    obstacle_fense.fused_velocity_z = 0;

    for (int x = 200; x <= 500; ++x)
    {
        obstacle_fense.map_2d_location.push_back({x, 500});
    }

    mapData.obstacle_list.push_back(obstacle_fense);


    for (const auto &vehicle : vehicles)
    {
        if (vehicle->vehicle_class != 0)
        {
            mapData.vehicle_list.push_back(ConvertToVehicleListStruct(*vehicle, mapData.map_2d));
            if (vehicle->road_z.size() > 1) // 시뮬레이션에선 road_z size가 1이므로 예외 처리
            {
                /** 두대 노면 데이터가 다를때 어떻게 반영할지? 0930_테스트 */
                std::vector<adcm::roadListStruct> road_list = ConvertRoadZToRoadList(*vehicle);

                mapData.road_list.insert(mapData.road_list.end(), road_list.begin(), road_list.end());
            }
            else
                adcm::Log::Info() << vehicle->vehicle_class << " 차량 road_z 데이터 없음, road_list 반영 X";
        }
    }

    adcm::Log::Info() << "mapData 장애물 반영 완료 개수: " << mapData.obstacle_list.size();
    adcm::Log::Info() << "mapData 차량 반영 완료 개수: " << mapData.vehicle_list.size();
    adcm::Log::Info() << "mapData road_list 반영 완료 개수: " << mapData.road_list.size();
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
    adcm::Log::Info() << "차량 반영 road_z size: " << data->road_z.size();
    return;
}

// 장애물 데이터 저장
void fillObstacleList(std::vector<ObstacleData> &obstacle_list_fill, const std::shared_ptr<adcm::hub_data_Objects> &data)
{
    obstacle_list_fill.clear();
    adcm::Log::Info() << "fillObstacleList 함수 시작: data->obstacle.size() = " << data->obstacle.size();
    
    int idx = 0;
    for (const auto &obstacle : data->obstacle)
    {
        // adcm::Log::Info() << "  [" << idx << "] obstacle_id: " << obstacle.obstacle_id << ", class: " << obstacle.obstacle_class;
        
        ObstacleData obstacle_to_push;
        obstacle_to_push.obstacle_class = obstacle.obstacle_class;
        obstacle_to_push.obstacle_id = 0; // ignore incoming IDs
        obstacle_to_push.timestamp = data->timestamp;
        obstacle_to_push.fused_cuboid_x = obstacle.cuboid_x;
        obstacle_to_push.fused_cuboid_y = obstacle.cuboid_y;
        obstacle_to_push.fused_cuboid_z = obstacle.cuboid_z;
        // adcm::Log::Info() << "장애물 크기: " << obstacle.cuboid_x << ", " << obstacle.cuboid_y << ", " << obstacle.cuboid_z;
        obstacle_to_push.fused_heading_angle = obstacle.heading_angle;
        obstacle_to_push.fused_position_x = obstacle.position_x;
        obstacle_to_push.fused_position_y = obstacle.position_y;
        obstacle_to_push.fused_position_z = obstacle.position_z;
        obstacle_to_push.fused_velocity_x = obstacle.velocity_x;
        obstacle_to_push.fused_velocity_y = obstacle.velocity_y;
        obstacle_to_push.fused_velocity_z = obstacle.velocity_z;
        obstacle_to_push.stop_count = 0;
        obstacle_list_fill.push_back(obstacle_to_push);
        idx++;
    }
    adcm::Log::Info() << "fillObstacleList 함수 종료: 총 " << obstacle_list_fill.size() << "개 장애물 추가됨";
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

                // 수신된 데이터 handling 위한 추가 코드
                adcm::Log::Info() << "수신 데이터: " << data->vehicle_class;
                adcm::Log::Info() << "원본 data->obstacle 크기 (파싱 전): " << data->obstacle.size();

                FusionData fusionData;
                fillVehicleData(fusionData.vehicle, data);
                fillObstacleList(fusionData.obstacle_list, data);
                
                adcm::Log::Info() << "파싱 후 fusionData.obstacle_list 크기: " << fusionData.obstacle_list.size();

                if (data->road_z.size() != 0)
                {
                    adcm::Log::Info() << "road_z size: " << data->road_z.size();
                    // for (int i = 0; i < data->road_z.size(); i++)
                    //     adcm::Log::Info() << i << "번째 road_z: " << data->road_z[i];
                }
                else
                    adcm::Log::Info() << "road_z empty";
                // road_index에 반영 필요

                switch (data->vehicle_class)
                {
                case EGO_VEHICLE:
                    main_vehicle_data = fusionData;
                    order.push(EGO_VEHICLE);
                    ego = true;
                    adcm::Log::Info() << "[EGO_VEHICLE(0xF0)] 데이터 할당: obstacle_list size = " << fusionData.obstacle_list.size();
                    break;

                case SUB_VEHICLE_1:
                    sub1_vehicle_data = fusionData;
                    order.push(SUB_VEHICLE_1);
                    sub1 = true;
                    adcm::Log::Info() << "[SUB_VEHICLE_1(0x01)] 데이터 할당: obstacle_list size = " << fusionData.obstacle_list.size() << ", sub1 flag = true";
                    break;

                case SUB_VEHICLE_2:
                    sub2_vehicle_data = fusionData;
                    order.push(SUB_VEHICLE_2);
                    sub2 = true;
                    adcm::Log::Info() << "[SUB_VEHICLE_2(0x02)] 데이터 할당: obstacle_list size = " << fusionData.obstacle_list.size() << ", sub2 flag = true";
                    break;

                case SUB_VEHICLE_3:
                    sub3_vehicle_data = fusionData;
                    order.push(SUB_VEHICLE_3);
                    sub3 = true;
                    adcm::Log::Info() << "[SUB_VEHICLE_3(0x03)] 데이터 할당: obstacle_list size = " << fusionData.obstacle_list.size() << ", sub3 flag = true";
                    break;

                case SUB_VEHICLE_4:
                    sub4_vehicle_data = fusionData;
                    order.push(SUB_VEHICLE_4);
                    sub4 = true;
                    adcm::Log::Info() << "[SUB_VEHICLE_4(0x04)] 데이터 할당: obstacle_list size = " << fusionData.obstacle_list.size() << ", sub4 flag = true";
                    break;

                default:
                    adcm::Log::Info() << "[Unknown vehicle class: 0x" << std::hex << (int)data->vehicle_class << std::dec << "] 데이터 무시됨";
                    continue; // 미확인 데이터는 처리하지 않고 다음으로 넘어감
                }
            }
            dataReady.notify_one();
            adcm::Log::Info() << ++receiveVer << "번째 허브 데이터 수신 완료";

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

            main_vehicle_size.length = data->main_vehicle.length / 100.0;
            main_vehicle_size.width = data->main_vehicle.width / 100.0;
            if (main_vehicle_size.length != 0) // 메인차량이 있다면 workego = true
            {
                workego = true;
                adcm::Log::Info() << "[WorkInfo] 메인차량 길이: " << main_vehicle_size.length << ", 폭: " << main_vehicle_size.width;
            }

            sub_vehicle_size.clear();
            for (const auto &sub_vehicle : data->sub_vehicle)
            {
                sub_vehicle_size.push_back({sub_vehicle.length / 100, sub_vehicle.width / 100});
            }
            if (sub_vehicle_size.size() >= 1) // 서브차량이 있다면 work상태 true
            {
                worksub1 = true;
                adcm::Log::Info() << "[WorkInfo] 서브차량1 길이: " << sub_vehicle_size[0].length << ", 폭: " << sub_vehicle_size[0].width;
            }
            if (sub_vehicle_size.size() >= 2)
            {
                worksub2 = true;
                adcm::Log::Info() << "[WorkInfo] 서브차량2 길이: " << sub_vehicle_size[1].length << ", 폭: " << sub_vehicle_size[1].width;
            }
            if (sub_vehicle_size.size() >= 3)
            {
                worksub3 = true;
                adcm::Log::Info() << "[WorkInfo] 서브차량3 길이: " << sub_vehicle_size[2].length << ", 폭: " << sub_vehicle_size[2].width;
            }
            if (sub_vehicle_size.size() >= 4)
            {
                worksub4 = true;
                adcm::Log::Info() << "[WorkInfo] 서브차량4 길이: " << sub_vehicle_size[3].length << ", 폭: " << sub_vehicle_size[3].width;
            }

            adcm::Log::Info() << "[WorkInfo] workego: " << workego << ", worksub1: " << worksub1 << ", worksub2: " << worksub2 << ", worksub3: " << worksub3 << ", worksub4: " << worksub4;
            work_boundary.clear();
            for (const auto &boundary : data->working_area_boundary)
            {
                work_boundary.push_back({boundary.x, boundary.y});
            }

            type = data->type;

            // if (!type) // 시뮬레이션이라면, (126.5482, 35.9398)의 utm좌표가 맵의 (0, 0)이 된다.
            // {
            //     origin_x = 278835;
            //     origin_y = 3980050;
            //     map_x = 2000;
            //     map_y = 1000;
            //     adcm::Log::Info() << "[WorkInfo] 시뮬레이션 테스트";
            //     adcm::Log::Info() << "맵 사이즈: (" << map_x << ", " << map_y << ")";

            // }
            // else // 실증이라면, boundary 좌표의 가장 작은 지점 min_x, min_y의 utm좌표가 맵의 (0, 0)이 된다.
            // {
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
            adcm::Log::Info() << "[WorkInfo] 실증 테스트";
            adcm::Log::Info() << "map의 min(lon, lat) 값: (" << min_lon << ", " << min_lat << "), max(lon, lat) 값 : (" << max_lon << ", " << max_lat << ")";

            GPStoUTM(min_lon, min_lat, min_utm_x, min_utm_y);
            GPStoUTM(max_lon, max_lat, max_utm_x, max_utm_y);
            adcm::Log::Info() << "map의 minutm(x, y) 값: (" << min_utm_x << ", " << min_utm_y << "), maxutm(x, y) 값 : (" << max_utm_x << ", " << max_utm_y << ")";
            map_x = (max_utm_x - min_utm_x) * 10;
            map_y = (max_utm_y - min_utm_y) * 10;
            adcm::Log::Info() << "맵 사이즈: (" << map_x << ", " << map_y << ")";
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
    std::vector<adcm::map_2dListVector> map_2d_init(map_x, adcm::map_2dListVector(map_y, map_2dStruct_init));
    std::vector<adcm::map_2dListVector> map_2d_test(1, adcm::map_2dListVector(1, map_2dStruct_init));

    mapData.map_2d = map_2d_test;

    std::vector<ObstacleData> obstacle_list;

    adcm::map_2dListVector map_2dListVector;
    while (continueExecution)
    {
        // 수신한 허브 데이터가 없으면 송신 X
        std::chrono::high_resolution_clock::time_point fusion_startTime, fusion_endTime;
        
        adcm::Log::Info() << "Wait Hub Data";
        {
            unique_lock<mutex> lock(mtx_data);
            dataReady.wait(lock, []
                           { return (get_workinfo && ((!workego || ego) || ((!worksub1 || sub1) && (!worksub2 || sub2) && (!worksub3 || sub3) && (!worksub4 || sub4)))); });

            // adcm::Log::Info() << "송신이 필요한 남은 허브 데이터 개수: " << main_vehicle_queue.size_approx() + sub1_vehicle_queue.size_approx() + sub2_vehicle_queue.size_approx();
            adcm::Log::Info() << "==============KATECH modified code start==========";

            // Check order queue is not empty before accessing front()
            if (order.empty())
            {
                adcm::Log::Error() << "ERROR: order queue is empty! Skipping this cycle.";
                ego = false;
                sub1 = false;
                sub2 = false;
                sub3 = false;
                sub4 = false;
                continue;
            }

            adcm::Log::Info() << "KATECH: 이번 데이터 기준 차량: " << order.front();
            //==============1. 차량 및 장애물 데이터 위치 변환=================

            if (workego && ego) {
                processVehicleData(main_vehicle_data, main_vehicle, obstacle_list_main);
                adcm::Log::Info() << "[EGO_VEHICLE] 처리: obstacle_list_main size = " << obstacle_list_main.size();
            }
            if (worksub1 && sub1) {
                processVehicleData(sub1_vehicle_data, sub1_vehicle, obstacle_list_sub1);
                adcm::Log::Info() << "[SUB_VEHICLE_1] 처리: obstacle_list_sub1 size = " << obstacle_list_sub1.size();
            }
            if (worksub2 && sub2) {
                processVehicleData(sub2_vehicle_data, sub2_vehicle, obstacle_list_sub2);
                adcm::Log::Info() << "[SUB_VEHICLE_2] 처리: obstacle_list_sub2 size = " << obstacle_list_sub2.size();
            }
            if (worksub3 && sub3) {
                processVehicleData(sub3_vehicle_data, sub3_vehicle, obstacle_list_sub3);
                adcm::Log::Info() << "[SUB_VEHICLE_3] 처리: obstacle_list_sub3 size = " << obstacle_list_sub3.size();
            }
            if (worksub4 && sub4) {
                processVehicleData(sub4_vehicle_data, sub4_vehicle, obstacle_list_sub4);
                adcm::Log::Info() << "[SUB_VEHICLE_4] 처리: obstacle_list_sub4 size = " << obstacle_list_sub4.size();
            }
            
            adcm::Log::Info() << "차량 및 장애물 좌표계 변환 완료";

            // ==============2. 장애물 데이터 융합 / 3. 특장차 및 보조차량 제거 / 4. 장애물 ID 부여 =================
            fusion_startTime = std::chrono::high_resolution_clock::now();
            
            adcm::Log::Info() << "[mergeAndCompareLists] Before calling: main=" << obstacle_list_main.size() 
                              << ", sub1=" << obstacle_list_sub1.size() 
                              << ", sub2=" << obstacle_list_sub2.size() 
                              << ", sub3=" << obstacle_list_sub3.size() 
                              << ", sub4=" << obstacle_list_sub4.size();
            
            obstacle_list = mergeAndCompareLists(previous_obstacle_list, obstacle_list_main, obstacle_list_sub1,
                                                 obstacle_list_sub2, obstacle_list_sub3, obstacle_list_sub4,
                                                 main_vehicle, sub1_vehicle, sub2_vehicle, sub3_vehicle, sub4_vehicle);

            adcm::Log::Info() << "mergeAndCompareLists 호출 후: 최종 obstacle_list size = " << obstacle_list.size();

            previous_obstacle_list = obstacle_list;
            
            // Pop order queue and reset flags while still holding lock
            if (!order.empty())
            {
                order.pop();
            }
            else
            {
                adcm::Log::Error() << "ERROR: order queue is empty at pop time! This should not happen.";
            }
            
            ego = false;
            sub1 = false;
            sub2 = false;
            sub3 = false;
            sub4 = false;
        } // lock 해제

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

        if (result)
        { // execute only if all true!
            /*
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
            
            fusion_endTime = std::chrono::high_resolution_clock::now();
            auto fusion_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(fusion_endTime - fusion_startTime).count();
            adcm::Log::Info() << "[FUSION_TIME] 맵 융합 및 생성 소요 시간: " << fusion_elapsed_ms << " ms";

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

            adcm::Log::Info() << "mapdata 융합 완료";
        }
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
        long long wait_ms = 0;
        
        {
            unique_lock<mutex> lock(mtx_map_someip);
            someipReady.wait(lock, []
                             { return sendEmptyMap == true ||
                                      !map_someip_queue.empty(); });
            
            // Timer starts after data is ready
            auto startTime = std::chrono::high_resolution_clock::now();
            
            if (sendEmptyMap)
            {
                mapData_provider.send(mapData);
                adcm::Log::Info() << "Send empty map data";
                sendEmptyMap = false;
                continue;
            }

            adcm::map_data_Objects tempMap = map_someip_queue.front();
            map_someip_queue.pop();
            adcm::Log::Info() << ++mapVer << "번째 로컬 mapdata 전송 시작";

            // map_data_object 의 생성시간 추가
            auto now = std::chrono::system_clock::now();
            auto mapData_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            adcm::Log::Info() << "Current timestamp in milliseconds: " << mapData_timestamp;
            tempMap.timestamp = mapData_timestamp;
            // json 저장
            if (saveJson)
                makeJSON(tempMap);
            // 맵전송
            // mapData.map_2d.clear(); // json 데이터 경량화를 위해 map_2d 삭제
            mapData_provider.send(tempMap);
            auto endTime = std::chrono::high_resolution_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

            adcm::Log::Info() << mapVer << "번째 로컬 mapdata 전송 완료, 소요 시간: " << elapsed_ms << " ms.";

            // Guarantee 300ms period from data arrival
            if (elapsed_ms < 300)
            {
                wait_ms = 300 - elapsed_ms;
            }
            
        } // Lock released here before sleep
        
        // Sleep OUTSIDE the lock to allow ThreadKatech to push new data
        if (wait_ms > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
            adcm::Log::Info() << wait_ms << "ms 만큼 전송 대기";
        }
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
            adcm::Log::Info() << "NATS 전송 시작";
            // mapData.map_2d.clear(); // json 데이터 경량화를 위해 map_2d 삭제
            NatsSend(mapData);
            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = endTime - startTime;
            adcm::Log::Info() << "NATS 전송 완료, 소요 시간: " << duration.count() << " ms.";
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
    adcm::Log::Info() << "Ok, let's produce some DataFusion data...";
    // adcm::Log::Info() << "SDK release_251209_interface v2.5 for sa8195";
    adcm::Log::Info() << "SDK release_251211_interface v2.5 for orin";
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