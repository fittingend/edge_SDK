#include "main_riskassessment.hpp"
static obstacleListVector obstacle_pedes_initial;   // 시나리오 5용 전역변수
static obstacleListVector obstacle_vehicle_initial; // 시나리오 6용 전역변수

void evaluateScenario1(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 1 START==============";

    obstacleListVector obstacle_stop;
    double distance_scenario_1;
    double confidence_scenario_1;

    // i) 정지상태 판정: STOP_VALUE 이상 정지 + 구조물/보행자 제외
    for (const auto& obstacle : obstacle_list)
    {
        if ((obstacle.stop_count >= STOP_VALUE) && 
            (obstacle.obstacle_class != STRUCTURE) && 
            (obstacle.obstacle_class != PEDESTRIAN))
        {
            adcm::Log::Info() << "시나리오1-i) 장애물 정지 상태 감지";
            obstacle_stop.push_back(obstacle);
        }
    }

    // ii) 특장차로부터 30m 이내 장애물 필터링
    for (auto iter = obstacle_stop.begin(); iter != obstacle_stop.end(); )
    {
        double distance_ego_obs = getDistance(*iter, ego_vehicle);
        adcm::Log::Info() << "distance_ego_obs: " << distance_ego_obs;

        if (distance_ego_obs >= 300)  // 30m 이상
        {
            adcm::Log::Info() << "시나리오1-ii) 특장차와 장애물 거리가 30m 이상이라 해당사항 없음";
            iter = obstacle_stop.erase(iter);
        }
        else
        {
            ++iter;
        }
    }

    // iii) 장애물과 전역경로간 거리 및 위험도 계산
    for (const auto& obstacle : obstacle_stop)
    {
        distance_scenario_1 = getDistance_LinearTrajectory(obstacle);
        confidence_scenario_1 = 200 / getDistance(obstacle, ego_vehicle) * 0.7;

        adcm::riskAssessmentStruct riskAssessment1;
        riskAssessment1.obstacle_id = obstacle.obstacle_id;
        riskAssessment1.hazard_class = SCENARIO_1;
        riskAssessment1.confidence = confidence_scenario_1;

        adcm::Log::Info() << "Risk assessment generated for #1: " 
                          << obstacle.obstacle_id 
                          << " with confidence: " 
                          << confidence_scenario_1;

        riskAssessment.riskAssessmentList.push_back(riskAssessment1);
    }

    adcm::Log::Info() << "scenario 1 DONE";

    clear_and_free_memory(obstacle_stop);
}

void evaluateScenario2(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 2 START==============";

    obstacleListVector obstacle_static_stop;
    double confidence_scenario_2;

    // i) 정지 상태 판정
    for (const auto& obs : obstacle_list) {
        if ((obs.stop_count >= STOP_VALUE && obs.obstacle_class != STRUCTURE && obs.obstacle_class != PEDESTRIAN) ||
            (obs.obstacle_class == STRUCTURE && obs.fused_cuboid_z > 1)) {
            obstacle_static_stop.push_back(obs);
        }
    }

    // ii) 40m 이내 필터링
    obstacle_static_stop.erase(
        std::remove_if(obstacle_static_stop.begin(), obstacle_static_stop.end(),
                       [&](const auto& obs) { return getDistance(obs, ego_vehicle) >= 400; }),
        obstacle_static_stop.end());

    // iii) 경로 근접 + 리스크 평가
    for (const auto& obs : obstacle_static_stop) {
        if (getDistance_LinearTrajectory(obs) > 0) {
            confidence_scenario_2 = 200 / getDistance(obs, ego_vehicle) * 0.7;
            riskAssessment.riskAssessmentList.push_back({obs.obstacle_id, SCENARIO_2, confidence_scenario_2});
        }
    }

    adcm::Log::Info() << "scenario 2 DONE";
    clear_and_free_memory(obstacle_static_stop);
}
void evaluateScenario3(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 3 START==============";

    obstacleListVector obstacle_near_10_30;
    constexpr double SAFE_DISTANCE = 300.0;
    constexpr double TTC_THRESHOLD = 100.0;

    // i) 특장차로부터 10~30m 거리의 동적 객체 필터링
    for (const auto& obstacle : obstacle_list) {
        double distance_ego_obs = getDistance(obstacle, ego_vehicle);

        if (distance_ego_obs >= TTC_THRESHOLD && distance_ego_obs <= SAFE_DISTANCE &&
            obstacle.obstacle_class != STRUCTURE) 
        {
            adcm::Log::Info() << "시나리오3-i) 10~30m 동적객체 추출: " << obstacle.obstacle_id;
            obstacle_near_10_30.push_back(obstacle);
        }
    }

    // ii) TTC와 안전영역 진입 여부 평가
    for (const auto& obstacle : obstacle_near_10_30) {
        double ttc = getTTC(obstacle, ego_vehicle);
        double dist_approx = getLinearApprox(obstacle, ego_vehicle);

        if (ttc == INVALID_RETURN_VALUE || dist_approx == 0) {
            continue;  // 유효하지 않은 경우 skip
        }

        double ttc_confidence = (ttc > 0) ? (5.0 / ttc * 0.7) : 0.0;
        double area_confidence = (dist_approx > 0) ? (SAFE_DISTANCE / dist_approx * 0.7) : 0.0;

        if (ttc_confidence > 0 || area_confidence > 0) {
            double confidence = std::max(ttc_confidence, area_confidence);

            adcm::riskAssessmentStruct riskAssessment3;
            riskAssessment3.obstacle_id = obstacle.obstacle_id;
            riskAssessment3.hazard_class = SCENARIO_3;
            riskAssessment3.confidence = confidence;

            adcm::Log::Info() << "Risk assessment generated for #3: " 
                              << obstacle.obstacle_id 
                              << " with confidence: " << confidence;

            riskAssessment.riskAssessmentList.push_back(riskAssessment3);
        }
    }

    adcm::Log::Info() << "scenario 3 DONE";
    clear_and_free_memory(obstacle_near_10_30);
}

void evaluateScenario4(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 4 START==============";

    obstacleListVector obstacle_near_20_40;

    for (const auto& obs : obstacle_list) {
        double distance = getDistance(obs, ego_vehicle);
        if (distance > 200 && distance < 400 && obs.obstacle_class != STRUCTURE) {
            obstacle_near_20_40.push_back(obs);
        }
    }

    for (const auto& obs : obstacle_near_20_40) {
        double dist_approx = getLinearApprox(obs, ego_vehicle);
        double confidence = 400 / dist_approx * 0.7;

        riskAssessment.riskAssessmentList.push_back({obs.obstacle_id, SCENARIO_4, confidence});
    }

    adcm::Log::Info() << "scenario 4 DONE";
    clear_and_free_memory(obstacle_near_20_40);
}

void evaluateScenario5(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       const doubleVector& path_x, const doubleVector& path_y,
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 5 START==============";

    obstacleListVector obstacle_pedes_40_50, obstacle_pedes_new;

    for (const auto& obs : obstacle_list) {
        double distance = getDistance(obs, ego_vehicle);
        if (distance < 500 && obs.obstacle_class == PEDESTRIAN) {
            obstacle_pedes_40_50.push_back(obs);
        }
    }

    obstacle_pedes_40_50.erase(
        std::remove_if(obstacle_pedes_40_50.begin(), obstacle_pedes_40_50.end(),
                       [&](const auto& obs) { return getDistance_LinearTrajectory(obs, path_x, path_y) > 300; }),
        obstacle_pedes_40_50.end());

    if (obstacle_pedes_initial.empty()) {
        obstacle_pedes_initial = obstacle_pedes_40_50;
    } else {
        symmDiff(obstacle_pedes_initial, obstacle_pedes_40_50, obstacle_pedes_new);

        if (!obstacle_pedes_new.empty()) {
            uint64_t ori_max = getMaxTimestamp(obstacle_pedes_initial);
            uint64_t new_min = getMinTimestamp(obstacle_pedes_new);

            if (new_min - ori_max < 2000) {
                double max_dist = getMaxDistance(obstacle_pedes_initial, obstacle_pedes_new);
                if (max_dist < 400) {
                    for (const auto& obs : obstacle_pedes_new) {
                        double confidence = 300 / getDistance(obs, ego_vehicle) * 0.7;
                        riskAssessment.riskAssessmentList.push_back({obs.obstacle_id, SCENARIO_5, confidence});
                    }
                }
            }
        }
    }

    adcm::Log::Info() << "scenario 5 DONE";
    clear_and_free_memory(obstacle_pedes_40_50);
    clear_and_free_memory(obstacle_pedes_new);
}

//=====시나리오 #6. 주행 경로상 통행량이 과다한 환경(차량)=====
void evaluateScenario6(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       const doubleVector& path_x, const doubleVector& path_y,
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 6 START==============";
    obstacleListVector obstacle_vehicle_50_60, obstacle_vehicle_new;
    uint64_t ori_timestamp_max = 0
    uint64_t new_timestamp_min = std::numeric_limits<uint64_t>::max();
    double max_distance = 0;
    
    //=========i) 50~60m 사이의 차량 추출
    for (const auto& obs : obstacle_list) {
        double distance = getDistance(obs, ego_vehicle);
        if (distance > 500 && distance < 600 &&
            (obs.obstacle_class == VEHICLE_LARGE || obs.obstacle_class == VEHICLE_SMALL)) {
            obstacle_vehicle_50_60.push_back(obs);
        }
    }
    //=========ii) 주행경로 반경 15m 이내인지 계산
    obstacle_vehicle_50_60.erase(
        std::remove_if(obstacle_vehicle_50_60.begin(), obstacle_vehicle_50_60.end(),
                       [&](const auto& obs) { return getDistance_LinearTrajectory(obs, path_x, path_y) > 150; }),
        obstacle_vehicle_50_60.end());
    
    // 최초 실행시 장애물 리스트만 생성
    if (obstacle_vehicle_initial.empty()) {
        obstacle_vehicle_initial = obstacle_vehicle_50_60;
    } else {
        obstacle_vehicle_new = getNewObstacles(obstacle_vehicle_initial, obstacle_vehicle_50_60);

        if (!obstacle_vehicle_new.empty()) {
            uint64_t ori_max = getMaxTimestamp(obstacle_vehicle_initial);
            uint64_t new_min = getMinTimestamp(obstacle_vehicle_new);

            if (new_min - ori_max < 1500) {
                double max_dist = getMaxDistance(obstacle_vehicle_initial, obstacle_vehicle_new);
                if (max_dist < 600) {
                    for (const auto& obs : obstacle_vehicle_new) {
                        double ego_dist = getDistance(obs, ego_vehicle);
                        if (ego_dist < 40) {
                            double confidence = 400 / ego_dist * 0.7;
                            riskAssessment.riskAssessmentList.push_back({obs.obstacle_id, SCENARIO_6, confidence});
                        }
                    }
                }
            }
        }
    }

    adcm::Log::Info() << "scenario 6 DONE";
    clear_and_free_memory(obstacle_vehicle_50_60);
    clear_and_free_memory(obstacle_vehicle_new);
}

void evaluateScenario7(const doubleVector& path_x, 
                       const doubleVector& path_y, 
                       const std::vector<adcm::map_2dListVector>& map_2d, 
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 7 START==============";
    detectUnscannedPath(map_2d, path_x, path_y, riskAssessment);
    adcm::Log::Info() << "scenario 7 DONE";
}

void evaluateScenario8(const doubleVector& path_x, 
                       const doubleVector& path_y, 
                       const std::vector<adcm::map_2dListVector>& map_2d, 
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 8 START==============";

    constexpr int SHIFT_DISTANCE = 50;   // 전역경로 근방 5m 스캔
    constexpr int RISK_THRESHOLD = 20;   // 위험 카운트 임계값
    constexpr double VEHICLE_ROAD_Z = 0; // TODO: 실제 노면 정보로 수정 필요    
    Point2D p1, p2, p3, p4;

    for (size_t i = 0; i < path_x.size() - 1; ++i) {

        int x_start = static_cast<int>(floor(path_x[i]));
        int x_end   = static_cast<int>(floor(path_x[i + 1]));
        int y_start = static_cast<int>(floor(path_y[i]));
        int y_end   = static_cast<int>(floor(path_y[i + 1]));

        double original_m, original_c, up_c, down_c, x_up, x_down;
        bool isVertical;
        calculateShiftedLines(x_start, x_end, y_start, y_end, SHIFT_DISTANCE, 
                              original_m, original_c, up_c, down_c, isVertical, x_up, x_down);
        // Print the line equations
        if (isVertical) 
        {
            adcm::Log::Info()<< "Original line equation: x = " << x_start << "\n";
            adcm::Log::Info() << "Line shifted right: x = " << x_up << "\n";
            adcm::Log::Info() << "Line shifted left: x = " << x_down << "\n";

            p1 = {x_down, y_start};  // First line start point
            p2 = {x_down, y_end}; // First line end point
            p3 = {x_up, y_start};   // Second line start point
            p4 = {x_up, y_end}; // Second line end point
        } 
        else 
        {
            adcm::Log::Info() << "Original line equation: y = " << original_m << "x + " << original_c << "\n";
            adcm::Log::Info() << "Line shifted up: y = " << original_m << "x + " << up_c << "\n";
            adcm::Log::Info() << "Line shifted down: y = " << original_m << "x + " << down_c << "\n";

            // Points of the parallelogram (calculated for x = 0 and x = 10)
            p1 = {x_start, original_m * x_start + up_c};  // First line start point
            p2 = {x_end, original_m * x_end + up_c}; // First line end point
            p3 = {x_start, original_m * x_start + down_c};   // Second line start point
            p4 = {x_end, original_m * x_end + down_c}; // Second line end point
        }

        int minX = std::min({p1.x, p2.x, p3.x, p4.x});
        int maxX = std::max({p1.x, p2.x, p3.x, p4.x});
        int minY = std::min({p1.y, p2.y, p3.y, p4.y});
        int maxY = std::max({p1.y, p2.y, p3.y, p4.y});
        
        int risk_count = 0;
        //노면상태 스캔
        for (int x = minX; x <= maxX; ++x) {
            for (int y = minY; y <= maxY; ++y) {

                if (x < 0 || y < 0 || x >= map_2d.size() || y >= map_2d[0].size()) continue;

                if (std::abs(map_2d[x][y].road_z - VEHICLE_ROAD_Z) > WHEEL_DIAMETER_M / 2) {
                    risk_count++;
                }

                if (risk_count >= RISK_THRESHOLD) {
                    adcm::Log::Info() << "Road cave-in detected! Generating risk assessment.";

                    adcm::riskAssessmentStruct riskAssessment8;
                    adcm::globalPathPosition start_path{path_x[i], path_y[i]};
                    adcm::globalPathPosition end_path{path_x[i + 1], path_y[i + 1]};

                    riskAssessment8.wgs84_xy_start.push_back(start_path);
                    riskAssessment8.wgs84_xy_end.push_back(end_path);
                    riskAssessment8.hazard_class = SCENARIO_8;
                    riskAssessment8.isHarzard = true;

                    adcm::Log::Info() << "Risk assessment generated for #8 at X: " 
                                      << path_x[i] << ", Y: " << path_y[i];

                    riskAssessment.riskAssessmentList.push_back(riskAssessment8);
                    break;
                }
            }

            if (risk_count >= RISK_THRESHOLD) break;
        }
    }
    adcm::Log::Info() << "scenario 8 DONE";
}


