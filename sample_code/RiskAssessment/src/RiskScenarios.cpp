#include "main_riskassessment.hpp"

//=====시나리오 #5. 전역변수=====
static obstacleListVector ped_fix, ped_previous, ped_new_accumulated;
static int newPedFound;
static uint64_t init_ped_timestamp_max, new_ped_timestamp_min; 

//=====시나리오 #6. 전역변수 =====
static obstacleListVector veh_fix, veh_previous, veh_new_accumulated; 
static int newVehFound; 
static uint64_t init_veh_timestamp_max, new_veh_timestamp_min;


//=====시나리오 #1. 주행중 전역경로 근방 이동가능한 정지 장애물이 존재하는 위험 환경=====
void evaluateScenario1(const obstacleListVector& obstacle_list, 
                        const adcm::vehicleListStruct& ego_vehicle, 
                        const std::vector<double>& path_x,
                        const std::vector<double>& path_y,
                        adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 1 START==============";

    constexpr double DISTANCE_TO_EGO_THRESHOLD = 300.0;   // 30m
    constexpr double DISTANCE_TO_PATH_THRESHOLD = 100.0;   // 10m
    constexpr double CONFIDENCE_FACTOR  = 20.0;
    constexpr double CONFIDENCE_WEIGHT  = 0.7;

    obstacleListVector obstacle_stop;

    // i) 정지상태 판정: STOP_VALUE 이상 정지 + 구조물/보행자 제외
    // 정지 장애물 (구조물과 보행자는 제외)을 obstacle_stop에 저장
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

    // ii) 특장차로부터 30m 이내 장애물만 선택한다
    for (auto iter = obstacle_stop.begin(); iter != obstacle_stop.end(); )
    {
        // 특장차와 장애물 사이의 거리 계산
        double ego_distance = calculateDistance(*iter, ego_vehicle);
        // 특장차와 장애물 사이의 거리가 30m 이상이면 해당 장애물은 제외한다
        if (ego_distance >= DISTANCE_TO_EGO_THRESHOLD) 
        {
            adcm::Log::Info() << "[1-ii] 30m 초과로 제외: ID " << iter->obstacle_id;
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
        double path_distance = 0.0;

        if (calculateMinDistanceToPath(obstacle, path_x, path_y, path_distance) && path_distance < DISTANCE_TO_PATH_THRESHOLD) 
        {
            double distance_to_ego = calculateDistance(obstacle, ego_vehicle);
            double confidence = (CONFIDENCE_FACTOR / distance_to_ego) * CONFIDENCE_WEIGHT;

            adcm::riskAssessmentStruct riskAssessment1{
                .obstacle_id = obstacle.obstacle_id,
                .hazard_class = SCENARIO_1,
                .confidence = confidence
            };

            adcm::Log::Info() << "위험판단 시나리오 #1에 해당하는 장애물 ID: " 
                            << obstacle.obstacle_id 
                            << " | 컨피던스 값: " 
                            << confidence;

            riskAssessment.riskAssessmentList.push_back(riskAssessment1);
        }
    }
    adcm::Log::Info() << "============= KATECH: Scenario 1 DONE =============";
}

//=====시나리오 #2. 주행중 사각영역 존재 환경 판단=====
void evaluateScenario2(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       const std::vector<double>& path_x,
                       const std::vector<double>& path_y,
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 2 START==============";

    constexpr double DISTANCE_TO_EGO_THRESHOLD = 400.0;   // 객체-특장차 간 거리 threshold 40m
    constexpr double DISTANCE_TO_PATH_THRESHOLD = 100.0;  // 객체-전역경로 간 거리 threshold 10m
    constexpr double CONFIDENCE_FACTOR  = 40.0;
    constexpr double CONFIDENCE_WEIGHT  = 0.7;
    obstacleListVector obstacle_temp;


    // i) 높이 1m 초과 정적 장애물 && 동적 장애물이지만 정지상태 (STOP_VALUE) 만 obstacle_temp 에 저장
    for (const auto& obs : obstacle_list) {
        if ((obs.stop_count >= STOP_VALUE && obs.obstacle_class != STRUCTURE && obs.obstacle_class != PEDESTRIAN) ||
            (obs.obstacle_class == STRUCTURE && obs.fused_cuboid_z > 1)) 
        {
            adcm::Log::Info() << "[2-i] 정지 or 높은 정적 장애물 감지: ID " << obs.obstacle_id;
            obstacle_temp.push_back(obs);
        }
    }

    // ii) 특장차로부터 40m 초과 제거
    obstacle_temp.erase(
    std::remove_if(obstacle_temp.begin(), obstacle_temp.end(),
                    [&](const auto& obs)  {
                        double distance = calculateDistance(obs, ego_vehicle);
                        if (distance > DISTANCE_TO_EGO_THRESHOLD) {
                        adcm::Log::Info() << "[2-ii] 40m 초과로 제외: ID " << obs.obstacle_id;
                        return true;
                        }
            return false;
        }),
    obstacle_temp.end()
    );

    // iii) 경로 근접 + 리스크 평가
    for (const auto& obs : obstacle_temp) {
        double distance_to_path = 0.0;
        if (calculateMinDistanceToPath(obs, path_x, path_y, distance_to_path) && distance_to_path < DISTANCE_TO_PATH_THRESHOLD) 
        {
            double distance_to_ego = calculateDistance(obs, ego_vehicle);
        
            double confidence = (CONFIDENCE_FACTOR / distance_to_ego) * CONFIDENCE_WEIGHT;

            adcm::riskAssessmentStruct riskAssessment2{
                .obstacle_id = obs.obstacle_id,
                .hazard_class = SCENARIO_2,
                .confidence = confidence
            };

            adcm::Log::Info() << "위험판단 시나리오 #2에 해당하는 장애물 ID: " 
                            << obs.obstacle_id 
                            << " | 컨피던스 값: " 
                            << confidence;

            riskAssessment.riskAssessmentList.push_back(riskAssessment2);
        }
        else 
        {
            adcm::Log::Info() << "[2-iii] 객체-전역경로 근접하지 않음: ID " << obs.obstacle_id;
        }
    }
    adcm::Log::Info() << "============= KATECH: Scenario 2 DONE =============";
}


//=====시나리오 #3. 주행중 경로 주변 동적 장애물 통행 환경=====
void evaluateScenario3(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 3 START==============";

    obstacleListVector obstacle_temp;
    constexpr double MAX_SAFE_DISTANCE = 300.0;   // 30m
    constexpr double TTC_THRESHOLD = 10.0;    // 10초
    constexpr double MIN_SAFE_DISTANCE = 100;   //10m
    constexpr double CONFIDENCE_FACTOR  = 5.0; 
    constexpr double CONFIDENCE_WEIGHT  = 0.7;

    // i) 특장차로부터 10~30m 거리의 동적 객체를 obstacle_temp 에 저장
    for (const auto& obs : obstacle_list) {
        double distance_to_ego = calculateDistance(obs, ego_vehicle);

        if (distance_to_ego >= MIN_SAFE_DISTANCE && distance_to_ego <= MAX_SAFE_DISTANCE &&
            obs.obstacle_class != STRUCTURE) 
        {
            adcm::Log::Info() << "시나리오3-i) 10~30m 동적객체 추출: " << obs.obstacle_id;
            obstacle_temp.push_back(obs);
        }
    }

    // ii) TTC와 안전영역 진입 여부 평가
    for (const auto& obs : obstacle_temp) {

        double min_distance, ttc; 
        bool isTTCValid = getTTC(obs, ego_vehicle, ttc) && ttc < TTC_THRESHOLD;
        bool isDistanceValid = calculateMinDistanceLinear(obs, ego_vehicle, min_distance) && min_distance < MAX_SAFE_DISTANCE;

        if (isTTCValid && isDistanceValid)
        {
            double ttc_confidence = CONFIDENCE_FACTOR / ttc * CONFIDENCE_WEIGHT;
            double area_confidence = MAX_SAFE_DISTANCE / min_distance * CONFIDENCE_WEIGHT;
            double confidence = std::max(ttc_confidence, area_confidence);

            adcm::riskAssessmentStruct riskAssessment3{
                .obstacle_id = obs.obstacle_id,
                .hazard_class = SCENARIO_3,
                .confidence = confidence
            };

            adcm::Log::Info() << "위험판단 시나리오 #3에 해당하는 장애물 ID: " 
                            << obs.obstacle_id 
                            << " | 컨피던스 값: " 
                            << confidence;

            riskAssessment.riskAssessmentList.push_back(riskAssessment3);
        }
    }
    adcm::Log::Info() << "=============KATECH: scenario 3 DONE==============";
}

void evaluateScenario4(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 4 START==============";

    obstacleListVector obstacle_temp;

    constexpr double MIN_DISTANCE = 200.0;   // 20m
    constexpr double MAX_DISTANCE = 400.0;   // 40m
    constexpr double CONFIDENCE_WEIGHT = 0.7;
    //constexpr double MIN_DIST_APPROX = 1.0;
    //constexpr double MAX_CONFIDENCE = 100.0;    // 예시 최대값

    // i) 20m ~ 40m 동적 장애물만 obstacle_temp에 저장
    for (const auto& obs : obstacle_list) {
        double distance = calculateDistance(obs, ego_vehicle);
        if (distance > MIN_DISTANCE && distance < MAX_DISTANCE && obs.obstacle_class != STRUCTURE) {
            obstacle_temp.push_back(obs);
        }
    }

    for (const auto& obs : obstacle_temp) {
        double min_distance;
        if (calculateMinDistanceLinear(obs, ego_vehicle, min_distance))
        {
            double confidence = MAX_DISTANCE / min_distance * CONFIDENCE_WEIGHT;
            adcm::riskAssessmentStruct riskAssessment4{
                .obstacle_id = obs.obstacle_id,
                .hazard_class = SCENARIO_4,
                .confidence = confidence
            };

            adcm::Log::Info() << "위험판단 시나리오 #4에 해당하는 장애물 ID: " 
                            << obs.obstacle_id 
                            << " | 컨피던스 값: " 
                            << confidence;

            riskAssessment.riskAssessmentList.push_back(riskAssessment4);
        }
    }
    adcm::Log::Info() << "=============KATECH: scenario 4 DONE==============";
}
/**
 * @brief 시나리오 5: 보행자 밀집 및 신규 객체 출현 기반 위험 판단
 *
 * ego 차량 주변 (40m~50m) 내에서 보행자가 일정 수 이상 탐지되고,
 * 주행 경로 반경 10m 이내며
 * 이후 10초 이내에 새로운 보행자가 등장하며,
 * 등장한 보행자 간 거리가 20m 이하인 경우 위험 판단 수행.
 * 이때 ego 차량과 가장 가까운 보행자를 대상으로 confidence 값을 계산하여 평가 목록에 추가.
 * 현재 구조로 obstacle_list 에서 장애물이 계속 추가가 되지 있던 장애물이 없어지지는 않음 -> 이 가정하에 알고리즘 작성
 * confidence 값은 min_distance = 0이면 1.0 (가장 위험)하며 min_distance = 200이면 0.0 (경계선), min_distance > 200이면 0으로 클램핑
 * @param obstacle_list      현재 인지된 장애물 리스트
 * @param ego_vehicle        ego 차량 정보 (위치 등)
 * @param path_x             전역 경로 x 좌표 리스트
 * @param path_y             전역 경로 y 좌표 리스트
 * @param riskAssessment     시나리오 판단 결과가 저장될 객체
 */
void evaluateScenario5(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       const std::vector<double>& path_x, 
                       const std::vector<double>& path_y,
                       adcm::risk_assessment_Objects& riskAssessment)
{
    using namespace adcm;
    Log::Info() << "=============KATECH: scenario 5 START==============";

    constexpr double MIN_DISTANCE = 400.0;     // 40m
    constexpr double MAX_DISTANCE = 500.0;     // 50m
    constexpr double DISTANCE_TO_PATH = 100.0; // 10m
    constexpr double MAX_DISTANCE_BTW_OBS = 200.0;
    constexpr double TIMESTAMP_THRESHOLD = 1000.0;
   
    // (1) 보행자 필터링 및 거리 조건
    obstacleListVector ped_current, ped_new;
    for (const auto& obs : obstacle_list) {
        if (obs.obstacle_class != PEDESTRIAN) continue;

        double dist = calculateDistance(obs, ego_vehicle);
        if (dist < MIN_DISTANCE || dist > MAX_DISTANCE) continue;

        double path_dist;
        if (calculateMinDistanceToPath(obs, path_x, path_y, path_dist) && path_dist <= DISTANCE_TO_PATH)
            ped_current.push_back(obs);
        else
            Log::Debug() << "제거된 장애물 ID: " << obs.obstacle_id << " | 경로 거리: " << path_dist;
    }

    // (2) 최초 프레임일 경우 기준 설정
    if (ped_previous.empty()) {
        ped_previous = ped_current;
        Log::Info() << "============= scenario 5 DONE (보행자 없음)=============";
        return;
    }

   // (3) 신규 객체 추출
    if (extractNewObstacles(ped_previous, ped_current, ped_new)) {
        newPedFound++;

        if (newPedFound == 1) {
            Log::Info() << "New obstacle first detected! 시나리오 5 계속";
            ped_fix = ped_previous; 
            ped_new_accumulated = ped_new; // ped_new_accumulated 초기화

            //신규 객체가 발견되기 바로 이전 프레임 ped_fix 의 timestamp 값을 init_ped_timestamp_max 로 저장한다 
            init_ped_timestamp_max = 0.0;
            for (const auto& obs : ped_fix) {
                init_ped_timestamp_max = std::max(init_ped_timestamp_max, obs.timestamp);
            }
        }

        if (newPedFound >= 2) {
            ped_new_accumulated.insert(ped_new_accumulated.end(), ped_new.begin(), ped_new.end());

            new_ped_timestamp_min = std::numeric_limits<double>::max();
            for (const auto& obs : ped_new_accumulated) {
                new_ped_timestamp_min = std::min(new_ped_timestamp_min, obs.timestamp);
            }

            uint64_t timestamp_diff = new_ped_timestamp_min - init_ped_timestamp_max;
            Log::Info() << "timestamp diff: " << timestamp_diff;

            if (timestamp_diff < TIMESTAMP_THRESHOLD) {
                double max_distance = 0.0;
                for (size_t i = 0; i < ped_current.size(); ++i) {
                    for (size_t j = i + 1; j < ped_current.size(); ++j) {
                        if (ped_current[i].obstacle_class != PEDESTRIAN || 
                            ped_current[j].obstacle_class != PEDESTRIAN) continue;

                        max_distance = std::max(max_distance, calculateDistance(ped_current[i], ped_current[j]));
                    }
                }

                Log::Info() << "객체간 최대거리: " << max_distance;
                if (max_distance < MAX_DISTANCE_BTW_OBS) {
                    double min_distance = std::numeric_limits<double>::max();
                    obstacleListStruct closest_obs;

                    for (const auto& obs : ped_current) {
                        double dist = calculateDistance(obs, ego_vehicle);
                        if (dist < min_distance) {
                            min_distance = dist;
                            closest_obs = obs;
                        }
                    }
                    double confidence = clampValue((MAX_DISTANCE_BTW_OBS - min_distance) / MAX_DISTANCE_BTW_OBS, 0.0, 1.0);

                    riskAssessmentStruct scenario5{
                        .obstacle_id = closest_obs.obstacle_id,
                        .hazard_class = SCENARIO_5,
                        .confidence = confidence
                    };

                    Log::Info() << "위험판단 시나리오 #5 | ID: " << closest_obs.obstacle_id
                                << " | confidence: " << confidence;

                    riskAssessment.riskAssessmentList.push_back(scenario5);
                    resetScenario5State();
                }
            } else {
                //해당 else statement 는 삭제해도 무관 
                Log::Info() << "[Scenario 5] 두 번째 객체 10초 내 미등장 → 초기화";
                resetScenario5State();
            }
        }
    }

    // (4) 첫 객체만 감지된 상태에서 시간이 너무 지나도 초기화
    if (newPedFound == 1) {
        uint64_t now_timestamp = 0.0;
        for (const auto& obs : ped_current)
            now_timestamp = std::max(now_timestamp, obs.timestamp);

        uint64_t timestamp_diff = now_timestamp - init_ped_timestamp_max;

        if (timestamp_diff >= TIMESTAMP_THRESHOLD) {
            Log::Info() << "[Scenario 5] 첫 객체만 있고 10초 경과 → 상태 초기화";
            resetScenario5State();
        }
    }

    ped_previous = ped_current;
    Log::Info() << "============= scenario 5 DONE =============";
}

void resetScenario5State() {
    newPedFound = 0;
    ped_fix.clear();
    ped_new_accumulated.clear();
    init_ped_timestamp_max = 0.0;
    new_ped_timestamp_min = std::numeric_limits<double>::max();
}

/**
 * @brief 시나리오 6: 주행 경로상 장애물 통향량이 과다한 환경 (차량)
 *[ 건설기계류 (1~9) : 굴착기 = 1, 지게차 = 2, 콘크리트믹서트럭 = 3, 기중기 = 4, 콘크리트펌프 = 5, 덤프트럭 = 6 ]
 *[ 차량류 (10~19) :승용차 = 10, SUV = 11, 트럭 = 12, 기타차량 = 13, 오토바이 = 14, 자전거 = 15 ]
 * 대상의 위험시나리오
 * 
 * ego 차량 주변 (50m~60m) 내에서 차량이 탐지되고,
 * 주행 경로 반경 15m 이내이며
 * 이후 10초 이내에 새로운 차량이 등장하고,
 * 등장한 차량 간 거리가 30m 이하인 경우 위험 판단 수행.
 * 이때 ego 차량과 가장 가까운 차량을 대상으로 confidence 값을 계산하여 평가 목록에 추가.
 * confidence 값은 min_distance = 0이면 1.0 (가장 위험)이며 min_distance = 300이면 0.0 (경계선), min_distance > 300이면 0으로 클램핑
 */
void evaluateScenario6(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       const std::vector<double>& path_x, 
                       const std::vector<double>& path_y,
                       adcm::risk_assessment_Objects& riskAssessment)
{
    using namespace adcm;
    Log::Info() << "=============KATECH: scenario 6 START==============";

    constexpr double MIN_DISTANCE = 500.0;
    constexpr double MAX_DISTANCE = 600.0;
    constexpr double DISTANCE_TO_PATH = 150.0;
    constexpr double MAX_DISTANCE_BTW_OBS = 300.0;
    constexpr double TIMESTAMP_THRESHOLD = 1000.0;

    obstacleListVector veh_current, veh_new;
    for (const auto& obs : obstacle_list) {
        if (obs.obstacle_class < 1 || obs.obstacle_class > 19) continue;

        double dist = calculateDistance(obs, ego_vehicle);
        if (dist < MIN_DISTANCE || dist > MAX_DISTANCE) continue;

        double path_dist;
        if (calculateMinDistanceToPath(obs, path_x, path_y, path_dist) && path_dist <= DISTANCE_TO_PATH)
            veh_current.push_back(obs);
    }

    if (veh_previous.empty()) {
        veh_previous = veh_current;
        Log::Info() << "=============KATECH: scenario 6 DONE (ego 차량 주변 (50m~60m) && 주행경로 15m 이내 차 없음)==============";
        return;
    }

    if (extractNewObstacles(veh_previous, veh_current, veh_new)) {
        newVehFound++;

        if (newVehFound == 1) {
            Log::Info() << "New vehicle first detected! 시나리오 6 계속";
            veh_fix = veh_previous;
            veh_new_accumulated = veh_new;

            init_veh_timestamp_max = 0.0;
            for (const auto& obs : veh_fix)
                init_veh_timestamp_max = std::max(init_veh_timestamp_max, obs.timestamp);
        }

        if (newVehFound >= 2) {
            veh_new_accumulated.insert(veh_new_accumulated.end(), veh_new.begin(), veh_new.end());

            new_veh_timestamp_min = std::numeric_limits<double>::max();
            for (const auto& obs : veh_new_accumulated)
                new_veh_timestamp_min = std::min(new_veh_timestamp_min, obs.timestamp);

            uint64_t timestamp_diff = new_veh_timestamp_min - init_veh_timestamp_max;
            Log::Info() << "timestamp diff: " << timestamp_diff;

            if (timestamp_diff < TIMESTAMP_THRESHOLD) {
                double max_distance = 0.0;
                for (size_t i = 0; i < veh_current.size(); ++i) {
                    for (size_t j = i + 1; j < veh_current.size(); ++j) {
                        if (veh_current[i].obstacle_class < 1 || veh_current[i].obstacle_class > 19 || 
                            veh_current[j].obstacle_class < 1 || veh_current[j].obstacle_class > 19) continue;

                        max_distance = std::max(max_distance, calculateDistance(veh_current[i], veh_current[j]));
                    }
                }

                Log::Info() << "차량 간 최대거리: " << max_distance;
                if (max_distance < MAX_DISTANCE_BTW_OBS) {
                    double min_distance = std::numeric_limits<double>::max();
                    obstacleListStruct closest_obs;

                    for (const auto& obs : veh_current) {
                        double dist = calculateDistance(obs, ego_vehicle);
                        if (dist < min_distance) {
                            min_distance = dist;
                            closest_obs = obs;
                        }
                    }

                    double confidence = clampValue((MAX_DISTANCE_BTW_OBS - min_distance) / MAX_DISTANCE_BTW_OBS, 0.0, 1.0);

                    riskAssessmentStruct scenario6{
                        .obstacle_id = closest_obs.obstacle_id,
                        .hazard_class = SCENARIO_6,
                        .confidence = confidence
                    };

                    Log::Info() << "위험판단 시나리오 #6 | ID: " << closest_obs.obstacle_id
                                << " | confidence: " << confidence;

                    riskAssessment.riskAssessmentList.push_back(scenario6);
                    resetScenario6State();  // reset 함수명이 시나리오5에 묶여 있다면, 시나리오6용으로 변경 고려
                }
            } else {
                Log::Info() << "[Scenario 6] 두 번째 차량 10초 내 미등장 → 초기화";
                resetScenario6State();
            }
        }
    }

    if (newVehFound == 1) {
        uint64_t now_timestamp = 0.0;
        for (const auto& obs : veh_current)
            now_timestamp = std::max(now_timestamp, obs.timestamp);

        uint64_t timestamp_diff = now_timestamp - init_veh_timestamp_max;

        if (timestamp_diff >= TIMESTAMP_THRESHOLD) {
            Log::Info() << "[Scenario 6] 첫 차량만 있고 10초 경과 → 상태 초기화";
            resetScenario6State();
        }
    }

    veh_previous = veh_current;
    Log::Info() << "============= scenario 6 DONE =============";
}

void resetScenario6State() {
    newVehFound = 0;
    veh_fix.clear();
    veh_new_accumulated.clear();
    init_veh_timestamp_max = 0.0;
    new_veh_timestamp_min = std::numeric_limits<double>::max();
}

void evaluateScenario7(const std::vector<double>& path_x, 
                       const std::vector<double>& path_y, 
                       const std::vector<adcm::map_2dListVector>& map_2d, 
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "=============KATECH: scenario 7 START==============";
    detectUnscannedPath(map_2d, path_x, path_y, riskAssessment);
    adcm::Log::Info() << "=============KATECH: scenario 7 DONE==============";
}

void evaluateScenario8(const std::vector<double>& path_x, 
                       const std::vector<double>& path_y, 
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
            // adcm::Log::Info()<< "Original line equation: x = " << x_start << "\n";
            // adcm::Log::Info() << "Line shifted right: x = " << x_up << "\n";
            // adcm::Log::Info() << "Line shifted left: x = " << x_down << "\n";

            p1 = {x_down, y_start};  // First line start point
            p2 = {x_down, y_end}; // First line end point
            p3 = {x_up, y_start};   // Second line start point
            p4 = {x_up, y_end}; // Second line end point
        } 
        else 
        {
            // adcm::Log::Info() << "Original line equation: y = " << original_m << "x + " << original_c << "\n";
            // adcm::Log::Info() << "Line shifted up: y = " << original_m << "x + " << up_c << "\n";
            // adcm::Log::Info() << "Line shifted down: y = " << original_m << "x + " << down_c << "\n";

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
    adcm::Log::Info() << "=============KATECH: scenario 8 DONE==============";
}

