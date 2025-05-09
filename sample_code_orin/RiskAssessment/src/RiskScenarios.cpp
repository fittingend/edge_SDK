#include "main_riskassessment.hpp"
static obstacleListVector obstacle_fix, obstacle_previous, obstacle_new_accmulated;   // 시나리오 5용 전역변수
static int newObsFound; //시나리오 5용 전역 변수 
static uint64_t init_timestamp_max, new_timestamp_min; //시나리오 5용 전역 변수
static obstacleListVector obstacle_vehicle_initial; // 시나리오 6용 전역변수


//=====시나리오 #1. 주행중 전역경로 근방 이동가능한 정지 장애물이 존재하는 위험 환경=====
void evaluateScenario1(const obstacleListVector& obstacle_list, 
                        const adcm::vehicleListStruct& ego_vehicle, 
                        const doubleVector& path_x,
                        const doubleVector& path_y,
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
                       const doubleVector& path_x,
                       const doubleVector& path_y,
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
                        if (distance >= DISTANCE_TO_EGO_THRESHOLD) {
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
    adcm::Log::Info() << "scenario 3 DONE";
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
    adcm::Log::Info() << "scenario 4 DONE";
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
                       const doubleVector& path_x, 
                       const doubleVector& path_y,
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
    obstacleListVector obstacle_current, obstacle_new;
    for (const auto& obs : obstacle_list) {
        if (obs.obstacle_class != PEDESTRIAN) continue;

        double dist = calculateDistance(obs, ego_vehicle);
        if (dist < MIN_DISTANCE || dist > MAX_DISTANCE) continue;

        double path_dist;
        if (calculateMinDistanceToPath(obs, path_x, path_y, path_dist) && path_dist <= DISTANCE_TO_PATH)
            obstacle_current.push_back(obs);
        else
            Log::Debug() << "제거된 장애물 ID: " << obs.obstacle_id << " | 경로 거리: " << path_dist;
    }

    // (2) 최초 프레임일 경우 기준 설정
    if (obstacle_previous.empty()) {
        obstacle_previous = obstacle_current;
        return;
    }

   // (3) 신규 객체 추출
    if (extractNewObstacles(obstacle_previous, obstacle_current, obstacle_new)) {
        newObsFound++;

        if (newObsFound == 1) {
            Log::Info() << "New obstacle first detected! 시나리오 5 계속";
            obstacle_fix = obstacle_previous; 
            obstacle_new_accmulated = obstacle_new; // obstacle_new_accmulated 초기화

            //신규 객체가 발견되기 바로 이전 프레임 obstacle_fix 의 timestamp 값을 init_timestamp_max 로 저장한다 
            init_timestamp_max = 0.0;
            for (const auto& obs : obstacle_fix) {
                init_timestamp_max = std::max(init_timestamp_max, obs.timestamp);
            }
        }

        if (newObsFound >= 2) {
            obstacle_new_accmulated.insert(obstacle_new_accmulated.end(), obstacle_new.begin(), obstacle_new.end());

            new_timestamp_min = std::numeric_limits<double>::max();
            for (const auto& obs : obstacle_new_accmulated) {
                new_timestamp_min = std::min(new_timestamp_min, obs.timestamp);
            }

            uint64_t timestamp_diff = new_timestamp_min - init_timestamp_max;
            Log::Info() << "timestamp diff: " << timestamp_diff;

            if (timestamp_diff < TIMESTAMP_THRESHOLD) {
                double max_distance = 0.0;
                for (size_t i = 0; i < obstacle_current.size(); ++i) {
                    for (size_t j = i + 1; j < obstacle_current.size(); ++j) {
                        if (obstacle_current[i].obstacle_class != PEDESTRIAN || 
                            obstacle_current[j].obstacle_class != PEDESTRIAN) continue;

                        max_distance = std::max(max_distance, calculateDistance(obstacle_current[i], obstacle_current[j]));
                    }
                }

                Log::Info() << "객체간 최대거리: " << max_distance;
                if (max_distance < MAX_DISTANCE_BTW_OBS) {
                    double min_distance = std::numeric_limits<double>::max();
                    obstacleListStruct closest_obs;

                    for (const auto& obs : obstacle_current) {
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
    if (newObsFound == 1) {
        uint64_t now_timestamp = 0.0;
        for (const auto& obs : obstacle_current)
            now_timestamp = std::max(now_timestamp, obs.timestamp);

        uint64_t timestamp_diff = now_timestamp - init_timestamp_max;

        if (timestamp_diff >= TIMESTAMP_THRESHOLD) {
            Log::Info() << "[Scenario 5] 첫 객체만 있고 10초 경과 → 상태 초기화";
            resetScenario5State();
        }
    }

    obstacle_previous = obstacle_current;
    Log::Info() << "============= scenario 5 DONE =============";
}

void resetScenario5State() {
    newObsFound = 0;
    obstacle_fix.clear();
    obstacle_new_accmulated.clear();
    init_timestamp_max = 0.0;
    new_timestamp_min = std::numeric_limits<double>::max();
}
/*
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
    
    // ① 50~60m 사이의 차량 필터링
    for (const auto& obs : obstacle_list) {
        double distance = getDistance(obs, ego_vehicle);
        if (distance > 500 && distance < 600 &&
            (obs.obstacle_class == VEHICLE_LARGE || obs.obstacle_class == VEHICLE_SMALL)) {
            obstacle_vehicle_50_60.push_back(obs);
        }
    }
    // ② 주행경로 반경 15m 초과 제거
    obstacle_vehicle_50_60.erase(
        std::remove_if(obstacle_vehicle_50_60.begin(), obstacle_vehicle_50_60.end(),
        [&](const auto& obs) { 
            double dist = 0.0;
            bool isValid = calculateDistanceToPath(obs, path_x, path_y, dist);
            if (!isValid || dist > 150) {
                adcm::Log::Info() << "[6-ii] 제거 대상 - Obstacle ID: " << obs.obstacle_id
                                  << " | Valid: " << isValid << " | Distance: " << dist;
                return true;  // 제거
            }
            return false;     // 유지
        }),
    obstacle_vehicle_50_60.end()
    );
    
    // ③ 최초 실행 시 초기 리스트 저장
    if (obstacle_vehicle_initial.empty())
    {
        adcm::Log::Info() << "[6] 최초 실행 - 리스트 저장 후 종료";
        obstacle_vehicle_initial = obstacle_vehicle_50_60;
        adcm::Log::Info() << "============= KATECH: Scenario 6 DONE =============";
        return;
    }

    // ④ 신규 장애물 탐색
    std::unordered_set<int> initial_ids;
    for (const auto& obs : obstacle_vehicle_initial)
        initial_ids.insert(obs.obstacle_id);

    for (const auto& obs : obstacle_vehicle_50_60)
    {
        if (initial_ids.find(obs.obstacle_id) == initial_ids.end())
            obstacle_vehicle_new.push_back(obs);
    }

    if (obstacle_vehicle_new.empty())
    {
        adcm::Log::Info() << "[6] 신규 장애물 없음";
        adcm::Log::Info() << "============= KATECH: Scenario 6 DONE =============";
        return;
    }

    // ⑤ 타임스탬프 비교
    for (const auto& obs : obstacle_vehicle_initial)
        ori_timestamp_max = std::max(ori_timestamp_max, obs.timestamp);

    for (const auto& obs : obstacle_vehicle_new)
        new_timestamp_min = std::min(new_timestamp_min, obs.timestamp);

    uint64_t timestamp_diff = new_timestamp_min - ori_timestamp_max;

    // ⑥ 출현시간 10초 이내 && 최대 거리 계산
    if (timestamp_diff < 1500)
    {
        for (const auto& ori_obs : obstacle_vehicle_initial)
        {
            for (const auto& new_obs : obstacle_vehicle_new)
            {
                double dist = getDistance(ori_obs, new_obs);
                max_distance = std::max(max_distance, dist);
            }
        }

        adcm::Log::Info() << "[6] 객체간 최대거리: " << max_distance;

        // ⑦ 통행과다환경 판단 및 컨피던스 계산
        if (max_distance < 600)
        {
            for (const auto& obs : obstacle_vehicle_new)
            {
                double dist_ego = getDistance(obs, ego_vehicle);
                if (dist_ego < 40)
                {
                    double confidence = 400 / dist_ego * 0.7;
                    adcm::riskAssessmentStruct riskAssessment6{
                        .obstacle_id = obs.obstacle_id,
                        .hazard_class = SCENARIO_6,
                        .confidence = confidence
                    };
                    riskAssessment.riskAssessmentList.push_back(riskAssessment6);

                    adcm::Log::Info() << "[6] Risk generated: " << obs.obstacle_id 
                                      << " | Confidence: " << confidence;
                }
            }
        }
    }

    adcm::Log::Info() << "============= KATECH: Scenario 6 DONE =============";

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

*/
