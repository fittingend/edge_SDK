#include "main_riskassessment.hpp"
#include "config.hpp"
#include <unordered_map>
namespace {
inline bool scenarioLogEnabled() { return gScenarioLogEnabled; }

/*
double calculateFrontRearFactor(const adcm::obstacleListStruct& obs,
                                const adcm::vehicleListStruct& ego_vehicle)
{
    // 방향 가중치 계산 계수 (전방일수록 1.0에 가까움)
    constexpr double kBase = 0.65;
    constexpr double kGain = 0.35;
    constexpr double kMin  = 0.3;
    constexpr double kMax  = 1.0;
    constexpr double kEps  = 1e-6;
    constexpr double kDegToRad = M_PI / 180.0;

    // ego → obstacle 벡터
    const double dx = obs.fused_position_x - ego_vehicle.position_x;
    const double dy = obs.fused_position_y - ego_vehicle.position_y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < kEps) {
        // 거의 같은 위치면 중립값 반환
        return kBase;
    }

    // ego heading(deg) 기준 전방 투영값 계산
    const double heading_rad = ego_vehicle.heading_angle * kDegToRad;
    const double forward_proj = dx * std::cos(heading_rad) + dy * std::sin(heading_rad);
    const double alignment = forward_proj / dist; // [-1, 1] (전방=1, 후방=-1)

    // 방향 가중치 계산 후 범위 제한
    const double dir_factor = kBase + kGain * alignment;
    return clampValue(dir_factor, kMin, kMax);
}
*/
}

#define SCENARIO_LOG_INFO() if (!scenarioLogEnabled()) ; else adcm::Log::Info()

//=====시나리오 #1. 주행중 전역경로 근방 이동가능한 정지 장애물이 존재하는 위험 환경=====
void evaluateScenario1(const obstacleListVector& obstacle_list, 
                        const adcm::vehicleListStruct& ego_vehicle, 
                        const std::vector<double>& path_x,
                        const std::vector<double>& path_y,
                        adcm::risk_assessment_Objects& riskAssessment,
                        std::uint8_t edge_state)
{
    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 1 START =============";

    if (edge_state != 3) {
        SCENARIO_LOG_INFO() << "[시나리오1] MOVE 상태 아님(" << static_cast<int>(edge_state) << ") → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 1 DONE =============";
        return;
    }

    // 단위: 거리 dm(0.1 m), 시간 ms
    constexpr double DIST_TO_EGO_MAX_DM  = 300.0; // 장애물 <-> 특장차와 30 m 이내
    constexpr double DIST_TO_PATH_MAX_DM = 300.0; // 장애물 <-> 전역경로와 30 m 이내

    // 컨피던스 파라미터 (confidence 0.7 이상 목표)
    constexpr double EGO_THRESH_DM  = 300.0;  // 거리 기준 (변경 없음)
    constexpr double PATH_THRESH_DM = 300.0;  // 거리 기준 (변경 없음)
    constexpr double W_EGO  = 0.6;
    constexpr double W_PATH = 0.4;
    constexpr double CONFIDENCE_MULTIPLIER = 1.5;  // confidence 부스트 (0.54 → 0.70)

    obstacleListVector candidates; // 최종 후보군

    // === (i)~(iii) 조건 검사 ===
    for (const auto& obs : obstacle_list) {
        // (i) 동적 장애물 & 정지 상태
        const bool is_vehicle = (obs.obstacle_class == 20);
        if (!is_vehicle) continue;
        if (obs.stop_count < gStopValue) continue;

        SCENARIO_LOG_INFO() << "[1-i] 차량 & 정지 상태: ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | stop_count=" << obs.stop_count;

        // (ii) Ego 거리
        const double d_ego_dm = calculateDistance(obs, ego_vehicle);
        if (d_ego_dm > DIST_TO_EGO_MAX_DM) continue;
        SCENARIO_LOG_INFO() << "[1-ii] ego 거리 통과: ID=" << obs.obstacle_id
                    << " | Ego거리=" << (d_ego_dm/10.0) << " m";

        // (iii) 경로 거리
        double d_path_dm = 0.0;
        if (!calculateMinDistanceToPath(obs, path_x, path_y, d_path_dm)) continue;
        if (d_path_dm > DIST_TO_PATH_MAX_DM) 
        {
            SCENARIO_LOG_INFO() << "[1-iii 제외] ID=" << obs.obstacle_id
                              << " | 경로거리=" << (d_path_dm/10.0) << " m (>10)";
            continue;
        }
        // 세 조건 모두 통과한 경우 후보군에 추가
        candidates.push_back(obs);
    }

    // === 후보군 로그 출력 ===
    if (candidates.empty()) {
        SCENARIO_LOG_INFO() << "[시나리오1] 최종 후보 없음 → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 1 DONE =============";
        return;
    }

    SCENARIO_LOG_INFO() << "[시나리오1] 최종 후보군 " << candidates.size() << "개:";
    for (const auto& obs : candidates) {
        double d_ego_dm  = calculateDistance(obs, ego_vehicle);
        double d_path_dm = 0.0;
        calculateMinDistanceToPath(obs, path_x, path_y, d_path_dm);

        SCENARIO_LOG_INFO() << "   - ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | stop_count=" << obs.stop_count
                          << " | Ego거리=" << (d_ego_dm/10.0) << " m"
                          << " | 경로거리=" << (d_path_dm/10.0) << " m";
    }

    // === 컨피던스 계산 및 결과 등록 ===
    for (const auto& obs : candidates) {
        double d_ego_dm  = calculateDistance(obs, ego_vehicle);
        double d_path_dm = 0.0;
        calculateMinDistanceToPath(obs, path_x, path_y, d_path_dm);

        const double s_ego  = clampValue((EGO_THRESH_DM  - d_ego_dm) / EGO_THRESH_DM, 0.0, 1.0);
        const double s_path = clampValue((PATH_THRESH_DM - d_path_dm) / PATH_THRESH_DM, 0.0, 1.0);
        const double base_confidence = W_EGO * s_ego + W_PATH * s_path;
        // const double dir_factor = calculateFrontRearFactor(obs, ego_vehicle); // 전방/후방 가중치
        double confidence   = clampValue(base_confidence * CONFIDENCE_MULTIPLIER, 0.0, 1.0); // 최종 confidence에 부스트 적용

        adcm::riskAssessmentStruct r{};
        r.obstacle_id  = obs.obstacle_id;
        r.hazard_class = SCENARIO_1;
        r.confidence   = confidence;

        SCENARIO_LOG_INFO() << "[시나리오1 활성] ID=" << obs.obstacle_id
                          << " | s_ego=" << s_ego << ", s_path=" << s_path
                          << " | confidence=" << confidence;

        riskAssessment.riskAssessmentList.push_back(r);
    }

    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 1 DONE =============";
}

//=====시나리오 #2. 주행중 사각영역 존재 환경 판단=====
void evaluateScenario2(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       const std::vector<double>& path_x,
                       const std::vector<double>& path_y,
                       adcm::risk_assessment_Objects& riskAssessment,
                       std::uint8_t edge_state)
{
    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 2 START =============";

    if (edge_state != 3) {
        SCENARIO_LOG_INFO() << "[시나리오2] MOVE 상태 아님(" << static_cast<int>(edge_state) << ") → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 2 DONE =============";
        return;
    }

    // 단위: 거리 dm(0.1m)
    constexpr double DIST_TO_EGO_MAX_DM  = 400.0; // 특장차와 40 m 이내
    constexpr double DIST_TO_PATH_MAX_DM = 400.0; // 특장차 전역경로에서 10 m 이내 (03.26 수정: 30 m → 40 m)

    // 컨피던스 파라미터
    // 트리거 조건은 유지하고, confidence 정규화 범위만 완화해서
    // 너무 가까워지기 전에 0.7 이상이 나오도록 조정한다.
    constexpr double EGO_THRESH_DM  = 450.0;
    constexpr double PATH_THRESH_DM = 450.0;
    constexpr double W_EGO  = 0.7;
    constexpr double W_PATH = 0.3;
    constexpr double CONFIDENCE_MULTIPLIER = 1;

    obstacleListVector candidates;

    for (const auto& obs : obstacle_list) {
        // ----------------------------------------------------
        // (i) 조건: 차량이면서 정지 상태 OR 높이 2m 이상 정적 장애물
        bool cond_vehicle_stopped = (obs.obstacle_class == 1 && obs.stop_count >= gStopValue);
        //bool cond_static_high     = (obs.obstacle_class >= 30 && obs.obstacle_class <= 49 && obs.fused_cuboid_z > HEIGHT_THRESH_M);

        if (!cond_vehicle_stopped) continue;

        SCENARIO_LOG_INFO() << "[2-i] 통과: ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | stop_count=" << obs.stop_count
                          << " | height=" << obs.fused_cuboid_z;

        // ----------------------------------------------------
        // (ii) Ego와의 거리 ≤ 40m
        const double d_ego_dm = calculateDistance(obs, ego_vehicle);
        if (d_ego_dm > DIST_TO_EGO_MAX_DM) {
            // SCENARIO_LOG_INFO() << "[2-ii 제외] ID=" << obs.obstacle_id
            //                   << " | Ego거리=" << (d_ego_dm/10.0) << " m (>40)";
            continue;
        }
        SCENARIO_LOG_INFO() << "[2-ii] 통과: ID=" << obs.obstacle_id
                          << " | Ego거리=" << (d_ego_dm/10.0) << " m";

        // ----------------------------------------------------
        // (iii) 전역경로와의 거리 ≤ 10m
        double d_path_dm = 0.0;
        if (!calculateMinDistanceToPath(obs, path_x, path_y, d_path_dm)) {
            // SCENARIO_LOG_INFO() << "[2-iii 제외] ID=" << obs.obstacle_id
            //                   << " | 경로거리 계산 실패";
            continue;
        }
        if (d_path_dm > DIST_TO_PATH_MAX_DM) {
            SCENARIO_LOG_INFO() << "[2-iii 제외] ID=" << obs.obstacle_id
                              << " | 경로거리=" << (d_path_dm/10.0) << " m (>10)";
            continue;
        }
        SCENARIO_LOG_INFO() << "[2-iii] 통과: ID=" << obs.obstacle_id
                            << " | 경로거리=" << (d_path_dm/10.0) << " m";

        // 세 조건 모두 만족 → 후보군 추가
        candidates.push_back(obs);
    }

    // === 최종 후보군 로그 ===
    if (candidates.empty()) {
        SCENARIO_LOG_INFO() << "[시나리오2] 최종 후보 없음 → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 2 DONE =============";
        return;
    }

    SCENARIO_LOG_INFO() << "[시나리오2] 최종 후보군 " << candidates.size() << "개";

    // === 컨피던스 계산 및 결과 등록 ===
    for (const auto& obs : candidates) {
        double d_ego_dm  = calculateDistance(obs, ego_vehicle);
        double d_path_dm = 0.0;
        calculateMinDistanceToPath(obs, path_x, path_y, d_path_dm);

        // Ego/Path 정규화 기반 confidence 계산
        const double s_ego  = clampValue((EGO_THRESH_DM  - d_ego_dm) / EGO_THRESH_DM, 0.0, 1.0);
        const double s_path = clampValue((PATH_THRESH_DM - d_path_dm) / PATH_THRESH_DM, 0.0, 1.0);
        const double base_confidence = W_EGO * s_ego + W_PATH * s_path;
        // const double dir_factor = calculateFrontRearFactor(obs, ego_vehicle); // 전방/후방 가중치
        double confidence   = clampValue(base_confidence * CONFIDENCE_MULTIPLIER, 0.0, 1.0); // 최종 confidence 부스트 적용

        adcm::riskAssessmentStruct r{};
        r.obstacle_id  = obs.obstacle_id;
        r.hazard_class = SCENARIO_2;
        r.confidence   = confidence;

        SCENARIO_LOG_INFO() << "[시나리오2 활성] ID=" << obs.obstacle_id
                          << " | s_ego=" << s_ego << ", s_path=" << s_path
                          << " | confidence=" << confidence;

        riskAssessment.riskAssessmentList.push_back(r);
    }

    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 2 DONE =============";
}//===== 시나리오 #3. 주행중 경로 주변 동적 장애물 통행 환경 =====
void evaluateScenario3(const obstacleListVector& obstacle_list,
                       const adcm::vehicleListStruct& ego_vehicle,
                       adcm::risk_assessment_Objects& riskAssessment,
                       std::uint8_t edge_state)
{
    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 3 START =============";

    if (edge_state != 3) {
        SCENARIO_LOG_INFO() << "[시나리오3] MOVE 상태 아님(" << static_cast<int>(edge_state) << ") → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 3 DONE =============";
        return;
    }

    // 단위: dm (0.1 m)
    constexpr double TRIGGER_DIST_DM   = 250.0; // 25 m 이내 진입 시 즉시 트리거
    constexpr double RELEASE_THRESH_DM = 300.0; // 30 m 이상이면 해제(히스테리시스)
    constexpr int    FRAMES_TO_0P7     = 10;    // 유지 10프레임에서 confidence=0.7
    constexpr int    FRAMES_TO_MAX     = 20;    // 유지 20프레임에서 confidence=1.0
    constexpr int    MISS_GRACE        = 2;     // 허용 이탈 프레임 수
    constexpr double CONF_START        = 0.40;
    constexpr double CONF_AT_10        = 0.70;
    constexpr double CONF_MAX          = 1.00;
    constexpr double MIN_TRIGGER_X     = 800.0;
    constexpr double MIN_TRIGGER_Y     = 400.0;

    // 장애물별 누적 상태 (프레임 간 유지)
    struct S3State { int near_count = 0; int miss_count = 0; };
    static std::unordered_map<uint16_t, S3State> s3_state;

    // 이번 프레임에 관측된 ID 수집 (소멸 장애물 정리용)
    std::unordered_set<uint16_t> seen_ids;

    for (const auto& obs : obstacle_list) {
        const bool target_class = (obs.obstacle_class >= 1 && obs.obstacle_class <= 10);
        if (!target_class) continue;

        const bool coord_in_range =
            (obs.fused_position_x >= MIN_TRIGGER_X) &&
            (obs.fused_position_y >= MIN_TRIGGER_Y);
        if (!coord_in_range) {
            SCENARIO_LOG_INFO() << "[3-reject] ID=" << obs.obstacle_id
                                << " | class=" << static_cast<int>(obs.obstacle_class)
                                << " | pos=(" << obs.fused_position_x << ", " << obs.fused_position_y << ")"
                                << " | threshold=(x>=" << MIN_TRIGGER_X
                                << ", y>=" << MIN_TRIGGER_Y << ")";
            continue;
        }

        const double d_ego_dm = calculateDistance(obs, ego_vehicle);
        seen_ids.insert(static_cast<uint16_t>(obs.obstacle_id));

        auto& st = s3_state[static_cast<uint16_t>(obs.obstacle_id)];

        if (d_ego_dm <= TRIGGER_DIST_DM) {
            // 25 m 이내 유지 프레임 누적
            st.near_count++;
            st.miss_count = 0;
            SCENARIO_LOG_INFO() << "[3] ID=" << obs.obstacle_id
                                << " | near_count=" << st.near_count
                                << " | trigger_dist=" << (TRIGGER_DIST_DM / 10.0) << " m"
                                << " | dist=" << (d_ego_dm / 10.0) << " m";
        } else if (st.near_count > 0 && d_ego_dm < RELEASE_THRESH_DM) {
            // 히스테리시스 구간 (25~30 m): miss 프레임 허용
            st.miss_count++;
            if (st.miss_count <= MISS_GRACE) {
                SCENARIO_LOG_INFO() << "[3] ID=" << obs.obstacle_id
                                    << " | miss grace " << st.miss_count
                                    << "/" << MISS_GRACE
                                    << " | dist=" << (d_ego_dm / 10.0) << " m";
            } else {
                // grace 초과 → 리셋
                st.near_count = 0;
                st.miss_count = 0;
                SCENARIO_LOG_INFO() << "[3] ID=" << obs.obstacle_id
                                    << " | grace 초과 → 카운터 리셋";
            }
        } else {
            // 30 m 이상 → 완전 해제
            if (st.near_count > 0 || st.miss_count > 0) {
                SCENARIO_LOG_INFO() << "[3] ID=" << obs.obstacle_id
                                    << " | 해제 거리 초과(" << (d_ego_dm / 10.0) << " m) → 리셋";
            }
            st.near_count = 0;
            st.miss_count = 0;
        }

        // 트리거 판정: 25m 진입 즉시 트리거, 유지 프레임 기반으로 confidence 상승
        if (st.near_count >= 1) {
            double confidence = CONF_START;
            if (st.near_count <= FRAMES_TO_0P7) {
                const double t = clampValue(
                    static_cast<double>(st.near_count - 1) / static_cast<double>(std::max(1, FRAMES_TO_0P7 - 1)),
                    0.0, 1.0);
                confidence = CONF_START + (CONF_AT_10 - CONF_START) * t;
            } else {
                const double t = clampValue(
                    static_cast<double>(st.near_count - FRAMES_TO_0P7) /
                        static_cast<double>(std::max(1, FRAMES_TO_MAX - FRAMES_TO_0P7)),
                    0.0, 1.0);
                confidence = CONF_AT_10 + (CONF_MAX - CONF_AT_10) * t;
            }
            confidence = clampValue(confidence, 0.0, 1.0);

            adcm::riskAssessmentStruct r{};
            r.obstacle_id  = obs.obstacle_id;
            r.hazard_class = SCENARIO_3;
            r.confidence   = confidence;

            SCENARIO_LOG_INFO() << "[시나리오3 활성] ID=" << obs.obstacle_id
                                << " | 유지 " << st.near_count << " 프레임"
                                << " | dist=" << (d_ego_dm / 10.0) << " m"
                                << " | confidence=" << confidence;

            riskAssessment.riskAssessmentList.push_back(r);
        }
    }

    // 이번 프레임에 보이지 않는 ID 상태 제거
    for (auto it = s3_state.begin(); it != s3_state.end(); ) {
        if (seen_ids.find(it->first) == seen_ids.end()) {
            it = s3_state.erase(it);
        } else {
            ++it;
        }
    }

    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 3 DONE =============";
}

//===== 시나리오 #4. 정지중(ego=0 m/s) 주변 동적 객체 접근 위험 =====
void evaluateScenario4(const obstacleListVector& obstacle_list,
                       const adcm::vehicleListStruct& ego_vehicle,
                       adcm::risk_assessment_Objects& riskAssessment,
                       std::uint8_t& edge_state)
{
    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 4 START =============";

    // 좌표 단위: dm(0.1 m) 가정
    constexpr double MAX_DIST_DM = 400.0;   // 40 m
    constexpr double CONFIDENCE_MULTIPLIER = 1.2;

    // ① 특장차 작업중 
    if (edge_state != 4) {
        SCENARIO_LOG_INFO() << "[시나리오4] 특장차 작업 상태 아님(" << static_cast<int>(edge_state) << ") → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 4 DONE =============";
        return;
    }

    obstacleListVector candidates;

    for (const auto& obs : obstacle_list) {
        // ② 동적 객체(속도/stop_count 조건은 사용하지 않음)
        const bool dynamic_class = (obs.obstacle_class >= 1 && obs.obstacle_class <= 29);
        if (!dynamic_class) continue;

        // Ego-장애물 직선거리 (dm)
        const double d_ego_dm = calculateDistance(obs, ego_vehicle);
        if (d_ego_dm > MAX_DIST_DM) continue;
        SCENARIO_LOG_INFO() << "[4-i] 통과: ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | Ego거리=" << (d_ego_dm/10.0) << " m";

        // ③ 선형 최소거리 ≤ 40 m 조건 (dm 단위 반환 가정)
        double d_min_dm = 0.0;
        if (!calculateMinDistanceLinear(obs, ego_vehicle, d_min_dm) || d_min_dm > MAX_DIST_DM) {
            SCENARIO_LOG_INFO() << "[4-ii 제외] ID=" << obs.obstacle_id
                              << " | linMinDist=" << (d_min_dm/10.0) << " m (>40 또는 계산실패)";
            continue;
        }
        SCENARIO_LOG_INFO() << "[4-ii] 통과: ID=" << obs.obstacle_id
                          << " | linMinDist=" << (d_min_dm/10.0) << " m";

        // 세 조건 모두 통과 → 후보군 편입
        candidates.push_back(obs);
    }

    // === 최종 후보군 요약 출력 (컨피던스 계산 전) ===
    if (candidates.empty()) {
        SCENARIO_LOG_INFO() << "[시나리오4] 최종 후보 없음 → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 4 DONE =============";
        return;
    }

    SCENARIO_LOG_INFO() << "[시나리오4] 최종 후보군 " << candidates.size() << "개:";
    for (const auto& obs : candidates) {
        double d_min_dm = 0.0;
        (void)calculateMinDistanceLinear(obs, ego_vehicle, d_min_dm);
        const double d_ego_dm = calculateDistance(obs, ego_vehicle);

        SCENARIO_LOG_INFO() << "   - ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | Ego거리=" << (d_ego_dm/10.0) << " m"
                          << " | linMinDist=" << (d_min_dm/10.0) << " m";
    }

    // === 컨피던스 계산 & 등록 ===
    for (const auto& obs : candidates) {
        double d_min_dm = 0.0;
        (void)calculateMinDistanceLinear(obs, ego_vehicle, d_min_dm);

        // 정규화 (0~1)
        // - 선형 최소거리: 0 m → 1, 40 m → 0
        double s_mind = 1.0 - (d_min_dm / MAX_DIST_DM);
        s_mind = clampValue(s_mind, 0.0, 1.0);

        // 속도 기반 항목 제거: 거리 기반 confidence만 사용
        // 15m 부근에서 0.7을 넘기도록 배율 적용
        double confidence = clampValue(s_mind * CONFIDENCE_MULTIPLIER, 0.0, 1.0);

        adcm::riskAssessmentStruct r{};
        r.obstacle_id  = obs.obstacle_id;
        r.hazard_class = SCENARIO_4;
        r.confidence   = confidence;

        SCENARIO_LOG_INFO() << "[시나리오4 활성] ID=" << obs.obstacle_id
                          << " | s_minDist=" << s_mind
                          << " | confidence=" << confidence;

        riskAssessment.riskAssessmentList.push_back(r);
    }

    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 4 DONE =============";
}
/**
 * @brief 시나리오 5: 보행자 순간 밀집 기반 위험 판단
 *
 * ego 차량 주변 거리 조건과 전역 경로 거리 조건을 만족하는 보행자 후보를 현재 프레임에서만 추출한다.
 * 후보 보행자들 사이 거리 중 가장 가까운 페어가 임계치 이하이면 시나리오5를 트리거한다.
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
                       adcm::risk_assessment_Objects& riskAssessment,
                       std::uint8_t edge_state)
{
    using namespace adcm;
    SCENARIO_LOG_INFO() << "==================== [시나리오5 시작] ====================";

    if (edge_state != 3) {
        SCENARIO_LOG_INFO() << "[시나리오5] MOVE 상태 아님(" << static_cast<int>(edge_state) << ") → 종료";
        SCENARIO_LOG_INFO() << "==================== [시나리오5 종료] ====================";
        return;
    }

    // 상수 (dm)
    constexpr double EGO_MIN_DM          = 50.0;   // 5 m
    constexpr double EGO_MAX_DM          = 500.0;  // 50 m
    constexpr double DIST_TO_PATH_MAX_DM = 200.0;  // 20 m
    constexpr double MAX_PAIR_DIST_DM    = 100.0;  // 10 m

    if (obstacle_list.empty()) {
        SCENARIO_LOG_INFO() << "[시나리오5] 입력 장애물 없음 → 종료";
        SCENARIO_LOG_INFO() << "==================== [시나리오5 종료] ====================";
        return;
    }

    // 프레임 timestamp 범위 (디버깅)
    double frame_ts_min = std::numeric_limits<double>::max();
    double frame_ts_max = std::numeric_limits<double>::lowest();
    for (size_t i = 0; i < obstacle_list.size(); ++i) {
        const double ts = static_cast<double>(obstacle_list[i].timestamp);
        frame_ts_min = std::min(frame_ts_min, ts);
        frame_ts_max = std::max(frame_ts_max, ts);
    }
    SCENARIO_LOG_INFO() << "[시나리오5] 프레임 수신: N=" << obstacle_list.size()
                << " | ts=[" << frame_ts_min << "," << frame_ts_max << "] ms";

    // (1) 후보 추출
    obstacleListVector cand; cand.reserve(obstacle_list.size());
    for (size_t k = 0; k < obstacle_list.size(); ++k) {
        const auto& obs = obstacle_list[k];

        if (static_cast<ObstacleClass>(obs.obstacle_class) != ObstacleClass::PEDESTRIAN) {
            continue;
        }
        double d_ego_dm = calculateDistance(obs, ego_vehicle);
        if (d_ego_dm < EGO_MIN_DM || d_ego_dm > EGO_MAX_DM) {
            SCENARIO_LOG_INFO() << "  └─[제외] ID=" << obs.obstacle_id
                       << " | Ego거리=" << (d_ego_dm/10.0) << " m (요구: 40~50 m)";
            continue;
        }
        double path_dist_dm = 0.0;
        if (!calculateMinDistanceToPath(obs, path_x, path_y, path_dist_dm)) {
            SCENARIO_LOG_INFO() << "  └─[제외] ID=" << obs.obstacle_id << " | 경로거리 계산 실패";
            continue;
        }
        if (path_dist_dm > DIST_TO_PATH_MAX_DM) {
            SCENARIO_LOG_INFO() << "  └─[제외] ID=" << obs.obstacle_id
                        << " | 경로거리=" << (path_dist_dm/10.0) << " m (요구: ≤10 m)";
            continue;
        }

        cand.push_back(obs);
        SCENARIO_LOG_INFO() << "  └─[후보] ID=" << obs.obstacle_id
                    << " | ts=" << obs.timestamp << " ms"
                    << " | Ego거리=" << (d_ego_dm/10.0) << " m"
                    << " | 경로거리=" << (path_dist_dm/10.0) << " m";
    }

    if (cand.size() < 2) {
        SCENARIO_LOG_INFO() << "[시나리오5] 유효 후보 없음 → 종료";
        SCENARIO_LOG_INFO() << "==================== [시나리오5 종료] ====================";
        return;
    }

    SCENARIO_LOG_INFO() << "[시나리오5] 현재 프레임 후보 수=" << cand.size();

    // (2) 현재 프레임 후보 간 페어 탐지
    bool triggered = false;
    double best_pair_dm = std::numeric_limits<double>::max();
    adcm::obstacleListStruct best_a{}, best_b{};

    for (size_t ai = 0; ai < cand.size(); ++ai) {
        for (size_t bi = ai + 1; bi < cand.size(); ++bi) {
            const auto& a = cand[ai];
            const auto& b = cand[bi];

            const double pair_dist_dm = calculateDistance(a, b);
            SCENARIO_LOG_INFO() << "    · [검사] 페어(" << a.obstacle_id << "," << b.obstacle_id
                                << ") | 거리=" << (pair_dist_dm / 10.0) << " m";

            if (pair_dist_dm <= MAX_PAIR_DIST_DM && pair_dist_dm < best_pair_dm) {
                best_pair_dm = pair_dist_dm;
                best_a = a;
                best_b = b;
                triggered = true;
            }
        }
    }

    if (!triggered) {
        SCENARIO_LOG_INFO() << "[시나리오5] 현재 프레임: 유효(≤10m) 페어 없음";
        SCENARIO_LOG_INFO() << "==================== [시나리오5 종료] ====================";
        return;
    }

    // (3) 트리거 처리
    const double pair_m   = best_pair_dm / 10.0;
    const double base_conf = clampValue((MAX_PAIR_DIST_DM - best_pair_dm) / MAX_PAIR_DIST_DM, 0.0, 1.0);
    const double d_ego_a_dm = calculateDistance(best_a, ego_vehicle);
    const double d_ego_b_dm = calculateDistance(best_b, ego_vehicle);
    const double ego_conf_a = clampValue((EGO_MAX_DM - d_ego_a_dm) / EGO_MAX_DM, 0.0, 1.0);
    const double ego_conf_b = clampValue((EGO_MAX_DM - d_ego_b_dm) / EGO_MAX_DM, 0.0, 1.0);
    constexpr double W_PAIR = 0.7;
    constexpr double W_EGO  = 0.3;
    const double fused_conf_a = clampValue(W_PAIR * base_conf + W_EGO * ego_conf_a, 0.0, 1.0);
    const double fused_conf_b = clampValue(W_PAIR * base_conf + W_EGO * ego_conf_b, 0.0, 1.0);
    // const double dir_factor_a = calculateFrontRearFactor(best_a, ego_vehicle); // 전방/후방 가중치
    // const double dir_factor_b = calculateFrontRearFactor(best_b, ego_vehicle); // 전방/후방 가중치
    const double conf_a = fused_conf_a;
    const double conf_b = fused_conf_b;

    adcm::riskAssessmentStruct s5a{ .obstacle_id=best_a.obstacle_id, .hazard_class=SCENARIO_5, .confidence=conf_a };
    adcm::riskAssessmentStruct s5b{ .obstacle_id=best_b.obstacle_id, .hazard_class=SCENARIO_5, .confidence=conf_b };

    SCENARIO_LOG_INFO() << "[시나리오5 활성]"
                << " | 페어 IDs=(" << s5a.obstacle_id << "," << s5b.obstacle_id << ")"
                << " | 거리=" << pair_m << " m"
                << " | base_conf=" << base_conf
                << " | ego_dist_a=" << (d_ego_a_dm / 10.0) << " m"
                << " | ego_dist_b=" << (d_ego_b_dm / 10.0) << " m"
                << " | ego_conf_a=" << ego_conf_a
                << " | ego_conf_b=" << ego_conf_b
                << " | fused_conf_a=" << fused_conf_a
                << " | fused_conf_b=" << fused_conf_b
                << " | confidence=(" << conf_a << "," << conf_b << ")";

    riskAssessment.riskAssessmentList.push_back(s5a);
    riskAssessment.riskAssessmentList.push_back(s5b);

    SCENARIO_LOG_INFO() << "==================== [시나리오5 종료] ====================";
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
                       adcm::risk_assessment_Objects& riskAssessment,
                       std::uint8_t edge_state)
{
    using namespace adcm;
    SCENARIO_LOG_INFO() << "==================== [시나리오6 시작] ====================";

    if (edge_state != 3) {
        SCENARIO_LOG_INFO() << "[시나리오6] MOVE 상태 아님(" << static_cast<int>(edge_state) << ") → 종료";
        SCENARIO_LOG_INFO() << "==================== [시나리오6 종료] ====================";
        return;
    }

    constexpr double EGO_MIN_DM          = 100.0;   // 10 m
    constexpr double EGO_MAX_DM          = 600.0;   // 60 m
    constexpr double DIST_TO_PATH_MAX_DM = 150.0;   // 15 m
    constexpr double MAX_PAIR_DIST_DM    = 300.0;   // 30 m
    constexpr double MIN_TRIGGER_X       = 800.0;
    constexpr double MIN_TRIGGER_Y       = 400.0;

    if (obstacle_list.empty()) {
        SCENARIO_LOG_INFO() << "[시나리오6] 입력 없음 → 종료";
        return;
    }

    // 프레임 timestamp 범위
    double frame_ts_min = std::numeric_limits<double>::max();
    double frame_ts_max = std::numeric_limits<double>::lowest();
    for (auto &o : obstacle_list) {
        double ts = static_cast<double>(o.timestamp);
        frame_ts_min = std::min(frame_ts_min, ts);
        frame_ts_max = std::max(frame_ts_max, ts);
    }
    SCENARIO_LOG_INFO() << "[시나리오6] 프레임 N=" << obstacle_list.size()
                << " ts=[" << frame_ts_min << "," << frame_ts_max << "]";

    // (1) 후보 추출: 차량류 & 50~60m & 경로 15m
    obstacleListVector cand;
    for (auto &obs : obstacle_list) {
        if (obs.obstacle_class < 1 || obs.obstacle_class > 19) continue;

        const bool coord_in_range =
            (obs.fused_position_x >= MIN_TRIGGER_X) &&
            (obs.fused_position_y >= MIN_TRIGGER_Y);
        if (!coord_in_range) {
            SCENARIO_LOG_INFO() << "[6-reject] ID=" << obs.obstacle_id
                                << " | class=" << static_cast<int>(obs.obstacle_class)
                                << " | pos=(" << obs.fused_position_x << ", " << obs.fused_position_y << ")"
                                << " | threshold=(x>=" << MIN_TRIGGER_X
                                << ", y>=" << MIN_TRIGGER_Y << ")";
            continue;
        }

        double d_ego_dm = calculateDistance(obs, ego_vehicle);
        if (d_ego_dm < EGO_MIN_DM || d_ego_dm > EGO_MAX_DM) continue;

        double path_dist_dm=0.0;
        if (!calculateMinDistanceToPath(obs, path_x, path_y, path_dist_dm)) continue;
        if (path_dist_dm > DIST_TO_PATH_MAX_DM) continue;

        cand.push_back(obs);
        SCENARIO_LOG_INFO() << "  └─[후보차량] ID=" << obs.obstacle_id
                    << " ts=" << obs.timestamp
                    << " Ego거리=" << d_ego_dm/10.0 << "m"
                    << " 경로거리=" << path_dist_dm/10.0 << "m";
    }

    if (cand.size() < 2) {
        SCENARIO_LOG_INFO() << "[시나리오6] 후보차량 부족(2대 미만) → 종료";
        return;
    }

    // (2) 현재 프레임 후보 내 페어 탐지
    bool triggered = false;
    double best_pair_dm = std::numeric_limits<double>::max();
    adcm::obstacleListStruct best_a{}, best_b{};

    SCENARIO_LOG_INFO() << "[시나리오6] 페어 탐지 시작: 현재 프레임, 허용거리 ≤ 30 m";
    for (size_t ai = 0; ai < cand.size(); ++ai) {
        for (size_t bi = ai + 1; bi < cand.size(); ++bi) {
            const auto &a = cand[ai];
            const auto &b = cand[bi];
            const double pair_dist_dm = calculateDistance(a, b);

            SCENARIO_LOG_INFO() << "    · [검사] 차량쌍(" << a.obstacle_id << "," << b.obstacle_id << ")"
                                << " | 페어거리=" << (pair_dist_dm / 10.0) << " m";

            if (pair_dist_dm <= MAX_PAIR_DIST_DM && pair_dist_dm < best_pair_dm) {
                best_pair_dm = pair_dist_dm;
                best_a = a;
                best_b = b;
                triggered = true;
            }
        }
    }

    if (!triggered) {
        SCENARIO_LOG_INFO() << "[시나리오6] 유효 차량쌍 없음";
        return;
    }

    // (3) 트리거 처리
    // - 페어 양쪽 차량을 모두 결과에 추가
    const double d_ego_a_dm = calculateDistance(best_a, ego_vehicle);
    const double d_ego_b_dm = calculateDistance(best_b, ego_vehicle);
    const double pair_m     = best_pair_dm / 10.0;

    // confidence = 400dm / (ego_dist_dm) * 0.7  (각 차량별 계산)
    const double base_conf_a = (400.0 / std::max(d_ego_a_dm, 1.0)) * 0.7; // 0 보호
    const double base_conf_b = (400.0 / std::max(d_ego_b_dm, 1.0)) * 0.7; // 0 보호
    // const double dir_factor_a = calculateFrontRearFactor(best_a, ego_vehicle); // 전방/후방 가중치
    // const double dir_factor_b = calculateFrontRearFactor(best_b, ego_vehicle); // 전방/후방 가중치
    const double conf_a = clampValue(base_conf_a, 0.0, 1.0);
    const double conf_b = clampValue(base_conf_b, 0.0, 1.0);

    adcm::riskAssessmentStruct s6a{};
    s6a.obstacle_id  = best_a.obstacle_id;
    s6a.hazard_class = SCENARIO_6;
    s6a.confidence   = conf_a;

    adcm::riskAssessmentStruct s6b{};
    s6b.obstacle_id  = best_b.obstacle_id;
    s6b.hazard_class = SCENARIO_6;
    s6b.confidence   = conf_b;

    SCENARIO_LOG_INFO() << "[시나리오6 활성]"
                << " | 페어 IDs=(" << best_a.obstacle_id << "," << best_b.obstacle_id << ")"
                << " | 페어거리=" << pair_m << " m"
                << " | ID=" << best_a.obstacle_id
                << " Ego거리=" << (d_ego_a_dm / 10.0) << " m"
                << " confidence=" << conf_a
                << " (계산식: 400dm/" << d_ego_a_dm << " *0.7)"
                << " | ID=" << best_b.obstacle_id
                << " Ego거리=" << (d_ego_b_dm / 10.0) << " m"
                << " confidence=" << conf_b
                << " (계산식: 400dm/" << d_ego_b_dm << " *0.7)";

    riskAssessment.riskAssessmentList.push_back(s6a);
    riskAssessment.riskAssessmentList.push_back(s6b);
    SCENARIO_LOG_INFO()<<"==================== [시나리오6 종료] ====================";
}
/**
 * @brief 시나리오 7: 특정 ROI 내부 미스캔 노면 비율 기반 위험 판단
 *
 * 고정 ROI( x:[600,650], y:[520,540] )를 순회하여
 * UNSCANNED(0xFE) 셀 비율이 임계치 이상이면 시나리오7을 트리거한다.
 * 미스캔 비율은 매 평가마다 로깅한다.
 */
void evaluateScenario7(const std::vector<double>& path_x,
                       const std::vector<double>& path_y,
                       const std::vector<adcm::map_2dListVector>& map_2d,
                       const Config& config,
                       adcm::risk_assessment_Objects& riskAssessment,
                       std::uint8_t edge_state)
{
    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 7 START =============";

    if (edge_state != 3) {
        SCENARIO_LOG_INFO() << "[시나리오7] MOVE 상태 아님(" << static_cast<int>(edge_state) << ") → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 7 DONE =============";
        return;
    }

    auto isUnscanned = [&](uint8_t road_z)
    {
        return road_z == static_cast<uint8_t>(IndexToValue::UNSCANNED);
        // 0 ~ 22 : SCAN 완료
        // 0xFF : 작업영역 외/기본값
        // 0xFE : 스캔되지 않은 지역
    };

    const double kMinUnscannedRatio =
        clampValue(config.scenario7MinUnscannedRatio, 0.0, 1.0);
    constexpr int kRoiMinX = 600;
    constexpr int kRoiMaxX = 650;
    constexpr int kRoiMinY = 520;
    constexpr int kRoiMaxY = 540;

    adcm::riskAssessmentStruct r{};
    r.hazard_class = SCENARIO_7;
    r.hazard_path = true;

    SCENARIO_LOG_INFO() << "[시나리오7] ROI X=[" << kRoiMinX << ", " << kRoiMaxX
                        << "] Y=[" << kRoiMinY << ", " << kRoiMaxY
                        << "] | min_unscanned_ratio=" << kMinUnscannedRatio;

    const int map_width  = static_cast<int>(map_2d.size());
    const int map_height = map_width ? static_cast<int>(map_2d[0].size()) : 0;

    if (map_width <= 0 || map_height <= 0)
    {
        SCENARIO_LOG_INFO() << "[시나리오7] map_2d is empty -> 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 7 DONE =============";
        return;
    }

    const int roi_x_start = std::max(0, kRoiMinX);
    const int roi_x_end = std::min(kRoiMaxX, map_width - 1);
    const int roi_y_start = std::max(0, kRoiMinY);
    const int roi_y_end = std::min(kRoiMaxY, map_height - 1);

    if (roi_x_start > roi_x_end || roi_y_start > roi_y_end) {
        SCENARIO_LOG_INFO() << "[시나리오7] ROI is out of map bounds. "
                            << "map_size=(" << map_width << "," << map_height << ")";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 7 DONE =============";
        return;
    }

    int roi_total_cells = 0;
    int roi_unscanned_cells = 0;
    for (int x = roi_x_start; x <= roi_x_end; ++x) {
        for (int y = roi_y_start; y <= roi_y_end; ++y) {
            ++roi_total_cells;
            if (isUnscanned(map_2d[x][y].road_z)) {
                ++roi_unscanned_cells;
            }
        }
    }

    const double unscanned_ratio =
        (roi_total_cells > 0)
            ? static_cast<double>(roi_unscanned_cells) / static_cast<double>(roi_total_cells)
            : 0.0;

    SCENARIO_LOG_INFO() << "[시나리오7] ROI 미스캔 비율=" << unscanned_ratio
                        << " (unscanned=" << roi_unscanned_cells
                        << " / total=" << roi_total_cells
                        << ") | threshold=" << kMinUnscannedRatio
                        << " | roi_clamped=[X:" << roi_x_start << "~" << roi_x_end
                        << ", Y:" << roi_y_start << "~" << roi_y_end << "]";

    if (unscanned_ratio >= kMinUnscannedRatio)
    {
        bool found_path_in_roi = false;
        adcm::globalPathPosition start{};
        adcm::globalPathPosition end{};

        if (!path_x.empty() && path_x.size() == path_y.size()) {
            std::size_t first_idx = 0;
            std::size_t last_idx = 0;
            for (std::size_t i = 0; i < path_x.size(); ++i) {
                const bool inside_roi =
                    (path_x[i] >= static_cast<double>(roi_x_start) && path_x[i] <= static_cast<double>(roi_x_end) &&
                     path_y[i] >= static_cast<double>(roi_y_start) && path_y[i] <= static_cast<double>(roi_y_end));
                if (!inside_roi) {
                    continue;
                }
                if (!found_path_in_roi) {
                    first_idx = i;
                    last_idx = i;
                    found_path_in_roi = true;
                } else {
                    last_idx = i;
                }
            }

            if (found_path_in_roi) {
                const std::size_t start_idx = (first_idx > 0) ? (first_idx - 1) : first_idx;
                const std::size_t end_idx = (last_idx + 1 < path_x.size()) ? (last_idx + 1) : last_idx;
                start = adcm::globalPathPosition{path_x[start_idx], path_y[start_idx]};
                end = adcm::globalPathPosition{path_x[end_idx], path_y[end_idx]};
            }
        }

        // ROI 내부 경로 포인트가 없으면 기존 ROI 코너로 폴백
        if (!found_path_in_roi) {
            start = adcm::globalPathPosition{static_cast<double>(roi_x_start), static_cast<double>(roi_y_start)};
            end = adcm::globalPathPosition{static_cast<double>(roi_x_end), static_cast<double>(roi_y_end)};
        }

        r.hazard_path_start.push_back(start);
        r.hazard_path_end.push_back(end);

        SCENARIO_LOG_INFO() << "[시나리오7] Triggered by ROI ratio."
                            << " ratio=" << unscanned_ratio
                            << " threshold=" << kMinUnscannedRatio
                            << " | start=(" << start.x << "," << start.y << ")"
                            << " end=(" << end.x << "," << end.y << ")"
                            << " | path_based=" << (found_path_in_roi ? "true" : "false");

        riskAssessment.riskAssessmentList.push_back(r);
    }
    else
    {
        SCENARIO_LOG_INFO() << "[시나리오7] No trigger by ROI ratio."
                            << " ratio=" << unscanned_ratio
                            << " threshold=" << kMinUnscannedRatio;
    }

    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 7 DONE =============";
}

namespace {
    bool s8_triggered_once = false;
}

void evaluateScenario8(const std::vector<double>& path_x, 
                       const std::vector<double>& path_y, 
                       const std::vector<adcm::map_2dListVector>& map_2d, 
                       adcm::risk_assessment_Objects& riskAssessment,
                       std::uint8_t edge_state)
{
    SCENARIO_LOG_INFO() << "=============KATECH: scenario 8 START==============";

    if (edge_state != 3) {
        SCENARIO_LOG_INFO() << "[시나리오8] MOVE 상태 아님(" << static_cast<int>(edge_state) << ") → 종료";
        SCENARIO_LOG_INFO() << "=============KATECH: scenario 8 DONE==============";
        return;
    }

    if (s8_triggered_once) {
        SCENARIO_LOG_INFO() << "Scenario 8 already triggered once -> skip re-evaluation";
        SCENARIO_LOG_INFO() << "=============KATECH: scenario 8 DONE==============";
        return;
    }

    // =========================
    // Scenario 8: 급경사(주행 불가 수준) 검출
    // - road_z: RoadZHeight enum (0~22 valid, 0xFF OUT)
    // - ROI: 전역경로 주변 SHIFT_DISTANCE 내 스캔
    // - 판정: slope >= 0.8 (80% 경사)인 급격한 높이 변화가 ROI 내에서 일정 횟수 이상 발견되면 Hazard
    // =========================

    // 파라미터
    constexpr int    SHIFT_DISTANCE   = 50;     // 전역경로 근방 5m (10cm/cell 가정)
    constexpr int    STEP             = 10;      // 5셀 = 50cm 간격에서 높이 변화 측정
    constexpr double CELL_SIZE_CM     = 10.0;   // 1셀 = 10cm
    constexpr double BIN_CM           = 5.0;    // road_z 1단계(bin) = 5cm
    constexpr double SLOPE_HARD       = 0.9;    // 80% 경사 이상이면 위험(주행 안정성 저해/불가 수준)
    constexpr int    RISK_HIT_THRESH  = 3;      // ROI 내 급경사 발견 횟수 임계(노이즈 방지)

    // slope = dz_cm / dist_cm
    // dz_cm = |bin1 - bin0| * BIN_CM
    // dist_cm = STEP * CELL_SIZE_CM
    // slope >= SLOPE_HARD  <=> |bin1 - bin0| >= SLOPE_HARD * dist_cm / BIN_CM
    constexpr int BIN_DIFF_THRESHOLD =
        static_cast<int>(std::ceil(SLOPE_HARD * (STEP * CELL_SIZE_CM) / BIN_CM)); // 예: 0.8*50/5=8

    Point2D p1{}, p2{}, p3{}, p4{};
    const int width  = static_cast<int>(map_2d.size());
    const int height = width ? static_cast<int>(map_2d[0].size()) : 0;

    auto isValidZ = [&](uint8_t z) -> bool {
        return z <= static_cast<uint8_t>(RoadIndex::GE_POS_55); // 0~22
    };

    adcm::riskAssessmentStruct out{};
    bool has_uneven = false;
    out.hazard_class = SCENARIO_8;
    out.hazard_path = true;

    for (size_t i = 0; i + 1 < path_x.size(); ++i) {

        // 결과 저장용 원본 좌표는 변형하지 않고 유지한다.
        const double seg_start_x = path_x[i];
        const double seg_start_y = path_y[i];
        const double seg_end_x   = path_x[i + 1];
        const double seg_end_y   = path_y[i + 1];

        // 내부 그리드 인덱스 계산은 floor 편향을 줄이기 위해 반올림 사용.
        int x_start = static_cast<int>(std::lround(seg_start_x));
        int x_end   = static_cast<int>(std::lround(seg_end_x));
        int y_start = static_cast<int>(std::lround(seg_start_y));
        int y_end   = static_cast<int>(std::lround(seg_end_y));

        double original_m = 0, original_c = 0, up_c = 0, down_c = 0, x_up = 0, x_down = 0;
        bool isVertical = false;

        calculateShiftedLines(x_start, x_end, y_start, y_end, SHIFT_DISTANCE,
                              original_m, original_c, up_c, down_c, isVertical, x_up, x_down);

        // ROI(평행사변형) 4점 구성
        if (isVertical) {
            p1 = {x_down, y_start};
            p2 = {x_down, y_end};
            p3 = {x_up,   y_start};
            p4 = {x_up,   y_end};
        } else {
            p1 = {x_start, original_m * x_start + up_c};
            p2 = {x_end,   original_m * x_end   + up_c};
            p3 = {x_start, original_m * x_start + down_c};
            p4 = {x_end,   original_m * x_end   + down_c};
        }

        // 바운딩 박스 (double → int 인덱스)
        int minX = std::max(0,        static_cast<int>(std::floor(std::min({p1.x, p2.x, p3.x, p4.x}))));
        int maxX = std::min(width-1,  static_cast<int>(std::ceil (std::max({p1.x, p2.x, p3.x, p4.x}))));
        int minY = std::max(0,        static_cast<int>(std::floor(std::min({p1.y, p2.y, p3.y, p4.y}))));
        int maxY = std::min(height-1, static_cast<int>(std::ceil (std::max({p1.y, p2.y, p3.y, p4.y}))));

        int hit = 0;

        // === 급경사 스캔 시작 ===
        for (int x = minX; x <= maxX && hit < RISK_HIT_THRESH; ++x) {
            for (int y = minY; y <= maxY && hit < RISK_HIT_THRESH; ++y) {

                const uint8_t z0_u = map_2d[x][y].road_z;
                if (!isValidZ(z0_u)) continue;

                const int z0 = static_cast<int>(z0_u);

                // +X 방향 (x + STEP, y)
                if (x + STEP <= maxX) {
                    const uint8_t z1_u = map_2d[x + STEP][y].road_z;
                    if (isValidZ(z1_u)) {
                        const int dz_bin = std::abs(static_cast<int>(z1_u) - z0);
                        if (dz_bin >= BIN_DIFF_THRESHOLD) {
                            ++hit;
                            // 필요 시 디버깅 로그
                            SCENARIO_LOG_INFO() << "Steep slope hit(+X): seg=" << i
                                                << " at (" << x << "," << y << ") dz_bin=" << dz_bin;
                        }
                    }
                }

                // +Y 방향 (x, y + STEP)
                if (y + STEP <= maxY) {
                    const uint8_t z1_u = map_2d[x][y + STEP].road_z;
                    if (isValidZ(z1_u)) {
                        const int dz_bin = std::abs(static_cast<int>(z1_u) - z0);
                        if (dz_bin >= BIN_DIFF_THRESHOLD) {
                            ++hit;
                            SCENARIO_LOG_INFO() << "Steep slope hit(+Y): seg=" << i
                                                << " at (" << x << "," << y << ") dz_bin=" << dz_bin;
                        }
                    }
                }

                // (선택) 대각선: 더 강하게 잡고 싶으면 활성화
                if (x + STEP <= maxX && y + STEP <= maxY) {
                    const uint8_t z1_u = map_2d[x + STEP][y + STEP].road_z;
                    if (isValidZ(z1_u)) {
                        const int dz_bin = std::abs(static_cast<int>(z1_u) - z0);
                        if (dz_bin >= BIN_DIFF_THRESHOLD) {
                            ++hit;
                            SCENARIO_LOG_INFO() << "Steep slope hit(DIAG): seg=" << i
                                                << " at (" << x << "," << y << ") dz_bin=" << dz_bin;
                        }
                    }
                }
            }
        }

        if (hit >= RISK_HIT_THRESH) {
            SCENARIO_LOG_INFO() << "Scenario 8 steep-slope hazard detected at segment " << i
                                << " (hit=" << hit << ", bin_diff_th=" << BIN_DIFF_THRESHOLD << ")";

            out.hazard_path_start.push_back(adcm::globalPathPosition{ seg_start_x, seg_start_y });
            out.hazard_path_end.push_back  (adcm::globalPathPosition{ seg_end_x,   seg_end_y   });
            has_uneven = true;
            // break; // 필요 시 첫 검출 후 종료
        }
    }

    if (has_uneven)
    {
        SCENARIO_LOG_INFO() << "Scenario 8 total hazard segments: " << out.hazard_path_start.size();
        riskAssessment.riskAssessmentList.push_back(std::move(out));
        s8_triggered_once = true;
        SCENARIO_LOG_INFO() << "Scenario 8 latch enabled: future triggers are suppressed";
    }

    SCENARIO_LOG_INFO() << "=============KATECH: scenario 8 DONE==============";
}


/*
void evaluateScenario8(const std::vector<double>& path_x, 
                       const std::vector<double>& path_y, 
                       const std::vector<adcm::map_2dListVector>& map_2d, 
                       adcm::risk_assessment_Objects& riskAssessment)
{
    SCENARIO_LOG_INFO() << "=============KATECH: scenario 8 START==============";

    // 파라미터
    constexpr int SHIFT_DISTANCE = 50;            // 전역경로 근방 5m
    constexpr int RISK_THRESHOLD = 20;            // 위험 카운트 임계값
    constexpr double HEIGHT_DIFF_CM = 30.0;       // 높이차 30cm 이상
    constexpr double SLOPE_THRESHOLD = 0.20;      // 기울기 20% 이상
    constexpr int STEP = 5;                       // 5셀 = 50cm
    constexpr double STDDEV_THRESHOLD = 15.0;     // 표준편차 15cm 이상
    constexpr double CELL_SIZE_CM = 10.0;         // 1셀 = 10cm

    Point2D p1{}, p2{}, p3{}, p4{};
    const int width  = static_cast<int>(map_2d.size());
    const int height = width ? static_cast<int>(map_2d[0].size()) : 0;

    for (size_t i = 0; i < path_x.size() - 1; ++i) {

        int x_start = static_cast<int>(floor(path_x[i]));
        int x_end   = static_cast<int>(floor(path_x[i + 1]));
        int y_start = static_cast<int>(floor(path_y[i]));
        int y_end   = static_cast<int>(floor(path_y[i + 1]));

        double original_m = 0, original_c = 0, up_c = 0, down_c = 0, x_up = 0, x_down = 0;
        bool isVertical = false;
        calculateShiftedLines(x_start, x_end, y_start, y_end, SHIFT_DISTANCE, 
                              original_m, original_c, up_c, down_c, isVertical, x_up, x_down);
        // Print the line equations
        if (isVertical) 
        {
            // SCENARIO_LOG_INFO()<< "Original line equation: x = " << x_start << "\n";
            // SCENARIO_LOG_INFO() << "Line shifted right: x = " << x_up << "\n";
            // SCENARIO_LOG_INFO() << "Line shifted left: x = " << x_down << "\n";

            p1 = {x_down, y_start};  // First line start point
            p2 = {x_down, y_end}; // First line end point
            p3 = {x_up, y_start};   // Second line start point
            p4 = {x_up, y_end}; // Second line end point
        } 
        else 
        {
            // SCENARIO_LOG_INFO() << "Original line equation: y = " << original_m << "x + " << original_c << "\n";
            // SCENARIO_LOG_INFO() << "Line shifted up: y = " << original_m << "x + " << up_c << "\n";
            // SCENARIO_LOG_INFO() << "Line shifted down: y = " << original_m << "x + " << down_c << "\n";

            // Points of the parallelogram (calculated for x = 0 and x = 10)
            p1 = {x_start, original_m * x_start + up_c};  // First line start point
            p2 = {x_end, original_m * x_end + up_c}; // First line end point
            p3 = {x_start, original_m * x_start + down_c};   // Second line start point
            p4 = {x_end, original_m * x_end + down_c}; // Second line end point
        }
      
        // 바운딩 박스 (double → int 인덱스 변환)
        int minX = std::max(0, static_cast<int>(std::floor(std::min({p1.x, p2.x, p3.x, p4.x}))));
        int maxX = std::min(width-1,  static_cast<int>(std::ceil (std::max({p1.x, p2.x, p3.x, p4.x}))));
        int minY = std::max(0, static_cast<int>(std::floor(std::min({p1.y, p2.y, p3.y, p4.y}))));
        int maxY = std::min(height-1, static_cast<int>(std::ceil (std::max({p1.y, p2.y, p3.y, p4.y}))));
        
        // 위험 카운트 & 표준편차 계산을 위한 샘플
        int risk_count = 0;
        std::vector<double> z_cm_samples;
        z_cm_samples.reserve(std::max(1, (maxX - minX + 1) * (maxY - minY + 1)));        
        
        // === 스캔 시작 ===
        for (int x = minX; x <= maxX && risk_count < RISK_THRESHOLD; ++x) {
            for (int y = minY; y <= maxY && risk_count < RISK_THRESHOLD; ++y) {

                const uint8_t z_xy = map_2d[x][y].road_z;
                double z_xy_cm = 0.0;
                if (!isOutOfWork(z_xy) && zToCm(z_xy, z_xy_cm)) {
                    z_cm_samples.push_back(z_xy_cm);
                } else {
                    // 작업영역 외/무효는 스킵
                    continue;
                }

                // 1) 인접 셀 높이차 30cm 이상 (x-1, y 동일 / x 동일, y-1)
                if (x > minX) {
                    const uint8_t z_prev = map_2d[x - 1][y].road_z;
                    double z_prev_cm;
                    if (!isOutOfWork(z_prev) && zToCm(z_prev, z_prev_cm)) {
                        const double dz_cm = std::fabs(z_xy_cm - z_prev_cm);
                        if (dz_cm >= HEIGHT_DIFF_CM) ++risk_count;
                    }
                }
                if (y > minY) {
                    const uint8_t z_prev = map_2d[x][y - 1].road_z;
                    double z_prev_cm;
                    if (!isOutOfWork(z_prev) && zToCm(z_prev, z_prev_cm)) {
                        const double dz_cm = std::fabs(z_xy_cm - z_prev_cm);
                        if (dz_cm >= HEIGHT_DIFF_CM) ++risk_count;
                    }
                }

                // 2) 기울기 20% 이상 (STEP 셀 간격: 5셀=50cm)
                if (x + STEP <= maxX) {
                    const uint8_t z_far = map_2d[x + STEP][y].road_z;
                    double z_far_cm;
                    if (!isOutOfWork(z_far) && zToCm(z_far, z_far_cm)) {
                        const double dz_cm = std::fabs(z_far_cm - z_xy_cm);
                        const double slope = dz_cm / (STEP * CELL_SIZE_CM);
                        if (slope >= SLOPE_THRESHOLD) ++risk_count;
                    }
                }
                if (y + STEP <= maxY) {
                    const uint8_t z_far = map_2d[x][y + STEP].road_z;
                    double z_far_cm;
                    if (!isOutOfWork(z_far) && zToCm(z_far, z_far_cm)) {
                        const double dz_cm = std::fabs(z_far_cm - z_xy_cm);
                        const double slope = dz_cm / (STEP * CELL_SIZE_CM);
                        if (slope >= SLOPE_THRESHOLD) ++risk_count;
                    }
                }
            }
        }

        // 3) 표준편차 15cm 이상
        if (!z_cm_samples.empty()) {
            const double mean = std::accumulate(z_cm_samples.begin(), z_cm_samples.end(), 0.0) /
                                static_cast<double>(z_cm_samples.size());
            double sq_sum = 0.0;
            for (double v : z_cm_samples) {
                const double d = v - mean;
                sq_sum += d * d;
            }
            const double stddev = std::sqrt(sq_sum / static_cast<double>(z_cm_samples.size()));
            if (stddev >= STDDEV_THRESHOLD) {
                risk_count += RISK_THRESHOLD; // 강제 위험 판정
            }
        }

        if (risk_count >= RISK_THRESHOLD) {
            SCENARIO_LOG_INFO() << "Scenario 8 hazard detected at segment " << i;

            adcm::riskAssessmentStruct out;
            out.hazard_path_start.push_back(adcm::globalPathPosition{ path_x[i],     path_y[i]     });
            out.hazard_path_end.push_back  (adcm::globalPathPosition{ path_x[i + 1], path_y[i + 1] });
            out.hazard_class = SCENARIO_8;
            out.hazard_path = true;

            SCENARIO_LOG_INFO() << "Risk assessment generated for #8 at X: " 
                                << path_x[i] << ", Y: " << path_y[i];

            riskAssessment.riskAssessmentList.push_back(std::move(out));
            //break;
        }
    }

    SCENARIO_LOG_INFO() << "=============KATECH: scenario 8 DONE==============";

}
*/
//===== 시나리오 #9. 목적지 반경 30m 내 사각영역 유발 위험 판단 (정차 차량 포함 높이 weight) =====
void evaluateScenario9(const obstacleListVector& obstacle_list,
                       const adcm::vehicleListStruct& ego_vehicle,
                       const std::vector<double>& path_x,
                       const std::vector<double>& path_y,
                       adcm::risk_assessment_Objects& riskAssessment)
{
    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 9 START =============";

    if (path_x.empty() || path_y.empty() || path_x.size() != path_y.size()) {
        SCENARIO_LOG_INFO() << "[시나리오9] 전역경로 비정상 → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 9 DONE =============";
        return;
    }

    // 목적지 좌표 = 경로의 마지막 점
    const double goal_x = path_x.back();
    const double goal_y = path_y.back();

    // 단위: dm(0.1m)
    constexpr double GOAL_RADIUS_DM = 300.0;  // 30m
    constexpr double EGO_THRESH_DM  = 400.0;  // 40m

    // 단위: m
    constexpr double HEIGHT_MAX_M   = 5.0;    // 5m 이상이면 s_height=1

    // 가중치(합계 = 1.0): 목적지/ego/높이
    constexpr double W_GOAL   = 0.5;
    constexpr double W_EGO    = 0.3;
    constexpr double W_HEIGHT = 0.2;

    // Ego가 목적지 반경 30m 이내인지 선행 확인
    const double ego_dx = (ego_vehicle.position_x - goal_x); //dm
    const double ego_dy = (ego_vehicle.position_y - goal_y); //dm
    const double ego_to_goal_dm = std::sqrt(ego_dx*ego_dx + ego_dy*ego_dy);
    if (ego_to_goal_dm > GOAL_RADIUS_DM) {
        SCENARIO_LOG_INFO() << "[시나리오9] Ego 목적지 반경 밖(" << (ego_to_goal_dm/10.0) << " m) → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 9 DONE =============";
        return;
    }

    obstacleListVector candidates;
    candidates.reserve(obstacle_list.size());

    for (const auto& obs : obstacle_list) {
        // (i) 정차 차량 OR 높이 2m 이상 정적 장애물
        const bool cond_vehicle_stopped =
            (obs.obstacle_class >= 1 && obs.obstacle_class <= 19 && obs.stop_count >= gStopValue);
        const bool cond_static_high =
            (obs.obstacle_class >= 30 && obs.obstacle_class <= 49 && obs.fused_cuboid_z > HEIGHT_THRESH_M);
        if (!(cond_vehicle_stopped || cond_static_high)) continue;

        // (ii) 목적지 반경 30m 이내
        const double d_goal_dm = distanceObsToPointDm(obs, goal_x, goal_y);
        if (d_goal_dm > GOAL_RADIUS_DM) continue;

        SCENARIO_LOG_INFO() << "[9 후보] ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | stop_count=" << obs.stop_count
                          << " | height=" << obs.fused_cuboid_z
                          << " | 목적지거리=" << (d_goal_dm/10.0) << " m";
        candidates.push_back(obs);
    }

    if (candidates.empty()) {
        SCENARIO_LOG_INFO() << "[시나리오9] 후보 없음 → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 9 DONE =============";
        return;
    }

    SCENARIO_LOG_INFO() << "[시나리오9] 최종 후보군 " << candidates.size() << "개";

    for (const auto& obs : candidates) {
        // 목적지 근접도 (30m에서 0, 가까울수록 1)
        const double d_goal_dm = distanceObsToPointDm(obs, goal_x, goal_y);
        const double s_goal = clampValue((GOAL_RADIUS_DM - d_goal_dm) / GOAL_RADIUS_DM, 0.0, 1.0);

        // Ego 근접도 (40m에서 0, 가까울수록 1)
        const double d_ego_dm = calculateDistance(obs, ego_vehicle);
        const double s_ego = clampValue((EGO_THRESH_DM - d_ego_dm) / EGO_THRESH_DM, 0.0, 1.0);

        // 높이 점수: 1m → 0, 5m 이상 → 1 (정차 차량과 정적 장애물 모두 적용)
        double s_height = 0.0;
        s_height = clampValue((obs.fused_cuboid_z - HEIGHT_THRESH_M) / (HEIGHT_MAX_M - HEIGHT_THRESH_M), 0.0, 1.0);
    
        // 목적지 근접 시나리오 특성상 기본 위험도를 반영해 과도한 저평가를 방지
        constexpr double SCENARIO9_PRIOR = 0.60;
        const double base_confidence = clampValue(W_GOAL * s_goal +
                              W_EGO  * s_ego  +
                              W_HEIGHT * s_height, 0.0, 1.0);
        double confidence = clampValue(SCENARIO9_PRIOR + (1.0 - SCENARIO9_PRIOR) * base_confidence, 0.0, 1.0);

        adcm::riskAssessmentStruct r{};
        r.obstacle_id  = obs.obstacle_id;
        r.hazard_class = SCENARIO_9;
        r.confidence   = confidence;

        SCENARIO_LOG_INFO() << "[시나리오9 활성] ID=" << obs.obstacle_id
                          << " | 목적지 근접도=" << s_goal
                          << ", ego 근접도=" << s_ego
                          << ", 높이점수 =" << s_height
                          << " | confidence=" << confidence;

        riskAssessment.riskAssessmentList.push_back(r);
    }

    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 9 DONE =============";
}

//===== 시나리오 #10. 목적지 반경 30m 내 정지 동적 장애물 기반 위험 판단 =====
void evaluateScenario10(const obstacleListVector& obstacle_list,
                        const adcm::vehicleListStruct& ego_vehicle,
                        const std::vector<double>& path_x,
                        const std::vector<double>& path_y,
                        adcm::risk_assessment_Objects& riskAssessment)
{
    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 10 START =============";

    if (path_x.empty() || path_y.empty() || path_x.size() != path_y.size()) {
        SCENARIO_LOG_INFO() << "[시나리오10] 전역경로 비정상 → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 10 DONE =============";
        return;
    }

    // 목적지 좌표 = 경로의 마지막 점
    const double goal_x = path_x.back();
    const double goal_y = path_y.back();

    // 단위: dm
    constexpr double GOAL_RADIUS_DM = 300.0;  // 30m
    constexpr double EGO_THRESH_DM  = 400.0;  // 40m
    constexpr int    STOP_THRESH    = 10;     // stop_count 기준값
    constexpr int    STOP_MAX       = 50;     // stop_count 최대 정규화 기준 (튜닝 포인트)

    // 가중치 (합 = 1.0)
    constexpr double W_GOAL  = 0.5;
    constexpr double W_EGO   = 0.4;
    constexpr double W_STOP  = 0.1;

    // Ego가 목적지 반경 30m 이내인지 확인
    double ego_to_goal_dm = distanceEgoToPointDm(ego_vehicle, goal_x, goal_y);
    if (ego_to_goal_dm > GOAL_RADIUS_DM) {
        SCENARIO_LOG_INFO() << "[시나리오10] Ego 목적지 반경 밖(" << (ego_to_goal_dm/10.0) << " m) → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 10 DONE =============";
        return;
    }

    obstacleListVector candidates;
    for (const auto& obs : obstacle_list) {
        // (i) 동적 장애물 + 정지 상태 (stop_count >= 10)
        const bool cond_stopped_vehicle =
            (obs.obstacle_class >= 1 && obs.obstacle_class <= 19 && obs.stop_count >= STOP_THRESH);
        if (!cond_stopped_vehicle) continue;

        // (ii) 목적지 반경 30m 이내
        const double d_goal_dm = distanceObsToPointDm(obs, goal_x, goal_y);
        if (d_goal_dm > GOAL_RADIUS_DM) continue;

        SCENARIO_LOG_INFO() << "[10 후보] ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | stop_count=" << obs.stop_count
                          << " | 목적지거리=" << (d_goal_dm/10.0) << " m";
        candidates.push_back(obs);
    }

    if (candidates.empty()) {
        SCENARIO_LOG_INFO() << "[시나리오10] 후보 없음 → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 10 DONE =============";
        return;
    }

    SCENARIO_LOG_INFO() << "[시나리오10] 최종 후보군 " << candidates.size() << "개";

    for (const auto& obs : candidates) {
        // (1) 목적지 근접도
        const double d_goal_dm = distanceObsToPointDm(obs, goal_x, goal_y);
        const double s_goal = clampValue((GOAL_RADIUS_DM - d_goal_dm) / GOAL_RADIUS_DM, 0.0, 1.0);

        // (2) Ego 근접도
        const double d_ego_dm = calculateDistance(obs, ego_vehicle);
        const double s_ego = clampValue((EGO_THRESH_DM - d_ego_dm) / EGO_THRESH_DM, 0.0, 1.0);

        // (3) 정차 시간 점수: stop_count=10 → 0, stop_count=50 이상 → 1
        double s_stop = clampValue((obs.stop_count - STOP_THRESH) / double(STOP_MAX - STOP_THRESH), 0.0, 1.0);

        // 목적지 근접 정체 시나리오 특성상 기본 위험도를 반영해 과도한 저평가를 방지
        constexpr double SCENARIO10_PRIOR = 0.60;
        const double base_confidence = clampValue(W_GOAL * s_goal +
                              W_EGO  * s_ego  +
                              W_STOP * s_stop, 0.0, 1.0);
        double confidence = clampValue(SCENARIO10_PRIOR + (1.0 - SCENARIO10_PRIOR) * base_confidence, 0.0, 1.0);

        adcm::riskAssessmentStruct r{};
        r.obstacle_id  = obs.obstacle_id;
        r.hazard_class = SCENARIO_10;
        r.confidence   = confidence;

        SCENARIO_LOG_INFO() << "[시나리오10 활성] ID=" << obs.obstacle_id
                          << " | s_goal=" << s_goal
                          << ", s_ego=" << s_ego
                          << ", s_stop=" << s_stop
                          << " | confidence=" << confidence;

        riskAssessment.riskAssessmentList.push_back(r);
    }

    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 10 DONE =============";
}
