#include "main_riskassessment.hpp"
namespace {
inline bool scenarioLogEnabled() { return gScenarioLogEnabled; }
}

#define SCENARIO_LOG_INFO() if (!scenarioLogEnabled()) ; else adcm::Log::Info()

//=====시나리오 #1. 주행중 전역경로 근방 이동가능한 정지 장애물이 존재하는 위험 환경=====
void evaluateScenario1(const obstacleListVector& obstacle_list, 
                        const adcm::vehicleListStruct& ego_vehicle, 
                        const std::vector<double>& path_x,
                        const std::vector<double>& path_y,
                        adcm::risk_assessment_Objects& riskAssessment)
{
    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 1 START =============";

    // 단위: 거리 dm(0.1 m), 시간 ms
    constexpr double DIST_TO_EGO_MAX_DM  = 300.0; // 30 m 이내
    constexpr double DIST_TO_PATH_MAX_DM = 100.0; // 10 m 이내

    // 컨피던스 파라미터
    constexpr double EGO_THRESH_DM  = 300.0;
    constexpr double PATH_THRESH_DM = 100.0;
    constexpr double W_EGO  = 0.6;
    constexpr double W_PATH = 0.4;

    obstacleListVector candidates; // 최종 후보군

    // === (i)~(iii) 조건 검사 ===
    for (const auto& obs : obstacle_list) {
        // (i) 동적 장애물 & 정지 상태
        const bool is_vehicle = (obs.obstacle_class >= 1 && obs.obstacle_class <= 21);
        if (!is_vehicle) continue;
        //if (obs.stop_count < STOP_VALUE) continue;

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
        double confidence   = clampValue(W_EGO * s_ego + W_PATH * s_path, 0.0, 1.0);

        adcm::riskAssessmentStruct r{};
        r.obstacle_id  = obs.obstacle_id;
        r.hazard_class = SCENARIO_1;
        r.confidence   = confidence;

        SCENARIO_LOG_INFO() << "[시나리오1 TRIGGER] ID=" << obs.obstacle_id
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
                       adcm::risk_assessment_Objects& riskAssessment)
{
    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 2 START =============";

    // 단위: 거리 dm(0.1m)
    constexpr double DIST_TO_EGO_MAX_DM  = 400.0; // 40 m 이내
    constexpr double DIST_TO_PATH_MAX_DM = 100.0; // 10 m 이내

    // 컨피던스 파라미터
    constexpr double EGO_THRESH_DM  = 400.0;
    constexpr double PATH_THRESH_DM = 100.0;
    constexpr double W_EGO  = 0.7;
    constexpr double W_PATH = 0.3;

    obstacleListVector candidates;

    for (const auto& obs : obstacle_list) {
        // ----------------------------------------------------
        // (i) 조건: 차량이면서 정지 상태 OR 높이 2m 이상 정적 장애물
        bool cond_vehicle_stopped = (obs.obstacle_class >= 1 && obs.obstacle_class <= 19 && obs.stop_count >= STOP_VALUE);
        bool cond_static_high     = (obs.obstacle_class >= 30 && obs.obstacle_class <= 49 && obs.fused_cuboid_z > HEIGHT_THRESH_M);

        if (!(cond_vehicle_stopped || cond_static_high)) continue;

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
        // SCENARIO_LOG_INFO() << "[2-iii] 통과: ID=" << obs.obstacle_id
        //                   << " | 경로거리=" << (d_path_dm/10.0) << " m";

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
        double confidence   = clampValue(W_EGO * s_ego + W_PATH * s_path, 0.0, 1.0);

        adcm::riskAssessmentStruct r{};
        r.obstacle_id  = obs.obstacle_id;
        r.hazard_class = SCENARIO_2;
        r.confidence   = confidence;

        SCENARIO_LOG_INFO() << "[시나리오2 TRIGGER] ID=" << obs.obstacle_id
                          << " | s_ego=" << s_ego << ", s_path=" << s_path
                          << " | confidence=" << confidence;

        riskAssessment.riskAssessmentList.push_back(r);
    }

    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 2 DONE =============";
}//===== 시나리오 #3. 주행중 경로 주변 동적 장애물 통행 환경 =====
void evaluateScenario3(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       adcm::risk_assessment_Objects& riskAssessment)
{
    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 3 START =============";

    // 단위: 거리 dm(0.1 m), 시간 s
    constexpr double MIN_DIST_DM = 100.0;   // 10 m
    constexpr double MAX_DIST_DM = 300.0;   // 30 m
    constexpr double TTC_TH_S    = 10.0;    // 10 s

    // 컨피던스 가중치 (튜닝 포인트)
    constexpr double W_TTC  = 0.7;
    constexpr double W_DIST = 0.3;

    obstacleListVector candidates;

    // (i) 기본 필터: 동적 클래스 & 실제 이동중 & Ego 거리 10~30m
    for (const auto& obs : obstacle_list) {
        const bool dynamic_class = (obs.obstacle_class >= 1 && obs.obstacle_class <= 29);
        const bool moving        = (obs.stop_count < STOP_VALUE);
        if (!dynamic_class || !moving) continue;

        const double d_ego_dm = calculateDistance(obs, ego_vehicle);
        if (d_ego_dm < MIN_DIST_DM || d_ego_dm > MAX_DIST_DM) continue;

        SCENARIO_LOG_INFO() << "[3-i] 통과: ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | stop_count=" << obs.stop_count
                          << " | Ego거리=" << (d_ego_dm/10.0) << " m";

        // (ii) TTC & 선형 최소거리 평가 → 통과한 것만 candidates에 편입
        double ttc = 0.0, d_lin_dm = 0.0;
        const bool ttc_ok  = (getTTC(obs, ego_vehicle, ttc) && ttc < TTC_TH_S);
        const bool dist_ok = (calculateMinDistanceLinear(obs, ego_vehicle, d_lin_dm) && d_lin_dm < MAX_DIST_DM);

        if (!ttc_ok || !dist_ok) {
            SCENARIO_LOG_INFO() << "[3-ii 제외] ID=" << obs.obstacle_id
                              << " | TTC=" << ttc << " s (th=" << TTC_TH_S << ")"
                              << " | linDist=" << (d_lin_dm/10.0) << " m (max=" << (MAX_DIST_DM/10.0) << ")";
            continue;
        }

        SCENARIO_LOG_INFO() << "[3-ii] 통과: ID=" << obs.obstacle_id
                          << " | TTC=" << ttc << " s"
                          << " | linDist=" << (d_lin_dm/10.0) << " m";

        candidates.push_back(obs);
    }

    // === 최종 후보군 로그 (컨피던스 계산 전에 전부 출력) ===
    if (candidates.empty()) {
        SCENARIO_LOG_INFO() << "[시나리오3] 최종 후보 없음 → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 3 DONE =============";
        return;
    }

    SCENARIO_LOG_INFO() << "[시나리오3] 최종 후보군 " << candidates.size() << "개:";
    for (const auto& obs : candidates) {
        double ttc = 0.0, d_lin_dm = 0.0;
        double d_ego_dm = calculateDistance(obs, ego_vehicle);
        bool ttcr = getTTC(obs, ego_vehicle, ttc);
        bool linr = calculateMinDistanceLinear(obs, ego_vehicle, d_lin_dm);
        SCENARIO_LOG_INFO() << "   - ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | Ego거리=" << (d_ego_dm/10.0) << " m"
                          << " | TTC=" << (ttcr ? ttc : -1.0) << " s"
                          << " | linDist=" << (linr ? (d_lin_dm/10.0) : -1.0) << " m";
    }

    // === 컨피던스 계산 및 결과 등록 ===
    for (const auto& obs : candidates) {
        double ttc = 0.0, d_lin_dm = 0.0;
        (void)getTTC(obs, ego_vehicle, ttc);
        (void)calculateMinDistanceLinear(obs, ego_vehicle, d_lin_dm);

        // 정규화 (0~1) — TTC 작을수록↑, 거리 가까울수록↑
        double s_ttc  = clampValue((TTC_TH_S - ttc) / TTC_TH_S, 0.0, 1.0);
        double s_dist = clampValue(1.0 - ((d_lin_dm - MIN_DIST_DM) / (MAX_DIST_DM - MIN_DIST_DM)), 0.0, 1.0);

        double confidence = clampValue(W_TTC * s_ttc + W_DIST * s_dist, 0.0, 1.0);

        adcm::riskAssessmentStruct r{};
        r.obstacle_id  = obs.obstacle_id;
        r.hazard_class = SCENARIO_3;
        r.confidence   = confidence;

        SCENARIO_LOG_INFO() << "[시나리오3 TRIGGER] ID=" << obs.obstacle_id
                          << " | s_ttc=" << s_ttc
                          << " | s_dist=" << s_dist
                          << " | confidence=" << confidence
                          << " (W_TTC=" << W_TTC << ", W_DIST=" << W_DIST << ")";

        riskAssessment.riskAssessmentList.push_back(r);
    }

    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 3 DONE =============";
}

//===== 시나리오 #4. 정지중(ego=0 m/s) 주변 동적 객체 접근 위험 =====
void evaluateScenario4(const obstacleListVector& obstacle_list,
                       const adcm::vehicleListStruct& ego_vehicle,
                       adcm::risk_assessment_Objects& riskAssessment)
{
    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 4 START =============";

    // 좌표 단위: dm(0.1 m) 가정, 속도 단위: m/s
    constexpr double MIN_DIST_DM = 200.0;   // 20 m
    constexpr double MAX_DIST_DM = 400.0;   // 40 m
    constexpr double STOP_SPEED_TH_MPS = 0.2; // 자차 '정지' 판단 임계
    constexpr double TTC_TH_S = 10.0;       // TTC 정규화 기준(튜닝)

    // 컨피던스 가중치 (합=1.0로 유지 권장)
    constexpr double W_TTC  = 0.6;
    constexpr double W_MIND = 0.4;

    // ① 자차 정지 조건
    const double ego_speed = std::hypot(ego_vehicle.velocity_x, ego_vehicle.velocity_y);
    if (ego_speed > STOP_SPEED_TH_MPS) {
        SCENARIO_LOG_INFO() << "[시나리오4] 자차 정지 상태 아님(" << ego_speed << " m/s) → 종료";
        SCENARIO_LOG_INFO() << "============= KATECH: Scenario 4 DONE =============";
        return;
    }

    obstacleListVector candidates;

    for (const auto& obs : obstacle_list) {
        // ② 동적 객체(프로젝트 규약에 맞춰 클래스/정지여부 정의)
        const bool dynamic_class = (obs.obstacle_class >= 1 && obs.obstacle_class <= 29);
        const bool moving        = (obs.stop_count < STOP_VALUE);
        if (!dynamic_class || !moving) continue;

        // Ego-장애물 직선거리 (dm)
        const double d_ego_dm = calculateDistance(obs, ego_vehicle);
        if (d_ego_dm < MIN_DIST_DM || d_ego_dm > MAX_DIST_DM) continue;
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
        double ttc = -1.0, d_min_dm = 0.0;
        (void)calculateMinDistanceLinear(obs, ego_vehicle, d_min_dm);
        const double d_ego_dm = calculateDistance(obs, ego_vehicle);
        const bool ttc_ok = getTTC(obs, ego_vehicle, ttc); // s 단위

        SCENARIO_LOG_INFO() << "   - ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | Ego거리=" << (d_ego_dm/10.0) << " m"
                          << " | linMinDist=" << (d_min_dm/10.0) << " m"
                          << " | TTC=" << (ttc_ok ? ttc : -1.0) << " s";
    }

    // === 컨피던스 계산 & 등록 ===
    for (const auto& obs : candidates) {
        double ttc = -1.0, d_min_dm = 0.0;
        (void)calculateMinDistanceLinear(obs, ego_vehicle, d_min_dm);
        (void)getTTC(obs, ego_vehicle, ttc); // 실패 시 ttc는 음수/미사용

        // 정규화 (0~1)
        // - 선형 최소거리: 0 m → 1, 40 m → 0
        double s_mind = 1.0 - (d_min_dm / MAX_DIST_DM);
        s_mind = clampValue(s_mind, 0.0, 1.0);

        // - TTC: 0 s → 1, 10 s → 0 (클램프)
        double s_ttc = (ttc >= 0.0) ? (TTC_TH_S - ttc) / TTC_TH_S : 0.0;
        s_ttc = clampValue(s_ttc, 0.0, 1.0);

        // 가중합 + clamp
        double confidence = W_TTC * s_ttc + W_MIND * s_mind;
        confidence = clampValue(confidence, 0.0, 1.0);

        adcm::riskAssessmentStruct r{};
        r.obstacle_id  = obs.obstacle_id;
        r.hazard_class = SCENARIO_4;
        r.confidence   = confidence;

        SCENARIO_LOG_INFO() << "[시나리오4 TRIGGER] ID=" << obs.obstacle_id
                          << " | s_ttc=" << s_ttc
                          << " | s_minDist=" << s_mind
                          << " | confidence=" << confidence
                          << " (W_TTC=" << W_TTC << ", W_MIN=" << W_MIND << ")";

        riskAssessment.riskAssessmentList.push_back(r);
    }

    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 4 DONE =============";
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
namespace {
    using namespace adcm;

    // <obstacle_id, first_seen_ts_ms>
    static std::vector<std::pair<std::uint64_t, double>> s5_first_seen_vec;

    // 활성 앵커 윈도우: <anchor_id, window_start_ms>
    static std::vector<std::pair<std::uint64_t, double>> s5_anchor_windows;

    inline void s5_reset_state() {
        s5_first_seen_vec.clear();
        s5_anchor_windows.clear();
        SCENARIO_LOG_INFO() << "[시나리오5] 상태 리셋 (first_seen, 앵커 윈도우 초기화)";
    }

    inline double s5_getFirstSeen(std::uint64_t id) {
        for (size_t i = 0; i < s5_first_seen_vec.size(); ++i)
            if (s5_first_seen_vec[i].first == id) return s5_first_seen_vec[i].second;
        return -1.0;
    }
    inline void s5_recordFirstSeen(std::uint64_t id, double ts) {
        for (size_t i = 0; i < s5_first_seen_vec.size(); ++i)
            if (s5_first_seen_vec[i].first == id) {
                if (ts < s5_first_seen_vec[i].second) s5_first_seen_vec[i].second = ts;
                return;
            }
        s5_first_seen_vec.push_back(std::make_pair(id, ts));
    }
    inline double s5_getAnchorStart(std::uint64_t id) {
        for (size_t i = 0; i < s5_anchor_windows.size(); ++i)
            if (s5_anchor_windows[i].first == id) return s5_anchor_windows[i].second;
        return -1.0;
    }
    inline void s5_addAnchor(std::uint64_t id, double start_ts) {
        if (s5_getAnchorStart(id) < 0.0) {
            s5_anchor_windows.push_back(std::make_pair(id, start_ts));
            SCENARIO_LOG_INFO() << "  · [앵커 생성] ID=" << id << " | start=" << start_ts << " ms";
        }
    }
    inline void s5_pruneAnchors(double now, double window_ms, double grace_ms) {
        size_t kept = 0;
        for (size_t i = 0; i < s5_anchor_windows.size(); ++i) {
            double start = s5_anchor_windows[i].second;
            if (now - start <= window_ms + grace_ms) {
                if (kept != i) s5_anchor_windows[kept] = s5_anchor_windows[i];
                ++kept;
            } else {
                SCENARIO_LOG_INFO() << "  · [앵커 만료] ID=" << s5_anchor_windows[i].first
                            << " | start=" << start << " ms | now=" << now << " ms";
            }
        }
        s5_anchor_windows.resize(kept);
    }
    inline const adcm::obstacleListStruct*
    s5_findInCand(const obstacleListVector& cand, std::uint64_t id) {
        for (size_t i = 0; i < cand.size(); ++i)
            if (cand[i].obstacle_id == id) return &cand[i];
        return nullptr;
    }
}
void evaluateScenario5(const obstacleListVector& obstacle_list,
                       const adcm::vehicleListStruct& ego_vehicle,
                       const std::vector<double>& path_x,
                       const std::vector<double>& path_y,
                       adcm::risk_assessment_Objects& riskAssessment)
{
    using namespace adcm;
    SCENARIO_LOG_INFO() << "==================== [시나리오5 시작] ====================";

    // 상수 (dm/ms)
    constexpr double EGO_MIN_DM          = 300.0;    // 40 m
    constexpr double EGO_MAX_DM          = 700.0;    // 50 m
    constexpr double DIST_TO_PATH_MAX_DM = 100.0;    // 10 m
    constexpr double MAX_PAIR_DIST_DM    = 200.0;    // 20 m
    constexpr double WINDOW_MS           = 10000.0;  // 10 s
    constexpr double FRAME_GRACE_MS      = 120.0;    // 동시 프레임 관용치
    constexpr double EPS                 = 1e-6;

   
    SCENARIO_LOG_INFO() << "==================== [시나리오5 시작] ====================";

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
            //SCENARIO_LOG_INFO() << "  └─[제외] ID=" << obs.obstacle_id << " | 보행자 아님";
            continue;
        }
        double d_ego_dm = calculateDistance(obs, ego_vehicle);
        if (d_ego_dm < EGO_MIN_DM || d_ego_dm > EGO_MAX_DM) {
            //SCENARIO_LOG_INFO() << "  └─[제외] ID=" << obs.obstacle_id
            //            << " | Ego거리=" << (d_ego_dm/10.0) << " m (요구: 40~50 m)";
            continue;
        }
        double path_dist_dm = 0.0;
        if (!calculateMinDistanceToPath(obs, path_x, path_y, path_dist_dm)) {
            //SCENARIO_LOG_INFO() << "  └─[제외] ID=" << obs.obstacle_id << " | 경로거리 계산 실패";
            continue;
        }
        if (path_dist_dm > DIST_TO_PATH_MAX_DM) {
            //SCENARIO_LOG_INFO() << "  └─[제외] ID=" << obs.obstacle_id
            //            << " | 경로거리=" << (path_dist_dm/10.0) << " m (요구: ≤10 m)";
            continue;
        }

        cand.push_back(obs);
        SCENARIO_LOG_INFO() << "  └─[후보] ID=" << obs.obstacle_id
                    << " | ts=" << obs.timestamp << " ms"
                    << " | Ego거리=" << (d_ego_dm/10.0) << " m"
                    << " | 경로거리=" << (path_dist_dm/10.0) << " m";

        // 처음 조건을 만족한 보행자면 first_seen 기록하고, 자신의 앵커 윈도우 시작
        double prev_fs = s5_getFirstSeen(obs.obstacle_id);
        s5_recordFirstSeen(obs.obstacle_id, obs.timestamp);
        if (prev_fs < 0.0) {
            s5_addAnchor(obs.obstacle_id, obs.timestamp); // 새 앵커 생성
        }
    }

    if (cand.empty()) {
        SCENARIO_LOG_INFO() << "[시나리오5] 유효 후보 없음 → 종료";
        SCENARIO_LOG_INFO() << "==================== [시나리오5 종료] ====================";
        return;
    }

    // (2) 만료된 앵커 제거
    s5_pruneAnchors(frame_ts_max, WINDOW_MS, FRAME_GRACE_MS);
    SCENARIO_LOG_INFO() << "[시나리오5] 활성 앵커 수=" << s5_anchor_windows.size();

    if (s5_anchor_windows.empty()) {
        SCENARIO_LOG_INFO() << "[시나리오5] 활성 앵커 없음 → 종료";
        SCENARIO_LOG_INFO() << "==================== [시나리오5 종료] ====================";
        return;
    }

    // (3) 페어 탐지: 앵커 × 현재 후보
    bool triggered = false;
    double best_pair_dm = std::numeric_limits<double>::max();
    adcm::obstacleListStruct best_a{}, best_b{};

    for (size_t ai = 0; ai < s5_anchor_windows.size(); ++ai) {
        const std::uint64_t anchor_id = s5_anchor_windows[ai].first;
        const double        t0        = s5_anchor_windows[ai].second;

        // 현재 프레임에 앵커 객체가 실제로 존재해야 거리 계산 가능
        const auto* anchor_ptr = s5_findInCand(cand, anchor_id);
        if (!anchor_ptr) {
            SCENARIO_LOG_INFO() << "  └─[스킵] 앵커 ID=" << anchor_id << " 가 현재 후보에 없음";
            continue;
        }

        for (size_t ci = 0; ci < cand.size(); ++ci) {
            const auto& obj = cand[ci];
            if (obj.obstacle_id == anchor_id) continue;

            const double fs = s5_getFirstSeen(obj.obstacle_id);
            if (fs < 0.0) continue; // 방어적

            const bool both_start_frame =
                (fs >= t0 - EPS) && (fs <= t0 + FRAME_GRACE_MS + EPS);

            const bool newcomer_in_window =
                (fs >  t0 + FRAME_GRACE_MS + EPS) && (fs <= t0 + WINDOW_MS + EPS);

            if (!(both_start_frame || newcomer_in_window)) {
                 SCENARIO_LOG_INFO() << "[무시] 앵커(ID=" << anchor_id
                << ", start=" << t0 << "ms)"
                << " vs 후보(ID=" << obj.obstacle_id
                << ", first_seen=" << fs << "ms)"
                << " -> 시간조건 불일치"
                << " (동시등장=" << (both_start_frame ? "예" : "아니오")
                << ", 10초내신규=" << (newcomer_in_window ? "예" : "아니오") << ")";
                continue;
            }

            double dij_dm = calculateDistance(*anchor_ptr, obj);
            SCENARIO_LOG_INFO() << "    · [검사] 페어(" << anchor_id << "," << obj.obstacle_id
                        << ") | 유형=" << (both_start_frame? "동시등장":"앵커-뉴커머")
                        << " | 거리=" << (dij_dm/10.0) << " m";

            if (dij_dm <= MAX_PAIR_DIST_DM) {
                //SCENARIO_LOG_INFO() << "      -> [유효쌍] 거리 ≤ 20 m";
                if (dij_dm < best_pair_dm) {
                    best_pair_dm = dij_dm;
                    best_a = *anchor_ptr;
                    best_b = obj;
                    triggered = true;
                }
            }
        }
    }

    if (!triggered) {
        SCENARIO_LOG_INFO() << "[시나리오5] 현재 프레임: 유효(≤20m) 페어 없음";
        SCENARIO_LOG_INFO() << "==================== [시나리오5 종료] ====================";
        return;
    }

    // (4) 트리거 처리
    const double pair_m   = best_pair_dm / 10.0;
    const double conf     = clampValue((MAX_PAIR_DIST_DM - best_pair_dm) / MAX_PAIR_DIST_DM, 0.0, 1.0);

    adcm::riskAssessmentStruct s5a{ .obstacle_id=best_a.obstacle_id, .hazard_class=SCENARIO_5, .confidence=conf };
    adcm::riskAssessmentStruct s5b{ .obstacle_id=best_b.obstacle_id, .hazard_class=SCENARIO_5, .confidence=conf };

    SCENARIO_LOG_INFO() << "[시나리오5 TRIGGER]"
                << " | 페어 IDs=(" << s5a.obstacle_id << "," << s5b.obstacle_id << ")"
                << " | 거리=" << pair_m << " m"
                << " | confidence=" << conf;

    riskAssessment.riskAssessmentList.push_back(s5a);
    riskAssessment.riskAssessmentList.push_back(s5b);

    // 정책 선택:
    //  (A) 한 번 트리거하면 전체 리셋 → 깔끔 (아래 사용)
    //  (B) 트리거 후에도 남은 앵커 유지 → 다중 트리거 허용 (원하시면 이 라인 제거)
    s5_reset_state();

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
namespace {
    using namespace adcm;
    static std::vector<std::pair<std::uint64_t, double>> s6_first_seen_vec;
    static std::vector<std::pair<std::uint64_t, double>> s6_anchor_windows;

    inline void s6_reset_state() {
        s6_first_seen_vec.clear();
        s6_anchor_windows.clear();
        SCENARIO_LOG_INFO() << "[시나리오6] 상태 리셋";
    }

    inline double s6_getFirstSeen(std::uint64_t id) {
        for (auto &p : s6_first_seen_vec)
            if (p.first == id) return p.second;
        return -1.0;
    }
    inline void s6_recordFirstSeen(std::uint64_t id, double ts) {
        for (auto &p : s6_first_seen_vec)
            if (p.first == id) { p.second = std::min(p.second, ts); return; }
        s6_first_seen_vec.emplace_back(id, ts);
    }
    inline double s6_getAnchorStart(std::uint64_t id) {
        for (auto &p : s6_anchor_windows)
            if (p.first == id) return p.second;
        return -1.0;
    }
    inline void s6_addAnchor(std::uint64_t id, double start_ts) {
        if (s6_getAnchorStart(id) < 0.0) {
            s6_anchor_windows.emplace_back(id, start_ts);
            SCENARIO_LOG_INFO() << "  · [앵커 생성] 차량ID=" << id << " | start=" << start_ts;
        }
    }
    inline void s6_pruneAnchors(double now, double window_ms) {
        size_t kept=0;
        for (size_t i=0;i<s6_anchor_windows.size();++i) {
            if (now - s6_anchor_windows[i].second <= window_ms) {
                if (i!=kept) s6_anchor_windows[kept]=s6_anchor_windows[i];
                kept++;
            } else {
                SCENARIO_LOG_INFO() << "  · [만료] 앵커 차량ID="<< s6_anchor_windows[i].first;
            }
        }
        s6_anchor_windows.resize(kept);
    }
    inline const adcm::obstacleListStruct* s6_findInCand(const obstacleListVector &cand, std::uint64_t id) {
        for (auto &o : cand) if (o.obstacle_id==id) return &o;
        return nullptr;
    }
}
void evaluateScenario6(const obstacleListVector& obstacle_list,
                       const adcm::vehicleListStruct& ego_vehicle,
                       const std::vector<double>& path_x,
                       const std::vector<double>& path_y,
                       adcm::risk_assessment_Objects& riskAssessment)
{
    using namespace adcm;
    SCENARIO_LOG_INFO() << "==================== [시나리오6 시작] ====================";

    constexpr double EGO_MIN_DM          = 400.0;   // 50 m
    constexpr double EGO_MAX_DM          = 600.0;   // 60 m
    constexpr double DIST_TO_PATH_MAX_DM = 150.0;   // 15 m
    constexpr double MAX_PAIR_DIST_DM    = 300.0;   // 30 m
    constexpr double WINDOW_MS           = 10000.0; // 10초
    constexpr double FRAME_GRACE_MS      = 120.0;
    constexpr double EPS                 = 1e-6;

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

        // 처음 조건을 만족했다면 first_seen 기록 & 앵커 시작
        double prev = s6_getFirstSeen(obs.obstacle_id);
        s6_recordFirstSeen(obs.obstacle_id, obs.timestamp);
        if (prev<0.0) s6_addAnchor(obs.obstacle_id, obs.timestamp);
    }

    if (cand.empty()) {
        SCENARIO_LOG_INFO() << "[시나리오6] 후보차량 없음 → 종료";
        return;
    }

    // (2) 앵커 윈도우 유지
    s6_pruneAnchors(frame_ts_max, WINDOW_MS);
    SCENARIO_LOG_INFO() << "[시나리오6] 활성 앵커 수=" << s6_anchor_windows.size();
    if (s6_anchor_windows.empty()) {
        SCENARIO_LOG_INFO() << "[시나리오6] 활성 앵커 없음";
        return;
    }

    // (3) 페어 탐지: 앵커 × 현재 후보
    bool triggered=false;
    double best_pair_dm=std::numeric_limits<double>::max();
    adcm::obstacleListStruct best_a{}, best_b{};
    
    SCENARIO_LOG_INFO() << "[시나리오6] 페어 탐지 시작: window=10s, 허용거리 ≤ 30 m";

    for (auto &anc : s6_anchor_windows) {
        auto anchor_id=anc.first; double t0=anc.second;
        auto* anchor_ptr=s6_findInCand(cand,anchor_id);
        if (!anchor_ptr) {
            SCENARIO_LOG_INFO() << "  └─[스킵] 앵커 차량 ID="<<anchor_id<<" 현재 프레임 후보에 없음";
            continue;
        }
        for (auto &obj:cand) {
            if (obj.obstacle_id==anchor_id) continue;
            double fs=s6_getFirstSeen(obj.obstacle_id);
            if (fs<0.0) continue;

            bool both_start=(fs>=t0-EPS && fs<=t0+FRAME_GRACE_MS+EPS);
            bool newcomer=(fs>t0+FRAME_GRACE_MS+EPS && fs<=t0+WINDOW_MS+EPS);

            if (!(both_start||newcomer)) {
                SCENARIO_LOG_INFO() << "    · [무시] 앵커(ID="<<anchor_id<<", start="<<t0<<"ms)"
                            << " vs 후보(ID="<<obj.obstacle_id<<", first_seen="<<fs<<"ms)"
                            << " -> 시간조건 불일치"
                            << " (동시등장="<<(both_start?"예":"아니오")
                            << ", 10초내신규="<<(newcomer?"예":"아니오")<<")";
                continue;
            }
            double dij=calculateDistance(*anchor_ptr,obj);
            SCENARIO_LOG_INFO() << "    · [검사] 차량쌍(" << anchor_id << "," << obj.obstacle_id << ")"
            << " | 유형=" << (both_start ? "동시등장" : "앵커-뉴커머")
            << " | 페어거리=" << (dij/10.0) << " m";
            if (dij<=MAX_PAIR_DIST_DM && dij<best_pair_dm){
                best_pair_dm=dij;
                best_a=*anchor_ptr; best_b=obj;
                triggered=true;
            }
        }
    }

    if (!triggered) {
        SCENARIO_LOG_INFO() << "[시나리오6] 유효 차량쌍 없음";
        return;
    }

    // (4) 트리거 처리
    // - 결과는 'Ego와 더 가까운 차량 1대'만 추가
    const double d_ego_a_dm = calculateDistance(best_a, ego_vehicle);
    const double d_ego_b_dm = calculateDistance(best_b, ego_vehicle);
    const bool   a_is_closer = (d_ego_a_dm <= d_ego_b_dm);

    const adcm::obstacleListStruct& chosen = a_is_closer ? best_a : best_b;
    const double min_ego_dm = a_is_closer ? d_ego_a_dm : d_ego_b_dm;
    const double min_ego_m  = min_ego_dm / 10.0;
    const double pair_m     = best_pair_dm / 10.0;

    // confidence = 400dm / (min_ego_dm) * 0.7  (단위 일치)
    double confidence = (400.0 / std::max(min_ego_dm, 1.0)) * 0.7; // 0 보호
    confidence = clampValue(confidence, 0.0, 1.0);

    adcm::riskAssessmentStruct s6{};
    s6.obstacle_id  = chosen.obstacle_id;
    s6.hazard_class = SCENARIO_6;
    s6.confidence   = confidence;

    SCENARIO_LOG_INFO() << "[시나리오6 TRIGGER]"
                << " | 페어 IDs=(" << best_a.obstacle_id << "," << best_b.obstacle_id << ")"
                << " | 페어거리=" << pair_m << " m"
                << " | Ego-가까운차 ID=" << chosen.obstacle_id
                << " | Ego-가까운차 거리=" << min_ego_m << " m"
                << " | confidence=" << confidence
                << " (계산식: 400dm/" << min_ego_dm << " *0.7)";

    riskAssessment.riskAssessmentList.push_back(s6);

    // 정책: 한 번 트리거 후 상태 초기화 (원하면 유지 정책으로 변경 가능)
    s6_reset_state();
    SCENARIO_LOG_INFO()<<"==================== [시나리오6 종료] ====================";
}

void evaluateScenario7(const std::vector<double>& path_x,
                       const std::vector<double>& path_y,
                       const std::vector<adcm::map_2dListVector>& map_2d,
                       adcm::risk_assessment_Objects& riskAssessment)
{
    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 7 START =============";

    auto isUnscanned = [&](uint8_t road_z) 
    {
        return !(road_z <= 10);  // 0~10 범위가 아니면 전부 unscanned 이므로 true 를 return
    };

    for (size_t count = 0; count + 1 < path_x.size(); count++)
    {
        int x_start = static_cast<int>(path_x[count]);
        int x_end   = static_cast<int>(path_x[count + 1]);
        int y_start = static_cast<int>(path_y[count]);
        int y_end   = static_cast<int>(path_y[count + 1]);

        // Bresenham's line algorithm
        bool steep = (std::abs(y_end - y_start) > std::abs(x_end - x_start));
        if (steep) { std::swap(x_start, y_start); std::swap(x_end, y_end); }
        if (x_start > x_end) { std::swap(x_start, x_end); std::swap(y_start, y_end); }

        int dx = x_end - x_start;
        int dy = std::abs(y_end - y_start);
        int error = dx / 2;
        int ystep = (y_start < y_end) ? 1 : -1;
        int y = y_start;

        for (int x = x_start; x <= x_end; ++x)
        {
            int gridX = steep ? y : x;
            int gridY = steep ? x : y;

            if (gridX < 0 || gridX >= map_2d.size() ||
                gridY < 0 || gridY >= map_2d[0].size())
                continue;

            if (isUnscanned(map_2d[gridX][gridY].road_z))
            {
                adcm::riskAssessmentStruct r{};
                r.hazard_class = SCENARIO_7;
                r.isHarzard = true;

                adcm::globalPathPosition start{path_x[count], path_y[count]};
                adcm::globalPathPosition end{path_x[count+1], path_y[count+1]};
                r.wgs84_xy_start.push_back(start);
                r.wgs84_xy_end.push_back(end);

                SCENARIO_LOG_INFO() << "[시나리오7] Unscanned path detected: "
                                  << "X=" << start.x << " Y=" << start.y
                                  << " road_z=" << static_cast<int>(map_2d[gridX][gridY].road_z);

                riskAssessment.riskAssessmentList.push_back(r);
                break; // 한 구간당 한 번만 리스크 등록
            }

            error -= dy;
            if (error < 0) { y += ystep; error += dx; }
        }
    }

    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 7 DONE =============";
}

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
            out.wgs84_xy_start.push_back(adcm::globalPathPosition{ path_x[i],     path_y[i]     });
            out.wgs84_xy_end.push_back  (adcm::globalPathPosition{ path_x[i + 1], path_y[i + 1] });
            out.hazard_class = SCENARIO_8;
            out.isHarzard = true;

            SCENARIO_LOG_INFO() << "Risk assessment generated for #8 at X: " 
                                << path_x[i] << ", Y: " << path_y[i];

            riskAssessment.riskAssessmentList.push_back(std::move(out));
            //break;
        }
    }

    SCENARIO_LOG_INFO() << "=============KATECH: scenario 8 DONE==============";

}

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
            (obs.obstacle_class >= 1 && obs.obstacle_class <= 19 && obs.stop_count >= STOP_VALUE);
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
    
        // 최종 컨피던스 (가중합)
        double confidence = clampValue(W_GOAL * s_goal +
                                       W_EGO  * s_ego  +
                                       W_HEIGHT * s_height, 0.0, 1.0);

        adcm::riskAssessmentStruct r{};
        r.obstacle_id  = obs.obstacle_id;
        r.hazard_class = SCENARIO_9;
        r.confidence   = confidence;

        SCENARIO_LOG_INFO() << "[시나리오9 TRIGGER] ID=" << obs.obstacle_id
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

        // 최종 confidence 계산
        double confidence = clampValue(W_GOAL * s_goal +
                                       W_EGO  * s_ego  +
                                       W_STOP * s_stop, 0.0, 1.0);

        adcm::riskAssessmentStruct r{};
        r.obstacle_id  = obs.obstacle_id;
        r.hazard_class = SCENARIO_10;
        r.confidence   = confidence;

        SCENARIO_LOG_INFO() << "[시나리오10 TRIGGER] ID=" << obs.obstacle_id
                          << " | s_goal=" << s_goal
                          << ", s_ego=" << s_ego
                          << ", s_stop=" << s_stop
                          << " | confidence=" << confidence;

        riskAssessment.riskAssessmentList.push_back(r);
    }

    SCENARIO_LOG_INFO() << "============= KATECH: Scenario 10 DONE =============";
}
