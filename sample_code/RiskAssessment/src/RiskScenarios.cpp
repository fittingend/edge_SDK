#include "main_riskassessment.hpp"

//=====시나리오 #1. 주행중 전역경로 근방 이동가능한 정지 장애물이 존재하는 위험 환경=====
void evaluateScenario1(const obstacleListVector& obstacle_list, 
                        const adcm::vehicleListStruct& ego_vehicle, 
                        const std::vector<double>& path_x,
                        const std::vector<double>& path_y,
                        adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "============= KATECH: Scenario 1 START =============";

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
        if (obs.stop_count < STOP_VALUE) continue;

        adcm::Log::Info() << "[1-i] 차량 & 정지 상태: ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | stop_count=" << obs.stop_count;

        // (ii) Ego 거리
        const double d_ego_dm = calculateDistance(obs, ego_vehicle);
        if (d_ego_dm > DIST_TO_EGO_MAX_DM) continue;
        adcm::Log::Info() << "[1-ii] ego 거리 통과: ID=" << obs.obstacle_id
                    << " | Ego거리=" << (d_ego_dm/10.0) << " m";

        // (iii) 경로 거리
        double d_path_dm = 0.0;
        if (!calculateMinDistanceToPath(obs, path_x, path_y, d_path_dm)) continue;
        if (d_path_dm > DIST_TO_PATH_MAX_DM) 
        {
            adcm::Log::Info() << "[1-iii 제외] ID=" << obs.obstacle_id
                              << " | 경로거리=" << (d_path_dm/10.0) << " m (>10)";
            continue;
        }
        // 세 조건 모두 통과한 경우 후보군에 추가
        candidates.push_back(obs);
    }

    // === 후보군 로그 출력 ===
    if (candidates.empty()) {
        adcm::Log::Info() << "[시나리오1] 최종 후보 없음 → 종료";
        adcm::Log::Info() << "============= KATECH: Scenario 1 DONE =============";
        return;
    }

    adcm::Log::Info() << "[시나리오1] 최종 후보군 " << candidates.size() << "개:";
    for (const auto& obs : candidates) {
        double d_ego_dm  = calculateDistance(obs, ego_vehicle);
        double d_path_dm = 0.0;
        calculateMinDistanceToPath(obs, path_x, path_y, d_path_dm);

        adcm::Log::Info() << "   - ID=" << obs.obstacle_id
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

        adcm::Log::Info() << "[시나리오1 TRIGGER] ID=" << obs.obstacle_id
                          << " | s_ego=" << s_ego << ", s_path=" << s_path
                          << " | confidence=" << confidence;

        riskAssessment.riskAssessmentList.push_back(r);
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
    adcm::Log::Info() << "============= KATECH: Scenario 2 START =============";

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
        // (i) 조건: 차량이면서 정지 상태 OR 높이 1m 이상 정적 장애물
        bool cond_vehicle_stopped = (obs.obstacle_class >= 1 && obs.obstacle_class <= 19 && obs.stop_count >= STOP_VALUE);
        bool cond_static_high     = (obs.obstacle_class >= 30 && obs.obstacle_class <= 49 && obs.fused_cuboid_z > 1.0);

        if (!(cond_vehicle_stopped || cond_static_high)) continue;

        adcm::Log::Info() << "[2-i] 통과: ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | stop_count=" << obs.stop_count
                          << " | height=" << obs.fused_cuboid_z;

        // ----------------------------------------------------
        // (ii) Ego와의 거리 ≤ 40m
        const double d_ego_dm = calculateDistance(obs, ego_vehicle);
        if (d_ego_dm > DIST_TO_EGO_MAX_DM) {
            // adcm::Log::Info() << "[2-ii 제외] ID=" << obs.obstacle_id
            //                   << " | Ego거리=" << (d_ego_dm/10.0) << " m (>40)";
            continue;
        }
        adcm::Log::Info() << "[2-ii] 통과: ID=" << obs.obstacle_id
                          << " | Ego거리=" << (d_ego_dm/10.0) << " m";

        // ----------------------------------------------------
        // (iii) 전역경로와의 거리 ≤ 10m
        double d_path_dm = 0.0;
        if (!calculateMinDistanceToPath(obs, path_x, path_y, d_path_dm)) {
            // adcm::Log::Info() << "[2-iii 제외] ID=" << obs.obstacle_id
            //                   << " | 경로거리 계산 실패";
            continue;
        }
        if (d_path_dm > DIST_TO_PATH_MAX_DM) {
            adcm::Log::Info() << "[2-iii 제외] ID=" << obs.obstacle_id
                              << " | 경로거리=" << (d_path_dm/10.0) << " m (>10)";
            continue;
        }
        // adcm::Log::Info() << "[2-iii] 통과: ID=" << obs.obstacle_id
        //                   << " | 경로거리=" << (d_path_dm/10.0) << " m";

        // 세 조건 모두 만족 → 후보군 추가
        candidates.push_back(obs);
    }

    // === 최종 후보군 로그 ===
    if (candidates.empty()) {
        adcm::Log::Info() << "[시나리오2] 최종 후보 없음 → 종료";
        adcm::Log::Info() << "============= KATECH: Scenario 2 DONE =============";
        return;
    }

    adcm::Log::Info() << "[시나리오2] 최종 후보군 " << candidates.size() << "개";

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

        adcm::Log::Info() << "[시나리오2 TRIGGER] ID=" << obs.obstacle_id
                          << " | s_ego=" << s_ego << ", s_path=" << s_path
                          << " | confidence=" << confidence;

        riskAssessment.riskAssessmentList.push_back(r);
    }

    adcm::Log::Info() << "============= KATECH: Scenario 2 DONE =============";
}//===== 시나리오 #3. 주행중 경로 주변 동적 장애물 통행 환경 =====
void evaluateScenario3(const obstacleListVector& obstacle_list, 
                       const adcm::vehicleListStruct& ego_vehicle, 
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "============= KATECH: Scenario 3 START =============";

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

        adcm::Log::Info() << "[3-i] 통과: ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | stop_count=" << obs.stop_count
                          << " | Ego거리=" << (d_ego_dm/10.0) << " m";

        // (ii) TTC & 선형 최소거리 평가 → 통과한 것만 candidates에 편입
        double ttc = 0.0, d_lin_dm = 0.0;
        const bool ttc_ok  = (getTTC(obs, ego_vehicle, ttc) && ttc < TTC_TH_S);
        const bool dist_ok = (calculateMinDistanceLinear(obs, ego_vehicle, d_lin_dm) && d_lin_dm < MAX_DIST_DM);

        if (!ttc_ok || !dist_ok) {
            adcm::Log::Info() << "[3-ii 제외] ID=" << obs.obstacle_id
                              << " | TTC=" << ttc << " s (th=" << TTC_TH_S << ")"
                              << " | linDist=" << (d_lin_dm/10.0) << " m (max=" << (MAX_DIST_DM/10.0) << ")";
            continue;
        }

        adcm::Log::Info() << "[3-ii] 통과: ID=" << obs.obstacle_id
                          << " | TTC=" << ttc << " s"
                          << " | linDist=" << (d_lin_dm/10.0) << " m";

        candidates.push_back(obs);
    }

    // === 최종 후보군 로그 (컨피던스 계산 전에 전부 출력) ===
    if (candidates.empty()) {
        adcm::Log::Info() << "[시나리오3] 최종 후보 없음 → 종료";
        adcm::Log::Info() << "============= KATECH: Scenario 3 DONE =============";
        return;
    }

    adcm::Log::Info() << "[시나리오3] 최종 후보군 " << candidates.size() << "개:";
    for (const auto& obs : candidates) {
        double ttc = 0.0, d_lin_dm = 0.0;
        double d_ego_dm = calculateDistance(obs, ego_vehicle);
        bool ttcr = getTTC(obs, ego_vehicle, ttc);
        bool linr = calculateMinDistanceLinear(obs, ego_vehicle, d_lin_dm);
        adcm::Log::Info() << "   - ID=" << obs.obstacle_id
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

        adcm::Log::Info() << "[시나리오3 TRIGGER] ID=" << obs.obstacle_id
                          << " | s_ttc=" << s_ttc
                          << " | s_dist=" << s_dist
                          << " | confidence=" << confidence
                          << " (W_TTC=" << W_TTC << ", W_DIST=" << W_DIST << ")";

        riskAssessment.riskAssessmentList.push_back(r);
    }

    adcm::Log::Info() << "============= KATECH: Scenario 3 DONE =============";
}

//===== 시나리오 #4. 정지중(ego=0 m/s) 주변 동적 객체 접근 위험 =====
void evaluateScenario4(const obstacleListVector& obstacle_list,
                       const adcm::vehicleListStruct& ego_vehicle,
                       adcm::risk_assessment_Objects& riskAssessment)
{
    adcm::Log::Info() << "============= KATECH: Scenario 4 START =============";

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
        adcm::Log::Info() << "[시나리오4] 자차 정지 상태 아님(" << ego_speed << " m/s) → 종료";
        adcm::Log::Info() << "============= KATECH: Scenario 4 DONE =============";
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
        adcm::Log::Info() << "[4-i] 통과: ID=" << obs.obstacle_id
                          << " | class=" << static_cast<int>(obs.obstacle_class)
                          << " | Ego거리=" << (d_ego_dm/10.0) << " m";

        // ③ 선형 최소거리 ≤ 40 m 조건 (dm 단위 반환 가정)
        double d_min_dm = 0.0;
        if (!calculateMinDistanceLinear(obs, ego_vehicle, d_min_dm) || d_min_dm > MAX_DIST_DM) {
            adcm::Log::Info() << "[4-ii 제외] ID=" << obs.obstacle_id
                              << " | linMinDist=" << (d_min_dm/10.0) << " m (>40 또는 계산실패)";
            continue;
        }
        adcm::Log::Info() << "[4-ii] 통과: ID=" << obs.obstacle_id
                          << " | linMinDist=" << (d_min_dm/10.0) << " m";

        // 세 조건 모두 통과 → 후보군 편입
        candidates.push_back(obs);
    }

    // === 최종 후보군 요약 출력 (컨피던스 계산 전) ===
    if (candidates.empty()) {
        adcm::Log::Info() << "[시나리오4] 최종 후보 없음 → 종료";
        adcm::Log::Info() << "============= KATECH: Scenario 4 DONE =============";
        return;
    }

    adcm::Log::Info() << "[시나리오4] 최종 후보군 " << candidates.size() << "개:";
    for (const auto& obs : candidates) {
        double ttc = -1.0, d_min_dm = 0.0;
        (void)calculateMinDistanceLinear(obs, ego_vehicle, d_min_dm);
        const double d_ego_dm = calculateDistance(obs, ego_vehicle);
        const bool ttc_ok = getTTC(obs, ego_vehicle, ttc); // s 단위

        adcm::Log::Info() << "   - ID=" << obs.obstacle_id
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

        adcm::Log::Info() << "[시나리오4 TRIGGER] ID=" << obs.obstacle_id
                          << " | s_ttc=" << s_ttc
                          << " | s_minDist=" << s_mind
                          << " | confidence=" << confidence
                          << " (W_TTC=" << W_TTC << ", W_MIN=" << W_MIND << ")";

        riskAssessment.riskAssessmentList.push_back(r);
    }

    adcm::Log::Info() << "============= KATECH: Scenario 4 DONE =============";
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
        Log::Info() << "[시나리오5] 상태 리셋 (first_seen, 앵커 윈도우 초기화)";
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
            Log::Info() << "  · [앵커 생성] ID=" << id << " | start=" << start_ts << " ms";
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
                Log::Info() << "  · [앵커 만료] ID=" << s5_anchor_windows[i].first
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
    Log::Info() << "==================== [시나리오5 시작] ====================";

    // 상수 (dm/ms)
    constexpr double EGO_MIN_DM          = 300.0;    // 40 m
    constexpr double EGO_MAX_DM          = 700.0;    // 50 m
    constexpr double DIST_TO_PATH_MAX_DM = 100.0;    // 10 m
    constexpr double MAX_PAIR_DIST_DM    = 200.0;    // 20 m
    constexpr double WINDOW_MS           = 10000.0;  // 10 s
    constexpr double FRAME_GRACE_MS      = 120.0;    // 동시 프레임 관용치
    constexpr double EPS                 = 1e-6;

   
    Log::Info() << "==================== [시나리오5 시작] ====================";

    if (obstacle_list.empty()) {
        Log::Info() << "[시나리오5] 입력 장애물 없음 → 종료";
        Log::Info() << "==================== [시나리오5 종료] ====================";
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
    Log::Info() << "[시나리오5] 프레임 수신: N=" << obstacle_list.size()
                << " | ts=[" << frame_ts_min << "," << frame_ts_max << "] ms";

    // (1) 후보 추출
    obstacleListVector cand; cand.reserve(obstacle_list.size());
    for (size_t k = 0; k < obstacle_list.size(); ++k) {
        const auto& obs = obstacle_list[k];

        if (static_cast<ObstacleClass>(obs.obstacle_class) != ObstacleClass::PEDESTRIAN) {
            //Log::Info() << "  └─[제외] ID=" << obs.obstacle_id << " | 보행자 아님";
            continue;
        }
        double d_ego_dm = calculateDistance(obs, ego_vehicle);
        if (d_ego_dm < EGO_MIN_DM || d_ego_dm > EGO_MAX_DM) {
            //Log::Info() << "  └─[제외] ID=" << obs.obstacle_id
            //            << " | Ego거리=" << (d_ego_dm/10.0) << " m (요구: 40~50 m)";
            continue;
        }
        double path_dist_dm = 0.0;
        if (!calculateMinDistanceToPath(obs, path_x, path_y, path_dist_dm)) {
            //Log::Info() << "  └─[제외] ID=" << obs.obstacle_id << " | 경로거리 계산 실패";
            continue;
        }
        if (path_dist_dm > DIST_TO_PATH_MAX_DM) {
            //Log::Info() << "  └─[제외] ID=" << obs.obstacle_id
            //            << " | 경로거리=" << (path_dist_dm/10.0) << " m (요구: ≤10 m)";
            continue;
        }

        cand.push_back(obs);
        Log::Info() << "  └─[후보] ID=" << obs.obstacle_id
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
        Log::Info() << "[시나리오5] 유효 후보 없음 → 종료";
        Log::Info() << "==================== [시나리오5 종료] ====================";
        return;
    }

    // (2) 만료된 앵커 제거
    s5_pruneAnchors(frame_ts_max, WINDOW_MS, FRAME_GRACE_MS);
    Log::Info() << "[시나리오5] 활성 앵커 수=" << s5_anchor_windows.size();

    if (s5_anchor_windows.empty()) {
        Log::Info() << "[시나리오5] 활성 앵커 없음 → 종료";
        Log::Info() << "==================== [시나리오5 종료] ====================";
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
            Log::Info() << "  └─[스킵] 앵커 ID=" << anchor_id << " 가 현재 후보에 없음";
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
                 Log::Info() << "[무시] 앵커(ID=" << anchor_id
                << ", start=" << t0 << "ms)"
                << " vs 후보(ID=" << obj.obstacle_id
                << ", first_seen=" << fs << "ms)"
                << " -> 시간조건 불일치"
                << " (동시등장=" << (both_start_frame ? "예" : "아니오")
                << ", 10초내신규=" << (newcomer_in_window ? "예" : "아니오") << ")";
                continue;
            }

            double dij_dm = calculateDistance(*anchor_ptr, obj);
            Log::Info() << "    · [검사] 페어(" << anchor_id << "," << obj.obstacle_id
                        << ") | 유형=" << (both_start_frame? "동시등장":"앵커-뉴커머")
                        << " | 거리=" << (dij_dm/10.0) << " m";

            if (dij_dm <= MAX_PAIR_DIST_DM) {
                //Log::Info() << "      -> [유효쌍] 거리 ≤ 20 m";
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
        Log::Info() << "[시나리오5] 현재 프레임: 유효(≤20m) 페어 없음";
        Log::Info() << "==================== [시나리오5 종료] ====================";
        return;
    }

    // (4) 트리거 처리
    const double pair_m   = best_pair_dm / 10.0;
    const double conf     = clampValue((MAX_PAIR_DIST_DM - best_pair_dm) / MAX_PAIR_DIST_DM, 0.0, 1.0);

    adcm::riskAssessmentStruct s5a{ .obstacle_id=best_a.obstacle_id, .hazard_class=SCENARIO_5, .confidence=conf };
    adcm::riskAssessmentStruct s5b{ .obstacle_id=best_b.obstacle_id, .hazard_class=SCENARIO_5, .confidence=conf };

    Log::Info() << "[시나리오5 TRIGGER]"
                << " | 페어 IDs=(" << s5a.obstacle_id << "," << s5b.obstacle_id << ")"
                << " | 거리=" << pair_m << " m"
                << " | confidence=" << conf;

    riskAssessment.riskAssessmentList.push_back(s5a);
    riskAssessment.riskAssessmentList.push_back(s5b);

    // 정책 선택:
    //  (A) 한 번 트리거하면 전체 리셋 → 깔끔 (아래 사용)
    //  (B) 트리거 후에도 남은 앵커 유지 → 다중 트리거 허용 (원하시면 이 라인 제거)
    s5_reset_state();

    Log::Info() << "==================== [시나리오5 종료] ====================";
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
        Log::Info() << "[시나리오6] 상태 리셋";
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
            Log::Info() << "  · [앵커 생성] 차량ID=" << id << " | start=" << start_ts;
        }
    }
    inline void s6_pruneAnchors(double now, double window_ms) {
        size_t kept=0;
        for (size_t i=0;i<s6_anchor_windows.size();++i) {
            if (now - s6_anchor_windows[i].second <= window_ms) {
                if (i!=kept) s6_anchor_windows[kept]=s6_anchor_windows[i];
                kept++;
            } else {
                Log::Info() << "  · [만료] 앵커 차량ID="<< s6_anchor_windows[i].first;
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
    Log::Info() << "==================== [시나리오6 시작] ====================";

    constexpr double EGO_MIN_DM          = 400.0;   // 50 m
    constexpr double EGO_MAX_DM          = 600.0;   // 60 m
    constexpr double DIST_TO_PATH_MAX_DM = 150.0;   // 15 m
    constexpr double MAX_PAIR_DIST_DM    = 300.0;   // 30 m
    constexpr double WINDOW_MS           = 10000.0; // 10초
    constexpr double FRAME_GRACE_MS      = 120.0;
    constexpr double EPS                 = 1e-6;

    if (obstacle_list.empty()) {
        Log::Info() << "[시나리오6] 입력 없음 → 종료";
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
    Log::Info() << "[시나리오6] 프레임 N=" << obstacle_list.size()
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
        Log::Info() << "  └─[후보차량] ID=" << obs.obstacle_id
                    << " ts=" << obs.timestamp
                    << " Ego거리=" << d_ego_dm/10.0 << "m"
                    << " 경로거리=" << path_dist_dm/10.0 << "m";

        // 처음 조건을 만족했다면 first_seen 기록 & 앵커 시작
        double prev = s6_getFirstSeen(obs.obstacle_id);
        s6_recordFirstSeen(obs.obstacle_id, obs.timestamp);
        if (prev<0.0) s6_addAnchor(obs.obstacle_id, obs.timestamp);
    }

    if (cand.empty()) {
        Log::Info() << "[시나리오6] 후보차량 없음 → 종료";
        return;
    }

    // (2) 앵커 윈도우 유지
    s6_pruneAnchors(frame_ts_max, WINDOW_MS);
    Log::Info() << "[시나리오6] 활성 앵커 수=" << s6_anchor_windows.size();
    if (s6_anchor_windows.empty()) {
        Log::Info() << "[시나리오6] 활성 앵커 없음";
        return;
    }

    // (3) 페어 탐지: 앵커 × 현재 후보
    bool triggered=false;
    double best_pair_dm=std::numeric_limits<double>::max();
    adcm::obstacleListStruct best_a{}, best_b{};
    
    Log::Info() << "[시나리오6] 페어 탐지 시작: window=10s, 허용거리 ≤ 30 m";

    for (auto &anc : s6_anchor_windows) {
        auto anchor_id=anc.first; double t0=anc.second;
        auto* anchor_ptr=s6_findInCand(cand,anchor_id);
        if (!anchor_ptr) {
            Log::Info() << "  └─[스킵] 앵커 차량 ID="<<anchor_id<<" 현재 프레임 후보에 없음";
            continue;
        }
        for (auto &obj:cand) {
            if (obj.obstacle_id==anchor_id) continue;
            double fs=s6_getFirstSeen(obj.obstacle_id);
            if (fs<0.0) continue;

            bool both_start=(fs>=t0-EPS && fs<=t0+FRAME_GRACE_MS+EPS);
            bool newcomer=(fs>t0+FRAME_GRACE_MS+EPS && fs<=t0+WINDOW_MS+EPS);

            if (!(both_start||newcomer)) {
                Log::Info() << "    · [무시] 앵커(ID="<<anchor_id<<", start="<<t0<<"ms)"
                            << " vs 후보(ID="<<obj.obstacle_id<<", first_seen="<<fs<<"ms)"
                            << " -> 시간조건 불일치"
                            << " (동시등장="<<(both_start?"예":"아니오")
                            << ", 10초내신규="<<(newcomer?"예":"아니오")<<")";
                continue;
            }
            double dij=calculateDistance(*anchor_ptr,obj);
            Log::Info() << "    · [검사] 차량쌍(" << anchor_id << "," << obj.obstacle_id << ")"
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
        Log::Info() << "[시나리오6] 유효 차량쌍 없음";
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

    Log::Info() << "[시나리오6 TRIGGER]"
                << " | 페어 IDs=(" << best_a.obstacle_id << "," << best_b.obstacle_id << ")"
                << " | 페어거리=" << pair_m << " m"
                << " | Ego-가까운차 ID=" << chosen.obstacle_id
                << " | Ego-가까운차 거리=" << min_ego_m << " m"
                << " | confidence=" << confidence
                << " (계산식: 400dm/" << min_ego_dm << " *0.7)";

    riskAssessment.riskAssessmentList.push_back(s6);

    // 정책: 한 번 트리거 후 상태 초기화 (원하면 유지 정책으로 변경 가능)
    s6_reset_state();
    Log::Info()<<"==================== [시나리오6 종료] ====================";
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

