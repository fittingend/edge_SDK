#include "main_riskassessment.hpp"

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

bool isRouteValid(routeVector& route)
{
    for (int count = 0; count < route.size(); count++)
    {
        if (!(33 < route[count].latitude && route[count].latitude < 39 &&
              124 < route[count].longitude && route[count].longitude < 132))
        {
            return false; // Return false immediately if any condition is not met
        }
    }
    return true; // Return true if all values meet the condition
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

const char* to_string(ObstacleClass cls) 
{
    switch (cls) {
        case ObstacleClass::NONE:          return "없음";

        // 건설기계류
        case ObstacleClass::EXCAVATOR:     return "굴착기";
        case ObstacleClass::FORKLIFT:      return "지게차";
        case ObstacleClass::CONCRETE_MIXER:return "콘크리트 믹서트럭";
        case ObstacleClass::CRANE:         return "기중기";
        case ObstacleClass::CONCRETE_PUMP: return "콘크리트 펌프";
        case ObstacleClass::DUMP_TRUCK:    return "덤프트럭";

        // 차량류
        case ObstacleClass::SEDAN:         return "승용차";
        case ObstacleClass::SUV:           return "SUV";
        case ObstacleClass::TRUCK:         return "트럭";
        case ObstacleClass::OTHER_VEH:     return "기타차량";
        case ObstacleClass::MOTORCYCLE:    return "오토바이";
        case ObstacleClass::BICYCLE:       return "자전거";

        // 사람
        case ObstacleClass::PEDESTRIAN:    return "일반인";
        case ObstacleClass::SAFETY_ROBOT:  return "안전유도로봇";

        // 건축자재
        case ObstacleClass::PIPE:          return "비계파이프";
        case ObstacleClass::CEMENT:        return "시멘트";
        case ObstacleClass::BRICK:         return "벽돌";

        // 안전용품
        case ObstacleClass::DRUM:          return "드럼통";
        case ObstacleClass::SHIELD:        return "가림막";
        case ObstacleClass::LARGE_CONE:    return "대형 라바콘";
        case ObstacleClass::SIGNBOARD:     return "입간판";

        default: return "알 수 없음";
    }
}

void printObstacleList(obstacleListVector obstacle_list)
{
    adcm::Log::Info() << "현재 장애물 리스트 프린트";
    if (obstacle_list.empty()) {
        adcm::Log::Info() << "장애물이 없습니다.";
        return;
    }

    for (const auto& obs : obstacle_list) 
    {
        adcm::Log::Info()
            << "ID=" << obs.obstacle_id
            << ", Class=" << obs.obstacle_class << " (" << to_string(static_cast<ObstacleClass>(obs.obstacle_class)) << ")"
            << ", Pos=(" << obs.fused_position_x << ", " << obs.fused_position_y << ")"
            << ", StopCount=" << obs.stop_count
            << ", Height=" << obs.fused_cuboid_z;
    }
}

void gpsToMapcoordinate(const routeVector& route, 
                        std::vector<double>& path_x, 
                        std::vector<double>& path_y)
{
    INFO("[RiskAssessment] gpsToMapcoordinate");
    // if (!type)
    // {   
    //     INFO("[RiskAssessment] simulation");
    //     // wps84기반 gps(global)좌표계를 작업환경 XY 기반의 Map 좌표계로 변환
    //     // 시뮬레이터 map 기준 원점(0,0) global좌표
    //     double mapOrigin_x = 453.088714;
    //     double mapOrigin_y = 507.550078;
    //     // 시뮬레이터 기준점 utm좌표
    //     double ref_x = 278296.968;
    //     double ref_y = 3980466.846;
    //     double angle_radians = -MAP_ANGLE * M_PI / 180.0;

    //     for (std::size_t i = 0; i < route.size(); ++i)
    //     {
    //         const auto& point = route[i];
    //         double utm_x, utm_y;
    //         Point2D mapPoint;

    //         GPStoUTM(point.longitude, point.latitude, utm_x, utm_y);
    //         utm_x -= ref_x;
    //         utm_y -= ref_y;
    //         mapPoint.x = (utm_x * cos(angle_radians) - utm_y * sin(angle_radians) - mapOrigin_x) * M_TO_10CM_PRECISION;
    //         mapPoint.y = (utm_x * sin(angle_radians) + utm_y * cos(angle_radians) - mapOrigin_y) * M_TO_10CM_PRECISION;

    //         checkRange(mapPoint);
    //         //checkRange 함수 수정 이후 반영 예정 
    //         path_x[i] = mapPoint.x;
    //         path_y[i] = mapPoint.y;
            
    //         adcm::Log::Info() << "gpsToMapcoordinate 변환: (" 
    //                         << point.longitude << ", " << point.latitude 
    //                         << ") → (" << mapPoint.x << ", " << mapPoint.y << ")";
    //     }
    // }
    
    // else // 실증
    // {
        INFO("[RiskAssessment] route 의 좌표변환");
        for (std::size_t i = 0; i < route.size(); ++i)
        {
            const auto& point = route[i];
            double utm_x, utm_y;
            Point2D mapPoint;

            GPStoUTM(point.longitude, point.latitude, utm_x, utm_y);
            mapPoint.x = (utm_x - origin_x) * M_TO_10CM_PRECISION;
            mapPoint.y = (utm_y - origin_y) * M_TO_10CM_PRECISION;
            checkRange(mapPoint);

            path_x[i] = mapPoint.x;
            path_y[i] = mapPoint.y;

            adcm::Log::Info() << "gpsToMapcoordinate 변환: (" 
                            << point.longitude << ", " << point.latitude 
                            << ") → (" << mapPoint.x << ", " << mapPoint.y << ")";
        }
    //}
}

double calculateDistance(const adcm::obstacleListStruct& obstacle1, 
                   const adcm::obstacleListStruct& obstacle2)
{
    double dx = obstacle1.fused_position_x - obstacle2.fused_position_x;
    double dy = obstacle1.fused_position_y - obstacle2.fused_position_y;

    return std::sqrt(dx * dx + dy * dy);
}

double calculateDistance(const adcm::obstacleListStruct& obstacle, 
                   const adcm::vehicleListStruct& vehicle)
{
    double dx = obstacle.fused_position_x - vehicle.position_x;
    double dy = obstacle.fused_position_y - vehicle.position_y;

    return std::sqrt(dx * dx + dy * dy);
}

double getMagnitude(Point2D point)
{
    return sqrt(point.x * point.x + point.y * point.y);
}

/**
 * @brief 장애물과 특장차(Ego Vehicle) 간의 TTC(Time To Collision)를 계산하는 함수
 *e 반환
 * 이 함수는 차량과 장애물의 상대 위치 및 상대 속도를 기반으로,
 * 선형 이동 가정 하에 두 객체가 충돌할 때까지 걸리는 예상 시간을 계산합니다.
 *
 * - 상대 속도가 거의 없을 경우(TTC 계산 불가) false 반환
 * - TTC가 0 이하일 경우(충돌 없음) false 반환
 * - 유효한 TTC가 계산되면 해당 값을 ttc에 저장하고 tru
 *
 * @param obstacle   장애물 정보 (위치 및 속도 포함)
 * @param vehicle    자차 정보 (위치 및 속도 포함)
 * @param ttc        계산된 TTC 값 (출력값, 단위: 초)
 * @return true      유효한 TTC가 계산된 경우 (양수)
 * @return false     상대 속도 부족 또는 충돌 예상 없음
 */
bool getTTC(const adcm::obstacleListStruct& obstacle, const adcm::vehicleListStruct& vehicle, double& ttc)
{
    constexpr double pos_unit_to_m = 0.1;
    constexpr double EPS_V = 1e-3; // m/s 임계(상대속도 매우 작음)

    // 상대 위치(미터로 스케일) & 상대 속도(m/s)
    const double rx = (obstacle.fused_position_x - vehicle.position_x) * pos_unit_to_m;
    const double ry = (obstacle.fused_position_y - vehicle.position_y) * pos_unit_to_m;
    const double vx =  obstacle.fused_velocity_x - vehicle.velocity_x; // m/s
    const double vy =  obstacle.fused_velocity_y - vehicle.velocity_y; // m/s

    const double v2 = vx*vx + vy*vy;
    if (v2 < EPS_V*EPS_V) {
        // 상대속도 ≈ 0 → TTC 의미 없음
        return false;
    }

    // 접근 여부 확인: r·v < 0 (접근), >=0 이면 멀어지는 중
    const double dot = rx*vx + ry*vy;
    if (dot >= 0.0) return false;

    // 표준식: 최근접 시각 (time-to-closest-approach)
    const double t = -dot / v2;  // 단위: 초(s)
    if (t <= 0.0) return false;

    ttc = t; // 초 단위
    return true;
}
/**
 * @brief 장애물과 주어진 전역 경로(Path) 사이의 최소 거리를 계산하는 함수
 *
 * 이 함수는 장애물(obstacle)과 전역 경로(path_x, path_y) 사이의 최소 수직 거리를 계산합니다.
 * 경로는 직선 구간들의 집합으로 구성되어 있으며, 각 구간(segment)에 대해 장애물로부터의 
 * 수직 거리를 계산하여 가장 짧은 거리를 반환합니다.
 *
 * - 경로가 유효하지 않으면(false) 반환
 * - 최소 거리가 계산되면 out_distance에 값을 저장하고 true 반환
 * - 장애물이 경로의 연장선상에 없으면 false 반환
 *
 * @param obstacle       장애물 정보 (위치 포함)
 * @param path_x         경로의 x 좌표 리스트
 * @param path_y         경로의 y 좌표 리스트
 * @param out_distance   계산된 최소 거리 (출력값)
 * @return true          최소 거리가 유효하게 계산된 경우
 * @return false         경로 데이터 오류 또는 유효한 거리 계산 불가 시
 */
bool calculateMinDistanceToPath(const adcm::obstacleListStruct& obstacle,
                                const std::vector<double>& path_x, 
                                const std::vector<double>& path_y,
                                double& out_distance)
{
    if (path_x.size() < 2 || path_y.size() < 2 || path_x.size() != path_y.size()) {
        adcm::Log::Error() << "Invalid trajectory data.";
        return false;
    }

    bool foundValid = false;
    double min_distance = std::numeric_limits<double>::max();

    for (size_t count = 0; count < path_x.size() - 1; ++count)
    {
        double x_start = path_x[count];
        double x_end   = path_x[count + 1];
        double y_start = path_y[count];
        double y_end   = path_y[count + 1];

        Point2D vec_path = {x_end - x_start, y_end - y_start};
        Point2D vec_obs  = {obstacle.fused_position_x - x_start, obstacle.fused_position_y - y_start};

        double mag_path = getMagnitude(vec_path);
        double mag_obs  = getMagnitude(vec_obs);

        double dot_product = vec_path.x * vec_obs.x + vec_path.y * vec_obs.y;

        if (dot_product > 0 && dot_product < mag_path * mag_path)
        {
            double cos_theta = dot_product / (mag_path * mag_obs);
            double perpendicular_distance = mag_obs * sqrt(1 - cos_theta * cos_theta);

            if (perpendicular_distance < min_distance) {
                min_distance = perpendicular_distance;
                foundValid = true;
            }
        }
    }

    if (foundValid) {
        out_distance = min_distance;
        //adcm::Log::Info() << "Min distance to path for Obstacle " << obstacle.obstacle_id 
        //                  << " is " << out_distance;
        return true;
    }

    adcm::Log::Info() << "Obstacle " << obstacle.obstacle_id << " is not within any trajectory segment.";
    return false;
}


/**
 * @brief 차량과 장애물의 선형 예측 기반 최소 거리 계산 함수
 *
 * 차량(Vehicle)과 장애물(Obstacle)의 현재 위치와 속도를 기준으로,
 * 향후 5초 동안 0.5초 간격으로 미래 위치를 선형적으로 예측합니다.
 * 각 시점마다 두 객체 간의 거리를 계산하여, 가장 가까운 거리를 반환합니다.
 *
 * - 속도가 일정하다고 가정 (선형 이동)
 * - 충돌 가능성 또는 근접 위험도 평가에 활용 가능
 *
 * @param obstacle  장애물 정보 (위치, 속도 포함)
 * @param vehicle   자차(ego vehicle) 정보
 * @return double   예측된 최소 거리 (단위: m 또는 cm, 시스템 기준에 따름)
 *                  유효한 값이 없을 경우 INVALID_RETURN_VALUE 반환
 */
// 선형(속도 일정) 가정 하에서 해석식으로 최소거리 계산
// - positions: dm(0.1 m) 가정, speeds: m/s
// - 반환: min_distance (dm). 좌표가 m이면 pos_unit_to_m=1.0로 호출.
// - horizon_s: 예측 시간창(초)
bool calculateMinDistanceLinear(const adcm::obstacleListStruct& obstacle,
                                const adcm::vehicleListStruct& vehicle,
                                double& min_distance)
{
    constexpr double horizon_s = 5.0;
    constexpr double pos_unit_to_m = 0.1;
    constexpr double EPS_V = 1e-3;  // m/s, 상대속도 거의 0
    if (horizon_s <= 0.0) return false;

    // 1) 좌표를 미터로 변환
    const double ox = obstacle.fused_position_x * pos_unit_to_m;
    const double oy = obstacle.fused_position_y * pos_unit_to_m;
    const double vx = vehicle.position_x        * pos_unit_to_m;
    const double vy = vehicle.position_y        * pos_unit_to_m;

    // 2) 상대 위치/속도 (미터, m/s)
    const double rx = ox - vx;
    const double ry = oy - vy;
    const double vxr = obstacle.fused_velocity_x - vehicle.velocity_x; // m/s
    const double vyr = obstacle.fused_velocity_y - vehicle.velocity_y; // m/s

    const double v2 = vxr*vxr + vyr*vyr;

    // 3) 상대속도 거의 0 → 거리 변화 거의 없음: t=0에서의 거리 반환
    if (v2 < EPS_V*EPS_V) {
        const double d0_m = std::hypot(rx, ry);
        // 반환은 입력 좌표 단위(dm)에 맞춰 환산
        min_distance = d0_m / pos_unit_to_m;
        return true;
    }

    // 4) 최근접 시각 t* 계산 및 [0, horizon_s]로 clamp
    const double dot = rx*vxr + ry*vyr;          // r·v
    double t_star = -dot / v2;                   // s
    if (t_star < 0.0)        t_star = 0.0;
    else if (t_star > horizon_s) t_star = horizon_s;

    // 5) t*에서의 최소거리(m) → 반환 단위(dm)로 변환
    const double rx_t = rx + vxr * t_star;
    const double ry_t = ry + vyr * t_star;
    const double dmin_m = std::hypot(rx_t, ry_t);

    min_distance = dmin_m / pos_unit_to_m;       // dm 로 환산 (pos_unit_to_m=0.1)
    return true;
}

/**
 * @brief 정렬되지 않은 장애물 리스트에서 "신규 객체" 추출 (자동 정렬 포함)
 *
 * vec_old와 vec_new를 obstacle_id 기준으로 정렬한 후,
 * vec_new에만 존재하는 장애물을 vec_output에 저장.
 *
 * @param vec_old     이전 장애물 리스트 (정렬 여부 무관)
 * @param vec_new     최신 장애물 리스트 (정렬 여부 무관)
 * @param vec_output  신규 장애물 리스트 (결과 저장)
 * @return true       신규 객체가 하나 이상 존재할 경우
 * @return false      신규 객체가 없을 경우
 */
bool extractNewObstacles(const obstacleListVector& vec_old,
                         const obstacleListVector& vec_new,
                         obstacleListVector& vec_output)
{
     vec_output.clear();

    std::unordered_set<decltype(adcm::obstacleListStruct{}.obstacle_id)> old_ids;
    old_ids.reserve(vec_old.size() * 2);

    for (const auto& o : vec_old) old_ids.insert(o.obstacle_id);

    adcm::Log::Info() << "[extractNewObstacles] 기존 개수=" << vec_old.size()
                      << ", 신규 후보 개수=" << vec_new.size();

    std::unordered_set<decltype(adcm::obstacleListStruct{}.obstacle_id)> pushed;
    pushed.reserve(vec_new.size());

    for (const auto& n : vec_new) {
        if (old_ids.find(n.obstacle_id) == old_ids.end()) {
            if (pushed.insert(n.obstacle_id).second) {
                adcm::Log::Info() << "[extractNewObstacles] 신규 발견 → ID=" << n.obstacle_id
                                  << ", Class=" << to_string(static_cast<ObstacleClass>(n.obstacle_class));
                vec_output.push_back(n);
            }
        } else {
            adcm::Log::Info() << "[extractNewObstacles] 기존과 동일 → ID=" << n.obstacle_id;
        }
    }

    adcm::Log::Info() << "[extractNewObstacles] 최종 신규 개수=" << vec_output.size();
    return !vec_output.empty();
//     // 1. obstacle_id 기준 정렬
//     std::sort(vec_old.begin(), vec_old.end(), 
//               [](const auto& a, const auto& b) { return a.obstacle_id < b.obstacle_id; });

//     std::sort(vec_new.begin(), vec_new.end(), 
//               [](const auto& a, const auto& b) { return a.obstacle_id < b.obstacle_id; });
              
//     adcm::Log::Info() << "[extractNewObstacles] 기존 객체 개수=" << vec_old.size()
//                     << ", 신규 객체 개수=" << vec_new.size();

//     // 2. 투 포인터 방식으로 신규 객체 추출
//     size_t i = 0, j = 0;
//     while (i < vec_old.size() && j < vec_new.size())
//     {
//         if (vec_new[j].obstacle_id < vec_old[i].obstacle_id)
//         {
//             adcm::Log::Info() << "[extractNewObstacles] 신규 장애물 발견 → ID=" << vec_new[j].obstacle_id
//                               << ", Class=" << to_string(static_cast<ObstacleClass>(vec_new[j].obstacle_class));
//             vec_output.push_back(vec_new[j]);
//             j++;
//         }
//         else if (vec_old[i].obstacle_id < vec_new[j].obstacle_id)
//         {
//             i++;
//         }
//         else
//         {
//             // 동일한 ID → 신규 아님
//             adcm::Log::Info() << "[extractNewObstacles] 기존 장애물과 동일 → ID=" << vec_new[j].obstacle_id;
//             i++;
//             j++;
//         }
//     }

// while (j < vec_new.size())
//     {
//         vec_output.push_back(vec_new[j++]);
//     }
    
//     adcm::Log::Info() << "[extractNewObstacles] 최종 신규 장애물 개수=" << vec_output.size();
//     return !vec_output.empty();

}

void detectUnscannedPath(const std::vector<adcm::map_2dListVector>& map_2d,
                         const std::vector<double>& path_x,
                         const std::vector<double>& path_y,
                         adcm::risk_assessment_Objects& riskAssessment)
{
    bool breakFlag; // 지정된 전역경로 (x1,y1) 과 (x2, y2) 사이 하나라도
    for (int count = 0; count < path_x.size() - 1; count++)
    {
        int x_start = floor(path_x[count]);
        int x_end = floor(path_x[count + 1]);
        int y_start = floor(path_y[count]);
        int y_end = floor(path_y[count + 1]);

        // Bresenham's line algorithm
        const bool steep = (fabs(y_end - y_start) > fabs(x_end - x_start));
        if (steep)
        {
            std::swap(x_start, y_start);
            std::swap(x_end, y_end);
        }

        if (x_start > x_end)
        {
            std::swap(x_start, x_end);
            std::swap(y_start, y_end);
        }
        const float dx = x_end - x_start;
        const float dy = fabs(y_end - y_start);
        float error = dx / 2.0f;
        const int ystep = (y_start < y_end) ? 1 : -1;
        int y = (int)y_start;
        const int maxX = x_end;

        for (int x = x_start; x <= maxX; x++)
        {
            if (steep)
            {
                if (map_2d[y][x].road_z != 1) // 스캔되지 않은 map 의 index - 노면정보 X
                {
                    adcm::riskAssessmentStruct riskAssessment7;
                    riskAssessment7.wgs84_xy_start.clear();
                    riskAssessment7.wgs84_xy_end.clear();
                    adcm::globalPathPosition unscanned_start_path, unscanned_end_path;
                    unscanned_start_path.x = path_x[count];
                    unscanned_start_path.y = path_y[count];
                    unscanned_end_path.x = path_x[count + 1];
                    unscanned_end_path.y = path_y[count + 1];
                    riskAssessment7.wgs84_xy_start.push_back(unscanned_start_path);
                    riskAssessment7.wgs84_xy_end.push_back(unscanned_end_path);
                    riskAssessment7.hazard_class = SCENARIO_7;
                    riskAssessment7.isHarzard = true;
                    adcm::Log::Info() << "Risk assessment generated for #7 is X: " << path_x[count] << " Y: " << path_y[count] << " with flag 1 ";
                    riskAssessment.riskAssessmentList.push_back(riskAssessment7);
                    break; // 한번만 들어가도 for loop break
                }
            }

            else if (map_2d[x][y].road_z != 1) // 스캔되지 않은 map 의 index - 노면정보 X
            {
                adcm::riskAssessmentStruct riskAssessment7;
                // adcm::Log::Info() << "detectUnscannedPath test else";
                adcm::Log::Info() << "map_2d[" << x << "][" << y << "] is 0";
                riskAssessment7.wgs84_xy_start.clear();
                riskAssessment7.wgs84_xy_end.clear();
                adcm::globalPathPosition unscanned_start_path, unscanned_end_path;
                unscanned_start_path.x = path_x[count];
                unscanned_start_path.y = path_y[count];
                unscanned_end_path.x = path_x[count + 1];
                unscanned_end_path.y = path_y[count + 1];
                riskAssessment7.wgs84_xy_start.push_back(unscanned_start_path);
                riskAssessment7.wgs84_xy_end.push_back(unscanned_end_path);
                riskAssessment7.hazard_class = SCENARIO_7;
                riskAssessment7.isHarzard = true;
                adcm::Log::Info() << "Risk assessment generated for #7 is X: " << riskAssessment7.wgs84_xy_start[0].x << " Y: " << riskAssessment7.wgs84_xy_start[0].y << " with flag" << riskAssessment7.isHarzard;
                riskAssessment.riskAssessmentList.push_back(riskAssessment7);
                break;
            }
            /*if (map_2d[x][y].road_z ==1)
            {
                adcm::Log::Info() << "map_2d[" << x << "][" <<x << "] is 1";
                adcm::Log::Info() << "scanned! safe to go";
            }*/

            error -= dy;
            if (error < 0)
            {
                y += ystep;
                error += dx;
            }
        }
    }
}

void calculateShiftedLines(int &x_start, int &x_end, int &y_start, int &y_end, int shift, double &original_m, double &original_c, double &up_c, double &down_c, bool &isVertical, double &x_up, double &x_down)
{
    // Calculate the slope (gradient) of the original line

    double dx = x_end - x_start;
    double dy = y_end - y_start;

    // Check if the line is vertical
    if (dx == 0)
    {
        isVertical = true;
        original_m = INFINITY; // Undefined slope
        original_c = INFINITY; // No y-intercept
        x_up = x_start + shift;
        x_down = x_start - shift;
        return;
    }
    isVertical = false;
    original_m = dy / dx; // Slope of the original line
    original_c = y_start - original_m * x_start; // Intercept of the original line

    // Adjust intercepts for vertical shifts
    up_c = original_c + shift;
    down_c = original_c - shift;
}

// 특장차와 임의의 좌표(px, py) 사이의 거리를 dm(0.1m) 단위로 계산
double distanceEgoToPointDm(const adcm::vehicleListStruct& vehicle, double px, double py)
{
    double dx = vehicle.position_x - px;  // x축 차이 (dm)
    double dy = vehicle.position_y - py;  // y축 차이 (dm)

    double dist_dm = std::sqrt(dx * dx + dy * dy);  // 거리 (dm)
    return dist_dm;
}

// 장애물과 임의의 좌표(px, py) 사이의 거리를 dm(0.1m) 단위로 계산
double distanceObsToPointDm(const adcm::obstacleListStruct& obs, double px, double py)
{
    double dx = obs.fused_position_x - px;  // x축 차이 (dm)
    double dy = obs.fused_position_y - py;  // y축 차이 (dm)

    double dist_dm = std::sqrt(dx * dx + dy * dy);  // 거리 (dm)
    return dist_dm;
}

bool isOutOfWork(uint8_t z) { return z == 0xFF; }

bool zToCm(uint8_t z, double& out_cm) {
    switch (z) {
        case 0:   out_cm = -50.0; return true; // ≤ -50
        case 1:   out_cm = -47.5; return true; // -50 ~ -45
        case 2:   out_cm = -42.5; return true; // -45 ~ -40
        case 3:   out_cm = -37.5; return true; // -40 ~ -35
        case 4:   out_cm = -32.5; return true; // -35 ~ -30
        case 5:   out_cm = -27.5; return true; // -30 ~ -25
        case 6:   out_cm = -22.5; return true; // -25 ~ -20
        case 7:   out_cm = -17.5; return true; // -20 ~ -15
        case 8:   out_cm = -12.5; return true; // -15 ~ -10
        case 9:   out_cm =  -7.5; return true; // -10 ~ -5
        case 10:  out_cm =  -2.5; return true; // -5  ~ 0
        case 11:  out_cm =  +2.5; return true; // 0   ~ +5
        case 12:  out_cm =  +7.5; return true; // +5  ~ +10
        case 13:  out_cm = +12.5; return true; // +10 ~ +15
        case 14:  out_cm = +17.5; return true; // +15 ~ +20
        case 15:  out_cm = +22.5; return true; // +20 ~ +25
        case 16:  out_cm = +27.5; return true; // +25 ~ +30
        case 17:  out_cm = +32.5; return true; // +30 ~ +35
        case 18:  out_cm = +37.5; return true; // +35 ~ +40
        case 19:  out_cm = +42.5; return true; // +40 ~ +45
        case 20:  out_cm = +47.5; return true; // +45 ~ +50
        case 21:  out_cm = +52.5; return true; // +50 ~ +55
        case 22:  out_cm = +55.0; return true; // ≥ +55
        case 0xFF: return false;               // 작업영역 외 지역
        default:   return false;               // 잘못된 값
    }
}
