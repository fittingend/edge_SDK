#include "main_riskassessment.hpp"

void GPStoUTM(double lat, double lon, double &utmX, double &utmY)
{
    INFO("[RiskAssessment] GPStoUTM");
    // WGS84 Parameters
    const double WGS84_A = 6378137.0;    // Major semiaxis [m]
    const double WGS84_E = 0.0818191908; // First Eccentricity
    // Constants
    const double k0 = 0.9996;
    const double eSquared = WGS84_E * WGS84_E;
    const double ePrimeSquared = eSquared / (1 - eSquared);
    const double RADIANS_PER_DEGREE = M_PI / 180.0;
    int zone = 52;
    // Calculate the Central Meridian of the Zone
    double lonOrigin = (zone - 1) * 6 - 180 + 3;

    // Convert lat/lon to radians
    double latRad = lat * RADIANS_PER_DEGREE;
    double lonRad = lon * RADIANS_PER_DEGREE;
    double lonOriginRad = lonOrigin * RADIANS_PER_DEGREE;

    // Calculate UTM coordinates
    double N = WGS84_A / sqrt(1 - eSquared * sin(latRad) * sin(latRad));
    double T = tan(latRad) * tan(latRad);
    double C = ePrimeSquared * cos(latRad) * cos(latRad);
    double A = cos(latRad) * (lonRad - lonOriginRad);

    double M = WGS84_A * ((1 - eSquared / 4 - 3 * pow(eSquared, 2) / 64 - 5 * pow(eSquared, 3) / 256) * latRad - (3 * eSquared / 8 + 3 * pow(eSquared, 2) / 32 + 45 * pow(eSquared, 3) / 1024) * sin(2 * latRad) + (15 * pow(eSquared, 2) / 256 + 45 * pow(eSquared, 3) / 1024) * sin(4 * latRad) - (35 * pow(eSquared, 3) / 3072) * sin(6 * latRad));
    utmX = (k0 * N * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ePrimeSquared) * A * A * A * A * A / 120) + 500000.0);
    utmY = (k0 * (M + N * tan(latRad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * ePrimeSquared) * A * A * A * A * A * A / 720)));

    if (lat < 0)
    {
        utmY += 10000000.0; // 10000000 meter offset for southern hemisphere
    }
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
    if (point.x > MAP_N)
        point.x = MAP_N - 1;
    if (point.y > MAP_M)
        point.y = MAP_M - 1;
}

void gpsToMapcoordinate(const routeVector& route, 
                        doubleVector& path_x, 
                        doubleVector& path_y)
{
    INFO("[RiskAssessment] gpsToMapcoordinate");

    // wps84기반 gps(global)좌표계를 작업환경 XY 기반의 Map 좌표계로 변환
    // 시뮬레이터 map 기준 원점(0,0) global좌표
    constexpr double mapOrigin_x = 453.088714;
    constexpr double mapOrigin_y = 507.550078;
    // 시뮬레이터 기준점 utm좌표
    constexpr double origin_x = 278296.968;
    constexpr double origin_y = 3980466.846;
    const double angle_radians = -MAP_ANGLE * M_PI / 180.0;

    for (std::size_t i = 0; i < route.size(); ++i)
    {
        const auto& point = route[i];
        double utm_x, utm_y;
        GPStoUTM(point.latitude, point.longitude, utm_x, utm_y);

        utm_x -= origin_x;
        utm_y -= origin_y;

        Point2D mapPoint;
        mapPoint.x = (utm_x * cos(angle_radians) - utm_y * sin(angle_radians) - mapOrigin_x) * M_TO_10CM_PRECISION;
        mapPoint.y = (utm_x * sin(angle_radians) + utm_y * cos(angle_radians) - mapOrigin_y) * M_TO_10CM_PRECISION;

        checkRange(mapPoint);

        path_x[i] = mapPoint.x;
        path_y[i] = mapPoint.y;
        
        adcm::Log::Info() << "gpsToMapcoordinate 변환: (" 
                        << point.latitude << ", " << point.longitude 
                        << ") → (" << mapPoint.x << ", " << mapPoint.y << ")";
    }
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
 *
 * 이 함수는 차량과 장애물의 상대 위치 및 상대 속도를 기반으로,
 * 선형 이동 가정 하에 두 객체가 충돌할 때까지 걸리는 예상 시간을 계산합니다.
 *
 * - 상대 속도가 거의 없을 경우(TTC 계산 불가) false 반환
 * - TTC가 0 이하일 경우(충돌 없음) false 반환
 * - 유효한 TTC가 계산되면 해당 값을 ttc에 저장하고 true 반환
 *
 * @param obstacle   장애물 정보 (위치 및 속도 포함)
 * @param vehicle    자차 정보 (위치 및 속도 포함)
 * @param ttc        계산된 TTC 값 (출력값, 단위: 초)
 * @return true      유효한 TTC가 계산된 경우 (양수)
 * @return false     상대 속도 부족 또는 충돌 예상 없음
 */
bool getTTC(const adcm::obstacleListStruct& obstacle, const adcm::vehicleListStruct& vehicle, double& ttc)
{
    constexpr double EPSILON = 1e-6;
    // Calculate relative position and velocity of the obstacle with respect to the vehicle
    double relative_position_x = obstacle.fused_position_x - vehicle.position_x;
    double relative_position_y = obstacle.fused_position_y - vehicle.position_y;
    double relative_velocity_x = obstacle.fused_velocity_x - vehicle.velocity_x;
    double relative_velocity_y = obstacle.fused_velocity_y - vehicle.velocity_y;

    // Prevent division by zero when relative_velocity_y is zero
    if (std::abs(relative_velocity_y) < EPSILON || std::abs(relative_velocity_x) < EPSILON)
    {
        return false; // No meaningful TTC can be calculated
    }

    // Calculate determinant (cross product) for relative motion
    double c = (relative_velocity_x * relative_position_y) - (relative_velocity_y * relative_position_x);

    // Calculate TTC using relative motion
    double calculated_ttc = ((c / (2 * relative_velocity_y)) - relative_position_x) / relative_velocity_x;

    // Only assign TTC if it's positive (future collision)
    if (calculated_ttc > 0) {
        ttc = calculated_ttc;
        return true;
    }
    
    return false;
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
                                const doubleVector& path_x, 
                                const doubleVector& path_y,
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
        adcm::Log::Info() << "Min distance to path for Obstacle " << obstacle.obstacle_id 
                          << " is " << out_distance;
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
bool calculateMinDistanceLinear(const adcm::obstacleListStruct& obstacle, const adcm::vehicleListStruct& vehicle, double& min_distance)
{
    // 최소 거리를 매우 큰 값으로 초기화
    min_distance = std::numeric_limits<double>::max();
    bool is_valid = false;

    // Loop to calculate positions at intervals and find the minimum distance
    for (double k = 0.0; k <= 5.0; k += 0.5)
    {
        // Compute positions at time `k`
        double vehicle_future_x = vehicle.position_x + vehicle.velocity_x * k;
        double vehicle_future_y = vehicle.position_y + vehicle.velocity_y * k;
        double obstacle_future_x = obstacle.fused_position_x + obstacle.fused_velocity_x * k;
        double obstacle_future_y = obstacle.fused_position_y + obstacle.fused_velocity_y * k;

        // Calculate the distance between future positions
        double temp_distance = sqrt(pow(obstacle_future_x - vehicle_future_x, 2) +
                                    pow(obstacle_future_y - vehicle_future_y, 2));

        // Update minimum distance if a smaller value is found
        if (temp_distance < min_distance)
        {
            min_distance = temp_distance;
            is_valid = true;
        }
    }

    return is_valid;
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
bool extractNewObstacles(obstacleListVector vec_old, 
                        obstacleListVector vec_new, 
                        obstacleListVector& vec_output)
{
    // 1. obstacle_id 기준 정렬
    std::sort(vec_old.begin(), vec_old.end(), 
              [](const auto& a, const auto& b) { return a.obstacle_id < b.obstacle_id; });

    std::sort(vec_new.begin(), vec_new.end(), 
              [](const auto& a, const auto& b) { return a.obstacle_id < b.obstacle_id; });

    // 2. 투 포인터 방식으로 신규 객체 추출
    size_t i = 0, j = 0;
    while (i < vec_old.size() && j < vec_new.size())
    {
        if (vec_new[j].obstacle_id < vec_old[i].obstacle_id)
        {
            vec_output.push_back(vec_new[j]);
            j++;
        }
        else if (vec_old[i].obstacle_id < vec_new[j].obstacle_id)
        {
            i++;
        }
        else
        {
            i++;
            j++;
        }
    }

while (j < vec_new.size())
    {
        vec_output.push_back(vec_new[j++]);
    }
    
    return !vec_output.empty();

}

void detectUnscannedPath(const std::vector<adcm::map_2dListVector>& map_2d,
                         const doubleVector& path_x,
                         const doubleVector& path_y,
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
