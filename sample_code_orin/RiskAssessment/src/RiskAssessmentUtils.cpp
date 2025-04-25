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

    for (int count = 0; count < route.size(); count++)
    {
        double utm_x, utm_y; // 해당 경로의 utm x,y 좌표
        GPStoUTM(route[count].latitude, route[count].longitude, utm_x, utm_y);

        utm_x -= origin_x;
        utm_y -= origin_y;

        adcm::Log::Info() << "utm_x:" << utm_x;
        adcm::Log::Info() << "utm_y:" << utm_y;

        Point2D mapPoint;

        mapPoint.x = (utm_x * cos(angle_radians) - utm_y * sin(angle_radians) - mapOrigin_x) * M_TO_10CM_PRECISION;
        mapPoint.y = (utm_x * sin(angle_radians) + utm_y * cos(angle_radians) - mapOrigin_y) * M_TO_10CM_PRECISION;

        checkRange(mapPoint);

        path_x[count] = mapPoint.x;
        path_y[count] = mapPoint.y;
        adcm::Log::Info() << "경로생성 값 gpsToMapcoordinate 좌표변환 before (" << route[count].latitude << " , " << route[count].longitude << ")";
        adcm::Log::Info() << "경로생성 값 gpsToMapcoordinate 좌표변환 after (" << path_x[count] << " , " << path_y[count] << ")";
    }
}

double getDistance(adcm::obstacleListStruct obstacle1, adcm::obstacleListStruct obstacle2)
{
    return sqrt(pow(obstacle1.fused_position_x - obstacle2.fused_position_x, 2) + pow(obstacle1.fused_position_y - obstacle2.fused_position_y, 2));
}
double getDistance(adcm::obstacleListStruct obstacle, adcm::vehicleListStruct vehicle)
{
    return sqrt(pow(obstacle.fused_position_x - vehicle.position_x, 2) + pow(obstacle.fused_position_y - vehicle.position_y, 2));
}
double getMagnitude(Point2D point)
{
    return sqrt(point.x * point.x + point.y * point.y);
}
double getTTC(const adcm::obstacleListStruct& obstacle, const adcm::vehicleListStruct& vehicle)
{
    // Calculate relative position and velocity of the obstacle with respect to the vehicle
    double relative_position_x = obstacle.fused_position_x - vehicle.position_x;
    double relative_position_y = obstacle.fused_position_y - vehicle.position_y;
    double relative_velocity_x = obstacle.fused_velocity_x - vehicle.velocity_x;
    double relative_velocity_y = obstacle.fused_velocity_y - vehicle.velocity_y;

    // Calculate determinant (cross product) for relative motion
    double c = (relative_velocity_x * relative_position_y) - (relative_velocity_y * relative_position_x);

    // Prevent division by zero when relative_velocity_y is zero
    if (relative_velocity_y == 0 || relative_velocity_x == 0) {
        return INVALID_RETURN_VALUE; // No meaningful TTC can be calculated
    }

    // Calculate TTC using relative motion
    double ttc = ((c / (2 * relative_velocity_y)) - relative_position_x) / relative_velocity_x;

    // Return TTC if valid, otherwise return a predefined invalid value
    return (ttc > 0) ? ttc : INVALID_RETURN_VALUE;
}
double getDistance_LinearTrajectory(const adcm::obstacleListStruct& obstacle,const doubleVector& path_x, const doubleVector& path_y)
{
    double distance = 0;

    if (path_x.size() < 2 || path_y.size() < 2 || path_x.size() != path_y.size()) {
        adcm::Log::Error() << "Invalid trajectory data.";
        return INVALID_RETURN_VALUE;
    }

    for (int count = 0; count < path_x.size(); count++)
    {
        double x_start = path_x[count];
        double x_end = path_x[count + 1];
        double y_start = path_y[count];
        double y_end = path_y[count + 1];

        // Vectors: start-to-end and start-to-obstacle
        Point2D start_to_end_vector = {x_end - x_start, y_end - y_start};
        Point2D start_to_obs_vector = {obstacle.fused_position_x - x_start, obstacle.fused_position_y - y_start};

        // Calculate angle
        double angle = atan2(start_to_end_vector.y, start_to_end_vector.x) - atan2(start_to_obs_vector.y, start_to_obs_vector.x);
        double cosine_angle = cos(angle);

        // Check if obstacle is between the start and end points
        double start_to_end_dot_start_to_obs = cosine_angle * getMagnitude(start_to_end_vector) * getMagnitude(start_to_obs_vector);
        double magnitude_start_to_end = getMagnitude(start_to_end_vector);

        if (start_to_end_dot_start_to_obs > 0 && start_to_end_dot_start_to_obs < pow(magnitude_start_to_end, 2))
        {
            // Calculate perpendicular distance
            double magnitude_start_to_obs = getMagnitude(start_to_obs_vector);
            double distance = magnitude_start_to_obs * sqrt(1 - pow(start_to_end_dot_start_to_obs / (magnitude_start_to_end * magnitude_start_to_obs), 2));

            adcm::Log::Info() << "Obstacle " << obstacle.obstacle_id << " is within trajectory segment " << count
                              << ". Distance: " << distance;
            return distance;
        }
    }
    adcm::Log::Info() << "Obstacle " << obstacle.obstacle_id << " is not within any trajectory segment.";
    return INVALID_RETURN_VALUE;
}
double getLinearApprox(const adcm::obstacleListStruct& obstacle, const adcm::vehicleListStruct& vehicle)
{
    // Initialize minimum distance to a very large value
    double min_distance_ego_obs = INVALID_RETURN_VALUE;

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
        if (temp_distance < min_distance_ego_obs)
        {
            min_distance_ego_obs = temp_distance;
        }
    }

    // Return the smallest distance found
    return min_distance_ego_obs;
}
void symmDiff(const obstacleListVector& vec1, const obstacleListVector& vec2, obstacleListVector &output, int n, int m)
{
    // Traverse both arrays simultaneously.
    int i = 0, j = 0;
    while (i < n && j < m)
    {
        // Print smaller element and move
        // ahead in array with smaller element
        if (vec1[i].obstacle_id < vec2[j].obstacle_id)
        {
            output.push_back(vec1[i]);
            i++;
        }
        else if (vec2[j].obstacle_id < vec1[i].obstacle_id)
        {
            output.push_back(vec2[j]);
            j++;
        }
        // If both elements same, move ahead
        // in both arrays.
        else
        {
            i++;
            j++;
        }
    }
    while (i < n)
    {
        output.push_back(vec1[i]);
        i++;
    }
    while (j < m)
    {
        output.push_back(vec2[j]);
        j++;
    }
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
