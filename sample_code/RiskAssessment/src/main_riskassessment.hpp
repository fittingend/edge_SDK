#define map_n 4000
#define map_m 5000
#define STRUCTURE_DISTANCE 20
#define PEDESTRIAN_DISTANCE 15
#define PEDESTRIAN_TTC 7
#define COLLISION_DISTANCE 3
#define CONFIDENCE_THRESHOLD 0.7
#define STOP_VALUE 10 // 1초간 정지시 정지 장애물로 판단
#define INVALID_RETURN_VALUE 99999 
typedef struct
{
    long x, y;
} Point2D; 

enum ObstacleClass
{
    NO_OBSTACLE,
    VEHICLE_LARGE,
    VEHICLE_SMALL,
    PEDESTRIAN,
    STRUCTURE
};

enum VehicleClass
{
    NO_VEHICLE,
    EGO_VEHICLE,
    SUB_VEHICLE_1,
    SUB_VEHICLE_2,
    SUB_VEHICLE_3,
    SUB_VEHICLE_4
};
// enum ActionClass
// {
//     NO_ACTION,
//     REMOVE_BLIND_SPOT,
//     ALERT_OBSTACLE
// };

struct ObstacleData
{
    unsigned short obstacle_id;
    ObstacleClass obstacle_class;
    std::time_t timestamp;
    std::vector<std::pair<unsigned short,unsigned short>> map_2d_location; //장애물이 위치한 2d 그리드 맵의 index 페어를 저장
    // ActionClass action_class;
    int stop_count; 
    float fused_cuboid_x;
    float fused_cuboid_y;
    float fused_cuboid_z;
    float fused_heading_angle;
    float fused_position_x;
    float fused_position_y;
    float fused_position_z;
    float fused_velocity_x;
    float fused_velocity_y;
    float fused_velocity_z;
};

struct VehicleData
{
    VehicleClass vehicle_class;
    std::vector<std::pair<unsigned short,unsigned short>> map_2d_location; //해당 차량이 위치한 2d 그리드 맵의 index 페어를 저장
    std::time_t timestamp;
    float road_z[4];
    float position_lat;
    float position_long;
    float position_height;
    float position_x;
    float position_y;
    float position_z;             
    float yaw;
    float roll;
    float pitch;
    float velocity_long;
    float velocity_lat;
    float velocity_x;
    float velocity_y;
    float velocity_ang;
};
struct GridCellData
{
//    ObstacleData *obstacle_data; //8 bytes
    unsigned short obstacle_id;
//    VehicleData *vehicle_data; //8 bytes
    VehicleClass vehicle_class;
    float road_z; //2 bytes
};  //패딩때문에 토탈 24 bytes



struct MapData
{
    GridCellData map_2d[map_n][map_m]; //각각 24bytes
    std::vector<ObstacleData> obstacle_list;
    std::vector<VehicleData> vehicle_list; //24bytes
}; //maptdata 사이즈는 24*4000*5000+24 = 480000024 byte = 약 450MB 

//=====전역경로 관련====

struct build_path_Objects
{
    std::vector<float> utm_x;
    std::vector<float> utm_y;
};

//=====main_riskassessment specific definitions=======

enum HazardClass
{
    NO_HAZARD,
    SCENARIO_1,
    SCENARIO_2,
    SCENARIO_3,
    SCENARIO_4,
    SCENARIO_5,
    SCENARIO_6,
    SCENARIO_7
};

struct RiskAssessment
{
    unsigned short obstacle_id; //obstacle 고유 id
    std::pair<float, float> utm_xy_start; //위험한 전역경로 XY 시작 값
    std::pair<float, float> utm_xy_end;   //위험한 전역경로 XY 끝 값
    HazardClass hazard_class;
    bool isHazard;
    float confidence;

    RiskAssessment(const std::pair<float, float> utm_xy_start, const std::pair<float, float> utm_xy_end, const HazardClass hazard_class, const bool isHazard) :\
       obstacle_id(INVALID_RETURN_VALUE), utm_xy_start(utm_xy_start), utm_xy_end(utm_xy_end), hazard_class(hazard_class), isHazard(isHazard) {}

    RiskAssessment(const unsigned short obstacle_id, const HazardClass hazard_class, const float confidence) :\
        obstacle_id(obstacle_id),hazard_class(hazard_class), confidence(confidence) {}
};
