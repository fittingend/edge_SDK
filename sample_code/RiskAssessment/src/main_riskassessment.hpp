#define n 4000
#define m 5000
#define STRUCTURE_DISTANCE 20
#define PEDESTRIAN_DISTANCE 15
#define PEDESTRIAN_TTC 7
#define COLLISION_DISTANCE 3
#define CONFIDENCE_THRESHOLD 0.7

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
enum ActionClass
{
    NO_ACTION,
    REMOVE_BLIND_SPOT,
    ALERT_OBSTACLE
};

struct ObstacleData
{
    unsigned short int obstacle_id;
    ObstacleClass obstacle_class;
    std::string timestamp;
    std::vector<std::pair<unsigned short int,unsigned short int>> map_2d_location; //장애물이 위치한 2d 그리드 맵의 index 페어를 저장
    ActionClass action_required;
    unsigned short int fused_cuboid_x;
    unsigned short int fused_cuboid_y;
    unsigned short int fused_cuboid_z;
    unsigned short int fused_heading_angle;
    unsigned short int fused_Position_x;
    unsigned short int fused_Position_y;
    unsigned short int fused_Position_z;
    unsigned short int fused_velocity_x;
    unsigned short int fused_velocity_y;
    unsigned short int fused_velocity_z;

};

struct ObstacleEnvData
{
    ObstacleData *obstacle_data; //8 bytes
    short int road_z; //2 bytes
    //패딩때문에 토탈 16 bytes
};

struct VehicleData
{
    VehicleClass vehicle_class;
    unsigned short int Position_lat;
    unsigned short int Position_long;
    unsigned short int Position_height;
    unsigned short int Yaw;
    unsigned short int Roll;
    unsigned short int Pitch;
    unsigned short int Velocity_long;
    unsigned short int Velocity_lat;
    unsigned short int Velocity_ang;
};

struct MapData
{
    ObstacleEnvData map_2d[n][m]; //16bytes
    std::vector<VehicleData> vehicle_list; //32bytes

};

//=====main_riskassessment specific definitions=======

enum HazardClass
{
    NO_HAZARD,
    BLIND_SPOT,
    PEDESTRIAN_HAZARD
};

struct RiskAssessment 
{
    unsigned short int obstacle_id; //obstacle 고유 id
    HazardClass hazard_class;
    bool isHazard;
    float confidence;
        
    RiskAssessment(const unsigned short int obstacle_id, const HazardClass hazard_class, const bool isHazard, const float confidence):\
    obstacle_id(obstacle_id), hazard_class(hazard_class), isHazard(isHazard), confidence(confidence) {}
};
