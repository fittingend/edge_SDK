#define n 4000
#define m 5000

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
    unsigned short obstacle_id;
    ObstacleClass obstacle_class;
    std::string timestamp;
    std::vector<std::pair<unsigned short,unsigned short>> map_2d_location; //장애물이 위치한 2d 그리드 맵의 index 페어를 저장
    ActionClass action_class;
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

struct ObstacleEnvData
{
    ObstacleData *obstacle_data; //8 bytes
    float road_z; //2 bytes
    //패딩때문에 토탈 16 bytes
};

struct VehicleData
{
    VehicleClass vehicle_class;
    float position_lat;
    float position_long;
    float position_height;
    float yaw;
    float roll;
    float pitch;
    float velocity_long;
    float velocity_lat;
    float velocity_ang;
};

struct MapData
{
    ObstacleEnvData map_2d[n][m]; //16bytes
    std::vector<VehicleData> vehicle_list; //32bytes

};