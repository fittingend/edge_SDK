#define n 40
#define m 50
#define EDGE_WORKSHOP
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
    int obstacle_id; // 추가됨 obstacle tracking 및 고유 id 부여가 필요
    ObstacleClass obstacle_class;
    std::string timestamp;
    //    std::vector<int> map_2d_location;
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

    ObstacleData(const int obstacle_id, const ObstacleClass obstacle_class, const std::string timestamp,
                            const ActionClass action_required, const unsigned short int fused_cuboid_x, const unsigned short int fused_cuboid_y, const unsigned short int fused_cuboid_z,
                            const unsigned short int fused_heading_angle, const unsigned short int fused_Position_x, const unsigned short int fused_Position_y, const unsigned short int fused_Position_z,
                            const unsigned short int fused_velocity_x, const unsigned short int fused_velocity_y, const unsigned short int fused_velocity_z)
        : obstacle_id(obstacle_id), obstacle_class(obstacle_class), timestamp(timestamp), action_required(action_required),
          fused_cuboid_x(fused_cuboid_x), fused_cuboid_y(fused_cuboid_y), fused_cuboid_z(fused_cuboid_z), fused_heading_angle(fused_heading_angle),
          fused_Position_x(fused_Position_x), fused_Position_y(fused_Position_y), fused_Position_z(fused_Position_z), fused_velocity_x(fused_velocity_x),
          fused_velocity_y(fused_velocity_y), fused_velocity_z(fused_velocity_z) {}
};

struct ObstacleEnvData
{
    // ObstacleData *obstacle_data; //8bytes
    unsigned short int obstacle_id;
    short int road_z; //2 bytes
    //패딩때문에 토탈 16 bytes
};

// obstacle list 중복 확인을 위한 operator overload
bool operator==(const ObstacleEnvData &m1, const ObstacleEnvData &m2)
{
    if (m1.obstacle_id == m2.obstacle_id)
        return true;
    else
        return false;
}

struct VehicleData
{
    VehicleClass vehicle_id;
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

    // member function declaration
    bool map_2d_init(ObstacleEnvData map_2d[n][m])
    {
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < m; j++)
                map_2d[i][j] = {0}; // initize rest of members to 0
        }
        return true;
    }
};