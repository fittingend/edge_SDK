#include <vector> 
#include <chrono>
HubData hub_data;

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

struct HubObstacleData
{
//    unsigned short obstacle_id;
    ObstacleClass obstacle_class;
    std::time_t timestamp;
    float cuboid_x;
    float cuboid_y;
    float cuboid_z;
    float heading_angle;
    float position_x; // m 로 가정
    float position_x_time_corrected; 
    float position_y;
    float position_y_time_corrected;
    float position_z;
    float velocity_x; // m/s 으로 가정
    float velocity_y;
    float velocity_z;

};
struct HubVehicleData
{
    std::vector<HubObstacleData> obstacle;
    std::vector<float> road_z; 
    VehicleClass vehicle_class;
    std::time_t timestamp;
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
struct HubData
{
    std::vector<HubVehicleData> vehicle;
};

