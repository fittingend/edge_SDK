#ifndef __MAIN_DATAFUSION_HPP__
#define __MAIN_DATAFUSION_HPP__

#include "id_manager.hpp"

#define map_n 2500
#define map_m 1500
#define ABS(x) ((x >= 0) ? x : -x)
#define INVALID_VALUE 999

#define M_TO_10CM_PRECISION 10.0
#define MAIN_VEHICLE_SIZE_X 10
#define MAIN_VEHICLE_SIZE_Y 10

#define SUB_VEHICLE_SIZE_X 21.92
#define SUB_VEHICLE_SIZE_Y 15.99
#define MAP_ANGLE -86
//사이즈는 10cm 단위 기준

//#define KM_TO_MS_CONVERSION 5/18

enum ObstacleClass
{
    NO_OBSTACLE,
    VEHICLE_LARGE,
    VEHICLE_SMALL,
    PEDESTRIAN,
    STRUCTURE
};
/*
enum VehicleClass
{
    EGO_VEHICLE,
    SUB_VEHICLE_1,
    SUB_VEHICLE_2,
    SUB_VEHICLE_3,
    SUB_VEHICLE_4,
    NO_VEHICLE
};
*/

enum VehicleClass
{
    EGO_VEHICLE = 240,
    SUB_VEHICLE_1 = 0,
    SUB_VEHICLE_2 = 1 ,
    SUB_VEHICLE_3 = 2,
    SUB_VEHICLE_4 = 3,
    NO_VEHICLE = 4
};

typedef struct
{
    long x, y;
} Point2D; 

struct ObstacleData
{
    std::uint16_t obstacle_id;
    std::uint8_t obstacle_class;
    std::uint64_t timestamp;
//    std::vector<std::pair<unsigned short,unsigned short>> map_2d_location; //장애물이 위치한 2d 그리드 맵의 index 페어를 저장
    std::vector<Point2D> map_2d_location;
    std::uint8_t stop_count; 
    double fused_cuboid_x;
    double fused_cuboid_y;
    double fused_cuboid_z;
    double fused_heading_angle;
    double fused_position_x;
    double fused_position_y;
    double fused_position_z;
    double fused_velocity_x;
    double fused_velocity_y;
    double fused_velocity_z;
};

struct VehicleSizeData
{
    std::uint16_t length;
    std::uint16_t width;
};

struct BoundaryData
{
    double x;
    double y;
};

struct VehicleData
{
    unsigned char vehicle_class;
//    std::vector<std::pair<unsigned short,unsigned short>> map_2d_location; //해당 차량이 위치한 2d 그리드 맵의 index 페어를 저장
//    std::vector<Point2D>
    std::vector<Point2D> map_2d_location;
//    std::time_t timestamp;
    std::uint64_t timestamp;
    std::vector<double> road_z;
//    double road_z[4];
    double position_long; // x equivalent
    double position_lat; // y equivalent
    double position_height;
    double position_x; // new - to assign
    double position_y; //new - to assign
    double position_z;             
    double yaw;
    double roll;
    double pitch;
    double velocity_long;
    double velocity_lat;
    double velocity_x; //new- to assign
    double velocity_y;// new- to assign
    double velocity_ang;
};

struct GridCellData
{
    unsigned short obstacle_id;
    VehicleClass vehicle_class;
    double road_z; //2 bytes
};  //패딩때문에 토탈 24 bytes


struct MapData
{
    GridCellData map_2d[map_n][map_m]; //각각 24bytes
    std::vector<ObstacleData> obstacle_list;
    std::vector<VehicleData> vehicle_list;

}; //maptdata 사이즈는 24*4000*5000+24+24 = 480000048 byte = 약 450MB 

struct FusionData
{
    std::vector<ObstacleData> obstacle_list;
    std::vector<VehicleData> vehicle_list;
};
#endif