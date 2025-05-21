#define map_n 3000
#define map_m 1000
#define MAP_TILT_ANGLE -86
#define STOP_VALUE 10 // 1초간 정지시 정지 장애물로 판단
#define INVALID_RETURN_VALUE 99999 
#define M_TO_10CM_PRECISION 10 
#define MAP_ANGLE -86

using namespace std;
typedef struct
{
    double x, y;
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
    EGO_VEHICLE = 0xF0,
    SUB_VEHICLE_1 = 0x00,
    SUB_VEHICLE_2 = 0x01,
    SUB_VEHICLE_3 = 0x02,
    SUB_VEHICLE_4 = 0x03,
    NO_VEHICLE = 0x04
};

enum HazardClass
{
    NO_HAZARD,
    SCENARIO_1,
    SCENARIO_2,
    SCENARIO_3,
    SCENARIO_4,
    SCENARIO_5,
    SCENARIO_6,
    SCENARIO_7,
    SCENARIO_8
};
