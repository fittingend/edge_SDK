#define map_n 3000
#define map_m 1000
#define MAP_TILT_ANGLE -86
#define STOP_VALUE 10 // 1초간 정지시 정지 장애물로 판단
#define INVALID_RETURN_VALUE 99999 
#define M_TO_10CM_PRECISION 10 


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
