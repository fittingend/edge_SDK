#define n 1000 
#define m 2000
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

struct ObstacleEnvData
{
    int obstacle_id; //추가됨 obstacle tracking 및 고유 id 부여가 필요 
    ObstacleClass obstacle_class;
    std::string timestamp;
    double fused_cuboid_x;
    double fused_cuboid_y;
    double fused_cuboid_z;
    double fused_heading_angle;
    double fused_Position_x;
    double fused_Position_y;
    double fused_Position_z;
    double fused_velocity_x;
    double fused_velocity_y;
    double fused_velocity_z;
    double road_z[3];//TO DO: to define later 
};

//obstacle list 중복 확인을 위한 operator overload
bool operator == (const ObstacleEnvData &m1, const ObstacleEnvData &m2)
{
   if(m1.obstacle_id == m2.obstacle_id)
     return true;
   else
     return false;
}

struct VehicleData
{
    VehicleClass vehicle_id;
    double Position_lat;
    double Position_long;
    double Position_height;
    double Yaw;
    double Roll;
    double Pitch;
    double Velocity_long;
    double Velocity_lat;
    double Velocity_ang;
};

class MapData
{
    public:
    ObstacleEnvData map_2d[n][m];
    std::vector <VehicleData> vehicle_list; 
    //vehicle 개수가 가변적이라 vector 로 변경 필요 -> 그것보다는 array가 나아서?

 //member function declaration   
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
class ObstacleDataList
{
    public:
        ObstacleEnvData obstacle_data;
        std::vector<std::pair<int,int>> map_2d_location; 
        ActionClass action_required;

        ObstacleDataList(ObstacleEnvData obstacle_data, const std::vector<std::pair<int,int>> map_2d_location, const ActionClass action_required):\
        obstacle_data(obstacle_data), map_2d_location(map_2d_location),action_required(action_required) {}

};


//=====main_riskassessment specific definitions=======
enum HazardClass
{
    NO_HAZARD,
    BLIND_SPOT,
    PEDESTRIAN_HAZARD
};

class RiskAssessment 
{
    public:
        int obstacle_id; //obstacle 고유 id
        HazardClass hazard_class;
        bool isHazard;
        float confidence;
        
        RiskAssessment(const int obstacle_id, const HazardClass hazard_class, const bool isHazard, const float confidence):\
        obstacle_id(obstacle_id), hazard_class(hazard_class), isHazard(isHazard), confidence(confidence) {}
};
