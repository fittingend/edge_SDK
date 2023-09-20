#define n 40
#define m 50
#define EDGE_WORKSHOP
enum obstacle_type
{
    NO_OBSTACLE,
    VEHICLE_LARGE,
    VEHICLE_SMALL,
    PEDESTRIAN,
    STRUCTURE
};

enum vehicle_ID
{
    NO_VEHICLE,
    EGO_VEHICLE,
    SUB_VEHICLE_1,
    SUB_VEHICLE_2,
    SUB_VEHICLE_3,
    SUB_VEHICLE_4
};
enum action_type{
    NO_ACTION,
    REMOVE_BLIND_SPOT,
    ALERT_OBSTACLE
};
struct fused_obstacle_env_data_type
{
    int obstacle_id; //추가됨 obstacle tracking 및 고유 id 부여가 필요 
    obstacle_type fused_index;
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
bool operator == (const fused_obstacle_env_data_type &m1, const fused_obstacle_env_data_type &m2)
{
   if(m1.obstacle_id == m2.obstacle_id)
     return true;
   else
     return false;
}

struct fused_vehicle_data_type
{
    vehicle_ID vehicle_id;
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


struct obstacle_list_data_type
{
    int obstacle_id; //추가됨 obstacle tracking 및 고유 id 부여가 필요 
    obstacle_type obstacle_index;
    std::string timestamp;
//    std::vector<int> map_2d_location; 
    action_type action_required;
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

    obstacle_list_data_type(const int obstacle_id, const obstacle_type obstacle_index, const std::string timestamp,\
    const action_type action_required, const double fused_cuboid_x, const double fused_cuboid_y, const double fused_cuboid_z,\
    const double fused_heading_angle, const double fused_Position_x,const double fused_Position_y, const double fused_Position_z,\
    const double fused_velocity_x, const double fused_velocity_y, const double fused_velocity_z)
    :obstacle_id(obstacle_id), obstacle_index(obstacle_index), timestamp(timestamp), action_required(action_required),\
    fused_cuboid_x(fused_cuboid_x),fused_cuboid_y(fused_cuboid_y),fused_cuboid_z(fused_cuboid_z),fused_heading_angle(fused_heading_angle),\
    fused_Position_x(fused_Position_x),fused_Position_y(fused_Position_y),fused_Position_z(fused_Position_z),fused_velocity_x(fused_velocity_x),\
    fused_velocity_y(fused_velocity_y),fused_velocity_z(fused_velocity_z) {}
};

class map_data_type
{
    public:
    fused_obstacle_env_data_type map_2d[n][m];
    std::vector <fused_vehicle_data_type> vehicle_list; 
    //vehicle 개수가 가변적이라 vector 로 변경 필요

 //member function declaration   
    bool map_2d_init(fused_obstacle_env_data_type map_2d[n][m]) 
    {
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < m; j++)
            map_2d[i][j] = {0}; // initize rest of members to 0 
        }
        return true;
    }


};