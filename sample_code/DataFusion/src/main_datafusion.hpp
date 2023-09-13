#define MAP_GENERATION
#define n 1000 
#define m 2000
#ifdef MAP_GENERATION

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

struct map_2d_data_type
{
    char timestamp[10];
    obstacle_type fused_index;
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

struct vehicle_list_data_type
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


class obstacle_list_data_type
{
    char timestamp[10];
    obstacle_type obstacle_index;
//    std::vector<int> map_2d_location; 
    enum action_type{
        NO_ACTION,
        REMOVE_BLIND_SPOT,
        ALERT_OBSTACEL
    };
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
};

class map_data_type
{
    public:
    map_2d_data_type map_2d[n][m];
    vehicle_list_data_type vehicle_list[5]; 
    //vehicle 개수가 가변적이라 vector 로 변경 필요

 //member function declaration   
    bool map_2d_init(map_2d_data_type map_2d[n][m]) {
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        map_2d[i][j] = {NO_OBSTACLE}; // initize rest of members to 0 
    }
    return true;
}


};

#endif