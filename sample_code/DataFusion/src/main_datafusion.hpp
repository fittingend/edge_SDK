#define MAP_GENERATION
#define n 1000 
#define m 2000
#ifdef MAP_GENERATION
struct map_data_type
{
    bool obstacle; // 해당 cell 에 obstacle 존재 여부
    char obstacle_fused_index[10]; 
    float obstacle_fused_cuboid_x;
    float obstacle_fused_cuboid_y;
    float obstacle_fused_cuboid_z;
    float obstacle_fused_heading_angle;
    float obstacle_fused_Position_x;
    float obstacle_fused_Position_y;
    float obstacle_fused_Position_z;
    float obstacle_fused_velocity_x;
    float obstacle_fused_velocity_y;
    float obstacle_fused_velocity_z;
    float road_z;
};


std::vector <std::vector<map_data_type>> map_2d (n, std::vector<map_data_type>(m));

//map initialization

bool map_2d_init(std::vector < std::vector<map_data_type>> map_2d) {
    for (int i = 0; i < map_2d.size();i++)
    {
        for (int j = 0; j < map_2d[i].size(); j++)
            map_2d[i][j] = { 0 };
    }
    return true;
}
#endif