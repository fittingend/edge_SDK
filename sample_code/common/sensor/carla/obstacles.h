#ifndef __CARLA_SENSOR_Obstacles__
#define __CARLA_SENSOR_Obstacles__

#include <thread>

namespace Carla
{

namespace Sensor
{

class Obstacles
{

public:
    struct CarlaObstacles
    {
        float geolocation_latitude, geolocation_longitude, geolocation_altitude, rotation_pitch, rotation_roll, rotation_yaw, x_axis_size, y_axis_size, z_axis_size;
    };

    typedef void (*ObstaclesCallback)(std::unique_ptr<CarlaObstacles>);

    int init(ObstaclesCallback callback);
    void start();

    ~Obstacles();

    static std::shared_ptr<Obstacles> getInstance();

private:
    std::shared_ptr<std::thread> mUdpReceiver;
    const unsigned short mPort = 21300;
    int mUdpSocket;
    ObstaclesCallback mCallback;
    bool mThreadRunning;
    static void Thread_UdpReceiver();
};

}  // namespace Sensor

}  // namespace Carla
#endif
