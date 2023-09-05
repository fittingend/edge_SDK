#ifndef __CARLA_SENSOR_GNSS__
#define __CARLA_SENSOR_GNSS__

#include <thread>

namespace Carla
{

namespace Sensor
{

class Gnss
{

public:
    struct CarlaGnss
    {
        int32_t lat, lon, alt;
    };

    typedef void (*GnssCallback)(std::unique_ptr<CarlaGnss>);

    int init(GnssCallback callback);
    void start();

    ~Gnss();

    static std::shared_ptr<Gnss> getInstance();

private:
    std::shared_ptr<std::thread> mUdpReceiver;
    const unsigned short mPort = 21200;
    int mUdpSocket;
    GnssCallback mCallback;
    bool mThreadRunning = true;
    static void Thread_UdpReceiver();
};

}  // namespace Sensor

}  // namespace Carla
#endif
