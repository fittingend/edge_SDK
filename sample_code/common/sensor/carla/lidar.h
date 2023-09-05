#ifndef __CARLA_SENSOR_LIDAR__
#define __CARLA_SENSOR_LIDAR__

#include <thread>
#include <vector>

#include "lidar/velodyne_H800.h"

namespace Carla
{

namespace Sensor
{

typedef struct
{
    float x;
    float y;
    float z;
} xPoint3D;

class Lidar
{

public:
    typedef void (*xCallback)(std::shared_ptr<Velodyne::Lidar::xPCD>);

    int init(xCallback callback);
    void start();

    ~Lidar();

    static std::shared_ptr<Lidar> getInstance();
    bool inferenceReady = true;

    static void mergePacket(unsigned char buffer[], int length, int packetIndex);
    static void getPCDData(std::shared_ptr<Velodyne::Lidar::xPCD> outBuffer, xPoint3D* inBuffer, int count);

private:
    std::shared_ptr<std::thread> mUdpReceiver;
    const unsigned short mPort = 21000;
    int mUdpSocket;
    xCallback mCallback;
    bool mThreadRunning = true;
    static Lidar* mInstance;
    static void Thread_UdpReceiver();
};

}  // namespace Sensor

}  // namespace Carla
#endif
