#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <logger.h>

#include "obstacles.h"

#include "error.h"
#include <vector>
#include <cmath>

namespace Carla
{
namespace Sensor
{

std::shared_ptr<Obstacles> Obstacles::getInstance()
{
    static std::shared_ptr<Obstacles> instance = std::make_shared<Obstacles>();
    return instance;
}

int Obstacles::init(ObstaclesCallback callback)
{
    adcm::Log::Info() << "Obstacles init start";
    struct sockaddr_in serv_adr;

    // set callback
    mCallback = callback;

    // udp port open
    mUdpSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (mUdpSocket < 0) {
        adcm::Log::Error() << "UDP socket creation error(" << errno << ")";
        return -1;
    }

    // receive udp
    serv_adr.sin_family = AF_INET;
    serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_adr.sin_port = htons(mPort);
    adcm::Log::Info() << "listen port = " << mPort;

    if (bind(mUdpSocket, reinterpret_cast<struct sockaddr*>(&serv_adr), sizeof(serv_adr)) < 0) {
        adcm::Log::Error() << "bind() error (" << errno << ")";
        return -2;
    }

    adcm::Log::Info() << "init complete";
    return 0;
}

void Obstacles::start()
{
    mThreadRunning = true;
    mUdpReceiver = std::make_shared<std::thread>(Thread_UdpReceiver);
    adcm::Log::Info() << "start";
}

void Obstacles::Thread_UdpReceiver()
{
    int receivedLength;
    struct sockaddr server_addr;
    socklen_t server_addr_size;
    unsigned char lowBuffer[64 * 1024];
    int index;
    adcm::Log::Info() << "Carla Obstacles Rx start";

    while (getInstance()->mThreadRunning) {
        receivedLength
            = recvfrom(getInstance()->mUdpSocket, lowBuffer, sizeof(lowBuffer), 0, &server_addr, &server_addr_size);
        index = static_cast<int>(lowBuffer[receivedLength - 1]);
        adcm::Log::Info() << "Obstacles receivedLength: " << receivedLength << ", index: " << index;

        for (int i = 0; i < receivedLength; ++i) {
            std::unique_ptr<CarlaObstacles> obstacle = std::make_unique<CarlaObstacles>();

            std::memcpy(&obstacle->geolocation_latitude, lowBuffer, sizeof(float));
            std::memcpy(&obstacle->geolocation_longitude, lowBuffer + sizeof(float), sizeof(float));
            std::memcpy(&obstacle->geolocation_altitude, lowBuffer + sizeof(float) * 2, sizeof(float));
            std::memcpy(&obstacle->rotation_pitch, lowBuffer + sizeof(float) * 3, sizeof(float));
            std::memcpy(&obstacle->rotation_roll, lowBuffer + sizeof(float) * 4, sizeof(float));
            std::memcpy(&obstacle->rotation_yaw, lowBuffer + sizeof(float) * 5, sizeof(float));
            std::memcpy(&obstacle->x_axis_size, lowBuffer + sizeof(float) * 6, sizeof(float));
            std::memcpy(&obstacle->y_axis_size, lowBuffer + sizeof(float) * 7, sizeof(float));
            std::memcpy(&obstacle->z_axis_size, lowBuffer + sizeof(float) * 8, sizeof(float));

            getInstance()->mCallback(std::move(obstacle));
        }
    }

    adcm::Log::Info() << "Obstacles Rx stop";
}

Obstacles::~Obstacles()
{
    mThreadRunning = false;
    mUdpReceiver->join();
    // close udp port
    close(mUdpSocket);
}

}  // namespace Sensor
}  // namespace Carla
