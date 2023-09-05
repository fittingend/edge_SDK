#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <logger.h>

#include "gnss.h"

#include "error.h"
#include <vector>
#include <cmath>

namespace Carla
{
namespace Sensor
{

std::shared_ptr<Gnss> Gnss::getInstance()
{
    static std::shared_ptr<Gnss> instance = std::make_shared<Gnss>();
    return instance;
}

int Gnss::init(GnssCallback callback)
{
    adcm::Log::Info() << "init start";
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

void Gnss::start()
{
    mThreadRunning = true;
    mUdpReceiver = std::make_shared<std::thread>(Thread_UdpReceiver);
    adcm::Log::Info() << "start";
}

void Gnss::Thread_UdpReceiver()
{
    int receivedLength;
    struct sockaddr server_addr;
    socklen_t server_addr_size;
    unsigned char lowBuffer[64 * 1024];
    int index;
    adcm::Log::Info() << "Carla gnss Rx start";

    while (getInstance()->mThreadRunning) {
        receivedLength
            = recvfrom(getInstance()->mUdpSocket, lowBuffer, sizeof(lowBuffer), 0, &server_addr, &server_addr_size);
        index = static_cast<int>(lowBuffer[receivedLength - 1]);
        adcm::Log::Info() << "Gnss receivedLength: " << receivedLength << ", index: " << index;

        for (int i = 0; i < receivedLength; ++i) {
            std::unique_ptr<CarlaGnss> g = std::make_unique<CarlaGnss>();
            std::memcpy(&g->lat, lowBuffer, sizeof(int32_t));
            std::memcpy(&g->lon, lowBuffer + sizeof(int32_t), sizeof(int32_t));
            std::memcpy(&g->alt, lowBuffer + sizeof(int32_t) * 2, sizeof(int32_t));
            adcm::Log::Info() << "lat: " << g->lat / 1000000.0f << ", lon: " << g->lon / 1000000.0f
                              << " alt: " << g->alt / 1000.0f;
            getInstance()->mCallback(std::move(g));
        }
    }

    adcm::Log::Info() << "Gnss Rx stop";
}

Gnss::~Gnss()
{
    mThreadRunning = false;
    mUdpReceiver->join();
    // close udp port
    close(mUdpSocket);
}

}  // namespace Sensor
}  // namespace Carla
