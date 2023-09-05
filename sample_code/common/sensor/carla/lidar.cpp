#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <logger.h>

#include "carla/lidar.h"

#include "error.h"
#include <vector>
#include <cmath>

namespace Carla
{
namespace Sensor
{

std::shared_ptr<Lidar> Lidar::getInstance()
{
    static std::shared_ptr<Lidar> instance = std::make_shared<Lidar>();
    return instance;
}

int Lidar::init(xCallback callback)
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
    adcm::Log::Info() << "SMSON listen port = " << mPort;

    if (bind(mUdpSocket, reinterpret_cast<struct sockaddr*>(&serv_adr), sizeof(serv_adr)) < 0) {
        adcm::Log::Error() << "bind() error (" << errno << ")";
        return -2;
    }

    adcm::Log::Info() << "init complete";
    return 0;
}

void Lidar::start()
{
    mThreadRunning = true;
    mUdpReceiver = std::make_shared<std::thread>(Thread_UdpReceiver);
    adcm::Log::Info() << "start";
}
void Lidar::getPCDData(std::shared_ptr<Velodyne::Lidar::xPCD> outBuffer, xPoint3D* inBuffer, int count)
{
    // INFO("SMSON count = %ld", count);
    for (int i = 0; i < count; i++) {
        Velodyne::Lidar::xPoint3D point;
        point.x = inBuffer[i].x;
        point.y = inBuffer[i].y;
        point.z = inBuffer[i].z;
        point.intensity = 0.1;
        outBuffer->push_back(point);
    }
    // INFO("buffer = %ld", outBuffer->size());
}

void Lidar::mergePacket(unsigned char buffer[], int length, int packetIndex)
{
    static int lastReceived_PSEQF = -1;
    static std::shared_ptr<Velodyne::Lidar::xPCD> pointCloude = std::make_shared<Velodyne::Lidar::xPCD>();
    int count = (length-4) / (4 * 3);
    INFO("udp rx = %d / %d / %d", length, packetIndex, count);

    getPCDData(pointCloude, reinterpret_cast<xPoint3D*>(buffer), count);
    if (packetIndex < 0) {
        if (getInstance()->inferenceReady){
            getInstance()->inferenceReady = false;
            getInstance()->mCallback(pointCloude);
        }
        pointCloude->clear();
    }
    // if (lastReceived_PSEQF > packetIndex) {
    //     getInstance()->mCallback(pointCloude);
    //     pointCloude->clear();
    // }

    // getPCDData(pointCloude, reinterpret_cast<xPoint3D*>(buffer), count);

    // if (count < 4000) {
    //     getInstance()->mCallback(pointCloude);
    //     pointCloude->clear();
    //     lastReceived_PSEQF = -1;

    // } else {
    //     lastReceived_PSEQF = packetIndex;
    // }
}

void Lidar::Thread_UdpReceiver()
{
    int receivedLength;
    struct sockaddr server_addr;
    socklen_t server_addr_size;
    unsigned char lowBuffer[64 * 1024];
    float index;
    adcm::Log::Info() << "Carla lidar Rx start";

    while (getInstance()->mThreadRunning) {
        receivedLength = recvfrom(getInstance()->mUdpSocket, lowBuffer, sizeof(lowBuffer), 0, &server_addr, &server_addr_size);
        memcpy(&index, lowBuffer+receivedLength-4, sizeof(float));
        // index = static_cast<float>(lowBuffer[receivedLength - 4]);
        {
            mergePacket(lowBuffer, receivedLength, (int)index);
#if 0
            INFO("Size -> %d, %d, %d, %d", sizeof(xPacket), sizeof(xHeader), sizeof(xFiringReturn), sizeof(xFooter));
            INFO("Header -> %X %X %02X %04X", rx_packet->Header.GLEN, rx_packet->Header.FLEN, rx_packet->Header.DSET, rx_packet->Header.ISET);
            INFO("Header -> %X%X %02X %X%X", rx_packet->Header.VER, rx_packet->Header.HLEN, rx_packet->Header.NXHDR, rx_packet->Header.PTYPE, rx_packet->Header.TLEN);
            INFO("FiringReturn -> %X %X %04X %04X %04X %02X %02X",
                 rx_packet->FiringReturn[0].HDIR,
                 rx_packet->FiringReturn[0].VDIR,
                 rx_packet->FiringReturn[0].VDFL,
                 rx_packet->FiringReturn[0].AZM,
                 rx_packet->FiringReturn[0].DIST,
                 rx_packet->FiringReturn[0].RFT,
                 rx_packet->FiringReturn[0].LCN
                );
            INFO("Footer -> %04X %02X %02X", rx_packet->Footer.CRC, rx_packet->Footer.AC, rx_packet->Footer.PSEQF);
#endif
        }
    }

    adcm::Log::Info() << "Velodyne H800 Rx stop";
}

Lidar::~Lidar()
{
    mThreadRunning = false;
    mUdpReceiver->join();
    // close udp port
    close(mUdpSocket);
}

}  // namespace Sensor
}  // namespace Carla
