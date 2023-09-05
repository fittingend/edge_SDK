#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <logger.h>

#include "lidar/velodyne_H800.h"

#include "error.h"
#include <vector>
#include <chrono>
#include <cmath>

namespace Velodyne
{
namespace Lidar
{

double dgree2radian(double degree);
void changeByteOrder16(unsigned short* value);
void changeByteOrder32(unsigned int* value);
void changeByteOrder64(unsigned long long* value);
void changeByteOrder(std::shared_ptr<Velodyne::Lidar::H800::xPacket> packet);

constexpr double const_pi()
{
    return std::atan(1) * 4;
}

double dgree2radian(double degree)
{
    return degree * M_PI / 180;
}

static const unsigned short CRC_CCITT_TABLE[256] = {0x0000,
    0x1021,
    0x2042,
    0x3063,
    0x4084,
    0x50A5,
    0x60C6,
    0x70E7,
    0x8108,
    0x9129,
    0xA14A,
    0xB16B,
    0xC18C,
    0xD1AD,
    0xE1CE,
    0xF1EF,
    0x1231,
    0x0210,
    0x3273,
    0x2252,
    0x52B5,
    0x4294,
    0x72F7,
    0x62D6,
    0x9339,
    0x8318,
    0xB37B,
    0xA35A,
    0xD3BD,
    0xC39C,
    0xF3FF,
    0xE3DE,
    0x2462,
    0x3443,
    0x0420,
    0x1401,
    0x64E6,
    0x74C7,
    0x44A4,
    0x5485,
    0xA56A,
    0xB54B,
    0x8528,
    0x9509,
    0xE5EE,
    0xF5CF,
    0xC5AC,
    0xD58D,
    0x3653,
    0x2672,
    0x1611,
    0x0630,
    0x76D7,
    0x66F6,
    0x5695,
    0x46B4,
    0xB75B,
    0xA77A,
    0x9719,
    0x8738,
    0xF7DF,
    0xE7FE,
    0xD79D,
    0xC7BC,
    0x48C4,
    0x58E5,
    0x6886,
    0x78A7,
    0x0840,
    0x1861,
    0x2802,
    0x3823,
    0xC9CC,
    0xD9ED,
    0xE98E,
    0xF9AF,
    0x8948,
    0x9969,
    0xA90A,
    0xB92B,
    0x5AF5,
    0x4AD4,
    0x7AB7,
    0x6A96,
    0x1A71,
    0x0A50,
    0x3A33,
    0x2A12,
    0xDBFD,
    0xCBDC,
    0xFBBF,
    0xEB9E,
    0x9B79,
    0x8B58,
    0xBB3B,
    0xAB1A,
    0x6CA6,
    0x7C87,
    0x4CE4,
    0x5CC5,
    0x2C22,
    0x3C03,
    0x0C60,
    0x1C41,
    0xEDAE,
    0xFD8F,
    0xCDEC,
    0xDDCD,
    0xAD2A,
    0xBD0B,
    0x8D68,
    0x9D49,
    0x7E97,
    0x6EB6,
    0x5ED5,
    0x4EF4,
    0x3E13,
    0x2E32,
    0x1E51,
    0x0E70,
    0xFF9F,
    0xEFBE,
    0xDFDD,
    0xCFFC,
    0xBF1B,
    0xAF3A,
    0x9F59,
    0x8F78,
    0x9188,
    0x81A9,
    0xB1CA,
    0xA1EB,
    0xD10C,
    0xC12D,
    0xF14E,
    0xE16F,
    0x1080,
    0x00A1,
    0x30C2,
    0x20E3,
    0x5004,
    0x4025,
    0x7046,
    0x6067,
    0x83B9,
    0x9398,
    0xA3FB,
    0xB3DA,
    0xC33D,
    0xD31C,
    0xE37F,
    0xF35E,
    0x02B1,
    0x1290,
    0x22F3,
    0x32D2,
    0x4235,
    0x5214,
    0x6277,
    0x7256,
    0xB5EA,
    0xA5CB,
    0x95A8,
    0x8589,
    0xF56E,
    0xE54F,
    0xD52C,
    0xC50D,
    0x34E2,
    0x24C3,
    0x14A0,
    0x0481,
    0x7466,
    0x6447,
    0x5424,
    0x4405,
    0xA7DB,
    0xB7FA,
    0x8799,
    0x97B8,
    0xE75F,
    0xF77E,
    0xC71D,
    0xD73C,
    0x26D3,
    0x36F2,
    0x0691,
    0x16B0,
    0x6657,
    0x7676,
    0x4615,
    0x5634,
    0xD94C,
    0xC96D,
    0xF90E,
    0xE92F,
    0x99C8,
    0x89E9,
    0xB98A,
    0xA9AB,
    0x5844,
    0x4865,
    0x7806,
    0x6827,
    0x18C0,
    0x08E1,
    0x3882,
    0x28A3,
    0xCB7D,
    0xDB5C,
    0xEB3F,
    0xFB1E,
    0x8BF9,
    0x9BD8,
    0xABBB,
    0xBB9A,
    0x4A75,
    0x5A54,
    0x6A37,
    0x7A16,
    0x0AF1,
    0x1AD0,
    0x2AB3,
    0x3A92,
    0xFD2E,
    0xED0F,
    0xDD6C,
    0xCD4D,
    0xBDAA,
    0xAD8B,
    0x9DE8,
    0x8DC9,
    0x7C26,
    0x6C07,
    0x5C64,
    0x4C45,
    0x3CA2,
    0x2C83,
    0x1CE0,
    0x0CC1,
    0xEF1F,
    0xFF3E,
    0xCF5D,
    0xDF7C,
    0xAF9B,
    0xBFBA,
    0x8FD9,
    0x9FF8,
    0x6E17,
    0x7E36,
    0x4E55,
    0x5E74,
    0x2E93,
    0x3EB2,
    0x0ED1,
    0x1EF0};

unsigned short H800::Calculate_CRC_RPF(const unsigned char* buffer, int size, unsigned char AC)
{
    unsigned short tmp;
    unsigned short crc = 0xffff;
    tmp = static_cast<unsigned short>(crc >> 8) ^ static_cast<unsigned short>(AC);
    crc = static_cast<unsigned short>(crc << 8) ^ static_cast<unsigned short>(CRC_CCITT_TABLE[tmp]);

    for (int i = 0; i < size; i++) {
        tmp = static_cast<unsigned short>(crc >> 8) ^ buffer[i];
        crc = static_cast<unsigned short>(crc << 8) ^ CRC_CCITT_TABLE[tmp];
    }

    tmp = static_cast<unsigned short>(crc >> 8) ^ static_cast<unsigned char>(0x40);
    crc = static_cast<unsigned short>(crc << 8) ^ CRC_CCITT_TABLE[tmp];
    tmp = static_cast<unsigned short>(crc >> 8) ^ static_cast<unsigned char>(0xF8);
    crc = static_cast<unsigned short>(crc << 8) ^ CRC_CCITT_TABLE[tmp];
    return crc;
}

void changeByteOrder16(unsigned short* value)
{
    unsigned short temp;
    temp = *value;
    *value = ((temp >> 8) & 0x00FF);
    *value |= ((temp << 8) & 0xFF00);
}

void changeByteOrder32(unsigned int* value)
{
    unsigned int temp32;
    temp32 = *value;
    *value = ((temp32 >> 24) & 0x000000FF);
    *value |= ((temp32 >> 8) & 0x0000FF00);
    *value |= ((temp32 << 8) & 0x00FF0000);
    *value |= ((temp32 << 24) & 0xFF000000);
}

void changeByteOrder64(unsigned long long* value)
{
    unsigned long long temp;
    temp = *value;
    *value = ((temp >> 56) & 0x00000000000000FF);
    *value |= ((temp >> 40) & 0x000000000000FF00);
    *value |= ((temp >> 24) & 0x0000000000FF0000);
    *value |= ((temp >> 8) & 0x00000000FF000000);
    *value |= ((temp << 8) & 0x000000FF00000000);
    *value |= ((temp << 24) & 0x0000FF0000000000);
    *value |= ((temp << 40) & 0x00FF000000000000);
    *value |= ((temp << 56) & 0xFF00000000000000);
}

void changeByteOrder(std::shared_ptr<Velodyne::Lidar::H800::xPacket> packet)
{
    // changeByteOrder32(&packet->Header.PSEQ);
    // changeByteOrder64(&packet->Header.TREF);
    changeByteOrder16(&packet->Header.ISET);

    for (int i = 0; i < 160; i++) {
        unsigned char buffer[2];
        memcpy(buffer, &packet->FiringReturn[i], 2);
        packet->FiringReturn[i].HDIR = (buffer[0] & 0x80) ? 1 : 0;
        packet->FiringReturn[i].VDIR = (buffer[0] & 0x40) ? 1 : 0;
        packet->FiringReturn[i].VDFL
            = static_cast<unsigned short>((buffer[0] & 0x3F) << 8) + static_cast<unsigned short>(buffer[1]);
        changeByteOrder16(&packet->FiringReturn[i].AZM);
        changeByteOrder16(&packet->FiringReturn[i].DIST);
    }

    // changeByteOrder16(&rx_packet->Footer.CRC);
}

H800::H800(xCallback callback, unsigned short port, int timeOut_)
{
    index = mCount++;

    if (mPcds.empty()) {
        for (int i = 0; i < 3; ++i) {
            std::queue<std::shared_ptr<xPCD>> tmp;
            mPcds.push_back(tmp);
        }
    }

    adcm::Log::Info() << "init start";

    // Init data
    mPort = port;

    mLastReceived_PSEQF = -1;
    mPcdInput = std::make_shared<std::vector<xFiringReturn>>();
    mTimeOut = timeOut_;

    // set callback
    mCallback = callback;

    struct sockaddr_in serv_adr;

    // udp port open
    mSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (mSocket < 0) {
        adcm::Log::Error() << "UDP socket creation error(" << errno << ")";
    }

    // receive udp
    serv_adr.sin_family = AF_INET;
    serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_adr.sin_port = htons(mPort);
    adcm::Log::Info() << "listen port = " << mPort;
    if (bind(mSocket, reinterpret_cast<struct sockaddr*>(&serv_adr), sizeof(serv_adr)) < 0) {
        adcm::Log::Error() << "bind() error (" << errno << ")";
    }

    mTimeoutLock = std::make_unique<std::mutex>();
    mCv = std::make_unique<std::condition_variable>();
    // thread start
    mThreadRunning = true;
    mUdpReceiver = std::make_unique<std::thread>(&H800::Thread_UdpReceiver, this);
    mPacketProcessorRunning = true;
    mPacketProcessor = std::make_unique<std::thread>(&H800::ThreadPacketProcessor, this);

    adcm::Log::Info() << "init complete";
}

void H800::printPacket(std::shared_ptr<xPacket> packet)
{
    // header info
    INFO("Header -> %X %X %X %X %X %02X %X %X %02X %04X",
        packet->Header.VER,
        packet->Header.HLEN,
        packet->Header.NXHDR,
        packet->Header.PTYPE,
        packet->Header.TLEN,
        packet->Header.MIC,
        packet->Header.GLEN,
        packet->Header.FLEN,
        packet->Header.DSET,
        packet->Header.ISET);
    INFO("Sequence : %d / %d, TREF : %ld", packet->Footer.PSEQF, packet->Header.PSEQ, packet->Header.TREF);

    for (int i = 0; i < 160; i++) {
        INFO("[%3d] %8d : %d %5d %d %5d %5d %3d %3d",
            i,
            packet->Header.PSEQ,
            packet->FiringReturn[i].VDIR,
            packet->FiringReturn[i].VDFL,
            packet->FiringReturn[i].HDIR,
            packet->FiringReturn[i].AZM,
            packet->FiringReturn[i].DIST,
            packet->FiringReturn[i].RFT,
            packet->FiringReturn[i].LCN);
    }

#if 0
    adcm::Log::Verbose() << "Callback called";
    VERBOSE("structure Size -> %ld, %ld, %ld, %ld",
            sizeof(Velodyne::Lidar::H800::xPacket),
            sizeof(Velodyne::Lidar::H800::xHeader),
            sizeof(Velodyne::Lidar::H800::xFiringReturn),
            sizeof(Velodyne::Lidar::H800::xFooter)
           );
    VERBOSE("Header -> %X %X %02X %04X %X %X %02X %X %X",
            packet->Header.GLEN,
            packet->Header.FLEN,
            packet->Header.DSET,
            packet->Header.ISET,
            packet->Header.VER,
            packet->Header.HLEN,
            packet->Header.NXHDR,
            packet->Header.PTYPE,
            packet->Header.TLEN
           );
    VERBOSE("FiringReturn[0] -> %X %X %04X %04X %04X %02X %02X",
            packet->FiringReturn[0].HDIR,
            packet->FiringReturn[0].VDIR,
            packet->FiringReturn[0].VDFL,
            packet->FiringReturn[0].AZM,
            packet->FiringReturn[0].DIST,
            packet->FiringReturn[0].RFT,
            packet->FiringReturn[0].LCN
           );
    VERBOSE("Footer -> %04X %02X %02X",
            packet->Footer.CRC,
            packet->Footer.AC,
            packet->Footer.PSEQF
           );
#endif
}

std::shared_ptr<xPCD> H800::getPCDData(std::shared_ptr<std::vector<xFiringReturn>> pcd_input)
{
    std::shared_ptr<xPCD> result = std::make_shared<xPCD>();
    double d_azi;
    double d_ver;
    double d_length;
    double dx, dy, dz;
    // init offset
    result->reserve(pcd_input->size());

    for (auto data : (*pcd_input)) {
        xPoint3D temp;

        if (data.RFT == 0) {
            continue;
        }

        d_azi = 0.0;
        d_ver = 0.0;
        dx = 0.0;
        dy = 0.0;
        dz = 0.0;
        // INFO("AZM/VDFL = %lf(%d) / %lf(%d) / %lfm / %d", data.HDIR, static_cast<double>(data.AZM) / 100.0,
        // data.VDIR,static_cast<double>(data.VDFL) / 100.0, static_cast<double>(data.DIST) / 100.0,data.RFT );
        d_azi
            += (data.HDIR == 0) ? static_cast<double>(data.AZM) / 100.0 : (static_cast<double>(data.AZM) / 100.0) * -1;
        d_ver += (data.VDIR == 0) ? static_cast<double>(data.VDFL) / 100.0
                                  : (static_cast<double>(data.VDFL) / 100.0) * -1;
        d_length = static_cast<double>(data.DIST) / 100.0;
        double cos_v = std::cos(dgree2radian(d_ver));
        double sin_v = std::sin(dgree2radian(d_ver));
        double cos_a = std::cos(dgree2radian(d_azi));
        double sin_a = std::sin(dgree2radian(d_azi));
        dx += cos_v * sin_a * d_length;
        dy += cos_v * cos_a * d_length;
        dz += sin_v * d_length;
        temp.x = static_cast<float>(dx);
        temp.y = static_cast<float>(dy);
        temp.z = static_cast<float>(dz);
        temp.intensity = static_cast<float>(data.RFT);
#if 0
        INFO("AZM/VDFL = %0.2lf(%d) / %0.2lf(%d) / %0.2lfm / %d >>> %0.2lf %0.2lf %0.2lf %0.2lf >> %0.2lf %0.2lf %0.2lf >> %0.2f %0.2f %0.2f",
             data.HDIR,
             static_cast<double>(data.AZM) / 100.0,
             data.VDIR,
             static_cast<double>(data.VDFL) / 100.0,
             static_cast<double>(data.DIST) / 100.0,
             data.RFT,
             cos_v,
             sin_v,
             cos_a,
             sin_a,
             dx,
             dy,
             dz,
             temp.x,
             temp.y,
             temp.z
            );
#endif
        result->push_back(temp);
    }

    // INFO("in = %ld / out = %ld", pcd_input->size(), result->size());
    return result;
}

void H800::mergePacket(std::shared_ptr<xPacket> packet)
{
    if (mLastReceived_PSEQF > packet->Footer.PSEQF) {
        std::shared_ptr<xPCD> pcd = getPCDData(mPcdInput);

        mPcdInput->clear();
        mPcds[index].push(pcd);

        mCallbackCv.notify_all();
    }

    mLastReceived_PSEQF = packet->Footer.PSEQF;

    for (int i = 0; i < 160; i++) {
        mPcdInput->push_back(packet->FiringReturn[i]);
    }
}

void H800::Thread_UdpReceiver()
{
    if (!mWrapperRunning) {
        mWrapperRunning = true;
        mCallbackWrapper = std::make_unique<std::thread>(&H800::ThreadCallbackWrapper, this);
    }
    // int receivedLength;
    struct sockaddr server_addr;
    socklen_t server_addr_size;
    std::shared_ptr<xPacket> rx_packet;

    adcm::Log::Info() << index << "Velodyne H800 Rx start";
    while (mThreadRunning) {
        rx_packet = std::make_shared<xPacket>();
        recvfrom(mSocket, rx_packet.get(), sizeof(xPacket), 0, &server_addr, &server_addr_size);
        mPacket.push(rx_packet);

        // Notify after receiving
        mCv->notify_all();
    }
    adcm::Log::Info() << "Velodyne H800 Rx stop";
}

void H800::ThreadPacketProcessor()
{
    while (mPacketProcessorRunning) {
        std::unique_lock<std::mutex> lk(*mTimeoutLock);

        // Received in time
        if (mCv->wait_until(lk, std::chrono::system_clock::now() + std::chrono::milliseconds(mTimeOut))
            == std::cv_status::no_timeout) {
            std::shared_ptr<xPacket> rx_packet = mPacket.front();
            mPacket.pop();

            unsigned short calCrc;
            changeByteOrder16(&rx_packet->Footer.CRC);
            calCrc = H800::Calculate_CRC_RPF(reinterpret_cast<unsigned char*>(rx_packet.get()),
                sizeof(xPacket) - sizeof(xFooter),
                rx_packet->Footer.AC);
            changeByteOrder(rx_packet);
            mergePacket(rx_packet);
        }
        // Timeout
        else {
            adcm::Log::Error() << "Lidar" << index << "packet timeout!";
        }
    }
}

void H800::ThreadCallbackWrapper()
{
    while (mWrapperRunning) {
        std::unique_lock<std::mutex> uLock(mCallbackLock);

        // Wait until every queue has at least 1
        mCallbackCv.wait(uLock, [&] {
            bool atLeastOneEmpty = false;
            for (int i = 0; i < mCount; ++i) {
                if (mPcds[i].empty()) {
                    atLeastOneEmpty = true;
                    break;
                }
            }

            return !atLeastOneEmpty;
        });

        adcm::Log::Info() << "Received all";

        std::vector<std::shared_ptr<xPCD>> pcds;

        for (int i = 0; i < mCount; ++i) {
            pcds.push_back(mPcds[i].front());
            mPcds[i].pop();
        }

        mCallback(pcds);
    }
}

H800::~H800()
{
    mThreadRunning = false;
    mUdpReceiver->join();

    mPacketProcessorRunning = false;
    mPacketProcessor->join();
    // close udp port
    close(mSocket);

    mWrapperRunning = false;
    mCallbackWrapper->join();
}

}  // namespace Lidar
}  // namespace Velodyne
