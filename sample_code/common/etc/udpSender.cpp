#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<iostream>
#include <logger.h>
#include <string.h>
#include <thread>


#include "udpSender.h"


namespace adcm
{
namespace etc
{

udpSender::udpSender(std::string DestAddress, unsigned short port)
{
    mSocket = ::socket(AF_INET, SOCK_DGRAM, 0);
    mAddress.sin_family = AF_INET;
    mAddress.sin_port = htons(port);
    mAddress.sin_addr.s_addr = inet_addr(DestAddress.c_str());
}

int udpSender::send2(std::vector<float> result)
{   
    int nBytes;
    int totalBytes = result.size()*4;
    adcm::Log::Info() << "SMSON sending " << totalBytes<<" bytes";
    nBytes = ::sendto(mSocket, result.data(), totalBytes, 0, reinterpret_cast<sockaddr*>(&mAddress), sizeof(mAddress));
    if(nBytes < 0) {
        ERROR("UDP Sending Error = %d,  errono = %d(%s)", nBytes, errno, strerror(errno));
        return -1;
    }
    adcm::Log::Info() << "SMSON nBytes " << nBytes<<" sent";

    return 0;
}

int udpSender::send(unsigned char buffer[], int length)
{
    int nBytes;
    int start_idx = 0;
    int flag;
    unsigned char header[3];
    const unsigned short SEND_SIZE_MAX = 40 * 1024;
    unsigned short tx_len;
    tx_index++;

    if(tx_index == 0) {
        tx_index = 1;
    }

    header[0] = tx_index;
    header[1] = 1;

    if(length % SEND_SIZE_MAX == 0) {
        header[2] = length / SEND_SIZE_MAX;

    } else {
        header[2] = length / SEND_SIZE_MAX + 1;
    }

    while(length > 0) {
        if(length > SEND_SIZE_MAX) {
            tx_len = SEND_SIZE_MAX;
            length -= SEND_SIZE_MAX;

        } else {
            tx_len = length;
            length = 0;
        }
        //INFO("Header %d, %d / %d",header[0],header[1],header[2]);

        nBytes = ::sendto(mSocket, &header, sizeof(header), MSG_MORE, reinterpret_cast<sockaddr*>(&mAddress), sizeof(mAddress));
        nBytes += ::sendto(mSocket, buffer + start_idx, tx_len, 0, reinterpret_cast<sockaddr*>(&mAddress), sizeof(mAddress));
        //std::this_thread::yield();
        //std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if(nBytes < 0) {
            ERROR("UDP Sending Error = %d,  errono = %d(%s)", nBytes, errno, strerror(errno));
            return -1;

        } else {
            //INFO("Header = %d / %d", header[0], header[1]);
        }

        start_idx += tx_len;
        header[1] += 1;
    }

    //INFO("Send %d bytes.", start_idx + tx_len);
    return 0;
}

udpSender::~udpSender()
{
    ::close(mSocket);
}

}
}
