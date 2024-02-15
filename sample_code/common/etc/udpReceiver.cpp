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



#include "udpReceiver.h"


namespace adcm
{
namespace etc
{

udpReceiver::udpReceiver(udpReceiverCallback callback, unsigned short port)
{
    adcm::Log::Info() << "init start";
    struct sockaddr_in serv_adr;
    // set callback
    cb = callback;
    // udp port open
    mSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if(mSocket < 0) {
        ERROR("UDP socket creation error, errono = %d(%s)", errno, strerror(errno));
        return;
    }

    // receive udp
    mPort = port;
    serv_adr.sin_family = AF_INET;
    serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_adr.sin_port = htons(port);
    adcm::Log::Info() << "listen port = " << port;

    if(bind(mSocket, reinterpret_cast<struct sockaddr*>(&serv_adr), sizeof(serv_adr)) < 0) {
        ERROR("bind() error, errono = %d(%s)", errno, strerror(errno));
        return;
    }

    // thread start
    alive = true;
    mThreadUdpReceiver = std::make_shared<std::thread>(thread_udpReceiver, this);
    adcm::Log::Info() << "init complete";
}



udpReceiver::~udpReceiver()
{
    ::close(mSocket);
    mThreadUdpReceiver->join();
}


void udpReceiver::mergePacket(std::shared_ptr<udpBuffer> merge_Buffer, unsigned char* buffer, int size)
{
    merge_Buffer->insert(merge_Buffer->end(), buffer, buffer + size);
}

void udpReceiver::thread_udpReceiver(udpReceiver* instance)
{
    const int buffer_size = 40 * 1024 + 3;
    unsigned char low_buffer[buffer_size];
    struct sockaddr server_addr;
    socklen_t server_addr_size;
    int received_length;
    std::shared_ptr<udpBuffer> merge_buffer = std::make_shared<udpBuffer>();
    unsigned short lastSequence = 0;
    int port = instance->mPort;

    while(instance->alive) {
        received_length = recvfrom(instance->mSocket, reinterpret_cast<char*>(low_buffer), buffer_size, 0, &server_addr, &server_addr_size);
        //INFO("udp low buffer rx[%d/%d] -- %d-  %d/%d", merge_buffer->size(), received_length, low_buffer[0], low_buffer[1], low_buffer[2]);

        if(received_length > 0) {
            if((lastSequence == 0) || (lastSequence == low_buffer[0])) {
                mergePacket(merge_buffer, low_buffer + 3, received_length - 3);

            } else {
                instance->cb(port,merge_buffer);
                merge_buffer = std::make_shared<udpBuffer>();
                mergePacket(merge_buffer, low_buffer + 3, received_length - 3);
            }

            if(low_buffer[1] == low_buffer[2]) {
                instance->cb(port,merge_buffer);
                merge_buffer = std::make_shared<udpBuffer>();
                lastSequence = 0;

            } else {
                lastSequence = low_buffer[0];
            }
        }else{
            ERROR("UDP Sending Error = %d,  errono = %d(%s)", received_length, errno, strerror(errno));
        }
    }
}

}
}
