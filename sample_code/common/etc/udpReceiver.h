#ifndef __ADCM_ETC_UDP_RECEIVER_H__
#define __ADCM_ETC_UDP_RECEIVER_H__

#include <netinet/in.h>
#include <thread>
#include <vector>


namespace adcm
{
namespace etc
{
class udpReceiver
{
public:

    typedef std::vector<unsigned char> udpBuffer;

    typedef void(* udpReceiverCallback)(int port, std::shared_ptr<udpBuffer> buffer);



    udpReceiver(udpReceiverCallback callback, unsigned short port);
    ~udpReceiver();


private:

    int mSocket;
    int mPort;
    udpReceiverCallback cb;
    bool alive;
    std::shared_ptr<std::thread> mThreadUdpReceiver;
    static void thread_udpReceiver(udpReceiver* instance);
    static void mergePacket(std::shared_ptr<udpBuffer> merge_Buffer, unsigned char* buffer, int size);

};
}
}

#endif
