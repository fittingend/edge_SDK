#ifndef __ADCM_ETC_UDP_SENDER_H__
#define __ADCM_ETC_UDP_SENDER_H__

#include <netinet/in.h>
#include <vector>

namespace adcm
{
namespace etc
{
class udpSender
{
public:

    udpSender(std::string DestAddress, unsigned short port);
    ~udpSender();

    int send(unsigned char buffer[], int length);
    int send2(std::vector<float> result);

private:
    struct sockaddr_in mAddress;
    int mSocket;
    unsigned char tx_index;

};
}
}
#endif
