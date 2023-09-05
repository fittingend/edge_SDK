
namespace adcm
{

namespace etc
{

class zlib
{
public:
    static int Compress(unsigned char* dest, unsigned long int* destLen, const unsigned char* source, unsigned long int sourceLen);
    static int unCompress(unsigned char* dest, unsigned long int* destLen, const unsigned char* source, unsigned long int sourceLen);
};

}
}
