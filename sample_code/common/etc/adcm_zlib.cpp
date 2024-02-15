#include <zlib.h>
#include "adcm_zlib.h"
#include <stdio.h>
#include <string.h>

namespace adcm
{

namespace etc
{int zlib::Compress(unsigned char* dest, unsigned long int* destLen, const unsigned char* source, unsigned long int sourceLen)
{
    /*
    0 ~ 9
    Z_NO_COMPRESSION - 압축 안함
    Z_BEST_SPEED - 빠른 압축 시간을 위한 경우
    Z_BEST_COMPRESSION - 최고의 압축률을 원할 경우
    Z_DEFAULT_COMPRESSION - 압축시간과 압축률사이의 중간값
    */
    return compress2(dest, destLen, source, sourceLen, 2);
}


int zlib::unCompress(unsigned char* dest, unsigned long int* destLen, const unsigned char* source, unsigned long int sourceLen)
{
    return uncompress(dest, destLen, source, sourceLen);
}
}
}
