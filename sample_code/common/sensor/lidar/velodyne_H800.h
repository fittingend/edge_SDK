#ifndef __VELODYNE_LIDAR_H800__
#define __VELODYNE_LIDAR_H800__

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace Velodyne
{

namespace Lidar
{

typedef struct
{
    float x;
    float y;
    float z;
    float intensity;
} xPoint3D;

typedef std::vector<xPoint3D> xPCD;

class H800
{

public:
    typedef struct __attribute__((packed))
    {
        unsigned char HLEN : 4;
        unsigned char VER : 4;
        unsigned char NXHDR;
        unsigned char TLEN : 4;
        unsigned char PTYPE : 4;
        unsigned char MIC;
        unsigned int PSEQ;  // it mustbe 32bit
        unsigned long long TREF;  // it mustbe 64bit
        unsigned char FLEN : 4;
        unsigned char GLEN : 4;
        unsigned char DSET;
        unsigned short ISET;
    } xHeader;

    typedef struct __attribute__((packed))
    {
        unsigned short VDFL : 14;
        unsigned char VDIR : 1;
        unsigned char HDIR : 1;
        unsigned short AZM;
        unsigned short DIST;
        unsigned char RFT;
        unsigned char LCN;
    } xFiringReturn;

    typedef struct __attribute__((packed))
    {
        unsigned short CRC;
        unsigned char AC;
        unsigned char PSEQF;
    } xFooter;

    typedef struct __attribute__((packed))
    {
        xHeader Header;
        xFiringReturn FiringReturn[160];
        xFooter Footer;
    } xPacket;

    typedef void (*xCallback)(std::vector<std::shared_ptr<xPCD>>&);

    H800(xCallback callback, unsigned short port, int timeOut_);
    unsigned short Calculate_CRC_RPF(const unsigned char* buffer, int size, unsigned char AC);

    ~H800();

    static std::shared_ptr<H800> getInstance();
    void printPacket(std::shared_ptr<xPacket> packet);
    void mergePacket(std::shared_ptr<xPacket> packet);
    std::shared_ptr<xPCD> getPCDData(std::shared_ptr<std::vector<xFiringReturn>> pcd_input);

private:
    /// @brief Ports
    unsigned short mPort;

    /// @brief Threads for receiving data with sockets. Size is equal to mPorts.size()
    std::unique_ptr<std::thread> mUdpReceiver;

    /// @brief Sockets for receiving lidar data. Size equal to mPorts.size()
    int mSocket;

    /// @brief bool for thread while loop. Size equal to mPorts.size()
    bool mThreadRunning;

    std::shared_ptr<std::vector<xFiringReturn>> mPcdInput;

    int mLastReceived_PSEQF;

    std::queue<std::shared_ptr<xPacket>> mPacket;

    bool mPacketProcessorRunning;

    std::unique_ptr<std::thread> mPacketProcessor;

    std::unique_ptr<std::mutex> mTimeoutLock;

    std::mutex mCallbackLock;

    std::unique_ptr<std::condition_variable> mCv;

    xCallback mCallback;

    static H800* mInstance;
    void ThreadCallbackWrapper();
    void Thread_UdpReceiver();
    void ThreadPacketProcessor();

    int mTimeOut;

    int index;
};

static int mCount = 0;

/// @brief queue for pcd data. Size is equal to mPorts.size()
static std::vector<std::queue<std::shared_ptr<xPCD>>> mPcds;

/// @brief Thread for callback
static std::unique_ptr<std::thread> mCallbackWrapper;

/// @brief bool for thread wrapper running
static bool mWrapperRunning = false;

static std::condition_variable mCallbackCv;

}  // namespace Lidar

}  // namespace Velodyne
#endif
