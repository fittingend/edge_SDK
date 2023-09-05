#ifndef __AICM_CAMERA_H__
#define __AICM_CAMERA_H__

#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>



namespace adcm
{
namespace sensor
{
class AICM_Cam
{

public:
#define IMG_BUF_WIDTH 960
#define IMG_BUF_HEIGHT 540
#define IMG_BUF_PIXEL_SIZE 2
#define CameraCount 7

    typedef struct {
        std::vector<unsigned char> buffer;
        unsigned long height;
        unsigned long width;
    } xImgBuffer;


    typedef void (*aicm_camera_callback)(int, std::shared_ptr<xImgBuffer>);


    static std::shared_ptr<AICM_Cam> getInstance();
    void init(aicm_camera_callback cb);
    void start();
    void stop();
    int getFPS(int id);
private:


    typedef struct {
        std::queue<std::shared_ptr<xImgBuffer>> queue;
        std::mutex m;
        std::mutex m_cv;
        std::condition_variable cv;
    } img_queue;


    bool continueExecution = false;
    static img_queue camera_queue[CameraCount];
    static unsigned int cam_count[CameraCount];
    static unsigned int cam_fps[CameraCount];

    std::shared_ptr<std::thread> mCamReceiver[CameraCount];
    std::shared_ptr<std::thread> mCamInfo;
    static aicm_camera_callback callback;

    static void inqueue(int id, std::shared_ptr<xImgBuffer> buffer);
    static std::shared_ptr<xImgBuffer> dequeue(int id);
    static std::shared_ptr<xImgBuffer> flash_queue(int id);
    static void qcarcam_client_event_callback(int input_id, unsigned char* buf_ptr, size_t buf_len);
    static void thread_camProcess_waitfor(int id);
    static void Thred_info();




};
}
}
#endif
