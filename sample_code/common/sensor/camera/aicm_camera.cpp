#include <logger.h>
#include <queue>
#include <mutex>


#include <atomic>
#include <condition_variable>
#include <thread>
#include <chrono>



#include <qcarcam_client.h>

#include "aicm_camera.h"

#include "adcm_zlib.h"

#include <pthread.h>

namespace adcm
{
namespace sensor
{

AICM_Cam::img_queue AICM_Cam::camera_queue[CameraCount];
unsigned int AICM_Cam::cam_count[CameraCount] = {0,};
unsigned int AICM_Cam::cam_fps[CameraCount];
AICM_Cam::aicm_camera_callback AICM_Cam::callback;


void AICM_Cam::inqueue(int id, std::shared_ptr<xImgBuffer> buffer)
{
    std::lock_guard<std::mutex> guard(camera_queue[id].m);
    camera_queue[id].queue.push(buffer);
    camera_queue[id].cv.notify_all();
}

std::shared_ptr<AICM_Cam::xImgBuffer> AICM_Cam::dequeue(int id)
{
    std::lock_guard<std::mutex> guard(camera_queue[id].m);
    std::shared_ptr<xImgBuffer> temp;
    temp = camera_queue[id].queue.front();
    camera_queue[id].queue.pop();
    return temp;
}

std::shared_ptr<AICM_Cam::xImgBuffer> AICM_Cam::flash_queue(int id)
{
    std::lock_guard<std::mutex> guard(camera_queue[id].m);
    std::shared_ptr<xImgBuffer> temp;
    temp = camera_queue[id].queue.back();
    camera_queue[id].queue =  std::queue<std::shared_ptr<xImgBuffer>>();
    return temp;
}


void AICM_Cam::qcarcam_client_event_callback(int input_id, unsigned char* buf_ptr, size_t buf_len)
{
    //INFO("id = %d , buflen = %ld", input_id, buf_len);
    if(input_id == 8)
    {
        input_id = 6;
    }
    if(input_id < CameraCount) {
        std::shared_ptr<xImgBuffer> imgbuffer = std::make_shared<xImgBuffer>();
        imgbuffer->width = IMG_BUF_WIDTH;
        imgbuffer->height = IMG_BUF_HEIGHT;
        //imgbuffer->buffer.assign(buf_ptr,buf_ptr+buf_len);
        imgbuffer->buffer.assign(buf_ptr,buf_ptr+(IMG_BUF_WIDTH * IMG_BUF_HEIGHT * IMG_BUF_PIXEL_SIZE));
        inqueue(input_id, imgbuffer);
        //INFO("queue[%d] = %d", input_id, camera_queue[input_id].queue.size());
    }
}



void AICM_Cam::thread_camProcess_waitfor(int id)
{
    std::shared_ptr<xImgBuffer> temp;
    int rx_cnt;
    bool wait_init = true;
    auto instance = getInstance();
    std::cv_status result;

    {
        int policy, s;
        struct sched_param param;
        s = pthread_getschedparam(pthread_self(), &policy, &param);

        if(s != 0) {
            ERROR("pthread_getschedparam");
        }

        INFO("    policy=%s, priority=%d\n",
             (policy == SCHED_FIFO)  ? "SCHED_FIFO" :
             (policy == SCHED_RR)    ? "SCHED_RR" :
             (policy == SCHED_OTHER) ? "SCHED_OTHER" :
             "???", param.sched_priority);
        const int max = sched_get_priority_min(SCHED_RR);
        param.sched_priority = max;
        s = pthread_setschedparam(pthread_self(), policy, &param);

        if(s != 0) {
            ERROR("pthread_getschedparam");
        }

        s = pthread_getschedparam(pthread_self(), &policy, &param);

        if(s != 0) {
            ERROR("pthread_getschedparam");
        }

        INFO("    policy=%s, priority=%d\n",
             (policy == SCHED_FIFO)  ? "SCHED_FIFO" :
             (policy == SCHED_RR)    ? "SCHED_RR" :
             (policy == SCHED_OTHER) ? "SCHED_OTHER" :
             "???", param.sched_priority);
        cpu_set_t  mask;
        CPU_ZERO(&mask);
        CPU_SET(id, &mask);
        pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);
    }

    while(instance->continueExecution && wait_init) {
        std::unique_lock<std::mutex> lk(camera_queue[id].m_cv);

        if(camera_queue[id].cv.wait_for(lk, std::chrono::milliseconds(120)) !=  std::cv_status::timeout) {
            wait_init = false;
        }
    }

    while(instance->continueExecution) {
        {
            std::unique_lock<std::mutex> lk(camera_queue[id].m_cv);
            result = camera_queue[id].cv.wait_for(lk, std::chrono::milliseconds(120));
        }

        if(result ==  std::cv_status::timeout) {
            ERROR("Camera[%d] time out", id);

        } else {
            rx_cnt = camera_queue[id].queue.size();
            temp = flash_queue(id);
            cam_count[id] += rx_cnt;
            callback(id, temp);

            if(rx_cnt > 1) {
                ERROR("Camera image drop[%d] = %d", id, rx_cnt - 1);
            }
        }
    }
}

void AICM_Cam::Thred_info()
{
    auto instance = getInstance();

    while(instance->continueExecution) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        for(int i = 0 ; i < CameraCount; i++) {
            cam_fps[i] = cam_count[i];
            cam_count[i] = 0;
        }
    }
}

std::shared_ptr<AICM_Cam> AICM_Cam::getInstance()
{
    static std::shared_ptr<AICM_Cam> instance = std::make_shared<AICM_Cam>();
    return instance;
}
void AICM_Cam::init(aicm_camera_callback cb)
{
    callback = cb;
}
void AICM_Cam::start()
{
    continueExecution = true;
    const char* szInputConfPath = "./etc/camera_settings.xml";
    qcarcam_client_start_preview(szInputConfPath, qcarcam_client_event_callback);
    mCamInfo = std::make_shared<std::thread>(Thred_info);

    for(int i = 0; i < CameraCount; i++) {
        mCamReceiver[i] = std::make_shared<std::thread>(thread_camProcess_waitfor, i);
    }
}
void AICM_Cam::stop()
{
    continueExecution = false;
    mCamInfo->join();

    for(int i = 0; i < CameraCount; i++) {
        mCamReceiver[i]->join();
    }

    qcarcam_client_stop_preview();
}

int AICM_Cam::getFPS(int id)
{
    return cam_fps[id];
}

}
}
