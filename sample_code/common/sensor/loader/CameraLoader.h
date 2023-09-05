#ifndef __CAMERA_LOADER_H__
#define __CAMERA_LOADER_H__

#include "loader/FileLoader.h"
#include "camera/aicm_camera.h"
namespace Loader
{
namespace FileLoader
{


typedef void (*xCameraCallback)(int, std::shared_ptr<adcm::sensor::AICM_Cam::xImgBuffer>);

class CameraLoader : public BaseFileLoader
{
public:
    CameraLoader();
    static std::shared_ptr<CameraLoader> getInstance();
    void init(int id, std::string filePath, xCameraCallback cb);
    void start();

private:
    int id;
    xCameraCallback cb_to_uppder;
    std::string mDataPath;
    static void callback(std::string path);
};
}  // namespace FileLoader
}  // namespace Loader
#endif
