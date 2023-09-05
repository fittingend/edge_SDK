

#include <logger.h>
#include <dirent.h>
#include <sys/stat.h>
#include <cstring>

#include "loader/LidarLoader.h"

namespace Loader
{
namespace FileLoader
{

std::shared_ptr<LidarLoader> LidarLoader::getInstance()
{
    static std::shared_ptr<LidarLoader> instance = std::make_shared<LidarLoader>();
    return instance;
}

LidarLoader::LidarLoader()
    : BaseFileLoader()
{ }

void LidarLoader::init(std::string filePath, xCallback_PCD cb)
{
    cb_to_uppder = cb;
    mDataPath = filePath;
}

void LidarLoader::start()
{
    readStart(mDataPath, callback);
}

void LidarLoader::callback(std::string path)
{
    FILE* filehandle;
    int readCount;
    //float temp;
    Velodyne::Lidar::xPoint3D data;
    std::shared_ptr<Velodyne::Lidar::xPCD> pcd = std::make_shared<Velodyne::Lidar::xPCD>();
    // get file list in File system
    // INFO("file : %s", path.c_str());
    filehandle = fopen(path.c_str(), "r");

    if (filehandle != NULL) {
        do {
            readCount = fread(&data, sizeof(data), 1, filehandle);

            if (readCount != 0) {
                // INFO("readcount = %d -> %f, %f, %f, %f", readCount, data.x, data.y, data.z, data.intensity);
                pcd->push_back(data);
            }
        } while (readCount);

        getInstance()->cb_to_uppder(pcd);
        fclose(filehandle);

    } else {
        ERROR("std error : %d", errno);
    }
}

}  // namespace FileLoader
}  // namespace Loader
