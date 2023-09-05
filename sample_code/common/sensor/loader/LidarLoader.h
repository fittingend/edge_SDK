#ifndef __LIDAR_LOADER_H__
#define __LIDAR_LOADER_H__

#include "loader/FileLoader.h"
#include "lidar/velodyne_H800.h"

namespace Loader
{

namespace FileLoader
{

typedef void (*xCallback_PCD)(std::shared_ptr<Velodyne::Lidar::xPCD>);

typedef std::vector<unsigned char> xBaseBuffer;

class LidarLoader : public BaseFileLoader
{
public:
    LidarLoader();
    static std::shared_ptr<LidarLoader> getInstance();
    void init(std::string filePath, xCallback_PCD cb);
    void start();

private:
    xCallback_PCD cb_to_uppder;
    std::string mDataPath;
    static void callback(std::string path);
};
}  // namespace FileLoader
}  // namespace Loader
#endif
