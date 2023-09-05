

#include <logger.h>

#include "loader/CameraLoader.h"
#include <dirent.h>
#include <sys/stat.h>

namespace Loader
{
namespace FileLoader
{

std::shared_ptr<CameraLoader> CameraLoader::getInstance()
{
    static std::shared_ptr<CameraLoader> instance = std::make_shared<CameraLoader>();
    return instance;
}

CameraLoader::CameraLoader()
    : BaseFileLoader()
{ }

void CameraLoader::init(int id, std::string filePath, xCameraCallback cb)
{
    cb_to_uppder = cb;
    mDataPath = filePath;
    this->id = id;
}

void CameraLoader::start()
{
    readStart(mDataPath, callback);
}
void CameraLoader::callback(std::string path)
{
    FILE* filehandle;
    struct stat sb;
    std::shared_ptr<adcm::sensor::AICM_Cam::xImgBuffer> fileBuffer;
    stat(path.c_str(), &sb);
    fileBuffer = std::make_shared<adcm::sensor::AICM_Cam::xImgBuffer>();
    fileBuffer->buffer.reserve(sb.st_size);
    filehandle = fopen(path.c_str(), "r");

    for (int i = 0; i < sb.st_size; i++) {
        unsigned char temp;

        if (fread(&temp, 1, 1, filehandle)) {
            fileBuffer->buffer.push_back(temp);
        }
    }

    // INFO("%s - %ld / %ld -  %s", path.c_str(), sb.st_size, fileBuffer->size(), &(*fileBuffer)[6]);
    getInstance()->cb_to_uppder(getInstance()->id,fileBuffer);
    fclose(filehandle);
}

}  // namespace FileLoader
}  // namespace Loader
