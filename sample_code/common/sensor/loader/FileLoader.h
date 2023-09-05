#ifndef __FILE_LOADER_H__
#define __FILE_LOADER_H__

#include <thread>
#include <vector>

namespace Loader
{
namespace FileLoader
{

typedef void (*xBaseCallback)(std::string filename);

class BaseFileLoader
{
public:
    BaseFileLoader();
    ~BaseFileLoader();
    void readStart(std::string folderPath, xBaseCallback cb);
    static unsigned long long getTimestamp(const char* name);

private:
    bool mEnable;
    std::shared_ptr<std::thread> mThreadHandler;
    static void ThreadLoader(BaseFileLoader* base, std::string targetPath);
    xBaseCallback mCallback;

    int pushFileList(std::string path);
};
}  // namespace FileLoader
}  // namespace Loader

#endif
