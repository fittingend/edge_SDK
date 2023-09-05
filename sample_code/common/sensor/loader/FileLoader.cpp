#include <iostream>
#include <iomanip>
#include <vector>
#include <numeric>
#include <chrono>

#include "loader/FileLoader.h"

#include <logger.h>
#include <dirent.h>
#include <sys/stat.h>
#include <cstring>

namespace Loader
{
namespace FileLoader
{

int checkFile(const dirent* entry);

unsigned long long getTimedistance(std::chrono::time_point<std::chrono::system_clock> start, std::chrono::time_point<std::chrono::system_clock> end);

BaseFileLoader::BaseFileLoader()
{
    mEnable = false;
}

BaseFileLoader::~BaseFileLoader()
{
    mEnable = false;
    mThreadHandler->join();
}

void BaseFileLoader::readStart(std::string folderPath, xBaseCallback cb)
{
    mEnable = true;
    mCallback = cb;
    mThreadHandler = std::make_shared<std::thread>(ThreadLoader, this, folderPath);
}

unsigned long long BaseFileLoader::getTimestamp(const char* name)
{
    unsigned long long result = 0;
    int i;
    // INFO("Name = %s", name);

    for (i = 0; i < 19; i++) {
        result *= 10;

        if ((name[i] >= '0') && (name[i] <= '9')) {
            result += name[i] - '0';

        } else {
            return 0;
        }
    }

    if (name[i] == '.') {
        // INFO("%s = %lld", name, result);
        return result;

    } else {
        ERROR("ERROR : File name error -> %s", name);
        return 0;
    }
}

unsigned long long getTimedistance(std::chrono::time_point<std::chrono::system_clock> start, std::chrono::time_point<std::chrono::system_clock> end)
{
    std::chrono::duration<double> diff = end - start;
    unsigned long long result = diff.count() * 1000000000;
    // INFO("diff = %lf / %lld", diff.count(), result);
    return result;
}
int BaseFileLoader::pushFileList(std::string path)
{
    mCallback(path);
    return 0;
}

int checkFile(const dirent* entry)
{
#if 1
    unsigned long long temp;
    temp = BaseFileLoader::getTimestamp(entry->d_name);

    if (temp != 0) {
        return 1;

    } else {
        return 0;
    }

#else
    int len;
    len = strlen(entry->d_name);

    if (len == 23) {
        return 1;

    } else {
        return 0;
    }

#endif
}

void BaseFileLoader::ThreadLoader(BaseFileLoader* base, std::string targetPath)
{
    INFO("Loader Thread Start");
    std::chrono::time_point<std::chrono::system_clock> mBaseTime;
    unsigned long long startTime;
    unsigned long long logTime;
    DIR* dirhandle;
    struct dirent** entry_list;
    int entry_count;
    int entry_index;
    INFO("Folder = %s", targetPath.c_str());
    dirhandle = opendir(targetPath.c_str());

    if (dirhandle == NULL) {
        base->mEnable = false;
        ERROR("ERROR : Folder Open !!!");
        return;

    } else {
        // get list
        entry_count = scandir(targetPath.c_str(), &entry_list, checkFile, alphasort);

        if (entry_count <= 0) {
            ERROR("ERROR : empty folder");
            return;
        }
    }

    startTime = getTimestamp(entry_list[0]->d_name);
    mBaseTime = std::chrono::system_clock::now();
    INFO("start Log time = %lld, entry count = %d", startTime, entry_count);
    entry_index = 0;

    while (base->mEnable) {
        std::string path;
        unsigned long long temp;
        // calculating time
        logTime = startTime + getTimedistance(mBaseTime, std::chrono::system_clock::now());

        // INFO("log time = %lld", logTime);
        do {
            temp = getTimestamp(entry_list[entry_index]->d_name);

            if (logTime > temp) {
                path = targetPath + "/" + entry_list[entry_index]->d_name;
                // INFO("file -> %s", path.c_str());
                base->pushFileList(path);
                entry_index++;

                if (entry_index >= entry_count) {
                    entry_index = 0;
                    startTime = getTimestamp(entry_list[0]->d_name);
                    logTime = startTime;
                    mBaseTime = std::chrono::system_clock::now();
                    INFO("restart Log time = %lld, entry count = %d", startTime, entry_count);
                }
            }
        } while (logTime > temp);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    closedir(dirhandle);
    INFO("Loader Thread End");
}

}  // namespace FileLoader
}  // namespace Loader
