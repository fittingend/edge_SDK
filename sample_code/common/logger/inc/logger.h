#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <ara/log/logger.h>
using namespace ara::log;  // 'using' ara::log should be OK

ara::log::Logger& get_instance();

namespace adcm
{

class Log
{
public:
    static LogStream& F();
    static LogStream& E();
    static LogStream& W();
    static LogStream& I();
    static LogStream& D();
    static LogStream& V();

    static LogStream Fatal();
    static LogStream Error();
    static LogStream Warn();
    static LogStream Info();
    static LogStream Debug();
    static LogStream Verbose();
    static unsigned long long getUptime();
};

#define UNUSED(x) (void)(x)

#define FATAL(...)                                                            \
    do {                                                                      \
        char buf_tag[512];                                                    \
        char buf_cmt[512];                                                    \
        sprintf(buf_tag,                                                      \
            "%s/%s(%d) "                                                      \
            "",                                                               \
            (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__), \
            __FUNCTION__,                                                     \
            __LINE__);                                                        \
        sprintf(buf_cmt, ##__VA_ARGS__);                                      \
        get_instance().LogFatal() << buf_tag << buf_cmt;                      \
    } while (0)

#define ERROR(...)                                                            \
    do {                                                                      \
        char buf_tag[512];                                                    \
        char buf_cmt[512];                                                    \
        sprintf(buf_tag,                                                      \
            "%s/%s(%d) "                                                      \
            "",                                                               \
            (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__), \
            __FUNCTION__,                                                     \
            __LINE__);                                                        \
        sprintf(buf_cmt, ##__VA_ARGS__);                                      \
        get_instance().LogError() << buf_tag << buf_cmt;                      \
    } while (0)

#define WARNNING(...)                                                         \
    do {                                                                      \
        char buf_tag[512];                                                    \
        char buf_cmt[512];                                                    \
        sprintf(buf_tag,                                                      \
            "%s/%s(%d) "                                                      \
            "",                                                               \
            (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__), \
            __FUNCTION__,                                                     \
            __LINE__);                                                        \
        sprintf(buf_cmt, ##__VA_ARGS__);                                      \
        get_instance().LogWarn() << buf_tag << buf_cmt;                       \
    } while (0)

#define INFO(...)                                                             \
    do {                                                                      \
        char buf_tag[512];                                                    \
        char buf_cmt[512];                                                    \
        sprintf(buf_tag,                                                      \
            "%s/%s(%d) "                                                      \
            "",                                                               \
            (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__), \
            __FUNCTION__,                                                     \
            __LINE__);                                                        \
        sprintf(buf_cmt, ##__VA_ARGS__);                                      \
        get_instance().LogInfo() << buf_tag << buf_cmt;                       \
    } while (0)

#if (1)
#    define DEBUG(...)                                                            \
        do {                                                                      \
            char buf_tag[512];                                                    \
            char buf_cmt[512];                                                    \
            sprintf(buf_tag,                                                      \
                "%s/%s(%d) "                                                      \
                "",                                                               \
                (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__), \
                __FUNCTION__,                                                     \
                __LINE__);                                                        \
            sprintf(buf_cmt, ##__VA_ARGS__);                                      \
            get_instance().LogDebug() << buf_tag << buf_cmt;                      \
        } while (0)

#    define VERBOSE(...)                                                          \
        do {                                                                      \
            char buf_tag[512];                                                    \
            char buf_cmt[512];                                                    \
            sprintf(buf_tag,                                                      \
                "%s/%s(%d) "                                                      \
                "",                                                               \
                (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__), \
                __FUNCTION__,                                                     \
                __LINE__);                                                        \
            sprintf(buf_cmt, ##__VA_ARGS__);                                      \
            get_instance().LogVerbose() << buf_tag << buf_cmt;                    \
        } while (0)

#elif (1)

#    define DEBUG(...)                                                            \
        do {                                                                      \
            char buf_tag[512];                                                    \
            char buf_cmt[512];                                                    \
            sprintf(buf_tag,                                                      \
                "%s(%d)/%s "                                                      \
                "",                                                               \
                (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__), \
                __LINE__,                                                         \
                __FUNCTION__);                                                    \
            sprintf(buf_cmt, ##__VA_ARGS__);                                      \
            get_instance().LogDebug() << buf_tag << buf_cmt;                      \
        } while (0)

#    define VERBOSE(...)

#else

#    define DEBUG(...)
#    define VERBOSE(...)

#endif

}  // namespace adcm
#endif
