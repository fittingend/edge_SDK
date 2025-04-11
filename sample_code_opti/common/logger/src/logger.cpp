#include "logger.h"
#include <sys/time.h>
#include <ctime>

ara::log::Logger& get_instance()
{
    static ara::log::Logger& g_logger{
        ara::log::CreateLogger(DEFAULT_LOG_TAG, DEFAULT_LOG_TAG, ara::log::LogLevel::kInfo)};
    return g_logger;
}

namespace adcm
{

LogStream& Log::F()
{
    static LogStream g_fatal_logStream{get_instance().LogFatal()};
    return g_fatal_logStream;
}

LogStream& Log::E()
{
    static LogStream g_error_logStream{get_instance().LogError()};
    return g_error_logStream;
}

LogStream& Log::W()
{
    static LogStream g_warn_logStream{get_instance().LogWarn()};
    return g_warn_logStream;
}

LogStream& Log::I()
{
    static LogStream g_info_logStream{get_instance().LogInfo()};
    return g_info_logStream;
}

LogStream& Log::D()
{
    static LogStream g_debug_logStream{get_instance().LogDebug()};
    return g_debug_logStream;
}

LogStream& Log::V()
{
    static LogStream g_verbose_logStream{get_instance().LogVerbose()};
    return g_verbose_logStream;
}

LogStream Log::Fatal()
{
    return get_instance().LogFatal();
}

LogStream Log::Error()
{
    return get_instance().LogError();
}

LogStream Log::Warn()
{
    return get_instance().LogWarn();
}

LogStream Log::Info()
{
    return get_instance().LogInfo();
}

LogStream Log::Debug()
{
    return get_instance().LogDebug();
}

LogStream Log::Verbose()
{
    return get_instance().LogVerbose();
}

unsigned long long Log::getUptime()
{
    unsigned long millisec_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return millisec_since_epoch;
}

}  // namespace adcm
