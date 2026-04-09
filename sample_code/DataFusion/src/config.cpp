#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <ara/log/logger.h>
#include "logger.h"
#include "ara/core/initialization.h"

struct BoundaryPointConfig
{
    double lon = 0.0;
    double lat = 0.0;
};

struct BoundaryPolygonConfig
{
    std::string sectionName;
    bool enabled = false;
    std::vector<BoundaryPointConfig> points;
};

class Config
{
public:
    // 멤버 변수
    std::string serverAddress;
    int serverPort;
    bool useSSL;
    bool useNats = false;
    bool saveJson = false;
    std::string listFilePath;
    std::vector<BoundaryPolygonConfig> boundary255Zones;

private:
    template <typename T>
    T readOrDefault(const boost::property_tree::ptree &pt, const std::string &key, const T &defaultValue)
    {
        return pt.get<T>(key, defaultValue);
    }

    bool startsWith(const std::string &value, const std::string &prefix)
    {
        return value.rfind(prefix, 0) == 0;
    }

    void loadBoundary255Zones(const boost::property_tree::ptree &pt)
    {
        boundary255Zones.clear();

        for (const auto &sectionEntry : pt)
        {
            const std::string &sectionName = sectionEntry.first;
            if (!startsWith(sectionName, "Boundary255"))
                continue;

            BoundaryPolygonConfig zone;
            zone.sectionName = sectionName;
            zone.enabled = readOrDefault<bool>(sectionEntry.second, "Enable", false);
            zone.points.reserve(4);

            for (int i = 1; i <= 4; ++i)
            {
                const std::string index = std::to_string(i);
                BoundaryPointConfig point;
                point.lon = readOrDefault<double>(sectionEntry.second, "P" + index + "Lon", 0.0);
                point.lat = readOrDefault<double>(sectionEntry.second, "P" + index + "Lat", 0.0);
                zone.points.push_back(point);
            }

            boundary255Zones.push_back(std::move(zone));
        }
    }

    void loadFromTree(const boost::property_tree::ptree &pt)
    {
        serverAddress = readOrDefault<std::string>(pt, "Network.NATSServerAddress", "https://nats.beyless.com");
        serverPort = readOrDefault<int>(pt, "Network.ServerPort", 0);
        useSSL = readOrDefault<bool>(pt, "Network.UseSSL", false);
        useNats = readOrDefault<bool>(pt, "Network.UseNats", false);
        saveJson = readOrDefault<bool>(pt, "Network.SaveJson", false);
        listFilePath = readOrDefault<std::string>(pt, "Network.ListFilePath", "/opt/DataFusion/etc/lists.ini");

        loadBoundary255Zones(pt);
    }

public:
    // INI 파일 읽기 및 멤버 변수 초기화
    bool loadFromFile(const std::string &filePath)
    {
        boost::property_tree::ptree pt;
        try
        {
            boost::property_tree::ini_parser::read_ini(filePath, pt);
            loadFromTree(pt);

            return true; // 성공
        }
        catch (const std::exception &ex)
        {
            adcm::Log::Info() << "Error reading INI file: " << ex.what();
            loadFromTree(pt);

            return false; // 실패
        }
    }

    // 디버그용 설정값 출력
    void print() const
    {
        adcm::Log::Info() << "Server Address: " << serverAddress;
        adcm::Log::Info() << "Server Port: " << serverPort;
        adcm::Log::Info() << "Use SSL: " << (useSSL ? "true" : "false");
        adcm::Log::Info() << "Use NATS: " << (useNats ? "true" : "false");
        adcm::Log::Info() << "Save JSON: " << (saveJson ? "true" : "false");
        adcm::Log::Info() << "List File Path: " << listFilePath;
        adcm::Log::Info() << "Boundary255 Zone Count: " << boundary255Zones.size();
        for (const auto &zone : boundary255Zones)
        {
            adcm::Log::Info() << zone.sectionName << " Enable: " << (zone.enabled ? "true" : "false");
            for (size_t i = 0; i < zone.points.size(); ++i)
            {
                adcm::Log::Info() << zone.sectionName << " P" << (i + 1) << "(lon,lat): ("
                                  << zone.points[i].lon << ", " << zone.points[i].lat << ")";
            }
        }
    }
};
