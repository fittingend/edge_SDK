#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <iostream>
#include <string>
#include <ara/log/logger.h>
#include "logger.h"
#include "ara/core/initialization.h"

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
    bool boundary255Enabled = false;
    double b255P1Lon = 0.0;
    double b255P1Lat = 0.0;
    double b255P2Lon = 0.0;
    double b255P2Lat = 0.0;
    double b255P3Lon = 0.0;
    double b255P3Lat = 0.0;
    double b255P4Lon = 0.0;
    double b255P4Lat = 0.0;

private:
    template <typename T>
    T readOrDefault(const boost::property_tree::ptree &pt, const std::string &key, const T &defaultValue)
    {
        return pt.get<T>(key, defaultValue);
    }

    void loadFromTree(const boost::property_tree::ptree &pt)
    {
        serverAddress = readOrDefault<std::string>(pt, "Network.NATSServerAddress", "https://nats.beyless.com");
        serverPort = readOrDefault<int>(pt, "Network.ServerPort", 0);
        useSSL = readOrDefault<bool>(pt, "Network.UseSSL", false);
        useNats = readOrDefault<bool>(pt, "Network.UseNats", false);
        saveJson = readOrDefault<bool>(pt, "Network.SaveJson", false);
        listFilePath = readOrDefault<std::string>(pt, "Network.ListFilePath", "/opt/DataFusion/etc/lists.ini");

        boundary255Enabled = readOrDefault<bool>(pt, "Boundary255.Enable", false);
        b255P1Lon = readOrDefault<double>(pt, "Boundary255.P1Lon", 0.0);
        b255P1Lat = readOrDefault<double>(pt, "Boundary255.P1Lat", 0.0);
        b255P2Lon = readOrDefault<double>(pt, "Boundary255.P2Lon", 0.0);
        b255P2Lat = readOrDefault<double>(pt, "Boundary255.P2Lat", 0.0);
        b255P3Lon = readOrDefault<double>(pt, "Boundary255.P3Lon", 0.0);
        b255P3Lat = readOrDefault<double>(pt, "Boundary255.P3Lat", 0.0);
        b255P4Lon = readOrDefault<double>(pt, "Boundary255.P4Lon", 0.0);
        b255P4Lat = readOrDefault<double>(pt, "Boundary255.P4Lat", 0.0);
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
        adcm::Log::Info() << "Boundary255 Enable: " << (boundary255Enabled ? "true" : "false");
        adcm::Log::Info() << "Boundary255 P1(lon,lat): (" << b255P1Lon << ", " << b255P1Lat << ")";
        adcm::Log::Info() << "Boundary255 P2(lon,lat): (" << b255P2Lon << ", " << b255P2Lat << ")";
        adcm::Log::Info() << "Boundary255 P3(lon,lat): (" << b255P3Lon << ", " << b255P3Lat << ")";
        adcm::Log::Info() << "Boundary255 P4(lon,lat): (" << b255P4Lon << ", " << b255P4Lat << ")";
    }
};
