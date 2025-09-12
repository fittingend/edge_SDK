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
    bool useNats;
    bool saveJson;

    void setDefault(boost::property_tree::ptree &pt, std::string &serverAddress, int &serverPort, bool &useSSL, bool &useNats)
    {
        // 멤버 변수에 값 저장
        serverAddress = pt.get<std::string>("Network.NATSServerAddress", "https://nats.beyless.com"); // 기본값 설정
        serverPort = pt.get<int>("Network.ServerPort", 0);                                            // 기본값 설정
        useSSL = pt.get<bool>("Network.UseSSL", false);                                               // 기본값 설정
        useNats = pt.get<bool>("Network.UseNats", false);
        saveJson = pt.get<bool>("Network.SaveJson", false);
    }

    // INI 파일 읽기 및 멤버 변수 초기화
    bool loadFromFile(const std::string &filePath)
    {
        boost::property_tree::ptree pt;
        try
        {
            boost::property_tree::ini_parser::read_ini(filePath, pt);
            setDefault(pt, serverAddress, serverPort, useSSL, useNats);

            return true; // 성공
        }
        catch (const std::exception &ex)
        {
            adcm::Log::Info() << "Error reading INI file: " << ex.what();
            setDefault(pt, serverAddress, serverPort, useSSL, useNats);

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
    }
};
