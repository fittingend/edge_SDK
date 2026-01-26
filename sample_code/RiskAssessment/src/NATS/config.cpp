#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <iostream>
#include <string>
#include <ara/log/logger.h>
#include "logger.h"

class Config
{
public:
    std::string serverAddress;
    int serverPort;
    bool useSSL;
    bool useNats = false;
    bool saveJson = false;

    void setDefault(boost::property_tree::ptree &pt, std::string &serverAddress, int &serverPort, bool &useSSL, bool &useNats)
    {
        serverAddress = pt.get<std::string>("Network.NATSServerAddress", "https://nats.beyless.com");
        serverPort = pt.get<int>("Network.ServerPort", 0);
        useSSL = pt.get<bool>("Network.UseSSL", false);
        useNats = pt.get<bool>("Network.UseNats", false);
        saveJson = pt.get<bool>("Network.SaveJson", false);
    }

    bool loadFromFile(const std::string &filePath)
    {
        boost::property_tree::ptree pt;
        try
        {
            boost::property_tree::ini_parser::read_ini(filePath, pt);
            setDefault(pt, serverAddress, serverPort, useSSL, useNats);
            return true;
        }
        catch (const std::exception &ex)
        {
            adcm::Log::Info() << "Error reading INI file: " << ex.what();
            setDefault(pt, serverAddress, serverPort, useSSL, useNats);
            return false;
        }
    }

    void print() const
    {
        adcm::Log::Info() << "Server Address: " << serverAddress;
        adcm::Log::Info() << "Server Port: " << serverPort;
        adcm::Log::Info() << "Use SSL: " << (useSSL ? "true" : "false");
        adcm::Log::Info() << "Use NATS: " << (useNats ? "true" : "false");
        adcm::Log::Info() << "Save JSON: " << (saveJson ? "true" : "false");
    }
};
