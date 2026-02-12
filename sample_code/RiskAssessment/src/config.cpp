#include "config.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <iostream>
#include <string>
#include <ara/log/logger.h>
#include "logger.h"

void Config::setDefault(
    boost::property_tree::ptree &pt,
    std::string &serverAddress,
    int &serverPort,
    bool &useNats,
    bool &saveJson,
    bool &scenarioLog,
    bool &labelWrite,
    std::string &labelOutputPath
) {
    serverAddress = pt.get<std::string>("Network.NATSServerAddress", "https://nats.beyless.com");
    serverPort = pt.get<int>("Network.ServerPort", 0);
    useNats = pt.get<bool>("Network.UseNats", false);
    saveJson = pt.get<bool>("Network.SaveJson", false);
    scenarioLog = pt.get<bool>("Network.ScenarioLog", false);
    labelWrite = pt.get<bool>("label.LabelWrite", false);
    labelOutputPath = pt.get<std::string>("label.LabelOutputPath", "");
}

bool Config::loadFromFile(const std::string &filePath)
{
    boost::property_tree::ptree pt;
    try
    {
        boost::property_tree::ini_parser::read_ini(filePath, pt);
        setDefault(pt, serverAddress, serverPort, useNats, saveJson, scenarioLog, labelWrite, labelOutputPath);
        return true;
    }
    catch (const std::exception &ex)
    {
        adcm::Log::Info() << "Error reading INI file: " << ex.what();
        setDefault(pt, serverAddress, serverPort, useNats, saveJson, scenarioLog, labelWrite, labelOutputPath);
        return false;
    }
}

void Config::print() const
{
    adcm::Log::Info() << "Server Address: " << serverAddress;
    adcm::Log::Info() << "Server Port: " << serverPort;
    adcm::Log::Info() << "Use NATS: " << (useNats ? "true" : "false");
    adcm::Log::Info() << "Save JSON: " << (saveJson ? "true" : "false");
    adcm::Log::Info() << "Scenario Log: " << (scenarioLog ? "true" : "false");
    adcm::Log::Info() << "Label Write: " << (labelWrite ? "true" : "false");
    adcm::Log::Info() << "Label Output Path: " << labelOutputPath;
}
