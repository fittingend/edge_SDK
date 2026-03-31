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
    std::string &labelOutputPath,
    int &scenario7MinUnscanned,
    int &stopValue,
    double &scenario7VehicleWidthM,
    bool &aiModelAnalysis,
    std::string &aiModelPath,
    std::string &aiMetaPath,
    std::string &aiPreprocessScript,
    std::string &aiPythonBin,
    float &aiThreshold,
    bool &aiFallbackToRule
) {
    serverAddress = pt.get<std::string>("Network.NATSServerAddress", "https://nats.beyless.com");
    serverPort = pt.get<int>("Network.ServerPort", 0);
    useNats = pt.get<bool>("Network.UseNats", false);
    saveJson = pt.get<bool>("Network.SaveJson", false);
    scenarioLog = pt.get<bool>("Network.ScenarioLog", false);
    labelWrite = pt.get<bool>("label.LabelWrite", false);
    labelOutputPath = pt.get<std::string>("label.LabelOutputPath", "");
    scenario7MinUnscanned = pt.get<int>("Scenario.MinUnscanned7", 10);
    stopValue = pt.get<int>("Scenario.StopValue", 1);
    scenario7VehicleWidthM = pt.get<double>("Scenario.VehicleWidthM", 2.5);
    aiModelAnalysis = pt.get<bool>("AI.AIModelAnalysis", false);
    aiModelPath = pt.get<std::string>("AI.ModelPath", "");
    aiMetaPath = pt.get<std::string>("AI.MetaPath", "");
    aiPreprocessScript = pt.get<std::string>("AI.PreprocessScript", "");
    aiPythonBin = pt.get<std::string>("AI.PythonBin", "python3");
    aiThreshold = pt.get<float>("AI.Threshold", 0.5f);
    aiFallbackToRule = pt.get<bool>("AI.FallbackToRule", true);
}

bool Config::loadFromFile(const std::string &filePath)
{
    boost::property_tree::ptree pt;
    try
    {
        boost::property_tree::ini_parser::read_ini(filePath, pt);
        setDefault(pt, serverAddress, serverPort, useNats, saveJson, scenarioLog, labelWrite, labelOutputPath, scenario7MinUnscanned, stopValue, scenario7VehicleWidthM, aiModelAnalysis, aiModelPath, aiMetaPath, aiPreprocessScript, aiPythonBin, aiThreshold, aiFallbackToRule);
        return true;
    }
    catch (const std::exception &ex)
    {
        adcm::Log::Info() << "Error reading INI file: " << ex.what();
        setDefault(pt, serverAddress, serverPort, useNats, saveJson, scenarioLog, labelWrite, labelOutputPath, scenario7MinUnscanned, stopValue, scenario7VehicleWidthM, aiModelAnalysis, aiModelPath, aiMetaPath, aiPreprocessScript, aiPythonBin, aiThreshold, aiFallbackToRule);
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
    adcm::Log::Info() << "Scenario7 Min Unscanned: " << scenario7MinUnscanned;
    adcm::Log::Info() << "Stop Value: " << stopValue;
    adcm::Log::Info() << "Scenario7 Vehicle Width (m): " << scenario7VehicleWidthM;
    adcm::Log::Info() << "AIModelAnalysis: " << (aiModelAnalysis ? "true" : "false");
    adcm::Log::Info() << "AI Model Path: " << aiModelPath;
    adcm::Log::Info() << "AI Meta Path: " << aiMetaPath;
    adcm::Log::Info() << "AI Preprocess Script: " << aiPreprocessScript;
    adcm::Log::Info() << "AI Python Bin: " << aiPythonBin;
    adcm::Log::Info() << "AI Threshold: " << aiThreshold;
    adcm::Log::Info() << "AI FallbackToRule: " << (aiFallbackToRule ? "true" : "false");
}
