#pragma once

#include <string>
#include <boost/property_tree/ptree_fwd.hpp>

class Config
{
public:
    std::string serverAddress;
    int serverPort;
    bool useNats = false;
    bool saveJson = false;
    bool scenarioLog = false;
    bool labelWrite = false;
    std::string labelOutputPath;
    int scenario7MinUnscanned = 10;
    int stopValue = 1;
    double scenario7VehicleWidthM = 2.5;
    bool aiModelAnalysis = false;
    std::string aiModelPath;

    void setDefault(
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
        std::string &aiModelPath
    );

    bool loadFromFile(const std::string &filePath);
    void print() const;
};
