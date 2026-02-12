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

    void setDefault(
        boost::property_tree::ptree &pt,
        std::string &serverAddress,
        int &serverPort,
        bool &useNats,
        bool &saveJson,
        bool &scenarioLog,
        bool &labelWrite,
        std::string &labelOutputPath
    );

    bool loadFromFile(const std::string &filePath);
    void print() const;
};
