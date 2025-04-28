#include "NatsHandler.hpp"
#include <sstream>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <mutex>

// 전역 변수 정의
#ifdef NATS
bool firstTime = true;
natsStatus s = NATS_OK;
std::vector<const char*> subject = {"test1.*", "test2.*"};
std::shared_ptr<adcm::etc::NatsConnManager> natsManager;
std::mutex mtx;

void asyncCb(natsConnection* nc, natsSubscription* sub, natsStatus err, void* closure)
{
    std::cout << "Async error: " << err << " - " << natsStatus_GetText(err) << std::endl;
    natsManager->NatsSubscriptionGetDropped(sub, (int64_t*) &natsManager->dropped);
}

void onMsg(natsConnection* nc, natsSubscription* sub, natsMsg* msg, void* closure)
{
    std::lock_guard<std::mutex> lock(mtx);
    const char* subject = natsMsg_GetSubject(msg);

    std::cout << "Received msg: [" << subject << " : " << natsMsg_GetDataLength(msg) << "]" << natsMsg_GetData(msg) << std::endl;
    natsManager->NatsMsgDestroy(msg);
}

void saveToJsonFile(const std::string& key, const std::string& value, int& fileCount)
{
    std::stringstream fileNameStream;
    fileNameStream << key << "_" << fileCount << ".json";
    std::string fileName = fileNameStream.str();

    std::ofstream outFile(fileName);
    if (outFile.is_open()) {
        outFile << value;
        outFile.close();
        adcm::Log::Info() << "JSON data stored in " << fileName;
    } else {
        adcm::Log::Error() << "Failed to open file " << fileName << " for writing!";
    }
    ++fileCount;
}

std::string convertRiskAssessmentToJsonString(const adcm::risk_assessment_Objects& riskAssessment)
{
    uint64_t timestamp_map = 111;
    uint64_t timestamp_risk = riskAssessment.timestamp;
    std::string model_id = "test_id_v1";

    std::stringstream oss;
    oss << "{\n\"riskAssessmentList\": [\n";

    for (size_t i = 0; i < riskAssessment.riskAssessmentList.size(); ++i) {
        const auto& item = riskAssessment.riskAssessmentList[i];
        oss << "        {\n"
            << "            \"obstacle_id\": " << item.obstacle_id << ",\n"
            << "            \"wgs84_xy_start\": [\n";

        for (size_t j = 0; j < item.wgs84_xy_start.size(); ++j) {
            oss << "                { \"x\": " << item.wgs84_xy_start[j].x
                << ", \"y\": " << item.wgs84_xy_start[j].y << "}";
            if (j < item.wgs84_xy_start.size() - 1) oss << ",";
            oss << "\n";
        }

        oss << "            ],\n\"wgs84_xy_end\": [\n";

        for (size_t j = 0; j < item.wgs84_xy_end.size(); ++j) {
            oss << "                { \"x\": " << item.wgs84_xy_end[j].x
                << ", \"y\": " << item.wgs84_xy_end[j].y << "}";
            if (j < item.wgs84_xy_end.size() - 1) oss << ",";
            oss << "\n";
        }

        oss << "            ],\n"
            << "            \"hazard_class\": " << static_cast<int>(item.hazard_class) << ",\n"
            << "            \"isHarzard\": " << (item.isHarzard ? "true" : "false") << ",\n"
            << "            \"confidence\": " << item.confidence << ",\n"
            << "            \"timestamp_map\": " << timestamp_map << ",\n"
            << "            \"timestamp_risk\": " << timestamp_risk << ",\n"
            << "            \"model_id\": \"" << model_id << "\"\n"
            << "        }";
        if (i < riskAssessment.riskAssessmentList.size() - 1) oss << ",";
        oss << "\n";
    }
    oss << "    ]\n}";
    return oss.str();
}

void NatsSend(const adcm::risk_assessment_Objects& riskAssessment)
{
    static int risk_count = 0;

    if (firstTime) {
        adcm::Log::Info() << "NATS first time setup!";
        natsManager = std::make_shared<adcm::etc::NatsConnManager>(HMI_SERVER_URL, subject, onMsg, asyncCb, adcm::etc::NatsConnManager::Mode::Default);
        s = natsManager->NatsExecute();
        firstTime = false;
    }

    if (s == NATS_OK) {
        const char* pubSubject = "test2.JSON";
        natsManager->ClearJsonData();
        std::string riskToStr = convertRiskAssessmentToJsonString(riskAssessment);
        natsManager->addJsonData("riskAssessment", riskToStr);
        natsManager->NatsPublishJson(pubSubject);
        adcm::Log::Info() << "NatsPublishJson";
        saveToJsonFile("riskAssessment", riskToStr, risk_count);
    } else {
        std::cout << "Nats Connection error" << std::endl;
        try {
            natsManager = std::make_shared<adcm::etc::NatsConnManager>(HMI_SERVER_URL, subject, onMsg, asyncCb, adcm::etc::NatsConnManager::Mode::Default);
            s = natsManager->NatsExecute();
        } catch (std::exception& e) {
            std::cout << "Nats reConnection error" << std::endl;
        }
    }
}
#endif
