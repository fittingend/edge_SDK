#include "NatsHandler.hpp"
#include "../main_riskassessment.hpp"
#include <sstream>
#include <fstream>
#include <iostream>
#include <mutex>
#include <boost/filesystem.hpp>
#include <Poco/JSON/Object.h>
#include <Poco/JSON/Array.h>
#include <Poco/JSON/Stringifier.h>

using namespace Poco::JSON;

// 전역 변수 정의
std::vector<const char*> subject = {};
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

void NatsSend(const adcm::risk_assessment_Objects& riskAssessment)
{
    static bool firstTime = true;
    static natsStatus s = NATS_OK;

    if (firstTime) {
        adcm::Log::Info() << "NATS first time setup!";
        natsManager = std::make_shared<adcm::etc::NatsConnManager>(
            nats_server_url.c_str(), subject, onMsg, asyncCb,
            adcm::etc::NatsConnManager::Mode::Default);
        s = natsManager->NatsExecute();
        firstTime = false;
    }

    if (s == NATS_OK) {
        //const char* pubSubject = "riskAssessment.json";
        const char* pubSubject = "riskAssessmentObjects.create";
        natsManager->ClearJsonData();
        std::string riskToStr = convertRiskAssessmentToJsonString(riskAssessment);
        natsManager->addJsonData("riskAssessment", riskToStr);
        natsManager->NatsPublishJson(pubSubject);
        adcm::Log::Info() << "NatsPublishJson";
    } else {
        std::cout << "Nats Connection error" << std::endl;
        try {
            natsManager = std::make_shared<adcm::etc::NatsConnManager>(
                nats_server_url.c_str(), subject, onMsg, asyncCb,
                adcm::etc::NatsConnManager::Mode::Default);
            s = natsManager->NatsExecute();
        } catch (std::exception& e) {
            std::cout << "Nats reConnection error" << std::endl;
        }
    }
}


void SaveAsJson(const adcm::risk_assessment_Objects& riskAssessment)
{
    static int risk_count = 0;
    std::string riskToStr = convertRiskAssessmentToJsonString(riskAssessment);
    saveToJsonFile("riskAssessment", riskToStr, risk_count);
}

void saveToJsonFile(const std::string& key, const std::string& value, int& fileCount)
{
    namespace fs = boost::filesystem;
    boost::system::error_code ec;

    fs::path outDir = "/opt/RiskAssessment/json";

    if (!fs::exists(outDir)) {
        if (!fs::create_directories(outDir, ec)) {
            adcm::Log::Error() << "Failed to create directory " 
                               << outDir.string() 
                               << " : " << ec.message();
            return;
        }
    }

    std::stringstream fileNameStream;
    fileNameStream << key << "_" << fileCount << ".json";
    fs::path filePath = outDir / fileNameStream.str();

    std::ofstream outFile(filePath.string());
    if (outFile.is_open()) {
        outFile << value;
        outFile.close();
        adcm::Log::Info() << "JSON data stored in " << filePath.string();
    } else {
        adcm::Log::Error() << "Failed to open file " << filePath.string()
                           << " for writing! errno=" << strerror(errno);
    }
    ++fileCount;
}
std::string convertRiskAssessmentToJsonString(const adcm::risk_assessment_Objects& riskAssessment)
{
    using namespace Poco::JSON;

    Object root;
    Array::Ptr riskList = new Array;

    //uint64_t timestamp_map = 111;
    uint64_t timestamp_risk = riskAssessment.timestamp;
    std::string model_id = "test_id_v1";

    for (const auto& item : riskAssessment.riskAssessmentList) {
        Object::Ptr obj = new Object;

        obj->set("obstacle_id", item.obstacle_id);

        // wgs84_xy_start
        Array::Ptr startArr = new Array;
        for (const auto& pt : item.wgs84_xy_start) {
            Object::Ptr xy = new Object;
            xy->set("x", pt.x);
            xy->set("y", pt.y);
            startArr->add(xy);
        }
        obj->set("wgs84_xy_start", startArr);

        // wgs84_xy_end
        Array::Ptr endArr = new Array;
        for (const auto& pt : item.wgs84_xy_end) {
            Object::Ptr xy = new Object;
            xy->set("x", pt.x);
            xy->set("y", pt.y);
            endArr->add(xy);
        }
        obj->set("wgs84_xy_end", endArr);

        obj->set("hazard_class", static_cast<int>(item.hazard_class));
        obj->set("isHazard", item.isHarzard);
        obj->set("confidence", item.confidence);
        obj->set("timestamp_map", timestamp_map);
        obj->set("timestamp_risk", timestamp_risk);
        obj->set("model_id", model_id);

        riskList->add(obj);
    }

    root.set("riskAssessmentList", riskList);

    std::ostringstream oss;
    root.stringify(oss, 2);  // 2 = 들여쓰기 (pretty print)
    return oss.str();
}
