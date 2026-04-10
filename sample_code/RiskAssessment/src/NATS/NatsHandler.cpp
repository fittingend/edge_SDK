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

namespace {
    bool firstTime = true;
    natsStatus s = NATS_OK;
    std::vector<const char*> subject = {"test1.*", "test2.*"};
    std::shared_ptr<adcm::etc::NatsConnManager> natsManager;
    std::mutex mtx;
}

static Poco::JSON::Object::Ptr buildRiskAssessmentJson(const adcm::risk_assessment_Objects& riskAssessment,
                                                       const obstacleListVector& obstacle_list);

void asyncCb(natsConnection* nc, natsSubscription* sub, natsStatus err, void* closure)
{
    adcm::Log::Info() << "Async error: " << err << " - " << natsStatus_GetText(err);
    natsManager->NatsSubscriptionGetDropped(sub, (int64_t*) &natsManager->dropped);
}

void onMsg(natsConnection* nc, natsSubscription* sub, natsMsg* msg, void* closure)
{
    std::lock_guard<std::mutex> lock(mtx);
    const char* subject = natsMsg_GetSubject(msg);

    adcm::Log::Info() << "Received msg: [" << subject << " : " << natsMsg_GetDataLength(msg) << "]" << natsMsg_GetData(msg);
    natsManager->NatsMsgDestroy(msg);
}

bool NatsConnectOnStartup()
{
    if (!firstTime)
    {
        return (s == NATS_OK);
    }

    adcm::Log::Info() << "NATS startup probe, target=" << nats_server_url;
    natsManager = std::make_shared<adcm::etc::NatsConnManager>(
        nats_server_url.c_str(), subject, onMsg, asyncCb,
        adcm::etc::NatsConnManager::Mode::Default);
    s = natsManager->NatsExecute();
    firstTime = false;

    if (s == NATS_OK)
    {
        adcm::Log::Info() << "NATS startup probe OK to " << nats_server_url;
        return true;
    }

    adcm::Log::Error() << "NATS startup probe FAIL(" << s << ": "
                       << natsStatus_GetText(s) << ") to " << nats_server_url;
    return false;
}

bool NatsSend(const adcm::risk_assessment_Objects& riskAssessment, const obstacleListVector& obstacle_list)
{
    if (firstTime) {
        (void)NatsConnectOnStartup();
    }

    if (s == NATS_OK) {
        const char* pubSubject = "riskAssessmentObjects.create";
        natsManager->ClearJsonData();
        Poco::JSON::Object::Ptr riskObj = buildRiskAssessmentJson(riskAssessment, obstacle_list);
        natsManager->addJsonData("riskAssessment", riskObj);
        adcm::Log::Info() << "RiskAssessment JSON object added to NATS payload";
        natsStatus pubStatus = natsManager->NatsPublishJson(pubSubject);
        if (pubStatus == NATS_OK) {
            adcm::Log::Info() << "NATS publish OK: " << pubSubject;
            return true;
        } else {
            adcm::Log::Error() << "NATS publish FAIL(" << pubStatus << "): " << pubSubject;
            return false;
        }
    } else {
        adcm::Log::Error() << "NATS connection state is not OK(" << s << ": "
                           << natsStatus_GetText(s) << "), reconnect target=" << nats_server_url;
        try {
            natsManager = std::make_shared<adcm::etc::NatsConnManager>(
                nats_server_url.c_str(), subject, onMsg, asyncCb,
                adcm::etc::NatsConnManager::Mode::Publish_Only);
            s = natsManager->NatsExecute();
            if (s == NATS_OK) {
                adcm::Log::Info() << "NATS reconnect OK to " << nats_server_url;
            } else {
                adcm::Log::Error() << "NATS reconnect FAIL(" << s << ": "
                                   << natsStatus_GetText(s) << ") to " << nats_server_url;
            }
        } catch (std::exception& e) {
            adcm::Log::Error() << "NATS reconnect exception to " << nats_server_url
                               << ": " << e.what();
        }
    }

    return false;
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
static Poco::JSON::Object::Ptr buildRiskAssessmentJson(const adcm::risk_assessment_Objects& riskAssessment,
                                                       const obstacleListVector& obstacle_list)
{
    using namespace Poco::JSON;

    Object::Ptr root = new Object;
    Array::Ptr riskList = new Array;

    //uint64_t timestamp_map = 111;
    uint64_t timestamp_risk = riskAssessment.timestamp;
    std::string model_id = "test_id_v1";

    for (const auto& item : riskAssessment.riskAssessmentList) {
        Object::Ptr obj = new Object;
        obj->set("obstacle_id", item.obstacle_id);
        // default when no match
        obj->set("obstacle_class", -1);
        obj->set("obstacle_class_kr", std::string("알 수 없음"));
        const adcm::obstacleListStruct* matched = nullptr;
        for (const auto& obs : obstacle_list) {
            if (obs.obstacle_id == item.obstacle_id) {
                matched = &obs;
                break;
            }
        }
        if (matched != nullptr) {
            Object::Ptr xy = new Object;
            xy->set("x", matched->fused_position_x);
            xy->set("y", matched->fused_position_y);
            obj->set("obstacle_xy", xy);
            obj->set("obstacle_class", static_cast<int>(matched->obstacle_class));
            obj->set("obstacle_class_kr",
                     std::string(to_string(static_cast<ObstacleClass>(matched->obstacle_class))));
        }

        // hazard_path_start
        Array::Ptr startArr = new Array;
        for (const auto& pt : item.hazard_path_start) {
            Object::Ptr xy = new Object;
            xy->set("x", pt.x);
            xy->set("y", pt.y);
            startArr->add(xy);
        }
        obj->set("hazard_path_start", startArr);

        // hazard_path_end
        Array::Ptr endArr = new Array;
        for (const auto& pt : item.hazard_path_end) {
            Object::Ptr xy = new Object;
            xy->set("x", pt.x);
            xy->set("y", pt.y);
            endArr->add(xy);
        }
        obj->set("hazard_path_end", endArr);
        obj->set("hazard_path", item.hazard_path);
        
        obj->set("hazard_class", static_cast<int>(item.hazard_class));
        obj->set("confidence", item.confidence);
        obj->set("timestamp_map", timestamp_map);
        obj->set("timestamp_risk", timestamp_risk);
        obj->set("model_id", model_id);

        riskList->add(obj);
    }

    root->set("riskAssessmentList", riskList);
    return root;
}

std::string convertRiskAssessmentToJsonString(const adcm::risk_assessment_Objects& riskAssessment)
{
    static const obstacleListVector empty;
    Poco::JSON::Object::Ptr root = buildRiskAssessmentJson(riskAssessment, empty);
    std::ostringstream oss;
    root->stringify(oss, 2);  // 2 = 들여쓰기 (pretty print)
    return oss.str();
}

std::string convertRiskAssessmentToJsonString(const adcm::risk_assessment_Objects& riskAssessment,
                                              const obstacleListVector& obstacle_list)
{
    Poco::JSON::Object::Ptr root = buildRiskAssessmentJson(riskAssessment, obstacle_list);
    std::ostringstream oss;
    root->stringify(oss, 2);  // 2 = 들여쓰기 (pretty print)
    return oss.str();
}
