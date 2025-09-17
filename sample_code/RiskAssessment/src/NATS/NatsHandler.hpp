#ifndef NATS_HANDLER_HPP
#define NATS_HANDLER_HPP

#ifdef NATS

#include <vector>
#include <memory>
#include <string>
#include <mutex>
#include "NatsConnManager.h"
#include "../main_riskassessment.hpp"  // 필요한 데이터 타입 참조

#define HMI_SERVER_URL  "https://nats.beyless.com"

// 전역 변수 (extern 선언)
extern bool firstTime;
extern natsStatus s;
extern std::vector<const char*> subject;
extern std::shared_ptr<adcm::etc::NatsConnManager> natsManager;

// 콜백 함수 및 NATS 관련 함수 선언
void asyncCb(natsConnection* nc, natsSubscription* sub, natsStatus err, void* closure);
void onMsg(natsConnection* nc, natsSubscription* sub, natsMsg* msg, void* closure);
void saveToJsonFile(const std::string& key, const std::string& value, int& fileCount);
std::string convertRiskAssessmentToJsonString(const adcm::risk_assessment_Objects& riskAssessment);
void NatsSend(const adcm::risk_assessment_Objects& riskAssessment);

#endif // NATS


#ifdef ENABLE_JSON
#include "../main_riskassessment.hpp"  // 필요한 데이터 타입 참조
void SaveAsJson(const adcm::risk_assessment_Objects& riskAssessment);
void saveToJsonFile(const std::string& key, const std::string& value, int& fileCount);
std::string convertRiskAssessmentToJsonString(const adcm::risk_assessment_Objects& riskAssessment);
#endif

#endif // NATS_HANDLER_HPP
