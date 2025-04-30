// --------------------------------------------------------------------------
// |              _    _ _______     .----.      _____         _____        |
// |         /\  | |  | |__   __|  .  ____ .    / ____|  /\   |  __ \       |
// |        /  \ | |  | |  | |    .  / __ \ .  | (___   /  \  | |__) |      |
// |       / /\ \| |  | |  | |   .  / / / / v   \___ \ / /\ \ |  _  /       |
// |      / /__\ \ |__| |  | |   . / /_/ /  .   ____) / /__\ \| | \ \       |
// |     /________\____/   |_|   ^ \____/  .   |_____/________\_|  \_\      |
// |                              . _ _  .                                  |
// --------------------------------------------------------------------------
//
// All Rights Reserved.
// Any use of this source code is subject to a license agreement with the
// AUTOSAR development cooperation.
// More information is available at www.autosar.org.
//
// Disclaimer
//
// This work (specification and/or software implementation) and the material
// contained in it, as released by AUTOSAR, is for the purpose of information
// only. AUTOSAR and the companies that have contributed to it shall not be
// liable for any use of the work.
//
// The material contained in this work is protected by copyright and other
// types of intellectual property rights. The commercial exploitation of the
// material contained in this work requires a license to such intellectual
// property rights.
//
// This work may be utilized or reproduced without any modification, in any
// form or by any means, for informational purposes only. For any other
// purpose, no part of the work may be utilized or reproduced, in any form
// or by any means, without permission in writing to the publisher.
//
// The work has been developed for automotive applications only. It has
// neither been developed, nor tested for non-automotive applications.
//
// The word AUTOSAR and the AUTOSAR logo are registered trademarks.
// --------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////
// Activity specific implementation, skeleton can be generated to the model
// Discovery of services and sending/receiving of data according
// to the communication API
///////////////////////////////////////////////////////////////////////

#include "work_information_subscriber.h"

#include <stdint.h>

#include <iomanip>
#include <cstdlib>
#include <exception>
#include <cassert>
#include <iostream>
#include <stdexcept>

// includes for used services
#include "adcm/work_information_proxy.h"

#include "ara/com/e2exf/types.h"
#include "ara/com/e2e_helper.h"

#include "ara/core/instance_specifier.h"
#include "logger.h"

using namespace std::chrono_literals;

namespace adcm
{

WorkInformation_Subscriber::WorkInformation_Subscriber()
{
    m_proxy = nullptr;
    DEBUG("object address %p", static_cast<void*>(this));
}

void WorkInformation_Subscriber::init(std::string instance)
{
    adcm::Log::Info() << "enter WorkInformation_Subscriber::init()";
    ara::core::InstanceSpecifier portSpecifier = ara::core::InstanceSpecifier(instance.c_str());
    INFO("Port In Executable Ref: %s", (std::string(portSpecifier.ToString().data())).c_str());
    auto instanceIDs = ara::com::runtime::ResolveInstanceIDs(portSpecifier);

    if(instanceIDs.empty()) {
        ERROR("Unknown Reference %s", instance.c_str());
        throw std::runtime_error{"No InstanceIdentifiers resolved to provided InstanceSpecifier"};
    }

    INFO("Searching for Service Instance: %s", (std::string(instanceIDs[0].ToString().data())).c_str());
    Proxy::StartFindService(
        [this](ara::com::ServiceHandleContainer<Proxy::HandleType> handles, ara::com::FindServiceHandle handler) {
            serviceAvailabilityCallback(std::move(handles), handler);
        },
        instanceIDs[0]);
    adcm::Log::Info() << "exit WorkInformation_Subscriber::init()";
}

void WorkInformation_Subscriber::serviceAvailabilityCallback(ara::com::ServiceHandleContainer<Proxy::HandleType> handles, ara::com::FindServiceHandle handler)
{
    (void)handler; // unused
    if (handles.empty()) {
        ERROR("No service handles found during discovery.");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(m_proxy_mutex);
        if (!m_proxy) {
            m_proxy = std::make_shared<Proxy>(handles[0]);
            INFO("Created proxy with instance: %s", m_proxy->GetHandle().GetInstanceId().ToString().data());
        }
    }
    setupSubscription();
}

void WorkInformation_Subscriber::setupSubscription()
{
    std::lock_guard<std::mutex> lock(m_proxy_mutex);
    if (m_proxy && !m_proxy->workInformationEvent.IsSubscribed()) {
        m_proxy->workInformationEvent.SetReceiveHandler([this]() {
            receivedCallback_workInformation();
        });
        m_proxy->workInformationEvent.Subscribe(1);
        INFO("WorkInformation_Subscriber::Subscribe() complete");
    }
}

void WorkInformation_Subscriber::receivedCallback_workInformation()
{
    // execute callback for every samples in the context of GetNewSamples
    std::shared_ptr<work_information_Objects> temp;
    auto callback = [&temp](auto sample) {
        // DEBUG("Callback: workInformationEvent ");
        if(sample){
            temp = std::make_shared<work_information_Objects>(*sample);
        }
    };
    m_proxy->workInformationEvent.GetNewSamples(callback);
    
    //if valid sample received, enQueue it
    if(temp) {
        this->enQueue(temp);
    } else {
        ERROR("No valid sample received. skip this event");
    }
}

int WorkInformation_Subscriber::getQueueSize()
{
    std::lock_guard<std::mutex> guard(m_Mutex_eventQueue);
    return static_cast<int>(m_Queue_event.size());
}

bool WorkInformation_Subscriber::waitEvent(int deadLine)
{
    std::unique_lock<std::mutex> lock(m_Mutex_eventQueue);
    // wait untile the event queue is not empty
    bool eventOccurred = m_cv_eventQueue.wait_for(lock, std::chrono::milliseconds(deadLine), [this] {
        return !m_Queue_event.empty();
    });

    return eventOccurred;
}
 
void WorkInformation_Subscriber::enQueue(std::shared_ptr<work_information_Objects> item)
{
    std::lock_guard<std::mutex> guard(m_Mutex_eventQueue);
    
    if(static_cast<int>(m_Queue_event.size()) >= MAX_QUEUE_LENGTH) {
        WARNNING("Queue length is too high!!(%d)", static_cast<int>(m_Queue_event.size()));
        m_Queue_event.pop(); // remove the oldest element
    } 
    
    m_Queue_event.push(item);
    
    if (!m_Queue_event.empty()) {
        m_cv_eventQueue.notify_one();
    }
}

bool WorkInformation_Subscriber::popEvent(std::shared_ptr<work_information_Objects>& event)
{
    bool result = false;
    
    std::lock_guard<std::mutex> lock(m_Mutex_eventQueue);
    if (!m_Queue_event.empty()) {
        event = m_Queue_event.front();
        m_Queue_event.pop();

        result = true;
    }

    return result;
}

} // namespace adcm
