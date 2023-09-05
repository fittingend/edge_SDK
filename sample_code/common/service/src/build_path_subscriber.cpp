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

#include "build_path_subscriber.h"

#include <stdint.h>

#include <iomanip>
#include <cstdlib>
#include <exception>
#include <cassert>
#include <iostream>
#include <stdexcept>

// includes for used services
#include "adcm/build_path_proxy.h"

#include "ara/com/e2exf/types.h"
#include "ara/com/e2e_helper.h"

#include "ara/core/instance_specifier.h"
#include "logger.h"

using namespace std::chrono_literals;

namespace adcm
{

BuildPath_Subscriber::BuildPath_Subscriber()
{
    m_proxy = nullptr;
    m_promise_waitEvent = nullptr;
    DEBUG("object address %p", static_cast<void*>(this));
}

void BuildPath_Subscriber::init(std::string instance)
{
    adcm::Log::Info() << "enter BuildPath_Subscriber::init()";
    ara::core::InstanceSpecifier portSpecifier = ara::core::InstanceSpecifier(instance.c_str());
    INFO("Port In Executable Ref: %s", (std::string(portSpecifier.ToString().data())).c_str());
    auto instanceIDs = ara::com::runtime::ResolveInstanceIDs(portSpecifier);

    if(instanceIDs.empty()) {
        ERROR("Unknow Reference %s", instance.c_str());
        throw std::runtime_error{"No InstanceIdentifiers resolved to provided InstanceSpecifier"};
    }

    INFO("Searching for Service Instance: %s", (std::string(instanceIDs[0].ToString().data())).c_str());
    Proxy::StartFindService(
    [this](ara::com::ServiceHandleContainer<Proxy::HandleType> handles, ara::com::FindServiceHandle handler) {
        BuildPath_Subscriber::serviceAvailabilityCallback(std::move(handles), handler);
    },
    instanceIDs[0]);
    adcm::Log::Info() << "exit BuildPath_Subscriber::init()";
}

void BuildPath_Subscriber::subscribe_buildPath()
{
    if(!m_proxy->buildPathEvent.IsSubscribed()) {
        // m_proxy got initialized via callback.
        m_proxy->buildPathEvent.SetReceiveHandler([this]() {
            BuildPath_Subscriber::receivedCallback_buildPath();
        });
        // subscribe to event
        m_proxy->buildPathEvent.Subscribe(1);
        INFO("BuildPath_Subscriber::subscribe_buildPath() complete");
    }
}

void BuildPath_Subscriber::serviceAvailabilityCallback(ara::com::ServiceHandleContainer<Proxy::HandleType> handles,
        ara::com::FindServiceHandle handler)
{
    UNUSED(handler);

    for(auto it : handles) {
        INFO("Instance %s is available", (std::string(it.GetInstanceId().ToString().data())).c_str());
    }

    if(handles.size() > 0) {
        std::lock_guard<std::mutex> lock(m_proxy_mutex);

        if(nullptr == m_proxy) {
            m_proxy = std::make_shared<Proxy>(handles[0]);
            INFO("Created proxy to handle with instance: %s", (std::string(m_proxy->GetHandle().GetInstanceId().ToString().data())).c_str());
            // Construct some handles (implementation-specific).
            auto first_handle = handles[0];
            auto aux_handle = first_handle;

            for(auto current_handle : handles) {
                // Call equality operator.
                INFO("Check handle::operator==: %d", static_cast<uint8_t>(aux_handle == current_handle));
                // Call copy assignment operator.
                aux_handle = current_handle;
                // Call less-than operator.
                INFO("Check handle::operator<: %d", static_cast<uint8_t>(aux_handle < first_handle));
            }

            INFO("subscribe buildPath service");
            subscribe_buildPath();
        }
    }
}

void BuildPath_Subscriber::receivedCallback_buildPath()
{
    // execute callback for every samples in the context of GetNewSamples
    std::shared_ptr<build_path_Objects> temp;
    auto callback = [&temp](auto sample) {
        DEBUG("Callback: buildPathEvent ");
        temp = std::make_shared<build_path_Objects>(*sample);
    };
    m_proxy->buildPathEvent.GetNewSamples(callback);
    this->enQueue(temp);
}

int BuildPath_Subscriber::getQueueSize()
{
    std::lock_guard<std::mutex> guard(m_Mutex_eventQueue);
    return static_cast<int>(m_Queue_event.size());
}

auto BuildPath_Subscriber::setTrigger()
{
    std::lock_guard<std::mutex> guard(m_Mutex_eventQueue);
    m_promise_waitEvent = std::make_shared< ara::core::Promise<bool>>();
    return m_promise_waitEvent->get_future();
}

bool BuildPath_Subscriber::waitEvent(int deadLine)
{
    if(isEventQueueEmpty() && (deadLine != 0)) {
        auto future_wait = setTrigger();
        future_wait.wait_for(std::chrono::milliseconds(deadLine));
    }

    return !isEventQueueEmpty();
}


void BuildPath_Subscriber::enQueue(std::shared_ptr<build_path_Objects> item)
{
    std::lock_guard<std::mutex> guard(m_Mutex_eventQueue);
    m_Queue_event.push(item);


    if(static_cast<int>(m_Queue_event.size()) > 11) {
        WARNNING("Queue length is too high!!(%d)", static_cast<int>(m_Queue_event.size()));
    }


    if(m_promise_waitEvent != nullptr) {
        m_promise_waitEvent->set_value(true);
        m_promise_waitEvent = nullptr;
    }
}

std::shared_ptr<build_path_Objects> BuildPath_Subscriber::deQueue()
{
    std::shared_ptr<build_path_Objects> temp;
    std::lock_guard<std::mutex> guard(m_Mutex_eventQueue);
    temp = m_Queue_event.front();
    m_Queue_event.pop();
    return temp;
}

std::shared_ptr<build_path_Objects> BuildPath_Subscriber::getEvent()
{
    std::shared_ptr<build_path_Objects> result;

    if(isEventQueueEmpty()) {
        result = nullptr;

    } else {
        result = deQueue();
    }

    return result;
}

bool BuildPath_Subscriber::isEventQueueEmpty()
{
    std::lock_guard<std::mutex> guard(m_Mutex_eventQueue);
    if(static_cast<int>(m_Queue_event.size()) != 0)
    {
        return  false;
    }else{
        return true;
    }

}

} // namespace adcm
