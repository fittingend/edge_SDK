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
// or by any means, without permission in writing from the publisher.
//
// The work has been developed for automotive applications only. It has
// neither been developed, nor tested for non-automotive applications.
//
// The word AUTOSAR and the AUTOSAR logo are registered trademarks.
// --------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////
// Activity specific implementation, skeleton can be generated from the model
// Discovery of services and sending/receiving of data according
// to the communication API
///////////////////////////////////////////////////////////////////////

#include "build_path_test_provider.h"

#include <stdint.h>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <memory>
#include <utility>

#include <chrono>
#include <thread>
#include "adcm/build_path_test_skeleton.h"
#include "ara/com/com_error_domain.h"
#include "ara/core/instance_specifier.h"
#include "logger.h"

using namespace ara::log;

using ara::com::ComErrorDomainErrc;
//using adcm::skeleton::fields::UpdateRate;

// <<operator implementation for logging custom types
// similar handler is also available in fusion sources, so for real projects it make sense to
// have some common place where custom type log-handlers are provided.

namespace adcm
{

void BuildPathTestImp::ProcessRequests()
{
    constexpr auto period = std::chrono::milliseconds(500);

    // 다음 실행 시각을 현재 시각 + 주기로 설정
    auto next_time = std::chrono::steady_clock::now() + period;

    while (!m_finished)
    {
        // Record start time at each loop
        auto start_time = std::chrono::steady_clock::now();

        auto request_future = ProcessNextMethodCall();
        if (request_future.valid())
        {
            // Wait until next execution time
            auto status = request_future.wait_until(next_time);
            if (status != ara::core::future_status::ready)
            {
                // If next excexution is not reacy on time
                FATAL("ProcessRequests timeout");
            }
            else
            {
                DEBUG("Request finished successfully.");
            }
        }

        // After rest time is calculated, if rest time is exist, sleep until rest time
        auto now = std::chrono::steady_clock::now();
        if (now < next_time)
        {
            std::this_thread::sleep_until(next_time);
        }
        else
        {
            // Next Iteration is over
            DEBUG("Processing overrun: iteration took longer than the period");
        }

        // Next iteration time is calculated base on current time for avoiding over run
        next_time = std::chrono::steady_clock::now() + period;
    }
}

BuildPathTest_Provider::BuildPathTest_Provider()
{
    DEBUG("object address : %p", static_cast<void*>(this));
}

void BuildPathTest_Provider::init(std::string instance)
{
    adcm::Log::Info() << "enter BuildPathTest_Provider::init()";
    ara::core::InstanceSpecifier instanceSpec = ara::core::InstanceSpecifier(instance.c_str());
    INFO("Port In Executable Ref: %s", instanceSpec.ToString().data());
    m_skeleton = std::make_unique<BuildPathTestImp>(instanceSpec, ara::com::MethodCallProcessingMode::kPoll);

    // The instance id resolution is not mandatory for service creation (but could be an option)
    // here it's intended to list ids for the offered service instances
    auto instanceIDs = ara::com::runtime::ResolveInstanceIDs(instanceSpec);

    for(const auto& instanceId : instanceIDs) {
        INFO("Service Instance offered: %s", instanceId.ToString().data());
    }

    m_skeleton->OfferService();
    adcm::Log::Info() << "exit BuildPathTest_Provider::init()";
}

void BuildPathTest_Provider::send(build_path_test_Objects& data)
{
    try {
        auto allocation = m_skeleton->buildPathTestEvent.Allocate();
        auto l_sampleData = std::move(allocation).Value();
        *l_sampleData = data;
        m_skeleton->buildPathTestEvent.Send(std::move(l_sampleData));
        // DEBUG("sent");
    }
    catch (const ara::com::Exception &e)
    {
        ERROR("ara::com::Exception: %s", e.what());
    }
    catch (const std::exception &e)
    {
        ERROR("std::exception: %s", e.what());
    }
}

}// namespace adcm
