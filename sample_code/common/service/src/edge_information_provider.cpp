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

#include "edge_information_provider.h"

#include <stdint.h>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include "adcm/edge_information_skeleton.h"
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

void EdgeInformationImp::ProcessRequests()
{
    while(!m_finished) {
        std::chrono::time_point<std::chrono::system_clock> deadline
            = std::chrono::system_clock::now() + std::chrono::milliseconds(500);
        auto request_finished = ProcessNextMethodCall();
#if defined(R19_11_1)

        if(request_finished.wait_until(deadline) != ara::core::future_status::kReady) {
#else

        if(request_finished.wait_until(deadline) != ara::core::future_status::ready) {
#endif
            FATAL("Request took too long :S");

        } else {
            if(!m_finished) {
                std::this_thread::sleep_until(deadline);
            }
        }
    }
}

EdgeInformation_Provider::EdgeInformation_Provider()
{
    DEBUG("object address : %p", static_cast<void*>(this));
}

EdgeInformation_Provider::~EdgeInformation_Provider()
{
    delete m_skeleton;
}

void EdgeInformation_Provider::init(std::string instance)
{
    adcm::Log::Info() << "enter EdgeInformation_Provider::init()";
    ara::core::InstanceSpecifier instanceSpec = ara::core::InstanceSpecifier(instance.c_str());
    INFO("Port In Executable Ref: %s", instanceSpec.ToString().data());
    m_skeleton = new EdgeInformationImp(instanceSpec, ara::com::MethodCallProcessingMode::kPoll);
    // The instance id resolution is not mandatory for service creation (but could be an option)
    // here it's intended to list ids for the offered service instances
    auto instanceIDs = ara::com::runtime::ResolveInstanceIDs(instanceSpec);

    for(auto const& instanceId : instanceIDs) {
        INFO("Service Instance offered: %s", instanceId.ToString().data());
    }

    m_skeleton->OfferService();
    adcm::Log::Info() << "exit EdgeInformation_Provider::init()";
}

void EdgeInformation_Provider::send(edge_information_Objects& data)
{
    try {
        auto allocation = m_skeleton->edgeInformationEvent.Allocate();
        auto l_sampleData = std::move(allocation).Value();
        *l_sampleData = data;
        m_skeleton->edgeInformationEvent.Send(std::move(l_sampleData));
        // DEBUG("sent");

    } catch(ara::com::Exception e) {
        ERROR("Exeception : %s", e.what());
    }
}

}// namespace adcm
