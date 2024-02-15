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

#ifndef ADCM_ADAPTER_RISK_ASSESSMENT_H_
#define ADCM_ADAPTER_RISK_ASSESSMENT_H_

#include "ara/com/internal/vsomeip/skeleton/vsomeip_skeleton_base.h"
#include "service_desc_risk_assessment.h"
#include "adcm/risk_assessment_skeleton.h"

namespace adcm
{
namespace risk_assessment_binding
{
namespace vsomeip
{

class risk_assessmentServiceAdapter : public ::ara::com::internal::vsomeip::skeleton::ServiceImplBase
{
public:
    using ServiceInterface = skeleton::risk_assessmentSkeleton;
    using ServiceDescriptor = descriptors::internal::Service;

    risk_assessmentServiceAdapter(ServiceInterface& service,
        ara::com::internal::vsomeip::types::InstanceId instance)
        : ::ara::com::internal::vsomeip::skeleton::ServiceImplBase(service, instance)
        , riskAssessmentEvent(instance)
    {
        Connect(service);
        OfferService(ServiceDescriptor::service_id,
            GetInstanceId(),
            ServiceDescriptor::service_version_major,
            ServiceDescriptor::service_version_minor);
        OfferEvent(ServiceDescriptor::service_id,
            GetInstanceId(),
            descriptors::riskAssessmentEvent::event_id,
            descriptors::riskAssessmentEvent::event_groups);
    }

    virtual ~risk_assessmentServiceAdapter()
    {
        StopOfferEvent(descriptors::riskAssessmentEvent::service_id,
            GetInstanceId(),
            descriptors::riskAssessmentEvent::event_id);
        StopOfferService(ServiceDescriptor::service_id, GetInstanceId());
        Disconnect(dynamic_cast<ServiceInterface&>(service_));
    }

private:
    void Connect(ServiceInterface& service)
    {
        service.AddDelegate(*this);
        service.riskAssessmentEvent.AddDelegate(riskAssessmentEvent);
    }

    void Disconnect(ServiceInterface& service)
    {
        service.RemoveDelegate(*this);
        service.riskAssessmentEvent.RemoveDelegate(riskAssessmentEvent);
    }

    ara::com::internal::vsomeip::skeleton::EventImpl<::adcm::risk_assessment_Objects, descriptors::riskAssessmentEvent>
        riskAssessmentEvent;
};

}  // namespace vsomeip
}  // namespace risk_assessment_binding
}  // namespace adcm

#endif  // ADCM_ADAPTER_RISK_ASSESSMENT_H_
