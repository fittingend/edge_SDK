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

/*!
 *  \file
 *  \brief Specification of perception activity.
 *
 *  \details Contains specification of perception activity.
 *  Stub for an activity, can be completely generated from the model. It includes the generic API of an activity and the
 * activity specific data access points (event sender/receiver, ...).
 */

#ifndef __RISK_ASSESSMENT_PROVIDER_H__
#define __RISK_ASSESSMENT_PROVIDER_H__

#include <string>
#include <random>

#include "ara/core/future.h"
#include "ara/core/promise.h"

// includes for used services
#include "adcm/risk_assessment_skeleton.h"

#include "ara/core/future.h"
#include "ara/core/promise.h"

namespace adcm
{

/*!
 *  \brief Class implementing skeleton methods.
 *
 *  risk_assessment's implementation.
 *
 *  \uptrace{SWS_CM_00191}
 */
class RiskAssessmentImp : public adcm::skeleton::risk_assessmentSkeleton
{
    using Skeleton = adcm::skeleton::risk_assessmentSkeleton;

public:
    RiskAssessmentImp(ara::core::InstanceSpecifier instanceSpec, ara::com::MethodCallProcessingMode mode)
        : Skeleton(std::move(instanceSpec), mode)
        , m_worker(&RiskAssessmentImp::ProcessRequests, this)
    {}

    virtual ~RiskAssessmentImp()
    {
        m_finished = true;
        m_worker.join();
    }

private:
    /*!
     * \brief Defines how the incoming service method invocations are processed.
     *
     * \uptrace{SWS_CM_00198}
     * \uptrace{SWS_CM_00199}
     */
    void ProcessRequests();

    std::atomic<bool> m_finished{false};
    std::thread m_worker;

};

/*!
 *  \brief Class implementing RiskAssessment activity.
 *
 *  Radar activity implementing function of RiskAssessment.
 */
class RiskAssessment_Provider
{
public:
    RiskAssessment_Provider();
    ~RiskAssessment_Provider();

    void init(std::string instance);

    void send(risk_assessment_Objects& data);

protected:

    adcm::skeleton::risk_assessmentSkeleton* m_skeleton;
};

}// namespace adcm
#endif  // __RISK_ASSESSMENT_PROVIDER_H__
