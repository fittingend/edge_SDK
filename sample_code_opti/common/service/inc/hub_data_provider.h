
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

#ifndef __HUB_DATA_PROVIDER_H__
#define __HUB_DATA_PROVIDER_H__

#include <string>
#include <random>
#include <memory>

#include "ara/core/future.h"
#include "ara/core/promise.h"

// includes for used services
#include "adcm/hub_data_skeleton.h"

namespace adcm
{

/*!
 *  \brief Class implementing skeleton methods.
 *
 *  hub_data's implementation.
 *
 *  \uptrace{SWS_CM_00191}
 */
class HubDataImp : public adcm::skeleton::hub_dataSkeleton
{
    using Skeleton = adcm::skeleton::hub_dataSkeleton;

public:
    HubDataImp(ara::core::InstanceSpecifier instanceSpec, ara::com::MethodCallProcessingMode mode)
        : Skeleton(std::move(instanceSpec), mode)
        , m_worker(&HubDataImp::ProcessRequests, this)
    {}

    virtual ~HubDataImp()
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
 *  \brief Class implementing HubData activity.
 *
 *  Radar activity implementing function of HubData.
 */
class HubData_Provider
{
public:
    HubData_Provider();
    ~HubData_Provider() = default;

    void init(std::string instance);

    void send(hub_data_Objects& data);

protected:

    std::unique_ptr<adcm::skeleton::hub_dataSkeleton> m_skeleton;
};

}// namespace adcm
#endif  // __HUB_DATA_PROVIDER_H__
