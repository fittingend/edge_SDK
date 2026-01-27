
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

#ifndef __BUILD_PATH_PROVIDER_H__
#define __BUILD_PATH_PROVIDER_H__

#include <string>
#include <random>

#include "ara/core/future.h"
#include "ara/core/promise.h"

// includes for used services
#include "adcm/build_path_skeleton.h"

#include "ara/core/future.h"
#include "ara/core/promise.h"

namespace adcm
{

typedef void (*BuildPathCallback)(const std::vector<adcm::destinationStruct>& destination, const std::uint8_t& mve_type);
static BuildPathCallback mCallback = nullptr;
static std::shared_ptr<adcm::build_path::GetBuildPathOutput> output = std::make_shared<adcm::build_path::GetBuildPathOutput>();

/*!
 *  \brief Class implementing skeleton methods.
 *
 *  build_path's implementation.
 *
 *  \uptrace{SWS_CM_00191}
 */
class BuildPathImp : public adcm::skeleton::build_pathSkeleton
{
    using Skeleton = adcm::skeleton::build_pathSkeleton;

public:
    BuildPathImp(ara::core::InstanceSpecifier instanceSpec, ara::com::MethodCallProcessingMode mode)
        : Skeleton(std::move(instanceSpec), mode)
        , m_worker(&BuildPathImp::ProcessRequests, this)
    {}

    virtual ~BuildPathImp()
    {
        m_finished = true;
        m_worker.join();
    }

    virtual auto GetBuildPath(const std::vector<adcm::destinationStruct>& destination, const std::uint8_t& mve_type)
        -> decltype(Skeleton::GetBuildPath(destination, mve_type)) override;

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
 *  \brief Class implementing BuildPath activity.
 *
 *  Radar activity implementing function of BuildPath.
 */
class BuildPath_Provider
{
public:
    BuildPath_Provider();
    ~BuildPath_Provider();

    static void setCallback(BuildPathCallback cb);
    static std::shared_ptr<adcm::build_path::GetBuildPathOutput> getPtrOutput();

    void init(std::string instance);

    void send(build_path_Objects& data);

protected:

    adcm::skeleton::build_pathSkeleton* m_skeleton;
};

}// namespace adcm
#endif  // __BUILD_PATH_PROVIDER_H__
