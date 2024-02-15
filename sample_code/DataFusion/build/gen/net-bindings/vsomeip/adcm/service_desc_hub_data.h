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

#ifndef ADCM_SERVICE_DESC_HUB_DATA_H_
#define ADCM_SERVICE_DESC_HUB_DATA_H_

#include "adcm/hub_data_common.h"

#include "ara/com/internal/vsomeip/vsomeip_types.h"

namespace adcm
{
namespace hub_data_binding
{
namespace vsomeip
{
namespace descriptors
{

namespace internal
{
struct Service
{
    static constexpr ara::com::internal::vsomeip::types::ServiceId service_id
        = 0x7d1;
    static constexpr ara::com::internal::vsomeip::types::ServiceVersionMajor service_version_major
        = 0x1;
    static constexpr ara::com::internal::vsomeip::types::ServiceVersionMinor service_version_minor
        = 0x0;
    static constexpr ara::com::internal::vsomeip::types::ServiceVersionMinor required_minimum_minor_version
        = 0;
    static constexpr bool is_minimum_minor_policy_enabled = true;
    static constexpr uint32_t blacklisted_versions_length = 0;
    static constexpr ara::com::internal::vsomeip::types::BlacklistedVersion blacklisted_versions[]{
        // Just a placeholder due to blacklisted versions absence.
        // Value will never be used but required for compilation.
        {0x0, 0x0}
    };
};
}  // namespace internal

struct hubDataEvent : public internal::Service
{
    static constexpr ara::com::internal::vsomeip::types::EventId event_id
        = 0x8001;
    static constexpr ara::com::internal::vsomeip::types::EventGroupId event_groups[]
        {0x1};
};

}  // namespace descriptors
}  // namespace vsomeip
}  // namespace hub_data_binding
}  // namespace adcm

#endif  // ADCM_SERVICE_DESC_HUB_DATA_H_
