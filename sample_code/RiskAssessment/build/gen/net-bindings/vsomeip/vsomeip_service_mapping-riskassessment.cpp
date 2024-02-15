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

#include "ara/com/internal/vsomeip/vsomeip_service_mapping.h"
#include "ara/com/internal/vsomeip/vsomeip_service_mapping_impl.h"

#include "adcm/proxy_impl_build_path.h"
#include "adcm/proxy_impl_build_path_test.h"
#include "adcm/proxy_impl_map_data.h"
#include "adcm/adapter_risk_assessment.h"

namespace ara
{
namespace com
{
namespace internal
{
namespace vsomeip
{
namespace runtime
{

namespace
{

ServiceMappingImpl<adcm::risk_assessment::service_id,
    adcm::risk_assessment_binding::vsomeip::descriptors::internal::Service::service_id,
    adcm::risk_assessment_binding::vsomeip::descriptors::internal::Service::service_version_major,
    adcm::risk_assessment_binding::vsomeip::descriptors::internal::Service::service_version_minor,
    adcm::risk_assessment_binding::vsomeip::descriptors::internal::Service::required_minimum_minor_version,
    adcm::risk_assessment_binding::vsomeip::descriptors::internal::Service::is_minimum_minor_policy_enabled,
    adcm::risk_assessment_binding::vsomeip::descriptors::internal::Service::blacklisted_versions_length,
    adcm::risk_assessment_binding::vsomeip::descriptors::internal::Service::blacklisted_versions,
    NoProxy,
    adcm::risk_assessment_binding::vsomeip::risk_assessmentServiceAdapter>
    adcm__risk_assessment__mapping;

ServiceMappingImpl<adcm::build_path::service_id,
    adcm::build_path_binding::vsomeip::descriptors::internal::Service::service_id,
    adcm::build_path_binding::vsomeip::descriptors::internal::Service::service_version_major,
    adcm::build_path_binding::vsomeip::descriptors::internal::Service::service_version_minor,
    adcm::build_path_binding::vsomeip::descriptors::internal::Service::required_minimum_minor_version,
    adcm::build_path_binding::vsomeip::descriptors::internal::Service::is_minimum_minor_policy_enabled,
    adcm::build_path_binding::vsomeip::descriptors::internal::Service::blacklisted_versions_length,
    adcm::build_path_binding::vsomeip::descriptors::internal::Service::blacklisted_versions,
    adcm::build_path_binding::vsomeip::build_pathImpl,
    NoAdapter>
    adcm__build_path__mapping;

ServiceMappingImpl<adcm::build_path_test::service_id,
    adcm::build_path_test_binding::vsomeip::descriptors::internal::Service::service_id,
    adcm::build_path_test_binding::vsomeip::descriptors::internal::Service::service_version_major,
    adcm::build_path_test_binding::vsomeip::descriptors::internal::Service::service_version_minor,
    adcm::build_path_test_binding::vsomeip::descriptors::internal::Service::required_minimum_minor_version,
    adcm::build_path_test_binding::vsomeip::descriptors::internal::Service::is_minimum_minor_policy_enabled,
    adcm::build_path_test_binding::vsomeip::descriptors::internal::Service::blacklisted_versions_length,
    adcm::build_path_test_binding::vsomeip::descriptors::internal::Service::blacklisted_versions,
    adcm::build_path_test_binding::vsomeip::build_path_testImpl,
    NoAdapter>
    adcm__build_path_test__mapping;

ServiceMappingImpl<adcm::map_data::service_id,
    adcm::map_data_binding::vsomeip::descriptors::internal::Service::service_id,
    adcm::map_data_binding::vsomeip::descriptors::internal::Service::service_version_major,
    adcm::map_data_binding::vsomeip::descriptors::internal::Service::service_version_minor,
    adcm::map_data_binding::vsomeip::descriptors::internal::Service::required_minimum_minor_version,
    adcm::map_data_binding::vsomeip::descriptors::internal::Service::is_minimum_minor_policy_enabled,
    adcm::map_data_binding::vsomeip::descriptors::internal::Service::blacklisted_versions_length,
    adcm::map_data_binding::vsomeip::descriptors::internal::Service::blacklisted_versions,
    adcm::map_data_binding::vsomeip::map_dataImpl,
    NoAdapter>
    adcm__map_data__mapping;

}  // namespace

const VSomeIPServiceMapping::Mapping* VSomeIPServiceMapping::GetMappingForServiceId(ServiceId service_id)
{
    switch (service_id) {
    case adcm::risk_assessment::service_id:
        return &adcm__risk_assessment__mapping;
    case adcm::build_path::service_id:
        return &adcm__build_path__mapping;
    case adcm::build_path_test::service_id:
        return &adcm__build_path_test__mapping;
    case adcm::map_data::service_id:
        return &adcm__map_data__mapping;
    default:
        return nullptr;
    }
}

VSomeIPServiceMapping::MultiMapping VSomeIPServiceMapping::GetMappingForVSomeIPServiceId(
    ::vsomeip::service_t service_id)
{
    switch (service_id) {
    case adcm::risk_assessment_binding::vsomeip::descriptors::internal::Service::service_id:
        return {
            &adcm__risk_assessment__mapping,
        };
    case adcm::build_path_binding::vsomeip::descriptors::internal::Service::service_id:
        return {
            &adcm__build_path__mapping,
        };
    case adcm::build_path_test_binding::vsomeip::descriptors::internal::Service::service_id:
        return {
            &adcm__build_path_test__mapping,
        };
    case adcm::map_data_binding::vsomeip::descriptors::internal::Service::service_id:
        return {
            &adcm__map_data__mapping,
        };
    default:
        return {};
    }
}

}  // namespace runtime
}  // namespace vsomeip
}  // namespace internal
}  // namespace com
}  // namespace ara
