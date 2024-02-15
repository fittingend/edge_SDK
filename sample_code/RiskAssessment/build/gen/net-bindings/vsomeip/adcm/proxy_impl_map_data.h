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

#ifndef ADCM_PROXY_IMPL_MAP_DATA_H_
#define ADCM_PROXY_IMPL_MAP_DATA_H_

#include "service_desc_map_data.h"
#include "adcm/map_data_proxy.h"

namespace adcm
{
namespace map_data_binding
{
namespace vsomeip
{

class map_dataImpl
    : public proxy::map_dataProxyBase
    , public ::ara::com::internal::vsomeip::proxy::ProxyImplBase
{
public:
    using ProxyInterface = proxy::map_dataProxy;
    using ServiceDescriptor = descriptors::internal::Service;

    explicit map_dataImpl(ara::com::internal::vsomeip::types::InstanceId instance)
        : ::ara::com::internal::vsomeip::proxy::ProxyImplBase(ServiceDescriptor::service_id,
            instance,
            ServiceDescriptor::service_version_major,
            ServiceDescriptor::required_minimum_minor_version)
        , mapDataEvent(instance)
    { }

    virtual ~map_dataImpl()
    { }

    virtual proxy::events::mapDataEvent& GetmapDataEvent() override
    {
        return mapDataEvent;
    }

private:
    ara::com::internal::vsomeip::proxy::EventImpl<descriptors::mapDataEvent,
        ::adcm::map_data_Objects>
        mapDataEvent;
};

}  // namespace vsomeip
}  // namespace map_data_binding
}  // namespace adcm

#endif  // ADCM_PROXY_IMPL_MAP_DATA_H_
