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

#ifndef ADCM_MAP_DATA_PROXY_H_
#define ADCM_MAP_DATA_PROXY_H_

#include "ara/com/internal/proxy/ara_proxy_base.h"
#include "map_data_common.h"

namespace adcm
{
namespace proxy
{

namespace events
{
/// @uptrace{SWS_CM_00005, 0de7ac7dc8067e6ab2ad095a9097d02d83e7f83d}
using mapDataEvent = ara::com::internal::proxy::Event<::adcm::map_data_Objects>;
}  // namespace events

class map_dataProxyBase
    : public ara::com::internal::proxy::ProxyBindingBase
    , public adcm::map_data
{
public:
    virtual events::mapDataEvent& GetmapDataEvent() = 0;
};

/// @uptrace{SWS_CM_00004, bedfd6edb7e40a5e31639e01621800cfc077b498}
class map_dataProxy
    : public ara::com::internal::proxy::ProxyBase<map_dataProxyBase>
    , public adcm::map_data
{
public:
    /// @brief Proxy constructor.
    ///
    /// @uptrace{SWS_CM_00131, a4bb0196ec1ba17aed6d599f4dc3259f7b7ed02b}
    explicit map_dataProxy(const HandleType& proxy_base_factory)
        : ara::com::internal::proxy::ProxyBase<map_dataProxyBase>(proxy_base_factory)
        , mapDataEvent(proxy_base_->GetmapDataEvent())
    { }

    /// @brief Proxy shall be move constructable.
    ///
    /// @uptrace{SWS_CM_00137, c198eaaf815fde7c1ebe1f7c31a75bf1ccc57f19}
    explicit map_dataProxy(map_dataProxy&&) = default;

    /// @brief Proxy shall be move assignable.
    ///
    /// @uptrace{SWS_CM_00137, c198eaaf815fde7c1ebe1f7c31a75bf1ccc57f19}
    map_dataProxy& operator=(map_dataProxy&&) = default;

    /// @brief Proxy shall not be copy constructable.
    ///
    /// @uptrace{SWS_CM_00136, 2565b350c03a33dff1af61b8fc65aacbccb7ea9a}
    explicit map_dataProxy(const map_dataProxy&) = delete;

    /// @brief Proxy shall not be copy assignable.
    ///
    /// @uptrace{SWS_CM_00136, 2565b350c03a33dff1af61b8fc65aacbccb7ea9a}
    map_dataProxy& operator=(const map_dataProxy&) = delete;

    events::mapDataEvent& mapDataEvent;
};

}  // namespace proxy
}  // namespace adcm

#endif  // ADCM_MAP_DATA_PROXY_H_