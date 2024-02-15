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

#ifndef ADCM_PROXY_IMPL_BUILD_PATH_H_
#define ADCM_PROXY_IMPL_BUILD_PATH_H_

#include "service_desc_build_path.h"
#include "adcm/build_path_proxy.h"

namespace adcm
{
namespace build_path_binding
{
namespace vsomeip
{

class build_pathImpl
    : public proxy::build_pathProxyBase
    , public ::ara::com::internal::vsomeip::proxy::ProxyImplBase
{
public:
    using ProxyInterface = proxy::build_pathProxy;
    using ServiceDescriptor = descriptors::internal::Service;

    explicit build_pathImpl(ara::com::internal::vsomeip::types::InstanceId instance)
        : ::ara::com::internal::vsomeip::proxy::ProxyImplBase(ServiceDescriptor::service_id,
            instance,
            ServiceDescriptor::service_version_major,
            ServiceDescriptor::required_minimum_minor_version)
        , ServiceFlag(instance)
        , buildPathEvent(instance)
        , GetBuildPath(instance)
    { }

    virtual ~build_pathImpl()
    { }

    virtual proxy::fields::ServiceFlag& GetServiceFlag() override
    {
        return ServiceFlag;
    }

    virtual proxy::events::buildPathEvent& GetbuildPathEvent() override
    {
        return buildPathEvent;
    }

    virtual adcm::proxy::methods::GetBuildPath& GetGetBuildPath() override
    {
        return GetBuildPath;
    }

private:
    ara::com::internal::vsomeip::proxy::MutableFieldImpl<descriptors::ServiceFlag,
        ::String>
        ServiceFlag;
    ara::com::internal::vsomeip::proxy::EventImpl<descriptors::buildPathEvent,
        ::adcm::build_path_Objects>
        buildPathEvent;
    ara::com::internal::vsomeip::proxy::MethodImpl<descriptors::GetBuildPath,
        adcm::proxy::methods::GetBuildPath::signature_type>
        GetBuildPath;
};

}  // namespace vsomeip
}  // namespace build_path_binding
}  // namespace adcm

#endif  // ADCM_PROXY_IMPL_BUILD_PATH_H_
