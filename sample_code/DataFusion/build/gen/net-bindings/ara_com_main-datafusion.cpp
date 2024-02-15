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

#if defined(HAS_VSOMEIP_BINDING) && defined(HAS_WRSOMEIP_BINDING)
#    error HAS_VSOMEIP_BINDING and HAS_WRSOMEIP_BINDING are mutually exclusive
#endif

#include "ara/com/internal/ara_com_main.h"

#ifdef HAS_VSOMEIP_BINDING
#    include "ara/com/internal/vsomeip/vsomeip_binding.h"
#    include "ara/com/internal/vsomeip/vsomeip_error_domains.h"
#endif  // HAS_VSOMEIP_BINDING

#ifdef HAS_WRSOMEIP_BINDING
#    include "ara/com/internal/wrsomeip/wrsomeip_binding.h"
#    include "ara/com/internal/wrsomeip/wrsomeip_error_domains.h"
#endif  // HAS_WRSOMEIP_BINDING

#ifdef HAS_OPENDDS_BINDING
#    include "ara/com/internal/dds_idl/binding.h"
#    include "ara/com/internal/dds_idl/error_domains.h"
#endif  // HAS_OPENDDS_BINDING


namespace ara
{
namespace com
{
namespace internal
{
namespace runtime
{

void Initialize()
{
#ifdef HAS_VSOMEIP_BINDING
    vsomeip::runtime::Register();
#endif  // HAS_VSOMEIP_BINDING

#ifdef HAS_WRSOMEIP_BINDING
    ara::com::internal::wrsomeip::runtime::Register();
#endif  // HAS_WRSOMEIP_BINDING

#ifdef HAS_OPENDDS_BINDING
    dds::runtime::Register();
#endif  // HAS_OPENDDS_BINDING
}

}  // namespace runtime
}  // namespace internal
}  // namespace com
}  // namespace ara
