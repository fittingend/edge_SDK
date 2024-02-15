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

#ifndef ADCM_IMPL_TYPE_RISKASSESSMENTSTRUCT_H_
#define ADCM_IMPL_TYPE_RISKASSESSMENTSTRUCT_H_

#include "impl_type_globalpathpositionvector.h"
#include <cstdint>
namespace adcm
{

struct riskAssessmentStruct
{
    std::uint16_t obstacle_id;
    ::globalPathPositionVector wgs84_xy_start;
    ::globalPathPositionVector wgs84_xy_end;
    std::uint8_t hazard_class;
    bool isHarzard;
    float confidence;

    using IsEnumerableTag = void;
    template <typename F>
    void enumerate(F& fun)
    {
        fun(this->obstacle_id);
        fun(this->wgs84_xy_start);
        fun(this->wgs84_xy_end);
        fun(this->hazard_class);
        fun(this->isHarzard);
        fun(this->confidence);
    }
};
}  // namespace adcm

#endif  // ADCM_IMPL_TYPE_RISKASSESSMENTSTRUCT_H_
