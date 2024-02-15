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

#ifndef ADCM_IMPL_TYPE_VEHICLELISTSTRUCT_H_
#define ADCM_IMPL_TYPE_VEHICLELISTSTRUCT_H_

#include "impl_type_map2dindexvector.h"
#include <cstdint>
namespace adcm
{

struct vehicleListStruct
{
    std::uint8_t vehicle_class;
    std::uint64_t timestamp;
    ::map2dIndexVector map_2d_location;
    double position_long;
    double position_lat;
    double position_height;
    double position_x;
    double position_y;
    double position_z;
    double yaw;
    double roll;
    double pitch;
    double velocity_long;
    double velocity_lat;
    double velocity_x;
    double velocity_y;
    double velocity_ang;

    using IsEnumerableTag = void;
    template <typename F>
    void enumerate(F& fun)
    {
        fun(this->vehicle_class);
        fun(this->timestamp);
        fun(this->map_2d_location);
        fun(this->position_long);
        fun(this->position_lat);
        fun(this->position_height);
        fun(this->position_x);
        fun(this->position_y);
        fun(this->position_z);
        fun(this->yaw);
        fun(this->roll);
        fun(this->pitch);
        fun(this->velocity_long);
        fun(this->velocity_lat);
        fun(this->velocity_x);
        fun(this->velocity_y);
        fun(this->velocity_ang);
    }
};
}  // namespace adcm

#endif  // ADCM_IMPL_TYPE_VEHICLELISTSTRUCT_H_
