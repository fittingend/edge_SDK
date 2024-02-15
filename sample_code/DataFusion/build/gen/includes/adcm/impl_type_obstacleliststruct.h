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

#ifndef ADCM_IMPL_TYPE_OBSTACLELISTSTRUCT_H_
#define ADCM_IMPL_TYPE_OBSTACLELISTSTRUCT_H_

#include "impl_type_map2dindexvector.h"
#include <cstdint>
namespace adcm
{

struct obstacleListStruct
{
    std::uint16_t obstacle_id;
    std::uint8_t obstacle_class;
    std::uint64_t timestamp;
    ::map2dIndexVector map_2d_location;
    std::uint8_t stop_count;
    double fused_cuboid_x;
    double fused_cuboid_y;
    double fused_cuboid_z;
    double fused_heading_angle;
    double fused_position_x;
    double fused_position_y;
    double fused_position_z;
    double fused_velocity_x;
    double fused_velocity_y;
    double fused_velocity_z;

    using IsEnumerableTag = void;
    template <typename F>
    void enumerate(F& fun)
    {
        fun(this->obstacle_id);
        fun(this->obstacle_class);
        fun(this->timestamp);
        fun(this->map_2d_location);
        fun(this->stop_count);
        fun(this->fused_cuboid_x);
        fun(this->fused_cuboid_y);
        fun(this->fused_cuboid_z);
        fun(this->fused_heading_angle);
        fun(this->fused_position_x);
        fun(this->fused_position_y);
        fun(this->fused_position_z);
        fun(this->fused_velocity_x);
        fun(this->fused_velocity_y);
        fun(this->fused_velocity_z);
    }
};
}  // namespace adcm

#endif  // ADCM_IMPL_TYPE_OBSTACLELISTSTRUCT_H_
