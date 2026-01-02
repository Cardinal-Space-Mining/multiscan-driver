/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/

#pragma once

#include "point_fields.hpp"

#include <sensor_msgs/msg/point_field.hpp>


#ifndef MS_DRIVER_POINT_TYPE_FIELDS
#define MS_DRIVER_POINT_TYPE_FIELDS     MS_POINT_FIELD_ENABLE_ALL
#endif


#if POINT_FIELDS_HAVE_INTENSITY(MS_DRIVER_POINT_TYPE_FIELDS)
    #define MS_DRIVER_POINT_FIELD_LIST_INTENSITY \
        sensor_msgs::msg::PointField{} \
            .set__name("intensity") \
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32) \
            .set__count(1) \
            .set__offset(12),
#else
    #define MS_DRIVER_POINT_FIELD_LIST_INTENSITY
#endif
#if POINT_FIELDS_HAVE_RANGE(MS_DRIVER_POINT_TYPE_FIELDS)
    #define MS_DRIVER_POINT_FIELD_LIST_RANGE \
        sensor_msgs::msg::PointField{} \
            .set__name("range") \
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32) \
            .set__count(1) \
            .set__offset(16),
#else
    #define MS_DRIVER_POINT_FIELD_LIST_RANGE
#endif
#if POINT_FIELDS_HAVE_ANGULAR(MS_DRIVER_POINT_TYPE_FIELDS)
    #define MS_DRIVER_POINT_FIELD_LIST_ANGULAR \
        sensor_msgs::msg::PointField{} \
            .set__name("azimuth") \
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32) \
            .set__count(1) \
            .set__offset(20), \
        sensor_msgs::msg::PointField{} \
            .set__name("elevation") \
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32) \
            .set__count(1) \
            .set__offset(24),
#else
    #define MS_DRIVER_POINT_FIELD_LIST_ANGULAR
#endif
#if POINT_FIELDS_HAVE_METADATA(MS_DRIVER_POINT_TYPE_FIELDS)
    #define MS_DRIVER_POINT_FIELD_LIST_METADATA \
        sensor_msgs::msg::PointField{} \
            .set__name("layer") \
            .set__datatype(sensor_msgs::msg::PointField::UINT32) \
            .set__count(1) \
            .set__offset(28), \
        sensor_msgs::msg::PointField{} \
            .set__name("echo") \
            .set__datatype(sensor_msgs::msg::PointField::UINT32) \
            .set__count(1) \
            .set__offset(32), \
        sensor_msgs::msg::PointField{} \
            .set__name("index") \
            .set__datatype(sensor_msgs::msg::PointField::UINT32) \
            .set__count(1) \
            .set__offset(36),
#else
    #define MS_DRIVER_POINT_FIELD_LIST_METADATA
#endif
#if POINT_FIELDS_HAVE_TIMESTAMP(MS_DRIVER_POINT_TYPE_FIELDS)
    #define MS_DRIVER_POINT_FIELD_LIST_TIMESTAMP \
        sensor_msgs::msg::PointField{} \
            .set__name("tl") \
            .set__datatype(sensor_msgs::msg::PointField::UINT32) \
            .set__count(1) \
            .set__offset(4 * COMPUTE_NUM_CONTIGUOUS_POINT_FIELDS(MS_DRIVER_POINT_TYPE_FIELDS)), \
        sensor_msgs::msg::PointField{} \
            .set__name("th") \
            .set__datatype(sensor_msgs::msg::PointField::UINT32) \
            .set__count(1) \
            .set__offset(4 * COMPUTE_NUM_CONTIGUOUS_POINT_FIELDS(MS_DRIVER_POINT_TYPE_FIELDS) + 4),
#else
    #define MS_DRIVER_POINT_FIELD_LIST_TIMESTAMP
#endif
#if POINT_FIELDS_HAVE_REFLECTOR(MS_DRIVER_POINT_TYPE_FIELDS)
    #define MS_DRIVER_POINT_FIELD_LIST_REFLECTOR \
        sensor_msgs::msg::PointField{} \
            .set__name("reflective") \
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32) \
            .set__count(1) \
            .set__offset( \
                (4 * COMPUTE_NUM_CONTIGUOUS_POINT_FIELDS(MS_DRIVER_POINT_TYPE_FIELDS)) + \
                (8 * POINT_FIELDS_HAVE_TIMESTAMP(MS_DRIVER_POINT_TYPE_FIELDS)) )
#else
    #define MS_DRIVER_POINT_FIELD_LIST_REFLECTOR
#endif

#define MS_DRIVER_POINT_FIELD_LIST \
    { \
        sensor_msgs::msg::PointField{} \
            .set__name("x") \
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32) \
            .set__count(1) \
            .set__offset(0), \
        sensor_msgs::msg::PointField{} \
            .set__name("y") \
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32) \
            .set__count(1) \
            .set__offset(4), \
        sensor_msgs::msg::PointField{} \
            .set__name("z") \
            .set__datatype(sensor_msgs::msg::PointField::FLOAT32) \
            .set__count(1) \
            .set__offset(8), \
        MS_DRIVER_POINT_FIELD_LIST_INTENSITY \
        MS_DRIVER_POINT_FIELD_LIST_RANGE \
        MS_DRIVER_POINT_FIELD_LIST_ANGULAR \
        MS_DRIVER_POINT_FIELD_LIST_METADATA \
        MS_DRIVER_POINT_FIELD_LIST_TIMESTAMP \
        MS_DRIVER_POINT_FIELD_LIST_REFLECTOR \
    }

// #undef MS_DRIVER_POINT_FIELD_LIST_INTENSITY
// #undef MS_DRIVER_POINT_FIELD_LIST_RANGE
// #undef MS_DRIVER_POINT_FIELD_LIST_ANGULAR
// #undef MS_DRIVER_POINT_FIELD_LIST_METADATA
// #undef MS_DRIVER_POINT_FIELD_LIST_TIMESTAMP
// #undef MS_DRIVER_POINT_FIELD_LIST_REFLECTOR
