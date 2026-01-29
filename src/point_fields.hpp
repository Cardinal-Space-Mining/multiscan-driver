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


// ------------ MultiScan Fields --------------------------
/*
    These fields are copied contiguously from the sick_scan_xd struct.
    Values are mutually exclusive and represent the "maximum" field that will
    be copied into the output
*/
#define MS_POINT_FIELD_ENABLE_UP_TO_XYZ       0x0  // just xyz (default value)
#define MS_POINT_FIELD_ENABLE_UP_TO_INTENSITY 0x1  // xyz, intensity
#define MS_POINT_FIELD_ENABLE_UP_TO_RANGE     0x2  // xyz, intensity, range
#define MS_POINT_FIELD_ENABLE_UP_TO_ANGULAR           \
    0x3  // xyz, intensity, range, azimuth, elevation
#define MS_POINT_FIELD_ENABLE_UP_TO_POINT_IDX                             \
    0x4  // xyz, intensity, range, azimuth, elevation, layer, echo, index
/*
    These fields are non-contiguous and are separately copied when enabled
    (not mutually exclusive)
*/
#define MS_POINT_FIELD_ENABLE_TS        0b01000
#define MS_POINT_FIELD_ENABLE_REFLECTOR 0b10000


// ------------ Example Configurations ---------------------
/*
    Copies all sick_scan_xd fields
*/
#define MS_POINT_FIELD_ENABLE_ALL                                       \
    (MS_POINT_FIELD_ENABLE_UP_TO_POINT_IDX | MS_POINT_FIELD_ENABLE_TS | \
     MS_POINT_FIELD_ENABLE_REFLECTOR)
/*
    Copies xyz, timestamp, and reflector data only
*/
#define MS_POINT_FIELD_ENABLE_XYZTR                               \
    (MS_POINT_FIELD_ENABLE_UP_TO_XYZ | MS_POINT_FIELD_ENABLE_TS | \
     MS_POINT_FIELD_ENABLE_REFLECTOR)
/*
    Copies xyz, polar coordinates, timestamp, and reflector
*/
#define MS_POINT_FIELD_ENABLE_XYZPTR                                  \
    (MS_POINT_FIELD_ENABLE_UP_TO_ANGULAR | MS_POINT_FIELD_ENABLE_TS | \
     MS_POINT_FIELD_ENABLE_REFLECTOR)


// ------------ Helpers ------------------------------------
#define POINT_FIELDS_HAVE_XYZ(fields) 1
#define POINT_FIELDS_HAVE_INTENSITY(fields)                       \
    (((fields) & 0b111) >= MS_POINT_FIELD_ENABLE_UP_TO_INTENSITY)
#define POINT_FIELDS_HAVE_RANGE(fields)                       \
    (((fields) & 0b111) >= MS_POINT_FIELD_ENABLE_UP_TO_RANGE)
#define POINT_FIELDS_HAVE_ANGULAR(fields)                       \
    (((fields) & 0b111) >= MS_POINT_FIELD_ENABLE_UP_TO_ANGULAR)
#define POINT_FIELDS_HAVE_METADATA(fields)                        \
    (((fields) & 0b111) >= MS_POINT_FIELD_ENABLE_UP_TO_POINT_IDX)
#define POINT_FIELDS_HAVE_TIMESTAMP(fields)     \
    (((fields) & MS_POINT_FIELD_ENABLE_TS) > 0)
#define POINT_FIELDS_HAVE_REFLECTOR(fields)            \
    (((fields) & MS_POINT_FIELD_ENABLE_REFLECTOR) > 0)
/*
    Count the number of fields that are contiguous given a field configuration.
    All fields are 4 bytes in size, so the number of contiguous bytes per point
    is this value times 4.
*/
#define COMPUTE_NUM_CONTIGUOUS_POINT_FIELDS(fields)                   \
    (POINT_FIELDS_HAVE_XYZ(fields) * 3 +     /* xyz */                \
     POINT_FIELDS_HAVE_INTENSITY(fields) +   /* intensity */          \
     POINT_FIELDS_HAVE_RANGE(fields) +       /* range */              \
     POINT_FIELDS_HAVE_ANGULAR(fields) * 2 + /* azimuth, elevation */ \
     POINT_FIELDS_HAVE_METADATA(fields) * 3  /* layer, echo, index */ \
    )
/*
    Count the total number of fields for a configuration.
    All fields are 4 bytes in length, so the number of bytes used per point is
    this value times 4.
*/
#define COMPUTE_NUM_POINT_FIELDS(fields)                           \
    (COMPUTE_NUM_CONTIGUOUS_POINT_FIELDS(fields) + /* see above */ \
     POINT_FIELDS_HAVE_TIMESTAMP(fields) *                         \
         2 + /* timestamp high and low bits (8 bytes total) */     \
     POINT_FIELDS_HAVE_REFLECTOR(fields) /* reflector */           \
    )
