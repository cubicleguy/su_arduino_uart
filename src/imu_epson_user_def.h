/**************************************************************************/
/*!
    @file     imu_epson_user_def.h

    Epson M-Gxxx/M-Vxxx Series IMU user configuration

    @section  HISTORY

    v1.0 - First release
    v1.1 - Added 16G macro for G330/G366

    @section LICENSE

    Software License Agreement (BSD License, see license.txt)

    Copyright (c) 2022-2023 Seiko Epson Corporation.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
    THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
    PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
    OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
    OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
    ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#ifndef EPSONGXXX_VXXX_USER_DEF_H_
#define EPSONGXXX_VXXX_USER_DEF_H_

// This file is used if EPSON__A352AD macro is NOT defined in epson_devices.h

// Select common values for Filter and Output Rate
#define CMD_RATEX              CMD_RATE250  // check imu_epson_M-Gxxx_M-Vxxx.h for model's supported rate
#define CMD_FILTERX            CMD_FLTAP32  // check imu_epson_M-Gxxx_M-Vxxx.h for model's supported filter

// Select counter output field options
#define ENABLE_COUNT          (1)    /* 0=disabled, 1=enabled */
#define ENABLE_COUNTER_RESET  (0)    /* 0=COUNT field is sample counter, 1=COUNT field is counter reset */


// The below settings are NOT supported for V340 (ignored) which has fixed output fields

// Select fields to enable in sensor output
#define ENABLE_TEMP           (1)    /* 0=disabled, 1=enabled */
#define ENABLE_GYRO           (1)    /* 0=disabled, 1=enabled */
#define ENABLE_ACCL           (1)    /* 0=disabled, 1=enabled */
#define ENABLE_TEMP_32        (0)    /* 0=16bit, 1=32bit */
#define ENABLE_GYRO_32        (1)    /* 0=16bit, 1=32bit */
#define ENABLE_ACCL_32        (1)    /* 0=16bit, 1=32bit */

#define ENABLE_FLAG           (1)    /* 0=disabled, 1=enabled */
#define ENABLE_GPIO           (0)    /* 0=disabled, 1=enabled */
#define ENABLE_CHKSM          (1)    /* 0=disabled, 1=enabled */

// NOTE: DLTA/DLTV cannot be enabled if ATTI or QTN is enabled
#define ENABLE_DLTA           (0)    /* 0=disabled, 1=enabled */
#define ENABLE_DLTV           (0)    /* 0=disabled, 1=enabled */
#define ENABLE_DLTA_32        (0)    /* 0=16bit, 1=32bit */
#define ENABLE_DLTV_32        (0)    /* 0=16bit, 1=32bit */


// The below settings are only supported for G365

// Select QTN or ATTI settings for G365 only
// DLTA/DLTV, ATTI, QTN are mutually exclusive (only one can be enabled at the same time)
#define ENABLE_QTN            (0)    /* 0=disabled, 1=enabled */
#define ENABLE_QTN_32         (0)    /* 0=16bit, 1=32bit */
#define ENABLE_ATTI           (0)    /* 0=disabled, 1=enabled */
#define ENABLE_ATTI_32        (0)    /* 0=16bit, 1=32bit */
#define ATTI_EULER            (0)    /* 0=Inclination Angle, 1=Euler Angle */
#define ATTI_MOTION           (0)    /* 0=modeA, 1=modeB, 2=modeC */
#define ATTI_CNV              (0)    /* 0x00 ~ 0x17 for Attitude Output Axis Conversion */

// The below settings are only supported for G330 or G366
#define ENABLE_16G            (0)    /* 0=8G, 1=16G accelerometer output range */

#endif //< EPSONGXXX_VXXX_USER_DEF_H_
