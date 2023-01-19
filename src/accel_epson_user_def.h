/**************************************************************************/
/*!
    @file     accel_epson_user_def.h

    Epson M-A3xx Accelerometer user configuration

    @section  HISTORY

    v1.0 - First release
    v1.1 - Comment update and copyright year

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
#ifndef EPSONACCL_USER_DEF_H_
#define EPSONACCL_USER_DEF_H_

// This file is used if EPSON__A352AD macro is defined in epson_devices.h

// Select common values for Filter and Output Rate
#define CMD_RATEX              CMD_RATE200        // check accel_epson_M-A352.h for model's supported rate
#define CMD_FILTERX            CMD_FIRTAP512FC60  // check accel_epson_M-A352.h for model's supported filter

// Select fields to enable in sensor output
#define ENABLE_FLAG           (1)    /* 0=disabled, 1=enabled */
#define ENABLE_TEMP           (1)    /* 0=disabled, 1=enabled */
#define ENABLE_ACCLX          (1)    /* 0=disabled, 1=enabled */
#define ENABLE_ACCLY          (1)    /* 0=disabled, 1=enabled */
#define ENABLE_ACCLZ          (1)    /* 0=disabled, 1=enabled */
#define ENABLE_COUNT          (1)    /* 0=disabled, 1=enabled */
#define ENABLE_CHKSM          (1)    /* 0=disabled, 1=enabled */

// Select tilt output on axis
// these have no effect if field is not enabled for output
#define TILTX                 (0)    /* 0=ACCL, 1=TILT */
#define TILTY                 (0)    /* 0=ACCL, 1=TILT */
#define TILTZ                 (0)    /* 0=ACCL, 1=TILT */

// Select bias and noise option
#define TEMP_SHOCK            (1)    /* 0=disabled, 1=enable thermal shock bias stabilization */
#define REDUCED_NOISE         (1)    /* 0=disabled, 1=enable reduced noise floor */


// Select EXT trigger option
#define ENABLE_EXT            (0)    /* EXT function: 0=External trigger disabled, 1=enabled */
#define POL_ACT_LOW           (0)    /* EXT polarity: 0=rising edge, 1=falling edge */


#endif /* EPSONACCL_USER_DEF_H_ */




