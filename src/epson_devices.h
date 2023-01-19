/**************************************************************************/
/*!
    @file     epson_devices.h

    Epson devices definitions

    @section  HISTORY

    v1.0 - First release
    v1.1 - Refactoring
    v1.2 - Remove unsupported IMU models
    v1.3 - Add models G370PDS0, G330PDG0, G366PDG0, refactor

    @section LICENSE

    Software License Agreement (BSD License, see license.txt)

    Copyright (c) 2019-2023 Seiko Epson Corporation.
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
#ifndef EPSON_DEVICES_H_
#define EPSON_DEVICES_H_

// Uncomment the selected IMU/Accel Model connected to the Arduino
// Then modify the user specific settings in the appropriate xxxx_epson_user_def.h file

/*** IMU ***/
// #define EPSON_G320
// #define EPSON_G354
// #define EPSON_G364PDC0
// #define EPSON_G364PDCA
// #define EPSON_V340
// #define EPSON_G365PDF1
// #define EPSON_G365PDC1
#define EPSON_G370PDF1
// #define EPSON_G370PDS0
// #define EPSON_G330PDG0
// #define EPSON_G366PDG0
/*** ACCEL ***/
// #define EPSON_A352AD

#if defined(EPSON_A352AD)
#include "accel_epson_M-Axxx.h"
#else
#include "imu_epson_M-Gxxx_M-Vxxx.h"
#endif

#endif //< EPSON_DEVICES_H_
