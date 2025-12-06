/**************************************************************************/
/*!
    @file     epson_vibe_regs.h

    Epson vibration sensor register definitions

    @section  HISTORY

    v1.0 - First release

    @section LICENSE

    Software License Agreement (BSD License, see license.txt)

    Copyright (c) 2025 Seiko Epson Corporation.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

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
#pragma once

#include <stdbool.h>
#include <stdint.h>

/*                                      -- Commands --
    - ADDR_ address byte of transfer to select the register to access
    - CMD_  data byte of transfer to write to the register selected
    - VAL_  data byte of transfer read from the register selected

    - All accesses are 16 bit transfers
    - For UART IF:
        - For UART write accesses - 8-bit address with msb=1b(can be even or
          odd) + 8-bit write data + Delimiter Byte
                                  - No response
        - For UART read accesses - 8-bit address with msb=0b(even only) + 8-bit
          dummy data + Delimiter Byte
            - Response is transferred immediately
            - Response consists of Register Read Address +
                          16-bit read data (high byte + low byte) + Delimiter
   Byte

    - NOTE: Register Address Maps depend on the WINDOW_ID (page)

*/

namespace EPSON_V_U {

// WINDOW_ID 0
constexpr uint8_t ADDR_BURST = 0x00;
constexpr uint8_t ADDR_MODE_CTRL_LO = 0x02;
constexpr uint8_t ADDR_MODE_CTRL_HI = 0x03;
constexpr uint8_t ADDR_DIAG_STAT1 = 0x04;
constexpr uint8_t ADDR_FLAG = 0x06;
constexpr uint8_t ADDR_COUNT = 0x0A;
constexpr uint8_t ADDR_DIAG_STAT2 = 0x0C;
constexpr uint8_t ADDR_TEMP1 = 0x10;
constexpr uint8_t ADDR_ACC_SELFTEST_DATA1_LO = 0x2A;
constexpr uint8_t ADDR_ACC_SELFTEST_DATA1_HI = 0x2B;
constexpr uint8_t ADDR_ACC_SELFTEST_DATA2_LO = 0x2C;
constexpr uint8_t ADDR_ACC_SELFTEST_DATA2_HI = 0x2D;
constexpr uint8_t ADDR_TEMP2 = 0x2E;
constexpr uint8_t ADDR_XVELC_HIGH = 0x30;
constexpr uint8_t ADDR_XVELC_LOW = 0x32;
constexpr uint8_t ADDR_YVELC_HIGH = 0x34;
constexpr uint8_t ADDR_YVELC_LOW = 0x36;
constexpr uint8_t ADDR_ZVELC_HIGH = 0x38;
constexpr uint8_t ADDR_ZVELC_LOW = 0x3A;

constexpr uint8_t ADDR_ID = 0x4C;

// WINDOW_ID 1
constexpr uint8_t ADDR_SIG_CTRL_LO = 0x00;
constexpr uint8_t ADDR_SIG_CTRL_HI = 0x01;
constexpr uint8_t ADDR_MSC_CTRL_LO = 0x02;
constexpr uint8_t ADDR_MSC_CTRL_HI = 0x03;
constexpr uint8_t ADDR_SMPL_CTRL_LO = 0x04;
constexpr uint8_t ADDR_SMPL_CTRL_HI = 0x05;
constexpr uint8_t ADDR_UART_CTRL_LO = 0x08;
constexpr uint8_t ADDR_UART_CTRL_HI = 0x09;
constexpr uint8_t ADDR_GLOB_CMD_LO = 0x0A;
constexpr uint8_t ADDR_BURST_CTRL_LO = 0x0C;
constexpr uint8_t ADDR_BURST_CTRL_HI = 0x0D;
constexpr uint8_t ADDR_ALIGNMENT_COEFF_CMD = 0x38;
constexpr uint8_t ADDR_ALIGNMENT_COEFF_DATA = 0x3A;
constexpr uint8_t ADDR_ALIGNMENT_COEFF_ADDR = 0x3C;
constexpr uint8_t ADDR_X_ALARM_LO = 0x46;
constexpr uint8_t ADDR_X_ALARM_HI = 0x47;
constexpr uint8_t ADDR_Y_ALARM_LO = 0x48;
constexpr uint8_t ADDR_Y_ALARM_HI = 0x49;
constexpr uint8_t ADDR_Z_ALARM_LO = 0x4A;
constexpr uint8_t ADDR_Z_ALARM_HI = 0x4B;

constexpr uint8_t ADDR_PROD_ID1 = 0x6A;
constexpr uint8_t ADDR_PROD_ID2 = 0x6C;
constexpr uint8_t ADDR_PROD_ID3 = 0x6E;
constexpr uint8_t ADDR_PROD_ID4 = 0x70;
constexpr uint8_t ADDR_VERSION = 0x72;
constexpr uint8_t ADDR_SERIAL_NUM1 = 0x74;
constexpr uint8_t ADDR_SERIAL_NUM2 = 0x76;
constexpr uint8_t ADDR_SERIAL_NUM3 = 0x78;
constexpr uint8_t ADDR_SERIAL_NUM4 = 0x7A;
constexpr uint8_t ADDR_WIN_CTRL = 0x7E;

// Write value to Issue Burst Read
constexpr uint8_t CMD_BURST = 0x80;
// Write value for WIN_CTRL to change to Window 0
constexpr uint8_t CMD_WINDOW0 = 0x00;
// Write value for WIN_CTRL to change to Window 1
constexpr uint8_t CMD_WINDOW1 = 0x01;
// Write value for MODE_CMD_HI to begin sampling
constexpr uint8_t CMD_BEGIN_SAMPLING = 0x01;
// Write value for MODE_CMD_HI to stop sampling
constexpr uint8_t CMD_END_SAMPLING = 0x02;
// Write value for MODE_CMD_HI to go to the Sleep Mode
constexpr uint8_t CMD_GOTO_SLEEP_MODE = 0x03;
// Write value for MSC_CTRL_HI to issue Structural Resonance Test
constexpr uint8_t CMD_EXITEST = 0x80;
// Write value for MSC_CTRL_HI to issue Flash Test
constexpr uint8_t CMD_FLASHTEST = 0x08;
// Does ACCTEST, TEMPTEST and VDDTEST for selftest
constexpr uint8_t CMD_SELFTEST = 0x07;
// Write value for MSC_CTRL_HI to issue Accelerometer Test
constexpr uint8_t CMD_ACCTEST = 0x04;
// Write value for MSC_CTRL_HI to issue Temp Sensor Test
constexpr uint8_t CMD_TEMPTEST = 0x02;
// Write value for MSC_CTRL_HI to issue Voltage Test
constexpr uint8_t CMD_VDDTEST = 0x01;
// Write value for GLOB_CMD_LO to issue Software Reset
constexpr uint8_t CMD_SOFTRESET = 0x80;
// Write value for GLOB_CMD_LO to issue Flash Backup
constexpr uint8_t CMD_FLASHBKUP = 0x08;
// Write value for GLOB_CMD_LO to issue Flash Reset
constexpr uint8_t CMD_FLASHRST = 0x04;

// Write value for UART_CTRL_LO to enable UART_AUTO mode and set
// AUTO_START = disabled
constexpr uint8_t CMD_UART_AUTO_EN = 0x01;
// Write value for UART_CTRL_LO to disable UART_AUTO mode and set
// AUTO_START = disabled
constexpr uint8_t CMD_UART_AUTO_DIS = 0x00;

// Write values for ADDR_FIR_UCMD
// These values are used to read/write FIR user coefficient values
constexpr uint8_t CMD_USERFIR_READ = 0x01;
constexpr uint8_t CMD_USERFIR_WRITE = 0x02;

// MODE STAT
constexpr uint8_t VAL_SAMPLING_MODE = 0x00;

// Other Values
constexpr uint16_t VAL_CONFIG_MASK = 0x0C00;
constexpr uint16_t VAL_CONFIG_MODE = 0x0400;
constexpr uint16_t VAL_SLEEP_MODE = 0x0800;
constexpr uint16_t VAL_NOT_READY = 0x0400;
constexpr uint16_t VAL_FILTER_STAT_BIT = 0x0020;
constexpr uint16_t VAL_SELF_TEST_BIT = 0x0700;
constexpr uint16_t VAL_FLASH_STATUS_BIT = 0x0800;
constexpr uint16_t VAL_DIAG_FLASHBU_ERROR = 0x0001;
constexpr uint16_t VAL_DIAG_ST_ERR_ALL = 0x0002;
constexpr uint16_t VAL_DIAG_FLASH_ERR = 0x0004;
constexpr uint16_t VAL_DIAG_STAT_MASK = 0xF302;
constexpr uint16_t VAL_DIAG_STAT2_MASK = 0x3F00;
constexpr uint16_t VAL_BURST_CTRL_MASK = 0xC703;
}  // namespace EPSON_V_U
