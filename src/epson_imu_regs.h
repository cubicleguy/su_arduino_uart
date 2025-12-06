/**************************************************************************/
/*!
    @file     epson_imu_regs.h

    Epson IMU specific register definitions

    @section  HISTORY

    v1.0 - First release restructure

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
                                 - Response consists of Register Read
   Address + 16-bit read data (high byte + low byte) + Delimiter Byte

    - NOTE: Register Address Maps depend on the WINDOW_ID (page)

*/

namespace EPSON_I_U {

// WINDOW_ID 0
constexpr uint8_t ADDR_MODE_CTRL_LO = 0x02;
constexpr uint8_t ADDR_MODE_CTRL_HI = 0x03;
constexpr uint8_t ADDR_DIAG_STAT = 0x04;
constexpr uint8_t ADDR_FLAG = 0x06;
constexpr uint8_t ADDR_GPIO = 0x08;
constexpr uint8_t ADDR_COUNT = 0x0A;

constexpr uint8_t ADDR_RANGE_OVER = 0x0C;
constexpr uint8_t ADDR_TEMP_HIGH = 0x0E;
constexpr uint8_t ADDR_TEMP_LOW = 0x10;
constexpr uint8_t ADDR_XGYRO_HIGH = 0x12;
constexpr uint8_t ADDR_XGYRO_LOW = 0x14;
constexpr uint8_t ADDR_YGYRO_HIGH = 0x16;
constexpr uint8_t ADDR_YGYRO_LOW = 0x18;
constexpr uint8_t ADDR_ZGYRO_HIGH = 0x1A;
constexpr uint8_t ADDR_ZGYRO_LOW = 0x1C;
constexpr uint8_t ADDR_XACCL_HIGH = 0x1E;
constexpr uint8_t ADDR_XACCL_LOW = 0x20;
constexpr uint8_t ADDR_YACCL_HIGH = 0x22;
constexpr uint8_t ADDR_YACCL_LOW = 0x24;
constexpr uint8_t ADDR_ZACCL_HIGH = 0x26;
constexpr uint8_t ADDR_ZACCL_LOW = 0x28;

constexpr uint8_t ADDR_RT_DIAG = 0x2B;

constexpr uint8_t ADDR_ID = 0x4C;

constexpr uint8_t ADDR_QTN0_HIGH = 0x50;
constexpr uint8_t ADDR_QTN0_LOW = 0x52;
constexpr uint8_t ADDR_QTN1_HIGH = 0x54;
constexpr uint8_t ADDR_QTN1_LOW = 0x56;
constexpr uint8_t ADDR_QTN2_HIGH = 0x58;
constexpr uint8_t ADDR_QTN2_LOW = 0x5A;
constexpr uint8_t ADDR_QTN3_HIGH = 0x5C;
constexpr uint8_t ADDR_QTN3_LOW = 0x5E;

constexpr uint8_t ADDR_ANG1_HIGH = 0x64;
constexpr uint8_t ADDR_ANG1_LOW = 0x66;
constexpr uint8_t ADDR_ANG2_HIGH = 0x68;
constexpr uint8_t ADDR_ANG2_LOW = 0x6A;
constexpr uint8_t ADDR_ANG3_HIGH = 0x6C;
constexpr uint8_t ADDR_ANG3_LOW = 0x6E;

constexpr uint8_t ADDR_XDLTA_HIGH = 0x64;
constexpr uint8_t ADDR_XDLTA_LOW = 0x66;
constexpr uint8_t ADDR_YDLTA_HIGH = 0x68;
constexpr uint8_t ADDR_YDLTA_LOW = 0x6A;
constexpr uint8_t ADDR_ZDLTA_HIGH = 0x6C;
constexpr uint8_t ADDR_ZDLTA_LOW = 0x6E;
constexpr uint8_t ADDR_XDLTV_HIGH = 0x70;
constexpr uint8_t ADDR_XDLTV_LOW = 0x72;
constexpr uint8_t ADDR_YDLTV_HIGH = 0x74;
constexpr uint8_t ADDR_YDLTV_LOW = 0x76;
constexpr uint8_t ADDR_ZDLTV_HIGH = 0x78;
constexpr uint8_t ADDR_ZDLTV_LOW = 0x7A;

// WINDOW_ID 1
constexpr uint8_t ADDR_SIG_CTRL_LO = 0x00;
constexpr uint8_t ADDR_SIG_CTRL_HI = 0x01;
constexpr uint8_t ADDR_MSC_CTRL_LO = 0x02;
constexpr uint8_t ADDR_MSC_CTRL_HI = 0x03;
constexpr uint8_t ADDR_SMPL_CTRL_LO = 0x04;
constexpr uint8_t ADDR_SMPL_CTRL_HI = 0x05;
constexpr uint8_t ADDR_FILTER_CTRL_LO = 0x06;
constexpr uint8_t ADDR_FILTER_CTRL_HI = 0x07;
constexpr uint8_t ADDR_UART_CTRL_LO = 0x08;
constexpr uint8_t ADDR_UART_CTRL_HI = 0x09;
constexpr uint8_t ADDR_GLOB_CMD_LO = 0x0A;
constexpr uint8_t ADDR_GLOB_CMD_HI = 0x0B;
constexpr uint8_t ADDR_BURST_CTRL1_LO = 0x0C;
constexpr uint8_t ADDR_BURST_CTRL1_HI = 0x0D;
constexpr uint8_t ADDR_BURST_CTRL2_LO = 0x0E;
constexpr uint8_t ADDR_BURST_CTRL2_HI = 0x0F;
constexpr uint8_t ADDR_POL_CTRL_LO = 0x10;
constexpr uint8_t ADDR_POL_CTRL_HI = 0x11;

constexpr uint8_t ADDR_DLT_CTRL_LO = 0x12;
// DLT_CTRL Byte1 (W1) called GLOB_CMD3 for G330/G366/G370PDG0/G370PDT0
constexpr uint8_t ADDR_DLT_CTRL_HI = 0x13;
// GLOB_CMD3 Byte0 (W1) Same address as DLT_CTRL
constexpr uint8_t ADDR_GLOB_CMD3_LO = 0x12;
constexpr uint8_t ADDR_GLOB_CMD3_HI = 0x13;

constexpr uint8_t ADDR_ATTI_CTRL_LO = 0x14;
constexpr uint8_t ADDR_ATTI_CTRL_HI = 0x15;
constexpr uint8_t ADDR_GLOB_CMD2_LO = 0x16;
constexpr uint8_t ADDR_GLOB_CMD2_HI = 0x17;

constexpr uint8_t ADDR_R_MATRIX_G_M11 = 0x38;
constexpr uint8_t ADDR_R_MATRIX_G_M12 = 0x3A;
constexpr uint8_t ADDR_R_MATRIX_G_M13 = 0x3C;
constexpr uint8_t ADDR_R_MATRIX_G_M21 = 0x3E;
constexpr uint8_t ADDR_R_MATRIX_G_M22 = 0x40;
constexpr uint8_t ADDR_R_MATRIX_G_M23 = 0x42;
constexpr uint8_t ADDR_R_MATRIX_G_M31 = 0x44;
constexpr uint8_t ADDR_R_MATRIX_G_M32 = 0x46;
constexpr uint8_t ADDR_R_MATRIX_G_M33 = 0x48;

constexpr uint8_t ADDR_R_MATRIX_A_M11 = 0x4A;
constexpr uint8_t ADDR_R_MATRIX_A_M12 = 0x4C;
constexpr uint8_t ADDR_R_MATRIX_A_M13 = 0x4E;
constexpr uint8_t ADDR_R_MATRIX_A_M21 = 0x50;
constexpr uint8_t ADDR_R_MATRIX_A_M22 = 0x52;
constexpr uint8_t ADDR_R_MATRIX_A_M23 = 0x54;
constexpr uint8_t ADDR_R_MATRIX_A_M31 = 0x56;
constexpr uint8_t ADDR_R_MATRIX_A_M32 = 0x58;
constexpr uint8_t ADDR_R_MATRIX_A_M33 = 0x5A;

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

constexpr uint8_t CMD_BURST = 0x80;  // Write value to Issue Burst Read
// Write value for WIN_CTRL to change to Window 0
constexpr uint8_t CMD_WINDOW0 = 0x00;
// Write value for WIN_CTRL to change to Window 1
constexpr uint8_t CMD_WINDOW1 = 0x01;
// Write value for MODE_CMD_HI to begin sampling
constexpr uint8_t CMD_BEGIN_SAMPLING = 0x01;
// Write value for MODE_CMD_HI to stop sampling
constexpr uint8_t CMD_END_SAMPLING = 0x02;
// Write value for GLOB_CMD_LO to issue Software Reset
constexpr uint8_t CMD_SOFTRESET = 0x80;
// Write value for MSC_CTRL_HI to issue Flashtest
constexpr uint8_t CMD_FLASHTEST = 0x08;
// Write value for MSC_CTRL_HI to issue Selftest
constexpr uint8_t CMD_SELFTEST = 0x04;
// Write value for UART_CTRL_LO to enable UART_AUTO mode and set
// AUTO_START = disabled
constexpr uint8_t CMD_UART_AUTO_EN = 0x01;
// Write value for UART_CTRL_LO to disable UART_AUTO mode and set
// AUTO_START = disabled
constexpr uint8_t CMD_UART_AUTO_DIS = 0x00;

// Write values for ADDR_SMPL_CTRL_HI to set Output Rate
constexpr uint8_t CMD_RATE2000 = 0x00;
constexpr uint8_t CMD_RATE1000 = 0x01;
constexpr uint8_t CMD_RATE500 = 0x02;
constexpr uint8_t CMD_RATE250 = 0x03;
constexpr uint8_t CMD_RATE125 = 0x04;
constexpr uint8_t CMD_RATE62_5 = 0x05;
constexpr uint8_t CMD_RATE31_25 = 0x06;
constexpr uint8_t CMD_RATE15_625 = 0x07;
constexpr uint8_t CMD_RATE400 = 0x08;
constexpr uint8_t CMD_RATE200 = 0x09;
constexpr uint8_t CMD_RATE100 = 0x0A;
constexpr uint8_t CMD_RATE80 = 0x0B;
constexpr uint8_t CMD_RATE50 = 0x0C;
constexpr uint8_t CMD_RATE40 = 0x0D;
constexpr uint8_t CMD_RATE25 = 0x0E;
constexpr uint8_t CMD_RATE20 = 0x0F;

// Write values for FILTER_CTRL_LO to set Filter
constexpr uint8_t CMD_FLTAP0 = 0x00;
constexpr uint8_t CMD_FLTAP2 = 0x01;
constexpr uint8_t CMD_FLTAP4 = 0x02;
constexpr uint8_t CMD_FLTAP8 = 0x03;
constexpr uint8_t CMD_FLTAP16 = 0x04;
constexpr uint8_t CMD_FLTAP32 = 0x05;
constexpr uint8_t CMD_FLTAP64 = 0x06;
constexpr uint8_t CMD_FLTAP128 = 0x07;

// Only use when G370PDF1 & G370PDS0 DOUT_RATE != 2000, 400, or 80 sps
constexpr uint8_t CMD_FIRTAP32FC25_G370FS = 0x08;
constexpr uint8_t CMD_FIRTAP32FC50_G370FS = 0x09;
constexpr uint8_t CMD_FIRTAP32FC100_G370FS = 0x0A;
constexpr uint8_t CMD_FIRTAP32FC200_G370FS = 0x0B;
constexpr uint8_t CMD_FIRTAP64FC25_G370FS = 0x0C;
constexpr uint8_t CMD_FIRTAP64FC50_G370FS = 0x0D;
constexpr uint8_t CMD_FIRTAP64FC100_G370FS = 0x0E;
constexpr uint8_t CMD_FIRTAP64FC200_G370FS = 0x0F;
constexpr uint8_t CMD_FIRTAP128FC25_G370FS = 0x10;
constexpr uint8_t CMD_FIRTAP128FC50_G370FS = 0x11;
constexpr uint8_t CMD_FIRTAP128FC100_G370FS = 0x12;
constexpr uint8_t CMD_FIRTAP128FC200_G370FS = 0x13;

// Use for all other IMUs
// Use only when G370PDF1 or G370PDS0 has DOUT_RATE == 2000, 400, or 80 sps
constexpr uint8_t CMD_FIRTAP32FC50 = 0x08;
constexpr uint8_t CMD_FIRTAP32FC100 = 0x09;
constexpr uint8_t CMD_FIRTAP32FC200 = 0x0A;
constexpr uint8_t CMD_FIRTAP32FC400 = 0x0B;
constexpr uint8_t CMD_FIRTAP64FC50 = 0x0C;
constexpr uint8_t CMD_FIRTAP64FC100 = 0x0D;
constexpr uint8_t CMD_FIRTAP64FC200 = 0x0E;
constexpr uint8_t CMD_FIRTAP64FC400 = 0x0F;
constexpr uint8_t CMD_FIRTAP128FC50 = 0x10;
constexpr uint8_t CMD_FIRTAP128FC100 = 0x11;
constexpr uint8_t CMD_FIRTAP128FC200 = 0x12;
constexpr uint8_t CMD_FIRTAP128FC400 = 0x13;

// MODE STAT
constexpr uint8_t VAL_SAMPLING_MODE = 0x00;

// Other Values
constexpr uint16_t VAL_CONFIG_MASK = 0x0400;
constexpr uint16_t VAL_CONFIG_MODE = 0x0400;
constexpr uint16_t VAL_NOT_READY = 0x0400;
constexpr uint16_t VAL_FILTER_STAT_BIT = 0x0020;
constexpr uint16_t VAL_SELF_TEST_BIT = 0x0400;
constexpr uint16_t VAL_FLASH_STATUS_BIT = 0x0800;
constexpr uint16_t VAL_DIAG_FLASHBU_ERROR = 0x0001;
constexpr uint16_t VAL_DIAG_ST_ERR_ALL = 0x0002;
constexpr uint16_t VAL_DIAG_FLASH_ERR = 0x0004;
constexpr uint16_t VAL_DIAG_STAT_MASK = 0x7B7F;
}  // namespace EPSON_I_U