/**************************************************************************/
/*!
    @file     imu_epson_M-Gxxx_M-Vxxx.h

    Epson M-Gxxx/M-Vxxx Series IMU specific definitions

    @section  HISTORY

    v1.0 - First release
    v1.1 - Refactoring
    v1.2 - Remove legacy models
    v1.3   Add models G370PDS0, G330PDG0, G366PDG0, refactor

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

#ifndef EPSONGXXX_VXXX_H_
#define EPSONGXXX_VXXX_H_

#include "imu_epson_user_def.h"

#define EPSON_UNIT_TYPE       "IMU"

// Device specific scale factors and model
#ifdef EPSON_G320
#define EPSON_ACCL_SF         (.200)
#define EPSON_GYRO_SF         (.008)
#define EPSON_TEMPC_SF        (-0.0037918)
#define EPSON_ATTI_SF         (0)
#define EPSON_DLTA_SF         (1.638e-2f)
#define EPSON_DLTV_SF         (4.017e-3f)
#define EPSON_MODEL_STR       "G320PDG0"

#elif defined(EPSON_G354)
#define EPSON_ACCL_SF         (.200)
#define EPSON_GYRO_SF         (.016)
#define EPSON_TEMPC_SF        (-0.0037918)
#define EPSON_ATTI_SF         (0)
#define EPSON_DLTA_SF         (1.638e-2f)
#define EPSON_DLTV_SF         (4.017e-3f)
#define EPSON_MODEL_STR       "G354PDH0"

#elif defined(EPSON_G364PDC0)
#define EPSON_ACCL_SF         (.125)
#define EPSON_GYRO_SF         (.0075)
#define EPSON_TEMPC_SF        (-0.0037918)
#define EPSON_ATTI_SF         (0)
#define EPSON_DLTA_SF         (1.638e-2f)
#define EPSON_DLTV_SF         (4.017e-3f)
#define EPSON_MODEL_STR       "G364PDC0"

#elif defined(EPSON_G364PDCA)
#define EPSON_ACCL_SF         (.125)
#define EPSON_GYRO_SF         (.00375)
#define EPSON_TEMPC_SF        (-0.0037918)
#define EPSON_ATTI_SF         (0)
#define EPSON_DLTA_SF         (1.638e-2f)
#define EPSON_DLTV_SF         (4.017e-3f)
#define EPSON_MODEL_STR       "G364PDCA"

#elif defined(EPSON_V340)
#define EPSON_ACCL_SF         (.18)
#define EPSON_GYRO_SF         (.015)
#define EPSON_TEMPC_SF        (-0.0053964)
#define EPSON_ATTI_SF         (0)
#define EPSON_DLTA_SF         (0)
#define EPSON_DLTV_SF         (0)
#define EPSON_MODEL_STR       "V340PDD0"

#elif defined(EPSON_G365PDF1)
#define EPSON_ACCL_SF         (.40)
#define EPSON_GYRO_SF         (.0151515)
#define EPSON_TEMPC_SF        (-0.0037918)
#define EPSON_ATTI_SF         (.00699411)
#define EPSON_DLTA_SF         (3.103e-2f)
#define EPSON_DLTV_SF         (8.034e-3f)
#define EPSON_MODEL_STR       "G365PDF1"

#elif defined(EPSON_G365PDC1)
#define EPSON_ACCL_SF         (.16)
#define EPSON_GYRO_SF         (.0151515)
#define EPSON_TEMPC_SF        (-0.0037918)
#define EPSON_ATTI_SF         (.00699411)
#define EPSON_DLTA_SF         (3.103e-2f)
#define EPSON_DLTV_SF         (3.213e-3f)
#define EPSON_MODEL_STR       "G365PDC1"

#elif defined(EPSON_G370PDF1)
#define EPSON_ACCL_SF         (.40)
#define EPSON_GYRO_SF         (.0151515)
#define EPSON_TEMPC_SF        (-0.0037918)
#define EPSON_ATTI_SF         (0)
#define EPSON_DLTA_SF         (6.206e-2f)
#define EPSON_DLTV_SF         (1.607e-2f)
#define EPSON_MODEL_STR       "G370PDF1"

#elif defined(EPSON_G370PDS0)
#define EPSON_ACCL_SF         (.40)
#define EPSON_GYRO_SF         (0.006666666667)
#define EPSON_TEMPC_SF        (-0.0037918)
#define EPSON_ATTI_SF         (0)
#define EPSON_DLTA_SF         (2.713e-2f)
#define EPSON_DLTV_SF         (1.067e-2f)
#define EPSON_MODEL_STR       "G370PDS0"

#elif defined(EPSON_G330PDG0)
#if ENABLE_16G
  #define EPSON_ACCL_SF       (.50)
#else
  #define EPSON_ACCL_SF       (.25)
#endif // ENABLE_16G
#define EPSON_GYRO_SF         (0.0151515)
#define EPSON_TEMPC_SF        (0.00390625)
#define EPSON_ATTI_SF         (.00699411)
#define EPSON_DLTA_SF         (3.103e-2f)
#if ENABLE_16G
  #define EPSON_DLTV_SF       (1.004e-2f)
#else
  #define EPSON_DLTV_SF       (5.021e-2f)
#endif // ENABLE_16G
#define EPSON_MODEL_STR       "G330PDG0"

#elif defined(EPSON_G366PDG0)
#if ENABLE_16G
  #define EPSON_ACCL_SF       (.50)
#else
  #define EPSON_ACCL_SF       (.25)
#endif // ENABLE_16G
#define EPSON_GYRO_SF         (0.0151515)
#define EPSON_TEMPC_SF        (0.00390625)
#define EPSON_ATTI_SF         (.00699411)
#define EPSON_DLTA_SF         (3.103e-2f)
#if ENABLE_16G
  #define EPSON_DLTV_SF       (1.004e-2f)
#else
  #define EPSON_DLTV_SF       (5.021e-2f)
#endif // ENABLE_16G
#define EPSON_MODEL_STR       "G366PDG0"

#else
#define EPSON_ACCL_SF         (0)
#define EPSON_GYRO_SF         (0)
#define EPSON_TEMPC_SF        (0)
#define EPSON_ATTI_SF         (0)
#define EPSON_DLTA_SF         (0)
#define EPSON_DLTV_SF         (0)
#define EPSON_MODEL_STR       "UNKNOWN"

#endif    // EPSON_G320


/*                                      -- Commands --
    - ADDR_ address byte of transfer to select the register to access
    - CMD_  data byte of transfer to write to the register selected
    - VAL_  data byte of transfer read from the register selected

    - All accesses are 16 bit transfers
    - For UART IF:
        - For UART write accesses - 8-bit address with msb=1b(can be even or odd) + 8-bit write data + Delimiter Byte
                                  - No response
        - For UART read accesses - 8-bit address with msb=0b(even only) + 8-bit dummy data + Delimiter Byte
                                 - Response is transferred immediately
                                 - Return value consists of Register Read Address + 16-bit read data (high byte + low byte) + Delimiter Byte

    - NOTE: G3xx have Register Address Maps that depend on the WINDOW_ID (page)
            For V340 the WINDOW_ID must be specified, however it is not used.
*/

#if defined (EPSON_V340)
#define ADDR_FLAG                  0x00     // FLAG(ND/EA)
#define ADDR_TEMP_HIGH             0x02     // TEMPC HIGH
#define ADDR_TEMP_LOW              0x02     // TEMPC LOW
#define ADDR_XGYRO_HIGH            0x04     // XGYRO HIGH
#define ADDR_XGYRO_LOW             0x04     // XGYRO LOW
#define ADDR_YGYRO_HIGH            0x06     // YGYRO HIGH
#define ADDR_YGYRO_LOW             0x06     // YGYRO LOW
#define ADDR_ZGYRO_HIGH            0x08     // ZGYRO HIGH
#define ADDR_ZGYRO_LOW             0x08     // ZGYRO LOW
#define ADDR_XACCL_HIGH            0x0A     // XACCL HIGH
#define ADDR_XACCL_LOW             0x0A     // XACCL LOW
#define ADDR_YACCL_HIGH            0x0C     // YACCL HIGH
#define ADDR_YACCL_LOW             0x0C     // YACCL LOW
#define ADDR_ZACCL_HIGH            0x0E     // ZACCL HIGH
#define ADDR_ZACCL_LOW             0x0E     // ZACCL LOW
#define ADDR_GPIO                  0x10     // GPIO
#define ADDR_COUNT                 0x12     // COUNT
#define ADDR_SIG_CTRL_LO           0x32     // SIG_CTRL Byte0
#define ADDR_SIG_CTRL_HI           0x33     // SIG_CTRL Byte1
#define ADDR_MSC_CTRL_LO           0x34     // MSC_CTRL Byte0
#define ADDR_MSC_CTRL_HI           0x35     // MSC_CTRL Byte1
#define ADDR_SMPL_CTRL_LO          0x36     // SMPL_CTRL Byte0
#define ADDR_SMPL_CTRL_HI          0x37     // SMPL_CTRL Byte1
#define ADDR_FILTER_CTRL_LO        0x38     // FILTER_CTRL Byte0
#define ADDR_MODE_CTRL_LO          0x38     // MODE_CTRL Byte0
#define ADDR_MODE_CTRL_HI          0x39     // MODE_CTRL Byte1
#define ADDR_UART_CTRL_LO          0x3A     // UART_CTRL Byte0
#define ADDR_UART_CTRL_HI          0x3B     // UART_CTRL Byte1
#define ADDR_DIAG_STAT             0x3C     // DIAG_STAT Byte0
#define ADDR_GLOB_CMD_LO           0x3E     // GLOB_CMD Byte0
#define ADDR_GLOB_CMD_HI           0x3F     // GLOB_CMD Byte1
#define ADDR_COUNT_CTRL_LO         0x50     // COUNT_CTRL Byte0
#define ADDR_COUNT_CTRL_HI         0x51     // COUNT_CTRL Byte1
#define ADDR_PROD_ID1              0x6A     // PROD_ID1
#define ADDR_PROD_ID2              0x6C     // PROD_ID2
#define ADDR_PROD_ID3              0x6E     // PROD_ID3
#define ADDR_PROD_ID4              0x70     // PROD_ID4
#define ADDR_VERSION               0x72     // VERSION
#define ADDR_SERIAL_NUM1           0x74     // SERIAL_NUM1
#define ADDR_SERIAL_NUM2           0x76     // SERIAL_NUM2
#define ADDR_SERIAL_NUM3           0x78     // SERIAL_NUM3
#define ADDR_SERIAL_NUM4           0x7A     // SERIAL_NUM4

#define CMD_BURST                  0x20     // Write value to Issue Burst Read
#define CMD_WINDOW0                0x00     // Write value for WIN_CTRL to change to Window 0
#define CMD_WINDOW1                0x01     // Write value for WIN_CTRL to change to Window 1
#define CMD_BEGIN_SAMPLING         0x01     // Write value for MODE_CMD_HI to begin sampling
#define CMD_END_SAMPLING           0x02     // Write value for MODE_CMD_HI to stop sampling
#define CMD_SOFTRESET              0x80     // Write value for GLOB_CMD_LO to issue Software Reset
#define CMD_FLASHTEST              0x00     // Flash is not supported for V340
#define CMD_SELFTEST               0x04     // Write value for MSC_CTRL_HI to issue Selftest
#define CMD_UART_AUTO_EN           0x01     // Write value for UART_CTRL_LO to enable UART_AUTO mode and set AUTO_START = disabled
#define CMD_UART_AUTO_DIS          0x00     // Write value for UART_CTRL_LO to disable UART_AUTO mode and set AUTO_START = disabled

// Write values for ADDR_SMPL_CTRL_HI to set Output Rate
#define CMD_RATE1000               0x01     // TAP>=1
#define CMD_RATE500                0x02     // TAP>=2
#define CMD_RATE250                0x03     // TAP>=4
#define CMD_RATE125                0x04     // TAP>=8
#define CMD_RATE62_5               0x05     // TAP>=16
#define CMD_RATE31_25              0x06     // TAP>=32

// Write values for MODE_CTRL_LO to set Filter
#define CMD_FLTAP1                 0x02
#define CMD_FLTAP2                 0x03
#define CMD_FLTAP4                 0x04
#define CMD_FLTAP8                 0x05
#define CMD_FLTAP16                0x06
#define CMD_FLTAP32                0x07

// Write value for SIG_CTRL_HI
#define ND_ENABLE_ACCL             (7<<1)
#define ND_ENABLE_GYRO             (7<<4)
#define ND_ENABLE_TEMP             (1<<7)
#define CMD_EN_NDFLAGS_HI          (ND_ENABLE_TEMP | ND_ENABLE_GYRO | ND_ENABLE_ACCL)

// Write value for MSC_CTRL_LO
#define EXT_SEL                    (ENABLE_COUNTER_RESET<<6)      // 0=gpio, 1=counter_reset, 2=trigger
#define ENABLE_DRDY                (1<<2)
#define DRDY_POL                   (1<<1)
#define CMD_MSC_CTRL_LO            (EXT_SEL | ENABLE_DRDY | DRDY_POL)


#else  // !defined EPSON_V340

// WINDOW_ID 0
#define ADDR_MODE_CTRL_LO          0x02     // MODE_CTRL Byte0 (W0)
#define ADDR_MODE_CTRL_HI          0x03     // MODE_CTRL Byte1 (W0)
#define ADDR_DIAG_STAT             0x04     // DIAG_STAT Byte0 (W0)
#define ADDR_FLAG                  0x06     // FLAG(ND/EA) (W0)
#define ADDR_GPIO                  0x08     // GPIO  (W0)
#define ADDR_COUNT                 0x0A     // COUNT (W0)

#if defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G370PDF1 || defined EPSON_G370PDS0 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0
#define ADDR_RANGE_OVER            0x0C     // RANGE_OVER (W0)
#endif

#define ADDR_TEMP_HIGH             0x0E     // TEMPC HIGH (W0)
#define ADDR_TEMP_LOW              0x10     // TEMPC LOW  (W0)
#define ADDR_XGYRO_HIGH            0x12     // XGYRO HIGH (W0)
#define ADDR_XGYRO_LOW             0x14     // XGYRO LOW  (W0)
#define ADDR_YGYRO_HIGH            0x16     // YGYRO HIGH (W0)
#define ADDR_YGYRO_LOW             0x18     // YGYRO LOW  (W0)
#define ADDR_ZGYRO_HIGH            0x1A     // ZGYRO HIGH (W0)
#define ADDR_ZGYRO_LOW             0x1C     // ZGYRO LOW  (W0)
#define ADDR_XACCL_HIGH            0x1E     // XACCL HIGH (W0)
#define ADDR_XACCL_LOW             0x20     // XACCL LOW  (W0)
#define ADDR_YACCL_HIGH            0x22     // YACCL HIGH (W0)
#define ADDR_YACCL_LOW             0x24     // YACCL LOW  (W0)
#define ADDR_ZACCL_HIGH            0x26     // ZACCL HIGH (W0)
#define ADDR_ZACCL_LOW             0x28     // ZACCL LOW  (W0)

#if defined EPSON_G370PDF1 || defined EPSON_G370PDS0
#define ADDR_RT_DIAG               0x2B     // RT_DIAG (W0)
#endif    // defined EPSON_G370PDF1 || defined EPSON_G370PDS0

#define ADDR_XDLTA_HIGH            0x64     // XDLTA HIGH (W0)
#define ADDR_XDLTA_LOW             0x66     // XDLTA LOW  (W0)
#define ADDR_YDLTA_HIGH            0x68     // YDLTA HIGH (W0)
#define ADDR_YDLTA_LOW             0x6A     // YDLTA LOW  (W0)
#define ADDR_ZDLTA_HIGH            0x6C     // ZDLTA HIGH (W0)
#define ADDR_ZDLTA_LOW             0x6E     // ZDLTA LOW  (W0)
#define ADDR_XDLTV_HIGH            0x70     // XDLTV HIGH (W0)
#define ADDR_XDLTV_LOW             0x72     // XDLTV LOW  (W0)
#define ADDR_YDLTV_HIGH            0x74     // YDLTV HIGH (W0)
#define ADDR_YDLTV_LOW             0x76     // YDLTV LOW  (W0)
#define ADDR_ZDLTV_HIGH            0x78     // ZDLTV HIGH (W0)
#define ADDR_ZDLTV_LOW             0x7A     // ZDLTV LOW  (W0)


// WINDOW_ID 1
#define ADDR_SIG_CTRL_LO           0x00     // SIG_CTRL Byte0 (W1)
#define ADDR_SIG_CTRL_HI           0x01     // SIG_CTRL Byte1 (W1)
#define ADDR_MSC_CTRL_LO           0x02     // MSC_CTRL Byte0 (W1)
#define ADDR_MSC_CTRL_HI           0x03     // MSC_CTRL Byte1 (W1)
#define ADDR_SMPL_CTRL_LO          0x04     // SMPL_CTRL Byte0 (W1)
#define ADDR_SMPL_CTRL_HI          0x05     // SMPL_CTRL Byte1 (W1)
#define ADDR_FILTER_CTRL_LO        0x06     // FILTER_CTRL Byte0 (W1)
#define ADDR_FILTER_CTRL_HI        0x07     // FILTER_CTRL Byte1 (W1)
#define ADDR_UART_CTRL_LO          0x08     // UART_CTRL Byte0 (W1)
#define ADDR_UART_CTRL_HI          0x09     // UART_CTRL Byte1 (W1)
#define ADDR_GLOB_CMD_LO           0x0A     // GLOB_CMD Byte0 (W1)
#define ADDR_GLOB_CMD_HI           0x0B     // GLOB_CMD Byte1 (W1)
#define ADDR_BURST_CTRL1_LO        0x0C     // BURST_CTRL1 Byte0 (W1)
#define ADDR_BURST_CTRL1_HI        0x0D     // BURST_CTRL1 Byte1 (W1)
#define ADDR_BURST_CTRL2_LO        0x0E     // BURST_CTRL2 Byte0 (W1)
#define ADDR_BURST_CTRL2_HI        0x0F     // BURST_CTRL2 Byte1 (W1)
#define ADDR_POL_CTRL_LO           0x10     // POL_CTRL Byte0 (W1)
#define ADDR_POL_CTRL_HI           0x11     // POL_CTRL Byte1 (W1)
#define ADDR_DLT_CTRL_LO           0x12     // DLT_CTRL Byte0 (W1) called GLOB_CMD3 for G330/G366
#define ADDR_DLT_CTRL_HI           0x13     // DLT_CTRL Byte1 (W1) called GLOB_CMD3 for G330/G366

#if defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G370PDF1 || defined EPSON_G370PDS0 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0
#define ADDR_ATTI_CTRL_LO          0x14     // ATTI_CTRL Byte0 (W1)
#define ADDR_ATTI_CTRL_HI          0x15     // ATTI_CTRL Byte1 (W1)
#define ADDR_GLOB_CMD2_LO          0x16     // GLOB_CMD2 Byte0 (W1)
#define ADDR_GLOB_CMD2_HI          0x17     // GLOB_CMD2 Byte1 (W1)
#endif

#define ADDR_PROD_ID1              0x6A     // PROD_ID1(W1)
#define ADDR_PROD_ID2              0x6C     // PROD_ID2(W1)
#define ADDR_PROD_ID3              0x6E     // PROD_ID3(W1)
#define ADDR_PROD_ID4              0x70     // PROD_ID4(W1)
#define ADDR_VERSION               0x72     // VERSION(W1)
#define ADDR_SERIAL_NUM1           0x74     // SERIAL_NUM1(W1)
#define ADDR_SERIAL_NUM2           0x76     // SERIAL_NUM2(W1)
#define ADDR_SERIAL_NUM3           0x78     // SERIAL_NUM3(W1)
#define ADDR_SERIAL_NUM4           0x7A     // SERIAL_NUM4(W1)
#define ADDR_WIN_CTRL              0x7E     // WIN_CTRL(W0 or W1)

#define CMD_BURST                  0x80     // Write value to Issue Burst Read
#define CMD_WINDOW0                0x00     // Write value for WIN_CTRL to change to Window 0
#define CMD_WINDOW1                0x01     // Write value for WIN_CTRL to change to Window 1
#define CMD_BEGIN_SAMPLING         0x01     // Write value for MODE_CMD_HI to begin sampling
#define CMD_END_SAMPLING           0x02     // Write value for MODE_CMD_HI to stop sampling
#define CMD_SOFTRESET              0x80     // Write value for GLOB_CMD_LO to issue Software Reset
#define CMD_FLASHTEST              0x08     // Write value for MSC_CTRL_HI to issue Flashtest
#define CMD_SELFTEST               0x04     // Write value for MSC_CTRL_HI to issue Selftest
#define CMD_UART_AUTO_EN           0x01     // Write value for UART_CTRL_LO to enable UART_AUTO mode and set AUTO_START = disabled
#define CMD_UART_AUTO_DIS          0x00     // Write value for UART_CTRL_LO to disable UART_AUTO mode and set AUTO_START = disabled

// Write value for SIG_CTRL_HI
#ifdef ENABLE_ACCL
  #define ND_ENABLE_ACCL             (7<<1)   /* 0=disabled, 7=enabled */
#else
  #define ND_ENABLE_ACCL             (0<<1)   /* 0=disabled, 7=enabled */
#endif //ENABLE_ACCL

#ifdef ENABLE_GYRO
  #define ND_ENABLE_GYRO             (7<<4)   /* 0=disabled, 7=enabled */
#else
  #define ND_ENABLE_GYRO             (0<<4)   /* 0=disabled, 7=enabled */
#endif //ENABLE_GYRO

#define ND_ENABLE_TEMP             (ENABLE_TEMP<<7)   /* 0=disabled, 1=enabled */

#define CMD_EN_NDFLAGS_HI          (ND_ENABLE_TEMP | ND_ENABLE_GYRO | ND_ENABLE_ACCL)

// Write value for SIG_CTRL_LO
#ifdef ENABLE_DLTA
  #define ND_ENABLE_DLTA             (7<<5)   /* 0=disabled, 7=enabled */
#else
  #define ND_ENABLE_DLTA             (0<<5)   /* 0=disabled, 7=enabled */
#endif //ENABLE_DLTA

#ifdef ENABLE_DLTV
  #define ND_ENABLE_DLTV             (7<<2)   /* 0=disabled, 7=enabled */
#else
  #define ND_ENABLE_DLTV             (0<<2)   /* 0=disabled, 7=enabled */
#endif //ENABLE_DLTV

#define CMD_EN_NDFLAGS_LO          (ND_ENABLE_DLTA | ND_ENABLE_DLTV)

// Write value for MSC_CTRL_LO
#define EXT_SEL                    (ENABLE_COUNTER_RESET<<6)      // 0=gpio, 1=counter_reset, 2=triggerA, 3=triggerB
#define ENABLE_DRDY                (1<<2)      /* 0=disabled, 1=enabled */
#define DRDY_POL                   (1<<1)      /* 0=active LOW, 1=active HIGH */
#define CMD_MSC_CTRL_LO            (EXT_SEL | ENABLE_DRDY | DRDY_POL)

// Write values for ADDR_SMPL_CTRL_HI to set Output Rate
#define CMD_RATE2000               0x00     // TAP>=0
#define CMD_RATE1000               0x01     // TAP>=2
#define CMD_RATE500                0x02     // TAP>=4
#define CMD_RATE250                0x03     // TAP>=8
#define CMD_RATE125                0x04     // TAP>=16
#define CMD_RATE62_5               0x05     // TAP>=32
#define CMD_RATE31_25              0x06     // TAP>=64
#define CMD_RATE15_625             0x07     // TAP=128
#define CMD_RATE400                0x08     // TAP>=8
#define CMD_RATE200                0x09     // TAP>=16
#define CMD_RATE100                0x0A     // TAP>=32
#define CMD_RATE80                 0x0B     // TAP>=32
#define CMD_RATE50                 0x0C     // TAP>=64
#define CMD_RATE40                 0x0D     // TAP>=64
#define CMD_RATE25                 0x0E     // TAP=128
#define CMD_RATE20                 0x0F     // TAP=128

// Write values for FILTER_CTRL_LO to set Filter
#define CMD_FLTAP0                 0x00
#define CMD_FLTAP2                 0x01
#define CMD_FLTAP4                 0x02
#define CMD_FLTAP8                 0x03
#define CMD_FLTAP16                0x04
#define CMD_FLTAP32                0x05
#define CMD_FLTAP64                0x06
#define CMD_FLTAP128               0x07

#if defined EPSON_G370PDF1 || defined EPSON_G370PDS0
  // assumes DOUT_RATE is not 2000, 400, or 80 sps
  #define CMD_FIRTAP32FC25           0x08
  #define CMD_FIRTAP32FC50           0x09
  #define CMD_FIRTAP32FC100          0x0A
  #define CMD_FIRTAP32FC200          0x0B
  #define CMD_FIRTAP64FC25           0x0C
  #define CMD_FIRTAP64FC50           0x0D
  #define CMD_FIRTAP64FC100          0x0E
  #define CMD_FIRTAP64FC200          0x0F
  #define CMD_FIRTAP128FC25          0x10
  #define CMD_FIRTAP128FC50          0x11
  #define CMD_FIRTAP128FC100         0x12
  #define CMD_FIRTAP128FC200         0x13
#else
  #define CMD_FIRTAP32FC50           0x08
  #define CMD_FIRTAP32FC100          0x09
  #define CMD_FIRTAP32FC200          0x0A
  #define CMD_FIRTAP32FC400          0x0B
  #define CMD_FIRTAP64FC50           0x0C
  #define CMD_FIRTAP64FC100          0x0D
  #define CMD_FIRTAP64FC200          0x0E
  #define CMD_FIRTAP64FC400          0x0F
  #define CMD_FIRTAP128FC50          0x10
  #define CMD_FIRTAP128FC100         0x11
  #define CMD_FIRTAP128FC200         0x12
  #define CMD_FIRTAP128FC400         0x13
#endif

// BURST_CTRL1_HI
#define ENABLE_FLAG_OUT         (ENABLE_FLAG<<7)    /* 0=disabled, 1=enabled */
#define ENABLE_TEMP_OUT         (ENABLE_TEMP<<6)    /* 0=disabled, 1=enabled */
#define ENABLE_GYRO_OUT         (ENABLE_GYRO<<5)    /* 0=disabled, 1=enabled */
#define ENABLE_ACCL_OUT         (ENABLE_ACCL<<4)    /* 0=disabled, 1=enabled */
#define ENABLE_DLTA_OUT         (ENABLE_DLTA<<3)    /* 0=disabled, 1=enabled NOTE: For DLTA/DLTV, ATTI, QTN, only one can be enabled at the same time */
#define ENABLE_DLTV_OUT         (ENABLE_DLTV<<2)    /* 0=disabled, 1=enabled NOTE: For DLTA/DLTV, ATTI, QTN, only one can be enabled at the same time */

#if defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0
  #define ENABLE_QTN_OUT        (ENABLE_QTN<<1)     /* QTN output only for G365/G330/G366, 0=disabled, 1=enabled NOTE: For DLTA/DLTV, ATTI, QTN, only one can be enabled at the same time */
  #define ENABLE_ATTI_OUT       (ENABLE_ATTI<<0)    /* ATTI output only for G365/G330/G366, 0=disabled, 1=enabled NOTE: For DLTA/DLTV, ATTI, QTN, only one can be enabled at the same time */
#else
  #define ENABLE_QTN_OUT        (0<<1)     /* QTN output not supported */
  #define ENABLE_ATTI_OUT       (0<<0)     /* ATTI output not supported */
#endif // defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0

// Write value for BURST_CTRL1_HI to enable FLAG, TempC, Gyro, Accl, DeltaA/V, Quaternion, Attitude in burst mode
#define CMD_BRST_CTRL1_HI       (ENABLE_FLAG_OUT | ENABLE_TEMP_OUT | ENABLE_GYRO_OUT | ENABLE_ACCL_OUT | \
                                 ENABLE_DLTA_OUT | ENABLE_DLTV_OUT | ENABLE_QTN_OUT | ENABLE_ATTI_OUT)


// BURST_CTRL1_LO
#define ENABLE_GPIO_OUT         (ENABLE_GPIO<<2)     /* 0=disabled, 1=enabled */
#define ENABLE_COUNT_OUT        (ENABLE_COUNT<<1)    /* 0=disabled, 1=enabled */
#define ENABLE_CHKSM_OUT        (ENABLE_CHKSM<<0)    /* 0=disabled, 1=enabled */
// Write value for BURST_CTRL_LO to enable CHKSM, and COUNT bytes in burst mode
#define CMD_BRST_CTRL1_LO       (ENABLE_GPIO_OUT | ENABLE_COUNT_OUT | ENABLE_CHKSM_OUT)

// BURST_CTRL2_HI
#define ENABLE_TEMP_BIT         (ENABLE_TEMP_32<<6)    /* 0=16bit, 1=32bit */
#define ENABLE_GYRO_BIT         (ENABLE_GYRO_32<<5)    /* 0=16bit, 1=32bit */
#define ENABLE_ACCL_BIT         (ENABLE_ACCL_32<<4)    /* 0=16bit, 1=32bit */
#define ENABLE_DLTA_BIT         (ENABLE_DLTA_32<<3)    /* 0=16bit, 1=32bit */
#define ENABLE_DLTV_BIT         (ENABLE_DLTV_32<<2)    /* 0=16bit, 1=32bit */

#if defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0
  #define ENABLE_QTN_BIT        (ENABLE_QTN_32<<1)     /* QTN output only for G365/G330/G366, 0=16bit, 1=32bit */
  #define ENABLE_ATTI_BIT       (ENABLE_ATTI_32<<0)    /* ATTI output only for G365/G330/G366, 0=16bit, 1=32bit */
#else //
  #define ENABLE_QTN_BIT        (0<<1)     /* QTN output not supported */
  #define ENABLE_ATTI_BIT       (0<<0)     /* ATTI output not supported */
#endif // defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0

// Write value for BURST_CTRL2_HI to set fields 16bit or 32bit in burst mode
#define CMD_BRST_CTRL2_HI      (ENABLE_TEMP_BIT | ENABLE_GYRO_BIT | ENABLE_ACCL_BIT | ENABLE_DLTA_BIT | \
                                ENABLE_DLTV_BIT | ENABLE_QTN_BIT | ENABLE_ATTI_BIT)

// ATTI_DLT_CTRL_HI
#if defined EPSON_G330PDG0 || defined EPSON_G366PDG0
  #define CMD_DLT_CTRL_HI      (ENABLE_16G<<0)    /* 0=8G, 1=16G */
#endif // defined EPSON_G330PDG0 || defined EPSON_G366PDG0


// ATTI_CTRL_HI
#if defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0
  #define ATTI_MODE            (ATTI_EULER<<3)    /* ATTI output only for G365/G330/G366, 0=Inclination Angle, 1=Euler Angle */

  // NOTE: DLTA/DLTV & ATTI/QTN cannot be both enabled at the same time
  #if (ENABLE_ATTI | ENABLE_QTN)
    #define ATTI_ON            (2<<1)    /* 0=disable, 1=DeltaA/V, 2=Attitude/Quaternion(Only for G365/G330/G366) */
  #elif (ENABLE_DLTA | ENABLE_DLTV)
    #define ATTI_ON            (1<<1)    /* 0=disable, 1=DeltaA/V, 2=Attitude/Quaternion(Only for G365/G330/G366) */
  #else
    #define ATTI_ON            (0<<1)    /* 0=disable, 1=DeltaA/V, 2=Attitude/Quaternion(Only for G365/G330/G366) */
  #endif // (ENABLE_ATTI | ENABLE_QTN)
  // Write value for CMD_ATTI_CTRL_HI
  #define CMD_ATTI_CTRL_HI     (ATTI_MODE | ATTI_ON)
#endif // defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0

#if defined EPSON_G370PDF1 || defined EPSON_G370PDS0
  #if (ENABLE_DLTA | ENABLE_DLTV)
    #define ATTI_ON            (1<<1)    /* 0=disable, 1=DeltaA/V, 2=Attitude/Quaternion(Only for G365/G330/G366) */
  #else
    #define ATTI_ON            (0<<1)    /* 0=disable, 1=DeltaA/V, 2=Attitude/Quaternion(Only for G365/G330/G366) */
  #endif // (ENABLE_DLTA | ENABLE_DLTV)
  // Write value for CMD_ATTI_CTRL_HI
  #define CMD_ATTI_CTRL_HI     (ATTI_ON)
#endif // defined EPSON_G370PDF1 || defined EPSON_G370PDS0

// Write value for CMD_ATTI_CTRL_LO
#define CMD_ATTI_CTRL_LO       (ATTI_CNV)          /* ATTI output only for G365/G330/G366, 0x00 ~ 0x17 for Attitude Output Axis Conversion */
// GLOB_CMD2_LO
#define ATTI_MOTION_PROFILE    (ATTI_MOTION<<3)    /* ATTI output only for G365/G330/G366, 0=modeA, 1=modeB, 2=modeC */
// Write value for CMD_GLOB_CMD2_LO
#define CMD_GLOB_CMD2_LO       (ATTI_MOTION_PROFILE)

#endif    // defined (EPSON_V340)

// MODE STAT
#define VAL_SAMPLING_MODE          0x00

// Return Values
#define VAL_CONFIG_MASK            0x0400
#define VAL_CONFIG_MODE            0x0400
#define VAL_NOT_READY              0x0400
#define VAL_FILTER_STAT_BIT        0x0020
#define VAL_SELF_TEST_BIT          0x0400
#define VAL_FLASH_STATUS_BIT       0x0800
#define VAL_DIAG_FLASHBU_ERROR     0x0001
#define VAL_DIAG_ST_ERR_ALL        0x0002
#define VAL_DIAG_FLASH_ERR         0x0004
#define VAL_DIAG_STAT_MASK         0x7B7F


class EPSON_DEV:public UART_EPSON_COM {

 protected:
   // Stores burst flags after decodeBurstCtrl() method
   struct _burstFlag {
     // 0 = Off, 1 = 16bit, 2 = 32bit
     uint8_t nd_ea;
     uint8_t tempc;
     uint8_t gyro;
     uint8_t accl;
     uint8_t dlta;
     uint8_t dltv;
     uint8_t qtn;
     uint8_t atti;
     uint8_t gpio;
     uint8_t count;
     uint8_t chksm;
   } _burstFlag = {0};

 public:
   EPSON_DEV(int8_t nrst, int8_t drdy):UART_EPSON_COM(nrst, drdy){};   // UART non-AUTO Mode (with DRDY)

   /**************************************************************************/
   /*!
       @brief  Output the sensor scalefactors to console
   */
   /**************************************************************************/
   void sensorScaleFactorsPrint(void){

     SerialConsole.println("*****************************************************************");
     SerialConsole.print("Gyro SF: "); SerialConsole.print(float(EPSON_GYRO_SF),6); SerialConsole.println(" dps/bit");
     SerialConsole.print("Accl SF: "); SerialConsole.print(float(EPSON_ACCL_SF),6); SerialConsole.println(" mG/bit");
   #if defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0
     SerialConsole.print("Atti SF: "); SerialConsole.print(float(EPSON_ATTI_SF),6); SerialConsole.println(" deg/bit");
   #endif    // defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0
   #ifndef EPSON_V340
     SerialConsole.print("DltA SF: "); SerialConsole.print(float(EPSON_DLTA_SF),6); SerialConsole.println(" deg/bit");
     SerialConsole.print("DltV SF: "); SerialConsole.print(float(EPSON_DLTV_SF),6); SerialConsole.println(" (m/s)/bit");
   #endif
     SerialConsole.println("*****************************************************************");
   }


   /**************************************************************************/
   /*!
       @brief  Decodes the DOUT_RATE register value to output rate in Hz

       @returns    float of output rate (Hz)
   */
   /**************************************************************************/
   float decodeDoutRate(void) {

   #ifdef EPSON_V340
     uint8_t dout_rate = (regRead16(CMD_WINDOW0, ADDR_SMPL_CTRL_LO)&0x0700)>>8;
     switch (dout_rate) {
       case 1: return 1000.0; break;
       case 2: return 500.0; break;
       case 3: return 250.0; break;
       case 4: return 125.0; break;
       case 5: return 62.5;  break;
       case 6: return 31.25; break;
       default:
   #ifdef DEBUG
         SerialConsole.print("Invalid DOUT_RATE");
   #endif //DEBUG
         return -1;
         break;
     }
   #else // !EPSON_V340
     uint8_t dout_rate = (regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO)&0x0F00)>>8;
     if (dout_rate <= 7) {
       return float(2000.0 / pow(2, dout_rate));
     }
     else {
       switch (dout_rate) {
       case 8: return 400.0; break;
       case 9: return 200.0; break;
       case 0xA: return 100.0; break;
       case 0xB: return 80.0;  break;
       case 0xC: return 50.0;  break;
       case 0xD: return 40.0;  break;
       case 0xE: return 25.0;  break;
       case 0xF: return 20.0;  break;
       default:
   #ifdef DEBUG
         SerialConsole.print("Invalid DOUT_RATE");
   #endif //DEBUG
         return -1;
         break;
       }
     }
   #endif    // EPSON_V340
   }


   /**************************************************************************/
   /*!
       @brief  Decodes the FILTER_SEL register value to return filter in
               as decoded string value

       @param [in]  filterString
                    Pointer to string to store decoded filter
                    setting
   */
   /**************************************************************************/
   void decodeFilterSel(char* filterString) {

   #ifdef EPSON_V340
     uint8_t filter_sel = regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO)&0x07;
     switch (filter_sel) {
       case 2: sprintf(filterString, "MVAVG_TAP1"); break;
       case 3: sprintf(filterString, "MVAVG_TAP2"); break;
       case 4: sprintf(filterString, "MVAVG_TAP4"); break;
       case 5: sprintf(filterString, "MVAVG_TAP8"); break;
       case 6: sprintf(filterString, "MVAVG_TAP16"); break;
       case 7: sprintf(filterString, "MVAVG_TAP32"); break;
       default:
         sprintf(filterString, "INVALID");
         break;
     }


   #elif defined (EPSON_G370PDF1) || defined (EPSON_G370PDS0)
     uint8_t dout_rate = (regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO)&0x0F00)>>8;
     uint8_t filter_sel = regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO)&0x1F;
     if (filter_sel <= 7) {
       uint8_t filter_val = pow(2, filter_sel);
       sprintf(filterString, "MVAVG_TAP%d", filter_val);
     }
     else if ((dout_rate == 0) || (dout_rate == 8) || (dout_rate == 0xB)) {
       // 2000, 400, 80sps
       switch (filter_sel) {
       case 8: sprintf(filterString, "KAISER32FC50"); break;
       case 9: sprintf(filterString, "KAISER32FC100"); break;
       case 0xA: sprintf(filterString, "KAISER32FC200"); break;
       case 0xB: sprintf(filterString, "KAISER32FC400"); break;
       case 0xC: sprintf(filterString, "KAISER64FC50"); break;
       case 0xD: sprintf(filterString, "KAISER64FC100"); break;
       case 0xE: sprintf(filterString, "KAISER64FC200"); break;
       case 0xF: sprintf(filterString, "KAISER64FC400"); break;
       case 0x10: sprintf(filterString, "KAISER128FC50"); break;
       case 0x11: sprintf(filterString, "KAISER128FC100"); break;
       case 0x12: sprintf(filterString, "KAISER128FC200"); break;
       case 0x13: sprintf(filterString, "KAISER128FC400"); break;
       default:
         sprintf(filterString, "INVALID");
         break;
       }
     }
     else {
       // !(2000, 400, 80sps)
       switch (filter_sel) {
       case 8: sprintf(filterString, "KAISER32FC25"); break;
       case 9: sprintf(filterString, "KAISER32FC50"); break;
       case 0xA: sprintf(filterString, "KAISER32FC100"); break;
       case 0xB: sprintf(filterString, "KAISER32FC200"); break;
       case 0xC: sprintf(filterString, "KAISER64FC25"); break;
       case 0xD: sprintf(filterString, "KAISER64FC50"); break;
       case 0xE: sprintf(filterString, "KAISER64FC100"); break;
       case 0xF: sprintf(filterString, "KAISER64FC200"); break;
       case 0x10: sprintf(filterString, "KAISER128FC25"); break;
       case 0x11: sprintf(filterString, "KAISER128FC50"); break;
       case 0x12: sprintf(filterString, "KAISER128FC100"); break;
       case 0x13: sprintf(filterString, "KAISER128FC200"); break;
       default:
         sprintf(filterString, "INVALID");
         break;
       }
     }
   #else // Other than G370PDF1 or G370PDS0
     uint8_t filter_sel = regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO)&0x1F;
     if (filter_sel <= 7) {
       uint8_t filter_val = pow(2, filter_sel);
       sprintf(filterString, "MVAVG_TAP%d", filter_val);
     }
     else {
       switch (filter_sel) {
       case 8: sprintf(filterString, "KAISER32FC50"); break;
       case 9: sprintf(filterString, "KAISER32FC100"); break;
       case 0xA: sprintf(filterString, "KAISER32FC200"); break;
       case 0xB: sprintf(filterString, "KAISER32FC400"); break;
       case 0xC: sprintf(filterString, "KAISER64FC50"); break;
       case 0xD: sprintf(filterString, "KAISER64FC100"); break;
       case 0xE: sprintf(filterString, "KAISER64FC200"); break;
       case 0xF: sprintf(filterString, "KAISER64FC400"); break;
       case 0x10: sprintf(filterString, "KAISER128FC50"); break;
       case 0x11: sprintf(filterString, "KAISER128FC100"); break;
       case 0x12: sprintf(filterString, "KAISER128FC200"); break;
       case 0x13: sprintf(filterString, "KAISER128FC400"); break;
       default:
         sprintf(filterString, "INVALID");
         break;
       }
     }
   #endif    // EPSON_V340
   }


   /**************************************************************************/
   /*!
       @brief  Decodes the BURST_CTRL1 & BURST_CTRL2 register and stores
       status in _burstFlag struct and returns burst length in bytes

       @returns    sensor burst length in bytes(excluding header & delimiter byte)
   */
   /**************************************************************************/
   uint8_t decodeBurstCtrl(void) {

     uint8_t burst_len = 0;

   #ifdef EPSON_V340
     // V340 Burst Output Fields are fixed 16-bit with optional COUNT field
     // ND_EA FLAGS, TEMPC, GYROX, GYROY, GYROZ...
     // ACCLX, ACCLY, ACCLZ, GPIO, COUNT (optional)
     _burstFlag.nd_ea = 1;
     _burstFlag.tempc = 1;
     _burstFlag.gyro = 1;
     _burstFlag.accl = 1;
     _burstFlag.gpio = 1;
     _burstFlag.count = regRead16(CMD_WINDOW0, ADDR_COUNT_CTRL_LO)&0x01;

     burst_len = 18;
     if (_burstFlag.count) burst_len += 2;
   #else // !EPSON_V340

     uint16_t burst_ctrl1 = regRead16(CMD_WINDOW1, ADDR_BURST_CTRL1_LO)&0xFF07;
     uint8_t burst_ctrl2 = (regRead16(CMD_WINDOW1, ADDR_BURST_CTRL2_LO)&0x7F00)>>8;

     // burst_ctrl1 check
     _burstFlag.nd_ea = (burst_ctrl1&0x8000) ? 1 : 0;
     _burstFlag.tempc = (burst_ctrl1&0x4000) ? 1 : 0;
     _burstFlag.gyro = (burst_ctrl1&0x2000) ? 1 : 0;
     _burstFlag.accl = (burst_ctrl1&0x1000) ? 1 : 0;
     _burstFlag.dlta = (burst_ctrl1&0x800) ? 1 : 0;
     _burstFlag.dltv = (burst_ctrl1&0x400) ? 1 : 0;
     _burstFlag.qtn = (burst_ctrl1&0x200) ? 1 : 0;
     _burstFlag.atti = (burst_ctrl1&0x100) ? 1 : 0;
     _burstFlag.gpio = (burst_ctrl1&0x4) ? 1 : 0;
     _burstFlag.count = (burst_ctrl1&0x2) ? 1 : 0;
     _burstFlag.chksm = (burst_ctrl1&0x1) ? 1 : 0;

     // burst_ctrl2 check for 32-bit
     if ((_burstFlag.tempc) && (burst_ctrl2&0x40)) _burstFlag.tempc++;
     if ((_burstFlag.gyro) && (burst_ctrl2&0x20)) _burstFlag.gyro++;
     if ((_burstFlag.accl) && (burst_ctrl2&0x10)) _burstFlag.accl++;
     if ((_burstFlag.dlta) && (burst_ctrl2&0x08)) _burstFlag.dlta++;
     if ((_burstFlag.dltv) && (burst_ctrl2&0x04)) _burstFlag.dltv++;
     if ((_burstFlag.qtn) && (burst_ctrl2&0x02)) _burstFlag.qtn++;
     if ((_burstFlag.atti) && (burst_ctrl2&0x01)) _burstFlag.atti++;

     // Calc burst_len
     if (_burstFlag.nd_ea) burst_len += 2;
     if (_burstFlag.tempc) burst_len += _burstFlag.tempc*2;
     if (_burstFlag.gyro) burst_len += _burstFlag.gyro*6;
     if (_burstFlag.accl) burst_len += _burstFlag.accl*6;
     if (_burstFlag.dlta) burst_len += _burstFlag.dlta*6;
     if (_burstFlag.dltv) burst_len += _burstFlag.dltv*6;
     if (_burstFlag.qtn) burst_len += _burstFlag.qtn*8;
     if (_burstFlag.atti) burst_len += _burstFlag.atti*6;
     if (_burstFlag.gpio) burst_len += 2;
     if (_burstFlag.count) burst_len += 2;
     if (_burstFlag.chksm) burst_len += 2;
   #endif    // EPSON_V340
     return burst_len;
   }


   /**************************************************************************/
   /*!
       @brief      Enters Config Mode to check and output
                   sensor configuration in table format to console
   */
   /**************************************************************************/
   void sensorConfigDump(void) {

     if (sensorStop() == true) {

       // Print to formated table
       SerialConsole.println("\n*****************************************************************");
       char prod_id[]="XXXXXXXX";
       getProdID(prod_id);
       char serial_id[]="XXXXXXXX";
       getSerialNumber(serial_id);

       char linebuf[80];
       sprintf(linebuf, "PROD_ID: %s\tSERIAL_ID: %s\tVERSION: %x", prod_id, serial_id, getVersion());
       SerialConsole.println(linebuf);

       char buf[16];
       decodeFilterSel(buf);
       sprintf(linebuf, "DOUT_RATE: %0.3f\tFILTER_SEL: %s", decodeDoutRate(), buf);
       SerialConsole.println(linebuf);
       decodeBurstCtrl();
       if (_burstFlag.nd_ea) sprintf(linebuf, "ND_EA: ON");
       else sprintf(linebuf, "ND_EA: OFF");

       switch (_burstFlag.tempc) {
         case 2: strcat(linebuf, "\tTempC: 32"); break;
         case 1: strcat(linebuf, "\tTempC: 16"); break;
         default: strcat(linebuf, "\tTempC: OFF"); break;
       }

       switch (_burstFlag.gyro) {
         case 2: strcat(linebuf, "\tGyro: 32"); break;
         case 1: strcat(linebuf, "\tGyro: 16"); break;
         default: strcat(linebuf, "\tGyro: OFF"); break;
       }

       switch (_burstFlag.accl) {
         case 2: strcat(linebuf, "\tAccl: 32"); break;
         case 1: strcat(linebuf, "\tAccl: 16"); break;
         default: strcat(linebuf, "\tAccl: OFF"); break;
       }
       SerialConsole.println(linebuf);

       switch (_burstFlag.dlta) {
         case 2: sprintf(linebuf, "DltA: 32"); break;
         case 1: sprintf(linebuf, "DltA: 16"); break;
         default: sprintf(linebuf, "DltA: OFF"); break;
       }

       switch (_burstFlag.dltv) {
         case 2: strcat(linebuf, "\tDltV: 32"); break;
         case 1: strcat(linebuf, "\tDltV: 16"); break;
         default: strcat(linebuf, "\tDltV: OFF"); break;
       }

   #if defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0
       switch (_burstFlag.qtn) {
         case 2: strcat(linebuf, "\tQtn: 32"); break;
         case 1: strcat(linebuf, "\tQtn: 16"); break;
         default: strcat(linebuf, "\tQtn: OFF"); break;
       }

       switch (_burstFlag.atti) {
         case 2: strcat(linebuf, "\tAtti: 32"); break;
         case 1: strcat(linebuf, "\tAtti: 16"); break;
         default: strcat(linebuf, "\tAtti: OFF"); break;
       }
   #endif    // defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0

       SerialConsole.println(linebuf);
       if (_burstFlag.gpio) SerialConsole.print("Gpio: ON\t");
       else SerialConsole.print("Gpio: OFF\t");

       if (_burstFlag.count) SerialConsole.print("Count: ON\t");
       else SerialConsole.print("Count: OFF\t");

       if (_burstFlag.chksm) SerialConsole.print("Chksm: ON");
       else SerialConsole.print("Chksm: OFF");

       SerialConsole.println("\n*****************************************************************");
     }
     else {
       SerialConsole.println("Error entering CONFIG mode.");
     }
   }


   /**************************************************************************/
   /*!
       @brief      Sends column headers to console
   */
   /**************************************************************************/
   void sensorHeaderPrint(void) {
   #ifdef EPSON_V340
     SerialConsole.print("Sample#\tGx\tGy\tGz\tAx\tAy\tAz\tTempC");
     if (_burstFlag.count) SerialConsole.print("\tCounter");
     SerialConsole.println("\tFlags\tGpio");

   #else  // !EPSON_V340
     SerialConsole.print("\nSample#");
     if (_burstFlag.gyro) SerialConsole.print("\tGx\tGy\tGz");
     if (_burstFlag.accl) SerialConsole.print("\tAx\tAy\tAz");
     if (_burstFlag.dlta) SerialConsole.print("\tDAx\tDAy\tDAz");
     if (_burstFlag.dltv) SerialConsole.print("\tDVx\tDVy\tDVz");
     if (_burstFlag.qtn) SerialConsole.print("\tQTN0\tQTN1\tQTN2\tQTN3");
     if (_burstFlag.atti) SerialConsole.print("\tANG1\tANG2\tANG3");
     if (_burstFlag.tempc) SerialConsole.print("\tTempC");
     if (_burstFlag.count) SerialConsole.print("\tCounter");
     if (_burstFlag.nd_ea) SerialConsole.print("\tFlags");
     if (_burstFlag.gpio) SerialConsole.print("\tGpio");
     if (_burstFlag.chksm) SerialConsole.print("\tChksm");
     SerialConsole.print("\n");
   #endif   // EPSON_V340
   }


   /**************************************************************************/
   /*!
       @brief  Process sensor - convert and output sensor data to console

       @param [in]  readBuf
                    Pointer to 16-bit Array
       @param [in]  sampleCount
                    Current sample count
   */
   /**************************************************************************/
   void sensorDataPrint(const uint16_t *readBuf, uint32_t sampleCount){

     uint16_t nd_flags = 0;
     int32_t tempC = 0;
     int32_t gyroXYZ[3] = {0};
     int32_t accelXYZ[3] = {0};
     uint16_t gpio = 0;
     uint16_t counter = 0;
     #if defined(EPSON_G330PDG0) || defined(EPSON_G366PDG0)
       #define EPSON_TEMPC_OFF (0)
     #else
       #define EPSON_TEMPC_OFF (-172621824)
     #endif // defined(EPSON_G330PDG0) || defined(EPSON_G366PDG0)

   #ifdef EPSON_V340
     nd_flags = readBuf[0];
     tempC = readBuf[1]<<16; // temperature is set for 16-bit output
           // For V340 gyro & accel values are 16-bit, bit shift to 32-bit
     gyroXYZ[0] = readBuf[2]<<16;
     gyroXYZ[1] = readBuf[3]<<16;
     gyroXYZ[2] = readBuf[4]<<16;

     accelXYZ[0] = readBuf[5]<<16;
     accelXYZ[1] = readBuf[6]<<16;
     accelXYZ[2] = readBuf[7]<<16;
     gpio = readBuf[8];
     counter = readBuf[9];

   #else
     int32_t dltaXYZ[3] = {0};
     int32_t dltvXYZ[3] = {0};
     int32_t qtn4[4] = {0};
     int32_t attiXYZ[3] = {0};
     uint16_t chksm = 0;

     int i = 0;
     if (_burstFlag.nd_ea) {
       nd_flags = readBuf[i];
       i++;
     }

     if (_burstFlag.tempc == 2) {
       tempC = readBuf[i]<<16|readBuf[i+1];
       i = i + 2;
     }
     else if (_burstFlag.tempc == 1) {
       tempC = readBuf[i]<<16;
       i++;
     }

     if (_burstFlag.gyro == 2) {
       gyroXYZ[0] = readBuf[i]<<16|readBuf[i+1];
       gyroXYZ[1] = readBuf[i+2]<<16|readBuf[i+3];
       gyroXYZ[2] = readBuf[i+4]<<16|readBuf[i+5];
       i = i + 6;
     }
     else if (_burstFlag.gyro == 1) {
       gyroXYZ[0] = readBuf[i]<<16;
       gyroXYZ[1] = readBuf[i+1]<<16;
       gyroXYZ[2] = readBuf[i+2]<<16;
       i = i + 3;
     }

     if (_burstFlag.accl == 2) {
       accelXYZ[0] = readBuf[i]<<16|readBuf[i+1];
       accelXYZ[1] = readBuf[i+2]<<16|readBuf[i+3];
       accelXYZ[2] = readBuf[i+4]<<16|readBuf[i+5];
       i = i + 6;
     }
     else if (_burstFlag.accl == 1) {
       accelXYZ[0] = readBuf[i]<<16;
       accelXYZ[1] = readBuf[i+1]<<16;
       accelXYZ[2] = readBuf[i+2]<<16;
       i = i + 3;
     }

     if (_burstFlag.dlta == 2) {
       dltaXYZ[0] = readBuf[i]<<16|readBuf[i+1];
       dltaXYZ[1] = readBuf[i+2]<<16|readBuf[i+3];
       dltaXYZ[2] = readBuf[i+4]<<16|readBuf[i+5];
       i = i + 6;
     }
     else if (_burstFlag.dlta == 1) {
       dltaXYZ[0] = readBuf[i]<<16;
       dltaXYZ[1] = readBuf[i+1]<<16;
       dltaXYZ[2] = readBuf[i+2]<<16;
       i = i + 3;
     }

     if (_burstFlag.dltv == 2) {
       dltvXYZ[0] = readBuf[i]<<16|readBuf[i+1];
       dltvXYZ[1] = readBuf[i+2]<<16|readBuf[i+3];
       dltvXYZ[2] = readBuf[i+4]<<16|readBuf[i+5];
       i = i + 6;
     }
     else if (_burstFlag.dltv == 1) {
       dltvXYZ[0] = readBuf[i]<<16;
       dltvXYZ[1] = readBuf[i+1]<<16;
       dltvXYZ[2] = readBuf[i+2]<<16;
       i = i + 3;
     }

     if (_burstFlag.qtn == 2) {
       qtn4[0] = readBuf[i]<<16|readBuf[i+1];
       qtn4[1] = readBuf[i+2]<<16|readBuf[i+3];
       qtn4[2] = readBuf[i+4]<<16|readBuf[i+5];
       qtn4[3] = readBuf[i+6]<<16|readBuf[i+7];
       i = i + 8;
     }
     else if (_burstFlag.qtn == 1) {
       qtn4[0] = readBuf[i]<<16;
       qtn4[1] = readBuf[i+1]<<16;
       qtn4[2] = readBuf[i+2]<<16;
       qtn4[3] = readBuf[i+3]<<16;
       i = i + 4;
     }

     if (_burstFlag.atti == 2) {
       attiXYZ[0] = readBuf[i]<<16|readBuf[i+1];
       attiXYZ[1] = readBuf[i+2]<<16|readBuf[i+3];
       attiXYZ[2] = readBuf[i+4]<<16|readBuf[i+5];
       i = i + 6;
     }
     else if (_burstFlag.atti == 1) {
       attiXYZ[0] = readBuf[i]<<16;
       attiXYZ[1] = readBuf[i+1]<<16;
       attiXYZ[2] = readBuf[i+2]<<16;
       i = i + 3;
     }

     if (_burstFlag.gpio) {
       gpio = readBuf[i];
       i++;
     }

     if (_burstFlag.count) {
       counter = readBuf[i];
       i++;
     }

     if (_burstFlag.chksm) {
       chksm = readBuf[i];
     }
   #endif  //EPSON_V340

     // Print to serial console in columns: sample, gyrox, gyroy, gyroz,
     // accelx, accely, accelz, temperature, reset counter
     // Sensor data converted to floating point sensor values
     SerialConsole.print(sampleCount); SerialConsole.print("\t");
     if (_burstFlag.gyro) {
       SerialConsole.print(float(gyroXYZ[0])*EPSON_GYRO_SF/65536, 6); SerialConsole.print("\t");
       SerialConsole.print(float(gyroXYZ[1])*EPSON_GYRO_SF/65536, 6); SerialConsole.print("\t");
       SerialConsole.print(float(gyroXYZ[2])*EPSON_GYRO_SF/65536, 6); SerialConsole.print("\t");
     }
     if (_burstFlag.accl) {
       SerialConsole.print(float(accelXYZ[0])*EPSON_ACCL_SF/65536, 6); SerialConsole.print("\t");
       SerialConsole.print(float(accelXYZ[1])*EPSON_ACCL_SF/65536, 6); SerialConsole.print("\t");
       SerialConsole.print(float(accelXYZ[2])*EPSON_ACCL_SF/65536, 6); SerialConsole.print("\t");
     }
     if (_burstFlag.dlta) {
       SerialConsole.print(float(dltaXYZ[0])*EPSON_DLTA_SF/65536, 6); SerialConsole.print("\t");
       SerialConsole.print(float(dltaXYZ[1])*EPSON_DLTA_SF/65536, 6); SerialConsole.print("\t");
       SerialConsole.print(float(dltaXYZ[2])*EPSON_DLTA_SF/65536, 6); SerialConsole.print("\t");
     }
     if (_burstFlag.dltv) {
       SerialConsole.print(float(dltvXYZ[0])*EPSON_DLTV_SF/65536, 6); SerialConsole.print("\t");
       SerialConsole.print(float(dltvXYZ[1])*EPSON_DLTV_SF/65536, 6); SerialConsole.print("\t");
       SerialConsole.print(float(dltvXYZ[2])*EPSON_DLTV_SF/65536, 6); SerialConsole.print("\t");
     }
     if (_burstFlag.atti) {
       SerialConsole.print(float(attiXYZ[0])*EPSON_ATTI_SF/65536, 6); SerialConsole.print("\t");
       SerialConsole.print(float(attiXYZ[1])*EPSON_ATTI_SF/65536, 6); SerialConsole.print("\t");
       SerialConsole.print(float(attiXYZ[2])*EPSON_ATTI_SF/65536, 6); SerialConsole.print("\t");
     }
     if (_burstFlag.qtn) {
       SerialConsole.print(float(qtn4[0]/pow(2, 30)), 6); SerialConsole.print("\t");
       SerialConsole.print(float(qtn4[1]/pow(2, 30)), 6); SerialConsole.print("\t");
       SerialConsole.print(float(qtn4[2]/pow(2, 30)), 6); SerialConsole.print("\t");
       SerialConsole.print(float(qtn4[3]/pow(2, 30)), 6); SerialConsole.print("\t");
     }
     if (_burstFlag.tempc) {
       SerialConsole.print((float(tempC+EPSON_TEMPC_OFF)*EPSON_TEMPC_SF/65536)+25, 6); SerialConsole.print("\t");
     }
     if (_burstFlag.count) {
       SerialConsole.print(counter); SerialConsole.print("\t");
     }
     if (_burstFlag.nd_ea) {
       SerialConsole.print(nd_flags, HEX); SerialConsole.print("\t");
     }
     if (_burstFlag.gpio) {
       SerialConsole.print(gpio, HEX); SerialConsole.print("\t");
     }
     if (_burstFlag.chksm) SerialConsole.print(chksm, HEX);
     SerialConsole.println();
   }


   /**************************************************************************/
   /*!
       @brief  Assumes currently in Configuration Mode to set counter, ND_Flags, Burst Settings,
               output rate & filter settings.
               For valid settings, refer to device datasheet

       @param [in]  outputRate
                    refer to possible #defines CMD_RATExxx earlier in this file
       @param [in]  filterSetting
                    refer to possible #defines CMD_FLTAPxx or CMD_FIRTAPxxxFCxxx
                    earlier in this file

       @returns    True on success, False on busy bit
   */
   /**************************************************************************/
   boolean sensorInit(uint8_t outputRate, uint8_t filterSetting) {

     boolean result = false;
     _burstCnt_calculated = 0;

 #ifdef EPSON_V340
     // For V340, output is 16-bit and fixed fields, but need to enable counter in the burst read
     regWrite8(CMD_WINDOW0, ADDR_SIG_CTRL_HI, CMD_EN_NDFLAGS_HI);     // SIG_CTRL = Temp, Gyro, Accl
     regWrite8(CMD_WINDOW0, ADDR_MSC_CTRL_LO, CMD_MSC_CTRL_LO);       // EXT & DRDY
     regWrite8(CMD_WINDOW0, ADDR_SMPL_CTRL_HI, outputRate);           // Set Output Rate
     regWrite8(CMD_WINDOW0, ADDR_FILTER_CTRL_LO, filterSetting);      // Refer for #defines for CMD_FILTERX values
     regWrite8(CMD_WINDOW0, ADDR_COUNT_CTRL_LO, ENABLE_COUNT);        // Enable Counter
     result = true;
 #else
     // For all other IMUs, enable Flags, Temperature, Gyro=32, Accel=32, Counter, Chksm
     regWrite8(CMD_WINDOW1, ADDR_SIG_CTRL_HI, CMD_EN_NDFLAGS_HI);     // SIG_CTRL = Temp, Gyro, Accl
     regWrite8(CMD_WINDOW1, ADDR_SIG_CTRL_LO, CMD_EN_NDFLAGS_LO);     // SIG_CTRL = DltA, DltV
     regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_LO, CMD_MSC_CTRL_LO);       // EXT & DRDY
     regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL1_LO, CMD_BRST_CTRL1_LO);  // BURST_CTRL1 = Count
     regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL1_HI, CMD_BRST_CTRL1_HI);  // BURST_CTRL1 = Temp, Gyro, Accl
     regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL2_HI, CMD_BRST_CTRL2_HI);  // BURST_CTRL2 = Temp=16, Gyro=32, Accl=32
#if defined EPSON_G330PDG0 || defined EPSON_G366PDG0
     regWrite8(CMD_WINDOW1, ADDR_DLT_CTRL_HI, CMD_DLT_CTRL_HI);       // A_RANGE_CTRL = 8G, 16G output range
#endif // defined EPSON_G330PDG0 || defined EPSON_G366PDG0
 #if defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G370PDF1 || defined EPSON_G370PDS0 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0
     // For G365/G330/G366 also enable 32-bit attitude output in Euler mode in the sensor burst
     regWrite8(CMD_WINDOW1, ADDR_ATTI_CTRL_LO, CMD_ATTI_CTRL_LO);     // ATTI_CONV = 0x00
     regWrite8(CMD_WINDOW1, ADDR_ATTI_CTRL_HI, CMD_ATTI_CTRL_HI);     // ATTI_MODE = Euler, ATTI_ON = Attitude Output
 #endif // defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G370PDF1 || defined EPSON_G370PDS0 || defined EPSON_G330PDG0 || defined EPSON_G366PDG0
     regWrite8(CMD_WINDOW1, ADDR_SMPL_CTRL_HI, outputRate);           // Set Output Rate
     regWrite8(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, filterSetting);      // Refer for #defines for CMD_FILTERX values
     EpsonFilterDelay();                                              // Delay for internal processing

     // Check that busy bit to return 0
     uint16_t valRead = regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO) & VAL_FILTER_STAT_BIT;
     result = (valRead == 0) ? true : false;
 #endif    // EPSON_V340
     // If DRDY is not used then enable UART_AUTO mode, otherwise keep in UART Manual Mode
     if ( getDRDY() == -1 ) {
       // When DRDY is not used, enable UART AUTO mode
       regWrite8(CMD_WINDOW1, ADDR_UART_CTRL_LO, CMD_UART_AUTO_EN); // UART_CTRL Low Byte enables/disables UART_AUTO mode
       setUARTAuto(true);
     }
     else                // When DRDY is used, use UART Manual Mode and sent BURST command on every DRDY edge
       regWrite8(CMD_WINDOW1, ADDR_UART_CTRL_LO, CMD_UART_AUTO_DIS); // UART_CTRL Low Byte enables/disables UART_AUTO mode

     _burstCnt_calculated = decodeBurstCtrl();

     return result;
   }


   /**************************************************************************/
   /*!
       @brief      Enters Config Mode to read back all register values from the
                   Sensor.
   */
   /**************************************************************************/
   void registerDump(void) {

     if (sensorStop() == true) {

   #ifdef EPSON_V340
       //print the current values in all the Sensor registers
       SerialConsole.println("\r\nRegister Dump:");
       regRead16(CMD_WINDOW0, ADDR_FLAG, true);
       regRead16(CMD_WINDOW0, ADDR_TEMP_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_XGYRO_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_YGYRO_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_ZGYRO_LOW, true);
       SerialConsole.println();
       regRead16(CMD_WINDOW0, ADDR_XACCL_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_YACCL_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_ZACCL_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_GPIO, true);
       regRead16(CMD_WINDOW0, ADDR_COUNT, true);
       regRead16(CMD_WINDOW0, ADDR_SIG_CTRL_LO, true);
       regRead16(CMD_WINDOW0, ADDR_MSC_CTRL_LO, true);
       regRead16(CMD_WINDOW0, ADDR_SMPL_CTRL_LO, true);
       SerialConsole.println();
       regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO, true);
       regRead16(CMD_WINDOW0, ADDR_UART_CTRL_LO, true);
       regRead16(CMD_WINDOW0, ADDR_DIAG_STAT, true);
       regRead16(CMD_WINDOW0, ADDR_GLOB_CMD_LO, true);
       regRead16(CMD_WINDOW0, ADDR_COUNT_CTRL_LO, true);
       SerialConsole.println();
       regRead16(CMD_WINDOW0, ADDR_PROD_ID1, true);
       regRead16(CMD_WINDOW0, ADDR_PROD_ID2, true);
       regRead16(CMD_WINDOW0, ADDR_PROD_ID3, true);
       regRead16(CMD_WINDOW0, ADDR_PROD_ID4, true);
       regRead16(CMD_WINDOW0, ADDR_VERSION, true);
       regRead16(CMD_WINDOW0, ADDR_SERIAL_NUM1, true);
       regRead16(CMD_WINDOW0, ADDR_SERIAL_NUM2, true);
       regRead16(CMD_WINDOW0, ADDR_SERIAL_NUM3, true);
       regRead16(CMD_WINDOW0, ADDR_SERIAL_NUM4, true);
   #else
       //print the current values in all the Sensor registers
       SerialConsole.println("\r\nRegister Dump:");
       regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO, true);
       regRead16(CMD_WINDOW0, ADDR_DIAG_STAT, true);
       regRead16(CMD_WINDOW0, ADDR_FLAG, true);
       regRead16(CMD_WINDOW0, ADDR_GPIO, true);
       regRead16(CMD_WINDOW0, ADDR_COUNT, true);
   #if defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G370PDF1 || defined EPSON_G370PDS0 || defined EPSON_G330PDG0 || defined EPSON_G366PDG
       regRead16(CMD_WINDOW0, ADDR_RANGE_OVER, true);
   #endif    // defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G370PDF1 || defined EPSON_G370PDS0 || defined EPSON_G330PDG0 || defined EPSON_G366PDG
       SerialConsole.println();
       regRead16(CMD_WINDOW0, ADDR_TEMP_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_TEMP_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_XGYRO_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_XGYRO_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_YGYRO_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_YGYRO_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_ZGYRO_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_ZGYRO_LOW, true);
       SerialConsole.println();
       regRead16(CMD_WINDOW0, ADDR_XACCL_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_XACCL_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_YACCL_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_YACCL_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_ZACCL_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_ZACCL_LOW, true);
       SerialConsole.println();
   #if defined EPSON_G370PDF1 || defined EPSON_G370PDS0
       regRead16(CMD_WINDOW0, ADDR_RT_DIAG, true);
       SerialConsole.println();
   #endif    // defined EPSON_G370PDF1 || defined EPSON_G370PDS0

       regRead16(CMD_WINDOW0, ADDR_XDLTA_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_XDLTA_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_YDLTA_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_YDLTA_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_ZDLTA_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_ZDLTA_LOW, true);
       SerialConsole.println();
       regRead16(CMD_WINDOW0, ADDR_XDLTV_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_XDLTV_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_YDLTV_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_YDLTV_LOW, true);
       regRead16(CMD_WINDOW0, ADDR_ZDLTV_HIGH, true);
       regRead16(CMD_WINDOW0, ADDR_ZDLTV_LOW, true);
       SerialConsole.println();
       regRead16(CMD_WINDOW1, ADDR_SIG_CTRL_LO, true);
       regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, true);
       regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO, true);
       regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, true);
       regRead16(CMD_WINDOW1, ADDR_UART_CTRL_LO, true);
       regRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO, true);
       regRead16(CMD_WINDOW1, ADDR_BURST_CTRL1_LO, true);
       regRead16(CMD_WINDOW1, ADDR_BURST_CTRL2_LO, true);
       regRead16(CMD_WINDOW1, ADDR_POL_CTRL_LO, true);
       regRead16(CMD_WINDOW1, ADDR_DLT_CTRL_LO, true);
   #if defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G370PDF1 || defined EPSON_G370PDS0 || defined EPSON_G330PDG0 || defined EPSON_G366PDG
       regRead16(CMD_WINDOW1, ADDR_ATTI_CTRL_LO, true);
       regRead16(CMD_WINDOW1, ADDR_GLOB_CMD2_LO, true);
   #endif //defined EPSON_G365PDC1 || defined EPSON_G365PDF1 || defined EPSON_G370PDF1 || defined EPSON_G370PDS0 || defined EPSON_G330PDG0 || defined EPSON_G366PDG
       SerialConsole.println();
       regRead16(CMD_WINDOW1, ADDR_PROD_ID1, true);
       regRead16(CMD_WINDOW1, ADDR_PROD_ID2, true);
       regRead16(CMD_WINDOW1, ADDR_PROD_ID3, true);
       regRead16(CMD_WINDOW1, ADDR_PROD_ID4, true);
       regRead16(CMD_WINDOW1, ADDR_VERSION, true);
       regRead16(CMD_WINDOW1, ADDR_SERIAL_NUM1, true);
       regRead16(CMD_WINDOW1, ADDR_SERIAL_NUM2, true);
       regRead16(CMD_WINDOW1, ADDR_SERIAL_NUM3, true);
       regRead16(CMD_WINDOW1, ADDR_SERIAL_NUM4, true);
   #endif    // EPSON_V340
       }
     else {
       SerialConsole.println("Warning: Not entering Config Mode");
     }
   }

};

#endif //< EPSONGXXX_VXXX_H_
