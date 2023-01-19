/**************************************************************************/
/*!
    @file     accel_epson_M-Axxx.h

    Epson M-Axxx Accelerometer specific definitions

    @section  HISTORY

    v1.0 - First release
    v1.1 - Rename file

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
#ifndef EPSONAXXX_H_
#define EPSONAXXX_H_

#include "accel_epson_user_def.h"

#ifdef EPSON_SELF_TEST_DELAY
#undef EPSON_SELF_TEST_DELAY
#endif
#define EPSON_SELF_TEST_DELAY       (200000)    /*! @def EPSON_SELF_TEST_DELAY for M-A352 is <200 ms */

#define EPSON_ACCL_SF          (0.00000006f)    //   G/LSB
#define EPSON_TEMP_SF          (-0.0037918f)    //   C/LSB
#define EPSON_TILT_SF         (0.000000002f)    // rad/LSB
#define EPSON_MODEL_STR           "A352AD10"
#define EPSON_UNIT_TYPE      "Accelerometer"


/*                                      -- Commands --
    - ADDR_ address byte of transfer to select the register to access
    - CMD_  data byte of transfer to write to the register selected

    - All accesses are 16 bit transfers
    - For UART IF:
        - For UART write accesses - 8-bit address with msb=1b(can be even or odd) + 8-bit write data + Delimiter Byte
                                  - No response
        - For UART read accesses - 8-bit address with msb=0b(even only) + 8-bit dummy data + Delimiter Byte
                                 - Response is transferred immediately
                                 - Return value consists of Register Read Address + 16-bit read data (high byte + low byte) + Delimiter Byte

    - NOTE: Register Address Maps that depend on the WINDOW_ID (page) */


// WINDOW_ID 0
#define ADDR_BURST              0x00    // BURST (W0)
#define ADDR_MODE_CTRL_LO       0x02    // MODE_CTRL Byte0 (W0)
#define ADDR_MODE_CTRL_HI       0x03    // MODE_CTRL Byte1 (W0)
#define ADDR_DIAG_STAT          0x04    // DIAG_STAT (W0)
#define ADDR_FLAG               0x06    // FLAG(ND/EA) (W0)
#define ADDR_COUNT              0x0A    // COUNT (W0)
#define ADDR_TEMP_HIGH          0x0E    // TEMPC HIGH (W0)
#define ADDR_TEMP_LOW           0x10    // TEMPC LOW  (W0)
#define ADDR_XACCL_HIGH         0x30    // XACCL HIGH (W0)
#define ADDR_XACCL_LOW          0x32    // XACCL LOW  (W0)
#define ADDR_YACCL_HIGH         0x34    // YACCL HIGH (W0)
#define ADDR_YACCL_LOW          0x36    // YACCL LOW  (W0)
#define ADDR_ZACCL_HIGH         0x38    // ZACCL HIGH (W0)
#define ADDR_ZACCL_LOW          0x3A    // ZACCL LOW  (W0)
#define ADDR_XTILT_HIGH         0x3C    // XTILT HIGH (W0)
#define ADDR_XTILT_LOW          0x3E    // XTILT LOW  (W0)
#define ADDR_YTILT_HIGH         0x40    // YTILT HIGH (W0)
#define ADDR_YTILT_LOW          0x42    // YTILT LOW  (W0)
#define ADDR_ZTILT_HIGH         0x44    // ZTILT HIGH (W0)
#define ADDR_ZTILT_LOW          0x46    // ZTILT LOW  (W0)

// WINDOW_ID 1
#define ADDR_SIG_CTRL_LO        0x00    // SIG_CTRL Byte0 (W1)
#define ADDR_SIG_CTRL_HI        0x01    // SIG_CTRL Byte1 (W1)
#define ADDR_MSC_CTRL_LO        0x02    // MSC_CTRL Byte0 (W1)
#define ADDR_MSC_CTRL_HI        0x03    // MSC_CTRL Byte1 (W1)
#define ADDR_SMPL_CTRL_LO       0x04    // SMPL_CTRL Byte0 (W1)
#define ADDR_SMPL_CTRL_HI       0x05    // SMPL_CTRL Byte1 (W1)
#define ADDR_FILTER_CTRL_LO     0x06    // FILTER_CTRL (W1)
#define ADDR_UART_CTRL_LO       0x08    // UART_CTRL Byte0 (W1)
#define ADDR_UART_CTRL_HI       0x09    // UART_CTRL Byte1 (W1)
#define ADDR_GLOB_CMD_LO        0x0A    // GLOB_CMD Byte0 (W1)
#define ADDR_BURST_CTRL_LO      0x0C    // BURST_CTRL1 Byte0 (W1)
#define ADDR_BURST_CTRL_HI      0x0D    // BURST_CTRL1 Byte1 (W1)
#define ADDR_FIR_UCMD           0x16    // FIR_UCMD (W1)
#define ADDR_FIR_UDATA          0x18    // FIR_UDATA (W1)
#define ADDR_FIR_UADDR_LO       0x1A    // FIR_UADDR Byte0 (W1)
#define ADDR_FIR_UADDR_HI       0x1B    // FIR_UADDR Byte1 (W1)
#define ADDR_LONGFILT_CTRL      0x1C    // LONGFILT_CTRL (W1)
#define ADDR_LONGFILT_TAP       0x1E    // LONGFILT_TAP (W1)
#define ADDR_XOFFSET_HIGH_L     0x2C    // XOFFSET_HIGH_L (W1)
#define ADDR_XOFFSET_HIGH_H     0x2D    // XOFFSET_HIGH_H (W1)
#define ADDR_XOFFSET_LOW_L      0x2E    // XOFFSET_LOW_L (W1)
#define ADDR_XOFFSET_LOW_H      0x2F    // XOFFSET_LOW_H (W1)
#define ADDR_YOFFSET_HIGH_L     0x30    // YOFFSET_HIGH_L (W1)
#define ADDR_YOFFSET_HIGH_H     0x31    // YOFFSET_HIGH_H (W1)
#define ADDR_YOFFSET_LOW_L      0x32    // YOFFSET_LOW_L (W1)
#define ADDR_YOFFSET_LOW_H      0x33    // YOFFSET_LOW_H (W1)
#define ADDR_ZOFFSET_HIGH_L     0x34    // ZOFFSET_HIGH_L (W1)
#define ADDR_ZOFFSET_HIGH_H     0x35    // ZOFFSET_HIGH_H (W1)
#define ADDR_ZOFFSET_LOW_L      0x36    // ZOFFSET_LOW_L (W1)
#define ADDR_ZOFFSET_LOW_H      0x37    // ZOFFSET_LOW_H (W1)
#define ADDR_XALARM_LO          0x46    // XALARM_LO (W1)
#define ADDR_XALARM_UP          0x47    // XALARM_UP (W1)
#define ADDR_YALARM_LO          0x48    // YALARM_LO (W1)
#define ADDR_YALARM_UP          0x49    // YALARM_UP (W1)
#define ADDR_ZALARM_LO          0x4A    // ZALARM_LO (W1)
#define ADDR_ZALARM_UP          0x4B    // ZALARM_UP (W1)

#define ADDR_PROD_ID1           0x6A    // PROD_ID1(W1)
#define ADDR_PROD_ID2           0x6C    // PROD_ID2(W1)
#define ADDR_PROD_ID3           0x6E    // PROD_ID3(W1)
#define ADDR_PROD_ID4           0x70    // PROD_ID4(W1)
#define ADDR_VERSION            0x72    // VERSION(W1)
#define ADDR_SERIAL_NUM1        0x74    // SERIAL_NUM1(W1)
#define ADDR_SERIAL_NUM2        0x76    // SERIAL_NUM2(W1)
#define ADDR_SERIAL_NUM3        0x78    // SERIAL_NUM3(W1)
#define ADDR_SERIAL_NUM4        0x7A    // SERIAL_NUM4(W1)
#define ADDR_WIN_CTRL           0x7E    // WIN_CTRL(W0 or W1)

#define CMD_BURST               0x80    // Write value to Issue Burst Read
#define CMD_WINDOW0             0x00    // Write value for WIN_CTRL to change to Window 0
#define CMD_WINDOW1             0x01    // Write value for WIN_CTRL to change to Window 1
#define CMD_BEGIN_SAMPLING      0x01    // Write value for MODE_CMD_HI to begin sampling
#define CMD_END_SAMPLING        0x02    // Write value for MODE_CMD_HI to stop sampling
#define CMD_GOTO_SLEEP_MODE     0x03    // Write value for MODE_CMD_HI to go to the Sleep Mode
#define CMD_ZSENSTEST           0x40    // Write value for MSC_CTRL_HI to issue Z Accelerometer Test
#define CMD_YSENSTEST           0x20    // Write value for MSC_CTRL_HI to issue Y Accelerometer Test
#define CMD_XSENSTEST           0x10    // Write value for MSC_CTRL_HI to issue X Accelerometer Test
#define CMD_FLASHTEST           0x08    // Write value for MSC_CTRL_HI to issue Flash Test
#define CMD_SELFTEST            0x07    // Does ACCTEST, TEMPTEST and VDDTEST for selftest
#define CMD_ACCTEST             0x04    // Write value for MSC_CTRL_HI to issue Accelerometer Test
#define CMD_TEMPTEST            0x02    // Write value for MSC_CTRL_HI to issue Temp Sensor Test
#define CMD_VDDTEST             0x01    // Write value for MSC_CTRL_HI to issue Voltage Test

#define CMD_SOFTRESET           0x80    // Write value for GLOB_CMD_LO to issue Software Reset
#define CMD_FLASHBKUP           0x08    // Write value for GLOB_CMD_LO to issue Flash Backup
#define CMD_FLASHRST            0x04    // Write value for GLOB_CMD_LO to issue Flash Reset

#define CMD_UART_AUTO_EN        0x01     // Write value for UART_CTRL_LO to enable UART_AUTO mode and set AUTO_START = disabled
#define CMD_UART_AUTO_DIS       0x00     // Write value for UART_CTRL_LO to disable UART_AUTO mode and set AUTO_START = disabled

// Write value for SIG_CTRL_LO
#define OPT_TEMP_STAB           (TEMP_SHOCK<<2)       /* bias stabilization against thermal shock */
#define OPT_MESMOD_SEL          (REDUCED_NOISE<<4)    /* measurement cond: 0=standard noise floor, 1=reduced noise floor */
#define OPT_OUT_SEL_Z           (TILTZ<<5)    /* 0=acceleration, 1=tilt */
#define OPT_OUT_SEL_Y           (TILTY<<6)    /* 0=acceleration, 1=tilt */
#define OPT_OUT_SEL_X           (TILTX<<7)    /* 0=acceleration, 1=tilt */
#define CMD_SIG_CTR_LO_FLAGS    (OPT_OUT_SEL_X | OPT_OUT_SEL_Y | OPT_OUT_SEL_Z | OPT_MESMOD_SEL | OPT_TEMP_STAB)

// Write value for SIG_CTRL_HI to Enables new data (ND) flags for accel_out X,Y,Z are enabled if accel_out is enabled
#define ND_ENABLE_ZACCL         (ENABLE_ACCLZ<<1)
#define ND_ENABLE_YACCL         (ENABLE_ACCLY<<2)
#define ND_ENABLE_XACCL         (ENABLE_ACCLX<<3)
#define ND_ENABLE_TEMP          (ENABLE_TEMP<<7)
#define CMD_EN_NDFLAGS          (ND_ENABLE_TEMP | ND_ENABLE_XACCL | ND_ENABLE_YACCL | ND_ENABLE_ZACCL)

// MSC_CTRL
#define EXT_SEL                 (ENABLE_EXT<<6)    /* EXT function: 0=External trigger disabled, 1=enabled */
#define EXT_POL                 (POL_ACT_LOW<<5)   /* EXT polarity: 0=rising edge, 1=falling edge */
#define ENABLE_DRDY             (1<<2)    /* DRDY function: 0=DRDY disabled, 1=enabled */
#define OPT_DRDY_POL            (1<<1)    /* DRDY polarity: 0=Active low, 1=Active high */
#define CMD_CNTR_DRDY           (EXT_SEL | EXT_POL | ENABLE_DRDY | OPT_DRDY_POL)

// BURST_CTRL_HI (default: 0x47)
#define ENABLE_FLAG_OUT         (ENABLE_FLAG<<7)    /* 0=disabled, 1=enabled 0*/
#define ENABLE_TEMP_OUT         (ENABLE_TEMP<<6)    /* 0=disabled, 1=enabled */
#define ENABLE_ACCX_OUT         (ENABLE_ACCLX<<2)    /* 0=disabled, 1=enabled */
#define ENABLE_ACCY_OUT         (ENABLE_ACCLY<<1)    /* 0=disabled, 1=enabled */
#define ENABLE_ACCZ_OUT         (ENABLE_ACCLZ<<0)    /* 0=disabled, 1=enabled */
// Write value for BURST_CTRL1_HI to enable FLAG, TempC, ACC X,Y,Z registers in burst mode
#define CMD_EN_BRSTDATA_HI      (ENABLE_FLAG_OUT |  ENABLE_TEMP_OUT |  ENABLE_ACCX_OUT | ENABLE_ACCY_OUT | ENABLE_ACCZ_OUT)

// BURST_CTRL_LO (default: 0x02)
#define ENABLE_COUNT_OUT        (ENABLE_COUNT<<1)    /* 0=disabled, 1=enabled */
#define ENABLE_CHKSM_OUT        (ENABLE_CHKSM<<0)    /* 0=disabled, 1=enabled */
// Write value for BURST_CTRL_LO to enable CHKSM, and COUNT bytes in burst mode
#define CMD_EN_BRSTDATA_LO      (ENABLE_COUNT_OUT | ENABLE_CHKSM_OUT)


// Write values for ADDR_SMPL_CTRL_HI to set Output Rate
#define CMD_RATE1000            0x02
#define CMD_RATE500             0x03
#define CMD_RATE200             0x04
#define CMD_RATE100             0x05
#define CMD_RATE50              0x06

// Write values for FILTER_CTRL_LO to set Filter
// The Filter setting should be set according to the sampling rate. Refer
// to the Accelerometer DataSheet to determine a valid filter setting.
#define CMD_FIRTAP64FC83        0x01
#define CMD_FIRTAP64FC220       0x02
#define CMD_FIRTAP128FC36       0x03
#define CMD_FIRTAP128FC110      0x04
#define CMD_FIRTAP128FC350      0x05
#define CMD_FIRTAP512FC9        0x06
#define CMD_FIRTAP512FC16       0x07
#define CMD_FIRTAP512FC60       0x08
#define CMD_FIRTAP512FC210      0x09
#define CMD_FIRTAP512FC460      0x0A
#define CMD_USERFIRTAP4         0x0B //< user filter starts here
#define CMD_USERFIRTAP64        0x0C
#define CMD_USERFIRTAP128       0x0D
#define CMD_USERFIRTAP512       0x0E

// Write values for ADDR_FIR_UCMD
// These values are used to read/write FIR user coefficient values
#define CMD_USERFIR_READ        0x01
#define CMD_USERFIR_WRITE       0x02

// MODE STAT
#define VAL_SAMPLING_MODE       0x00

// Return Values
#define VAL_CONFIG_MASK         0x0C00
#define VAL_CONFIG_MODE         0x0400
#define VAL_SLEEP_MODE          0x0800
#define VAL_NOT_READY           0x0400
#define VAL_FILTER_STAT_BIT     0x0020
#define VAL_SELF_TEST_BIT       0x0700
#define VAL_FLASH_STATUS_BIT    0x0800
#define VAL_DIAG_FLASHBU_ERROR  0x0001
#define VAL_DIAG_ST_ERR_ALL     0x0002
#define VAL_DIAG_FLASH_ERR      0x0004
#define VAL_DIAG_STAT_MASK      0x7302


#define REG_DUMP(fnRead, dbgv) {            \
  SerialConsole.println("\r\nRegister Dump:\r\n"); \
  SerialConsole.println("Window 0:");              \
                                            \
  fnRead(0x00, 0x02, dbgv);                 \
  fnRead(0x00, 0x04, dbgv);                 \
  fnRead(0x00, 0x06, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x00, 0x0A, dbgv);                 \
  fnRead(0x00, 0x0E, dbgv);                 \
  fnRead(0x00, 0x10, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x00, 0x30, dbgv);                 \
  fnRead(0x00, 0x32, dbgv);                 \
  fnRead(0x00, 0x34, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x00, 0x36, dbgv);                 \
  fnRead(0x00, 0x38, dbgv);                 \
  fnRead(0x00, 0x3A, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x00, 0x3C, dbgv);                 \
  fnRead(0x00, 0x3E, dbgv);                 \
  fnRead(0x00, 0x40, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x00, 0x42, dbgv);                 \
  fnRead(0x00, 0x44, dbgv);                 \
  fnRead(0x00, 0x46, dbgv);                 \
  SerialConsole.println();                  \
                                            \
  SerialConsole.println("Window 1:\r\n");   \
  fnRead(0x01, 0x00, dbgv);                 \
  fnRead(0x01, 0x02, dbgv);                 \
  fnRead(0x01, 0x04, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x06, dbgv);                 \
  fnRead(0x01, 0x08, dbgv);                 \
  fnRead(0x01, 0x0A, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x0C, dbgv);                 \
  fnRead(0x01, 0x16, dbgv);                 \
  fnRead(0x01, 0x18, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x1A, dbgv);                 \
  fnRead(0x01, 0x1C, dbgv);                 \
  fnRead(0x01, 0x1E, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x2C, dbgv);                 \
  fnRead(0x01, 0x2E, dbgv);                 \
  fnRead(0x01, 0x30, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x32, dbgv);                 \
  fnRead(0x01, 0x34, dbgv);                 \
  fnRead(0x01, 0x36, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x46, dbgv);                 \
  fnRead(0x01, 0x48, dbgv);                 \
  fnRead(0x01, 0x4A, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x6A, dbgv);                 \
  fnRead(0x01, 0x6C, dbgv);                 \
  fnRead(0x01, 0x6E, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x70, dbgv);                 \
  fnRead(0x01, 0x72, dbgv);                 \
  fnRead(0x01, 0x74, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x76, dbgv);                 \
  fnRead(0x01, 0x78, dbgv);                 \
  fnRead(0x01, 0x7A, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x7E, dbgv);                 \
  SerialConsole.println();                  \
}


class EPSON_DEV:public UART_EPSON_COM {

 protected:
    // Stores burst flags after decodeBurstCtrl() method
    struct _burstFlag {
      // 0 = Off, 1 = On
      uint8_t nd_ea;
      uint8_t tempc;
      uint8_t acclx;
      uint8_t accly;
      uint8_t acclz;
      uint8_t inclx;
      uint8_t incly;
      uint8_t inclz;
      uint8_t count;
      uint8_t chksm;
    } _burstFlag = {0};

 public:
    EPSON_DEV(int8_t nrst, int8_t drdy):UART_EPSON_COM(nrst, drdy){};   // UART non-AUTO Mode (with DRDY)

    /**************************************************************************/
    /*!
        @brief  Decodes the DOUT_RATE register value to output rate in Hz

        @returns    float of output rate (Hz)
    */
    /**************************************************************************/
    float decodeDoutRate(void) {

      uint8_t dout_rate = (regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO)&0x0F00)>>8;
      switch (dout_rate) {
        case 2: return 1000.0; break;
        case 3: return 500.0; break;
        case 4: return 200.0; break;
        case 5: return 100.0; break;
        case 6: return 50.0; break;
        default:
    #ifdef DEBUG
        SerialConsole.print("Invalid DOUT_RATE");
    #endif //DEBUG
        return -1; break;
      }
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

      uint8_t filter_sel = regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO)&0x0F;
      switch (filter_sel) {
        case 1: sprintf(filterString, "KAISER64FC83"); break;
        case 2: sprintf(filterString, "KAISER64FC220"); break;
        case 3: sprintf(filterString, "KAISER128FC36"); break;
        case 4: sprintf(filterString, "KAISER128FC110"); break;
        case 5: sprintf(filterString, "KAISER128FC350"); break;
        case 6: sprintf(filterString, "KAISER512FC9"); break;
        case 7: sprintf(filterString, "KAISER512FC16"); break;
        case 8: sprintf(filterString, "KAISER512FC60"); break;
        case 9: sprintf(filterString, "KAISER512FC210"); break;
        case 10: sprintf(filterString, "KAISER512FC460"); break;
        case 11: sprintf(filterString, "UDF4"); break;
        case 12: sprintf(filterString, "UDF64"); break;
        case 13: sprintf(filterString, "UDF128"); break;
        case 14: sprintf(filterString, "UDF512"); break;
        default:
          sprintf(filterString, "INVALID");
          break;
      }
    }


    /**************************************************************************/
    /*!
        @brief  Decodes the BURST_CTRL register and stores
                status in _burstFlag struct and returns burst length in bytes

        @returns    sensor burst length in bytes(excluding header & delimiter byte)
    */
    /**************************************************************************/
    uint8_t decodeBurstCtrl(void) {

      uint8_t burst_len = 0;

      uint8_t sig_ctrl = regRead16(CMD_WINDOW1, ADDR_SIG_CTRL_LO)&0xE0;
      uint16_t burst_ctrl = regRead16(CMD_WINDOW1, ADDR_BURST_CTRL_LO)&0xC703;

      // burst_ctrl check
      _burstFlag.nd_ea = (burst_ctrl&0x8000) ? 1 : 0;
      _burstFlag.tempc = (burst_ctrl&0x4000) ? 1 : 0;
      _burstFlag.acclx = (burst_ctrl&0x400) ? 1 : 0;
      _burstFlag.accly = (burst_ctrl&0x200) ? 1 : 0;
      _burstFlag.acclz = (burst_ctrl&0x100) ? 1 : 0;
      _burstFlag.count = (burst_ctrl&0x2) ? 1 : 0;
      _burstFlag.chksm = (burst_ctrl&0x1) ? 1 : 0;
      // sig_ctrl check
      _burstFlag.inclx = (sig_ctrl&0x80) ? 1 : 0;
      _burstFlag.incly = (sig_ctrl&0x40) ? 1 : 0;
      _burstFlag.inclz = (sig_ctrl&0x20) ? 1 : 0;

      // Calc burst_len
      if (_burstFlag.nd_ea) burst_len += 2;
      if (_burstFlag.tempc) burst_len += 4;
      if (_burstFlag.acclx) burst_len += 4;
      if (_burstFlag.accly) burst_len += 4;
      if (_burstFlag.acclz) burst_len += 4;
      if (_burstFlag.count) burst_len += 2;
      if (_burstFlag.chksm) burst_len += 2;
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
        if (_burstFlag.nd_ea) SerialConsole.print("ND_EA: ON\t");
        else SerialConsole.print("ND_EA: OFF\t");

        if (_burstFlag.tempc) SerialConsole.print("TempC: 32\t");
        else SerialConsole.print("TempC: OFF\t");

        SerialConsole.println();
        if (_burstFlag.acclx) {
          if (_burstFlag.inclx) SerialConsole.print("IncX: 32\t");
          else SerialConsole.print("AccX: 32\t");
        }
        else SerialConsole.print("AccX: OFF\t");

        if (_burstFlag.accly) {
          if (_burstFlag.incly) SerialConsole.print("IncY: 32\t");
          else SerialConsole.print("AccY: 32\t");
        }
        else SerialConsole.print("AccY: OFF\t");

        if (_burstFlag.acclz) {
          if (_burstFlag.inclz) SerialConsole.print("IncZ: 32\t");
          else SerialConsole.print("AccZ: 32\t");
        }
        else SerialConsole.print("AccZ: OFF\t");

        SerialConsole.println();
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
        @brief  Prints sensor's scale factors
    */
    /**************************************************************************/
    void sensorScaleFactorsPrint(void){
      SerialConsole.println("*****************************************************************");
      SerialConsole.print("Accl SF: "); SerialConsole.print(float(EPSON_ACCL_SF),9); SerialConsole.println(" G/bit");
      SerialConsole.print("Tilt SF: "); SerialConsole.print(float(EPSON_TILT_SF),9); SerialConsole.println(" rad/bit");
      SerialConsole.println("*****************************************************************");
    }

    /**************************************************************************/
    /*!
        @brief  Prints header row of output data
    */
    /**************************************************************************/
    void sensorHeaderPrint(void){

      SerialConsole.println();
      SerialConsole.print("Sample#\t");
      if (_burstFlag.nd_ea) {
        SerialConsole.print("ND_EA\t");
      }

      if (_burstFlag.tempc) {
        SerialConsole.print("TempC\t\t");
      }

      if(_burstFlag.acclx) {
        if(_burstFlag.inclx) SerialConsole.print("Tilt X\t");
        else                SerialConsole.print("Accl X\t");
      }

      if(_burstFlag.accly) {
        if(_burstFlag.incly) SerialConsole.print("Tilt Y\t");
        else                SerialConsole.print("Accl Y\t");
      }

      if(_burstFlag.acclz) {
        if(_burstFlag.inclz) SerialConsole.print("Tilt Z\t");
        else                SerialConsole.print("Accl Z\t");
      }

      if(_burstFlag.count) {
        SerialConsole.print("Count\t");
      }

      if(_burstFlag.chksm) {
        SerialConsole.print("Checksum");
      }

      SerialConsole.println();
    }

    /**************************************************************************/
    /*!
        @brief  Convert and output sensor data to console

        @param [in]  data
                     Pointer to 16-bit Array
        @param [in]  sampleCount
                     Current Sample Count
    */
    void sensorDataPrint(const uint16_t* data, uint32_t sampleCount) {

      SerialConsole.print(sampleCount); SerialConsole.print("\t");

      // stores the accelerometer data array index when parsing out data fields
      int idx = 0;

      // parsing of data fields applying conversion factor if applicable
      if (_burstFlag.nd_ea) {
        //process ND flag data
        unsigned short ndflags = data[idx];
        idx += 1;
        SerialConsole.print(ndflags, HEX); SerialConsole.print("\t");
      }

      if (_burstFlag.tempc) {
        //process temperature data
        int32_t temp = (data[idx]<<16) | (data[idx+1]<<0);
        float temperature = ((float)temp*EPSON_TEMP_SF) + 34.987f;
        idx += 2;
        SerialConsole.print(temperature, 6); SerialConsole.print("\t");
      }

      if(_burstFlag.acclx) {
        //process x axis data
        float accel_x;
        int32_t x = (data[idx]<<16) | (data[idx+1]<<0);
        if(_burstFlag.inclx)
          accel_x = (EPSON_TILT_SF * (float)x); //< tilt
        else
          accel_x = (EPSON_ACCL_SF * (float)x); //< acceleration
        SerialConsole.print(accel_x, 6); SerialConsole.print("\t");
        idx += 2;
      }

      if(_burstFlag.accly) {
        //process y axis data
        float accel_y;
        int32_t y = (data[idx]<<16) | (data[idx+1]<<0);
        if(_burstFlag.incly)
          accel_y = (EPSON_TILT_SF * (float)y); //< tilt
        else
          accel_y = (EPSON_ACCL_SF * (float)y); //< acceleration
        SerialConsole.print(accel_y, 6); SerialConsole.print("\t");
        idx += 2;
      }

      if(_burstFlag.acclz) {
        //process z axis data
        float accel_z;
        int32_t z = (data[idx]<<16) | (data[idx+1]<<0);
        if(_burstFlag.inclz)
          accel_z = (EPSON_TILT_SF * (float)z); //< tilt
        else
          accel_z = (EPSON_ACCL_SF * (float)z); //< acceleration
        SerialConsole.print(accel_z, 6); SerialConsole.print("\t");
        idx += 2;
      }

      if(_burstFlag.count) {
        //process count out data
        SerialConsole.print(data[idx], DEC); SerialConsole.print("\t");
        idx += 1;
      }

      if(_burstFlag.chksm) {
        // process checksum data
        SerialConsole.print(data[idx], HEX); SerialConsole.print("\t");
      }

      SerialConsole.println();
    }

    /**************************************************************************/
    /*!
       @brief  Assumes currently in Configuration Mode to set counter, ND_Flags,
               Burst Settings, output rate & filter settings.
               For valid settings, refer to device datasheet

        @param [in]  outputRate
                     refer to possible #defines CMD_RATExxx earlier in this file
        @param [in]  filterSetting
                     refer to possible #defines CMD_FIRTAPxxxFCxxx
                     earlier in this file

        @returns     false if fail / true if success
    */
    /**************************************************************************/
    boolean sensorInit(uint8_t outputRate, uint8_t filterSetting) {

      if (filterSetting >= CMD_USERFIRTAP4)
      {
        SerialConsole.println("User filter coefficients not implemented!");
        return false;
      }

      regWrite8(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, filterSetting);

      // Delay for filter config
      EpsonFilterDelay();

      // Check that the FILTER_BUSY bit returns 0
      unsigned short rxData;
      unsigned short retryCount = 3000;
      do
      {
        rxData = regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO);
        retryCount--;
      } while((rxData & 0x0020) == 0x0020 && (retryCount != 0));

      if (retryCount == 0)
      {
        SerialConsole.println("...Error: Filter busy bit did not return to 0b.");
        return false;
      }
      regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_LO, CMD_CNTR_DRDY);
      regWrite8(CMD_WINDOW1, ADDR_SMPL_CTRL_HI, outputRate);
      regWrite8(CMD_WINDOW1, ADDR_UART_CTRL_LO, CMD_UART_AUTO_DIS);

      regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL_LO, CMD_EN_BRSTDATA_LO);
      regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL_HI, CMD_EN_BRSTDATA_HI);

      regWrite8(CMD_WINDOW1, ADDR_SIG_CTRL_LO, CMD_SIG_CTR_LO_FLAGS);
      regWrite8(CMD_WINDOW1, ADDR_SIG_CTRL_HI, CMD_EN_NDFLAGS);

      _burstCnt_calculated = decodeBurstCtrl();

      // If DRDY is not used then enable UART_AUTO mode, otherwise keep in UART Manual Mode
      if ( getDRDY() == -1 ) {
          // When DRDY is not used, enable UART AUTO mode
          regWrite8(CMD_WINDOW1, ADDR_UART_CTRL_LO, CMD_UART_AUTO_EN ); // UART_CTRL Low Byte enables/disables UART_AUTO mode
          setUARTAuto(true);
      }

      return true;
    }

    /**************************************************************************/
    /*!
        @brief  Output register values to console
    */
    /**************************************************************************/
    void registerDump(void){

      if (sensorStop()) {
        REG_DUMP(regRead16, true);
      }
      else  {
        SerialConsole.println("Warning: Not entering Config Mode");
      }
    }

};


#endif /* EPSONAXXX_H_ */




