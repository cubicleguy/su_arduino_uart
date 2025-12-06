/**************************************************************************/
/*!
    @file     epson_accl_uart.h

    Header for Epson accelerometer class

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

#include "epson_accl_regs.h"
#include "epson_uart_common.h"

#include <cstring>

namespace EPSON_A_U {

constexpr uint32_t READBURST_RETRIES = 100000;
constexpr int FILTERBUSY_RETRIES = 3000;

constexpr uint16_t BIT_0 = (1);
constexpr uint16_t BIT_1 = (1 << 1);
constexpr uint16_t BIT_2 = (1 << 2);
constexpr uint16_t BIT_3 = (1 << 3);
constexpr uint16_t BIT_4 = (1 << 4);
constexpr uint16_t BIT_5 = (1 << 5);
constexpr uint16_t BIT_6 = (1 << 6);
constexpr uint16_t BIT_7 = (1 << 7);
constexpr uint16_t BIT_8 = (1 << 8);
constexpr uint16_t BIT_9 = (1 << 9);
constexpr uint16_t BIT_10 = (1 << 10);
constexpr uint16_t BIT_11 = (1 << 11);
constexpr uint16_t BIT_12 = (1 << 12);
constexpr uint16_t BIT_13 = (1 << 13);
constexpr uint16_t BIT_14 = (1 << 14);
constexpr uint16_t BIT_15 = (1 << 15);

enum models {
  INVALID,
  A352AD10,
  A370AD10,
  MAX_MODELS,
};

enum features {
  HAS_REDUCED_NOISE = 1,
  HAS_BIAS_TEMP_SHOCK = 2,
  HAS_PPS_INPUT = 4,
};

enum burst_ctrl {
  OFF = 0,
  OUT16 = 1,
  OUT32 = 2,
};

}  // namespace EPSON_A_U

// Contains Epson sensor model-specific properties
struct AcclProperties {
  EPSON_A_U::models model;
  const char* product_id;
  int feature_flags;
  double accl_sf_g;
  double tempc_sf_degc;
  int tempc_25c_offset;
  double tilt_sf_rad;
  int delay_reset_ms;
  int delay_flashtest_ms;
  int delay_flashbackup_ms;
  int delay_flashreset_ms;
  int delay_selftest_ms;
  int delay_sensitivity_ms;
  int delay_filter_ms;
  int delay_UDF_write_ms;
  int delay_UDF_read_us;
  int delay_wakeup_ms;
};

// Contains Epson sensor initialization settings
struct AcclInitOptions {
  // SIG_STRL
  // 3-bit value bit[2:0] = x, y, z
  int incl_sel;
  int reduced_noise;
  int tempc_stabil;

  // MSC_CTRL
  int ext_sel;
  int ext_pol;
  int drdy_on;
  int drdy_pol;

  // SMPL_CTRL
  int dout_rate;

  // FILTER_CTRL
  int filter_sel;

  // UART_CTRL
  int uart_auto;

  // BURST_CTRL
  int flag_out;
  int temp_out;
  int accelx_out;
  int accely_out;
  int accelz_out;
  int count_out;
  int checksum_out;

  // LONGFILT
  int lfilt_sel;
  int lfilt_en;
  int lfilt_tap;

  // A_OFFSET (32-bit fixed-point, 8-bit decimal, 24-bit fractional)
  int xoffset;
  int yoffset;
  int zoffset;

  // A_ALARM (8-bit integer +/-15G)
  int xalarm_upper;
  int xalarm_lower;

  int yalarm_upper;
  int yalarm_lower;

  int zalarm_upper;
  int zalarm_lower;
};

// Stores burst field settings
struct AcclBurstFlag {
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
};

// Contains struct of raw sensor data
struct AcclUnscaledData {
  uint16_t nd_flags;
  int32_t tempC;
  int32_t acclXYZ[3];
  int32_t inclXYZ[3];
  uint16_t counter;
  uint16_t chksm;
};

// Contains struct of scaled sensor data
struct AcclScaledData {
  uint16_t nd_flags;
  float tempC;
  float acclXYZ[3];
  float inclXYZ[3];
  uint16_t counter;
  uint16_t chksm;
};

//------------------------
// EPSON_ACCL_UART class
//------------------------

class EPSON_ACCL_UART : public UART_EPSON_COM {
 public:
  EPSON_ACCL_UART();  // nrst and drdy pins unused
  EPSON_ACCL_UART(HardwareSerial& uartPort, uint32_t baudRate, int8_t nrst,
                  int8_t drdy, Stream& consolePort);

  uint8_t sensorGetBurstLength(void) { return _read_burst8_len; };
  void sensorScaleFactorsPrint(void);
  float sensorDecodeDoutRate(void);
  void sensorDecodeFilterSel(char* filterString, const size_t length);

  void sensorConfigPrint(void);
  void sensorHeaderPrint(void);
  void sensorDataPrint(const struct AcclScaledData* scaledField,
                       uint32_t sampleIndex);
  boolean sensorGetSensorBurst(struct AcclScaledData& scaledField);

  boolean sensorInitOptions(const struct AcclInitOptions*,
                            boolean verbose = false);
  void sensorRegisterDump(void);
  boolean sensorModelSelect(int model);
  int sensorModelDetect(void);
  boolean sensorStart(boolean verbose = false);
  boolean sensorStop(boolean verbose = false);
  uint16_t sensorSelfTest(boolean verbose = false);
  uint16_t sensorSensitivityTest(boolean verbose = false);
  boolean sensorReset(boolean verbose = false);
  boolean sensorFlashTest(boolean verbose = false);
  void sensorGetProdID(char* prodID, const size_t length = 9);
  uint16_t sensorGetVersion(void);
  void sensorGetSerialNumber(char* serialNumber, const size_t length = 9);

 private:
  int8_t _sensorDecodeBurstCtrl(boolean verbose = false);
  boolean _sensorReadBurst16(uint16_t* arr, const size_t length);
  void _sensorBuf2Field(struct AcclUnscaledData* unscaledField,
                        const uint16_t* readBuf, const size_t length);
  void _sensorField2Scaled(struct AcclScaledData* scaledField,
                           const struct AcclUnscaledData* unscaledField);

  // Stores current sensor read burst
  uint16_t _read_burst[64] = {};

  // Stores current unscaled sensor burst
  AcclUnscaledData _unscaled = {};

  // Stores state of burst flags after decodeBurstCtrl() method
  AcclBurstFlag _burst_flag = {};

  // Stores output from sensorModelSelect()
  AcclProperties _dev_prop = {};

  // Length of a read burst in bytes
  uint8_t _read_burst8_len = 0;

  // Stores UART_AUTO status after init
  boolean _uart_auto = false;

  // Serial console
  Stream& _consolePort;
};
