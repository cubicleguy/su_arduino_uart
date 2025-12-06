/**************************************************************************/
/*!
    @file     epson_vibe_uart.h

    Header file for Epson vibration sensor class

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

#include "epson_vibe_regs.h"
#include "epson_uart_common.h"

#include <cstring>
#include <math.h>

namespace EPSON_V_U {

constexpr uint32_t READBURST_RETRIES = 1000000;

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
  A342VD10,
  MAX_MODELS,
};

enum features {
  HAS_ALIGNMENT_COMP = 1,
};

enum burst_ctrl {
  OFF = 0,
  OUT16 = 1,
  OUT32 = 2,
};

}  // namespace EPSON_V_U

// Contains Epson sensor model-specific properties
struct VibeProperties {
  EPSON_V_U::models model;
  const char* product_id;
  int feature_flags;
  double vel_sf_m_s;
  double disp_sf_m;
  double tempc_sf_degc;
  int tempc_25c_offset;
  int delay_reset_ms;
  int delay_flashbackup_ms;
  int delay_flashreset_ms;
  int delay_flashtest_ms;
  int delay_selftest_ms;
  int delay_structres_test_ms;
  int delay_outputsel_ms;
  int delay_wakeup_ms;
};

// Contains Epson sensor initialization settings
struct VibeInitOptions {
  // SIG_STRL
  int output_sel;
  int temp_sel;

  // MSC_CTRL
  int ext_pol;
  int drdy_on;
  int drdy_pol;

  // SMPL_CTRL
  int dout_rate_rmspp;
  int update_rate_rmspp;

  // UART_CTRL
  int uart_auto;

  // BURST_CTRL
  int flag_out;
  int temp_out;
  int sensx_out;
  int sensy_out;
  int sensz_out;
  int count_out;
  int checksum_out;

  // ALARM (16-bit unsigned fixed point 2-bit unsigned int 14-bit fractional)
  int xalarm;
  int yalarm;
  int zalarm;
};

// Stores burst field settings
struct VibeBurstFlag {
  // 0 = Off, 1 = On
  uint8_t nd_ea;
  uint8_t tempc;
  uint8_t sensx;
  uint8_t sensy;
  uint8_t sensz;
  uint8_t count;
  uint8_t chksm;
};

// Contains struct of raw sensor data
struct VibeUnscaledData {
  uint16_t nd_flags;
  int16_t tempC;
  uint8_t errEXI;
  uint8_t errALARM;
  uint8_t two_bit_count;
  int32_t sensXYZ[3];
  uint16_t counter;
  uint16_t chksm;
};

// Contains struct of scaled sensor data
struct VibeScaledData {
  uint16_t nd_flags;
  float tempC;
  uint8_t errEXI;
  uint8_t errALARM;
  uint8_t two_bit_count;
  double sensXYZ[3];
  uint16_t counter;
  uint16_t chksm;
};

//------------------------
// EPSON_VIBE_UART class
//------------------------

class EPSON_VIBE_UART : public UART_EPSON_COM {
 public:
  EPSON_VIBE_UART();  // nrst and drdy pins unused
  EPSON_VIBE_UART(HardwareSerial& uartPort, uint32_t baudRate, int8_t nrst,
                  int8_t drdy, Stream& consolePort);

  uint8_t sensorGetBurstLength(void) { return _read_burst8_len; };
  void sensorScaleFactorsPrint(void);
  float sensorDecodeDoutRate(void);
  float sensorDecodeUpdateRate(void);

  void sensorConfigPrint(void);
  void sensorHeaderPrint(void);
  void sensorDataPrint(const struct VibeScaledData* scaledField,
                       uint32_t sampleIndex);
  boolean sensorGetSensorBurst(struct VibeScaledData& scaledField);

  boolean sensorInitOptions(const struct VibeInitOptions*,
                            boolean verbose = false);
  void sensorRegisterDump(void);
  boolean sensorModelSelect(int model);
  int sensorModelDetect(void);
  boolean sensorStart(boolean verbose = false);
  boolean sensorStop(boolean verbose = false);
  uint16_t sensorSelfTest(boolean verbose = false);
  uint16_t sensorStructuralResTest(boolean verbose = false);
  boolean sensorReset(boolean verbose = false);
  boolean sensorFlashTest(boolean verbose = false);
  void sensorGetProdID(char* prodID, const size_t length = 9);
  uint16_t sensorGetVersion(void);
  void sensorGetSerialNumber(char* serialNumber, const size_t length = 9);

 private:
  int8_t _sensorDecodeBurstCtrl(boolean verbose = false);
  boolean _sensorReadBurst16(uint16_t* arr, const size_t length);
  void _sensorBuf2Field(struct VibeUnscaledData* unscaledField,
                        const uint16_t* readBuf, const size_t length);
  void _sensorField2Scaled(struct VibeScaledData* scaledField,
                           const struct VibeUnscaledData* unscaledField);

  // Stores current sensor read burst
  uint16_t _read_burst[64] = {};

  // Stores current unscaled sensor burst
  VibeUnscaledData _unscaled = {};

  // Stores state of burst flags after decodeBurstCtrl() method
  VibeBurstFlag _burst_flag = {};

  // stores output from sensorModelSelect()
  VibeProperties _dev_prop = {};

  // length of a read burst in bytes
  uint8_t _read_burst8_len = 0;

  // Stores UART_AUTO status after init
  boolean _uart_auto = false;

  // Store output_sel status
  uint8_t _output_sel = 4;

  // Store temp_sel status
  boolean _temp_sel = 0;

  // Serial console
  Stream& _consolePort;
};
