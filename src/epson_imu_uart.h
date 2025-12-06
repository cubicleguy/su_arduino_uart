/**************************************************************************/
/*!
    @file     epson_imu_uart.h

    Header for Epson IMU Class

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

#include "epson_imu_regs.h"
#include "epson_uart_common.h"

#include <cstring>

namespace EPSON_I_U {

constexpr uint32_t READBURST_RETRIES = 100000;
constexpr int FILTERBUSY_RETRIES = 3000;
constexpr int ATTIMOTIONBUSY_RETRIES = 3000;

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
  G320PDG0,
  G320PDGN,
  G354PDH0,
  G355QDG0,
  G364PDCA,
  G364PDC0,
  G365PDC1,
  G365PDF1,
  G370PDF1,
  G370PDFN,
  G370PDS0,
  G330PDG0,
  G366PDG0,
  G370PDG0,
  G370PDT0,
  G570PR20,
  MAX_MODELS,
};

enum features {
  HAS_ATTITUDE_OUTPUT = 1,
  HAS_DLT_OUTPUT = 2,
  HAS_ATTI_ON_REG = 4,
  HAS_ROT_MATRIX = 8,
  HAS_RANGE_OVER = 16,
  HAS_RTDIAG = 32,
  HAS_ARANGE = 64,
  HAS_INITIAL_BACKUP = 128,
};

enum burst_ctrl {
  OFF = 0,
  OUT16 = 1,
  OUT32 = 2,
};

}  // namespace EPSON_I_U

// Contains Epson sensor model-specific properties
struct ImuProperties {
  EPSON_I_U::models model;
  const char* product_id;
  int feature_flags;
  double gyro_sf_dps;
  double accl_sf_mg;
  double tempc_sf_degc;
  int tempc_25c_offset;
  int rstcnt_freq_hz;
  double ang_sf_deg;
  double qtn_sf;
  double dlta0_sf_deg;
  double dltv0_sf_mps;
  int delay_reset_ms;
  int delay_flashtest_ms;
  int delay_flashbackup_ms;
  int delay_selftest_ms;
  int delay_filter_ms;
  int delay_atti_profile_ms;
};

// Contains Epson sensor initialization settings
struct ImuInitOptions {
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

  // BURST_CTRL1
  int flag_out;
  int temp_out;
  int gyro_out;
  int accel_out;
  int gyro_delta_out;
  int accel_delta_out;
  int qtn_out;
  int atti_out;

  int gpio_out;
  int count_out;
  int checksum_out;

  // BURST_CTRL2
  int temp_bit;
  int gyro_bit;
  int accel_bit;
  int gyro_delta_bit;
  int accel_delta_bit;
  int qtn_bit;
  int atti_bit;

  // POL_CTRL
  int invert_xgyro;
  int invert_ygyro;
  int invert_zgyro;
  int invert_xaccel;
  int invert_yaccel;
  int invert_zaccel;

  int a_range_ctrl;

  // DLT_CTRL
  int dlta_range_ctrl;
  int dltv_range_ctrl;

  // ATTI_CTRL
  int atti_mode;
  int atti_conv;

  // ATTI_MOTION_PROFILE
  int atti_profile;
};

// Stores burst field settings
struct ImuBurstFlag {
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
};

// Contains struct of raw sensor data
struct ImuUnscaledData {
  uint16_t nd_flags;
  int32_t tempC;
  int32_t gyroXYZ[3];
  int32_t accelXYZ[3];
  uint16_t gpio;
  uint16_t counter;
  int32_t dltaXYZ[3];
  int32_t dltvXYZ[3];
  int32_t qtn4[4];
  int32_t attiXYZ[3];
  uint16_t chksm;
};

// Contains struct of scaled sensor data
struct ImuScaledData {
  uint16_t nd_flags;
  float tempC;
  float gyroXYZ[3];
  float accelXYZ[3];
  uint16_t gpio;
  uint16_t counter;
  float dltaXYZ[3];
  float dltvXYZ[3];
  float qtn4[4];
  float attiXYZ[3];
  uint16_t chksm;
};

//------------------------
// EPSON_IMU_UART class
//------------------------

class EPSON_IMU_UART : public UART_EPSON_COM {
 public:
  EPSON_IMU_UART();  // nrst and drdy pins unused
  EPSON_IMU_UART(HardwareSerial& uartPort, uint32_t baudRate, int8_t nrst,
                 int8_t drdy, Stream& consolePort);

  uint8_t sensorGetBurstLength(void) { return _read_burst8_len; };
  void sensorScaleFactorsPrint(void);
  float sensorDecodeDoutRate(void);
  void sensorDecodeFilterSel(char* filterString, const size_t length);

  void sensorConfigPrint(void);
  void sensorHeaderPrint(void);
  void sensorDataPrint(const struct ImuScaledData* scaledField,
                       uint32_t sampleIndex);
  boolean sensorGetSensorBurst(struct ImuScaledData& scaledField);

  boolean sensorInitOptions(struct ImuInitOptions*, boolean verbose = false);
  void sensorRegisterDump(void);
  boolean sensorModelSelect(int model);
  int sensorModelDetect(void);
  boolean sensorStart(boolean verbose = false);
  boolean sensorStop(boolean verbose = false);
  uint16_t sensorSelfTest(boolean verbose = false);
  boolean sensorReset(boolean verbose = false);
  boolean sensorFlashTest(boolean verbose = false);
  void sensorGetProdID(char* prodID, const size_t length = 9);
  uint16_t sensorGetVersion(void);
  void sensorGetSerialNumber(char* serialNumber, const size_t length = 9);

 private:
  int8_t _sensorDecodeBurstCtrl(boolean verbose = false);
  boolean _sensorReadBurst16(uint16_t* arr, const size_t length);
  void _sensorBuf2Field(struct ImuUnscaledData* unscaledField,
                        const uint16_t* readBuf, const size_t length);
  void _sensorField2Scaled(struct ImuScaledData* scaledField,
                           const struct ImuUnscaledData* unscaledField);

  // Stores current sensor read burst
  uint16_t _read_burst[64] = {};

  // Stores current unscaled sensor burst
  ImuUnscaledData _unscaled = {};

  // Stores state of burst flags after decodeBurstCtrl() method
  ImuBurstFlag _burst_flag = {};

  // stores output from sensorModelSelect()
  ImuProperties _dev_prop = {};

  // length of a read burst in bytes
  uint8_t _read_burst8_len = 0;

  // Stores UART_AUTO status after init
  boolean _uart_auto = false;

  // Serial console
  Stream& _consolePort;
};
