/**************************************************************************/
/*!
    @file     epson_imu_uart.cpp

    Epson IMU Class

    @section  HISTORY

    v1.0 - First release
    v1.0.1 - Remove redundant break

    @section LICENSE

    Software License Agreement (BSD License, see license.txt)

    Copyright (c) 2025, 2026 Seiko Epson Corporation.
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

#include "epson_imu_uart.h"

using namespace EPSON_I_U;

//----------------------------------------------------------------------
// Array of Epson device properties struct for models
//----------------------------------------------------------------------

struct ImuProperties imu_model[] =
  {
    [INVALID] =
      {
        .model = INVALID,
        .product_id = "UNKNOWN",
        .feature_flags = 0,
        .gyro_sf_dps = (1),
        .accl_sf_mg = (1),
        .tempc_sf_degc = (1),
        .tempc_25c_offset = (0),
        .rstcnt_freq_hz = (1),
        .ang_sf_deg = (1),
        .qtn_sf = (1),
        .dlta0_sf_deg = (1),
        .dltv0_sf_mps = (1),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (30),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (150),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (1),
      },
    [G320PDG0] =
      {
        .model = G320PDG0,
        .product_id = "G320PDG0",
        .feature_flags = (HAS_DLT_OUTPUT),
        .gyro_sf_dps = (1.0 / 125),
        .accl_sf_mg = (1.0 / 5),
        .tempc_sf_degc = (-0.0037918),
        .tempc_25c_offset = (2364),
        .rstcnt_freq_hz = (46875),
        .ang_sf_deg = (0),
        .qtn_sf = (0),
        .dlta0_sf_deg = (1.0 / 125 * 1 / 2000),
        .dltv0_sf_mps = (1.0 / 5 * 1 / 1000 * 1 / 2000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (5),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (0),
      },
    [G320PDGN] =
      {
        .model = G320PDGN,
        .product_id = "G320PDGN",
        .feature_flags = (HAS_DLT_OUTPUT),
        .gyro_sf_dps = (1.0 / 125),
        .accl_sf_mg = (1.0 / 5),
        .tempc_sf_degc = (-0.0037918),
        .tempc_25c_offset = (2364),
        .rstcnt_freq_hz = (46875),
        .ang_sf_deg = (0),
        .qtn_sf = (0),
        .dlta0_sf_deg = (1.0 / 125 * 1 / 2000),
        .dltv0_sf_mps = (1.0 / 5 * 1 / 1000 * 1 / 2000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (5),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (0),
      },
    [G354PDH0] =
      {
        .model = G354PDH0,
        .product_id = "G354PDH0",
        .feature_flags = (HAS_DLT_OUTPUT),
        .gyro_sf_dps = (1.0 / 62.5),
        .accl_sf_mg = (1.0 / 5),
        .tempc_sf_degc = (-0.0037918),
        .tempc_25c_offset = (2364),
        .rstcnt_freq_hz = (46875),
        .ang_sf_deg = (0),
        .qtn_sf = (0),
        .dlta0_sf_deg = (1.0 / 62.5 * 1 / 2000),
        .dltv0_sf_mps = (1.0 / 5 * 1 / 1000 * 1 / 2000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (5),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (0),
      },
    [G355QDG0] =
      {
        .model = G355QDG0,
        .product_id = "G355QDG0",
        .feature_flags =
          (HAS_RANGE_OVER | HAS_ARANGE | HAS_INITIAL_BACKUP | HAS_RTDIAG),
        .gyro_sf_dps = (1.0 / 66),
        .accl_sf_mg = (1.0 / 4),
        .tempc_sf_degc = (0.00390625),
        .tempc_25c_offset = (0),
        .rstcnt_freq_hz = (62500),
        .ang_sf_deg = (0),
        .qtn_sf = (0),
        .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
        .dltv0_sf_mps = (1.0 / 4 * 1 / 1000 * 1 / 2000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (30),
        .delay_flashbackup_ms = (300),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (0),
      },
    [G364PDCA] =
      {
        .model = G364PDCA,
        .product_id = "G364PDCA",
        .feature_flags = (HAS_DLT_OUTPUT),
        .gyro_sf_dps = (0.00375),
        .accl_sf_mg = (1.0 / 8),
        .tempc_sf_degc = (-0.0037918),
        .tempc_25c_offset = (2364),
        .rstcnt_freq_hz = (46875),
        .ang_sf_deg = (0),
        .qtn_sf = (0),
        .dlta0_sf_deg = (0.00375 * 1 / 2000),
        .dltv0_sf_mps = (1.0 / 8 * 1 / 1000 * 1 / 2000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (5),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (0),
      },
    [G364PDC0] =
      {
        .model = G364PDC0,
        .product_id = "G364PDC0",
        .feature_flags = (HAS_DLT_OUTPUT),
        .gyro_sf_dps = (0.00750),
        .accl_sf_mg = (1.0 / 8),
        .tempc_sf_degc = (-0.0037918),
        .tempc_25c_offset = (2364),
        .rstcnt_freq_hz = (46875),
        .ang_sf_deg = (0),
        .qtn_sf = (0),
        .dlta0_sf_deg = (0.00750 * 1 / 2000),
        .dltv0_sf_mps = (1.0 / 8 * 1 / 1000 * 1 / 2000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (5),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (0),
      },
    [G365PDC1] =
      {
        .model = G365PDC1,
        .product_id = "G365PDC1",
        .feature_flags =
          (HAS_ATTITUDE_OUTPUT | HAS_DLT_OUTPUT | HAS_ATTI_ON_REG |
           HAS_ROT_MATRIX | HAS_RANGE_OVER | HAS_INITIAL_BACKUP),
        .gyro_sf_dps = (1.0 / 66),
        .accl_sf_mg = (1.0 / 6.25),
        .tempc_sf_degc = (-0.0037918),
        .tempc_25c_offset = (2364),
        .rstcnt_freq_hz = (62500),
        .ang_sf_deg = (0.00699411),
        .qtn_sf = (1.0 / (2 << 13)),
        .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
        .dltv0_sf_mps = (1.0 / 6.25 * 1 / 1000 * 1 / 2000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (5),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (1),
      },
    [G365PDF1] =
      {
        .model = G365PDF1,
        .product_id = "G365PDF1",
        .feature_flags =
          (HAS_ATTITUDE_OUTPUT | HAS_DLT_OUTPUT | HAS_ATTI_ON_REG |
           HAS_ROT_MATRIX | HAS_RANGE_OVER | HAS_INITIAL_BACKUP),
        .gyro_sf_dps = (1.0 / 66),
        .accl_sf_mg = (1.0 / 2.5),
        .tempc_sf_degc = (-0.0037918),
        .tempc_25c_offset = (2364),
        .rstcnt_freq_hz = (62500),
        .ang_sf_deg = (0.00699411),
        .qtn_sf = (1.0 / (2 << 13)),
        .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
        .dltv0_sf_mps = (1.0 / 2.5 * 1 / 1000 * 1 / 2000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (5),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (1),
      },
    [G370PDF1] =
      {
        .model = G370PDF1,
        .product_id = "G370PDF1",
        .feature_flags =
          (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG | HAS_ROT_MATRIX |
           HAS_RANGE_OVER | HAS_RTDIAG | HAS_INITIAL_BACKUP),
        .gyro_sf_dps = (1.0 / 66),
        .accl_sf_mg = (1.0 / 2.5),
        .tempc_sf_degc = (-0.0037918),
        .tempc_25c_offset = (2364),
        .rstcnt_freq_hz = (62500),
        .ang_sf_deg = (0),
        .qtn_sf = (0),
        .dlta0_sf_deg = (1.0 / 66 * 1 / 1000),
        .dltv0_sf_mps = (1.0 / 2.5 * 1 / 1000 * 1 / 1000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (5),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (0),
      },
    [G370PDFN] =
      {
        .model = G370PDFN,
        .product_id = "G370PDFN",
        .feature_flags =
          (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG | HAS_ROT_MATRIX |
           HAS_RANGE_OVER | HAS_RTDIAG | HAS_INITIAL_BACKUP),
        .gyro_sf_dps = (1.0 / 66),
        .accl_sf_mg = (1.0 / 2.5),
        .tempc_sf_degc = (-0.0037918),
        .tempc_25c_offset = (2364),
        .rstcnt_freq_hz = (62500),
        .ang_sf_deg = (0),
        .qtn_sf = (0),
        .dlta0_sf_deg = (1.0 / 66 * 1 / 1000),
        .dltv0_sf_mps = (1.0 / 2.5 * 1 / 1000 * 1 / 1000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (5),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (150),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (0),
      },
    [G370PDS0] =
      {
        .model = G370PDS0,
        .product_id = "G370PDS0",
        .feature_flags =
          (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG | HAS_ROT_MATRIX |
           HAS_RANGE_OVER | HAS_RTDIAG | HAS_INITIAL_BACKUP),
        .gyro_sf_dps = (1.0 / 150),
        .accl_sf_mg = (1.0 / 2.5),
        .tempc_sf_degc = (-0.0037918),
        .tempc_25c_offset = (2364),
        .rstcnt_freq_hz = (62500),
        .ang_sf_deg = (0),
        .qtn_sf = (0),
        .dlta0_sf_deg = (1.0 / 150 * 1 / 1000),
        .dltv0_sf_mps = (1.0 / 2.5 * 1 / 1000 * 1 / 1000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (5),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (150),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (0),
      },
    [G330PDG0] =
      {
        .model = G330PDG0,
        .product_id = "G330PDG0",
        .feature_flags =
          (HAS_ATTITUDE_OUTPUT | HAS_DLT_OUTPUT | HAS_ATTI_ON_REG |
           HAS_ROT_MATRIX | HAS_RANGE_OVER | HAS_ARANGE | HAS_INITIAL_BACKUP),
        .gyro_sf_dps = (1.0 / 66),
        .accl_sf_mg = (1.0 / 4),
        .tempc_sf_degc = (0.00390625),
        .tempc_25c_offset = (0),
        .rstcnt_freq_hz = (62500),
        .ang_sf_deg = (0.00699411),
        .qtn_sf = (1.0 / (2 << 13)),
        .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
        .dltv0_sf_mps = (1.0 / 4 * 1 / 1000 * 1 / 2000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (30),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (1),
      },
    [G366PDG0] =
      {
        .model = G366PDG0,
        .product_id = "G366PDG0",
        .feature_flags =
          (HAS_ATTITUDE_OUTPUT | HAS_DLT_OUTPUT | HAS_ATTI_ON_REG |
           HAS_ROT_MATRIX | HAS_RANGE_OVER | HAS_ARANGE | HAS_INITIAL_BACKUP),
        .gyro_sf_dps = (1.0 / 66),
        .accl_sf_mg = (1.0 / 4),
        .tempc_sf_degc = (0.00390625),
        .tempc_25c_offset = (0),
        .rstcnt_freq_hz = (62500),
        .ang_sf_deg = (0.00699411),
        .qtn_sf = (1.0 / (2 << 13)),
        .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
        .dltv0_sf_mps = (1.0 / 4 * 1 / 1000 * 1 / 2000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (30),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (1),
      },
    [G370PDG0] =
      {
        .model = G370PDG0,
        .product_id = "G370PDG0",
        .feature_flags =
          (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG | HAS_ROT_MATRIX |
           HAS_RANGE_OVER | HAS_ARANGE | HAS_INITIAL_BACKUP),
        .gyro_sf_dps = (1.0 / 66),
        .accl_sf_mg = (1.0 / 4),
        .tempc_sf_degc = (0.00390625),
        .tempc_25c_offset = (0),
        .rstcnt_freq_hz = (62500),
        .ang_sf_deg = (0),
        .qtn_sf = (0),
        .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
        .dltv0_sf_mps = (1.0 / 4 * 1 / 1000 * 1 / 2000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (30),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (0),
      },
    [G370PDT0] =
      {
        .model = G370PDT0,
        .product_id = "G370PDT0",
        .feature_flags =
          (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG | HAS_ROT_MATRIX |
           HAS_RANGE_OVER | HAS_ARANGE | HAS_INITIAL_BACKUP),
        .gyro_sf_dps = (1.0 / 150),
        .accl_sf_mg = (1.0 / 4),
        .tempc_sf_degc = (0.00390625),
        .tempc_25c_offset = (0),
        .rstcnt_freq_hz = (62500),
        .ang_sf_deg = (0),
        .qtn_sf = (0),
        .dlta0_sf_deg = (1.0 / 66 * 1 / 2000),
        .dltv0_sf_mps = (1.0 / 4 * 1 / 1000 * 1 / 2000 * 9.80665),
        .delay_reset_ms = (800),
        .delay_flashtest_ms = (30),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (80),
        .delay_filter_ms = (1),
        .delay_atti_profile_ms = (0),
      },
    [G570PR20] =
      {
        .model = G570PR20,
        .product_id = "G570PR20",
        .feature_flags = (HAS_ROT_MATRIX | HAS_RANGE_OVER | HAS_INITIAL_BACKUP),
        .gyro_sf_dps = (1.0 / 66),
        .accl_sf_mg = (1.0 / 2),
        .tempc_sf_degc = (0.00390625),
        .tempc_25c_offset = (0),
        .rstcnt_freq_hz = (0),
        .ang_sf_deg = (0),
        .qtn_sf = (0),
        .dlta0_sf_deg = (0),
        .dltv0_sf_mps = (0),
        .delay_reset_ms = (5000),
        .delay_flashtest_ms = (0),
        .delay_flashbackup_ms = (200),
        .delay_selftest_ms = (10000),
        .delay_filter_ms = (0),
        .delay_atti_profile_ms = (0),
      },
};

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/

EPSON_IMU_UART::EPSON_IMU_UART()
    :  // initializer list
      UART_EPSON_COM(Serial1, 460800, -1, -1, Serial),
      _dev_prop(imu_model[INVALID]),
      _consolePort(Serial) {};

EPSON_IMU_UART::EPSON_IMU_UART(HardwareSerial& uartPort, uint32_t baudRate,
                               int8_t nrst, int8_t drdy, Stream& consolePort)
    :  // initializer list
      UART_EPSON_COM(uartPort, baudRate, nrst, drdy, consolePort),
      _dev_prop(imu_model[INVALID]),
      _consolePort(consolePort) {};

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
  @brief  Output the sensor scalefactors to console
*/
/**************************************************************************/
void EPSON_IMU_UART::sensorScaleFactorsPrint(void) {
  _consolePort.println(
    "*****************************************************************");
  _consolePort.print("Gyro SF: ");
  _consolePort.print(_dev_prop.gyro_sf_dps, 8);
  _consolePort.println(" dps/bit");
  _consolePort.print("Accl SF: ");
  _consolePort.print(_dev_prop.accl_sf_mg, 8);
  _consolePort.println(" mG/bit");
  if (_dev_prop.feature_flags & HAS_ATTITUDE_OUTPUT) {
    _consolePort.print("ATTI SF: ");
    _consolePort.print(_dev_prop.ang_sf_deg, 8);
    _consolePort.println(" deg/bit");
    _consolePort.print("QTN SF: ");
    _consolePort.print(_dev_prop.qtn_sf, 8);
    _consolePort.println();
  }
  if (_dev_prop.feature_flags & HAS_DLT_OUTPUT) {
    _consolePort.print("DLTA SF: ");
    _consolePort.print(_dev_prop.dlta0_sf_deg, 8);
    _consolePort.println(" deg/bit");
    _consolePort.print("DLTV SF: ");
    _consolePort.print(_dev_prop.dltv0_sf_mps, 8);
    _consolePort.println(" (m/s)/bit");
  };
  _consolePort.print("TempC SF: ");
  _consolePort.print(_dev_prop.tempc_sf_degc, 8);
  _consolePort.println(" degC/bit");
  _consolePort.print("TempC 25C Offset: ");
  _consolePort.print(_dev_prop.tempc_25c_offset, DEC);
  _consolePort.println(" bits");
  _consolePort.print("Reset Counter Freq: ");
  _consolePort.print(_dev_prop.rstcnt_freq_hz, DEC);
  _consolePort.println(" Hz");
  _consolePort.println(
    "*****************************************************************");
}

/**************************************************************************/
/*!
    @brief  Decodes the DOUT_RATE register value to output rate in Hz

    @returns    float of output rate (Hz)
*/
/**************************************************************************/
float EPSON_IMU_UART::sensorDecodeDoutRate(void) {
  uint8_t dout_rate = (regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO) & 0x0F00) >> 8;
  switch (dout_rate) {
    case 0:
      return 2000.0;
    case 1:
      return 1000.0;
    case 2:
      return 500.0;
    case 3:
      return 250.0;
    case 4:
      return 125.0;
    case 5:
      return 62.5;
    case 6:
      return 31.25;
    case 7:
      return 15.625;
    case 8:
      return 400.0;
    case 9:
      return 200.0;
    case 0xA:
      return 100.0;
    case 0xB:
      return 80.0;
    case 0xC:
      return 50.0;
    case 0xD:
      return 40.0;
    case 0xE:
      return 25.0;
    case 0xF:
      return 20.0;
    default:
      _consolePort.print("Invalid DOUT_RATE");
      return -1;
  }
}

/**************************************************************************/
/*!
  @brief  Decodes the FILTER_SEL register value to return filter in
          as decoded string value

  @param [in]  filterString
               Pointer to string to store decoded filter
               setting
  @param [in]  length (minimum at least 15 bytes)
               Length of the string

*/
/**************************************************************************/
void EPSON_IMU_UART::sensorDecodeFilterSel(char* filterString,
                                           const size_t length) {
  if (length < 15) {
    _consolePort.println(
      "Error: char array size must be at least 15 bytes. Bypassing...");
    return;
  }
  uint8_t filter_sel = regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO) & 0x1F;
  boolean is_moving_avg = (filter_sel <= 7);
  boolean is_G370PDF1_G370PDS0 =
    ((_dev_prop.model == G370PDF1) || (_dev_prop.model == G370PDS0));

  // Moving Average Filter decodes the same for all models
  if (is_moving_avg) {
    uint8_t filter_val = 2 << (filter_sel - 1);
    sprintf(filterString, "MVAVG_TAP%d", filter_val);
  } else if (is_G370PDF1_G370PDS0) {
    // For G370PDF1/G370PDS0 and DOUT_RATE is 2000, 400, 80sps
    uint8_t dout_rate =
      (regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO) & 0x0F00) >> 8;
    boolean is_dout_rate_2000_400_80 =
      ((dout_rate == 0) || (dout_rate == 8) || (dout_rate == 0xB));

    if (is_dout_rate_2000_400_80) {
      switch (filter_sel) {
        case 8:
          sprintf(filterString, "KAISER32FC50");
          break;
        case 9:
          sprintf(filterString, "KAISER32FC100");
          break;
        case 0xA:
          sprintf(filterString, "KAISER32FC200");
          break;
        case 0xB:
          sprintf(filterString, "KAISER32FC400");
          break;
        case 0xC:
          sprintf(filterString, "KAISER64FC50");
          break;
        case 0xD:
          sprintf(filterString, "KAISER64FC100");
          break;
        case 0xE:
          sprintf(filterString, "KAISER64FC200");
          break;
        case 0xF:
          sprintf(filterString, "KAISER64FC400");
          break;
        case 0x10:
          sprintf(filterString, "KAISER128FC50");
          break;
        case 0x11:
          sprintf(filterString, "KAISER128FC100");
          break;
        case 0x12:
          sprintf(filterString, "KAISER128FC200");
          break;
        case 0x13:
          sprintf(filterString, "KAISER128FC400");
          break;
        default:
          sprintf(filterString, "INVALID");
          break;
      }
    } else {
      // For G370PDF1/G370PDS0 and DOUT_RATE is !(2000, 400, 80sps)
      switch (filter_sel) {
        case 8:
          sprintf(filterString, "KAISER32FC25");
          break;
        case 9:
          sprintf(filterString, "KAISER32FC50");
          break;
        case 0xA:
          sprintf(filterString, "KAISER32FC100");
          break;
        case 0xB:
          sprintf(filterString, "KAISER32FC200");
          break;
        case 0xC:
          sprintf(filterString, "KAISER64FC25");
          break;
        case 0xD:
          sprintf(filterString, "KAISER64FC50");
          break;
        case 0xE:
          sprintf(filterString, "KAISER64FC100");
          break;
        case 0xF:
          sprintf(filterString, "KAISER64FC200");
          break;
        case 0x10:
          sprintf(filterString, "KAISER128FC25");
          break;
        case 0x11:
          sprintf(filterString, "KAISER128FC50");
          break;
        case 0x12:
          sprintf(filterString, "KAISER128FC100");
          break;
        case 0x13:
          sprintf(filterString, "KAISER128FC200");
          break;
        default:
          sprintf(filterString, "INVALID");
          break;
      }
    }
  } else {
    // For all other sensor models KAISER filter decodes the same
    switch (filter_sel) {
      case 8:
        sprintf(filterString, "KAISER32FC50");
        break;
      case 9:
        sprintf(filterString, "KAISER32FC100");
        break;
      case 0xA:
        sprintf(filterString, "KAISER32FC200");
        break;
      case 0xB:
        sprintf(filterString, "KAISER32FC400");
        break;
      case 0xC:
        sprintf(filterString, "KAISER64FC50");
        break;
      case 0xD:
        sprintf(filterString, "KAISER64FC100");
        break;
      case 0xE:
        sprintf(filterString, "KAISER64FC200");
        break;
      case 0xF:
        sprintf(filterString, "KAISER64FC400");
        break;
      case 0x10:
        sprintf(filterString, "KAISER128FC50");
        break;
      case 0x11:
        sprintf(filterString, "KAISER128FC100");
        break;
      case 0x12:
        sprintf(filterString, "KAISER128FC200");
        break;
      case 0x13:
        sprintf(filterString, "KAISER128FC400");
        break;
      default:
        sprintf(filterString, "INVALID");
        break;
    }
  }
}

/**************************************************************************/
/*!
    @brief      Enters Config Mode to check and output
                sensor configuration in table format to console
*/
/**************************************************************************/
void EPSON_IMU_UART::sensorConfigPrint(void) {
  if (sensorStop() == true) {
    // Print to formatted table
    _consolePort.println(
      "\n****************************************************************"
      "*");
    char bufLine[80];
    char bufString[16];
    char prod_id[] = "XXXXXXXX";
    char serial_id[] = "XXXXXXXX";

    sensorGetProdID(prod_id, sizeof(prod_id));
    sensorGetSerialNumber(serial_id, sizeof(serial_id));
    sprintf(bufLine, "PROD_ID: %s\tSERIAL_ID: %s\tVERSION: %x", prod_id,
            serial_id, sensorGetVersion());
    _consolePort.println(bufLine);

    sensorDecodeFilterSel(bufString, sizeof(bufString));
    sprintf(bufLine, "DOUT_RATE: %0.3f\tFILTER_SEL: %s", sensorDecodeDoutRate(),
            bufString);
    _consolePort.println(bufLine);

    _read_burst8_len = _sensorDecodeBurstCtrl();
    if (_burst_flag.nd_ea)
      sprintf(bufLine, "ND_EA: ON");
    else
      sprintf(bufLine, "ND_EA: OFF");

    switch (_burst_flag.tempc) {
      case OUT32:
        strcat(bufLine, "\tTempC: 32");
        break;
      case OUT16:
        strcat(bufLine, "\tTempC: 16");
        break;
      default:
        strcat(bufLine, "\tTempC: OFF");
        break;
    }

    switch (_burst_flag.gyro) {
      case OUT32:
        strcat(bufLine, "\tGyro: 32");
        break;
      case OUT16:
        strcat(bufLine, "\tGyro: 16");
        break;
      default:
        strcat(bufLine, "\tGyro: OFF");
        break;
    }

    switch (_burst_flag.accl) {
      case OUT32:
        strcat(bufLine, "\tAccl: 32");
        break;
      case OUT16:
        strcat(bufLine, "\tAccl: 16");
        break;
      default:
        strcat(bufLine, "\tAccl: OFF");
        break;
    }
    _consolePort.println(bufLine);

    if (_dev_prop.feature_flags & HAS_DLT_OUTPUT) {
      switch (_burst_flag.dlta) {
        case OUT32:
          sprintf(bufLine, "DltA: 32");
          break;
        case OUT16:
          sprintf(bufLine, "DltA: 16");
          break;
        default:
          sprintf(bufLine, "DltA: OFF");
          break;
      }

      switch (_burst_flag.dltv) {
        case OUT32:
          strcat(bufLine, "\tDltV: 32");
          break;
        case OUT16:
          strcat(bufLine, "\tDltV: 16");
          break;
        default:
          strcat(bufLine, "\tDltV: OFF");
          break;
      }
      _consolePort.println(bufLine);
    };

    if (_dev_prop.feature_flags & HAS_ATTITUDE_OUTPUT) {
      switch (_burst_flag.qtn) {
        case OUT32:
          sprintf(bufLine, "Qtn: 32");
          break;
        case OUT16:
          sprintf(bufLine, "Qtn: 16");
          break;
        default:
          sprintf(bufLine, "Qtn: OFF");
          break;
      }

      switch (_burst_flag.atti) {
        case OUT32:
          strcat(bufLine, "\tAtti: 32");
          break;
        case OUT16:
          strcat(bufLine, "\tAtti: 16");
          break;
        default:
          strcat(bufLine, "\tAtti: OFF");
          break;
      }
      _consolePort.println(bufLine);
    };

    if (_burst_flag.gpio)
      _consolePort.print("GPIO: ON\t");
    else
      _consolePort.print("GPIO: OFF\t");

    if (_burst_flag.count)
      _consolePort.print("Count: ON\t");
    else
      _consolePort.print("Count: OFF\t");

    if (_burst_flag.chksm)
      _consolePort.print("Chksm: ON");
    else
      _consolePort.print("Chksm: OFF");

    _consolePort.println(
      "\n****************************************************************"
      "*");
  } else {
    _consolePort.println("Error entering CONFIG mode.");
  }
}

/**************************************************************************/
/*!
    @brief      Sends column headers to console
*/
/**************************************************************************/
void EPSON_IMU_UART::sensorHeaderPrint(void) {
  _consolePort.print("\nSample#");
  if (_burst_flag.gyro) _consolePort.print("\tGx\tGy\tGz");
  if (_burst_flag.accl) _consolePort.print("\tAx\tAy\tAz");
  if (_burst_flag.dlta) _consolePort.print("\tDAx\tDAy\tDAz");
  if (_burst_flag.dltv) _consolePort.print("\tDVx\tDVy\tDVz");
  if (_burst_flag.qtn) _consolePort.print("\tQTN0\tQTN1\tQTN2\tQTN3");
  if (_burst_flag.atti) _consolePort.print("\tANG1\tANG2\tANG3");
  if (_burst_flag.tempc) _consolePort.print("\tTempC");
  if (_burst_flag.count) _consolePort.print("\tCounter");
  if (_burst_flag.nd_ea) _consolePort.print("\tFlags");
  if (_burst_flag.gpio) _consolePort.print("\tGpio");
  if (_burst_flag.chksm) _consolePort.print("\tChksm");
  _consolePort.print("\n");
}

/**************************************************************************/
/*!
    @brief  Sends a row of scale sample data to console

    @param [in]  scaledField
                 Pointer to struct ImuScaledData
    @param [in]  sampleIndex
                 Current count of row
*/
/**************************************************************************/
void EPSON_IMU_UART::sensorDataPrint(const struct ImuScaledData* scaledField,
                                     uint32_t sampleIndex) {
  char bufLine[200];
  char bufString[64];
  sprintf(bufLine, "%i", (int)sampleIndex);
  if (_burst_flag.gyro) {
    sprintf(bufString, ", %0.6f, %0.6f, %0.6f", scaledField->gyroXYZ[0],
            scaledField->gyroXYZ[1], scaledField->gyroXYZ[2]);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.accl) {
    sprintf(bufString, ", %0.6f, %0.6f, %0.6f", scaledField->accelXYZ[0],
            scaledField->accelXYZ[1], scaledField->accelXYZ[2]);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.dlta) {
    sprintf(bufString, ", %0.6f, %0.6f, %0.6f", scaledField->dltaXYZ[0],
            scaledField->dltaXYZ[1], scaledField->dltaXYZ[2]);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.dltv) {
    sprintf(bufString, ", %0.6f, %0.6f, %0.6f", scaledField->dltvXYZ[0],
            scaledField->dltvXYZ[1], scaledField->dltvXYZ[2]);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.atti) {
    sprintf(bufString, ", %0.6f, %0.6f, %0.6f", scaledField->attiXYZ[0],
            scaledField->attiXYZ[1], scaledField->attiXYZ[2]);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.qtn) {
    sprintf(bufString, ", %0.6f, %0.6f, %0.6f, %0.6f", scaledField->qtn4[0],
            scaledField->qtn4[1], scaledField->qtn4[2], scaledField->qtn4[3]);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.tempc) {
    sprintf(bufString, ", %0.3f", scaledField->tempC);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.count) {
    sprintf(bufString, ", %05d", scaledField->counter);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.nd_ea) {
    sprintf(bufString, ", %04x", scaledField->nd_flags);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.gpio) {
    sprintf(bufString, ", %2x", scaledField->gpio);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.chksm) {
    sprintf(bufString, ", %5d", scaledField->chksm);
    strcat(bufLine, bufString);
  }
  _consolePort.println(bufLine);
}

/**************************************************************************/
/*!
    @brief  Reads sensor burst, parse and convert to unscaled data, and
                apply scale factor if necessary and store to ImuScaledData

    @param [out] scaledField
                 Pointer to struct ImuScaledData

    @returns  true if successful, false if burst read contained errors
*/
/**************************************************************************/
boolean EPSON_IMU_UART::sensorGetSensorBurst(
  struct ImuScaledData& scaledField) {
  boolean burst_ok = _sensorReadBurst16(_read_burst, sizeof(_read_burst));
  if (!burst_ok) {
    return false;
  }
  _sensorBuf2Field(&_unscaled, _read_burst, sizeof(_read_burst));
  _sensorField2Scaled(&scaledField, &_unscaled);
  return true;
}

/**************************************************************************/
/*!
    @brief  Programs the device registers from provided init_options struct

    @param [in]  struct ImuInitOptions
                 Initialization settings

    @param [in]  boolean verbose
                     Send register accesses to console for debugging

    @returns    True on success, False on incorrect status bit
*/
/**************************************************************************/
boolean EPSON_IMU_UART::sensorInitOptions(struct ImuInitOptions* options,
                                          boolean verbose) {
  // Disable attitude/quaternion output if not supported
  if (!(_dev_prop.feature_flags & HAS_ATTITUDE_OUTPUT)) {
    options->atti_out = 0;
    options->qtn_out = 0;
    _consolePort.println(
      "Warning: Device does not support attitude or quaternion output");
  }
  // Disable delta output if not supported
  if (!(_dev_prop.feature_flags & HAS_DLT_OUTPUT)) {
    options->gyro_delta_out = 0;
    options->accel_delta_out = 0;
    _consolePort.println("Warning: Device does not support delta output");
  }
  // Disable attitude output if delta output enabled
  if (options->gyro_delta_out | options->accel_delta_out) {
    options->atti_out = 0;
    options->qtn_out = 0;
    _consolePort.println(
      "Warning: Attitude or quaternion output not supported when delta output "
      "enabled");
  }

  // SIG_CTRL
  // ND flags for gyro_delta_out X,Y,Z are enabled if gyro_delta_out is enabled
  // ND flags for accel_delta_out X,Y,Z are enabled if accel_delta_out is
  // enabled
  int sig_ctrl_lo = 0;
  if (_dev_prop.feature_flags & HAS_DLT_OUTPUT) {
    sig_ctrl_lo = (options->accel_delta_out & 0x01) << 2 |
                  (options->accel_delta_out & 0x01) << 3 |
                  (options->accel_delta_out & 0x01) << 4 |
                  (options->gyro_delta_out & 0x01) << 5 |
                  (options->gyro_delta_out & 0x01) << 6 |
                  (options->gyro_delta_out & 0x01) << 7;
  }

  // ND flags for gyro_out X,Y,Z are enabled if gyro_out is enabled
  // ND flags for accel_out X,Y,Z are enabled if accel_out is enabled
  // ND flag for temp_out is enabled if temp_out is enabled
  int sig_ctrl_hi =
    (options->accel_out & 0x01) << 1 | (options->accel_out & 0x01) << 2 |
    (options->accel_out & 0x01) << 3 | (options->gyro_out & 0x01) << 4 |
    (options->gyro_out & 0x01) << 5 | (options->gyro_out & 0x01) << 6 |
    (options->temp_out & 0x01) << 7;

  // MSC_CTRL
  // Configure DRDY function (if needed) & EXT pin function on GPIO2 (if needed)
  // External Counter Reset is typically used when GPIO2 is connected to a
  // PPS-like signal
  int msc_ctrl_lo = 0;
  if (_dev_prop.model == G570PR20) {
    msc_ctrl_lo =
      ((options->drdy_pol & 0x01) << 1) | ((options->drdy_on & 0x01) << 2);
  } else {
    msc_ctrl_lo =
      (options->drdy_pol & 0x01) << 1 | (options->drdy_on & 0x01) << 2 |
      (options->ext_pol & 0x01) << 5 | (options->ext_sel & 0x03) << 6;
  }

  // SMPL_CTRL
  // Configures the Data Output Rate
  // Refer to Datasheet for valid Data Output Rate & Filter Setting combinations
  int smpl_ctrl_hi = (options->dout_rate & 0x0F);

  // FILTER_CTRL
  // Configures the FIR filter
  // Refer to Datasheet for valid Data Output Rate & Filter Setting combinations
  int filter_ctrl_lo = (options->filter_sel & 0x1F);

  // UART_CTRL
  // Enable or disable UART_AUTO
  // NOTE: UART_AUTO should not be enabled when using SPI interface
  int uart_ctrl_lo = (options->uart_auto & 0x01);

  // BURST_CTRL1
  // These enable or disable certain data fields in the burst read packet
  // For G330 or G366 specifically, attitude output can be enabled or disabled.
  int burst_ctrl1_lo = (options->checksum_out & 0x01) |
                       (options->count_out & 0x01) << 1 |
                       (options->gpio_out & 0x01) << 2;

  int burst_ctrl1_hi = 0;
  if (_dev_prop.feature_flags & HAS_ATTITUDE_OUTPUT) {
    burst_ctrl1_hi |=
      ((options->atti_out & 0x1) | (options->qtn_out & 0x01) << 1);
  }
  if (_dev_prop.feature_flags & HAS_DLT_OUTPUT) {
    burst_ctrl1_hi |= ((options->accel_delta_out & 0x01) << 2 |
                       (options->gyro_delta_out & 0x01) << 3);
  }
  burst_ctrl1_hi |=
    ((options->accel_out & 0x01) << 4 | (options->gyro_out & 0x01) << 5 |
     (options->temp_out & 0x01) << 6 | (options->flag_out & 0x01) << 7);

  // BURST_CTRL2
  // If certain data fields are enabled, these bits determine if the
  // data fields are 16 or 32 bit
  int burst_ctrl2_hi = 0;
  if (_dev_prop.feature_flags & HAS_ATTITUDE_OUTPUT) {
    burst_ctrl2_hi |=
      ((options->atti_bit & 0x01) | (options->qtn_bit & 0x01) << 1);
  }
  if (_dev_prop.feature_flags & HAS_DLT_OUTPUT) {
    burst_ctrl2_hi |= ((options->accel_delta_bit & 0x01) << 2 |
                       (options->gyro_delta_bit & 0x01) << 3);
  }
  burst_ctrl2_hi |=
    ((options->accel_bit & 0x01) << 4 | (options->gyro_bit & 0x01) << 5 |
     (options->temp_bit & 0x01) << 6);

  // POL_CTRL
  // If these bits are set, then the axis values are reverse polarity
  int pol_ctrl_lo =
    (options->invert_zaccel & 0x01) << 1 |
    (options->invert_yaccel & 0x01) << 2 |
    (options->invert_xaccel & 0x01) << 3 | (options->invert_zgyro & 0x01) << 4 |
    (options->invert_ygyro & 0x01) << 5 | (options->invert_xgyro & 0x01) << 6;

  // DLT_CTRL
  // Enable or disable Delta Angle/Velocity overflow flag in DIAG_STAT
  // Set A_RANGE_CTRL bit if device supports it
  // Set the Delta Angle/Velocity Scale Factor
  int dlt_ctrl_hi = 0;
  if (_dev_prop.feature_flags & HAS_ARANGE) {
    dlt_ctrl_hi = (options->a_range_ctrl & 0x01);
  }

  int dlt_ctrl_lo = 0;
  if (_dev_prop.feature_flags & HAS_DLT_OUTPUT) {
    dlt_ctrl_lo = (options->dlta_range_ctrl & 0x0F) << 4 |
                  (options->dltv_range_ctrl & 0x0F);
  }

  // ATTI_CTRL
  // Attitude Output & Delta Angle/Velocity functions are mutually exclusive.
  // User should only enable one or the other, but not both.
  // Attitude Mode = 0=Inclination 1=Euler
  int atti_ctrl_hi = 0;
  int atti_ctrl_lo = 0;
  if (_dev_prop.feature_flags & HAS_ATTITUDE_OUTPUT) {
    atti_ctrl_hi |= (((options->atti_out | options->qtn_out) & 0x01) << 2 |
                     (options->atti_mode & 0x01) << 3);

    // Refer to datasheet to determine the different Euler Angle configurations
    if (!options->qtn_out) {
      atti_ctrl_lo = (options->atti_conv & 0x1f);
    }
  }
  if ((_dev_prop.feature_flags & (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG)) ==
      (HAS_DLT_OUTPUT | HAS_ATTI_ON_REG)) {
    atti_ctrl_hi |=
      (((options->gyro_delta_out & 0x01) | (options->accel_delta_out & 0x01))
       << 1);
  }

  // GLOB_CMD2
  // Setting Attitude Motion Profile
  int glob_cmd2_lo = 0;
  if (_dev_prop.feature_flags & HAS_ATTITUDE_OUTPUT) {
    glob_cmd2_lo = (options->atti_profile & 0x03) << 4;
  }
  if (!(_dev_prop.model == G570PR20)) {
    regWrite8(CMD_WINDOW1, ADDR_SIG_CTRL_HI, sig_ctrl_hi, verbose);
    regWrite8(CMD_WINDOW1, ADDR_SIG_CTRL_LO, sig_ctrl_lo, verbose);
  }
  regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_LO, msc_ctrl_lo, verbose);
  regWrite8(CMD_WINDOW1, ADDR_SMPL_CTRL_HI, smpl_ctrl_hi, verbose);
  regWrite8(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, filter_ctrl_lo, verbose);
  delay(_dev_prop.delay_filter_ms);

  // Check that the FILTER_BUSY bit returns 0
  uint16_t rxData;
  int retryCount = FILTERBUSY_RETRIES;
  do {
    rxData = regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, verbose);
    retryCount--;
  } while ((rxData & BIT_5) && (retryCount > 0));

  if (retryCount == 0) {
    _consolePort.println("...Error: Filter busy bit did not return to 0b.");
    return false;
  }

  regWrite8(CMD_WINDOW1, ADDR_UART_CTRL_LO, uart_ctrl_lo, verbose);
  _uart_auto = boolean(uart_ctrl_lo);

  regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL1_LO, burst_ctrl1_lo, verbose);
  regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL1_HI, burst_ctrl1_hi, verbose);
  regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL2_HI, burst_ctrl2_hi, verbose);
  regWrite8(CMD_WINDOW1, ADDR_POL_CTRL_LO, pol_ctrl_lo, verbose);

  if (_dev_prop.feature_flags & HAS_ARANGE) {
    regWrite8(CMD_WINDOW1, ADDR_DLT_CTRL_HI, dlt_ctrl_hi, verbose);
  }
  if (_dev_prop.feature_flags & HAS_DLT_OUTPUT) {
    regWrite8(CMD_WINDOW1, ADDR_DLT_CTRL_LO, dlt_ctrl_lo, verbose);
  }
  if (_dev_prop.feature_flags & HAS_ATTI_ON_REG) {
    regWrite8(CMD_WINDOW1, ADDR_ATTI_CTRL_HI, atti_ctrl_hi, verbose);
  }
  if (_dev_prop.feature_flags & HAS_ATTITUDE_OUTPUT) {
    regWrite8(CMD_WINDOW1, ADDR_ATTI_CTRL_LO, atti_ctrl_lo, verbose);

    regWrite8(CMD_WINDOW1, ADDR_GLOB_CMD2_LO, glob_cmd2_lo, verbose);
    delay(_dev_prop.delay_atti_profile_ms);

    // Check that the ATTI_MOTION_PROFILE_STAT bit returns 0
    retryCount = ATTIMOTIONBUSY_RETRIES;
    do {
      rxData = regRead16(CMD_WINDOW1, ADDR_GLOB_CMD2_LO, verbose);
      retryCount--;
    } while ((rxData & BIT_6) && (retryCount > 0));

    if (retryCount == 0) {
      _consolePort.println(
        "...Error: ATTI_MOTION_PROFILE_STAT bit did not return to 0b.");
      return false;
    }
  }
  // Update _read_burst8_len after BURST_CTRL registers written
  _read_burst8_len = _sensorDecodeBurstCtrl(verbose);
  return true;
}

/**************************************************************************/
/*!
    @brief      Enters Config Mode to read back all register values from the
                Sensor.
*/
/**************************************************************************/
void EPSON_IMU_UART::sensorRegisterDump(void) {
  if (sensorStop() == true) {
    // print the current values in all the Sensor registers
    _consolePort.println("\r\nRegister Dump:");
    regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO, true);
    regRead16(CMD_WINDOW0, ADDR_DIAG_STAT, true);
    regRead16(CMD_WINDOW0, ADDR_FLAG, true);
    // For G570PR20 ADDR_GPIO is same as address for HARD_ERR
    regRead16(CMD_WINDOW0, ADDR_GPIO, true);
    if (_dev_prop.model != G570PR20) {
      regRead16(CMD_WINDOW0, ADDR_COUNT, true);
    }
    if (_dev_prop.feature_flags & HAS_RANGE_OVER) {
      regRead16(CMD_WINDOW0, ADDR_RANGE_OVER, true);
    }
    if (_dev_prop.model != G570PR20) {
      _consolePort.println();
      regRead16(CMD_WINDOW0, ADDR_TEMP_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_TEMP_LOW, true);
      regRead16(CMD_WINDOW0, ADDR_XGYRO_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_XGYRO_LOW, true);
      regRead16(CMD_WINDOW0, ADDR_YGYRO_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_YGYRO_LOW, true);
      regRead16(CMD_WINDOW0, ADDR_ZGYRO_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_ZGYRO_LOW, true);
      _consolePort.println();
      regRead16(CMD_WINDOW0, ADDR_XACCL_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_XACCL_LOW, true);
      regRead16(CMD_WINDOW0, ADDR_YACCL_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_YACCL_LOW, true);
      regRead16(CMD_WINDOW0, ADDR_ZACCL_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_ZACCL_LOW, true);
      _consolePort.println();
    }
    if (_dev_prop.feature_flags & HAS_RTDIAG) {
      regRead16(CMD_WINDOW0, ADDR_RT_DIAG, true);
      _consolePort.println();
    }
    regRead16(CMD_WINDOW0, ADDR_ID, true);
    _consolePort.println();
    if (_dev_prop.feature_flags & HAS_ATTITUDE_OUTPUT) {
      regRead16(CMD_WINDOW0, ADDR_QTN0_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_QTN0_LOW, true);
      regRead16(CMD_WINDOW0, ADDR_QTN1_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_QTN1_LOW, true);
      regRead16(CMD_WINDOW0, ADDR_QTN2_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_QTN2_LOW, true);
      regRead16(CMD_WINDOW0, ADDR_QTN3_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_QTN3_LOW, true);
    }
    if (_dev_prop.feature_flags & HAS_DLT_OUTPUT) {
      regRead16(CMD_WINDOW0, ADDR_XDLTA_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_XDLTA_LOW, true);
      regRead16(CMD_WINDOW0, ADDR_YDLTA_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_YDLTA_LOW, true);
      regRead16(CMD_WINDOW0, ADDR_ZDLTA_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_ZDLTA_LOW, true);
      _consolePort.println();
      regRead16(CMD_WINDOW0, ADDR_XDLTV_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_XDLTV_LOW, true);
      regRead16(CMD_WINDOW0, ADDR_YDLTV_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_YDLTV_LOW, true);
      regRead16(CMD_WINDOW0, ADDR_ZDLTV_HIGH, true);
      regRead16(CMD_WINDOW0, ADDR_ZDLTV_LOW, true);
      _consolePort.println();
    }
    if (_dev_prop.model != G570PR20) {
      regRead16(CMD_WINDOW1, ADDR_SIG_CTRL_LO, true);
    }
    regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_UART_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO, true);
    regRead16(CMD_WINDOW1, ADDR_BURST_CTRL1_LO, true);
    regRead16(CMD_WINDOW1, ADDR_BURST_CTRL2_LO, true);
    regRead16(CMD_WINDOW1, ADDR_POL_CTRL_LO, true);
    if ((_dev_prop.feature_flags & HAS_DLT_OUTPUT) |
        (_dev_prop.feature_flags & HAS_ARANGE)) {
      regRead16(CMD_WINDOW1, ADDR_DLT_CTRL_LO, true);
    }
    if (_dev_prop.feature_flags & HAS_ATTI_ON_REG) {
      regRead16(CMD_WINDOW1, ADDR_ATTI_CTRL_LO, true);
    }
    if (_dev_prop.feature_flags & HAS_ATTITUDE_OUTPUT) {
      regRead16(CMD_WINDOW1, ADDR_GLOB_CMD2_LO, true);
    }
    _consolePort.println();
    regRead16(CMD_WINDOW1, ADDR_PROD_ID1, true);
    regRead16(CMD_WINDOW1, ADDR_PROD_ID2, true);
    regRead16(CMD_WINDOW1, ADDR_PROD_ID3, true);
    regRead16(CMD_WINDOW1, ADDR_PROD_ID4, true);
    regRead16(CMD_WINDOW1, ADDR_VERSION, true);
    regRead16(CMD_WINDOW1, ADDR_SERIAL_NUM1, true);
    regRead16(CMD_WINDOW1, ADDR_SERIAL_NUM2, true);
    regRead16(CMD_WINDOW1, ADDR_SERIAL_NUM3, true);
    regRead16(CMD_WINDOW1, ADDR_SERIAL_NUM4, true);
  } else {
    _consolePort.println("Warning: Not entering Config Mode");
  }
}

/**************************************************************************/
/*!
    @brief  Selects the Epson sensor model by storing the model struct
                into private struct _dev_prop

    @param [in]  model
                 index for ImuProperties array (all supported
                 models)

    @returns    True on success, False if model is out of bounds
*/
/**************************************************************************/
boolean EPSON_IMU_UART::sensorModelSelect(int model) {
  if ((model <= INVALID) || (model >= MAX_MODELS)) {
    return false;
  }
  _dev_prop = imu_model[model];
  return true;
}

/**************************************************************************/
/*!
    @brief  Detects the Epson sensor model by searching for the Product ID
                and returning the index in the array of model structs

    @returns    int
                    index of product ID found within the model array
                                of device properties
*/
/**************************************************************************/
int EPSON_IMU_UART::sensorModelDetect(void) {
  char model_read[9];
  sensorGetProdID(model_read);
  _consolePort.print("Detected PROD_ID:");
  _consolePort.println(model_read);
  int index;
  for (index = INVALID; index < MAX_MODELS; index++) {
    if (strcmp(imu_model[index].product_id, model_read) == 0) break;
  }

  if ((index == INVALID) || (index == MAX_MODELS)) {
    _consolePort.println("ERROR: Unsupported product model.");
  }
  return index;
}

/**************************************************************************/
/*!
    @brief  Goes to SAMPLING Mode

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean EPSON_IMU_UART::sensorStart(boolean verbose) {
  regWrite8(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_BEGIN_SAMPLING, verbose);
  // NOTE: Minimize delays after entering SAMPLING mode when UART_AUTO=enabled
  // Delays before reading IMU data streaming to the serial port
  // can cause dropped bytes due to serial port buffer overflow
  if (_uart_auto == false) {
    // If UART_AUTO is disabled
    // Check that MODE_STAT bit returns 0, cycles out after 10,000 tries
    for (int32_t i = 0; i < 10000; i++) {
      uint16_t valRead =
        regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO, verbose) & VAL_CONFIG_MASK;
      if (valRead == VAL_SAMPLING_MODE) {
        return true;
      }
    }
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Places the sensor into CONFIG Mode

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean EPSON_IMU_UART::sensorStop(boolean verbose) {
  regWrite8(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_END_SAMPLING, verbose);
  delay((unsigned)1000);  // Wait atleast 1 second for any pending burst packets
                          // to finish

  // Clear any bytes in the UART Rx buffer
  clearRxBuffer();

  if (_uart_auto == false) {
    // If UART_AUTO is disabled
    // Check that MODE_STAT bit returns 1, cycles out after 10,000 tries
    for (int32_t i = 0; i < 10000; i++) {
      uint16_t valRead =
        regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO, verbose) & VAL_CONFIG_MASK;
      if (valRead == VAL_CONFIG_MODE) {
        return true;
      }
    }
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Enters Config Mode to initiate a selftest and returns the DIAG_STAT
            register value

    @param [in]  boolean verbose
                     Send register accesses to console for debugging

    @returns    DIAG_STAT register value, 0 on success, nonzero for any errors
                and 0xFFFF if SELF_TEST bit is stuck 1b.
*/
/**************************************************************************/
uint16_t EPSON_IMU_UART::sensorSelfTest(boolean verbose) {
  uint16_t valRead;

  if (!sensorStop(verbose)) {
    _consolePort.println("Warning: Not entering Config Mode");
  }

  // Send the self test command
  _consolePort.print("Initiate Selftest...");
  regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_SELFTEST, verbose);

  // Wait for self test to process
  delay(_dev_prop.delay_selftest_ms);

  // Check that SELF_TEST bit returns 0
  valRead =
    regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, verbose) & VAL_SELF_TEST_BIT;
  if (valRead != 0) {
    _consolePort.println("Warning: SELF_TEST bit not returning to 0");
    return 0xFFFF;
  }
  _consolePort.println("Done.");
  // Read the results in DIAG_STAT
  valRead = regRead16(CMD_WINDOW0, ADDR_DIAG_STAT, verbose);
  if ((valRead & VAL_DIAG_STAT_MASK) != 0) {
    _consolePort.print("Warning: DIAG_STAT returned some errors - ");
    _consolePort.println(valRead, HEX);
  }
  return valRead;
}

/**************************************************************************/
/*!
    @brief  Issues a software reset and waits for the required delay time.
            Then checks NOT_READY bit. This function can be called at anytime.

    @param [in]  boolean verbose
                     Send register accesses to console for debugging

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean EPSON_IMU_UART::sensorReset(boolean verbose) {
  _consolePort.println("Asserting Software Reset Epson SU");
  // Set the RESET bit
  regWrite8(CMD_WINDOW1, ADDR_GLOB_CMD_LO, CMD_SOFTRESET, verbose);

  // Wait for the sensor software reset to process
  delay(_dev_prop.delay_reset_ms);

  // Check NOT_READY bit = 0
  uint16_t valRead =
    regRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO, verbose) & VAL_NOT_READY;
  if (valRead != 0) {
    _consolePort.println("Error: NOT_READY is HIGH");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Enters Config Mode to initiate a Flash test and returns the
            DIAG_STAT bit 2 result

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

    @returns    true on no errors, false for any errors
*/
/**************************************************************************/
boolean EPSON_IMU_UART::sensorFlashTest(boolean verbose) {
  uint16_t valRead;

  if (!sensorStop(verbose)) {
    _consolePort.println("Warning: Not entering Config Mode");
  }

  // Send the flash test command
  _consolePort.print("Initiate FLASH_TEST...");
  regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_FLASHTEST, verbose);

  // Wait for flash test to process
  delay(_dev_prop.delay_flashtest_ms);

  // Check that Flash_TEST bit returns 0
  valRead =
    regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, verbose) & VAL_FLASH_STATUS_BIT;
  if (valRead != 0) {
    _consolePort.println("Warning: FLASH_TEST bit not returning to 0");
    return false;
  }
  _consolePort.println("Done.");
  // Read the results in DIAG_STAT
  valRead =
    regRead16(CMD_WINDOW0, ADDR_DIAG_STAT, verbose) & VAL_DIAG_FLASH_ERR;

  if (valRead != 0) {
    _consolePort.println("Warning: FLASH_ERR bit detected");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Assumes Config Mode and Reads the Product ID from the Sensor.

    @param [out]  prodID
                  Pointer to string of 8 ASCII bytes of PROD_ID
                  i.e.
                  Expected ASCII values: "G364PDC0"
                  Return Array Values: 0x47, 0x33, 0x36, 0x34, 0x50, 0x44, 0x43,
                                       0x30, 0x00

    @param [in]  length (minimum at least 9 bytes)
                 Length of the string
*/
/**************************************************************************/
void EPSON_IMU_UART::sensorGetProdID(char* prodID, const size_t length) {
  if (length < 9) {
    _consolePort.println(
      "Error: char array size must be atleast 9 bytes. Bypassing...");
    return;
  }

  uint8_t i;

  // read model name from registers, stored as ascii values
  for (i = 0; i < 8; i = i + 2) {
    uint16_t retVal = regRead16(CMD_WINDOW1, ADDR_PROD_ID1 + i);
    prodID[i] = (char)(retVal & 0xFF);
    prodID[i + 1] = (char)(retVal >> 8);
  }
  prodID[i] = 0x00;  // add NULL terminator to make it a string
}

/**************************************************************************/
/*!
    @brief  Assumes Config Mode and Reads the Firmware Version from the Sensor.

    @returns  16-bit Firmware Number
*/
/**************************************************************************/
uint16_t EPSON_IMU_UART::sensorGetVersion(void) {
  return regRead16(CMD_WINDOW1, ADDR_VERSION);
}

/**************************************************************************/
/*!
    @brief  Assumes Config Mode and Reads the Serial Number from the Sensor
            and terminate with NULL to create a string.

    @param [out]  serialNumber
                  Pointer to string of 8 ASCII bytes of Serial Number
                  Each byte is an ASCII number.

        @param [in]  length (minimum at least 9 bytes)
                 Length of the string
*/
/**************************************************************************/
void EPSON_IMU_UART::sensorGetSerialNumber(char* serialNumber,
                                           const size_t length) {
  if (length < 9) {
    _consolePort.println(
      "Error: char array size must be atleast 9 bytes. Bypassing...");
    return;
  }

  uint8_t i;

  // read serial number from registers, stored as ascii values
  for (i = 0; i < 8; i = i + 2) {
    uint16_t retVal = regRead16(CMD_WINDOW1, ADDR_SERIAL_NUM1 + i);
    serialNumber[i] = (char)(retVal & 0xFF);
    serialNumber[i + 1] = (char)(retVal >> 8);
  }
  serialNumber[i] = 0x00;  // add NULL terminator to make it a string
}

/*========================================================================*/
/*                           PRIVATE FUNCTIONS                            */
/*========================================================================*/
/**************************************************************************/
/*!
    @brief  Decodes the BURST_CTRL1 & BURST_CTRL2 register and stores
    status in _burst_flag struct and returns output burst length in bytes

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

    @returns    sensor burst length in bytes(excluding header & delimiter
                byte)
*/
/**************************************************************************/
int8_t EPSON_IMU_UART::_sensorDecodeBurstCtrl(boolean verbose) {
  int8_t burst_len = 0;

  uint16_t burst_ctrl1 =
    regRead16(CMD_WINDOW1, ADDR_BURST_CTRL1_LO, verbose) & 0xFF07;
  uint16_t burst_ctrl2 =
    regRead16(CMD_WINDOW1, ADDR_BURST_CTRL2_LO, verbose) & 0x7F00;

  // burst_ctrl1 check
  _burst_flag.nd_ea = (burst_ctrl1 & BIT_15) ? OUT16 : OFF;
  _burst_flag.tempc = (burst_ctrl1 & BIT_14) ? OUT16 : OFF;
  _burst_flag.gyro = (burst_ctrl1 & BIT_13) ? OUT16 : OFF;
  _burst_flag.accl = (burst_ctrl1 & BIT_12) ? OUT16 : OFF;
  _burst_flag.dlta = (burst_ctrl1 & BIT_11) ? OUT16 : OFF;
  _burst_flag.dltv = (burst_ctrl1 & BIT_10) ? OUT16 : OFF;
  _burst_flag.qtn = (burst_ctrl1 & BIT_9) ? OUT16 : OFF;
  _burst_flag.atti = (burst_ctrl1 & BIT_8) ? OUT16 : OFF;
  _burst_flag.gpio = (burst_ctrl1 & BIT_2) ? OUT16 : OFF;
  _burst_flag.count = (burst_ctrl1 & BIT_1) ? OUT16 : OFF;
  _burst_flag.chksm = (burst_ctrl1 & BIT_0) ? OUT16 : OFF;

  // burst_ctrl2 check for 32-bit
  if ((_burst_flag.tempc) && (burst_ctrl2 & BIT_14)) _burst_flag.tempc++;
  if ((_burst_flag.gyro) && (burst_ctrl2 & BIT_13)) _burst_flag.gyro++;
  if ((_burst_flag.accl) && (burst_ctrl2 & BIT_12)) _burst_flag.accl++;
  if ((_burst_flag.dlta) && (burst_ctrl2 & BIT_11)) _burst_flag.dlta++;
  if ((_burst_flag.dltv) && (burst_ctrl2 & BIT_10)) _burst_flag.dltv++;
  if ((_burst_flag.qtn) && (burst_ctrl2 & BIT_9)) _burst_flag.qtn++;
  if ((_burst_flag.atti) && (burst_ctrl2 & BIT_8)) _burst_flag.atti++;

  // Calc burst_len
  if (_burst_flag.nd_ea) burst_len += 2;
  if (_burst_flag.tempc) burst_len += _burst_flag.tempc * 2;
  if (_burst_flag.gyro) burst_len += _burst_flag.gyro * 6;
  if (_burst_flag.accl) burst_len += _burst_flag.accl * 6;
  if (_burst_flag.dlta) burst_len += _burst_flag.dlta * 6;
  if (_burst_flag.dltv) burst_len += _burst_flag.dltv * 6;
  if (_burst_flag.qtn) burst_len += _burst_flag.qtn * 8;
  if (_burst_flag.atti) burst_len += _burst_flag.atti * 6;
  if (_burst_flag.gpio) burst_len += 2;
  if (_burst_flag.count) burst_len += 2;
  if (_burst_flag.chksm) burst_len += 2;

  return burst_len;
}

/**************************************************************************/
/*!
    @brief  Assumes device is in Sampling mode
            If DRDY is used, then checks DRDY is active
            Issues a burst read command and generates N x 16-bit read cycles
            to read back one sensor sample set

    @param [out] pointer to 16-bit array (stores sensor values)

    @param [int] length in bytes for 16-bit array

    @returns    true if successful or false if errors detected

*/
/**************************************************************************/
boolean EPSON_IMU_UART::_sensorReadBurst16(uint16_t* arr, const size_t length) {
  if (_read_burst8_len < 1) {
    _consolePort.println("Error: Bypassing because burst length is invalid");
    _consolePort.println(
      "Has sensor been initialized with sensorInitOptions()?");
    return false;
  }

  if ((uint8_t)length < _read_burst8_len) {
    _consolePort.print("Error: Bypassing because the array is smaller");
    _consolePort.print(" than the read burst length of: ");
    _consolePort.print(_read_burst8_len, DEC);
    return false;
  }

  // Read burst sensor data
  boolean retval = readN(arr, CMD_BURST, _read_burst8_len, READBURST_RETRIES);

  return retval;
}

/**************************************************************************/
/*!
    @brief  Convert burst read sensor buffer data to fields

    @param [out] unscaledField
                 Pointer to struct ImuUnscaledData

    @param [in]  readBuf
                 Pointer to 16-bit Array

    @param [in]  length
                 Size of 16-bit Array in bytes
*/
/**************************************************************************/
void EPSON_IMU_UART::_sensorBuf2Field(struct ImuUnscaledData* unscaledField,
                                      const uint16_t* readBuf,
                                      const size_t length) {
  if ((uint8_t)length < _read_burst8_len) {
    _consolePort.print("Error: Bypassing because the array is smaller");
    _consolePort.print(" than the read burst length of: ");
    _consolePort.print(_read_burst8_len, DEC);
    return;
  }

  int i = 0;
  if (_burst_flag.nd_ea) {
    unscaledField->nd_flags = readBuf[i];
    i = i + 1;
  }

  if (_burst_flag.tempc == OUT32) {
    unscaledField->tempC =
      (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    i = i + 2;
  } else if (_burst_flag.tempc == OUT16) {
    unscaledField->tempC = (uint32_t)readBuf[i] << 16;
    i = i + 1;
  }

  if (_burst_flag.gyro == OUT32) {
    unscaledField->gyroXYZ[0] =
      (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    unscaledField->gyroXYZ[1] =
      (uint32_t)readBuf[i + 2] << 16 | (uint32_t)readBuf[i + 3];
    unscaledField->gyroXYZ[2] =
      (uint32_t)readBuf[i + 4] << 16 | (uint32_t)readBuf[i + 5];
    i = i + 6;
  } else if (_burst_flag.gyro == OUT16) {
    unscaledField->gyroXYZ[0] = (uint32_t)readBuf[i] << 16;
    unscaledField->gyroXYZ[1] = (uint32_t)readBuf[i + 1] << 16;
    unscaledField->gyroXYZ[2] = (uint32_t)readBuf[i + 2] << 16;
    i = i + 3;
  }

  if (_burst_flag.accl == OUT32) {
    unscaledField->accelXYZ[0] =
      (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    unscaledField->accelXYZ[1] =
      (uint32_t)readBuf[i + 2] << 16 | (uint32_t)readBuf[i + 3];
    unscaledField->accelXYZ[2] =
      (uint32_t)readBuf[i + 4] << 16 | (uint32_t)readBuf[i + 5];
    i = i + 6;
  } else if (_burst_flag.accl == OUT16) {
    unscaledField->accelXYZ[0] = (uint32_t)readBuf[i] << 16;
    unscaledField->accelXYZ[1] = (uint32_t)readBuf[i + 1] << 16;
    unscaledField->accelXYZ[2] = (uint32_t)readBuf[i + 2] << 16;
    i = i + 3;
  }

  if (_burst_flag.dlta == OUT32) {
    unscaledField->dltaXYZ[0] =
      (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    unscaledField->dltaXYZ[1] =
      (uint32_t)readBuf[i + 2] << 16 | (uint32_t)readBuf[i + 3];
    unscaledField->dltaXYZ[2] =
      (uint32_t)readBuf[i + 4] << 16 | (uint32_t)readBuf[i + 5];
    i = i + 6;
  } else if (_burst_flag.dlta == OUT16) {
    unscaledField->dltaXYZ[0] = (uint32_t)readBuf[i] << 16;
    unscaledField->dltaXYZ[1] = (uint32_t)readBuf[i + 1] << 16;
    unscaledField->dltaXYZ[2] = (uint32_t)readBuf[i + 2] << 16;
    i = i + 3;
  }

  if (_burst_flag.dltv == OUT32) {
    unscaledField->dltvXYZ[0] =
      (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    unscaledField->dltvXYZ[1] =
      (uint32_t)readBuf[i + 2] << 16 | (uint32_t)readBuf[i + 3];
    unscaledField->dltvXYZ[2] =
      (uint32_t)readBuf[i + 4] << 16 | (uint32_t)readBuf[i + 5];
    i = i + 6;
  } else if (_burst_flag.dltv == OUT16) {
    unscaledField->dltvXYZ[0] = (uint32_t)readBuf[i] << 16;
    unscaledField->dltvXYZ[1] = (uint32_t)readBuf[i + 1] << 16;
    unscaledField->dltvXYZ[2] = (uint32_t)readBuf[i + 2] << 16;
    i = i + 3;
  }

  if (_burst_flag.qtn == OUT32) {
    unscaledField->qtn4[0] =
      (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    unscaledField->qtn4[1] =
      (uint32_t)readBuf[i + 2] << 16 | (uint32_t)readBuf[i + 3];
    unscaledField->qtn4[2] =
      (uint32_t)readBuf[i + 4] << 16 | (uint32_t)readBuf[i + 5];
    unscaledField->qtn4[3] =
      (uint32_t)readBuf[i + 6] << 16 | (uint32_t)readBuf[i + 7];
    i = i + 8;
  } else if (_burst_flag.qtn == OUT16) {
    unscaledField->qtn4[0] = (uint32_t)readBuf[i] << 16;
    unscaledField->qtn4[1] = (uint32_t)readBuf[i + 1] << 16;
    unscaledField->qtn4[2] = (uint32_t)readBuf[i + 2] << 16;
    unscaledField->qtn4[3] = (uint32_t)readBuf[i + 3] << 16;
    i = i + 4;
  }

  if (_burst_flag.atti == OUT32) {
    unscaledField->attiXYZ[0] =
      (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    unscaledField->attiXYZ[1] =
      (uint32_t)readBuf[i + 2] << 16 | (uint32_t)readBuf[i + 3];
    unscaledField->attiXYZ[2] =
      (uint32_t)readBuf[i + 4] << 16 | (uint32_t)readBuf[i + 5];
    i = i + 6;
  } else if (_burst_flag.atti == OUT16) {
    unscaledField->attiXYZ[0] = (uint32_t)readBuf[i] << 16;
    unscaledField->attiXYZ[1] = (uint32_t)readBuf[i + 1] << 16;
    unscaledField->attiXYZ[2] = (uint32_t)readBuf[i + 2] << 16;
    i = i + 3;
  }

  if (_burst_flag.gpio) {
    unscaledField->gpio = readBuf[i];
    i++;
  }

  if (_burst_flag.count) {
    unscaledField->counter = readBuf[i];
    i++;
  }

  if (_burst_flag.chksm) {
    unscaledField->chksm = readBuf[i];
  }
}

/**************************************************************************/
/*!
    @brief  Convert unscaled sensor data to scaled sensor data

    @param [out] scaledField
                 Pointer to struct ImuScaledData
    @param [in]  unscaledField
                 Pointer to struct ImuUnscaledData
*/
/**************************************************************************/
void EPSON_IMU_UART::_sensorField2Scaled(
  struct ImuScaledData* scaledField,
  const struct ImuUnscaledData* unscaledField) {
  if (_burst_flag.gyro) {
    scaledField->gyroXYZ[0] =
      float(unscaledField->gyroXYZ[0]) * _dev_prop.gyro_sf_dps / 65536;
    scaledField->gyroXYZ[1] =
      float(unscaledField->gyroXYZ[1]) * _dev_prop.gyro_sf_dps / 65536;
    scaledField->gyroXYZ[2] =
      float(unscaledField->gyroXYZ[2]) * _dev_prop.gyro_sf_dps / 65536;
  }
  if (_burst_flag.accl) {
    scaledField->accelXYZ[0] =
      float(unscaledField->accelXYZ[0]) * _dev_prop.accl_sf_mg / 65536;
    scaledField->accelXYZ[1] =
      float(unscaledField->accelXYZ[1]) * _dev_prop.accl_sf_mg / 65536;
    scaledField->accelXYZ[2] =
      float(unscaledField->accelXYZ[2]) * _dev_prop.accl_sf_mg / 65536;
  }
  if (_burst_flag.dlta) {
    double dlta_sf_ = _dev_prop.dlta0_sf_deg;
    scaledField->dltaXYZ[0] =
      float(unscaledField->dltaXYZ[0]) * dlta_sf_ / 65536;
    scaledField->dltaXYZ[1] =
      float(unscaledField->dltaXYZ[1]) * dlta_sf_ / 65536;
    scaledField->dltaXYZ[2] =
      float(unscaledField->dltaXYZ[2]) * dlta_sf_ / 65536;
  }
  if (_burst_flag.dltv) {
    double dltv_sf_ = _dev_prop.dltv0_sf_mps;
    scaledField->dltvXYZ[0] =
      float(unscaledField->dltvXYZ[0]) * dltv_sf_ / 65536;
    scaledField->dltvXYZ[1] =
      float(unscaledField->dltvXYZ[1]) * dltv_sf_ / 65536;
    scaledField->dltvXYZ[2] =
      float(unscaledField->dltvXYZ[2]) * dltv_sf_ / 65536;
  }
  if (_burst_flag.atti) {
    scaledField->attiXYZ[0] =
      float(unscaledField->attiXYZ[0]) * _dev_prop.ang_sf_deg / 65536;
    scaledField->attiXYZ[1] =
      float(unscaledField->attiXYZ[1]) * _dev_prop.ang_sf_deg / 65536;
    scaledField->attiXYZ[2] =
      float(unscaledField->attiXYZ[2]) * _dev_prop.ang_sf_deg / 65536;
  }
  if (_burst_flag.qtn) {
    scaledField->qtn4[0] = float(unscaledField->qtn4[0]) / (1073741824);
    scaledField->qtn4[1] = float(unscaledField->qtn4[1]) / (1073741824);
    scaledField->qtn4[2] = float(unscaledField->qtn4[2]) / (1073741824);
    scaledField->qtn4[3] = float(unscaledField->qtn4[3]) / (1073741824);
  }
  if (_burst_flag.tempc) {
    scaledField->tempC =
      (float(unscaledField->tempC + _dev_prop.tempc_25c_offset) *
       _dev_prop.tempc_sf_degc / 65536) +
      25;
  }
  if (_burst_flag.nd_ea) {
    scaledField->nd_flags = unscaledField->nd_flags;
  }
  if (_burst_flag.gpio) {
    scaledField->gpio = unscaledField->gpio;
  }
  if (_burst_flag.count) {
    scaledField->counter = unscaledField->counter;
  }
  if (_burst_flag.chksm) {
    scaledField->chksm = unscaledField->chksm;
  }
}
