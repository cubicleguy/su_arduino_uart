/**************************************************************************/
/*!
    @file     epson_accl_uart.cpp

    Epson accelerometer class

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

#include "epson_accl_uart.h"

using namespace EPSON_A_U;

//----------------------------------------------------------------------
// Array of Epson device properties struct for supported models
//----------------------------------------------------------------------

struct AcclProperties accl_model[] = {
  [INVALID] =
    {
      .model = INVALID,
      .product_id = "UNKNOWN",
      .feature_flags = 0,
      .accl_sf_g = (1),
      .tempc_sf_degc = (1),
      .tempc_25c_offset = (0),
      .tilt_sf_rad = (1),
      .delay_reset_ms = (970),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (310),
      .delay_flashreset_ms = (1910),
      .delay_selftest_ms = (200),
      .delay_sensitivity_ms = (40000),
      .delay_filter_ms = (100),  // UDF Filter setting max delay
      .delay_UDF_write_ms = (7),
      .delay_UDF_read_us = (500),
      .delay_wakeup_ms = (16),
    },
  [A352AD10] =
    {
      .model = A352AD10,
      .product_id = "A352AD10",
      .feature_flags = (HAS_REDUCED_NOISE | HAS_BIAS_TEMP_SHOCK),
      .accl_sf_g = (1.0 / float(1 << 24)),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (0),
      .tilt_sf_rad = (1 / float(1 << 29)),
      .delay_reset_ms = (970),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (310),
      .delay_flashreset_ms = (1910),
      .delay_selftest_ms = (200),
      .delay_sensitivity_ms = (40000),
      .delay_filter_ms = (100),  // UDF Filter setting max delay
      .delay_UDF_write_ms = (7),
      .delay_UDF_read_us = (500),
      .delay_wakeup_ms = (16),
    },
  [A370AD10] =
    {
      .model = A370AD10,
      .product_id = "A370AD10",
      .feature_flags = (HAS_REDUCED_NOISE | HAS_BIAS_TEMP_SHOCK),
      .accl_sf_g = (1.0 / float(1 << 24)),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (0),
      .tilt_sf_rad = (1 / float(1 << 29)),
      .delay_reset_ms = (970),
      .delay_flashtest_ms = (5),
      .delay_flashbackup_ms = (310),
      .delay_flashreset_ms = (2300),
      .delay_selftest_ms = (200),
      .delay_sensitivity_ms = (13000),
      .delay_filter_ms = (100),  // UDF Filter setting max delay
      .delay_UDF_write_ms = (7),
      .delay_UDF_read_us = (500),
      .delay_wakeup_ms = (16),
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

EPSON_ACCL_UART::EPSON_ACCL_UART()
    :  // initializer list
      UART_EPSON_COM(Serial1, 460800, -1, -1, Serial),
      _dev_prop(accl_model[INVALID]),
      _consolePort(Serial) {};

EPSON_ACCL_UART::EPSON_ACCL_UART(HardwareSerial& uartPort, uint32_t baudRate,
                                 int8_t nrst, int8_t drdy, Stream& consolePort)
    :  // initializer list
      UART_EPSON_COM(uartPort, baudRate, nrst, drdy, consolePort),
      _dev_prop(accl_model[INVALID]),
      _consolePort(consolePort) {};

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
  @brief  Output the sensor scalefactors to console
*/
/**************************************************************************/
void EPSON_ACCL_UART::sensorScaleFactorsPrint(void) {
  _consolePort.println(
    "*****************************************************************");
  _consolePort.print("Accl SF: ");
  _consolePort.print(_dev_prop.accl_sf_g, 9);
  _consolePort.println(" G/bit");
  _consolePort.print("Tilt SF: ");
  _consolePort.print(_dev_prop.tilt_sf_rad, 9);
  _consolePort.println(" rad/bit");
  _consolePort.println(
    "*****************************************************************");
}

/**************************************************************************/
/*!
    @brief  Decodes the DOUT_RATE register value to output rate in Hz

    @returns    float of output rate (Hz)
*/
/**************************************************************************/
float EPSON_ACCL_UART::sensorDecodeDoutRate(void) {
  uint8_t dout_rate = (regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO) & 0x0F00) >> 8;
  switch (dout_rate) {
    case 2:
      return 1000.0;
    case 3:
      return 500.0;
    case 4:
      return 200.0;
    case 5:
      return 100.0;
    case 6:
      return 50.0;
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
*/
/**************************************************************************/
void EPSON_ACCL_UART::sensorDecodeFilterSel(char* filterString,
                                            const size_t length) {
  if (length < 15) {
    _consolePort.println(
      "Error: char array size must be atleast 15 bytes. Bypassing...");
    return;
  }
  uint8_t filter_sel = regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO) & 0x0F;
  switch (filter_sel) {
    case 1:
      sprintf(filterString, "KAISER64FC83");
      break;
    case 2:
      sprintf(filterString, "KAISER64FC220");
      break;
    case 3:
      sprintf(filterString, "KAISER128FC36");
      break;
    case 4:
      sprintf(filterString, "KAISER128FC110");
      break;
    case 5:
      sprintf(filterString, "KAISER128FC350");
      break;
    case 6:
      sprintf(filterString, "KAISER512FC9");
      break;
    case 7:
      sprintf(filterString, "KAISER512FC16");
      break;
    case 8:
      sprintf(filterString, "KAISER512FC60");
      break;
    case 9:
      sprintf(filterString, "KAISER512FC210");
      break;
    case 10:
      sprintf(filterString, "KAISER512FC460");
      break;
    case 11:
      sprintf(filterString, "UDF4");
      break;
    case 12:
      sprintf(filterString, "UDF64");
      break;
    case 13:
      sprintf(filterString, "UDF128");
      break;
    case 14:
      sprintf(filterString, "UDF512");
      break;
    default:
      sprintf(filterString, "INVALID");
      break;
  }
}

/**************************************************************************/
/*!
    @brief      Enters Config Mode to check and output
                sensor configuration in table format to console
*/
/**************************************************************************/
void EPSON_ACCL_UART::sensorConfigPrint(void) {
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

    if (_burst_flag.tempc)
      strcat(bufLine, "\tTempC: ON");
    else
      strcat(bufLine, "\tTempC: OFF");
    _consolePort.println(bufLine);

    if (_burst_flag.acclx) {
      if (_burst_flag.inclx)
        sprintf(bufLine, "X: IncX\t");
      else
        sprintf(bufLine, "X: AccX\t");
    } else
      sprintf(bufLine, "X: OFF\t");

    if (_burst_flag.accly) {
      if (_burst_flag.incly)
        strcat(bufLine, "Y: IncY\t");
      else
        strcat(bufLine, "Y: AccY\t");
    } else
      strcat(bufLine, "Y: OFF\t");

    if (_burst_flag.acclz) {
      if (_burst_flag.inclz)
        strcat(bufLine, "Z: IncZ\t");
      else
        strcat(bufLine, "Z: AccZ\t");
    } else
      strcat(bufLine, "Z: OFF\t");
    _consolePort.println(bufLine);

    if (_burst_flag.count)
      sprintf(bufLine, "Count: ON\t");
    else
      sprintf(bufLine, "Count: OFF\t");

    if (_burst_flag.chksm)
      strcat(bufLine, "Chksm: ON");
    else
      strcat(bufLine, "Chksm: OFF");
    _consolePort.println(bufLine);

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
void EPSON_ACCL_UART::sensorHeaderPrint(void) {
  _consolePort.println();
  _consolePort.print("Sample#");

  if (_burst_flag.acclx) {
    if (_burst_flag.inclx)
      _consolePort.print(", Tilt X");
    else
      _consolePort.print(", Accl X");
  }

  if (_burst_flag.accly) {
    if (_burst_flag.incly)
      _consolePort.print(", Tilt Y");
    else
      _consolePort.print(", Accl Y");
  }

  if (_burst_flag.acclz) {
    if (_burst_flag.inclz)
      _consolePort.print(", Tilt Z");
    else
      _consolePort.print(", Accl Z");
  }

  if (_burst_flag.tempc) {
    _consolePort.print(", TempC");
  }

  if (_burst_flag.count) {
    _consolePort.print(", Count");
  }

  if (_burst_flag.nd_ea) {
    _consolePort.print(", ND_EA");
  }

  if (_burst_flag.chksm) {
    _consolePort.print(", Checksum");
  }

  _consolePort.println();
}

/**************************************************************************/
/*!
    @brief  Sends a row of scale sample data to console

    @param [in]  scaledField
                 Pointer to struct AcclScaledData
    @param [in]  sampleIndex
                 Current count of row
*/
/**************************************************************************/
void EPSON_ACCL_UART::sensorDataPrint(const struct AcclScaledData* scaledField,
                                      uint32_t sampleIndex) {
  char bufLine[160];
  char bufString[64];
  sprintf(bufLine, "%i", (int)sampleIndex);
  if (_burst_flag.acclx) {
    if (_burst_flag.inclx) {
      sprintf(bufString, ", %0.8f", scaledField->inclXYZ[0]);
      strcat(bufLine, bufString);
    } else {
      sprintf(bufString, ", %0.8f", scaledField->acclXYZ[0]);
      strcat(bufLine, bufString);
    }
  }
  if (_burst_flag.accly) {
    if (_burst_flag.incly) {
      sprintf(bufString, ", %0.8f", scaledField->inclXYZ[1]);
      strcat(bufLine, bufString);
    } else {
      sprintf(bufString, ", %0.8f", scaledField->acclXYZ[1]);
      strcat(bufLine, bufString);
    }
  }
  if (_burst_flag.acclz) {
    if (_burst_flag.inclz) {
      sprintf(bufString, ", %0.8f", scaledField->inclXYZ[2]);
      strcat(bufLine, bufString);
    } else {
      sprintf(bufString, ", %0.8f", scaledField->acclXYZ[2]);
      strcat(bufLine, bufString);
    }
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
  if (_burst_flag.chksm) {
    sprintf(bufString, ", %5d", scaledField->chksm);
    strcat(bufLine, bufString);
  }
  _consolePort.println(bufLine);
}

/**************************************************************************/
/*!
    @brief  Reads sensor burst, processes, and store to
                struct AcclScaledData

    @param [out] scaledField
                 Pointer to struct AcclScaledData

    @returns  true if successful, false if burst read contained errors
*/
/**************************************************************************/
boolean EPSON_ACCL_UART::sensorGetSensorBurst(
  struct AcclScaledData& scaledField) {
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

    @param [in]  struct AcclInitOptions
                 Initialization settings

    @param [in]  boolean verbose
                     Send register accesses to console for debugging

    @returns    True on success, False on busy bit
*/
/**************************************************************************/
boolean EPSON_ACCL_UART::sensorInitOptions(
  const struct AcclInitOptions* options, boolean verbose) {
  // SIG_CTRL
  // Configure ND flags, incl XYZ, MESMOD_SEL, TEMP_STABIL
  int sig_ctrl_hi =
    (options->accelz_out & 0x01) << 1 | (options->accely_out & 0x01) << 2 |
    (options->accelx_out & 0x01) << 3 | (options->temp_out & 0x01) << 7;

  int sig_ctrl_lo = (options->tempc_stabil & 0x01) << 2 |
                    (options->reduced_noise & 0x01) << 4 |
                    (options->incl_sel & 0x07) << 5;

  // MSC_CTRL
  // Configure DRDY function (if needed) & EXT pin function (if needed)
  int msc_ctrl_lo =
    (options->drdy_pol & 0x01) << 1 | (options->drdy_on & 0x01) << 2 |
    (options->ext_pol & 0x01) << 5 | (options->ext_sel & 0x03) << 6;

  // SMPL_CTRL
  // Configures the Data Output Rate
  // Refer to Datasheet for valid Data Output Rate & Filter Setting combinations
  int smpl_ctrl_hi = (options->dout_rate & 0x0F);

  // FILTER_CTRL
  // Configures the FIR filter
  // Refer to Datasheet for valid Data Output Rate & Filter Setting combinations
  int filter_ctrl_lo = (options->filter_sel & 0x0F);

  // UART_CTRL
  // Enable or disable UART_AUTO
  // NOTE: UART_AUTO should not be enabled when using SPI interface
  int uart_ctrl_lo = (options->uart_auto & 0x01);

  // BURST_CTRL
  // These enable or disable certain data fields in the burst read packet
  int burst_ctrl_lo =
    (options->checksum_out & 0x01) | (options->count_out & 0x01) << 1;

  int burst_ctrl_hi = 0;
  burst_ctrl_hi |=
    ((options->accelz_out & 0x01) | (options->accely_out & 0x01) << 1 |
     (options->accelx_out & 0x01) << 2 | (options->temp_out & 0x01) << 6 |
     (options->flag_out & 0x01) << 7);

  // LONGFILT
  // Longterm moving average filter setting
  int longfilt_ctrl_lo = 0;
  switch (_dev_prop.model) {
    case (A352AD10):
      longfilt_ctrl_lo =
        (options->lfilt_en & 0x01) | (options->lfilt_sel & 0x01) << 1;
      break;
    case (A370AD10):
      longfilt_ctrl_lo = (options->lfilt_en & 0x01);
      break;
    default:
      _consolePort.print(
        "WARNING: Invalid Longterm Filter setting. Ignored...");
      break;
  }
  int longfilt_tap_lo = (options->lfilt_tap & 0x0F);

  regWrite8(CMD_WINDOW1, ADDR_SIG_CTRL_HI, sig_ctrl_hi, verbose);
  regWrite8(CMD_WINDOW1, ADDR_SIG_CTRL_LO, sig_ctrl_lo, verbose);

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

  regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL_LO, burst_ctrl_lo, verbose);
  regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL_HI, burst_ctrl_hi, verbose);

  regWrite8(CMD_WINDOW1, ADDR_LONGFILT_TAP_LO, longfilt_tap_lo, verbose);
  regWrite8(CMD_WINDOW1, ADDR_LONGFILT_CTRL_LO, longfilt_ctrl_lo, verbose);

  regWrite8(CMD_WINDOW1, ADDR_XA_OFFSET_HIGH_HI,
            (options->xoffset & 0xFF000000) >> 24, verbose);
  regWrite8(CMD_WINDOW1, ADDR_XA_OFFSET_HIGH_LO,
            (options->xoffset & 0xFF0000) >> 16, verbose);
  regWrite8(CMD_WINDOW1, ADDR_XA_OFFSET_LOW_HI,
            (options->xoffset & 0xFF00) >> 8, verbose);
  regWrite8(CMD_WINDOW1, ADDR_XA_OFFSET_LOW_LO, (options->xoffset & 0xFF),
            verbose);

  regWrite8(CMD_WINDOW1, ADDR_YA_OFFSET_HIGH_HI,
            (options->yoffset & 0xFF000000) >> 24, verbose);
  regWrite8(CMD_WINDOW1, ADDR_YA_OFFSET_HIGH_LO,
            (options->yoffset & 0xFF0000) >> 16, verbose);
  regWrite8(CMD_WINDOW1, ADDR_YA_OFFSET_LOW_HI,
            (options->yoffset & 0xFF00) >> 8, verbose);
  regWrite8(CMD_WINDOW1, ADDR_YA_OFFSET_LOW_LO, (options->yoffset & 0xFF),
            verbose);

  regWrite8(CMD_WINDOW1, ADDR_ZA_OFFSET_HIGH_HI,
            (options->zoffset & 0xFF000000) >> 24, verbose);
  regWrite8(CMD_WINDOW1, ADDR_ZA_OFFSET_HIGH_LO,
            (options->zoffset & 0xFF0000) >> 16, verbose);
  regWrite8(CMD_WINDOW1, ADDR_ZA_OFFSET_LOW_HI,
            (options->zoffset & 0xFF00) >> 8, verbose);
  regWrite8(CMD_WINDOW1, ADDR_ZA_OFFSET_LOW_LO, (options->zoffset & 0xFF),
            verbose);

  regWrite8(CMD_WINDOW1, ADDR_XA_ALARM_HI, (options->xalarm_upper & 0xFF),
            verbose);
  regWrite8(CMD_WINDOW1, ADDR_XA_ALARM_LO, (options->xalarm_lower & 0xFF),
            verbose);

  regWrite8(CMD_WINDOW1, ADDR_YA_ALARM_HI, (options->yalarm_upper & 0xFF),
            verbose);
  regWrite8(CMD_WINDOW1, ADDR_YA_ALARM_LO, (options->yalarm_lower & 0xFF),
            verbose);

  regWrite8(CMD_WINDOW1, ADDR_ZA_ALARM_HI, (options->zalarm_upper & 0xFF),
            verbose);
  regWrite8(CMD_WINDOW1, ADDR_ZA_ALARM_LO, (options->zalarm_lower & 0xFF),
            verbose);

  // Update _read_burst8_len after BURST_CTRL registers written
  _read_burst8_len = _sensorDecodeBurstCtrl();
  return true;
}

/**************************************************************************/
/*!
    @brief      Enters Config Mode to read back all register values from the
                Sensor.
*/
/**************************************************************************/
void EPSON_ACCL_UART::sensorRegisterDump(void) {
  if (sensorStop() == true) {
    // print the current values in all the Sensor registers
    _consolePort.println("\r\nRegister Dump:");
    regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO, true);
    switch (_dev_prop.model) {
      case (A352AD10):
        regRead16(CMD_WINDOW0, ADDR_DIAG_STAT, true);
        break;
      case (A370AD10):
        regRead16(CMD_WINDOW0, ADDR_DIAG_STAT1, true);
        break;
      default:
        break;
    }
    regRead16(CMD_WINDOW0, ADDR_FLAG, true);
    regRead16(CMD_WINDOW0, ADDR_COUNT, true);
    if (_dev_prop.model == A370AD10) {
      regRead16(CMD_WINDOW0, ADDR_DIAG_STAT2, true);
    }
    _consolePort.println();

    regRead16(CMD_WINDOW0, ADDR_TEMP_HIGH, true);
    regRead16(CMD_WINDOW0, ADDR_TEMP_LOW, true);
    regRead16(CMD_WINDOW0, ADDR_XACCL_HIGH, true);
    regRead16(CMD_WINDOW0, ADDR_XACCL_LOW, true);
    regRead16(CMD_WINDOW0, ADDR_YACCL_HIGH, true);
    regRead16(CMD_WINDOW0, ADDR_YACCL_LOW, true);
    regRead16(CMD_WINDOW0, ADDR_ZACCL_HIGH, true);
    regRead16(CMD_WINDOW0, ADDR_ZACCL_LOW, true);
    _consolePort.println();

    regRead16(CMD_WINDOW0, ADDR_XTILT_HIGH, true);
    regRead16(CMD_WINDOW0, ADDR_XTILT_LOW, true);
    regRead16(CMD_WINDOW0, ADDR_YTILT_HIGH, true);
    regRead16(CMD_WINDOW0, ADDR_YTILT_LOW, true);
    regRead16(CMD_WINDOW0, ADDR_ZTILT_HIGH, true);
    regRead16(CMD_WINDOW0, ADDR_ZTILT_LOW, true);
    _consolePort.println();

    regRead16(CMD_WINDOW0, ADDR_ID, true);
    _consolePort.println();

    regRead16(CMD_WINDOW1, ADDR_SIG_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_UART_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO, true);
    regRead16(CMD_WINDOW1, ADDR_BURST_CTRL_LO, true);
    _consolePort.println();

    regRead16(CMD_WINDOW1, ADDR_LONGFILT_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_LONGFILT_TAP_LO, true);
    _consolePort.println();

    regRead16(CMD_WINDOW1, ADDR_XA_OFFSET_HIGH_LO, true);
    regRead16(CMD_WINDOW1, ADDR_XA_OFFSET_LOW_LO, true);
    regRead16(CMD_WINDOW1, ADDR_YA_OFFSET_HIGH_LO, true);
    regRead16(CMD_WINDOW1, ADDR_YA_OFFSET_LOW_LO, true);
    regRead16(CMD_WINDOW1, ADDR_ZA_OFFSET_HIGH_LO, true);
    regRead16(CMD_WINDOW1, ADDR_ZA_OFFSET_LOW_LO, true);
    _consolePort.println();

    regRead16(CMD_WINDOW1, ADDR_XA_ALARM_LO, true);
    regRead16(CMD_WINDOW1, ADDR_YA_ALARM_LO, true);
    regRead16(CMD_WINDOW1, ADDR_ZA_ALARM_LO, true);
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
                 index for AcclProperties array (all supported
                 models)

    @returns    True on success, False if model is out of bounds
*/
/**************************************************************************/
boolean EPSON_ACCL_UART::sensorModelSelect(int model) {
  if ((model <= INVALID) || (model >= MAX_MODELS)) {
    return false;
  }
  _dev_prop = accl_model[model];
  return true;
}

/**************************************************************************/
/*!
    @brief  Detects the Epson sensor model by reading Product ID
                and returning the index in the array of model structs

    @returns    int
                    index of product ID found within the model array
                                of device properties
*/
/**************************************************************************/
int EPSON_ACCL_UART::sensorModelDetect(void) {
  char model_read[9];
  sensorGetProdID(model_read);
  _consolePort.print("Detected PROD_ID:");
  _consolePort.println(model_read);
  int index;
  for (index = INVALID; index < MAX_MODELS; index++) {
    if (strcmp(accl_model[index].product_id, model_read) == 0) break;
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
boolean EPSON_ACCL_UART::sensorStart(boolean verbose) {
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
boolean EPSON_ACCL_UART::sensorStop(boolean verbose) {
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
    @brief  Enters Config Mode to initiate a selftest ACC_TEST, TEMP_TEST,
            VDD_TEST, and returns the DIAG_STAT register value

    @param [in]  boolean verbose
                     Send register accesses to console for debugging

    @returns    DIAG_STAT return value, 0 on success, nonzero for any errors
                and 0xFFFF if SELF_TEST bit is stuck 1b.
*/
/**************************************************************************/
uint16_t EPSON_ACCL_UART::sensorSelfTest(boolean verbose) {
  uint16_t valRead;

  if (!sensorStop(verbose)) {
    _consolePort.println("Warning: Not entering Config Mode");
  }

  // Send the self test command
  _consolePort.print("Initiate ACC_TEST, TEMP_TEST, VDD_TEST...");
  regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_SELFTEST, verbose);

  // Wait for self test to process
  delay(_dev_prop.delay_selftest_ms);

  // Check that SELF_TEST bit returns 0
  valRead =
    regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, verbose) & VAL_SELF_TEST_BIT;
  if (valRead != 0) {
    _consolePort.println(
      "Warning: ACC_TEST, TEMP_TEST, or VDD_TEST bit not returning to 0");
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
    @brief  Enters Config Mode to initiate a sensitivity test Z_SENS_TEST,
            Y_SENS_TEST, X_SENS_TEST, and returns the DIAG_STAT register value
                        WARNING: Each axis can take up to 40 seconds to complete
   the test.

    @param [in]  boolean verbose
                     Send register accesses to console for debugging

    @returns    DIAG_STAT return value, 0 on success, nonzero for any errors
                and 0xFFFF if any of SENS_TEST bit is stuck 1b.
*/
/**************************************************************************/
uint16_t EPSON_ACCL_UART::sensorSensitivityTest(boolean verbose) {
  uint16_t valRead;

  if (!sensorStop(verbose)) {
    _consolePort.println("Warning: Not entering Config Mode");
  }

  // Send the self test command
  _consolePort.print("Initiate Z_SENS_TEST...");
  regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_ZSENSTEST, verbose);

  // Wait for self test to process
  delay(_dev_prop.delay_sensitivity_ms);

  // Check that bit returns 0
  valRead = regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, verbose) & BIT_14;
  if (valRead != 0) {
    _consolePort.println("Warning: Z_SENS_TEST bit not returning to 0");
    return 0xFFFF;
  }
  _consolePort.println("Done.");

  // Read the results in DIAG_STAT
  valRead = regRead16(CMD_WINDOW0, ADDR_DIAG_STAT, verbose);
  if ((valRead & VAL_DIAG_STAT_SENS) != 0) {
    _consolePort.print("Warning: DIAG_STAT returned some errors - ");
    _consolePort.println(valRead, HEX);
    return valRead;
  }

  _consolePort.print("Initiate Y_SENS_TEST...");
  regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_YSENSTEST, verbose);

  // Wait for self test to process
  delay(_dev_prop.delay_sensitivity_ms);

  // Check that bit returns 0
  valRead = regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, verbose) & BIT_13;
  if (valRead != 0) {
    _consolePort.println("Warning: Y_SENS_TEST bit not returning to 0");
    return 0xFFFF;
  }
  _consolePort.println("Done.");

  // Read the results in DIAG_STAT
  valRead = regRead16(CMD_WINDOW0, ADDR_DIAG_STAT, verbose);
  if ((valRead & VAL_DIAG_STAT_SENS) != 0) {
    _consolePort.print("Warning: DIAG_STAT returned some errors - ");
    _consolePort.println(valRead, HEX);
    return valRead;
  }

  _consolePort.print("Initiate X_SENS_TEST...");
  regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_XSENSTEST, verbose);

  // Wait for self test to process
  delay(_dev_prop.delay_sensitivity_ms);

  // Check that bit returns 0
  valRead = regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, verbose) & BIT_12;
  if (valRead != 0) {
    _consolePort.println("Warning: X_SENS_TEST bit not returning to 0");
    return 0xFFFF;
  }
  _consolePort.println("Done.");

  // Read the results in DIAG_STAT
  valRead = regRead16(CMD_WINDOW0, ADDR_DIAG_STAT, verbose);
  if ((valRead & VAL_DIAG_STAT_SENS) != 0) {
    _consolePort.print("Warning: DIAG_STAT returned some errors - ");
    _consolePort.println(valRead, HEX);
    return valRead;
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
boolean EPSON_ACCL_UART::sensorReset(boolean verbose) {
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
boolean EPSON_ACCL_UART::sensorFlashTest(boolean verbose) {
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
    regRead16(CMD_WINDOW0, ADDR_DIAG_STAT1, verbose) & VAL_DIAG_FLASH_ERR;

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
void EPSON_ACCL_UART::sensorGetProdID(char* prodID, const size_t length) {
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
uint16_t EPSON_ACCL_UART::sensorGetVersion(void) {
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
void EPSON_ACCL_UART::sensorGetSerialNumber(char* serialNumber,
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
int8_t EPSON_ACCL_UART::_sensorDecodeBurstCtrl(boolean verbose) {
  uint8_t burst_len = 0;

  uint16_t sig_ctrl = regRead16(CMD_WINDOW1, ADDR_SIG_CTRL_LO, verbose) & 0xE0;
  uint16_t burst_ctrl =
    regRead16(CMD_WINDOW1, ADDR_BURST_CTRL_LO, verbose) & 0xC703;

  // burst_ctrl check
  _burst_flag.nd_ea = (burst_ctrl & BIT_15) ? 1 : 0;
  _burst_flag.tempc = (burst_ctrl & BIT_14) ? 1 : 0;
  _burst_flag.acclx = (burst_ctrl & BIT_10) ? 1 : 0;
  _burst_flag.accly = (burst_ctrl & BIT_9) ? 1 : 0;
  _burst_flag.acclz = (burst_ctrl & BIT_8) ? 1 : 0;
  _burst_flag.count = (burst_ctrl & BIT_1) ? 1 : 0;
  _burst_flag.chksm = (burst_ctrl & BIT_0) ? 1 : 0;
  // sig_ctrl check
  _burst_flag.inclx = (sig_ctrl & 0x80) ? 1 : 0;
  _burst_flag.incly = (sig_ctrl & 0x40) ? 1 : 0;
  _burst_flag.inclz = (sig_ctrl & 0x20) ? 1 : 0;

  // Calc burst_len
  if (_burst_flag.nd_ea) burst_len += 2;
  if (_burst_flag.tempc) burst_len += 4;
  if (_burst_flag.acclx) burst_len += 4;
  if (_burst_flag.accly) burst_len += 4;
  if (_burst_flag.acclz) burst_len += 4;
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
boolean EPSON_ACCL_UART::_sensorReadBurst16(uint16_t* arr,
                                            const size_t length) {
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
  boolean retval =
    UART_EPSON_COM::readN(arr, CMD_BURST, _read_burst8_len, READBURST_RETRIES);

  return retval;
}

/**************************************************************************/
/*!
    @brief  Convert burst read sensor buffer data to fields

    @param [out] unscaledField
                 Pointer to struct AcclUnscaledData

    @param [in]  readBuf
                 Pointer to 16-bit Array

    @param [in]  length
                 Size of 16-bit Array in bytes
*/
/**************************************************************************/
void EPSON_ACCL_UART::_sensorBuf2Field(struct AcclUnscaledData* unscaledField,
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

  if (_burst_flag.tempc) {
    unscaledField->tempC =
      (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    i = i + 2;
  }

  if (_burst_flag.acclx) {
    if (_burst_flag.inclx) {
      unscaledField->inclXYZ[0] =
        (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    } else {
      unscaledField->acclXYZ[0] =
        (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    }
    i = i + 2;
  }

  if (_burst_flag.accly) {
    if (_burst_flag.incly) {
      unscaledField->inclXYZ[1] =
        (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    } else {
      unscaledField->acclXYZ[1] =
        (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    }
    i = i + 2;
  }

  if (_burst_flag.acclz) {
    if (_burst_flag.inclz) {
      unscaledField->inclXYZ[2] =
        (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    } else {
      unscaledField->acclXYZ[2] =
        (uint32_t)readBuf[i] << 16 | (uint32_t)readBuf[i + 1];
    }
    i = i + 2;
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
                 Pointer to struct AcclScaledData
    @param [in]  unscaledField
                 Pointer to struct AcclUnscaledData
*/
/**************************************************************************/
void EPSON_ACCL_UART::_sensorField2Scaled(
  struct AcclScaledData* scaledField,
  const struct AcclUnscaledData* unscaledField) {
  if (_burst_flag.acclx) {
    scaledField->acclXYZ[0] =
      float(unscaledField->acclXYZ[0]) * _dev_prop.accl_sf_g;
  }

  if (_burst_flag.accly) {
    scaledField->acclXYZ[1] =
      float(unscaledField->acclXYZ[1]) * _dev_prop.accl_sf_g;
  }

  if (_burst_flag.acclz) {
    scaledField->acclXYZ[2] =
      float(unscaledField->acclXYZ[2]) * _dev_prop.accl_sf_g;
  }

  if (_burst_flag.inclx) {
    scaledField->inclXYZ[0] =
      float(unscaledField->inclXYZ[0]) * _dev_prop.tilt_sf_rad;
  }

  if (_burst_flag.incly) {
    scaledField->inclXYZ[1] =
      float(unscaledField->inclXYZ[1]) * _dev_prop.tilt_sf_rad;
  }

  if (_burst_flag.inclz) {
    scaledField->acclXYZ[2] =
      float(unscaledField->inclXYZ[2]) * _dev_prop.tilt_sf_rad;
  }

  if (_burst_flag.tempc) {
    scaledField->tempC =
      (float(unscaledField->tempC + _dev_prop.tempc_25c_offset) *
       _dev_prop.tempc_sf_degc) +
      34.987;
  }

  if (_burst_flag.nd_ea) {
    scaledField->nd_flags = unscaledField->nd_flags;
  }
  if (_burst_flag.count) {
    scaledField->counter = unscaledField->counter;
  }
  if (_burst_flag.chksm) {
    scaledField->chksm = unscaledField->chksm;
  }
}
