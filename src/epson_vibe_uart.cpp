/**************************************************************************/
/*!
    @file     epson_vibe_uart.cpp

    Epson Vibration sensor Class

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

#include "epson_vibe_uart.h"

using namespace EPSON_V_U;

//----------------------------------------------------------------------
// Array of Epson device properties struct for supported models
//----------------------------------------------------------------------

struct VibeProperties vibe_model[] = {
  [INVALID] =
    {
      .model = INVALID,
      .product_id = "UNKNOWN",
      .feature_flags = 0,
      .vel_sf_m_s = (1),
      .disp_sf_m = (1),
      .tempc_sf_degc = (1),
      .tempc_25c_offset = (0),
      .delay_reset_ms = (970),
      .delay_flashbackup_ms = (310),
      .delay_flashreset_ms = (2300),
      .delay_flashtest_ms = (5),
      .delay_selftest_ms = (300),
      .delay_structres_test_ms = (820),
      .delay_outputsel_ms = (118),
      .delay_wakeup_ms = (16),
    },
  [A342VD10] =
    {
      .model = A342VD10,
      .product_id = "A342VD10",
      .feature_flags = (HAS_ALIGNMENT_COMP),
      .vel_sf_m_s = (1.0 / float(1 << 22)),
      .disp_sf_m = (1.0 / float(1 << 22)),
      .tempc_sf_degc = (-0.0037918),
      .tempc_25c_offset = (0),
      .delay_reset_ms = (970),
      .delay_flashbackup_ms = (310),
      .delay_flashreset_ms = (2300),
      .delay_flashtest_ms = (5),
      .delay_selftest_ms = (300),
      .delay_structres_test_ms = (820),
      .delay_outputsel_ms = (118),
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

EPSON_VIBE_UART::EPSON_VIBE_UART()
    :  // initializer list
      UART_EPSON_COM(Serial1, 460800, -1, -1, Serial),
      _dev_prop(vibe_model[INVALID]),
      _consolePort(Serial) {};

EPSON_VIBE_UART::EPSON_VIBE_UART(HardwareSerial& uartPort, uint32_t baudRate,
                                 int8_t nrst, int8_t drdy, Stream& consolePort)
    :  // initializer list
      UART_EPSON_COM(uartPort, baudRate, nrst, drdy, consolePort),
      _dev_prop(vibe_model[INVALID]),
      _consolePort(consolePort) {};

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
  @brief  Output the sensor scalefactors to console
*/
/**************************************************************************/
void EPSON_VIBE_UART::sensorScaleFactorsPrint(void) {
  _consolePort.println(
    "*****************************************************************");
  _consolePort.print("Velocity SF: ");
  _consolePort.print(_dev_prop.vel_sf_m_s, 9);
  _consolePort.println(" (m/s)/bit");
  _consolePort.print("Displacement SF: ");
  _consolePort.print(_dev_prop.disp_sf_m, 9);
  _consolePort.println(" m/bit");
  _consolePort.println(
    "*****************************************************************");
}

/**************************************************************************/
/*!
    @brief  Decodes the DOUT_RATE_RMSPP to output rate in Hz

    @returns    float of output rate (Hz)
*/
/**************************************************************************/
float EPSON_VIBE_UART::sensorDecodeDoutRate(void) {
  uint8_t dout_rate_rmspp =
    (regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO) & 0xFF00) >> 8;
  if (_output_sel == 0x00) {
    // RAW velocity
    return 3000.0;
  } else if (_output_sel == 0x04) {
    // RAW displacement
    return 300.0;
  } else if ((_output_sel & 0x4) == 0x00) {
    // RMS or PP velocity
    return 1 / (dout_rate_rmspp * 0.1);
  } else {
    // RMS or PP displacement
    return 1 / (dout_rate_rmspp * 1.0);
  }
}

/**************************************************************************/
/*!
    @brief  Decodes the UPDATE_RATERMSPP to update rate in Hz

    @returns    float of update rate (Hz)
*/
/**************************************************************************/
float EPSON_VIBE_UART::sensorDecodeUpdateRate(void) {
  uint8_t update_rate_rmspp =
    (regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO) & 0x0F);
  if (_output_sel == 0x00) {
    // RAW velocity
    return 3000.0;
  } else if (_output_sel == 0x04) {
    // RAW displacement
    return 300.0;
  } else if ((_output_sel & 0x4) == 0x00) {
    // RMS PP velocity
    return (float)(1 / 3000.0 * 16 * (double)exp2(update_rate_rmspp));
  } else {
    // RMS PP displacement
    return (float)(1 / 300.0 * 16 * (double)exp2(update_rate_rmspp));
  }
}

/**************************************************************************/
/*!
    @brief      Enters Config Mode to check and output
                sensor configuration in table format to console
*/
/**************************************************************************/
void EPSON_VIBE_UART::sensorConfigPrint(void) {
  if (sensorStop() == true) {
    // Print to formatted table
    _consolePort.println(
      "\n****************************************************************"
      "*");
    char bufLine[80];
    char prod_id[] = "XXXXXXXX";
    char serial_id[] = "XXXXXXXX";

    sensorGetProdID(prod_id, sizeof(prod_id));
    sensorGetSerialNumber(serial_id, sizeof(serial_id));
    sprintf(bufLine, "PROD_ID: %s\tSERIAL_ID: %s\tVERSION: %x", prod_id,
            serial_id, sensorGetVersion());
    _consolePort.println(bufLine);

    sprintf(bufLine, "DOUT_RATE: %0.4f\tUPDATE_RATE: %0.4f",
            sensorDecodeDoutRate(), sensorDecodeUpdateRate());
    _consolePort.println(bufLine);

    _read_burst8_len = _sensorDecodeBurstCtrl();
    if (_burst_flag.nd_ea)
      sprintf(bufLine, "ND_EA: ON\t");
    else
      sprintf(bufLine, "ND_EA: OFF\t");

    if (_burst_flag.tempc)
      strcat(bufLine, "\tTempC: ON");
    else
      strcat(bufLine, "\tTempC: OFF");
    _consolePort.println(bufLine);

    bufLine[0] = 0;
    if (_burst_flag.sensx) {
      switch (_output_sel) {
        case 0:
          strcat(bufLine, "X: Vel RAW\t");
          break;
        case 1:
          strcat(bufLine, "X: Vel RMS\t");
          break;
        case 2:
          strcat(bufLine, "X: Vel P-P\t");
          break;
        case 4:
          strcat(bufLine, "X: Disp RAW\t");
          break;
        case 5:
          strcat(bufLine, "X: Disp RMS\t");
          break;
        case 6:
          strcat(bufLine, "X: Disp P-P\t");
          break;
        default:
          strcat(bufLine, "X: RSVD\t");
          break;
      }
    } else {
      strcat(bufLine, "X: OFF\t");
    }
    if (_burst_flag.sensy) {
      switch (_output_sel) {
        case 0:
          strcat(bufLine, "\tY: Vel RAW\t");
          break;
        case 1:
          strcat(bufLine, "\tY: Vel RMS\t");
          break;
        case 2:
          strcat(bufLine, "\tY: Vel P-P\t");
          break;
        case 4:
          strcat(bufLine, "\tY: Disp RAW\t");
          break;
        case 5:
          strcat(bufLine, "\tY: Disp RMS\t");
          break;
        case 6:
          strcat(bufLine, "\tY: Disp P-P\t");
          break;
        default:
          strcat(bufLine, "\tY: RSVD\t");
          break;
      }
    } else {
      strcat(bufLine, "\tY: OFF\t");
    }
    if (_burst_flag.sensx) {
      switch (_output_sel) {
        case 0:
          strcat(bufLine, "\tZ: Vel RAW\t");
          break;
        case 1:
          strcat(bufLine, "\tZ: Vel RMS\t");
          break;
        case 2:
          strcat(bufLine, "\tZ: Vel P-P\t");
          break;
        case 4:
          strcat(bufLine, "\tZ: Disp RAW\t");
          break;
        case 5:
          strcat(bufLine, "\tZ: Disp RMS\t");
          break;
        case 6:
          strcat(bufLine, "\tZ: Disp P-P\t");
          break;
        default:
          strcat(bufLine, "\tZ: RSVD\t");
          break;
      }
    } else {
      strcat(bufLine, "\tZ: OFF\t");
    }
    _consolePort.println(bufLine);

    if (_burst_flag.count)
      sprintf(bufLine, "Count: ON\t\t");
    else
      sprintf(bufLine, "Count: OFF\t\t");
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
void EPSON_VIBE_UART::sensorHeaderPrint(void) {
  char bufLine[160];
  sprintf(bufLine, "Sample#");

  if (_burst_flag.sensx) {
    switch (_output_sel) {
      case 0:
        strcat(bufLine, ", VelX RAW");
        break;
      case 1:
        strcat(bufLine, ", VelX RMS");
        break;
      case 2:
        strcat(bufLine, ", VelX P-P");
        break;
      case 4:
        strcat(bufLine, ", DispX RAW");
        break;
      case 5:
        strcat(bufLine, ", DispX RMS");
        break;
      case 6:
        strcat(bufLine, ", DispX P-P");
        break;
      default:
        strcat(bufLine, ", X RSVD");
        break;
    }
  }

  if (_burst_flag.sensy) {
    switch (_output_sel) {
      case 0:
        strcat(bufLine, ", VelY RAW");
        break;
      case 1:
        strcat(bufLine, ", VelY RMS");
        break;
      case 2:
        strcat(bufLine, ", VelY P-P");
        break;
      case 4:
        strcat(bufLine, ", DispY RAW");
        break;
      case 5:
        strcat(bufLine, ", DispY RMS");
        break;
      case 6:
        strcat(bufLine, ", DispY P-P");
        break;
      default:
        strcat(bufLine, ", Y RSVD");
        break;
    }
  }

  if (_burst_flag.sensz) {
    switch (_output_sel) {
      case 0:
        strcat(bufLine, ", VelZ RAW");
        break;
      case 1:
        strcat(bufLine, ", VelZ RMS");
        break;
      case 2:
        strcat(bufLine, ", VelZ P-P");
        break;
      case 4:
        strcat(bufLine, ", DispZ RAW");
        break;
      case 5:
        strcat(bufLine, ", DispZ RMS");
        break;
      case 6:
        strcat(bufLine, ", DispZ P-P");
        break;
      default:
        strcat(bufLine, ", Z RSVD");
        break;
    }
  }

  if (_burst_flag.tempc) {
    strcat(bufLine, ", TempC");
    if (_temp_sel == 0) {
      strcat(bufLine, ", errEXI, errALARM, 2bit");
    }
  }
  if (_burst_flag.count) {
    strcat(bufLine, ", Count");
  }
  if (_burst_flag.nd_ea) {
    strcat(bufLine, ", ND_EA");
  }
  if (_burst_flag.chksm) {
    strcat(bufLine, ", Checksum");
  }
  _consolePort.println(bufLine);
}

/**************************************************************************/
/*!
    @brief  Sends a row of scale sample data to console

    @param [in]  scaledField
                 Pointer to struct VibeScaledData
    @param [in]  sampleIndex
                 Current count of row
*/
/**************************************************************************/
void EPSON_VIBE_UART::sensorDataPrint(const struct VibeScaledData* scaledField,
                                      uint32_t sampleIndex) {
  char bufLine[160];
  char bufString[64];
  sprintf(bufLine, "%i", (int)sampleIndex);
  if (_burst_flag.sensx) {
    sprintf(bufString, ", %0.9f", scaledField->sensXYZ[0]);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.sensy) {
    sprintf(bufString, ", %0.9f", scaledField->sensXYZ[1]);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.sensz) {
    sprintf(bufString, ", %0.9f", scaledField->sensXYZ[2]);
    strcat(bufLine, bufString);
  }
  if (_burst_flag.tempc) {
    sprintf(bufString, ", %0.3f", scaledField->tempC);
    strcat(bufLine, bufString);
    if (_temp_sel == 0) {
      sprintf(bufString, ", %i", scaledField->errEXI);
      strcat(bufLine, bufString);
      sprintf(bufString, ", %i", scaledField->errALARM);
      strcat(bufLine, bufString);
      sprintf(bufString, ", %i", scaledField->two_bit_count);
      strcat(bufLine, bufString);
    }
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
                struct VibeScaledData

    @param [out] scaledField
                 Pointer to struct VibeScaledData

    @returns  true if successful, false if burst read contained errors
*/
/**************************************************************************/
boolean EPSON_VIBE_UART::sensorGetSensorBurst(
  struct VibeScaledData& scaledField) {
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
    @brief  Programs the device registers from provided init_options
   struct

    @param [in]  struct VibeInitOptions
                 Initialization settings

    @param [in]  boolean verbose
                     Send register accesses to console for debugging

    @returns    True on success, False on busy bit
*/
/**************************************************************************/
boolean EPSON_VIBE_UART::sensorInitOptions(
  const struct VibeInitOptions* options, boolean verbose) {
  // SIG_CTRL
  // Configure TEMP, sens XYZ, OUTPUT_SEL, TEMP_SEL
  int sig_ctrl_hi =
    (options->sensz_out & 0x01) << 1 | (options->sensy_out & 0x01) << 2 |
    (options->sensx_out & 0x01) << 3 | (options->temp_out & 0x01) << 7;

  int sig_ctrl_lo =
    (options->temp_sel & 0x01) << 1 | (options->output_sel & 0x0F) << 4;

  // MSC_CTRL
  // Configure DRDY function (if needed) & EXT pin function (if needed)
  int msc_ctrl_lo = (options->drdy_pol & 0x01) << 1 |
                    (options->drdy_on & 0x01) << 2 |
                    (options->ext_pol & 0x01) << 5;

  // SMPL_CTRL
  // Configures the Data Output Rate and Update Rate
  // Refer to Datasheet for valid setting combinations
  int smpl_ctrl_hi = (options->dout_rate_rmspp & 0xFF);
  int smpl_ctrl_lo = (options->update_rate_rmspp & 0x0F);

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
    ((options->sensz_out & 0x01) | (options->sensy_out & 0x01) << 1 |
     (options->sensx_out & 0x01) << 2 | (options->temp_out & 0x01) << 6 |
     (options->flag_out & 0x01) << 7);

  regWrite8(CMD_WINDOW1, ADDR_SIG_CTRL_HI, sig_ctrl_hi, verbose);
  regWrite8(CMD_WINDOW1, ADDR_SIG_CTRL_LO, sig_ctrl_lo, verbose);
  _output_sel = options->output_sel & 0x0F;
  _temp_sel = options->temp_sel;
  delay(_dev_prop.delay_outputsel_ms);

  // Check that the OUTPUT_STAT bit returns 0
  uint16_t rxData = regRead16(CMD_WINDOW1, ADDR_SIG_CTRL_LO, verbose);
  if ((rxData & 0x01) == 1) {
    _consolePort.println("...Error: OUTPUT_STAT bit did not return to 0b.");
    return false;
  };

  regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_LO, msc_ctrl_lo, verbose);

  regWrite8(CMD_WINDOW1, ADDR_SMPL_CTRL_HI, smpl_ctrl_hi, verbose);
  regWrite8(CMD_WINDOW1, ADDR_SMPL_CTRL_LO, smpl_ctrl_lo, verbose);

  regWrite8(CMD_WINDOW1, ADDR_UART_CTRL_LO, uart_ctrl_lo, verbose);
  _uart_auto = boolean(uart_ctrl_lo);

  regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL_LO, burst_ctrl_lo, verbose);
  regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL_HI, burst_ctrl_hi, verbose);

  regWrite8(CMD_WINDOW1, ADDR_X_ALARM_LO, (options->xalarm & 0x00FF), verbose);
  regWrite8(CMD_WINDOW1, ADDR_X_ALARM_HI, (options->xalarm & 0xFF00) >> 8,
            verbose);

  regWrite8(CMD_WINDOW1, ADDR_Y_ALARM_LO, (options->yalarm & 0x00FF), verbose);
  regWrite8(CMD_WINDOW1, ADDR_Y_ALARM_HI, (options->yalarm & 0xFF00) >> 8,
            verbose);

  regWrite8(CMD_WINDOW1, ADDR_Z_ALARM_LO, (options->zalarm & 0x00FF), verbose);
  regWrite8(CMD_WINDOW1, ADDR_Z_ALARM_HI, (options->zalarm & 0xFF00) >> 8,
            verbose);

  // Update _read_burst8_len after BURST_CTRL registers written
  _read_burst8_len = _sensorDecodeBurstCtrl(verbose);
  return true;
}

/**************************************************************************/
/*!
    @brief      Enters Config Mode to read back all register values from
   the Sensor.
*/
/**************************************************************************/
void EPSON_VIBE_UART::sensorRegisterDump(void) {
  if (sensorStop() == true) {
    // print the current values in all the Sensor registers
    _consolePort.println("\r\nRegister Dump:");
    regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO, true);
    regRead16(CMD_WINDOW0, ADDR_DIAG_STAT1, true);
    regRead16(CMD_WINDOW0, ADDR_FLAG, true);
    regRead16(CMD_WINDOW0, ADDR_COUNT, true);
    regRead16(CMD_WINDOW0, ADDR_DIAG_STAT2, true);
    _consolePort.println();

    regRead16(CMD_WINDOW0, ADDR_TEMP1, true);
    regRead16(CMD_WINDOW0, ADDR_ACC_SELFTEST_DATA1_LO, true);
    regRead16(CMD_WINDOW0, ADDR_ACC_SELFTEST_DATA2_LO, true);
    regRead16(CMD_WINDOW0, ADDR_TEMP2, true);

    regRead16(CMD_WINDOW0, ADDR_XVELC_HIGH, true);
    regRead16(CMD_WINDOW0, ADDR_XVELC_LOW, true);
    regRead16(CMD_WINDOW0, ADDR_YVELC_HIGH, true);
    regRead16(CMD_WINDOW0, ADDR_YVELC_LOW, true);
    regRead16(CMD_WINDOW0, ADDR_ZVELC_HIGH, true);
    regRead16(CMD_WINDOW0, ADDR_ZVELC_LOW, true);
    _consolePort.println();

    regRead16(CMD_WINDOW0, ADDR_ID, true);
    _consolePort.println();

    regRead16(CMD_WINDOW1, ADDR_SIG_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_UART_CTRL_LO, true);
    regRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO, true);
    regRead16(CMD_WINDOW1, ADDR_BURST_CTRL_LO, true);
    _consolePort.println();

    regRead16(CMD_WINDOW1, ADDR_ALIGNMENT_COEFF_CMD, true);
    regRead16(CMD_WINDOW1, ADDR_ALIGNMENT_COEFF_DATA, true);
    regRead16(CMD_WINDOW1, ADDR_ALIGNMENT_COEFF_ADDR, true);
    _consolePort.println();

    regRead16(CMD_WINDOW1, ADDR_X_ALARM_LO, true);
    regRead16(CMD_WINDOW1, ADDR_Y_ALARM_LO, true);
    regRead16(CMD_WINDOW1, ADDR_Z_ALARM_LO, true);
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
                 index for VibeProperties array (all supported
                 models)

    @returns    True on success, False if model is out of bounds
*/
/**************************************************************************/
boolean EPSON_VIBE_UART::sensorModelSelect(int model) {
  if ((model <= INVALID) || (model >= MAX_MODELS)) {
    return false;
  }
  _dev_prop = vibe_model[model];
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
int EPSON_VIBE_UART::sensorModelDetect(void) {
  char model_read[9];
  sensorGetProdID(model_read);
  _consolePort.print("Detected PROD_ID:");
  _consolePort.println(model_read);
  int index;
  for (index = INVALID; index < MAX_MODELS; index++) {
    if (strcmp(vibe_model[index].product_id, model_read) == 0) break;
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
boolean EPSON_VIBE_UART::sensorStart(boolean verbose) {
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
boolean EPSON_VIBE_UART::sensorStop(boolean verbose) {
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
            VDD_TEST, and returns the DIAG_STAT1 register value

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

    @returns    DIAG_STAT1 return value, 0 on success, nonzero for any
   errors and 0xFFFF if _TEST bit is stuck 1b.
*/
/**************************************************************************/
uint16_t EPSON_VIBE_UART::sensorSelfTest(boolean verbose) {
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
  // Read the results in DIAG_STAT1
  valRead = regRead16(CMD_WINDOW0, ADDR_DIAG_STAT1, verbose);
  if ((valRead & VAL_DIAG_STAT_MASK) != 0) {
    _consolePort.print("Warning: DIAG_STAT1 returned some errors - ");
    _consolePort.println(valRead, HEX);
  }
  return valRead;
}

/**************************************************************************/
/*!
    @brief  Enters Config Mode to initiate a structural resonance test
            and returns the DIAG_STAT2 register value

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

    @returns    DIAG_STAT1 return value, 0 on success, nonzero for any
                errors and 0xFFFF if the _TEST bit is stuck 1b.
*/
/**************************************************************************/
uint16_t EPSON_VIBE_UART::sensorStructuralResTest(boolean verbose) {
  uint16_t valRead;

  if (!sensorStop(verbose)) {
    _consolePort.println("Warning: Not entering Config Mode");
  }

  // Send the self test command
  _consolePort.print("Initiate Structural Resonance test...");
  regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_EXITEST, verbose);

  // Wait for self test to process
  delay(_dev_prop.delay_structres_test_ms);

  // Check that bit returns 0
  valRead = regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, verbose) & BIT_15;
  if (valRead != 0) {
    _consolePort.println("Warning: EXI_TEST bit not returning to 0");
    return 0xFFFF;
  }
  _consolePort.println("Done.");

  // Read the results in DIAG_STAT2
  valRead = regRead16(CMD_WINDOW0, ADDR_DIAG_STAT2, verbose);
  if ((valRead & VAL_DIAG_STAT2_MASK) != 0) {
    _consolePort.print("Warning: DIAG_STAT2 returned some errors - ");
    _consolePort.println(valRead, HEX);
    return valRead;
  }
  return valRead;
}

/**************************************************************************/
/*!
    @brief  Issues a software reset and waits for the required delay time.
            Then checks NOT_READY bit. This function can be called at
   anytime.

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean EPSON_VIBE_UART::sensorReset(boolean verbose) {
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
            DIAG_STAT1 bit 2 result

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

    @returns    true on no errors, false for any errors
*/
/**************************************************************************/
boolean EPSON_VIBE_UART::sensorFlashTest(boolean verbose) {
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
  // Read the results in DIAG_STAT1
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
                  Return Array Values: 0x47, 0x33, 0x36, 0x34, 0x50, 0x44,
                                       0x43, 0x30, 0x00

    @param [in]  length (minimum at least 9 bytes)
                 Length of the string
*/
/**************************************************************************/
void EPSON_VIBE_UART::sensorGetProdID(char* prodID, const size_t length) {
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
    @brief  Assumes Config Mode and Reads the Firmware Version from the
   Sensor.

    @returns  16-bit Firmware Number
*/
/**************************************************************************/
uint16_t EPSON_VIBE_UART::sensorGetVersion(void) {
  return regRead16(CMD_WINDOW1, ADDR_VERSION);
}

/**************************************************************************/
/*!
    @brief  Assumes Config Mode and Reads the Serial Number from the
   Sensor and terminate with NULL to create a string.

    @param [out]  serialNumber
                  Pointer to string of 8 ASCII bytes of Serial Number
                  Each byte is an ASCII number.

    @param [in]  length (minimum at least 9 bytes)
                 Length of the string
*/
/**************************************************************************/
void EPSON_VIBE_UART::sensorGetSerialNumber(char* serialNumber,
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
/*                           PRIVATE FUNCTIONS */
/*========================================================================*/
/**************************************************************************/
/*!
    @brief  Decodes the BURST_CTRL register and stores
    status in _burst_flag struct and returns output burst length in bytes
        NOTE: SPI and UART interface format bytes differently

    @param [in]  boolean verbose
                 Send register accesses to console for debugging

    @returns    sensor burst length in bytes(excluding header & delimiter
                byte)
*/
/**************************************************************************/
int8_t EPSON_VIBE_UART::_sensorDecodeBurstCtrl(boolean verbose) {
  uint8_t burst_len = 0;

  uint16_t burst_ctrl =
    regRead16(CMD_WINDOW1, ADDR_BURST_CTRL_LO, verbose) & VAL_BURST_CTRL_MASK;

  // burst_ctrl check
  _burst_flag.nd_ea = (burst_ctrl & BIT_15) ? 1 : 0;
  _burst_flag.tempc = (burst_ctrl & BIT_14) ? 1 : 0;
  _burst_flag.sensx = (burst_ctrl & BIT_10) ? 1 : 0;
  _burst_flag.sensy = (burst_ctrl & BIT_9) ? 1 : 0;
  _burst_flag.sensz = (burst_ctrl & BIT_8) ? 1 : 0;
  _burst_flag.count = (burst_ctrl & BIT_1) ? 1 : 0;
  _burst_flag.chksm = (burst_ctrl & BIT_0) ? 1 : 0;

  // Calc burst_len
  if (_burst_flag.nd_ea) burst_len += 2;
  if (_burst_flag.tempc) burst_len += 2;
  if (_burst_flag.sensx) burst_len += 3;
  if (_burst_flag.sensy) burst_len += 3;
  if (_burst_flag.sensz) burst_len += 3;
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
boolean EPSON_VIBE_UART::_sensorReadBurst16(uint16_t* arr,
                                            const size_t length) {
  if (_read_burst8_len < 1) {
    _consolePort.println("Error: Bypassing because burst length is invalid");
    _consolePort.println(
      "Has sensor been initialized with sensorInitOptions()?");
    return false;
  }

  if ((uint8_t)length < _read_burst8_len) {
    _consolePort.print("Error: Bypassing because the array size is smaller");
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
        NOTE: SPI and UART interface format bytes differently

    @param [out] unscaledField
                 Pointer to struct VibeUnscaledData

    @param [in]  readBuf
                 Pointer to 16-bit Array

    @param [in]  length
                 Size of 16-bit Array in bytes
*/
/**************************************************************************/
void EPSON_VIBE_UART::_sensorBuf2Field(struct VibeUnscaledData* unscaledField,
                                       const uint16_t* readBuf,
                                       const size_t length) {
  if ((uint8_t)length < _read_burst8_len) {
    _consolePort.print("Error: Bypassing because the array is smaller");
    _consolePort.print(" than the read burst length of: ");
    _consolePort.print(_read_burst8_len, DEC);
    return;
  }

  // Convert 16 array -> 8-bit array for easier parsing
  int readBuf_size = (_read_burst8_len + 1) >> 1;
  uint8_t byteBuf[128] = {};
  int j = 0;
  for (int i = 0; i < readBuf_size; i++) {
    byteBuf[j] = (readBuf[i] & 0xFF00) >> 8;
    byteBuf[j + 1] = readBuf[i] & 0x00FF;
    j = j + 2;
  }

  int i = 0;
  if (_burst_flag.nd_ea) {
    unscaledField->nd_flags = byteBuf[i] << 8 | byteBuf[i + 1];
    i = i + 2;
  }

  if (_burst_flag.tempc) {
    if (_temp_sel == 0) {
      // TEMP2
      unscaledField->tempC = byteBuf[i] << 8;
      unscaledField->errEXI = (byteBuf[i + 1] & 0xE0) >> 5;
      unscaledField->errALARM = (byteBuf[i + 1] & 0x1C) >> 2;
      unscaledField->two_bit_count = byteBuf[i + 1] & 0x03;
    } else {
      // TEMP1
      unscaledField->tempC = byteBuf[i] << 8 | byteBuf[i + 1];
    }
    i = i + 2;
  }
  if (_burst_flag.sensx) {
    uint32_t sensData =
      (((uint32_t)byteBuf[i] << 24) | ((uint32_t)byteBuf[i + 1] << 16) |
       (uint32_t)byteBuf[i + 2]);
    unscaledField->sensXYZ[0] = (int32_t)sensData >> 8;
    i = i + 3;
  }

  if (_burst_flag.sensy) {
    uint32_t sensData =
      (((uint32_t)byteBuf[i] << 24) | ((uint32_t)byteBuf[i + 1] << 16) |
       (uint32_t)byteBuf[i + 2]);
    unscaledField->sensXYZ[1] = (int32_t)sensData >> 8;
    i = i + 3;
  }

  if (_burst_flag.sensz) {
    uint32_t sensData =
      (((uint32_t)byteBuf[i] << 24) | ((uint32_t)byteBuf[i + 1] << 16) |
       (uint32_t)byteBuf[i + 2]);
    unscaledField->sensXYZ[2] = (int32_t)sensData >> 8;
    i = i + 3;
  }

  if (_burst_flag.count) {
    unscaledField->counter = byteBuf[i] << 8 | byteBuf[i + 1];
    i = i + 2;
  }

  if (_burst_flag.chksm) {
    unscaledField->chksm = byteBuf[i] << 8 | byteBuf[i + 1];
  }
}

/**************************************************************************/
/*!
    @brief  Convert unscaled sensor data to scaled sensor data

    @param [out] scaledField
                 Pointer to struct VibeScaledData
    @param [in]  unscaledField
                 Pointer to struct VibeUnscaledData
*/
/**************************************************************************/
void EPSON_VIBE_UART::_sensorField2Scaled(
  struct VibeScaledData* scaledField,
  const struct VibeUnscaledData* unscaledField) {
  if (_burst_flag.sensx) {
    scaledField->sensXYZ[0] = unscaledField->sensXYZ[0] * _dev_prop.vel_sf_m_s;
  }

  if (_burst_flag.sensy) {
    scaledField->sensXYZ[1] = unscaledField->sensXYZ[1] * _dev_prop.vel_sf_m_s;
  }

  if (_burst_flag.sensz) {
    scaledField->sensXYZ[2] = unscaledField->sensXYZ[2] * _dev_prop.vel_sf_m_s;
  }

  if (_burst_flag.tempc) {
    scaledField->tempC =
      (float(unscaledField->tempC + _dev_prop.tempc_25c_offset) *
       _dev_prop.tempc_sf_degc) +
      34.987;
    if (_temp_sel == 0) {
      scaledField->errEXI = unscaledField->errEXI;
      scaledField->errALARM = unscaledField->errALARM;
      scaledField->two_bit_count = unscaledField->two_bit_count;
    }
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
