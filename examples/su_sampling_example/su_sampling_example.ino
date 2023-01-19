/***********************************************************************
  su_sampling_example.ino

  This Arduino sketch example application initializes the SU,
  and displays sensor readings.
  For V340, the sensor output is always 16bit.
  For all other SU models, the gyro & accel output is configurable
  16-bit or 32-bit.
  For G365/G330/G366, the sensor output also supports attitude output: Euler
  or Quaternion.
  This is tested for support on Teensy 3.6 or Arduino DUE.
  However, this software is expected to work on any 3.3V Arduino platform.

  For more information on the Epson Sensing Inits (SU) visit:

  --> http://global.epson.com/products_and_drivers/sensing_system/

  This test application assumes that the SU UART signals are connected
  to the Teensy 3.6 header (hardware). The pins are mapped as shown below.

  Circuit Pinmapping:
         Arduino DUE    Teensy 3.6 M-Gseries  M-Vseries
  --------------------------------------------------------------
  DRDY:  pin 6          pin 3      pin 13     pin 14         (* DRDY is optional)
  RX1:   pin-18         pin 0      pin 7      pin 10
  TX1:   pin-19         pin 1      pin 9      pin 12
  RST#:  pin-7          pin 2      pin 16     pin 18

************************************************************************

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

***********************************************************************/

// 1. Set the device model type in the epson_devices.h file
// 2. Set the user specific settings in the xxxx_epson_user_def.h file

// Epson UART Device Arduino Library Definitions
#include <uart_epson_common.h>

// Assign DataReady is optional, If DRDY not used set to -1 , Otherwise set to assigned pin
// Assign RESET to pin on Arduino
const int8_t EPSON_DRDY = -1;
//const int8_t EPSON_DRDY = 3;
const int8_t EPSON_RESET = 2; //Teensy pin 2

EPSON_DEV su(EPSON_RESET, EPSON_DRDY);

/*------------------------------------------------------------------------------
 * setup()
 *
 *------------------------------------------------------------------------------*/
void setup() {
  SerialConsole.begin(250000);   // Setup Serial communications for 250000bps
  while (!SerialConsole) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }

  if (!su.begin()){
    SerialConsole.println("Error. Can not communicate properly with " EPSON_UNIT_TYPE);
    while (1);
  }

  // Toggle SU Reset Line & wait for reset delay time
  su.sensorHWReset();

  // Initialize SU with settings
  if (!su.sensorInit(CMD_RATEX, CMD_FILTERX)) {
    SerialConsole.println("Error. Can not initialize " EPSON_UNIT_TYPE);
    while (1);
  }
  SerialConsole.println(EPSON_UNIT_TYPE " Init Done");
}

/*------------------------------------------------------------------------------
 * loop()
 *
 *------------------------------------------------------------------------------*/
void loop() {

  uint16_t readData[64];  // Set array large enough to store sensor read burst
  uint32_t sampleCount = 0;
  const uint32_t MAXSAMPLE = 100;

  su.sensorConfigDump();
  su.sensorScaleFactorsPrint();

  // Go to SAMPLING mode
  if (!su.sensorStart()) {
    SerialConsole.println("Error. Sensor not entering Sampling mode");
    while (1);
  }

  su.sensorHeaderPrint();

  while (1) {
    // Burst read one sensor sample set
    if (su.sensorReadBurst(readData, 64)) {
      // Output formatted to console
      su.sensorDataPrint(readData, sampleCount);
    }
    else {
      SerialConsole.println("#Corrupted data detected.");
    }
    sampleCount++;

    // If sampleCount reaches limit stop application
    if (sampleCount >= MAXSAMPLE) {
        if (!su.sensorStop()) {
          SerialConsole.println("Error. Sensor not entering Sampling mode");
          while (1);
        }
      SerialConsole.println("Done.");
      su.registerDump();
      while (1); // Stops further execution
    }
  }
}

