/**************************************************************************/
/*!
    @file     uart_epson_common.h

    Header file for Epson IMU/Accel Driver for Arduino UART

    @section  HISTORY

    v1.0 - First release
    v1.1 - Refactoring
    v1.2 - Added support to detect corrupted sensor packets
    v1.3 - Minor code cleanup

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
#ifndef UART_EPSON_COMMON_H_
#define UART_EPSON_COMMON_H_

#if ARDUINO >= 100
 #include <Arduino.h>   //include for Arduino
#else
 #include <WProgram.h>
#endif

#define SerialConsole Serial
#define SerialEpson Serial1

const uint8_t DELIMITER =           0x0D;          // EOL marker

// Epson specific timing delays
#define EPSON_STALL_DELAY           25             // Microseconds, minimum delay to wait between commands
#define BURST_STALL                 70             // Microseconds, minumum delay after initial BURST read command
#define EPSON_SWRESET_DELAY         800000         // Microseconds, minimum delay after software reset
#define EPSON_POWER_ON_DELAY        800000         // Microseconds, max delay for poweron startup completion
#define EPSON_SELF_TEST_DELAY       150000         // Microseconds, max delay for self-test completion (80ms is common for most IMUs, 150ms is for G370)
#define EPSON_FLASH_TEST_DELAY      5000           // Microseconds, max delay for flash-test completion
#define EPSON_FILTER_DELAY          1000           // Microseconds, max delay for filter setup completion
#define EPSON_NRESET_LOW_DELAY      100000         // Microseconds, min delay for nRESET assertion
const uint32_t EPSON_DRDYCHECK =    1000;          // Define max # of times to check

#define EpsonStall()                delayMicroseconds(EPSON_STALL_DELAY)
#define EpsonBurstStall()           delayMicroseconds(BURST_STALL)
#define EpsonPowerOnDelay()         delayMicroseconds(EPSON_POWER_ON_DELAY)
#define EpsonSelfTestDelay()        delayMicroseconds(EPSON_SELF_TEST_DELAY)
#define EpsonFlashTestDelay()       delayMicroseconds(EPSON_FLASH_TEST_DELAY)
#define EpsonSwResetDelay()         delayMicroseconds(EPSON_SWRESET_DELAY)
#define EpsonFilterDelay()          delayMicroseconds(EPSON_FILTER_DELAY)
#define EpsonResetAssertDelay()     delayMicroseconds(EPSON_NRESET_LOW_DELAY)

//------------------------
//UART_EPSON_COM driver class
//------------------------
class UART_EPSON_COM {
 public:
  UART_EPSON_COM(int8_t nrst, int8_t drdy);  // UART non-AUTO Mode (with DRDY)

  //user methods
  boolean  begin(void);
  void     regWrite8(uint8_t winid, uint8_t addr, uint8_t value, boolean verbose=false);
  uint16_t regRead16(uint8_t winid, uint8_t addr, boolean verbose=false);
  boolean  sensorStart(void);
  boolean  sensorStop(void);
  uint16_t sensorSelfTest(void);
  boolean  sensorHWReset(void);
  boolean  sensorReset(void);
  boolean  sensorFlashTest(void);
  boolean  sensorReadBurst(uint16_t* arr, uint8_t len);

 protected:
  int8_t   getDRDY(void){return _drdy;};
  void     setUARTAuto(boolean val){_uart_auto=val;};
  void     getProdID(char* prodID);        // return 9byte null terminated char array pointer
  uint16_t getVersion(void);            // return 16-bit version
  void     getSerialNumber(char* serialNumber);  // return 9byte null terminated char array pointer
  uint8_t  _burstCnt_calculated;

 private:
  boolean  waitDataReady(boolean polarity, uint32_t retryMaxCount);
  void     gotoSamplingMode(void);
  void     gotoConfigMode(void);
  boolean  readN(uint16_t* arrayOut, uint8_t addr, uint8_t readLength, uint32_t retryMaxCount);
  void     findDelimiter(void);        // search for next DELIMITER in sensor buffer

  boolean  _initialised;  // Device not initialized yet
  boolean  _uart_auto;    // UART_AUTO is disabled
  int8_t   _nrst, _drdy;  // pin for drdy if used
};

#include "epson_devices.h"

#endif //< UART_EPSON_COMMON_H_
