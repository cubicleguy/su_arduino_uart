/**************************************************************************/
/*!
    @file     epson_uart_common.h

    Header file for Epson UART class

    @section  HISTORY

    v1.0 - First release restructure

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

#include <stdbool.h>
#include <stdint.h>

#include <Arduino.h>
#include <HardwareSerial.h>

namespace EPSON_UART {
constexpr uint8_t DELIMITER = 0x0D;  // EOL marker

constexpr uint16_t ID_RETVAL = 0x5345;
constexpr uint16_t NOT_READY_BIT = 0x0400;
constexpr uint8_t WINDOW0 = 0x00;
constexpr uint8_t WINDOW1 = 0x01;

// Register Addresses
constexpr uint8_t ID = 0x4C;
constexpr uint8_t WIN_CTRL = 0x7E;
constexpr uint8_t GLOB_CMD_LO = 0x0A;

// Microseconds, minimum delay to wait between commands
constexpr unsigned int EPSON_STALL_DELAY = 25;

// Microseconds, minimum delay after initial BURST read command
constexpr unsigned int EPSON_BURST_STALL = 70;

// Milliseconds, max delay for power-on startup completion
constexpr unsigned long EPSON_POWER_ON_DELAY = 800;

// Milliseconds, min delay for nRESET assertion
constexpr unsigned long EPSON_NRESET_LOW_DELAY = 100;

// Microseconds, minimum delay between polling for DRDY pin
constexpr unsigned int EPSON_DRDYCHECK_DELAY = 5;

// Max # of to check for register read response
constexpr uint32_t EPSON_READ_RESPONSECHECK = 500;
}  // namespace EPSON_UART

//------------------------
// UART_EPSON_COM driver class
//------------------------
class UART_EPSON_COM {
 public:
  UART_EPSON_COM(HardwareSerial& uartPort, uint32_t baudRate, int8_t nrst,
                 int8_t drdy, Stream& consolePort);

  boolean begin(void);
  void regWrite8(uint8_t winid, uint8_t addr, uint8_t value,
                 boolean verbose = false);
  uint16_t regRead16(uint8_t winid, uint8_t addr, boolean verbose = false);
  boolean toggleReset(void);
  int8_t getDRDY(void) { return _drdy; };
  boolean readN(uint16_t* arrayOut, uint8_t addr, uint8_t readLength,
                uint32_t retryMaxCount);
  boolean waitDataReady(boolean polarity, uint32_t retryMaxCount);
  void clearRxBuffer(void);

 private:
  void _findDelimiter(void);  // search for next DELIMITER in sensor buffer
  HardwareSerial& _uartPort;
  uint32_t _baudRate = 460800;
  int8_t _nrst = -1;
  int8_t _drdy = -1;
  Stream& _consolePort;
  boolean _initialised = false;
  uint8_t _retByteN[128] =
    {};  // Burst read byte length should never exceed 128 bytes
  uint8_t _retByte[4] = {};  // Stores the return value during regRead16()
  uint8_t _xmtVal[3] = {};   // Stores the CMD during regWrite8()
};
