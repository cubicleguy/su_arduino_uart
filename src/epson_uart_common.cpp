/**************************************************************************/
/*!
    @file     epson_uart_common.cpp

    Epson UART class

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

#include "epson_uart_common.h"

using namespace EPSON_UART;
/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
UART_EPSON_COM::UART_EPSON_COM(HardwareSerial& uartPort, uint32_t baudRate,
                               int8_t nrst, int8_t drdy,
                               Stream& consolePort)
    :  // initializer list
      _uartPort(uartPort),
      _baudRate(baudRate),
      _nrst(nrst),
      _drdy(drdy),
      _consolePort(consolePort) {};

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Initializes UART and reads the Product ID register
            for validation (call this function before doing
            anything else)

    @returns   False if DataReady timeout or Product ID is unknown,
               otherwise True
*/
/**************************************************************************/
boolean UART_EPSON_COM::begin(void) {
  boolean ok = true;

  _consolePort.print("Platform: ");
#ifdef __SAM3X8E__
  _consolePort.println("Arduino DUE");
#elif defined(__MK66FX1M0__)  // Teensy 3.6
  _consolePort.println("Teensy 3.6");
#else
  _consolePort.println("Normal Arduino");
#endif
  _consolePort.print("Open UART Port for Epson device: ");
  _consolePort.print(_baudRate, DEC);
  _consolePort.println(" baud");
  _uartPort.begin(_baudRate);

  while (!_uartPort) {
    _consolePort.println("Waiting for Epson UART Port to be ready");
  }
  _consolePort.println("Epson device opened");
  // Configure nRESET Input pin if defined
  if (_nrst != -1) {
    // Set nRESET pin for output HIGH
    _consolePort.print("nRST on ");
    _consolePort.println(_nrst, DEC);
    pinMode(_nrst, OUTPUT);
    digitalWrite(_nrst, HIGH);
    // Issue a Hardware Reset to get device to known state
    _consolePort.println("HW Reset asserted");
    if (!toggleReset()) {
      _consolePort.println("Warning: NOT_READY is HIGH which should be LOW");
    };
  } else {
    _consolePort.println("nRST not used");
  }

  // Configure DataReady Input pin if defined
  if (_drdy != -1) {
    _consolePort.print("DRDY on pin ");
    _consolePort.println(_drdy, DEC);
    pinMode(_drdy, INPUT);

    // Sanity Check for DRDY = LOW  (This assumes DRDY is configured active
    // HIGH) If the device is configured for DRDY active LOW, then this
    // validation check will fail
    _consolePort.println("Check DRDY");
    if (getDRDY() == 1) {
      _consolePort.println(
        "Error: DRDY = HIGH, DataReady pin should be LOW."
        "Check your hardware connection.");
      ok = false;
    }
  } else {
    // if DRDY is not used bypass the DRDY pin check
    _consolePort.println("DRDY not used");
  }

  // Check ID register for IMU/Accel
  _consolePort.print("Checking device is present...");
  uint16_t retVal = regRead16(WINDOW0, ID);
  if (retVal == ID_RETVAL) {
    _consolePort.println("device responded to ID read");
  } else {
    ok = false;
    _consolePort.print("ERROR: Incorrect device response - 0x");
    _consolePort.println(retVal, HEX);
  }
  _initialised = true;
  return ok;
}

/**************************************************************************/
/*!
    @brief  Writes an 8-bit value at the specific register address

    @param [in] winid
                The 8-bit window ID
    @param [in] addr
                The 8-bit register address
    @param [in] value
                The 8-bit value to write at address
    @param [in] verbose
                boolean to enabled debug output of register access
*/
/**************************************************************************/
void UART_EPSON_COM::regWrite8(uint8_t winid, uint8_t addr, uint8_t value,
                               boolean verbose) {
  // Send the window command & win ID
  _xmtVal[0] = WIN_CTRL | 0x80;  // msb is set 1b for register write
  _xmtVal[1] = winid;
  _xmtVal[2] = DELIMITER;
  _uartPort.write(_xmtVal, 3);
  // Delay between commands
  delayMicroseconds(EPSON_STALL_DELAY);

  // Send the write register command & address
  _xmtVal[0] = addr | 0x80;  // msb is set 1b for register write
  _xmtVal[1] = value;
  _xmtVal[2] = DELIMITER;
  _uartPort.write(_xmtVal, 3);
  // Delay between commands
  delayMicroseconds(EPSON_STALL_DELAY);

  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("REG[0x");
    _consolePort.print((addr & 0x7F), HEX);
    _consolePort.print(" W(");
    _consolePort.print(winid, DEC);
    _consolePort.print(")");
    _consolePort.print("] < 0x");
    _consolePort.println(value, HEX);
  }
}

/**************************************************************************/
/*!
    @brief  Reads an 16 bit value from the specified register address

    @param [in] winid
                The 8-bit window ID
    @param [in] addr
                The 8-bit register address (must be even, 16-bit aligned)
    @param [in] verbose
                boolean to enable debug output of register access
    @returns    The 16-bit value retrieved from register
*/
/**************************************************************************/
uint16_t UART_EPSON_COM::regRead16(uint8_t winid, uint8_t addr,
                                   boolean verbose) {
  static uint16_t readData;
  uint32_t retryCycles = EPSON_READ_RESPONSECHECK;

  // Send the window command & win ID
  _xmtVal[0] = WIN_CTRL | 0x80;  // msb is set 1b for register write
  _xmtVal[1] = winid;
  _xmtVal[2] = DELIMITER;
  _uartPort.write(_xmtVal, 3);
  // Delay between commands
  delayMicroseconds(EPSON_STALL_DELAY);

  // Send the read register command & address
  _xmtVal[0] = addr & 0x7f;  // msb is set 0b for register read
  _xmtVal[1] = 0x00;         // Dummy byte
  _xmtVal[2] = DELIMITER;
  _uartPort.write(_xmtVal, 3);
  // Delay between commands
  delayMicroseconds(EPSON_STALL_DELAY);

  do {
    retryCycles--;
    delayMicroseconds(10);
    if (retryCycles < 1) {
      _consolePort.print(
        "ERROR: Timeout waiting for register read response. Aborting...");
      return 0x0000;
    }
  } while (_uartPort.available() < 4);

  for (int i = 0; i < 4; i++) {
    _retByte[i] = _uartPort.read();
  }

  // Delay between 16 bit transfers
  delayMicroseconds(EPSON_STALL_DELAY);

  // Check first byte in the response should be the register address
  // Check last byte in the response should be DELIMITER
  // Output error msg if not the case
  if ((_retByte[0] != (addr & 0x7f)) || (_retByte[3] != DELIMITER)) {
    _consolePort.print("Unexpected register read response:");
    for (int i = 0; i < 4; i++) {
      _consolePort.print(_retByte[i], HEX);
      _consolePort.print(", ");
    }
    _consolePort.println();
  }

  readData = (_retByte[1] << 8) | (_retByte[2]);

  // If debug output selected, print information about the transfer
  if (verbose) {
    _consolePort.print("REG[0x");
    _consolePort.print((addr & 0x7F), HEX);
    _consolePort.print(" W(");
    _consolePort.print(winid, DEC);
    _consolePort.print(")");
    _consolePort.print("] > 0x");
    _consolePort.println(readData, HEX);
  }
  // Return the data
  return readData;
}

/**************************************************************************/
/*!
    @brief  Issues a hardware reset and waits for the required delay time.
            Then checks NOT_READY bit. This function can be called at anytime.

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean UART_EPSON_COM::toggleReset(void) {
  if (_nrst == -1) {
    // bypass because nRESET pin not used
    _consolePort.println("nRESET not enabled, bypassing Reset toggling");
    return true;
  }
  _consolePort.println("Asserting HW Reset");
  // Asserts the nRESET pin LOW
  digitalWrite(_nrst, LOW);
  delay(EPSON_NRESET_LOW_DELAY);

  // Asserts the nRESET pin HIGH
  digitalWrite(_nrst, HIGH);
  // Wait for the sensor re-initialization
  delay(EPSON_POWER_ON_DELAY);
  // Check NOT_READY bit = 0
  if (regRead16(WINDOW1, GLOB_CMD_LO) & NOT_READY_BIT) {
    _consolePort.println("Warning: NOT_READY bit is HIGH");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Poll waiting for DataReady pin to go to specified state.
            Will continue to retry until the specified retry count is reached.
            There is a delay between polling reads.

    @param [in]  polarity
                 false for LOW, true for HIGH
    @param [in]  retryMaxCount
                 Maximum retries checking DRDY before a timeout

    @returns    True on success, False on timeout

*/
/**************************************************************************/
boolean UART_EPSON_COM::waitDataReady(boolean polarity,
                                      uint32_t retryMaxCount) {
  // Loop continuously until DataReady or timeout
  for (uint32_t retryCount = 0; retryCount < retryMaxCount; retryCount++) {
    if (digitalRead(_drdy) == polarity) {
      return true;
    }
    delayMicroseconds(EPSON_DRDYCHECK_DELAY);
  }
  _consolePort.println("Warning: Retry exceeded waiting for DRDY");
  return false;
}

/**************************************************************************/
/*!
    @brief  Reads a specified number of sequential 16-bit registers
            starting at the specified address.
            If DRDY is used then send burst read command, wait for UART
            buffer to fill to 1 burst length, then readout sample data
            Otherwise assume UART_AUTO mode, ignore DRDY, wait for UART
            buffer to fill to 1 burst length, then readout sample data

    @param [out]  arrayOut (max 64 elements)
                  Array of return 16-bit values

    @param [in]  addr
                 When DRDY is used, this is the burst command sent to start
                 the transfer
    @param [in]  readLength (in bytes, must be less than 128)
                 Specify the length of the burst read transfer in bytes
                 (excluding the header & delimiter bytes)
    @param [in]  retryMaxCount
                 Maximum retries waiting for sensor bytes before error

    @returns  true if successful, false if errors are detected
*/
/**************************************************************************/
boolean UART_EPSON_COM::readN(uint16_t* arrayOut, uint8_t addr,
                              uint8_t readLength, uint32_t retryMaxCount) {
  // byte length = sensor burst bytes + header byte + delimiter byte
  uint8_t readByteLength = readLength + 2;
  uint8_t j = 0;
  uint32_t retryCount = 0;

  // If DRDY is used then assume UART manual mode, and send a burst command
  // every sample
  if (_drdy != -1) {
    // Check DRDY
    waitDataReady(true, retryMaxCount);

    // Setup read burst command
    _xmtVal[0] = addr;  // This should be the BURST COMMAND (0x80)
    _xmtVal[1] = 0x00;
    _xmtVal[2] = DELIMITER;
    _uartPort.write(_xmtVal, 3);

    // delay after 1st command
    delayMicroseconds(EPSON_BURST_STALL);
  }

  // Wait for buffer to fill to 1 complete burst or return false on exceeding
  // max retries
  do {
    retryCount++;
    delayMicroseconds(5);
    if (retryCount > retryMaxCount) {
      _consolePort.print(
        "ERROR: Timeout waiting for sensor read burst data. Aborting...");
      return false;
    }
  } while (_uartPort.available() < readByteLength);

  // Read UART and store into byte array
  int bytesRead = _uartPort.readBytes(_retByteN, readByteLength);

  // Return false if # of bytes is incorrect
  if (bytesRead != readByteLength) {
    _consolePort.print("Incorrect size of bytes read:");
    _consolePort.println(bytesRead, DEC);
    _findDelimiter();  // read sensor Rx buffer until finds DELIMITER
    return false;
  }
  // Return false if first byte & last byte is incorrect
  if ((_retByteN[0] != (addr)) ||
      (_retByteN[readByteLength - 1] != DELIMITER)) {
    _consolePort.print("Start or Delimiter bytes incorrect:");
    for (int i = 0; i < readByteLength; i++) {
      _consolePort.print(_retByteN[i], HEX);
      _consolePort.print(", ");
    }
    _consolePort.println();
    _findDelimiter();  // read sensor Rx buffer until finds DELIMITER
    return false;
  }

  // Parse UART read byte array to 16-bit output array
  for (int i = 1; i < (readByteLength - 1); i = i + 2) {
    arrayOut[j] = _retByteN[i] << 8 | _retByteN[i + 1];
    j++;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Blindly reads a byte until available bytes are zero
*/
/**************************************************************************/
void UART_EPSON_COM::clearRxBuffer(void) {
  // Clear any bytes in the UART Rx buffer
  while (_uartPort.available() > 0) {
    _uartPort.read();
  }
}

/*========================================================================*/
/*                           PRIVATE FUNCTIONS                            */
/*========================================================================*/
/**************************************************************************/
/*!
    @brief  Blindly reads a byte at a time until it finds the DELIMTER byte
            in the sensor receive buffer
*/
/**************************************************************************/
void UART_EPSON_COM::_findDelimiter(void) {
  uint8_t _byte = 0x00;

  do {
    do {
      delayMicroseconds(5);
    } while (_uartPort.available() < 1);
    _byte = _uartPort.read();
  } while (_byte != DELIMITER);
}
