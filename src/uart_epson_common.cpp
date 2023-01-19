/**************************************************************************/
/*!
    @file     uart_epson_common.cpp

    Epson IMU/Accel Common Driver for Arduino UART

    @section  HISTORY

    v1.0 - First release
    v1.1 - Refactoring
    v1.2 - Added support to detect corrupted senosr packets
    v1.3 - Added constructor initialization list, remove debug code, cleanup

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

#include <math.h>
#include <string.h>
#include "uart_epson_common.h"

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
// constructor with DRDY
UART_EPSON_COM::UART_EPSON_COM(int8_t nrst, int8_t drdy):
    // initializer list
    _burstCnt_calculated(0),
    _initialised(false),
    _uart_auto(false) {
        _nrst = nrst;                 // Store nRESET pin
        _drdy = drdy;                 // Store DRDY pin
}


/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Initialize and reads the Product ID register
            for validation (call this function before doing
            anything else)

    @returns   False if DataReady timeout or Product ID is unknown,
               otherwise True
*/
/**************************************************************************/
boolean UART_EPSON_COM::begin(void){

    boolean ok;

    // Set nRESET pin for output HIGH
    SerialConsole.print("nRST on pin "); SerialConsole.println(_nrst, DEC);
    pinMode(_nrst, OUTPUT);
    digitalWrite(_nrst, HIGH);

    SerialConsole.print("Platform: ");
#ifdef __SAM3X8E__
  SerialConsole.println("Arduino DUE");
#elif defined(__MK66FX1M0__)    // Teensy 3.6
  SerialConsole.println("Teensy 3.6");
    SerialEpson.setTX(1);
    SerialEpson.setRX(0);
#else
  SerialConsole.println("Normal Arduino");
#endif
    //SerialEpson.setTimeout(1000);
    SerialConsole.println("Open UART Port for Epson SU");
    SerialEpson.begin(460800);

    while (!SerialEpson) {
      SerialConsole.println("Waiting for SU Serial Port to be ready");
    }

    // Issue a Hardware Reset to get device to known state
    sensorHWReset();
    SerialConsole.println("HW Reset Epson SU");

    // Configure DataReady Input pin if defined
    if (_drdy != -1) {
        SerialConsole.print("DRDY on pin "); SerialConsole.println(_drdy, DEC);
        pinMode(_drdy, INPUT);

        // Sanity Check for DRDY = LOW  (This assumes DRDY is configured active HIGH)
        // If the device is configured for DRDY active LOW, then this validation check will fail
        SerialConsole.println("Check DRDY");
        ok = waitDataReady(false, EPSON_DRDYCHECK);

        if (ok == false) {
            SerialConsole.println("Error: Timeout waiting for DRDY = LOW, DataReady pin appears stuck HIGH");
            SerialConsole.println("Attempting hardware reset...");
            sensorHWReset();

            // Sanity Check for DRDY = LOW  (This assumes DRDY is configured active HIGH)
            // If the device is configured for DRDY active LOW, then this validation check will fail
            ok = waitDataReady(false, EPSON_DRDYCHECK);

            if (ok == false) {
                SerialConsole.println("Error: Could not reset sensor, DataReady pin appears stuck HIGH");
            }
        }
    }
    else {
        // if DRDY is not used then just pass the DRDY pin check
        ok = true;
        SerialConsole.println("DRDY not used");
    }

    if (ok == true) {

        char modelNameReturned[9];
        getProdID(modelNameReturned);

        int result = strcmp(modelNameReturned, EPSON_MODEL_STR);
        if (result == 0) {
            SerialConsole.println(EPSON_MODEL_STR " Detected");
            _initialised = true;
        }
        else {
            char unexpstr[128];
            sprintf(unexpstr, EPSON_UNIT_TYPE " Device Error - Expected: " EPSON_MODEL_STR ", Detected : %s", modelNameReturned);
            SerialConsole.println(unexpstr);
        }
    }
    //return the current initialized state
    return _initialised;
}


/**************************************************************************/
/*!
    @brief  Writes an 8-bit value at the specific register address

    @param [in] winid
                The 8-bit window ID. Ignored for V340
    @param [in] addr
                The 8-bit register address
    @param [in] value
                The 8-bit value to write at address
    @param [in] verbose
                boolean to enabled debug output of register access
*/
/**************************************************************************/
void UART_EPSON_COM::regWrite8(uint8_t winid, uint8_t addr, uint8_t value, boolean verbose) {

    uint8_t xmtVal[3];

#ifndef EPSON_V340
    // Send the window command & win ID
    xmtVal[0] = ADDR_WIN_CTRL|0x80;  // msb is set 1b for register write
    xmtVal[1] = winid;
    xmtVal[2] = DELIMITER;
    SerialEpson.write(xmtVal, 3);
    // Delay between commands
    EpsonStall();
#endif

    // Send the write register command & address
    xmtVal[0] = addr|0x80;  // msb is set 1b for register write
    xmtVal[1] = value;
    xmtVal[2] = DELIMITER;
    SerialEpson.write(xmtVal, 3);
    // Delay between commands
    EpsonStall();

    // If debug output selected, print information about the transfer
    if (verbose) {
        SerialConsole.print("REG[0x");
        SerialConsole.print((addr&0x7F), HEX);
#ifndef EPSON_V340
        SerialConsole.print(" W(");
        SerialConsole.print(winid, DEC);
        SerialConsole.print(")");
#endif
        SerialConsole.print("] < 0x");
        SerialConsole.println(value, HEX);
    }
}


/**************************************************************************/
/*!
    @brief  Reads an 16 bit value from the specified register address

    @param [in] winid
                The 8-bit window ID. Ignored for V340.
    @param [in] addr
                The 8-bit register address (must be even, 16-bit aligned)
    @param [in] verbose
                boolean to enabled debug output of register access
    @returns    The 16-bit value retrieved from register
*/
/**************************************************************************/
uint16_t UART_EPSON_COM::regRead16(uint8_t winid, uint8_t addr, boolean verbose) {

    uint8_t retByte[4];
    uint8_t xmtVal[3];
    uint16_t readData;

#ifndef EPSON_V340
    // Send the window command & win ID
    xmtVal[0] = ADDR_WIN_CTRL|0x80; // msb is set 1b for register write
    xmtVal[1] = winid;
    xmtVal[2] = DELIMITER;
    SerialEpson.write(xmtVal, 3);
    // Delay between commands
    EpsonStall();
#endif

    // Send the read register command & address
    xmtVal[0] = addr&0x7f; // msb is set 0b for register read
    xmtVal[1] = 0x00;  // Dummy byte
    xmtVal[2] = DELIMITER;
    SerialEpson.write(xmtVal, 3);
    // Delay between commands
    EpsonStall();

    while (SerialEpson.available() < 4) {
    // Wait forever for 4 byte response
    // TO-DO add a timeout check to error on too many retries
    };

    for(int i=0; i<4; i++) {
        retByte[i] = SerialEpson.read();
    }

    // Delay between 16 bit transfers
    EpsonStall();

    // Check first byte in the response should be the register address
    // Check last byte in the response should be DELIMITER
    // Output error msg if not the case
    if ((retByte[0] != (addr&0x7f)) || (retByte[3] != DELIMITER)) {
        SerialConsole.print("Unexpected read response:");
        for(int i=0; i<4; i++) {
            SerialConsole.print(retByte[i], HEX);
            SerialConsole.print(", ");
        }
        SerialConsole.println();
    }

    readData = (retByte[1]<<8) | (retByte[2]);

    // If debug output selected, print information about the transfer
    if (verbose) {
        SerialConsole.print("REG[0x");
        SerialConsole.print((addr&0x7F), HEX);
#ifndef EPSON_V340
        SerialConsole.print(" W(");
        SerialConsole.print(winid, DEC);
        SerialConsole.print(")");
#endif
        SerialConsole.print("] > 0x");
        SerialConsole.println(readData, HEX);
    }
    // Return the data
    return readData;
}

/**************************************************************************/
/*!
    @brief  Goes to SAMPLING Mode

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean UART_EPSON_COM::sensorStart(void) {

    boolean result = false;

    gotoSamplingMode();
    delayMicroseconds(2000);    // TO-DO - If this delay is removed, it
                                // does not reliably enter sampling mode

    if (_uart_auto == false) {
        // If UART_AUTO is disabled
        // Check that MODE_STAT bit returns 0, cycles out after 10,000 tries
        for (int32_t i=0; i<10000; i++) {
            uint16_t valRead = regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO) & VAL_CONFIG_MASK;
            if (valRead == VAL_SAMPLING_MODE){
                result = true;
                break;
            }
        }
    }
    else
        result = true; // If UART_AUTO is enabled, can not read registers while in
                       // in sampling mode
    return result;
}


/**************************************************************************/
/*!
    @brief  Places the sensor into CONFIG Mode

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean UART_EPSON_COM::sensorStop(void) {

    boolean result = false;

    gotoConfigMode();
    delayMicroseconds(1000000);  // Wait atleast 1 second for any pending burst packets to finish sending before entering config mode

    // Clear any bytes in the UART Rx buffer
    while (SerialEpson.available() > 0) {
        SerialEpson.read();
    }

    if (_uart_auto == false) {
        // If UART_AUTO is disabled
        // Check that MODE_STAT bit returns 1, cycles out after 10,000 tries
        for (int32_t i=0; i<10000; i++) {
            uint16_t valRead = regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO) & VAL_CONFIG_MASK;
            if (valRead == VAL_CONFIG_MODE){
                result = true;
                break;
            }
        }
    }
    else
        result = true; // If UART_AUTO is enabled, can not read registers
  return result;
}


/**************************************************************************/
/*!
    @brief  Enters Config Mode to initiate a selftest and returns the DIAG_STAT
            register value

    @returns    DIAG_STAT return value, 0 on success, nonzero for any errors
                and 0xFFFF if SELF_TEST bit is stuck 1b.
*/
/**************************************************************************/
uint16_t UART_EPSON_COM::sensorSelfTest(void) {

    uint16_t valRead;

    if (!sensorStop()) {
        SerialConsole.println("Warning: Not entering Config Mode");
    }

    // Send the self test command
    regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_SELFTEST);

    // Wait for self test to process
    EpsonSelfTestDelay();

    // Check that SELF_TEST bit returns 0
    valRead = regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO) & VAL_SELF_TEST_BIT;
    if ( valRead ){
        valRead = 0xFFFF;   // SELF_TEST bit is stuck 1b
    }
    else {
        // Read the results in DIAG_STAT
        valRead = regRead16(CMD_WINDOW0, ADDR_DIAG_STAT) & VAL_DIAG_STAT_MASK;
    }
    return valRead;
}


/**************************************************************************/
/*!
    @brief  Issues a hardware reset and waits for the required delay time.
            Then checks NOT_READY bit. This function can be called at anytime.

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean UART_EPSON_COM::sensorHWReset(void) {

    boolean result = false;
    // Asserts the nRESET pin LOW
    digitalWrite(_nrst, LOW);
    EpsonResetAssertDelay();

    // Asserts the nRESET pin HIGH
    digitalWrite(_nrst, HIGH);

    // Wait for the sensor re-initialization
    EpsonPowerOnDelay();

    // Check NOT_READY bit = 0
    uint16_t valRead = regRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO) & VAL_NOT_READY;
    if (valRead == 0){
        result = true;
    }

    _burstCnt_calculated = 0;

    return result;
}


/**************************************************************************/
/*!
    @brief  Issues a software reset and waits for the required delay time.
            Then checks NOT_READY bit. This function can be called at anytime.

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean UART_EPSON_COM::sensorReset(void) {

    boolean result = false;

    // Set the RESET bit
    regWrite8(CMD_WINDOW1, ADDR_GLOB_CMD_LO, CMD_SOFTRESET);

    // Wait for the sensor software reset to process
    EpsonSwResetDelay();

    // Check NOT_READY bit = 0
    uint16_t valRead = regRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO) & VAL_NOT_READY;
    if (valRead == 0){
        result = true;
    }

    _burstCnt_calculated = 0;

    return result;
}


/**************************************************************************/
/*!
    @brief  Enters Config Mode to initiate a Flash test and returns the
            DIAG_STAT bit 2 result
            NOTE: V340 does not contain internal flash, therefore flash test is
                  not supported for V340

    @returns    FLASH Test return value, true on success, false for any errors
*/
/**************************************************************************/
boolean UART_EPSON_COM::sensorFlashTest(void) {

    uint16_t valRead;

    if (!sensorStop()) {
        SerialConsole.println("Warning: Not entering Config Mode");
    }

    // Send the self test command
    regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_FLASHTEST);

    // Wait for self test to process
    EpsonFlashTestDelay();

    // Check that Flash_TEST bit returns 0
    valRead = regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO) & VAL_FLASH_STATUS_BIT;
    if (valRead & VAL_FLASH_STATUS_BIT){
        valRead = 0xFFFF;   // SELF_TEST bit is stuck 1b
    }
    else {
        // Read the results in DIAG_STAT
        valRead = regRead16(CMD_WINDOW0, ADDR_DIAG_STAT) & VAL_DIAG_FLASH_ERR;
    }
    return (valRead == VAL_DIAG_FLASH_ERR) ? false : true;
}


/**************************************************************************/
/*!
    @brief  Assumes device is in Sampling mode
            If DRDY is used, then checks DRDY is active
            Issues a burst read command and generates N x 16-bit read cycles
            to read back one sensor sample set

    @param [out] arr pointer to 16-bit array (stores sensor values)

    @param [in]  len N length of array (len is overridden if _burstCnt_calculated is nonzero)

    @returns    true if successful or false if errors detected

*/
/**************************************************************************/
boolean UART_EPSON_COM::sensorReadBurst(uint16_t* arr, uint8_t len) {

    const uint32_t retries = 10000;
    
    // if burst count flag is nonzero, overwrite len parameter
    if (_burstCnt_calculated)
        len = _burstCnt_calculated>>1;

    // if DRDY is used then wait for DRDY = HIGH
    if (_drdy != -1)
        while (!waitDataReady(true, 100000));

    // Read burst sensor data
    boolean retval = readN(arr, CMD_BURST, len, retries);

    // if DRDY is used then wait for DRDY = LOW
    if (_drdy != -1)
        while (!waitDataReady(false, 1000));

    return retval;
}



/*========================================================================*/
/*                           PRIVATE FUNCTIONS                             */
/*========================================================================*/


/**************************************************************************/
/*!
    @brief  Poll waiting for DataReady pin to go to specified state.
            Will continue to retry until the specified retry count is reached.
            There is a 100usec delay between polling reads.

    @param [in]  polarity
                 false for LOW, true for HIGH
    @param [in]  retryMaxCount
                 Maximum retries checking DRDY before a timeout

    @returns    True on success, False on timeout

*/
/**************************************************************************/
boolean UART_EPSON_COM::waitDataReady(boolean polarity, uint32_t retryMaxCount) {

    uint32_t retryCount = 0;

    // Loop continuously to check the status of DataReady until Low or timeout
    do {
        retryCount++;
        delayMicroseconds(10);     // 10 usec
    } while ((digitalRead(_drdy)!=polarity) & (retryCount < retryMaxCount));

    // return true on success, or fail for a timeout
    if (retryCount < retryMaxCount)
        return true; // success
    else
        return false; // fail
}


/**************************************************************************/
/*!
    @brief  Places the sensor into Sampling Mode.
            The sensor must be correctly configured before calling this function.
            This should be only called from Config Mode.
*/
/**************************************************************************/
void UART_EPSON_COM::gotoSamplingMode(void) {
    regWrite8(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_BEGIN_SAMPLING);
}


/**************************************************************************/
/*!
    @brief  Places the sensor into Configuration Mode.
            This should be only called from Sampling Mode.
*/
/**************************************************************************/
void UART_EPSON_COM::gotoConfigMode(void) {
    regWrite8(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_END_SAMPLING);
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
    @param [in]  readLength
                 Specify the length of the burst read transfer in 16-bit words
                 (excluding the header & delimiter bytes)
    @param [in]  retryMaxCount
                 Maximum retries waiting for sensor bytes before error

    @returns  true if successful, false if errors are detected
*/
/**************************************************************************/
boolean UART_EPSON_COM::readN (uint16_t* arrayOut, uint8_t addr, uint8_t readLength, uint32_t retryMaxCount) {

    uint8_t retByte[128];  // Burst length should never exceed 128 bytes
    uint8_t readByteLength = (readLength * 2) + 2;  // (16-bit length * 2) + header byte + delimiter byte
    uint8_t j = 0;
    uint32_t retryCount = 0;

    // If DRDY is used then assume UART manual mode, and send a burst command every sample
    if (_drdy != -1) {
        uint8_t xmtVal[3];
        // Setup read burst command
        xmtVal[0] = addr;     // This should be the BURST COMMAND (0x20 for V340 or 0x80 for others)
        xmtVal[1] = 0x00;
        xmtVal[2] = DELIMITER;
        SerialEpson.write(xmtVal, 3);

        // delay after 1st command
        EpsonBurstStall();
    }

    // Wait for buffer to fill to 1 complete burst or return false on exceeding max retries
    do {
      retryCount++;
      delayMicroseconds(100);
      if (retryCount > retryMaxCount) {
        SerialConsole.print("Timeout waiting for sensor burst data");
        return false;
      }
    } while (SerialEpson.available() < readByteLength);

    // Read UART and store into byte array
    int bytesRead = SerialEpson.readBytes(retByte, readByteLength);

    // Return false if # of bytes is incorrect
    if (bytesRead != readByteLength) {
        SerialConsole.print("Incorrect bytes read:");
        SerialConsole.println(bytesRead, DEC);
        findDelimiter();     // read sensor Rx buffer until finds DELIMITER
        return false;
    }
    // Return false if first byte & last byte is incorrect
    else if ((retByte[0] != (addr)) || (retByte[readByteLength - 1] != DELIMITER)) {
        SerialConsole.print("Unexpected read response:");
        for(int i=0; i<readByteLength; i++) {
            SerialConsole.print(retByte[i], HEX);
            SerialConsole.print(", ");
        }
        SerialConsole.println();
        findDelimiter();     // read sensor Rx buffer until finds DELIMITER
        return false;
    }

    // Parse UART read byte array to 16-bit output array
    for (int i = 1; i < (readByteLength-1); i = i+2) {
        arrayOut[j] = retByte[i]<<8 | retByte[i+1];
        j++;
    }
    return true;
}


/**************************************************************************/
/*!
    @brief  Assumes Config Mode and Reads the Product ID from the Sensor.

    @param [out]  prodID
                  Pointer to string of 8 ASCII bytes of PROD_ID
                  i.e.
                  Expected ASCII values: "G","3","6","4","P","D","C","0"
                  Return Value: 0x3347, 0x3436, 0x4450, 0x3043
*/
/**************************************************************************/
void UART_EPSON_COM::getProdID(char* prodID) {

    uint8_t i;

    //read model name from registers, stored as ascii values
    for (i = 0; i < 8; i = i + 2) {
        uint16_t retVal = regRead16(CMD_WINDOW1, ADDR_PROD_ID1 + i);
        prodID[i] = (char) (retVal & 0xFF);
        prodID[i + 1] = (char) (retVal>>8);
#ifdef DEBUG
        // Print Return Values 16-bit
        SerialConsole.print(i, HEX);
        SerialConsole.print("\t");
        SerialConsole.println(retVal, HEX);
#endif
    }
    prodID[i] = 0x00;  // add NULL terminator to make it a string
#ifdef DEBUG
    // Print Return Values 8-bit
    for(int8_t j = 0; j < 9; j++) {
        SerialConsole.println(prodID[j], HEX);
    }
#endif
}


/**************************************************************************/
/*!
    @brief  Assumes Config Mode and Reads the Firmware Version from the Sensor.

    @returns  16-bit Firmware Number
*/
/**************************************************************************/
uint16_t UART_EPSON_COM::getVersion(void) {

    return regRead16(CMD_WINDOW1, ADDR_VERSION);
}


/**************************************************************************/
/*!
    @brief  Assumes Config Mode and Reads the Serial Number from the Sensor
            and terminate with NULL to create a string.

    @param [out]  serialNumber
                  Pointer to string of 8 ASCII bytes of Serial Number
                  Each byte is an ASCII number.
*/
/**************************************************************************/
void UART_EPSON_COM::getSerialNumber(char* serialNumber) {

    uint8_t i;

    //read serial number from registers, stored as ascii values
    for (i = 0; i < 8; i = i + 2) {
        uint16_t retVal = regRead16(CMD_WINDOW1, ADDR_SERIAL_NUM1 + i);
        serialNumber[i] = (char) (retVal & 0xFF);
        serialNumber[i + 1] = (char) (retVal>>8);
    }
    serialNumber[i] = 0x00;  // add NULL terminator to make it a string
}


/**************************************************************************/
/*!
    @brief  Blindly reads a byte at a time until it finds the DELIMTER byte
            in the sensor receive buffer
*/
/**************************************************************************/
void UART_EPSON_COM::findDelimiter(void) {

    uint8_t retByte;


    // Wait for 100usec if sensor receive buffer empty
    do {
        while (SerialEpson.available() < 1) {
        delayMicroseconds(100);
        };
        retByte = SerialEpson.read();

    } while (retByte != DELIMITER);
}
