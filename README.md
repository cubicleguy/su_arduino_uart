# Table of Contents

______________________________________________________________________

<!---toc start-->

- [Table of Contents](#table-of-contents)
- [Epson Sensing Unit UART Driver for Arduino](#epson-sensing-unit-uart-driver-for-arduino)
- [Hardware Considerations](#hardware-considerations)
- [Installation Instructions](#installation-instructions)
  - [1. Install Arduino IDE](#1-install-arduino-ide)
  - [2. Install the Teensy 3.6 board](#2-install-the-teensy-36-board)
  - [3. Install the Arduino SAM boards](#3-install-the-arduino-sam-boards)
  - [4. Install Epson SU Library and example sketches](#4-install-epson-su-library-and-example-sketches)
  - [5. Use the IDE to compile example sketches and upload to the board](#5-use-the-ide-to-compile-example-sketches-and-upload-to-the-board)
- [Change Record:](#change-record)

<!---toc end-->

# Epson Sensing Unit UART Driver for Arduino

______________________________________________________________________

This is an example test library for the Epson M-Gxxx/M-Axxx
Sensing Units (SU) using the UART interface.
It is developed on the Arduino Zero or Teensy 3.6
development board (Teensyduino) and includes example applications that
can be used within the Arduino IDE.
The library requires 2 UART ports on the Arduino (one UART for serial
console output, and one UART for connection to Epson device).

**NOTE:** Arduino boards with legacy ATMEL MCUs will not support accurate
baudrate timings (i.e. 230400 baud or 460800 baud) that are compatible
with Epson sensing devices.

For detailed information on the Epson Sensing Units, refer to the datasheet at
----> https://www.epsondevice.com/sensing/en/

For further information on the Arduino, refer to their website at
----> http://www.arduino.cc

This software is released under the BSD license (see license.txt).
All text must be included in any redistribution.

# Hardware Considerations

______________________________________________________________________

This library assumes that the user has the following:

- Epson M-Gxxx or M-Axxx sensor device
- Arduino Teensy 3.6 or Arduino Zero (or compatible) development board
- Arduino IDE software v2.3.6 or greater
- This software package
- Micro USB cable to power/connect to development board and use Serial Monitor

The default configuration of the driver assumes that:

- Epson device is connected to `Serial1`
- serial console output is connected to `Serial`

Additionally, the Epson device `RESET#` and `DataReady (optional)` is connected to available
pins on the Arduino.

The following table shows the default pin mapping used by the M-Gxxx/M-A352 driver.

Circuit Pinmapping:

| Signal on Host  | Teensy 3.6 | Arduino Zero | Epson Sensor |
| --------------- | ---------- | ------------ | ------------ |
| DRDY (Optional) | pin 3      | pin 3        | pin 13       |
| RESET#          | pin 2      | pin 2        | pin 16       |
| TXDO            | pin 1      | pin 1        | pin 9 (SIN)  |
| RXDI            | pin 0      | pin 0        | pin 7 (SOUT) |

**CAUTION**: The Epson device I/O interface is 3.3V CMOS.
Be sure to use only Arduino devices that are 3.3V I/O!

# Installation Instructions

______________________________________________________________________

To use the Epson SU Arduino driver and examples, the following steps are required.

1. Install Arduino IDE (if not already installed)
2. Install the Arduino board
3. Install Epson SU Arduino Library and example sketches
4. Use the IDE to compile example sketches and upload to the Arduino board

## 1. Install Arduino IDE

______________________________________________________________________

The Epson SU Arduino driver is designed to work with the Arduino IDE.
The IDE requires a platform running Windows, Mac OS X, or Linux.
If you do not have the IDE installed on your development platform, please visit the Arduino
website and download the version of the IDE compatible with your operating system.
Once the IDE is installed on your development platform, proceed to the next step.
For specific requirements and installation instructions, refer to the Arduino website at www.arduino.cc.

## 2. Install the Teensy 3.6 board

______________________________________________________________________

The default installation of Arduino IDE may not include support for the Teensy 3.6.
To confirm whether Teensy support is installed click on `Tools->Board->Boards Manager...` on the IDE menu.

Serach for "Teensy" and confirm whether the `Teensy (for Arduino IDE 2.0.4 or later)` board package is installed. If the
package is not installed, install it the latest version. Once the Teensy package is installed,
proceed to the next step.

## 3. Install the Arduino SAM boards

______________________________________________________________________

The default installation of Arduino IDE may not include support for the Arduino Zero.
To confirm whether Zero support is installed click on `Tools->Board->Boards Manager...` on the IDE menu.

Confirm whether the `Arduino SAM Boards (32-bits ARM Cortex-M0+)` board package is installed. If the
package is not installed, install the latest version that matches your version of the IDE.
Once the SAM Boards package is installed, proceed to the next step

## 4. Install Epson SU Library with example sketches

______________________________________________________________________

The Epson SU driver for Arduino is available for install within the Arduino IDE if connected to the internet.
In the Arduino IDE click on `Tools->Manage Libraries->Library Manager search for Epson_SU_UART...`.
Then select the Epson SU driver package to install the driver and examples.

The Epson SU Arduino driver is available as a .zip archive from https://github.com/cubicleguy/su_arduino_uart/releases.
The IDE can directly import the driver from a .zip file, so on the IDE menu click on `Sketch->Include Library->Add .ZIP Library...`.
Then select the Epson SU driver ZIP package. This will install the driver and examples.

## 5. Use the IDE to compile example sketches and upload to the board

______________________________________________________________________

Before compiling the example sketches, set the Board and Port settings in the IDE.
The Board and Port settings tell the IDE which Arduino product is being used and how to communicate with it.
To set the Board, click `Tools->Board->select the Teensy 3.6, Arduino Zero, or other compatible board`.
To set the Port, click `Tools->Port->select the proper serial port`.

The port that your Arduino board is located on may differ according to the operating system on the development system.
For issues regarding USB port connections, please refer to the Arduino website at http://www.arduino.cc.

The following are examples sketches included in the library:

- su_epson_accl_sampling.ino is designed to demonstrate initializing and reading accelerometer data.
- su_epson_imu_sampling.ino is similar but for the IMU sensor.
- su_epson_vibe_sampling.ino is similar but for the vibration sensor.

To open the example sketches click on `File->Examples`, find the `Epson SU UART...`, and then select one of the example sketches.
Once the example sketch is loaded, it can be compiled and uploaded to the Arduino.
**NOTE:** The Upload stage will fail if the IDE `Board` and `Port` settings are not configured correctly.

If the upload to the Arduino completes successfully, the output from the example sketch can be viewed using
the Serial Monitor available in `Tools->Serial Monitor`.
**NOTE:** You may have to set the serial baudrate on the Serial Monitor to match the baudrate setting in the sketch.

# Change Record:

| Date       | Ver    | Comment                                                                                     |
| ---------- | ------ | ------------------------------------------------------------------------------------------- |
| 2019-02-25 | v1.0   | - Initial release                                                                           |
| 2021-03-30 | v1.1   | - Unite IMU and Accelerometers under common Sensing Units                                   |
| 2022-07-21 | v1.2   | - Added support to detect corrupt sensor packets and cleanup                                |
| 2023-01-17 | v1.3   | - Added support G330/G366, and cleanup                                                      |
| 2023-08-16 | v1.4   | - Added support G370PDG0, G370PDT0, cleanup, minor fixes                                    |
| 2024-06-17 | v1.5   | - Remove column for "Other Arduinos" in Circuit Pinmapping because it varies based on board |
| 2025-10-27 | v2.0   | - Major revamp the driver, add support for M-G355QDG0, M-G570PR20, M-A370AD10, M-A342VD10   |
| 2026-04-11 | v2.0.1 | - Minor patch for Arduino Zero or similar not printing floating point values                |
