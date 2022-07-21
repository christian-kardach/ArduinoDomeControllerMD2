#pragma once

#define BAUDRATE 19200

/*
0   HC12_RX
1   HC12_TX
13  Home Probe
2  AZ Encoder
3  AZ Encoder

A4  Buttons
A5  Battery Voltage Reading
*/

#include "../Constants.h"
#include "../Configuration_local.h"

// Message Destination
#define TO_MAXDOME  0x00
#define TO_COMPUTER 0x80

#define VBAT_FACTOR (5.0 / 1024.0)
#define VBAT_OFFSET 10.55

#define BAUDRATE 19200

// Commands
#define ABORT_CMD   0x03  // Abort azimuth movement
#define HOME_CMD    0x04  // Move until 'home' position is detected
#define GOTO_CMD    0x05  // Go to azimuth position
#define SHUTTER_CMD 0x06  // Send a command to shutter
#define STATUS_CMD  0x07  // Retrieve status
#define TICKS_CMD   0x09  // Set the number of tick per revolution of the dome
#define ACK_CMD     0x0A  // ACK (?)
#define SETPARK_CMD 0x0B  // Set park coordinates and shutter closing policy
#define VBAT_CMD    0x0C  // Read shutter's battery voltage

// Shutter commands
#define OPEN_SHUTTER            0x01
#define OPEN_UPPER_ONLY_SHUTTER 0x02
#define CLOSE_SHUTTER           0x03
#define EXIT_SHUTTER            0x04  // Command sent to shutter on program exit
#define ABORT_SHUTTER           0x07

#define DIR_CW  0x01
#define DIR_CCW 0x02

// EEPROM addresses
#define ADDR_PARK_POS        0
#define ADDR_TICKS_PER_TURN  2
#define ADDR_PARK_ON_SHUTTER 4