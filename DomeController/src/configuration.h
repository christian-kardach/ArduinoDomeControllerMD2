#pragma once

#define BAUDRATE 19200

/*
0   HC12_RX
1   HC12_TX
7   Motor L Enable
8   Motor R Enable
9   Motor L PWM
10  Motor R PWM
13  Home Probe
2  AZ Encoder
3  AZ Encoder

A4  Buttons
A5  Battery Voltage Reading
*/

#define MOTOR_CONTROLLER BTS7960

#if MOTOR_CONTROLLER == BTS7960
    #define MOTOR_L_EN 7
    #define MOTOR_R_EN 8
    #define MOTOR_L_PWM 9
    #define MOTOR_R_PWM 10
#endif

// Configuration
// #define HAS_SHUTTER     // Uncomment if the shutter controller is available
#define MOTOR_SHIELD  // Uncomment if the motor driver is a Monster Motor Shield
//#define USE_BUTTONS   // Uncomment if you want to move the dome with push buttons

#define AZ_TIMEOUT      30000   // Azimuth movement timeout (in ms)
#define AZ_TOLERANCE    4        // Azimuth target tolerance in encoder ticks
#define AZ_SLOW_RANGE   8       // The motor will run at slower speed when the
                                // dome is at this number of ticks from the target

// pin definitions
#define ENCODER1 2      // Encoder
#define ENCODER2 3      // Encoder
#define HOME_SENSOR 13  // Home sensor (active low)
#define BUTTON_CW   11  // CW movement button (active low)
#define BUTTON_CCW  12  // CCW movement button (Active low)

#ifdef HAS_SHUTTER
#ifdef MOTOR_SHIELD
    #error "HAS_SHUTTER and MOTOR_SHIELD cannot be defined at the same time"
#endif

// pins of HC12 module (serial radio transceiver)
#define HC12_RX 4       // Receive Pin on HC12
#define HC12_TX 5       // Transmit Pin on HC12
#endif

// motor pins (if not using the Monster Motor Shield)
#ifndef MOTOR_SHIELD
#define MOTOR_JOG 7     // Motor jog mode (low speed)
#define MOTOR_CW 8      // Move motor clockwise
#define MOTOR_CCW 9     // Move motor counterclockwise
#endif

// Message Destination
#define TO_MAXDOME  0x00
#define TO_COMPUTER 0x80

#define VBAT_FACTOR (5.0/1024.0)
#define VBAT_OFFSET 10.55

#define BAUDRATE 19200

// Commands
#define ABORT_CMD   0x03 // Abort azimuth movement
#define HOME_CMD    0x04 // Move until 'home' position is detected
#define GOTO_CMD    0x05 // Go to azimuth position
#define SHUTTER_CMD 0x06 // Send a command to shutter
#define STATUS_CMD  0x07 // Retrieve status
#define TICKS_CMD   0x09 // Set the number of tick per revolution of the dome
#define ACK_CMD     0x0A // ACK (?)
#define SETPARK_CMD 0x0B // Set park coordinates and shutter closing policy
#define VBAT_CMD    0x0C // Read shutter's battery voltage

// Shutter commands
#define OPEN_SHUTTER            0x01
#define OPEN_UPPER_ONLY_SHUTTER 0x02
#define CLOSE_SHUTTER           0x03
#define EXIT_SHUTTER            0x04 // Command sent to shutter on program exit
#define ABORT_SHUTTER           0x07

#define DIR_CW  0x01
#define DIR_CCW 0x02

// EEPROM addresses
#define ADDR_PARK_POS           0
#define ADDR_TICKS_PER_TURN     2
#define ADDR_PARK_ON_SHUTTER    4