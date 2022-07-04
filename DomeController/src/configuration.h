#pragma once

#define BAUDRATE 19200

// Software Serial
#define HC12_RX   2     // Recieve Pin on HC12 (was 4)
#define HC12_TX   3     // Transmit Pin on HC12 (was 5)

// Motor Controller
#define MOTOR_EN 8
#define MOTOR_L_PWM 9
#define MOTOR_R_PWM 10


#define SW_HOME  10     // Home probe
#define ENCODER1 11     // Azimuth encoder (was 2)
#define ENCODER2 12     // Azimuth encoder (was 3)
// #define LED_ERR  13     // error LED
#define BUTTONS  A4     // analog input for reading the buttons
#define VBAT_PIN A5     // battery voltage reading
#define BUTTON_REPS 80  // Number of ADC readings required to detect a pressed button

// Message Destination
#define TO_MAXDOME  0x00
#define TO_COMPUTER 0x80

#define VBAT_FACTOR (5.0/1024.0)
#define VBAT_OFFSET 10.55

#define CONTROLLER_TIMEOUT 30000

// Commands
#define DIR_CW      0x01 // Clockwise movement
#define DIR_CCW     0x02 // Counterclockwise movement
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

#define AZ_TOLERANCE    4       // Azimuth target tolerance in encoder ticks
#define AZ_SLOW_RANGE   16      //
#define AZ_TIMEOUT      60000   // Azimuth movement timeout (in ms)
#define ENCODER_TICKS   221     // Number of encoder ticks in one dome rotation

// EEPROM addresses
#define ADDR_PARK_POS           0
#define ADDR_TICKS_PER_TURN     2
#define ADDR_PARK_ON_SHUTTER    4