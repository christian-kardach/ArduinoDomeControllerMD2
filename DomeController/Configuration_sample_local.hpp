//#define MOTOR_CONTROLLER MOTOR_CONTROLLER_BTS7960
//#define MOTOR_CONTROLLER MOTOR_CONTROLLER_SHIELDMD10

// BTS7960
    #define MOTOR_L_EN  7
    #define MOTOR_R_EN  8
    #define MOTOR_L_PWM 9
    #define MOTOR_R_PWM 10

// SHEILDMD10
    #define MOTOR_PWM 3
    #define MOTOR_DIR 2

// Configuration
// #define HAS_SHUTTER     // Uncomment if the shutter controller is available
#define MOTOR_SHIELD  // Uncomment if the motor driver is a Monster Motor Shield
//#define USE_BUTTONS   // Uncomment if you want to move the dome with push
//buttons

#define AZ_TIMEOUT   30000  // Azimuth movement timeout (in ms)
#define AZ_TOLERANCE 4      // Azimuth target tolerance in encoder ticks
#define AZ_SLOW_RANGE                                                                                                                      \
    8  // The motor will run at slower speed when the                                                                                      \
        // dome is at this number of ticks from the target

// pin definitions
#define ENCODER1    2   // Encoder
#define ENCODER2    3   // Encoder
#define HOME_SENSOR 13  // Home sensor (active low)
#define BUTTON_CW   11  // CW movement button (active low)
#define BUTTON_CCW  12  // CCW movement button (Active low)

#ifdef HAS_SHUTTER
    #ifdef MOTOR_SHIELD
        #error "HAS_SHUTTER and MOTOR_SHIELD cannot be defined at the same time"
    #endif

    // pins of HC12 module (serial radio transceiver)
    #define HC12_RX 4  // Receive Pin on HC12
    #define HC12_TX 5  // Transmit Pin on HC12
#endif

// motor pins (if not using the Monster Motor Shield)
#ifndef MOTOR_SHIELD
    #define MOTOR_JOG 7  // Motor jog mode (low speed)
    #define MOTOR_CW  8  // Move motor clockwise
    #define MOTOR_CCW 9  // Move motor counterclockwise
#endif