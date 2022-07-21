// Pin definitions
#define LED_ERR  13     // error LED
#define SW_A1    12     // shutter closed switch (NC)
#define SW_A2    11     // shutter open switch (NO)
#define BUTTONS  A4     // analog input for reading the buttons
#define VBAT_PIN A5     // battery voltage reading

#define BUTTON_REPS 80  // Number of ADC readings required to detect a pressed button

// Timeouts in ms
#define COMMAND_TIMEOUT 60000   // Max. time from last command
#define SHUTTER_TIMEOUT 75000   // Max. time the shutter takes to open/close

// Shutter commands
#define OPEN_SHUTTER            0x01
#define OPEN_UPPER_ONLY_SHUTTER 0x02
#define CLOSE_SHUTTER           0x03
#define EXIT_SHUTTER            0x04 // Command sent to shutter on program exit
#define ABORT_SHUTTER           0x07