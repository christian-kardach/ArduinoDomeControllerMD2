/*******************************************************************************
ArduinoDomeController
Azimuth control of an astronomical dome using Arduino


The MIT License

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*******************************************************************************/

#include <EEPROM.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#include "configuration.h"

#if MOTOR_CONTROLLER == BTS7960
    #include "BTS7960Controller.h"
#endif

#include "serialCommand.h"

enum MotorSpeed
{
    MOTOR_STOP,
    MOTOR_SLOW,
    MOTOR_FAST
};

enum AzimuthEvent
{
    EVT_NONE,
    EVT_GOTO,
    EVT_HOME,
    EVT_ABORT
};

enum AzimuthStatus
{
    ST_IDLE,
    ST_MOVING,
    ST_APPROACHING,
    ST_HOMING,
    ST_ERROR,
};

// MaxDome II azimuth status
enum MDAzimuthStatus
{
    AS_IDLE = 1,
    AS_MOVING_CW,
    AS_MOVING_CCW,
    AS_IDLE2,
    AS_ERROR
};

// MaxDome II shutter status
enum ShutterStatus
{
    SS_CLOSED = 0,
    SS_OPENING,
    SS_OPEN,
    SS_CLOSING,
    SS_ABORTED,
    SS_ERROR
};

#ifdef HAS_SHUTTER
// Create a Software Serial Port to communicate with the shutter controller
SoftwareSerial HC12(HC12_TX, HC12_RX);
#endif

SoftwareSerial ttl(4, 5);

SerialCommand sCmd;

#ifdef MOTOR_SHIELD
Motor motor(0);
#endif

bool park_on_shutter    = false;
bool home_reached       = false;
bool parking            = false;
uint8_t current_dir     = DIR_CW;  // Current azimuth movement direction
uint16_t park_pos       = 0;       // Parking position
uint16_t current_pos    = 0;       // Current dome position
uint16_t target_pos     = 0;       // Target dome position
uint16_t home_pos       = 0;       // Home position
uint16_t ticks_per_turn = 360;     // Encoder ticks per dome revolution
AzimuthStatus state     = ST_IDLE;
AzimuthEvent az_event   = EVT_NONE;

// Convert two bytes to a uint16_t value (big endian)
uint16_t bytesToInt(uint8_t *data)
{
    uint16_t value1 = data[1];
    uint16_t value2 = data[0];
    return (value1 & 0xff) | ((value2 << 8) & 0xff00);
}

// Convert a uint16_t value to bytes (big endian)
void intToBytes(uint16_t value, uint8_t *data)
{
    data[1] = value & 0xff;
    data[0] = (value >> 8) & 0xff;
}

// Read a uint16_t value from EEPROM (little endian)
uint16_t eepromReadUint16(int address)
{
    uint16_t value1 = EEPROM.read(address);
    uint16_t value2 = EEPROM.read(address + 1);
    return (value1 & 0xff) | ((value2 << 8) & 0xff00);
}

// Read a uint16_t value from EEPROM (little endian)
void eepromWriteUint16(int address, uint16_t value)
{
    uint8_t value1 = value & 0xff;
    uint8_t value2 = (value >> 8) & 0xff;
    EEPROM.write(address, value1);
    EEPROM.write(address + 1, value2);
}

// Obtain the direction of the shortest path between two positons
uint8_t getDirection(uint16_t current, uint16_t target)
{
    if (target > current)
        return ((target - current) > ticks_per_turn / 2) ? DIR_CCW : DIR_CW;

    return ((current - target) > ticks_per_turn / 2) ? DIR_CW : DIR_CCW;
}

// Obtain the distance between two positons
uint16_t getDistance(uint16_t current, uint16_t target)
{
    // obtain the absolute value of the difference
    uint16_t diff = (target > current) ? target - current : current - target;

    if (diff > ticks_per_turn / 2)
        return ticks_per_turn - diff;

    return diff;
}

inline void moveAzimuth(uint8_t dir, bool slow)
{
    digitalWrite(LED_BUILTIN, HIGH);
#ifdef MOTOR_SHIELD
    int speed = slow ? 100 : 244;
    motor.run(dir == DIR_CW, speed);
#else
    digitalWrite(MOTOR_JOG, slow);
    digitalWrite(MOTOR_CW, dir == DIR_CW);
    digitalWrite(MOTOR_CCW, dir != DIR_CW);
#endif
}

inline void stopAzimuth()
{
    digitalWrite(LED_BUILTIN, LOW);
#ifdef MOTOR_SHIELD
    motor.stop();
#else
    digitalWrite(MOTOR_JOG, LOW);
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
#endif
}

float getShutterVBat()
{
    int adc = 0;

#ifdef HAS_SHUTTER
    char buffer[5];

    HC12.flush();
    for (int i = 0; i < 4; i++)
    {
        HC12.println("vbat");
        delay(100);

        if (HC12.read() == 'v')
        {
            for (int j = 0; j < 4; j++)
            {
                buffer[j] = HC12.read();
            }

            buffer[4] = 0;
            adc       = atoi(buffer);
            break;
        }
    }
#endif

    // Convert ADC reading to voltage
    return (float) adc * VBAT_FACTOR + VBAT_OFFSET;
}

ShutterStatus getShutterStatus()
{
    ShutterStatus st = SS_OPEN;

#ifdef HAS_SHUTTER
    st = SS_ERROR;
    HC12.flush();
    for (int i = 0; i < 4; i++)
    {
        HC12.println("stat");
        delay(100);

        char c = 0;
        while (HC12.available() > 0)
        {
            c = HC12.read();
        }

        if (c >= '0' && c <= ('0' + SS_ERROR))
        {
            st = (ShutterStatus) (c - '0');
            break;
        }
    }
#endif
    return st;
}

void cmdAbort(uint8_t *cmd)
{
#ifdef HAS_SHUTTER
    HC12.println("abort");  // abort shutter movement
#endif

    az_event = EVT_ABORT;

    uint8_t resp[] = {START, 2, TO_COMPUTER | ABORT_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdHomeAzimuth(uint8_t *cmd)
{
    ttl.println("cmdHomeAzimuth");
    if (!parking)
    {
        current_dir = getDirection(current_pos, 0);
        az_event    = EVT_HOME;
    }

    uint8_t resp[] = {START, 3, TO_COMPUTER | HOME_CMD, 0x01, 0x00};
    sCmd.sendResponse(resp, 5);
}

void cmdGotoAzimuth(uint8_t *cmd)
{
    ttl.println("cmdGotoAzimuth");

    if (!parking)
    {
        current_dir = cmd[3];
        target_pos  = bytesToInt(cmd + 4);
        ttl.println(target_pos);
        az_event = EVT_GOTO;
    }

    uint8_t resp[] = {START, 3, TO_COMPUTER | GOTO_CMD, 0x01, 0x00};
    sCmd.sendResponse(resp, 5);
}

void parkDome()
{
    ttl.println("parkDome");
    parking    = true;
    target_pos = park_pos;
    az_event   = EVT_GOTO;
}

void cmdShutterCommand(uint8_t *cmd)
{
#ifdef HAS_SHUTTER
    switch (cmd[3])
    {
        case OPEN_SHUTTER:
            HC12.println("open");
            break;
        case OPEN_UPPER_ONLY_SHUTTER:
            HC12.println("open1");
            break;
        case CLOSE_SHUTTER:
            if (park_on_shutter)
                parkDome();
            else
                HC12.println("close");
            break;
        case EXIT_SHUTTER:
            HC12.println("exit");
            break;
        case ABORT_SHUTTER:
            HC12.println("abort");
            break;
    }
#endif

    uint8_t resp[] = {START, 2, TO_COMPUTER | SHUTTER_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdStatus(uint8_t *cmd)
{
    ttl.println("cmdStatus");
    MDAzimuthStatus az_status;
    if (state == ST_IDLE)
    {
        az_status = AS_IDLE;
    }
    else if (state == ST_ERROR)
    {
        az_status = AS_ERROR;
    }
    else
    {
        if (current_dir == DIR_CW)
            az_status = AS_MOVING_CW;
        else
            az_status = AS_MOVING_CCW;
    }

    uint8_t sh_status = (uint8_t) getShutterStatus();
    uint8_t resp[]    = {START, 9, TO_COMPUTER | STATUS_CMD, sh_status, az_status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    intToBytes(current_pos, resp + 5);
    intToBytes(home_pos, resp + 7);

    sCmd.sendResponse(resp, 11);
}

void cmdSetPark(uint8_t *cmd)
{
    ttl.println("cmdSetPark");
    park_on_shutter = cmd[3];
    park_pos        = bytesToInt(cmd + 4);

    EEPROM.write(ADDR_PARK_ON_SHUTTER, park_on_shutter);
    eepromWriteUint16(ADDR_PARK_POS, park_pos);

    uint8_t resp[] = {START, 2, TO_COMPUTER | SETPARK_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdSetTicks(uint8_t *cmd)
{
    ticks_per_turn = bytesToInt(cmd + 3);
    eepromWriteUint16(ADDR_TICKS_PER_TURN, ticks_per_turn);

    uint8_t resp[] = {START, 2, TO_COMPUTER | TICKS_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdVBat(uint8_t *cmd)
{
    uint8_t resp[] = {START, 4, TO_COMPUTER | VBAT_CMD, 0x00, 0x00, 0x00};

    int vbat = getShutterVBat() * 100;
    intToBytes(vbat, resp + 3);

    sCmd.sendResponse(resp, 6);
}

void cmdAck(uint8_t *cmd)
{
    uint8_t resp[] = {START, 3, TO_COMPUTER | ACK_CMD, 0x03, 0x00};
    sCmd.sendResponse(resp, 5);
}

void updateAzimuthFSM()
{
    static unsigned long t0;

    switch (state)
    {
        case ST_HOMING:
            if (az_event == EVT_ABORT)
            {
                parking = false;
                stopAzimuth();
                state = ST_IDLE;
            }
            else if (home_reached)
            {
                stopAzimuth();
                state        = ST_IDLE;
                home_reached = false;
            }
            else if (millis() - t0 > AZ_TIMEOUT)
            {
                stopAzimuth();
                state = ST_ERROR;
            }
            break;

        case ST_MOVING:
            if (az_event == EVT_ABORT)
            {
                parking = false;
                stopAzimuth();
                state = ST_IDLE;
            }
            else if (getDistance(current_pos, target_pos) < AZ_SLOW_RANGE)
            {
                moveAzimuth(current_dir, true);
                state = ST_APPROACHING;
            }
            else if (millis() - t0 > AZ_TIMEOUT)
            {
                stopAzimuth();
                state = ST_ERROR;
            }
            break;

        case ST_APPROACHING:
            if (az_event == EVT_ABORT)
            {
                parking = false;
                stopAzimuth();
                state = ST_IDLE;
            }
            else if (getDistance(current_pos, target_pos) < AZ_TOLERANCE)
            {
                stopAzimuth();

                // close shutter after parking
                if (parking)
                {
                    parking = false;
#ifdef HAS_SHUTTER
                    HC12.println("close");
#endif
                }

                state = ST_IDLE;
            }
            else if (millis() - t0 > AZ_TIMEOUT)
            {
                stopAzimuth();
                state = ST_ERROR;
            }
            break;

        case ST_ERROR:
        case ST_IDLE:
            if (az_event == EVT_HOME)
            {
                t0    = millis();
                state = ST_HOMING;
                moveAzimuth(current_dir, false);
            }
            else if (az_event == EVT_GOTO)
            {
                t0    = millis();
                state = ST_MOVING;
                moveAzimuth(current_dir, false);
            }
            break;
    }
    az_event = EVT_NONE;
}

// Encoder interrupt service routine
void encoderISR()
{
    if (digitalRead(ENCODER1) == digitalRead(ENCODER2))
    {
        if (current_pos == 0)
        {
            ttl.print("Current Position: ");
            ttl.println(current_pos);
            current_pos = ticks_per_turn - 1;
        }
        else
        {
            current_pos--;
        }
    }
    else
    {
        if (current_pos >= ticks_per_turn - 1)
            current_pos = 0;
        else
            current_pos++;
    }
}

void encoderPinChangeA()
{
    int test = digitalRead(ENCODER1);
    ttl.println(test);
    delay(500);
    //int encoder += digitalRead(encoder_a) == digitalRead(encoder_b) ? -1 : 1;
}

void setup()
{
    wdt_disable();
    wdt_enable(WDTO_2S);

    pinMode(ENCODER1, INPUT_PULLUP);
    pinMode(ENCODER2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER1), encoderISR, CHANGE);

    sCmd.addCommand(ABORT_CMD, 2, cmdAbort);
    sCmd.addCommand(HOME_CMD, 2, cmdHomeAzimuth);
    sCmd.addCommand(GOTO_CMD, 5, cmdGotoAzimuth);
    sCmd.addCommand(SHUTTER_CMD, 3, cmdShutterCommand);
    sCmd.addCommand(STATUS_CMD, 2, cmdStatus);
    sCmd.addCommand(SETPARK_CMD, 5, cmdSetPark);
    sCmd.addCommand(TICKS_CMD, 4, cmdSetTicks);
    sCmd.addCommand(ACK_CMD, 2, cmdAck);
    sCmd.addCommand(VBAT_CMD, 2, cmdVBat);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(HOME_SENSOR, INPUT_PULLUP);

#ifdef USE_BUTTONS
    pinMode(BUTTON_CW, INPUT_PULLUP);
    pinMode(BUTTON_CCW, INPUT_PULLUP);
#endif

    motor.setup();

    park_pos        = eepromReadUint16(ADDR_PARK_POS);
    park_on_shutter = EEPROM.read(ADDR_PARK_ON_SHUTTER);
    ticks_per_turn  = eepromReadUint16(ADDR_TICKS_PER_TURN);

    Serial.begin(19200);

#ifdef HAS_SHUTTER
    HC12.begin(9600);  // Open serial port to HC12
#endif

    ttl.begin(9600);
}

// move the motor when the buttons are pressed
void read_buttons()
{
    static int prev_cw_button = 0, prev_ccw_button = 0;
    int cw_button  = !digitalRead(BUTTON_CW);
    int ccw_button = !digitalRead(BUTTON_CCW);

    if (cw_button != prev_cw_button)
    {
        if (cw_button)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            moveAzimuth(DIR_CW, false);
        }
        else
        {
            digitalWrite(LED_BUILTIN, LOW);
            stopAzimuth();
        }
    }
    else if (ccw_button != prev_ccw_button)
    {
        if (ccw_button)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            moveAzimuth(DIR_CCW, false);
        }
        else
        {
            digitalWrite(LED_BUILTIN, LOW);
            stopAzimuth();
        }
    }
    prev_cw_button  = cw_button;
    prev_ccw_button = ccw_button;
}

void loop()
{
    sCmd.readSerial();
    updateAzimuthFSM();
#ifdef USE_BUTTONS
    read_buttons();
#endif
    wdt_reset();

    // store detected home position
    if (!digitalRead(HOME_SENSOR))
    {
        if (state == ST_HOMING)
        {
            ttl.println("Home Set");
            home_pos     = 0;
            current_pos  = 0;
            home_reached = true;
        }
        else
            home_pos = current_pos;
    }
}