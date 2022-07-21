#include <Arduino.h>
#include <avr/wdt.h>
#include "configuration.h"
#include "serialCommand.h"
#include "shutter.h"
#include "motorController.h"

enum {
    BTN_NONE,
    BTN_A_OPEN,
    BTN_A_CLOSE
};

Motor motorA(0);
Shutter shutter(&motorA, SW_A1, SW_A2, SHUTTER_TIMEOUT);
SerialCommand sCmd;

unsigned long lastCmdTime = 0;

// Detect a pressed button by reading an analog input.
// Every button puts a different voltage at the input.
int readButton()
{
    int buttonLimits[] = {92, 303, 518, 820};
    int val = analogRead(BUTTONS);

    for (int i = 0; i < 4; i++) {
        if (val < buttonLimits[i]) {
            return i + 1;
        }
    }
    return 0;
}

// Return dome status by combining shutter and flap statuses
State domeStatus()
{
    State sst = shutter.getState();

    if (sst == ST_ERROR)
        return ST_ERROR;
    else if (sst == ST_OPENING)
        return ST_OPENING;
    else if (sst == ST_CLOSING)
        return ST_CLOSING;
    else if (sst == ST_OPEN)
        return ST_OPEN;
    else if (sst == ST_CLOSED)
        return ST_CLOSED;

    return ST_ABORTED;
}

void cmdOpenShutter() {
    lastCmdTime = millis();
    shutter.open();
}

void cmdClose()
{
    lastCmdTime = millis();
    shutter.close();
}

void cmdAbort()
{
    lastCmdTime = millis();
    shutter.abort();
}

void cmdExit()
{
    lastCmdTime = 0;
    shutter.close();
}

void cmdStatus()
{
    lastCmdTime = millis();
    State st = domeStatus();
    Serial.write('0' + st);
}

void cmdGetVBat()
{
    lastCmdTime = millis();
    int val = analogRead(VBAT_PIN);
    char buffer[8];
    sprintf(buffer, "v%04d", val);
    Serial.write(buffer);
}

void setup()
{
    wdt_disable(); // Disable the watchdog timer
    wdt_enable(WDTO_1S); // Enable the watchdog timer to fire after a 1S freeze/hang/stall/crash

    pinMode(LED_ERR, OUTPUT);

    // Map serial commands to functions
    sCmd.addCommand("open", 5, cmdOpenShutter);
    sCmd.addCommand("close", 6, cmdClose);
    sCmd.addCommand("abort", 6, cmdAbort);
    sCmd.addCommand("exit", 5, cmdExit);
    sCmd.addCommand("stat", 5, cmdStatus);
    sCmd.addCommand("vbat", 5, cmdGetVBat);

    Serial.begin(9600);

    digitalWrite(LED_ERR, HIGH);
    delay(200);
    digitalWrite(LED_ERR, LOW);
}


void loop()
{
    int btn = readButton();
    static int btn_prev = 0;
    static int btn_count = 0;

    if (btn && (btn == btn_prev))
        btn_count++;
    else
        btn_count = 0;
    btn_prev = btn;

    if (btn_count == BUTTON_REPS) {
        switch(btn) {
        case BTN_A_OPEN:
            shutter.open();
            break;
        case BTN_A_CLOSE:
            shutter.close();
            break;
        }
    }

    // Close the dome if time from last command > COMMAND_TIMEOUT
    if ((lastCmdTime > 0) && ((millis() - lastCmdTime) > COMMAND_TIMEOUT)) {
        if (domeStatus() != ST_CLOSED) {
            lastCmdTime = 0;
            shutter.close();
        }
    }

    int err = (shutter.getState() == ST_ERROR);
    digitalWrite(LED_ERR, err);

    shutter.update();
    sCmd.readSerial();

    wdt_reset();
}
