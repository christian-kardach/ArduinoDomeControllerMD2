/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the ArduinoDomeController project:
        https://github.com/juanmb/ArduinoDomeController

*******************************************************************/

#include "controller.h"
#include <Arduino.h>
#include <util/atomic.h>

#define MOTOR_CW        0
#define MOTOR_CCW       1
#define SPEED           254
#define DEFAULT_TIMEOUT 30000  // controller timeout (in ms)

// Controller constructor.
// motor: pointer to an instance of Motor
// sw1: Home sensor (Hall Effect)

Controller::Controller(Motor *motorPtr, int homedSwitch, unsigned long timeout)
{
    motor      = motorPtr;
    swHomed    = homedSwitch;  // normally open - should have a value of 1 when homed
    runTimeout = timeout;
    nextAction = DO_NONE;
    initState();
}

void Controller::initState()
{
    if (digitalRead(swHomed))
        state = ST_HOMED;
    else if (!digitalRead(swHomed))
        // state = ST_NOTHOMED;
        state = ST_HOMED;
    else
        state = ST_ABORTED;
}

void Controller::cw()
{
    nextAction = DO_CW;
}
void Controller::ccw()
{
    nextAction = DO_CCW;
}
void Controller::abort()
{
    nextAction = DO_ABORT;
}

State Controller::getState()
{
    return state;
}

// Controller state machine
void Controller::update()
{
    Action action;
    static unsigned long t0;

    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        action     = nextAction;
        nextAction = DO_NONE;
    }
    switch (state)
    {
        case ST_NOTHOMED:
            if (action == DO_CW)
            {
                t0 = millis();
                Serial.println("DO_CW");
                state = ST_CWING;
            }
            else if (action == DO_CCW)
            {
                t0 = millis();
                Serial.println("DO_CW");
                state = ST_CCWING;
            }
            break;
        case ST_HOMED:
            if (action == DO_CW)
            {
                t0 = millis();
                Serial.println("DO_CW");
                state = ST_CWING;
            }
            else if (action == DO_CCW)
            {
                t0 = millis();
                Serial.println("DO_CCW");
                state = ST_CCWING;
            }
            break;
        case ST_ABORTED:
        case ST_ERROR:
            for (int i = 0; i < 3; i++)
            {
                digitalWrite(LED_BUILTIN,
                             HIGH);  // turn the LED on (HIGH is the voltage level)
                delay(200);          // wait for a second
                digitalWrite(LED_BUILTIN,
                             LOW);  // turn the LED off by making the voltage LOW
                delay(200);         // wait for a second
            }
            if (action == DO_CW)
            {
                t0    = millis();
                state = ST_CWING;
            }
            else if (action == DO_CCW)
            {
                t0    = millis();
                state = ST_CCWING;
            }
            break;
        case ST_CWING:
            for (int i = 0; i < 1; i++)
            {
                digitalWrite(LED_BUILTIN,
                             HIGH);  // turn the LED on (HIGH is the voltage level)
                delay(200);          // wait for a second
                digitalWrite(LED_BUILTIN,
                             LOW);  // turn the LED off by making the voltage LOW
                delay(200);         // wait for a second
            }
            motor->run(MOTOR_CW, 100);
            if (action == DO_ABORT || action == DO_CCW)
            {
                state = ST_ABORTED;
                motor->brake();
            }
            else if (millis() - t0 > runTimeout)
            {
                state = ST_ERROR;
                motor->brake();
            }
            break;
        case ST_CCWING:
            motor->run(MOTOR_CCW, 100);
            if (action == DO_ABORT || action == DO_CW)
            {
                state = ST_ABORTED;
                motor->brake();
            }
            else if (millis() - t0 > runTimeout)
            {
                state = ST_ERROR;
                motor->brake();
            }
            break;
    }
}
