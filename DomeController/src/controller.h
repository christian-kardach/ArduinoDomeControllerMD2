/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the ArduinoDomeController project:
        https://github.com/juanmb/ArduinoDomeController

*******************************************************************/

#ifndef _controller_h_
#define _controller_h_

#include "enums.h"

#if MOTOR_CONTROLLER == BTS7960
#include "BTS7960Controller.h"
#endif

// Define a pointer to a function for checking controller/flap
// typedef bool (*interFn)(State st);

class Controller {
public:
    Controller(Motor *motor, int homedSwitch, unsigned long timeout);
    void cw();
    void ccw();
    void abort();
    void update();
    State getState();
private:
    void initState();
    Motor *motor;
    State state;
    Action nextAction;
    unsigned long runTimeout;
    int swHomed;
};

#endif