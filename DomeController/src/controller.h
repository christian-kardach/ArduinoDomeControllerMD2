/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the ArduinoDomeController project:
        https://github.com/juanmb/ArduinoDomeController

*******************************************************************/

#ifndef _controller_h_
#define _controller_h_

#include "enums.h"
#include "configuration.h"

#if MOTOR_CONTROLLER == MOTOR_CONTROLLER_BTS7960
    #include "controllers/BTS7960Controller.h"
#elif MOTOR_CONTROLLER == MOTOR_CONTROLLER_SHIELDMD10
    #include "controllers/SHIELDMD10Controller.h"
#endif

// Define a pointer to a function for checking controller/flap
// typedef bool (*interFn)(State st);

class Controller
{
  public:
  #if MOTOR_CONTROLLER == MOTOR_CONTROLLER_BTS7960
    Controller(BTS7960::Motor *motor, int homedSwitch, unsigned long timeout);
  #elif MOTOR_CONTROLLER == MOTOR_CONTROLLER_SHIELDMD10
    Controller(SHIELDMD10::Motor *motor, int homedSwitch, unsigned long timeout);
  #endif
    void cw();
    void ccw();
    void abort();
    void update();
    State getState();

  private:
    void initState();
    
    #if MOTOR_CONTROLLER == MOTOR_CONTROLLER_BTS7960
    BTS7960::Motor *motor;
    #elif MOTOR_CONTROLLER == MOTOR_CONTROLLER_SHIELDMD10
    SHIELDMD10::Motor *motor;
    #endif
    State state;
    Action nextAction;
    unsigned long runTimeout;
    int swHomed;
};

#endif
