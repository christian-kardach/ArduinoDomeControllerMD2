#pragma once

#include <Arduino.h>
#include "BTS7960.h"
#include "configuration.h"

class Motor {
public:
    Motor(uint8_t nmotor);
    void run(bool dir, int pwm);
    void stop();
    void brake();
    bool isRunning();
    int readCurrent();
private:
    uint8_t _nmotor;
    BTS7960* motorController;
};
