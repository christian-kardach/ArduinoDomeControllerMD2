#pragma once

#include <Arduino.h>
#include <stdlib.h>

class Motor {
public:
    Motor(uint8_t nmotor);
    void run(bool dir, int pwm);
    void stop();
    void brake();
    bool isRunning();
    int readCurrent();
private:
    bool _isRunning;
    uint8_t _nmotor;
};