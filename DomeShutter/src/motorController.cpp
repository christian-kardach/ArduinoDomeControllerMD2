#include "motorController.h"

Motor::Motor(uint8_t n)
{
    _isRunning = false;
}

void Motor::run(bool dir, int pwm)
{
}

void Motor::stop()
{
}

void Motor::brake()
{
}

bool Motor::isRunning()
{
    return _isRunning;
}

int Motor::readCurrent()
{
    return 0;
}