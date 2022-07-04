#include "BTS7960Controller.h"

Motor::Motor(uint8_t n)
{
    motorController = new BTS7960(MOTOR_EN, MOTOR_L_PWM, MOTOR_R_PWM);
    motorController->Enable();
}

void Motor::run(bool dir, int pwm) // dir is 0 or 1
{
    // 0 = Clockwise
    // 1 = Counter Clockwise
    switch (dir)
    {
    case 0:
        motorController->TurnRight(pwm);
        break;
    case 1:
        motorController->TurnLeft(pwm);
        break;
    default:
        break;
    }
}

void Motor::stop()
{
    motorController->Disable();
}

void Motor::brake()
{
    motorController->Stop();
}

bool Motor::isRunning()
{
    return false;
}

int Motor::readCurrent()
{
    return 0;
}