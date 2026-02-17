#pragma once

#include "motors.h"


class Buggy {
private:
    Motor &leftMotor;
    Motor &rightMotor;

public:
    Buggy(Motor &L, Motor &R) : leftMotor(L), rightMotor(R) {}

    void forward_backward(float moving_speed, float time_fb){
        leftMotor.move(moving_speed);
        rightMotor.move(moving_speed);
        wait(time_fb);
        leftMotor.move(0.0f);
        rightMotor.move(0.0f); //stops the buggy after the wait finishes
    }

    void turn_left_right(float turning_speed, float time_lr){ //ensure that speed value is > 0 to turn right, and < 0 to turn left
        leftMotor.move(turning_speed);
        rightMotor.move(-turning_speed);
        wait(time_lr);
        leftMotor.move(0.0f);
        rightMotor.move(0.0f); //stops the buggy after the wait finishes
    }
};
