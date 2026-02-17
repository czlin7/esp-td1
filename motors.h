#pragma once

#include "mbed.h"
#include <cmath>

// ============================================
// MotorData
// ============================================
// Control Signals towards Motor Driver board
struct MotorData {
  bool motor_enable;      // disabled by default
  bool motor_unipolar;    // unipolar by default
  bool motor_dir;         // forward by default
  float duty_cycle;     // 0..100
  float encoder_speed_ms; // m/s

  MotorData()
      : motor_enable(false), motor_unipolar(true), motor_dir(true),
        duty_cycle(0.5), encoder_speed_ms(0.0f) {}
};

class Motor {
private:
    PwmOut pwmPin;
    DigitalOut direction;
    DigitalOut bipolar;
    bool bipolar_status;

public:
    Motor(PinName pwm, PinName dir, PinName bi) : pwmPin(pwm), direction(dir), bipolar(bi) {
        pwmPin.period(0.001f);
        bipolar_status = 0;
        direction = 0;
    }

    void BiUni_Setter(bool bipolarity) { //sets the mode of the motor to either bipolar or unipolar
        bipolar_status = bipolarity;
        if (bipolar_status == 1) {
            bipolar = 1; //set as bipolar
        } else {
            bipolar = 0; //set as unipolar
        }
    }

    bool PolarityCheck(void){
        return bipolar_status;
    }

    void DirectionModifier(bool dirChange){
        direction = dirChange;
    }

    void PWMModifier (float duty_PWM){
        pwmPin.write(duty_PWM);
    }

    void move(float speed) { //custom speed, ranges from -1 to 1, where negative indicate backwards for both unipolar and bipolar
    if (bipolar_status == 1) {
        float duty = (speed * 0.5f) + 0.5f; //converts the value to the vaule between 0~1 (duty cycle)
        pwmPin.write(duty);
    } else {
        pwmPin.write(fabs(speed)); //converts to absolute value to a value between 0~1 (duty cycle)
        if (speed >= 0) {
            direction = 1; // sets direction to forward when speed value > 0
        }
        else {
            direction = 0; // sets direction to backward when speed value < 0
        }
    }
}
};

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
