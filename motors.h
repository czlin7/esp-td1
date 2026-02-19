#pragma once

#include "mbed.h"
#include <cmath>

//Motor Control Signals
struct MotorData {
  bool motor_enable;      // disabled by default
  bool motor_bipolar;    // unipolar by default
  bool motor_dir;         // forward by default
  float duty_cycle;     // 0..1
  float encoder_speed_ms; // m/s

  MotorData()
      : motor_enable(false), motor_bipolar(true), motor_dir(true),
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
        pwmPin.write(0.0f); // motor OFF at startup
    }

    void setMode(bool is_bipolar) { //sets the mode of the motor to either bipolar or unipolar
        bipolar_status = is_bipolar;
        if (bipolar_status == 1) {
            bipolar = 1; //set as bipolar
        } else {
            bipolar = 0; //set as unipolar
        }
    }

    bool readPolarity(void) const
    {
        return bipolar_status;
    }

    void setDirection(bool dir){
        direction = dir;
    }

    void setDuty(float duty_PWM){
        pwmPin.write(duty_PWM);
    }

    void move(float speed) { //custom speed, ranges from -1 to 1, where negative indicate backwards for both unipolar and bipolar
    if (bipolar_status) {
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
