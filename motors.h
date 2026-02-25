#pragma once

#include "mbed.h"

// Motor Control Signals
struct MotorData {
  bool motor_bipolar;
  bool motor_dir;
  float duty_cycle;
  float encoder_speed_ms;
  bool motor_enable;

  MotorData()
      : motor_bipolar(true), motor_dir(true),
        duty_cycle(0.5f), encoder_speed_ms(0.0f), motor_enable(false) {}
};

class Motor {
private:
    PwmOut pwmPin;
    DigitalOut direction;
    DigitalOut bipolarPin;
    bool isBipolar;

    // Cached outputs, so only updates if change in value
    float lastDuty;
    bool lastDirection;

    const float PWM_PERIOD;
    const float SCALE;

    // optimized writers
    inline void writeDuty(float duty) {
        if (duty != lastDuty) {
            pwmPin.write(duty);
            lastDuty = duty;
        }
    }

    inline void writeDirection(bool dir) {
        if (dir != lastDirection) {
            direction = dir;
            lastDirection = dir;
        }
    }
    //@ Updated logic based on driver behavior
    //@ Dir flips polarity
    //@ Move motors using bipolar mode, speed (-1000 to +1000)
    inline void moveBipolar(int speed) {
    int magnitude = (speed >= 0) ? speed : -speed;
    float norm = magnitude * SCALE;             // -1..1
    float duty = 0.5f - 0.5f * norm;       // 0 → full forward, 1 → full backward

    bool dirPin = (speed < 0);             // negative speed → Dir = 1
    writeDirection(dirPin);
    writeDuty(duty);
}

    //@ Updated logic based on driver behavior
    //@ Dir selects direction
    //@ Move motors using unipolar mode, speed (-1000 to +1000)
    inline void moveUnipolar(int speed) {
        bool reverse = (speed < 0);
        int magnitude = (speed >= 0) ? speed : -speed;
        float duty = 1.0f - (magnitude * SCALE); // inverted PWM

        writeDirection(reverse);
        writeDuty(duty);
    }

public:
    Motor(PinName pwm, PinName dir, PinName bi)
        : pwmPin(pwm), direction(dir), bipolarPin(bi),
          isBipolar(true), lastDuty(-1.0f), lastDirection(false), //initialization
          PWM_PERIOD(0.00004f), SCALE(1.0f / 1000.0f)
    {
        //Default settings
        pwmPin.period(PWM_PERIOD);

        bipolarPin = 0;
        direction = 0;

        pwmPin.write(1.0f);  
        lastDuty = 1.0f;
    }
    // COnfiguration member functions

    // Bipolarity set if 1 and Unipolar if 0
    void setMode(bool bipolar) {
        isBipolar = bipolar;
        bipolarPin = bipolar;
    }
    // Outputs polarity
    bool readPolarity() const {
        return isBipolar;
    }
    // Duty cycle modifier
    void setDuty(float duty) {
        writeDuty(duty);
    }
    // Direction modifier
    void setDirection(bool dir) {
        writeDirection(dir);
    }

    // MAin control function
    void move(int speed) {
        if (speed > 1000) speed = 1000;
        if (speed < -1000) speed = -1000;

        if (isBipolar)
            moveBipolar(speed);
        else
            moveUnipolar(speed);
    }
};
