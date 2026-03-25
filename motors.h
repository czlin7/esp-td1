#pragma once

#include "mbed.h"

/**
 * @file motors.h
 * @brief Motor command data and low-level H-bridge motor driver wrapper.
 */

/**
 * @brief Mutable command/state bundle used by the user interface.
 */
struct MotorData {
    bool motor_bipolar;   /**< Driver mode flag: bipolar when true. */
    bool motor_dir;       /**< Direction flag used by UI logic. */
    float duty_cycle;     /**< Duty cycle command in the range [0.0, 1.0]. */
    float encoder_speed_ms; /**< Last measured wheel speed in m/s. */
    bool motor_enable;    /**< Motor enable flag. */

    /**
     * @brief Constructs a default motor command set.
     */
    MotorData()
        : motor_bipolar(true),
          motor_dir(true),
          duty_cycle(0.5f),
          encoder_speed_ms(0.0f),
          motor_enable(false) {}
};

/**
 * @brief Controls one motor channel through PWM, direction, and mode pins.
 */
class Motor {
private:
    PwmOut pwmPin;
    DigitalOut direction;
    DigitalOut bipolarPin;
    bool isBipolar;

    float lastDuty;      /**< Cached PWM duty value. */
    bool lastDirection;  /**< Cached direction value. */

    const float PWM_PERIOD; /**< PWM period in seconds. */
    const float SCALE;      /**< Integer speed to normalized scalar conversion. */

    /**
     * @brief Writes PWM duty only if the value changed.
     * @param duty Duty cycle in the range accepted by mbed PWM output.
     */
    inline void writeDuty(float duty) {
        if (duty != lastDuty) {
            pwmPin.write(duty);
            lastDuty = duty;
        }
    }

    /**
     * @brief Writes direction only if the value changed.
     * @param dir Direction output level.
     */
    inline void writeDirection(bool dir) {
        if (dir != lastDirection) {
            direction = dir;
            lastDirection = dir;
        }
    }

    /**
     * @brief Applies bipolar driver output mapping.
     * @param speed Signed speed command in the range [-1000, 1000].
     */
    inline void moveBipolar(int speed) {
        const int magnitude = (speed >= 0) ? speed : -speed;
        const float norm = magnitude * SCALE;
        const float duty = 0.5f - 0.5f * norm;
        const bool dirPin = (speed < 0);

        writeDirection(dirPin);
        writeDuty(duty);
    }

    /**
     * @brief Applies unipolar driver output mapping.
     * @param speed Signed speed command in the range [-1000, 1000].
     */
    inline void moveUnipolar(int speed) {
        const bool reverse = (speed < 0);
        const int magnitude = (speed >= 0) ? speed : -speed;
        const float duty = 1.0f - (magnitude * SCALE);

        writeDirection(reverse);
        writeDuty(duty);
    }

public:
    /**
     * @brief Constructs a motor driver wrapper.
     * @param pwm PWM output pin.
     * @param dir Direction control pin.
     * @param bi Driver mode selection pin.
     */
    Motor(PinName pwm, PinName dir, PinName bi)
        : pwmPin(pwm),
          direction(dir),
          bipolarPin(bi),
          isBipolar(true),
          lastDuty(-1.0f),
          lastDirection(false),
          PWM_PERIOD(0.00004f),
          SCALE(1.0f / 1000.0f) {
        pwmPin.period(PWM_PERIOD);
        bipolarPin = 0;
        direction = 0;
        pwmPin.write(1.0f);
        lastDuty = 1.0f;
    }

    /**
     * @brief Sets output mode.
     * @param bipolar True for bipolar mode, false for unipolar mode.
     */
    void setMode(bool bipolar) {
        isBipolar = bipolar;
        bipolarPin = bipolar;
    }

    /**
     * @brief Returns the active output mode.
     * @return True when bipolar mode is selected.
     */
    bool readPolarity() const { return isBipolar; }

    /**
     * @brief Sets raw PWM duty.
     * @param duty Duty cycle value written to PWM output.
     */
    void setDuty(float duty) { writeDuty(duty); }

    /**
     * @brief Sets raw direction output.
     * @param dir Direction pin level.
     */
    void setDirection(bool dir) { writeDirection(dir); }

    /**
     * @brief Applies a signed speed command.
     * @param speed Signed command clamped to [-1000, 1000].
     */
    void move(int speed) {
        if (speed > 1000) speed = 1000;
        if (speed < -1000) speed = -1000;

        if (isBipolar) {
            moveBipolar(speed);
        } else {
            moveUnipolar(speed);
        }
    }
};
