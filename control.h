#pragma once

#include "enconder.h"
#include "mbed.h"

#include <cmath>

/**
 * @file control.h
 * @brief Legacy motor and buggy control abstractions.
 */

/**
 * @brief Motor driver wrapper using normalized speed commands.
 */
class Motor {
private:
    PwmOut pwmPin;
    DigitalOut direction;
    DigitalOut bipolar;
    bool bipolar_status;

public:
    /**
     * @brief Constructs a motor output wrapper.
     * @param pwm PWM output pin.
     * @param dir Direction output pin.
     * @param bi Bipolar mode select pin.
     */
    Motor(PinName pwm, PinName dir, PinName bi)
        : pwmPin(pwm), direction(dir), bipolar(bi), bipolar_status(false) {
        pwmPin.period(0.001f);
        direction = 0;
    }

    /**
     * @brief Sets unipolar or bipolar drive mode.
     * @param bipolarity True for bipolar mode, false for unipolar mode.
     */
    void BiUni_Setter(bool bipolarity) {
        bipolar_status = bipolarity;
        bipolar = bipolar_status ? 1 : 0;
    }

    /**
     * @brief Returns the configured polarity mode.
     * @return True when bipolar mode is selected.
     */
    bool PolarityCheck(void) { return bipolar_status; }

    /**
     * @brief Sets direction output directly.
     * @param dirChange Direction output level.
     */
    void DirectionModifier(bool dirChange) { direction = dirChange; }

    /**
     * @brief Writes PWM duty directly.
     * @param duty_PWM Duty cycle in the range accepted by mbed PWM output.
     */
    void PWMModifier(float duty_PWM) { pwmPin.write(duty_PWM); }

    /**
     * @brief Applies normalized speed command.
     * @param speed Signed speed in the approximate range [-1.0, 1.0].
     */
    void move(float speed) {
        if (bipolar_status == 1) {
            const float duty = (speed * 0.5f) + 0.5f;
            pwmPin.write(duty);
        } else {
            pwmPin.write(fabs(speed));
            direction = (speed >= 0);
        }
    }
};

/**
 * @brief Legacy distance and turn maneuvers using wheel encoder feedback.
 */
class Buggy {
private:
    Motor &leftMotor;
    Motor &rightMotor;
    WheelEncoder &leftEnc;
    WheelEncoder &rightEnc;

public:
    /**
     * @brief Constructs a buggy controller from motor and encoder references.
     * @param L Left motor.
     * @param R Right motor.
     * @param LE Left encoder.
     * @param RE Right encoder.
     */
    Buggy(Motor &L, Motor &R, WheelEncoder &LE, WheelEncoder &RE)
        : leftMotor(L), rightMotor(R), leftEnc(LE), rightEnc(RE) {}

    /**
     * @brief Drives forward or backward for an estimated distance.
     * @param moving_speed Signed normalized speed command.
     * @param distance_fb Target linear distance.
     */
    void forward_backward(float moving_speed, float distance_fb) {
        leftEnc.reset();
        rightEnc.reset();

        leftMotor.move(moving_speed);
        rightMotor.move(moving_speed);

        wait(0.2);

        const float v_left = fabs(leftEnc.getVelocity());
        const float v_right = fabs(rightEnc.getVelocity());
        const float avg_velocity = (v_left + v_right) / 2.0f;

        if (avg_velocity > 0) {
            const float time_needed = distance_fb / avg_velocity;
            const float time_remaining = time_needed - 0.2f;
            if (time_remaining > 0) {
                wait(time_remaining);
            }
        }

        leftMotor.move(0);
        rightMotor.move(0);
    }

    /**
     * @brief Rotates the buggy left or right by a commanded angle.
     * @param turning_speed Signed normalized turn speed magnitude.
     * @param wheel_distance Unused parameter retained for API compatibility.
     * @param degree Positive values turn right and negative values turn left.
     */
    void turn_left_right(float turning_speed, float wheel_distance, float degree) {
        leftEnc.reset();
        rightEnc.reset();

        float left_speed;
        float right_speed;

        if (degree > 0) {
            left_speed = fabs(turning_speed);
            right_speed = -fabs(turning_speed);
        } else {
            left_speed = -fabs(turning_speed);
            right_speed = fabs(turning_speed);
        }

        leftMotor.move(left_speed);
        rightMotor.move(right_speed);

        wait(0.2);

        const float v_left = fabs(leftEnc.getVelocity());
        const float v_right = fabs(rightEnc.getVelocity());
        const float avg_velocity = (v_left + v_right) / 2.0f;

        const float absolute_degree = fabs(degree);
        const float required_distance =
            (absolute_degree * (M_PI / 180.0f) * track_width) / 2.0f;

        if (avg_velocity > 0) {
            const float time_needed = required_distance / avg_velocity;
            const float time_remaining = time_needed - 0.2f;
            if (time_remaining > 0) {
                wait(time_remaining);
            }
        }

        leftMotor.move(0);
        rightMotor.move(0);
    }
};
