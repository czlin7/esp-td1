#pragma once

#include "encoder.h"
#include "motors.h"

/**
 * @file buggy.h
 * @brief Differential-drive buggy motion primitives.
 */

/**
 * @brief Coordinates two motors and two encoders for basic maneuvers.
 */
class Buggy {
private:
    Motor *leftMotor;
    Motor *rightMotor;
    WheelEncoder *leftEncoder;
    WheelEncoder *rightEncoder;
    DigitalOut enable;

public:
    /**
     * @brief Constructs a buggy controller.
     * @param L Left motor instance.
     * @param R Right motor instance.
     * @param LE Left wheel encoder instance.
     * @param RE Right wheel encoder instance.
     * @param en Motor driver enable pin.
     */
    Buggy(Motor *L, Motor *R, WheelEncoder *LE, WheelEncoder *RE, PinName en)
        : leftMotor(L), rightMotor(R), leftEncoder(LE), rightEncoder(RE), enable(en) {}

    /**
     * @brief Sets motor driver enable output.
     * @param enable_signal Output level for the enable pin.
     */
    void setEnable(bool enable_signal) { enable.write(enable_signal); }

    /**
     * @brief Returns the current enable output state.
     * @return True when enable output is asserted.
     */
    bool getEnable() { return enable.read(); }

    /**
     * @brief Drives forward until the average encoder distance reaches a target.
     * @param targetDistance_m Target distance in meters.
     * @param max_speed Maximum speed command magnitude.
     */
    void moveDistance(float targetDistance_m, float max_speed) {
        leftEncoder->reset();
        rightEncoder->reset();

        enable.write(1);

        const float Kp_sync = 5000.0f;
        const float Kp_speed = 4000.0f;

        while (true) {
            const float distL = leftEncoder->getDistance();
            const float distR = rightEncoder->getDistance();
            const float averageDistance = (distR + distL) * 0.5f;
            const float distanceRemaining = targetDistance_m - averageDistance;

            if (distanceRemaining <= 0.0f) {
                break;
            }

            int currentBaseSpeed = static_cast<int>(Kp_speed * distanceRemaining);

            if (currentBaseSpeed > max_speed) {
                currentBaseSpeed = static_cast<int>(max_speed);
            }

            if (currentBaseSpeed < 150) {
                currentBaseSpeed = 150;
            }

            const float error = distL - distR;
            const int adjustment = static_cast<int>(Kp_sync * error);

            const int leftSpeed = currentBaseSpeed - adjustment;
            const int rightSpeed = currentBaseSpeed + adjustment;

            leftMotor->move(leftSpeed);
            rightMotor->move(rightSpeed);

            wait(0.0004f);
        }

        enable.write(0);
        leftMotor->move(0);
        rightMotor->move(0);
    }

    /**
     * @brief Rotates the buggy by commanding opposite wheel motion.
     * @param angle_deg Desired turn angle in degrees.
     * @param speed Speed command magnitude.
     */
    void rotateAngle(float angle_deg, float speed) {
        const float PI = 3.14159265359f;
        const float angle_rad = angle_deg * PI / 180.0f;
        const float wheelBase = 0.17f;

        float targetDistance = (angle_rad * wheelBase) / 2.0f;

        if (angle_deg < 0) {
            targetDistance = targetDistance * 0.90f;
        }

        leftEncoder->reset();
        rightEncoder->reset();

        const int dirL = (angle_deg > 0) ? 1 : -1;
        const int dirR = (angle_deg > 0) ? -1 : 1;

        const float Kp = 5000.0f;

        enable.write(1);

        while (true) {
            const float distL = fabs(leftEncoder->getDistance());
            const float distR = fabs(rightEncoder->getDistance());
            const float avgDist = (distL + distR) * 0.5f;

            if (avgDist >= fabs(targetDistance)) {
                break;
            }

            const float error = distL - distR;
            const int adjustment = static_cast<int>(Kp * error);

            const int currentLeftSpeed = dirL * (static_cast<int>(speed) - adjustment);
            const int currentRightSpeed = dirR * (static_cast<int>(speed) + adjustment);

            leftMotor->move(currentLeftSpeed);
            rightMotor->move(currentRightSpeed);

            wait(0.0004f);
        }

        enable.write(0);
        leftMotor->move(0);
        rightMotor->move(0);
    }
};
