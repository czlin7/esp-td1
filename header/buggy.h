#pragma once

#include <algorithm>

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

    void drive(int leftSpeed, int rightSpeed) {
    enable.write(1);
    leftMotor->move(leftSpeed);
    rightMotor->move(rightSpeed);
    }

    void stop(void) {
        enable.write(0);
        leftMotor->move(0);
        rightMotor->move(0);
    }

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
void rotateAngle(float angle_deg, float maxSpeed)
{
    const float PI = 3.14159265359f;
    // Track width (m). Measure axle-to-axle; wrong value → wrong angle.
    const float wheelBase = 0.17f;

    const float Kp_angle = 4000.0f;
    const float Kp_sync  = 5000.0f;

    // Stop when within this distance (m) of target arc length.
    const float stopThreshold = 0.001f;

    // Each wheel arc length for pivot turn: s = |θ| * (wheelBase / 2) [rad·m].
    const float angle_rad = angle_deg * PI / 180.0f;
    const float arcHalf = (angle_rad * wheelBase) / 2.0f;
    const float targetDistance = (arcHalf >= 0.0f) ? arcHalf : -arcHalf;

    const int dirL = (angle_deg > 0.0f) ? 1 : -1;
    const int dirR = -dirL;

    // Must stay below maxSpeed. Old code used minSpeed=800, which was *higher* than
    // maxSpeed=50 from main → every iteration forced 800 and ignored the limit → slip / random angle.
    const float minCmd = std::min(800.0f, std::max(30.0f, 0.35f * maxSpeed));

    leftEncoder->reset();
    rightEncoder->reset();

    enable.write(1);

    while (true) {
        float distL = leftEncoder->getDistance();
        float distR = rightEncoder->getDistance();
        if (distL < 0.0f) distL = -distL;
        if (distR < 0.0f) distR = -distR;
        const float avgDist = 0.5f * (distL + distR);

        const float error = targetDistance - avgDist;

        if (error <= stopThreshold) {
            break;
        }

        float baseSpeed = Kp_angle * error;

        // Final approach: cap speed to reduce overshoot when error is small
        const float errRatio = (targetDistance > 1e-6f) ? (error / targetDistance) : 1.0f;
        float vMax = maxSpeed;
        if (errRatio < 0.15f) {
            vMax = std::max(40.0f, maxSpeed * errRatio * 2.0f);
        }

        if (baseSpeed > vMax) {
            baseSpeed = vMax;
        }
        if (baseSpeed < minCmd) {
            baseSpeed = minCmd;
        }
        if (baseSpeed > vMax) {
            baseSpeed = vMax;
        }

        const float syncError = distL - distR;
        const float correction = Kp_sync * syncError;

        const int leftCmd  = static_cast<int>(dirL * (baseSpeed - correction));
        const int rightCmd = static_cast<int>(dirR * (baseSpeed + correction));

        leftMotor->move(leftCmd);
        rightMotor->move(rightCmd);

        wait(0.001f);
    }

    leftMotor->move(0);
    rightMotor->move(0);
    enable.write(0);
}
};
