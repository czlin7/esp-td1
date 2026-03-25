#pragma once

#include "QEI.h"
#include "mbed.h"
#include <cmath>

/**
 * @file encoder.h
 * @brief Wheel encoder wrapper for pulse, distance, velocity, and RPM access.
 */

/**
 * @brief Provides kinematic values derived from a quadrature encoder.
 */
class WheelEncoder {
private:
    QEI enc;          /**< Quadrature decoder instance. */
    Timer t;          /**< Time base for derivative calculations. */
    float functionVal; /**< Meters traveled per encoder tick. */
    float CPR;        /**< Encoder counts per revolution configuration value. */
    int previousTick; /**< Previous tick sample for velocity calculation. */
    float previousTime; /**< Previous time sample for velocity calculation. */
    float velocity;   /**< Last computed linear velocity in m/s. */
    float gear;       /**< Gear ratio used for RPM conversion. */
    float rpm;        /**< Last computed shaft speed in RPM. */

public:
    const float diameter = 0.079f; /**< Wheel diameter in meters. */
    const float PI = 3.14159f;     /**< Pi constant for geometry conversion. */

    /**
     * @brief Constructs a wheel encoder interface.
     * @param A Encoder channel A pin.
     * @param B Encoder channel B pin.
     * @param index Optional index pin.
     * @param gear_ratio Gear ratio used to report RPM.
     * @param cpr Encoder counts per revolution parameter.
     */
    WheelEncoder(PinName A, PinName B, PinName index, float gear_ratio, int cpr)
        : enc(A, B, index, cpr, QEI::X2_ENCODING),
          t(),
          functionVal(0.0f),
          CPR(static_cast<float>(cpr)),
          previousTick(0),
          previousTime(0.0f),
          velocity(0.0f),
          gear(gear_ratio),
          rpm(0.0f) {
        functionVal = (PI * diameter) / (CPR * gear_ratio);

        previousTime = t.read();
        t.start();
        enc.reset();
    }

    /**
     * @brief Computes instantaneous velocity from elapsed pulses and time.
     * @return Linear velocity in meters per second.
     */
    float getVelocity(void) {
        const int currentTick = enc.getPulses();
        const float currentTime = t.read();

        const int elapsedTick = currentTick - previousTick;
        const float elapsedTime = currentTime - previousTime;

        if (elapsedTime <= 0.0f) {
            return 0.0f;
        }

        const float tickRate = elapsedTick / elapsedTime;
        velocity = tickRate * functionVal;
        rpm = (tickRate / (CPR * gear)) * 60.0f;

        previousTick = currentTick;
        previousTime = currentTime;

        return velocity;
    }

    /**
     * @brief Returns accumulated quadrature pulses.
     * @return Signed pulse count from the decoder.
     */
    int getPulses(void) { return enc.getPulses(); }

    /**
     * @brief Returns the last computed RPM value.
     * @return Revolutions per minute from the latest velocity update.
     */
    float getRPM(void) { return rpm; }

    /**
     * @brief Returns traveled distance derived from pulse count.
     * @return Linear distance in meters.
     */
    float getDistance() {
        const float distance = (enc.getPulses() / (CPR * 2.0f)) * (PI * diameter);
        return distance;
    }

    /**
     * @brief Resets pulse and timing state used by this interface.
     */
    void reset() {
        enc.reset();
        t.reset();
        t.start();
        previousTick = 0;
        previousTime = t.read();
    }
};
