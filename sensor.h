#pragma once

#include "mbed.h"

/**
 * @file sensor.h
 * @brief Analog sensor wrapper that returns scaled voltage readings.
 */

/**
 * @brief Represents a single analog sensor channel.
 */
class Sensor {
private:
    AnalogIn sensorAnalog;   /**< ADC input pin for the sensor channel. */
    float referenceVoltage;  /**< ADC reference voltage used for scaling. */

public:
    /**
     * @brief Constructs a sensor wrapper for the specified analog pin.
     * @param pin ADC-capable pin connected to the sensor output.
     * @param voltage ADC reference voltage in volts.
     */
    Sensor(PinName pin, float voltage = 3.3f)
        : sensorAnalog(pin), referenceVoltage(voltage) {}

    /**
     * @brief Returns the instantaneous sensor voltage.
     * @return Sensor voltage in volts.
     */
    float get_voltage() {
        return sensorAnalog.read() * referenceVoltage;
    }
};
