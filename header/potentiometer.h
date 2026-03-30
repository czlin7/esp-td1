#pragma once

#include "mbed.h"

/**
 * @file potentiometer.h
 * @brief Potentiometer abstractions for direct and periodic ADC sampling.
 */

/**
 * @brief Reads an analog input and stores the most recent sample.
 */
class Potentiometer {
private:
    AnalogIn inputSignal;         /**< ADC input source. */
    float VDD;                    /**< ADC reference voltage in volts. */
    volatile float currentSampleNorm;  /**< Last normalized sample in [0, 1]. */
    volatile float currentSampleVolts; /**< Last sample converted to volts. */

public:
    /**
     * @brief Constructs a potentiometer wrapper.
     * @param pin ADC-capable input pin.
     * @param v ADC reference voltage in volts.
     */
    Potentiometer(PinName pin, float v)
        : inputSignal(pin), VDD(v), currentSampleNorm(0.0f), currentSampleVolts(0.0f) {}

    /**
     * @brief Returns an immediate voltage reading.
     * @return Input voltage in volts.
     */
    float amplitudeVolts(void) { return inputSignal.read() * VDD; }

    /**
     * @brief Returns an immediate normalized reading.
     * @return Normalized input value in the range [0, 1].
     */
    float amplitudeNorm(void) { return inputSignal.read(); }

    /**
     * @brief Captures and stores the latest sample values.
     */
    void sample(void) {
        currentSampleNorm = inputSignal.read();
        currentSampleVolts = currentSampleNorm * VDD;
    }

    /**
     * @brief Returns the most recently stored normalized sample.
     * @return Normalized sample in the range [0, 1].
     */
    float getCurrentSampleNorm(void) { return currentSampleNorm; }

    /**
     * @brief Returns the most recently stored voltage sample.
     * @return Sample voltage in volts.
     */
    float getCurrentSampleVolts(void) { return currentSampleVolts; }
};

/**
 * @brief Potentiometer variant that samples itself periodically with a ticker.
 */
class SamplingPotentiometer : public Potentiometer {
private:
    float samplingFrequency; /**< Sampling frequency in hertz. */
    float samplingPeriod;    /**< Sampling period in seconds. */
    Ticker sampler;          /**< Periodic trigger for sampling callback. */

public:
    /**
     * @brief Constructs a periodically sampled potentiometer.
     * @param p ADC-capable input pin.
     * @param v ADC reference voltage in volts.
     * @param fs Sampling frequency in hertz.
     */
    SamplingPotentiometer(PinName p, float v, float fs)
        : Potentiometer(p, v),
          samplingFrequency(fs),
          samplingPeriod(1.0f / fs) {
        sampler.attach(callback(this, &Potentiometer::sample), samplingPeriod);
    }
};
