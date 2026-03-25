#pragma once

#include "mbed.h"
#include "sensor.h"

/**
 * @file sensor_array.h
 * @brief Utilities for computing line position from six analog sensors.
 */

/**
 * @brief Fixed-size container for six line sensors.
 *
 * The class estimates a peak position using a three-point quadratic fit around
 * the maximum sensor sample.
 */
class sensorArray {
private:
    Sensor *sensors[6]; /**< Sensor pointers in physical left-to-right order. */
    float x_peak;       /**< Last computed local peak offset. */

public:
    /**
     * @brief Constructs a sensor array from six sensor objects.
     * @param one Sensor at index 0.
     * @param two Sensor at index 1.
     * @param three Sensor at index 2.
     * @param four Sensor at index 3.
     * @param five Sensor at index 4.
     * @param six Sensor at index 5.
     */
    sensorArray(Sensor *one, Sensor *two, Sensor *three, Sensor *four,
                Sensor *five, Sensor *six)
        : sensors{one, two, three, four, five, six}, x_peak(0.0f) {}

    /**
     * @brief Estimates peak offset around the strongest sensor sample.
     * @return Interpolated peak position in normalized local coordinates.
     */
    float calculate_position() {
        float voltages[6];
        int max_index = 0;
        float max_val = 0.0f;

        for (int i = 0; i < 6; i++) {
            voltages[i] = sensors[i]->get_voltage();
            if (voltages[i] > max_val) {
                max_val = voltages[i];
                max_index = i;
            }
        }

        const float v_center = voltages[max_index];
        const float v_left = (max_index > 0) ? voltages[max_index - 1] : 0.0f;
        const float v_right = (max_index < 5) ? voltages[max_index + 1] : 0.0f;

        const float denominator = 2.0f * (v_left - (2.0f * v_center) + v_right);

        if (denominator != 0.0f) {
            x_peak = (v_left - v_right) / denominator;
        }

        return x_peak;
    }
};
