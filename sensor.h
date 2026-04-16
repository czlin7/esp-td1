#pragma once

#include "mbed.h"

class SensorArray {
private:
    AnalogIn s1, s2, s3, s4, s5, s6;
    float Vref;
    float last_position;

    float readSensor(AnalogIn &sensor) {
        float active = sensor.read();
        return active; // Get ambient light rejected readings
    }

public:
    SensorArray(PinName Sen1, PinName Sen2, PinName Sen3, PinName Sen4, PinName Sen5, PinName Sen6) : 
        s1(Sen1), s2(Sen2), s3(Sen3), s4(Sen4), s5(Sen5), s6(Sen6),
        last_position(0.0f) {
        Vref = 3.3f; 
    }

    float getPosition() {
        float v[6];

        v[0] = readSensor(s1);
        v[1] = readSensor(s2);
        v[2] = readSensor(s3);
        v[3] = readSensor(s4);
        v[4] = readSensor(s5);
        v[5] = readSensor(s6);

        // Parabolic peak detection
        int max_index = 0;
        float max_val = v[0];

        for (int i = 1; i < 6; i++) {
            if (v[i] > max_val) {
                max_val = v[i];
                max_index = i;
            }
        }

        // Line lost check
        if (max_val < 0.05f) {
            return last_position; // Returns the last known position if line is lost
        }

        float v_center = v[max_index];
        float v_left  = (max_index > 0) ? v[max_index - 1] : v_center;
        float v_right = (max_index < 5) ? v[max_index + 1] : v_center;

        float denom = 2.0f * (v_left - 2.0f * v_center + v_right);

        float x_peak = 0.0f;

        if (fabs(denom) > 1e-5f) {
            x_peak = (v_left - v_right) / denom;
        }

        // Clamp
        if (x_peak > 1.0f) x_peak = 1.0f;
        if (x_peak < -1.0f) x_peak = -1.0f;

        float position = (float)max_index + x_peak;

        // Center
        position -= 2.5f;

        last_position = position;

        return position;
    }

    void getRawValues(float &a, float &b, float &c, float &d, float &e, float &f) {
        a = readSensor(s1);
        b = readSensor(s2);
        c = readSensor(s3);
        d = readSensor(s4);
        e = readSensor(s5);
        f = readSensor(s6);
    }
};
