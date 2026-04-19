#pragma once

#include "mbed.h"

class SensorArray {
private:
    AnalogIn s1, s2, s3, s4, s5, s6;
    float last_position;

    static float readSensor(AnalogIn &sensor) {
        return sensor.read();
    }

public:
    SensorArray(PinName Sen1, PinName Sen2, PinName Sen3, PinName Sen4, PinName Sen5, PinName Sen6)
        : s1(Sen1), s2(Sen2), s3(Sen3), s4(Sen4), s5(Sen5), s6(Sen6), last_position(0.0f) {}
    
    void readRaw(float v[6]) {
    v[0] = readSensor(s1);
    v[1] = readSensor(s2);
    v[2] = readSensor(s3);
    v[3] = readSensor(s4);
    v[4] = readSensor(s5);
    v[5] = readSensor(s6);
}

    float getPosition() {
    float v[6];
    v[0] = readSensor(s1);
    v[1] = readSensor(s2);
    v[2] = readSensor(s3);
    v[3] = readSensor(s4);
    v[4] = readSensor(s5);
    v[5] = readSensor(s6);

    int   max_index = 0;
    float max_val   = v[0];
    for (int i = 1; i < 6; i++) {
        if (v[i] > max_val) { max_val = v[i]; max_index = i; }
    }

    if (max_val < 0.4f) {
        return last_position;   // line lost — hold last known
    }

    float x_peak = 0.0f;

    if (max_index > 0 && max_index < 5) {   // only interpolate with real neighbours
        float vc = v[max_index];
        float vl = v[max_index - 1];
        float vr = v[max_index + 1];

        float denom = 2.0f * (vl - 2.0f * vc + vr);
        if (fabs(denom) > 1e-5f) {
            x_peak = (vl - vr) / denom;
            if (x_peak >  1.0f) x_peak =  1.0f;
            if (x_peak < -1.0f) x_peak = -1.0f;
        }
    }

    float position = ((float)max_index + x_peak - 2.5f) / 2.5f;  // → [-1, +1]

    last_position = position;
    return position;
}
};
