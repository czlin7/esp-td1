#pragma once

#include "mbed.h"

/** Single analog line sensor: normalized ADC scaled to volts (0..3.3). */
class Sensor {
public:
    explicit Sensor(PinName pin) : adc(pin) {}

    float get_voltage() { return adc.read() * 3.3f; }

private:
    AnalogIn adc;
};

/**
 * Six-channel line position using parabolic interpolation around the brightest cell.
 *
 * mbed AnalogIn::read() is normalized 0..1 (not volts). Assumes the line is a local
 * maximum (reflective tape). For a dark line on a bright surface, invert in
 * readNormalized() (e.g. return 1.f - sensor.read()).
 */
class SensorArray {
private:
    AnalogIn s1, s2, s3, s4, s5, s6;
    float last_position;

    static float readNormalized(AnalogIn& sensor) { return sensor.read(); }

    static void fillReadings(float v[6], AnalogIn& a1, AnalogIn& a2, AnalogIn& a3, AnalogIn& a4, AnalogIn& a5, AnalogIn& a6)
    {
        v[0] = readNormalized(a1);
        v[1] = readNormalized(a2);
        v[2] = readNormalized(a3);
        v[3] = readNormalized(a4);
        v[4] = readNormalized(a5);
        v[5] = readNormalized(a6);
    }

public:
    /** Center so straight-ahead reads ~0 (middle of indices 0..5 is 2.5). */
    enum { kCenterOffsetTimes10 = 25 };
    /** Peak below this (0..1 scale) => treat as line not visible (tunable). */
    enum { kLineLostThresholdTimes1000 = 50 };
    /** Minimum parabola curvature before trusting sub-cell fit. */
    enum { kParabolaDenomEpsTimes1e6 = 10 };

    SensorArray(PinName Sen1, PinName Sen2, PinName Sen3, PinName Sen4, PinName Sen5, PinName Sen6)
        : s1(Sen1), s2(Sen2), s3(Sen3), s4(Sen4), s5(Sen5), s6(Sen6), last_position(0.0f) {}

    /**
     * @param line_detected optional; pass 0 to ignore. When not 0, set false if returning
     *        last_position (line not seen), true if a fresh peak was used.
     */
    float getPosition(bool* line_detected = 0)
    {
        float v[6];
        fillReadings(v, s1, s2, s3, s4, s5, s6);

        int max_index = 0;
        float max_val = v[0];
        for (int i = 1; i < 6; i++) {
            if (v[i] > max_val) {
                max_val = v[i];
                max_index = i;
            }
        }

        const float line_lost_thresh = kLineLostThresholdTimes1000 / 1000.0f;

        if (max_val < line_lost_thresh) {
            if (line_detected != 0) {
                *line_detected = false;
            }
            return last_position;
        }

        if (line_detected != 0) {
            *line_detected = true;
        }

        const float v_center = v[max_index];
        const float v_left = (max_index > 0) ? v[max_index - 1] : v_center;
        const float v_right = (max_index < 5) ? v[max_index + 1] : v_center;

        const float denom = 2.0f * (v_left - 2.0f * v_center + v_right);

        float x_peak = 0.0f;
        const float parab_eps = kParabolaDenomEpsTimes1e6 / 1e6f;
        const float denom_abs = (denom >= 0.0f) ? denom : -denom;
        if (denom_abs > parab_eps) {
            x_peak = (v_left - v_right) / denom;
        }

        if (x_peak > 1.0f) {
            x_peak = 1.0f;
        }
        if (x_peak < -1.0f) {
            x_peak = -1.0f;
        }

        const float center_off = kCenterOffsetTimes10 / 10.0f;
        float position = (float)max_index + x_peak - center_off;

        last_position = position;
        return position;
    }

    void getRawValues(float& a, float& b, float& c, float& d,float& e,float& f)
    {
        float v[6];
        fillReadings(v, s1, s2, s3, s4, s5, s6);
        a = v[0];
        b = v[1];
        c = v[2];
        d = v[3];
        e = v[4];
        f = v[5];
    }
};
