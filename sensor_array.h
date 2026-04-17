#pragma once

#include <cmath>

/**
 * Discrete PID with output clamping, conditional integral anti-windup,
 * first-order derivative filtering, and invalid-dt guarding.
 */
class PID {
private:
    float Kp, Ki, Kd;
    float prev_error;
    float integral;
    float out_min, out_max;
    float d_filtered;
    float d_filter_alpha;
    float last_output;

public:
    PID(float p, float i, float d, float min, float max)
        : Kp(p),
          Ki(i),
          Kd(d),
          prev_error(0),
          integral(0),
          out_min(min),
          out_max(max),
          d_filtered(0),
          d_filter_alpha(0.35f),
          last_output(0) {}

    /** Replace proportional, integral, and derivative gains (output limits unchanged). */
    void setGains(float p, float i, float d) {
        Kp = p;
        Ki = i;
        Kd = d;
    }

    /** Alpha in (0,1]: higher = derivative follows changes faster (less filtering). */
    void setDerivativeFilterAlpha(float alpha) {
        if (alpha < 1e-4f) {
            alpha = 1e-4f;
        }
        if (alpha > 1.0f) {
            alpha = 1.0f;
        }
        d_filter_alpha = alpha;
    }

    float compute(float error, float dt) {
        if (dt < 1e-6f) {
            return last_output;
        }

        const float P = Kp * error;

        if (fabsf(Ki) > 1e-12f) {
            integral += error * dt;
        }
        float I = Ki * integral;

        const float d_raw = (error - prev_error) / dt;
        d_filtered += d_filter_alpha * (d_raw - d_filtered);
        const float D = Kd * d_filtered;

        float output = P + I + D;

        if (output > out_max) {
            output = out_max;
            if (error > 0.0f && fabsf(Ki) > 1e-12f) {
                integral -= error * dt;
            }
        } else if (output < out_min) {
            output = out_min;
            if (error < 0.0f && fabsf(Ki) > 1e-12f) {
                integral -= error * dt;
            }
        }

        prev_error = error;
        last_output = output;
        return output;
    }

    void reset() {
        prev_error = 0;
        integral = 0;
        d_filtered = 0;
        last_output = 0;
    }
};

