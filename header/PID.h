#pragma once

class PID {
private:
    float Kp, Ki, Kd;
    float prev_error;
    float integral;
    float out_min, out_max;

public:
    PID(float p, float i, float d, float min, float max)
        : Kp(p), Ki(i), Kd(d),
          prev_error(0), integral(0),
          out_min(min), out_max(max) {}

    void setGains(float p, float i, float d)
    {
        Kp = p;
        Ki = i;
        Kd = d;
        reset();
    }

    float getKp() const { return Kp; }
    float getKi() const { return Ki; }
    float getKd() const { return Kd; }

    float compute(float error, float dt) {
        float P = Kp * error;

        integral += error * dt;
        float I = Ki * integral;

        float D = Kd * (error - prev_error) / dt;

        float output = P + I + D;

        // clamp
        if (output > out_max) output = out_max;
        if (output < out_min) output = out_min;

        prev_error = error;

        return output;
    }
    
    void reset() {
        prev_error = 0;
        integral = 0;
    }

    void setOutputLimits(float min_out, float max_out)
    {
        out_min = min_out;
        out_max = max_out;
    }

    float getOutMin() const { return out_min; }
    float getOutMax() const { return out_max; }
};
