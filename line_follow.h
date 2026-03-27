#pragma once

#include "mbed.h"
#include "sensor.h"
#include "motors.h"

// -------- CONFIG --------
#define NUM_SENSORS 6
#define DT 0.002f  // 2ms loop

// -------- LINE FOLLOWER --------
class LineFollower {
private:
    Sensor* sensors[NUM_SENSORS];
    Motor& leftMotor;
    Motor& rightMotor;

    // Calibration
    float minV[NUM_SENSORS];
    float maxV[NUM_SENSORS];

    // Filter
    float filtered[NUM_SENSORS];

    // PID
    float Kp, Ki, Kd;
    float prev_error;
    float integral;

    int baseSpeed;

public:
    LineFollower(Sensor s[NUM_SENSORS], Motor& L, Motor& R)
        : leftMotor(L), rightMotor(R),
          Kp(0.8f), Ki(0.0f), Kd(0.4f),
          prev_error(0), integral(0),
          baseSpeed(500) {

        for (int i = 0; i < NUM_SENSORS; i++) {
            sensors[i] = &s[i];
            filtered[i] = 0;

            // default calibration (REPLACE THESE)
            minV[i] = 0.2f;
            maxV[i] = 3.3f;
        }
    }

    // -------- NORMALIZATION --------
    float readNormalized(int i) {
        float raw = sensors[i]->get_voltage();

        // Low-pass filter
        filtered[i] = 0.7f * filtered[i] + 0.3f * raw;

        float value = (filtered[i] - minV[i]) / (maxV[i] - minV[i]);

        if (value < 0) value = 0;
        if (value > 1) value = 1;

        return value; // white ≈ 1
    }

    // -------- ERROR (CENTER + HYBRID) --------
    float getError() {
        float leftMid  = readNormalized(2);
        float rightMid = readNormalized(3);

        float error = (rightMid - leftMid);

        // Use outer sensors only when needed
        if (fabs(error) > 0.3f) {
            float leftOuter  = readNormalized(1);
            float rightOuter = readNormalized(4);

            error += 0.5f * (rightOuter - leftOuter);
        }

        return error;
    }

    // -------- PID --------
    float computePID(float error) {
        integral += error * DT;

        // Anti-windup
        if (integral > 1.0f) integral = 1.0f;
        if (integral < -1.0f) integral = -1.0f;

        float derivative = (error - prev_error) / DT;

        float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        prev_error = error;

        return output;
    }

    // -------- MAIN STEP --------
    void update() {
        float error = getError();
        float correction = computePID(error);

        // Dynamic speed
        int speed;
        if (fabs(error) < 0.05f)
            speed = 650;   // straight
        else
            speed = 450;   // turning

        int left  = speed + (int)(correction * 500.0f);
        int right = speed - (int)(correction * 500.0f);

        // Clamp to Motor range
        if (left > 1000) left = 1000;
        if (left < -1000) left = -1000;
        if (right > 1000) right = 1000;
        if (right < -1000) right = -1000;

        leftMotor.move(left);
        rightMotor.move(right);
    }

    // -------- OPTIONAL: SET PID --------
    void setPID(float p, float i, float d) {
        Kp = p;
        Ki = i;
        Kd = d;
    }
};
