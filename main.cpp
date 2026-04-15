#include "mbed.h"
#include "sensor.h"
#include "motors.h"
#include "encoder.h"
#include "buggy.h"
#include "PID.h"

Serial pc(USBTX, USBRX); // tx, rx

PwmOut red(D5);
PwmOut green(D6);
PwmOut blue(D9);

void setColor(float r, float g, float b)
{
    red = r;
    green = g;
    blue = b;
}

// Bluetooth
Serial bt(PA_11, PA_12);   // TX, RX

// Fixed: Assigned variable names to the DigitalOut pins
DigitalOut myPin_PA2(PA_2);
DigitalOut myPin_PA10(PA_10);

// Sensors
SensorArray sensorPCB(A5,A4,A3,A2,A1,A0,PA_3,PB_2,PH_1,PH_0,PC_15,PC_14,PB_2);

// Motors
Motor rightMotor(PA_15,PC_2,PA_14);
Motor leftMotor(PB_7,PC_3,PA_13);

// Encoders
WheelEncoder leftEncoder (PC_10, PC_12, NC, 10.0f, 256);
WheelEncoder rightEncoder (PC_8, PC_6, NC, 10.0f , 256);

// Buggy
Buggy buggy(&leftMotor, &rightMotor, &leftEncoder, &rightEncoder, PC_4);

// PID
PID linePID(1.0f, 0.0f, 0.1f, -0.3f, 0.3f);
PID leftSpeedPID(2.0f, 0.0f, 0.1f, -1000.0f, 1000.0f);
PID rightSpeedPID(2.0f, 0.0f, 0.1f, -1000.0f, 1000.0f);

// Parameters
float baseSpeed = 0.5f; // m/s

const float LINE_DT  = 0.01f;   // 100 Hz
const float SPEED_DT = 0.001f;  // 1 kHz

// Targets (IMPORTANT: persistent)
float targetLeft  = 0.0f;
float targetRight = 0.0f;

float save1 = 0;
float save2 = 0;
float save3 = 0;
float save4 = 0;
float save5 = 0;
float save6 = 0;

int main() {
    //Bluetooth initialization
    bt.baud(9600);

    buggy.setEnable(0);

    // Initialization, runs once
    float position = sensorPCB.getPosition();
    float correction = linePID.compute(position, LINE_DT);

    targetLeft  = baseSpeed + correction;
    targetRight = baseSpeed - correction;

    float lineTimer = 0.0f;

    while (true) {
        if (bt.readable()) {
            char c = bt.getc();

            if (c == 'G')
                buggy.rotateAngle(180,500);

            else if (c == 'Y')
                setColor(0,1.0,0);

            else if (c == 'R')
                setColor(0,0,1.0);
            
            else if (c == 'S')
                buggy.stop();

            else if (c == 'W')
                setColor(0,0,0);

            else if (c == 'O')
                setColor(1.0,1.0,1.0);

        }

        // Inner control loop (1 kHz)
        wait_us(1000);  // enforce 1 kHz
        float dt_speed = SPEED_DT;

        // Measure actual speed
        float actualLeft  = leftEncoder.getVelocity();
        float actualRight = rightEncoder.getVelocity();

        // Speed error
        float errorLeft  = targetLeft - actualLeft;
        float errorRight = targetRight - actualRight;

        // Speed PID → PWM
        float leftCmd  = leftSpeedPID.compute(errorLeft, dt_speed);
        float rightCmd = rightSpeedPID.compute(errorRight, dt_speed);

        // Drive motors
        buggy.drive((int)leftCmd, (int)rightCmd);

        // Outer control loop (100 Hz)
        lineTimer += dt_speed;

        if (lineTimer >= LINE_DT) {
            lineTimer = 0.0f;

            float position = sensorPCB.getPosition();

            float correction = linePID.compute(position, LINE_DT);

            targetLeft  = baseSpeed + correction;
            targetRight = baseSpeed - correction;
        }
    }
}
