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
SensorArray sensorPCB(A5,A4,A3,A2,A1,A0,PB_2,PD_2,PH_1,PH_0,PC_15,PC_14,PC_11);

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

int main() {
    // 1. Initialize Bluetooth
    bt.baud(9600);

    // 2. FORCE MOTORS OFF (Safety first!)
    myPin_PA2 = 0;
    myPin_PA10 = 0;
    buggy.setEnable(0);

    while (true) {
        // 3. Read the calculated line position
        float pos = sensorPCB.getPosition();

        // 4. Print the exact math value to your Bluetooth terminal
        bt.printf("Line Position: %.2f\r\n", pos);

        // 5. Visual Feedback using the RGB LED
        if (pos > 0.5f) {
            setColor(1.0f, 0.0f, 0.0f); // RED: Line is to the right
        } else if (pos < -0.5f) {
            setColor(0.0f, 0.0f, 1.0f); // BLUE: Line is to the left
        } else {
            setColor(0.0f, 1.0f, 0.0f); // GREEN: Line is centered!
        }

        // 6. Wait 100ms so we don't spam the terminal too fast
        wait_us(100000); 
    }
}
